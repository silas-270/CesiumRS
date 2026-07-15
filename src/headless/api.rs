use std::ffi::CStr;
use std::os::raw::c_char;
use log::{error, info};
use cesium_engine::globe::tiles::config::TileEngineConfig;

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct LatLon {
    pub lat: f64,
    pub lon: f64,
}

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct HeadlessRoute {
    pub start: LatLon,
    pub end: LatLon,
}

#[no_mangle]
pub extern "C" fn render_routes_headless(
    width: u32,
    height: u32,
    routes: *const HeadlessRoute,
    routes_count: usize,
    out_path: *const c_char,
) -> bool {
    if routes.is_null() || out_path.is_null() {
        error!("render_routes_headless: null pointer passed");
        return false;
    }

    let route_slice = unsafe { std::slice::from_raw_parts(routes, routes_count) };
    let path_str = unsafe { CStr::from_ptr(out_path) }.to_string_lossy().into_owned();

    info!("Starting headless route rendering to {}", path_str);

    let mut extension_routes = Vec::with_capacity(routes_count);
    let mut total_x = 0.0;
    let mut total_y = 0.0;
    let mut total_z = 0.0;
    let mut count = 0;

    for r in route_slice {
        let p1 = cesium_engine::globe::geometry::lon_lat_alt_to_ecef_f64(r.start.lon, r.start.lat, 0.0);
        let p2 = cesium_engine::globe::geometry::lon_lat_alt_to_ecef_f64(r.end.lon, r.end.lat, 0.0);
        total_x += p1[0] + p2[0];
        total_y += p1[1] + p2[1];
        total_z += p1[2] + p2[2];
        count += 2;
        extension_routes.push(*r);
    }

    let hub_ecef = cesium_engine::globe::geometry::lon_lat_alt_to_ecef_f64(route_slice[0].start.lon, route_slice[0].start.lat, 0.0);
    let target_center = glam::Vec3::new(hub_ecef[0] as f32, hub_ecef[1] as f32, hub_ecef[2] as f32);

    // 1. Target frame (The hub of the routes)
    let hub_up = target_center.normalize_or_zero();
    
    // In CesiumRS, global Y is the North Pole.
    let true_east = glam::Vec3::Y.cross(hub_up).normalize_or_zero();
    let true_north = hub_up.cross(true_east).normalize_or_zero();

    // 2. Distance to camera
    let distance = 16.36;

    // 3. Tilt camera South by ~15 degrees so airport appears higher on screen (top 1/3 height)
    // Moving the camera South (along -true_north) makes the airport appear North (up) on the screen.
    let tilt_angle = f32::to_radians(-15.0);
    
    // 4. Final transformed coordinates
    let eye = (hub_up * tilt_angle.cos() + true_north * tilt_angle.sin()) * distance;

    let extension = Box::new(crate::headless::route_builder::RoutesExtension::new(&extension_routes));
    let mut config = TileEngineConfig::default();
    config.offline_mode = false;
    config.base_imagery_url =
        "https://a.basemaps.cartocdn.com/dark_nolabels/{z}/{x}/{y}.png"
            .to_string();
    config.transparent_background = true;
    
    // Lowered mesh subdivision to prevent massive VRAM over-allocation on mobile
    config.lod_factor = 2.0; 
    config.mesh_segments = 32; 
    config.max_cache_size = std::num::NonZeroUsize::new(2048).unwrap();
    config.mesh_cache_size = std::num::NonZeroUsize::new(1024).unwrap();

    // Remove unused total_x, total_y, total_z warnings
    let _ = total_x;
    let _ = total_y;
    let _ = total_z;
    let _ = count;

    pollster::block_on(crate::headless::routes_headless_app::run_headless_render(
        width,
        height,
        config,
        Some(extension),
        eye,
        glam::Vec3::ZERO,
        &path_str,
    ));

    true
}
