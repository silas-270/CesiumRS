use std::ffi::CStr;
use std::os::raw::c_char;
use log::{error, info};
use cesium_engine::globe::tiles::config::{TileEngineConfig, STANDARD_IMAGERY_URL};

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
    render_routes_headless_horizon(width, height, routes, routes_count, out_path, 1.00, 11.0, 46.0, 0.0, 0.0)
}

#[no_mangle]
pub extern "C" fn render_routes_headless_custom(
    width: u32,
    height: u32,
    routes: *const HeadlessRoute,
    routes_count: usize,
    out_path: *const c_char,
    distance: f32,
    tilt_deg: f32,
    pan_deg: f32,
) -> bool {
    if routes.is_null() || out_path.is_null() {
        error!("render_routes_headless_custom: null pointer passed");
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

    let (hub_lon, hub_lat) = if routes_count > 0 {
        (route_slice[0].start.lon, route_slice[0].start.lat)
    } else {
        (0.0, 0.0)
    };
    let hub_ecef = cesium_engine::globe::geometry::lon_lat_alt_to_ecef_f64(hub_lon, hub_lat, 0.0);
    let target_center = glam::Vec3::new(hub_ecef[0] as f32, hub_ecef[1] as f32, hub_ecef[2] as f32);

    // 1. Target frame (The hub of the routes)
    let hub_up = target_center.normalize_or_zero();
    
    // In CesiumRS, global Y is the North Pole.
    let true_east = glam::Vec3::Y.cross(hub_up).normalize_or_zero();
    let true_north = hub_up.cross(true_east).normalize_or_zero();

    // 2. Spherical orientation: tilt (pitch along North-South) and pan (yaw along East-West)
    let tilt_rad = f32::to_radians(tilt_deg);
    let pan_rad = f32::to_radians(pan_deg);
    
    // 3. Direction from center of Earth towards camera
    let dir = (hub_up * (tilt_rad.cos() * pan_rad.cos()) + true_north * tilt_rad.sin() + true_east * (tilt_rad.cos() * pan_rad.sin())).normalize_or_zero();
    let eye = dir * distance;

    let extension = Box::new(crate::headless::route_builder::RoutesExtension::new(&extension_routes));
    let mut config = TileEngineConfig::default();
    config.offline_mode = false;
    config.base_imagery_url = STANDARD_IMAGERY_URL.to_string();
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
        None,
        &path_str,
    ));

    true
}

#[no_mangle]
pub extern "C" fn render_routes_headless_horizon(
    width: u32,
    height: u32,
    routes: *const HeadlessRoute,
    routes_count: usize,
    out_path: *const c_char,
    altitude: f32,
    back_deg: f32,
    pitch_deg: f32,
    heading_deg: f32,
    roll_deg: f32,
) -> bool {
    if routes.is_null() || out_path.is_null() {
        error!("render_routes_headless_horizon: null pointer passed");
        return false;
    }

    let route_slice = unsafe { std::slice::from_raw_parts(routes, routes_count) };
    let path_str = unsafe { CStr::from_ptr(out_path) }.to_string_lossy().into_owned();

    info!("Starting headless horizon route rendering to {}", path_str);

    let mut extension_routes = Vec::with_capacity(routes_count);
    for r in route_slice {
        extension_routes.push(*r);
    }

    let (hub_lon, hub_lat) = if routes_count > 0 {
        (route_slice[0].start.lon, route_slice[0].start.lat)
    } else {
        (0.0, 0.0)
    };
    let hub_ecef = cesium_engine::globe::geometry::lon_lat_alt_to_ecef_f64(hub_lon, hub_lat, 0.0);
    let hub_pos = glam::Vec3::new(hub_ecef[0] as f32, hub_ecef[1] as f32, hub_ecef[2] as f32);
    let earth_radius = hub_pos.length();

    // 1. Local coordinate frame at the hub airport
    let hub_up = hub_pos.normalize_or_zero();
    let true_east = glam::Vec3::Y.cross(hub_up).normalize_or_zero();
    let true_north = hub_up.cross(true_east).normalize_or_zero();

    // 2. Heading: direction the camera looks across the ground
    let heading_rad = f32::to_radians(heading_deg);
    let v_fwd = (true_north * heading_rad.cos() + true_east * heading_rad.sin()).normalize_or_zero();
    let v_right = (true_east * heading_rad.cos() - true_north * heading_rad.sin()).normalize_or_zero();

    // 3. Move camera backward along the spherical great circle opposite to v_fwd
    let back_rad = f32::to_radians(back_deg);
    let u_cam = (hub_up * back_rad.cos() - v_fwd * back_rad.sin()).normalize_or_zero();
    let cam_up = u_cam;
    let cam_fwd = (v_fwd * back_rad.cos() + hub_up * back_rad.sin()).normalize_or_zero();

    // 4. 3D Camera eye position (following Earth's curvature + altitude)
    let eye = u_cam * (earth_radius + altitude);

    // 5. Look direction: pitch down from horizon (0 deg = towards horizon, >0 deg = down towards ground/hub)
    let pitch_rad = f32::to_radians(pitch_deg);
    let look_dir = (cam_fwd * pitch_rad.cos() - cam_up * pitch_rad.sin()).normalize_or_zero();
    let target = eye + look_dir * 10.0;

    // 6. Camera up vector (with roll)
    let cam_plane_up = (cam_up * pitch_rad.cos() + cam_fwd * pitch_rad.sin()).normalize_or_zero();
    let roll_rad = f32::to_radians(roll_deg);
    let final_up = (cam_plane_up * roll_rad.cos() + v_right * roll_rad.sin()).normalize_or_zero();

    let extension = Box::new(crate::headless::route_builder::RoutesExtension::new(&extension_routes));
    let mut config = TileEngineConfig::default();
    config.offline_mode = false;
    config.base_imagery_url = STANDARD_IMAGERY_URL.to_string();
    config.transparent_background = true;
    config.lod_factor = 2.0; 
    config.mesh_segments = 32; 
    config.max_cache_size = std::num::NonZeroUsize::new(2048).unwrap();
    config.mesh_cache_size = std::num::NonZeroUsize::new(1024).unwrap();

    pollster::block_on(crate::headless::routes_headless_app::run_headless_render(
        width,
        height,
        config,
        Some(extension),
        eye,
        target,
        Some(final_up),
        &path_str,
    ));

    true
}
