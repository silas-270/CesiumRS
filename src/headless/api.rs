use std::ffi::CStr;
use std::os::raw::c_char;
use log::{error, info};

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

    let target_center = if count > 0 {
        glam::Vec3::new(
            (total_x / count as f64) as f32,
            (total_y / count as f64) as f32,
            (total_z / count as f64) as f32,
        )
    } else {
        glam::Vec3::new(0.0, 0.0, 6.378) // default surface
    };

    let target = target_center;
    let r = 6.378137f32; // Earth radius in Megameters
    let d = r * 3.0; // Zoomed out
    
    let normal = target_center.normalize_or_zero();
    let eye = normal * d;

    let extension = Box::new(crate::headless::route_builder::RoutesExtension::new(&extension_routes));
    let config = cesium_engine::globe::tiles::config::TileEngineConfig::default();

    let mut app = crate::headless::routes_headless_app::RoutesHeadlessApp {
        wgpu_state: None,
        window: None,
        config,
        frames_stable: 0,
        total_frames: 0,
        setup_done: false,
        width,
        height,
        out_path: path_str,
        extension: Some(extension),
        initial_cam_pos: eye,
        initial_cam_target: target,
    };

    let event_loop = winit::event_loop::EventLoop::new().unwrap();
    event_loop.set_control_flow(winit::event_loop::ControlFlow::Poll);
    
    if let Err(e) = event_loop.run_app(&mut app) {
        error!("Event loop error: {:?}", e);
        return false;
    }

    true
}
