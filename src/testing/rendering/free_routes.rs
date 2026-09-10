//! Headless capture of flight routes in Free camera mode at laptop native resolution.
//!
//! Renders short and long routes from Free mode, where the camera automatically
//! frames the entire route across the globe.

use std::path::PathBuf;
use cesium_engine::camera::camera::CameraMode;
use cesium_engine::globe::tiles::config::TileEngineConfig;
use cesium_engine::render::wgpu_state::WgpuState;
use crate::testing::VerifyConfig;

pub struct RouteShot {
    pub id: &'static str,
    pub name: &'static str,
    pub dep_lon: f64,
    pub dep_lat: f64,
    pub arr_lon: f64,
    pub arr_lat: f64,
    pub duration_ms: u64,
    pub out_filename: &'static str,
}

async fn render_route(shot: &RouteShot, out_dir: &PathBuf) {
    let out_path = out_dir.join(shot.out_filename);
    let out_str = out_path.to_str().unwrap().to_string();

    let mut flight_app = Box::new(cesium_flight::tracker::FlightTrackerApp::new(
        std::sync::Arc::new(std::sync::Mutex::new(0.5)),
    ));

    flight_app.add_flight_path(
        shot.id,
        shot.dep_lon,
        shot.dep_lat,
        shot.arr_lon,
        shot.arr_lat,
        shot.duration_ms,
        false,
        Vec::new(),
    );

    flight_app.view_mode = CameraMode::Free;
    flight_app.last_view_mode = CameraMode::Tracking;
    flight_app.reset_viewport = true;

    let mut state = WgpuState::new(
        None,
        Some(winit::dpi::PhysicalSize::new(1920, 1080)),
        TileEngineConfig::default(),
        Some(flight_app),
    )
    .await;

    let aspect_ratio = state.size.width as f32 / state.size.height as f32;

    // Settle camera, build route mesh, and stream tiles in
    for _ in 0..15 {
        let view_proj =
            state.camera.get_projection_matrix(aspect_ratio) * state.camera.get_view_matrix();
        let visible = state.update_logic(aspect_ratio, view_proj);
        state
            .tile_system
            .texture_manager
            .fetch_and_upload_all(&state.device, &state.queue, &visible)
            .await;
        std::thread::sleep(std::time::Duration::from_millis(150));
    }

    println!(
        "[{}] Camera alt {:.3} Mm, pos {:?}",
        shot.name,
        state.camera.altitude(),
        state.camera.local_pos
    );

    #[cfg(feature = "debug_panel")]
    let res = state.render(Some(&out_str), false, |_, _| {});
    #[cfg(not(feature = "debug_panel"))]
    let res = state.render(Some(&out_str), false);
    res.expect("headless render failed");

    println!("Saved image to {}", out_str);
}

pub fn run(_config: VerifyConfig) {
    let home = std::env::var("HOME").unwrap_or_else(|_| "/home/silas270".to_string());
    let out_dir = PathBuf::from(home).join("img");
    std::fs::create_dir_all(&out_dir).unwrap();

    let routes = [
        // Short routes
        RouteShot {
            id: "flight_FRA_STR",
            name: "Frankfurt to Stuttgart (Short, ~160km)",
            dep_lon: 8.5706,
            dep_lat: 50.0333,
            arr_lon: 9.2219,
            arr_lat: 48.6899,
            duration_ms: 1_800_000,
            out_filename: "route_short_fra_str.png",
        },
        RouteShot {
            id: "flight_LHR_CDG",
            name: "London to Paris (Short, ~340km)",
            dep_lon: -0.4619,
            dep_lat: 51.4706,
            arr_lon: 2.5479,
            arr_lat: 49.0097,
            duration_ms: 2_400_000,
            out_filename: "route_short_lhr_cdg.png",
        },
        RouteShot {
            id: "flight_ZRH_GVA",
            name: "Zurich to Geneva (Short, ~230km)",
            dep_lon: 8.5555,
            dep_lat: 47.4581,
            arr_lon: 6.1092,
            arr_lat: 46.2370,
            duration_ms: 2_100_000,
            out_filename: "route_short_zrh_gva.png",
        },
        // Long routes
        RouteShot {
            id: "flight_JFK_LHR",
            name: "New York to London (Long, ~5500km)",
            dep_lon: -73.7781,
            dep_lat: 40.6413,
            arr_lon: -0.4619,
            arr_lat: 51.4706,
            duration_ms: 25_200_000,
            out_filename: "route_long_jfk_lhr.png",
        },
        RouteShot {
            id: "flight_LHR_NRT",
            name: "London to Tokyo (Long, ~9600km)",
            dep_lon: -0.4619,
            dep_lat: 51.4706,
            arr_lon: 140.3864,
            arr_lat: 35.7647,
            duration_ms: 43_200_000,
            out_filename: "route_long_lhr_nrt.png",
        },
        RouteShot {
            id: "flight_DXB_SYD",
            name: "Dubai to Sydney (Long, ~12000km)",
            dep_lon: 55.3657,
            dep_lat: 25.2532,
            arr_lon: 151.1753,
            arr_lat: -33.9399,
            duration_ms: 50_400_000,
            out_filename: "route_long_dxb_syd.png",
        },
        RouteShot {
            id: "flight_SIN_LHR",
            name: "Singapore to London (Long, ~11000km)",
            dep_lon: 103.9915,
            dep_lat: 1.3644,
            arr_lon: -0.4619,
            arr_lat: 51.4706,
            duration_ms: 46_800_000,
            out_filename: "route_long_sin_lhr.png",
        },
    ];

    for shot in &routes {
        pollster::block_on(render_route(shot, &out_dir));
    }
}
