//! Headless capture of the cockpit view at the Samsung S23's native resolution.
//!
//! Same render path as `cockpit_capture`, but sized to match the phone's display in
//! landscape (2340x1080) and portrait (1080x2340) so iterations on the interior's
//! placement, scale and materials can be checked from this machine without building
//! and flashing an APK.
//!
//! Driven by `cesium_app --cockpit-s23`. The cockpit GLB is read from `assets/`, so
//! run it from the repository root.

use cesium_engine::camera::camera::CameraMode;
use cesium_engine::globe::tiles::config::TileEngineConfig;
use cesium_engine::render::wgpu_state::WgpuState;

use crate::testing::VerifyConfig;

/// One capture: a physical pixel size matching the S23 in some orientation.
struct Shot {
    width: u32,
    height: u32,
    out_path: &'static str,
}

/// Renders one frame of the FRA→STR flight, from the cockpit seat, at `shot`'s size.
async fn render_shot(shot: &Shot, progress: f64) {
    let mut flight_app = Box::new(cesium_flight::tracker::FlightTrackerApp::new(
        std::sync::Arc::new(std::sync::Mutex::new(progress)),
    ));
    // Frankfurt to Stuttgart over 30 minutes, matching the desktop viewer's default.
    flight_app.add_flight_path(
        "flight_FRA_STR",
        8.5706,
        50.0333,
        9.2219,
        48.6899,
        1_800_000,
        false,
        Vec::new(),
    );
    // `view_mode != last_view_mode` is what makes the tracker push the mode onto the
    // camera; matching them instead lets the camera's own default win the sync.
    flight_app.view_mode = CameraMode::Cockpit;
    flight_app.last_view_mode = CameraMode::Free;
    flight_app.reset_viewport = true;

    let mut state = WgpuState::new(
        None,
        Some(winit::dpi::PhysicalSize::new(shot.width, shot.height)),
        TileEngineConfig::default(),
        Some(flight_app),
    )
    .await;

    let aspect_ratio = state.size.width as f32 / state.size.height as f32;

    // Several passes: the first materialises the flight and lazily loads the cockpit, the
    // rest settle the camera onto the aircraft and stream the terrain in.
    for _ in 0..10 {
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
        "[{}x{} {:?}] altitude {:.7} Mm, local_pos {:?}",
        shot.width, shot.height, state.camera.mode, state.camera.altitude(), state.camera.local_pos
    );

    #[cfg(feature = "debug_panel")]
    let res = state.render(Some(shot.out_path), false, |_, _| {});
    #[cfg(not(feature = "debug_panel"))]
    let res = state.render(Some(shot.out_path), false);
    res.expect("headless render failed");

    println!("wrote {}", shot.out_path);
}

pub fn run(_config: VerifyConfig) {
    let shots = [
        Shot {
            width: 2340,
            height: 1080,
            out_path: "cockpit_s23_landscape.png",
        },
        Shot {
            width: 1080,
            height: 2340,
            out_path: "cockpit_s23_portrait.png",
        },
    ];

    for shot in &shots {
        pollster::block_on(render_shot(shot, 0.5));
    }
}
