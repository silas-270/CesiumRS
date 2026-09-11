//! Headless capture sweep across flight phases and camera modes, for eyeballing the
//! lighting. Not an assertion — it writes PNGs for a human (or a model) to look at.
//!
//! Run with `cargo test --lib light_audit -- --nocapture --ignored`.

use cesium_engine::camera::camera::CameraMode;
use cesium_engine::globe::tiles::config::TileEngineConfig;
use cesium_engine::render::wgpu_state::WgpuState;

async fn shoot(mode: CameraMode, progress: f64, out: &str) {
    let mut flight_app = Box::new(cesium_flight::tracker::FlightTrackerApp::new(
        std::sync::Arc::new(std::sync::Mutex::new(progress)),
    ));
    // Frankfurt to Stuttgart over 30 minutes: short enough that a handful of progress
    // values covers taxi, climb, cruise and approach.
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
    flight_app.view_mode = mode;
    flight_app.last_view_mode = if mode == CameraMode::Free {
        CameraMode::Tracking
    } else {
        CameraMode::Free
    };
    flight_app.reset_viewport = true;

    let mut state = WgpuState::new(
        None,
        Some(winit::dpi::PhysicalSize::new(960, 540)),
        TileEngineConfig::default(),
        Some(flight_app),
    )
    .await;

    let aspect = state.size.width as f32 / state.size.height as f32;
    for _ in 0..10 {
        let vp = state.camera.get_projection_matrix(aspect) * state.camera.get_view_matrix();
        let visible = state.update_logic(aspect, vp);
        state
            .tile_system
            .texture_manager
            .fetch_and_upload_all(&state.device, &state.queue, &visible)
            .await;
        std::thread::sleep(std::time::Duration::from_millis(120));
    }

    println!(
        "[{:?} @ {:.2}] camera altitude {:.6} Mm  depth(sun_intensity)={:.3} -> {}",
        mode,
        progress,
        state.camera.altitude(),
        state.camera.sun_intensity,
        out
    );

    #[cfg(feature = "debug_panel")]
    let res = state.render(Some(out), false, |_, _| {});
    #[cfg(not(feature = "debug_panel"))]
    let res = state.render(Some(out), false);
    res.expect("headless render failed");
}

#[test]
#[ignore = "writes PNGs; run explicitly"]
fn light_audit_sweep() {
    let dir = std::env::var("LIGHT_AUDIT_DIR").unwrap_or_else(|_| "light_audit".to_string());
    std::fs::create_dir_all(&dir).unwrap();
    // Taxi, climb-out, cruise, and short final.
    for (label, progress) in [
        ("00_taxi", 0.02),
        ("01_climb", 0.10),
        ("02_sunset", 0.17),
        ("03_cruise", 0.50),
        ("04_descent", 0.90),
        ("05_final", 0.97),
    ] {
        for (mname, mode) in [
            ("free", CameraMode::Free),
            ("track", CameraMode::Tracking),
            ("cockpit", CameraMode::Cockpit),
        ] {
            let out = format!("{dir}/{label}_{mname}.png");
            pollster::block_on(shoot(mode, progress, &out));
        }
    }
}
