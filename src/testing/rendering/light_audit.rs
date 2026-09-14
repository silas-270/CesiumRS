//! Headless capture sweep across flight phases and camera modes, for eyeballing the
//! lighting. Not an assertion — it writes PNGs for a human (or a model) to look at.
//!
//! Run with `cargo test --lib light_audit -- --nocapture --ignored`.

use cesium_engine::camera::camera::CameraMode;
use cesium_engine::globe::tiles::config::TileEngineConfig;
use cesium_engine::render::wgpu_state::WgpuState;

async fn shoot(
    mode: CameraMode,
    progress: f64,
    out: &str,
    turn_orbit_rad: Option<f32>,
    pitch_orbit_rad: Option<f32>,
    zoom_factor: Option<f32>,
) {
    shoot_with_config(mode, progress, out, turn_orbit_rad, pitch_orbit_rad, zoom_factor, TileEngineConfig::default()).await;
}

async fn shoot_with_config(
    mode: CameraMode,
    progress: f64,
    out: &str,
    turn_orbit_rad: Option<f32>,
    pitch_orbit_rad: Option<f32>,
    zoom_factor: Option<f32>,
    tile_config: TileEngineConfig,
) {
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
        tile_config,
        Some(flight_app),
    )
    .await;

    let aspect = state.size.width as f32 / state.size.height as f32;
    for i in 0..10 {
        if i == 1 {
            if let Some(angle) = turn_orbit_rad {
                state.camera.orbit_anchor(glam::Quat::from_rotation_y(angle));
            }
            if let Some(pitch) = pitch_orbit_rad {
                state.camera.orbit_anchor(glam::Quat::from_rotation_x(pitch));
            }
            if let Some(zf) = zoom_factor {
                state.camera.local_pos *= zf;
            }
        }
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
fn light_audit_sunset_track_turn_45() {
    let dir = std::env::var("LIGHT_AUDIT_DIR").unwrap_or_else(|_| "light_audit".to_string());
    std::fs::create_dir_all(&dir).unwrap();
    pollster::block_on(shoot(
        CameraMode::Tracking,
        0.17,
        &format!("{dir}/02_sunset_track.png"),
        Some(std::f32::consts::FRAC_PI_4),
        None,
        None,
    ));
}

#[test]
#[ignore = "writes PNGs; run explicitly"]
fn light_audit_sunset_360() {
    let dir = std::env::var("LIGHT_AUDIT_DIR").unwrap_or_else(|_| "light_audit".to_string());
    std::fs::create_dir_all(&dir).unwrap();
    for deg in [0, 45, 90, 135, 180, 225, 270, 315] {
        let rad = (deg as f32) * std::f32::consts::PI / 180.0;
        // Pitch up by 15 degrees (-0.26 rad) and bring camera 2x closer (0.5 zoom factor)
        pollster::block_on(shoot(
            CameraMode::Tracking,
            0.17,
            &format!("{dir}/02_sunset_track_{deg}deg.png"),
            Some(rad),
            Some(-0.26),
            Some(0.5),
        ));
    }
}

#[test]
#[ignore = "writes PNGs; run explicitly"]
fn light_audit_night_360() {
    let dir = std::env::var("LIGHT_AUDIT_DIR").unwrap_or_else(|_| "light_audit".to_string());
    std::fs::create_dir_all(&dir).unwrap();
    for deg in [0, 45, 90, 135, 180, 225, 270, 315] {
        let rad = (deg as f32) * std::f32::consts::PI / 180.0;
        pollster::block_on(shoot(
            CameraMode::Tracking,
            0.50,
            &format!("{dir}/03_night_track_{deg}deg.png"),
            Some(rad),
            Some(-0.26),
            Some(0.5),
        ));
    }
    pollster::block_on(shoot(
        CameraMode::Cockpit,
        0.50,
        &format!("{dir}/03_night_cockpit.png"),
        None,
        None,
        None,
    ));
}

#[test]
#[ignore = "writes PNGs; run explicitly"]
fn light_audit_progress_021() {
    let dir = std::env::var("LIGHT_AUDIT_DIR").unwrap_or_else(|_| "light_audit".to_string());
    std::fs::create_dir_all(&dir).unwrap();
    pollster::block_on(shoot(
        CameraMode::Tracking,
        0.21,
        &format!("{dir}/progress_021_tracking.png"),
        Some(-1.8),
        Some(0.40),
        Some(0.5),
    ));
}

#[test]
#[ignore = "writes PNGs; run explicitly"]
fn light_audit_cockpit_03533() {
    let dir = std::env::var("LIGHT_AUDIT_DIR").unwrap_or_else(|_| "light_audit".to_string());
    std::fs::create_dir_all(&dir).unwrap();
    pollster::block_on(shoot(
        CameraMode::Cockpit,
        0.3533,
        &format!("{dir}/cockpit_progress_03533.png"),
        None,
        None,
        None,
    ));
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
            pollster::block_on(shoot(mode, progress, &out, None, None, None));
        }
    }
}

#[test]
#[ignore = "writes PNGs; run explicitly"]
fn light_audit_dark_vs_satellite_sweep() {
    use cesium_engine::globe::tiles::config::SATELLITE_IMAGERY_URL;

    let dir = std::env::var("LIGHT_AUDIT_DIR").unwrap_or_else(|_| "light_audit".to_string());
    std::fs::create_dir_all(&dir).unwrap();

    let progresses = [0.0, 0.2, 0.4, 0.6, 0.8, 1.0];

    // 1. Dark Matter (Standard) Map
    for &p in &progresses {
        let out = format!("{dir}/dark_{:.1}.png", p);
        pollster::block_on(shoot_with_config(
            CameraMode::Tracking,
            p,
            &out,
            None,
            None,
            None,
            TileEngineConfig::default(),
        ));
    }

    // 2. Satellite (Esri) Map
    let sat_config = TileEngineConfig {
        base_imagery_url: SATELLITE_IMAGERY_URL.to_string(),
        ..Default::default()
    };
    for &p in &progresses {
        let out = format!("{dir}/sat_{:.1}.png", p);
        pollster::block_on(shoot_with_config(
            CameraMode::Tracking,
            p,
            &out,
            None,
            None,
            None,
            sat_config.clone(),
        ));
    }
}
