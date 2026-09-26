//! Headless visual check of the city labels' occlusion rules
//! (`cesium_engine::render::label_pipeline`), one shot per camera mode.
//!
//! Not an assertion: it writes PNGs to look at. What each should show:
//! * `free_*`: labels over the globe exactly as before, with nothing covering them.
//! * `tracking_top_london`: the aircraft over London, cutting London's label along its
//!   own outline.
//! * `tracking_chase`: a chase view; distant labels drawn, the aircraft unobstructed.
//! * `cockpit_approach`: labels only inside the window openings; the panel, frame and
//!   screens cover them.
//! * `debug_camera`: the same rules seen from the debug god-camera.
//!
//! Run with `cargo test --release --lib label_capture -- --nocapture --ignored`.
//! The cockpit GLB is read from `assets/`, so run it from the repository root.

use cesium_engine::camera::camera::CameraMode;
use cesium_engine::globe::tiles::config::{TerrainConfig, TileEngineConfig};
use cesium_engine::render::wgpu_state::WgpuState;
use cesium_flight::tracker::FlightTrackerApp;
use glam::Vec3;
use std::sync::{Arc, Mutex};

const WIDTH: u32 = 1280;
const HEIGHT: u32 = 720;

enum View {
    Free { eye: Vec3, target: Vec3 },
    /// Tracking, camera at `dist_m` from the aircraft, `pitch_deg` above its horizon.
    Tracking { dist_m: f32, pitch_deg: f32, yaw_deg: f32 },
    Cockpit,
    /// Debug god-camera `height_m` up, south of London, looking at it.
    Debug { height_m: f32 },
}

/// The engine's ECEF frame: y is the polar axis, z points to −90° longitude.
fn ecef(lat_deg: f32, lon_deg: f32, height_m: f32) -> Vec3 {
    let (phi, theta) = (lat_deg.to_radians(), lon_deg.to_radians());
    let r = 6.378137 + height_m / 1_000_000.0;
    Vec3::new(r * phi.cos() * theta.cos(), r * phi.sin(), -r * phi.cos() * theta.sin())
}

fn config() -> TileEngineConfig {
    TileEngineConfig {
        terrain: TerrainConfig {
            enabled: false,
            ..TerrainConfig::default()
        },
        ..TileEngineConfig::default()
    }
}

async fn shoot(name: &str, dir: &str, progress: f64, view: View) {
    let mut flight_app = Box::new(FlightTrackerApp::new(Arc::new(Mutex::new(progress))));
    flight_app.load_route(cesium_flight::preset::parse_route("JFK-LHR").unwrap());
    let mode = match view {
        View::Cockpit => CameraMode::Cockpit,
        View::Free { .. } => CameraMode::Free,
        _ => CameraMode::Tracking,
    };
    flight_app.view_mode = mode;
    flight_app.last_view_mode = if mode == CameraMode::Cockpit { CameraMode::Free } else { mode };
    flight_app.reset_viewport = mode == CameraMode::Cockpit;

    let mut state = WgpuState::new(
        None,
        Some(winit::dpi::PhysicalSize::new(WIDTH, HEIGHT)),
        config(),
        Some(flight_app),
    )
    .await;
    let aspect = WIDTH as f32 / HEIGHT as f32;

    for _ in 0..12 {
        match &view {
            View::Free { eye, target } => {
                state.camera.mode = CameraMode::Free;
                state.camera.set_eye(*eye, *target);
            }
            View::Tracking { dist_m, pitch_deg, yaw_deg } => {
                let (d, p, y) = (dist_m / 1_000_000.0, pitch_deg.to_radians(), yaw_deg.to_radians());
                state.camera.mode = CameraMode::Tracking;
                state.camera.local_pos = Vec3::new(d * p.cos() * y.sin(), d * p.sin(), d * p.cos() * y.cos());
                state.camera.look_at_plane();
            }
            View::Debug { height_m } => {
                let pos = ecef(50.3, -0.3, *height_m);
                let dir = (ecef(51.5, -0.12, 0.0) - pos).normalize();
                state.debug_mode = true;
                state.debug_camera_initialized = true;
                state.debug_camera.position = pos;
                state.debug_camera.pitch = dir.y.asin();
                state.debug_camera.yaw = dir.x.atan2(-dir.z);
            }
            View::Cockpit => {}
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

    let out = format!("{dir}/{name}.png");
    #[cfg(feature = "debug_panel")]
    let res = state.render(Some(&out), false, |_, _| {});
    #[cfg(not(feature = "debug_panel"))]
    let res = state.render(Some(&out), false);
    res.expect("headless render failed");

    // Steady-state cost of building the label instances, once glyphs and layouts are
    // cached: what every frame after the first pays.
    const FRAMES: usize = 100;
    let mut total_us = 0.0;
    for _ in 0..FRAMES {
        #[cfg(feature = "debug_panel")]
        let res = state.render(None, false, |_, _| {});
        #[cfg(not(feature = "debug_panel"))]
        let res = state.render(None, false);
        res.expect("headless render failed");
        total_us += state.last_subsystem_timings.label_render_us;
    }
    println!(
        "[{name}] mode {:?}  visible labels {}  instances {}  label build {:.1} us/frame -> {out}",
        state.camera.mode,
        state.label_manager.visible_labels.len(),
        state.label_renderer.instance_count(),
        total_us / FRAMES as f64,
    );
}

#[test]
#[ignore = "writes PNGs; run explicitly"]
fn label_capture() {
    let dir = std::env::var("LABEL_CAPTURE_DIR").unwrap_or_else(|_| "label_capture".to_string());
    std::fs::create_dir_all(&dir).unwrap();

    let shots: Vec<(&str, f64, View)> = vec![
        (
            "free_england",
            0.985,
            View::Free {
                eye: ecef(51.0, -1.0, 400_000.0),
                target: ecef(51.8, -0.5, 0.0),
            },
        ),
        (
            "tracking_top_london",
            0.989,
            View::Tracking { dist_m: 300_000.0, pitch_deg: 85.0, yaw_deg: 0.0 },
        ),
        (
            "tracking_chase",
            0.98,
            View::Tracking { dist_m: 8_000.0, pitch_deg: 25.0, yaw_deg: 180.0 },
        ),
        ("cockpit_approach", 0.975, View::Cockpit),
        ("debug_camera", 0.985, View::Debug { height_m: 150_000.0 }),
    ];
    for (name, progress, view) in shots {
        pollster::block_on(shoot(name, &dir, progress, view));
    }
}
