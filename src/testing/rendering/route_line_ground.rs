//! Headless captures of the route line where the flight meets the ground.
//!
//! Near the airports the aircraft and its route line are both moved onto the drawn
//! terrain, by one rule (`cesium_flight::terrain_fit`). These frames are where that is
//! looked at: the aircraft standing at the start and the end of FRA-STR, rolling, lifting
//! off and on short final — from the side and low, where a height difference between the
//! line and the aircraft is plainest, and from the default tracking view near and far.
//!
//! `#[ignore]`d because it needs the network (imagery and heights) and writes PNGs:
//!
//! ```text
//! CESIUM_SHOT_DIR=/tmp/shots \
//!   cargo test --release --lib rendering::route_line_ground -- --ignored --nocapture
//! ```

use cesium_engine::camera::camera::CameraMode;
use cesium_engine::globe::tiles::config::{TerrainConfig, TileEngineConfig, SATELLITE_IMAGERY_URL};
use cesium_engine::render::wgpu_state::WgpuState;
use cesium_flight::tracker::FlightTrackerApp;

struct Shot {
    name: &'static str,
    progress: f64,
    /// Camera distance from the orbit centre, metres.
    dist_m: f32,
    /// Camera elevation above the aircraft's horizontal, degrees.
    pitch_deg: f32,
    /// Camera bearing around the aircraft, degrees: 0 behind, 90 off the left wing.
    yaw_deg: f32,
}

const fn shot(name: &'static str, progress: f64, dist_m: f32, pitch_deg: f32, yaw_deg: f32) -> Shot {
    Shot {
        name,
        progress,
        dist_m,
        pitch_deg,
        yaw_deg,
    }
}

/// FRA-STR is 1800 s: lift-off is at ~75 s, 30 m above the field at ~87 s and 300 m at
/// ~114 s; the last 30 m of the approach start at ~1739 s and touchdown is at ~1752 s.
///
/// The "side" views are not lower than 12°: the line is a flat ribbon, and from much
/// lower it is seen edge-on and vanishes.
const SHOTS: &[Shot] = &[
    shot("fra_start_side", 0.0, 150.0, 12.0, 75.0),
    shot("fra_start_behind", 0.0, 120.0, 8.0, 0.0),
    shot("fra_start_default", 0.0, 250.0, 22.0, 45.0),
    shot("fra_start_1km", 0.0, 1_000.0, 22.0, 45.0),
    shot("fra_start_3km", 0.0, 3_000.0, 22.0, 45.0),
    shot("fra_roll_side", 0.03, 150.0, 12.0, 75.0),
    shot("fra_liftoff_side", 0.0489, 150.0, 12.0, 75.0),
    shot("fra_climb300_default", 0.0633, 250.0, 22.0, 45.0),
    shot("str_final_side", 0.9672, 150.0, 12.0, 75.0),
    shot("str_end_side", 1.0, 150.0, 12.0, 75.0),
    shot("str_end_behind", 1.0, 120.0, 8.0, 0.0),
    shot("str_end_default", 1.0, 250.0, 22.0, 45.0),
    shot("str_end_1km", 1.0, 1_000.0, 22.0, 45.0),
];

async fn render(shot: &Shot, out_path: &str) {
    let mut app = Box::new(FlightTrackerApp::new(std::sync::Arc::new(std::sync::Mutex::new(
        shot.progress,
    ))));
    // The preset exactly as the viewer loads it, headings and elevations included.
    app.load_route(cesium_flight::preset::parse_route("FRA-STR").unwrap());
    app.view_mode = CameraMode::Tracking;
    app.last_view_mode = CameraMode::Free;
    app.reset_viewport = true;

    let config = TileEngineConfig {
        base_imagery_url: SATELLITE_IMAGERY_URL.to_string(),
        target_texel_ratio: 1.0,
        terrain: TerrainConfig {
            enabled: true,
            ..TerrainConfig::default()
        },
        ..TileEngineConfig::default()
    };
    let mut state = WgpuState::new(
        None,
        Some(winit::dpi::PhysicalSize::new(1280, 720)),
        config,
        Some(app),
    )
    .await;
    let aspect = state.size.width as f32 / state.size.height as f32;

    // The first pass puts the camera on the aircraft at the mode's default framing; the
    // pose replaces it, and later passes keep it (only a reset re-frames).
    let view_proj = state.camera.get_projection_matrix(aspect) * state.camera.get_view_matrix();
    state.update_logic(aspect, view_proj);
    let d = shot.dist_m / 1_000_000.0;
    let (pitch, yaw) = (shot.pitch_deg.to_radians(), shot.yaw_deg.to_radians());
    state.camera.local_pos = glam::Vec3::new(
        -d * pitch.cos() * yaw.sin(),
        d * pitch.sin(),
        d * pitch.cos() * yaw.cos(),
    );
    state.camera.look_at_plane();

    // Settle as `terrain_capture::render_settled` does: nothing missing, nothing loading,
    // then a few quiet frames for meshes finished on a worker to reach the GPU.
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(120);
    let mut quiet = 0;
    loop {
        let view_proj =
            state.camera.get_projection_matrix(aspect) * state.camera.get_view_matrix();
        let visible = state.update_logic(aspect, view_proj);
        state
            .tile_system
            .texture_manager
            .fetch_and_upload_all(&state.device, &state.queue, &visible)
            .await;
        let settled =
            state.last_missing_tiles_count == 0 && state.tile_system.is_loading_complete();
        quiet = if settled { quiet + 1 } else { 0 };
        if quiet >= 8 || std::time::Instant::now() >= deadline {
            break;
        }
        std::thread::sleep(std::time::Duration::from_millis(50));
    }

    #[cfg(feature = "debug_panel")]
    let res = state.render(Some(out_path), false, |_, _| {});
    #[cfg(not(feature = "debug_panel"))]
    let res = state.render(Some(out_path), false);
    res.expect("headless render");
}

#[test]
#[ignore = "visual verification: needs the network for imagery and heights, writes PNGs"]
fn capture_route_line_on_the_ground() {
    let dir = crate::testing::rendering::terrain_capture::shot_dir();
    println!("  writing captures to {}", dir.display());
    for s in SHOTS {
        let out = dir.join(format!("{}.png", s.name));
        pollster::block_on(render(s, out.to_str().unwrap()));
        println!("  {} (progress {:.4}, {} m)", s.name, s.progress, s.dist_m);
    }
}
