//! Runtime switching into and out of the offline SVG map, as the debug panel's
//! "Map Style" radio does it: Standard → Offline → Standard on one live `WgpuState`.
//!
//! Checks that the tile source mode in the config follows each switch (the panel reads it
//! to show which style is active) and writes one PNG per stage for a visual check.
//! `#[ignore]`d because Standard needs the network and it writes PNGs:
//!
//! ```text
//! CESIUM_SHOT_DIR=/tmp/shots \
//!   cargo test --lib rendering::offline_switch -- --ignored --nocapture
//! ```

use cesium_engine::globe::tiles::config::{TileEngineConfig, TileSourceMode, STANDARD_IMAGERY_URL};
use cesium_engine::globe::tiles::vector;
use cesium_engine::render::wgpu_state::WgpuState;
use std::sync::Arc;

/// Renders until every visible tile is resident (or 60 s pass), then writes `out_path`.
async fn render_settled(state: &mut WgpuState<'_>, out_path: &str) {
    let aspect = state.size.width as f32 / state.size.height as f32;
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(60);
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
            println!("  {out_path}: {} tiles visible, {} missing", visible.len(), state.last_missing_tiles_count);
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

fn is_offline(state: &WgpuState<'_>) -> bool {
    matches!(state.tile_system.config.tile_source_mode, TileSourceMode::SvgVector(_))
}

/// Mean absolute per-channel RGB difference of two same-sized captures.
fn mean_diff(a: &str, b: &str) -> f64 {
    let a = image::open(a).unwrap().into_rgb8();
    let b = image::open(b).unwrap().into_rgb8();
    let sum: u64 = a
        .as_raw()
        .iter()
        .zip(b.as_raw())
        .map(|(x, y)| (*x as i32 - *y as i32).unsigned_abs() as u64)
        .sum();
    sum as f64 / a.as_raw().len() as f64
}

async fn run(dir: &std::path::Path) {
    let mut state = WgpuState::new(
        None,
        Some(winit::dpi::PhysicalSize::new(1280, 720)),
        TileEngineConfig::default(),
        None,
    )
    .await;
    let path = |name: &str| dir.join(name).to_str().unwrap().to_string();

    // Straight down on central Europe from 2 500 km: coastlines, seas and borders all
    // in frame. Free mode, since Tracking clamps the camera to 20 km of its anchor.
    let ground = cesium_engine::globe::geometry::lon_lat_alt_to_ecef_f64(10.0, 48.0, 0.0);
    let ground = glam::Vec3::new(ground[0] as f32, ground[1] as f32, ground[2] as f32);
    state.camera.mode = cesium_engine::camera::camera::CameraMode::Free;
    state.camera.set_eye_with_up(ground * (1.0 + 2.5 / ground.length()), ground, glam::Vec3::Y);

    assert!(!is_offline(&state));
    render_settled(&mut state, &path("switch_1_standard.png")).await;

    // What the panel's "Offline (SVG)" radio does.
    let renderer = vector::bundled_world_renderer().expect("bundled SVG parses");
    state.set_tile_source_mode(String::new(), TileSourceMode::SvgVector(Arc::new(renderer)));
    state.set_terrain_enabled(false);
    assert!(is_offline(&state), "config must report offline after switching in");
    render_settled(&mut state, &path("switch_2_offline.png")).await;

    // What the panel's "Standard" radio does.
    state.set_base_imagery_url(STANDARD_IMAGERY_URL.to_string());
    state.set_terrain_enabled(false);
    assert!(!is_offline(&state), "config must report HTTP after leaving offline");
    assert_eq!(state.tile_system.config.base_imagery_url, STANDARD_IMAGERY_URL);
    render_settled(&mut state, &path("switch_3_standard_again.png")).await;

    let std_vs_off = mean_diff(&path("switch_1_standard.png"), &path("switch_2_offline.png"));
    let std_vs_std = mean_diff(&path("switch_1_standard.png"), &path("switch_3_standard_again.png"));
    println!("  mean |diff| standard vs offline: {std_vs_off:.2}, standard vs standard again: {std_vs_std:.2}");
    assert!(std_vs_off > std_vs_std, "the offline capture should differ from standard");
}

#[test]
#[ignore = "visual verification: needs the network for standard imagery, writes PNGs"]
fn capture_offline_switch_round_trip() {
    let dir = crate::testing::rendering::terrain_capture::shot_dir();
    println!("  writing captures to {}", dir.display());
    pollster::block_on(run(&dir));
}
