//! Headless visual check for WP5 (`docs/pre-terrain-plan.md`) — per `AGENTS.md`,
//! anything touching rendering gets a headless capture before it is called done.
//! Not an assertion — it writes PNGs for a human (or a model) to look at, same
//! idiom as `light_audit.rs`.
//!
//! Two things to look for, per WP5's own verification requirement:
//! * At cruise altitude (10-12km, this product's real operating envelope), fog
//!   should visibly thin out tile density toward the horizon without holes.
//! * Crossing `FogConfig::max_height_m` (800km) should show no popping — no tile
//!   should appear or disappear between the frame just below and the frame just
//!   above, matching `test_wp5b_max_height_boundary_step`'s measured 0% step.
//!
//! Run with `cargo test --release --lib fog_capture -- --nocapture --ignored`.

use cesium_engine::camera::camera::CameraMode;
use cesium_engine::globe::tiles::config::{TerrainConfig, TileEngineConfig};
use cesium_engine::render::wgpu_state::WgpuState;

use crate::testing::culling::cameras::{build_camera, ViewParams};

/// The flat globe these captures were recorded against.
///
/// **Section 9 F4 flipped `TerrainConfig::enabled` on by default**, and this is an
/// instrument whose committed baseline predates it: it measures atmospheric fog against a known globe, not relief, and a
/// surface that moved under it would make every future comparison two changes wide. The
/// config is therefore stated rather than inherited — the same rule the LOD harness and
/// the culling gate already follow by constructing their trees explicitly.
///
/// The captures that *are* about relief (`terrain_capture`, `terrain_e1_capture`,
/// `terrain_e2_capture`, `terrain_e3_capture`) set the flag themselves, both ways.
fn flat_config() -> TileEngineConfig {
    TileEngineConfig {
        terrain: TerrainConfig {
            enabled: false,
            ..TerrainConfig::default()
        },
        ..TileEngineConfig::default()
    }
}

async fn shoot(params: &ViewParams, out: &str) {
    let mut state = WgpuState::new(
        None,
        Some(winit::dpi::PhysicalSize::new(params.width, params.height)),
        flat_config(),
        None,
    )
    .await;

    // The harness's own camera construction (`ViewParams` -> `Camera`), so this
    // capture is positioned exactly the way `src/testing/lod/`'s measurements are
    // — no separate camera-placement logic to drift out of step with them.
    state.camera = build_camera(params);

    let aspect = state.size.width as f32 / state.size.height as f32;
    for _ in 0..8 {
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
        "[{out}] alt={:.6}Mm fog_density={:.4e} tiles_visible={}",
        state.camera.altitude(),
        state.quadtree_manager.fog_density(),
        state.quadtree_manager.get_visible_tiles().len(),
    );

    #[cfg(feature = "debug_panel")]
    let res = state.render(Some(out), false, |_, _| {});
    #[cfg(not(feature = "debug_panel"))]
    let res = state.render(Some(out), false);
    res.expect("headless render failed");
}

fn base_params() -> ViewParams {
    ViewParams {
        sweep: "fog_capture",
        lat_deg: 48.0,
        lon_deg: 9.0,
        pitch_deg: 0.0,
        yaw_deg: 0.0,
        roll_deg: 0.0,
        width: 1920,
        height: 1080,
        mode: CameraMode::Free,
        ..ViewParams::default()
    }
}

#[test]
#[ignore = "writes PNGs; run explicitly"]
fn fog_capture_sweep() {
    let dir = std::env::var("FOG_CAPTURE_DIR").unwrap_or_else(|_| "fog_capture".to_string());
    std::fs::create_dir_all(&dir).unwrap();

    // Cruise altitude, nadir and a grazing look, to see fog thin the horizon.
    for (label, alt_m, pitch_deg) in [
        ("00_cruise_10km_nadir", 10_000.0, 0.0),
        ("01_cruise_10km_grazing80", 10_000.0, 80.0),
        ("02_cruise_12km_grazing80", 12_000.0, 80.0),
    ] {
        let p = ViewParams { alt_m, pitch_deg, ..base_params() };
        let out = format!("{dir}/{label}.png");
        pollster::block_on(shoot(&p, &out));
    }

    // Bracketing the 800km maxHeight cutoff, nadir. NOTE: `Camera::altitude()` (what
    // fog density is actually computed from) reads a few metres higher than the
    // `alt_m` requested here, so the true crossing is at requested alt_m ~= 799996,
    // not the nominal 800000 — see test_wp5b_max_height_boundary_step, which
    // locates it precisely by bisection before trusting a ladder built from round
    // numbers. This ladder is built from that same measured crossing.
    for (label, alt_m) in [
        ("10_pre_boundary_750km", 750_000.0),
        ("11_just_below_crossing_799995m", 799_995.0),
        ("12_just_above_crossing_799997m", 799_997.0),
        ("13_post_boundary_850km", 850_000.0),
    ] {
        let p = ViewParams { alt_m, ..base_params() };
        let out = format!("{dir}/{label}.png");
        pollster::block_on(shoot(&p, &out));
    }
}
