//! Headless visual check for the terrain's aerial haze across the whole zoom
//! range — anything touching rendering gets a headless capture
//! before it is called done. Not an assertion — it writes PNGs for a human (or a
//! model) to look at, same idiom as `light_audit.rs` and `fog_capture.rs`.
//!
//! What to look for: the ground must stay visible at every altitude. Haze belongs
//! to the air the view looks *through*, so a near-vertical look from orbit — which
//! crosses the whole atmosphere but only a scale height of actual air — must be
//! essentially clear, while a grazing look along the ground at any altitude hazes.
//! The failure this sweep exists to catch is haze that grows with zoom until the
//! whole globe is covered.
//!
//! Run with `cargo test --release --lib haze_capture -- --nocapture --ignored`.

use cesium_engine::camera::camera::CameraMode;
use cesium_engine::globe::tiles::config::{TerrainConfig, TileEngineConfig};
use cesium_engine::render::wgpu_state::WgpuState;

use crate::testing::culling::cameras::{build_camera, ViewParams};

/// The flat globe these captures were recorded against.
///
/// **`TerrainConfig::enabled` is on by default**, and this is an
/// instrument whose committed baseline predates it: it measures atmospheric haze against a known globe, not relief, and a
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

    // The harness's own camera construction (`ViewParams` -> `Camera`), exactly as
    // `fog_capture.rs` does it, so this capture is positioned the same way the
    // `src/testing/lod/` measurements are.
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
        "[{out}] alt={:.6}Mm pitch={:.0}deg tiles_visible={}",
        state.camera.altitude(),
        params.pitch_deg,
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
        sweep: "haze_capture",
        lat_deg: 48.0,
        lon_deg: 9.0,
        pitch_deg: 0.0,
        yaw_deg: 0.0,
        roll_deg: 0.0,
        width: 960,
        height: 540,
        mode: CameraMode::Free,
        ..ViewParams::default()
    }
}

#[test]
#[ignore = "writes PNGs; run explicitly"]
fn haze_capture_sweep() {
    let dir = std::env::var("HAZE_CAPTURE_DIR").unwrap_or_else(|_| "haze_capture".to_string());
    std::fs::create_dir_all(&dir).unwrap();

    // The zoom-out ladder: cruise, then out past the atmosphere shell to a whole
    // hemisphere in frame. Nadir, where there is almost no air along the view ray.
    for (label, alt_m) in [
        ("00_nadir_10km", 10_000.0),
        ("01_nadir_100km", 100_000.0),
        ("02_nadir_500km", 500_000.0),
        ("03_nadir_2000km", 2_000_000.0),
        ("04_nadir_10000km", 10_000_000.0),
        // `Camera::max_distance` — as far out as the user can ever zoom.
        ("05_nadir_30000km", 30_000_000.0),
    ] {
        let p = ViewParams { alt_m, ..base_params() };
        pollster::block_on(shoot(&p, &format!("{dir}/{label}.png")));
    }

    // The same ladder looking along the ground, where the haze SHOULD show.
    for (label, alt_m) in [
        ("10_grazing_10km", 10_000.0),
        ("11_grazing_100km", 100_000.0),
        ("12_grazing_2000km", 2_000_000.0),
    ] {
        let p = ViewParams { alt_m, pitch_deg: 80.0, ..base_params() };
        pollster::block_on(shoot(&p, &format!("{dir}/{label}.png")));
    }
}
