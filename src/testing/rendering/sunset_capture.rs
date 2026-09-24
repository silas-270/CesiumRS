//! Headless sunset sweep: the same handful of views rendered at fixed sun elevations,
//! for tuning the sky's twilight colours and the ground's golden-hour grading by eye.
//! Not an assertion — it writes PNGs, same idiom as `light_audit.rs`.
//!
//! The engine has no clock: the sun's elevation is a function of the depth scalar
//! (`Camera::sun_intensity`), see `render/celestial.rs`. So each elevation here is
//! turned back into the depth that produces it, which also means the depth-driven parts
//! of the look (the sky thinning with altitude, the map's saturation) come out exactly
//! as the real app shows them at that point in the flight.
//!
//! Run with
//! `SUNSET_DIR=out cargo test --lib sunset_capture -- --nocapture --ignored`.
//! `SUNSET_ELEVS=3,-3` and `SUNSET_VIEWS=g_sun,air_anti` narrow the sweep.

use cesium_engine::camera::camera::CameraMode;
use cesium_engine::globe::tiles::config::{TerrainConfig, TileEngineConfig, SATELLITE_IMAGERY_URL};
use cesium_engine::render::celestial;
use cesium_engine::render::wgpu_state::WgpuState;

use crate::testing::culling::cameras::{build_camera, ViewParams};

/// Mirrors the private constants in `render/celestial.rs`; the printed check after each
/// capture catches any drift.
const HORIZON_DEPTH: f32 = 0.55;
const PEAK_ELEVATION: f32 = 0.85;
const HORIZON_EASING: f32 = 1.5;

/// Inverse of `celestial::compute`'s depth -> sin(elevation) curve.
fn depth_for_elevation_deg(deg: f32) -> f32 {
    let s = deg.to_radians().sin();
    let u = (s.abs() / PEAK_ELEVATION).powf(1.0 / HORIZON_EASING);
    if s >= 0.0 {
        HORIZON_DEPTH + u * (1.0 - HORIZON_DEPTH)
    } else {
        HORIZON_DEPTH - u * HORIZON_DEPTH
    }
}

/// The sun is held at a fixed bearing, west and a little south (celestial.rs:
/// `-east * 0.92 - north * 0.39`). In `ViewParams` a positive yaw turns the view from
/// north toward west, so these point straight at it and straight away from it.
const YAW_SUN: f64 = 113.0;
const YAW_ANTI: f64 = -67.0;

struct View {
    name: &'static str,
    alt_m: f64,
    pitch_deg: f64,
    yaw_deg: f64,
}

const VIEWS: &[View] = &[
    // Standing on the ground looking at the sunset, and away from it.
    View { name: "g_sun", alt_m: 150.0, pitch_deg: 100.0, yaw_deg: YAW_SUN },
    View { name: "g_anti", alt_m: 150.0, pitch_deg: 102.0, yaw_deg: YAW_ANTI },
    // Looking high up, perpendicular to the sun, to see the zenith colour.
    View { name: "g_zenith", alt_m: 150.0, pitch_deg: 140.0, yaw_deg: 23.0 },
    // From a climbing aircraft: ground and sky in one frame.
    View { name: "air_sun", alt_m: 3000.0, pitch_deg: 84.0, yaw_deg: YAW_SUN },
    View { name: "air_anti", alt_m: 3000.0, pitch_deg: 80.0, yaw_deg: YAW_ANTI },
    // Mostly ground, from higher up, to judge the map's colour.
    View { name: "high_ground", alt_m: 9000.0, pitch_deg: 55.0, yaw_deg: 23.0 },
];

const ELEVATIONS: &[f32] = &[10.0, 3.0, 0.0, -3.0, -6.0, -10.0];

fn config() -> TileEngineConfig {
    TileEngineConfig {
        base_imagery_url: SATELLITE_IMAGERY_URL.to_string(),
        terrain: TerrainConfig { enabled: false, ..TerrainConfig::default() },
        ..TileEngineConfig::default()
    }
}

fn filter<T: Copy>(env: &str, all: &[T], parse: impl Fn(&str) -> Option<T>) -> Vec<T> {
    match std::env::var(env) {
        Ok(v) if !v.trim().is_empty() => v.split(',').filter_map(|s| parse(s.trim())).collect(),
        _ => all.to_vec(),
    }
}

#[test]
#[ignore = "writes PNGs; run explicitly"]
fn sunset_capture_sweep() {
    let dir = std::env::var("SUNSET_DIR").unwrap_or_else(|_| "sunset_capture".to_string());
    std::fs::create_dir_all(&dir).unwrap();
    let elevs = filter("SUNSET_ELEVS", ELEVATIONS, |s| s.parse().ok());
    let names: Vec<&str> = VIEWS.iter().map(|v| v.name).collect();
    let wanted = filter("SUNSET_VIEWS", &names, |s| names.iter().copied().find(|n| *n == s));
    let (w, h) = (640u32, 360u32);

    pollster::block_on(async {
        let mut state =
            WgpuState::new(None, Some(winit::dpi::PhysicalSize::new(w, h)), config(), None).await;
        let aspect = w as f32 / h as f32;

        for view in VIEWS.iter().filter(|v| wanted.contains(&v.name)) {
            let params = ViewParams {
                sweep: "sunset_capture",
                lat_deg: 47.85,
                lon_deg: 11.1,
                alt_m: view.alt_m,
                pitch_deg: view.pitch_deg,
                yaw_deg: view.yaw_deg,
                roll_deg: 0.0,
                width: w,
                height: h,
                mode: CameraMode::Free,
            };
            for &elev in &elevs {
                state.camera = build_camera(&params);
                let depth = depth_for_elevation_deg(elev);
                state.camera.sun_intensity = depth;
                for _ in 0..6 {
                    let vp = state.camera.get_projection_matrix(aspect)
                        * state.camera.get_view_matrix();
                    let visible = state.update_logic(aspect, vp);
                    state
                        .tile_system
                        .texture_manager
                        .fetch_and_upload_all(&state.device, &state.queue, &visible)
                        .await;
                    std::thread::sleep(std::time::Duration::from_millis(60));
                }
                let pos = state.camera.global_transform_f64().0;
                let check = celestial::compute(
                    depth,
                    glam::Vec3::new(pos.x as f32, pos.y as f32, pos.z as f32),
                );
                let out = format!("{dir}/{}_{:+03}.png", view.name, elev as i32);
                println!(
                    "[{out}] target {elev:+.1}deg depth {depth:.3} -> actual {:+.2}deg",
                    check.sun_elevation.asin().to_degrees()
                );
                #[cfg(feature = "debug_panel")]
                let res = state.render(Some(&out), false, |_, _| {});
                #[cfg(not(feature = "debug_panel"))]
                let res = state.render(Some(&out), false);
                res.expect("headless render failed");
            }
        }
    });
}
