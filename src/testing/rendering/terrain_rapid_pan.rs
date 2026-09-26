//! Rapid camera movement with terrain on: what the update thread pays per frame while
//! the camera sweeps, and how long the ground takes to fill in once it stops.
//!
//! The pose the user complained about is not a pose, it is a *motion* — every static
//! capture in `terrain_capture` settles first and so never sees it. This one flies the
//! camera ~190 km along the Alps at 5 km altitude in 150 frames, then zooms out to
//! 300 km and back in 60 more, all paced at 60 Hz so fetches land between frames the
//! way they do live, and only then settles and renders.
//!
//! `#[ignore]`d: it needs the network and writes a PNG.
//!
//! ```text
//! CESIUM_SHOT_DIR=/tmp/shots \
//!   cargo test --release --lib rendering::terrain_rapid_pan -- --ignored --nocapture
//! ```

use cesium_engine::globe::tiles::config::{TerrainConfig, TileEngineConfig, satellite_imagery_url};
use std::time::{Duration, Instant};

use super::terrain_capture::{oblique, shot_dir};
use crate::testing::benchmark::percentiles;

const FRAME: Duration = Duration::from_millis(16);
const SWEEP_FRAMES: usize = 150;
const ZOOM_FRAMES: usize = 60;

fn config() -> TileEngineConfig {
    TileEngineConfig {
        base_imagery_url: satellite_imagery_url(),
        offline_mode: false,
        transparent_background: true,
        target_texel_ratio: 1.0,
        mesh_segments: 16,
        enable_prefetch: true,
        terrain: TerrainConfig {
            enabled: true,
            ..TerrainConfig::default()
        },
        ..TileEngineConfig::default()
    }
}

/// The camera pose at frame `i` of the scripted flight.
fn pose_at(i: usize) -> super::terrain_capture::Pose {
    if i < SWEEP_FRAMES {
        let t = i as f64 / SWEEP_FRAMES as f64;
        oblique("sweep", 10.0 + 2.5 * t, 47.2, 5_000.0, 10.0, 0.12, "")
    } else {
        let t = ((i - SWEEP_FRAMES) as f64 / ZOOM_FRAMES as f64).min(1.0);
        // 5 km → 300 km → 5 km, log-spaced so both halves are equally fast on screen.
        let k = 1.0 - (2.0 * t - 1.0).abs();
        let alt = 5_000.0 * (300_000.0f64 / 5_000.0).powf(k);
        oblique("zoom", 12.5, 47.2, alt, 10.0 + 50.0 * k, 0.12 + 1.0 * k, "")
    }
}

#[test]
#[ignore = "needs the network for imagery and heights, writes a PNG"]
fn rapid_pan_then_settle() {
    use cesium_engine::render::wgpu_state::WgpuState;

    pollster::block_on(async {
        let mut state = WgpuState::new(
            None,
            Some(winit::dpi::PhysicalSize::new(1280, 720)),
            config(),
            None,
        )
        .await;
        // Free, not the default Tracking: Tracking treats `local_pos` as an offset from
        // an aircraft anchor (at the origin here) and its clamp would drag the camera
        // there the first time a ground height lands.
        state.camera.mode = cesium_engine::camera::CameraMode::Free;
        let aspect = state.size.width as f32 / state.size.height as f32;

        let set_pose = |state: &mut WgpuState, p: &super::terrain_capture::Pose| match p.up {
            Some(up) => state.camera.set_eye_with_up(p.eye, p.target, up),
            None => state.camera.set_eye(p.eye, p.target),
        };

        let mut stream_ms = Vec::new();
        let mut update_ms = Vec::new();
        for i in 0..SWEEP_FRAMES + ZOOM_FRAMES {
            let frame_start = Instant::now();
            set_pose(&mut state, &pose_at(i));
            let vp = state.camera.get_projection_matrix(aspect) * state.camera.get_view_matrix();
            let t = Instant::now();
            state.update_logic(aspect, vp);
            update_ms.push(t.elapsed().as_secs_f64() * 1000.0);
            stream_ms.push(state.last_subsystem_timings.tile_streaming_us / 1000.0);
            if i % 30 == 0 {
                let (req, miss) = state.get_fetch_stats();
                println!(
                    "    frame {i:3}: visible {req:4} missing {miss:4} alt {:.0} m",
                    state.camera.altitude() * 1.0e6
                );
            }
            if let Some(rest) = FRAME.checked_sub(frame_start.elapsed()) {
                std::thread::sleep(rest);
            }
        }

        // Settle at the final pose: how long until the ground under it is complete.
        let final_pose = pose_at(SWEEP_FRAMES + ZOOM_FRAMES);
        set_pose(&mut state, &final_pose);
        let vp = state.camera.get_projection_matrix(aspect) * state.camera.get_view_matrix();
        let settle_start = Instant::now();
        let deadline = settle_start + Duration::from_secs(60);
        let mut quiet = 0;
        let mut settle_stream_ms = Vec::new();
        let mut settled_at = None;
        while Instant::now() < deadline {
            state.update_logic(aspect, vp);
            settle_stream_ms.push(state.last_subsystem_timings.tile_streaming_us / 1000.0);
            let settled =
                state.last_missing_tiles_count == 0 && state.tile_system.is_loading_complete();
            quiet = if settled { quiet + 1 } else { 0 };
            if quiet == 1 {
                settled_at = Some(settle_start.elapsed());
            }
            if quiet >= 8 {
                break;
            }
            std::thread::sleep(FRAME);
        }

        let s = percentiles(&stream_ms);
        let u = percentiles(&update_ms);
        let max = |v: &[f64]| v.iter().cloned().fold(0.0, f64::max);
        println!("  motion ({} frames):", stream_ms.len());
        println!(
            "    stream   avg {:6.2} ms  p90 {:6.2}  p99 {:6.2}  max {:7.2}",
            s.avg, s.p90, s.p99, max(&stream_ms)
        );
        println!(
            "    update   avg {:6.2} ms  p90 {:6.2}  p99 {:6.2}  max {:7.2}",
            u.avg, u.p90, u.p99, max(&update_ms)
        );
        println!(
            "    frames with stream > 16 ms: {}",
            stream_ms.iter().filter(|&&m| m > 16.0).count()
        );
        let (requested, _) = state.get_fetch_stats();
        println!(
            "  final pose: {} visible tiles, {} height tiles resident, {} imagery entries",
            requested,
            state
                .tile_system
                .height_manager
                .as_ref()
                .map(|h| h.residency().0)
                .unwrap_or(0),
            state.tile_system.texture_manager.cache.len(),
        );
        println!(
            "  settle: {} (stream max {:.2} ms over {} frames), missing={}",
            match settled_at {
                Some(d) => format!("{:.2} s", d.as_secs_f64()),
                None => "did not settle in 60 s".to_string(),
            },
            max(&settle_stream_ms),
            settle_stream_ms.len(),
            state.last_missing_tiles_count,
        );

        let out = shot_dir().join("terrain_rapid_pan_final.png");
        let out_str = out.to_string_lossy().into_owned();
        #[cfg(feature = "debug_panel")]
        let res = state.render(Some(&out_str), false, |_, _| {});
        #[cfg(not(feature = "debug_panel"))]
        let res = state.render(Some(&out_str), false);
        res.expect("headless render");
        println!("  -> {}", out.display());
    });
}
