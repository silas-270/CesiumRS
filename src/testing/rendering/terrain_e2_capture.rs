//! Headless captures for Phase E2 of `docs/terrain-plan.md` §8 — **does the ground
//! jump when a mesh is rebuilt?**
//!
//! Two tests, because E2 has two questions and only one of them can be answered by
//! flying the aircraft.
//!
//! **1. `capture_an_approach_into_innsbruck`** flies the real thing: a descent down the
//! Inn valley onto runway 26, eight steps from 6 km AGL to 100 m, each settled and
//! rendered, with the number of mesh rebuilds the engine actually performed printed
//! next to each. It also measures the thing the section is about directly — how much of
//! the frame changes on the *next* frame with the camera held perfectly still, which is
//! precisely "the ground moving underneath you". This is the empirical answer to
//! whether E2's case occurs in normal flight at all.
//!
//! **2. `capture_a_staged_rebuild_burst`** forces it, because the answer to (1) is
//! "almost never" and a mechanism nobody has seen work is a mechanism nobody should
//! trust. It stages the worst case the engine can produce — every visible tile's height
//! data failed, every mesh built from a z10 ancestor, and then all the real data
//! arriving in a single frame — and renders the drain frame by frame. What that pair of
//! pictures shows is the difference E2 makes; what the per-frame table shows is that
//! the difference is delivered in bounded instalments rather than all at once.
//!
//! Both are `#[ignore]`d: they need the network for imagery and heights, and they write
//! PNGs.
//!
//! ```text
//! CESIUM_SHOT_DIR=/tmp/shots \
//!   cargo test --release --lib rendering::terrain_e2_capture -- --ignored --nocapture
//! ```

use std::collections::HashMap;
use std::sync::Arc;

use cesium_engine::globe::quadtree::TileId;
use cesium_engine::globe::terrain::HeightTile;
use cesium_engine::globe::tiles::config::{TerrainConfig, TileEngineConfig, SATELLITE_IMAGERY_URL};
use cesium_engine::globe::tiles::system::MESH_REBUILD_BUDGET_PER_FRAME;
use cesium_engine::render::wgpu_state::WgpuState;

use super::terrain_capture::shot_dir;

const WIDTH: u32 = 1280;
const HEIGHT: u32 = 720;

/// Innsbruck, runway 26 threshold — the same ground `rendering::terrain_e3_capture`
/// lands on, so the two sets of pictures are of the same valley.
const LOWI: (f64, f64) = (11.3439, 47.2602);
/// Field elevation, metres. The DEM reads 579 m here; the published figure is 581 m.
const LOWI_ELEV_M: f64 = 581.0;
/// Down the valley, roughly the runway 26 heading.
const APPROACH_BEARING_DEG: f64 = 250.0;

/// The level the staged burst poisons from. Everything at z11 and below is allowed to
/// load, so a poisoned tile's `status_of` walks its failed chain up to a **z10**
/// ancestor and the whole valley is drawn from a continent-scale field — coarse enough
/// that the before/after pair is unambiguous in a still image.
const POISON_FROM_Z: u8 = 11;

fn ecef(lon_deg: f64, lat_deg: f64, alt_m: f64) -> glam::Vec3 {
    let p = cesium_engine::globe::geometry::lon_lat_alt_to_ecef_f64(lon_deg, lat_deg, alt_m);
    glam::Vec3::new(p[0] as f32, p[1] as f32, p[2] as f32)
}

fn up_at(lon_deg: f64, lat_deg: f64) -> glam::DVec3 {
    const A: f64 = 6.378137;
    const B: f64 = 6.3567523142;
    let p = cesium_engine::globe::geometry::lon_lat_to_ecef_f64(lon_deg, lat_deg);
    glam::DVec3::new(p[0] / (A * A), p[1] / (B * B), p[2] / (A * A)).normalize()
}

/// Eye, target and up for a camera `alt_m` above the ellipsoid at (`lon`, `lat`),
/// looking along a compass bearing, pitched `pitch_deg` down.
///
/// Same construction as `terrain_e3_capture::look`. The altitudes below are all
/// field elevation plus a height above ground, because a pose given in metres above the
/// *ellipsoid* over an alpine valley is a pose inside a mountain — the mistake §8
/// records two predecessors making.
fn look(
    lon: f64,
    lat: f64,
    alt_m: f64,
    bearing_deg: f64,
    pitch_deg: f64,
) -> (glam::Vec3, glam::Vec3, glam::Vec3) {
    let eye = ecef(lon, lat, alt_m);
    let up = up_at(lon, lat);
    let east = glam::DVec3::new(-lon.to_radians().sin(), 0.0, -lon.to_radians().cos());
    let north = up.cross(east).normalize();
    let b = bearing_deg.to_radians();
    let horizontal = (north * b.cos() + east * b.sin()).normalize();
    let pitch = pitch_deg.to_radians();
    let dir = (horizontal * pitch.cos() - up * pitch.sin()).normalize();
    (
        eye,
        eye + glam::Vec3::new(dir.x as f32, dir.y as f32, dir.z as f32) * 0.2,
        glam::Vec3::new(up.x as f32, up.y as f32, up.z as f32),
    )
}

/// The same config the Phase C/D/E captures use, terrain on.
fn config() -> TileEngineConfig {
    TileEngineConfig {
        base_imagery_url: SATELLITE_IMAGERY_URL.to_string(),
        offline_mode: false,
        transparent_background: true,
        target_texel_ratio: 1.0,
        mesh_segments: 16,
        terrain: TerrainConfig {
            enabled: true,
            exaggeration: 1.0,
            occlusion: cesium_engine::globe::quadtree::TerrainOcclusionConfig {
                enabled: true,
                ..Default::default()
            },
            ..TerrainConfig::default()
        },
        ..TileEngineConfig::default()
    }
}

async fn new_state(config: TileEngineConfig) -> WgpuState<'static> {
    WgpuState::new(
        None,
        Some(winit::dpi::PhysicalSize::new(WIDTH, HEIGHT)),
        config,
        None,
    )
    .await
}

/// One frame: advances `update_logic`, renders, and hands back the RGBA pixels.
///
/// `render` runs `update_logic` itself, so this is exactly one frame of engine time —
/// which is what the drain below counts in.
fn frame(state: &mut WgpuState<'_>) -> Vec<u8> {
    #[cfg(feature = "debug_panel")]
    let res = state.render(None, true, |_, _| {});
    #[cfg(not(feature = "debug_panel"))]
    let res = state.render(None, true);
    res.expect("headless render").expect("pixels requested")
}

fn write_png(pixels: &[u8], path: &std::path::Path) {
    let _ = image::save_buffer(path, pixels, WIDTH, HEIGHT, image::ColorType::Rgba8);
}

/// Fraction of pixels that differ between two frames, sampled every second pixel in
/// each axis — the same sampling `terrain_e1_capture`'s table uses, so the percentages
/// in the two sections are comparable.
fn differing_fraction(a: &[u8], b: &[u8]) -> f64 {
    let mut differ = 0usize;
    let mut total = 0usize;
    for y in (0..HEIGHT as usize).step_by(2) {
        for x in (0..WIDTH as usize).step_by(2) {
            let i = (y * WIDTH as usize + x) * 4;
            total += 1;
            if a[i..i + 4] != b[i..i + 4] {
                differ += 1;
            }
        }
    }
    differ as f64 / total.max(1) as f64
}

/// Settles a state on the same three conditions `terrain_capture::render_settled` uses:
/// no missing meshes, nothing loading anywhere, and a few quiet frames after that.
async fn settle(state: &mut WgpuState<'_>, aspect: f32, view_proj: glam::Mat4, secs: u64) -> usize {
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(secs);
    let mut quiet = 0;
    let mut visible = 0;
    loop {
        state
            .tile_system
            .texture_manager
            .fetch_and_upload_all(&state.device, &state.queue, &[])
            .await;
        visible = state.update_logic(aspect, view_proj).len();
        let settled =
            state.last_missing_tiles_count == 0 && state.tile_system.is_loading_complete();
        quiet = if settled { quiet + 1 } else { 0 };
        if quiet >= 8 || std::time::Instant::now() >= deadline {
            break;
        }
        std::thread::sleep(std::time::Duration::from_millis(50));
    }
    visible
}

// ── 1. The approach ──────────────────────────────────────────────────────────

/// Eight steps down the Inn valley onto runway 26, each settled and photographed, with
/// the engine's own rebuild counter read off at each one.
///
/// The column that answers §8's question is **still-frame delta**: the camera is held
/// exactly where it was, one more frame is rendered, and the pixels are compared. On a
/// settled globe that number is zero, and a non-zero one is literally the ground moving
/// while nobody moved the camera. It is measured after the settle rather than during
/// it, because during a descent the near field is *supposed* to change — tiles are
/// arriving — and E2's claim is about geometry that is already drawn.
#[test]
#[ignore = "visual verification: needs the network for imagery and heights, writes PNGs"]
fn capture_an_approach_into_innsbruck() {
    let dir = shot_dir();
    println!("  writing E2 approach captures to {}", dir.display());

    // Metres above the **field**, not above the ellipsoid: 6 km down to a 100 m flare.
    // The valley floor is at 581 m, so the last of these is an eye 681 m up the
    // ellipsoid and the first is 6 581 m.
    let agl_ladder = [6000.0, 3000.0, 1500.0, 800.0, 450.0, 250.0, 150.0, 100.0];

    println!(
        "\n  | step | eye AGL (m) | visible tiles | height tiles | rebuilds this frame | \
         cumulative rebuilds | still-frame delta |"
    );
    println!("  |--:|--:|--:|--:|--:|--:|--:|");

    let mut worst_still = 0.0f64;
    let mut total_rebuilds = 0usize;

    for (step, agl) in agl_ladder.iter().enumerate() {
        let (eye, target, up) = look(
            LOWI.0,
            LOWI.1,
            LOWI_ELEV_M + agl,
            APPROACH_BEARING_DEG,
            // Steeper the lower we get: a 3-degree glideslope view from 6 km would be
            // all horizon and no ground.
            (3.0 + 12.0 * (1.0 - agl / 6000.0)).min(15.0),
        );

        let mut state = pollster::block_on(new_state(config()));
        state.camera.set_eye_with_up(eye, target, up);
        let aspect = WIDTH as f32 / HEIGHT as f32;
        let view_proj = state.camera.get_projection_matrix(aspect) * state.camera.get_view_matrix();

        let visible = pollster::block_on(settle(&mut state, aspect, view_proj, 120));

        // Two consecutive frames with the camera untouched. Anything that differs
        // between them moved on its own.
        let a = frame(&mut state);
        let rebuilds_this_frame = state.last_mesh_rebuilds;
        let b = frame(&mut state);
        total_rebuilds += rebuilds_this_frame + state.last_mesh_rebuilds;
        let still = differing_fraction(&a, &b);
        worst_still = worst_still.max(still);

        let out = dir.join(format!("e2_approach_{step}_{:.0}m_agl.png", agl));
        write_png(&b, &out);

        let height_tiles = state
            .tile_system
            .height_manager
            .as_ref()
            .map(|h| h.residency().0)
            .unwrap_or(0);
        println!(
            "  | {step} | {agl:6.0} | {visible:4} | {height_tiles:4} | {rebuilds_this_frame} | \
             {total_rebuilds} | {:6.3} % |",
            still * 100.0
        );
    }

    println!(
        "\n  worst still-frame delta over the approach: {:.3} %, {total_rebuilds} rebuilds in total",
        worst_still * 100.0
    );
    println!(
        "  Read the eight frames in order: the valley floor must rise smoothly, with no step."
    );

    // A settled globe that is still rebuilding meshes under a stationary camera is
    // exactly the failure E2's rate limit and no-downgrade rule exist to prevent.
    assert!(
        worst_still < 0.01,
        "a settled, stationary frame changed by {:.3} % — the ground moved on its own",
        worst_still * 100.0
    );
}

// ── 2. The staged burst ──────────────────────────────────────────────────────

/// Every height tile the poses below need, harvested out of a normally-settled engine
/// so the burst can be delivered in one frame instead of waiting on 150 fetches.
///
/// `HeightTileManager::source_for` hands out the decoded `Arc` the cache holds, and
/// `insert_ready` puts one back — the same call the fetcher makes when a retry lands.
/// So the injection below is not a simulation of a retry succeeding; it is a retry
/// succeeding, with the network latency taken out.
fn harvest_height_tiles(
    state: &mut WgpuState<'_>,
    ids: &[TileId],
) -> HashMap<TileId, Arc<HeightTile>> {
    let mut out = HashMap::new();
    let Some(heights) = state.tile_system.height_manager.as_mut() else {
        return out;
    };
    for id in ids {
        let mut curr = heights.source_tile_for(*id);
        loop {
            if let Some((src, tile)) = heights.source_for(curr) {
                out.insert(src, tile);
            }
            match curr.parent() {
                Some(p) => curr = p,
                None => break,
            }
        }
    }
    out
}

/// Marks every height tile at or below `POISON_FROM_Z` in each visible tile's ancestor
/// chain as failed, so `status_of` has to walk past them to a z10 ancestor.
fn poison(state: &mut WgpuState<'_>, ids: &[TileId]) {
    let Some(heights) = state.tile_system.height_manager.as_mut() else {
        return;
    };
    for id in ids {
        let mut curr = heights.source_tile_for(*id);
        while curr.z >= POISON_FROM_Z {
            heights.insert_failed(curr);
            match curr.parent() {
                Some(p) => curr = p,
                None => break,
            }
        }
    }
}

/// The worst burst the engine can produce, photographed: every visible mesh built from
/// a z10 ancestor, then every real height tile arriving in one frame.
///
/// What the two stills show is what E2 buys — a valley floor that is a smooth bowl
/// becoming a valley floor. What the per-frame table shows is that it arrives in
/// instalments of at most [`MESH_REBUILD_BUDGET_PER_FRAME`], so no single frame carries
/// the whole change.
#[test]
#[ignore = "visual verification: needs the network for imagery and heights, writes PNGs"]
fn capture_a_staged_rebuild_burst() {
    let dir = shot_dir();
    println!("  writing E2 burst captures to {}", dir.display());

    // 900 m over the field, looking down the valley — high enough that the Nordkette
    // and both walls are in frame, low enough that the near field is at full zoom.
    let (eye, target, up) = look(
        LOWI.0,
        LOWI.1,
        LOWI_ELEV_M + 900.0,
        APPROACH_BEARING_DEG,
        9.0,
    );
    let aspect = WIDTH as f32 / HEIGHT as f32;

    // Phase 0 — a normal settle, to find out what this pose draws and to collect the
    // real height data for the injection.
    let mut warm = pollster::block_on(new_state(config()));
    warm.camera.set_eye_with_up(eye, target, up);
    let view_proj = warm.camera.get_projection_matrix(aspect) * warm.camera.get_view_matrix();
    let visible = pollster::block_on(settle(&mut warm, aspect, view_proj, 120));
    let ids: Vec<TileId> = warm
        .quadtree_manager
        .get_visible_tiles()
        .iter()
        .map(|(id, _, _)| *id)
        .collect();
    let real = harvest_height_tiles(&mut warm, &ids);
    println!(
        "  reference settle: {visible} visible tiles, {} height tiles harvested",
        real.len()
    );
    drop(warm);

    // Phase 1 — the same pose on a fresh engine, with every height tile from z11 down
    // failing. Poisoned *before* the first mesh is built, because a mesh already built
    // from good data is never downgraded (that is the rule, not an accident).
    let mut state = pollster::block_on(new_state(config()));
    state.camera.set_eye_with_up(eye, target, up);
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(120);
    let mut quiet = 0;
    loop {
        poison(&mut state, &ids);
        pollster::block_on(state.tile_system.texture_manager.fetch_and_upload_all(
            &state.device,
            &state.queue,
            &[],
        ));
        state.update_logic(aspect, view_proj);
        let settled =
            state.last_missing_tiles_count == 0 && state.tile_system.is_loading_complete();
        quiet = if settled { quiet + 1 } else { 0 };
        if quiet >= 8 || std::time::Instant::now() >= deadline {
            break;
        }
        std::thread::sleep(std::time::Duration::from_millis(50));
    }
    let coarse = frame(&mut state);
    write_png(&coarse, &dir.join("e2_burst_0_coarse_ancestors.png"));

    // Phase 2 — every retry lands at once.
    if let Some(heights) = state.tile_system.height_manager.as_mut() {
        for (id, tile) in &real {
            heights.insert_ready(*id, Arc::clone(tile));
        }
    }

    println!("\n  | frame | rebuilds queued | pixels changed vs previous frame |");
    println!("  |--:|--:|--:|");
    let mut prev = coarse.clone();
    let mut worst_frame_delta = 0.0f64;
    let mut worst_rebuilds = 0usize;
    let mut total_rebuilds = 0usize;
    let mut frames = 0usize;
    let mut idle = 0usize;
    loop {
        let pixels = frame(&mut state);
        let queued = state.last_mesh_rebuilds;
        let delta = differing_fraction(&prev, &pixels);
        if queued > 0 || delta > 0.0 {
            println!("  | {frames} | {queued} | {:6.3} % |", delta * 100.0);
        }
        worst_frame_delta = worst_frame_delta.max(delta);
        worst_rebuilds = worst_rebuilds.max(queued);
        total_rebuilds += queued;
        prev = pixels;
        frames += 1;
        idle = if queued == 0 && delta == 0.0 {
            idle + 1
        } else {
            0
        };
        if idle >= 20 || frames > 2000 {
            break;
        }
        std::thread::sleep(std::time::Duration::from_millis(10));
    }
    write_png(&prev, &dir.join("e2_burst_1_rebuilt.png"));

    let total_change = differing_fraction(&coarse, &prev);
    println!(
        "\n  {total_rebuilds} rebuilds over {frames} frames, at most {worst_rebuilds} in any one \
         frame (budget {MESH_REBUILD_BUDGET_PER_FRAME})"
    );
    println!(
        "  coarse -> rebuilt changed {:.2} % of the frame in total; the worst single frame \
         carried {:.3} % of it",
        total_change * 100.0,
        worst_frame_delta * 100.0
    );
    println!(
        "  Read `e2_burst_0_coarse_ancestors.png` against `e2_burst_1_rebuilt.png`: the same \
         camera, the same imagery, a valley that was a smooth bowl and is now a valley."
    );

    assert!(
        worst_rebuilds <= MESH_REBUILD_BUDGET_PER_FRAME,
        "one frame queued {worst_rebuilds} rebuilds, budget is {MESH_REBUILD_BUDGET_PER_FRAME}"
    );
    assert!(
        total_rebuilds > 0,
        "the staged burst did not make a single mesh stale — the staging is broken, not the engine"
    );
    assert!(
        total_change > 0.02,
        "the rebuilt frame is barely different from the coarse one ({:.3} %); \
         the poison did not take",
        total_change * 100.0
    );
    // The whole point of the budget: no single frame may carry the bulk of the change.
    assert!(
        worst_frame_delta < total_change * 0.5,
        "one frame carried {:.3} % of a {:.3} % total change — that is a jolt, not a ripple",
        worst_frame_delta * 100.0,
        total_change * 100.0
    );
}
