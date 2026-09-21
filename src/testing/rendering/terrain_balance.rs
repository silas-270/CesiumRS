//! **The balance instrument** — what one frame costs with D3 on against the same frame
//! with D3 off, at the same pose, through the real renderer.
//!
//! `docs/terrain-plan.md` §7b–§7e report two numbers side by side and never add them up:
//! *tiles removed* and *march microseconds*. Neither answers the only question a culling
//! stage has to answer, which is whether the frame got cheaper. A tile that stops being
//! drawn stops costing a draw call, 289 vertices, 512 triangles, a texture bind, its
//! fragments, and — over the frames after it — a mesh build and a height fetch. A march
//! that runs costs what it costs whether or not anything is removed. This module puts
//! both on the same clock.
//!
//! # How the two arms are compared
//!
//! Two `WgpuState`s are built for the same pose, one with
//! `TerrainOcclusionConfig::enabled = false` and one with it `true`, both settled to
//! quiescence, and then **frames are taken alternately from the two** — A, B, A, B — for
//! as many rounds as `CESIUM_BALANCE_ROUNDS` asks for. Interleaving is the whole point:
//! this machine carries a few hundred other users and its load drifts on a timescale of
//! seconds, so two arms measured one after the other are two different machines. The
//! statistic reported is the **minimum** over rounds, which is the frame that got the
//! least interference, plus the median so that a minimum standing alone can be checked.
//!
//! # What "one frame" includes here
//!
//! `WgpuState::render` runs `update_logic` (the quadtree, the streaming, the labels) and
//! then encodes and submits the scene. Submission is asynchronous, so the harness follows
//! it with `device.poll(Maintain::Wait)`: without that the GPU half of the frame lands in
//! whichever later frame happens to block, and the arm that submits *more* work looks
//! cheaper.
//!
//! # The adapter is llvmpipe, and the report says so
//!
//! There is no GPU on this machine — `vulkaninfo` reports
//! `PHYSICAL_DEVICE_TYPE_CPU / llvmpipe`. Vertex and fragment work is therefore done on
//! the same cores as everything else, which makes a drawn tile **more** expensive than it
//! would be on the S23 §9 F3 targets, and makes the fragment half of a hidden tile
//! (which a real depth buffer discards early) count for more than it should. Every
//! frame-time number here is read with that in mind, and the per-tile cost model in
//! [`terrain_balance_cost_per_tile`] is the cross-check that does not depend on it.
//!
//! ```text
//! CESIUM_BALANCE_ROUNDS=40 \
//!   cargo test --release --lib rendering::terrain_balance -- --ignored --nocapture
//! ```

use cesium_engine::globe::tiles::config::{TerrainConfig, TileEngineConfig, SATELLITE_IMAGERY_URL};

use super::terrain_capture::{oblique, Pose};
use super::terrain_step_capture::bearing_pose;

/// One measured arm of one pose.
#[derive(Clone, Debug)]
struct Sample {
    /// Wall time of `render` + `poll(Wait)`, microseconds.
    frame_us: Vec<f64>,
    /// `FrameTimings::update_logic_us`.
    update_us: Vec<f64>,
    /// `SubsystemTimings::quadtree_us` — D1's bounds refresh, D3's march, and `update`.
    quadtree_us: Vec<f64>,
    /// `SubsystemTimings::terrain_draw_us` — the tile draw pass's own encode time.
    draw_us: Vec<f64>,
    /// `SubsystemTimings::terrain_horizon_us` — the march alone, once per frame.
    march_us: Vec<f64>,
    /// Tiles handed to `render_scene`.
    tiles: usize,
}

fn min_of(v: &[f64]) -> f64 {
    v.iter().copied().fold(f64::INFINITY, f64::min)
}

fn median_of(v: &[f64]) -> f64 {
    let mut s = v.to_vec();
    s.sort_by(|a, b| a.partial_cmp(b).unwrap());
    if s.is_empty() {
        return f64::NAN;
    }
    s[s.len() / 2]
}

fn config(occlusion: bool, texel_ratio: f32) -> TileEngineConfig {
    TileEngineConfig {
        base_imagery_url: SATELLITE_IMAGERY_URL.to_string(),
        offline_mode: false,
        transparent_background: true,
        target_texel_ratio: texel_ratio,
        mesh_segments: 16,
        terrain: TerrainConfig {
            enabled: true,
            exaggeration: 1.0,
            occlusion: cesium_engine::globe::quadtree::TerrainOcclusionConfig {
                enabled: occlusion,
                // Swept rather than hard-coded: `max_range_m` is how far the *occluder*
                // walk looks, and the walk is where the march's time goes.
                max_range_m: std::env::var("CESIUM_BALANCE_RANGE")
                    .ok()
                    .and_then(|s| s.parse().ok())
                    .unwrap_or(120_000.0),
                ..Default::default()
            },
            ..TerrainConfig::default()
        },
        ..TileEngineConfig::default()
    }
}

/// The pose families §7's verdict has to hold over.
///
/// Two terrain steps (§7d's own poses, the shape D3 was specified for), one Alpine near
/// field, one approach, one cockpit-height traverse and one cruise. The two step poses
/// are built with [`bearing_pose`] because their headings are not due north and
/// `ViewParams::yaw_deg` is not a compass bearing — §7d's first pose trap.
fn poses() -> Vec<Pose> {
    vec![
        bearing_pose(
            "reutlingen_albtrauf",
            9.2043,
            48.4914,
            377.7,
            135.0,
            1.0,
            0.06,
            "terrain step: the Albtrauf at 8 km, 40 km of plateau behind it",
        ),
        bearing_pose(
            "stuttgart_kessel",
            9.1829,
            48.7758,
            252.0,
            180.0,
            1.0,
            0.06,
            "terrain step: the basin rim at 2.5 km, the Filder plateau behind it",
        ),
        oblique(
            "alps_inn_valley",
            11.40,
            47.26,
            900.0,
            1.5,
            0.20,
            "alpine near field: the Nordkette wall from the Inn valley at 900 m",
        ),
        oblique(
            "alps_approach",
            10.985,
            47.10,
            3_000.0,
            8.0,
            0.12,
            "approach: the main ridge from 3 km, 35 km out",
        ),
        oblique(
            "alps_cockpit",
            11.20,
            47.05,
            2_000.0,
            3.0,
            0.20,
            "cockpit: 2 km over the Zillertal, shallow pitch along the valley axis",
        ),
        oblique(
            "alps_cruise_11km",
            10.985,
            46.60,
            11_000.0,
            10.0,
            0.40,
            "cruise: the Alps from 11 km, just under the altitude gate",
        ),
    ]
}

/// Builds a state at `pose` and runs it until nothing is in flight.
///
/// The same three-condition settle `terrain_capture::render_settled` uses, and for its
/// reason: `run_headless_render`'s missing-tile counter does not see height fetches
/// behind a deferred mesh, so a frame captured on it has no near-field geometry.
async fn settled(
    width: u32,
    height: u32,
    cfg: TileEngineConfig,
    pose: &Pose,
) -> cesium_engine::render::wgpu_state::WgpuState<'static> {
    use cesium_engine::render::wgpu_state::WgpuState;

    let mut state = WgpuState::new(
        None,
        Some(winit::dpi::PhysicalSize::new(width, height)),
        cfg,
        None,
    )
    .await;

    match pose.up {
        Some(up) => state.camera.set_eye_with_up(pose.eye, pose.target, up),
        None => state.camera.set_eye(pose.eye, pose.target),
    }

    let aspect = state.size.width as f32 / state.size.height as f32;
    let view_proj = state.camera.get_projection_matrix(aspect) * state.camera.get_view_matrix();

    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(150);
    let mut quiet = 0;
    loop {
        state
            .tile_system
            .texture_manager
            .fetch_and_upload_all(&state.device, &state.queue, &[])
            .await;
        state.update_logic(aspect, view_proj);
        let done = state.last_missing_tiles_count == 0 && state.tile_system.is_loading_complete();
        quiet = if done { quiet + 1 } else { 0 };
        if quiet >= 8 || std::time::Instant::now() >= deadline {
            break;
        }
        std::thread::sleep(std::time::Duration::from_millis(50));
    }
    state
}

/// One timed frame: encode, submit, and **wait for the queue to drain**.
fn timed_frame(state: &mut cesium_engine::render::wgpu_state::WgpuState<'_>) -> f64 {
    let t0 = std::time::Instant::now();
    #[cfg(feature = "debug_panel")]
    let res = state.render(None, false, |_, _| {});
    #[cfg(not(feature = "debug_panel"))]
    let res = state.render(None, false);
    res.expect("headless render");
    state.device.poll(wgpu::Maintain::Wait);
    t0.elapsed().as_secs_f64() * 1.0e6
}

fn rounds() -> usize {
    std::env::var("CESIUM_BALANCE_ROUNDS")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(24)
}

/// Alternating A/B frames off two settled states, `n` rounds of each.
fn interleave<'a>(
    a: &mut cesium_engine::render::wgpu_state::WgpuState<'a>,
    b: &mut cesium_engine::render::wgpu_state::WgpuState<'a>,
    n: usize,
) -> (Sample, Sample) {
    let mut sa = Sample {
        frame_us: Vec::new(),
        update_us: Vec::new(),
        quadtree_us: Vec::new(),
        draw_us: Vec::new(),
        march_us: Vec::new(),
        tiles: 0,
    };
    let mut sb = sa.clone();
    // Two warm-up frames each, outside the statistics: the first `render` after a settle
    // loop pays for pipeline caches and the first depth-buffer touch.
    for _ in 0..2 {
        timed_frame(a);
        timed_frame(b);
    }
    for _ in 0..n {
        for (state, s) in [(&mut *a, &mut sa), (&mut *b, &mut sb)] {
            let us = timed_frame(state);
            s.frame_us.push(us);
            s.update_us.push(state.last_timings.update_logic_us);
            s.quadtree_us.push(state.last_subsystem_timings.quadtree_us);
            s.draw_us.push(state.last_subsystem_timings.terrain_draw_us);
            s.march_us
                .push(state.last_subsystem_timings.terrain_horizon_us);
        }
    }
    let aspect = a.size.width as f32 / a.size.height as f32;
    for (state, s) in [(&mut *a, &mut sa), (&mut *b, &mut sb)] {
        let vp = state.camera.get_projection_matrix(aspect) * state.camera.get_view_matrix();
        s.tiles = state.update_logic(aspect, vp).len();
    }
    (sa, sb)
}

/// **The balance table.** D3 on against D3 off, same pose, same clock.
#[test]
#[ignore = "measurement: needs the network for imagery and heights, takes minutes"]
fn terrain_balance_d3_on_vs_off() {
    let n = rounds();
    println!("  balance: {n} interleaved rounds per arm, 1280x720, adapter is llvmpipe (CPU)");
    println!(
        "  {:<22} {:>6} {:>6} {:>10} {:>10} {:>9} {:>9} {:>9}",
        "pose", "off", "on", "frame off", "frame on", "delta", "quadtree", "draw"
    );
    let mut rows: Vec<(String, usize, usize, f64, f64, f64, f64)> = Vec::new();
    // One pose per process. Each `WgpuState` stands up a tokio runtime, a rayon pool and
    // llvmpipe's own workers — some 400 threads on a 128-core host — and dropping it does
    // not join them fast enough to keep six poses × two arms under `ulimit -u`. The loop
    // still runs all six when the variable is unset, which is what a single-pose machine
    // wants.
    let only = std::env::var("CESIUM_BALANCE_POSE").ok();
    for p in poses()
        .into_iter()
        .filter(|p| only.as_deref().map(|o| o == p.name).unwrap_or(true))
    {
        let mut off = pollster::block_on(settled(1280, 720, config(false, 1.0), &p));
        let mut on = pollster::block_on(settled(1280, 720, config(true, 1.0), &p));
        let (s_off, s_on) = interleave(&mut off, &mut on, n);

        let f_off = min_of(&s_off.frame_us);
        let f_on = min_of(&s_on.frame_us);
        println!(
            "  {:<22} {:>6} {:>6} {:>9.0}u {:>9.0}u {:>+8.0}u {:>8.0}u {:>8.0}u",
            p.name,
            s_off.tiles,
            s_on.tiles,
            f_off,
            f_on,
            f_on - f_off,
            min_of(&s_on.quadtree_us) - min_of(&s_off.quadtree_us),
            min_of(&s_on.draw_us) - min_of(&s_off.draw_us),
        );
        println!(
            "    {:<20} median off {:.0} us, on {:.0} us | update off {:.0} on {:.0} | \
             quadtree off {:.0} on {:.0} | draw off {:.0} on {:.0}",
            "",
            median_of(&s_off.frame_us),
            median_of(&s_on.frame_us),
            min_of(&s_off.update_us),
            min_of(&s_on.update_us),
            min_of(&s_off.quadtree_us),
            min_of(&s_on.quadtree_us),
            min_of(&s_off.draw_us),
            min_of(&s_on.draw_us),
        );
        println!(
            "    {:<20} march alone {:.0} us, stage inside update {:.0} us",
            "",
            min_of(&s_on.march_us),
            (min_of(&s_on.quadtree_us) - min_of(&s_off.quadtree_us)) - min_of(&s_on.march_us),
        );
        rows.push((
            p.name.to_string(),
            s_off.tiles,
            s_on.tiles,
            f_off,
            f_on,
            min_of(&s_off.quadtree_us),
            min_of(&s_on.quadtree_us),
        ));
    }
    println!(
        "\n  csv: pose,tiles_off,tiles_on,frame_off_us,frame_on_us,quadtree_off_us,quadtree_on_us"
    );
    for (name, to, tn, fo, fnn, qo, qn) in &rows {
        println!("  csv: {name},{to},{tn},{fo:.1},{fnn:.1},{qo:.1},{qn:.1}");
    }
}

/// **The cost model.** What one drawn tile is worth, measured rather than assumed.
///
/// The balance table's delta mixes two things: the march's cost and the removed tiles'
/// saving. This separates the second by sweeping `target_texel_ratio`, which changes how
/// many tiles the imagery LOD asks for **at a fixed camera** — so the pose, the frustum
/// and the fragment coverage are identical and only the tile count moves. The slope of
/// frame time against tile count is the marginal cost of a tile, and multiplying it by
/// the tiles D3 removes is the saving the balance table's noise has to be judged against.
///
/// Every sweep point is measured **interleaved against the same reference arm**
/// (`target_texel_ratio = 1.0`, which is what the balance table renders at), for the
/// reason `interleave` exists: three points taken one after the other on this machine are
/// three different machines. Two states at a time, because six of them do not fit under
/// `ulimit -u`.
#[test]
#[ignore = "measurement: needs the network for imagery and heights, takes minutes"]
fn terrain_balance_cost_per_tile() {
    let n = rounds();
    let only = std::env::var("CESIUM_BALANCE_POSE").ok();
    for p in poses()
        .into_iter()
        .filter(|p| only.as_deref().map(|o| o == p.name).unwrap_or(true))
    {
        println!("  {} — {}", p.name, p.what);
        let mut pts: Vec<(f64, f64)> = Vec::new();
        for r in [0.6f32, 1.4, 2.0] {
            // D3 off on both sides of the sweep: the question here is what a tile costs,
            // not what removing one buys.
            let mut a = pollster::block_on(settled(1280, 720, config(false, r), &p));
            let mut b = pollster::block_on(settled(1280, 720, config(false, 1.0), &p));
            let (sa, sb) = interleave(&mut a, &mut b, n);
            let (fa, fb) = (min_of(&sa.frame_us), min_of(&sb.frame_us));
            println!(
                "    texel_ratio {r:>4.1}: {:>4} tiles {:>8.0} us   vs  ref 1.0: {:>4} tiles \
                 {:>8.0} us   => {:>7.1} us/tile   (draw {:>6.0} vs {:>6.0}, update {:>6.0} vs {:>6.0})",
                sa.tiles,
                fa,
                sb.tiles,
                fb,
                if sa.tiles == sb.tiles {
                    f64::NAN
                } else {
                    (fa - fb) / (sa.tiles as f64 - sb.tiles as f64)
                },
                min_of(&sa.draw_us),
                min_of(&sb.draw_us),
                min_of(&sa.update_us),
                min_of(&sb.update_us),
            );
            pts.push((sa.tiles as f64, fa));
            pts.push((sb.tiles as f64, fb));
        }
        // Ordinary least squares through every point taken at this pose.
        let k = pts.len() as f64;
        let sx: f64 = pts.iter().map(|p| p.0).sum();
        let sy: f64 = pts.iter().map(|p| p.1).sum();
        let sxx: f64 = pts.iter().map(|p| p.0 * p.0).sum();
        let sxy: f64 = pts.iter().map(|p| p.0 * p.1).sum();
        let slope = (k * sxy - sx * sy) / (k * sxx - sx * sx);
        println!(
            "    => OLS over {} points: {slope:.1} us per drawn tile",
            pts.len()
        );
    }
}

/// **The altitude ladder, on real terrain and through the renderer.**
///
/// §7b's `d3_altitude_gate_is_where_the_benefit_stops` walks a synthetic ridge world up in
/// altitude and reads the reduction off the tile count; `max_camera_altitude_m = 12 000`
/// is the first altitude where that reads zero. Under the balance rule that is the wrong
/// place to cut: the question is not where the benefit reaches zero but where it stops
/// **paying for the march**, and the march costs 250–550 µs whether or not it removes
/// anything.
///
/// This is the same ladder on the real DEM, at one horizontal pose, through the engine
/// the phone runs — tiles off, tiles on, and the march's own cost, at every rung. The
/// altitude reported alongside each rung is **above ground**, because that is what D3's
/// geometry actually depends on and what the gate is now written in.
#[test]
#[ignore = "measurement: needs the network for imagery and heights, takes minutes"]
fn terrain_balance_altitude_ladder() {
    let n = rounds();
    let rungs: Vec<f64> = std::env::var("CESIUM_BALANCE_RUNGS")
        .ok()
        .map(|s| s.split(',').filter_map(|t| t.trim().parse().ok()).collect())
        .unwrap_or_else(|| vec![900.0, 1_300.0, 1_800.0, 2_500.0, 3_500.0, 5_000.0]);
    println!("  altitude ladder over the Inn valley, {n} interleaved rounds per rung");
    println!(
        "  {:>9} {:>9} {:>6} {:>6} {:>8} {:>11} {:>11} {:>10}",
        "alt (m)", "agl (m)", "off", "on", "removed", "frame off", "frame on", "march"
    );
    for alt in rungs {
        let p = oblique(
            "alps_inn_valley_ladder",
            11.40,
            47.26,
            alt,
            1.5,
            0.20,
            "the Nordkette wall from the Inn valley, walked up in altitude",
        );
        let mut off = pollster::block_on(settled(1280, 720, config(false, 1.0), &p));
        let mut on = pollster::block_on(settled(1280, 720, config(true, 1.0), &p));
        let agl = on.camera.altitude_agl() as f64 * 1.0e6;
        let (s_off, s_on) = interleave(&mut off, &mut on, n);
        println!(
            "  {alt:>9.0} {agl:>9.0} {:>6} {:>6} {:>8} {:>10.0}u {:>10.0}u {:>9.0}u",
            s_off.tiles,
            s_on.tiles,
            s_off.tiles as i64 - s_on.tiles as i64,
            min_of(&s_off.frame_us),
            min_of(&s_on.frame_us),
            min_of(&s_on.quadtree_us) - min_of(&s_off.quadtree_us),
        );
        println!(
            "            march alone {:.0} us, stage inside update {:.0} us",
            min_of(&s_on.march_us),
            (min_of(&s_on.quadtree_us) - min_of(&s_off.quadtree_us)) - min_of(&s_on.march_us),
        );
    }
}
