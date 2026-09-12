//! # Layer 2 — globe sweep against the independent f64 oracle.
//!
//! Every test here is the same shape: generate parameter cells
//! ([`super::cells`]), measure each one ([`super::sweep::measure_cell`]), write a
//! CSV and a failure map ([`super::report`]), then assert against a named,
//! documented threshold.
//!
//! ## Which tests must pass, and which are the to-do list
//!
//! The engine has real, pre-existing culling defects. Rather than weaken
//! thresholds until everything is green, the tests are split:
//!
//! * **Regression guards** (no attribute) — regimes that are clean today. They
//!   fail only if something gets *worse*.
//! * **`#[ignore]`d defect probes** — each names the specific defect it exposes
//!   and carries the real measured number in its comment. Run them with
//!   `cargo test --release --lib culling:: -- --ignored --test-threads=1`.
//!   They are the fix-phase to-do list.
//!
//! No threshold here has been relaxed to make a test green.
//!
//! ## The three defects these probes pin down
//!
//! 1. **Horizon-culling conservatism** — tiles are dropped up to **8.07°** inside
//!    the visible limb, widening with altitude.
//!    [`test_limb_band_has_no_false_negatives`].
//! 2. **Tracking mode at ~5 m altitude returns 0-1 tiles** for a viewport
//!    completely filled with ground.
//!    [`test_near_ground_high_zoom_has_no_false_negatives`].
//! 3. **The far plane is never enforced** — the depth-plane extraction in
//!    `calculate_frustum_planes` assumes GL's z ∈ [-1, 1] under a reverse-Z
//!    z ∈ [0, 1] projection, leaving plane index 4 degenerate.
//!    [`super::test_analytic_planes::test_far_plane_is_enforced`].
//!
//! See [`super`] for threading and build-profile guidance — `--release` is ~8x
//! faster for this workload and the probes are sized for a many-core machine.

use cesium_engine::camera::camera::CameraMode;
use glam::DVec3;

use super::cameras::{build_camera, ViewParams};
use super::cells;
use super::oracle::VisibilityOracle;
use super::report;
use super::sweep::{self, measure_cell, CellResult, UPDATE_ITERATIONS};

/// Measure a sweep and emit its artefacts.
///
/// All parallelism goes through [`sweep::measure_cells`], which runs inside the
/// harness's own bounded rayon pool — see [`sweep::harness_pool`] for why the
/// global pool is deliberately not used.
fn run(name: &str, cells: Vec<ViewParams>) -> (Vec<CellResult>, String) {
    let t0 = std::time::Instant::now();
    let results: Vec<CellResult> = sweep::measure_cells(&cells);
    let elapsed = t0.elapsed();
    let text = report::emit(name, &results);
    println!(
        "  [{name}] {} cells in {:.2?} ({} rayon threads)",
        results.len(),
        elapsed,
        sweep::harness_pool().current_num_threads()
    );
    (results, text)
}

// ─────────────────────────────────────────────────────────────────────────────
// Harness self-checks. These must pass or none of the numbers below mean anything.
// ─────────────────────────────────────────────────────────────────────────────

/// `QuadtreeNode::update` recurses into children it has just created, so the tree
/// reaches full depth in a single call; extra calls only settle the LOD hysteresis
/// band (`collapse_dist = subdivide_dist * 1.20`).
///
/// This pins [`UPDATE_ITERATIONS`] to a number that is actually sufficient: the
/// visible set after `UPDATE_ITERATIONS` updates must equal the set after four
/// times as many. Without this, every FN number in this file would be suspect.
#[test]
fn test_update_iterations_reach_fixed_point() {
    use cesium_engine::globe::quadtree::{QuadtreeManager, TileId};
    use std::collections::HashSet;

    let mut probes = cells::nadir_ladder();
    probes.extend(cells::zoom_cliff_cells());
    probes.truncate(24);

    for p in probes {
        let cam = build_camera(&p);
        let planes = cam.calculate_frustum_planes(p.aspect() as f32);
        let (gp, _) = cam.global_transform_f64();
        let frustum = cesium_engine::globe::quadtree::Frustum::new(planes, gp);

        let collect = |iters: usize| -> HashSet<TileId> {
            let mut qt = QuadtreeManager::new();
            for _ in 0..iters {
                qt.update(&frustum);
            }
            qt.get_visible_tiles().into_iter().map(|(id, _, _)| id).collect()
        };

        let settled = collect(UPDATE_ITERATIONS);
        let over_settled = collect(UPDATE_ITERATIONS * 4);
        assert_eq!(
            settled.len(),
            over_settled.len(),
            "visible set is still changing after {UPDATE_ITERATIONS} updates at \
             lat={} alt={}m pitch={}: {} tiles vs {} after {} updates",
            p.lat_deg,
            p.alt_m,
            p.pitch_deg,
            settled.len(),
            over_settled.len(),
            UPDATE_ITERATIONS * 4
        );
        assert!(
            settled == over_settled,
            "visible set membership changed after {UPDATE_ITERATIONS} updates at \
             lat={} alt={}m pitch={}",
            p.lat_deg,
            p.alt_m,
            p.pitch_deg
        );
    }
}

/// The oracle's two halves must agree with each other: every surface point found
/// by unprojecting a viewport position and intersecting the ellipsoid must, when
/// re-projected, land back at that viewport position.
///
/// This is a round-trip check on the ground truth itself.
#[test]
fn test_oracle_unprojection_round_trips() {
    let mut worst = 0.0_f64;
    let mut worst_ctx = String::new();

    for p in cells::nadir_ladder().into_iter().take(12) {
        let cam = build_camera(&p);
        let oracle = VisibilityOracle::new(&cam, p.aspect());

        for i in 0..17 {
            let nx = -1.0 + 2.0 * (i as f64 + 0.5) / 17.0;
            for j in 0..11 {
                let ny = -1.0 + 2.0 * (j as f64 + 0.5) / 11.0;
                let Some(hit) = oracle.surface_point_at_ndc(nx, ny) else {
                    continue;
                };
                let Some(ndc) = oracle.ndc(hit) else { continue };
                let err = (ndc.x - nx).abs().max((ndc.y - ny).abs());
                if err > worst {
                    worst = err;
                    worst_ctx =
                        format!("alt={}m ndc=({nx:.3},{ny:.3}) -> ({:.6},{:.6})", p.alt_m, ndc.x, ndc.y);
                }
            }
        }
    }

    println!("  oracle unprojection round-trip worst NDC error: {worst:.3e}  [{worst_ctx}]");
    // 1e-9 of half-screen is ~1e-6 px on a 1920-wide viewport; anything above
    // that means the inverse view-projection is not the inverse it claims to be.
    assert!(
        worst < 1.0e-9,
        "oracle unprojection does not round-trip: worst NDC error {worst:.3e} [{worst_ctx}]"
    );
}

/// Documents *why* `Camera::screen_to_world_ray` is not used as ground truth.
///
/// It hardcodes `FRAC_PI_4` (45°) for the vertical FOV, while the real projection
/// uses `2·atan(24/(2·focal_length))` ≈ 46.4° in Free/Tracking and a fixed 60° in
/// Cockpit. The rays therefore agree at the screen centre and diverge toward the
/// edges. This test measures the divergence instead of asserting it away.
///
/// The assertion is deliberately inverted: it fails if the divergence *vanishes*,
/// because that would mean the production bug was fixed and this harness's careful
/// avoidance of the function is no longer needed.
#[test]
fn test_screen_to_world_ray_is_not_ground_truth() {
    let p = ViewParams {
        sweep: "ray-bug",
        alt_m: 2_000_000.0,
        mode: CameraMode::Free,
        ..Default::default()
    };
    let cam = build_camera(&p);
    let oracle = VisibilityOracle::new(&cam, p.aspect());

    let w = p.width as f32;
    let h = p.height as f32;

    let mut centre_deg = 0.0_f64;
    let mut edge_deg = 0.0_f64;

    for (label, sx, sy) in [
        ("centre", w * 0.5, h * 0.5),
        ("top edge", w * 0.5, 0.5),
        ("left edge", 0.5, h * 0.5),
        ("corner", 0.5, 0.5),
    ] {
        let (_, ray_dir) = cam.screen_to_world_ray(sx, sy, w, h);
        let ndc_x = (2.0 * sx / w - 1.0) as f64;
        let ndc_y = (1.0 - 2.0 * sy / h) as f64;
        let (_, truth_dir) = oracle.ray_through_ndc(ndc_x, ndc_y);

        let d = glam::DVec3::new(ray_dir.x as f64, ray_dir.y as f64, ray_dir.z as f64).normalize();
        let deg = d.dot(truth_dir).clamp(-1.0, 1.0).acos().to_degrees();
        println!("  {label:<10} screen_to_world_ray deviates {deg:.4} deg from the true ray");

        if label == "centre" {
            centre_deg = deg;
        } else {
            edge_deg = edge_deg.max(deg);
        }
    }

    assert!(
        centre_deg < 1.0e-4,
        "screen_to_world_ray should still be correct at the screen centre, off by {centre_deg} deg"
    );
    assert!(
        edge_deg > 0.1,
        "screen_to_world_ray now agrees with the true projection at the screen edges \
         (max deviation {edge_deg:.6} deg). The hardcoded FRAC_PI_4 in camera.rs:542 \
         appears to have been fixed — raycast-based oracles are viable again and this \
         test should be replaced."
    );
}

// ─────────────────────────────────────────────────────────────────────────────
// Regression guards — clean today, must stay clean.
// ─────────────────────────────────────────────────────────────────────────────

/// Nadir views at a range of altitudes and latitudes: the tamest geometry the
/// engine ever sees, and the case a user spends most of their time in.
///
/// Threshold: **zero** false negatives. This regime is measured clean, so any FN
/// at all is a regression. The FP threshold is a rate, not zero, because bounding
/// volumes are conservative by construction — a tile whose OBB pokes over the
/// horizon or past a screen edge is legitimately scheduled.
#[test]
fn test_nadir_ladder_has_no_false_negatives() {
    /// Measured: **0 FN** over 3 568 512 visible samples in 72 cells.
    const MAX_FN: usize = 0;
    /// Measured worst-cell FP rate: **0.2727** (3 tiles of 11, at 20 000 km where
    /// the visible set is a handful of z=1/z=2 tiles whose OBBs bulge well past
    /// the limb). 0.45 is ~1.6x headroom — enough for legitimate bounding-volume
    /// conservatism, still far below the 1.00 a deliberately broken frustum plane
    /// produced.
    const MAX_FP_RATE: f64 = 0.45;

    let (results, text) = run("nadir_ladder", cells::nadir_ladder());
    let s = report::summarize(&results);

    assert!(
        s.total_false_negatives <= MAX_FN,
        "nadir ladder produced {} false negatives (limit {MAX_FN}).{text}",
        s.total_false_negatives
    );
    assert!(
        s.worst_fp_rate <= MAX_FP_RATE,
        "nadir ladder worst-cell false-positive rate {:.4} exceeds {MAX_FP_RATE}.{text}",
        s.worst_fp_rate
    );
}

/// Aspect-ratio extremes at a fixed, tame viewpoint. Ultra-wide and ultra-tall
/// viewports are where a frustum-plane sign or an aspect mix-up shows up first.
///
/// Threshold: **zero** false negatives.
#[test]
fn test_aspect_extremes_have_no_false_negatives() {
    const MAX_FN: usize = 0;

    let base = cells::baseline();
    let mut probes = Vec::new();
    for (w, h) in cells::aspects() {
        for pitch in [0.0, 30.0, 45.0, 70.0, 88.0] {
            probes.push(ViewParams {
                sweep: "aspect-extreme",
                width: w,
                height: h,
                pitch_deg: pitch,
                ..base.clone()
            });
        }
    }

    let (results, text) = run("aspect_extremes", probes);
    let s = report::summarize(&results);
    assert!(
        s.total_false_negatives <= MAX_FN,
        "aspect extremes produced {} false negatives (limit {MAX_FN}).{text}",
        s.total_false_negatives
    );
}

/// All three camera modes at a tame viewpoint. The modes differ only in znear and
/// fovy, both of which feed the frustum planes, so a mode-specific plane bug would
/// show here.
///
/// Threshold: **zero** false negatives.
#[test]
fn test_all_camera_modes_have_no_false_negatives() {
    const MAX_FN: usize = 0;

    let base = cells::baseline();
    let mut probes = Vec::new();
    for mode in cells::modes() {
        for alt in [500.0, 5_000.0, 50_000.0, 1_000_000.0, 10_000_000.0, 25_000_000.0] {
            probes.push(ViewParams {
                sweep: "mode",
                mode,
                alt_m: alt,
                ..base.clone()
            });
        }
    }

    let (results, text) = run("camera_modes", probes);
    let s = report::summarize(&results);
    assert!(
        s.total_false_negatives <= MAX_FN,
        "camera-mode sweep produced {} false negatives (limit {MAX_FN}).{text}",
        s.total_false_negatives
    );
}

/// The clamped camera path. `Camera::set_eye` does not clamp, but
/// `set_local_transform` does, via `enforce_bounds`, which pushes the camera back
/// out to `ellipsoid_radius + 2 mm` along its own direction.
///
/// The rest of the sweep deliberately bypasses that clamp (see
/// [`super::cameras::build_camera`]) so that sub-surface cameras are actually
/// tested. This test covers the other branch: it asserts the clamp really does
/// move a sub-surface camera, and that culling from the clamped position is still
/// hole-free.
///
/// Threshold: **zero** false negatives — measured **0**.
#[test]
fn test_clamped_camera_path_has_no_false_negatives() {
    use super::cameras::build_camera_clamped;

    const MAX_FN: usize = 0;

    /// Distance from the ellipsoid centre to its surface along `dir`.
    fn surface_radius(dir: DVec3) -> f64 {
        let d = dir.normalize();
        1.0 / DVec3::new(
            d.x / super::geodesy::A,
            d.y / super::geodesy::B,
            d.z / super::geodesy::A,
        )
        .length()
    }

    let base = cells::baseline();
    let mut failures = Vec::new();
    let mut total_fn = 0usize;
    let mut total_visible = 0usize;

    for alt in [-100_000.0, -1_000.0, -50.0, 0.0, 1_000.0, 50_000.0] {
        for lat in [-89.0, -45.0, 0.0, 48.0, 89.0] {
            let p = ViewParams {
                sweep: "clamped",
                lat_deg: lat,
                alt_m: alt,
                pitch_deg: 60.0,
                ..base.clone()
            };

            let clamped = build_camera_clamped(&p);
            let free = build_camera(&p);
            let (clamped_pos, _) = clamped.global_transform_f64();
            let (free_pos, _) = free.global_transform_f64();

            if alt < 0.0 {
                assert!(
                    clamped_pos.length() > free_pos.length(),
                    "enforce_bounds did not lift a camera {alt} m below the surface \
                     at lat {lat}: {} vs {}",
                    clamped_pos.length(),
                    free_pos.length()
                );
                // The clamp targets the ellipsoid radius along the camera's own
                // direction, plus 2 mm.
                let lift_m = (clamped_pos.length() - surface_radius(clamped_pos)) * 1.0e6;
                println!(
                    "  alt={alt:>10.0}m lat={lat:>5.1} -> clamped to {lift_m:.4} m above the surface"
                );
            }

            // Measure culling from the clamped position by re-expressing it as a
            // cell whose altitude is the post-clamp altitude.
            let (lat_c, lon_c) = super::geodesy::dvec3_to_lat_lon(clamped_pos);
            let alt_c_m = (clamped_pos.length() - surface_radius(clamped_pos)) * 1.0e6;

            let measured = measure_cell(&ViewParams {
                sweep: "clamped",
                lat_deg: lat_c,
                lon_deg: lon_c,
                alt_m: alt_c_m,
                ..p.clone()
            });
            total_fn += measured.false_negatives;
            total_visible += measured.samples_visible;
            if measured.false_negatives > 0 {
                failures.push(format!(
                    "alt={alt} lat={lat}: {} false negatives",
                    measured.false_negatives
                ));
            }
        }
    }

    println!("  clamped path: {total_fn} FN over {total_visible} visible samples");
    assert!(
        total_fn <= MAX_FN,
        "clamped camera path produced {total_fn} false negatives (limit {MAX_FN}):\n  {}",
        failures.join("\n  ")
    );
}

/// False positives are acceptable but not unbounded. This is the waste budget.
///
/// Threshold is a *whole-sweep* rate so a handful of pathological cells cannot
/// blow it, but a systematic regression (e.g. the tight sub-OBB rejection being
/// disabled) would.
#[test]
fn test_false_positive_rate_within_budget() {
    /// Whole-sweep FP budget. Measured: **0.0047** (1 wasted tile of 213).
    /// 0.10 is ~20x headroom, and still well under the 0.24 a deliberately
    /// broken frustum plane produced.
    const MAX_OVERALL_FP_RATE: f64 = 0.10;

    let (results, text) = run("fp_budget", cells::nadir_ladder());
    let s = report::summarize(&results);
    let rate = if s.total_tiles == 0 {
        0.0
    } else {
        s.total_false_positive_tiles as f64 / s.total_tiles as f64
    };
    println!("  overall FP rate: {rate:.4} over {} tiles", s.total_tiles);
    assert!(
        rate <= MAX_OVERALL_FP_RATE,
        "overall false-positive rate {rate:.4} exceeds {MAX_OVERALL_FP_RATE}.{text}"
    );
}

/// The full one-factor-at-a-time sweep across latitude (poles and both Mercator
/// limits), longitude (antimeridian), altitude (−50 m to 30 Mm), pitch (nadir
/// through horizon to above-horizon), yaw, roll, aspect and all three camera
/// modes.
///
/// 153 cells.
///
/// Threshold: **zero** false negatives — measured **0** over 4 830 316 visible
/// samples. The per-cell CSV lands in
/// `$TMPDIR/cesium_culling_harness/axis_sweep.csv`.
#[test]
fn test_axis_sweep_has_no_false_negatives() {
    const MAX_FN: usize = 0;
    /// Measured worst-cell FP rate: **0.2727**. 0.45 is ~1.6x headroom.
    const MAX_FP_RATE: f64 = 0.45;

    let (results, text) = run("axis_sweep", cells::axis_sweep());
    let s = report::summarize(&results);
    assert!(
        s.total_false_negatives <= MAX_FN,
        "axis sweep produced {} false negatives across {} cells (limit {MAX_FN}).{text}",
        s.total_false_negatives,
        s.cells
    );
    assert!(
        s.worst_fp_rate <= MAX_FP_RATE,
        "axis sweep worst-cell false-positive rate {:.4} exceeds {MAX_FP_RATE}.{text}",
        s.worst_fp_rate
    );
}

/// Pitch sweep from nadir to well above the horizon, across altitudes,
/// latitudes and the ±0.1° neighbourhood of the exact horizon. 1 344 cells.
///
/// **Ignored: this fails today.** Measured: **11 false negatives of 27 556 756
/// visible samples, in 2 of 1 344 cells** — both at 12 000 km altitude with the
/// camera tilted 15–25° off nadir, deepest zoom 1–2. Every miss sits between
/// 0.49° and 1.48° inside the limb *and* within 1.4% of the bottom edge of the
/// viewport (`ndc.y ≈ −0.99`), i.e. coarse z=1/z=2 tiles at the intersection of
/// the limb and the screen edge. Same defect family as
/// [`test_limb_band_has_no_false_negatives`].
///
/// This sweep also produced the worst false-positive cells in the harness
/// (individual cells at 100%): above-horizon views where the frustum still
/// intersects a tile's bounding volume but none of the tile's surface is visible.
#[test]
#[ignore = "defect probe: at 12 000 km, tiles within ~1.5 deg of the limb and at the bottom screen edge are culled while still visible"]
fn test_horizon_pitch_sweep_has_no_false_negatives() {
    const MAX_FN: usize = 0;

    let base = cells::baseline();
    let mut probes = Vec::new();
    for alt in [100.0, 2_000.0, 10_000.0, 100_000.0, 400_000.0, 2_000_000.0, 12_000_000.0] {
        for pitch in cells::pitches() {
            for lat in [-80.0, -30.0, 0.0, 30.0, 48.0, 80.0] {
                probes.push(ViewParams {
                    sweep: "horizon-pitch",
                    lat_deg: lat,
                    alt_m: alt,
                    pitch_deg: pitch,
                    ..base.clone()
                });
            }
        }
    }

    let (results, text) = run("horizon_pitch", probes);
    let s = report::summarize(&results);
    assert!(
        s.total_false_negatives <= MAX_FN,
        "horizon pitch sweep produced {} false negatives (limit {MAX_FN}).{text}",
        s.total_false_negatives
    );
}

/// Explicit probes either side of the **z=16 `tight_obbs` cliff**.
///
/// `compute_bounding_volume` builds the 8×8 grid of tight sub-OBBs only for
/// `id.z <= 16`. From z=17 up, a node is culled by its single loose OBB alone and
/// the per-sub-OBB back-face rejection disappears entirely, so the conservatism
/// changes character exactly at that boundary.
///
/// The altitude ladder below walks the deepest reached zoom from 11 up to 20 over
/// 132 cells, so it straddles the cliff in both directions. Per-cell zoom range
/// and FP rate are printed so the discontinuity stays visible in the log.
///
/// Threshold: **zero** false negatives — measured **0** over 4 033 679 visible
/// samples, and the FP rate shows **no jump at the z=16/17 boundary**: the tiles
/// that get flagged are the handful of near-nadir low-altitude cells with only
/// 3–15 tiles in view, not the z>16 ones.
#[test]
fn test_zoom_cliff_probe() {
    const MAX_FN: usize = 0;
    /// Measured worst-cell FP rate across the ladder: **0.333** (1 tile of 3 at
    /// 10 m altitude, 20° pitch — a cell with almost nothing in view, where one
    /// conservative tile is a third of the set). 0.50 is 1.5x headroom.
    const MAX_FP_RATE: f64 = 0.50;

    let (results, text) = run("zoom_cliff", cells::zoom_cliff_cells());

    println!("  alt(m) | pitch | tiles | z range | FN | FP rate");
    for r in &results {
        println!(
            "  {:>6.0} | {:>5.0} | {:>5} | {:>2}..{:<2} | {:>2} | {:.3}",
            r.params.alt_m,
            r.params.pitch_deg,
            r.tiles,
            r.min_z,
            r.max_z,
            r.false_negatives,
            r.fp_rate()
        );
    }

    let s = report::summarize(&results);
    assert!(
        s.total_false_negatives <= MAX_FN,
        "zoom cliff probe produced {} false negatives (limit {MAX_FN}).{text}",
        s.total_false_negatives
    );
    assert!(
        s.worst_fp_rate <= MAX_FP_RATE,
        "zoom cliff worst-cell false-positive rate {:.4} exceeds {MAX_FP_RATE}.{text}",
        s.worst_fp_rate
    );
}

// ─────────────────────────────────────────────────────────────────────────────
// Defect probes — the fix-phase to-do list. Each one fails today; each names the
// defect it exposes and carries the real measured number.
// Run with: cargo test --lib culling:: -- --ignored --nocapture
// ─────────────────────────────────────────────────────────────────────────────

/// Isolates the horizon/occlusion stage: how far *inside* the visible limb does
/// the engine start dropping tiles that are still on screen?
///
/// The randomised sweep found its false negatives with `facing_cos ≈ 0.004`, i.e.
/// about 0.2° inside the limb, which points at tile-level horizon culling rather
/// than the frustum. This probe measures that band directly and reproducibly
/// instead of relying on a lucky fuzz cell.
///
/// **Ignored: this fails today.** Measured over 128 cells / **664 097 246 visible
/// samples**: **305 448 false negatives (0.046%), reaching 8.0715° inside the
/// limb.** The band widens with altitude — under 1° at 4 000 km and mid
/// latitudes, 1.6° at 8 700 km, 8.0° at 12 000 km — and nothing beyond 10° is
/// ever affected. The two viewport shapes (1920×1080 and 3840×720) give byte-for
/// byte identical counts, which rules the frustum out and points squarely at the
/// per-tile horizon-culling point (`compute_horizon_culling_point`, computed in
/// f32, and the `is_occluded` test at the top of `QuadtreeNode::update`).
///
/// The printed bucket table is the finding: it shows how wide the broken band is
/// at each altitude. The threshold is the target (zero), not the measured value.
#[test]
#[ignore = "defect probe: tiles are dropped up to 8.07 deg inside the visible limb (horizon-culling conservatism in QuadtreeNode::update); widens with altitude"]
fn test_limb_band_has_no_false_negatives() {
    /// Target: a point inside the visible limb by any margin at all must be
    /// covered. Anything closer to the limb than this is the oracle's own
    /// marginal band and is never scored.
    const MAX_FN_LIMB_DEG: f64 = 0.0;

    let base = cells::baseline();
    let mut probes = Vec::new();
    for alt in [
        400_000.0, 1_000_000.0, 2_000_000.0, 4_000_000.0, 8_700_000.0, 12_000_000.0,
        20_000_000.0, 30_000_000.0,
    ] {
        for lat in [-80.0, -61.4, -30.0, 0.0, 30.0, 48.0, 61.4, 80.0] {
            for (w, h) in [(1920u32, 1080u32), (3840, 720)] {
                probes.push(ViewParams {
                    sweep: "limb-band",
                    lat_deg: lat,
                    alt_m: alt,
                    width: w,
                    height: h,
                    ..base.clone()
                });
            }
        }
    }

    let t0 = std::time::Instant::now();
    let results = sweep::measure_limb_bands(&probes);
    println!(
        "  [limb_band] {} cells in {:.2?} ({} rayon threads)",
        results.len(),
        t0.elapsed(),
        sweep::harness_pool().current_num_threads()
    );

    println!(
        "  limb angle bucket (deg, upper edge) -> false negatives / visible samples, \
         summed over {} cells",
        results.len()
    );
    let mut totals = vec![(0.0_f64, 0usize, 0usize); sweep::LIMB_BUCKET_EDGES_DEG.len()];
    for r in &results {
        for (i, (edge, vis, fneg)) in r.buckets.iter().enumerate() {
            totals[i].0 = *edge;
            totals[i].1 += vis;
            totals[i].2 += fneg;
        }
    }
    for (edge, vis, fneg) in &totals {
        println!("    <= {edge:>5.2} deg : {fneg:>6} / {vis:>7}");
    }

    let worst = results
        .iter()
        .map(|r| r.worst_fn_deg)
        .fold(0.0_f64, f64::max);
    let total_fn: usize = results.iter().map(|r| r.total_false_negatives).sum();
    let total_vis: usize = results.iter().map(|r| r.total_visible).sum();
    println!(
        "  => {total_fn} false negatives out of {total_vis} visible samples; \
         the broken band reaches {worst:.4} deg inside the limb"
    );
    for r in &results {
        if r.total_false_negatives > 0 {
            println!(
                "     lat={:>6.1} alt={:>12.0}m {}x{} : FN={} band={:.4} deg",
                r.params.lat_deg,
                r.params.alt_m,
                r.params.width,
                r.params.height,
                r.total_false_negatives,
                r.worst_fn_deg
            );
        }
    }

    assert!(
        worst <= MAX_FN_LIMB_DEG,
        "tiles are being culled up to {worst:.4} deg inside the visible limb \
         ({total_fn} false negatives of {total_vis} visible samples); limit {MAX_FN_LIMB_DEG} deg"
    );
}

/// The **second, distinct** defect family the fuzz sweep turned up: false
/// negatives that are nowhere near the limb.
///
/// One fuzz cell — Tracking mode, 35 m altitude, deepest zoom 20 — produced 101
/// false negatives at limb angles of 60-65°, i.e. in the middle of the visible
/// disc, not at its edge. That rules out horizon culling entirely and points at
/// the near-ground/high-zoom regime, where
/// [`super::test_analytic_planes::test_f32_plane_downcast_error_vs_tile_size`]
/// measures the f32 frustum-plane quantum at ~0.128 m against a zoom-20 tile's
/// ~0.3 m projected radius — the culling decision is made with an error worth
/// 40% of the tile.
///
/// This probe sweeps that regime deliberately instead of waiting for the fuzzer
/// to stumble into it, and reports how many of its misses are *not* limb-related.
///
/// **Ignored: this fails today, and this is the most serious thing the harness
/// found.** Measured over 576 cells / 16 612 092 visible samples:
///
/// * **8 cells with false negatives, 223 592 misses in total (1.35%).**
/// * **6 of those 8 are `CameraMode::Tracking` at 5 m altitude looking straight
///   down, and they are 100% false negative** — every sample on screen is ground,
///   and the quadtree returns **1 tile** (roll 0°) or **zero tiles at all**
///   (roll 90° and 176°). That is a blank globe, not a hole in it. `Free` and
///   `Cockpit` at the same position are clean, so it is specific to Tracking's
///   znear, which is `clamp(|local_pos| * 0.05, 1e-8, 5e-6)` megameters — pinned
///   at 5 m against a `zfar` of 6 378 km, a depth range of 1.3e9:1 that the f32
///   frustum-plane downcast cannot represent.
/// * The remaining 2 cells lose a single sample each at 35 m / 30° pitch, one in
///   Free and one in Tracking, at zoom 19-20.
/// * **None** of the misses is within 5° of the limb, confirming this is a
///   separate defect from the limb band.
#[test]
#[ignore = "defect probe: Tracking mode at 5 m altitude returns 0-1 tiles for a screen full of ground (100% false negative); plus isolated z19-20 misses"]
fn test_near_ground_high_zoom_has_no_false_negatives() {
    const MAX_FN: usize = 0;
    /// A miss further inside the limb than this cannot be horizon-culling
    /// conservatism, so it is counted separately.
    const NOT_LIMB_DEG: f64 = 5.0;

    let mut probes = Vec::new();
    for mode in cells::modes() {
        for alt in [5.0, 10.0, 20.0, 35.0, 50.0, 100.0, 200.0, 500.0] {
            for pitch in [0.0, 12.87, 30.0, 45.0] {
                for roll in [0.0, 90.0, 176.0] {
                    for (w, h) in [(1080u32, 1920u32), (1920, 1080)] {
                        probes.push(ViewParams {
                            sweep: "near-ground",
                            lat_deg: -12.7,
                            lon_deg: 147.564,
                            alt_m: alt,
                            pitch_deg: pitch,
                            yaw_deg: 38.25,
                            roll_deg: roll,
                            width: w,
                            height: h,
                            mode,
                        });
                    }
                }
            }
        }
    }

    let (results, text) = run("near_ground_high_zoom", probes);
    let s = report::summarize(&results);

    let far_from_limb: usize = results
        .iter()
        .flat_map(|r| r.fn_records.iter())
        .filter(|f| f.limb_deg() > NOT_LIMB_DEG)
        .count();
    println!(
        "  {} of {} false negatives are more than {NOT_LIMB_DEG} deg inside the limb \
         (i.e. not horizon-culling conservatism)",
        far_from_limb, s.total_false_negatives
    );

    assert!(
        s.total_false_negatives <= MAX_FN,
        "near-ground high-zoom sweep produced {} false negatives in {} of {} cells \
         ({far_from_limb} of them far from the limb); limit {MAX_FN}.{text}",
        s.total_false_negatives,
        s.cells_with_fn,
        s.cells
    );
}

/// Seeded random sweep over the whole parameter space at once — the interactions
/// the one-factor-at-a-time sweep cannot reach.
///
/// Deterministic: seed 0x5EED_C0DE, 400 cells, identical on every machine.
///
/// **Ignored: this fails today.** Measured over 100 000 cells /
/// **1 079 616 535 visible samples**: **62 573 false negatives (0.0058%) in 416
/// cells**, worst cell 27.24%. Broken down by where the misses are, they are the
/// same two families the focused probes isolate:
///
/// * **81.9% lie within 5° of the visible limb**, on coarse tiles — by deepest
///   zoom: z1 29 372, z2 17 928, z3 7 329, z4 1 647. See
///   [`test_limb_band_has_no_false_negatives`].
/// * **6 127 are at zoom 19-20**, near the ground and far from the limb. See
///   [`test_near_ground_high_zoom_has_no_false_negatives`].
/// * By camera mode: Free 58 831, Tracking 3 297, Cockpit 445 — Free dominates
///   simply because it is the mode that gets used at high altitude.
///
/// Also measured here: whole-sweep FP rate **5.44%** (73 640 wasted tiles of
/// 1 353 699), with individual cells reaching 100% — views where the frustum
/// intersects a tile's bounding volume but no part of the tile's surface is
/// actually visible. That is the conservatism the plane-only separating-axis test
/// buys, quantified.
#[test]
#[ignore = "defect probe: randomised 100k-cell sweep exposes 62573 false negatives in 416 cells, split between the limb band and the near-ground high-zoom regime"]
fn test_fuzz_sweep_has_no_false_negatives() {
    const MAX_FN: usize = 0;
    const SEED: u32 = 0x5EED_C0DE;
    /// Sized for a many-core machine: see the module header for the recommended
    /// command line and measured wall-clock.
    const CELLS: usize = 100_000;

    let (results, text) = run("fuzz_sweep", cells::fuzz_cells(CELLS, SEED));
    let s = report::summarize(&results);
    assert!(
        s.total_false_negatives <= MAX_FN,
        "fuzz sweep (seed {SEED:#x}, {CELLS} cells) produced {} false negatives \
         in {} of {} cells (limit {MAX_FN}).{text}",
        s.total_false_negatives,
        s.cells_with_fn,
        s.cells
    );
}
