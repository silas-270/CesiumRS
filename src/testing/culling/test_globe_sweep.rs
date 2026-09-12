//! # Layer 2 — globe sweep against the independent f64 oracle.
//!
//! Every test here is the same shape: generate parameter cells
//! ([`super::cells`]), measure each one ([`super::sweep::measure_cell`]), write a
//! CSV and a failure map ([`super::report`]), then assert against a named,
//! documented threshold.
//!
//! ## Which tests must pass
//!
//! **All of them.** Every test in this file is a regression guard; none is
//! `#[ignore]`d. That was not true when the harness was written: it shipped with
//! five ignored defect probes naming real, pre-existing culling defects, on the
//! principle that thresholds should never be weakened to make a suite green. The
//! culling rework (`docs/culling-math.md`) closed all five, and each probe has been
//! un-ignored in place and re-documented as the invariant it now protects, with the
//! measured number.
//!
//! ## What those five now guard
//!
//! 1. **The limb band.** Tiles used to be dropped up to **8.07°** inside the visible
//!    limb, widening with altitude — 305 448 misses. The cause was the sub-OBB
//!    back-face heuristic, whose margin was short by a factor `h = √(C²−1)`
//!    (§4.1), not the horizon-culling point the harness originally suspected.
//!    [`test_limb_band_has_no_false_negatives`] now measures **0.0000°**.
//! 2. **Tracking at ~5 m** used to return 0–1 tiles for a viewport completely
//!    filled with ground — the near plane rejecting a z=17 ancestor on 0.058 m of
//!    true clearance computed in f32 from 6.378 Mm operands.
//!    [`test_near_ground_high_zoom_has_no_false_negatives`].
//! 3. **z = 19–20 tile edges**, where the f32 tile-bounds quantum (1.7 m) is a real
//!    fraction of a 38 m tile. Same test, plus the fuzz sweep.
//! 4. **The 100 000-cell fuzz sweep**, 1 079 616 535 visible samples, now 0 FN.
//! 5. **The plane array has no dead entry** —
//!    [`super::test_analytic_planes::test_frustum_plane_set_has_no_dead_entry`].
//!
//! No threshold here has been relaxed to make a test green.
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
        let frustum = cesium_engine::globe::quadtree::Frustum::new(planes, gp)
            .with_corners(cam.frustum_corners_relative(p.aspect() as f32));

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

// ## How the false-positive thresholds are calibrated
//
// Every FP threshold below was re-measured after
// [`sweep::TILE_SAMPLE_STEPS`] went from 4 to 32, i.e. from 25 to 1089 sample
// points per tile. That change moved the whole-harness FP total from 27 103
// tiles (2.054 %) to 3 264 (0.247 %) without touching one line of engine code:
// most of what the old density called waste was a tile whose genuinely visible
// part was a strip thinner than the sample spacing. See `TILE_SAMPLE_STEPS` for
// the convergence table and the monotonicity argument (more samples can only
// lower FP, so the reported figure is an upper bound).
//
// Two things follow for the thresholds here.
//
// **The measured value is now zero on every asserted sweep.** nadir_ladder,
// axis_sweep and zoom_cliff each score exactly **0 false-positive tiles** at
// N = 32. The numbers the old doc comments quoted (0.2727, 0.333, 0.0047) were
// measured on the pre-rework engine; on the current engine they were already
// 0.0000, 0.0833 and 0.0000 at N = 4.
//
// **The worst-*cell* FP rate stopped being a usable statistic.** It is a rate
// over a per-cell denominator that is routinely tiny — the sparsest cells in
// these sweeps hold **2** tiles — so a single legitimately conservative tile
// scores 0.50 there. A worst-cell threshold therefore cannot be tightened below
// ~0.5 without becoming "zero FP anywhere" in disguise, and at 0.45 it was
// satisfiable by hundreds of wasted tiles spread across the fatter cells. The
// guards below assert on the **absolute count of wasted tiles over the sweep**
// instead, which has no denominator pathology and is two to three orders of
// magnitude tighter. This is a tightening in every direction, so it cannot cost
// the suite teeth: any perturbation the old rate guard caught, the count guard
// catches too.
//
// **Headroom rule**, applied uniformly: `round(0.5 % of the sweep's own tile
// total)`, over a measured 0. That is room for a handful of individually
// defensible conservative tiles — the OBB of a limb-straddling tile really can
// bulge past the horizon — while a systematic regression is one to two orders
// above it (a deliberately broken frustum plane produced a whole-sweep FP rate
// of 0.24 on nadir_ladder, i.e. ~150 tiles against a budget of 3). The
// per-sweep totals are recorded in each threshold's doc comment so a future
// change of cell set forces a re-derivation rather than an inherited number.

/// Nadir views at a range of altitudes and latitudes: the tamest geometry the
/// engine ever sees, and the case a user spends most of their time in.
///
/// Threshold: **zero** false negatives. This regime is measured clean, so any FN
/// at all is a regression. The FP threshold is a small non-zero tile budget, not
/// zero, because bounding volumes are conservative by construction — a tile whose
/// OBB pokes over the horizon or past a screen edge is legitimately scheduled.
#[test]
fn test_nadir_ladder_has_no_false_negatives() {
    /// Measured: **0 FN** over 3 568 512 visible samples in 72 cells.
    const MAX_FN: usize = 0;
    /// Measured at N = 32: **0 false-positive tiles** of 636 over 72 cells
    /// (worst-cell rate 0.0000). At N = 4 this sweep also measured 0 — its FP was
    /// never density-limited; the old `0.2727` in this comment was a pre-rework
    /// engine number.
    ///
    /// Budget: 3 tiles = 0.5 % of the sweep's 636. ~50x below the ~150 tiles a
    /// deliberately broken frustum plane produced here.
    const MAX_FP_TILES: usize = 3;

    let (results, text) = run("nadir_ladder", cells::nadir_ladder());
    let s = report::summarize(&results);

    assert!(
        s.total_false_negatives <= MAX_FN,
        "nadir ladder produced {} false negatives (limit {MAX_FN}).{text}",
        s.total_false_negatives
    );
    assert!(
        s.total_false_positive_tiles <= MAX_FP_TILES,
        "nadir ladder wasted {} tiles of {} (limit {MAX_FP_TILES}, worst cell {:.4}).{text}",
        s.total_false_positive_tiles,
        s.total_tiles,
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
/// blow it, but a systematic regression (e.g. the sub-grid rejection being
/// disabled) would. It runs the same cells as
/// [`test_nadir_ladder_has_no_false_negatives`] deliberately: that guard bounds
/// the absolute number of wasted tiles, this one bounds the fraction, so the pair
/// stays meaningful if the cell set ever grows or shrinks.
#[test]
fn test_false_positive_rate_within_budget() {
    /// Whole-sweep FP budget. Measured at N = 32: **0.0000** — 0 wasted tiles of
    /// 636 over 72 cells. (The old `0.0047` here was a pre-rework engine number
    /// against a 213-tile set; the current engine measures 0 at N = 4 as well.)
    ///
    /// 0.005 is the same 0.5 % headroom rule as the count guards above — 3 tiles
    /// of 636 — and still ~48x under the 0.24 a deliberately broken frustum plane
    /// produced.
    const MAX_OVERALL_FP_RATE: f64 = 0.005;

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
    /// Measured at N = 32: **0 false-positive tiles** of 1 365 over the 132
    /// non-degenerate cells of 153 (worst-cell rate 0.0000). At N = 4 the same
    /// engine measured 14 tiles / worst cell 0.1111 — every one of those 14 was a
    /// sampling artifact, not waste.
    ///
    /// Budget: 7 tiles = 0.5 % of 1 365.
    const MAX_FP_TILES: usize = 7;

    let (results, text) = run("axis_sweep", cells::axis_sweep());
    let s = report::summarize(&results);
    assert!(
        s.total_false_negatives <= MAX_FN,
        "axis sweep produced {} false negatives across {} cells (limit {MAX_FN}).{text}",
        s.total_false_negatives,
        s.cells
    );
    assert!(
        s.total_false_positive_tiles <= MAX_FP_TILES,
        "axis sweep wasted {} tiles of {} (limit {MAX_FP_TILES}, worst cell {:.4}).{text}",
        s.total_false_positive_tiles,
        s.total_tiles,
        s.worst_fp_rate
    );
}

/// Pitch sweep from nadir to well above the horizon, across altitudes,
/// latitudes and the ±0.1° neighbourhood of the exact horizon. 1 344 cells.
///
/// **Was a defect probe; now a guard.** It used to measure **11 false negatives of
/// 27 556 756 visible samples, in 2 of 1 344 cells** — both at 12 000 km with the
/// camera tilted 15–25° off nadir, deepest zoom 1–2, every miss between 0.49° and
/// 1.48° inside the limb *and* within 1.4 % of the bottom edge of the viewport, i.e.
/// coarse tiles at the intersection of the limb and the screen edge. Same family as
/// [`test_limb_band_has_no_false_negatives`]: the sub-OBB back-face heuristic.
///
/// **Now: 0 false negatives of 27 556 756 visible samples.**
///
/// The invariant: a tile that is simultaneously near the limb *and* near a screen
/// edge is the hardest case for the two culling stages to get right jointly, because
/// each one alone sees only a marginal rejection. This sweep is where a regression
/// in either stage's tolerance would surface first.
#[test]
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
/// samples, and at N = 32 sample points per tile the FP column is **identically
/// zero across the whole ladder**, so there is no discontinuity at the boundary
/// to explain. (At N = 4 two tiles were flagged, both in near-nadir low-altitude
/// cells with 3–15 tiles in view, neither above z = 16 — a sampling artifact in
/// the same place the earlier note attributed to conservatism.)
#[test]
fn test_zoom_cliff_probe() {
    const MAX_FN: usize = 0;
    /// Measured at N = 32: **0 false-positive tiles** of 3 286 over 132 cells
    /// (worst-cell rate 0.0000). At N = 4 the same engine measured 2 tiles /
    /// worst cell 0.0833; the old `0.333` here was a pre-rework number.
    ///
    /// Budget: 16 tiles = 0.5 % of 3 286. This is the sweep that straddles the
    /// z = 16/17 conservatism change, so the count guard matters more here than a
    /// worst-cell rate would: it catches a drift spread thinly across the ladder,
    /// which is exactly the shape a sub-grid regression takes.
    const MAX_FP_TILES: usize = 16;

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
        s.total_false_positive_tiles <= MAX_FP_TILES,
        "zoom cliff wasted {} tiles of {} (limit {MAX_FP_TILES}, worst cell {:.4}).{text}",
        s.total_false_positive_tiles,
        s.total_tiles,
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
/// **Was the harness's headline defect; now its headline guard.** It used to measure
/// over 128 cells / 664 097 246 visible samples: **305 448 false negatives
/// (0.046 %), reaching 8.0715° inside the limb**, widening with altitude — under 1°
/// at 4 000 km, 1.6° at 8 700 km, 8.0° at 12 000 km. That the two viewport shapes
/// (1920×1080 and 3840×720) gave byte-for-byte identical counts ruled the frustum
/// out.
///
/// **Now: 0 false negatives, band 0.0000°, over the same 664 097 246 samples.**
///
/// The harness's own note blamed `compute_horizon_culling_point`. It was wrong, and
/// usefully so: the 4-corner spherical-cap reduction is *provably sound* (§3.6) — it
/// is merely loose. The band came from the **sub-OBB back-face heuristic**
/// `normal·(cam − centre) > −max_extent`, whose margin is short by a factor
/// `h = √(C²−1)` and is therefore unsound above 2 642 km altitude (§4.1). Simulating
/// that one line reproduced 8.0760° at 12 000 km against this test's 8.0715°.
///
/// The invariant: **the horizon stage may never cull a tile with a visible point.**
/// It is now the exact supremum of `q·c` over the tile's lon/lat rectangle, so on
/// zero-relief terrain it has FP = 0 as well as FN = 0 — it is not a bound, it is
/// the answer. The bucket table stays printed because it is the cheapest way to see
/// a tolerance regression: a band of *any* width is a failure.
#[test]
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

/// **Was the most serious thing the harness found; now a guard.** Near-ground,
/// high-zoom views, far from the limb — the regime that rules horizon culling out
/// entirely.
///
/// It used to measure, over 576 cells / 16 612 092 visible samples, **223 592 misses
/// in 8 cells (1.35 %)**, of which:
///
/// * **6 cells were `CameraMode::Tracking` at 5 m looking straight down, and they
///   were 100 % false negative** — every sample on screen was ground and the
///   quadtree returned **1 tile** (roll 0°) or **none at all** (roll 90°, 176°).
///   A blank globe, not a hole in one. `Free` and `Cockpit` at the same position
///   were clean, which localised it to Tracking's znear, `clamp(|local_pos|·0.05,
///   1e-8, 5e-6)` Mm — pinned at 5 m against a zfar of 6 378 km.
/// * 2 more cells lost a single sample each at 35 m / 30° pitch, at zoom 19–20.
///
/// **Now: 0 false negatives over the same 16 612 092 samples.** Two independent
/// causes, both removed:
///
/// 1. The near plane is gone from tile culling (§2.6). It rejected the z=17 ancestor
///    on a *true* signed distance of 0.058 m computed in f32 from 6.378 Mm operands,
///    where the rounding noise is ±0.5 m; `QuadtreeNode::update` nulls `children` on
///    a cull, so killing z=17 killed z=18–20 and the branch returned nothing. The
///    camera-relative f64 subtraction (I-2) would have fixed the arithmetic, but the
///    exact answer is still −0.27 m against a 0.30 m projected radius — 10 % from a
///    cliff edge is not a design, so the test itself was deleted as provably vacuous
///    whenever `znear < altitude`.
/// 2. Tile bounds are derived in f64 (I-5). The f32 quantum is 1.7 m of ground,
///    which at z=20 is 4.5 % of a tile.
///
/// The invariant: **a viewport full of ground must be covered by tiles, in every
/// camera mode.** `NOT_LIMB_DEG` keeps the two defect families separable in the
/// printout if this ever goes red again.
#[test]
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
/// the one-factor-at-a-time sweep cannot reach. Deterministic: seed 0x5EED_C0DE,
/// 100 000 cells, identical on every machine. This is the broadest statement the
/// harness can make.
///
/// **Was a defect probe; now the top-level guard.** It used to measure, over
/// **1 079 616 535 visible samples**, **62 573 false negatives (0.0058 %) in 416
/// cells**, worst cell 27.24 %, splitting into exactly the two families the focused
/// probes isolate: 81.9 % within 5° of the limb on coarse tiles, and 6 127 at zoom
/// 19–20 near the ground.
///
/// **Now: 0 false negatives over the same 1 079 616 535 visible samples.**
///
/// False positives, measured on the same run and excluding cells where the oracle
/// finds no visible surface at all (see `CellResult::is_degenerate`):
///
/// | | FP | mean `QuadtreeManager::update` |
/// |---|---|---|
/// | before the rework | 5.58 % | 7.7 µs |
/// | after, before the sub-box taper (`15114a9`) | 4.18 % | 4.3 µs |
/// | after, at N = 4 sample points | 1.99 % | **4.3 µs** |
/// | after, at N = 32 sample points | **0.212 %** (1 385 tiles of 654 250) | — |
///
/// The last two rows are the *same engine*: only the FP instrument's density
/// changed. This sweep is where the sampling artifact was largest, because a
/// uniformly random camera pose lands a large share of tiles in exactly the
/// limb-straddling geometry that a 5 × 5 grid cannot resolve.
///
/// The FP number is *not* asserted here. It is a rate, bounded deliberately by
/// [`test_false_positive_rate_within_budget`] and by the whole-sweep tile budgets
/// in [`test_axis_sweep_has_no_false_negatives`] and
/// [`test_nadir_ladder_has_no_false_negatives`], because a whole-sweep FP
/// assertion over 100 000 randomised cells would be a threshold nobody could
/// reason about. Individual cells still reach 100 %: views where the frustum
/// meets a tile's bounding volume but no part of the tile's surface is visible.
/// That is the residual floor §5.4 describes and deliberately leaves open.
#[test]
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
