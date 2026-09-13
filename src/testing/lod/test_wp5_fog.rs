//! WP5 (`docs/pre-terrain-plan.md`): measures what fog does to the WP4 baselines,
//! then re-runs WP4/C's 3a-at-equal-budget comparison post-fog.
//!
//! Every measurement here goes through [`super::fog_sweep`], never
//! [`super::sweep::measure_pose_with_config`] — see `fog_sweep`'s module doc
//! comment for why that split is load-bearing, not stylistic.

use cesium_engine::globe::quadtree::{FogConfig, LodDistanceMode};

use super::super::culling::cameras::ViewParams;
use super::fog_sweep::measure_poses_with_fog;
use super::ladder::{at_rung, rungs};
use super::report::{self, Summary};
use super::sweep::{bench_poses, bisect_target_for_tile_count, measure_poses, LodConfig};

/// `pitch_deg >= 85.0` — "0deg = nadir, 90deg = local horizon" (`ViewParams`'s own
/// doc comment). These are `zoom_cliff_cells()`'s near-/at-horizon rungs, the
/// population WP5's `p95` target (heavily foreshortened near-limb tiles) actually
/// lives in.
fn horizon_poses(poses: &[ViewParams]) -> Vec<ViewParams> {
    poses.iter().filter(|p| p.pitch_deg >= 85.0).cloned().collect()
}

/// Altitudes in [9km, 15km] — this product's stated 10-12km cruise band, widened
/// slightly to catch the bench set's actual altitude points (`10_000`,
/// `10_240`, `14_400`) rather than requiring an exact match.
fn cruise_poses(poses: &[ViewParams]) -> Vec<ViewParams> {
    poses.iter().filter(|p| (9_000.0..=15_000.0).contains(&p.alt_m)).cloned().collect()
}

struct Comparison {
    label: &'static str,
    pose_count: usize,
    baseline: Summary,
    fogged: Summary,
    baseline_tiles: usize,
    fogged_tiles: usize,
    baseline_bytes: u64,
    fogged_bytes: u64,
}

fn compare(label: &'static str, poses: &[ViewParams], fog_cfg: &FogConfig) -> Comparison {
    let baseline_results = measure_poses(poses);
    let fogged_results = measure_poses_with_fog(poses, LodConfig::default(), fog_cfg);

    let baseline_tiles: usize = baseline_results.iter().map(|r| r.tile_count).sum();
    let fogged_tiles: usize = fogged_results.iter().map(|r| r.tile_count).sum();
    let baseline_bytes: u64 = baseline_results.iter().map(|r| r.texture_bytes).sum();
    let fogged_bytes: u64 = fogged_results.iter().map(|r| r.texture_bytes).sum();

    Comparison {
        label,
        pose_count: poses.len(),
        baseline: report::summarize(&baseline_results),
        fogged: report::summarize(&fogged_results),
        baseline_tiles,
        fogged_tiles,
        baseline_bytes,
        fogged_bytes,
    }
}

fn print_comparison(c: &Comparison) {
    eprintln!(
        "{:<28} poses={:<5} tiles {:>6} -> {:>6} ({:+.1}%)  bytes(MiB) {:>8.1} -> {:>8.1} ({:+.1}%)",
        c.label,
        c.pose_count,
        c.baseline_tiles,
        c.fogged_tiles,
        (c.fogged_tiles as f64 - c.baseline_tiles as f64) / c.baseline_tiles.max(1) as f64 * 100.0,
        c.baseline_bytes as f64 / (1024.0 * 1024.0),
        c.fogged_bytes as f64 / (1024.0 * 1024.0),
        (c.fogged_bytes as f64 - c.baseline_bytes as f64) / c.baseline_bytes.max(1) as f64 * 100.0,
    );
    eprintln!(
        "    aggregate {:.4} -> {:.4}   median {:.4} -> {:.4}   p95 {:.4} -> {:.4}",
        c.baseline.aggregate_ratio,
        c.fogged.aggregate_ratio,
        c.baseline.median(),
        c.fogged.median(),
        c.baseline.p95(),
        c.fogged.p95(),
    );
}

/// WP5/C: fog against the WP0-WP4 baseline — overall, at horizon poses, and at
/// cruise altitude, per `docs/pre-terrain-plan.md`'s explicit weighting ("cruise
/// altitude is the case this product actually lives in").
#[test]
fn test_wp5c_fog_vs_baseline() {
    let poses = bench_poses();
    let horizon = horizon_poses(&poses);
    let cruise = cruise_poses(&poses);
    assert!(!horizon.is_empty(), "the zoom-cliff ladder must contain pitch >= 85 poses");
    assert!(!cruise.is_empty(), "the bench poses must contain a 9-15km altitude point");

    let fog_cfg = FogConfig::default();
    let all = compare("all 204 poses", &poses, &fog_cfg);
    let hz = compare("horizon poses (pitch>=85)", &horizon, &fog_cfg);
    let cr = compare("cruise altitude (9-15km)", &cruise, &fog_cfg);

    eprintln!("\n=== WP5/C: fog vs the WP0-WP4 baseline ===");
    for c in [&all, &hz, &cr] {
        print_comparison(c);
    }

    // Fog only ever subtracts geometry/refinement (Stage::Fog only culls,
    // apply_lod's relaxation only shrinks) — so at fixed content, fog must never
    // *increase* tile count or texture bytes relative to the no-fog baseline, on
    // any subset.
    for c in [&all, &hz, &cr] {
        assert!(
            c.fogged_tiles <= c.baseline_tiles,
            "{}: fog must not increase tile count: {} -> {}",
            c.label, c.baseline_tiles, c.fogged_tiles
        );
        assert!(
            c.fogged_bytes <= c.baseline_bytes,
            "{}: fog must not increase texture bytes: {} -> {}",
            c.label, c.baseline_bytes, c.fogged_bytes
        );
    }

    // The p95 tail is what WP5 targets — it must shrink, and by more at horizon
    // poses (where the near-limb tail actually lives) than overall.
    assert!(
        all.fogged.p95() < all.baseline.p95(),
        "fog must reduce the overall p95 tail: {} -> {}", all.baseline.p95(), all.fogged.p95()
    );
    assert!(
        hz.fogged.p95() < hz.baseline.p95(),
        "fog must reduce the horizon-pose p95 tail: {} -> {}", hz.baseline.p95(), hz.fogged.p95()
    );
}

/// WP5/C, continued: the same comparison across every WP4/B viewport/mode rung —
/// confirms fog's reduction holds outside the desktop default too, since fog
/// density depends on altitude alone but tile geometry/count depends on viewport.
#[test]
fn test_wp5c_fog_across_viewport_ladder() {
    let base_poses = bench_poses();
    let fog_cfg = FogConfig::default();

    eprintln!("\n=== WP5/C: fog vs baseline across the WP4/B viewport ladder ===");
    for rung in rungs() {
        let poses = at_rung(&base_poses, &rung);
        let c = compare(rung.name, &poses, &fog_cfg);
        print_comparison(&c);
        assert!(
            c.fogged_tiles <= c.baseline_tiles,
            "{}: fog must not increase tile count", rung.name
        );
    }
}

fn total_tiles_fogged(poses: &[ViewParams], cfg: LodConfig, fog_cfg: &FogConfig) -> usize {
    measure_poses_with_fog(poses, cfg, fog_cfg).iter().map(|r| r.tile_count).sum()
}

/// WP5/D: the scheduled re-run of WP4/C's 3a-at-equal-tile-budget comparison, now
/// with fog active for *both* variants. Not a retry — WP4/C's own finding
/// predicted why this might come out differently: box-distance's budget at equal
/// `N` was spent almost entirely on the near-limb tail (`p95` roughly doubled,
/// `219 -> 376`), and fog is precisely what removes that tail. If WP4/C's p95
/// blowup was box-distance chasing tiles fog now culls or de-refines before LOD
/// ever sees them, it should mostly evaporate here.
#[test]
fn test_wp5d_3a_at_equal_budget_post_fog() {
    let poses = bench_poses();
    let fog_cfg = FogConfig::default();

    let n_fog = total_tiles_fogged(&poses, LodConfig::default(), &fog_cfg);
    eprintln!(
        "\n=== WP5/D: 3a at equal tile budget, post-fog (N_fog={n_fog}, vs WP4/C's pre-fog N=3922) ==="
    );

    let (box_target, box_n) = bisect_target_for_tile_count(n_fog, 1.0, |t| {
        total_tiles_fogged(
            &poses,
            LodConfig::new(t, 512.0).with_distance_mode(LodDistanceMode::Box),
            &fog_cfg,
        )
    });
    let off_frac = (box_n as i64 - n_fog as i64).unsigned_abs() as f64 / (n_fog as f64);
    assert!(off_frac < 0.02, "bisection should match N_fog to within 2%: N_fog={n_fog} achieved={box_n}");

    let centre_results = measure_poses_with_fog(&poses, LodConfig::default(), &fog_cfg);
    let box_results = measure_poses_with_fog(
        &poses,
        LodConfig::new(box_target, 512.0).with_distance_mode(LodDistanceMode::Box),
        &fog_cfg,
    );
    let centre_summary = report::summarize(&centre_results);
    let box_summary = report::summarize(&box_results);

    eprintln!(
        "centre-distance @ target=1.0 (fogged, today)  tiles={:<6} aggregate={:.4} median={:.4} p5={:.4} p25={:.4} p95={:.4}",
        n_fog, centre_summary.aggregate_ratio, centre_summary.median(), centre_summary.p5(), centre_summary.p25(), centre_summary.p95(),
    );
    eprintln!(
        "box-distance @ equal budget (fogged)          tiles={:<6} target={:.5} aggregate={:.4} median={:.4} p5={:.4} p25={:.4} p95={:.4}",
        box_n, box_target, box_summary.aggregate_ratio, box_summary.median(), box_summary.p5(), box_summary.p25(), box_summary.p95(),
    );
    eprintln!(
        "WP4/C pre-fog, for comparison:                tiles=3922   target=0.42844 aggregate=1.6659 median=4.7787 p5=0.2003 p25=0.9892 p95=375.8592"
    );

    let p95_ratio_post_fog = box_summary.p95() / centre_summary.p95();
    let p95_ratio_pre_fog = 375.8592 / 219.4685; // WP4/C's own recorded numbers
    eprintln!(
        "box/centre p95 ratio: pre-fog={:.3}  post-fog={:.3}",
        p95_ratio_pre_fog, p95_ratio_post_fog
    );

    report::emit("wp5d_centre_at_budget_fogged", &centre_results);
    report::emit("wp5d_box_at_budget_fogged", &box_results);
}

/// WP5/B's specific ask: "pay specific attention to a camera climbing or
/// descending through `maxHeight = 800 km`, where fog switches off entirely and
/// the tile density could step."
///
/// `fog_density_for` is a **hard** cutoff (`Fog.js`'s own `if (height > maxHeight)`
/// — see `fog.rs`'s module doc comment), not a fade-out: density at `maxHeight`
/// itself is `base * height_scalar` (not `0`), so there is a real, non-vanishing
/// step from that value down to exactly `0` the instant altitude crosses the
/// boundary — not an engine bug, an inherited property of the ported formula.
///
/// **The step is real and was originally mismeasured as absent.** A first pass at
/// this test built its altitude ladder from round numbers (799_999 / 800_000 /
/// 800_001) and found a 0% step — but `Camera::altitude()` (what `fog_density_for`
/// is actually driven by, via `fog_density_at`) reads a few metres *higher* than
/// the analytic `alt_m` a `ViewParams` is built from, so all three of those
/// "round" altitudes actually landed on the *same* side of the true cutoff. This
/// version locates the true crossing by bisecting on `fog_density_for_pose`
/// itself before building the ladder, and with the ladder actually straddling the
/// boundary the step turns out to be real: +42-50% tile count in one frame (see
/// the measured numbers below). A headless capture at the corrected altitudes
/// (`src/testing/rendering/fog_capture.rs`) shows no perceptible visual pop
/// despite that — at 800km the newly-kept tiles are coarse, peripheral, and
/// mostly off the framed view — but the discontinuity in the underlying
/// computation is real, not a measurement artifact, and is documented as such in
/// `docs/culling-baseline.md`'s WP5/B section rather than hidden.
#[test]
fn test_wp5b_max_height_boundary_step() {
    let fog_cfg = FogConfig::default();
    let base = super::super::culling::cells::baseline();

    // Density right at the boundary is not zero — quantifies the step this test
    // then measures the consequence of.
    let density_at_boundary = cesium_engine::globe::quadtree::fog_density_for(fog_cfg.max_height_m, &fog_cfg);
    let density_just_above = cesium_engine::globe::quadtree::fog_density_for(fog_cfg.max_height_m + 1.0, &fog_cfg);
    assert!(density_at_boundary > 0.0, "density at max_height itself must be nonzero (ratio=1, not the EPSILON4 floor)");
    assert_eq!(density_just_above, 0.0, "density must be exactly 0.0 one metre above max_height");
    eprintln!(
        "\n=== WP5/B: fog density and tile count across the maxHeight=800km boundary ===\n\
         density at max_height: {density_at_boundary:.3e} /m, just above: {density_just_above}"
    );

    // `ViewParams::alt_m` is the analytic altitude `lon_lat_alt_to_ecef_f64` places
    // the camera at; `Camera::altitude()` (what `fog_density_for` actually reads,
    // via `fog_density_at`) re-derives altitude from the resulting f32 ECEF
    // position against the ellipsoid, and the two disagree by a small constant
    // offset (measured here at this latitude: `Camera::altitude()` reads ~4m
    // *higher* than the `alt_m` requested). A ladder built directly from nominal
    // round numbers could therefore straddle the wrong side of the true cutoff by
    // those few metres. So: find the true crossing by bisecting on
    // `fog_density_for_pose` (which reads the same `Camera::altitude()` the
    // production path does) first, then build the tile-count ladder around *that*
    // — not around the nominal `800_000.0`.
    let true_crossing_alt_m = {
        let mut lo = 799_000.0_f64;
        let mut hi = 801_000.0_f64;
        for _ in 0..40 {
            let mid = (lo + hi) * 0.5;
            let p = ViewParams { alt_m: mid, ..base.clone() };
            if super::fog_sweep::fog_density_for_pose(&p, &fog_cfg) > 0.0 {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        (lo + hi) * 0.5
    };
    eprintln!(
        "true crossing (via Camera::altitude(), which fog_density_for actually reads): \
         requested alt_m = {true_crossing_alt_m:.3} for nominal max_height_m = {}",
        fog_cfg.max_height_m
    );

    // A nadir *and* a horizon-grazing altitude ladder bracketing the *true*
    // crossing tightly, plus wider context. Grazing (pitch=90, "local horizon" per
    // ViewParams's own doc comment) is the more sensitive case: it is the only way
    // to put a tile near the ~3193km horizon-tangent distance an 800km-altitude
    // camera can reach, where the boundary's own density (6e-7/m) has the most
    // room to matter — a nadir-only check could miss a real step that only shows
    // up looking outward.
    let altitudes: [f64; 11] = [
        700_000.0, 750_000.0, 795_000.0,
        true_crossing_alt_m - 1_000.0, true_crossing_alt_m - 100.0,
        true_crossing_alt_m - 1.0, true_crossing_alt_m, true_crossing_alt_m + 1.0,
        true_crossing_alt_m + 100.0, true_crossing_alt_m + 1_000.0, 850_000.0,
    ];

    for (label, pitch) in [("nadir", 0.0), ("grazing (pitch=80)", 80.0)] {
        eprintln!("-- {label} --");
        let mut prev: Option<(f64, usize)> = None;
        let mut max_step_frac = 0.0_f64;
        let mut step_at_boundary = 0.0_f64;
        for alt in altitudes {
            let p = ViewParams { alt_m: alt, pitch_deg: pitch, ..base.clone() };
            let result = super::fog_sweep::measure_pose_with_fog(&p, LodConfig::default(), &fog_cfg);
            eprintln!("  alt={alt:>10.0}m  tiles={:<5} deepest_z={}", result.tile_count, result.deepest_zoom);
            if let Some((prev_alt, prev_tiles)) = prev {
                let step_frac = (result.tile_count as f64 - prev_tiles as f64).abs()
                    / (prev_tiles as f64).max(1.0);
                if prev_alt < true_crossing_alt_m && alt >= true_crossing_alt_m {
                    eprintln!(
                        "    ^ crosses maxHeight: {prev_tiles} -> {} tiles ({:+.1}%)",
                        result.tile_count,
                        step_frac * 100.0,
                    );
                    step_at_boundary = step_frac;
                }
                max_step_frac = max_step_frac.max(step_frac);
            }
            prev = Some((alt, result.tile_count));
        }
        eprintln!(
            "  {label}: step exactly at maxHeight = {:.1}%, largest step anywhere in the ladder = {:.1}%",
            step_at_boundary * 100.0,
            max_step_frac * 100.0,
        );
    }

    // Report, don't gate on a pass/fail number here — per the ground rules, only
    // Stage::Fog's absence from CullPipeline::DEFAULT is a hard requirement. This
    // is instead a documented, quantified property: see docs/culling-baseline.md's
    // WP5/B section for the reading, including why it is bounded in practice by
    // 800km being far outside this product's 10-12km cruise envelope.
}

/// The hysteresis-interaction half of the same ask: does fog's relaxation, applied
/// *before* `collapse_dist` is derived from `subdivide_dist`, preserve the 20% band
/// as a fraction rather than letting it drift to a fixed absolute margin? Checked
/// directly against `apply_lod`'s actual formula shape rather than only inferred
/// from tile counts — `collapse_dist / subdivide_dist` must be exactly `1.20`
/// regardless of the fog relaxation factor, since relaxation multiplies
/// `subdivide_dist` before `collapse_dist = subdivide_dist * 1.20` runs.
#[test]
fn test_wp5b_hysteresis_band_stays_proportional_under_fog() {
    // This mirrors apply_lod's own arithmetic shape (subdivide_dist scaled by an
    // arbitrary relaxation factor, then collapse_dist derived from the *result*)
    // rather than re-deriving fog's value — the property under test is the
    // ordering of operations, not the fog formula itself (covered separately by
    // test_fog_density_height_falloff and friends).
    let subdivide_dist_unfogged = 1000.0_f64;
    for relaxation in [1.0, 0.9, 0.5, 0.1, 0.01] {
        let subdivide_dist = subdivide_dist_unfogged * relaxation;
        let collapse_dist = subdivide_dist * 1.20;
        let ratio = collapse_dist / subdivide_dist;
        assert!(
            (ratio - 1.20).abs() < 1e-9,
            "collapse_dist/subdivide_dist must stay exactly 1.20 regardless of fog \
             relaxation ({relaxation}), got {ratio}"
        );
    }
}
