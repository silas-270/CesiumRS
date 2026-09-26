//! Harness self-checks, and the measurement run itself.
//!
//! Per `docs/pre-terrain-plan.md` WP1, this module **measures and does not assert**
//! a target yet — there is no calibrated `target_texel_ratio` until WP4. What it
//! does assert are properties of the *instrument*: that sampling is faithful to the
//! shared tile-bounds definition, and that the measurement run itself behaves (no
//! NaNs leaking into aggregates, every counted tile lands in exactly one bucket).

use cesium_engine::globe::quadtree::{
    lod_factor_for, tile_bounds, tile_bounds_unstretched, QuadtreeNode, TileId,
};
use glam::{DMat4, DVec3};

use super::report;
use super::sweep::{
    self, bench_poses, measure_poses, measure_poses_with_config, measure_poses_with_target,
    patch_grid_points, project_patch, LodConfig, ESRI_TEXTURE_SIZE_PX, TEXTURE_SIZE_PX,
};

fn to_dvec3(p: [f64; 3]) -> DVec3 {
    DVec3::new(p[0], p[1], p[2])
}

/// `patch_grid_points`'s four corners (`u, v ∈ {0, 1}`) must land exactly on the
/// same rectangle `tile_bounds` defines — the shared I-5 source the drawn mesh, the
/// culling rectangle and this harness are all required to agree on.
#[test]
fn test_patch_grid_corners_match_tile_bounds() {
    let ids = [
        TileId { z: 0, x: 0, y: 0 },
        TileId { z: 1, x: 0, y: 0 }, // touches the north pole cap
        TileId { z: 1, x: 1, y: 1 }, // touches the south pole cap
        TileId { z: 5, x: 10, y: 12 },
        TileId { z: 12, x: 2000, y: 1500 },
        TileId { z: 20, x: 3, y: 3 },
    ];

    for id in ids {
        let bounds = tile_bounds(&id);
        let grid = patch_grid_points(&id, &bounds);

        let expect = |lon: f64, lat: f64| to_dvec3(cesium_engine::globe::geometry::lon_lat_to_ecef_f64(lon, lat));

        assert_eq!(grid[0][0], expect(bounds.lon_min, bounds.lat_max), "NW corner, {id:?}");
        assert_eq!(grid[0][2], expect(bounds.lon_max, bounds.lat_max), "NE corner, {id:?}");
        assert_eq!(grid[2][0], expect(bounds.lon_min, bounds.lat_min), "SW corner, {id:?}");
        assert_eq!(grid[2][2], expect(bounds.lon_max, bounds.lat_min), "SE corner, {id:?}");

        // The centre sample must be strictly between the two latitude extremes
        // (or exactly the pole itself for a polar tile's opposite-of-the-cap row),
        // and it must not silently collapse onto an edge.
        let center = grid[1][1];
        assert!(center != grid[0][1] && center != grid[2][1], "{id:?}: centre row collapsed");
    }
}

/// Every visible tile lands in exactly one of "has screen area" / "offscreen or
/// clipped away entirely", and every ratio that enters an aggregate is finite and
/// positive. A harness that let a NaN or an infinity leak into `Summary::mean()`
/// would make every later WP3/WP4 comparison meaningless.
#[test]
fn test_lod_sweep_produces_sane_aggregates() {
    let poses = bench_poses();
    assert_eq!(poses.len(), 204, "expected the same 204 bench poses bench_update uses");

    let results = measure_poses(&poses);
    assert_eq!(results.len(), poses.len());

    let mut total_tiles = 0usize;
    for r in &results {
        assert_eq!(r.degenerate, r.tile_count == 0, "degenerate must track tile_count == 0");
        total_tiles += r.tile_count;
        assert_eq!(
            r.texture_bytes,
            r.tile_count as u64 * (r.texture_size_px as u64).pow(2) * sweep::BYTES_PER_TEXEL,
            "texture_bytes must be tile_count x texel area x bytes-per-texel"
        );
        for t in &r.tiles {
            assert!(t.screen_px >= 0.0 && t.screen_px.is_finite(), "{:?}: screen_px must be finite and non-negative", t.id);
            if t.has_screen_area() {
                assert!(t.ratio.is_finite() && t.ratio > 0.0, "{:?}: a tile with screen area must have a finite positive ratio", t.id);
            } else {
                assert!(t.ratio.is_infinite(), "{:?}: a tile with no screen area must report ratio = inf, not silently drop out", t.id);
            }
        }
    }
    assert!(total_tiles > 0, "the nadir/zoom-cliff bench poses must produce visible tiles");

    let text = report::emit("bench_poses", &results);

    let s = report::summarize(&results);
    assert!(s.mean().is_finite(), "mean ratio must be finite when any tile was sampled:\n{text}");
    assert!(s.sampled_tiles() > 0);
    assert_eq!(
        s.offscreen_tiles + s.sampled_tiles(),
        total_tiles,
        "every tile must be counted exactly once, in sampled or offscreen:\n{text}"
    );
}

/// Regression guard on the area-weighted `texels` fix: a tile whose patch is
/// entirely onscreen, entirely unclipped, and entirely in front of the eye must
/// still report the full `TEXTURE_SIZE_PX²` texel count — the area-weighting only
/// ever discounts a quad, never inflates it, so the base case (today's flat
/// behaviour) has to fall out of the general formula exactly.
///
/// Built directly against `project_patch` with a synthetic flat 3×3 grid and a
/// hand-built orthographic view-projection (`w = 1` for every point, by
/// construction) rather than a real camera/tile, so the test exercises the
/// area-weighting formula in isolation instead of real tile-bounds/camera
/// machinery that could hide a broken formula behind an unrelated pass.
#[test]
fn test_project_patch_fully_onscreen_quad_gets_full_texel_count() {
    // A flat 3x3 grid spanning [-1, 1] x [-1, 1] at z = 0.
    let samples: [[DVec3; 3]; 3] = [
        [DVec3::new(-1.0, 1.0, 0.0), DVec3::new(0.0, 1.0, 0.0), DVec3::new(1.0, 1.0, 0.0)],
        [DVec3::new(-1.0, 0.0, 0.0), DVec3::new(0.0, 0.0, 0.0), DVec3::new(1.0, 0.0, 0.0)],
        [DVec3::new(-1.0, -1.0, 0.0), DVec3::new(0.0, -1.0, 0.0), DVec3::new(1.0, -1.0, 0.0)],
    ];

    // Orthographic: w = 1 identically (the last row of the matrix is (0,0,0,1)),
    // so there is no behind-eye sample by construction. The [-4, 4] extent maps
    // the grid's [-1, 1] span to NDC [-0.25, 0.25] — comfortably inside [-1, 1]
    // with margin, so nothing clips against the viewport either.
    let vp = DMat4::orthographic_rh(-4.0, 4.0, -4.0, 4.0, -1.0, 1.0);
    let width = 800.0_f64;
    let height = 600.0_f64;

    let id = TileId { z: 3, x: 1, y: 1 };
    let metric = project_patch(id, &samples, &vp, width, height, TEXTURE_SIZE_PX as f64);

    assert!(!metric.behind_eye, "synthetic grid must not report any behind-eye sample");
    assert!(!metric.partly_offscreen, "synthetic grid must not be clipped at all");
    assert!(metric.screen_px > 0.0, "synthetic grid must project to nonzero area");

    let texels_full = (TEXTURE_SIZE_PX as f64) * (TEXTURE_SIZE_PX as f64);
    assert!(
        (metric.texels - texels_full).abs() < 1e-6,
        "a fully onscreen, unclipped, in-front-of-eye tile must report the full texel \
         count: got {}, expected {}",
        metric.texels,
        texels_full,
    );
}

/// `target_texel_ratio` must be a *multiplier* on `lod_factor`, not a divisor — the
/// docs on `lod_factor_for` and on `TileEngineConfig::target_texel_ratio` both say
/// "higher = more texels demanded per pixel = sharper", so higher must mean a
/// *larger* `lod_factor` (refine out to a larger distance), not a smaller one. And
/// since `target_texel_ratio` is an *area* ratio (`texels / screen_px`, both areas)
/// while `lod_factor` scales a *linear* distance, the relationship must be a square
/// root, not the first power. Both bugs were invisible at `target = 1.0`, where
/// `sqrt(1) == 1 == 1/1` — so this checks two other target values, at two different
/// viewport/texture-size/focal-length configurations, so no single config's
/// coincidence can hide either bug.
#[test]
fn test_lod_factor_scales_with_sqrt_target_not_inverse_linear() {
    let configs: [(f32, f32, f32); 2] = [
        // (texture_size_px, viewport_height_px, fovy_rad) — the engine default.
        (512.0, 1080.0, 2.0 * (24.0_f32 / 56.0).atan()),
        // A different texture size, viewport height and focal length.
        (256.0, 720.0, 2.0 * (18.0_f32 / 40.0).atan()),
    ];

    for (texture_size, height, fovy) in configs {
        let base = lod_factor_for(1.0, texture_size, height, fovy);
        for target in [2.0_f32, 4.0, 9.0] {
            let scaled = lod_factor_for(target, texture_size, height, fovy);
            assert!(
                scaled > base,
                "target={target} texture={texture_size} height={height}: lod_factor must \
                 increase with target_texel_ratio (more texels/px demanded => sharper => \
                 refine further out), got base={base} scaled={scaled}"
            );
            let expected = base * target.sqrt();
            let rel_err = (scaled - expected).abs() / expected;
            assert!(
                rel_err < 1e-5,
                "target={target} texture={texture_size} height={height}: expected \
                 lod_factor = base * sqrt(target) = {expected}, got {scaled} (rel err {rel_err})"
            );
        }
    }
}

/// WP4/A's premise (`docs/pre-terrain-plan.md`): a smaller decoded texture size must
/// raise `lod_factor` proportionally — a 256px style needs the engine to refine
/// further out than a 512px one to hold the same target texel/pixel ratio, since
/// each texel then covers twice the ground per side at a given zoom level. This was
/// already correct in `lod_factor_for` (only `target_texel_ratio`'s direction and
/// exponent were bugs last round); this test exists because WP4/A is the first thing
/// that actually *feeds* a non-default `texture_size_px` into it, so the relationship
/// deserves its own explicit check rather than trusting it was never exercised.
/// Checked at a second target and a second viewport/focal length, same rationale as
/// the sibling `sqrt(target)` test above.
#[test]
fn test_lod_factor_scales_inversely_with_texture_size() {
    let configs: [(f32, f32, f32); 2] = [
        (1.0, 1080.0, 2.0 * (24.0_f32 / 56.0).atan()),
        (4.0, 720.0, 2.0 * (18.0_f32 / 40.0).atan()),
    ];

    for (target, height, fovy) in configs {
        let at_512 = lod_factor_for(target, 512.0, height, fovy);
        let at_256 = lod_factor_for(target, 256.0, height, fovy);
        let at_1024 = lod_factor_for(target, 1024.0, height, fovy);

        assert!(
            (at_256 - 2.0 * at_512).abs() / at_512 < 1e-5,
            "target={target} height={height}: halving texture_size (512 -> 256) must \
             double lod_factor, got at_512={at_512} at_256={at_256}"
        );
        assert!(
            (at_1024 - 0.5 * at_512).abs() / at_512 < 1e-5,
            "target={target} height={height}: doubling texture_size (512 -> 1024) must \
             halve lod_factor, got at_512={at_512} at_1024={at_1024}"
        );
    }
}

/// Same fix, checked end-to-end through the real quadtree and this harness's own
/// `texels / screen_px` metric rather than through the isolated function —
/// `test_lod_factor_scales_with_sqrt_target_not_inverse_linear` proves
/// `lod_factor_for` itself is right, this proves nothing between it and the measured
/// ratio (the subdivide/collapse comparisons, hysteresis, tile counting) silently
/// cancels or inverts the relationship.
///
/// Doubling `lod_factor` (target=4 vs target=1, since `lod_factor` scales as
/// `sqrt(target)`) pushes the subdivision boundary out to roughly twice the
/// distance — roughly one more quadtree level for the tiles it moves — and one more
/// level doubles the linear texel/pixel resolution, hence *quadruples* the area
/// ratio this harness reports. `docs/pre-terrain-plan.md`'s WP3 follow-up asks for
/// exactly this: "set target to 4.0 ... confirm the measured aggregate_ratio moves
/// to ~4x its target=1.0 value." The tolerance is wide because a discrete quadtree
/// cannot land exactly on 4x — a wrong-direction or wrong-exponent bug would miss it
/// by far more than this band, landing near 1x (no-op direction bug) or ~2x/16x
/// (linear instead of sqrt).
#[test]
fn test_lod_harness_aggregate_ratio_scales_with_target() {
    let poses = bench_poses();

    let baseline = measure_poses_with_target(&poses, 1.0);
    let scaled = measure_poses_with_target(&poses, 4.0);

    let base_ratio = report::summarize(&baseline).aggregate_ratio;
    let scaled_ratio = report::summarize(&scaled).aggregate_ratio;
    let factor = scaled_ratio / base_ratio;

    assert!(
        (3.0..=5.5).contains(&factor),
        "target_texel_ratio=4.0 must move aggregate_ratio to ~4x its target=1.0 value: \
         base={base_ratio:.4} scaled={scaled_ratio:.4} factor={factor:.4}"
    );
}

/// Guards the naming fix on `LOD_CALIBRATION_CONSTANT` (formerly `GROUND_PER_RADIUS`,
/// `quadtree.rs`): the true geometric ratio the old name claimed — a tile's ground
/// width (the west-to-east chord at its centre latitude) over
/// `QuadtreeNode::unstretched_radius` — is a real, level-independent quantity near
/// `sqrt(2)` (~1.415), which is *not* the `256/315` (~0.8127) the constant is
/// actually set to (a calibration residual, not this geometric ratio). This pins the
/// geometric quantity down directly so the doc comment's claim can't silently drift
/// from the tile geometry it describes; it does not by itself guard against someone
/// swapping the constant back to the geometric value without re-deriving
/// `target_texel_ratio`'s default (that would still need the no-op / byte-identical
/// CSV check to catch).
///
/// Sampled at z = 8, 11, 14, 18 rather than from the root: a flat 3×3-corner sample
/// (what `unstretched_radius` and this chord both are) under-states a coarse tile's
/// true curved extent, exactly the effect the WP1 harness's own doc comment measures
/// for its patch grid — at z ≤ 5 this ratio reads measurably below `sqrt(2)` for that
/// reason, not because the quantity stops being level-independent. z ≥ 8 is where it
/// has converged to a stable value, which is the regime this test pins down.
#[test]
fn test_true_ground_per_radius_is_not_the_calibration_constant() {
    use cesium_engine::globe::geometry::lon_lat_to_ecef_f64;

    const CALIBRATION_CONSTANT: f64 = 256.0 / 315.0;

    // Four widely separated zoom levels (z >= 8, see doc comment above), away from
    // poles/equator/meridian degeneracies, mirroring
    // `reorder_children_tests::make_node`'s choice.
    let ids = [
        TileId { z: 8, x: 130, y: 90 },
        TileId { z: 11, x: 1000, y: 700 },
        TileId { z: 14, x: 8500, y: 6000 },
        TileId { z: 18, x: 130000, y: 95000 },
    ];

    let mut ratios = Vec::new();
    for id in ids {
        let unstretched_radius = QuadtreeNode::new(id).unstretched_radius as f64;
        let b = tile_bounds_unstretched(&id);
        let lat = b.center_lat();
        let west = to_dvec3(lon_lat_to_ecef_f64(b.lon_min, lat));
        let east = to_dvec3(lon_lat_to_ecef_f64(b.lon_max, lat));
        let ground_width = (east - west).length();
        let ratio = ground_width / unstretched_radius;

        assert!(
            (1.40..1.43).contains(&ratio),
            "{id:?}: ground_width/unstretched_radius = {ratio:.4}, expected ~sqrt(2) (~1.41-1.42)"
        );
        ratios.push(ratio);
    }

    for pair in ratios.windows(2) {
        let spread = (pair[0] - pair[1]).abs();
        assert!(
            spread < 0.01,
            "the true ground_per_radius ratio should be level-independent for z >= 8, \
             got {ratios:?} (spread {spread:.4})"
        );
    }

    for ratio in &ratios {
        let rel_diff = (ratio - CALIBRATION_CONSTANT).abs() / CALIBRATION_CONSTANT;
        assert!(
            rel_diff > 0.5,
            "the true geometric ground_per_radius ({ratio:.4}) should be far from \
             LOD_CALIBRATION_CONSTANT ({CALIBRATION_CONSTANT:.4}) — they are not the \
             same quantity, which is exactly why the constant was renamed off its old \
             geometric name"
        );
    }
}

/// WP4/A (`docs/pre-terrain-plan.md`): measures `satellite_imagery_url()`'s 256px
/// style two ways and records both, per that package's instructions.
///
/// **Compensated** (`LodConfig::new(1.0, 256.0)`) is what the engine actually does
/// now: `lod_factor_for` is told the real 256px size, so it refines further out to
/// compensate — this is a genuinely different quadtree run from the 512px baseline,
/// not a rescaling of it.
///
/// **Uncompensated** (`LodConfig::uncompensated(1.0, 512.0, 256.0)`) reproduces the
/// pre-WP4/A bug for comparison: `lod_factor_for` still told 512px (so *exactly* the
/// 512px baseline's `lod_factor`, hence *exactly* its tile set and `screen_px` per
/// tile — nothing about subdivision depends on the texel-counting size), while the
/// harness counts real 256px texels. Because the tile set and every `screen_px` are
/// therefore identical to the 512px baseline, `texels` and hence `aggregate_ratio`
/// must scale by *exactly* `(256/512)² = 0.25` — a provable-by-construction relation,
/// not a measured coincidence, so this is checked to a tight tolerance rather than a
/// wide band like the other WP4 checks.
#[test]
fn test_wp4a_esri_texture_size_compensated_vs_uncompensated() {
    let poses = bench_poses();

    let baseline_512 = measure_poses(&poses);
    let compensated_256 =
        measure_poses_with_config(&poses, LodConfig::new(1.0, ESRI_TEXTURE_SIZE_PX));
    let uncompensated_256 = measure_poses_with_config(
        &poses,
        LodConfig::uncompensated(1.0, TEXTURE_SIZE_PX as f32, ESRI_TEXTURE_SIZE_PX),
    );

    let baseline_ratio = report::summarize(&baseline_512).aggregate_ratio;
    let compensated_ratio = report::summarize(&compensated_256).aggregate_ratio;
    let uncompensated_ratio = report::summarize(&uncompensated_256).aggregate_ratio;

    // Exact by construction: same tile set and screen_px as the 512px baseline,
    // texels scaled by (256/512)^2.
    let expected_uncompensated = baseline_ratio * 0.25;
    let rel_err = (uncompensated_ratio - expected_uncompensated).abs() / expected_uncompensated;
    assert!(
        rel_err < 1e-6,
        "uncompensated aggregate_ratio should be exactly 0.25x the 512px baseline: \
         baseline={baseline_ratio:.4} uncompensated={uncompensated_ratio:.4} \
         expected={expected_uncompensated:.4} (rel err {rel_err})"
    );

    // Not asserted tightly (this is a real, independent quadtree run, not a rescaling)
    // beyond: compensation must land far closer to the intended ~1x-of-baseline than
    // the uncompensated 0.25x does, and it must not overshoot into a similarly
    // one-sided miss the other way — i.e. it should look like Carto's own
    // distribution, not like a different bug.
    let compensated_vs_baseline = compensated_ratio / baseline_ratio;
    assert!(
        (0.5..2.0).contains(&compensated_vs_baseline),
        "compensated 256px aggregate_ratio should land close to the 512px baseline's \
         own order of magnitude, not systematically low like the uncompensated 0.25x \
         case: baseline={baseline_ratio:.4} compensated={compensated_ratio:.4} \
         ratio={compensated_vs_baseline:.4}"
    );

    eprintln!(
        "WP4/A: baseline(512)={baseline_ratio:.4}  compensated(256)={compensated_ratio:.4}  \
         uncompensated(256)={uncompensated_ratio:.4}"
    );
    report::emit("wp4a_esri_compensated_256", &compensated_256);
    report::emit("wp4a_esri_uncompensated_256", &uncompensated_256);
}

/// **E1d** — the sentence this module's doc comment used to make in prose, as a value.
///
/// `src/testing/lod/mod.rs` justified `texels / screen_px` as a complete description of
/// LOD quality *because* with zero relief the only per-tile error is imagery resolution.
/// E1 gave the engine a second error term, so the justification needs checking rather than
/// restating: over all 204 bench poses, every tile's projected geometric error must be
/// **exactly** zero — not small, zero — because `Ellipsoid::HAS_GEOMETRIC_ERROR` is a
/// compile-time `false` and `apply_lod` never reaches the term at all on this tree.
///
/// If this ever reads non-zero, one of two things has happened and both matter: the flat
/// globe has acquired relief (it must not), or this harness has stopped measuring the flat
/// globe (in which case `docs/culling-baseline.md`'s cross-package comparisons are no
/// longer like for like).
#[test]
fn the_flat_globe_leaves_no_geometric_error_on_screen() {
    let poses = bench_poses();
    let results = measure_poses(&poses);
    let mut tiles = 0usize;
    for r in &results {
        for t in &r.tiles {
            tiles += 1;
            assert_eq!(
                t.geom_err_px, 0.0,
                "{:?} reported {} px of geometric error on a globe that has none",
                t.id, t.geom_err_px
            );
        }
    }
    assert!(
        tiles > 0,
        "the bench poses must produce visible tiles to check"
    );

    // And the metric is not vacuous — the same function, handed a real error and a real
    // distance, produces the number `TerrainConfig::max_geometric_error_px` budgets. A
    // 300 m error 100 km away at 1080 px and the engine's default fovy is ~3.8 px.
    let fovy = 2.0 * (3.0f64 / 7.0).atan();
    let px = sweep::geometric_error_px(300.0e-6, 0.100, 1080.0, fovy);
    assert!(
        (px - 3.78).abs() < 0.05,
        "the second metric must actually measure something: got {px}"
    );
}
