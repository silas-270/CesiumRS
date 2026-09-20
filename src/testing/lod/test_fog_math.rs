//! Pure-function checks on the fog port (`crates/cesium-engine/src/globe/quadtree/fog.rs`)
//! — WP5 of `docs/pre-terrain-plan.md` — plus a guard on what is left of fog's reach into
//! the tree.
//!
//! There used to be two guards here on `Stage::Fog`'s placement: that it never entered
//! `CullPipeline::DEFAULT` (where it would void every FN = 0 guarantee in
//! `docs/culling-math.md`), and that `DEFAULT_WITH_FOG` was `DEFAULT` plus exactly that
//! stage. **E1c deleted the stage**, having measured that it culls nothing at any camera
//! (`docs/terrain-plan.md` §8, and
//! [`super::test_wp5_fog::the_fog_stage_never_ran_and_this_is_what_it_would_have_culled`]
//! for the reconstruction that still checks it). With no stage there is nothing to keep
//! out of `DEFAULT`, and production now runs `DEFAULT` itself — so the constraint those
//! two tests protected is satisfied structurally rather than by assertion, which is the
//! stronger arrangement and the reason they are gone rather than rewritten.

use cesium_engine::globe::quadtree::{cesium_fog, fog_density_for, CullPipeline, FogConfig, Stage};

/// What is left of the constraint: production's pipeline **is** the pipeline the culling
/// gate proves FN = 0 against, on both arms.
///
/// Before E1c the two differed by `Stage::Fog` and a paragraph explaining why that was
/// safe. Now they are the same constant, and this is the check that keeps them so — the
/// failure it catches is someone reintroducing an unsound stage into the pipeline
/// `wgpu_state` selects without noticing that the harness measures a different one.
#[test]
fn test_production_runs_the_pipeline_the_gate_proves() {
    for stage in CullPipeline::DEFAULT.stages() {
        assert!(
            matches!(
                stage,
                Stage::Horizon | Stage::NodeFrustum | Stage::SubPatchGrid
            ),
            "CullPipeline::DEFAULT grew an unexpected stage: {stage:?} — if it is not \
             provably sound at every level (invariant I-7), it does not belong in the \
             pipeline the culling gate measures"
        );
    }
    let terrain = CullPipeline::TERRAIN_DEFAULT.stages().to_vec();
    assert_eq!(
        terrain.len(),
        CullPipeline::DEFAULT.stages().len() + 1,
        "TERRAIN_DEFAULT is DEFAULT plus Stage::TerrainOcclusion and nothing else"
    );
    assert!(terrain.contains(&Stage::TerrainOcclusion));
}

/// `cesium_fog`'s boundary behaviour: zero at zero distance or zero density
/// (matches `scalar = 0 => 1 - exp(0) = 0`), and saturating toward `1.0` as either
/// grows — checked at values far from `0`, not just at it.
#[test]
fn test_cesium_fog_boundaries_and_monotonicity() {
    assert_eq!(cesium_fog(0.0, 0.0006), 0.0);
    assert_eq!(cesium_fog(10_000.0, 0.0), 0.0);

    // Monotonic increasing in distance, at fixed density, checked at three
    // well-separated points rather than just two.
    let density = 0.0006_f32;
    let f1 = cesium_fog(1000.0, density);
    let f2 = cesium_fog(5000.0, density);
    let f3 = cesium_fog(20_000.0, density);
    assert!(f1 < f2 && f2 < f3, "fog must increase monotonically with distance: {f1} {f2} {f3}");

    // Monotonic increasing in density, at fixed distance.
    let dist = 5000.0_f32;
    let g1 = cesium_fog(dist, 0.0002);
    let g2 = cesium_fog(dist, 0.0006);
    let g3 = cesium_fog(dist, 0.0020);
    assert!(g1 < g2 && g2 < g3, "fog must increase monotonically with density: {g1} {g2} {g3}");

    // Saturates toward 1.0, never exceeds it (it is 1 - exp(-x), x >= 0).
    let saturated = cesium_fog(1_000_000.0, 0.0006);
    assert!(
        (0.999_999..=1.0).contains(&saturated),
        "fog must saturate to ~1.0 at extreme distance*density, got {saturated}"
    );
    assert!(saturated <= 1.0);
}

/// `cesium_fog` at a hand-computable point: `distance=1000, density=0.001` gives
/// `scalar = 1.0` exactly, so `fog = 1 - exp(-1) = 0.6321205588...` — checked to
/// f32 precision, away from the `distance=0` / `density=0` trivial points above.
#[test]
fn test_cesium_fog_matches_hand_computed_value_at_scalar_one() {
    let fog = cesium_fog(1000.0, 0.001);
    let expected = 1.0 - std::f32::consts::E.recip();
    assert!(
        (fog - expected).abs() < 1e-6,
        "expected 1 - 1/e = {expected}, got {fog}"
    );
}

/// `fog_density_for`: disabled ("fog off above maxHeight") for any altitude past
/// `max_height_m`, exactly `base * height_scalar` *at* `max_height_m` (the ratio is
/// exactly 1 there, so the falloff term is `1^-falloff = 1`), and strictly
/// increasing as altitude falls below it — checked well below the boundary, not
/// just adjacent to it, so a sign error in the exponent would be caught.
#[test]
fn test_fog_density_height_falloff() {
    let cfg = FogConfig::default();

    assert_eq!(fog_density_for(cfg.max_height_m + 1.0, &cfg), 0.0);
    assert_eq!(fog_density_for(cfg.max_height_m * 10.0, &cfg), 0.0);

    let at_max = fog_density_for(cfg.max_height_m, &cfg);
    let expected_at_max = cfg.density * cfg.height_scalar;
    assert!(
        (at_max - expected_at_max).abs() / expected_at_max < 1e-5,
        "at height == max_height, ratio == 1 so density should be base*height_scalar \
         exactly: expected {expected_at_max}, got {at_max}"
    );

    let at_cruise = fog_density_for(10_000.0, &cfg); // ~10km, this product's cruise altitude
    let at_low = fog_density_for(1_000.0, &cfg);
    let at_ground = fog_density_for(1.0, &cfg);
    assert!(
        at_max < at_cruise && at_cruise < at_low && at_low < at_ground,
        "density must strictly increase as altitude falls: max={at_max} cruise={at_cruise} \
         low={at_low} ground={at_ground}"
    );
}

/// The `EPSILON4` floor: without it, `(height/max_height)^-falloff` diverges as
/// `height -> 0`. Confirms the floor actually engages and density stays finite
/// (and does not, say, silently become `inf` or `NaN`) at and below the clamp.
#[test]
fn test_fog_density_epsilon4_floor_prevents_blowup() {
    let cfg = FogConfig::default();

    let at_zero = fog_density_for(0.0, &cfg);
    let at_negative = fog_density_for(-100.0, &cfg); // below the ellipsoid; must not panic or NaN
    assert!(at_zero.is_finite() && at_zero > 0.0, "density at height=0 must be finite and positive, got {at_zero}");
    assert!(at_negative.is_finite() && at_negative > 0.0, "density below the ellipsoid must still be finite, got {at_negative}");

    // Both should be clamped to the same EPSILON4 floor, hence (nearly) equal —
    // anything below the floor collapses to the same ratio.
    assert!(
        (at_zero - at_negative).abs() / at_zero < 1e-4,
        "heights at or below the EPSILON4 floor should give the same density: \
         at_zero={at_zero} at_negative={at_negative}"
    );
}
