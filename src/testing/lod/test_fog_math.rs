//! Pure-function checks on the fog port (`crates/cesium-engine/src/globe/quadtree/fog.rs`)
//! — WP5 of `docs/pre-terrain-plan.md` — plus a direct guard on the constraint that
//! matters most for this package: `Stage::Fog` must never reach the pipeline the
//! culling harness builds.

use cesium_engine::globe::quadtree::{cesium_fog, fog_density_for, CullPipeline, FogConfig, Stage};

/// THE constraint. `CullPipeline::DEFAULT` — what every culling-gate sweep in
/// `src/testing/culling/` runs — must never contain `Stage::Fog`. Fog culling makes
/// false negatives non-zero by design; if this ever fails, every FN = 0 guarantee in
/// `docs/culling-math.md` is void, silently, until the next gate run turns red with
/// no explanation pointing here. A one-line check, but it is the one line that
/// catches "a future reader who 'helpfully' adds the fog stage to the harness's
/// pipeline" before they get as far as running the gate.
#[test]
fn test_fog_stage_is_not_in_the_default_pipeline() {
    assert!(
        !CullPipeline::DEFAULT.stages().contains(&Stage::Fog),
        "Stage::Fog must never be in CullPipeline::DEFAULT — see fog.rs's module doc \
         comment and DEFAULT_WITH_FOG's doc comment for why"
    );
}

/// `DEFAULT_WITH_FOG` is `DEFAULT` plus exactly `Stage::Fog`, appended, not
/// substituted — the "prefix" property `test_stage_prefix_only_grows_the_kept_set`
/// (`src/testing/culling/test_stage_pipeline.rs`) depends on for pipelines it knows
/// about applies here too, informally: production's pipeline is a strict superset
/// of stages, never a swap.
#[test]
fn test_default_with_fog_is_default_plus_fog_appended() {
    let default_stages = CullPipeline::DEFAULT.stages().to_vec();
    let with_fog_stages = CullPipeline::DEFAULT_WITH_FOG.stages().to_vec();
    assert_eq!(with_fog_stages.len(), default_stages.len() + 1);
    assert_eq!(&with_fog_stages[..default_stages.len()], default_stages.as_slice());
    assert_eq!(with_fog_stages[default_stages.len()], Stage::Fog);
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
