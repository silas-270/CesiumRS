//! Coverage for `cesium_engine::label::culling` — the per-point culling used for
//! labels, which is a second, independent implementation of the same ideas the
//! quadtree uses.
//!
//! All three functions there are pure, so they can be pinned against exact f64
//! ground truth with no globe, no GPU and no quadtree.

use cesium_engine::globe::quadtree::Frustum;
use cesium_engine::label::culling::{intersects_sphere, is_behind_horizon, is_in_frustum};
use glam::{DVec3, Vec3};

use super::cameras::{build_camera, ViewParams};
use super::geodesy::{ellipsoid_normal, lon_lat_alt_to_ecef, A, B};
use super::oracle::VisibilityOracle;

fn to_f32(v: DVec3) -> Vec3 {
    Vec3::new(v.x as f32, v.y as f32, v.z as f32)
}

/// Scaled (unit-sphere) camera position and `|cv|² − 1`, the two pre-computed
/// inputs `is_behind_horizon` expects.
fn scaled_camera(cam_pos: DVec3) -> (Vec3, f32) {
    let cv = Vec3::new(
        (cam_pos.x / A) as f32,
        (cam_pos.y / B) as f32,
        (cam_pos.z / A) as f32,
    );
    (cv, cv.length_squared() - 1.0)
}

/// `is_behind_horizon` works in the space where the ellipsoid becomes a unit
/// sphere. That map is linear and invertible, so it preserves occlusion exactly —
/// the engine's test should therefore agree with the exact convexity test
/// `normal · (camera − p) > 0` everywhere except within f32 noise of the limb.
///
/// The two directions of disagreement are not equally serious and are therefore
/// measured separately:
///
/// * **Over-cull** — the engine hides a label that is genuinely visible. That is a
///   missing label on screen: a real bug.
/// * **Under-cull** — the engine keeps a label that is genuinely occluded. That is
///   a label bleeding through the Earth for a fraction of a degree past the limb:
///   cosmetic, and the conservative direction.
///
/// Measured at the time of writing: **48 under-culls out of 651 600 samples,
/// reaching 0.1036° past the limb** (worst case: camera at 30 000 km over the
/// equator, label at 80°S). **Zero over-culls.**
#[test]
fn test_is_behind_horizon_matches_exact_convexity() {
    /// Over-culling — a visible label hidden — is a real bug, and measures zero
    /// today, so this is a regression guard at the oracle's own resolution.
    const MAX_OVER_CULL_DEG: f64 = 0.0;
    /// Under-culling is conservative. 0.25° is ~2.4x the measured 0.1036° — wide
    /// enough not to trip on rounding, narrow enough that a real geometric error
    /// in the scaled-space horizon test would break it. 0.25° of limb angle is
    /// ~28 km of ground distance at Earth's surface.
    const MAX_UNDER_CULL_DEG: f64 = 0.25;

    let mut worst_over_deg = 0.0_f64;
    let mut worst_under_deg = 0.0_f64;
    let mut worst_over_case = String::new();
    let mut worst_under_case = String::new();
    let mut over_culls = 0usize;
    let mut under_culls = 0usize;
    let mut samples = 0usize;

    for cam_alt_m in [1.0, 100.0, 10_000.0, 400_000.0, 5_000_000.0, 30_000_000.0] {
        for cam_lat in [-89.0, -45.0, 0.0, 37.5, 80.0] {
            let cam_pos = lon_lat_alt_to_ecef(11.0, cam_lat, cam_alt_m);
            let (cv, vh_mag_sq) = scaled_camera(cam_pos);

            for lat_i in -90..=90 {
                for lon_i in (-180..180).step_by(3) {
                    let lat = lat_i as f64;
                    let lon = lon_i as f64;
                    let p = lon_lat_alt_to_ecef(lon, lat, 0.0);

                    let to_eye = (cam_pos - p).normalize();
                    let cos_limb = ellipsoid_normal(p).dot(to_eye);
                    let truth_hidden = cos_limb < 0.0;

                    let engine_hidden = is_behind_horizon(cv, vh_mag_sq, to_f32(p));
                    samples += 1;

                    if engine_hidden != truth_hidden {
                        let deg = cos_limb.abs().asin().to_degrees();
                        let ctx = format!(
                            "cam_alt={cam_alt_m}m cam_lat={cam_lat} label=({lat},{lon}) \
                             limb_angle={deg:.6}deg"
                        );
                        if engine_hidden {
                            // Engine hid a genuinely visible label: over-cull.
                            over_culls += 1;
                            if deg > worst_over_deg {
                                worst_over_deg = deg;
                                worst_over_case = ctx;
                            }
                        } else {
                            // Engine kept a genuinely occluded label: under-cull.
                            under_culls += 1;
                            if deg > worst_under_deg {
                                worst_under_deg = deg;
                                worst_under_case = ctx;
                            }
                        }
                    }
                }
            }
        }
    }

    println!("  is_behind_horizon over {samples} samples:");
    println!(
        "    over-cull  (visible label hidden)  : {over_culls} worst {worst_over_deg:.6} deg  {worst_over_case}"
    );
    println!(
        "    under-cull (occluded label kept)   : {under_culls} worst {worst_under_deg:.6} deg  {worst_under_case}"
    );

    assert!(
        worst_over_deg <= MAX_OVER_CULL_DEG,
        "is_behind_horizon hid a label {worst_over_deg:.6} deg inside the visible limb \
         (limit {MAX_OVER_CULL_DEG} deg), {over_culls} times: {worst_over_case}"
    );
    assert!(
        worst_under_deg <= MAX_UNDER_CULL_DEG,
        "is_behind_horizon kept a label {worst_under_deg:.6} deg past the limb \
         (limit {MAX_UNDER_CULL_DEG} deg), {under_culls} times: {worst_under_case}"
    );
}

/// A camera at or below the surface has `vh_mag_sq <= -0.1` disabled and
/// `vh_mag_sq` near zero otherwise; the documented behaviour is "cull nothing".
/// This pins that early-out so a future change to the threshold is noticed.
#[test]
fn test_is_behind_horizon_disabled_below_surface() {
    // 300 km below the surface: the engine's own comment says culling is allowed
    // down to ~300 km, so go well past it.
    let deep = lon_lat_alt_to_ecef(0.0, 0.0, -1_000_000.0);
    let (cv, vh) = scaled_camera(deep);
    assert!(
        vh <= -0.1,
        "expected vh_mag_sq <= -0.1 for a camera 1000 km below the surface, got {vh}"
    );

    // The antipode is unambiguously occluded for any sane camera, yet the
    // early-out must report "not behind the horizon".
    let antipode = lon_lat_alt_to_ecef(180.0, 0.0, 0.0);
    assert!(
        !is_behind_horizon(cv, vh, to_f32(antipode)),
        "the sub-surface early-out should disable horizon culling entirely"
    );
}

/// `intersects_sphere` with radius 0 must be exactly `contains_point`, and must be
/// monotone in radius (a bigger sphere can never be culled when a smaller one at
/// the same centre was not).
#[test]
fn test_intersects_sphere_degenerates_and_is_monotone() {
    let params = ViewParams {
        sweep: "label",
        lat_deg: 20.0,
        lon_deg: -70.0,
        alt_m: 2_000_000.0,
        pitch_deg: 35.0,
        ..Default::default()
    };
    let cam = build_camera(&params);
    let frustum = Frustum::from_planes(cam.calculate_frustum_planes(params.aspect() as f32));
    let oracle = VisibilityOracle::new(&cam, params.aspect());

    let mut checked = 0usize;
    for lat_i in (-90..=90).step_by(5) {
        for lon_i in (-180..180).step_by(5) {
            let p = lon_lat_alt_to_ecef(lon_i as f64, lat_i as f64, 0.0);
            let pf = to_f32(p);

            assert_eq!(
                intersects_sphere(&frustum, pf, 0.0),
                frustum.contains_point(pf),
                "radius-0 sphere disagreed with contains_point at ({lat_i},{lon_i})"
            );
            assert_eq!(
                is_in_frustum(&frustum, pf),
                frustum.contains_point(pf),
                "is_in_frustum is not contains_point at ({lat_i},{lon_i})"
            );

            let mut prev = intersects_sphere(&frustum, pf, 0.0);
            for r in [0.001_f32, 0.01, 0.1, 1.0, 10.0] {
                let now = intersects_sphere(&frustum, pf, r);
                assert!(
                    now || !prev,
                    "intersects_sphere is not monotone in radius at ({lat_i},{lon_i}), r={r}"
                );
                prev = now;
            }

            // Anything the point test accepts must also be inside the oracle's
            // clip box, modulo the marginal band. (Frustum planes are derived
            // from the same view-projection the oracle projects with, so this is
            // a pure f32-vs-f64 consistency check.)
            if frustum.contains_point(pf) {
                if let Some(ndc) = oracle.ndc(p) {
                    let slack = 1.0 + 1.0e-3;
                    assert!(
                        ndc.x.abs() <= slack && ndc.y.abs() <= slack,
                        "contains_point accepted a point at ndc=({:.6},{:.6}) at ({lat_i},{lon_i})",
                        ndc.x,
                        ndc.y
                    );
                }
            }
            checked += 1;
        }
    }
    println!("  intersects_sphere / contains_point: {checked} positions checked");
}
