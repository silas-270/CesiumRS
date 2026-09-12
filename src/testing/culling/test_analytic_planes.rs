//! # Layer 1 — analytic frustum/OBB tests, no globe involved.
//!
//! Synthetic [`OrientedBoundingBox`] values are placed at controlled offsets
//! relative to each individual frustum plane, and the engine's
//! `Frustum::intersects_obb` is compared against ground truth computed here in
//! f64 — either the exact plane arithmetic (for the documented contract) or full
//! separating-axis intersection over the frustum's convex hull (for the cases
//! where the two disagree by design).
//!
//! Plane order is `[Left, Right, Bottom, Top]`. There are **four**: under the
//! engine's reverse-Z, `z_ndc ∈ [0,1]` projection the near and far constraints are
//! `r3 − r2` and `r2`, and tile culling drops both as provably vacuous for the
//! globe — see `Camera::calculate_frustum_planes` and `docs/culling-math.md` §2.6.
//!
//! The engine works in the **camera-relative** frame: all four side planes pass
//! through the eye, so their offset is identically zero and only the normal is
//! stored. This file reconstructs the world-space offset `d = −n·eye` in f64 so the
//! probe geometry below can still be placed against an absolute plane.

use cesium_engine::camera::camera::{Camera, CameraMode};
use cesium_engine::globe::quadtree::{Frustum, OrientedBoundingBox, QuadtreeNode, TileId};
use glam::{DVec3, Quat, Vec3};

use super::sat::{obb_hull, Hull};

pub const PLANE_NAMES: [&str; 4] = ["Left", "Right", "Bottom", "Top"];

/// Half-extent of the probe box, in megameters (50 km).
///
/// Small enough to sit comfortably inside the reference frustum's narrowest
/// cross-section (the near face is ~1.16 × 2.08 Mm), large enough that its
/// projected radius dwarfs f32 representation error at this scale.
const PROBE_HALF_EXTENT: f64 = 0.05;

/// The decision band, in megameters (1 km).
///
/// The engine culls in f32 after `Frustum::from_planes` downcasts. Plane distances
/// here are O(1..30) megameters, whose f32 error is O(2e-6) megameters. 1 km is
/// ~500× that, so an assertion outside this band is testing culling logic, not
/// rounding. Cases *inside* the band are reported, never asserted.
const DECISION_EPS: f64 = 1.0e-3;

/// A well-conditioned reference frustum: Free mode, ~14 Mm up, deliberately
/// **oblique** — off-axis latitude/longitude plus pitch, yaw and roll.
///
/// The obliquity is load-bearing. An axis-aligned camera (say, at `(0, 0, 20)`
/// looking at the origin) produces a frustum that is mirror-symmetric in y and z,
/// so whole classes of sign errors — flipping the y component of every plane
/// normal, for instance — map the plane *set* onto itself and are invisible to any
/// test built on it. A generic pose has no such symmetry, so a sign or axis error
/// anywhere in the plane pipeline changes the answer.
///
/// Free mode is chosen deliberately — `Camera::new` defaults to `Tracking`, whose
/// znear collapses to 1e-8..5e-6 megameters and produces a frustum so sliver-thin
/// that f64 unprojection of its near corners is itself ill-conditioned. Tracking
/// and Cockpit frusta are exercised by the Layer 2 sweep instead.
fn reference_camera() -> Camera {
    super::cameras::build_camera(&reference_params())
}

fn reference_params() -> super::cameras::ViewParams {
    super::cameras::ViewParams {
        sweep: "analytic-reference",
        lat_deg: 23.0,
        lon_deg: 41.0,
        alt_m: 13_600_000.0,
        pitch_deg: 35.0,
        yaw_deg: 57.0,
        roll_deg: 22.0,
        width: 1920,
        height: 1080,
        mode: CameraMode::Free,
    }
}

const ASPECT: f64 = 16.0 / 9.0;

struct Reference {
    /// `(unit normal, offset)` in **world** space, with `d = −n·eye` computed in
    /// f64. The engine stores only the normal; this is the same plane, re-expressed
    /// absolutely so probe boxes can be positioned against it.
    planes_f64: [(DVec3, f64); 4],
    frustum: Frustum,
    hull: Hull,
    eye: DVec3,
    /// A point comfortably inside the frustum: the centroid of its eight corners.
    centroid: DVec3,
}

fn reference() -> Reference {
    let cam = reference_camera();
    let normals = cam.calculate_frustum_planes(ASPECT as f32);
    let (eye, _) = cam.global_transform_f64();
    let frustum = Frustum::new(normals, eye).with_corners(cam.frustum_corners_relative(ASPECT as f32));
    let mut planes_f64 = [(DVec3::ZERO, 0.0); 4];
    for i in 0..4 {
        planes_f64[i] = (normals[i], -normals[i].dot(eye));
    }
    let view_proj = cam.get_projection_matrix_f64(ASPECT) * cam.get_view_matrix_f64();
    let hull = Hull::from_view_proj(view_proj);
    let centroid = hull.points.iter().copied().sum::<DVec3>() / hull.points.len() as f64;
    Reference {
        planes_f64,
        frustum,
        hull,
        eye,
        centroid,
    }
}

/// The probe box's three half-axes, deliberately *not* world-aligned so the test
/// exercises a genuine oriented box rather than an AABB.
fn probe_half_axes(scale: f64) -> [DVec3; 3] {
    let q = Quat::from_euler(glam::EulerRot::YXZ, 0.7, -0.4, 1.1);
    let to_d = |v: Vec3| DVec3::new(v.x as f64, v.y as f64, v.z as f64);
    [
        to_d(q * Vec3::X) * scale,
        to_d(q * Vec3::Y) * (scale * 0.6),
        to_d(q * Vec3::Z) * (scale * 1.4),
    ]
}

fn to_obb(center: DVec3, half_axes: [DVec3; 3]) -> OrientedBoundingBox {
    let f = |v: DVec3| Vec3::new(v.x as f32, v.y as f32, v.z as f32);
    // The centre stays f64 (invariant I-2): it is what makes `centre − eye` an f64
    // subtraction, which is the whole point of the camera-relative frame.
    OrientedBoundingBox::new(center, [f(half_axes[0]), f(half_axes[1]), f(half_axes[2])])
}

/// Projected radius of the box onto `n` — the same quantity the engine computes,
/// but in f64.
fn projected_radius(n: DVec3, half_axes: &[DVec3; 3]) -> f64 {
    n.dot(half_axes[0]).abs() + n.dot(half_axes[1]).abs() + n.dot(half_axes[2]).abs()
}

/// How close the frustum's own hull gets to a plane.
///
/// A plane that genuinely bounds the frustum *supports a face of it*, so at least
/// four hull corners lie on it and this distance is ~0. A plane that never touches
/// the hull is redundant: it is a constraint the frustum already satisfies with
/// room to spare, and it can never reject anything the other five accept.
fn hull_clearance_to_plane(hull: &Hull, (n, d): (DVec3, f64)) -> f64 {
    hull.points
        .iter()
        .map(|p| n.dot(*p) + d)
        .fold(f64::INFINITY, f64::min)
}

/// Planes whose clearance exceeds this (in megameters) are reported as redundant.
/// Real supporting planes come out at ~1e-12; the degenerate one is off by Mm.
const REDUNDANT_PLANE_CLEARANCE: f64 = 1.0e-3;

// ─────────────────────────────────────────────────────────────────────────────
// Test 1: per-plane offsets. MUST PASS.
// ─────────────────────────────────────────────────────────────────────────────

/// For every plane, at five controlled offsets, the engine's OBB test must match
/// the exact f64 plane arithmetic it claims to implement.
///
/// Offsets are expressed as `delta` relative to the rejection boundary: the box
/// centre is placed so that `n·c + d = -r + delta`, where `r` is the box's
/// projected radius on that plane. The engine rejects iff `n·c + d < -r`, i.e.
/// iff `delta < 0`.
///
/// The `delta == 0` case (box exactly touching the plane) is *reported* rather
/// than asserted: it sits on the discontinuity, and the f32 downcast decides it.
#[test]
fn test_plane_offset_partitions() {
    let r = reference();
    let half_axes = probe_half_axes(PROBE_HALF_EXTENT);

    let mut mismatches: Vec<String> = Vec::new();
    let mut boundary_answers: Vec<String> = Vec::new();
    let mut skipped: Vec<String> = Vec::new();

    for (i, name) in PLANE_NAMES.iter().enumerate() {
        let (n, d) = r.planes_f64[i];
        let rad = projected_radius(n, &half_axes);

        // A redundant plane has no boundary to place a box against: offsets
        // relative to it are meaningless. See `test_far_plane_is_enforced`.
        let clearance = hull_clearance_to_plane(&r.hull, (n, d));
        if clearance > REDUNDANT_PLANE_CLEARANCE {
            skipped.push(format!(
                "{name}: plane is REDUNDANT (frustum hull clears it by {clearance:.6} Mm) — \
                 nothing to place a box against; see test_far_plane_is_enforced"
            ));
            continue;
        }

        let cases: [(&str, f64); 5] = [
            ("well_inside", 2.0 * rad),
            ("epsilon_inside", DECISION_EPS),
            ("exactly_on", 0.0),
            ("epsilon_outside", -DECISION_EPS),
            ("well_outside", -2.0 * rad),
        ];

        for (label, delta) in cases {
            let target = -rad + delta;
            let current = n.dot(r.centroid) + d;
            let center = r.centroid + n * (target - current);

            // Guard: the box must be unambiguously inside every *other* plane, or
            // the case is not measuring what it claims to measure.
            let mut blocked_by: Option<String> = None;
            for (j, (nj, dj)) in r.planes_f64.iter().enumerate() {
                if j == i {
                    continue;
                }
                let rj = projected_radius(*nj, &half_axes);
                let margin = nj.dot(center) + dj - rj;
                if margin <= DECISION_EPS {
                    blocked_by = Some(format!(
                        "{}/{} blocked by {} (margin {:.6} Mm)",
                        name, label, PLANE_NAMES[j], margin
                    ));
                }
            }
            if let Some(reason) = blocked_by {
                skipped.push(reason);
                continue;
            }

            let obb = to_obb(center, half_axes);
            let engine = r.frustum.intersects_obb(&obb);
            let truth = delta >= 0.0;

            if delta == 0.0 {
                boundary_answers.push(format!("{name}: engine says intersects={engine}"));
                continue;
            }

            if engine != truth {
                mismatches.push(format!(
                    "plane {name} case {label}: engine={engine} expected={truth} \
                     (signed_dist={:.9}, radius={:.9})",
                    n.dot(center) + d,
                    rad
                ));
            }

            // The box is deep inside all other planes, so the plane test and exact
            // convex intersection must agree here; if they don't, the setup is wrong.
            let sat = super::sat::hulls_intersect(&r.hull, &obb_hull(&obb), 0.0);
            if sat != truth {
                mismatches.push(format!(
                    "plane {name} case {label}: exact SAT={sat} disagrees with \
                     analytic truth={truth} — test setup is unsound"
                ));
            }
        }
    }

    println!("-- exactly-on-plane behaviour (reported, not asserted) --");
    for line in &boundary_answers {
        println!("   {line}");
    }
    if !skipped.is_empty() {
        println!("-- skipped (box not clear of other planes) --");
        for line in &skipped {
            println!("   {line}");
        }
    }

    assert!(
        mismatches.is_empty(),
        "Frustum::intersects_obb disagreed with exact f64 plane arithmetic:\n  {}",
        mismatches.join("\n  ")
    );
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 2: edge and corner straddles. MUST PASS.
// ─────────────────────────────────────────────────────────────────────────────

/// A box centred exactly on a frustum edge (two planes) or corner (three planes)
/// genuinely straddles the boundary, so both the engine and exact SAT must call it
/// visible. This is the case a naive "centre inside?" test gets wrong.
#[test]
fn test_edge_and_corner_straddles() {
    let r = reference();
    let half_axes = probe_half_axes(PROBE_HALF_EXTENT);

    // Hull corner indices: 0..3 near quad (bl, br, tr, tl), 4..7 far quad.
    let corners = &r.hull.points;
    let probes: [(&str, DVec3); 6] = [
        ("near-bottom-left corner (3 planes)", corners[0]),
        ("near-top-right corner (3 planes)", corners[2]),
        ("far-bottom-left corner (3 planes)", corners[4]),
        (
            "left/bottom edge midpoint (2 planes)",
            (corners[0] + corners[4]) * 0.5,
        ),
        (
            "right/top edge midpoint (2 planes)",
            (corners[2] + corners[6]) * 0.5,
        ),
        (
            "near face centre (1 plane)",
            (corners[0] + corners[2]) * 0.5,
        ),
    ];

    let mut failures = Vec::new();
    for (name, center) in probes {
        let obb = to_obb(center, half_axes);
        let engine = r.frustum.intersects_obb(&obb);
        let sat = super::sat::hulls_intersect(&r.hull, &obb_hull(&obb), 0.0);

        println!("  {name:<40} engine={engine:<5} exact_sat={sat}");
        if !sat {
            failures.push(format!("{name}: exact SAT says no straddle — setup unsound"));
        }
        if !engine {
            failures.push(format!(
                "{name}: FALSE NEGATIVE — box straddles the frustum boundary but \
                 Frustum::intersects_obb culled it"
            ));
        }
    }

    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 3: the classic separating-axis limitation. MEASURED, not failed.
// ─────────────────────────────────────────────────────────────────────────────

/// The well-known conservative case: a large box that lies entirely outside the
/// frustum, yet is not outside any *single* frustum plane, so the plane-by-plane
/// test cannot reject it.
///
/// This is a **measured property**, not a bug report. Testing only the frustum's
/// own face normals is an incomplete separating-axis test by construction; the
/// price is false positives (extra draw calls), never false negatives (holes).
/// The test asserts the direction of the error — the engine may over-report but
/// must never under-report — and prints the actual answer.
#[test]
fn test_large_box_past_corner_is_conservative() {
    let r = reference();

    // Every frustum corner, and every direction that leads out of it: the outward
    // bisector of each pair and triple of planes meeting there. The classic case
    // only exists in a narrow band of (box size, offset), so it has to be searched
    // for rather than guessed at.
    // With the depth planes gone there is no near plane to bisect against, so the
    // "corner" sets are the two side planes meeting at each lateral edge. The probe
    // is still anchored at the near quad's corners, which is where a box first
    // escapes the hull.
    let corner_plane_sets: [(&str, [usize; 3], usize); 8] = [
        ("near bottom-left", [0, 2, 0], 0),
        ("near bottom-right", [1, 2, 1], 1),
        ("near top-right", [1, 3, 3], 2),
        ("near top-left", [0, 3, 0], 3),
        ("left/bottom edge", [0, 2, 0], 0),
        ("right/bottom edge", [1, 2, 1], 1),
        ("right/top edge", [1, 3, 2], 2),
        ("left/top edge", [0, 3, 3], 3),
    ];

    let mut violations = Vec::new();
    let mut conservative_hits = 0usize;
    let mut provably_outside = 0usize;
    let mut example = String::new();

    for (name, planes, corner_idx) in corner_plane_sets {
        let mut normal_sum = DVec3::ZERO;
        let mut seen: Vec<usize> = Vec::new();
        for p in planes {
            if !seen.contains(&p) {
                seen.push(p);
                normal_sum += r.planes_f64[p].0;
            }
        }
        let outward = -normal_sum.normalize();
        let corner = r.hull.points[corner_idx];

        for scale in [0.5_f64, 1.0, 2.0, 4.0, 8.0, 16.0, 32.0] {
            let half_axes = probe_half_axes(scale);
            for step in 1..=16 {
                let t = scale * 0.25 * step as f64;
                let center = corner + outward * t;

                let obb = to_obb(center, half_axes);
                let sat = super::sat::hulls_intersect(&r.hull, &obb_hull(&obb), 0.0);
                let engine = r.frustum.intersects_obb(&obb);

                if sat && !engine {
                    violations.push(format!(
                        "{name} scale={scale} t={t}: FALSE NEGATIVE — box provably \
                         intersects the frustum but Frustum::intersects_obb culled it"
                    ));
                }
                if !sat {
                    provably_outside += 1;
                    if engine {
                        conservative_hits += 1;
                        if example.is_empty() {
                            // Confirm it really is the separating-axis limitation:
                            // not outside any single plane, yet outside the frustum.
                            let outside_single = r.planes_f64.iter().any(|(n, d)| {
                                n.dot(center) + d < -projected_radius(*n, &half_axes)
                            });
                            example = format!(
                                "{name}: half-extent {scale} Mm at {t:.3} Mm past the corner \
                                 — exact SAT says MISS, plane test says HIT, \
                                 outside_any_single_plane={outside_single}"
                            );
                        }
                    }
                }
            }
        }
    }

    println!(
        "  of {provably_outside} boxes that provably miss the frustum, the plane test \
         reported {conservative_hits} as visible ({:.1}% conservative false positives)",
        100.0 * conservative_hits as f64 / provably_outside.max(1) as f64
    );
    if !example.is_empty() {
        println!("  first such case: {example}");
    } else {
        println!(
            "  no case found where the plane test over-reports — on this frustum the \
             plane-only test happened to be exact for every probe."
        );
    }

    // The only thing that would be a real bug: culling a box that genuinely
    // intersects the frustum. Over-reporting is the documented price of the
    // plane-only separating-axis test.
    assert!(
        violations.is_empty(),
        "Frustum::intersects_obb culled a box that provably intersects:\n{}",
        violations.join("\n")
    );
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 3b: the plane set has no dead entry. PERMANENT GUARD (was a defect probe).
// ─────────────────────────────────────────────────────────────────────────────

/// Every plane the engine emits must actually bound the frustum.
///
/// **This was a defect probe.** `Camera::calculate_frustum_planes` used to extract
/// the depth planes as `r3 + r2` ("Near") and `r3 − r2` ("Far"), which is the
/// OpenGL extraction, valid when NDC z spans `[-1, 1]`. Under this engine's
/// `perspective_rh` (z ∈ `[0, 1]`) plus reverse-Z remap, the two depth constraints
/// are `z_clip ≤ w` and `z_clip ≥ 0`, extracted as `r3 − r2` and `r2`. The
/// consequence, measured by this test at the time: index 5 really was the near
/// plane, index 4 was a plane sitting **2.8557 Mm behind** the frustum hull — a
/// dead entry that could never reject anything — and there was no far plane at all.
///
/// The invariant this now protects: **the plane array has no redundant entry.**
/// Measured hull clearance for all four planes is ~1e-12 Mm, i.e. each one supports
/// a face of the frustum. If a fifth entry ever appears it must support a face too.
///
/// The old "a point at 2× zfar must be rejected" assertion is deliberately gone. It
/// is not a property of the tile-culling plane set any more, by design: the far
/// plane is omitted because it is **provably vacuous** for the globe. Every
/// ellipsoid point is within `‖cam‖ + a = ‖cam‖ + 6.378 Mm` of the eye, and
/// `zfar = ‖cam‖ + 10 Mm`, so no tile can ever be beyond it. That is invariant
/// **I-3**, and [`test_far_plane_is_vacuous_for_the_globe`] asserts the premise
/// directly instead of asserting a consequence the plane set no longer has.
#[test]
fn test_frustum_plane_set_has_no_dead_entry() {
    let r = reference();

    println!("  plane | normal                              | d           | hull clearance (Mm)");
    for (i, name) in PLANE_NAMES.iter().enumerate() {
        let (n, d) = r.planes_f64[i];
        println!(
            "  {name:<5} | ({:>9.6},{:>9.6},{:>9.6}) | {:>11.6} | {:>12.6}",
            n.x,
            n.y,
            n.z,
            d,
            hull_clearance_to_plane(&r.hull, (n, d))
        );
    }

    let redundant: Vec<&str> = PLANE_NAMES
        .iter()
        .enumerate()
        .filter(|(i, _)| {
            hull_clearance_to_plane(&r.hull, r.planes_f64[*i]) > REDUNDANT_PLANE_CLEARANCE
        })
        .map(|(_, n)| *n)
        .collect();

    assert!(
        redundant.is_empty(),
        "frustum planes {redundant:?} never touch the frustum hull — they are dead \
         entries in the plane array"
    );

    // All four side planes pass through the eye, which is what licenses `d ≡ 0` in
    // the camera-relative frame. Verify the premise rather than assume it.
    let mut worst = 0.0_f64;
    for (n, d) in r.planes_f64.iter() {
        worst = worst.max((n.dot(r.eye) + d).abs());
    }
    println!("  worst |n·eye + d| over the four side planes: {worst:.3e} Mm");
    assert!(
        worst < 1.0e-12,
        "a side plane does not pass through the eye ({worst:.3e} Mm); the d ≡ 0 \
         assumption behind the camera-relative frame is invalid"
    );
}

/// **Invariant I-3.** The far plane is omitted from tile culling because it cannot
/// reject an ellipsoid point: `‖p − cam‖ ≤ ‖cam‖ + a < ‖cam‖ + 10 = zfar`.
///
/// This asserts the premise. If anyone tightens `zfar`, this test fails and
/// `π_far = r2` must be reinstated in `Camera::calculate_frustum_planes` and in
/// `Frustum`.
#[test]
fn test_far_plane_is_vacuous_for_the_globe() {
    use super::geodesy::A;

    for alt_m in [0.0_f64, 500.0, 400_000.0, 12_000_000.0, 30_000_000.0] {
        for mode in [CameraMode::Free, CameraMode::Tracking, CameraMode::Cockpit] {
            let p = super::cameras::ViewParams {
                sweep: "i3",
                lat_deg: 23.0,
                lon_deg: 41.0,
                alt_m,
                mode,
                ..Default::default()
            };
            let cam = super::cameras::build_camera(&p);
            let (eye, _) = cam.global_transform_f64();
            let zfar = eye.length() + 10.0;
            let worst = eye.length() + A;
            assert!(
                zfar >= worst,
                "I-3 violated: zfar = {zfar} Mm but an ellipsoid point can be \
                 {worst} Mm from the eye (alt {alt_m} m, {:?}). Reinstate the far \
                 plane π = r2.",
                mode
            );
        }
    }
    println!("  I-3 holds: zfar = ‖cam‖ + 10 Mm ≥ ‖cam‖ + a = ‖cam‖ + {A} Mm");
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 4: the f32 downcast precision floor. MEASURED, with a guard.
// ─────────────────────────────────────────────────────────────────────────────

/// **This was the precision floor. It is now the precision guard.**
///
/// The old pipeline evaluated `n·m + d` in f32 with `‖m‖ ≈ ‖d‖ ≈ 6.378 Mm, whose
/// f32 ulp is 0.477 m, to obtain a distance of order metres — catastrophic
/// cancellation, and a *distance-independent* ~3 m error floor. Measured here at
/// the time: 0.128 m of plane-distance error against a zoom-20 tile's ~0.3 m
/// projected radius, i.e. the culling decision was made with an error worth 40 % of
/// the tile.
///
/// The pipeline now evaluates `n·Δ` with `Δ = centre − eye` formed in f64 and only
/// the small difference downcast. The error becomes `1.5·10⁻⁷·‖Δ‖`, and because the
/// LOD rule keeps a leaf at `D ≈ 2..4 ×` its own half-diagonal, the ratio
/// `error / tile_size` is a **constant ~6·10⁻⁷ at every zoom** (2.3) instead of
/// growing without bound as tiles shrink.
///
/// The table below is the finding; the assertion is that the ratio never approaches
/// 1 at any zoom, which the old pipeline could not satisfy.
#[test]
fn test_camera_relative_plane_error_vs_tile_size() {
    // A low-altitude camera is where high-zoom tiles actually get tested.
    let params = super::cameras::ViewParams {
        sweep: "f32-probe",
        lat_deg: 48.0,
        lon_deg: 9.0,
        alt_m: 500.0,
        pitch_deg: 0.0,
        mode: CameraMode::Free,
        ..Default::default()
    };
    let cam = super::cameras::build_camera(&params);
    let normals = cam.calculate_frustum_planes(ASPECT as f32);
    let (eye, _) = cam.global_transform_f64();
    let frustum = Frustum::new(normals, eye).with_corners(cam.frustum_corners_relative(ASPECT as f32));

    println!("  zoom | tile radius (m) | max |Δ(n·Δ)| (m) | error / radius");
    let mut first_bad_zoom: Option<u8> = None;
    let mut worst_ratio = 0.0_f64;

    for z in [4_u8, 8, 12, 14, 16, 17, 18, 19, 20] {
        // The tile directly under the camera.
        let id = super::geodesy::tile_for_lat_lon(params.lat_deg, params.lon_deg, z);
        let node = QuadtreeNode::new(TileId {
            z,
            x: id.x,
            y: id.y,
        });
        let ha64 = [
            to_d(node.obb.half_axes[0]),
            to_d(node.obb.half_axes[1]),
            to_d(node.obb.half_axes[2]),
        ];

        // What the engine actually computes, and what it should have computed.
        let delta_f32 = frustum.relative(node.obb.center);
        let delta_f64 = node.obb.center - eye;

        let mut max_err = 0.0_f64;
        let mut min_radius = f64::INFINITY;
        for (i, n64) in normals.iter().enumerate() {
            let n32 = frustum.normals[i];
            let approx = to_d(n32).dot(to_d(delta_f32));
            let exact = n64.dot(delta_f64);
            max_err = max_err.max((exact - approx).abs());
            min_radius = min_radius.min(projected_radius(*n64, &ha64));
        }

        let ratio = max_err / min_radius;
        worst_ratio = worst_ratio.max(ratio);
        println!(
            "  {z:>4} | {:>15.3} | {:>17.6} | {:>14.3e}",
            min_radius * 1.0e6,
            max_err * 1.0e6,
            ratio
        );
        if ratio > 1.0 && first_bad_zoom.is_none() {
            first_bad_zoom = Some(z);
        }
    }

    println!("  => worst error/radius over all zooms probed: {worst_ratio:.3e}");

    assert!(
        first_bad_zoom.is_none(),
        "camera-relative plane error exceeded the tile's own projected radius at \
         zoom {first_bad_zoom:?}. Under the f64 subtraction this ratio should be a \
         scale-free ~1e-6 at every zoom; something has reintroduced absolute-frame \
         arithmetic into the frustum path (invariant I-2)."
    );
    // Generous by three orders of magnitude over the derived 6e-7, so this does not
    // go red on unrelated LOD tuning — but tight enough that the old absolute-frame
    // pipeline (0.43 at z=20) could never pass it.
    assert!(
        worst_ratio < 1.0e-3,
        "camera-relative plane error is {worst_ratio:.3e} of the tile radius; the \
         derivation predicts ~6e-7 at every zoom"
    );
}

fn to_d(v: Vec3) -> DVec3 {
    DVec3::new(v.x as f64, v.y as f64, v.z as f64)
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 5: contains_point agrees with intersects_obb for a degenerate box.
// ─────────────────────────────────────────────────────────────────────────────

/// A zero-extent OBB is a point, so `intersects_obb` must agree with
/// `contains_point` on it. This pins the two entry points to the same plane set.
#[test]
fn test_degenerate_obb_matches_contains_point() {
    let r = reference();
    let zero = [DVec3::ZERO; 3];

    let mut probes = vec![r.centroid];
    probes.extend(r.hull.points.iter().copied());
    for (n, _) in r.planes_f64.iter() {
        probes.push(r.centroid - *n * 40.0);
        probes.push(r.centroid + *n * 0.01);
    }

    for p in probes {
        let obb = to_obb(p, zero);
        let via_obb = r.frustum.intersects_obb(&obb);
        let via_point = r
            .frustum
            .contains_point(Vec3::new(p.x as f32, p.y as f32, p.z as f32));
        assert_eq!(
            via_obb,
            via_point,
            "point {p:?}: intersects_obb={via_obb} but contains_point={via_point}"
        );
    }
}
