//! # Layer 1 — analytic frustum/OBB tests, no globe involved.
//!
//! Synthetic [`OrientedBoundingBox`] values are placed at controlled offsets
//! relative to each individual frustum plane, and the engine's
//! `Frustum::intersects_obb` is compared against ground truth computed here in
//! f64 — either the exact plane arithmetic (for the documented contract) or full
//! separating-axis intersection over the frustum's convex hull (for the cases
//! where the two disagree by design).
//!
//! Plane order, from the upstream Cesium convention the engine inherits, is
//! `[Left, Right, Bottom, Top, Near, Far]`.

use cesium_engine::camera::camera::{Camera, CameraMode};
use cesium_engine::globe::quadtree::{Frustum, OrientedBoundingBox, QuadtreeNode, TileId};
use glam::{DVec3, Quat, Vec3};

use super::sat::{obb_hull, Hull};

pub const PLANE_NAMES: [&str; 6] = ["Left", "Right", "Bottom", "Top", "Near", "Far"];

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
    planes_f64: [(DVec3, f64); 6],
    frustum: Frustum,
    hull: Hull,
    /// A point comfortably inside the frustum: the centroid of its eight corners.
    centroid: DVec3,
}

fn reference() -> Reference {
    let cam = reference_camera();
    let planes_f64 = cam.calculate_frustum_planes(ASPECT as f32);
    let frustum = Frustum::from_planes(planes_f64);
    let view_proj = cam.get_projection_matrix_f64(ASPECT) * cam.get_view_matrix_f64();
    let hull = Hull::from_view_proj(view_proj);
    let centroid = hull.points.iter().copied().sum::<DVec3>() / hull.points.len() as f64;
    Reference {
        planes_f64,
        frustum,
        hull,
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
    OrientedBoundingBox {
        center: f(center),
        half_axes: [f(half_axes[0]), f(half_axes[1]), f(half_axes[2])],
    }
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
    let corner_plane_sets: [(&str, [usize; 3], usize); 8] = [
        ("near bottom-left", [0, 2, 5], 0),
        ("near bottom-right", [1, 2, 5], 1),
        ("near top-right", [1, 3, 5], 2),
        ("near top-left", [0, 3, 5], 3),
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
// Test 3b: the far plane is never enforced. DEFECT PROBE.
// ─────────────────────────────────────────────────────────────────────────────

/// **Measured defect.** `Camera::calculate_frustum_planes` extracts the depth
/// planes as `r3 + r2` (index 4, "Near") and `r3 - r2` (index 5, "Far").
///
/// That pair is the OpenGL extraction, valid when NDC z spans `[-1, 1]`. The
/// engine's projection is `glam::Mat4::perspective_rh` (NDC z in `[0, 1]`) with a
/// reverse-Z remap on top, so the two depth constraints are `z_clip ≥ 0` and
/// `z_clip ≤ w` — extracted as `r2` and `r3 - r2`, not `r3 + r2` and `r3 - r2`.
///
/// The consequence, measured below: index 5 really is the near plane, and index 4
/// is a plane sitting *behind the eye* that the frustum hull clears by megameters
/// and which therefore never rejects anything. **There is no far-plane culling.**
///
/// This is conservative — it can only cause false positives, never holes — and it
/// is currently harmless because `zfar = |camera| + 10 Mm` already encloses the
/// whole Earth. It is recorded here so the fix phase knows the plane array has a
/// dead entry, and so that anyone who later tightens `zfar` finds out immediately.
#[test]
#[ignore = "defect probe: calculate_frustum_planes uses the GL z-in-[-1,1] depth-plane extraction under a reverse-Z z-in-[0,1] projection, leaving plane 4 degenerate and the far plane unenforced"]
fn test_far_plane_is_enforced() {
    let r = reference();
    let cam = reference_camera();

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

    // A point twice as far away as zfar, straight down the view axis. The oracle's
    // clip box rejects it (reverse-Z ndc.z < 0); a correct frustum must too.
    let (cam_pos, cam_ori) = cam.global_transform_f64();
    let forward = (cam_ori * DVec3::NEG_Z).normalize();
    let zfar = cam_pos.length() + 10.0;
    let beyond = cam_pos + forward * (zfar * 2.0);

    let oracle = super::oracle::VisibilityOracle::new(&cam, ASPECT);
    let ndc = oracle.ndc(beyond).expect("point is in front of the eye");
    println!("  probe at 2x zfar: ndc.z = {:.6} (must be in [0,1] to be visible)", ndc.z);
    assert!(
        ndc.z < 0.0,
        "test setup: the probe point should be beyond the far plane in NDC"
    );

    let redundant: Vec<&str> = PLANE_NAMES
        .iter()
        .enumerate()
        .filter(|(i, _)| {
            hull_clearance_to_plane(&r.hull, r.planes_f64[*i]) > REDUNDANT_PLANE_CLEARANCE
        })
        .map(|(_, n)| *n)
        .collect();
    println!("  redundant planes (never bound the frustum): {redundant:?}");

    assert!(
        redundant.is_empty(),
        "frustum planes {redundant:?} never touch the frustum hull — they are dead \
         entries in the plane array"
    );
    assert!(
        !r.frustum
            .contains_point(Vec3::new(beyond.x as f32, beyond.y as f32, beyond.z as f32)),
        "a point at 2x zfar is outside the clip box but the frustum accepted it: \
         the far plane is not being enforced"
    );
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 4: the f32 downcast precision floor. MEASURED, with a guard.
// ─────────────────────────────────────────────────────────────────────────────

/// `Frustum::from_planes` downcasts plane normals and offsets to f32 before any
/// culling happens. At Earth scale the offset `d` is O(1..30) megameters, so its
/// f32 quantum is O(1e-6..1e-6·30) megameters — i.e. metres. A zoom-20 tile is
/// about 30 m across.
///
/// This test measures, per zoom level, the f32 plane-distance error against the
/// tile's own projected radius, and reports the zoom at which the error stops
/// being negligible. It asserts only a loose, documented guard so it does not go
/// red on unrelated changes; the numbers in the output are the finding.
#[test]
fn test_f32_plane_downcast_error_vs_tile_size() {
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
    let planes_f64 = cam.calculate_frustum_planes(ASPECT as f32);
    let frustum_f32 = Frustum::from_planes(planes_f64);

    println!("  zoom | tile radius (m) | max |Δ(n·c+d)| (m) | error / radius");
    let mut first_bad_zoom: Option<u8> = None;

    for z in [4_u8, 8, 12, 14, 16, 17, 18, 19, 20] {
        // The tile directly under the camera.
        let id = super::geodesy::tile_for_lat_lon(params.lat_deg, params.lon_deg, z);
        let node = QuadtreeNode::new(TileId {
            z,
            x: id.x,
            y: id.y,
        });
        let c64 = DVec3::new(
            node.obb.center.x as f64,
            node.obb.center.y as f64,
            node.obb.center.z as f64,
        );
        let ha64 = [
            DVec3::new(
                node.obb.half_axes[0].x as f64,
                node.obb.half_axes[0].y as f64,
                node.obb.half_axes[0].z as f64,
            ),
            DVec3::new(
                node.obb.half_axes[1].x as f64,
                node.obb.half_axes[1].y as f64,
                node.obb.half_axes[1].z as f64,
            ),
            DVec3::new(
                node.obb.half_axes[2].x as f64,
                node.obb.half_axes[2].y as f64,
                node.obb.half_axes[2].z as f64,
            ),
        ];

        let mut max_err = 0.0_f64;
        let mut min_radius = f64::INFINITY;
        for (i, (n64, d64)) in planes_f64.iter().enumerate() {
            let (n32, d32) = frustum_f32.planes[i];
            let n32_64 = DVec3::new(n32.x as f64, n32.y as f64, n32.z as f64);
            let exact = n64.dot(c64) + d64;
            let approx = n32_64.dot(c64) + d32 as f64;
            max_err = max_err.max((exact - approx).abs());
            min_radius = min_radius.min(projected_radius(*n64, &ha64));
        }

        let ratio = max_err / min_radius;
        println!(
            "  {z:>4} | {:>15.3} | {:>18.3} | {:>14.4}",
            min_radius * 1.0e6,
            max_err * 1.0e6,
            ratio
        );
        if ratio > 1.0 && first_bad_zoom.is_none() {
            first_bad_zoom = Some(z);
        }
    }

    match first_bad_zoom {
        Some(z) => println!(
            "  => from zoom {z} onward, the f32 plane downcast error exceeds the \
             tile's own projected radius: culling decisions at that scale are noise."
        ),
        None => println!("  => f32 downcast error stayed below tile radius at every zoom probed."),
    }

    // Guard only. The documented expectation at the time of writing is that the
    // error stays below the tile radius through at least zoom 14; if that stops
    // being true something has changed about the plane pipeline, not about f32.
    assert!(
        first_bad_zoom.map(|z| z > 14).unwrap_or(true),
        "f32 plane downcast error exceeded the tile radius as early as zoom {:?}, \
         which is far coarser than expected — the frustum plane pipeline changed.",
        first_bad_zoom
    );
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
