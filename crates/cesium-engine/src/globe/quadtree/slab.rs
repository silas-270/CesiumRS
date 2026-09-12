//! The separating axes the four side planes leave out.
//!
//! Derivation: `docs/culling-math.md` §5.2, recalibrated in §13. The four-plane
//! test only tries the frustum's own face normals, which is an *incomplete*
//! separating-axis set — a box can lie wholly outside the frustum while not being
//! outside any single plane, because it pokes past a frustum **corner** or edge.
//! This module holds the two families that finish the set:
//!
//! | family | axes | this module |
//! |--------|------|-------------|
//! | the frustum's face normals | 4 | no — [`Frustum::intersects_obb`] |
//! | the **box's** face normals | 3 | [`separated_on_box_axes`], off |
//! | **edge × edge** | 4 × 3 | [`separated_on_edge_cross_axes`], on |
//!
//! With near and far dropped (invariant I-3), the volume the side planes describe
//! is the infinite pyramid with apex at the eye and the four corner rays as edges.
//! All three families together are therefore the complete set for pyramid × OBB,
//! and the test stops being a bound and becomes the answer.
//!
//! **Soundness is free.** A separating axis is a separating axis: if the two hulls'
//! projections onto an axis are disjoint, the hulls are disjoint. Adding axes can
//! only reject more, and every rejection is a proof. FN stays 0. ∎
//!
//! Both stages are skipped when the caller did not attach the frustum corners
//! (`Frustum::corners == None`), which is what [`Frustum::new`] leaves them as — so
//! a caller that only has plane normals is unaffected, and correspondingly gets the
//! looser answer.
//!
//! # What the measurement says now
//!
//! Both stages were A/B-ed on all nine sweeps of the harness with everything else
//! held at its final shape (`quadtree::SUB_BOXES_PER_AXIS`), FN zero throughout:
//!
//! | | total FP | mean update |
//! |---|---|---|
//! | neither | 5.91 % — *worse than the pre-rework baseline* | 5.9 µs |
//! | edge-cross only | **2.05 %** | **6.7 µs** |
//! | edge-cross + box axes | 2.04 % | 7.2 µs |
//!
//! The edge-cross family is the whole story, and the earlier conclusion that the
//! corner over-report was out of budget was wrong — not because 1 900 flops became
//! affordable, but because [`Frustum::intersects_obb`] reaches them for perhaps one
//! box in a hundred (see its vertex witness). Whole sweeps that the subdivision
//! rule could not fix at any `k` fall to it: `camera_modes`' one stubborn tile
//! survived a 16 × 16 grid and dies here.
//!
//! The **box's own axes**, by contrast, are now worth almost nothing: 0.01 points
//! of FP for 0.5 µs, because what they used to catch the vertex witness and the
//! edge crosses already catch. They are kept, compiled out behind [`ENABLED`],
//! because they are correct, they are measured, and if tile bounding volumes ever
//! get much larger relative to the frustum (a tighter `zfar`, or 3D tiles) the
//! trade flips back. On the derivation's synthetic corner probe, where the boxes
//! are 0.5 to 32 Mm across and sit just past a frustum corner, they still do what
//! §5.2 says: `test_large_box_past_corner_is_conservative`'s 658 boxes over-report
//! 91 times (13.8 %) with the four planes alone and 24 times (3.6 %) with this
//! stage added.
//!
use glam::{DVec3, Vec3};

use super::bounding_volume::{Frustum, FRUSTUM_EPS_COEFF};

/// Master switch for the box-axis stage. `false` compiles it out entirely — it is
/// the one line to flip when A/B-ing the measurements quoted in the module header.
///
/// Kept as a `const` rather than a runtime flag on purpose: the stage sits in the
/// innermost loop of `QuadtreeNode::update`, and even an `env::var_os` probe there
/// costs more than the test it guards (measured: +4 µs on a 10 µs update).
pub const ENABLED: bool = false;

/// Is the box separated from the frustum along one of its **own** axes?
///
/// Projects the eight camera-relative frustum corners onto each box axis and looks
/// for a gap against the box's slab. Uses the un-normalised half-axis `h_j` as the
/// axis, which makes the slab `[−‖h_j‖², +‖h_j‖²]` and removes three square roots
/// and three divisions per box per frame.
///
/// `delta` is the box centre in the camera-relative frame.
#[inline]
pub fn separated_on_box_axes(frustum: &Frustum, delta: Vec3, half_axes: &[Vec3; 3]) -> bool {
    if !ENABLED {
        return false;
    }
    let Some(corners) = &frustum.corners else {
        return false;
    };

    let delta_l1 = delta.x.abs() + delta.y.abs() + delta.z.abs();

    for h in half_axes {
        let extent_sq = h.length_squared();
        // A degenerate (zero) half-axis gives the zero vector, whose projections are
        // all 0; `lo > 0` and `hi < 0` are then both false, so it rejects nothing.
        let mut lo = f32::INFINITY;
        let mut hi = f32::NEG_INFINITY;
        for c in corners {
            let t = (*c - delta).dot(*h);
            lo = lo.min(t);
            hi = hi.max(t);
        }

        // f32 rounding bound of the products above, applied in the conservative
        // direction (invariant I-6): the gap must be real, not rounding.
        let eps = FRUSTUM_EPS_COEFF
            * (delta_l1 + frustum.corner_l1_max)
            * (h.x.abs() + h.y.abs() + h.z.abs());

        if hi < -(extent_sq + eps) || lo > extent_sq + eps {
            return true;
        }
    }
    false
}

/// Master switch for the edge-cross stage ([`separated_on_edge_cross_axes`]).
///
/// On: without it the culler is worse than the code it replaced. See the module
/// header for the A/B, and `quadtree::SUB_BOXES_PER_AXIS` for the rest of the
/// calibration it was done against.
pub const EDGE_CROSS_ENABLED: bool = true;

/// f64 counterpart of [`FRUSTUM_EPS_COEFF`]: the inputs are f32-rounded (`delta`,
/// the half-axes, and the rays, which come from f32 corners), so ~8 f32 ulps is
/// still the honest bound even though the arithmetic below is double.
const EDGE_EPS_COEFF: f64 = 8.0 * 5.960_464_5e-8;

/// Is the box separated from the frustum along one of the **edge × edge** axes?
///
/// This is the stage that makes the side-plane test *exact*.
///
/// # Why it completes the set
///
/// With near and far dropped (§2.2, invariant I-3) the volume the four planes
/// describe is the infinite pyramid `P = cone(r₀..r₃)` with apex at the eye — a
/// convex polyhedron whose faces are the four side planes and whose **edges** are
/// the four rays. For two convex polyhedra, the separating-axis set is complete
/// when it holds every face normal of each and every cross product of an edge of
/// one with an edge of the other. Here that is
///
/// * 4 face normals of `P` — [`Frustum::separated_from_box`];
/// * 3 face normals of the box — [`separated_on_box_axes`];
/// * 4 × 3 = 12 edge crosses — **this function**.
///
/// So `separated_from_box || separated_on_box_axes || separated_on_edge_cross_axes`
/// is not a bound at all: it is disjointness, decided. What was left over after the
/// first two stages is exactly §5.2's corner over-report, and it does not vanish
/// with subdivision — a patch that grazes a frustum *corner* has every sub-box
/// grazing it too, which is why `camera_modes`' one stubborn tile survived a 16×16
/// grid (measured; see `quadtree::SUB_BOXES_PER_AXIS`).
///
/// # Soundness
///
/// `P` is a cone with apex at the origin of this frame, so its support along an
/// axis `a` is `0` when every `a·r_m ≤ 0`, and `+∞` otherwise. A rejection
/// therefore needs two facts, and both are taken with the rounding bound applied in
/// the conservative direction (invariant **I-6**): every ray strictly on the far
/// side, and the whole box strictly on the near side. Being an exact criterion does
/// not make it an aggressive one — refusing to separate is always available, and a
/// tolerance that is too wide only keeps tiles.
///
/// `a = r_i × h_j` is perpendicular to `r_i`, so `a·r_i = 0` holds exactly and only
/// the other three rays are tested; a near-zero `a` (ray parallel to a box axis)
/// separates nothing and falls out through the same comparisons.
///
/// # Cost, and why it is affordable
///
/// ~660 flops per box against the four-plane test's ~92 — but it runs **only on a
/// box the cheap stages already accepted**, and `SubGrid::any_visible` stops at the
/// first survivor. A visible tile therefore pays for one; a tile being culled pays
/// for the handful of its sub-boxes that reached the frustum's corner.
#[inline]
pub fn separated_on_edge_cross_axes(frustum: &Frustum, delta: Vec3, half_axes: &[Vec3; 3]) -> bool {
    if !EDGE_CROSS_ENABLED {
        return false;
    }
    let Some(rays) = &frustum.rays else {
        return false;
    };

    let d = DVec3::new(delta.x as f64, delta.y as f64, delta.z as f64);
    let h: [DVec3; 3] = [
        DVec3::new(
            half_axes[0].x as f64,
            half_axes[0].y as f64,
            half_axes[0].z as f64,
        ),
        DVec3::new(
            half_axes[1].x as f64,
            half_axes[1].y as f64,
            half_axes[1].z as f64,
        ),
        DVec3::new(
            half_axes[2].x as f64,
            half_axes[2].y as f64,
            half_axes[2].z as f64,
        ),
    ];
    let d_l1 = d.x.abs() + d.y.abs() + d.z.abs();
    let h_l1: f64 = h.iter().map(|v| v.x.abs() + v.y.abs() + v.z.abs()).sum();

    for (i, r) in rays.iter().enumerate() {
        for hj in &h {
            let a = r.cross(*hj);
            let a_l1 = a.x.abs() + a.y.abs() + a.z.abs();
            if a_l1 == 0.0 {
                continue;
            }
            // Rounding bounds: the rays are unit, so the cone side scales with ‖a‖
            // alone; the box side carries ‖Δ‖ and the half-extents as well.
            let ray_eps = EDGE_EPS_COEFF * a_l1;
            let box_eps = EDGE_EPS_COEFF * a_l1 * (d_l1 + h_l1);

            let mut hi = f64::NEG_INFINITY;
            let mut lo = f64::INFINITY;
            for (m, rm) in rays.iter().enumerate() {
                if m == i {
                    continue;
                }
                let t = a.dot(*rm);
                hi = hi.max(t);
                lo = lo.min(t);
            }

            let c = a.dot(d);
            let e = a.dot(h[0]).abs() + a.dot(h[1]).abs() + a.dot(h[2]).abs();

            // Cone on the −side (sup = 0), box strictly on the +side.
            if hi <= -ray_eps && c - e > box_eps {
                return true;
            }
            // Cone on the +side (inf = 0), box strictly on the −side.
            if lo >= ray_eps && c + e < -box_eps {
                return true;
            }
        }
    }
    false
}
