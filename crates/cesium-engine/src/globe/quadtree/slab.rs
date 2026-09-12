//! Optional second separating-axis stage: the **box's own three axes**.
//!
//! Derivation: `docs/culling-math.md` §5.2. The four-plane test only tries the
//! frustum's face normals, which is an incomplete separating-axis set — a box can
//! lie wholly outside the frustum while not being outside any single plane (it
//! pokes past a frustum *corner* or edge). The complete set for frustum × OBB is 27
//! axes, about 1 900 flops, which is out of budget. This module adds the three
//! cheapest useful ones: the box's own face normals.
//!
//! **Soundness is free.** A separating axis is a separating axis: if the two hulls'
//! projections onto `u_j` are disjoint, the hulls are disjoint. Adding axes can only
//! reject more, and every rejection is a proof. FN stays 0. ∎
//!
//! # Measured, and switched off
//!
//! On the derivation's synthetic corner probe it does what §5.2 says it does. With
//! `test_large_box_past_corner_is_conservative`'s 658 boxes that provably miss the
//! reference frustum:
//!
//! | test | over-reports |
//! |------|--------------|
//! | 4 side planes alone | 91 (13.8 %) — the derivation predicted 12.4 % |
//! | 4 side planes + this stage | 24 (3.6 %) — the derivation predicted 2.1 % |
//!
//! On **real tiles** it is not worth its cost. Whole-sweep numbers from the
//! 100 000-cell fuzz sweep, against `QuadtreeManager::update` latency over 204
//! representative poses:
//!
//! | | FP | mean update |
//! |---|---|---|
//! | off | 4.18 % | 4.3 µs |
//! | on  | 3.98 % | 7.0 µs |
//!
//! 0.2 points of false positives for **+63 % of the culling budget**. That is about
//! 0.04 tiles saved per frame for 2.7 µs, i.e. ~65 µs of CPU per tile avoided —
//! an order of magnitude more than the tile costs to schedule.
//!
//! The reason the synthetic result does not transfer: the probe's boxes are 0.5 to
//! 32 Mm across and sit just past a frustum corner, where the box's own axes really
//! do separate. A tile's OBB is tiny next to a frustum that reaches `‖cam‖ + 10 Mm`,
//! so the frustum's projection onto a box axis swallows the box's slab whenever the
//! four planes have already accepted it. Subdividing the box
//! (`quadtree::SUB_BOXES_MIN`) attacks the same §5.2 over-report far more
//! effectively — 7.30 % → 4.18 % for 1.8 µs.
//!
//! Kept, compiled out, behind [`ENABLED`], because it is correct, it is measured,
//! and if tile bounding volumes ever get much larger relative to the frustum (a
//! tighter `zfar`, or 3D tiles) the trade flips back.
//!
//! The stage is also skipped when the caller did not attach the frustum corners
//! (`Frustum::corners == None`), which is what [`Frustum::new`] leaves them as — so
//! a caller that only has plane normals is unaffected.

use glam::Vec3;

use super::bounding_volume::{Frustum, FRUSTUM_EPS_COEFF};

/// Master switch for this stage. `false` compiles it out entirely — it is the one
/// line to flip when A/B-ing the measurements quoted in the module header.
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
