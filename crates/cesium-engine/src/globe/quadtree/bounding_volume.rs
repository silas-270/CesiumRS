//! Bounding volumes and the view frustum, in the **camera-relative frame**.
//!
//! Derivation: `docs/culling-math.md` §2 (planes and precision) and §5 (the
//! separating-axis set). Two design decisions dominate this file.
//!
//! **1. Four planes, not six.** Under the engine's reverse-Z projection the near
//! constraint is `r3 − r2` and the far constraint is `r2`; the old code emitted
//! `r3 + r2` as "Near", which is a plane sitting `znear` *behind* the eye and
//! bounds nothing, and never emitted the far plane at all (§2.2). Rather than fix
//! the pair, tile culling drops both:
//!
//! * **Far** is vacuous. `zfar = ‖cam‖ + 10 Mm` and every ellipsoid point is within
//!   `‖cam‖ + a` of the eye, so no tile is ever beyond it (invariant **I-3**).
//! * **Near** is vacuous whenever `znear < altitude`, which holds in Free and
//!   Cockpit always and in Tracking above 5 m. Below that it is not vacuous and it
//!   is catastrophic: it is what blanked the globe at 5 m in Tracking mode (§2.7).
//! * Nothing behind the eye survives anyway: the Left and Right half-spaces sum to
//!   `−2·z_eye ≥ 0` (Lemma 2.1).
//!
//! **2. Camera-relative.** All four side planes pass through the eye, so in a frame
//! centred on the eye their offset is `d = 0` — written as a literal zero, not
//! computed. The test then evaluates `n·(p − cam)` with the subtraction done in f64
//! and only the (small) difference downcast, which turns a distance-independent
//! ~3 m f32 error floor into `1.5·10⁻⁷·‖Δ‖` — a *constant* 6·10⁻⁷ of a tile at
//! every zoom (§2.5).

use glam::{DVec3, Vec3};

/// f32 unit roundoff, `2⁻²⁴`.
const U_F32: f32 = 5.960_464_5e-8;

/// Coefficient of the f32 rounding bound (2.2) on `n·Δ + Σ|n·h_j|`.
///
/// Eight `u`: one for the downcast of `Δ`, one for the half-axes, `2√3 u` for the
/// plane normal's components and `3u` for accumulating each three-term dot product.
/// Applied in the conservative direction — it widens the *kept* set (invariant
/// **I-6**), never the culled one.
pub(super) const FRUSTUM_EPS_COEFF: f32 = 8.0 * U_F32;

/// An oriented bounding box.
///
/// The centre is **f64** (invariant **I-2**): `Δ = centre − cam` must be an f64
/// subtraction or the camera-relative frame buys nothing — an f32 centre already
/// carries ~0.5 m of construction error at Earth scale, which no later arithmetic
/// can undo. The half-axes stay f32: they are at most half a tile across and are
/// never differenced against an Earth-scale quantity.
#[derive(Clone, Copy, Debug)]
pub struct OrientedBoundingBox {
    pub center: DVec3,
    pub half_axes: [Vec3; 3],
    /// `Σ_j ‖h_j‖₁`, cached. Feeds the rounding tolerance (2.4); the L1 norm is an
    /// upper bound on the L2 norm the derivation uses, so it stays conservative and
    /// costs no square roots per frame.
    pub half_axis_l1: f32,
}

impl OrientedBoundingBox {
    pub fn new(center: DVec3, half_axes: [Vec3; 3]) -> Self {
        let half_axis_l1 = half_axes
            .iter()
            .map(|h| h.x.abs() + h.y.abs() + h.z.abs())
            .sum();
        Self {
            center,
            half_axes,
            half_axis_l1,
        }
    }
}

/// The four side planes of the view frustum, in the camera-relative frame.
///
/// Plane order is `[Left, Right, Bottom, Top]`, derived from the rows of
/// `P_rz · V` as `r3 + r0`, `r3 − r0`, `r3 + r1`, `r3 − r1` (§2.2). All four pass
/// through the eye, so there is no offset to store — see the module header.
#[derive(Clone, Copy, Debug)]
pub struct Frustum {
    /// Inward-pointing **unit** normals.
    pub normals: [Vec3; 4],
    /// The eye, in world space (megameters): the origin of this frame.
    pub eye: DVec3,
    /// The eight frustum corners, camera-relative, for the optional box-slab stage
    /// ([`super::slab`]). `None` skips that stage entirely.
    pub corners: Option<[Vec3; 8]>,
    /// `max_k ‖corner_k‖₁`, cached for the slab stage's rounding bound.
    pub corner_l1_max: f32,
}

impl Frustum {
    /// Builds a frustum from the four f64 side-plane normals and the eye.
    pub fn new(normals: [DVec3; 4], eye: DVec3) -> Self {
        let mut n32 = [Vec3::ZERO; 4];
        for i in 0..4 {
            n32[i] = Vec3::new(normals[i].x as f32, normals[i].y as f32, normals[i].z as f32);
        }
        Self {
            normals: n32,
            eye,
            corners: None,
            corner_l1_max: 0.0,
        }
    }

    /// Attaches the eight camera-relative frustum corners (from
    /// `Camera::frustum_corners_relative`), enabling the box-slab stage.
    pub fn with_corners(mut self, corners: [Vec3; 8]) -> Self {
        self.corner_l1_max = corners
            .iter()
            .map(|c| c.x.abs() + c.y.abs() + c.z.abs())
            .fold(0.0_f32, f32::max);
        self.corners = Some(corners);
        self
    }

    /// Camera-relative position of a world point, differenced in f64.
    #[inline]
    pub fn relative(&self, p: DVec3) -> Vec3 {
        let d = p - self.eye;
        Vec3::new(d.x as f32, d.y as f32, d.z as f32)
    }

    /// As [`Frustum::relative`], for an f32 world point. The point is promoted
    /// before the subtraction, so only the point's own f32 quantisation survives.
    #[inline]
    pub fn relative_f32(&self, p: Vec3) -> Vec3 {
        self.relative(DVec3::new(p.x as f64, p.y as f64, p.z as f64))
    }

    /// The f32 rounding bound (2.4) for a box of L1 half-extent `half_axis_l1` at
    /// camera-relative offset `delta`.
    #[inline]
    fn eps(delta: Vec3, half_axis_l1: f32) -> f32 {
        FRUSTUM_EPS_COEFF * (delta.x.abs() + delta.y.abs() + delta.z.abs() + half_axis_l1)
    }

    /// Is a camera-relative point inside all four side half-spaces?
    ///
    /// Uses the same tolerance as [`Frustum::separated_from_box`] with zero extents,
    /// so a degenerate (zero half-axis) box and a point give the same answer.
    #[inline]
    pub fn contains_relative(&self, rel: Vec3) -> bool {
        let eps = Self::eps(rel, 0.0);
        for n in &self.normals {
            if n.dot(rel) < -eps {
                return false;
            }
        }
        true
    }

    /// Is a world-space point inside the frustum? Convenience wrapper; the
    /// subtraction is f64.
    #[inline]
    pub fn contains_point(&self, p: Vec3) -> bool {
        self.contains_relative(self.relative_f32(p))
    }

    /// Is a world-space sphere at least partly inside the frustum?
    ///
    /// Degenerates to [`Frustum::contains_point`] at `radius = 0`, and is monotone
    /// in `radius`.
    #[inline]
    pub fn intersects_sphere(&self, center: Vec3, radius: f32) -> bool {
        let rel = self.relative_f32(center);
        let bound = -(radius + Self::eps(rel, 0.0));
        for n in &self.normals {
            if n.dot(rel) < bound {
                return false;
            }
        }
        true
    }

    /// The separating-plane test for a box already expressed camera-relative.
    ///
    /// Rejects iff the box lies strictly outside one plane:
    /// `n·Δ + Σ_j |n·h_j| < −ε`, with `ε` the f32 rounding bound (2.4) of that very
    /// expression. Soundness: rejection means `sup_{p∈B} n·(p − cam) < 0`, so `B`
    /// is in the open half-space outside a frustum plane and cannot meet the
    /// frustum. Dropping the depth planes can only add false positives.
    #[inline]
    pub fn separated_from_box(&self, delta: Vec3, half_axes: &[Vec3; 3], half_axis_l1: f32) -> bool {
        let eps = Self::eps(delta, half_axis_l1);
        for n in &self.normals {
            let s = n.dot(delta);
            let r = n.dot(half_axes[0]).abs()
                + n.dot(half_axes[1]).abs()
                + n.dot(half_axes[2]).abs();
            if s + r < -eps {
                return true;
            }
        }
        false
    }

    /// `true` when the box may intersect the frustum.
    #[inline]
    pub fn intersects_obb(&self, obb: &OrientedBoundingBox) -> bool {
        let delta = self.relative(obb.center);
        !self.separated_from_box(delta, &obb.half_axes, obb.half_axis_l1)
            && !super::slab::separated_on_box_axes(self, delta, &obb.half_axes)
    }
}
