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

/// Where a box sits relative to the frustum's four side half-spaces.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BoxVerdict {
    /// Provably outside one of the four planes — culled, no further test needed.
    Outside,
    /// Provably inside all four — it meets the frustum, no further test possible.
    Inside,
    /// Neither: the only case where the extra separating axes can say anything.
    Straddling,
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
    /// The four **edge rays** of the side-plane pyramid — unit direction of each
    /// far corner in the camera-relative frame, in f64. `None` alongside `corners`.
    ///
    /// The side planes all pass through the eye, so the volume the four-plane test
    /// models is the infinite pyramid `cone(r₀..r₃)` with apex at the eye. These
    /// rays are its *edges*, and edges are half of what a complete separating-axis
    /// set needs (see [`super::slab::separated_on_edge_cross_axes`]).
    pub rays: Option<[DVec3; 4]>,
}

impl Frustum {
    /// Builds a frustum from the four f64 side-plane normals and the eye.
    pub fn new(normals: [DVec3; 4], eye: DVec3) -> Self {
        let mut n32 = [Vec3::ZERO; 4];
        for i in 0..4 {
            n32[i] = Vec3::new(
                normals[i].x as f32,
                normals[i].y as f32,
                normals[i].z as f32,
            );
        }
        Self {
            normals: n32,
            eye,
            corners: None,
            corner_l1_max: 0.0,
            rays: None,
        }
    }

    /// Attaches the eight camera-relative frustum corners (from
    /// `Camera::frustum_corners_relative`), enabling the box-slab stage.
    pub fn with_corners(mut self, corners: [Vec3; 8]) -> Self {
        self.corner_l1_max = corners
            .iter()
            .map(|c| c.x.abs() + c.y.abs() + c.z.abs())
            .fold(0.0_f32, f32::max);
        // The far quad, normalised in f64: the pyramid's four edge directions.
        // f32 corner components carry ~6·10⁻⁸ of relative error, i.e. ~6·10⁻⁸ rad
        // of direction error, which the edge-cross test's tolerance covers with
        // three orders to spare (see `slab::separated_on_edge_cross_axes`).
        let mut rays = [DVec3::ZERO; 4];
        for i in 0..4 {
            let c = corners[4 + i];
            rays[i] = DVec3::new(c.x as f64, c.y as f64, c.z as f64).normalize_or_zero();
        }
        self.rays = Some(rays);
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
    pub fn separated_from_box(
        &self,
        delta: Vec3,
        half_axes: &[Vec3; 3],
        half_axis_l1: f32,
    ) -> bool {
        let eps = Self::eps(delta, half_axis_l1);
        for n in &self.normals {
            let s = n.dot(delta);
            let r =
                n.dot(half_axes[0]).abs() + n.dot(half_axes[1]).abs() + n.dot(half_axes[2]).abs();
            if s + r < -eps {
                return true;
            }
        }
        false
    }

    /// Where a box sits relative to the four side half-spaces.
    ///
    /// `s + r < 0` puts the box wholly outside a plane, `s − r ≥ 0` puts it wholly
    /// inside one, and the `s_p = n_p·Δ` both need are shared. Both verdicts are
    /// taken with the rounding bound in the conservative direction (I-6) — a box is
    /// only called `Outside` on a proof, and only called `Inside` on a proof.
    ///
    /// The four `s_p` come first on their own, tested against the box's
    /// **circumsphere** (`half_axis_l1` bounds its radius, since
    /// `‖Σ t_j h_j‖ ≤ Σ ‖h_j‖ ≤ Σ ‖h_j‖₁`). That decides most boxes — a sub-cell is
    /// either well inside the frustum or well outside it — for 20 flops instead of
    /// 92, and only a box the sphere leaves undecided pays for the twelve
    /// `n_p·h_j`. It matters because `SubGrid::any_visible`'s first pass runs this
    /// `k²` times per node.
    #[inline]
    pub fn classify_box(
        &self,
        delta: Vec3,
        half_axes: &[Vec3; 3],
        half_axis_l1: f32,
    ) -> BoxVerdict {
        let eps = Self::eps(delta, half_axis_l1);
        let mut s = [0.0_f32; 4];
        let mut sphere_inside = true;
        for p in 0..4 {
            s[p] = self.normals[p].dot(delta);
            if s[p] + half_axis_l1 < -eps {
                return BoxVerdict::Outside;
            }
            if s[p] - half_axis_l1 < eps {
                sphere_inside = false;
            }
        }
        if sphere_inside {
            return BoxVerdict::Inside;
        }

        let mut inside_all = true;
        for p in 0..4 {
            let n = self.normals[p];
            let r =
                n.dot(half_axes[0]).abs() + n.dot(half_axes[1]).abs() + n.dot(half_axes[2]).abs();
            if s[p] + r < -eps {
                return BoxVerdict::Outside;
            }
            if s[p] - r < eps {
                inside_all = false;
            }
        }
        if inside_all {
            BoxVerdict::Inside
        } else {
            BoxVerdict::Straddling
        }
    }

    /// `true` when the box may intersect the frustum — exact, up to the rounding
    /// tolerances, for the four-plane pyramid (§5.2 + [`super::slab`]).
    ///
    /// Three cheap verdicts come first, in increasing order of cost, and each ends
    /// the question outright:
    ///
    /// 0. **Circumsphere outside a plane, or inside all four** — 20 flops, and it
    ///    settles any box that is not close to the frustum's boundary.
    /// 1. **Outside one plane** — separated, culled. One pass, ~92 flops.
    /// 2. **Inside all four** — it meets the frustum, and no axis could separate it.
    /// 3. **A box vertex inside all four** — likewise a witness of intersection,
    ///    and it costs only sign flips: the vertex's plane distance is
    ///    `s_p ± r_{p,0} ± r_{p,1} ± r_{p,2}` in quantities the first pass already
    ///    computed. ~96 adds for all eight vertices.
    ///
    /// Only a box with no vertex inside and no separating plane — one truly wedged
    /// against a frustum edge or corner — reaches the ~760-flop remainder. That is
    /// what makes an exact test affordable per node *and* per sub-box: the
    /// expensive branch is not the boundary, it is the boundary's corners.
    ///
    /// Verdicts 2 and 3 keep the box, so their tolerance is applied in the keeping
    /// direction; verdict 1 culls, so its tolerance is applied in the keeping
    /// direction too (invariant **I-6**).
    #[inline]
    pub fn intersects_obb(&self, obb: &OrientedBoundingBox) -> bool {
        let delta = self.relative(obb.center);
        let eps = Self::eps(delta, obb.half_axis_l1);
        let half_axes = &obb.half_axes;

        let mut s = [0.0_f32; 4];
        let mut sphere_inside = true;
        for p in 0..4 {
            s[p] = self.normals[p].dot(delta);
            if s[p] + obb.half_axis_l1 < -eps {
                return false;
            }
            if s[p] - obb.half_axis_l1 < eps {
                sphere_inside = false;
            }
        }
        if sphere_inside {
            return true;
        }

        let mut r = [[0.0_f32; 3]; 4];
        let mut inside_all = true;
        for p in 0..4 {
            let n = self.normals[p];
            r[p] = [
                n.dot(half_axes[0]),
                n.dot(half_axes[1]),
                n.dot(half_axes[2]),
            ];
            let ra = r[p][0].abs() + r[p][1].abs() + r[p][2].abs();
            if s[p] + ra < -eps {
                return false;
            }
            if s[p] - ra < eps {
                inside_all = false;
            }
        }
        if inside_all {
            return true;
        }

        for m in 0..8u32 {
            let t0 = if m & 1 == 0 { 1.0 } else { -1.0 };
            let t1 = if m & 2 == 0 { 1.0 } else { -1.0 };
            let t2 = if m & 4 == 0 { 1.0 } else { -1.0 };
            let mut inside = true;
            for p in 0..4 {
                if s[p] + t0 * r[p][0] + t1 * r[p][1] + t2 * r[p][2] < -eps {
                    inside = false;
                    break;
                }
            }
            if inside {
                return true;
            }
        }

        !super::slab::separated_on_box_axes(self, delta, half_axes)
            && !super::slab::separated_on_edge_cross_axes(self, delta, half_axes)
    }
}
