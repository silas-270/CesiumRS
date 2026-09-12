//! The independent f64 visibility oracle.
//!
//! This is *ground truth* for the globe sweep. It knows nothing about the quadtree,
//! bounding volumes, horizon-culling points or LOD; it answers a single question in
//! double precision:
//!
//! > Given this camera, would a point on the ellipsoid surface end up inside the
//! > viewport with the ellipsoid itself not in the way?
//!
//! Two independent conditions, both exact:
//!
//! 1. **Front-face test.** The ellipsoid is convex, so the only thing that can
//!    occlude a surface point is the ellipsoid itself. A surface point is
//!    self-occluded exactly when the outward normal faces away from the eye:
//!    `normal · (camera_pos − p) > 0` means visible. No horizon approximation, no
//!    tangent-plane fudge — convexity makes this exact.
//!
//! 2. **Projection test.** `proj_f64(aspect) * view_f64` applied to the point,
//!    divided by `w`. Accept iff `w > 0 && |ndc.x| ≤ 1 && |ndc.y| ≤ 1 && 0 ≤ ndc.z ≤ 1`.
//!    The engine's projection is **reverse-Z**: NDC z is 1 at the near plane and 0
//!    at the far plane, so the depth interval is [0, 1] with the *near* end at 1.
//!
//! ## Why there is a `Marginal` verdict
//!
//! The engine culls in f32 (`Frustum::from_planes` downcasts). A point that sits
//! within f32 noise of a frustum plane, or within f32 noise of the limb, genuinely
//! has no defensible answer, and scoring it either way turns the harness into a
//! random-number generator. Such points are classified [`Verdict::Marginal`] and
//! excluded from both the false-negative and false-positive tallies; their count is
//! reported so the exclusion stays visible and can't quietly hide a regression.
//!
//! The margins are *deliberately* much larger than f32 epsilon so the instrument is
//! measuring real culling defects, not rounding.

use cesium_engine::camera::camera::Camera;
use glam::{DMat4, DVec3, DVec4};

use super::geodesy::{ellipsoid_normal, ray_ellipsoid_f64};

/// How far (in cosine of the angle between the surface normal and the direction to
/// the eye) a point must be from the limb before the oracle commits to an answer.
///
/// 1e-3 rad-equivalent ≈ 0.06°. At Earth scale the limb sweeps ~0.06° in roughly
/// 6 km of ground distance, so this excludes a band a few kilometres wide around
/// the horizon — far wider than f32 error, far narrower than any real cull bug.
pub const LIMB_COS_MARGIN: f64 = 1.0e-3;

/// How far outside/inside the NDC cube a point must be before the oracle commits.
///
/// 2e-3 of half-screen ≈ 2 px on a 1920-wide viewport. Frustum-plane distances at
/// Earth scale are O(1..10) megameters, whose f32 representation error is O(1e-6)
/// megameters — several orders below this band.
pub const NDC_MARGIN: f64 = 2.0e-3;

/// Ground-truth verdict for one surface point.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Verdict {
    /// Unambiguously on screen and front-facing. A visible tile **must** cover it.
    Visible,
    /// Within the numeric no-man's-land around the limb or a viewport edge.
    /// Scored neither way.
    Marginal,
    /// Unambiguously off screen or facing away. No tile is obliged to cover it.
    Hidden,
}

/// A camera frozen into its f64 matrices, plus the unprojection needed to shoot
/// ground-truth rays through the viewport.
pub struct VisibilityOracle {
    pub cam_pos: DVec3,
    pub view_proj: DMat4,
    pub inv_view_proj: DMat4,
    pub aspect: f64,
    pub limb_cos_margin: f64,
    pub ndc_margin: f64,
}

impl VisibilityOracle {
    pub fn new(cam: &Camera, aspect: f64) -> Self {
        let (cam_pos, _) = cam.global_transform_f64();
        let view_proj = cam.get_projection_matrix_f64(aspect) * cam.get_view_matrix_f64();
        Self {
            cam_pos,
            view_proj,
            inv_view_proj: view_proj.inverse(),
            aspect,
            limb_cos_margin: LIMB_COS_MARGIN,
            ndc_margin: NDC_MARGIN,
        }
    }

    /// Cosine of the angle between the outward surface normal at `p` and the
    /// direction from `p` to the eye. Positive = front-facing.
    pub fn facing_cos(&self, p: DVec3) -> f64 {
        let to_eye = self.cam_pos - p;
        let len = to_eye.length();
        if len <= 0.0 {
            return 0.0;
        }
        ellipsoid_normal(p).dot(to_eye / len)
    }

    /// Clip-space position of `p`.
    pub fn clip(&self, p: DVec3) -> DVec4 {
        self.view_proj * p.extend(1.0)
    }

    /// NDC position of `p`, or `None` when the point is at/behind the eye plane.
    pub fn ndc(&self, p: DVec3) -> Option<DVec3> {
        let c = self.clip(p);
        if c.w <= 0.0 {
            return None;
        }
        Some(c.truncate() / c.w)
    }

    /// Signed "outsideness" of an NDC point w.r.t. the clip box
    /// `[-1,1] × [-1,1] × [0,1]` (reverse-Z: 1 = near, 0 = far).
    ///
    /// Negative means strictly inside, by that margin.
    fn ndc_outsideness(ndc: DVec3) -> f64 {
        (ndc.x.abs() - 1.0)
            .max(ndc.y.abs() - 1.0)
            .max(-ndc.z)
            .max(ndc.z - 1.0)
    }

    /// The full ground-truth verdict for a point on (or very near) the surface.
    pub fn classify(&self, p: DVec3) -> Verdict {
        // 1. Front-face / self-occlusion. Exact by convexity of the ellipsoid.
        let f = self.facing_cos(p);
        if f <= -self.limb_cos_margin {
            return Verdict::Hidden;
        }
        if f < self.limb_cos_margin {
            return Verdict::Marginal;
        }

        // 2. Projection into the reverse-Z clip box.
        let c = self.clip(p);
        if c.w <= 0.0 {
            return Verdict::Hidden;
        }
        // A near-degenerate w makes the NDC divide meaningless.
        if c.w < 1.0e-12 * c.truncate().length().max(1.0) {
            return Verdict::Marginal;
        }
        let ndc = c.truncate() / c.w;
        let out = Self::ndc_outsideness(ndc);
        if out < -self.ndc_margin {
            Verdict::Visible
        } else if out > self.ndc_margin {
            Verdict::Hidden
        } else {
            Verdict::Marginal
        }
    }

    /// Unproject an NDC point to world space (megameters), in f64.
    pub fn unproject(&self, ndc: DVec3) -> DVec3 {
        let p = self.inv_view_proj * ndc.extend(1.0);
        p.truncate() / p.w
    }

    /// A correct world-space ray through a viewport position.
    ///
    /// **This exists because `Camera::screen_to_world_ray` must not be used as
    /// ground truth.** That function hardcodes `FRAC_PI_4` (45°) for the vertical
    /// FOV while the real projection uses ~46.4° (Free/Tracking) or 60° (Cockpit),
    /// so every raycast built on it is systematically wrong towards the screen
    /// edges. Here the ray is the exact inverse of the very matrices the engine
    /// projects and derives its frustum planes from: near point at reverse-Z
    /// `ndc.z = 1`, far point at `ndc.z = 0`.
    pub fn ray_through_ndc(&self, ndc_x: f64, ndc_y: f64) -> (DVec3, DVec3) {
        let near = self.unproject(DVec3::new(ndc_x, ndc_y, 1.0));
        let far = self.unproject(DVec3::new(ndc_x, ndc_y, 0.0));
        (near, (far - near).normalize())
    }

    /// The surface point seen at a viewport position, if the ellipsoid is there.
    pub fn surface_point_at_ndc(&self, ndc_x: f64, ndc_y: f64) -> Option<DVec3> {
        let (o, d) = self.ray_through_ndc(ndc_x, ndc_y);
        ray_ellipsoid_f64(o, d)
    }
}
