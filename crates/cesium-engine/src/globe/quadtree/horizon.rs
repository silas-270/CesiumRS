//! Exact self-occlusion (limb) culling.
//!
//! Derivation: `docs/culling-math.md` §3. The short version:
//!
//! Scale space by `T(p) = (p.x/a, p.y/b, p.z/a)`. `T` is linear and invertible, so
//! "the segment eye→p meets the solid Earth" is invariant under it, and under `T`
//! the WGS-84 ellipsoid becomes the **unit sphere** while a Web-Mercator tile stays
//! an exact spherical rectangle `[λ₀,λ₁] × [φ₀,φ₁]` with the *same numeric* λ and φ.
//!
//! For a point `q` **on** the unit sphere, occlusion from `c = T(cam)` collapses to
//! one linear inequality, `q·c ≤ 1` (Theorem 3.4 + 3.5): the tangent-cone condition
//! is automatic for surface points, and this same inequality is, up to a strictly
//! positive factor, the back-face test `n̂(p)·(cam − p) ≤ 0`. Back-face culling is
//! therefore not an additional test — it is this one, and the engine's separate
//! sub-OBB back-face heuristic is deleted (§4).
//!
//! Because `q·c` is linear in `q`, its supremum over the rectangle has a closed
//! form (§3.4): maximise over λ, then over φ, each a single sinusoid on an interval
//! of width ≤ π. Roughly 25 flops, no transcendentals at run time, and — on
//! zero-relief terrain — **exact**: zero false negatives *and* zero false positives
//! for this stage. It is not a bound; it is the answer.
//!
//! # Invariant I-1
//!
//! The collapse to `q·c ≤ 1` is licensed by `TileMesh::generate` placing every
//! vertex at altitude ≤ 0. If terrain relief is ever applied, replace
//! [`TilePatch::is_occluded`] with the scaled-space cone test (§3.7, Theorem 3.7);
//! [`point_is_occluded`] below is its `ρ = 0` special case and is the natural place
//! to grow the radius term.
//!
//! # Invariant I-4
//!
//! Everything here is f64 and stays f64. The test's conditioning near the surface
//! scales as `1/h` with `h = √(C²−1)`: in f32 the error in `S` is ~1.2·10⁻⁷, which
//! at 3 m altitude is 0.23° of limb angle — 26 km of ground. It is 25 flops. Run it
//! in double.

use glam::DVec3;

use super::tile_id::TileBounds;
use crate::globe::geometry::{EARTH_RADIUS_A_F64, EARTH_RADIUS_B_F64};

/// f64 rounding bound on `S` (3.4): `S` is a sum of four products of quantities of
/// magnitude ≤ `C`, so its error is `≤ 8·u₆₄·C`.
const HORIZON_EPS_ROUNDING: f64 = 8.9e-16;

/// Angular slack, in radians, between the culling rectangle and the drawn mesh.
///
/// [`super::tile_id::tile_bounds`] and `TileMesh::generate` share their four corner
/// values exactly (invariant I-5), and the mesh's longitude lerp is exact at both
/// ends. What is *not* bit-exact is the mesh's interior rows: they come from
/// `web_mercator_y_to_lat` evaluated at intermediate `y`, in f32, whose monotonicity
/// is only good to an ulp of the returned latitude (~7.6·10⁻⁶ ° ≈ 0.85 m at 85°).
///
/// 10⁻⁶ rad ≈ 6.4 m of ground covers several such ulps. It is applied in the
/// conservative direction (I-6): the horizon stage must see the whole patch below
/// the limb *by this margin* before it culls. The cost is that a tile stays
/// scheduled for the last ~6 m of its descent behind the limb — immeasurable as a
/// false-positive rate, and it is the difference between "exact" and "exact for the
/// rectangle but not quite for the mesh inside it".
const HORIZON_EPS_BOUNDS_RAD: f64 = 1.0e-6;

/// Per-frame camera constants in scaled space. Built once, read by every node.
#[derive(Clone, Copy, Debug)]
pub struct HorizonCamera {
    /// `c = T(cam)`.
    pub c: DVec3,
    /// `C² = c·c`.
    pub c2: f64,
    /// `ρ = hypot(c.x, c.z)` — the maximum of `A(λ) = c.x·cos λ − c.z·sin λ`.
    pub rho: f64,
    /// Cull threshold: `S ≤ 1 − eps`.
    pub eps: f64,
    /// `false` when `C² ≤ 1` (eye at or below the surface), in which case the test
    /// is **skipped entirely**.
    ///
    /// There is no guard band. The old code allowed the test down to `h² > −0.1`,
    /// i.e. 327 km *below* the surface, where `h² < 0` makes the squared-cone
    /// condition vacuously true and the answer is "everything is occluded" — garbage
    /// (§3.1). From a point on or inside the sphere there is no useful polar plane;
    /// the only correct answer is to cull nothing.
    pub active: bool,
}

impl HorizonCamera {
    pub fn new(cam: DVec3) -> Self {
        let c = transform_to_scaled_space(cam);
        let c2 = c.dot(c);
        Self {
            c,
            c2,
            rho: c.x.hypot(c.z),
            eps: (HORIZON_EPS_ROUNDING + HORIZON_EPS_BOUNDS_RAD) * c2.max(1.0).sqrt(),
            active: c2 > 1.0,
        }
    }
}

/// `T(p)`: the map under which the WGS-84 ellipsoid becomes the unit sphere.
///
/// The **third** component is divided by `a`, not `b`, because this ECEF frame is
/// Y-up with negated Z — it is `y` that carries the semi-minor axis. A future move
/// to a Z-up convention would silently invert this;
/// `test_scaled_space_maps_surface_to_unit_sphere` pins it.
#[inline]
pub fn transform_to_scaled_space(p: DVec3) -> DVec3 {
    DVec3::new(
        p.x / EARTH_RADIUS_A_F64,
        p.y / EARTH_RADIUS_B_F64,
        p.z / EARTH_RADIUS_A_F64,
    )
}

/// Theorem 3.1 — exact occlusion of an **arbitrary** point (not necessarily on the
/// surface) by the ellipsoid.
///
/// `q` is occluded iff it is strictly beyond the polar plane of `c` *and* the line
/// `c→q` pierces the open unit ball:
///
/// ```text
/// s = C² − q·c ,  v = q − c
/// occluded  ⟺  s > h²  ∧  s² > h²·‖v‖²        with h² = C² − 1
/// ```
///
/// `s > h² ≥ 0` forces `s > 0`, so the squaring is safe and no division is needed.
/// Used by the label path, where points sit off the surface and the §3.4 collapse
/// does not apply. Returns `false` (cull nothing) when the eye is at or inside the
/// surface.
#[inline]
pub fn point_is_occluded(cam: &HorizonCamera, p: DVec3) -> bool {
    if !cam.active {
        return false;
    }
    let h2 = cam.c2 - 1.0;
    let q = transform_to_scaled_space(p);
    let v = q - cam.c;
    let s = cam.c2 - q.dot(cam.c);
    s > h2 && s * s > h2 * v.dot(v)
}

/// A tile's lon/lat rectangle, pre-reduced to the eight trig constants the closed
/// form needs. 64 B per node; no transcendentals at run time.
///
/// Index 0 is the low end (`λ₀`, `φ₀ = lat_min`), index 1 the high end.
#[derive(Clone, Copy, Debug)]
pub struct TilePatch {
    pub cos_lon: [f64; 2],
    pub sin_lon: [f64; 2],
    pub cos_lat: [f64; 2],
    pub sin_lat: [f64; 2],
}

impl TilePatch {
    pub fn new(b: &TileBounds) -> Self {
        let (s0, c0) = b.lon_min.to_radians().sin_cos();
        let (s1, c1) = b.lon_max.to_radians().sin_cos();
        let (t0, d0) = b.lat_min.to_radians().sin_cos();
        let (t1, d1) = b.lat_max.to_radians().sin_cos();
        Self {
            cos_lon: [c0, c1],
            sin_lon: [s0, s1],
            cos_lat: [d0, d1],
            sin_lat: [t0, t1],
        }
    }

    /// `S = max over the spherical rectangle of q·c` — exact (§3.4).
    ///
    /// Both maximisations have the same shape: a single sinusoid on an interval of
    /// width ≤ π, whose maximum is interior exactly when the derivative is ≥ 0 at the
    /// low end and ≤ 0 at the high end. No `atan2`, no circular-clamp special case
    /// for tiles that straddle the antimeridian relative to the camera.
    ///
    /// For λ the sinusoid is `A(λ) = c.x·cos λ − c.z·sin λ = ρ·cos(λ − λ_cam)`, whose
    /// interior maximum is `ρ`. For φ it is `g(φ) = A*·cos φ + c.y·sin φ`, whose
    /// interior maximum is `√(A*² + c.y²)`.
    ///
    /// The interior candidate is folded in with `max` rather than replacing the
    /// endpoints: `√(A*² + c.y²)` is the *global* maximum of `g`, so including it
    /// when the interior test is wrong can only over-estimate `S`, i.e. keep the
    /// tile. Conservative either way (I-6).
    #[inline]
    pub fn max_dot(&self, cam: &HorizonCamera) -> f64 {
        let c = cam.c;

        // ── maximise over λ ──────────────────────────────────────────────────
        // A'(λ) = −(c.x·sin λ + c.z·cos λ)
        let da0 = -(c.x * self.sin_lon[0] + c.z * self.cos_lon[0]);
        let da1 = -(c.x * self.sin_lon[1] + c.z * self.cos_lon[1]);
        let a_star = if da0 >= 0.0 && da1 <= 0.0 {
            cam.rho
        } else {
            let a0 = c.x * self.cos_lon[0] - c.z * self.sin_lon[0];
            let a1 = c.x * self.cos_lon[1] - c.z * self.sin_lon[1];
            a0.max(a1)
        };

        // ── maximise over φ ──────────────────────────────────────────────────
        let g0 = a_star * self.cos_lat[0] + c.y * self.sin_lat[0];
        let g1 = a_star * self.cos_lat[1] + c.y * self.sin_lat[1];
        let mut s = g0.max(g1);

        // g'(φ) = −A*·sin φ + c.y·cos φ
        let dg0 = -a_star * self.sin_lat[0] + c.y * self.cos_lat[0];
        let dg1 = -a_star * self.sin_lat[1] + c.y * self.cos_lat[1];
        if dg0 >= 0.0 && dg1 <= 0.0 {
            s = s.max((a_star * a_star + c.y * c.y).sqrt());
        }
        s
    }

    /// Is every drawable point of this tile hidden behind the limb?
    ///
    /// Soundness (§3.5): `S ≤ 1` means every point `p` of the drawn patch has
    /// `q·c ≤ 1`, hence `n̂(p)·(cam − p) ≤ 0` (Theorem 3.4), hence `p` is beyond the
    /// polar plane and — the cone condition being automatic for surface points — is
    /// occluded by the ellipsoid. The skirts lie strictly *inside* the ellipsoid, so
    /// their segments to an exterior eye cross the sphere too. The ellipsoid is
    /// convex and is the only occluder, so nothing can un-occlude them. ∎
    #[inline]
    pub fn is_occluded(&self, cam: &HorizonCamera) -> bool {
        cam.active && self.max_dot(cam) <= 1.0 - cam.eps
    }
}
