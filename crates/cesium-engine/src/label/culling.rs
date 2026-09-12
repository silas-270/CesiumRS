//! Per-label visibility: horizon occlusion and frustum containment.
//!
//! Labels are *points*, and they are not necessarily on the ellipsoid, so this path
//! cannot use the tile path's collapse to `q·c ≤ 1` (which is licensed only for
//! surface points — `docs/culling-math.md` §3.3b). It uses the full two-condition
//! exact test, Theorem 3.1, instead.

use crate::globe::quadtree::{point_is_occluded, Frustum, HorizonCamera};
use glam::{DVec3, Vec3};

/// Is this label behind the Earth's limb?
///
/// Exact (Theorem 3.1), f64, division-free, and with **no guard band**. The old
/// f32 version carried `if vh_mag_sq <= -0.1 { return false }`, which let the test
/// run with the camera up to ~327 km *below* the surface — a regime where `h² < 0`
/// makes the squared-cone condition vacuously true and every label reports as
/// occluded. The correct branch is `C² > 1` or nothing, and it now lives inside
/// [`HorizonCamera`]: build one per frame and pass it in.
pub fn is_behind_horizon(cam: &HorizonCamera, label_pos: Vec3) -> bool {
    point_is_occluded(
        cam,
        DVec3::new(label_pos.x as f64, label_pos.y as f64, label_pos.z as f64),
    )
}

/// Returns true if the label is inside the camera's viewing frustum.
pub fn is_in_frustum(frustum: &Frustum, label_pos: Vec3) -> bool {
    frustum.contains_point(label_pos)
}

/// Returns true if the sphere intersects the frustum.
pub fn intersects_sphere(frustum: &Frustum, center: Vec3, radius: f32) -> bool {
    frustum.intersects_sphere(center, radius)
}
