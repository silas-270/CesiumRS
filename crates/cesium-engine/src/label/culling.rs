use glam::Vec3;
use crate::globe::quadtree::Frustum;

/// Returns true if the label's ECEF position is behind the Earth's horizon relative to the camera.
/// Uses pre-scaled unit-sphere camera position cv and vh_mag_sq to avoid redundant calculations.
pub fn is_behind_horizon(cv: Vec3, vh_mag_sq: f32, label_pos: Vec3) -> bool {
    if vh_mag_sq <= -0.1 {
        return false; // Camera is too close or inside the ellipsoid surface
    }
    
    let a = 6.378137_f32;
    let b = 6.356_752_4_f32;
    
    let hcp = Vec3::new(label_pos.x / a, label_pos.y / b, label_pos.z / a);
    
    let vt = hcp - cv;
    let vt_dot_vc = -vt.dot(cv);
    
    vt_dot_vc > vh_mag_sq && (vt_dot_vc * vt_dot_vc) / vt.length_squared() > vh_mag_sq
}

/// Returns true if the label is inside the camera's viewing frustum.
pub fn is_in_frustum(frustum: &Frustum, label_pos: Vec3) -> bool {
    frustum.contains_point(label_pos)
}
