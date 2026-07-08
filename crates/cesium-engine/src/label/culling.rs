use glam::Vec3;
use crate::globe::quadtree::Frustum;

/// Returns true if the label's ECEF position is behind the Earth's horizon relative to the camera.
/// Uses the exact same ellipsoidal horizon culling math as quadtree.rs.
pub fn is_behind_horizon(camera_pos: Vec3, label_pos: Vec3) -> bool {
    let a = 6.378137_f32;
    let b = 6.356_752_4_f32;
    
    // Scale camera and label to the unit-sphere space
    let cv = Vec3::new(camera_pos.x / a, camera_pos.y / b, camera_pos.z / a);
    let hcp = Vec3::new(label_pos.x / a, label_pos.y / b, label_pos.z / a);
    
    let vh_mag_sq = cv.length_squared() - 1.0;
    if vh_mag_sq <= -0.1 {
        return false; // Camera is too close or inside the ellipsoid surface
    }
    
    let vt = hcp - cv;
    let vt_dot_vc = -vt.dot(cv);
    
    vt_dot_vc > vh_mag_sq && (vt_dot_vc * vt_dot_vc) / vt.length_squared() > vh_mag_sq
}

/// Returns true if the label is inside the camera's viewing frustum.
pub fn is_in_frustum(frustum: &Frustum, label_pos: Vec3) -> bool {
    frustum.contains_point(label_pos)
}
