use cesium_engine::camera::camera::Camera;
use cesium_engine::math::trajectory::TransformState;

pub fn update_tracking_mode(
    camera: &mut Camera,
    state: &TransformState,
    mode_switched_or_reset: bool,
) {
    // Orbit around the plane, but without banking. 
    // We extract the forward vector and use velocity_to_orientation.
    let forward = state.rotation * glam::DVec3::new(0.0, 0.0, -1.0);
    let no_bank_quat = cesium_engine::math::transform::velocity_to_orientation(state.position, forward);
    
    camera.set_anchor(state.position, no_bank_quat);
    
    if mode_switched_or_reset {
        let dist = 250.0 / 1_000_000.0;
        let pitch = 22.0 * std::f32::consts::PI / 180.0;
        let yaw = std::f32::consts::FRAC_PI_4; // 45 degrees
        
        let y = dist * pitch.sin();
        let horizontal_dist = dist * pitch.cos();
        
        // Negative sin for left wing (-X axis), positive cos for back (+Z axis)
        let x = horizontal_dist * -yaw.sin();
        let z = horizontal_dist * yaw.cos();
        
        let local_pos = glam::Vec3::new(x, y, z);
        
        let forward = -local_pos.normalize_or_zero();
        let right = forward.cross(glam::Vec3::Y).normalize_or_zero();
        let up = right.cross(forward).normalize_or_zero();
        let rot_mat = glam::Mat3::from_cols(right, up, -forward);
        
        camera.set_local_transform(local_pos, glam::Quat::from_mat3(&rot_mat));
    }
}
