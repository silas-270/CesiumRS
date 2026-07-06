use crate::engine::camera::camera::Camera;
use crate::engine::math::trajectory::TransformState;

pub fn update_tracking_mode(
    camera: &mut Camera,
    state: &TransformState,
    mode_switched_or_reset: bool,
) {
    let pos_f32 = glam::Vec3::new(state.position.x as f32, state.position.y as f32, state.position.z as f32);
    
    // Orbit around the plane, but without banking. 
    // We extract the forward vector and use velocity_to_orientation.
    let forward = state.rotation * glam::DVec3::new(0.0, 0.0, -1.0);
    let no_bank_quat = crate::engine::math::transform::velocity_to_orientation(state.position, forward);
    let rot_f32 = glam::Quat::from_xyzw(no_bank_quat.x as f32, no_bank_quat.y as f32, no_bank_quat.z as f32, no_bank_quat.w as f32).normalize();
    
    camera.set_anchor(pos_f32, rot_f32);
    
    if mode_switched_or_reset {
        // 50m behind, 15m up
        camera.local_pos = glam::Vec3::new(0.0, 15.0 / 1_000_000.0, 50.0 / 1_000_000.0);
        // Pitch down slightly to look at plane
        camera.local_ori = glam::Quat::from_euler(glam::EulerRot::YXZ, 0.0, -0.25, 0.0);
    }
}
