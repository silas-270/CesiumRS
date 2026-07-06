use crate::engine::camera::camera::Camera;
use crate::engine::math::trajectory::TransformState;

pub fn update_cockpit_mode(
    camera: &mut Camera,
    state: &TransformState,
    mode_switched_or_reset: bool,
) {
    let pos_f32 = glam::Vec3::new(state.position.x as f32, state.position.y as f32, state.position.z as f32);
    
    // Cockpit is tied to the plane's exact rotation (banks and pitches with it).
    let cur_rot = state.rotation;
    let rot_f32 = glam::Quat::from_xyzw(cur_rot.x as f32, cur_rot.y as f32, cur_rot.z as f32, cur_rot.w as f32).normalize();
    
    camera.set_anchor(pos_f32, rot_f32);
    
    if mode_switched_or_reset {
        camera.local_pos = glam::Vec3::new(0.0, 2.0 / 1_000_000.0, -32.0 / 1_000_000.0);
        camera.local_ori = glam::Quat::IDENTITY;
    }
}
