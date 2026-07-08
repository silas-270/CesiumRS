use crate::engine::camera::camera::Camera;
use crate::engine::math::trajectory::TransformState;

pub fn update_cockpit_mode(
    camera: &mut Camera,
    state: &TransformState,
    mode_switched_or_reset: bool,
) {
    // Cockpit is tied to the plane's exact rotation (banks and pitches with it).
    camera.set_anchor(state.position, state.rotation);
    
    if mode_switched_or_reset {
        // Plane's local -Z points forward. Move 44m forward along -Z and 17m up along Y.
        camera.local_pos = glam::Vec3::new(0.0, 17.0 / 1_000_000.0, -44.0 / 1_000_000.0);
        // Look directly along the forward axis
        camera.local_ori = glam::Quat::IDENTITY;
    }
}
