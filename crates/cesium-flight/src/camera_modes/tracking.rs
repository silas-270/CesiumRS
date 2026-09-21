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
    let no_bank_quat =
        cesium_engine::math::transform::velocity_to_orientation(state.position, forward);

    let up_dir = state.position.normalize_or_zero();
    let elevated_anchor = state.position + up_dir * 0.0000075;
    camera.set_anchor(elevated_anchor, no_bank_quat);

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
        camera.set_local_pos(local_pos);
        camera.look_at_plane();
    } else {
        camera.enforce_bounds();
        camera.look_at_plane();
    }
}
