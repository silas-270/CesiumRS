#[cfg(test)]
mod tests {
    use cesium_engine::camera::camera::Camera;
    use glam::{Vec3, Quat, Mat3, DVec3, DQuat, DMat3};

    #[test]
    fn test_tracking_camera_orbit_mouse() {
        // Setup anchor (airplane flying at equator, 0m high)
        let earth_radius = 6.378137;
        let alt = 0.0;
        let anchor_pos = DVec3::new(earth_radius + alt, 0.0, 0.0);
        
        // Plane is flying North. So forward is +Z. Up is +X.
        // Wait, +X is away from earth center.
        let anchor_ori = DQuat::from_mat3(&DMat3::from_cols(
            DVec3::new(0.0, 1.0, 0.0), // Right: Y
            DVec3::new(1.0, 0.0, 0.0), // Up: X
            DVec3::new(0.0, 0.0, 1.0), // Forward: Z
        ));

        let mut camera = Camera::new(Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, -1.0));
        camera.set_anchor(anchor_pos, anchor_ori);
        
        // Start 250m behind and 22 deg up
        let dist = 250.0 / 1_000_000.0;
        let pitch = 22.0 * std::f32::consts::PI / 180.0;
        let yaw = std::f32::consts::FRAC_PI_4; // 45 degrees
        
        let y = dist * pitch.sin();
        let horizontal_dist = dist * pitch.cos();
        let x = horizontal_dist * -yaw.sin();
        let z = horizontal_dist * yaw.cos();
        
        let local_pos = Vec3::new(x, y, z);
        let forward = -local_pos.normalize_or_zero();
        let right = Vec3::Y.cross(forward).normalize_or_zero();
        let up = forward.cross(right).normalize_or_zero();
        let rot_mat = Mat3::from_cols(right, up, -forward);
        
        camera.set_local_transform(local_pos, Quat::from_mat3(&rot_mat));

        let initial_length = camera.local_pos.length();

        // Orbit down aggressively to trigger bounds
        for _ in 0..10 {
            camera.orbit_mouse(0.0, -100.0);
            println!("Camera local pos: {:?}", camera.local_pos);
        }

        // Output info for debugging
        println!("Camera local pos after pitch down: {:?}", camera.local_pos);
        println!("Camera length after pitch down: {} (Initial: {})", camera.local_pos.length(), initial_length);
        
        // Verify length shouldn't drastically change if we just slide along the surface.
        // Wait, if we slide, the distance to the plane MUST change?
        // Actually, if we want to stay at the same orbit distance, we CANNOT go below the surface.
        // The proper fix is to reject the pitch if it causes `global_pos` to go below surface!
        assert!((camera.local_pos.length() - initial_length).abs() < 0.000001, "Camera distance changed due to orbit collision!");
    }

    #[test]
    fn test_tracking_camera_zoom() {
        // Setup anchor (airplane flying at equator, 10000m high)
        let earth_radius = 6.378137;
        let alt = 0.01;
        let anchor_pos = DVec3::new(earth_radius + alt, 0.0, 0.0);
        
        let anchor_ori = DQuat::from_mat3(&DMat3::from_cols(
            DVec3::new(0.0, 1.0, 0.0), // Right: Y
            DVec3::new(1.0, 0.0, 0.0), // Up: X
            DVec3::new(0.0, 0.0, 1.0), // Forward: Z
        ));

        let mut camera = Camera::new(Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, -1.0));
        camera.set_anchor(anchor_pos, anchor_ori);
        camera.mode = cesium_engine::camera::camera::CameraMode::Tracking;

        // Position camera 250m behind the plane
        // 250m is 0.00025 megameters
        let start_local_pos = Vec3::new(0.0, 0.0, 0.00025);
        camera.set_local_transform(start_local_pos, Quat::IDENTITY);

        // Zoom in (delta > 0, e.g. 1.0)
        camera.zoom(1.0);

        // At distance 250m (0.00025), which is above 20m threshold (0.00002):
        // speed = 0.00025
        // move_distance = 0.00025 * 0.15 * 1.0 = 0.0000375
        // forward = -Vec3::Z
        // With Quat::IDENTITY, local_pos.z becomes 0.00025 - 0.0000375 = 0.0002125
        assert!((camera.local_pos.z - 0.0002125).abs() < 1e-7, "Expected camera to move to 0.0002125, got {:?}", camera.local_pos.z);

        // Now try to move camera extremely close (e.g. 5m behind the plane)
        // 5m is 0.000005 megameters.
        // set_local_transform enforces bounds immediately, so it should be clamped to 20m (0.00002).
        camera.set_local_transform(Vec3::new(0.0, 0.0, 0.000005), Quat::IDENTITY);
        assert!((camera.local_pos.z - 0.00002).abs() < 1e-7, "Expected camera to be clamped to 0.00002, got {:?}", camera.local_pos.z);

        // Zoom in again. This attempts to move closer, but enforce_bounds will reject the closer position
        // and push it back to the minimum threshold.
        camera.zoom(1.0);

        // The camera should remain clamped at 0.00002.
        assert!((camera.local_pos.z - 0.00002).abs() < 1e-7, "Expected camera to remain clamped at 0.00002, got {:?}", camera.local_pos.z);
    }
}
