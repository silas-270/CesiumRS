use cesium_engine::camera::camera::Camera;
use crate::tracker::FlightEntity;

pub fn update_free_mode(
    camera: &mut Camera,
    flights: &[FlightEntity],
    aspect_ratio: f32,
    mode_switched_or_reset: bool,
) {
    if !mode_switched_or_reset {
        return;
    }

    if let Some(flight) = flights.first() {
        let samples = flight.property.samples();
        if samples.len() >= 2 {
            let s = samples.first().unwrap().1;
            let e = samples.last().unwrap().1;
            
            let s_f32 = glam::Vec3::new(s.x as f32, s.y as f32, s.z as f32);
            let e_f32 = glam::Vec3::new(e.x as f32, e.y as f32, e.z as f32);
            
            let m = (s_f32 + e_f32) * 0.5;
            let n = m.normalize();
            
            let v = e_f32 - s_f32;
            let v_tangent = v - v.dot(n) * n;
            let d_vec = v_tangent.normalize_or_zero();
            
            let p = 0.05;
            let half_h = 0.5;
            let half_w = 0.5 * aspect_ratio;
            
            let py = half_h - p;
            let px = half_w - p;
            
            let theta = (py / px).atan();
            
            let right = glam::Quat::from_axis_angle(n, -theta) * d_vec;
            let up = glam::Quat::from_axis_angle(n, std::f32::consts::PI / 2.0) * right;
            
            let m_surface = n * 6.378137;
            let mut u_min = f32::MAX; let mut u_max = f32::MIN;
            let mut v_min = f32::MAX; let mut v_max = f32::MIN;
            for (_, pos) in samples {
                let pos_f32 = glam::Vec3::new(pos.x as f32, pos.y as f32, pos.z as f32);
                let vec_from_m = pos_f32 - m_surface;
                let u = vec_from_m.dot(right);
                let v_val = vec_from_m.dot(up);
                if u < u_min { u_min = u; }
                if u > u_max { u_max = u; }
                if v_val < v_min { v_min = v_val; }
                if v_val > v_max { v_max = v_val; }
            }
            
            let u_center = (u_min + u_max) * 0.5;
            let v_center = (v_min + v_max) * 0.5;
            let w_req = u_max - u_min;
            let h_req = v_max - v_min;
            
            let new_m = m_surface + right * u_center + up * v_center;
            let new_n = new_m.normalize();
            
            let fov_y = 2.0 * (12.0 / camera.focal_length).atan();
            let d_h = h_req / (4.0 * py * (fov_y / 2.0).tan());
            let d_w = w_req / (4.0 * px * (fov_y / 2.0).tan());
            let distance = d_h.max(d_w).max(0.01);
            
            let final_cam_pos = new_n * 6.378137 + new_n * distance;
            
            let forward = -new_n;
            let safe_right = forward.cross(up).normalize();
            let safe_up = safe_right.cross(forward).normalize();
            let rot_mat = glam::Mat3::from_cols(safe_right, safe_up, -forward);
            
            let q = glam::Quat::from_mat3(&rot_mat);
            camera.set_anchor(glam::DVec3::ZERO, glam::DQuat::IDENTITY);
            camera.set_local_transform(final_cam_pos, q);
        }
    } else {
        // Default if no flights
        camera.set_anchor(glam::DVec3::ZERO, glam::DQuat::IDENTITY);
        camera.set_local_transform(glam::Vec3::new(0.0, 0.0, 20.0), glam::Quat::IDENTITY);
    }
}
