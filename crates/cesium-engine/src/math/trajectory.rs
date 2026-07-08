use glam::{DQuat, DVec3, Mat4, Quat, Vec3};
use crate::property::sampled::SampledPositionProperty;
use crate::property::Property;
use crate::time::SimulationTime;

#[derive(Clone, Copy, Debug)]
pub struct TransformState {
    pub position: DVec3,
    pub rotation: DQuat,
}

pub struct TrajectoryEvaluator<'a> {
    pub property: &'a SampledPositionProperty,
    pub inertia_window_seconds: f64,
}

impl<'a> TrajectoryEvaluator<'a> {
    pub fn new(property: &'a SampledPositionProperty, inertia_window_seconds: f64) -> Self {
        Self {
            property,
            inertia_window_seconds,
        }
    }

    /// Evaluates the raw trajectory state at the given simulation time.
    pub fn evaluate_raw(&self, time: SimulationTime) -> Option<TransformState> {
        let pos = self.property.evaluate(time)?;
        
        // Use a larger delta to find the instantaneous tangent (forward vector) and acceleration
        let delta_seconds = 0.2;
        let next_time = SimulationTime::new(time.seconds + delta_seconds);
        
        let forward = if let Some(next_pos) = self.property.evaluate(next_time) {
            let dir = next_pos - pos;
            if dir.length_squared() > 1e-20 {
                let d = dir.normalize();
                Vec3::new(d.x as f32, d.y as f32, d.z as f32)
            } else {
                Vec3::new(0.0, 1.0, 0.0) // Fallback if no movement
            }
        } else {
            // Fallback for the very end of the flight path: look backward instead
            let prev_time = SimulationTime::new(time.seconds - delta_seconds);
            if let Some(prev_pos) = self.property.evaluate(prev_time) {
                let dir = pos - prev_pos;
                if dir.length_squared() > 1e-20 {
                    let d = dir.normalize();
                    Vec3::new(d.x as f32, d.y as f32, d.z as f32)
                } else {
                    Vec3::new(0.0, 1.0, 0.0)
                }
            } else {
                Vec3::new(0.0, 1.0, 0.0)
            }
        };

        let pos_f32 = Vec3::new(pos.x as f32, pos.y as f32, pos.z as f32);
        let earth_up = pos_f32.normalize_or_zero();
        let mut up = earth_up;

        // Calculate centripetal acceleration to simulate banking (roll)
        // To prevent abrupt changes and simulate massive inertia, we average the acceleration.
        let num_samples = 30;
        let mut avg_a = Vec3::ZERO;
        let mut total_weight = 0.0;
        
        // We look ahead much further than we look behind so the plane can anticipate curves.
        // For example, if inertia_window is 120s, lookahead=90s, lookbehind=30s.
        let lookbehind = self.inertia_window_seconds * 0.25;
        let _lookahead = self.inertia_window_seconds * 0.75;
        
        for i in 0..=num_samples {
            let fraction = i as f32 / num_samples as f32;
            let weight = (fraction * std::f32::consts::PI).sin();

            let t_offset = -lookbehind + (i as f64 / num_samples as f64) * self.inertia_window_seconds;
            let sample_time = SimulationTime::new(time.seconds + t_offset);
            
            let prev_t = SimulationTime::new(sample_time.seconds - delta_seconds);
            let next_t = SimulationTime::new(sample_time.seconds + delta_seconds);
            
            if let (Some(p), Some(prev), Some(next)) = (
                self.property.evaluate(sample_time),
                self.property.evaluate(prev_t),
                self.property.evaluate(next_t)
            ) {
                let v1 = (p - prev) / delta_seconds;
                let v2 = (next - p) / delta_seconds;
                let a = (v2 - v1) / delta_seconds;
                avg_a += Vec3::new(a.x as f32, a.y as f32, a.z as f32) * weight;
                total_weight += weight;
            }
        }

        if total_weight > 0.0 {
            avg_a /= total_weight;
            
            // Gravity is 9.81 m/s^2. In Megameters, that's 9.81e-6 Mm/s^2.
            let g_mm = 9.81e-6;
            let gravity_vector = -earth_up * g_mm;
            
            // Apply a cinematic multiplier to the centripetal acceleration so the banking is visually noticeable
            let cinematic_g_multiplier = 2.5;
            
            // Perceived gravity is actual gravity minus the acceleration (F_apparent = m*g - m*a)
            let perceived_gravity = gravity_vector - avg_a * cinematic_g_multiplier;
            
            if perceived_gravity.length_squared() > 1e-20 {
                // The plane aligns its "up" vector to oppose perceived gravity
                up = -perceived_gravity.normalize();
            }
        }

        // Construct orthonormal basis
        // We prioritize the `forward` vector so the plane perfectly aligns with the polyline.
        // `earth_up` is only used to determine the `right` vector.
        let right = forward.cross(up).normalize_or_zero();
        let up_adjusted = right.cross(forward).normalize_or_zero();

        let rotation_mat = Mat4::from_cols(
            right.extend(0.0),               // Local X -> Right
            up_adjusted.extend(0.0),         // Local Y -> Up (adjusted for pitch)
            (-forward).extend(0.0),          // Local Z -> Backward (exact tangent)
            Vec3::ZERO.extend(1.0),
        );

        let quat = Quat::from_mat4(&rotation_mat).normalize();
        let rotation = DQuat::from_xyzw(quat.x as f64, quat.y as f64, quat.z as f64, quat.w as f64);

        Some(TransformState {
            position: pos,
            rotation,
        })
    }

    /// Evaluates the trajectory at the given simulation time, returning a stateless, smoothed TransformState.
    pub fn evaluate(&self, time: SimulationTime) -> Option<TransformState> {
        let base_state = self.evaluate_raw(time)?;

        // Apply a symmetric quaternion smoothing filter around `time` to eliminate high-frequency angular jerks.
        let mut sum_q = glam::DQuat::IDENTITY;
        let mut first_q: Option<glam::DQuat> = None;
        let mut total_weight = 0.0;

        let start_t = self.property.start_time().map(|t| t.seconds).unwrap_or(time.seconds);
        let stop_t = self.property.stop_time().map(|t| t.seconds).unwrap_or(time.seconds);

        let d_start = time.seconds - start_t;
        let d_end = stop_t - time.seconds;

        let max_half_window = 0.8; // 1.6s total window / 2
        let half_window = d_start.min(d_end).max(0.0).min(max_half_window);

        let num_samples = 8;
        for i in 0..=num_samples {
            let frac = i as f64 / num_samples as f64;
            let t_offset = -half_window + frac * (2.0 * half_window);
            let sample_time = SimulationTime::new(time.seconds + t_offset);

            if let Some(raw_state) = self.evaluate_raw(sample_time) {
                let q = raw_state.rotation;
                let weight = (frac * std::f64::consts::PI).sin();

                if let Some(first) = first_q {
                    let dot = q.dot(first);
                    let aligned_q = if dot < 0.0 { -q } else { q };
                    sum_q = sum_q + aligned_q * weight;
                } else {
                    first_q = Some(q);
                    sum_q = q * weight;
                }
                total_weight += weight;
            }
        }

        let smoothed_rotation = if total_weight > 0.0 {
            sum_q.normalize()
        } else {
            base_state.rotation
        };

        Some(TransformState {
            position: base_state.position,
            rotation: smoothed_rotation,
        })
    }
}
