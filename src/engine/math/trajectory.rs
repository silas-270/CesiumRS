use glam::{DQuat, DVec3, Mat4, Quat, Vec3};
use crate::engine::property::sampled::SampledPositionProperty;
use crate::engine::property::Property;
use crate::engine::time::SimulationTime;

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

    /// Evaluates the trajectory at the given simulation time, returning a stateless TransformState.
    /// Rotation is derived from the instantaneous tangent to the trajectory.
    pub fn evaluate(&self, time: SimulationTime) -> Option<TransformState> {
        let pos = self.property.evaluate(time)?;
        
        // Use a tiny delta to find the instantaneous tangent (forward vector)
        let delta_seconds = 0.01;
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
        let up = pos_f32.normalize_or_zero();

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
}
