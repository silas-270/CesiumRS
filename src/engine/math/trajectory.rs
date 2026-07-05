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
    /// Rotation is derived smoothly by averaging velocity within an inertia window.
    pub fn evaluate(&self, time: SimulationTime) -> Option<TransformState> {
        let pos = self.property.evaluate(time)?;
        
        // Number of samples to take looking into the past for inertia
        let num_samples = 5;
        let time_step = self.inertia_window_seconds / (num_samples as f64);
        
        let mut avg_forward = DVec3::ZERO;
        let mut valid_samples = 0;

        for i in 0..=num_samples {
            let sample_time = SimulationTime::new(time.seconds - (i as f64) * time_step);
            let sample_time_next = SimulationTime::new(sample_time.seconds + 0.1);
            
            if let (Some(p0), Some(p1)) = (self.property.evaluate(sample_time), self.property.evaluate(sample_time_next)) {
                let dir = p1 - p0;
                if dir.length_squared() > 1e-10 {
                    avg_forward += dir.normalize();
                    valid_samples += 1;
                }
            }
        }

        let pos_f32 = Vec3::new(pos.x as f32, pos.y as f32, pos.z as f32);
        let up = pos_f32.normalize_or_zero();

        let forward = if valid_samples > 0 {
            let avg = (avg_forward / (valid_samples as f64)).normalize_or_zero();
            Vec3::new(avg.x as f32, avg.y as f32, avg.z as f32)
        } else {
            // Fallback if no valid samples
            let next_time = SimulationTime::new(time.seconds + 0.1);
            if let Some(next_pos) = self.property.evaluate(next_time) {
                let dir = next_pos - pos;
                if dir.length_squared() > 1e-10 {
                    let d = dir.normalize();
                    Vec3::new(d.x as f32, d.y as f32, d.z as f32)
                } else {
                    Vec3::new(0.0, 1.0, 0.0)
                }
            } else {
                Vec3::new(0.0, 1.0, 0.0)
            }
        };

        let right = forward.cross(up).normalize_or_zero();
        let adjusted_forward = up.cross(right).normalize_or_zero();

        let rotation_mat = Mat4::from_cols(
            right.extend(0.0),               // Local X -> Right
            up.extend(0.0),                  // Local Y -> Up
            (-adjusted_forward).extend(0.0), // Local Z -> Backward
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
