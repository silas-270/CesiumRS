use glam::{DQuat, DVec3};

/// A KinematicState stores position and rotation, and smoothly
/// interpolates rotation over time to simulate inertia.
pub struct KinematicState {
    pub position: DVec3,
    pub target_rotation: DQuat,
    pub current_rotation: DQuat,
    pub rotational_inertia: f64,
}

impl KinematicState {
    pub fn new(position: DVec3, rotation: DQuat, rotational_inertia: f64) -> Self {
        Self {
            position,
            target_rotation: rotation,
            current_rotation: rotation,
            rotational_inertia,
        }
    }

    /// Update the state by interpolating the current rotation towards the target rotation.
    pub fn update(&mut self, dt: f64) {
        if self.rotational_inertia > 0.0 && dt > 0.0 {
            // Using exponential smoothing for frame-rate independent interpolation
            // that mimics an asymptotic approach (inertia-like lagging).
            let factor = 1.0 - (-self.rotational_inertia * dt).exp();
            self.current_rotation = self.current_rotation.slerp(self.target_rotation, factor).normalize();
        } else if self.rotational_inertia <= 0.0 {
            self.current_rotation = self.target_rotation;
        }
    }
}
