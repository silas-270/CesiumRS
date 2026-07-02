use glam::{Mat4, Vec3};

pub struct GodCamera {
    pub position: Vec3,
    pub yaw: f32,
    pub pitch: f32,
    pub base_speed: f32,
    pub fast_speed: f32,
    pub sensitivity: f32,
}

impl Default for GodCamera {
    fn default() -> Self {
        Self {
            position: Vec3::new(0.0, 0.0, 25.0),
            yaw: 0.0,
            pitch: 0.0,
            base_speed: 5.0,
            fast_speed: 25.0,
            sensitivity: 0.005,
        }
    }
}

impl GodCamera {
    pub fn new(position: Vec3, yaw: f32, pitch: f32) -> Self {
        Self {
            position,
            yaw,
            pitch,
            ..Default::default()
        }
    }

    pub fn update(&mut self, dt: f32, movement: Vec3, fast: bool) {
        let speed = if fast { self.fast_speed } else { self.base_speed };
        let velocity = movement * speed * dt;

        let (yaw_sin, yaw_cos) = self.yaw.sin_cos();

        // Forward vector for movement (ignores pitch for WASD, so you don't fly into the ground when looking down, 
        // but since it's a god camera, it's often preferred to fly in the look direction.
        // Let's make it fly in the look direction for true 3D movement).
        let (pitch_sin, pitch_cos) = self.pitch.sin_cos();
        let forward = Vec3::new(
            pitch_cos * yaw_sin,
            pitch_sin,
            -pitch_cos * yaw_cos,
        ).normalize_or_zero();

        // Right vector is perpendicular to forward and world up
        let right = Vec3::new(yaw_cos, 0.0, yaw_sin).normalize_or_zero();

        let up = Vec3::Y;

        // movement: x is right/left, y is up/down, z is forward/back
        self.position += right * velocity.x + up * velocity.y + forward * velocity.z;
    }

    pub fn process_mouse(&mut self, dx: f32, dy: f32) {
        self.yaw -= dx * self.sensitivity;
        self.pitch -= dy * self.sensitivity;

        // Clamp pitch to avoid gimbal lock
        self.pitch = self.pitch.clamp(-std::f32::consts::FRAC_PI_2 + 0.01, std::f32::consts::FRAC_PI_2 - 0.01);
    }

    pub fn get_view_matrix(&self) -> Mat4 {
        let (yaw_sin, yaw_cos) = self.yaw.sin_cos();
        let (pitch_sin, pitch_cos) = self.pitch.sin_cos();

        let forward = Vec3::new(
            pitch_cos * yaw_sin,
            pitch_sin,
            -pitch_cos * yaw_cos,
        ).normalize_or_zero();

        Mat4::look_to_rh(self.position, forward, Vec3::Y)
    }

    pub fn get_projection_matrix(&self, aspect_ratio: f32) -> Mat4 {
        Mat4::perspective_rh(std::f32::consts::FRAC_PI_4, aspect_ratio, 0.01, 1000.0)
    }
}
