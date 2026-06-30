use glam::{Mat4, Quat, Vec4, Vec3};

pub struct Camera {
    target: Vec3,
    rotation: Quat,
    distance: f32,
    
    min_distance: f32,
    pan_sensitivity: f32,
    zoom_sensitivity: f32,
    pitch_sensitivity: f32,
    
    drag_start_point: Option<Vec3>,
}

impl Camera {
    pub fn new(target: Vec3, distance: f32) -> Self {
        Self {
            target,
            rotation: Quat::IDENTITY,
            distance,
            min_distance: 6.378137 + 0.05,
            pan_sensitivity: 0.005,
            zoom_sensitivity: 0.1,
            pitch_sensitivity: 0.05,
            drag_start_point: None,
        }
    }

    pub fn set_target(&mut self, target: Vec3) {
        self.target = target;
    }

    pub fn set_eye(&mut self, eye: Vec3) {
        let dir = eye - self.target;
        self.distance = dir.length().max(self.min_distance);
        
        if self.distance > 0.0001 {
            let view = Mat4::look_at_rh(eye, self.target, Vec3::Y);
            self.rotation = Quat::from_mat4(&view.inverse());
        }
    }

    pub fn screen_to_world_ray(&self, screen_x: f32, screen_y: f32, screen_width: f32, screen_height: f32) -> (Vec3, Vec3) {
        let aspect_ratio = screen_width / screen_height;
        
        let ndc_x = (2.0 * screen_x) / screen_width - 1.0;
        let ndc_y = 1.0 - (2.0 * screen_y) / screen_height;

        let ndc_near = Vec4::new(ndc_x, ndc_y, 0.0, 1.0);
        let ndc_far = Vec4::new(ndc_x, ndc_y, 1.0, 1.0);

        let view_proj = self.get_projection_matrix(aspect_ratio) * self.get_view_matrix();
        let inv_view_proj = view_proj.inverse();

        let mut world_near = inv_view_proj * ndc_near;
        world_near /= world_near.w;

        let mut world_far = inv_view_proj * ndc_far;
        world_far /= world_far.w;

        let ray_origin = world_near.truncate();
        let ray_dir = (world_far.truncate() - world_near.truncate()).normalize();

        (ray_origin, ray_dir)
    }

    pub fn intersect_sphere(&self, ray_origin: Vec3, ray_dir: Vec3, radius: f32) -> Option<Vec3> {
        let b = 2.0 * ray_origin.dot(ray_dir);
        let c = ray_origin.length_squared() - radius * radius;
        
        let discriminant = b * b - 4.0 * c;
        if discriminant < 0.0 {
            return None;
        }
        
        let t = (-b - discriminant.sqrt()) / 2.0;
        if t < 0.0 {
            return None;
        }
        
        Some(ray_origin + ray_dir * t)
    }

    pub fn begin_drag(&mut self, screen_x: f32, screen_y: f32, screen_width: f32, screen_height: f32) {
        let (ray_origin, ray_dir) = self.screen_to_world_ray(screen_x, screen_y, screen_width, screen_height);
        let earth_radius = 6.378137;
        self.drag_start_point = self.intersect_sphere(ray_origin, ray_dir, earth_radius);
    }

    pub fn drag(&mut self, screen_x: f32, screen_y: f32, screen_width: f32, screen_height: f32) {
        if let Some(start_point) = self.drag_start_point {
            let (ray_origin, ray_dir) = self.screen_to_world_ray(screen_x, screen_y, screen_width, screen_height);
            let earth_radius = 6.378137;
            
            if let Some(current_point) = self.intersect_sphere(ray_origin, ray_dir, earth_radius) {
                // Rotate the camera so that the ray at the current screen coordinate hits the start point
                let rot_delta = Quat::from_rotation_arc(current_point.normalize(), start_point.normalize());
                
                self.rotation = (rot_delta * self.rotation).normalize();
            } else {
                // Ray missed the earth, we could optionally do fallback panning, but ignoring is usually standard for strict sticky pan.
            }
        }
    }

    pub fn end_drag(&mut self) {
        self.drag_start_point = None;
    }

    pub fn orbit(&mut self, delta_x: f32, delta_y: f32) {
        // Fallback or explicit orbit method if sticky panning isn't used
        if delta_x == 0.0 && delta_y == 0.0 { return; }
        
        let yaw = Quat::from_rotation_y(-delta_x * self.pan_sensitivity);
        let pitch = Quat::from_axis_angle(self.rotation * Vec3::X, -delta_y * self.pan_sensitivity);

        let new_rot = (yaw * pitch * self.rotation).normalize();
        
        let local_up = new_rot * Vec3::Y;
        if local_up.y > 0.0 {
            self.rotation = new_rot;
        }
    }

    pub fn zoom(&mut self, delta: f32) {
        if delta == 0.0 { return; }
        
        let distance_above_surface = self.distance - self.min_distance;
        let step = delta * self.zoom_sensitivity * distance_above_surface.max(0.0001);
        
        self.distance -= step;
        self.distance = self.distance.max(self.min_distance);
    }

    pub fn pitch(&mut self, delta: f32) {
        if delta == 0.0 { return; }

        let pitch_angle = delta * self.pitch_sensitivity;
        let pitch_quat = Quat::from_axis_angle(self.rotation * Vec3::X, pitch_angle);
        
        let new_rot = (pitch_quat * self.rotation).normalize();
        
        let forward = new_rot * -Vec3::Z;
        let dot = forward.dot(Vec3::Y);
        
        if dot > -0.99 && dot < 0.99 {
            self.rotation = new_rot;
        }
    }

    pub fn get_view_matrix(&self) -> Mat4 {
        let eye = self.target + (self.rotation * Vec3::Z) * self.distance;
        let up = self.rotation * Vec3::Y;
        Mat4::look_at_rh(eye, self.target, up)
    }

    pub fn get_projection_matrix(&self, aspect_ratio: f32) -> Mat4 {
        Mat4::perspective_rh(std::f32::consts::FRAC_PI_4, aspect_ratio, 0.01, 200.0)
    }
}
