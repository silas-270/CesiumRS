//! # Hierarchical 6-DOF Camera System
//!
//! This module implements a decoupled, hierarchical 6-Degrees-of-Freedom (6-DOF) camera
//! system designed for GIS and flight tracking applications.
//!
//! ## Architecture Overview
//! The camera uses an anchor-local coordinate hierarchy to support three main tracking modes:
//! 1. **Free Mode**: Anchored to the Earth center (`Vec3::ZERO`, `Quat::IDENTITY`). Controls allow orbiting, zooming, and local pitching.
//! 2. **Tracking Mode**: Anchored to an aircraft's position and orientation. Controls allow orbiting and zooming relative to the plane.
//! 3. **Cockpit Mode**: Anchored to the aircraft. Local offset is zeroed; controls allow looking around in-place.
//!
//! Global state is derived dynamically:
//! `global_position = anchor_pos + (anchor_ori * local_pos)`
//! `global_orientation = anchor_ori * local_ori`
//!
//! ## Core APIs and Methods
//!
//! ### Mode Configuration / Anchoring
//! - `set_anchor(pos: Vec3, ori: Quat)`: Updates the parent reference frame.
//! - `set_local_transform(pos: Vec3, ori: Quat)`: Manually overrides local offset/rotation.
//! - `set_distance_clamp(min: f32, max: f32)`: Sets distance limits to avoid clipping.
//!
//! ### Movement & 6-DOF Operations
//! - `orbit_anchor(rotation: Quat)`: Orbits the camera around the anchor point.
//! - `rotate_local(rotation: Quat)`: Rotates the camera in-place (local roll, pitch, yaw).
//! - `translate_local(offset: Vec3)`: Translates camera along its local axes (handles bounds checks).
//!
//! ### Input Wrappers
//! - `zoom(delta: f32)`: Scales distance to earth exponentially (15% per unit).
//! - `pitch(delta: f32)`: Local X-axis look rotation.
//! - `begin_drag()`, `drag()`, `end_drag()`: Raycast-driven globe-dragging controls.
//!
//! ### Matrices & Spatial Queries
//! - `get_view_matrix() -> Mat4`: Right-handed view matrix mapping global space to camera space.
//! - `get_projection_matrix(aspect_ratio: f32) -> Mat4`: Dynamically adjusted near/far perspective matrix.
//! - `global_transform() -> (Vec3, Quat)`: Returns computed global position and orientation.
//! - `altitude() -> f32`: Returns current height above the scaled WGS84 surface.
//! - `screen_to_world_ray(screen_x, screen_y, w, h) -> (Vec3, Vec3)`: Projects screen coordinates to a 3D ray.

use glam::{Mat4, Quat, Vec3, Vec4};

pub struct Camera {
    // 1. Anchor Transform (The focal point / tracking target)
    pub anchor_pos: Vec3,
    pub anchor_ori: Quat,

    // 2. Local Transform (Offset & rotation relative to the anchor)
    pub local_pos: Vec3,
    pub local_ori: Quat,

    // 3. Constraints
    pub min_distance: f32,
    pub max_distance: f32,

    pitch_sensitivity: f32,

    // Sticky Drag State
    drag_start_point: Option<Vec3>,
    drag_start_local_pos: Vec3,
    drag_start_local_ori: Quat,
}

impl Camera {
    pub fn new(position: Vec3, target: Vec3) -> Self {
        let mut cam = Self {
            anchor_pos: Vec3::ZERO,
            anchor_ori: Quat::IDENTITY,
            local_pos: position,
            local_ori: Quat::IDENTITY,
            min_distance: 6.378137 + 0.000002,
            max_distance: 6.378137 + 30.0,
            pitch_sensitivity: 0.05,
            drag_start_point: None,
            drag_start_local_pos: Vec3::ZERO,
            drag_start_local_ori: Quat::IDENTITY,
        };
        cam.set_eye(position, target);
        cam
    }

    pub fn set_eye(&mut self, eye: Vec3, target: Vec3) {
        self.local_pos = eye;
        let dir = (target - eye).normalize_or_zero();
        if dir.length_squared() > 0.0001 {
            let view = Mat4::look_at_rh(eye, target, Vec3::Y);
            self.local_ori = Quat::from_mat4(&view.inverse()).normalize();
        }
    }

    /// Computes the absolute global state of the camera.
    pub fn global_transform(&self) -> (Vec3, Quat) {
        let global_pos = self.anchor_pos + (self.anchor_ori * self.local_pos);
        let global_ori = self.anchor_ori * self.local_ori;
        (global_pos, global_ori)
    }

    // --- CLEAN API: Hierarchical Control ---

    pub fn set_anchor(&mut self, pos: Vec3, ori: Quat) {
        self.anchor_pos = pos;
        self.anchor_ori = ori;
    }

    pub fn set_local_transform(&mut self, pos: Vec3, ori: Quat) {
        self.local_pos = pos;
        self.local_ori = ori;
        self.enforce_bounds();
    }

    pub fn set_distance_clamp(&mut self, min: f32, max: f32) {
        self.min_distance = min;
        self.max_distance = max;
        self.enforce_bounds();
    }

    pub fn orbit_anchor(&mut self, rotation: Quat) {
        self.local_pos = rotation * self.local_pos;
        self.local_ori = (rotation * self.local_ori).normalize();
    }

    pub fn rotate_local(&mut self, rotation: Quat) {
        self.local_ori = (self.local_ori * rotation).normalize();
    }

    pub fn translate_local(&mut self, offset: Vec3) {
        self.local_pos += self.local_ori * offset;
        self.enforce_bounds();
    }

    fn enforce_bounds(&mut self) {
        let (global_pos, _) = self.global_transform();
        let dist = global_pos.length();

        let dir = global_pos.normalize_or_zero();
        let a = 6.378137_f32;
        let b = 6.3567523142_f32;
        let inv_a2 = 1.0 / (a * a);
        let inv_b2 = 1.0 / (b * b);
        let t =
            1.0 / (dir.x * dir.x * inv_a2 + dir.y * dir.y * inv_b2 + dir.z * dir.z * inv_a2).sqrt();
        let dynamic_min_distance = t + 0.000002;

        if dist < dynamic_min_distance {
            let new_global_pos = global_pos.normalize_or_zero() * dynamic_min_distance;
            self.local_pos = self.anchor_ori.inverse() * (new_global_pos - self.anchor_pos);
        } else if dist > self.max_distance {
            let new_global_pos = global_pos.normalize_or_zero() * self.max_distance;
            self.local_pos = self.anchor_ori.inverse() * (new_global_pos - self.anchor_pos);
        }
    }

    // --- CONVENIENCE INPUT WRAPPERS ---

    pub fn pitch(&mut self, delta: f32) {
        if delta == 0.0 {
            return;
        }
        let pitch_angle = delta * self.pitch_sensitivity;

        // Rotate around local X axis
        let pitch_quat = Quat::from_axis_angle(Vec3::X, pitch_angle);
        self.rotate_local(pitch_quat);
    }

    pub fn zoom(&mut self, delta: f32) {
        if delta == 0.0 {
            return;
        }

        let altitude = self.altitude();
        let speed_alt = altitude.max(0.000002);
        let move_distance = speed_alt * 0.15 * delta;

        let forward = -Vec3::Z; // Translate local expects local offset.
        self.translate_local(forward * move_distance);
    }

    // --- MATRICES & PROJECTIONS ---

    pub fn get_view_matrix(&self) -> Mat4 {
        let (pos, ori) = self.global_transform();
        Mat4::from_rotation_translation(ori, pos).inverse()
    }

    pub fn altitude(&self) -> f32 {
        let (pos, _) = self.global_transform();

        let a = 6.378137_f32;
        let b = 6.3567523142_f32;
        let inv_a2 = 1.0 / (a * a);
        let inv_b2 = 1.0 / (b * b);

        let dir = pos.normalize_or_zero();
        let t =
            1.0 / (dir.x * dir.x * inv_a2 + dir.y * dir.y * inv_b2 + dir.z * dir.z * inv_a2).sqrt();

        pos.length() - t
    }

    pub fn get_projection_matrix(&self, aspect_ratio: f32) -> Mat4 {
        let alt = self.altitude().max(0.000002);
        let znear = (alt * 0.1).clamp(0.0000001, 10.0);
        let (pos, _) = self.global_transform();
        let zfar = pos.length() + 10.0;
        Mat4::perspective_rh(std::f32::consts::FRAC_PI_4, aspect_ratio, znear, zfar)
    }

    // --- RAYCASTING & DRAGGING (Earth Free Mode) ---

    pub fn screen_to_world_ray(
        &self,
        screen_x: f32,
        screen_y: f32,
        screen_width: f32,
        screen_height: f32,
    ) -> (Vec3, Vec3) {
        let aspect_ratio = screen_width / screen_height;

        let ndc_x = (2.0 * screen_x) / screen_width - 1.0;
        let ndc_y = 1.0 - (2.0 * screen_y) / screen_height;

        let ndc_far = Vec4::new(ndc_x, ndc_y, 1.0, 1.0);

        let view_proj = self.get_projection_matrix(aspect_ratio) * self.get_view_matrix();
        let inv_view_proj = view_proj.inverse();

        let mut world_far = inv_view_proj * ndc_far;
        world_far /= world_far.w;

        let ray_origin = self.global_transform().0;
        let ray_dir = (world_far.truncate() - ray_origin).normalize();

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

    pub fn begin_drag(
        &mut self,
        screen_x: f32,
        screen_y: f32,
        screen_width: f32,
        screen_height: f32,
    ) {
        let (ray_origin, ray_dir) =
            self.screen_to_world_ray(screen_x, screen_y, screen_width, screen_height);
        let earth_radius = 6.378137;
        self.drag_start_point = self.intersect_sphere(ray_origin, ray_dir, earth_radius);
        self.drag_start_local_pos = self.local_pos;
        self.drag_start_local_ori = self.local_ori;
    }

    pub fn drag(&mut self, screen_x: f32, screen_y: f32, screen_width: f32, screen_height: f32) {
        if let Some(start_point) = self.drag_start_point {
            let current_pos = self.local_pos;
            let current_ori = self.local_ori;

            // Revert to start state to compute the accurate single-gesture ray
            self.local_pos = self.drag_start_local_pos;
            self.local_ori = self.drag_start_local_ori;

            let (ray_origin, ray_dir) =
                self.screen_to_world_ray(screen_x, screen_y, screen_width, screen_height);
            let earth_radius = 6.378137;

            if let Some(current_point) = self.intersect_sphere(ray_origin, ray_dir, earth_radius) {
                // The drag orbits the earth, so we orbit the anchor
                let rot_delta =
                    Quat::from_rotation_arc(start_point.normalize(), current_point.normalize());
                let inv_rot = rot_delta.inverse();

                // Get the starting global transform
                let start_global_pos =
                    self.anchor_pos + (self.anchor_ori * self.drag_start_local_pos);
                let start_global_ori = self.anchor_ori * self.drag_start_local_ori;

                // Orbit the camera around the earth (origin)
                let new_global_pos = inv_rot * start_global_pos;
                let new_global_ori = inv_rot * start_global_ori;

                // Project back into anchor-local space
                self.local_pos = self.anchor_ori.inverse() * (new_global_pos - self.anchor_pos);
                self.local_ori = (self.anchor_ori.inverse() * new_global_ori).normalize();
            } else {
                // If ray doesn't intersect anymore, retain the current position
                self.local_pos = current_pos;
                self.local_ori = current_ori;
            }
        }
    }

    pub fn end_drag(&mut self) {
        self.drag_start_point = None;
    }
}
