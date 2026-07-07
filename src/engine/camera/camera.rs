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

use glam::{Mat4, Quat, Vec3};

const INV_A2_F64: f64 = 1.0 / (crate::engine::globe::geometry::EARTH_RADIUS_A_F64 * crate::engine::globe::geometry::EARTH_RADIUS_A_F64);
const INV_B2_F64: f64 = 1.0 / (crate::engine::globe::geometry::EARTH_RADIUS_B_F64 * crate::engine::globe::geometry::EARTH_RADIUS_B_F64);

const EARTH_RADIUS_A_F64: f64 = 6.378137;
const EARTH_RADIUS_B_F64: f64 = 6.3567523142;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CameraMode {
    Free,
    Tracking,
    Cockpit,
}

pub struct Camera {
    // 1. Anchor Transform (The focal point / tracking target)
    pub anchor_pos: glam::DVec3,
    pub anchor_ori: glam::DQuat,

    // 2. Local Transform (Offset & rotation relative to the anchor)
    pub local_pos: Vec3,
    pub local_ori: Quat,

    // 3. Constraints
    pub min_distance: f32,
    pub max_distance: f32,

    pub pitch_sensitivity: f32,
    pub sun_intensity: f32,

    // Sticky Drag State
    drag_start_point: Option<glam::DVec3>,
    drag_start_local_pos: Vec3,
    drag_start_local_ori: Quat,

    pub focal_length: f32, // Camera Lens focal length in mm (assuming 24mm vertical sensor height)
    pub mode: CameraMode,
}

impl Camera {
    pub fn new(position: Vec3, target: Vec3) -> Self {
        let mut cam = Self {
            anchor_pos: glam::DVec3::ZERO,
            anchor_ori: glam::DQuat::IDENTITY,
            local_pos: position,
            local_ori: Quat::IDENTITY,
            min_distance: 6.378137 + 0.000002,
            max_distance: 6.378137 + 30.0,
            pitch_sensitivity: 0.05,
            drag_start_point: None,
            drag_start_local_pos: Vec3::ZERO,
            drag_start_local_ori: Quat::IDENTITY,
            focal_length: 28.0,
            sun_intensity: 1.0,
            mode: CameraMode::Free,
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

    /// Computes the absolute global state of the camera in double precision.
    pub fn global_transform_f64(&self) -> (glam::DVec3, glam::DQuat) {
        let local_pos_dvec = glam::DVec3::new(self.local_pos.x as f64, self.local_pos.y as f64, self.local_pos.z as f64);
        let local_ori_dquat = glam::DQuat::from_xyzw(self.local_ori.x as f64, self.local_ori.y as f64, self.local_ori.z as f64, self.local_ori.w as f64);
        let global_pos = self.anchor_pos + (self.anchor_ori * local_pos_dvec);
        let global_ori = self.anchor_ori * local_ori_dquat;
        (global_pos, global_ori)
    }

    /// Computes the absolute global state of the camera in single precision.
    pub fn global_transform(&self) -> (Vec3, Quat) {
        let (pos_dvec, ori_dquat) = self.global_transform_f64();
        (
            Vec3::new(pos_dvec.x as f32, pos_dvec.y as f32, pos_dvec.z as f32),
            Quat::from_xyzw(ori_dquat.x as f32, ori_dquat.y as f32, ori_dquat.z as f32, ori_dquat.w as f32).normalize(),
        )
    }

    // --- CLEAN API: Hierarchical Control ---

    pub fn set_anchor(&mut self, pos: glam::DVec3, ori: glam::DQuat) {
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
        if self.mode == CameraMode::Tracking {
            let dist_to_plane = self.local_pos.length();
            if dist_to_plane < 0.00002 {
                if dist_to_plane > 1e-8 {
                    self.local_pos = (self.local_pos / dist_to_plane) * 0.00002;
                } else {
                    self.local_pos = Vec3::new(0.0, 0.0, 0.00002);
                }
            }
        }

        let (global_pos_dvec, _) = self.global_transform_f64();
        let dist = global_pos_dvec.length();

        let dir = global_pos_dvec.normalize_or_zero();
        let t =
            1.0 / (dir.x * dir.x * INV_A2_F64 + dir.y * dir.y * INV_B2_F64 + dir.z * dir.z * INV_A2_F64).sqrt();
        let dynamic_min_distance = t + 0.000002;

        if dist < dynamic_min_distance {
            let new_global_pos_dvec = dir * dynamic_min_distance;
            let local_pos_dvec = self.anchor_ori.inverse() * (new_global_pos_dvec - self.anchor_pos);
            self.local_pos = Vec3::new(local_pos_dvec.x as f32, local_pos_dvec.y as f32, local_pos_dvec.z as f32);
        } else if dist > self.max_distance as f64 {
            let new_global_pos_dvec = dir * (self.max_distance as f64);
            let local_pos_dvec = self.anchor_ori.inverse() * (new_global_pos_dvec - self.anchor_pos);
            self.local_pos = Vec3::new(local_pos_dvec.x as f32, local_pos_dvec.y as f32, local_pos_dvec.z as f32);
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

        let speed = match self.mode {
            CameraMode::Tracking => {
                let dist_to_plane = self.local_pos.length();
                dist_to_plane.max(0.00002) // 20 meters threshold
            }
            _ => {
                let altitude = self.altitude();
                altitude.max(0.000002) // 2 meters threshold
            }
        };
        let move_distance = speed * 0.15 * delta;

        let forward = -Vec3::Z; // Translate local expects local offset.
        self.translate_local(forward * move_distance);
    }

    pub fn orbit_mouse(&mut self, dx: f32, dy: f32) {
        let yaw = -dx * self.pitch_sensitivity * 0.2;
        let pitch = dy * self.pitch_sensitivity * 0.2;

        let right = Vec3::Y.cross(-self.local_pos).normalize_or_zero();
        if right.length_squared() > 0.001 {
            let rot_yaw = Quat::from_axis_angle(Vec3::Y, yaw);
            let rot_pitch = Quat::from_axis_angle(right, pitch);
            
            let new_pos_both = (rot_yaw * rot_pitch) * self.local_pos;
            let new_pos_yaw = rot_yaw * self.local_pos;
            
            // Helper to check if a local_pos is above the ground
            let is_above_ground = |pos: Vec3| -> bool {
                let pos_dvec = glam::DVec3::new(pos.x as f64, pos.y as f64, pos.z as f64);
                let global_pos = self.anchor_pos + (self.anchor_ori * pos_dvec);
                let dir = global_pos.normalize_or_zero();
                let t = 1.0 / (dir.x * dir.x * INV_A2_F64 + dir.y * dir.y * INV_B2_F64 + dir.z * dir.z * INV_A2_F64).sqrt();
                global_pos.length() >= t + 0.000002
            };

            let dot_y_both = new_pos_both.normalize_or_zero().dot(Vec3::Y);
            
            let final_pos = if dot_y_both.abs() < 0.99 && is_above_ground(new_pos_both) {
                Some(new_pos_both)
            } else if new_pos_yaw.normalize_or_zero().dot(Vec3::Y).abs() < 0.99 && is_above_ground(new_pos_yaw) {
                Some(new_pos_yaw)
            } else {
                None
            };

            if let Some(pos) = final_pos {
                self.local_pos = pos;
                self.enforce_bounds();
                
                let forward = -self.local_pos.normalize_or_zero();
                if forward.length_squared() > 0.1 {
                    let actual_right = forward.cross(Vec3::Y).normalize_or_zero();
                    if actual_right.length_squared() > 0.1 {
                        let up = actual_right.cross(forward).normalize_or_zero();
                        let rot_mat = glam::Mat3::from_cols(actual_right, up, -forward);
                        self.local_ori = Quat::from_mat3(&rot_mat);
                    }
                }
            }
        }
    }

    pub fn look_around(&mut self, dx: f32, dy: f32) {
        let yaw = dx * self.pitch_sensitivity * 0.1;
        let pitch = dy * self.pitch_sensitivity * 0.1;

        let yaw_quat = Quat::from_axis_angle(Vec3::Y, yaw);
        let pitch_quat = Quat::from_axis_angle(Vec3::X, pitch);
        
        let new_ori = self.local_ori * yaw_quat * pitch_quat;
        
        let (y, p, _r) = new_ori.to_euler(glam::EulerRot::YXZ);
        
        let mut rel_y = y;
        while rel_y > std::f32::consts::PI { rel_y -= std::f32::consts::PI * 2.0; }
        while rel_y < -std::f32::consts::PI { rel_y += std::f32::consts::PI * 2.0; }
        
        let clamped_rel_y = rel_y.clamp(-std::f32::consts::FRAC_PI_4, std::f32::consts::FRAC_PI_4);
        
        let clamped_p = p.clamp(-0.35, 0.35); // roughly +/- 20 deg
        
        self.local_ori = Quat::from_euler(glam::EulerRot::YXZ, clamped_rel_y, clamped_p, 0.0).normalize();
    }

    // --- MATRICES & PROJECTIONS ---

    pub fn get_view_matrix(&self) -> Mat4 {
        let (pos_dvec, ori_dquat) = self.global_transform();
        let pos = glam::Vec3::new(pos_dvec.x as f32, pos_dvec.y as f32, pos_dvec.z as f32);
        let ori = glam::Quat::from_xyzw(ori_dquat.x as f32, ori_dquat.y as f32, ori_dquat.z as f32, ori_dquat.w as f32).normalize();
        Mat4::from_rotation_translation(ori, pos).inverse()
    }

    pub fn altitude(&self) -> f32 {
        let (pos_dvec, _) = self.global_transform_f64();

        let dir = pos_dvec.normalize_or_zero();
        let t =
            1.0 / (dir.x * dir.x * INV_A2_F64 + dir.y * dir.y * INV_B2_F64 + dir.z * dir.z * INV_A2_F64).sqrt();

        (pos_dvec.length() - t) as f32
    }

    pub fn get_projection_matrix(&self, aspect_ratio: f32) -> Mat4 {
        let alt = self.altitude().max(0.000002);
        let znear = match self.mode {
            CameraMode::Free => (alt * 0.1).clamp(0.0000001, 10.0),
            CameraMode::Tracking | CameraMode::Cockpit => {
                // In tracking or cockpit mode, the camera is anchored to the aircraft.
                // The aircraft and its immediate trajectory polyline are very close to the camera.
                // We must use a small znear to prevent clipping the aircraft or nearby polyline.
                // We scale znear with the local distance to the aircraft target, but keep it small.
                let dist = self.local_pos.length();
                (dist * 0.05).clamp(0.00000001, 0.000005)
            }
        };
        let (pos_dvec, _) = self.global_transform();
        let zfar = (pos_dvec.length() + 10.0) as f32;
        
        let sensor_height = 24.0;
        let fovy = 2.0 * (sensor_height / (2.0 * self.focal_length)).atan();
        let proj = Mat4::perspective_rh(fovy, aspect_ratio, znear, zfar);
        
        // Convert to Reverse-Z: map [0, 1] to [1, 0]
        let reverse_z = Mat4::from_cols_array(&[
            1.0, 0.0,  0.0, 0.0,
            0.0, 1.0,  0.0, 0.0,
            0.0, 0.0, -1.0, 0.0,
            0.0, 0.0,  1.0, 1.0,
        ]);
        reverse_z * proj
    }

    pub fn get_projection_matrix_f64(&self, aspect_ratio: f64) -> glam::DMat4 {
        let alt = self.altitude().max(0.000002) as f64;
        let znear = match self.mode {
            CameraMode::Free => (alt * 0.1).clamp(0.0000001, 10.0),
            CameraMode::Tracking | CameraMode::Cockpit => {
                let dist = self.local_pos.length() as f64;
                (dist * 0.05).clamp(0.00000001, 0.000005)
            }
        };
        let (pos_dvec, _) = self.global_transform_f64();
        let zfar = pos_dvec.length() + 10.0;
        
        let sensor_height = 24.0;
        let fovy = 2.0 * (sensor_height / (2.0 * self.focal_length as f64)).atan();
        let proj = glam::DMat4::perspective_rh(fovy, aspect_ratio, znear, zfar);
        
        // Convert to Reverse-Z: map [0, 1] to [1, 0]
        let reverse_z = glam::DMat4::from_cols_array(&[
            1.0, 0.0,  0.0, 0.0,
            0.0, 1.0,  0.0, 0.0,
            0.0, 0.0, -1.0, 0.0,
            0.0, 0.0,  1.0, 1.0,
        ]);
        reverse_z * proj
    }

    pub fn get_view_matrix_f64(&self) -> glam::DMat4 {
        let (pos_dvec, ori_dquat) = self.global_transform_f64();
        glam::DMat4::from_rotation_translation(ori_dquat, pos_dvec).inverse()
    }

    pub fn calculate_frustum_planes(&self, aspect_ratio: f32) -> [(glam::DVec3, f64); 6] {
        let vp = self.get_projection_matrix_f64(aspect_ratio as f64) * self.get_view_matrix_f64();
        let r0 = vp.row(0);
        let r1 = vp.row(1);
        let r2 = vp.row(2);
        let r3 = vp.row(3);

        let planes = [
            r3 + r0, // Left
            r3 - r0, // Right
            r3 + r1, // Bottom
            r3 - r1, // Top
            r3 + r2, // Near
            r3 - r2, // Far
        ];

        let mut result = [(glam::DVec3::ZERO, 0.0); 6];
        for i in 0..6 {
            let n = glam::DVec3::new(planes[i].x, planes[i].y, planes[i].z);
            let len = n.length();
            if len > 0.000001 {
                let norm = n / len;
                result[i] = (norm, planes[i].w / len);
            }
        }
        result
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

        let fov_y = std::f32::consts::FRAC_PI_4;
        let tan_half_fov = (fov_y / 2.0).tan();
        
        let local_dir = Vec3::new(
            ndc_x * aspect_ratio * tan_half_fov,
            ndc_y * tan_half_fov,
            -1.0,
        ).normalize();

        let (global_pos, global_ori) = self.global_transform();
        let ray_dir = global_ori * local_dir;

        (global_pos, ray_dir)
    }

    pub fn intersect_ellipsoid(&self, ray_origin: Vec3, ray_dir: Vec3) -> Option<glam::DVec3> {
        let ro = glam::DVec3::new(
            ray_origin.x as f64 / EARTH_RADIUS_A_F64,
            ray_origin.y as f64 / EARTH_RADIUS_B_F64,
            ray_origin.z as f64 / EARTH_RADIUS_A_F64,
        );
        let rd = glam::DVec3::new(
            ray_dir.x as f64 / EARTH_RADIUS_A_F64,
            ray_dir.y as f64 / EARTH_RADIUS_B_F64,
            ray_dir.z as f64 / EARTH_RADIUS_A_F64,
        );

        let qa = rd.length_squared();
        let qb = 2.0 * ro.dot(rd);
        let qc = ro.length_squared() - 1.0;

        let discriminant = qb * qb - 4.0 * qa * qc;
        if discriminant < 0.0 {
            return None;
        }

        let t = (-qb - discriminant.sqrt()) / (2.0 * qa);
        if t < 0.0 {
            return None;
        }

        Some(glam::DVec3::new(
            ray_origin.x as f64 + ray_dir.x as f64 * t,
            ray_origin.y as f64 + ray_dir.y as f64 * t,
            ray_origin.z as f64 + ray_dir.z as f64 * t,
        ))
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

        self.drag_start_point = self.intersect_ellipsoid(ray_origin, ray_dir);
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

            if let Some(current_point) = self.intersect_ellipsoid(ray_origin, ray_dir) {
                let start_f64 = start_point.normalize();
                let current_f64 = current_point.normalize();

                let dot = start_f64.dot(current_f64);
                let cross = start_f64.cross(current_f64);
                let q = glam::DQuat::from_xyzw(cross.x, cross.y, cross.z, 1.0 + dot).normalize();
                let inv_rot = q.inverse();

                // Get the starting global transform
                let start_local_pos_dvec = glam::DVec3::new(self.drag_start_local_pos.x as f64, self.drag_start_local_pos.y as f64, self.drag_start_local_pos.z as f64);
                let start_global_pos = self.anchor_pos + (self.anchor_ori * start_local_pos_dvec);
                
                let start_local_ori_dquat = glam::DQuat::from_xyzw(self.drag_start_local_ori.x as f64, self.drag_start_local_ori.y as f64, self.drag_start_local_ori.z as f64, self.drag_start_local_ori.w as f64);
                let start_global_ori = self.anchor_ori * start_local_ori_dquat;

                // Orbit the camera around the earth (origin)
                let new_global_pos = inv_rot * start_global_pos;
                let new_global_ori = inv_rot * start_global_ori;

                // Project back into anchor-local space
                let new_local_pos_dvec = self.anchor_ori.inverse() * (new_global_pos - self.anchor_pos);
                self.local_pos = Vec3::new(new_local_pos_dvec.x as f32, new_local_pos_dvec.y as f32, new_local_pos_dvec.z as f32);
                
                let new_local_ori_dquat = (self.anchor_ori.inverse() * new_global_ori).normalize();
                self.local_ori = Quat::from_xyzw(new_local_ori_dquat.x as f32, new_local_ori_dquat.y as f32, new_local_ori_dquat.z as f32, new_local_ori_dquat.w as f32).normalize();
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
            sensitivity: 0.002, // Reduced from 0.005
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
        // Calculate altitude above Earth's surface (approx radius 6.378137)
        let altitude = (self.position.length() - 6.378137).max(0.0001);
        
        // Scale speed dynamically: slower near surface, normal speed far away.
        let altitude_factor = altitude.clamp(0.0001, 1.0);

        let speed = if fast { self.fast_speed } else { self.base_speed };
        let dynamic_speed = speed * altitude_factor;
        let velocity = movement * dynamic_speed * dt;

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
        self.pitch += dy * self.sensitivity; // Inverted Y-axis

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

    pub fn global_transform_f64(&self) -> (glam::DVec3, glam::DQuat) {
        let (yaw_sin, yaw_cos) = self.yaw.sin_cos();
        let (pitch_sin, pitch_cos) = self.pitch.sin_cos();

        let forward = glam::DVec3::new(
            (pitch_cos * yaw_sin) as f64,
            pitch_sin as f64,
            -(pitch_cos * yaw_cos) as f64,
        ).normalize_or_zero();

        let rot = glam::DQuat::from_rotation_arc(glam::DVec3::Z, -forward);

        (
            glam::DVec3::new(self.position.x as f64, self.position.y as f64, self.position.z as f64),
            rot
        )
    }

    pub fn calculate_frustum_planes(&self, aspect_ratio: f32) -> [(glam::DVec3, f64); 6] {
        let vp = self.get_projection_matrix(aspect_ratio) * self.get_view_matrix();
        let r0 = vp.row(0);
        let r1 = vp.row(1);
        let r2 = vp.row(2);
        let r3 = vp.row(3);

        let planes = [
            r3 + r0, // Left
            r3 - r0, // Right
            r3 + r1, // Bottom
            r3 - r1, // Top
            r3 + r2, // Near
            r3 - r2, // Far
        ];

        let mut result = [(glam::DVec3::ZERO, 0.0); 6];
        for i in 0..6 {
            let n = glam::DVec3::new(planes[i].x as f64, planes[i].y as f64, planes[i].z as f64);
            let len = n.length();
            if len > 0.000001 {
                let norm = n / len;
                result[i] = (norm, (planes[i].w as f64) / len);
            }
        }
        result
    }
}

