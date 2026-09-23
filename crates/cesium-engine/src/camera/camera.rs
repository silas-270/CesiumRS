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

const INV_A2_F64: f64 =
    1.0 / (crate::globe::geometry::EARTH_RADIUS_A_F64 * crate::globe::geometry::EARTH_RADIUS_A_F64);
const INV_B2_F64: f64 =
    1.0 / (crate::globe::geometry::EARTH_RADIUS_B_F64 * crate::globe::geometry::EARTH_RADIUS_B_F64);

const EARTH_RADIUS_A_F64: f64 = 6.378137;
const EARTH_RADIUS_B_F64: f64 = 6.3567523142;

/// Near plane used in cockpit mode, in Megametres (5 cm).
///
/// The cockpit interior is modelled at true scale and wraps around the camera, with the
/// nearest surfaces only tens of centimetres away, so the distance-derived near plane used
/// by the other modes would clip the entire cabin. Reverse-Z keeps depth precision usable
/// at this range.
const COCKPIT_ZNEAR: f64 = 5e-8;

/// Vertical FOV used in cockpit mode, in radians (~60°), independent of `focal_length`.
///
/// The shared `focal_length` field yields ~46° vertical FOV at its default, which is a
/// "soda-straw" ~22° horizontal in portrait aspect ratios. A wider fixed FOV frames more
/// of the interior regardless of orientation, without affecting the debug-panel slider's
/// effect on Free/Tracking modes.
const COCKPIT_FOVY_RAD: f32 = 1.0472; // 60 degrees

/// Head-turn limits for [`Camera::look_around`], in radians (±100° yaw, ±34° pitch).
const LOOK_AROUND_MAX_YAW: f32 = 1.75;
const LOOK_AROUND_MAX_PITCH: f32 = 0.6;

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

    // Inertia & Kinetic Pan State
    pub inertia_active: bool,
    pub inertia_axis: glam::Vec3,
    pub inertia_velocity: f32,
    pub last_drag_time: std::time::Instant,

    /// Height of the terrain directly below the camera, in megametres above the
    /// ellipsoid — `None` whenever there is no terrain to speak of.
    ///
    /// Phase E3 (`docs/terrain-plan.md` §8). The camera cannot reach the height cache:
    /// it is constructed before the tile system, borrowed mutably by the extension, and
    /// used by tests that have no `WgpuState` at all. So the ground comes *to* it —
    /// `WgpuState::update_logic` samples it once per frame from
    /// `TileSystem::ground_height_at` and writes it here through
    /// [`set_ground_height`](Self::set_ground_height), and the camera reads a plain
    /// field.
    ///
    /// # `None` is the flat path, exactly
    ///
    /// `TileSystem::ground_height_at` returns `None` whenever
    /// `TerrainConfig::enabled` is false, so with terrain off this field is `None` on
    /// every frame of every run, and every consumer below takes a branch that is
    /// character-for-character the arithmetic it did before Phase E3. This is not "the
    /// same to within a rounding error"; it is the same expression on the same operands.
    ///
    /// Default `None`, so a `Camera` nobody feeds — the LOD harness, the culling tests,
    /// a headless pose — behaves exactly as it always did without opting out of
    /// anything.
    ground_height: Option<f32>,
}

/// Above this height above the **ellipsoid**, the camera stops testing itself against the
/// terrain. Megametres; 15 km.
///
/// Cesium's `ScreenSpaceCameraController.minimumCollisionTerrainHeight`, same value and
/// the same two reasons for a threshold rather than an unconditional test:
///
/// 1. **The sample is only as good as what has landed.** Far from the ground the deepest
///    resident ancestor is a z4-z6 tile and its height is a continental average. Enforcing
///    a floor against that would shove a cruising camera around by hundreds of metres as
///    tiles stream in and the average changes — a floor that moves is worse than no floor.
///    Below 15 km the camera is over ground it is drawing at depth, so the sample is the
///    real one.
/// 2. **15 km clears the planet.** Everest is 8 849 m; with `exaggeration` at its default
///    1.0 no terrain on Earth reaches the threshold, so nothing that could be collided
///    with is skipped.
///
/// Note the asymmetry with `exaggeration`: at an exaggeration high enough to lift a summit
/// past 15 km the camera could pass through it. That is a debug setting doing what a debug
/// setting does, and the alternative — scaling the threshold — would make the flat path's
/// constant depend on a terrain field.
pub const MIN_COLLISION_TERRAIN_HEIGHT: f32 = 0.015;

/// How far the camera is kept off whatever it is standing on, in megametres (2 m).
///
/// The same clearance `enforce_bounds` has always kept off the ellipsoid, reused verbatim
/// for the ground so that switching terrain on does not change how close the camera can
/// get to the surface — only *which* surface.
const SURFACE_CLEARANCE: f64 = 0.000002;

/// Largest spacing between the points tested along the line of sight from the tracking
/// camera to the aircraft, megametres (25 m), and the bounds on their number. A failing
/// pitch usually fails at the first few points, so the cost is mostly the one pitch
/// that passes.
const LOS_SPACING: f32 = 0.000025;
const LOS_MIN_SAMPLES: u32 = 12;
const LOS_MAX_SAMPLES: u32 = 96;
/// How far towards the aircraft the line of sight is tested, as a fraction of the
/// distance: the last stretch is the ground the aircraft itself stands on.
const LOS_MAX_T: f32 = 0.9;
/// Coarse step of the upward pitch scan in the tracking collision, radians (3°).
const PITCH_SCAN_STEP: f32 = 0.05236;

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
            mode: CameraMode::Tracking,
            inertia_active: false,
            inertia_axis: glam::Vec3::Y,
            inertia_velocity: 0.0,
            last_drag_time: std::time::Instant::now(),
            ground_height: None,
        };
        cam.set_eye(position, target);
        cam
    }

    pub fn set_eye(&mut self, eye: Vec3, target: Vec3) {
        self.set_eye_with_up(eye, target, Vec3::Y);
    }

    pub fn set_eye_with_up(&mut self, eye: Vec3, target: Vec3, up: Vec3) {
        self.local_pos = eye;
        let dir = (target - eye).normalize_or_zero();
        if dir.length_squared() > 0.0001 {
            let view = Mat4::look_at_rh(eye, target, up);
            self.local_ori = Quat::from_mat4(&view.inverse()).normalize();
        }
    }

    /// Computes the absolute global state of the camera in double precision.
    pub fn global_transform_f64(&self) -> (glam::DVec3, glam::DQuat) {
        let local_pos_dvec = glam::DVec3::new(
            self.local_pos.x as f64,
            self.local_pos.y as f64,
            self.local_pos.z as f64,
        );
        let local_ori_dquat = glam::DQuat::from_xyzw(
            self.local_ori.x as f64,
            self.local_ori.y as f64,
            self.local_ori.z as f64,
            self.local_ori.w as f64,
        );
        let global_pos = self.anchor_pos + (self.anchor_ori * local_pos_dvec);
        let global_ori = self.anchor_ori * local_ori_dquat;
        (global_pos, global_ori)
    }

    /// Computes the absolute global state of the camera in single precision.
    pub fn global_transform(&self) -> (Vec3, Quat) {
        let (pos_dvec, ori_dquat) = self.global_transform_f64();
        (
            Vec3::new(pos_dvec.x as f32, pos_dvec.y as f32, pos_dvec.z as f32),
            Quat::from_xyzw(
                ori_dquat.x as f32,
                ori_dquat.y as f32,
                ori_dquat.z as f32,
                ori_dquat.w as f32,
            )
            .normalize(),
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

    pub fn look_at_plane(&mut self) {
        let forward = -self.local_pos.normalize_or_zero();
        if forward.length_squared() < 0.001 {
            return;
        }
        // Right comes from the orbit yaw rather than from `forward × Y`, so the basis
        // stays continuous at steep pitch instead of rolling over when forward nears Y.
        let yaw = self.local_pos.x.atan2(self.local_pos.z);
        let right = Vec3::new(yaw.cos(), 0.0, -yaw.sin());
        let up = right.cross(forward).normalize_or_zero();
        self.local_ori = Quat::from_mat3(&glam::Mat3::from_cols(right, up, -forward));
    }

    /// Distance from the Earth's centre the camera may not come below at `global_pos`:
    /// the ellipsoid, or the ground `ground` reports **at that position**, plus clearance.
    fn floor_at(global_pos: glam::DVec3, ground: &dyn Fn(glam::DVec3) -> Option<f64>) -> f64 {
        let d = global_pos.length();
        let dir = global_pos.normalize_or_zero();
        let t = 1.0
            / (dir.x * dir.x * INV_A2_F64 + dir.y * dir.y * INV_B2_F64 + dir.z * dir.z * INV_A2_F64)
                .sqrt();
        let terrain = if d - t <= MIN_COLLISION_TERRAIN_HEIGHT as f64 {
            ground(global_pos).filter(|g| *g > 0.0).unwrap_or(0.0)
        } else {
            0.0
        };
        t + terrain + SURFACE_CLEARANCE
    }

    /// [`Self::enforce_bounds_with`] against the ellipsoid alone.
    pub fn enforce_bounds(&mut self) {
        self.enforce_bounds_with(&|_| None);
    }

    /// Keeps the camera within its distance limits and above the ground, sampling
    /// `ground` (height above the ellipsoid, megametres) at every position it tests.
    pub fn enforce_bounds_with(&mut self, ground: &dyn Fn(glam::DVec3) -> Option<f64>) {
        if self.mode == CameraMode::Cockpit {
            return;
        }

        if self.mode == CameraMode::Tracking {
            let mut dist = self.local_pos.length();
            if dist.is_nan() || dist < 1e-6 {
                dist = 250.0 / 1_000_000.0;
                self.local_pos = Vec3::new(0.0, dist * 0.3746, dist * 0.9271);
            } else if dist < 0.00002 {
                dist = 0.00002;
                self.local_pos = self.local_pos.normalize_or_zero() * dist;
            } else if dist > 30.0 {
                dist = 30.0;
                self.local_pos = self.local_pos.normalize_or_zero() * dist;
            }

            let cur_yaw = if self.local_pos.x.abs() > 1e-8 || self.local_pos.z.abs() > 1e-8 {
                self.local_pos.x.atan2(self.local_pos.z)
            } else {
                0.0
            };
            let cur_pitch = (self.local_pos.y / dist).clamp(-0.999, 0.999).asin();

            let anchor_dist = self.anchor_pos.length();
            let anchor_floor = if anchor_dist > 1.0 {
                Self::floor_at(self.anchor_pos, ground)
            } else {
                EARTH_RADIUS_A_F64 + SURFACE_CLEARANCE
            };

            let h_plane = (anchor_dist - anchor_floor) as f32;
            let min_sin_pitch = ((SURFACE_CLEARANCE as f32 - h_plane) / dist).clamp(-0.999, 0.999);
            let min_pitch = min_sin_pitch.asin();

            let test_pos = |p: f32| -> Vec3 {
                Vec3::new(
                    dist * p.cos() * cur_yaw.sin(),
                    dist * p.sin(),
                    dist * p.cos() * cur_yaw.cos(),
                )
            };

            let global = |local: Vec3| -> glam::DVec3 {
                self.anchor_pos
                    + self.anchor_ori * glam::DVec3::new(local.x as f64, local.y as f64, local.z as f64)
            };
            // Valid: the camera is clear of the ground under it, and the terrain does not
            // block its line of sight to the aircraft. The second half is what keeps the
            // aircraft on screen when the orbit swings behind a hill — without it the
            // camera sat in the valley behind the ridge, looking at grass.
            let los_samples = ((dist / LOS_SPACING).ceil() as u32).clamp(LOS_MIN_SAMPLES, LOS_MAX_SAMPLES);
            let valid = |local: Vec3| -> bool {
                let g = global(local);
                if g.length() < Self::floor_at(g, ground) {
                    return false;
                }
                (1..los_samples).all(|i| {
                    let t = i as f32 / los_samples as f32;
                    if t > LOS_MAX_T {
                        return true;
                    }
                    let q = global(local * (1.0 - t));
                    q.length() >= Self::floor_at(q, ground) - SURFACE_CLEARANCE
                })
            };

            if cur_pitch < min_pitch || !valid(self.local_pos) {
                let target_pitch = cur_pitch.max(min_pitch);
                let max_pitch = 85.0_f32.to_radians();
                // The lowest valid pitch at or above the one asked for: scan up in coarse
                // steps (a hill can make validity non-monotone in pitch, which a plain
                // bisection from the top would step over), then bisect the last step.
                let mut new_pitch = max_pitch;
                let mut below = target_pitch;
                let mut p = target_pitch;
                while p < max_pitch {
                    if valid(test_pos(p)) {
                        new_pitch = p;
                        break;
                    }
                    below = p;
                    p = (p + PITCH_SCAN_STEP).min(max_pitch);
                }
                if new_pitch > target_pitch {
                    let (mut lo, mut hi) = (below, new_pitch);
                    for _ in 0..10 {
                        let mid = (lo + hi) * 0.5;
                        if valid(test_pos(mid)) {
                            hi = mid;
                        } else {
                            lo = mid;
                        }
                    }
                    new_pitch = hi;
                }

                self.local_pos = test_pos(new_pitch);
                log::debug!(
                    "[CAMERA CLAMP TRACKING] lifted pitch from {:.1}° to {:.1}° (dist={:.1}m, h_plane={:.1}m)",
                    cur_pitch.to_degrees(),
                    new_pitch.to_degrees(),
                    dist * 1_000_000.0,
                    h_plane * 1_000_000.0
                );
                self.look_at_plane();
            }
            return;
        }

        let (global_pos_dvec, _) = self.global_transform_f64();
        let dist = global_pos_dvec.length();
        let dir = global_pos_dvec.normalize_or_zero();
        let dynamic_min_distance = Self::floor_at(global_pos_dvec, ground);

        if dist < dynamic_min_distance {
            log::info!(
                "[CAMERA CLAMP FREE] dist={:.7} < floor={:.7} -> clamped to surface",
                dist, dynamic_min_distance
            );
            let new_global_pos_dvec = dir * dynamic_min_distance;
            let local_pos_dvec =
                self.anchor_ori.inverse() * (new_global_pos_dvec - self.anchor_pos);
            self.local_pos = Vec3::new(
                local_pos_dvec.x as f32,
                local_pos_dvec.y as f32,
                local_pos_dvec.z as f32,
            );
        } else if dist > self.max_distance as f64 {
            let new_global_pos_dvec = dir * (self.max_distance as f64);
            let local_pos_dvec =
                self.anchor_ori.inverse() * (new_global_pos_dvec - self.anchor_pos);
            self.local_pos = Vec3::new(
                local_pos_dvec.x as f32,
                local_pos_dvec.y as f32,
                local_pos_dvec.z as f32,
            );
        }
    }

    // --- CONVENIENCE INPUT WRAPPERS ---

    pub fn pitch(&mut self, delta: f32) {
        if delta == 0.0 {
            return;
        }
        if self.mode == CameraMode::Cockpit {
            // The seat is fixed to the airframe; pitching is looking up and down.
            self.look_around(0.0, delta);
            return;
        }
        let pitch_angle = delta * self.pitch_sensitivity;

        // 1. Find distance to focus point (center of screen)
        let start_local_pos_dvec = glam::DVec3::new(
            self.local_pos.x as f64,
            self.local_pos.y as f64,
            self.local_pos.z as f64,
        );
        let start_global_pos = self.anchor_pos + (self.anchor_ori * start_local_pos_dvec);
        
        let ray_dir_local = self.local_ori * Vec3::new(0.0, 0.0, -1.0);
        let ray_dir_dvec = glam::DVec3::new(ray_dir_local.x as f64, ray_dir_local.y as f64, ray_dir_local.z as f64);
        let ray_dir_global = self.anchor_ori * ray_dir_dvec;

        let ray_origin = Vec3::new(start_global_pos.x as f32, start_global_pos.y as f32, start_global_pos.z as f32);
        let ray_dir = Vec3::new(ray_dir_global.x as f32, ray_dir_global.y as f32, ray_dir_global.z as f32);
        
        let focus_dist = if let Some(target_global) = self.intersect_ellipsoid(ray_origin, ray_dir) {
            (start_global_pos - target_global).length() as f32
        } else {
            self.altitude() // fallback if looking at space
        };

        // 2. Perform orbital pitch around the focus point
        let vec_focus_to_cam = Vec3::new(0.0, 0.0, focus_dist);
        let pitch_quat = Quat::from_axis_angle(Vec3::X, pitch_angle);
        let new_vec_focus_to_cam = pitch_quat * vec_focus_to_cam;
        
        let local_translation = new_vec_focus_to_cam - vec_focus_to_cam;
        let anchor_translation = self.local_ori * local_translation;
        self.local_pos += anchor_translation;
        
        self.rotate_local(pitch_quat);
        self.enforce_bounds();
    }

    pub fn roll(&mut self, delta: f32) {
        if delta == 0.0 {
            return;
        }
        // Rotate around local Z axis (roll)
        let roll_quat = Quat::from_axis_angle(Vec3::Z, delta);
        self.rotate_local(roll_quat);
    }

    pub fn set_local_pos(&mut self, pos: Vec3) {
        self.local_pos = pos;
        self.enforce_bounds();
    }

    pub fn zoom(&mut self, delta: f32) {
        if delta == 0.0 {
            return;
        }
        if self.mode == CameraMode::Cockpit {
            // The seat is rigidly attached to the airframe — zooming would slide the
            // camera out through the cockpit walls.
            return;
        }

        if self.mode == CameraMode::Tracking {
            let mut dist = self.local_pos.length();
            dist *= (1.0 - delta * 0.1).clamp(0.2, 5.0);
            dist = dist.clamp(0.00002, 30.0); // 20m to 30,000km
            let dir = self.local_pos.normalize_or_zero();
            self.local_pos = if dir.length_squared() > 0.001 {
                dir * dist
            } else {
                Vec3::new(0.0, 0.0, dist)
            };
            self.enforce_bounds();
            self.look_at_plane();
            log::info!("[CAMERA ZOOM TRACKING] delta={:.2} -> new_dist={:.1}m", delta, self.local_pos.length() * 1_000_000.0);
            return;
        }

        let speed = {
            let altitude = self.altitude();
            altitude.max(0.000002) // 2 meters threshold
        };
        let move_distance = speed * 0.15 * delta;

        let forward = -Vec3::Z; // Translate local expects local offset.
        self.translate_local(forward * move_distance);
        log::info!("[CAMERA ZOOM FREE] delta={:.2} -> new_alt={:.1}m", delta, self.altitude() * 1_000_000.0);
    }
    
    pub fn cancel_all_inertia(&mut self) {
        self.inertia_active = false;
        self.inertia_velocity = 0.0;
    }

    pub fn orbit_mouse(&mut self, dx: f32, dy: f32) {
        if self.mode != CameraMode::Tracking {
            return;
        }

        let mut dist = self.local_pos.length();
        if dist.is_nan() || dist < 1e-6 {
            dist = 250.0 / 1_000_000.0;
            self.local_pos = Vec3::new(0.0, dist * 0.3746, dist * 0.9271);
        } else if dist < 0.00002 {
            dist = 0.00002;
            self.local_pos = self.local_pos.normalize_or_zero() * dist;
        } else if dist > 30.0 {
            dist = 30.0;
            self.local_pos = self.local_pos.normalize_or_zero() * dist;
        }

        let cur_yaw = if self.local_pos.x.abs() > 1e-8 || self.local_pos.z.abs() > 1e-8 {
            self.local_pos.x.atan2(self.local_pos.z)
        } else {
            0.0
        };
        let cur_pitch = (self.local_pos.y / dist).clamp(-0.999, 0.999).asin();

        let delta_yaw = -dx * self.pitch_sensitivity * 0.2;
        let delta_pitch = dy * self.pitch_sensitivity * 0.2;

        let new_yaw = cur_yaw + delta_yaw;
        let target_pitch = (cur_pitch + delta_pitch).clamp(-80.0_f32.to_radians(), 85.0_f32.to_radians());

        let test_pos = |p: f32, y: f32| -> Vec3 {
            Vec3::new(
                dist * p.cos() * y.sin(),
                dist * p.sin(),
                dist * p.cos() * y.cos(),
            )
        };

        // Ground collision is not decided here: `enforce_bounds_with`, run once per frame
        // with the real ground under each position it tests, lifts the pitch if needed.
        let new_pitch = target_pitch;

        self.local_pos = test_pos(new_pitch, new_yaw);
        self.look_at_plane();

        log::info!(
            "[CAMERA ORBIT] dx={:.1} dy={:.1} | cur(pitch={:.1}°, yaw={:.1}°) -> new(pitch={:.1}°, yaw={:.1}°) | dist={:.1}m",
            dx, dy,
            cur_pitch.to_degrees(), cur_yaw.to_degrees(),
            new_pitch.to_degrees(), new_yaw.to_degrees(),
            dist * 1_000_000.0
        );
    }

    pub fn look_around(&mut self, dx: f32, dy: f32) {
        log::info!("[CAMERA LOOK_AROUND] dx={:.1} dy={:.1}", dx, dy);
        let yaw = dx * self.pitch_sensitivity * 0.1;
        let pitch = dy * self.pitch_sensitivity * 0.1;

        let yaw_quat = Quat::from_axis_angle(Vec3::Y, yaw);
        let pitch_quat = Quat::from_axis_angle(Vec3::X, pitch);

        let new_ori = self.local_ori * yaw_quat * pitch_quat;

        let (y, p, _r) = new_ori.to_euler(glam::EulerRot::YXZ);

        let mut rel_y = y;
        while rel_y > std::f32::consts::PI {
            rel_y -= std::f32::consts::PI * 2.0;
        }
        while rel_y < -std::f32::consts::PI {
            rel_y += std::f32::consts::PI * 2.0;
        }

        // Wide enough to turn and look out of the side windows, but short of spinning
        // the head all the way round.
        let clamped_rel_y = rel_y.clamp(-LOOK_AROUND_MAX_YAW, LOOK_AROUND_MAX_YAW);

        let clamped_p = p.clamp(-LOOK_AROUND_MAX_PITCH, LOOK_AROUND_MAX_PITCH);

        self.local_ori =
            Quat::from_euler(glam::EulerRot::YXZ, clamped_rel_y, clamped_p, 0.0).normalize();
    }

    // --- MATRICES & PROJECTIONS ---

    pub fn get_view_matrix(&self) -> Mat4 {
        let (pos_dvec, ori_dquat) = self.global_transform();
        let pos = glam::Vec3::new(pos_dvec.x, pos_dvec.y, pos_dvec.z);
        let ori =
            glam::Quat::from_xyzw(ori_dquat.x, ori_dquat.y, ori_dquat.z, ori_dquat.w).normalize();
        Mat4::from_rotation_translation(ori, pos).inverse()
    }

    /// Height above the **ellipsoid**, in megametres.
    ///
    /// Left exactly as it was, deliberately. Several things genuinely want the distance
    /// to the reference surface and not to the ground: the fog density (an atmospheric
    /// depth, and its consumer in `apply_lod` is under review in §7b/§7c), the label
    /// zoom bucket (a map scale), and `TerrainHorizon::begin`'s own altitude gate, which
    /// D3 calibrated against this quantity. What wanted the ground all along is
    /// [`altitude_agl`](Self::altitude_agl).
    pub fn altitude(&self) -> f32 {
        let (pos_dvec, _) = self.global_transform_f64();

        let dir = pos_dvec.normalize_or_zero();
        let t = 1.0
            / (dir.x * dir.x * INV_A2_F64
                + dir.y * dir.y * INV_B2_F64
                + dir.z * dir.z * INV_A2_F64)
                .sqrt();

        (pos_dvec.length() - t) as f32
    }

    /// Height above the **ground**, in megametres: [`altitude`](Self::altitude) minus the
    /// terrain height below the camera, or exactly [`altitude`](Self::altitude) when no
    /// terrain height is known.
    ///
    /// The quantity `znear` always meant. Flying up the Inn valley at 900 m the old
    /// number is 900 m and this one is ~330 m; standing at the foot of the Nordkette the
    /// old number is unchanged while the wall 400 m ahead is well inside a near plane
    /// chosen as `0.1 × 900 m = 90 m` — which is why near terrain clipped. The rock is
    /// at the distance this function measures, not the other one.
    ///
    /// Clamped at zero: a camera below the sampled ground (it happens — the sample is
    /// bilinear over a 30 m post spacing, the drawn mesh is a 16×16 patch of it, and they
    /// disagree by metres) would otherwise ask for a negative near plane.
    ///
    /// **With terrain off this is `altitude()`**, the same call, not a recomputation of
    /// it — see [`ground_height`](Self::set_ground_height).
    pub fn altitude_agl(&self) -> f32 {
        match self.ground_height {
            Some(ground) => (self.altitude() - ground).max(0.0),
            None => self.altitude(),
        }
    }

    /// The terrain height below the camera as last sampled, in megametres above the
    /// ellipsoid. `None` when terrain is off or nothing has arrived.
    pub fn ground_height(&self) -> Option<f32> {
        self.ground_height
    }

    /// Hands the camera this frame's terrain height below it, in megametres above the
    /// ellipsoid.
    ///
    /// Called once per frame by `WgpuState::update_logic` with
    /// `TileSystem::ground_height_at(camera_position)`, which is `None` whenever terrain
    /// is off. Passing `None` restores the pre-Phase-E3 camera exactly, which is what
    /// running flat does on every frame.
    ///
    /// A *rising* floor is enforced immediately: a camera parked in a valley when the
    /// z15 tile under it finally arrives, or panned into a hillside a frame ago, is
    /// pushed out by [`enforce_bounds`] here rather than on the next input event. The
    /// call is skipped entirely when the new value is `None`, so on the flat path this
    /// setter is one field write per frame and `enforce_bounds` runs exactly where it
    /// always ran.
    pub fn set_ground_height(&mut self, ground_height: Option<f32>) {
        self.ground_height = ground_height;
    }

    /// Vertical field of view, in radians — the one the projection matrix uses.
    ///
    /// Free and Tracking derive it from the virtual 35 mm camera
    /// (`2·atan(sensor_height / (2·focal_length))`, `24 mm` sensor); Cockpit is a
    /// fixed 60°. Exposed because the LOD rule needs it too — see
    /// [`lod_factor_for`](crate::globe::quadtree::lod_factor_for) — and a second
    /// hand-copied `atan` would be a silent way for culling and rendering to disagree
    /// about the frustum.
    pub fn fovy(&self) -> f32 {
        let sensor_height = 24.0;
        match self.mode {
            CameraMode::Cockpit => COCKPIT_FOVY_RAD,
            _ => 2.0 * (sensor_height / (2.0 * self.focal_length)).atan(),
        }
    }

    /// [`fovy`](Self::fovy) evaluated in f64.
    ///
    /// Not `fovy() as f64`: the f64 projection matrix has always computed its `atan`
    /// at f64 precision, and narrowing it through f32 would perturb every f64
    /// projection (the LOD harness projects its patches with exactly this matrix).
    pub fn fovy_f64(&self) -> f64 {
        let sensor_height = 24.0_f64;
        match self.mode {
            CameraMode::Cockpit => COCKPIT_FOVY_RAD as f64,
            _ => 2.0 * (sensor_height / (2.0 * self.focal_length as f64)).atan(),
        }
    }

    pub fn get_projection_matrix(&self, aspect_ratio: f32) -> Mat4 {
        // Phase E3.2: clearance over the *ground*, which is what the near plane was
        // always trying to express. `altitude_agl()` is `altitude()` with terrain off.
        let alt = self.altitude_agl().max(0.000002);
        let znear = match self.mode {
            CameraMode::Free => (alt * 0.1).clamp(0.0000001, 10.0),
            CameraMode::Tracking => {
                // In tracking mode the camera is anchored to the aircraft, and the aircraft
                // and its immediate trajectory polyline are very close to the camera.
                // We must use a small znear to prevent clipping the aircraft or nearby polyline.
                // We scale znear with the local distance to the aircraft target, but keep it small.
                let dist = self.local_pos.length();
                (dist * 0.05).clamp(0.00000001, 0.000005)
            }
            CameraMode::Cockpit => COCKPIT_ZNEAR as f32,
        };
        let (pos_dvec, _) = self.global_transform();
        let zfar = pos_dvec.length() + 10.0;

        let proj = Mat4::perspective_rh(self.fovy(), aspect_ratio, znear, zfar);

        // Convert to Reverse-Z: map [0, 1] to [1, 0]
        let reverse_z = Mat4::from_cols_array(&[
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0, 1.0,
        ]);
        reverse_z * proj
    }

    pub fn get_projection_matrix_f64(&self, aspect_ratio: f64) -> glam::DMat4 {
        // The f32 matrix's `alt`, to the bit — this is the culling frustum and it must
        // not disagree with the drawn one about where the near plane is.
        let alt = self.altitude_agl().max(0.000002) as f64;
        let znear = match self.mode {
            CameraMode::Free => (alt * 0.1).clamp(0.0000001, 10.0),
            CameraMode::Tracking => {
                let dist = self.local_pos.length() as f64;
                (dist * 0.05).clamp(0.00000001, 0.000005)
            }
            CameraMode::Cockpit => COCKPIT_ZNEAR,
        };
        let (pos_dvec, _) = self.global_transform_f64();
        let zfar = pos_dvec.length() + 10.0;

        let proj = glam::DMat4::perspective_rh(self.fovy_f64(), aspect_ratio, znear, zfar);

        // Convert to Reverse-Z: map [0, 1] to [1, 0]
        let reverse_z = glam::DMat4::from_cols_array(&[
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0, 1.0,
        ]);
        reverse_z * proj
    }

    pub fn get_view_matrix_f64(&self) -> glam::DMat4 {
        let (pos_dvec, ori_dquat) = self.global_transform_f64();
        glam::DMat4::from_rotation_translation(ori_dquat, pos_dvec).inverse()
    }

    /// The four **side** planes of the view frustum, as inward-pointing f64 unit
    /// normals in the order `[Left, Right, Bottom, Top]`.
    ///
    /// No plane offsets are returned, because there are none to return: all four
    /// side planes pass through the eye, so in the camera-relative frame the culling
    /// code works in, `d ≡ 0` by construction. See
    /// `globe::quadtree::bounding_volume` and `docs/culling-math.md` §2.5.
    ///
    /// # Why the depth planes are gone
    ///
    /// The wgpu clip volume is `−w ≤ x,y ≤ w`, `0 ≤ z ≤ w`, so under the reverse-Z
    /// remap the depth constraints extract as `r3 − r2` (**near**) and `r2`
    /// (**far**) — *not* the OpenGL `r3 ± r2` pair this function used to emit. The
    /// old index 5 ("Far") was really the near plane, the old index 4 ("Near") was a
    /// plane sitting `≈ znear` **behind** the eye that bounded nothing (it cleared
    /// the frustum hull by 2.86 Mm on the harness's reference camera), and the real
    /// far plane was never extracted at all.
    ///
    /// Rather than fix the pair, tile culling drops both, because both are vacuous
    /// for the globe and the near plane is actively harmful:
    ///
    /// * far — `zfar = ‖cam‖ + 10 Mm` exceeds `‖cam‖ + a`, so no ellipsoid point is
    ///   ever beyond it (invariant **I-3**: tighten `zfar` and `π_far = r2` must
    ///   come back);
    /// * near — vacuous whenever `znear < altitude`, and below that it is what
    ///   blanked the globe in Tracking mode at 5 m, rejecting a z=17 tile on
    ///   0.058 m of true clearance computed from 6.378 Mm operands in f32;
    /// * nothing behind the eye survives regardless: `πL + πR = −2·z_eye ≥ 0`.
    ///
    /// `render::debug_geometry::get_frustum_corners` still draws all six faces of
    /// the frustum; it works from the inverse view-projection, not from here.
    pub fn calculate_frustum_planes(&self, aspect_ratio: f32) -> [glam::DVec3; 4] {
        let vp = self.get_projection_matrix_f64(aspect_ratio as f64) * self.get_view_matrix_f64();
        let r0 = vp.row(0);
        let r1 = vp.row(1);
        let r3 = vp.row(3);

        let planes = [
            r3 + r0, // Left    (x_c ≥ −w_c)
            r3 - r0, // Right   (x_c ≤ +w_c)
            r3 + r1, // Bottom  (y_c ≥ −w_c)
            r3 - r1, // Top     (y_c ≤ +w_c)
        ];

        let mut result = [glam::DVec3::ZERO; 4];
        for i in 0..4 {
            let n = glam::DVec3::new(planes[i].x, planes[i].y, planes[i].z);
            let len = n.length();
            if len > 0.000001 {
                result[i] = n / len;
            }
        }
        result
    }

    /// The eight frustum corners, expressed **relative to the eye**.
    ///
    /// Order: near quad `(-1,-1), (1,-1), (1,1), (-1,1)` then the far quad, matching
    /// `render::debug_geometry::get_frustum_corners`. Reverse-Z, so `ndc.z = 1` is
    /// near and `ndc.z = 0` is far.
    ///
    /// Only the optional box-slab stage (`globe::quadtree::slab`) needs these; the
    /// four-plane test does not. The unprojection is f64 and the subtraction of the
    /// eye happens before the downcast.
    pub fn frustum_corners_relative(&self, aspect_ratio: f32) -> [Vec3; 8] {
        let vp = self.get_projection_matrix_f64(aspect_ratio as f64) * self.get_view_matrix_f64();
        let inv = vp.inverse();
        let (eye, _) = self.global_transform_f64();

        let ndc = [
            glam::DVec3::new(-1.0, -1.0, 1.0),
            glam::DVec3::new(1.0, -1.0, 1.0),
            glam::DVec3::new(1.0, 1.0, 1.0),
            glam::DVec3::new(-1.0, 1.0, 1.0),
            glam::DVec3::new(-1.0, -1.0, 0.0),
            glam::DVec3::new(1.0, -1.0, 0.0),
            glam::DVec3::new(1.0, 1.0, 0.0),
            glam::DVec3::new(-1.0, 1.0, 0.0),
        ];

        let mut out = [Vec3::ZERO; 8];
        for i in 0..8 {
            let h = inv * ndc[i].extend(1.0);
            let p = h.truncate() / h.w;
            let d = p - eye;
            out[i] = Vec3::new(d.x as f32, d.y as f32, d.z as f32);
        }
        out
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
        )
        .normalize();

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
        self.inertia_active = false;
        self.inertia_velocity = 0.0;
        self.last_drag_time = std::time::Instant::now();

        let (ray_origin, ray_dir) =
            self.screen_to_world_ray(screen_x, screen_y, screen_width, screen_height);

        let mut drag_point = self.intersect_ellipsoid(ray_origin, ray_dir);
        if drag_point.is_none() {
            let t = -ray_origin.dot(ray_dir);
            if t > 0.0 {
                let p_close = ray_origin + t * ray_dir;
                let dir = p_close.normalize_or_zero();
                let dir_f64 = glam::DVec3::new(dir.x as f64, dir.y as f64, dir.z as f64);
                let t_ellipsoid = 1.0
                    / (dir_f64.x * dir_f64.x * INV_A2_F64
                        + dir_f64.y * dir_f64.y * INV_B2_F64
                        + dir_f64.z * dir_f64.z * INV_A2_F64)
                        .sqrt();
                drag_point = Some(dir_f64 * t_ellipsoid);
            }
        }

        self.drag_start_point = drag_point;
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

            let mut current_point = self.intersect_ellipsoid(ray_origin, ray_dir);
            if current_point.is_none() {
                let t = -ray_origin.dot(ray_dir);
                if t > 0.0 {
                    let p_close = ray_origin + t * ray_dir;
                    let dir = p_close.normalize_or_zero();
                    let dir_f64 = glam::DVec3::new(dir.x as f64, dir.y as f64, dir.z as f64);
                    let t_ellipsoid = 1.0
                        / (dir_f64.x * dir_f64.x * INV_A2_F64
                            + dir_f64.y * dir_f64.y * INV_B2_F64
                            + dir_f64.z * dir_f64.z * INV_A2_F64)
                            .sqrt();
                    current_point = Some(dir_f64 * t_ellipsoid);
                }
            }

            if let Some(current_point) = current_point {
                let start_f64 = start_point.normalize();
                let current_f64 = current_point.normalize();

                let dot = start_f64.dot(current_f64);
                let cross = start_f64.cross(current_f64);
                let q = glam::DQuat::from_xyzw(cross.x, cross.y, cross.z, 1.0 + dot).normalize();
                let inv_rot = q.inverse();

                // Get the starting global transform
                let start_local_pos_dvec = glam::DVec3::new(
                    self.drag_start_local_pos.x as f64,
                    self.drag_start_local_pos.y as f64,
                    self.drag_start_local_pos.z as f64,
                );
                let start_global_pos = self.anchor_pos + (self.anchor_ori * start_local_pos_dvec);

                let start_local_ori_dquat = glam::DQuat::from_xyzw(
                    self.drag_start_local_ori.x as f64,
                    self.drag_start_local_ori.y as f64,
                    self.drag_start_local_ori.z as f64,
                    self.drag_start_local_ori.w as f64,
                );
                let start_global_ori = self.anchor_ori * start_local_ori_dquat;

                // Orbit the camera around the earth (origin)
                let new_global_pos = inv_rot * start_global_pos;
                let new_global_ori = inv_rot * start_global_ori;

                // Project back into anchor-local space
                let new_local_pos_dvec =
                    self.anchor_ori.inverse() * (new_global_pos - self.anchor_pos);
                self.local_pos = Vec3::new(
                    new_local_pos_dvec.x as f32,
                    new_local_pos_dvec.y as f32,
                    new_local_pos_dvec.z as f32,
                );

                let new_local_ori_dquat = (self.anchor_ori.inverse() * new_global_ori).normalize();
                self.local_ori = Quat::from_xyzw(
                    new_local_ori_dquat.x as f32,
                    new_local_ori_dquat.y as f32,
                    new_local_ori_dquat.z as f32,
                    new_local_ori_dquat.w as f32,
                )
                .normalize();

                // Track the velocity for inertia
                let now = std::time::Instant::now();
                let dt = (now - self.last_drag_time).as_secs_f32();
                self.last_drag_time = now;

                if dt > 0.0001 {
                    let v_prev = current_pos.normalize_or_zero();
                    let v_curr = self.local_pos.normalize_or_zero();
                    let q_step = glam::Quat::from_rotation_arc(v_prev, v_curr);
                    let (axis, angle) = q_step.to_axis_angle();

                    if angle > 0.00001 && axis.is_finite() {
                        let inst_velocity = angle / dt;
                        if self.inertia_velocity > 0.0 {
                            self.inertia_velocity = self.inertia_velocity * 0.4 + inst_velocity * 0.6;
                        } else {
                            self.inertia_velocity = inst_velocity;
                        }
                        self.inertia_axis = axis;
                    } else {
                        self.inertia_velocity *= 0.2;
                    }
                }
            } else {
                // If ray doesn't intersect anymore, retain the current position
                self.local_pos = current_pos;
                self.local_ori = current_ori;
            }
        }
    }

    pub fn end_drag(&mut self) {
        let elapsed = (std::time::Instant::now() - self.last_drag_time).as_secs_f32();
        if elapsed < 0.1 && self.inertia_velocity > 0.05 {
            self.inertia_active = true;
        } else {
            self.inertia_active = false;
            self.inertia_velocity = 0.0;
        }
        self.drag_start_point = None;
    }

    pub fn update_inertia(&mut self, dt: f32) -> bool {
        let mut redrew = false;
        if self.inertia_active {
            // Apply friction decay (0.92 per 1/60th of a second)
            let decay = 0.92_f32.powf(dt * 60.0);
            self.inertia_velocity *= decay;
            
            if self.inertia_velocity < 0.05 {
                self.inertia_active = false;
                self.inertia_velocity = 0.0;
            } else {
                // Rotate the camera around the Earth's center (orbit anchor)
                let step_angle = self.inertia_velocity * dt;
                let rot = glam::Quat::from_axis_angle(self.inertia_axis, step_angle);
                self.orbit_anchor(rot);
                redrew = true;
            }
        }
        
        redrew
    }
}
