//! Deterministic camera construction for the culling harness.
//!
//! Two families live here:
//!
//! * [`ViewParams`] / [`build_camera`] — the sweep's camera, built from a local
//!   East/North/Up frame so that pitch/yaw/roll mean something geographically.
//! * The legacy `setup_camera` / `setup_camera_direct` / `setup_fuzz_camera` /
//!   [`Lcg`] helpers, migrated verbatim out of the deleted
//!   `test_frustum_coverage.rs` because `testing::camera::test_z_sweep` and
//!   `testing::terrain::test_parametric_sweeps` still depend on them.

use cesium_engine::camera::camera::{Camera, CameraMode};
use glam::{DVec3, Quat, Vec3};

use super::geodesy::{ellipsoid_normal, lon_lat_alt_to_ecef};

/// One point in the sweep's camera parameter space.
#[derive(Clone, Debug)]
pub struct ViewParams {
    /// Human-readable name of the sweep this cell came from.
    pub sweep: &'static str,
    pub lat_deg: f64,
    pub lon_deg: f64,
    /// Altitude above the ellipsoid, in **metres** (negative = below the surface).
    pub alt_m: f64,
    /// 0° = nadir, 90° = local horizon, >90° = above the horizon.
    pub pitch_deg: f64,
    /// Compass-style heading rotation about the local up axis.
    pub yaw_deg: f64,
    /// Rotation about the view axis.
    pub roll_deg: f64,
    pub width: u32,
    pub height: u32,
    pub mode: CameraMode,
}

impl ViewParams {
    pub fn aspect(&self) -> f64 {
        self.width as f64 / self.height as f64
    }

    pub fn mode_name(&self) -> &'static str {
        match self.mode {
            CameraMode::Free => "Free",
            CameraMode::Tracking => "Tracking",
            CameraMode::Cockpit => "Cockpit",
        }
    }
}

impl Default for ViewParams {
    fn default() -> Self {
        Self {
            sweep: "unnamed",
            lat_deg: 0.0,
            lon_deg: 0.0,
            alt_m: 1_000_000.0,
            pitch_deg: 0.0,
            yaw_deg: 0.0,
            roll_deg: 0.0,
            width: 1920,
            height: 1080,
            mode: CameraMode::Free,
        }
    }
}

/// Builds the camera for a sweep cell.
///
/// The local orientation is composed as `look_at_nadir * Rz(yaw) * Rx(pitch) * Rz(roll)`:
///
/// * `look_at_nadir` points −Z (the camera's forward axis) straight down and puts
///   local +Y along local north.
/// * `Rz(yaw)` spins about the (nadir-aligned) view axis, which is the local up
///   axis — i.e. a compass heading.
/// * `Rx(pitch)` lifts the forward axis from nadir toward the heading's horizon;
///   90° is exactly the local horizon.
/// * `Rz(roll)` rolls about the final view axis.
///
/// **The transform is written straight into `local_pos`/`local_ori`, bypassing
/// `enforce_bounds`.** That is deliberate: `set_local_transform` clamps the camera
/// out of the ellipsoid, which would silently rewrite every sub-surface and
/// on-surface cell in the sweep into something else. A dedicated test covers the
/// clamped path via [`build_camera_clamped`].
pub fn build_camera(p: &ViewParams) -> Camera {
    let (pos, ori) = camera_transform(p);
    let pos_f32 = Vec3::new(pos.x as f32, pos.y as f32, pos.z as f32);

    let mut cam = Camera::new(pos_f32, Vec3::ZERO);
    // Mode must be set before any bounds logic runs: `Camera::new` defaults to
    // `Tracking`, whose znear is 1e-8..5e-6 megameters, which is a completely
    // different frustum from Free's `alt * 0.1`.
    cam.mode = p.mode;
    cam.local_pos = pos_f32;
    cam.local_ori = ori;
    cam
}

/// As [`build_camera`], but routed through `set_local_transform` so the
/// `enforce_bounds` clamp applies. Used to cover the clamped code path.
pub fn build_camera_clamped(p: &ViewParams) -> Camera {
    let (pos, ori) = camera_transform(p);
    let pos_f32 = Vec3::new(pos.x as f32, pos.y as f32, pos.z as f32);
    let mut cam = Camera::new(pos_f32, Vec3::ZERO);
    cam.mode = p.mode;
    cam.set_local_transform(pos_f32, ori);
    cam
}

fn camera_transform(p: &ViewParams) -> (DVec3, Quat) {
    let pos = lon_lat_alt_to_ecef(p.lon_deg, p.lat_deg, p.alt_m);
    let up = ellipsoid_normal(pos);

    // The engine builds its tile tangent frames as `east = Y × normal`; matching it
    // keeps "north"/"east" meaning the same thing in the harness as in the renderer.
    let mut east = DVec3::Y.cross(up);
    if east.length_squared() < 1.0e-12 {
        // Exactly at a pole the cross product degenerates; the engine falls back
        // to +X in `compute_sub_obb`, so do the same.
        east = DVec3::X;
    }
    let east = east.normalize();
    let north = up.cross(east).normalize();

    let up_f = Vec3::new(up.x as f32, up.y as f32, up.z as f32);
    let north_f = Vec3::new(north.x as f32, north.y as f32, north.z as f32);
    let pos_f = Vec3::new(pos.x as f32, pos.y as f32, pos.z as f32);

    // Nadir-looking basis: forward = -up, screen-up = north.
    let view = glam::Mat4::look_at_rh(pos_f, pos_f - up_f, north_f);
    let q0 = Quat::from_mat4(&view.inverse()).normalize();

    let q = q0
        * Quat::from_axis_angle(Vec3::Z, (p.yaw_deg as f32).to_radians())
        * Quat::from_axis_angle(Vec3::X, (p.pitch_deg as f32).to_radians())
        * Quat::from_axis_angle(Vec3::Z, (p.roll_deg as f32).to_radians());

    (pos, q.normalize())
}

// ─────────────────────────────────────────────────────────────────────────────
// Legacy helpers, migrated from the removed `test_frustum_coverage.rs`.
// Kept byte-for-byte in behaviour so the call sites in `testing::camera` and
// `testing::terrain` keep measuring exactly what they measured before.
// ─────────────────────────────────────────────────────────────────────────────

/// Camera at (lat, lon, altitude-in-megameters) looking at the Earth's centre,
/// then pitched by `pitch_offset_deg` about its local X axis.
pub fn setup_camera(lat_deg: f32, lon_deg: f32, altitude: f32, pitch_offset_deg: f32) -> Camera {
    let pos = surface_normal_offset_pos(lat_deg, lon_deg, altitude);

    let mut cam = Camera::new(pos, Vec3::ZERO);
    cam.set_eye(pos, Vec3::ZERO); // Looks straight down (-Z points to centre)

    if pitch_offset_deg != 0.0 {
        let pitch_quat = Quat::from_axis_angle(Vec3::X, pitch_offset_deg.to_radians());
        cam.rotate_local(pitch_quat);
    }

    cam
}

/// Camera placed at an explicit ECEF position with explicit YXZ Euler angles.
pub fn setup_camera_direct(pos: Vec3, pitch_deg: f32, yaw_deg: f32, roll_deg: f32) -> Camera {
    let mut cam = Camera::new(pos, Vec3::ZERO);
    cam.set_local_transform(
        pos,
        Quat::from_euler(
            glam::EulerRot::YXZ,
            yaw_deg.to_radians(),
            pitch_deg.to_radians(),
            roll_deg.to_radians(),
        ),
    );
    cam
}

/// Camera at (lat, lon, altitude-in-megameters) with explicit YXZ Euler angles.
pub fn setup_fuzz_camera(
    lat_deg: f32,
    lon_deg: f32,
    altitude: f32,
    pitch_deg: f32,
    yaw_deg: f32,
    roll_deg: f32,
) -> Camera {
    let pos = surface_normal_offset_pos(lat_deg, lon_deg, altitude);
    let mut cam = Camera::new(pos, Vec3::ZERO);
    cam.set_local_transform(
        pos,
        Quat::from_euler(
            glam::EulerRot::YXZ,
            yaw_deg.to_radians(),
            pitch_deg.to_radians(),
            roll_deg.to_radians(),
        ),
    );
    cam
}

fn surface_normal_offset_pos(lat_deg: f32, lon_deg: f32, altitude: f32) -> Vec3 {
    let a = 6.378137_f32;
    let b = 6.3567523142_f32;

    let phi = lat_deg.to_radians();
    let theta = lon_deg.to_radians();

    let surface = Vec3::new(
        a * phi.cos() * theta.cos(),
        b * phi.sin(),
        -a * phi.cos() * theta.sin(),
    );
    let normal = Vec3::new(
        surface.x / (a * a),
        surface.y / (b * b),
        surface.z / (a * a),
    )
    .normalize();

    surface + normal * altitude
}

/// Hand-rolled linear congruential generator.
///
/// `rand`/`proptest` are not available to this crate (dev-dependencies are only
/// `image`, `rayon` and `tokio`), and a seeded LCG is exactly what a reproducible
/// fuzz sweep needs: same seed, same cases, every run, on every machine.
pub struct Lcg {
    pub state: u32,
}

impl Lcg {
    pub fn new(seed: u32) -> Self {
        Self { state: seed }
    }

    /// Uniform in [0, 1].
    pub fn next_f32(&mut self) -> f32 {
        self.state = self.state.wrapping_mul(1664525).wrapping_add(1013904223);
        (self.state as f32) / (u32::MAX as f32)
    }

    /// Uniform in [0, 1], in f64 (still driven by the same 32-bit stream, so the
    /// sequence is identical to `next_f32`'s).
    pub fn next_f64(&mut self) -> f64 {
        self.state = self.state.wrapping_mul(1664525).wrapping_add(1013904223);
        (self.state as f64) / (u32::MAX as f64)
    }

    /// Uniform in [lo, hi].
    pub fn range(&mut self, lo: f64, hi: f64) -> f64 {
        lo + self.next_f64() * (hi - lo)
    }
}
