//! Per-frame camera trace, for checking that the camera moves continuously.
//!
//! One CSV row per frame to `camera_trace.csv` in the working directory. Analyse it with
//! `tools/camera_jumps.py`.

use super::camera::{Camera, CameraMode};
use glam::{DVec3, Vec3};
use std::fs::File;
use std::io::{BufWriter, Write};
use std::time::Instant;

pub const TRACE_FILE: &str = "camera_trace.csv";

/// The orbit parameters of `local_pos`: yaw, pitch (degrees) and distance (metres).
fn orbit(local: Vec3) -> (f64, f64, f64) {
    let d = local.length() as f64;
    if d < 1e-12 {
        return (0.0, 0.0, 0.0);
    }
    let yaw = (local.x as f64).atan2(local.z as f64).to_degrees();
    let pitch = (local.y as f64 / d).clamp(-1.0, 1.0).asin().to_degrees();
    (yaw, pitch, d * 1.0e6)
}

pub struct CameraTrace {
    out: BufWriter<File>,
    start: Instant,
    frame: u64,
    pre_local: Vec3,
}

impl CameraTrace {
    pub fn create() -> Option<Self> {
        let file = File::create(TRACE_FILE)
            .map_err(|e| log::warn!("camera trace disabled: {TRACE_FILE}: {e}"))
            .ok()?;
        let mut out = BufWriter::new(file);
        let _ = writeln!(
            out,
            "frame,t_ms,mode,x_m,y_m,z_m,lat,lon,alt_m,agl_m,heading,pitch,roll,\
             orbit_yaw,orbit_pitch,orbit_dist_m,ground_cam_m,ground_anchor_m,\
             anchor_x_m,anchor_y_m,anchor_z_m,collision_move_m"
        );
        log::info!("Camera trace -> {TRACE_FILE}");
        Some(Self {
            out,
            start: Instant::now(),
            frame: 0,
            pre_local: Vec3::ZERO,
        })
    }

    /// Call right before the collision pass.
    pub fn before_collision(&mut self, camera: &Camera) {
        self.pre_local = camera.local_pos;
    }

    /// Call once per frame after the collision pass. `ground` is the same query the
    /// collision pass used (height above the ellipsoid, megametres).
    pub fn record(&mut self, camera: &Camera, ground: &dyn Fn(DVec3) -> Option<f64>) {
        let (pos, ori) = camera.global_transform_f64();
        let (lon, lat) = crate::globe::geometry::ecef_to_lon_lat_f64(pos);

        // Local east/north/up at the camera; Y is the polar axis in this frame.
        let up = pos.normalize_or_zero();
        let east = DVec3::Y.cross(up).normalize_or_zero();
        let north = up.cross(east);
        let fwd = ori * DVec3::NEG_Z;
        let cam_up = ori * DVec3::Y;
        let right = ori * DVec3::X;
        let heading = fwd.dot(east).atan2(fwd.dot(north)).to_degrees();
        let pitch = fwd.dot(up).clamp(-1.0, 1.0).asin().to_degrees();
        let roll = (-right.dot(up)).atan2(cam_up.dot(up)).to_degrees();

        let (oyaw, opitch, odist) = orbit(camera.local_pos);
        let m = |h: Option<f64>| h.map_or(f64::NAN, |v| v * 1.0e6);
        let ground_cam = m(ground(pos));
        let ground_anchor = if camera.mode == CameraMode::Free {
            f64::NAN
        } else {
            m(ground(camera.anchor_pos))
        };
        let collision_move = (camera.local_pos - self.pre_local).length() as f64 * 1.0e6;
        let a = camera.anchor_pos * 1.0e6;

        let _ = writeln!(
            self.out,
            "{},{:.3},{:?},{:.3},{:.3},{:.3},{:.8},{:.8},{:.3},{:.3},{:.4},{:.4},{:.4},\
             {:.4},{:.4},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.4}",
            self.frame,
            self.start.elapsed().as_secs_f64() * 1000.0,
            camera.mode,
            pos.x * 1.0e6,
            pos.y * 1.0e6,
            pos.z * 1.0e6,
            lat,
            lon,
            camera.altitude() as f64 * 1.0e6,
            camera.altitude_agl() as f64 * 1.0e6,
            heading,
            pitch,
            roll,
            oyaw,
            opitch,
            odist,
            ground_cam,
            ground_anchor,
            a.x,
            a.y,
            a.z,
            collision_move,
        );
        self.frame += 1;
        if self.frame % 120 == 0 {
            let _ = self.out.flush();
        }
    }
}
