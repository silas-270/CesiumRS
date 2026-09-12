//! Parameter-cell generation for the Layer 2 sweep.
//!
//! Kept separate from [`super::sweep`] (which only knows how to *measure* a cell)
//! and from the test cases (which only decide what to assert about the numbers).
//!
//! The structured sweep is one-factor-at-a-time around a baseline: a full cross
//! product of every axis would be ~10^6 cells, which nobody would ever run. OFAT
//! finds axis-aligned defects cheaply and deterministically; the seeded fuzz in
//! [`fuzz_cells`] covers the interactions OFAT misses.

use cesium_engine::camera::camera::CameraMode;

use super::cameras::{Lcg, ViewParams};
use super::geodesy::MERCATOR_LIMIT_DEG;

/// The centre of the parameter space: mid-latitude, 1000 km up, looking straight
/// down, 16:9, Free mode.
pub fn baseline() -> ViewParams {
    ViewParams {
        sweep: "baseline",
        lat_deg: 48.0,
        lon_deg: 9.0,
        alt_m: 1_000_000.0,
        pitch_deg: 0.0,
        yaw_deg: 0.0,
        roll_deg: 0.0,
        width: 1920,
        height: 1080,
        mode: CameraMode::Free,
    }
}

/// Altitudes in metres, log-spaced from ~10 m to 30 Mm, plus on-surface and
/// just-below-surface probes.
///
/// `-50.0` is there on purpose: `Camera::altitude()` clamps to 2 mm and
/// `enforce_bounds` would push the camera back out, but the sweep writes
/// `local_pos` directly, so a sub-surface camera really is tested. It is the case
/// where `vh_mag_sq` goes negative and the horizon-culling early-out changes
/// behaviour.
pub const ALTITUDE_LADDER_M: [f64; 11] = [
    -50.0,
    0.0,
    10.0,
    100.0,
    1_000.0,
    10_000.0,
    100_000.0,
    1_000_000.0,
    5_000_000.0,
    10_000_000.0,
    30_000_000.0,
];

/// Latitudes including both poles and both Mercator truncation latitudes.
pub fn latitudes() -> Vec<f64> {
    let mut v = vec![
        -90.0,
        -MERCATOR_LIMIT_DEG - 0.5,
        -MERCATOR_LIMIT_DEG,
        -MERCATOR_LIMIT_DEG + 0.5,
        -89.0,
        MERCATOR_LIMIT_DEG - 0.5,
        MERCATOR_LIMIT_DEG,
        MERCATOR_LIMIT_DEG + 0.5,
        89.0,
        90.0,
    ];
    // 5-degree ladder across the whole range, on top of the named special cases.
    let mut lat = -85.0;
    while lat <= 85.0 + 1e-9 {
        v.push(lat);
        lat += 5.0;
    }
    v
}

/// Longitudes including both sides of the antimeridian.
pub fn longitudes() -> Vec<f64> {
    let mut v = vec![-180.0, -179.999, 179.999, 9.0];
    let mut lon = -180.0;
    while lon < 180.0 {
        v.push(lon);
        lon += 15.0;
    }
    v
}

/// 0° = nadir, 90° = local horizon, >90° = above the horizon.
pub fn pitches() -> Vec<f64> {
    let mut v = vec![89.0, 89.9, 90.0, 90.1, 91.0];
    let mut p = 0.0;
    while p <= 130.0 + 1e-9 {
        v.push(p);
        p += 5.0;
    }
    v
}

/// Viewport shapes, including a very wide and a very tall one.
pub fn aspects() -> Vec<(u32, u32)> {
    vec![
        (1920, 1080),
        (1080, 1920),
        (1024, 1024),
        (3840, 720),  // ultra-wide: horizontal FOV ~5.3x vertical
        (480, 1920),  // ultra-tall: vertical FOV dominates
    ]
}

pub fn modes() -> Vec<CameraMode> {
    vec![CameraMode::Free, CameraMode::Tracking, CameraMode::Cockpit]
}

/// The structured one-factor-at-a-time sweep.
pub fn axis_sweep() -> Vec<ViewParams> {
    let mut out = Vec::new();
    let base = baseline();
    out.push(base.clone());

    for lat in latitudes() {
        out.push(ViewParams {
            sweep: "latitude",
            lat_deg: lat,
            ..base.clone()
        });
    }
    for lon in longitudes() {
        out.push(ViewParams {
            sweep: "longitude",
            lon_deg: lon,
            ..base.clone()
        });
    }
    for alt in ALTITUDE_LADDER_M {
        out.push(ViewParams {
            sweep: "altitude",
            alt_m: alt,
            ..base.clone()
        });
    }
    for pitch in pitches() {
        out.push(ViewParams {
            sweep: "pitch",
            pitch_deg: pitch,
            ..base.clone()
        });
    }
    for yaw in [0.0, 30.0, 45.0, 60.0, 90.0, 120.0, 135.0, 150.0, 180.0, 225.0, 270.0, 315.0] {
        out.push(ViewParams {
            sweep: "yaw",
            yaw_deg: yaw,
            // Yaw is meaningless at nadir with no pitch, so tilt first.
            pitch_deg: 60.0,
            ..base.clone()
        });
    }
    for roll in [
        0.0, 15.0, 30.0, 45.0, 60.0, 90.0, 120.0, 135.0, 150.0, 180.0, -45.0, -90.0, -135.0,
    ] {
        out.push(ViewParams {
            sweep: "roll",
            roll_deg: roll,
            pitch_deg: 45.0,
            ..base.clone()
        });
    }
    for (w, h) in aspects() {
        out.push(ViewParams {
            sweep: "aspect",
            width: w,
            height: h,
            ..base.clone()
        });
    }
    for mode in modes() {
        out.push(ViewParams {
            sweep: "mode",
            mode,
            ..base.clone()
        });
        // Modes differ most where their znear differs most: close to the ground.
        out.push(ViewParams {
            sweep: "mode-low",
            mode,
            alt_m: 2_000.0,
            pitch_deg: 70.0,
            ..base.clone()
        });
    }

    out
}

/// A nadir altitude ladder at a few latitudes: the tamest possible geometry, used
/// as the regression guard that must stay green.
pub fn nadir_ladder() -> Vec<ViewParams> {
    let base = baseline();
    let mut out = Vec::new();
    for lat in [-80.0, -48.0, -15.0, 0.0, 15.0, 30.0, 48.0, 60.0, 80.0] {
        for alt in [
            100.0, 1_000.0, 10_000.0, 100_000.0, 400_000.0, 1_000_000.0, 5_000_000.0,
            20_000_000.0,
        ] {
            out.push(ViewParams {
                sweep: "nadir-ladder",
                lat_deg: lat,
                alt_m: alt,
                pitch_deg: 0.0,
                ..base.clone()
            });
        }
    }
    out
}

/// Altitudes chosen to walk the deepest reached zoom across the `tight_obbs`
/// cliff at z=16 and up into z=17..20, where only the loose OBB is tested.
///
/// `QuadtreeNode::compute_bounding_volume` builds the 8×8 grid of tight sub-OBBs
/// only for `id.z <= 16`. Above that the node is culled by its single loose OBB,
/// which for a curved tile is noticeably fatter than the surface — so the FP rate
/// should jump and the back-face rejection in the tight-OBB branch disappears
/// entirely.
pub fn zoom_cliff_cells() -> Vec<ViewParams> {
    let base = baseline();
    let mut out = Vec::new();
    for alt in [
        10.0, 15.0, 20.0, 30.0, 40.0, 60.0, 80.0, 110.0, 160.0, 225.0, 320.0, 450.0, 640.0,
        900.0, 1_280.0, 1_800.0, 2_560.0, 3_600.0, 5_120.0, 7_200.0, 10_240.0, 14_400.0,
    ] {
        for pitch in [0.0, 20.0, 45.0, 70.0, 85.0, 90.0] {
            out.push(ViewParams {
                sweep: "zoom-cliff",
                alt_m: alt,
                pitch_deg: pitch,
                ..base.clone()
            });
        }
    }
    out
}

/// Seeded random cells. Same seed ⇒ same cells on every machine, every run.
pub fn fuzz_cells(count: usize, seed: u32) -> Vec<ViewParams> {
    let mut rng = Lcg::new(seed);
    let aspects = aspects();
    let modes = modes();
    let mut out = Vec::with_capacity(count);

    for _ in 0..count {
        // Altitude is log-uniform over [10 m, 30 Mm] so the low end — where the
        // quadtree does the most work and f32 precision is worst — is not drowned
        // out by the high end.
        let log_alt = rng.range(1.0, 7.477);
        let alt_m = 10f64.powf(log_alt);

        let (w, h) = aspects[(rng.next_f64() * aspects.len() as f64) as usize % aspects.len()];
        let mode = modes[(rng.next_f64() * modes.len() as f64) as usize % modes.len()];

        out.push(ViewParams {
            sweep: "fuzz",
            lat_deg: rng.range(-90.0, 90.0),
            lon_deg: rng.range(-180.0, 180.0),
            alt_m,
            pitch_deg: rng.range(0.0, 130.0),
            yaw_deg: rng.range(0.0, 360.0),
            roll_deg: rng.range(-180.0, 180.0),
            width: w,
            height: h,
            mode,
        });
    }
    out
}
