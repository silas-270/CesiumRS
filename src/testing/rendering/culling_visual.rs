//! Headless captures for visual verification of the culling rework.
//!
//! AGENTS.md requires that anything touching rendering, geometry or camera
//! positioning be checked with the headless path and *looked at*. Culling decides
//! which tiles exist, so it qualifies twice over: a false negative is a hole in the
//! globe and a false positive is over-draw, and neither shows up in a number.
//!
//! `#[ignore]`d because it needs network access for imagery and writes PNGs. Run it
//! against two builds and diff the images by eye:
//!
//! ```text
//! CESIUM_SHOT_DIR=/tmp/shots/after \
//!   cargo test --release --lib rendering::culling_visual -- --ignored --nocapture
//! ```
//!
//! The pose list is chosen to cover the regimes the harness measures numerically:
//! the whole globe (coarse tiles, where the old 8×8 sub-OBB grid lived), the limb
//! (where the back-face heuristic cut an 8.07° band), a pole (the stretch, and the
//! tangent-frame degeneracy), and near-ground high-zoom (where the near plane used
//! to blank the globe).

use cesium_engine::globe::tiles::config::TileEngineConfig;

/// Where the PNGs go. Set `CESIUM_SHOT_DIR`; defaults to the system temp dir.
fn shot_dir() -> std::path::PathBuf {
    let dir = std::env::var_os("CESIUM_SHOT_DIR")
        .map(std::path::PathBuf::from)
        .unwrap_or_else(|| std::env::temp_dir().join("cesium_culling_shots"));
    let _ = std::fs::create_dir_all(&dir);
    dir
}

const A: f64 = 6.378137;
const B: f64 = 6.3567523142;

/// ECEF position at (lon, lat) degrees and altitude in metres, engine convention.
fn ecef(lon_deg: f64, lat_deg: f64, alt_m: f64) -> glam::Vec3 {
    let p = cesium_engine::globe::geometry::lon_lat_alt_to_ecef_f64(lon_deg, lat_deg, alt_m);
    glam::Vec3::new(p[0] as f32, p[1] as f32, p[2] as f32)
}

/// Local north at (lon, lat), given the local up.
fn north_at(lon_deg: f64, _lat_deg: f64, up: glam::DVec3) -> glam::DVec3 {
    let east = glam::DVec3::new(-lon_deg.to_radians().sin(), 0.0, -lon_deg.to_radians().cos());
    up.cross(east).normalize()
}

fn up_at(lon_deg: f64, lat_deg: f64) -> glam::DVec3 {
    let p = cesium_engine::globe::geometry::lon_lat_to_ecef_f64(lon_deg, lat_deg);
    glam::DVec3::new(p[0] / (A * A), p[1] / (B * B), p[2] / (A * A)).normalize()
}

/// One capture: a name, an eye, a look-at target, and an optional up vector.
struct Pose {
    name: &'static str,
    eye: glam::Vec3,
    target: glam::Vec3,
    up: Option<glam::Vec3>,
    what: &'static str,
}

fn poses() -> Vec<Pose> {
    let to_f32 = |v: glam::DVec3| glam::Vec3::new(v.x as f32, v.y as f32, v.z as f32);

    // Limb view: stand 800 km up and pitch down onto the horizon. Looking along the
    // *local horizontal* would miss it entirely — the horizon dip at 800 km is
    // acos(a/(a+h)) = 27.2 deg, and the vertical half-FOV is only 23.2 deg, so the
    // limb falls below the bottom of the frame. Pitch down 27 deg to put it on the
    // centreline. This is the regime the sub-OBB back-face heuristic broke.
    let limb_lon = 9.0;
    let limb_lat = 48.0;
    let limb_eye = ecef(limb_lon, limb_lat, 800_000.0);
    let limb_up = up_at(limb_lon, limb_lat);
    let limb_north = north_at(limb_lon, limb_lat, limb_up);
    let limb_pitch = 27.2f64.to_radians();
    let limb_dir = (limb_north * limb_pitch.cos() - limb_up * limb_pitch.sin()).normalize();
    let limb_target = limb_eye + to_f32(limb_dir) * 3.0;

    // Near-ground: 60 m up, looking 25 deg below the local horizontal. Deepest zoom,
    // and the pose where Tracking mode used to return zero tiles.
    let ng_lon = 147.564;
    let ng_lat = -12.7;
    let ng_eye = ecef(ng_lon, ng_lat, 60.0);
    let ng_up = up_at(ng_lon, ng_lat);
    let ng_north = north_at(ng_lon, ng_lat, ng_up);
    let ng_dir = (ng_north * 25.0f64.to_radians().cos() - ng_up * 25.0f64.to_radians().sin())
        .normalize();
    let ng_target = ng_eye + to_f32(ng_dir) * 0.01;

    // 12 000 km, nadir: the exact altitude at which the old sub-OBB back-face test
    // cut its widest band, 8.0715 deg inside the visible limb. Any hole it leaves is
    // an annulus just inside the edge of the disc.
    let band_eye = ecef(9.0, 48.0, 12_000_000.0);

    // 12 000 km, tilted 20 deg off nadir: the two cells the horizon-pitch sweep used
    // to lose, where the limb and the bottom screen edge coincide.
    let tilt_up = up_at(9.0, 48.0);
    let tilt_north = north_at(9.0, 48.0, tilt_up);
    let tilt_dir = (-tilt_up * 20.0f64.to_radians().cos()
        + tilt_north * 20.0f64.to_radians().sin())
    .normalize();
    let tilt_target = band_eye + to_f32(tilt_dir) * 12.0;

    vec![
        Pose {
            name: "01_globe_20Mm",
            eye: ecef(9.0, 25.0, 20_000_000.0),
            target: glam::Vec3::ZERO,
            up: None,
            what: "whole disc from 20 000 km — coarse tiles, the whole limb in frame",
        },
        Pose {
            name: "02_globe_2Mm",
            eye: ecef(9.0, 48.0, 2_000_000.0),
            target: glam::Vec3::ZERO,
            up: None,
            what: "regional view from 2 000 km — the limb across the top of the frame",
        },
        Pose {
            name: "03_limb_800km",
            eye: limb_eye,
            target: limb_target,
            up: Some(to_f32(limb_up)),
            what: "limb/horizon view along the local horizontal at 800 km",
        },
        Pose {
            name: "04_pole_north",
            eye: ecef(0.0, 90.0, 8_000_000.0),
            target: glam::Vec3::ZERO,
            up: None,
            what: "straight down on the north pole — pole stretch and the tangent frame",
        },
        Pose {
            name: "05_near_ground_60m",
            eye: ng_eye,
            target: ng_target,
            up: Some(to_f32(ng_up)),
            what: "60 m above the ground looking 25 deg down — deepest zoom",
        },
        Pose {
            name: "06_antimeridian",
            eye: ecef(180.0, 0.0, 5_000_000.0),
            target: glam::Vec3::ZERO,
            up: None,
            what: "over the antimeridian — the circular arc test in the horizon stage",
        },
        Pose {
            name: "07_limb_band_12Mm_nadir",
            eye: band_eye,
            target: glam::Vec3::ZERO,
            up: None,
            what: "nadir at 12 000 km — where the old back-face heuristic cut 8.07 deg",
        },
        Pose {
            name: "08_limb_band_12Mm_tilt20",
            eye: band_eye,
            target: tilt_target,
            up: Some(to_f32(tilt_up)),
            what: "12 000 km tilted 20 deg off nadir — limb meets the screen edge",
        },
    ]
}

#[test]
#[ignore = "visual verification: needs network for imagery, writes PNGs to CESIUM_SHOT_DIR"]
fn capture_culling_poses() {
    let dir = shot_dir();
    println!("  writing captures to {}", dir.display());

    for p in poses() {
        let out = dir.join(format!("{}.png", p.name));
        let out_str = out.to_string_lossy().into_owned();

        let mut config = TileEngineConfig::default();
        config.offline_mode = false;
        config.transparent_background = true;
        config.lod_factor = 2.0;
        config.mesh_segments = 32;

        pollster::block_on(crate::headless::routes_headless_app::run_headless_render(
            1280,
            720,
            config,
            None,
            p.eye,
            p.target,
            p.up,
            &out_str,
        ));

        println!("  {:<22} {}  -> {}", p.name, p.what, out.display());
    }
}
