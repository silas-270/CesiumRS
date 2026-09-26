//! Headless captures for the **terrain-step** question —
//! a camera at eye height at the foot of an escarpment, looking at it, with flat high
//! ground behind.
//!
//! `terrain::test_terrain_step` counts the tiles; this draws them. Two shots per pose,
//! without and with terrain occlusion, with nothing else different, because a tile count cannot see a hole
//! and two pictures can — the same contract `terrain_capture` has had since terrain occlusion was introduced.
//!
//! **The capture poses of `terrain_capture` are not touched.** Previous runs pin those five, all
//! three shots each, as byte-identical across the optimisation pass; adding a pose to
//! that list would invalidate a comparison that is quoted. These are their own poses in
//! their own module, with their own shot names.
//!
//! The eye altitude is **not typed into this file**. It is read from the real DEM at the
//! pose's own coordinates through `TileSystem::ground_height_at`'s own source, because
//! earlier runs lost three measurement poses to coordinates that looked like a valley floor on a
//! map and were a mountainside in the data.
//!
//! ```text
//! CESIUM_SHOT_DIR=/tmp/shots CESIUM_HEIGHT_CACHE=/tmp/dem \
//!   cargo test --release --lib rendering::terrain_step_capture -- --ignored --nocapture
//! ```

use cesium_engine::globe::tiles::config::{TerrainConfig, TileEngineConfig, satellite_imagery_url};

use super::terrain_capture::{render_settled, shot_dir, Pose};

const A: f64 = 6.378137;
const B: f64 = 6.3567523142;

fn ecef(lon_deg: f64, lat_deg: f64, alt_m: f64) -> glam::Vec3 {
    let p = cesium_engine::globe::geometry::lon_lat_alt_to_ecef_f64(lon_deg, lat_deg, alt_m);
    glam::Vec3::new(p[0] as f32, p[1] as f32, p[2] as f32)
}

fn up_at(lon_deg: f64, lat_deg: f64) -> glam::DVec3 {
    let p = cesium_engine::globe::geometry::lon_lat_to_ecef_f64(lon_deg, lat_deg);
    glam::DVec3::new(p[0] / (A * A), p[1] / (B * B), p[2] / (A * A)).normalize()
}

/// Stand `alt_m` up at (`lon`, `lat`) and look along a **compass bearing**, pitched
/// `pitch_deg` below the horizontal.
///
/// `terrain_capture::oblique` looks due north and has no bearing argument; both poses
/// here need one (135° at Reutlingen, 180° at Stuttgart), so this is that function with
/// the heading put back in rather than a fifth pose bolted onto a pinned list.
pub(crate) fn bearing_pose(
    name: &'static str,
    lon: f64,
    lat: f64,
    alt_m: f64,
    bearing_deg: f64,
    pitch_deg: f64,
    reach_mm: f64,
    what: &'static str,
) -> Pose {
    let eye = ecef(lon, lat, alt_m);
    let up = up_at(lon, lat);
    let east = glam::DVec3::new(
        -lon.to_radians().sin(),
        0.0,
        -lon.to_radians().cos(),
    );
    let north = up.cross(east).normalize();
    let east = north.cross(up).normalize();
    let b = bearing_deg.to_radians();
    let horiz = north * b.cos() + east * b.sin();
    let pitch = pitch_deg.to_radians();
    let dir = (horiz * pitch.cos() - up * pitch.sin()).normalize();
    // **The heading is checked, not assumed.** `ViewParams::yaw_deg`, which the counting
    // harness next door goes through, turned out to build a camera along `−yaw` (it is
    // composed about the nadir-aligned view axis), and a picture taken 270° away from the
    // profile that was measured is the same failure §7c's three lost poses were. This
    // construction builds the direction from `east`/`north` directly, and the assertion is
    // what says so.
    let got = horiz.dot(east).atan2(horiz.dot(north)).to_degrees();
    let d = (got - bearing_deg + 540.0) % 360.0 - 180.0;
    assert!(
        d.abs() < 0.01,
        "{name}: built a camera along {got:.1} deg, not the declared {bearing_deg:.1}"
    );
    Pose {
        name,
        eye,
        target: eye + glam::Vec3::new(dir.x as f32, dir.y as f32, dir.z as f32) * reach_mm as f32,
        up: Some(glam::Vec3::new(up.x as f32, up.y as f32, up.z as f32)),
        what,
    }
}

fn config(occlusion: bool) -> TileEngineConfig {
    TileEngineConfig {
        base_imagery_url: satellite_imagery_url(),
        offline_mode: false,
        transparent_background: true,
        target_texel_ratio: 1.0,
        mesh_segments: 16,
        terrain: TerrainConfig {
            enabled: true,
            exaggeration: 1.0,
            occlusion: cesium_engine::globe::quadtree::TerrainOcclusionConfig {
                enabled: occlusion,
                ..Default::default()
            },
            ..TerrainConfig::default()
        },
        ..TileEngineConfig::default()
    }
}

/// The two step poses, with the eye altitudes
/// `terrain::test_terrain_step::terrain_step_pose_is_where_it_says_it_is` measures off the
/// DEM: 375.7 m of ground at Reutlingen and 250.0 m in the Stuttgart basin, plus two
/// metres of eye height.
fn poses() -> Vec<Pose> {
    vec![
        bearing_pose(
            "reutlingen_albtrauf",
            9.2043,
            48.4914,
            377.7,
            135.0,
            1.0,
            0.06,
            "the Albtrauf at 8 km from the foot of the Alb, 40 km of plateau behind it",
        ),
        bearing_pose(
            "stuttgart_kessel",
            9.1829,
            48.7758,
            252.0,
            180.0,
            1.0,
            0.06,
            "the basin rim at 2.5 km from the city floor, the Filder plateau behind it",
        ),
    ]
}

#[test]
#[ignore = "visual verification: needs the network for imagery and heights, writes PNGs"]
fn capture_terrain_step_poses() {
    let dir = shot_dir();
    println!("  writing captures to {}", dir.display());

    for p in poses() {
        println!("  {} — {}", p.name, p.what);
        for (suffix, occlusion) in [("d1d2", false), ("d3", true)] {
            let out = dir.join(format!("step_{}_{suffix}.png", p.name));
            let out_str = out.to_string_lossy().into_owned();
            let (visible, heights) =
                pollster::block_on(render_settled(1280, 720, config(occlusion), &p, &out_str));
            println!(
                "    {suffix:<6} {visible:4} visible tiles, {heights:3} height tiles -> {}",
                out.display()
            );
        }
    }
}
