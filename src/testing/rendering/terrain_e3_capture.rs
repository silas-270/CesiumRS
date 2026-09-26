//! Headless captures for the terrain ground reference — the two parts of it that only
//! a picture can settle.
//!
//! **Field elevation.** The flight planner now puts an aircraft at its departure
//! field's true elevation. Whether that is *right* is a question about two numbers
//! agreeing: the elevation the planner was handed, and the elevation of the ground the
//! globe draws under it. This module asks the planner for a real plan, puts the camera
//! at the aircraft's own starting altitude, and renders. If the two agree, the eye is
//! just above the runway and the valley reads as a valley. If the planner is right and
//! the globe is not, the eye is inside a mountain and the frame is opaque.
//!
//! The control shot is the same pose with `terrain_elevation: false`, which starts the
//! aircraft at sea level — 581 m underground at Innsbruck, 2 548 m underground at
//! Bogotá.
//!
//! **The collision floor.** A pose is placed *inside* the Nordkette, 1 400 m below
//! the ridge line. `enforce_bounds` should lift it onto the ridge; the capture prints the
//! altitude before and after so the lift is a number as well as a picture, and the
//! picture says whether what comes out is a view from a summit or a view from inside
//! rock.
//!
//! `#[ignore]`d because it needs the network (imagery *and* height tiles) and writes
//! PNGs:
//!
//! ```text
//! CESIUM_SHOT_DIR=/tmp/shots \
//!   cargo test --release --lib rendering::terrain_e3_capture -- --ignored --nocapture
//! ```

use cesium_engine::globe::tiles::config::{TerrainConfig, TileEngineConfig, satellite_imagery_url};
use cesium_flight::telemetry::geo::LatLon;
use cesium_flight::telemetry::{generate, FlightPlanConfig, FlightRequest};

/// Where the PNGs go. Set `CESIUM_SHOT_DIR`; defaults to the system temp dir.
fn shot_dir() -> std::path::PathBuf {
    let dir = std::env::var_os("CESIUM_SHOT_DIR")
        .map(std::path::PathBuf::from)
        .unwrap_or_else(|| std::env::temp_dir().join("cesium_terrain_e3_shots"));
    let _ = std::fs::create_dir_all(&dir);
    dir
}

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

/// The local east and north unit vectors at a position.
fn east_north_at(lon_deg: f64, up: glam::DVec3) -> (glam::DVec3, glam::DVec3) {
    let east = glam::DVec3::new(
        -lon_deg.to_radians().sin(),
        0.0,
        -lon_deg.to_radians().cos(),
    );
    (east, up.cross(east).normalize())
}

struct Pose {
    name: &'static str,
    eye: glam::Vec3,
    target: glam::Vec3,
    up: glam::Vec3,
    /// What the pose *asked* for, in metres above the ellipsoid. Printed alongside where
    /// the camera actually ended up, because collision bounds enforcement may have moved it.
    requested_alt_m: f64,
    what: String,
}

/// Stand `alt_m` up at (`lon`, `lat`) and look along a compass `bearing_deg`, pitched
/// `pitch_deg` below the horizontal.
///
/// Same construction as `rendering::terrain_capture::oblique`, with the bearing freed
/// from due north — the poses look along runways and across valleys, not only at
/// the nearest wall.
fn look(
    name: &'static str,
    lon: f64,
    lat: f64,
    alt_m: f64,
    bearing_deg: f64,
    pitch_deg: f64,
    reach_mm: f64,
    what: String,
) -> Pose {
    let eye = ecef(lon, lat, alt_m);
    let up = up_at(lon, lat);
    let (east, north) = east_north_at(lon, up);
    let b = bearing_deg.to_radians();
    let horizontal = (north * b.cos() + east * b.sin()).normalize();
    let pitch = pitch_deg.to_radians();
    let dir = (horizontal * pitch.cos() - up * pitch.sin()).normalize();
    Pose {
        name,
        eye,
        target: eye + glam::Vec3::new(dir.x as f32, dir.y as f32, dir.z as f32) * reach_mm as f32,
        up: glam::Vec3::new(up.x as f32, up.y as f32, up.z as f32),
        requested_alt_m: alt_m,
        what,
    }
}

/// The same config the terrain captures use, so the two sets are comparable.
fn config(terrain: bool) -> TileEngineConfig {
    TileEngineConfig {
        base_imagery_url: satellite_imagery_url(),
        offline_mode: false,
        transparent_background: true,
        target_texel_ratio: 1.0,
        mesh_segments: 16,
        terrain: TerrainConfig {
            enabled: terrain,
            exaggeration: 1.0,
            occlusion: cesium_engine::globe::quadtree::TerrainOcclusionConfig {
                enabled: true,
                ..Default::default()
            },
            ..TerrainConfig::default()
        },
        ..TileEngineConfig::default()
    }
}

/// The altitude the planner puts the aircraft at on the departure runway, in metres.
fn departure_altitude_m(dep: (f64, f64), arr: (f64, f64), config: FlightPlanConfig) -> f64 {
    generate(&FlightRequest {
        departure: LatLon::new(dep.0, dep.1),
        arrival: LatLon::new(arr.0, arr.1),
        target_duration_ms: 90 * 60_000,
        dep_heading_deg: None,
        arr_heading_deg: None,
        runways: Vec::new(),
        config,
    })[0]
        .altitude
}

/// What the camera ended up at, and what it saw.
struct Shot {
    visible_tiles: usize,
    height_tiles: usize,
    altitude_m: f64,
    agl_m: Option<f64>,
}

/// Renders one pose with its own settle loop.
///
/// Deliberately the same three-condition settle as `rendering::terrain_capture` — no
/// missing meshes, nothing loading anywhere, and eight quiet frames — because with
/// terrain on the shared headless helper captures a frame whose near field has no
/// geometry yet. See that module's note.
async fn render_settled(
    width: u32,
    height: u32,
    config: TileEngineConfig,
    pose: &Pose,
    out_path: &str,
) -> Shot {
    use cesium_engine::render::wgpu_state::WgpuState;

    let mut state = WgpuState::new(
        None,
        Some(winit::dpi::PhysicalSize::new(width, height)),
        config,
        None,
    )
    .await;

    state.camera.set_eye_with_up(pose.eye, pose.target, pose.up);

    let aspect = state.size.width as f32 / state.size.height as f32;
    let view_proj = state.camera.get_projection_matrix(aspect) * state.camera.get_view_matrix();

    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(120);
    let mut quiet = 0;
    let mut visible;
    loop {
        state
            .tile_system
            .texture_manager
            .fetch_and_upload_all(&state.device, &state.queue, &[])
            .await;
        visible = state.update_logic(aspect, view_proj).len();

        let settled =
            state.last_missing_tiles_count == 0 && state.tile_system.is_loading_complete();
        quiet = if settled { quiet + 1 } else { 0 };
        if quiet >= 8 || std::time::Instant::now() >= deadline {
            break;
        }
        std::thread::sleep(std::time::Duration::from_millis(50));
    }

    let height_tiles = state
        .tile_system
        .height_manager
        .as_ref()
        .map(|h| h.residency().0)
        .unwrap_or(0);
    let altitude_m = state.camera.altitude() as f64 * 1.0e6;
    let agl_m = state
        .camera
        .ground_height()
        .map(|_| state.camera.altitude_agl() as f64 * 1.0e6);

    #[cfg(feature = "debug_panel")]
    let res = state.render(Some(out_path), false, |_, _| {});
    #[cfg(not(feature = "debug_panel"))]
    let res = state.render(Some(out_path), false);
    res.expect("headless render");

    Shot {
        visible_tiles: visible,
        height_tiles,
        altitude_m,
        agl_m,
    }
}

/// Renders `pose` once per `(suffix, terrain)` pair, printing what it is and where each
/// shot landed.
fn capture_all(pose: &Pose, shots: &[(&str, bool)]) {
    println!("  {} — {}", pose.name, pose.what);
    for (suffix, terrain) in shots {
        capture(pose, suffix, *terrain);
    }
}

fn capture(pose: &Pose, suffix: &str, terrain: bool) {
    let dir = shot_dir();
    let out = dir.join(format!("{}_{suffix}.png", pose.name));
    let out_str = out.to_string_lossy().into_owned();
    let shot = pollster::block_on(render_settled(1280, 720, config(terrain), pose, &out_str));
    println!(
        "    {suffix:<12} asked {:8.1} m, eye {:8.1} m ellipsoid, {} AGL, \
         {:4} tiles, {:3} height tiles -> {}",
        pose.requested_alt_m,
        shot.altitude_m,
        match shot.agl_m {
            Some(a) => format!("{a:8.1} m"),
            None => "       - (terrain off)".to_string(),
        },
        shot.visible_tiles,
        shot.height_tiles,
        out.display()
    );
}

/// Field elevation at two mountain airports: Innsbruck (581 m, at the bottom of a 2 km trench) and
/// Bogotá (2 548 m, on a plateau).
///
/// Three shots each, all from the same place, differing only in the altitude the planner
/// hands over and whether the globe has terrain:
///
/// - `on_ground` — the altitude field elevation now produces, on a globe with terrain. The eye is
///   300 m over the runway; the ground should be right there under it.
/// - `sea_level` — the altitude the old default produced, on the same globe. That
///   position is inside the mountainside, so the collision floor refuses it and the printed "asked" and
///   "eye" differ by the field elevation. The gap *is* the error the flip removes.
/// - `flat_globe` — the `on_ground` altitude with the globe's terrain switched off. The
///   documented caveat: a correct plan floating over a surface that is not there.
#[test]
#[ignore = "visual verification: needs the network for imagery and heights, writes PNGs"]
fn capture_field_elevation_at_mountain_airports() {
    println!("  writing captures to {}", shot_dir().display());

    // (name, dep lat/lon, dep elevation, arr lat/lon, arr elevation, bearing, what)
    //
    // The elevations are the published field elevations, which is where they come from
    // in production too — `nativeSetFieldElevations`, out of the caller's airport
    // database. The globe's own DEM reads 579 m and 2 546 m at these two thresholds;
    // that agreement is what this capture is really about.
    let fields: [(&'static str, (f64, f64), f64, (f64, f64), f64, f64, &str); 2] = [
        (
            "lowi_innsbruck",
            (47.2602, 11.3439),
            581.0,
            (50.0333, 8.5706), // Frankfurt
            111.0,
            250.0, // down the Inn valley, roughly runway 26
            "west down the Inn valley, the Nordkette wall on the right",
        ),
        (
            "skbo_bogota",
            (4.7016, -74.1469),
            2_548.0,
            (19.4363, -99.0721), // Mexico City
            2_230.0,
            70.0, // across the sabana towards the eastern hills
            "east across the sabana de Bogotá to the Cerros Orientales",
        ),
    ];

    for (name, dep, dep_elev, arr, arr_elev, bearing, what) in fields {
        let with_elevation = FlightPlanConfig {
            dep_elevation_m: dep_elev,
            arr_elevation_m: arr_elev,
            ..FlightPlanConfig::default()
        };
        let on_ground_m = departure_altitude_m(dep, arr, with_elevation);
        let sea_level_m = departure_altitude_m(
            dep,
            arr,
            FlightPlanConfig {
                terrain_elevation: false,
                ..with_elevation
            },
        );
        println!(
            "\n  planner at {name}: starts at {on_ground_m:.0} m with E3.1, \
             {sea_level_m:.0} m without"
        );

        // 300 m of eye height over the aircraft's own starting altitude — a short final
        // rather than a view from inside the fuselage, and far enough off the ground
        // that the shot is about where the ground *is* and not about the 2 m clearance.
        let view = |alt: f64| {
            look(
                name,
                dep.1,
                dep.0,
                alt + 300.0,
                bearing,
                4.0,
                0.20,
                what.into(),
            )
        };

        capture_all(
            &view(on_ground_m),
            &[("on_ground", true), ("flat_globe", false)],
        );
        capture_all(&view(sea_level_m), &[("sea_level", true)]);
    }
}

/// The collision floor: a pose 1 400 m inside the Nordkette.
///
/// The ridge at (47.3167 N, 11.3833 E) is at 2 043 m in the Terrarium source. The pose
/// asks for 600 m. Previously the camera stayed there, inside the rock; now,
/// `enforce_bounds` lifts it to 2 045 m and the printed altitude says so. The
/// `above_ridge` shot is the same place from 2 400 m, as a reference for what the ridge
/// is supposed to look like from up there.
#[test]
#[ignore = "visual verification: needs the network for imagery and heights, writes PNGs"]
fn capture_the_collision_floor_at_the_nordkette() {
    println!("  writing captures to {}", shot_dir().display());
    println!("  nordkette — the ridge is at 2 043 m; the sunk pose asks for 600 m");

    let sunk = look(
        "nordkette_sunk",
        11.3833,
        47.3167,
        600.0,
        0.0,
        2.0,
        0.30,
        "north off the Nordkette ridge, from a pose placed 1 400 m inside it".to_string(),
    );
    let above = look(
        "nordkette_above",
        11.3833,
        47.3167,
        2_400.0,
        0.0,
        2.0,
        0.30,
        "the same view from 2 400 m, where nothing has to be pushed out of anything".to_string(),
    );

    // With terrain off there is no ground and therefore no floor: the camera stays at
    // 600 m over a sea-level sphere. That frame is the control, not a failure.
    capture_all(&sunk, &[("terrain_on", true), ("terrain_off", false)]);
    capture_all(&above, &[("terrain_on", true)]);
}
