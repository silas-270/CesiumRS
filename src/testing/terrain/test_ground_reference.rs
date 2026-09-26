//! Ground reference: the engine stops assuming the
//! surface is the ellipsoid — first the near plane, then the floor the camera stands on.
//!
//! Four things are checked here, and the third is the load-bearing one.
//!
//! 1. **The geodetic query lands where the mesh does.** `ecef_to_lon_lat_f64` inverts
//!    this engine's own ECEF map (parametric latitude, not geodetic — 11 km of ground
//!    apart at mid-latitude), and `tile_uv_at_lon_lat` inverts the Web-Mercator row
//!    definition every tile boundary and every mesh row is derived from.
//! 2. **It answers from whatever has landed**, deep tile or distant ancestor.
//! 3. **With no ground known, nothing moved.** `altitude_agl` is `altitude` and both
//!    projection matrices are bit-identical to the previous expression, recomputed here
//!    from the old formula rather than recorded from a run. With terrain off
//!    `TileSystem::ground_height_at` returns `None` on every frame, so case 3 *is* the
//!    flat path.
//! 4. **The ground is a floor.** `enforce_bounds` stops flying the camera through the
//!    Alps, on Cesium's `minimumCollisionTerrainHeight` shape — and with terrain off
//!    keeps the same 2 m ellipsoid clearance, to the bit.

use std::sync::Arc;

use cesium_engine::camera::camera::{Camera, CameraMode};
use cesium_engine::globe::geometry::{ecef_to_lon_lat_f64, lon_lat_to_ecef_f64};
use cesium_engine::globe::quadtree::web_mercator_y_to_lat_f64;
use cesium_engine::globe::terrain::height_cache::HeightTileManager;
use cesium_engine::globe::terrain::height_tile::{HEIGHT_TILE_DIM, HEIGHT_TILE_TEXELS};
use cesium_engine::globe::terrain::HeightTile;
use cesium_engine::globe::tiles::config::{TerrainConfig, TileEngineConfig};
use cesium_engine::globe::tiles::system::TileSystem;
use glam::{Mat4, Vec3};

/// Megametres per metre, this engine's world unit over the DEM's.
const M_TO_MM: f64 = 1.0e-6;

fn terrain_manager() -> HeightTileManager {
    HeightTileManager::new(&TileEngineConfig {
        terrain: TerrainConfig {
            enabled: true,
            ..TerrainConfig::default()
        },
        ..TileEngineConfig::default()
    })
}

/// A field that is exactly linear in the tile's own `x` texel index, so a bilinear
/// sample has a closed form and a displaced UV shows up as a plain numeric mismatch.
fn x_ramp_metres_per_texel(scale: f64) -> Arc<HeightTile> {
    let mut data = Box::new([0i16; HEIGHT_TILE_TEXELS]);
    for y in 0..HEIGHT_TILE_DIM {
        for x in 0..HEIGHT_TILE_DIM {
            data[y * HEIGHT_TILE_DIM + x] = (x as f64 * scale) as i16;
        }
    }
    Arc::new(HeightTile::from_samples(data))
}

// ── 1. The geodetic query lands where the mesh does ──────────────────────────

/// Five landmarks, and the z15 tile each falls in.
///
/// `(name, lon, lat, x, y, u, v)`. The tile indices and the in-tile position were
/// computed from the textbook slippy-map formula in a standalone Python pass, *not*
/// read off this implementation, so what follows asserts agreement between two
/// derivations rather than recording one.
const LANDMARKS: &[(&str, f64, f64, u32, u32, f64, f64)] = &[
    (
        "LOWI Innsbruck",
        11.3439,
        47.2602,
        17416,
        11490,
        0.546_986_666_664_452_1,
        0.552_600_599_483_412_2,
    ),
    (
        "Denver",
        -104.9903,
        39.7392,
        6827,
        12436,
        0.549_582_222_221_943_1,
        0.211_454_032_309_120_52,
    ),
    (
        "SKBO Bogota",
        -74.1469,
        4.7016,
        9634,
        15955,
        0.984_391_111_111_108_3,
        0.568_837_169_726_975_8,
    ),
    (
        "MMMX Mexico City",
        -99.0721,
        19.4363,
        7366,
        14579,
        0.237_297_777_776_802_83,
        0.924_165_942_520_630_8,
    ),
    (
        "Zugspitze",
        10.9853,
        47.4211,
        17383,
        11468,
        0.906_417_777_776_368_9,
        0.940_043_959_697_504_8,
    ),
];

#[test]
fn ecef_round_trips_to_the_latitude_the_tiling_uses() {
    for &(name, lon, lat, ..) in LANDMARKS {
        let p = lon_lat_to_ecef_f64(lon, lat);
        let (back_lon, back_lat) = ecef_to_lon_lat_f64(glam::DVec3::from_array(p));
        assert!(
            (back_lon - lon).abs() < 1e-11 && (back_lat - lat).abs() < 1e-11,
            "{name}: ({lon}, {lat}) came back as ({back_lon}, {back_lat})"
        );
    }
    // The poles and the antimeridian, where an `atan2` with the wrong argument order
    // or a sign error is invisible everywhere else.
    for &(lon, lat) in &[
        (180.0, 0.0),
        (-180.0, 0.0),
        (0.0, 85.0),
        (0.0, -85.0),
        (179.999, -84.9),
    ] {
        let p = lon_lat_to_ecef_f64(lon, lat);
        let (back_lon, back_lat) = ecef_to_lon_lat_f64(glam::DVec3::from_array(p));
        assert!(
            (back_lat - lat).abs() < 1e-11 && (back_lon.abs() - lon.abs()).abs() < 1e-9,
            "({lon}, {lat}) came back as ({back_lon}, {back_lat})"
        );
    }
}

/// Altitude must not move the answer: the query is fed a camera position kilometres up.
#[test]
fn altitude_does_not_move_the_ground_position() {
    for &(name, lon, lat, ..) in LANDMARKS {
        let surface = glam::DVec3::from_array(lon_lat_to_ecef_f64(lon, lat));
        let (s_lon, s_lat) = ecef_to_lon_lat_f64(surface);
        // 12 km straight up, radially — the worst case for a parametric latitude.
        let (a_lon, a_lat) = ecef_to_lon_lat_f64(surface * (1.0 + 12_000.0 * M_TO_MM / 6.378_137));
        assert!(
            (a_lon - s_lon).abs() < 1e-9,
            "{name}: longitude moved by {} deg with altitude",
            a_lon - s_lon
        );
        // The flattening tilts the parametric latitude a little as the point rises.
        // 1e-4 deg is 11 m of ground: far inside the DEM's 30 m posts, which is the
        // resolution this query is ever asked for.
        assert!(
            (a_lat - s_lat).abs() < 1e-4,
            "{name}: latitude moved by {} deg with 12 km of altitude",
            a_lat - s_lat
        );
    }
}

#[test]
fn a_position_lands_in_the_tile_the_slippy_formula_names() {
    for &(name, lon, lat, x, y, u, v) in LANDMARKS {
        let (id, got_u, got_v) = HeightTileManager::tile_uv_at_lon_lat(lon, lat, 15);
        assert_eq!(
            (id.z, id.x, id.y),
            (15, x, y),
            "{name} landed in {id:?} rather than z15/{x}/{y}"
        );
        assert!(
            (got_u - u).abs() < 1e-9 && (got_v - v).abs() < 1e-9,
            "{name}: ({got_u}, {got_v}) rather than ({u}, {v})"
        );
    }
}

/// The inverse is the inverse of *this engine's* row definition, not of a textbook
/// Mercator: `web_mercator_y_to_lat_f64(y + v, z)` is the expression `HeightPatch` and
/// `TileMesh::generate` build their rows from, so the round trip through it is what
/// decides whether a sample lands on the ground the mesh draws.
#[test]
fn the_in_tile_position_round_trips_through_the_mesh_row_definition() {
    for z in [4u8, 8, 12, 15] {
        for &(name, lon, lat, ..) in LANDMARKS {
            let (id, u, v) = HeightTileManager::tile_uv_at_lon_lat(lon, lat, z);
            let back_lat = web_mercator_y_to_lat_f64(id.y as f64 + v, z);
            let n = (1_u64 << z) as f64;
            let back_lon = -180.0 + (id.x as f64 + u) * 360.0 / n;
            assert!(
                (back_lat - lat).abs() < 1e-9,
                "{name} z{z}: latitude {back_lat} rather than {lat}"
            );
            assert!(
                (back_lon - lon).abs() < 1e-9,
                "{name} z{z}: longitude {back_lon} rather than {lon}"
            );
        }
    }
}

#[test]
fn latitudes_past_the_mercator_limit_clamp_into_the_polar_row() {
    for z in [4u8, 15] {
        let n = 1_u32 << z;
        for lat in [89.9, 90.0, -89.9, -90.0] {
            let (id, _, v) = HeightTileManager::tile_uv_at_lon_lat(0.0, lat, z);
            let expected_row = if lat > 0.0 { 0 } else { n - 1 };
            assert_eq!(
                id.y, expected_row,
                "lat {lat} at z{z} landed in row {}",
                id.y
            );
            assert!(
                (0.0..1.0).contains(&v),
                "lat {lat} at z{z} gave v = {v}, outside the tile"
            );
        }
    }
}

// ── 2. It answers from whatever has landed ───────────────────────────────────

#[test]
fn a_resident_tile_answers_under_a_geodetic_position() {
    let mut heights = terrain_manager();
    let (lon, lat) = (LANDMARKS[0].1, LANDMARKS[0].2);
    let (id, u, v) = HeightTileManager::tile_uv_at_lon_lat(lon, lat, 15);
    heights.insert_ready(id, x_ramp_metres_per_texel(4.0));

    let geodetic = heights
        .peek_height_at_lon_lat(lon, lat)
        .expect("a resident tile under the position");
    let by_uv = heights
        .peek_height_at(id, u, v)
        .expect("the same tile, addressed directly");
    assert_eq!(
        geodetic.to_bits(),
        by_uv.to_bits(),
        "the geodetic entry point disagreed with the uv one: {geodetic} vs {by_uv}"
    );

    // And it is the ramp's own value, so a UV displaced by a tile or a quadrant would
    // not pass: `h = 4 m × (u·DIM − 0.5)`, the bilinear sample of a field linear in the
    // texel index, on `sample_bilinear`'s texel-centre convention.
    let expected = 4.0 * (u * HEIGHT_TILE_DIM as f64 - 0.5) * M_TO_MM;
    assert!(
        (geodetic - expected).abs() < 1e-9,
        "sampled {} m, expected {} m",
        geodetic / M_TO_MM,
        expected / M_TO_MM
    );
}

#[test]
fn an_ancestor_answers_when_the_deep_tile_has_not_landed() {
    let mut heights = terrain_manager();
    let (lon, lat) = (LANDMARKS[0].1, LANDMARKS[0].2);

    assert!(
        heights.peek_height_at_lon_lat(lon, lat).is_none(),
        "answered from an empty cache"
    );

    // Only a z6 ancestor is resident — the cruise-altitude case.
    let (coarse, cu, _) = HeightTileManager::tile_uv_at_lon_lat(lon, lat, 6);
    heights.insert_ready(coarse, x_ramp_metres_per_texel(4.0));

    let h = heights
        .peek_height_at_lon_lat(lon, lat)
        .expect("the z6 ancestor should answer for the z15 position");
    let expected = 4.0 * (cu * HEIGHT_TILE_DIM as f64 - 0.5) * M_TO_MM;
    assert!(
        (h - expected).abs() < 1e-9,
        "the ancestor walk sampled {} m, expected {} m",
        h / M_TO_MM,
        expected / M_TO_MM
    );
}

/// The one line that makes every consumer's flat path unreachable: with terrain off
/// there is no height manager, so `TileSystem::ground_height_at`'s first `?` returns
/// `None` before any geometry happens.
#[test]
fn no_height_manager_exists_at_all_with_terrain_off() {
    let off = TileEngineConfig {
        terrain: TerrainConfig {
            enabled: false,
            ..TerrainConfig::default()
        },
        ..TileEngineConfig::default()
    };
    assert!(TileSystem::build_height_manager(&off).is_none());
}

// ── 3. With no ground known, nothing moved ───────────────────────────────────

/// The poses the bit-exactness checks run over: cruise, approach, valley floor, the
/// top of the Nordkette, and a low orbit.
fn poses() -> Vec<(&'static str, Camera)> {
    let eye = |lon: f64, lat: f64, alt_m: f64| {
        let p = cesium_engine::globe::geometry::lon_lat_alt_to_ecef_f64(lon, lat, alt_m);
        Vec3::new(p[0] as f32, p[1] as f32, p[2] as f32)
    };
    let mut out = Vec::new();
    for (name, lon, lat, alt) in [
        ("cruise_over_the_alps", 11.3439, 47.2602, 11_000.0),
        ("approach_into_lowi", 11.3439, 47.2602, 1_800.0),
        ("inn_valley_floor", 11.40, 47.26, 900.0),
        ("nordkette_summit", 11.3833, 47.3167, 2_300.0),
        ("low_orbit", 86.925, 27.35, 400_000.0),
    ] {
        let e = eye(lon, lat, alt);
        let mut cam = Camera::new(e, Vec3::ZERO);
        cam.set_eye(e, Vec3::ZERO);
        out.push((name, cam));
    }
    out
}

#[test]
fn agl_is_bitwise_the_ellipsoid_altitude_when_no_ground_is_known() {
    for (name, cam) in poses() {
        assert!(
            cam.ground_height().is_none(),
            "{name}: a fresh camera should know no ground"
        );
        assert_eq!(
            cam.altitude_agl().to_bits(),
            cam.altitude().to_bits(),
            "{name}: agl {} vs ellipsoid {}",
            cam.altitude_agl(),
            cam.altitude()
        );
    }
}

/// The previous projection matrix, recomputed here from the formula the file carried
/// before — `alt = altitude()`, everything else untouched.
///
/// Written out rather than snapshotted, so it fails if the *shape* of the near/far
/// derivation changes and not merely if a number does.
fn pre_e3_projection(cam: &Camera, aspect: f32) -> Mat4 {
    let alt = cam.altitude().max(0.000002);
    let znear = match cam.mode {
        CameraMode::Free => (alt * 0.1).clamp(0.0000001, 10.0),
        CameraMode::Tracking => {
            let dist = cam.local_pos.length();
            (dist * 0.05).clamp(0.00000001, 0.000005)
        }
        CameraMode::Cockpit => unreachable!("cockpit's near plane never read the altitude"),
    };
    let (pos, _) = cam.global_transform();
    let zfar = pos.length() + 10.0;
    let proj = Mat4::perspective_rh(cam.fovy(), aspect, znear, zfar);
    let reverse_z = Mat4::from_cols_array(&[
        1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0, 1.0,
    ]);
    reverse_z * proj
}

/// [`pre_e3_projection`] in f64 — the matrix the culling frustum is built from, and
/// therefore the one the pinned visible-set digest depends on.
fn pre_e3_projection_f64(cam: &Camera, aspect: f64) -> glam::DMat4 {
    let alt = cam.altitude().max(0.000002) as f64;
    let znear = match cam.mode {
        CameraMode::Free => (alt * 0.1).clamp(0.0000001, 10.0),
        CameraMode::Tracking => {
            let dist = cam.local_pos.length() as f64;
            (dist * 0.05).clamp(0.00000001, 0.000005)
        }
        CameraMode::Cockpit => unreachable!("cockpit's near plane never read the altitude"),
    };
    let (pos, _) = cam.global_transform_f64();
    let zfar = pos.length() + 10.0;
    let proj = glam::DMat4::perspective_rh(cam.fovy_f64(), aspect, znear, zfar);
    let reverse_z = glam::DMat4::from_cols_array(&[
        1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0, 1.0,
    ]);
    reverse_z * proj
}

#[test]
fn the_projection_matrix_is_bitwise_unchanged_when_no_ground_is_known() {
    for (name, mut cam) in poses() {
        for mode in [CameraMode::Free, CameraMode::Tracking] {
            cam.mode = mode;
            for aspect in [16.0 / 9.0_f32, 1.0, 0.46] {
                let got = cam.get_projection_matrix(aspect);
                let want = pre_e3_projection(&cam, aspect);
                for i in 0..16 {
                    assert_eq!(
                        got.to_cols_array()[i].to_bits(),
                        want.to_cols_array()[i].to_bits(),
                        "{name} {mode:?} aspect {aspect}: element {i} moved, \
                         {} vs {}",
                        got.to_cols_array()[i],
                        want.to_cols_array()[i]
                    );
                }
                // The f64 matrix is what the culling frustum — and therefore the pinned
                // visible-set digest — is built from. It is pinned separately, against
                // the f64 form of the same previous expression. (It is *not* the f32
                // matrix widened: `fovy_f64` has always computed its own `atan`, and the
                // two differ in the last few digits by design.)
                let got64 = cam.get_projection_matrix_f64(aspect as f64);
                let want64 = pre_e3_projection_f64(&cam, aspect as f64);
                for i in 0..16 {
                    assert_eq!(
                        got64.to_cols_array()[i].to_bits(),
                        want64.to_cols_array()[i].to_bits(),
                        "{name} {mode:?} aspect {aspect}: f64 element {i} moved"
                    );
                }
            }
        }
    }
}

/// The change itself. At 900 m over the Inn valley, whose floor the Terrarium source
/// puts at 583 m under `(11.40 E, 47.26 N)`, the old near plane is
/// `0.1 × 900 m = 90 m` and the wall of rock across the valley is well inside it; the
/// new one is `0.1 × 317 m = 32 m`.
#[test]
fn the_near_plane_follows_the_ground_not_the_ellipsoid() {
    let p = cesium_engine::globe::geometry::lon_lat_alt_to_ecef_f64(11.40, 47.26, 900.0);
    let e = Vec3::new(p[0] as f32, p[1] as f32, p[2] as f32);
    let mut cam = Camera::new(e, Vec3::ZERO);
    cam.set_eye(e, Vec3::ZERO);
    cam.mode = CameraMode::Free;

    let flat_znear = cam.get_projection_matrix(1.0);
    let ellipsoid_alt = cam.altitude();

    cam.set_ground_height(Some((583.0 * M_TO_MM) as f32));
    let agl = cam.altitude_agl();
    assert!(
        (agl - (ellipsoid_alt - (583.0 * M_TO_MM) as f32)).abs() < 1e-9,
        "agl came out {agl} Mm"
    );
    assert!(
        (agl / M_TO_MM as f32 - 317.0).abs() < 2.0,
        "expected about 317 m of clearance, got {} m",
        agl / M_TO_MM as f32
    );

    let terrain_znear = cam.get_projection_matrix(1.0);
    assert_ne!(
        terrain_znear.to_cols_array()[10].to_bits(),
        flat_znear.to_cols_array()[10].to_bits(),
        "the near plane did not move when the ground did"
    );

    // And handing back `None` restores the flat matrix exactly — the property the
    // whole design rests on, checked here on a camera that has been in both states.
    cam.set_ground_height(None);
    let restored = cam.get_projection_matrix(1.0);
    for i in 0..16 {
        assert_eq!(
            restored.to_cols_array()[i].to_bits(),
            flat_znear.to_cols_array()[i].to_bits(),
            "element {i} did not come back"
        );
    }
}

/// A camera below the sampled ground must not ask for a negative near plane.
///
/// The collision floor makes this hard to reach — `set_ground_height` pushes the
/// camera out — but not impossible: `set_eye_with_up` deliberately does not enforce
/// bounds, so a pose placed after the ground is known lands wherever it was told to.
/// The clamp in `altitude_agl` is what stands behind that.
#[test]
fn a_camera_below_the_sampled_ground_still_has_a_positive_near_plane() {
    let mut cam = camera_at(11.40, 47.26, 900.0);
    cam.mode = CameraMode::Free;
    cam.set_ground_height(Some((900.0 * M_TO_MM) as f32));
    // Placed *after* the ground is known, so nothing pushes it back out.
    cam.set_eye(eye_at(11.40, 47.26, 600.0), Vec3::ZERO);

    assert_eq!(cam.altitude_agl(), 0.0, "clearance should clamp at zero");
    let m = cam.get_projection_matrix(1.0);
    assert!(
        m.to_cols_array().iter().all(|x| x.is_finite()),
        "projection matrix is not finite below ground: {m:?}"
    );
}

// ── 4. The collision floor: the ground is a floor, not a suggestion ──────────

/// The distance from the Earth's centre to the ellipsoid along a given direction —
/// `t` in `enforce_bounds`, recomputed here from the same expression so the floor
/// assertions below are against the engine's ellipsoid and not a sphere.
fn ellipsoid_radius_at(p: glam::DVec3) -> f64 {
    const INV_A2: f64 = 1.0 / (6.378137 * 6.378137);
    const INV_B2: f64 = 1.0 / (6.3567523142 * 6.3567523142);
    let d = p.normalize_or_zero();
    1.0 / (d.x * d.x * INV_A2 + d.y * d.y * INV_B2 + d.z * d.z * INV_A2).sqrt()
}

fn ellipsoid_radius_under(cam: &Camera) -> f64 {
    ellipsoid_radius_at(cam.global_transform_f64().0)
}

fn eye_at(lon: f64, lat: f64, alt_m: f64) -> Vec3 {
    let p = cesium_engine::globe::geometry::lon_lat_alt_to_ecef_f64(lon, lat, alt_m);
    Vec3::new(p[0] as f32, p[1] as f32, p[2] as f32)
}

fn camera_at(lon: f64, lat: f64, alt_m: f64) -> Camera {
    let e = eye_at(lon, lat, alt_m);
    let mut cam = Camera::new(e, Vec3::ZERO);
    // Free: these poses are world positions, not offsets from an aircraft. The default
    // became Tracking after these tests were written, and the clamp took the orbit arm.
    cam.mode = cesium_engine::camera::CameraMode::Free;
    cam.set_eye(e, Vec3::ZERO);
    cam
}

/// The collision pass the engine runs each frame, against a flat ground of `metres`.
fn collide_with_ground(cam: &mut Camera, metres: f64) {
    let ground = metres * M_TO_MM;
    cam.enforce_bounds_with(&|_| Some(ground));
}

/// `enforce_bounds`'s clamp arm, written out: where a camera pointing this way ends up
/// when it is pushed out to `floor` megametres from the centre.
///
/// The comparison has to be made in `local_pos`, which is `f32`. The clamp's own
/// arithmetic is f64 down to one `as f32` per component, so comparing `p.length()`
/// against an f64 floor would be comparing against a number the camera cannot represent
/// — 0.76 m of quantisation at Earth radius. Reproducing the narrowing here instead
/// makes the assertion exact.
fn clamped_local_pos(cam: &Camera, floor: f64) -> Vec3 {
    clamped_local_pos_from(cam, cam.global_transform_f64().0, floor)
}

/// [`clamped_local_pos`] for a camera that is about to be *moved* to `global` — the
/// clamp reads the direction of the position it is given, not the one it had.
fn clamped_local_pos_from(cam: &Camera, global: glam::DVec3, floor: f64) -> Vec3 {
    let placed = global.normalize_or_zero() * floor;
    let local = cam.anchor_ori.inverse() * (placed - cam.anchor_pos);
    Vec3::new(local.x as f32, local.y as f32, local.z as f32)
}

/// A position `alt_m` metres from the ellipsoid — negative altitudes included, which is
/// how the tests below get a camera underground without going through a clamp on the
/// way — returned both as the `f32` the camera will store and as the `f64` the clamp
/// will read back out of it.
///
/// Both, and from the same narrowing, because `local_pos` is `f32`: an expectation
/// derived from the un-narrowed f64 differs in the last bit, which is 0.4 m at Earth
/// radius and enough to fail a bitwise assertion for no reason at all.
fn placed_at(lon: f64, lat: f64, alt_m: f64) -> (Vec3, glam::DVec3) {
    let v = eye_at(lon, lat, alt_m);
    (v, glam::DVec3::new(v.x as f64, v.y as f64, v.z as f64))
}

/// Ground below the camera changes nothing — and changes it to the bit.
#[test]
fn ground_below_the_camera_does_not_move_it() {
    let mut cam = camera_at(11.40, 47.26, 900.0);
    let before = cam.local_pos;
    collide_with_ground(&mut cam, 583.0);
    assert_eq!(
        cam.local_pos.to_array().map(f32::to_bits),
        before.to_array().map(f32::to_bits),
        "a camera 317 m above the valley floor was moved"
    );
}

/// The one this phase exists for: a camera inside the Nordkette comes out onto it.
///
/// Previously `enforce_bounds` kept the camera 2 m off the *ellipsoid*, so a pose at
/// 600 m under a 2 000 m ridge sat 1 400 m inside solid rock and the globe was drawn
/// from within the mountain.
#[test]
fn a_camera_inside_a_mountain_is_pushed_onto_its_surface() {
    let mut cam = camera_at(11.3833, 47.3167, 600.0);
    assert!(cam.altitude() < (700.0 * M_TO_MM) as f32);

    let floor = ellipsoid_radius_under(&cam) + 2_000.0 * M_TO_MM + 2.0 * M_TO_MM;
    let want = clamped_local_pos(&cam, floor);
    collide_with_ground(&mut cam, 2_000.0);

    assert_eq!(
        cam.local_pos.to_array().map(f32::to_bits),
        want.to_array().map(f32::to_bits),
        "did not come to rest on the 2 002 m floor"
    );
    // And, stated the way it matters: it is out of the rock, 1.4 km higher than it was.
    assert!(
        (cam.altitude() / M_TO_MM as f32 - 2_002.0).abs() < 1.0,
        "ended at {} m",
        cam.altitude() / M_TO_MM as f32
    );
}

/// Cesium's `minimumCollisionTerrainHeight`, ported: above 15 km the terrain floor is
/// not enforced at all, because up there the sample is a coarse ancestor's average and
/// a floor built on it would move as tiles land.
#[test]
fn above_the_threshold_the_terrain_floor_is_not_enforced() {
    // 16 km of (exaggerated) ground, a camera at 20 km: above the gate, left alone.
    let mut high = camera_at(11.3833, 47.3167, 20_000.0);
    let before = high.local_pos;
    collide_with_ground(&mut high, 16_000.0);
    assert_eq!(
        high.local_pos.to_array().map(f32::to_bits),
        before.to_array().map(f32::to_bits),
        "a camera at 20 km was moved by a terrain floor it is above the gate for"
    );

    // The same ground, a camera at 14 km: under the gate, so the floor applies.
    let mut low = camera_at(11.3833, 47.3167, 14_000.0);
    let floor = ellipsoid_radius_under(&low) + 16_000.0 * M_TO_MM + 2.0 * M_TO_MM;
    let want = clamped_local_pos(&low, floor);
    collide_with_ground(&mut low, 16_000.0);
    assert_eq!(
        low.local_pos.to_array().map(f32::to_bits),
        want.to_array().map(f32::to_bits),
        "the camera under the gate was not lifted onto the floor"
    );
}

/// Ground below sea level must never *lower* the floor. The Dead Sea shore is at
/// −430 m; a DEM that says so should not license the camera to descend further than
/// the ellipsoid floor ever allowed it to.
#[test]
fn ground_below_sea_level_never_lowers_the_floor() {
    let mut cam = camera_at(35.5, 31.5, 0.0);
    cam.set_ground_height(Some((-430.0 * M_TO_MM) as f32));

    // Put it 5 km under the Dead Sea shore. `set_local_transform` writes the position
    // and then enforces bounds, so the clamp sees exactly this direction.
    let (sunk_v, sunk) = placed_at(35.5, 31.5, -5_000.0);
    let want = clamped_local_pos_from(&cam, sunk, ellipsoid_radius_at(sunk) + 0.000002);
    let ori = cam.local_ori;
    cam.set_local_transform(sunk_v, ori);

    assert_eq!(
        cam.local_pos.to_array().map(f32::to_bits),
        want.to_array().map(f32::to_bits),
        "a sub-sea-level DEM reading changed where the camera may descend to"
    );
}

/// With terrain off, `enforce_bounds` keeps the 2 m ellipsoid clearance it always kept
/// — checked by driving a camera into the surface and comparing against the previous
/// expression, bit for bit.
#[test]
fn the_ellipsoid_floor_is_bitwise_unchanged_with_no_ground_known() {
    for (lon, lat) in [(11.40, 47.26), (-104.99, 39.74), (0.0, 0.0), (0.0, 89.0)] {
        let mut cam = camera_at(lon, lat, 50.0);
        assert!(cam.ground_height().is_none());

        // 5 km underground, placed through the setter that enforces bounds.
        let (sunk_v, sunk) = placed_at(lon, lat, -5_000.0);
        // `t + 0.000002`, the previous expression, spelled out on the same operands.
        let want = clamped_local_pos_from(&cam, sunk, ellipsoid_radius_at(sunk) + 0.000002);
        let ori = cam.local_ori;
        cam.set_local_transform(sunk_v, ori);

        assert_eq!(
            cam.local_pos.to_array().map(f32::to_bits),
            want.to_array().map(f32::to_bits),
            "({lon}, {lat}): did not come to rest on the pre-E3 ellipsoid floor"
        );
    }
}

/// The flat path must not acquire an `enforce_bounds` call it did not have. A camera
/// placed outside its own distance clamp — which `set_eye` allows, because it does not
/// enforce anything — must still be exactly there after any number of `None` frames.
#[test]
fn feeding_none_every_frame_never_clamps_a_flat_camera() {
    let mut cam = camera_at(11.40, 47.26, 900.0);
    // Well past `max_distance` (6.378137 + 30 Mm), a state `set_eye` can leave behind
    // and `enforce_bounds` would silently correct.
    cam.set_eye(Vec3::new(0.0, 0.0, 100.0), Vec3::ZERO);
    let before = cam.local_pos;
    for _ in 0..240 {
        cam.set_ground_height(None);
    }
    assert_eq!(
        cam.local_pos.to_array().map(f32::to_bits),
        before.to_array().map(f32::to_bits),
        "a flat frame moved the camera"
    );
}

// ── 5. Labels stand on the ground ───────────────────────────────────────────

/// A stand-in height field: one answer everywhere, or nothing anywhere.
///
/// Synthetic on purpose. A real DEM would make the assertions below depend on what
/// Terrarium served that morning; what is under test is the lift, not the elevation.
struct FlatGround(Option<f32>);

impl cesium_engine::label::GroundHeights for FlatGround {
    fn ground_height_above_ellipsoid(&self, _pos: Vec3) -> Option<f32> {
        self.0
    }
}

/// A camera high over Denver, and the frustum the label pass culls against.
fn denver_view() -> (Camera, cesium_engine::globe::quadtree::Frustum) {
    let cam = camera_at(-104.9903, 39.7392, 900_000.0);
    let aspect = 16.0 / 9.0_f32;
    let (eye, _) = cam.global_transform_f64();
    let frustum = cesium_engine::globe::quadtree::Frustum::planes_only(
        cam.calculate_frustum_planes(aspect),
        eye,
    )
    .with_corners(cam.frustum_corners_relative(aspect));
    (cam, frustum)
}

/// Runs one label pass and returns `(name, position)` for everything visible.
///
/// A fresh `LabelManager` each time: the real one throttles itself to one update per
/// six frames unless the camera moves, so a second call on the same instance would
/// hand back the first call's answer.
fn labels_with(
    ground: Option<&dyn cesium_engine::label::GroundHeights>,
) -> Vec<(&'static str, Vec3)> {
    let (cam, frustum) = denver_view();
    let (pos, ori) = cam.global_transform();
    let mut labels = cesium_engine::label::LabelManager::new();
    labels.update(pos, ori, cam.altitude(), 15, &frustum, ground);
    labels
        .visible_labels
        .iter()
        .map(|l| (l.name, l.ecef_pos))
        .collect()
}

/// Height above the ellipsoid of an arbitrary point, in metres — `Camera::altitude`'s
/// expression, applied to something that is not a camera.
fn altitude_of(p: Vec3) -> f64 {
    let p = glam::DVec3::new(p.x as f64, p.y as f64, p.z as f64);
    (p.length() - ellipsoid_radius_at(p)) / M_TO_MM
}

#[test]
fn a_label_pass_with_no_ground_places_labels_exactly_where_it_always_did() {
    let flat = labels_with(None);
    assert!(
        flat.len() > 20,
        "only {} labels visible over Denver from 900 km; the fixture is not exercising anything",
        flat.len()
    );

    // Every one of them is on the ellipsoid, which is where the packed database puts
    // them — the lift did not run.
    for (name, p) in &flat {
        assert!(
            altitude_of(*p).abs() < 1.0,
            "{name} is {} m off the ellipsoid with no ground supplied",
            altitude_of(*p)
        );
    }

    // And a ground source that knows nothing is the same thing, to the bit.
    let unknown = labels_with(Some(&FlatGround(None)));
    assert_eq!(flat.len(), unknown.len());
    for ((n0, p0), (n1, p1)) in flat.iter().zip(unknown.iter()) {
        assert_eq!(n0, n1);
        assert_eq!(
            p0.to_array().map(f32::to_bits),
            p1.to_array().map(f32::to_bits),
            "{n0} moved when the height query returned None"
        );
    }
}

/// Denver's label sat 1 609 m underground. It does not any more.
#[test]
fn a_label_is_lifted_onto_the_ground_beneath_it() {
    let flat = labels_with(None);
    let lifted = labels_with(Some(&FlatGround(Some((1_609.0 * M_TO_MM) as f32))));

    assert_eq!(
        flat.len(),
        lifted.len(),
        "the lift changed which labels are visible; it is only supposed to change where"
    );

    for ((name, before), (_, after)) in flat.iter().zip(lifted.iter()) {
        let rise = altitude_of(*after) - altitude_of(*before);
        // f32 positions at Earth radius resolve to ~0.4 m, and the lift is applied in
        // f32, so a metre of tolerance is the floor of what is observable.
        assert!(
            (rise - 1_609.0).abs() < 2.0,
            "{name} rose {rise:.1} m rather than 1609 m"
        );
        // Straight up, where "up" is the ellipsoid normal and not the radius: those
        // differ by up to 0.19°, so a 1 609 m lift carries a few metres of sideways
        // motion by construction. Anything beyond that is the lift being applied along
        // the wrong vector. Measured in f64 as a rejection, not as an `acos` of two
        // f32 unit vectors — near 1 that arccosine has no significant digits left and
        // reports ~2 km of drift for a pair that differ by one ulp.
        let b = glam::DVec3::new(before.x as f64, before.y as f64, before.z as f64);
        let a = glam::DVec3::new(after.x as f64, after.y as f64, after.z as f64);
        let radial = b.normalize();
        let d = a - b;
        let sideways = (d - radial * d.dot(radial)).length() / M_TO_MM;
        assert!(
            sideways < 20.0,
            "{name} moved {sideways:.1} m sideways for a 1 609 m lift"
        );
    }
}

/// The lift is applied after culling, so what is *visible* cannot depend on it — the
/// same set, in the same order, whatever the ground says.
#[test]
fn the_lift_does_not_change_which_labels_are_visible() {
    let names = |v: Vec<(&'static str, Vec3)>| v.into_iter().map(|(n, _)| n).collect::<Vec<_>>();
    let flat = names(labels_with(None));
    for h in [0.0_f32, 1_609.0, 8_849.0, -430.0] {
        let got = names(labels_with(Some(&FlatGround(Some(
            (h as f64 * M_TO_MM) as f32,
        )))));
        assert_eq!(
            got, flat,
            "the visible set changed with a ground height of {h} m"
        );
    }
}
