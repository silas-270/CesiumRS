//! Phase E3.2 of `docs/terrain-plan.md` §8: the engine stops assuming the surface is
//! the ellipsoid, starting with the near plane.
//!
//! Three things are checked here, and the third is the load-bearing one.
//!
//! 1. **The geodetic query lands where the mesh does.** `ecef_to_lon_lat_f64` inverts
//!    this engine's own ECEF map (parametric latitude, not geodetic — 11 km of ground
//!    apart at mid-latitude), and `tile_uv_at_lon_lat` inverts the Web-Mercator row
//!    definition every tile boundary and every mesh row is derived from.
//! 2. **It answers from whatever has landed**, deep tile or distant ancestor.
//! 3. **With no ground known, nothing moved.** `altitude_agl` is `altitude` and both
//!    projection matrices are bit-identical to the pre-E3 expression, recomputed here
//!    from the old formula rather than recorded from a run. With terrain off
//!    `TileSystem::ground_height_at` returns `None` on every frame, so case 3 *is* the
//!    flat path.

use std::sync::Arc;

use cesium_engine::camera::camera::{Camera, CameraMode};
use cesium_engine::globe::geometry::{ecef_to_lon_lat_f64, lon_lat_to_ecef_f64};
use cesium_engine::globe::quadtree::{web_mercator_y_to_lat_f64, TileId};
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

/// The pre-E3 projection matrix, recomputed here from the formula the file carried
/// before this phase — `alt = altitude()`, everything else untouched.
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
                // the f64 form of the same pre-E3 expression. (It is *not* the f32
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

/// The change itself. At 900 m over an Inn valley whose floor is at 574 m, the old
/// near plane is `0.1 × 900 m = 90 m` and the wall of rock across the valley is well
/// inside it; the new one is `0.1 × 326 m = 33 m`.
#[test]
fn the_near_plane_follows_the_ground_not_the_ellipsoid() {
    let p = cesium_engine::globe::geometry::lon_lat_alt_to_ecef_f64(11.40, 47.26, 900.0);
    let e = Vec3::new(p[0] as f32, p[1] as f32, p[2] as f32);
    let mut cam = Camera::new(e, Vec3::ZERO);
    cam.set_eye(e, Vec3::ZERO);
    cam.mode = CameraMode::Free;

    let flat_znear = cam.get_projection_matrix(1.0);
    let ellipsoid_alt = cam.altitude();

    cam.set_ground_height(Some((574.0 * M_TO_MM) as f32));
    let agl = cam.altitude_agl();
    assert!(
        (agl - (ellipsoid_alt - (574.0 * M_TO_MM) as f32)).abs() < 1e-9,
        "agl came out {agl} Mm"
    );
    assert!(
        (agl / M_TO_MM as f32 - 326.0).abs() < 2.0,
        "expected about 326 m of clearance, got {} m",
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

/// A camera below the sampled ground — which happens, because the sample is bilinear
/// over 30 m posts and the drawn mesh is a 16×16 patch of the same field, so they
/// disagree by metres — must not ask for a negative near plane.
#[test]
fn a_camera_below_the_sampled_ground_still_has_a_positive_near_plane() {
    let p = cesium_engine::globe::geometry::lon_lat_alt_to_ecef_f64(11.40, 47.26, 600.0);
    let e = Vec3::new(p[0] as f32, p[1] as f32, p[2] as f32);
    let mut cam = Camera::new(e, Vec3::ZERO);
    cam.set_eye(e, Vec3::ZERO);
    cam.mode = CameraMode::Free;
    cam.set_ground_height(Some((900.0 * M_TO_MM) as f32));

    assert_eq!(cam.altitude_agl(), 0.0, "clearance should clamp at zero");
    let m = cam.get_projection_matrix(1.0);
    assert!(
        m.to_cols_array().iter().all(|x| x.is_finite()),
        "projection matrix is not finite below ground: {m:?}"
    );
}
