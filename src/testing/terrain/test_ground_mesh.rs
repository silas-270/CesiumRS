//! **The ground query measures the surface that is drawn** — the correction to E3
//! (`docs/terrain-plan.md` §8) that this file exists to hold.
//!
//! E3 wired three things to the terrain: the camera's collision floor
//! (`Camera::enforce_bounds`), its clearance (`Camera::altitude_agl`) and label
//! placement. All three ask `TileSystem::ground_height_at`, and all three were being
//! answered about the **wrong surface**: `HeightTileManager::peek_height_at_lon_lat`
//! samples the 256×256 DEM bilinearly, while the renderer draws a triangle net through a
//! `(segments+1)²` sub-grid of it.
//!
//! The two agree at the net's own vertices and nowhere else, and the disagreement has a
//! sign:
//!
//! * over a **summit** the net chords *under* the peak, so the field reads higher than
//!   what is drawn — the floor is then conservative and nothing is wrong;
//! * over a **valley floor** the net chords *over* the dip, so the field reads **lower**
//!   than what is drawn, `ground_height_at` under-reports the ground, and
//!   `enforce_bounds` parks the camera underneath the visible surface.
//!
//! [`the_old_floor_put_the_camera_under_the_drawn_ground`] is the measurement of exactly
//! that, on the committed Zugspitze tile, in metres.
//!
//! # Why `culling` is not in this path
//!
//! Its siblings' reason, restated because it has been got wrong twice: `cargo test
//! --release --lib culling::` must keep reporting **32 passed, 0 failed, 1 ignored**, and
//! libtest's filter is a plain substring match on the full test path. Nothing under
//! `testing::terrain::` may contain the substring `culling`.

use std::sync::Arc;

use cesium_engine::globe::geometry::{ecef_to_lon_lat_f64, TileMesh};
use cesium_engine::globe::quadtree::TileId;
use cesium_engine::globe::terrain::{
    decode_terrarium, HeightPatch, HeightTile, HeightTileManager, Heightfield,
};
use cesium_engine::globe::tiles::config::{OceanPolicy, TerrainConfig, TileEngineConfig};
use cesium_engine::globe::tiles::system::DrawnMeshes;
use glam::DVec3;

/// The mesh density the engine ships (`TileEngineConfig::mesh_segments`, §9 F1).
const SEGMENTS: u32 = 16;

/// Megametres to metres — every number this file reports is in metres.
const MM_TO_M: f64 = 1.0e6;

/// The committed Zugspitze tile's id. z12, ~8.9 km of ground, 1 300 m of relief, and —
/// what this file needs — real valley floors between real ridges.
const TILE: TileId = TileId {
    z: 12,
    x: 2172,
    y: 1433,
};

/// The committed Zugspitze fixture, decoded. No network.
fn zugspitze() -> Arc<HeightTile> {
    let path = format!(
        "{}/assets/terrain_fixtures/zugspitze_z12_2172_1433.png",
        env!("CARGO_MANIFEST_DIR")
    );
    let bytes = std::fs::read(&path).unwrap_or_else(|e| panic!("reading {path}: {e}"));
    let img = image::load_from_memory(&bytes).unwrap().to_rgba8();
    let (w, h) = (img.width(), img.height());
    Arc::new(decode_terrarium(w, h, &img.into_raw(), OceanPolicy::ClampToZero).unwrap())
}

/// A height manager holding nothing but that one tile.
///
/// `max_level` is the shipped 15, so `peek_height_at_lon_lat` starts its walk at z15 and
/// lands on the z12 tile — the *same* tile `peek_mesh_height_at_lon_lat` resolves for a
/// z12 query. That is what makes the comparison below about the interpolation and
/// nothing else: identical data, identical source tile, identical sample function at the
/// corners.
fn manager_with_the_fixture() -> HeightTileManager {
    let mut heights = HeightTileManager::new(&TileEngineConfig {
        terrain: TerrainConfig {
            enabled: true,
            ..TerrainConfig::default()
        },
        ..TileEngineConfig::default()
    });
    heights.insert_ready(TILE, zugspitze());
    heights
}

/// How far inside its own rectangle a sample is held.
///
/// A tile's four edges belong to its neighbours just as much as to it, and
/// `tile_uv_at_lon_lat` decides the tie by a `floor`: at `u = 1` it answers with
/// `x + 1`, and that neighbour is not resident here, so the query is `None` and the
/// comparison has nothing to say. 10⁻⁹ of a tile is 9 µm of ground — four orders below
/// the smallest quantity in this file, and nine above the ulp of the `fx` the floor is
/// taken on.
const EDGE: f64 = 1.0e-9;

/// `(lon, lat)` at tile-local `(u, v)` of [`TILE`] — the inverse of the map
/// `HeightTileManager::tile_uv_at_lon_lat` applies, so a round trip through it is an
/// identity and a sample lands where this says it does.
fn lon_lat_at(u: f64, v: f64) -> (f64, f64) {
    let n = (1_u64 << TILE.z) as f64;
    let (u, v) = (u.clamp(EDGE, 1.0 - EDGE), v.clamp(EDGE, 1.0 - EDGE));
    let lon = (TILE.x as f64 + u) / n * 360.0 - 180.0;
    let lat = cesium_engine::globe::quadtree::web_mercator_y_to_lat_f64(TILE.y as f64 + v, TILE.z);
    (lon, lat)
}

/// The two answers at one point, in metres: the bilinear field, and the drawn net.
fn field_and_net(heights: &HeightTileManager, lon: f64, lat: f64) -> (f64, f64) {
    let field = heights
        .peek_height_at_lon_lat(lon, lat)
        .expect("the fixture is resident");
    let net = heights
        .peek_mesh_height_at_lon_lat(lon, lat, TILE.z, SEGMENTS)
        .expect("the fixture is resident");
    (field * MM_TO_M, net * MM_TO_M)
}

// ── 1. the disagreement, measured ────────────────────────────────────────────────

/// The size and the **sign** of what E3 was reading, over one real z12 tile.
///
/// Sampled on a lattice deliberately offset from the mesh's own: 16 segments means grid
/// lines every 1/16, and this walks 1/160 steps, so five sixths of the samples fall
/// strictly inside a triangle where the two surfaces are free to disagree.
///
/// What it pins:
///
/// 1. At a **grid node** the two agree — the net passes through the field at its own
///    vertices, which is what makes this an interpolation error and not an offset. The
///    tolerance is a tenth of a millimetre rather than an ulp because of [`EDGE`]: a node
///    on the tile's own boundary is held 10⁻⁹ of a tile inside it, and the two queries
///    resolve that inset on different lattices (a 1/16 grid step against a 1/256 texel).
///    Measured, the worst node is 3 µm out.
/// 2. The worst **positive** excursion (net above field: a valley floor chorded over) is
///    the number `ground_height_at` was wrong by, downward, and it is three digits of
///    metres at z12, the order `HeightTile::detail` predicts.
/// 3. Both signs occur, and the negative one (a summit chorded under) is the harmless
///    direction — the floor is then conservative.
#[test]
fn the_drawn_net_and_the_bilinear_field_disagree_by_metres_and_the_valley_sign_is_the_dangerous_one(
) {
    let heights = manager_with_the_fixture();
    let tile = zugspitze();

    // 1. The vertices themselves.
    for j in 0..=SEGMENTS {
        for i in 0..=SEGMENTS {
            let (lon, lat) = lon_lat_at(i as f64 / SEGMENTS as f64, j as f64 / SEGMENTS as f64);
            let (field, net) = field_and_net(&heights, lon, lat);
            assert!(
                (net - field).abs() < 1.0e-4,
                "at grid node ({i},{j}) the net reads {net} m and the field {field} m — \
                 they must agree at a vertex"
            );
        }
    }

    // 2. and 3. Between them.
    let steps = SEGMENTS * 10;
    let mut worst_up = (0.0_f64, 0.0_f64, 0.0_f64); // (diff, u, v) — net above field
    let mut worst_down = (0.0_f64, 0.0_f64, 0.0_f64); // net below field
    let mut sum_abs = 0.0;
    let mut n = 0u32;
    for j in 0..=steps {
        for i in 0..=steps {
            let (u, v) = (i as f64 / steps as f64, j as f64 / steps as f64);
            let (lon, lat) = lon_lat_at(u, v);
            let (field, net) = field_and_net(&heights, lon, lat);
            let d = net - field;
            sum_abs += d.abs();
            n += 1;
            if d > worst_up.0 {
                worst_up = (d, u, v);
            }
            if d < worst_down.0 {
                worst_down = (d, u, v);
            }
        }
    }

    println!(
        "  [E3 fix] z{} {} {}, mesh_segments = {SEGMENTS}",
        TILE.z, TILE.x, TILE.y
    );
    println!(
        "  [E3 fix] tile detail (HeightTile::detail): {} m",
        tile.detail()
    );
    println!("  [E3 fix] {n} samples on a 1/{steps} lattice");
    println!(
        "  [E3 fix] net ABOVE field (valley chorded over, the dangerous sign): +{:.1} m at u={:.3} v={:.3}",
        worst_up.0, worst_up.1, worst_up.2
    );
    println!(
        "  [E3 fix] net BELOW field (summit chorded under, conservative):      {:.1} m at u={:.3} v={:.3}",
        worst_down.0, worst_down.1, worst_down.2
    );
    println!(
        "  [E3 fix] mean |disagreement|: {:.2} m",
        sum_abs / n as f64
    );

    assert!(
        worst_up.0 > 100.0,
        "expected three digits of metres of valley under-reporting on a real Alpine \
         tile, got {:.1} m",
        worst_up.0
    );
    assert!(
        worst_down.0 < -100.0,
        "expected the summit sign to occur too, got {:.1} m",
        worst_down.0
    );
}

// ── 2. the net this file claims is the net that is drawn ─────────────────────────

/// One `(row, col)` of a built mesh's interior, as an ECEF point in megametres.
fn interior_vertex(mesh: &TileMesh, row: usize, col: usize) -> DVec3 {
    let grid = SEGMENTS as usize + 3;
    let v = &mesh.vertices[(row + 1) * grid + (col + 1)];
    DVec3::from_array(mesh.center_f64)
        + DVec3::new(
            v.position[0] as f64,
            v.position[1] as f64,
            v.position[2] as f64,
        )
}

/// Where the ray `origin + t·dir` crosses the **drawn triangle** under `(lon, lat)` —
/// the parameter `t`, megametres.
///
/// Built from the mesh's own vertices and its own bisection, in ECEF, so it is the
/// surface the rasteriser produces and not a restatement of the function under test.
/// Which ray is asked decides what the answer means, and the two callers below want
/// different things: along the **ellipsoid normal from the surface point** it is the
/// drawn altitude, the quantity `peek_mesh_height_at_lon_lat` returns; along the
/// **camera's own direction from the Earth's centre** it is the radius `enforce_bounds`
/// compares against.
fn drawn_ray_hit(mesh: &TileMesh, lon: f64, lat: f64, origin: DVec3, dir: DVec3) -> f64 {
    let (_, u, v) = HeightTileManager::tile_uv_at_lon_lat(lon, lat, TILE.z);
    let n = SEGMENTS as f64;
    let (i0, j0) = (
        (u * n).floor().min(n - 1.0) as usize,
        (v * n).floor().min(n - 1.0) as usize,
    );
    let (fu, fv) = (u * n - i0 as f64, v * n - j0 as f64);

    // Rows run south with `v`, columns east with `u`.
    let nw = interior_vertex(mesh, j0, i0);
    let ne = interior_vertex(mesh, j0, i0 + 1);
    let sw = interior_vertex(mesh, j0 + 1, i0);
    let se = interior_vertex(mesh, j0 + 1, i0 + 1);

    // The index loop's shared edge is `(row+1, col)`–`(row, col+1)`, i.e. SW–NE.
    let (a, b, c) = if 1.0 - fv < fu {
        (sw, se, ne)
    } else {
        (sw, ne, nw)
    };
    let normal = (b - a).cross(c - a);
    // Plane through `a`: `(origin + t·dir − a) · normal = 0`.
    (a - origin).dot(normal) / dir.dot(normal)
}

/// The ellipsoid's outward unit normal at `p` — the mesh's own `up`, the direction every
/// vertex altitude is applied along.
fn ellipsoid_normal(p: DVec3) -> DVec3 {
    const INV_A2: f64 = 1.0 / (6.378137 * 6.378137);
    const INV_B2: f64 = 1.0 / (6.3567523142 * 6.3567523142);
    DVec3::new(p.x * INV_A2, p.y * INV_B2, p.z * INV_A2).normalize()
}

/// The drawn surface's altitude above the ellipsoid at `(lon, lat)`, metres — measured
/// **along the normal**, which is where the mesh put its vertices.
fn drawn_altitude_m(mesh: &TileMesh, lon: f64, lat: f64) -> f64 {
    let s = DVec3::from_array(cesium_engine::globe::geometry::lon_lat_to_ecef_f64(
        lon, lat,
    ));
    drawn_ray_hit(mesh, lon, lat, s, ellipsoid_normal(s)) * MM_TO_M
}

/// The mesh the renderer builds for [`TILE`], from the fixture.
fn drawn_mesh(heights: &mut HeightTileManager) -> TileMesh {
    let patch = HeightPatch::sample(heights, TILE, SEGMENTS, 1.0).expect("fixture is resident");
    TileMesh::generate_on::<Heightfield>(&TILE, SEGMENTS, &patch)
}

/// The new query reproduces the drawn triangle, not merely something closer to it.
///
/// The residual is the two curvature terms the query's doc comment states and declines to
/// correct: it interpolates *altitude* where the triangle is a chord in ECEF, and it takes
/// its weights in `(u, v)` where the triangle is planar in space. Both are of the sagitta
/// order over one grid step of a z12 tile — millimetres — against a quantity in the
/// hundreds of metres.
#[test]
fn the_query_reproduces_the_triangle_the_renderer_draws() {
    let mut heights = manager_with_the_fixture();
    let mesh = drawn_mesh(&mut heights);

    let steps = 53; // coprime with 16, so nothing lands on a grid line by accident
    let mut worst: f64 = 0.0;
    for j in 0..=steps {
        for i in 0..=steps {
            let (lon, lat) = lon_lat_at(i as f64 / steps as f64, j as f64 / steps as f64);
            let from_mesh = drawn_altitude_m(&mesh, lon, lat);
            let from_query = heights
                .peek_mesh_height_at_lon_lat(lon, lat, TILE.z, SEGMENTS)
                .expect("resident")
                * MM_TO_M;
            worst = worst.max((from_mesh - from_query).abs());
        }
    }
    println!("  [E3 fix] query vs drawn triangle, worst over {steps}² samples: {worst:.4} m");
    assert!(
        worst < 0.05,
        "the query is {worst:.4} m from the triangle the renderer draws — the residual \
         should be the millimetre-scale sagitta over one grid step"
    );
}

// ── 3. the core: the camera is not under the ground it can see ───────────────────

/// The distance from the centre to the ellipsoid along `dir` — `t` in `enforce_bounds`,
/// recomputed from the same expression.
fn ellipsoid_radius_along(dir: DVec3) -> f64 {
    const INV_A2: f64 = 1.0 / (6.378137 * 6.378137);
    const INV_B2: f64 = 1.0 / (6.3567523142 * 6.3567523142);
    let d = dir.normalize_or_zero();
    1.0 / (d.x * d.x * INV_A2 + d.y * d.y * INV_B2 + d.z * d.z * INV_A2).sqrt()
}

/// The pose this file is built on: the point where the drawn net stands furthest **above**
/// the bilinear field, found by search over the tile rather than picked.
///
/// On this tile it is a notch between two ridges at ~2 280 m — the shape the failure needs
/// is a *local dip inside one mesh cell*, which is what a valley floor is at this scale and
/// what a cirque or a stream cut is at any scale. The net bridges it; the field follows it
/// down.
fn worst_dip_pose(heights: &HeightTileManager) -> (f64, f64, f64, f64) {
    let steps = SEGMENTS * 10;
    let mut best = (0.0_f64, 0.0_f64, 0.0_f64, 0.0_f64, 0.0_f64);
    for j in 0..=steps {
        for i in 0..=steps {
            let (lon, lat) = lon_lat_at(i as f64 / steps as f64, j as f64 / steps as f64);
            let (field, net) = field_and_net(heights, lon, lat);
            if net - field > best.0 {
                best = (net - field, lon, lat, field, net);
            }
        }
    }
    let (diff, lon, lat, field, net) = best;
    println!(
        "  [E3 fix] worst dip: {lon:.5} E {lat:.5} N — field {field:.1} m, drawn net \
         {net:.1} m, under-report {diff:.1} m"
    );
    (lon, lat, field, net)
}

/// **The acceptance.** A camera pushed onto its collision floor over a dip in the ground
/// must not end up below the surface the renderer is drawing.
///
/// Both arms are the same camera, the same pose and the same `enforce_bounds`; the only
/// difference is which of the two heights is handed to `set_ground_height`, which is
/// precisely what the fix changes. The drawn surface is taken from the built `TileMesh`
/// in ECEF, so neither arm is being compared against the function under test.
#[test]
fn the_old_floor_put_the_camera_under_the_drawn_ground() {
    use cesium_engine::camera::{Camera, CameraMode};
    use glam::Vec3;

    let mut heights = manager_with_the_fixture();
    let (lon, lat, field_m, net_m) = worst_dip_pose(&heights);
    let mesh = drawn_mesh(&mut heights);

    // Start well inside the ground, so that both arms are decided by their floor and not
    // by where they were put: 200 m below the ellipsoid under the valley.
    let start = cesium_engine::globe::geometry::lon_lat_alt_to_ecef_f64(lon, lat, -200.0);
    let eye = Vec3::new(start[0] as f32, start[1] as f32, start[2] as f32);

    // The query's own lon/lat for that eye — `ground_height_at` inverts the ECEF map, and
    // the inverse of a point at altitude is not the latitude it was built from. Using the
    // engine's own answer here is what makes this the engine's arithmetic.
    let eye64 = DVec3::new(eye.x as f64, eye.y as f64, eye.z as f64);
    let (q_lon, q_lat) = ecef_to_lon_lat_f64(eye64);
    let dir = eye64.normalize();
    let drawn = drawn_ray_hit(&mesh, q_lon, q_lat, DVec3::ZERO, dir);
    let t = ellipsoid_radius_along(dir);

    let arm = |ground_m: f64| -> f64 {
        let mut cam = Camera::new(eye, Vec3::ZERO);
        cam.set_eye(eye, Vec3::ZERO);
        cam.mode = CameraMode::Free;
        let ground_mm = ground_m / MM_TO_M;
        cam.enforce_bounds_with(&|_| Some(ground_mm));
        cam.global_transform_f64().0.length()
    };

    // The two heights, taken through the engine's own query at the engine's own lon/lat.
    let old_ground = heights.peek_height_at_lon_lat(q_lon, q_lat).unwrap() * MM_TO_M;
    let new_ground = heights
        .peek_mesh_height_at_lon_lat(q_lon, q_lat, TILE.z, SEGMENTS)
        .unwrap()
        * MM_TO_M;

    let old_dist = arm(old_ground);
    let new_dist = arm(new_ground);

    let old_under = (drawn - old_dist) * MM_TO_M;
    let new_under = (drawn - new_dist) * MM_TO_M;

    println!(
        "  [E3 fix] drawn surface {:.1} m above the ellipsoid; floor arms: field \
         {:.1} m, net {:.1} m",
        (drawn - t) * MM_TO_M,
        (old_dist - t) * MM_TO_M,
        (new_dist - t) * MM_TO_M
    );
    println!(
        "  [E3 fix] camera BELOW the drawn mesh: before {:+.1} m, after {:+.1} m",
        old_under, new_under
    );
    println!(
        "  [E3 fix] (the pose's own field/net disagreement was {:.1} m / {:.1} m)",
        field_m, net_m
    );

    // Before: genuinely under the visible ground, by roughly the disagreement less the
    // 2 m clearance the floor adds.
    assert!(
        old_under > 50.0,
        "the pre-fix floor was supposed to sit under the drawn mesh; it is {old_under:+.1} m"
    );
    // After: on it, never under it. The 2 m clearance is the only slack, and it is above.
    assert!(
        new_under <= 0.0,
        "the camera is still {new_under:.3} m below the drawn mesh"
    );
    assert!(
        -new_under < 3.0,
        "the camera is {:.3} m above the drawn mesh — the floor should be the surface \
         plus the 2 m clearance, not a stand-off",
        -new_under
    );
}

// ── 4. the feed ──────────────────────────────────────────────────────────────────

/// The level handed to the query is the level of the **deepest drawn tile** over the
/// point, not the height source's ceiling and not the coarsest thing that contains it.
///
/// The mixed-level case is the one that matters: the renderer draws a parent in place of
/// a child whose mesh has not arrived, so the drawn set is not a clean cut and a
/// shallowest-first walk would answer with the parent over ground its child is covering.
#[test]
fn the_drawn_level_is_the_deepest_tile_covering_the_point() {
    let child = TileId {
        z: 13,
        x: TILE.x * 2,
        y: TILE.y * 2,
    };
    let mut drawn = DrawnMeshes::default();
    drawn.replace([TILE, child].into_iter());

    // A point inside the child's quadrant — the north-west sixteenth of the parent.
    let (lon, lat) = lon_lat_at(0.1, 0.1);
    assert_eq!(drawn.level_at(lon, lat), Some(13));

    // A point in the parent's south-east quadrant, which no child in the set covers.
    let (lon, lat) = lon_lat_at(0.9, 0.9);
    assert_eq!(drawn.level_at(lon, lat), Some(12));

    // Somewhere else entirely: nothing is drawn, and the caller falls back to the field.
    assert_eq!(drawn.level_at(-73.98, 40.75), None);

    // And an empty set answers `None` without walking anything.
    assert_eq!(DrawnMeshes::default().level_at(lon, lat), None);
}
