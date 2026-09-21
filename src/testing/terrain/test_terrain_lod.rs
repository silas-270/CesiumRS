//! **E1 acceptance** — the LOD term that refines on *shape* rather than on picture
//! sharpness (`docs/terrain-plan.md` §8).
//!
//! Until E1 this engine refined a tile while `dist < unstretched_radius · lod_factor`,
//! i.e. on imagery resolution alone, and that was the right rule for the globe it had:
//! with zero relief (I-1) imagery resolution is the *only* per-tile error. With terrain
//! it is not. A flat coastal tile and a shattered massif of the same on-screen size got —
//! and, on the imagery term alone, still get — identical treatment.
//!
//! E1 adds the second half: `subdivide_dist = max(imagery_dist, terrain_dist)`, with
//! `terrain_dist` built from the tile's **measured** deviation from the surface its own
//! mesh draws ([`HeightTile::detail`]). This file is what says the two halves do what
//! they claim.
//!
//! # Why `culling` is not in this path
//!
//! The same reason `test_terrain_occlusion`'s module doc gives: `cargo test --release
//! --lib culling::` must keep reporting **32 passed, 0 failed, 1 ignored**, and libtest's
//! filter is a plain substring match on the full test path.
//!
//! # What is in the gate and what is not
//!
//! The four tests at the top run on synthetic fields and take no network: they are the
//! *claims* — a measured error rather than a level-based one, rugged ground refining
//! before flat ground at equal screen size, the term stopping at the data ceiling, and
//! the flat path not moving. The `#[ignore]`d measurements below them run against the
//! real DEM over `curl`, like `test_terrain_occlusion`'s, and are what the tables in
//! `docs/terrain-plan.md` §8 are made of.
//!
//! [`HeightTile::detail`]: cesium_engine::globe::terrain::HeightTile::detail

use cesium_engine::globe::quadtree::{
    lod_factor_for, terrain_lod_factor_for, tile_bounds, CullContext, CullPipeline, Frustum,
    QuadtreeManager, QuadtreeNode, TerrainFogPolicy, TileId,
};
use cesium_engine::globe::terrain::height_tile::{
    HeightTile, HEIGHT_DETAIL_STEP, HEIGHT_TILE_DIM, HEIGHT_TILE_TEXELS,
};
use cesium_engine::globe::terrain::{
    fallback_detail_mm, HeightBounds, HeightTileManager, Heightfield, DETAIL_MAX_Z,
    HEIGHT_DETAIL_PYRAMID_CELLS, OCCLUDER_GRID_CELLS,
};
use glam::DVec3;

use super::test_terrain_occlusion::{
    bounds_source, collect_sources, fetch_missing, fill_cache_real, real_config, real_poses,
    RealWorld, SEGMENTS, UPDATE_ITERATIONS,
};
use crate::testing::culling::cameras::{build_camera, ViewParams};

// ── the claims, on synthetic fields ──────────────────────────────────────────────

/// A height tile whose samples are `f(x, y)` in metres.
fn synthetic_tile(f: impl Fn(usize, usize) -> f64) -> HeightTile {
    let mut data = Box::new([0i16; HEIGHT_TILE_TEXELS]);
    for y in 0..HEIGHT_TILE_DIM {
        for x in 0..HEIGHT_TILE_DIM {
            data[y * HEIGHT_TILE_DIM + x] = f(x, y).round() as i16;
        }
    }
    HeightTile::from_samples(data)
}

/// **The error term is measured off the data, not read off the level.**
///
/// Four fields at the same level, which a level-based error would give the same number:
/// sea level, a plane, a field that bends only *between* the mesh's own samples, and one
/// that bends *on* them. The first two must read exactly zero — a mesh reproduces a
/// linear field exactly — and the third must read the sag it hides, while the fourth
/// reads near zero because the mesh's vertices sit on the feature and interpolate it.
///
/// That last pair is the whole content of the metric: what is being measured is not "how
/// tall is this ground" but "how much of this ground does a 17 × 17 mesh fail to say".
#[test]
fn the_geometric_error_is_measured_off_the_field_and_not_off_the_level() {
    assert_eq!(
        synthetic_tile(|_, _| 0.0).detail(),
        0,
        "sea level has no error"
    );

    // A plane. `detail_lattice`'s last interval is fifteen texels rather than sixteen, so
    // this also pins the one place an assumed-uniform spacing would have gone wrong.
    let plane = synthetic_tile(|x, y| 3.0 * x as f64 - 2.0 * y as f64);
    assert_eq!(
        plane.detail(),
        0,
        "a mesh reproduces a linear field exactly, so its error is zero"
    );

    // A triangular ridge whose crest sits exactly half way between two mesh samples, so
    // the mesh chords its peak away entirely. Amplitude 800 m over one step.
    let step = HEIGHT_DETAIL_STEP as f64;
    let hidden = synthetic_tile(|x, _| {
        let t = (x as f64 % step) / step;
        800.0 * (1.0 - (2.0 * t - 1.0).abs())
    });
    assert!(
        (hidden.detail() as f64 - 800.0).abs() <= 2.0,
        "a ridge between two mesh samples must measure its own height, got {}",
        hidden.detail()
    );

    // The same ridge, one lattice step wider, so its crests land *on* mesh samples. Same
    // relief, same level, same height range — and almost no geometric error, because the
    // mesh draws it.
    let drawn = synthetic_tile(|x, _| {
        let t = (x as f64 % (2.0 * step)) / (2.0 * step);
        800.0 * (1.0 - (2.0 * t - 1.0).abs())
    });
    assert!(
        drawn.detail() < 40,
        "relief the mesh's own vertices land on is not an error, got {}",
        drawn.detail()
    );
    assert!(
        drawn.detail() * 10 < hidden.detail(),
        "the two fields have the same range and must not have the same error: {} vs {}",
        drawn.detail(),
        hidden.detail()
    );
}

/// One node with a given measured error, at a given level.
fn probe_node(id: TileId, detail_m: f64) -> QuadtreeNode<Heightfield> {
    let bounds = HeightBounds {
        lo: -0.001,
        hi: 0.001,
        floor: -0.001,
        floor_grid: [-0.001f32; OCCLUDER_GRID_CELLS],
        detail: (detail_m * 1.0e-6) as f32,
    };
    QuadtreeNode::<Heightfield>::for_surface_with(id, bounds)
}

/// Does this node subdivide with the camera `dist` megametres straight above it?
///
/// The pipeline is empty, so every node is kept and the only thing deciding anything is
/// `apply_lod` — which is the function under test. `max_zoom` stops one level down so a
/// large terrain term cannot recurse to z19 while the test is trying to read one bit.
fn subdivides(
    node: &mut QuadtreeNode<Heightfield>,
    dist_mm: f64,
    lod_factor: f32,
    terrain_lod_factor: f32,
) -> bool {
    let eye = node.center + node.center.normalize() * dist_mm;
    let frustum = Frustum::planes_only([DVec3::ZERO; 4], eye);
    let ctx = CullContext::with_pipeline(&frustum, CullPipeline::of(&[]))
        .with_terrain_lod_factor(terrain_lod_factor)
        .with_max_zoom(node.id.z + 1);
    node.update(&ctx, lod_factor);
    node.children.is_some()
}

/// **The E1a claim, as one assertion**: at the same level, the same on-screen size and
/// the same camera distance, rugged ground refines and flat ground does not.
///
/// Two z11 tiles of near-identical geometry — neighbours, so their `unstretched_radius`
/// is the same number — at 100 km, which is nearly four times the distance the imagery
/// term has anything left to say at (`unstretched_radius · 2 ≈ 27.6 km` there). The tile
/// with 300 m of measured error subdivides; the one with 2 m does not. Nothing but
/// `HeightBounds::detail` differs between the two calls.
///
/// The two numbers are the real spread: a z11 tile is 19.6 km across, so its mesh lays a
/// sample every 1.2 km, and 300 m is what an Alpine massif hides between two of them
/// while a coastal plain hides single metres.
///
/// This is the tile-count half of the picture in `docs/terrain-plan.md` §8's capture,
/// which is the same statement with pixels instead of a boolean.
#[test]
fn rugged_ground_refines_before_flat_ground_at_the_same_screen_size() {
    let alpine = TileId {
        z: 11,
        x: 1088,
        y: 719,
    };
    let coastal = TileId {
        z: 11,
        x: 1089,
        y: 719,
    };

    let lod_factor = 2.0_f32;
    let terrain_lod_factor = terrain_lod_factor_for(2.0, 1080.0, 2.0f32 * (3.0f32 / 7.0).atan());

    let dist = 0.100;

    let mut rough = probe_node(alpine, 300.0);
    let mut flat = probe_node(coastal, 2.0);
    assert!(
        (rough.unstretched_radius - flat.unstretched_radius).abs() < 1.0e-6,
        "the two probes must be the same size on screen, or this measures the wrong thing"
    );

    assert!(
        !subdivides(&mut flat, dist, lod_factor, terrain_lod_factor),
        "2 m of relief does not earn a subdivision at four times the imagery threshold"
    );
    assert!(
        subdivides(&mut rough, dist, lod_factor, terrain_lod_factor),
        "300 m of measured error, on a tile of exactly the same screen size, must"
    );

    // …and with the geometric term switched off, the rugged tile behaves exactly like the
    // flat one. That is the control: the difference above is the term, not the fixture.
    let mut rough_no_term = probe_node(alpine, 300.0);
    assert!(
        !subdivides(&mut rough_no_term, dist, lod_factor, 0.0),
        "with `terrain_lod_factor = 0` the threshold must be the pre-E1 imagery one"
    );
}

/// **The term stops where the mesh becomes exact — z19, not z15.**
///
/// This test used to assert the opposite, and §9 F5 is why it changed. E1 read Cesium's
/// rule across: past the source's deepest level a node's mesh is an interpolation of its
/// ancestor and buys no shape. In Cesium that is true, because a heightmap tile's
/// `width × height` **is** its mesh lattice. Here the source is 256² and the mesh is 17²,
/// so a z15 tile already draws a **16:1 decimation of data it holds**, and its descendants
/// draw 8:1, 4:1, 2:1 and finally 1:1 at z19.
///
/// So the ladder below is the claim now: a real relief tile inserted once, and its own
/// `height_bounds_for` error read at every level from z14 to z20. It must fall — each level
/// resolves half the spacing — and it must reach **exactly** zero at z19 and stay there,
/// because at that depth the mesh samples every texel of its window.
#[test]
fn the_geometric_term_stops_where_the_mesh_becomes_exact() {
    assert_eq!(DETAIL_MAX_Z, 19, "the ceiling this test is about — §9 F5");

    let config = cesium_engine::globe::tiles::config::TileEngineConfig {
        terrain: cesium_engine::globe::tiles::config::TerrainConfig {
            enabled: true,
            ..Default::default()
        },
        ..Default::default()
    };
    let mut heights = HeightTileManager::new(&config);

    // A field that is rough at *every* scale, so no decimation of it is exact until the
    // lattice lands on every texel. A smooth one would reach zero early and prove nothing.
    let src = TileId {
        z: 15,
        x: 17_408,
        y: 11_504,
    };
    heights.insert_ready(
        src,
        std::sync::Arc::new(synthetic_tile(|x, y| {
            (((x * 37 + y * 53) % 17) as f64) * 40.0 + ((x % 3) as f64) * 111.0
        })),
    );

    // The corner descendant at each level, so the window is the one starting at texel 0.
    let mut id = src;
    let mut ladder = Vec::new();
    for _ in 0..=5 {
        let d = heights
            .height_bounds_for(id, SEGMENTS, 1.0, SHIPPED_DETAIL_MAX_Z)
            .map(|b| b.detail as f64 * 1.0e6)
            .unwrap_or(-1.0);
        ladder.push((id.z, d));
        id = TileId {
            z: id.z + 1,
            x: id.x * 2,
            y: id.y * 2,
        };
    }

    for w in ladder.windows(2) {
        let ((z0, d0), (z1, d1)) = (w[0], w[1]);
        assert!(
            d1 <= d0,
            "z{z1} resolves half z{z0}'s spacing and cannot have more error: {d0} -> {d1}"
        );
    }
    for (z, d) in &ladder {
        if *z <= 18 {
            assert!(
                *d > 0.0,
                "z{z} still decimates its source {}:1 and has shape left to resolve",
                16 >> (z - 15)
            );
        } else {
            assert_eq!(
                *d, 0.0,
                "z{z} samples every texel of its window, so its mesh *is* the data"
            );
        }
    }

    // And the ceiling is a knob: set it to E1's 15 and every level from z15 down reads
    // zero again, which is the behaviour this test used to assert.
    for (z, _) in &ladder {
        let id = {
            let mut i = src;
            for _ in 0..(z - 15) {
                i = TileId {
                    z: i.z + 1,
                    x: i.x * 2,
                    y: i.y * 2,
                };
            }
            i
        };
        assert_eq!(
            heights
                .height_bounds_for(id, SEGMENTS, 1.0, 15)
                .map(|b| b.detail)
                .unwrap_or(-1.0),
            0.0,
            "at `detail_max_z = 15` the term is off from z15 down"
        );
    }
}

/// **The flat path does not have a geometric term at all**, and this is the pin.
///
/// `Ellipsoid::HAS_GEOMETRIC_ERROR` is a compile-time `false`, so `apply_lod`
/// monomorphises to the imagery-only threshold on the flat arm and the field below is
/// never read. Setting it to an absurd value and comparing the whole visible set is the
/// mechanical statement of that: byte-identical tile ids, in order.
///
/// The stronger statement — the 204-pose LOD harness producing byte-identical CSVs — is
/// in `docs/culling-baseline.md`; this is the cheap version that lives in the gate.
#[test]
fn the_flat_globe_has_no_geometric_term_to_set() {
    let p = ViewParams {
        sweep: "e1_flat_pin",
        lat_deg: 47.26,
        lon_deg: 11.40,
        alt_m: 9_000.0,
        pitch_deg: 78.0,
        yaw_deg: 0.0,
        roll_deg: 0.0,
        width: 1280,
        height: 720,
        mode: cesium_engine::camera::camera::CameraMode::Free,
    };
    let cam = build_camera(&p);
    let aspect = p.aspect() as f32;
    let (eye, _) = cam.global_transform_f64();
    let frustum = Frustum::planes_only(cam.calculate_frustum_planes(aspect), eye)
        .with_corners(cam.frustum_corners_relative(aspect));

    let settle = |terrain_lod_factor: f32| {
        let mut qt = QuadtreeManager::new();
        qt.lod_factor = lod_factor_for(1.0, 512.0, p.height as f32, cam.fovy());
        qt.terrain_lod_factor = terrain_lod_factor;
        for _ in 0..UPDATE_ITERATIONS {
            qt.update(&frustum);
        }
        qt.get_visible_tiles()
            .into_iter()
            .map(|(id, _, _)| id)
            .collect::<Vec<_>>()
    };

    let baseline = settle(0.0);
    assert!(!baseline.is_empty(), "the pose must see something");
    for absurd in [1.0e3_f32, 1.0e9] {
        assert_eq!(
            settle(absurd),
            baseline,
            "a flat globe has no geometric error, so no value of terrain_lod_factor may move it"
        );
    }
}

// ── the measurements, against the real DEM ───────────────────────────────────────

// The projected geometric error metric itself lives in the LOD harness
// (`testing::lod::sweep::geometric_error_px`) and is imported, not re-derived: it is *the
// same* second metric `src/testing/lod`'s module doc describes, evaluated here on a
// surface that has one. Two copies of that formula would be two chances for the terrain
// numbers and the flat ones to stop being comparable.
use crate::testing::lod::sweep::geometric_error_px;

/// `TerrainConfig::detail_max_z` as it ships — **19 since §9 F5**, where E1 had 15.
const SHIPPED_DETAIL_MAX_Z: u8 = cesium_engine::globe::terrain::DETAIL_MAX_Z;

/// What one settled tree costs and how wrong its surface is.
struct LodResult {
    tiles: usize,
    texture_bytes: u64,
    deepest_zoom: u8,
    /// p95 of [`geometric_error_px`] over the visible tiles — the column
    /// `max_geometric_error_px` is a budget for.
    p95_error_px: f64,
    /// The same, and the deepest level, restricted to tiles more than [`FAR_FIELD_MM`]
    /// away. **This is the column E1b is actually about**: fog is negligible in the near
    /// field by construction, so a policy's whole effect lives out here, and §7c's
    /// complaint — "at 900 m the far field never refines past z10/z11" — is a statement
    /// about exactly this pair of numbers.
    far_p95_error_px: f64,
    far_deepest_zoom: u8,
    far_tiles: usize,
}

/// Where "the far field" starts, megametres — 20 km.
///
/// Not a tuned number: it is past the near field at every pose in `real_poses` (the
/// highest is 400 km up and its whole visible set is far), and it is where
/// `cesium_fog` at a 900 m camera first passes 0.35, i.e. where the policies begin to
/// disagree at all.
const FAR_FIELD_MM: f64 = 0.020;

/// A settled terrain quadtree over the real DEM at `p`, with E1's geometric term at
/// `max_geometric_error_px` (`0.0` = off, i.e. the pre-E1 engine).
///
/// Deliberately the same shape as `test_terrain_occlusion::settled_real_tree`, including
/// its two non-obvious knobs: the capture's own `lod_factor` (at the 256 px satellite
/// style) and — the one that decides how coarse the far field is —
/// `fog_density_for(alt)`. §7c is the section that found out what leaving the second at
/// zero costs.
fn settled_real_tree(
    p: &ViewParams,
    frustum: &Frustum,
    max_geometric_error_px: f32,
    policy: TerrainFogPolicy,
    world: &mut RealWorld,
) -> (QuadtreeManager<Heightfield>, HeightTileManager) {
    let config = real_config();
    let mut heights = HeightTileManager::new(&config);
    let mut qt = QuadtreeManager::<Heightfield>::for_surface();
    let cam = build_camera(p);
    qt.lod_factor = lod_factor_for(1.0, 256.0, p.height as f32, cam.fovy());
    qt.max_zoom = config.max_zoom;
    qt.fog_density = cesium_engine::globe::quadtree::fog_density_for(p.alt_m as f32, &config.fog);
    qt.terrain_lod_factor =
        terrain_lod_factor_for(max_geometric_error_px, p.height as f32, cam.fovy());
    qt.terrain_fog_policy = policy;
    qt.terrain_fog_sse_ratio = if max_geometric_error_px > 0.0 {
        config.fog.sse / max_geometric_error_px
    } else {
        0.0
    };
    qt.pipeline = CullPipeline::TERRAIN_DEFAULT;
    let cam_alt = p.alt_m * 1.0e-6;

    for _ in 0..UPDATE_ITERATIONS {
        let mut wanted = Vec::new();
        for root in qt.roots.iter() {
            collect_sources(root, &heights, &mut wanted);
        }
        wanted.sort_unstable_by_key(|id| (id.z, id.x, id.y));
        wanted.dedup();
        fetch_missing(&world.dir, &wanted);
        for root in qt.roots.iter() {
            fill_cache_real(root, &mut heights, world);
        }
        qt.refresh_extras(&bounds_source(&heights));
        qt.refresh_terrain_horizon(frustum, cam_alt, cam_alt, &config.terrain.occlusion);
        qt.update(frustum);
    }
    let mut wanted = Vec::new();
    for root in qt.roots.iter() {
        collect_sources(root, &heights, &mut wanted);
    }
    wanted.sort_unstable_by_key(|id| (id.z, id.x, id.y));
    wanted.dedup();
    fetch_missing(&world.dir, &wanted);
    for root in qt.roots.iter() {
        fill_cache_real(root, &mut heights, world);
    }
    qt.refresh_extras(&bounds_source(&heights));
    (qt, heights)
}

/// Scores a settled tree: what it costs, and how many pixels of shape it is still wrong
/// by.
fn score(
    qt: &QuadtreeManager<Heightfield>,
    heights: &HeightTileManager,
    p: &ViewParams,
    frustum: &Frustum,
) -> LodResult {
    let cam = build_camera(p);
    let fovy = cam.fovy() as f64;
    let visible = qt.get_visible_tiles();
    // The satellite style the capture poses are measured at — 256², 4 bytes a texel, the
    // same accounting `src/testing/lod`'s harness uses.
    let texture_bytes = visible.len() as u64 * 256 * 256 * 4;
    let mut deepest_zoom = 0u8;
    let mut errors: Vec<f64> = Vec::with_capacity(visible.len());
    let mut far_errors: Vec<f64> = Vec::new();
    let mut far_deepest_zoom = 0u8;
    for (id, _, _) in &visible {
        deepest_zoom = deepest_zoom.max(id.z);
        let b = tile_bounds(id);
        let centre = {
            let lon = 0.5 * (b.lon_min + b.lon_max);
            let lat = 0.5 * (b.lat_min + b.lat_max);
            let q = cesium_engine::globe::geometry::lon_lat_to_ecef_f64(lon, lat);
            DVec3::new(q[0], q[1], q[2])
        };
        let dist = (centre - frustum.eye).length();
        // The error the *drawn* tile still carries. Past the source ceiling the mesh is an
        // interpolation of its z15 ancestor, so the error that is actually on screen is
        // that ancestor's own, scaled by how much of it this tile covers — which is what
        // reading the source tile's `detail` over this tile's rectangle amounts to.
        let error_mm = heights
            .height_bounds_for(*id, SEGMENTS, 1.0, SHIPPED_DETAIL_MAX_Z)
            .map(|b| b.detail as f64)
            .unwrap_or_else(|| fallback_detail_mm(id.z));
        let err_px = geometric_error_px(error_mm, dist, p.height as f64, fovy);
        errors.push(err_px);
        if dist > FAR_FIELD_MM {
            far_errors.push(err_px);
            far_deepest_zoom = far_deepest_zoom.max(id.z);
        }
    }
    let p95_of = |v: &mut Vec<f64>| -> f64 {
        v.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        if v.is_empty() {
            0.0
        } else {
            v[((v.len() as f64 * 0.95) as usize).min(v.len() - 1)]
        }
    };
    let far_tiles = far_errors.len();
    LodResult {
        tiles: visible.len(),
        texture_bytes,
        deepest_zoom,
        p95_error_px: p95_of(&mut errors),
        far_p95_error_px: p95_of(&mut far_errors),
        far_deepest_zoom,
        far_tiles,
    }
}

/// **The §8 calibration table**: what the measured error actually is, level by level and
/// region by region, against the level-based formula that stands in for it.
///
/// The fallback ([`fallback_detail_mm`]) is Cesium's heightmap rule — `2πa / (65 · 2^z)`,
/// content-blind — and the point of this table is to say how far off it is from the
/// thing it approximates, in both directions, before anyone relies on it.
#[test]
#[ignore = "measurement, needs the network for real height tiles"]
fn e1_measured_error_against_the_level_based_fallback() {
    let mut world = RealWorld::new();
    // One column of tiles per region, z6 → z13, each one containing the region's centre.
    let regions: [(&str, f64, f64); 5] = [
        ("alps_inn_valley", 11.40, 47.26),
        ("himalaya_everest", 86.925, 27.99),
        ("po_plain", 11.30, 45.15),
        ("bay_of_bengal", 88.50, 18.00),
        ("amazon_basin", -60.00, -3.00),
    ];
    println!("  [E1 error] measured geometric error (m) against the level-based fallback");
    print!("    {:<8}", "level");
    for (name, _, _) in regions {
        print!(" {name:>18}");
    }
    println!(" {:>12}", "fallback");
    for z in 6..=13u8 {
        print!("    z{z:<7}");
        for (_, lon, lat) in regions {
            let (id, _, _) = HeightTileManager::tile_uv_at_lon_lat(lon, lat, z);
            let tile = world.tile(id);
            print!(" {:>18}", tile.detail());
        }
        println!(" {:>12.0}", fallback_detail_mm(z) * 1.0e6);
    }
}

/// **The E1a cost table**: visible tiles, texture bytes and the projected geometric error
/// they leave on screen, against `TerrainConfig::max_geometric_error_px`.
///
/// `off` is the pre-E1 engine — imagery alone — and every other column is the same ten
/// poses with the geometric term switched on at that budget. Lower budget, sharper
/// surface, more tiles; the table is what picks the default.
#[test]
#[ignore = "measurement, needs the network for real height tiles"]
fn e1_cost_of_the_geometric_term_on_real_terrain() {
    let mut world = RealWorld::new();
    const KNOBS: [f32; 6] = [0.0, 24.0, 16.0, 12.0, 10.0, 8.0];

    println!("  [E1 cost] visible tiles / MiB of imagery / p95 geometric error (px)");
    print!("    {:<22}", "pose");
    for k in KNOBS {
        if k == 0.0 {
            print!(" {:>22}", "off (pre-E1)");
        } else {
            print!(" {:>22}", format!("{k} px"));
        }
    }
    println!();

    let mut totals = [0usize; KNOBS.len()];
    for (name, p) in real_poses() {
        let cam = build_camera(&p);
        let aspect = p.aspect() as f32;
        let (eye, _) = cam.global_transform_f64();
        let frustum = Frustum::planes_only(cam.calculate_frustum_planes(aspect), eye)
            .with_corners(cam.frustum_corners_relative(aspect));
        print!("    {name:<22}");
        for (i, k) in KNOBS.iter().enumerate() {
            let (qt, heights) =
                settled_real_tree(&p, &frustum, *k, TerrainFogPolicy::default(), &mut world);
            let r = score(&qt, &heights, &p, &frustum);
            totals[i] += r.tiles;
            print!(
                " {:>6} {:>6.1} {:>7.1} z{:<2}",
                r.tiles,
                r.texture_bytes as f64 / (1024.0 * 1024.0),
                r.p95_error_px,
                r.deepest_zoom
            );
        }
        println!();
    }
    print!("    {:<22}", "TOTAL tiles");
    for t in totals {
        print!(" {t:>22}");
    }
    println!();
    println!(
        "    (the error column is an upper bound past z15: a node deeper than the source \
         ceiling is scored with its z15 ancestor's error, which its own finer mesh can \
         only beat — see `score`)"
    );
}

/// The residency cost of the error term, stated rather than assumed: E1's two bytes per
/// resident height tile, and F5's 168 on top of them, and nothing else.
#[test]
fn the_error_term_costs_two_bytes_a_tile_and_f5_adds_a_hundred_and_sixty_eight() {
    use cesium_engine::globe::tiles::config::HEIGHT_TILE_BYTES;
    assert_eq!(
        HEIGHT_TILE_BYTES,
        256 * 256 * 2 + 2 * (16 * 16 * 2) + 2 + 2 * HEIGHT_DETAIL_PYRAMID_CELLS,
        "E1 adds one i16 per tile and F5 adds 84 more; if this moved, the cache entry \
         count in `docs/terrain-plan.md` §5 B4 needs re-deriving"
    );
    // The derived entry count must follow from the one constant that states the budget and
    // nothing else. It is 381 on desktop since §9 F2b resized the slice to 48 MiB; Android
    // keeps 253 until the soak of §9 F3 is run.
    let config = cesium_engine::globe::tiles::config::TileEngineConfig::default();
    assert_eq!(
        config.terrain.height_cache_budget_bytes / HEIGHT_TILE_BYTES,
        cesium_engine::globe::tiles::config::HEIGHT_CACHE_BUDGET_BYTES / HEIGHT_TILE_BYTES,
        "and the derived entry count must not have moved with it"
    );
    #[cfg(not(target_os = "android"))]
    assert_eq!(
        config.terrain.height_cache_budget_bytes / HEIGHT_TILE_BYTES,
        380
    );
    // Not a size_of pin: `HeightTile` boxes its grids, so the i16 lands in a struct whose
    // own size is dominated by three pointers. The accounting constant above is what the
    // cache budget actually divides by.
    assert_eq!(HeightTile::flat_zero().detail(), 0);
    // …and a flat field has nothing to resolve at any depth either.
    for k in 0..=4u32 {
        assert_eq!(HeightTile::flat_zero().detail_below(k, 0, 0), 0);
    }
}

/// **E1b** (`docs/terrain-plan.md` §8) — what fog is allowed to do to the *shape* budget.
///
/// `apply_lod` multiplies `subdivide_dist` by `1 − fog(d)`, and §7c measured what that
/// does: at 900 m, where fog is thickest, it stops the far field refining past z10/z11 in
/// the first place — the same pose reads 100 tiles and −36 % D3 reduction with fog off,
/// and 50 tiles and −4 % with it on. That was tuned for a globe with **no relief**, where
/// far-field coarsening is free because there is nothing out there but texture. With
/// terrain it is not free: it is distant mountains staying coarse bumps.
///
/// Three policies, same poses, same budget ([`TerrainFogPolicy`]):
///
/// * `Relax` — WP5's, extended to the geometric term. What E1a shipped.
/// * `ImageryOnly` — fog relaxes the picture and leaves the shape alone.
/// * `CesiumSse` — Cesium's own: fog widens the *pixel* budget by `fog · sse`, which
///   finally gives `FogConfig::sse` units in this engine.
///
/// The column to read them against is not tiles alone but tiles **and** p95 projected
/// geometric error: a policy that buys 100 tiles and removes no error is a worse deal
/// than one that buys 20 and removes half of it.
#[test]
#[ignore = "measurement, needs the network for real height tiles"]
fn e1b_what_fog_may_do_to_the_geometric_term() {
    let mut world = RealWorld::new();
    // The budget the cost table settles on. Measuring the fog policy at a budget nobody
    // ships would measure the budget instead.
    const BUDGET_PX: f32 = 8.0;
    const POLICIES: [(&str, TerrainFogPolicy); 3] = [
        ("Relax (WP5)", TerrainFogPolicy::Relax),
        ("ImageryOnly", TerrainFogPolicy::ImageryOnly),
        ("CesiumSse", TerrainFogPolicy::CesiumSse),
    ];

    println!("  [E1b] fog policy for the geometric term, at max_geometric_error_px = {BUDGET_PX}");
    println!("        tiles / MiB / p95 geometric error (px) / deepest zoom");
    print!("    {:<22} {:>22}", "pose", "no geometric term");
    for (name, _) in POLICIES {
        print!(" {name:>22}");
    }
    println!();

    let mut totals = [0usize; 4];
    for (name, p) in real_poses() {
        let cam = build_camera(&p);
        let aspect = p.aspect() as f32;
        let (eye, _) = cam.global_transform_f64();
        let frustum = Frustum::planes_only(cam.calculate_frustum_planes(aspect), eye)
            .with_corners(cam.frustum_corners_relative(aspect));
        print!("    {name:<22}");
        let (qt, heights) =
            settled_real_tree(&p, &frustum, 0.0, TerrainFogPolicy::Relax, &mut world);
        let r = score(&qt, &heights, &p, &frustum);
        totals[0] += r.tiles;
        print!(
            " {:>6} {:>6.1} {:>7.1} z{:<2}",
            r.tiles,
            r.texture_bytes as f64 / (1024.0 * 1024.0),
            r.p95_error_px,
            r.deepest_zoom
        );
        for (i, (_, policy)) in POLICIES.iter().enumerate() {
            let (qt, heights) = settled_real_tree(&p, &frustum, BUDGET_PX, *policy, &mut world);
            let r = score(&qt, &heights, &p, &frustum);
            totals[i + 1] += r.tiles;
            print!(
                " {:>6} {:>6.1} {:>7.1} z{:<2}",
                r.tiles,
                r.texture_bytes as f64 / (1024.0 * 1024.0),
                r.p95_error_px,
                r.deepest_zoom
            );
        }
        println!();
    }
    print!("    {:<22}", "TOTAL tiles");
    for t in totals {
        print!(" {t:>22}");
    }
    println!();
}

/// The three policies' relaxation factors side by side, as numbers rather than as
/// formulas — the structural half of E1b's answer, and it needs no network.
///
/// `Relax` goes to **zero**: past the distance where fog saturates, the geometric term is
/// switched off completely and the ground out there refines on imagery alone. `CesiumSse`
/// is bounded below by `1/(1 + sse/max_px)` however thick the fog gets — at this engine's
/// shipped budget that is a floor of `1/(1 + 2/8) = 0.8`. They are not two settings of the
/// same knob; they are different claims about what fog conceals.
#[test]
fn the_three_fog_policies_are_not_settings_of_one_knob() {
    let sse_ratio = 2.0f32 / 8.0;
    // `fog` at the four distances a 900 m camera sees: clear, half-obscured, saturated.
    for fog in [0.0f32, 0.35, 0.93, 1.0] {
        let relax = 1.0 - fog;
        let cesium = 1.0 / (1.0 + fog * sse_ratio);
        assert!(
            cesium >= 1.0 / (1.0 + sse_ratio) - 1.0e-6,
            "Cesium's form is bounded below by 1/(1+sse/max_px), got {cesium} at fog={fog}"
        );
        if fog >= 1.0 {
            assert_eq!(relax, 0.0, "WP5's form switches the term off entirely");
            assert!(
                cesium > 0.79,
                "…where Cesium's has only shortened it by a fifth: {cesium}"
            );
        }
    }
}

/// **E1b, at an equal tile budget** — the comparison that actually decides it.
///
/// Reading `Relax` at 897 tiles against `ImageryOnly` at 1 210 says only that refining
/// more refines more. WP4/C settled this repo's methodology for exactly this situation
/// (`docs/pre-terrain-plan.md`: "equal tile budget, not equal `target_texel_ratio`"), so
/// the honest question is: **given the same number of tiles, which policy spends them on
/// less error?**
///
/// The budget is bisected per policy so all three land on the same total tile count, and
/// the column that decides is the p95 projected geometric error summed over the poses.
#[test]
#[ignore = "measurement, needs the network for real height tiles"]
fn e1b_the_three_policies_at_an_equal_tile_budget() {
    let mut world = RealWorld::new();
    const POLICIES: [(&str, TerrainFogPolicy); 3] = [
        ("Relax (WP5)", TerrainFogPolicy::Relax),
        ("ImageryOnly", TerrainFogPolicy::ImageryOnly),
        ("CesiumSse", TerrainFogPolicy::CesiumSse),
    ];

    let poses: Vec<(&'static str, ViewParams, Frustum)> = real_poses()
        .into_iter()
        .map(|(name, p)| {
            let cam = build_camera(&p);
            let aspect = p.aspect() as f32;
            let (eye, _) = cam.global_transform_f64();
            let frustum = Frustum::planes_only(cam.calculate_frustum_planes(aspect), eye)
                .with_corners(cam.frustum_corners_relative(aspect));
            (name, p, frustum)
        })
        .collect();

    // `Relax` at the shipped budget sets the budget every policy has to hit.
    let mut target = 0usize;
    for (_, p, frustum) in &poses {
        let (qt, _) = settled_real_tree(p, frustum, 8.0, TerrainFogPolicy::Relax, &mut world);
        target += qt.get_visible_tiles().len();
    }
    println!("  [E1b equal budget] target = {target} tiles (Relax at 8 px)");

    println!(
        "    {:<22} {:>10} {:>8} {:>11} {:>11} {:>10} {:>12}",
        "policy", "budget px", "tiles", "sum p95 px", "far p95 sum", "far tiles", "far deepest z"
    );
    for (name, policy) in POLICIES {
        // Tile count falls monotonically as the budget rises, so bisect on the budget.
        let (mut lo, mut hi) = (1.0f32, 128.0f32);
        let count_at = |budget: f32, world: &mut RealWorld| -> usize {
            poses
                .iter()
                .map(|(_, p, f)| settled_real_tree(p, f, budget, policy, world).0)
                .map(|qt| qt.get_visible_tiles().len())
                .sum()
        };
        for _ in 0..12 {
            let mid = 0.5 * (lo + hi);
            if count_at(mid, &mut world) > target {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        let budget = hi;
        let mut tiles = 0usize;
        let mut sum_p95 = 0.0f64;
        let mut far_sum_p95 = 0.0f64;
        let mut far_tiles = 0usize;
        let mut far_deepest = String::new();
        for (_, p, frustum) in &poses {
            let (qt, heights) = settled_real_tree(p, frustum, budget, policy, &mut world);
            let r = score(&qt, &heights, p, frustum);
            tiles += r.tiles;
            sum_p95 += r.p95_error_px;
            far_sum_p95 += r.far_p95_error_px;
            far_tiles += r.far_tiles;
            far_deepest.push_str(&format!("{} ", r.far_deepest_zoom));
        }
        println!(
            "    {name:<22} {budget:>10.2} {tiles:>8} {sum_p95:>11.1} {far_sum_p95:>11.1} \
             {far_tiles:>10} {far_deepest:>12}"
        );
    }
}

/// **Where the extra tiles land** — the numeric twin of
/// `rendering::terrain_e1_capture`'s middle shot, and the sharpest statement of E1a that
/// does not need a GPU.
///
/// One pose, `po_plain_to_alps`: 300 m over the Po plain looking north, with 60 km of dead
/// flat alluvium in the foreground and the Alps standing on the horizon behind it. Both
/// ground types, one camera, one frame — so no argument about matched poses is needed.
/// Every visible tile is bucketed by its **own** measured error: "flat" under 10 m, "rugged"
/// over 100 m, and the middle left unlabelled.
///
/// The geometric term is then switched on and the buckets are compared. If E1a does what it
/// says, essentially every tile it adds is in the rugged bucket.
#[test]
#[ignore = "measurement, needs the network for real height tiles"]
fn e1_the_extra_tiles_land_on_the_mountains() {
    let mut world = RealWorld::new();
    let (_, p) = real_poses()
        .into_iter()
        .find(|(n, _)| *n == "po_plain_to_alps")
        .expect("the plain-and-mountains pose");
    let cam = build_camera(&p);
    let aspect = p.aspect() as f32;
    let (eye, _) = cam.global_transform_f64();
    let frustum = Frustum::planes_only(cam.calculate_frustum_planes(aspect), eye)
        .with_corners(cam.frustum_corners_relative(aspect));

    println!("  [E1 split] po_plain_to_alps, tiles by their own measured error");
    println!(
        "    {:<16} {:>8} {:>14} {:>14} {:>14}",
        "budget", "tiles", "flat (<10 m)", "mid", "rugged (>100 m)"
    );
    let mut buckets = Vec::new();
    for budget in [0.0f32, 12.0] {
        let (qt, heights) = settled_real_tree(
            &p,
            &frustum,
            budget,
            TerrainFogPolicy::default(),
            &mut world,
        );
        let (mut flat, mut mid, mut rugged) = (0usize, 0usize, 0usize);
        let visible = qt.get_visible_tiles();
        for (id, _, _) in &visible {
            let detail_m = heights
                .height_bounds_for(*id, SEGMENTS, 1.0, SHIPPED_DETAIL_MAX_Z)
                .map(|b| b.detail as f64 * 1.0e6)
                .unwrap_or_else(|| fallback_detail_mm(id.z) * 1.0e6);
            if detail_m < 10.0 {
                flat += 1;
            } else if detail_m > 100.0 {
                rugged += 1;
            } else {
                mid += 1;
            }
        }
        let label = if budget == 0.0 {
            "off (pre-E1)".to_string()
        } else {
            format!("{budget} px")
        };
        println!(
            "    {label:<16} {:>8} {flat:>14} {mid:>14} {rugged:>14}",
            visible.len()
        );
        buckets.push((visible.len(), flat, mid, rugged));
    }

    let (n0, flat0, mid0, rugged0) = buckets[0];
    let (n1, flat1, mid1, rugged1) = buckets[1];
    println!(
        "    {:<16} {:>8} {:>14} {:>14} {:>14}",
        "delta",
        n1 as i64 - n0 as i64,
        flat1 as i64 - flat0 as i64,
        mid1 as i64 - mid0 as i64,
        rugged1 as i64 - rugged0 as i64
    );
    assert!(
        rugged1 > rugged0,
        "the geometric term must add tiles over rugged ground: {rugged0} -> {rugged1}"
    );
    assert!(
        (rugged1 - rugged0) > (flat1.saturating_sub(flat0)),
        "…and more of them there than over the plain: rugged +{}, flat +{}",
        rugged1 - rugged0,
        flat1.saturating_sub(flat0)
    );
}
