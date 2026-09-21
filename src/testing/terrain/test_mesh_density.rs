//! **F1 and F2 acceptance** — choosing `mesh_segments`, and saying where terrain's bytes
//! actually go (`docs/terrain-plan.md` §9).
//!
//! Phase C deliberately deferred the density decision: C4 measured max and RMS deviation
//! at 16/32/64 against all 65 536 source samples of the committed fixtures and then said
//! *"the default stays 16 and Phase F picks against device measurements"*. There are no
//! device measurements — `adb` is not installed on the machine this was written on and no
//! phone is attached — so F1 decides on the data that does exist, and this file is that
//! data.
//!
//! # The three things measured here
//!
//! 1. **[`deviation_at_step`]** — [`HeightTile::detail`], generalised to an arbitrary
//!    decimation. The engine's own error term is hard-wired to a 16:1 decimation
//!    ([`HEIGHT_DETAIL_STEP`]) because that is *exactly* the mesh lattice at
//!    `mesh_segments = 16`. At 32 or 64 the identity breaks, and the first test below is
//!    what turns that doc-comment caveat into a number.
//! 2. **Tiles and bytes at the real poses**, at all three densities, at the shipped
//!    `max_geometric_error_px = 12`. This is the F1 table, read off its marginal column
//!    the way E1a read off its own.
//! 3. **The memory split** — imagery bytes against height bytes against vertex bytes, at
//!    the real poses and at the *shipped* budget rather than the measuring harness's
//!    inflated one, which is the only way to see whether B4's declared slice holds.
//!
//! # Why `culling` is not in this path
//!
//! The reason its two siblings give, restated because it is the mistake that has been
//! made twice: `cargo test --release --lib culling::` must keep reporting **32 passed, 0
//! failed, 1 ignored**, and libtest's filter is a plain substring match on the full test
//! path. Nothing under `testing::terrain::` may contain the substring `culling`.
//!
//! [`HeightTile::detail`]: cesium_engine::globe::terrain::HeightTile::detail

use std::collections::HashMap;

use cesium_engine::globe::geometry::{TileMesh, Vertex};
use cesium_engine::globe::quadtree::{
    lod_factor_for, terrain_lod_factor_for, tile_bounds, CullPipeline, Frustum, QuadtreeManager,
    TerrainFogPolicy, TileId,
};
use cesium_engine::globe::terrain::height_tile::{
    HeightTile, HEIGHT_DETAIL_STEP, HEIGHT_TILE_DIM, HEIGHT_TILE_TEXELS,
};
use cesium_engine::globe::terrain::{
    fallback_detail_mm, HeightBoundsSource, HeightTileManager, Heightfield,
};
use cesium_engine::globe::tiles::config::{
    OceanPolicy, TerrainConfig, TileEngineConfig, HEIGHT_TILE_BYTES,
};
use glam::DVec3;

use super::test_terrain_occlusion::{
    collect_sources, fetch_missing, fill_cache_real, real_config, real_poses, RealWorld,
    UPDATE_ITERATIONS,
};
use crate::testing::culling::cameras::{build_camera, ViewParams};
use crate::testing::lod::sweep::geometric_error_px;

/// The three densities C4 measured and F1 decides between.
const DENSITIES: [u32; 3] = [16, 32, 64];

/// Mesh cache entries — `TileEngineConfig::mesh_cache_size`'s shipped value, and the
/// multiplier that turns "bytes per tile" into "bytes on the card".
const MESH_CACHE_ENTRIES: usize = 512;

// ── the error term, generalised ──────────────────────────────────────────────────

/// [`HeightTile::detail`] with the decimation spelled out instead of fixed at 16:1.
///
/// The engine's `measure_detail` is this function at `step = HEIGHT_DETAIL_STEP`, and
/// [`the_error_term_measures_a_sixteen_to_one_decimation_whatever_the_mesh_draws`] is what
/// holds the two together — a re-implementation that had drifted would make every number
/// below incomparable with the engine's own.
///
/// `step = 1` returns zero by construction: a lattice that lands on every texel
/// reproduces the field exactly. That is not a special case bolted on, it is what the
/// general formula gives, and it is the right answer for a tile drawn at a density finer
/// than its own data.
pub(crate) fn deviation_at_step(tile: &HeightTile, step: usize) -> f64 {
    deviation_over(tile, step, [0, HEIGHT_TILE_DIM, 0, HEIGHT_TILE_DIM])
}

/// [`deviation_at_step`] restricted to one texel window `[x0, x1) × [y0, y1)`.
///
/// The window is how a tile **below** the source ceiling is scored. Such a tile draws a
/// `segments + 1` lattice across its own share of an ancestor's texels, and that share is
/// a corner-aligned sub-rectangle whose width is a whole multiple of the step — so the
/// mesh's lattice lines *are* the global lattice lines at that step, and only the set of
/// texels being scored changes. Scoring the whole ancestor instead would charge a z17 tile
/// on the far side of the valley for the icefall in the opposite corner, which is the same
/// over-statement `test_terrain_lod::score`'s footnote in §8 names, and there is no reason
/// to repeat it here when the window is known exactly.
pub(crate) fn deviation_over(tile: &HeightTile, step: usize, [x0, x1, y0, y1]: [usize; 4]) -> f64 {
    let step = step.max(1);
    if step == 1 {
        return 0.0;
    }
    let last = HEIGHT_TILE_DIM - 1;
    // The engine's `detail_lattice`, with `step` in place of the constant: lines at
    // `0, step, 2·step, …`, the last one pulled onto the final texel rather than off the
    // end, and each interval interpolated with its **own** width so a linear field
    // measures exactly zero.
    let lattice: Vec<(usize, usize, f64)> = (0..HEIGHT_TILE_DIM)
        .map(|x| {
            let lo = (x / step) * step;
            let hi = (lo + step).min(last);
            let span = hi - lo;
            let w = if span == 0 {
                0.0
            } else {
                (x - lo) as f64 / span as f64
            };
            (lo, hi, w)
        })
        .collect();

    let mut worst = 0.0f64;
    for y in y0.min(last)..y1.min(HEIGHT_TILE_DIM) {
        let (ly0, ly1, wy) = lattice[y];
        for x in x0.min(last)..x1.min(HEIGHT_TILE_DIM) {
            let (lx0, lx1, wx) = lattice[x];
            let h00 = tile.sample(lx0, ly0) as f64;
            let h10 = tile.sample(lx1, ly0) as f64;
            let h01 = tile.sample(lx0, ly1) as f64;
            let h11 = tile.sample(lx1, ly1) as f64;
            let top = h00 + (h10 - h00) * wx;
            let bottom = h01 + (h11 - h01) * wx;
            let interpolated = top + (bottom - top) * wy;
            let dev = (tile.sample(x, y) as f64 - interpolated).abs();
            if dev > worst {
                worst = dev;
            }
        }
    }
    // `ceil`, like the engine's, so a 0.4 m deviation is not reported as flat.
    worst.ceil().min(i16::MAX as f64)
}

/// The texel spacing of the mesh drawn over `id` at `segments`, measured on the texel
/// grid of the tile that actually answers for it.
///
/// A tile at or above the source ceiling is drawn from its own 256² samples, so the
/// spacing is `256 / segments`. A tile *below* the ceiling is drawn from an ancestor's
/// samples over `2^(z − src.z)` times less ground per axis, so the same number of mesh
/// samples land that many times closer together — and once they land on every texel the
/// mesh reproduces the data it has and the error is zero.
pub(crate) fn mesh_step_on_source(id: TileId, src: TileId, segments: u32) -> usize {
    let levels = id.z.saturating_sub(src.z) as u32;
    let denom = (segments as u64) << levels.min(16);
    ((HEIGHT_TILE_DIM as u64) / denom.max(1)).max(1) as usize
}

/// `id`'s own share of `src`'s texel grid, as `[x0, x1, y0, y1)` — the window
/// [`deviation_over`] scores it on. `src == id` gives the whole tile.
pub(crate) fn source_window(id: TileId, src: TileId) -> [usize; 4] {
    let (u0, v0) = HeightTileManager::ancestor_uv(id, src, 0.0, 0.0);
    let (u1, v1) = HeightTileManager::ancestor_uv(id, src, 1.0, 1.0);
    let n = HEIGHT_TILE_DIM as f64;
    let q = |t: f64| (t * n).round().clamp(0.0, n) as usize;
    let (x0, x1, y0, y1) = (q(u0), q(u1), q(v0), q(v1));
    // A degenerate window (a tile so deep that its share rounds to nothing) is widened to
    // one texel rather than scored as empty, which would read as zero error.
    [x0, x1.max(x0 + 1), y0, y1.max(y0 + 1)]
}

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

/// One of the committed Terrarium fixtures, decoded. No network.
fn fixture(name: &str) -> HeightTile {
    let path = format!(
        "{}/assets/terrain_fixtures/{name}",
        env!("CARGO_MANIFEST_DIR")
    );
    let bytes = std::fs::read(&path).unwrap_or_else(|e| panic!("reading {path}: {e}"));
    let img = image::load_from_memory(&bytes).unwrap().to_rgba8();
    let (w, h) = (img.width(), img.height());
    cesium_engine::globe::terrain::decode_terrarium(w, h, &img.into_raw(), OceanPolicy::ClampToZero)
        .unwrap()
}

/// **The coupling F1 had to check before it could argue about density at all.**
///
/// `HeightTile::detail` is documented as *the* geometric error of the drawn surface, and
/// that is true at exactly one setting: `mesh_segments = 16`, where the mesh's 17×17
/// lattice **is** the 16:1 decimation the number is measured against. The doc comment on
/// [`HEIGHT_DETAIL_STEP`] states the caveat for other densities and, until this test, only
/// stated it.
///
/// The fixture is a ridge whose crests sit half way between two 16-lattice lines and
/// exactly **on** the 8-lattice lines. A mesh at `mesh_segments = 32` samples every eighth
/// texel, lands on every crest and draws the field exactly; `detail()` still reports the
/// whole 800 m amplitude, because it is not looking at that mesh. So at 32 the error term
/// E1 feeds into `apply_lod` is not this tile's error — it is the error of a mesh the
/// engine is no longer building, and it is 800 m too large.
///
/// That is a coupling between two knobs that are documented as independent, and it is the
/// reason F1's decision is not simply "refine the mesh and keep everything else".
#[test]
fn the_error_term_measures_a_sixteen_to_one_decimation_whatever_the_mesh_draws() {
    // First: this file's generalisation reproduces the engine's number at the engine's
    // step. Without this the rest of the file is measuring its own bug.
    for tile in [
        synthetic_tile(|_, _| 0.0),
        synthetic_tile(|x, y| 3.0 * x as f64 - 2.0 * y as f64),
        fixture("zugspitze_z12_2172_1433.png"),
        fixture("monterey_coast_z12_661_1599.png"),
    ] {
        assert_eq!(
            deviation_at_step(&tile, HEIGHT_DETAIL_STEP) as i64,
            tile.detail() as i64,
            "the generalised deviation must be the engine's own at 16:1, or no column \
             below is comparable with `HeightTile::detail`"
        );
    }

    // A ridge the 16-mesh chords away entirely and the 32-mesh draws exactly.
    let step = HEIGHT_DETAIL_STEP as f64;
    let ridge = synthetic_tile(|x, _| {
        let t = (x as f64 % step) / step;
        800.0 * (1.0 - (2.0 * t - 1.0).abs())
    });

    assert!(
        (ridge.detail() as f64 - 800.0).abs() <= 2.0,
        "the 16:1 decimation misses this crest entirely: {}",
        ridge.detail()
    );
    assert!(
        deviation_at_step(&ridge, HEIGHT_DETAIL_STEP / 2) <= 2.0,
        "…and a mesh_segments = 32 mesh lands on it and draws it: {}",
        deviation_at_step(&ridge, HEIGHT_DETAIL_STEP / 2)
    );
    assert!(
        deviation_at_step(&ridge, HEIGHT_DETAIL_STEP / 4) <= 2.0,
        "…as does 64"
    );

    // And the statement that makes it a *coupling* rather than a curiosity: the number
    // `apply_lod` reads does not move when the mesh does, because it is stored on the
    // decoded tile and the decoder has no access to the configuration.
    assert_eq!(
        ridge.detail(),
        synthetic_tile(|x, _| {
            let t = (x as f64 % step) / step;
            800.0 * (1.0 - (2.0 * t - 1.0).abs())
        })
        .detail(),
        "`detail` is a property of the decode, so no mesh density can change it"
    );
}

/// Vertex and index bytes of one tile's mesh at `segments`, taken off a mesh the engine
/// actually generates rather than from a formula — the topology (skirt ring included) is
/// what it is, and a formula here would be a second place for it to be wrong.
pub(crate) fn mesh_bytes(segments: u32) -> (usize, usize, usize) {
    // A mid-latitude z12 tile: no pole cap, so this is the ordinary case. Relief moves
    // vertices, it does not add them, so the flat mesh has the same counts as a
    // `Heightfield` one at the same density.
    let id = TileId {
        z: 12,
        x: 2172,
        y: 1433,
    };
    let mesh = TileMesh::generate(&id, segments);
    let vbuf = mesh.vertices.len() * std::mem::size_of::<Vertex>();
    let ibuf = mesh.indices.len() * std::mem::size_of::<u16>();
    (mesh.vertices.len(), vbuf, ibuf)
}

/// **The buffer cost of a density, with nothing measured over the network.**
///
/// The C4 table in `docs/terrain-plan.md` §6 quotes these bytes; this is the assertion
/// that they are still the bytes, since F1 argues from them.
#[test]
fn the_vertex_cost_of_a_density_is_what_c4_quoted() {
    for (segments, verts, vbuf, ibuf) in [
        (16u32, 361usize, 11_552usize, 3_888usize),
        (32, 1_225, 39_200, 13_872),
        (64, 4_489, 143_648, 52_272),
    ] {
        assert_eq!(
            mesh_bytes(segments),
            (verts, vbuf, ibuf),
            "the C4 buffer columns moved at mesh_segments = {segments}; \
             `docs/terrain-plan.md` §6 C4 and §9 F1 both need re-deriving"
        );
    }
}

// ── the measurements, against the real DEM ───────────────────────────────────────

/// A settled terrain quadtree over the real DEM at `p`, at a given mesh density.
///
/// `test_terrain_lod::settled_real_tree` with two things unfrozen: the mesh density
/// (which reaches the tree through `HeightBoundsSource`, because the skirt allowance a
/// node's box has to swallow depends on it) and the config it is derived from. Everything
/// else — the capture's `lod_factor`, `fog_density_for`, `TERRAIN_DEFAULT` — is that
/// function's, so the tile counts printed here sit next to §8's rather than beside them.
///
/// `frames`, when given, collects the visible set after **every** `update` — the sequence
/// of request sets a camera arriving at this pose really produces, which is what
/// [`super::test_height_residency`] replays. It is `None` for the tables here, which only
/// ever look at the settled tree.
pub(crate) fn settled_at_density(
    p: &ViewParams,
    frustum: &Frustum,
    config: &TileEngineConfig,
    texture_size_px: f32,
    world: &mut RealWorld,
    mut frames: Option<&mut Vec<Vec<TileId>>>,
) -> (QuadtreeManager<Heightfield>, HeightTileManager, usize) {
    let segments = config.mesh_segments;
    let mut heights = HeightTileManager::new(config);
    let mut qt = QuadtreeManager::<Heightfield>::for_surface();
    let cam = build_camera(p);
    qt.lod_factor = lod_factor_for(
        config.target_texel_ratio,
        texture_size_px,
        p.height as f32,
        cam.fovy(),
    );
    qt.max_zoom = config.max_zoom;
    qt.fog_density = cesium_engine::globe::quadtree::fog_density_for(p.alt_m as f32, &config.fog);
    qt.terrain_lod_factor = terrain_lod_factor_for(
        config.terrain.max_geometric_error_px,
        p.height as f32,
        cam.fovy(),
    );
    qt.terrain_fog_policy = TerrainFogPolicy::default();
    qt.terrain_fog_sse_ratio = if config.terrain.max_geometric_error_px > 0.0 {
        config.fog.sse / config.terrain.max_geometric_error_px
    } else {
        0.0
    };
    qt.pipeline = CullPipeline::TERRAIN_DEFAULT;
    let cam_alt = p.alt_m * 1.0e-6;

    fn source<'a>(
        heights: &'a HeightTileManager,
        segments: u32,
        exaggeration: f32,
        detail_max_z: u8,
    ) -> HeightBoundsSource<'a> {
        HeightBoundsSource {
            heights,
            segments,
            exaggeration,
            detail_max_z,
        }
    }
    let exaggeration = config.terrain.exaggeration;

    // Distinct source tiles the tree asked for over the whole settle — the working set,
    // which is the number the height cache's capacity has to be read against.
    let mut asked: std::collections::HashSet<TileId> = std::collections::HashSet::new();

    for _ in 0..=UPDATE_ITERATIONS {
        let mut wanted = Vec::new();
        for root in qt.roots.iter() {
            collect_sources(root, &heights, &mut wanted);
        }
        wanted.sort_unstable_by_key(|id| (id.z, id.x, id.y));
        wanted.dedup();
        asked.extend(wanted.iter().copied());
        fetch_missing(&world.dir, &wanted);
        for root in qt.roots.iter() {
            fill_cache_real(root, &mut heights, world);
        }
        qt.refresh_extras(&source(
            &heights,
            segments,
            exaggeration,
            config.terrain.detail_max_z,
        ));
        qt.refresh_terrain_horizon(frustum, cam_alt, cam_alt, &config.terrain.occlusion);
        qt.update(frustum);
        if let Some(f) = frames.as_deref_mut() {
            f.push(
                qt.get_visible_tiles()
                    .into_iter()
                    .map(|(id, _, _)| id)
                    .collect(),
            );
        }
    }
    // One more fill **after** the last `update`, which is the one that creates the final
    // generation of children. Without it a few percent of the settled tree has no height
    // tile behind it, `height_bounds_for` returns `None`, and both error columns silently
    // fall back to the level-based estimate — which is density-blind, so the whole
    // measurement would read "refining the mesh changes nothing" at exactly the poses
    // that refine deepest. `test_terrain_lod::settled_real_tree` does the same thing for
    // the same reason.
    let mut wanted = Vec::new();
    for root in qt.roots.iter() {
        collect_sources(root, &heights, &mut wanted);
    }
    wanted.sort_unstable_by_key(|id| (id.z, id.x, id.y));
    wanted.dedup();
    asked.extend(wanted.iter().copied());
    fetch_missing(&world.dir, &wanted);
    for root in qt.roots.iter() {
        fill_cache_real(root, &mut heights, world);
    }
    qt.refresh_extras(&source(
        &heights,
        segments,
        exaggeration,
        config.terrain.detail_max_z,
    ));
    (qt, heights, asked.len())
}

/// The settled visible set at one real pose, at the **shipped** config — the tile ids
/// `TileSystem::update_logic` would walk when it asks for height chains.
///
/// Exposed for [`super::test_height_residency`], which needs the request sequence and not
/// the tree, and which must not grow a second settle of its own: two settles that drift
/// apart would make its churn numbers incomparable with F2's residency numbers.
pub(crate) fn settled_shipped(
    p: &ViewParams,
    texture_size_px: f32,
    world: &mut RealWorld,
) -> (Vec<Vec<TileId>>, usize) {
    let shipped = TileEngineConfig {
        terrain: TerrainConfig {
            enabled: true,
            ..TerrainConfig::default()
        },
        ..TileEngineConfig::default()
    };
    let frustum = frustum_of(p);
    let mut frames = Vec::new();
    let (qt, _, asked) = settled_at_density(
        p,
        &frustum,
        &shipped,
        texture_size_px,
        world,
        Some(&mut frames),
    );
    frames.push(
        qt.get_visible_tiles()
            .into_iter()
            .map(|(id, _, _)| id)
            .collect(),
    );
    (frames, asked)
}

/// The frustum of one real pose.
pub(crate) fn frustum_of(p: &ViewParams) -> Frustum {
    let cam = build_camera(p);
    let aspect = p.aspect() as f32;
    let (eye, _) = cam.global_transform_f64();
    Frustum::planes_only(cam.calculate_frustum_planes(aspect), eye)
        .with_corners(cam.frustum_corners_relative(aspect))
}

/// p95 of a list, the same rule `test_terrain_lod::score` uses.
pub(crate) fn p95(v: &mut Vec<f64>) -> f64 {
    v.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    if v.is_empty() {
        0.0
    } else {
        v[((v.len() as f64 * 0.95) as usize).min(v.len() - 1)]
    }
}

/// **The F1 table** — what each mesh density costs and buys at the ten real poses.
///
/// Four columns per density, and they do not all move together, which is the finding:
///
/// * **tiles** — the visible set. Density reaches the quadtree only through the skirt
///   allowance (`heightfield::skirt_allowance`, whose sagitta term falls with it), so
///   this column barely moves. The LOD term does **not** see the density at all: it
///   refines on `HeightBounds::detail`, which is a property of the decode.
/// * **believed p95 error** — what the engine thinks is left on screen: the detail-based
///   number `max_geometric_error_px` is budgeting against.
/// * **drawn p95 error** — what is *actually* left on screen, from [`deviation_over`] at
///   the lattice the mesh really lays down, over that tile's own window of its source.
///   For a tile **at or above** the source ceiling the two columns are the same number at
///   `segments = 16` by construction; above 16 they separate, and the gap is the coupling
///   the first test in this file pins. **Below** the ceiling they differ even at 16,
///   because the believed column reads a z15 ancestor's whole-tile error while the drawn
///   one reads the child's own window at the finer effective lattice — the over-statement
///   `test_terrain_lod::score`'s footnote already names, here quantified rather than
///   restated (`terai_to_himalaya`: 39.5 px believed, 13.1 px drawn).
/// * **bytes** — vertex and index buffers, per tile and over the shipped 512-entry mesh
///   cache. This is the column that is 3.4× per doubling.
///
/// Both error columns are a p95 over a per-tile **maximum**, so they inherit C4's
/// non-monotonicity: a cliff deviates from its chord by roughly half its own height at
/// *every* lattice spacing, so refining moves it hardly at all. `himalaya_everest` is that
/// case in one row — 93.5 → 93.6 → 93.7 px across the three densities, while the smooth
/// alpine relief next to it falls by more than half. A density argued from that pose alone
/// would conclude density does nothing.
///
/// ```text
/// cargo test --release --lib terrain::test_mesh_density::f1_ -- --ignored --nocapture
/// ```
#[test]
#[ignore = "measurement, needs the network for real height tiles"]
fn f1_mesh_density_at_the_real_poses() {
    let mut world = RealWorld::new();
    let poses: Vec<(&'static str, ViewParams, Frustum)> = real_poses()
        .into_iter()
        .map(|(n, p)| {
            let f = frustum_of(&p);
            (n, p, f)
        })
        .collect();

    // `deviation_over` walks up to 65 536 texels and the same source tile answers for many
    // nodes at many poses; memoise on (source tile, step, window).
    let mut dev_cache: HashMap<(TileId, usize, [usize; 4]), f64> = HashMap::new();

    println!("  [F1] visible tiles / believed p95 err px / drawn p95 err px, by mesh_segments");
    print!("    {:<22}", "pose");
    for d in DENSITIES {
        print!(" {:>26}", format!("segments = {d}"));
    }
    println!();

    let mut totals = [0usize; DENSITIES.len()];
    let mut believed_at_alps = [0.0f64; DENSITIES.len()];
    let mut drawn_at_alps = [0.0f64; DENSITIES.len()];
    let mut tiles_at_alps = [0usize; DENSITIES.len()];

    for (name, p, frustum) in &poses {
        print!("    {name:<22}");
        for (i, d) in DENSITIES.iter().enumerate() {
            let mut config = real_config();
            config.mesh_segments = *d;
            let (qt, mut heights, _) =
                settled_at_density(p, frustum, &config, 256.0, &mut world, None);
            let cam = build_camera(p);
            let fovy = cam.fovy() as f64;
            let visible = qt.get_visible_tiles();

            let mut fallbacks = 0usize;
            let mut believed: Vec<f64> = Vec::with_capacity(visible.len());
            let mut drawn: Vec<f64> = Vec::with_capacity(visible.len());
            for (id, _, _) in &visible {
                let b = tile_bounds(id);
                let q = cesium_engine::globe::geometry::lon_lat_to_ecef_f64(
                    0.5 * (b.lon_min + b.lon_max),
                    0.5 * (b.lat_min + b.lat_max),
                );
                let dist = (DVec3::new(q[0], q[1], q[2]) - frustum.eye).length();

                // What the engine believes, exactly as `test_terrain_lod::score` reads it.
                let believed_mm = heights
                    .height_bounds_for(
                        *id,
                        *d,
                        config.terrain.exaggeration,
                        config.terrain.detail_max_z,
                    )
                    .map(|b| b.detail as f64)
                    .unwrap_or_else(|| fallback_detail_mm(id.z));
                believed.push(geometric_error_px(believed_mm, dist, p.height as f64, fovy));

                // What the mesh at this density actually leaves on screen.
                let drawn_mm = match heights.source_for(*id) {
                    Some((src, tile)) => {
                        let step = mesh_step_on_source(*id, src, *d);
                        let window = source_window(*id, src);
                        let m = *dev_cache
                            .entry((src, step, window))
                            .or_insert_with(|| deviation_over(&tile, step, window));
                        m * 1.0e-6 * config.terrain.exaggeration as f64
                    }
                    None => {
                        fallbacks += 1;
                        fallback_detail_mm(id.z)
                    }
                };
                drawn.push(geometric_error_px(drawn_mm, dist, p.height as f64, fovy));
            }
            let (bp, dp) = (p95(&mut believed), p95(&mut drawn));
            totals[i] += visible.len();
            if *name == "alps_inn_valley" {
                believed_at_alps[i] = bp;
                drawn_at_alps[i] = dp;
                tiles_at_alps[i] = visible.len();
            }
            assert_eq!(
                fallbacks, 0,
                "{name} at segments = {d}: {fallbacks} visible tiles had no height tile \
                 behind them and were scored with the level-based fallback, which is \
                 density-blind — the settle did not fill the tree it measured"
            );
            print!(" {:>8} {:>8.1} {:>8.1}", visible.len(), bp, dp);
        }
        println!();
    }

    print!("    {:<22}", "TOTAL tiles");
    for t in totals {
        print!(" {t:>26}");
    }
    println!("\n");

    println!("  [F1] the cost side, and the marginal column F1 is decided on");
    println!(
        "    {:<10} {:>9} {:>8} {:>12} {:>14} {:>14} {:>14} {:>16}",
        "segments",
        "Σ tiles",
        "verts",
        "buf B/tile",
        "drawn MB@alps",
        "mesh cache MB",
        "believed px",
        "drawn px @alps"
    );
    for (i, d) in DENSITIES.iter().enumerate() {
        let (verts, vbuf, ibuf) = mesh_bytes(*d);
        let per_tile = vbuf + ibuf;
        println!(
            "    {:<10} {:>9} {:>8} {:>12} {:>14.1} {:>14.1} {:>14.1} {:>16.1}",
            d,
            totals[i],
            verts,
            per_tile,
            (per_tile * tiles_at_alps[i]) as f64 / 1.0e6,
            (per_tile * MESH_CACHE_ENTRIES) as f64 / 1.0e6,
            believed_at_alps[i],
            drawn_at_alps[i],
        );
    }
    println!(
        "    (believed = HeightBounds::detail, the number apply_lod refines on, which no \
         density changes; drawn = the same quantity at the lattice the mesh really lays \
         down. Equal at 16 for every tile at or above the z15 source ceiling; below it \
         the believed column is the ancestor's whole-tile error.)"
    );
}

/// **How far the error term is off at the densities it was not built for**, on the
/// committed fixtures, with no network at all.
///
/// One row per fixture: the deviation of the field from its own decimation at 16:1 (which
/// is `detail()`, i.e. `mesh_segments = 16`), 8:1 (32) and 4:1 (64). The C4 table in §6
/// measures the drawn mesh's full error including the map projection; this measures the
/// *same metric the engine stores*, so the three columns are directly comparable with
/// each other and with what `apply_lod` reads.
#[test]
#[ignore = "measurement, not a gate: prints the table §9 F1 argues from"]
fn f1_the_error_term_at_the_densities_it_was_not_built_for() {
    println!("\n  [F1] HeightTile::detail (m) against the decimation each density really draws");
    println!(
        "    {:<34} {:>12} {:>12} {:>12} {:>14}",
        "fixture", "16 (=detail)", "32", "64", "detail/64"
    );
    for name in [
        "everest_z12_3037_1716.png",
        "zugspitze_z12_2172_1433.png",
        "monterey_coast_z12_661_1599.png",
        "dead_sea_z12_2451_1670.png",
        "pacific_z12_341_2048.png",
    ] {
        let tile = fixture(name);
        let d16 = deviation_at_step(&tile, HEIGHT_DETAIL_STEP);
        let d32 = deviation_at_step(&tile, HEIGHT_DETAIL_STEP / 2);
        let d64 = deviation_at_step(&tile, HEIGHT_DETAIL_STEP / 4);
        let ratio = if d64 > 0.0 {
            format!("{:.1}x", d16 / d64)
        } else {
            "—".to_string()
        };
        println!("    {name:<34} {d16:>12.0} {d32:>12.0} {d64:>12.0} {ratio:>14}");
    }
    println!(
        "    (the first column is what `apply_lod` reads at every density; the other two \
         are what the drawn mesh is actually wrong by there.)"
    );
}

/// **The F2 split** — imagery bytes against height bytes against vertex bytes, at the ten
/// real poses and at the **shipped** budget.
///
/// §9 asks for the split and B4 makes a claim about it that a config listing cannot
/// settle: the height cache is a *declared slice* of `tile_cache_budget_bytes`, not an
/// addition to it. The measurement that settles it is the resident set at a real camera,
/// against the capacity that slice derives — so this runs `TileEngineConfig::default()`
/// with terrain on, **not** `real_config()`, whose 1 GiB height budget exists precisely so
/// that the occlusion measurements never evict and would hide the question.
///
/// Two imagery styles, because the answer differs and both ship: the default Carto `@2x`
/// (512², 1 MiB a tile) and the Esri satellite style the §7/§8 tables are measured at
/// (256², 256 kB a tile, and a `lod_factor` twice as eager, so it draws more tiles).
#[test]
#[ignore = "measurement, needs the network for real height tiles"]
fn f2_where_the_bytes_go_at_the_real_poses() {
    let mut world = RealWorld::new();
    let shipped = TileEngineConfig {
        terrain: TerrainConfig {
            enabled: true,
            ..TerrainConfig::default()
        },
        ..TileEngineConfig::default()
    };

    println!("\n  [F2] the declared split, from the shipped config");
    println!(
        "    tile_cache_budget_bytes   {:>8.1} MiB",
        shipped.tile_cache_budget_bytes as f64 / (1024.0 * 1024.0)
    );
    println!(
        "      imagery share           {:>8.1} MiB   ({} entries at 512²×4)",
        shipped.imagery_cache_budget_bytes() as f64 / (1024.0 * 1024.0),
        shipped.imagery_cache_budget_bytes() / (512 * 512 * 4)
    );
    println!(
        "      height share            {:>8.1} MiB   ({} entries at {} B)",
        shipped.terrain.height_cache_budget_bytes as f64 / (1024.0 * 1024.0),
        shipped.terrain.height_cache_budget_bytes / HEIGHT_TILE_BYTES,
        HEIGHT_TILE_BYTES,
    );
    let (verts, vbuf, ibuf) = mesh_bytes(shipped.mesh_segments);
    println!(
        "    vertex buffers            {:>8.1} MB    ({} verts/tile, {} B/tile, {} entries — \
         a COUNT cap, outside the byte budget)",
        ((vbuf + ibuf) * MESH_CACHE_ENTRIES) as f64 / 1.0e6,
        verts,
        vbuf + ibuf,
        shipped.mesh_cache_size.get(),
    );

    for (style, texture_px, bytes_per_tile) in [
        ("carto @2x 512²", 512.0f32, 512usize * 512 * 4),
        ("esri 256²", 256.0, 256 * 256 * 4),
    ] {
        println!("\n  [F2] measured at the real poses — {style}");
        println!(
            "    {:<22} {:>7} {:>12} {:>10} {:>10} {:>12} {:>12}",
            "pose", "tiles", "imagery MiB", "h wanted", "h resident", "height MiB", "vertex MB"
        );
        let (mut t_tiles, mut t_img, mut t_h, mut t_v) = (0usize, 0.0f64, 0.0f64, 0.0f64);
        for (name, p) in real_poses() {
            let frustum = frustum_of(&p);
            let (qt, heights, asked) =
                settled_at_density(&p, &frustum, &shipped, texture_px, &mut world, None);
            let tiles = qt.get_visible_tiles().len();
            let (resident, capacity) = heights.residency();
            let img_mib = (tiles * bytes_per_tile) as f64 / (1024.0 * 1024.0);
            let h_mib = heights.resident_bytes() as f64 / (1024.0 * 1024.0);
            let v_mb = (tiles * (vbuf + ibuf)) as f64 / 1.0e6;
            println!(
                "    {name:<22} {tiles:>7} {img_mib:>12.1} {asked:>10} {:>10} {h_mib:>12.1} \
                 {v_mb:>12.1}",
                format!("{resident}/{capacity}")
            );
            t_tiles += tiles;
            t_img += img_mib;
            t_h += h_mib;
            t_v += v_mb;
        }
        println!(
            "    {:<22} {t_tiles:>7} {t_img:>12.1} {:>10} {:>10} {t_h:>12.1} {t_v:>12.1}",
            "TOTAL / Σ", "", ""
        );
        println!(
            "    (one pose's working set at a time — the columns are per pose, not a sum \
             of resident sets; the TOTAL row sums them only to size the whole flight.)"
        );
    }
}
