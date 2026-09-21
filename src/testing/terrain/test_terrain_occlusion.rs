//! **D3 acceptance** — culling tiles hidden *behind mountains*
//! (`docs/terrain-plan.md` §3.3, §7b and §7c).
//!
//! **Nothing in the gate touches the network.** The three `#[ignore]`d measurements at the
//! bottom of this file do: they run the stage against the **real** DEM, because §7b's
//! central finding is that the synthetic ridge world and real terrain disagree by an order
//! of magnitude about what D3 is worth, and a harness that cannot be run in a loop over
//! real data cannot settle that. They cache the terrarium tiles under
//! `CESIUM_HEIGHT_CACHE` and fetch with `curl`, so the root crate acquires no HTTP
//! dependency for a measurement.
//!
//! # Why this is not in `culling::`, and why `culling` is not in its path
//!
//! Same reason as `test_terrain_visibility`'s module doc, restated because it is the
//! mistake that has already been made twice: `cargo test --release --lib culling::` must
//! keep reporting **32 passed, 0 failed, 1 ignored**, and libtest's filter is a plain
//! substring match on the full test path. A module called `test_terrain_culling` would
//! join the gate and change the number being held fixed. Hence `test_terrain_occlusion`.
//!
//! # The truth this file measures against, and why it had to change
//!
//! `test_terrain_visibility` calls a vertex visible when it is on screen and off the
//! limb, and says so explicitly: *"a vertex hidden behind a mountain still counts as
//! visible here. Closing that last gap is D3's job."* That criterion cannot score D3 —
//! every tile D3 correctly culls would register as a false negative under it.
//!
//! So the oracle here adds the third term, and it is deliberately built out of something
//! **other** than the machinery under test:
//!
//! > A node is a false negative if it was culled while some vertex of its own mesh is
//! > inside the frustum (exact f64), not occluded by the ellipsoid (exact, Theorem 3.1),
//! > **and** not occluded by the drawn surface — determined by marching the segment from
//! > the eye to that vertex in 128 steps and asking, at each step, whether the segment is
//! > below a *lower bound on the mesh that the reference (D1+D2-only) tree actually
//! > draws* at that point.
//!
//! The engine's own answer comes from a 32 × 16 polar grid of node floors; the oracle's
//! comes from a dense march against the analytic field the meshes are built from. They
//! share no code, which is the point.
//!
//! **The oracle's occluder is a lower bound on the drawn mesh, derived the safe way.**
//! A mesh vertex is a sample of the field, and the mesh between vertices is a linear
//! interpolation of four such samples, so the drawn surface over one grid cell is never
//! below the *minimum of the field over that cell*. [`mesh_floor`] computes exactly that
//! — minimum over the cell of the drawn tile, at the level the reference tree actually
//! drew — and then subtracts [`ORACLE_SLACK_M`] again. Every approximation in the oracle
//! therefore pushes it toward calling a vertex **visible**, which makes the false-negative
//! count an over-estimate and never an under-estimate. A green result here is the strong
//! statement.
//!
//! # The world
//!
//! A single continuous east–west ridge with one deliberate **col** in it
//! ([`LON_GAP`]). The col is not decoration: `docs/terrain-plan.md` §3.3's claim that
//! lateral gaps are handled automatically — because a cell's floor is the minimum over
//! its whole footprint — is exactly the claim a col tests, and
//! [`d3_does_not_cull_through_the_col`] is that test.

use std::collections::HashMap;
use std::sync::Arc;

use cesium_engine::camera::camera::CameraMode;
use cesium_engine::globe::geometry::{TileMesh, EARTH_RADIUS_A_F64, EARTH_RADIUS_B_F64};
use cesium_engine::globe::quadtree::terrain_occlusion::{ridge_safety_m, MIN_RANGE_M};
use cesium_engine::globe::quadtree::{
    lod_factor_for, tile_bounds, transform_to_scaled_space, web_mercator_y_to_lat_f64,
    CullPipeline, Frustum, HorizonCamera, QuadtreeManager, QuadtreeNode, TerrainHorizon,
    TerrainOcclusionConfig, TileId, AZIMUTH_SECTORS, RANGE_RINGS,
};
use cesium_engine::globe::terrain::height_tile::{HEIGHT_TILE_DIM, HEIGHT_TILE_TEXELS};
use cesium_engine::globe::terrain::{
    HeightBoundsSource, HeightPatch, HeightTile, HeightTileManager, Heightfield, PatchStatus,
};
use cesium_engine::globe::tiles::config::{OceanPolicy, TerrainConfig, TileEngineConfig};
use glam::DVec3;

use crate::testing::culling::cameras::{build_camera, Lcg, ViewParams};
use crate::testing::culling::oracle::{VisibilityOracle, NDC_MARGIN};

/// Mesh density, the shipped default — the geometry checked is the geometry that ships.
pub(crate) const SEGMENTS: u32 = 16;

/// Deepest level the sweep's quadtree refines to, and the depth the synthetic source
/// "serves" to. Deep enough that a valley tile is a kilometre across (which is the scale
/// D3 operates at) and shallow enough that a pose does not spend its budget building
/// meshes for tiles a metre wide.
const SWEEP_MAX_ZOOM: u8 = 14;

/// Update ticks per pose, matching every other sweep in the repo: `apply_lod`'s 20 %
/// hysteresis, `reorder_children_near_to_far` and D1's bounds refresh all need a few.
pub(crate) const UPDATE_ITERATIONS: usize = 4;

/// Limb band, in scaled-space units — `test_terrain_visibility`'s constant, same role.
const LIMB_BAND_SCALED: f64 = 1.0e-9;

/// Extra metres shaved off the oracle's occluder, on top of its already-conservative
/// minimum-over-the-cell construction.
///
/// Covers the two things that construction does not: the mesh vertex is a *bilinear
/// sample of a 256² texel grid* of the field rather than the field itself (sub-metre for
/// a field this smooth at these levels), and the mesh splits each grid cell into two
/// triangles whose planes can sit slightly below the bilinear surface the minimum bounds.
/// 25 m is one to two orders above either. It makes the oracle claim "visible" more
/// often, which is the direction that cannot hide a false negative.
const ORACLE_SLACK_M: f64 = 25.0;

/// Steps in the oracle's ray march from the eye to a candidate vertex.
///
/// Uniform in distance. At the ranges these poses use (10–60 km) that is 80–470 m per
/// step against a ridge whose half-width is 5 km, so the march cannot step over the
/// occluder — which would make the oracle report a false negative that is not there.
const MARCH_STEPS: usize = 256;

// ── the synthetic world ──────────────────────────────────────────────────────────

/// Latitude of the ridge crest, degrees.
const LAT_RIDGE: f64 = 47.40;
/// Gaussian half-width of the ridge in latitude, degrees (~5 km).
const SIG_RIDGE: f64 = 0.045;
/// Crest height above the valley floor, metres.
const RIDGE_M: f64 = 2_800.0;
/// Valley / plateau floor, metres.
const BASE_M: f64 = 600.0;
/// Longitude of the col — the one place the ridge drops back to the valley floor.
const LON_GAP: f64 = 11.35;
/// Gaussian half-width of the col in longitude, degrees (~5 km at this latitude).
const SIG_GAP: f64 = 0.065;

/// The field, in metres: a plateau with one continuous ridge across it and one col
/// through the ridge.
///
/// Smooth and closed-form on purpose. The engine reads it only through 256² `HeightTile`
/// texels (so it exercises B3's mip and D1's bounds exactly as real data would), and the
/// oracle reads it directly, so the two never share an approximation.
fn height_m(lon: f64, lat: f64, ridge_m: f64) -> f64 {
    let g = ((lat - LAT_RIDGE) / SIG_RIDGE).powi(2);
    let col = 1.0 - (-((lon - LON_GAP) / SIG_GAP).powi(2)).exp();
    BASE_M + ridge_m * col * (-g).exp()
}

/// Height tiles of [`height_m`], built on demand and cached.
struct RidgeWorld {
    ridge_m: f64,
    tiles: HashMap<TileId, Arc<HeightTile>>,
}

impl RidgeWorld {
    fn new(ridge_m: f64) -> Self {
        Self {
            ridge_m,
            tiles: HashMap::new(),
        }
    }

    fn tile(&mut self, id: TileId) -> Arc<HeightTile> {
        if let Some(t) = self.tiles.get(&id) {
            return t.clone();
        }
        let b = tile_bounds(&id);
        let n = HEIGHT_TILE_DIM as f64;
        let mut data = Box::new([0i16; HEIGHT_TILE_TEXELS]);
        for ty in 0..HEIGHT_TILE_DIM {
            // Texel centres, and `v` in Mercator y so the texel grid lines up with the
            // mesh's own parameterisation.
            let v = (ty as f64 + 0.5) / n;
            let lat = web_mercator_y_to_lat_f64(id.y as f64 + v, id.z);
            for tx in 0..HEIGHT_TILE_DIM {
                let u = (tx as f64 + 0.5) / n;
                let lon = b.lon_min + u * (b.lon_max - b.lon_min);
                data[ty * HEIGHT_TILE_DIM + tx] = height_m(lon, lat, self.ridge_m) as i16;
            }
        }
        let t = Arc::new(HeightTile::from_samples(data));
        self.tiles.insert(id, t.clone());
        t
    }
}

fn world_config() -> TileEngineConfig {
    TileEngineConfig {
        mesh_segments: SEGMENTS,
        terrain: TerrainConfig {
            enabled: true,
            exaggeration: 1.0,
            // The field is positive everywhere, so the policy cannot change a number;
            // `Raw` is chosen so it also cannot hide one.
            ocean: OceanPolicy::Raw,
            max_level: SWEEP_MAX_ZOOM,
            height_cache_budget_bytes: 512 * 1024 * 1024,
            ..TerrainConfig::default()
        },
        ..TileEngineConfig::default()
    }
}

/// Makes every node currently in the tree `Ready` from [`RidgeWorld`].
fn fill_cache(
    node: &QuadtreeNode<Heightfield>,
    heights: &mut HeightTileManager,
    w: &mut RidgeWorld,
) {
    let src = heights.source_tile_for(node.id);
    if heights.status_of(node.id) != PatchStatus::Ready {
        let t = w.tile(src);
        heights.insert_ready(src, t);
    }
    if let Some(children) = &node.children {
        for c in children.iter() {
            fill_cache(c, heights, w);
        }
    }
}

pub(crate) fn bounds_source(heights: &HeightTileManager) -> HeightBoundsSource<'_> {
    HeightBoundsSource {
        heights,
        segments: SEGMENTS,
        exaggeration: 1.0,
        // The shipped ceiling, so §7b/§7c keep measuring the engine that ships — §9 F5
        // raised it from 15 to 19 and the D3 numbers move with it, which is the point.
        detail_max_z: TerrainConfig::default().detail_max_z,
    }
}

fn frustum_for(p: &ViewParams) -> (Frustum, VisibilityOracle) {
    let cam = build_camera(p);
    let aspect = p.aspect();
    let (eye, _) = cam.global_transform_f64();
    let frustum = Frustum::planes_only(cam.calculate_frustum_planes(aspect as f32), eye)
        .with_corners(cam.frustum_corners_relative(aspect as f32));
    (frustum, VisibilityOracle::new(&cam, aspect))
}

/// Builds a settled terrain quadtree over the ridge world.
///
/// `occlusion` decides whether D3 runs: `None` gives the D1+D2 reference tree — what the
/// engine drew before this change — and `Some(cfg)` gives the tree under test. Everything
/// else about the two is identical, which is what makes the tile counts a controlled
/// delta rather than two unrelated numbers.
fn settled_tree(
    p: &ViewParams,
    frustum: &Frustum,
    occlusion: Option<TerrainOcclusionConfig>,
    ridge_m: f64,
) -> (QuadtreeManager<Heightfield>, HeightTileManager, RidgeWorld) {
    let config = world_config();
    let mut heights = HeightTileManager::new(&config);
    let mut world = RidgeWorld::new(ridge_m);
    let mut qt = QuadtreeManager::<Heightfield>::for_surface();
    qt.max_zoom = SWEEP_MAX_ZOOM;
    qt.pipeline = match occlusion {
        Some(_) => CullPipeline::TERRAIN_DEFAULT,
        None => CullPipeline::DEFAULT,
    };
    let cam_alt = p.alt_m * 1.0e-6;
    // §7f's gate is written in height above the **ground**, and the ridge world's ground
    // under every pose in this file is the plateau. `wgpu_state` gets this from
    // `Camera::altitude_agl`; here it is one subtraction, and getting it wrong would shut
    // the march off at every pose the file measures.
    let cam_agl = cam_alt - BASE_M * 1.0e-6;

    for _ in 0..UPDATE_ITERATIONS {
        for root in qt.roots.iter() {
            fill_cache(root, &mut heights, &mut world);
        }
        qt.refresh_extras(&bounds_source(&heights));
        match &occlusion {
            Some(cfg) => qt.refresh_terrain_horizon(frustum, cam_alt, cam_agl, cfg),
            None => qt.clear_terrain_horizon(),
        }
        qt.update(frustum);
    }
    for root in qt.roots.iter() {
        fill_cache(root, &mut heights, &mut world);
    }
    qt.refresh_extras(&bounds_source(&heights));
    (qt, heights, world)
}

// ── the oracle ───────────────────────────────────────────────────────────────────

/// Longitude, Web-Mercator latitude and altitude (megametres) of an ECEF point.
///
/// The inverse of `geometry::lon_lat_to_ecef_f64`, which parameterises the ellipsoid as
/// `(a cos φ cos θ, b sin φ, −a cos φ sin θ)`. Altitude is measured **radially** rather
/// than along the normal: the two differ by a factor `1 − 1.8·10⁻⁵` of the altitude, i.e.
/// centimetres at 3 km, which is three orders inside [`ORACLE_SLACK_M`].
fn geodetic_of(q: DVec3) -> (f64, f64, f64) {
    let r = q.length();
    let d = q / r;
    let lambda = 1.0
        / ((d.x * d.x + d.z * d.z) / (EARTH_RADIUS_A_F64 * EARTH_RADIUS_A_F64)
            + d.y * d.y / (EARTH_RADIUS_B_F64 * EARTH_RADIUS_B_F64))
            .sqrt();
    let s = d * lambda;
    let lon = (-s.z).atan2(s.x).to_degrees();
    let lat = (s.y / EARTH_RADIUS_B_F64)
        .atan2((s.x * s.x + s.z * s.z).sqrt() / EARTH_RADIUS_A_F64)
        .to_degrees();
    (lon, lat, r - lambda)
}

/// Web-Mercator column and row of a point at level `z`, as fractions.
fn mercator_xy(lon: f64, lat: f64, z: u8) -> (f64, f64) {
    let n = (1_u64 << z) as f64;
    let x = (lon + 180.0) / 360.0 * n;
    let lat = lat.clamp(-85.051_128, 85.051_128);
    let y = (1.0 - lat.to_radians().tan().asinh() / std::f64::consts::PI) * 0.5 * n;
    (x, y)
}

/// The **drawn** leaf of `roots` covering `(lon, lat)`, or `None` where nothing is drawn.
///
/// `None` is the honest answer for ground the reference tree culled: there is no surface
/// there, so nothing there can occlude. Claiming an occluder in a culled region would let
/// the oracle hide a false negative, which is the one thing it must not do.
fn drawn_leaf(roots: &[QuadtreeNode<Heightfield>; 4], lon: f64, lat: f64) -> Option<TileId> {
    let (rx, ry) = mercator_xy(lon, lat, 1);
    let (rx, ry) = (rx.floor() as u32, ry.floor() as u32);
    let mut node = roots
        .iter()
        .find(|n| n.id.x == rx.min(1) && n.id.y == ry.min(1))?;
    loop {
        if !node.visible {
            return None;
        }
        let Some(children) = &node.children else {
            return Some(node.id);
        };
        let (cx, cy) = mercator_xy(lon, lat, node.id.z + 1);
        let (cx, cy) = (cx.floor() as u32, cy.floor() as u32);
        let next = children
            .iter()
            .find(|c| c.id.x == cx && c.id.y == cy)
            .or_else(|| children.iter().find(|c| c.visible));
        match next {
            Some(c) => node = c,
            None => return Some(node.id),
        }
    }
}

/// A **lower bound on the drawn mesh** of `id` at `(lon, lat)`, in metres.
///
/// The mesh interpolates the field linearly between grid posts, so over one grid cell it
/// never dips below the minimum of the field over that cell. The field varies with
/// latitude far faster than with longitude here, so the minimum over the cell is taken at
/// whichever latitude edge is farther from the crest, and over longitude at whichever
/// edge is farther from the col's centre — both closed form, both exact for this field,
/// and neither shares a line of code with the engine's answer.
fn mesh_floor(id: TileId, lon: f64, lat: f64, ridge_m: f64) -> f64 {
    let b = tile_bounds(&id);
    let n = SEGMENTS as f64;
    let seg_lon = (b.lon_max - b.lon_min) / n;
    let i = ((lon - b.lon_min) / seg_lon).floor().clamp(0.0, n - 1.0);
    let (lon0, lon1) = (b.lon_min + i * seg_lon, b.lon_min + (i + 1.0) * seg_lon);

    // The mesh's rows are uniform in Mercator y, not in latitude.
    let (_, my) = mercator_xy(lon, lat, id.z);
    let v = (my - id.y as f64).clamp(0.0, 1.0);
    let j = (v * n).floor().clamp(0.0, n - 1.0);
    let lat0 = web_mercator_y_to_lat_f64(id.y as f64 + j / n, id.z);
    let lat1 = web_mercator_y_to_lat_f64(id.y as f64 + (j + 1.0) / n, id.z);

    // Farthest-from-the-crest latitude edge, farthest-from-the-col-centre longitude edge:
    // `height_m` decreases monotonically away from the crest in latitude and increases
    // monotonically away from the col centre in longitude, so the corner minimum is the
    // cell minimum.
    let lat_far = if (lat0 - LAT_RIDGE).abs() > (lat1 - LAT_RIDGE).abs() {
        lat0
    } else {
        lat1
    };
    let lon_near = if (lon0 - LON_GAP).abs() < (lon1 - LON_GAP).abs() {
        lon0
    } else {
        lon1
    };
    height_m(lon_near, lat_far, ridge_m)
}

/// Does the drawn surface block the segment from `eye` to `p`?
///
/// The oracle's half of D3, and deliberately nothing like the engine's: a dense march
/// against a closed-form field, with the drawn level read off the reference tree.
fn truth_occluded(
    roots: &[QuadtreeNode<Heightfield>; 4],
    eye: DVec3,
    p: DVec3,
    ridge_m: f64,
) -> bool {
    let seg = p - eye;
    for i in 1..MARCH_STEPS {
        let q = eye + seg * (i as f64 / MARCH_STEPS as f64);
        let (lon, lat, alt) = geodetic_of(q);
        let Some(id) = drawn_leaf(roots, lon, lat) else {
            continue;
        };
        if alt * 1.0e6 < mesh_floor(id, lon, lat, ridge_m) - ORACLE_SLACK_M {
            return true;
        }
    }
    false
}

/// Theorem 3.1 in scaled space — `test_terrain_visibility`'s helper, same expressions.
fn scaled_point_is_occluded(cam: &HorizonCamera, q: DVec3) -> bool {
    if !cam.active {
        return false;
    }
    let h2 = cam.c2 - 1.0;
    let v = q - cam.c;
    let s = cam.c2 - q.dot(cam.c);
    s > h2 && s * s > h2 * v.dot(v)
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Verdict {
    /// On screen, off the limb and **not behind terrain**. A culled node holding one of
    /// these is a false negative for D3.
    Visible,
    /// In the viewport or limb numeric band. Neither scored nor ignored — counted.
    Marginal,
    Hidden,
}

fn classify(
    oracle: &VisibilityOracle,
    cam: &HorizonCamera,
    roots: &[QuadtreeNode<Heightfield>; 4],
    eye: DVec3,
    p: DVec3,
) -> Verdict {
    let c = oracle.clip(p);
    if c.w <= 0.0 {
        return Verdict::Hidden;
    }
    if c.w < 1.0e-12 * c.truncate().length().max(1.0) {
        return Verdict::Marginal;
    }
    let ndc = c.truncate() / c.w;
    let out = (ndc.x.abs() - 1.0)
        .max(ndc.y.abs() - 1.0)
        .max(-ndc.z)
        .max(ndc.z - 1.0);
    if out > NDC_MARGIN {
        return Verdict::Hidden;
    }
    if out > -NDC_MARGIN {
        return Verdict::Marginal;
    }

    let q = transform_to_scaled_space(p);
    if scaled_point_is_occluded(cam, q) {
        return Verdict::Hidden;
    }
    if scaled_point_is_occluded(cam, q * (1.0 - LIMB_BAND_SCALED)) {
        return Verdict::Marginal;
    }
    if truth_occluded(roots, eye, p, RIDGE_M) {
        return Verdict::Hidden;
    }
    Verdict::Visible
}

fn mesh_points(mesh: &TileMesh) -> Vec<DVec3> {
    let c = DVec3::from_array(mesh.center_f64);
    mesh.vertices
        .iter()
        .map(|v| {
            c + DVec3::new(
                v.position[0] as f64,
                v.position[1] as f64,
                v.position[2] as f64,
            )
        })
        .collect()
}

/// Every node the traversal touched, with whether it was culled.
fn collect(node: &QuadtreeNode<Heightfield>, out: &mut Vec<(TileId, bool)>, leaves: &mut usize) {
    if !node.visible {
        out.push((node.id, true));
        return;
    }
    match &node.children {
        Some(children) => {
            for c in children.iter() {
                collect(c, out, leaves);
            }
        }
        None => {
            *leaves += 1;
            out.push((node.id, false));
        }
    }
}

// ── poses ────────────────────────────────────────────────────────────────────────

fn pose(sweep: &'static str, lat: f64, lon: f64, alt_m: f64, pitch: f64) -> ViewParams {
    ViewParams {
        sweep,
        lat_deg: lat,
        lon_deg: lon,
        alt_m,
        pitch_deg: pitch,
        yaw_deg: 0.0, // due north, into the ridge
        roll_deg: 0.0,
        width: 1280,
        height: 720,
        mode: CameraMode::Free,
    }
}

/// The four regimes `docs/terrain-plan.md` §3.3 says D3 pays off unevenly across, plus
/// the cruise control it says it should *not* pay off at.
///
/// All look due north into the ridge from the south side, differing only in how far back
/// and how high the camera is — which is precisely the variable D3's benefit depends on.
fn reduction_poses() -> Vec<(&'static str, ViewParams)> {
    vec![
        ("valley", pose("d3", LAT_RIDGE - 0.10, 11.0, 1_200.0, 88.0)),
        (
            "approach",
            pose("d3", LAT_RIDGE - 0.28, 11.0, 3_000.0, 84.0),
        ),
        ("cockpit", pose("d3", LAT_RIDGE - 0.16, 11.0, 2_000.0, 90.0)),
        (
            "cruise_11km",
            pose("d3", LAT_RIDGE - 0.60, 11.0, 11_000.0, 80.0),
        ),
    ]
}

// ── the safety distance, and the error it is there to cover ──────────────────────

/// Furthest a candidate's own bearing can sit from the centre bearing of a sector whose
/// ridge value is allowed to cull it — half a sector, in degrees, times a margin.
///
/// `TerrainHorizon::finish` builds every wall on the sector's **centre** bearing, and
/// `TerrainHorizon::occludes` takes the minimum over the sectors a candidate's disc
/// touches. The soundness paragraph only needs *one* of those sectors to hold against the
/// ray, and the one whose band contains the ray's own bearing is at most half a sector
/// from it — so half a sector is the exact spread a wall has to survive. The 1.5× is
/// because `sector_range`'s outward rounding is an inequality this constant should not
/// have to be exactly right about.
const SECTOR_BEARING_SPREAD_DEG: f64 = 1.5 * 180.0 / AZIMUTH_SECTORS as f64;

/// The factor [`ridge_safety_m`] is required to keep over the worst *measured* placement
/// need, across the whole swept box.
///
/// Asserted rather than merely printed, so that lowering `RIDGE_SAFETY_RATE` past what it
/// covers goes red here, in a test that names the quantity, rather than three commits
/// later in a capture that shows a hole in a mountain.
const REQUIRED_RESERVE: f64 = 3.0;

/// The flat constant `ridge_safety_m` replaced, metres — `RIDGE_SAFETY_M` as it stood at
/// `dfd6891`.
///
/// Kept as a number here because the useful claim about a **relaxation** is not "the new
/// law is sound" on its own but "it is short nowhere the old one held", and that is a
/// comparison, which needs both sides.
const REPLACED_CONSTANT_M: f64 = 100.0;

/// What one swept placement case reports.
struct PlacementNeed {
    /// Exact metres of lowering that put the march's wall level with the ground its cell
    /// stands for. Zero when the wall is already under it.
    need_m: f64,
    /// `|Δalt| + s²/R`, the shape [`ridge_safety_m`]'s law claims, metres.
    denom_m: f64,
    /// What [`ridge_safety_m`] actually charges here, metres.
    safety_m: f64,
}

/// **One case of the two constructions, solved exactly.**
///
/// `TerrainHorizon::finish` places a cell's wall by rotating the eye's ellipsoid normal by
/// the cell's angular distance `γ`, which moves the **geodetic** latitude by `γ`. But `γ`
/// is the equirectangular distance `extent_of` bins that cell with, and *that* one is
/// measured in the engine's **parametric** latitude — the `φ` of `lon_lat_to_ecef_f64`,
/// which is what names a tile row. One parametric radian of ground is not one geodetic
/// radian of ground, so the wall stands a little away from the ground its cell bounds.
///
/// # What the reference point is, and why that one
///
/// The module's soundness paragraph turns on one inequality. A candidate `p` is culled
/// only when `θ_p < ridge[β][i]` for some ring `i` nearer than `p`, and the conclusion
/// "the ray is inside solid ground" needs the ray to pass **under the real terrain where
/// ring `i`'s far edge actually is**. That place is named in `extent_of`'s metric, because
/// that is the metric the cell was binned in and the metric `occludes` compares `near`
/// against; and it is on **the ray's own bearing**, because that is where the ray is. So
/// what `ridge[β][i]` must not exceed is
///
/// > the elevation angle of the ground point at `extent_of`-distance `ring_far[i]`, on the
/// > ray's bearing, at altitude `floor[β][i]`.
///
/// Both sides are built here from scratch — neither of them by calling the engine, which
/// is the point — and the solve is a bisection on a function monotone in the lowering by
/// construction, with no small-angle step anywhere in it.
///
/// `off_deg` is how far the ray's bearing sits from the sector centre the wall was built
/// on; zero is the pure **placement** error, which is what `ridge_safety_m` is for.
fn placement_need(
    lat: f64,
    eye_m: f64,
    floor_m: f64,
    s_m: f64,
    b_deg: f64,
    off_deg: f64,
) -> PlacementNeed {
    const R_M: f64 = EARTH_RADIUS_A_F64 * 1.0e6;

    let surface = |lon: f64, lat: f64| -> DVec3 {
        let (phi, theta) = (lat.to_radians(), lon.to_radians());
        DVec3::new(
            EARTH_RADIUS_A_F64 * phi.cos() * theta.cos(),
            EARTH_RADIUS_B_F64 * phi.sin(),
            -EARTH_RADIUS_A_F64 * phi.cos() * theta.sin(),
        )
    };
    let normal = |p: DVec3| -> DVec3 {
        DVec3::new(
            p.x / (EARTH_RADIUS_A_F64 * EARTH_RADIUS_A_F64),
            p.y / (EARTH_RADIUS_B_F64 * EARTH_RADIUS_B_F64),
            p.z / (EARTH_RADIUS_A_F64 * EARTH_RADIUS_A_F64),
        )
        .normalize()
    };
    // A geodetic point, the way the *reference* side names one: parametric latitude,
    // altitude along the normal.
    let at = |lon: f64, lat: f64, alt_m: f64| -> DVec3 {
        let s = surface(lon, lat);
        s + normal(s) * (alt_m * 1.0e-6)
    };
    // The ellipsoid point whose outward normal is `n` — `finish`'s own placement, from the
    // implicit form.
    let foot = |n: DVec3| -> DVec3 {
        let lam = 1.0
            / (EARTH_RADIUS_A_F64 * EARTH_RADIUS_A_F64 * (n.x * n.x + n.z * n.z)
                + EARTH_RADIUS_B_F64 * EARTH_RADIUS_B_F64 * n.y * n.y)
                .sqrt();
        DVec3::new(
            EARTH_RADIUS_A_F64 * EARTH_RADIUS_A_F64 * n.x,
            EARTH_RADIUS_B_F64 * EARTH_RADIUS_B_F64 * n.y,
            EARTH_RADIUS_A_F64 * EARTH_RADIUS_A_F64 * n.z,
        ) * lam
    };
    let elev = |eye: DVec3, up: DVec3, p: DVec3| -> f64 {
        let v = p - eye;
        let vert = v.dot(up);
        vert.atan2((v - up * vert).length())
    };

    let eye = at(11.0, lat, eye_m);
    let up = normal(eye);
    let m = (up.x * up.x + up.z * up.z).sqrt();
    let east = DVec3::new(up.z / m, 0.0, -up.x / m);
    let north = up.cross(east).normalize();
    // The camera ground position **the engine derives from `up`**, not the one the eye was
    // built from: `begin` inverts the normal, and the two are not the same point once the
    // camera has altitude (see `d3_ridge_safety_never_falls_below_the_constant_it_replaced`,
    // which measures exactly that gap).
    let cam_lon = (-up.z).atan2(up.x).to_degrees();
    let cam_lat = ((EARTH_RADIUS_B_F64 / EARTH_RADIUS_A_F64) * up.y)
        .atan2(m)
        .to_degrees();

    let gamma = s_m / R_M;
    let (sin_g, cos_g) = gamma.sin_cos();
    let b = b_deg.to_radians();
    let dir = east * b.sin() + north * b.cos();
    // `finish`'s wall: the eye's normal rotated by `γ` toward the sector centre.
    let n = (up * cos_g + dir * sin_g).normalize();
    let base = foot(n);

    // The reference: the same `γ` read back through `extent_of`'s equirectangular metric,
    // on the bearing the **ray** may actually have.
    let br = (b_deg + off_deg).to_radians();
    let lat1 = cam_lat + (gamma * br.cos()).to_degrees();
    let cos_m = (0.5 * (lat1 + cam_lat)).to_radians().cos().max(1.0e-9);
    let lon1 = cam_lon + (gamma * br.sin()).to_degrees() / cos_m;
    let e_ref = elev(eye, up, at(lon1, lat1, floor_m));

    let denom_m = (eye_m - floor_m).abs() + s_m * s_m / R_M;
    let safety_m = ridge_safety_m(s_m, eye_m - floor_m);
    let drop_to = |d: f64| elev(eye, up, base + n * ((floor_m - d) * 1.0e-6));

    if drop_to(0.0) <= e_ref {
        return PlacementNeed {
            need_m: 0.0,
            denom_m,
            safety_m,
        };
    }
    let (mut lo, mut hi) = (0.0_f64, 1.0_f64);
    while drop_to(hi) > e_ref && hi < 1.0e9 {
        lo = hi;
        hi *= 4.0;
    }
    for _ in 0..50 {
        let mid = 0.5 * (lo + hi);
        if drop_to(mid) > e_ref {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    PlacementNeed {
        need_m: hi,
        denom_m,
        safety_m,
    }
}

/// The box both safety sweeps walk: latitudes, eye altitudes **at or under the shipped
/// altitude gate**, eye altitudes above it, cell floors, and ranges in kilometres.
///
/// 85° is the Web-Mercator limit, so it is the whole of the latitude range that has tiles
/// at all. The floor list spans far more than the DEM's own range on purpose: a node on an
/// **inherited** interval carries D1's level margin — 20 km at z0–z4, compounding down a
/// cold chain — so a cell floor tens of kilometres under the ellipsoid is something
/// `finish` really is handed. The ranges span `MIN_RANGE_M` to the config's
/// `max_range_m`, which is the whole of what `ring_far` can hold.
#[allow(clippy::type_complexity)]
fn safety_box() -> (
    &'static [f64],
    &'static [f64],
    &'static [f64],
    &'static [f64],
    &'static [f64],
) {
    const LATS: [f64; 11] = [
        -85.0, -70.0, -47.4, -30.0, -10.0, 0.0, 15.0, 45.0, 60.0, 75.0, 85.0,
    ];
    // The top of this list is `TerrainOcclusionConfig::default().max_camera_altitude_m`.
    const EYES_IN_GATE: [f64; 5] = [2.0, 200.0, 1_500.0, 6_000.0, 12_000.0];
    // What `d3_reduces_tiles_where_it_matters` and `d3_altitude_gate_is_where_the_benefit_stops`
    // open the gate to when they measure what the gate gives up.
    const EYES_ABOVE_GATE: [f64; 2] = [20_000.0, 40_000.0];
    const FLOORS: [f64; 7] = [-60_000.0, -6_000.0, -400.0, 0.0, 900.0, 8_000.0, 20_000.0];
    const RANGES_KM: [f64; 9] = [0.5, 1.0, 2.0, 4.0, 8.0, 16.0, 32.0, 64.0, 120.0];
    (&LATS, &EYES_IN_GATE, &EYES_ABOVE_GATE, &FLOORS, &RANGES_KM)
}

/// **Where [`ridge_safety_m`]'s law comes from, measured rather than asserted.**
///
/// [`placement_need`] states the inequality and builds both sides. This walks it over
/// latitude × eye altitude × cell floor × range × bearing **on the sector centre**, which
/// is the pure placement error and exactly what the safety distance is for; the bearing
/// spread is a different error of a different shape and
/// [`d3_ridge_safety_never_falls_below_the_constant_it_replaced`] is where it is measured.
///
/// # Why it prints a ratio
///
/// `RIDGE_SAFETY_RATE`'s doc comment derives `Δh ≲ κ·(|Δalt| + s²/R)` and claims
/// `κ ≈ e²/2`. A ratio that stays flat across the parameter box is the evidence for that
/// **shape**; a single worst-case number would not distinguish it from any other curve
/// through the same point. The factor on the front is then a reserve on a measured
/// constant rather than a guess at an unmeasured one — and it is a factor on a
/// measurement, not a proof, which is the standing `RIDGE_SAFETY_RATE`'s own doc comment
/// claims for it.
#[test]
fn d3_ridge_safety_covers_its_placement_error() {
    // Half the squared eccentricity of the engine's ellipsoid — the first-order relative
    // discrepancy between the parametric and the geodetic latitude metric, and what the
    // ratio below should land on if the derivation is the right one.
    let e2_over_2 = 0.5
        * (1.0
            - (EARTH_RADIUS_B_F64 * EARTH_RADIUS_B_F64)
                / (EARTH_RADIUS_A_F64 * EARTH_RADIUS_A_F64));

    let (lats, eyes_in, eyes_above, floors, ranges_km) = safety_box();
    let eyes: Vec<f64> = eyes_in.iter().chain(eyes_above.iter()).copied().collect();

    let mut worst_ratio = 0.0_f64;
    let mut worst_reserve = f64::INFINITY;
    let mut worst_at = String::new();
    let mut rows: Vec<String> = Vec::new();
    let mut unsound = 0usize;
    let mut cases = 0usize;

    for &km in ranges_km {
        let s_m = km * 1_000.0;
        let (mut row_ratio, mut row_need, mut row_denom) = (0.0_f64, 0.0_f64, 0.0_f64);
        let mut row_reserve = f64::INFINITY;
        for &lat in lats {
            for &eye_m in &eyes {
                for &floor_m in floors {
                    let mut b_deg = 0.0_f64;
                    while b_deg < 360.0 {
                        let n = placement_need(lat, eye_m, floor_m, s_m, b_deg, 0.0);
                        b_deg += 5.0;
                        cases += 1;
                        // **The claim**, directly: the wall, lowered by the shipped law, is
                        // not above the ground its cell actually stands for.
                        if n.need_m > n.safety_m {
                            unsound += 1;
                        }
                        if n.need_m <= 0.0 {
                            continue;
                        }
                        let ratio = n.need_m / n.denom_m;
                        worst_ratio = worst_ratio.max(ratio);
                        if ratio > row_ratio {
                            row_ratio = ratio;
                            row_need = n.need_m;
                            row_denom = n.denom_m;
                        }
                        let reserve = n.safety_m / n.need_m;
                        row_reserve = row_reserve.min(reserve);
                        if reserve < worst_reserve {
                            worst_reserve = reserve;
                            worst_at = format!(
                                "{km} km, lat {lat}, eye {eye_m} m, floor {floor_m} m, \
                                 bearing {:.0}",
                                b_deg - 5.0
                            );
                        }
                    }
                }
            }
        }
        rows.push(format!(
            "    {km:>7} {row_need:>10.3} {row_denom:>11.0} {row_ratio:>10.6} \
             {:>10.2} {row_reserve:>9.1}",
            ridge_safety_m(s_m, 0.0)
        ));
    }

    // **And the same box again, off the grid.** A grid over a smooth five-parameter box
    // can sit in the troughs of whatever it is measuring; a pseudo-random pass cannot sit
    // anywhere in particular. Same seed every run, so a failure is reproducible.
    let mut rng = Lcg::new(0x5EED_1234);
    let mut rand_reserve = f64::INFINITY;
    let mut rand_ratio = 0.0_f64;
    let mut rand_at = String::new();
    let max_range_m = TerrainOcclusionConfig::default().max_range_m as f64;
    for _ in 0..200_000 {
        let lat = rng.range(-85.0, 85.0);
        let eye_m = rng.range(2.0, 40_000.0);
        let floor_m = rng.range(-60_000.0, 20_000.0);
        let s_m = MIN_RANGE_M * (max_range_m / MIN_RANGE_M).powf(rng.next_f64());
        let b_deg = rng.range(0.0, 360.0);
        let n = placement_need(lat, eye_m, floor_m, s_m, b_deg, 0.0);
        cases += 1;
        if n.need_m > n.safety_m {
            unsound += 1;
        }
        if n.need_m <= 0.0 {
            continue;
        }
        rand_ratio = rand_ratio.max(n.need_m / n.denom_m);
        let reserve = n.safety_m / n.need_m;
        if reserve < rand_reserve {
            rand_reserve = reserve;
            rand_at = format!(
                "{:.1} km, lat {lat:.1}, eye {eye_m:.0} m, floor {floor_m:.0} m, \
                 bearing {b_deg:.1}",
                s_m / 1_000.0
            );
        }
    }

    println!("  [D3 safety] the placement error the safety distance covers — {cases} cases");
    println!("    e²/2 = {e2_over_2:.6}  —  the first-order prediction for the ratio");
    println!(
        "    {:>7} {:>10} {:>11} {:>10} {:>10} {:>9}",
        "km", "need (m)", "|dalt|+s²/R", "ratio", "safety@0", "reserve"
    );
    for r in &rows {
        println!("{r}");
    }
    println!(
        "    worst ratio {worst_ratio:.6} = {:.2}× e²/2;  worst reserve {worst_reserve:.2}× at \
         {worst_at}",
        worst_ratio / e2_over_2
    );
    println!(
        "    off-grid pass: worst ratio {rand_ratio:.6} = {:.2}× e²/2, worst reserve \
         {rand_reserve:.2}× at {rand_at}",
        rand_ratio / e2_over_2
    );

    assert_eq!(
        unsound, 0,
        "the shipped safety distance leaves the march's wall above the ground its cell \
         stands for in {unsound} of the swept cases"
    );
    let reserve = worst_reserve.min(rand_reserve);
    assert!(
        reserve >= REQUIRED_RESERVE,
        "the safety distance's reserve has fallen to {reserve:.2}× (at {worst_at} / \
         {rand_at}); RIDGE_SAFETY_RATE is no longer clear of what it covers"
    );
    // And the shape: if this stops being e²/2 the derivation in `RIDGE_SAFETY_RATE`'s doc
    // comment has stopped describing the code.
    let ratio = worst_ratio.max(rand_ratio);
    assert!(
        ratio < 2.0 * e2_over_2,
        "the measured ratio {ratio:.6} is no longer within a factor of two of e²/2 \
         = {e2_over_2:.6}; the law's shape, not just its constant, needs re-deriving"
    );
}

/// **The acceptance test for a *relaxation*: the new law is short nowhere the old constant
/// held.**
///
/// [`d3_ridge_safety_covers_its_placement_error`] proves the law covers the error it was
/// derived for. That is not the same as proving the *change* is safe, because
/// `RIDGE_SAFETY_M`'s flat 100 m was also, accidentally, paying for a second error nobody
/// had written down — and a range-proportional law that is under 100 m at short range
/// stops paying for it there.
///
/// # The second error, named and measured
///
/// `finish` builds every wall on the sector's **centre** bearing, and the ray it has to
/// hold against may be [`SECTOR_BEARING_SPREAD_DEG`] away. Over that spread the wall and
/// the ground move apart for two reasons, and only the first is small:
///
/// * the ellipsoid's Euler radius varies with azimuth by a fraction of `e²`, which is a
///   quarter of `κ` over half a sector — inside the law's own reserve;
/// * and `TerrainHorizon::begin` takes `up` from `ellipsoid_normal_at`, which is the
///   **confocal** normal, not the geodetic one, so the eye's own up-axis misses the ground
///   point `cam_lon`/`cam_lat` names. The table this test prints measures that miss: it is
///   7 mm at eye height and **40 m at the 12 km altitude gate**. A ground point one
///   half-sector round from a wall is that much further from, or nearer to, the eye's axis
///   — and under a high camera every metre of that is `|Δalt|/s` metres of wall.
///
/// This is **pre-existing, and not what the safety distance is for**: its shape is
/// `|Δalt|/s`, which grows as the range *falls*, where `ridge_safety_m`'s two terms both
/// shrink. No affordable constant covers it — at 40 km the worst case asks for 6 km of
/// wall. What this test therefore asserts is the pair of things that are actually true and
/// actually load-bearing:
///
/// 1. **inside the shipped altitude gate the shipped law covers it anyway**, over the whole
///    box, bearing spread included; and
/// 2. **there is no case, anywhere in the box, where `ridge_safety_m` is short and the flat
///    100 m was not** — so the relaxation strictly shrinks the set of geometries where the
///    grid's wall can stand over the ground, rather than trading one hole for another.
///
/// The above-gate numbers are printed rather than asserted, because they are a property of
/// the 24-sector grid and of `ellipsoid_normal_at`, not of this constant, and because the
/// only configuration that reaches them is a harness that has deliberately opened the gate.
#[test]
fn d3_ridge_safety_never_falls_below_the_constant_it_replaced() {
    let (lats, eyes_in, eyes_above, floors, ranges_km) = safety_box();
    let offsets_deg = {
        let s = SECTOR_BEARING_SPREAD_DEG;
        [0.0, 0.25 * s, 0.5 * s, 0.75 * s, s, -0.5 * s, -s]
    };

    // The mechanism, as a measurement: how far the eye's own up-axis misses the ground
    // point `begin` derives from it. `ellipsoid_normal_at` is the confocal normal, so the
    // eye is *not* on the ellipsoid normal through `foot(up)`, and the gap grows with the
    // square of the altitude.
    println!("  [D3 safety] the eye's up-axis against the ground point `begin` derives");
    println!("    {:>10} {:>16}", "eye (m)", "off-axis (m)");
    for &h in [2.0_f64, 200.0, 1_500.0, 6_000.0, 12_000.0, 40_000.0].iter() {
        // Both built the way the engine builds them, in this file's own arithmetic.
        let phi = 45f64.to_radians();
        let s = DVec3::new(
            EARTH_RADIUS_A_F64 * phi.cos(),
            EARTH_RADIUS_B_F64 * phi.sin(),
            0.0,
        );
        let n0 = DVec3::new(
            s.x / (EARTH_RADIUS_A_F64 * EARTH_RADIUS_A_F64),
            s.y / (EARTH_RADIUS_B_F64 * EARTH_RADIUS_B_F64),
            0.0,
        )
        .normalize();
        let eye = s + n0 * (h * 1.0e-6);
        let up = DVec3::new(
            eye.x / (EARTH_RADIUS_A_F64 * EARTH_RADIUS_A_F64),
            eye.y / (EARTH_RADIUS_B_F64 * EARTH_RADIUS_B_F64),
            0.0,
        )
        .normalize();
        let lam = 1.0
            / (EARTH_RADIUS_A_F64 * EARTH_RADIUS_A_F64 * up.x * up.x
                + EARTH_RADIUS_B_F64 * EARTH_RADIUS_B_F64 * up.y * up.y)
                .sqrt();
        let f = DVec3::new(
            EARTH_RADIUS_A_F64 * EARTH_RADIUS_A_F64 * up.x,
            EARTH_RADIUS_B_F64 * EARTH_RADIUS_B_F64 * up.y,
            0.0,
        ) * lam;
        let d = eye - f;
        let off = (d - up * d.dot(up)).length() * 1.0e6;
        println!("    {h:>10.0} {off:>16.3}");
    }

    let mut regressions = 0usize;
    let mut worst_regression = String::new();
    let (mut short_in_gate, mut short_above) = (0usize, 0usize);
    let (mut short_old_in_gate, mut short_old_above) = (0usize, 0usize);
    let mut worst_in_gate = f64::INFINITY;
    let mut worst_in_gate_at = String::new();
    let mut worst_above = f64::INFINITY;
    let mut worst_above_at = String::new();
    let mut worst_old = f64::INFINITY;
    let mut cases = 0usize;

    for &km in ranges_km {
        let s_m = km * 1_000.0;
        for &lat in lats {
            for (in_gate, eye_list) in [(true, eyes_in), (false, eyes_above)] {
                for &eye_m in eye_list {
                    for &floor_m in floors {
                        let mut b_deg = 0.0_f64;
                        while b_deg < 360.0 {
                            for &off in offsets_deg.iter() {
                                let n = placement_need(lat, eye_m, floor_m, s_m, b_deg, off);
                                cases += 1;
                                if n.need_m <= 0.0 {
                                    continue;
                                }
                                let what = || {
                                    format!(
                                        "{km} km, lat {lat}, eye {eye_m} m, floor {floor_m} m, \
                                         bearing {b_deg:.0} + {off:.2}, need {:.1} m against \
                                         {:.1} m",
                                        n.need_m, n.safety_m
                                    )
                                };
                                // The one thing a relaxation must not do.
                                if n.need_m > n.safety_m && n.need_m <= REPLACED_CONSTANT_M {
                                    regressions += 1;
                                    if worst_regression.is_empty() {
                                        worst_regression = what();
                                    }
                                }
                                if n.need_m > n.safety_m {
                                    if in_gate {
                                        short_in_gate += 1;
                                    } else {
                                        short_above += 1;
                                    }
                                }
                                if n.need_m > REPLACED_CONSTANT_M {
                                    if in_gate {
                                        short_old_in_gate += 1;
                                    } else {
                                        short_old_above += 1;
                                    }
                                }
                                worst_old = worst_old.min(REPLACED_CONSTANT_M / n.need_m);
                                let reserve = n.safety_m / n.need_m;
                                if in_gate {
                                    if reserve < worst_in_gate {
                                        worst_in_gate = reserve;
                                        worst_in_gate_at = what();
                                    }
                                } else if reserve < worst_above {
                                    worst_above = reserve;
                                    worst_above_at = what();
                                }
                            }
                            b_deg += 5.0;
                        }
                    }
                }
            }
        }
    }

    println!(
        "    {cases} cases, sector centre +/- {SECTOR_BEARING_SPREAD_DEG:.2} deg \
         (half a sector is {:.2})",
        180.0 / AZIMUTH_SECTORS as f64
    );
    println!(
        "    under the {} m altitude gate: ridge_safety_m short in {short_in_gate}, the flat \
         {REPLACED_CONSTANT_M:.0} m short in {short_old_in_gate}; worst reserve \
         {worst_in_gate:.2}× at {worst_in_gate_at}",
        TerrainOcclusionConfig::default().max_camera_altitude_m
    );
    println!(
        "    above it (harness-only): ridge_safety_m short in {short_above}, the flat \
         {REPLACED_CONSTANT_M:.0} m short in {short_old_above}; worst reserve \
         {worst_above:.2}× at {worst_above_at}"
    );
    println!(
        "    the flat {REPLACED_CONSTANT_M:.0} m's own worst reserve over the same box: \
         {worst_old:.3}×"
    );

    assert_eq!(
        regressions, 0,
        "in {regressions} cases `ridge_safety_m` is short where the flat \
         {REPLACED_CONSTANT_M:.0} m it replaced was not — the relaxation has opened a hole \
         rather than narrowed one. First: {worst_regression}"
    );
    assert_eq!(
        short_in_gate, 0,
        "under the shipped altitude gate the wall stands over the ground its cell stands \
         for in {short_in_gate} cases once the ray is allowed its half-sector of bearing. \
         Worst: {worst_in_gate_at}"
    );
}

// ── D3's acceptance: FN = 0 ──────────────────────────────────────────────────────

/// **The acceptance criterion for D3**: nothing D3 culls has a mesh vertex that is on
/// screen, off the limb *and* not behind the drawn surface.
///
/// This is the test the whole feature exists to pass, and the failure mode it guards is
/// the one `docs/terrain-plan.md` §3.3 warns about in bold: taking `h_max` for the
/// occluder instead of `h_min` over-occludes, and by I-7 that deletes an entire subtree.
/// Flipping [`SurfaceModel::occluder_floor`](cesium_engine::globe::quadtree::SurfaceModel)
/// to `hi` was tried by hand against this test and it goes red immediately.
#[test]
fn d3_never_hides_a_visible_vertex() {
    let mut poses: Vec<ViewParams> = reduction_poses().into_iter().map(|(_, p)| p).collect();
    // A few more angles and stand-off distances, so the sweep is not four hand-aimed
    // shots: the same ridge from further back, from beside the col, and looking down.
    for &lat_off in &[0.06, 0.20, 0.45] {
        for &pitch in &[78.0, 86.0, 92.0] {
            poses.push(pose("d3_fan", LAT_RIDGE - lat_off, 11.0, 2_500.0, pitch));
            poses.push(pose("d3_col", LAT_RIDGE - lat_off, LON_GAP, 2_500.0, pitch));
        }
    }

    let cfg = TerrainOcclusionConfig::default();
    let mut culled_total = 0usize;
    let mut leaves_total = 0usize;
    let mut false_negatives = 0usize;
    let mut marginal = 0usize;
    let mut examples: Vec<String> = Vec::new();

    for p in &poses {
        let (frustum, oracle) = frustum_for(p);
        let cam = HorizonCamera::new(frustum.eye);

        // The reference tree is what the engine draws without D3 — the surface the
        // oracle marches against — and the tree under test is the same thing with the
        // stage switched on.
        let (reference, _, _) = settled_tree(p, &frustum, None, RIDGE_M);
        let (qt, mut heights, _world) = settled_tree(p, &frustum, Some(cfg), RIDGE_M);

        let mut nodes = Vec::new();
        let mut leaves = 0usize;
        for root in qt.roots.iter() {
            collect(root, &mut nodes, &mut leaves);
        }
        leaves_total += leaves;

        for (id, was_culled) in nodes {
            if !was_culled {
                continue;
            }
            culled_total += 1;
            let Ok(patch) = HeightPatch::sample(&mut heights, id, SEGMENTS, 1.0) else {
                continue;
            };
            let mesh = TileMesh::generate_on::<Heightfield>(&id, SEGMENTS, &patch);
            let mut visible = 0usize;
            for q in mesh_points(&mesh) {
                match classify(&oracle, &cam, &reference.roots, frustum.eye, q) {
                    Verdict::Visible => visible += 1,
                    Verdict::Marginal => marginal += 1,
                    Verdict::Hidden => {}
                }
            }
            if visible > 0 {
                false_negatives += 1;
                if examples.len() < 20 {
                    examples.push(format!(
                        "{} lat={:.3} lon={:.3} alt={:.0}m pitch={:.0}: z{} {}/{} culled with \
                         {visible} vertices on screen, off the limb and in front of the terrain",
                        p.sweep, p.lat_deg, p.lon_deg, p.alt_m, p.pitch_deg, id.z, id.x, id.y
                    ));
                }
            }
        }
    }

    println!("  [D3 FN] {} poses", poses.len());
    println!("    {culled_total} culled nodes scored, {leaves_total} visible leaves");
    println!("    false negatives: {false_negatives}   (marginal vertices: {marginal})");
    assert!(
        culled_total > 500,
        "the sweep did not exercise the culler: only {culled_total} culled nodes"
    );
    assert_eq!(
        false_negatives,
        0,
        "D3 hid geometry that is genuinely visible:\n  {}",
        examples.join("\n  ")
    );
}

/// §3.3's **"lateral gaps are handled"** claim, as a test rather than a sentence.
///
/// The ridge has one col in it. A camera 11 km south of the col, looking north, must find
/// *no* guaranteed ridge on the bearing that runs through the gap, and a real one on a
/// bearing 45° off it where the crest is unbroken — at the same range, from the same
/// camera, in the same march.
///
/// It reads the march directly ([`TerrainHorizon::ridge_elevation`]) rather than counting
/// tiles, and that is deliberate: the claim is about the occluder grid, and at tile
/// granularity it is confounded twice over — a node's box spans a *range* of bearings far
/// wider than the col, and the two trees refine independently, so a leaf missing from one
/// of them may only have been replaced by its parent. Measured both ways; the tile-count
/// version answers a different question and answers it noisily.
#[test]
fn d3_does_not_cull_through_the_col() {
    let p = pose("d3_col", LAT_RIDGE - 0.10, LON_GAP, 1_200.0, 88.0);
    let (frustum, _) = frustum_for(&p);
    let cfg = TerrainOcclusionConfig::default();
    let (qt, _, _) = settled_tree(&p, &frustum, Some(cfg), RIDGE_M);

    let horizon = qt
        .terrain_horizon()
        .expect("the march was built for this pose");
    assert!(
        horizon.is_active(),
        "the altitude gate shut the march off at 1 200 m"
    );

    // 18 km out: past the crest, which sits 11 km due north and 13 km along the 30° bearing.
    const RANGE_M: f64 = 18_000.0;
    let through_col = horizon.ridge_elevation(0.0, RANGE_M);
    let over_ridge_e = horizon.ridge_elevation(45f64.to_radians(), RANGE_M);
    let over_ridge_w = horizon.ridge_elevation((-45f64).to_radians(), RANGE_M);

    println!(
        "  [D3 col] guaranteed ridge at {:.0} km — through the col {:.3} deg, \
         45 deg east {:.3} deg, 45 deg west {:.3} deg",
        RANGE_M / 1000.0,
        (through_col as f64).to_degrees(),
        (over_ridge_e as f64).to_degrees(),
        (over_ridge_w as f64).to_degrees(),
    );

    assert!(
        over_ridge_e > through_col && over_ridge_w > through_col,
        "the col's bearing reports as much guaranteed ridge as the unbroken crest either \
         side of it ({through_col} vs {over_ridge_e} / {over_ridge_w}) — a gap must pull \
         its sector's floor down to the valley, and an occluder taken over too small a \
         footprint, or with `h_max` instead of `h_min`, is what would stop it"
    );
}

/// The reduction table `docs/terrain-plan.md` §7 quotes, and the counter-check at cruise.
///
/// Measurement, not a gate — but it asserts the two things that would mean D3 is not
/// wired up at all: that it removes something in a valley, and that the cruise pose is
/// not where the benefit is.
#[test]
fn d3_reduces_tiles_where_it_matters() {
    let cfg = TerrainOcclusionConfig {
        // Deliberately above the cruise pose so the table can show what the gate is
        // *giving up*, rather than showing a zero the gate produced.
        max_camera_altitude_m: 40_000.0,
        ..TerrainOcclusionConfig::default()
    };

    println!("  [D3 reduction] ridge world, due north into the crest");
    println!(
        "    {:<14} {:>8} {:>10} {:>10} {:>9} {:>12}",
        "pose", "alt (m)", "D1+D2", "with D3", "delta", "flat world"
    );
    let mut valley_delta = 0.0_f64;
    let mut cruise_delta = 0.0_f64;

    for (name, p) in reduction_poses() {
        let (frustum, _) = frustum_for(&p);
        let (reference, _, _) = settled_tree(&p, &frustum, None, RIDGE_M);
        let (qt, _, _) = settled_tree(&p, &frustum, Some(cfg), RIDGE_M);
        let before = reference.get_visible_tiles().len();
        let after = qt.get_visible_tiles().len();
        let delta = 100.0 * (after as f64 / before.max(1) as f64 - 1.0);
        // The control: the same pose over the same world with the ridge flattened out.
        // Whatever D3 removes there is the *curvature* horizon, not a mountain.
        let (fref, _, _) = settled_tree(&p, &frustum, None, 0.0);
        let (fqt, _, _) = settled_tree(&p, &frustum, Some(cfg), 0.0);
        let fb = fref.get_visible_tiles().len();
        let fa = fqt.get_visible_tiles().len();
        let fdelta = 100.0 * (fa as f64 / fb.max(1) as f64 - 1.0);
        println!(
            "    {name:<14} {:>8.0} {before:>10} {after:>10} {delta:>8.1} % {fdelta:>10.1} %",
            p.alt_m
        );
        if name == "valley" {
            valley_delta = delta;
        }
        if name == "cruise_11km" {
            cruise_delta = delta;
        }
    }

    assert!(
        valley_delta < -5.0,
        "D3 removed only {valley_delta:.1} % of the valley pose's tiles — it is not doing \
         anything, which for the deliverable this whole plan is named after is a failure"
    );
    assert!(
        cruise_delta > valley_delta,
        "the cruise pose lost more than the valley pose ({cruise_delta:.1} % vs \
         {valley_delta:.1} %) — that inverts the whole argument for the altitude gate"
    );
}

/// **The measurement behind [`TerrainOcclusionConfig::max_camera_altitude_m`].**
///
/// `docs/terrain-plan.md` §3.3 says the threshold must come from a measurement rather than
/// a feeling, so this is the measurement: the same stand-off geometry walked up in
/// altitude, with the gate held wide open so what is read is the *benefit* and not the
/// gate's own effect.
///
/// It is run at **two ridge heights**, 2.8 km (Alpine) and 8.0 km (Himalayan), so that the
/// threshold is not read off one crest. What the two columns show is that the reduction
/// saturates rather than scaling with the crest: past ~3 km of camera altitude the tiles
/// in the ridge's shadow are the handful the LOD still keeps out there, and a taller crest
/// lengthens the shadow into ground that has no tiles left in it. So the curve to read a
/// threshold off is the *altitude* one, and it has three regimes — a large benefit below
/// ~2 km, a ~4 % plateau out to 12 km, and exactly zero from 20 km up.
#[test]
fn d3_altitude_gate_is_where_the_benefit_stops() {
    let cfg = TerrainOcclusionConfig {
        max_camera_altitude_m: 1.0e9,
        ..TerrainOcclusionConfig::default()
    };

    println!("  [D3 altitude gate] 0.20 deg south of the crest, pitched into the ridge");
    println!(
        "    {:>9} {:>22} {:>22}",
        "alt (m)", "2.8 km crest", "8.0 km crest"
    );
    println!(
        "    {:>9} {:>7} {:>7} {:>6} {:>7} {:>7} {:>6}",
        "", "D1+D2", "with D3", "delta", "D1+D2", "with D3", "delta"
    );

    let mut low_crest: Vec<(f64, f64)> = Vec::new();
    let mut high_crest: Vec<(f64, f64)> = Vec::new();
    for &alt in &[
        800.0, 1_500.0, 3_000.0, 5_000.0, 8_000.0, 12_000.0, 20_000.0, 40_000.0,
    ] {
        let p = pose("d3_alt", LAT_RIDGE - 0.20, 11.0, alt, 88.0);
        let (frustum, _) = frustum_for(&p);
        let mut cols = Vec::new();
        for ridge in [RIDGE_M, 8_000.0] {
            let (reference, _, _) = settled_tree(&p, &frustum, None, ridge);
            let (qt, _, _) = settled_tree(&p, &frustum, Some(cfg), ridge);
            let before = reference.get_visible_tiles().len();
            let after = qt.get_visible_tiles().len();
            cols.push((
                before,
                after,
                100.0 * (after as f64 / before.max(1) as f64 - 1.0),
            ));
        }
        println!(
            "    {alt:>9.0} {:>7} {:>7} {:>5.1}% {:>7} {:>7} {:>5.1}%",
            cols[0].0, cols[0].1, cols[0].2, cols[1].0, cols[1].1, cols[1].2
        );
        low_crest.push((alt, cols[0].2));
        high_crest.push((alt, cols[1].2));
    }

    let shipped = TerrainOcclusionConfig::default().max_camera_altitude_m as f64;
    println!("    shipped gate: {shipped:.0} m");

    let low_bottom = low_crest.first().expect("rows").1;
    let low_top = low_crest.last().expect("rows").1;
    assert!(
        low_bottom < low_top,
        "the reduction did not shrink with altitude ({low_bottom:.1} % at the bottom, \
         {low_top:.1} % at the top) — the altitude gate's whole premise is that it does"
    );
    // The gate must not shut the stage off while the measurement still shows a reduction,
    // on either crest. Rows at or below the gate are what it keeps; rows above it are what
    // it gives up, and every one of those must read zero.
    for rows in [&low_crest, &high_crest] {
        for (alt, delta) in rows.iter() {
            if *alt > shipped {
                assert!(
                    *delta >= 0.0,
                    "the gate gives up a real {delta:.1} % reduction at {alt:.0} m — \
                     `max_camera_altitude_m` is set below where the benefit actually stops"
                );
            }
        }
    }
}

/// What the march and the stage actually cost, measured rather than argued.
///
/// `#[ignore]`d for the same reason `culling::bench_update` is: it is a measurement, not a
/// gate, and on a loaded machine its wall clock is noise. Run it with
///
/// ```text
/// cargo test --release --lib terrain::test_terrain_occlusion::bench -- --ignored --nocapture
/// ```
///
/// Two numbers matter and they are charged to different places: `refresh_terrain_horizon`
/// is **once per frame** (a walk of the tree plus `AZIMUTH_SECTORS × RANGE_RINGS` elevation
/// angles), while `Stage::TerrainOcclusion` is **per node** and shows up inside `update`.
#[test]
#[ignore = "measurement, not a gate: prints the D3 march and stage cost"]
fn bench_terrain_occlusion_cost() {
    use std::time::Instant;

    let cfg = TerrainOcclusionConfig {
        max_camera_altitude_m: 1.0e9,
        ..TerrainOcclusionConfig::default()
    };
    println!("  [D3 cost] {AZIMUTH_SECTORS} sectors x {RANGE_RINGS} rings");
    for (name, p) in reduction_poses() {
        let (frustum, _) = frustum_for(&p);
        let (mut qt, _, _) = settled_tree(&p, &frustum, Some(cfg), RIDGE_M);
        let cam_alt = p.alt_m * 1.0e-6;

        const REPS: u32 = 200;
        let t0 = Instant::now();
        for _ in 0..REPS {
            qt.refresh_terrain_horizon(&frustum, cam_alt, cam_alt - BASE_M * 1.0e-6, &cfg);
        }
        let march_us = t0.elapsed().as_secs_f64() * 1.0e6 / REPS as f64;

        let t1 = Instant::now();
        for _ in 0..REPS {
            qt.update(&frustum);
        }
        let with_us = t1.elapsed().as_secs_f64() * 1.0e6 / REPS as f64;

        qt.clear_terrain_horizon();
        qt.pipeline = CullPipeline::DEFAULT;
        let t2 = Instant::now();
        for _ in 0..REPS {
            qt.update(&frustum);
        }
        let without_us = t2.elapsed().as_secs_f64() * 1.0e6 / REPS as f64;

        println!(
            "    {name:<14} march {march_us:7.1} us   update D1+D2 {without_us:7.1} us   \
             update +D3 {with_us:7.1} us"
        );
    }
}

// ── real terrain, and what a finer occludee granularity would buy on it ──────────

// Everything above this line is the synthetic ridge world. What follows measures the same
// stage on the **real** DEM at the poses `rendering::terrain_capture` photographs, because
// `docs/terrain-plan.md` §7b's finding is that the two disagree — and the disagreement,
// not the agreement, is what decides whether this stage is worth its march.

/// Where the terrarium PNGs are cached between runs. Set `CESIUM_HEIGHT_CACHE` to keep
/// them; otherwise the system temp dir, which is still shared between runs on one
/// machine.
pub(crate) fn height_cache_dir() -> std::path::PathBuf {
    let dir = std::env::var_os("CESIUM_HEIGHT_CACHE")
        .map(std::path::PathBuf::from)
        .unwrap_or_else(|| std::env::temp_dir().join("cesium_terrarium_cache"));
    let _ = std::fs::create_dir_all(&dir);
    dir
}

fn terrarium_path(dir: &std::path::Path, id: TileId) -> std::path::PathBuf {
    dir.join(format!("{}_{}_{}.png", id.z, id.x, id.y))
}

/// Downloads whatever is not on disk yet, in parallel, with `curl`.
///
/// `curl` rather than a Rust client on purpose: the root crate has no HTTP dependency
/// and a measurement harness is not a reason to add one to the shipped dependency graph.
pub(crate) fn fetch_missing(dir: &std::path::Path, ids: &[TileId]) {
    let missing: Vec<TileId> = ids
        .iter()
        .copied()
        .filter(|id| !terrarium_path(dir, *id).exists())
        .collect();
    if missing.is_empty() {
        return;
    }
    for chunk in missing.chunks(48) {
        let mut cmd = std::process::Command::new("curl");
        cmd.arg("-sS")
            .arg("--parallel")
            .arg("--parallel-max")
            .arg("12");
        for id in chunk {
            cmd.arg("-o").arg(terrarium_path(dir, *id)).arg(format!(
                "https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{}/{}/{}.png",
                id.z, id.x, id.y
            ));
        }
        let _ = cmd.status();
    }
}

/// The real DEM, tile by tile, off disk.
pub(crate) struct RealWorld {
    pub(crate) dir: std::path::PathBuf,
    tiles: HashMap<TileId, Arc<HeightTile>>,
}

impl RealWorld {
    pub(crate) fn new() -> Self {
        Self {
            dir: height_cache_dir(),
            tiles: HashMap::new(),
        }
    }

    pub(crate) fn tile(&mut self, id: TileId) -> Arc<HeightTile> {
        if let Some(t) = self.tiles.get(&id) {
            return t.clone();
        }
        fetch_missing(&self.dir, &[id]);
        let path = terrarium_path(&self.dir, id);
        let decoded = std::fs::read(&path).ok().and_then(|bytes| {
            let img = image::load_from_memory(&bytes).ok()?;
            let rgba = img.to_rgba8();
            cesium_engine::globe::terrain::height_tile::decode_terrarium(
                rgba.width(),
                rgba.height(),
                rgba.as_raw(),
                OceanPolicy::ClampToZero,
            )
            .ok()
        });
        // A tile the source does not serve is flat zero, which is the same thing the
        // engine's own fetcher ends up with and is a *low* floor, i.e. it occludes less.
        let t = Arc::new(decoded.unwrap_or_else(HeightTile::flat_zero));
        self.tiles.insert(id, t.clone());
        t
    }
}

pub(crate) fn real_config() -> TileEngineConfig {
    TileEngineConfig {
        mesh_segments: SEGMENTS,
        target_texel_ratio: 1.0,
        terrain: TerrainConfig {
            enabled: true,
            exaggeration: 1.0,
            // Production's default, so the floors are the ones the engine would use.
            ocean: OceanPolicy::ClampToZero,
            height_cache_budget_bytes: 1024 * 1024 * 1024,
            ..TerrainConfig::default()
        },
        ..TileEngineConfig::default()
    }
}

/// Every height source the tree currently wants, so one `curl` invocation can fetch
/// them all instead of one round trip per node.
pub(crate) fn collect_sources(
    node: &QuadtreeNode<Heightfield>,
    heights: &HeightTileManager,
    out: &mut Vec<TileId>,
) {
    let src = heights.source_tile_for(node.id);
    if heights.status_of(node.id) != PatchStatus::Ready {
        out.push(src);
    }
    if let Some(children) = &node.children {
        for c in children.iter() {
            collect_sources(c, heights, out);
        }
    }
}

pub(crate) fn fill_cache_real(
    node: &QuadtreeNode<Heightfield>,
    heights: &mut HeightTileManager,
    w: &mut RealWorld,
) {
    let src = heights.source_tile_for(node.id);
    if heights.status_of(node.id) != PatchStatus::Ready {
        let t = w.tile(src);
        heights.insert_ready(src, t);
    }
    if let Some(children) = &node.children {
        for c in children.iter() {
            fill_cache_real(c, heights, w);
        }
    }
}

/// The capture poses of `rendering::terrain_capture`, as `ViewParams`.
///
/// That harness takes a pitch *below the horizontal*; `ViewParams::pitch_deg` measures
/// from nadir, so the two differ by 90°. Everything else — longitude, latitude, altitude
/// and the due-north bearing — is copied from `terrain_capture::poses` verbatim, which is
/// what makes the numbers here comparable to the table in `docs/terrain-plan.md` §7b.
pub(crate) fn real_poses() -> Vec<(&'static str, ViewParams)> {
    let p = |name: &'static str, lon: f64, lat: f64, alt_m: f64, below: f64, yaw: f64| {
        (
            name,
            ViewParams {
                sweep: "real",
                lat_deg: lat,
                lon_deg: lon,
                alt_m,
                pitch_deg: 90.0 - below,
                yaw_deg: yaw,
                roll_deg: 0.0,
                width: 1280,
                height: 720,
                mode: CameraMode::Free,
            },
        )
    };
    vec![
        p("alps_inn_valley", 11.40, 47.26, 900.0, 1.5, 0.0),
        p("alps_low", 10.985, 47.10, 4_500.0, 8.0, 0.0),
        p("alps_zugspitze", 10.985, 46.79, 9_000.0, 12.0, 0.0),
        p("himalaya_everest", 86.925, 27.35, 11_000.0, 11.0, 0.0),
        p("himalaya_limb_400km", 86.925, 23.0, 400_000.0, 18.0, 0.0),
        // Five more low poses, added here rather than to the capture set. §7b's own
        // statement of where D3 should pay is "a valley floor, a plain behind a range,
        // water, a plateau", and one Alpine valley is a thin basis for either verdict.
        //
        // **Every one of these is placed a few hundred metres over ground whose height
        // was looked up first**, because the first attempt at this list was not: three of
        // four poses picked off a map — 11.75 E / 47.28 N "in the Inn valley" (1 998 m,
        // the Tuxer Alpen), 11.00 E / 45.60 N "the Po plain" (499 m, the Lessini hills),
        // 86.83 E / 27.80 N "the Khumbu valley" (5 440 m) — put the camera *inside* a
        // mountain, where `TerrainHorizon::finish`'s enclosure guard correctly switches
        // the whole march off. Four poses reporting a flat zero for a reason that has
        // nothing to do with the thing under test is exactly how a negative result gets
        // faked by accident.
        p("po_plain_to_alps", 11.30, 45.15, 300.0, 1.0, 0.0),
        p("terai_to_himalaya", 86.90, 26.90, 600.0, 1.0, 0.0),
        p("rhone_valley", 7.60, 46.30, 900.0, 1.0, 0.0),
        p("salzach_to_alps", 13.04, 47.80, 800.0, 1.0, 180.0),
        p("aosta_valley", 7.32, 45.74, 900.0, 1.0, 0.0),
    ]
}

/// A settled terrain quadtree over the real DEM at `p`, with D3 on or off.
///
/// `lod_factor` is the capture's, not `QuadtreeManager`'s default 2.0: the tile counts
/// this prints are meant to be read next to `rendering::terrain_capture`'s, and the LOD
/// threshold is what decides how coarse the far field is — which is half of §7b's
/// explanation for why D3 finds nothing out there.
fn settled_real_tree(
    p: &ViewParams,
    frustum: &Frustum,
    occlusion: Option<TerrainOcclusionConfig>,
    world: &mut RealWorld,
) -> (QuadtreeManager<Heightfield>, HeightTileManager) {
    let config = real_config();
    let mut heights = HeightTileManager::new(&config);
    let mut qt = QuadtreeManager::<Heightfield>::for_surface();
    let cam = build_camera(p);
    // Both of these are the capture's, not the harness defaults: `lod_factor` decides how
    // coarse the far field is (half of §7b's explanation for why D3 finds nothing out
    // there) and `max_zoom` decides how fine the near field is. `QuadtreeManager` defaults
    // to `MAX_ZOOM = 20`; `TileEngineConfig` — which is what `wgpu_state` actually feeds it
    // — defaults to **19**, and one level of near-field refinement doubles the tile count.
    qt.lod_factor = lod_factor_for(1.0, 256.0, p.height as f32, cam.fovy());
    qt.max_zoom = config.max_zoom;
    // **And the fog density, which is not cosmetic here.** WP5's `apply_lod` relaxation
    // multiplies `subdivide_dist` by `1 − fog(distance)`, so at 900 m — where fog is
    // thickest — the far field never refines past z10/z11 in the first place. Leaving it
    // at 0, as the ridge-world sweep does, doubles the visible set (100 tiles against the
    // capture's 49 at `alps_inn_valley`) and hands D3 a far field production never draws.
    // That difference is most of the gap between the synthetic reduction and §7b's
    // captures, and a harness that did not reproduce it would be measuring a globe the
    // engine does not render.
    qt.fog_density = cesium_engine::globe::quadtree::fog_density_for(p.alt_m as f32, &config.fog);
    qt.pipeline = match occlusion {
        Some(_) => CullPipeline::TERRAIN_DEFAULT,
        None => CullPipeline::DEFAULT,
    };
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
        match &occlusion {
            // `cam_alt` stands in for the AGL §7f's gate reads. It is an over-estimate
            // (the ground is at or above the ellipsoid at every pose here), so the gate
            // can only shut *earlier* than the engine's would — never later, which is the
            // direction that would make this harness report culls the renderer does not
            // get. The poses it shuts off are the four that already read 0.0 %.
            Some(cfg) => qt.refresh_terrain_horizon(frustum, cam_alt, cam_alt, cfg),
            None => qt.clear_terrain_horizon(),
        }
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

// ── the granularity probe ────────────────────────────────────────────────────────

/// Is **every** sub-rectangle of a `2^depth × 2^depth` division of this node hidden?
///
/// The ceiling of `docs/terrain-plan.md` §7b's proposal, measured without committing to
/// an implementation of it. A sub-rectangle's box is built by
/// [`QuadtreeNode::for_surface_with`] on the *descendant tile id* with the **parent's**
/// height interval, which is exactly what `SubGrid::build` does one level of abstraction
/// down: the node's `[lo, hi]` bounds every drawn point over any part of its ground
/// (I-1′), so a box fitted over a sub-rectangle at that span contains the geometry over
/// that sub-rectangle. Culling when every one of them is hidden is sound for the same
/// reason `SubGrid::has_surviving_sub_patch` is — the sub-rectangles' union is the whole
/// patch.
///
/// `depth = 0` is the shipped node-level test, so the probe brackets it.
fn occluded_at_depth(
    horizon: &TerrainHorizon,
    node: &QuadtreeNode<Heightfield>,
    depth: u8,
) -> bool {
    if depth == 0 {
        return horizon.occludes(&node.obb, &tile_bounds(&node.id));
    }
    let n = 1u32 << depth;
    let z = node.id.z + depth;
    if z > 30 {
        return horizon.occludes(&node.obb, &tile_bounds(&node.id));
    }
    for dy in 0..n {
        for dx in 0..n {
            let id = TileId {
                z,
                x: node.id.x * n + dx,
                y: node.id.y * n + dy,
            };
            let sub = QuadtreeNode::<Heightfield>::for_surface_with(id, node.extra);
            if !horizon.occludes(&sub.obb, &tile_bounds(&id)) {
                return false;
            }
        }
    }
    true
}

/// Per-level tallies of what the stage decides on one settled tree.
#[derive(Default, Clone)]
struct GranularityTally {
    /// Nodes the traversal reached and offered to the stage, by level.
    seen: HashMap<u8, usize>,
    /// …of which the node-level test (depth 0) culls, and each finer depth.
    culled: Vec<HashMap<u8, usize>>,
}

fn probe_node(
    node: &QuadtreeNode<Heightfield>,
    horizon: &TerrainHorizon,
    depths: &[u8],
    tally: &mut GranularityTally,
) {
    *tally.seen.entry(node.id.z).or_insert(0) += 1;
    for (i, d) in depths.iter().enumerate() {
        if occluded_at_depth(horizon, node, *d) {
            *tally.culled[i].entry(node.id.z).or_insert(0) += 1;
        }
    }
    if let Some(children) = &node.children {
        for c in children.iter() {
            probe_node(c, horizon, depths, tally);
        }
    }
}

/// **The refutation of `docs/terrain-plan.md` §7b's follow-up, kept re-runnable without
/// the code it refutes** — what a per-sub-patch occludee test would buy on real terrain.
///
/// §7b proposes testing per sub-patch "the way `Stage::SubPatchGrid` does", on the
/// grounds that `k²` small boxes hug a ridge line far more closely than one big box over
/// a node's own `h_max`. §7c built exactly that, measured it and removed it again: two to
/// four more tiles out of forty to sixty-seven, for six to ten times the cost of the whole
/// D1+D2 pass. This test is what survives, and deliberately so — it needs **no engine
/// support at all**, so the measurement outlives the implementation.
///
/// It runs the proposal at **its ceiling**: not `SubGrid`'s `k`, but a 2×2, 4×4 and 8×8
/// division of every node the traversal reaches, each sub-rectangle given its own box by
/// [`QuadtreeNode::for_surface_with`] on the descendant tile id at the *parent's* height
/// interval — which is what `SubGrid::build` does one abstraction down — and the node
/// counted as culled only when every sub-rectangle is hidden.
///
/// **Read the level histogram, not just the totals.** It is what says the extra culls are
/// all at z1–z6 and none below: a far-field z10 tile 19–39 km across is never wholly
/// hidden however finely it is cut, so what a finer bound removes is a coarse *leaf*, and
/// what hides it is the terrain-aware curvature horizon rather than a ridge.
///
/// The ridge world runs alongside as a positive control: a probe that finds nothing
/// everywhere is indistinguishable from a probe that is broken.
///
/// `#[ignore]`d — it needs the network for the DEM (cached under `CESIUM_HEIGHT_CACHE`)
/// and it is a measurement, not a gate.
///
/// ```text
/// CESIUM_HEIGHT_CACHE=/tmp/dem \
///   cargo test --release --lib terrain::test_terrain_occlusion::d3_sub_patch -- \
///   --ignored --nocapture
/// ```
#[test]
#[ignore = "measurement, needs the network for real height tiles"]
fn d3_sub_patch_granularity_probe() {
    const DEPTHS: [u8; 4] = [0, 1, 2, 3];
    let cfg = TerrainOcclusionConfig::default();
    let mut world = RealWorld::new();

    println!("  [D3 granularity] nodes the stage culls, by occludee granularity");
    println!(
        "    {:<22} {:>7} {:>8} {:>8} {:>8} {:>8}",
        "pose", "nodes", "node", "2x2", "4x4", "8x8"
    );

    for (name, p) in real_poses() {
        let (frustum, _) = frustum_for(&p);
        let (qt, _) = settled_real_tree(&p, &frustum, Some(cfg), &mut world);
        let Some(horizon) = qt.terrain_horizon() else {
            println!("    {name:<22} no march");
            continue;
        };
        if !horizon.is_active() {
            println!("    {name:<22} march inactive (altitude gate)");
            continue;
        }
        let mut tally = GranularityTally {
            seen: HashMap::new(),
            culled: vec![HashMap::new(); DEPTHS.len()],
        };
        for root in qt.roots.iter() {
            probe_node(root, horizon, &DEPTHS, &mut tally);
        }
        let seen: usize = tally.seen.values().sum();
        let totals: Vec<usize> = tally.culled.iter().map(|m| m.values().sum()).collect();
        println!(
            "    {name:<22} {seen:>7} {:>8} {:>8} {:>8} {:>8}",
            totals[0], totals[1], totals[2], totals[3]
        );
        // The level histogram is the point of the whole probe: §7b blames the far field's
        // coarseness, and `sub_boxes_per_axis` gives no sub-grid at all from z8 down.
        let mut levels: Vec<u8> = tally.seen.keys().copied().collect();
        levels.sort_unstable();
        for z in levels {
            let s = tally.seen[&z];
            let c: Vec<usize> = tally
                .culled
                .iter()
                .map(|m| m.get(&z).copied().unwrap_or(0))
                .collect();
            if c.iter().all(|v| *v == 0) {
                continue;
            }
            println!(
                "        z{z:<3} seen {s:>5}   node {:>5}  2x2 {:>5}  4x4 {:>5}  8x8 {:>5}",
                c[0], c[1], c[2], c[3]
            );
        }
    }

    println!("  [D3 granularity] positive control — the synthetic ridge world");
    let ridge_cfg = TerrainOcclusionConfig {
        max_camera_altitude_m: 40_000.0,
        ..TerrainOcclusionConfig::default()
    };
    for (name, p) in reduction_poses() {
        let (frustum, _) = frustum_for(&p);
        let (qt, _, _) = settled_tree(&p, &frustum, Some(ridge_cfg), RIDGE_M);
        let Some(horizon) = qt.terrain_horizon() else {
            continue;
        };
        if !horizon.is_active() {
            println!("    {name:<22} march inactive");
            continue;
        }
        let mut tally = GranularityTally {
            seen: HashMap::new(),
            culled: vec![HashMap::new(); DEPTHS.len()],
        };
        for root in qt.roots.iter() {
            probe_node(root, horizon, &DEPTHS, &mut tally);
        }
        let seen: usize = tally.seen.values().sum();
        let totals: Vec<usize> = tally.culled.iter().map(|m| m.values().sum()).collect();
        println!(
            "    {name:<22} {seen:>7} {:>8} {:>8} {:>8} {:>8}",
            totals[0], totals[1], totals[2], totals[3]
        );
    }
}

/// **The real-terrain reduction and the real-terrain cost**, in one table — what D3
/// removes at the capture poses and five more low ones, and what it charges for it.
///
/// `rendering::terrain_capture` measures the reduction too, but only alongside a render,
/// so it cannot be run in a loop while a change is being tuned. This is the same tree,
/// the same LOD threshold and — the part that turns out to matter most — the same **fog
/// relaxation**, counted instead of drawn. It agrees with the capture to a tile
/// (`alps_inn_valley` 50 → 48 here, 49 → 48 there).
///
/// The cost columns are what `docs/terrain-plan.md` §7c's optimisation pass is measured
/// against, and they are charged to different places: `refresh_terrain_horizon` is **once
/// per frame**, `Stage::TerrainOcclusion` is per node and shows up inside `update`.
#[test]
#[ignore = "measurement, needs the network for real height tiles"]
fn d3_on_real_terrain() {
    use std::time::Instant;

    let cfg = TerrainOcclusionConfig::default();
    let mut world = RealWorld::new();

    println!("  [D3 real] visible tiles at the capture poses and five more low ones");
    println!(
        "    {:<22} {:>8} {:>8} {:>9} {:>9}",
        "pose", "D1+D2", "with D3", "delta", "alt (m)"
    );
    let mut poses: Vec<(&'static str, ViewParams)> = Vec::new();
    for (name, p) in real_poses() {
        let (frustum, _) = frustum_for(&p);
        let (reference, _) = settled_real_tree(&p, &frustum, None, &mut world);
        let (qt, _) = settled_real_tree(&p, &frustum, Some(cfg), &mut world);
        let before = reference.get_visible_tiles().len();
        let after = qt.get_visible_tiles().len();
        println!(
            "    {name:<22} {before:>8} {after:>8} {:>7.1} % {:>9.0}",
            100.0 * (after as f64 / before.max(1) as f64 - 1.0),
            p.alt_m
        );
        poses.push((name, p));
    }

    println!("  [D3 real cost] per frame, same trees");
    println!(
        "    {:<22} {:>10} {:>12} {:>12}",
        "pose", "march", "upd D1+D2", "upd +D3"
    );
    const REPS: u32 = 400;
    for (name, p) in &poses {
        let (frustum, _) = frustum_for(p);
        let cam_alt = p.alt_m * 1.0e-6;
        let (mut qt, _) = settled_real_tree(p, &frustum, Some(cfg), &mut world);

        let t = Instant::now();
        for _ in 0..REPS {
            // `cam_alt` stands in for the AGL §7f's gate reads. It is an over-estimate
            // (the ground is at or above the ellipsoid at every pose here), so the gate
            // can only shut *earlier* than the engine's would — never later, which is the
            // direction that would make this harness report culls the renderer does not
            // get. The poses it shuts off are the four that already read 0.0 %.
            qt.refresh_terrain_horizon(&frustum, cam_alt, cam_alt, &cfg);
        }
        let march = t.elapsed().as_secs_f64() * 1.0e6 / REPS as f64;
        let t = Instant::now();
        for _ in 0..REPS {
            qt.update(&frustum);
        }
        let with_us = t.elapsed().as_secs_f64() * 1.0e6 / REPS as f64;

        qt.clear_terrain_horizon();
        qt.pipeline = CullPipeline::DEFAULT;
        let t = Instant::now();
        for _ in 0..REPS {
            qt.update(&frustum);
        }
        let without_us = t.elapsed().as_secs_f64() * 1.0e6 / REPS as f64;

        println!("    {name:<22} {march:>8.1} us {without_us:>10.1} us {with_us:>10.1} us");
    }
}
