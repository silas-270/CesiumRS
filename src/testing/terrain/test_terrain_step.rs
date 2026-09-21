//! **The terrain-step question**: a camera at eye height at the foot of an escarpment,
//! looking at it, with flat high ground behind it. Is that high ground removed, or is it
//! fetched and drawn?
//!
//! `docs/terrain-plan.md` §7b's own statement of where D3 should pay is "a valley floor,
//! a plain behind a range, water, **a plateau**", and §7c measured ten Alpine and
//! Himalayan poses — all of which have *more mountains* behind the first ridge, which is
//! exactly the case §7b says D3 cannot fire on. The Swabian Jura is the other shape: one
//! 400 m step (the Albtrauf) and then 40 km of plateau that never rises back to the
//! crest line. Same for the Stuttgart basin, whose rim stands 250 m over the city and
//! whose Filder plateau behind it is flat to the horizon.
//!
//! So this is not a tenth Alpine pose. It is the geometry D3 was specified for, and it
//! is the one shape §7c never measured.
//!
//! # What is measured, and why each piece is here
//!
//! 1. **The pose is checked against the DEM before anything else.** §7c lost three
//!    measurement poses to coordinates picked off a map that turned out to be inside a
//!    mountain, where `TerrainHorizon::finish`'s enclosure guard switches the whole march
//!    off and every reduction reads a flat zero for a reason that has nothing to do with
//!    the thing under test. [`terrain_step_pose_is_where_it_says_it_is`] reads the ground
//!    height, the horizon distance `√(2Rh)` and the whole sightline profile out of the
//!    real DEM and asserts the step is where the pose claims.
//!
//! 2. **Two height-fetch policies, not one.**
//!    `test_terrain_occlusion::d3_on_real_terrain` fills the height cache for *every node
//!    in the tree*, culled ones included. Production does not: `TileSystem::update` calls
//!    `request_height_chain` over `visible_tiles` and `missing_meshes` only
//!    (§9 F2b), so a node the culler removed asks for nothing. That difference is
//!    load-bearing for this question, because a node with no height tile of its own is on
//!    D1's **inherited margin** — 6 000 m at z10, 1 500 m at z12 (§7's table) — and a box
//!    that tall is not hidden behind anything. [`Fill::Visible`] is the production
//!    policy; [`Fill::Everything`] is the existing harness's, kept as the control that
//!    isolates the inheritance from the rest.
//!
//! 3. **The steady state and the path to it.** Both policies are run frame by frame, so
//!    a self-stabilising loop — no data, tall box, no cull, drawn, data arrives, cull —
//!    is visible as a trace rather than guessed at from a single settled number.
//!
//! 4. **Blame, as a number.** For every tile that is geometrically hidden and drawn
//!    anyway, the report prints the four angles that decide it: the true ridge from the
//!    DEM, the tile's true top, the tile's *box* top, and the march's guaranteed ridge
//!    ceiling. Whichever pair crosses is the answer to "what stops the cull", and it is
//!    read off the numbers rather than argued.
//!
//! Everything here is `#[ignore]`d measurement: it needs the network for the DEM (cached
//! under `CESIUM_HEIGHT_CACHE`) and it asserts nothing about the engine's behaviour that
//! a gate would want to enforce.
//!
//! ```text
//! CESIUM_HEIGHT_CACHE=/tmp/dem \
//!   cargo test --release --lib terrain::test_terrain_step -- --ignored --nocapture
//! ```

use cesium_engine::camera::camera::CameraMode;
use cesium_engine::globe::geometry::{
    lon_lat_alt_to_ecef_f64, TileMesh, EARTH_RADIUS_A_F64, EARTH_RADIUS_B_F64,
};
use cesium_engine::globe::quadtree::{
    lod_factor_for, terrain_lod_factor_for, tile_bounds, transform_to_scaled_space, CullPipeline,
    Frustum, HorizonCamera, QuadtreeManager, QuadtreeNode, TerrainFogPolicy, TerrainHorizon,
    TerrainOcclusionConfig, TileId, AZIMUTH_SECTORS,
};
use cesium_engine::globe::quadtree::terrain_occlusion::{ridge_safety_m, MIN_RANGE_M};
use cesium_engine::globe::terrain::heightfield::inherit_margin_mm;
use cesium_engine::globe::terrain::{
    HeightBounds, HeightPatch, HeightTileManager, Heightfield, PatchStatus,
};
use glam::DVec3;

use crate::testing::culling::cameras::{build_camera, ViewParams};
use crate::testing::culling::oracle::{VisibilityOracle, NDC_MARGIN};
use crate::testing::terrain::test_terrain_occlusion::{
    bounds_source, collect_sources, fetch_missing, fill_cache_real, real_config, RealWorld,
    SEGMENTS,
};

/// Level the ground-truth DEM march reads at — the deepest the source serves at full
/// resolution over Europe, so the ridge the oracle sees is the sharpest one available.
/// A z14 tile at 48° N is 1.62 km across over 256 texels, i.e. 6.3 m a texel.
const DEM_Z: u8 = 14;

/// Steps in the oracle's ray march from the eye to a sample point.
///
/// Uniform in distance over ranges of 10–45 km, so 40–180 m a step against an escarpment
/// whose horizontal run is some 2 km. The march cannot step over the Albtrauf, which is
/// the only way this oracle could claim a tile is visible when it is not.
const MARCH_STEPS: usize = 384;

/// Samples per axis over a candidate tile's ground rectangle.
///
/// 17 × 17 = 289 points. On the z11/z12 tiles the far field actually holds here (13 km
/// and 6.5 km across) that is 810 m and 405 m between samples over a plateau whose
/// relief at that scale is tens of metres — see the profile
/// [`terrain_step_pose_is_where_it_says_it_is`] prints, which is what says the sampling
/// is fine enough for the claim being made.
const TILE_SAMPLES: usize = 17;

/// Metres the oracle shaves off the terrain before calling a ray blocked.
///
/// Same role as `test_terrain_occlusion::ORACLE_SLACK_M`: it makes the oracle say
/// "visible" more often, which is the direction that cannot manufacture the finding.
const ORACLE_SLACK_M: f64 = 25.0;

/// Metres by which a tile's top must clear the ridge before the oracle stops calling it
/// hidden — the same slack, applied to the other end of the same comparison.
const TILE_SLACK_M: f64 = 25.0;

/// Range, metres, and half-angle, degrees, of the fan [`warm_dem`] prefetches and the
/// oracle is therefore allowed to answer inside.
///
/// 60 km is past both poses' `√(2Rh)` horizon (69 km at Reutlingen, 57 km at Stuttgart),
/// so a tile that reaches outside it is not part of the question. It is reported in its
/// own column rather than folded into either answer.
const ORACLE_FAN_M: f64 = 60_000.0;
const ORACLE_FAN_DEG: f64 = 60.0;

// ── the poses ────────────────────────────────────────────────────────────────────

/// A camera at eye height in front of a terrain step, looking at it.
pub(crate) struct StepPose {
    pub(crate) name: &'static str,
    pub(crate) lon: f64,
    pub(crate) lat: f64,
    /// Metres above the local ground, read from the DEM rather than assumed.
    pub(crate) eye_agl_m: f64,
    /// Compass bearing the camera looks along, degrees clockwise from north.
    pub(crate) bearing_deg: f64,
    /// Degrees below the horizontal.
    pub(crate) below_deg: f64,
    /// Where the step's crest is, kilometres out — checked against the DEM, not trusted.
    pub(crate) crest_km: f64,
    /// What is behind it, for the report.
    pub(crate) what: &'static str,
}

/// Two places of the same shape, so the answer does not hang on one pose.
///
/// Both are flat-land-behind-a-step, both are in the DEM at full z14 resolution, and
/// neither has a second range behind the first — which is precisely what disqualified
/// §7c's ten Alpine poses from answering this question.
pub(crate) fn step_poses() -> Vec<StepPose> {
    vec![
        StepPose {
            name: "reutlingen_albtrauf",
            lon: 9.2043,
            lat: 48.4914,
            eye_agl_m: 2.0,
            bearing_deg: 135.0,
            below_deg: 1.0,
            crest_km: 8.0,
            what: "the Albtrauf at 8 km, then 40 km of Alb plateau behind it",
        },
        StepPose {
            name: "stuttgart_kessel",
            lon: 9.1829,
            lat: 48.7758,
            eye_agl_m: 2.0,
            bearing_deg: 180.0,
            below_deg: 1.0,
            crest_km: 2.5,
            what: "the basin rim at 2.5 km, then the Filder plateau and the Alb",
        },
    ]
}

impl StepPose {
    /// The `ViewParams` this pose becomes, with the eye placed over the **DEM's** ground
    /// rather than over a number typed into the file.
    fn view(&self, ground_m: f64) -> ViewParams {
        ViewParams {
            sweep: "step",
            lat_deg: self.lat,
            lon_deg: self.lon,
            alt_m: ground_m + self.eye_agl_m,
            // `ViewParams::pitch_deg` is measured from nadir.
            pitch_deg: 90.0 - self.below_deg,
            // **Negated, and this is the third pose trap in this file's family.**
            // `ViewParams::yaw_deg` is documented as "compass-style heading", and
            // `camera_transform` composes it as `Rz(yaw)` about the *nadir*-aligned view
            // axis — which is local **down**. A rotation of `+yaw` about `−up` is a
            // rotation of `−yaw` about `+up`, so `yaw_deg = 135` builds a camera looking
            // along bearing **225°**. Every pose in
            // `test_terrain_occlusion::real_poses` uses 0° or 180°, where the sign cannot
            // show, which is why this has never mattered before.
            // `terrain_step_pose_is_where_it_says_it_is` measures the heading off the
            // built camera and fails if this line is removed.
            yaw_deg: -self.bearing_deg,
            roll_deg: 0.0,
            width: 1280,
            height: 720,
            mode: CameraMode::Free,
        }
    }
}

// ── the DEM, read directly ───────────────────────────────────────────────────────

/// Web-Mercator column and row of a point at level `z`, as fractions.
fn mercator_xy(lon: f64, lat: f64, z: u8) -> (f64, f64) {
    let n = (1_u64 << z) as f64;
    let x = (lon + 180.0) / 360.0 * n;
    let lat = lat.clamp(-85.051_128, 85.051_128);
    let y = (1.0 - lat.to_radians().tan().asinh() / std::f64::consts::PI) * 0.5 * n;
    (x, y)
}

/// Longitude, latitude and radial altitude (megametres) of an ECEF point — the inverse
/// of `geometry::lon_lat_to_ecef_f64`, copied from `test_terrain_occlusion` so the two
/// oracles do not share a line with each other or with the engine.
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

/// Bilinear DEM height at `(lon, lat)`, metres, read off the real terrarium tiles.
fn dem_height_m(world: &mut RealWorld, lon: f64, lat: f64) -> f64 {
    let (fx, fy) = mercator_xy(lon, lat, DEM_Z);
    let id = TileId {
        z: DEM_Z,
        x: fx.floor().max(0.0) as u32,
        y: fy.floor().max(0.0) as u32,
    };
    let tile = world.tile(id);
    tile.sample_bilinear(fx - fx.floor(), fy - fy.floor())
}

/// ECEF megametres of a geodetic point.
fn ecef(lon: f64, lat: f64, alt_m: f64) -> DVec3 {
    let p = lon_lat_alt_to_ecef_f64(lon, lat, alt_m);
    DVec3::new(p[0], p[1], p[2])
}

/// Great-circle destination from `(lon, lat)` on `bearing`, `dist_m` away — spherical,
/// which is centimetres off at these ranges and is only used to *place* probe points.
fn dest(lon: f64, lat: f64, bearing_deg: f64, dist_m: f64) -> (f64, f64) {
    const R: f64 = EARTH_RADIUS_A_F64 * 1.0e6;
    let (br, d) = (bearing_deg.to_radians(), dist_m / R);
    let (la, lo) = (lat.to_radians(), lon.to_radians());
    let la2 = (la.sin() * d.cos() + la.cos() * d.sin() * br.cos()).asin();
    let lo2 = lo + (br.sin() * d.sin() * la.cos()).atan2(d.cos() - la.sin() * la2.sin());
    (lo2.to_degrees(), la2.to_degrees())
}

/// Elevation angle of `p` seen from `eye` with local up `up`, degrees.
fn elevation_deg(eye: DVec3, up: DVec3, p: DVec3) -> f64 {
    let v = p - eye;
    let vert = v.dot(up);
    let horiz = (v - up * vert).length();
    vert.atan2(horiz).to_degrees()
}

/// Outward ellipsoid normal at `p`.
fn normal_at(p: DVec3) -> DVec3 {
    const INV_A2: f64 = 1.0 / (EARTH_RADIUS_A_F64 * EARTH_RADIUS_A_F64);
    const INV_B2: f64 = 1.0 / (EARTH_RADIUS_B_F64 * EARTH_RADIUS_B_F64);
    DVec3::new(p.x * INV_A2, p.y * INV_B2, p.z * INV_A2).normalize()
}

/// One batched download of every DEM tile the oracle's marches can touch.
///
/// `RealWorld::tile` fetches one tile per `curl` invocation, which is fine for the tens
/// of height tiles a quadtree wants and hopeless for the hundreds of z14 tiles a dense
/// ground-truth march reads: the first run of this measurement spent ten minutes at three
/// seconds of CPU, entirely in round trips. The fan below covers ±60° of the pose bearing
/// out to 60 km — past the horizon at both poses — and hands the whole list to
/// `fetch_missing`, which batches 48 at a time, twelve in parallel.
fn warm_dem(world: &mut RealWorld, pose: &StepPose) {
    let mut ids = Vec::new();
    let mut d = 0.0;
    while d <= 60_000.0 {
        let mut b = -60.0;
        while b <= 60.0 {
            let (lo, la) = dest(pose.lon, pose.lat, pose.bearing_deg + b, d);
            let (fx, fy) = mercator_xy(lo, la, DEM_Z);
            ids.push(TileId {
                z: DEM_Z,
                x: fx.floor().max(0.0) as u32,
                y: fy.floor().max(0.0) as u32,
            });
            b += 1.0;
        }
        d += 400.0;
    }
    ids.sort_unstable_by_key(|i| (i.z, i.x, i.y));
    ids.dedup();
    println!("    warming the DEM oracle: {} z{DEM_Z} tiles", ids.len());
    fetch_missing(&world.dir, &ids);
}

/// Does the **real DEM** block the straight segment from `eye` to `p`?
///
/// Nothing to do with the engine's occluder: a dense march against the terrarium tiles
/// themselves, at the source's own resolution, with no bounding volume anywhere in it.
fn dem_blocks(world: &mut RealWorld, eye: DVec3, p: DVec3) -> bool {
    let seg = p - eye;
    for i in 1..MARCH_STEPS {
        let q = eye + seg * (i as f64 / MARCH_STEPS as f64);
        let (lon, lat, alt) = geodetic_of(q);
        if alt * 1.0e6 < dem_height_m(world, lon, lat) - ORACLE_SLACK_M {
            return true;
        }
    }
    false
}

// ── the tree ─────────────────────────────────────────────────────────────────────

/// Which nodes get height data — the whole point of running this twice.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub(crate) enum Fill {
    /// Every node in the tree, culled ones included. What
    /// `test_terrain_occlusion::d3_on_real_terrain` does, and **not** what the engine
    /// does.
    Everything,
    /// The visible set and its height ancestor chain, which is exactly what
    /// `TileSystem::update` requests (`request_height_chain` over `visible_tiles`, then
    /// over `missing_meshes`). A node the culler removed asks for nothing, so it stays on
    /// D1's inherited interval for as long as it stays culled.
    Visible,
}

fn frustum_for(p: &ViewParams) -> Frustum {
    let cam = build_camera(p);
    let aspect = p.aspect();
    let (eye, _) = cam.global_transform_f64();
    Frustum::planes_only(cam.calculate_frustum_planes(aspect as f32), eye)
        .with_corners(cam.frustum_corners_relative(aspect as f32))
}

/// Fills the height cache for `id`'s own source tile and its whole ancestor chain —
/// `TileSystem::request_height_chain`, with the fetch completing instantly.
fn fill_chain(id: TileId, heights: &mut HeightTileManager, world: &mut RealWorld) {
    let mut curr = heights.source_tile_for(id);
    loop {
        if heights.status_of(curr) != PatchStatus::Ready {
            let t = world.tile(curr);
            heights.insert_ready(curr, t);
        }
        match curr.parent() {
            Some(p) => curr = p,
            None => break,
        }
    }
}

/// One settled tree, with the whole frame-by-frame trace of how it got there.
struct Settled {
    qt: QuadtreeManager<Heightfield>,
    heights: HeightTileManager,
    /// `(visible leaves, height tiles resident, visible leaves on an inherited interval)`
    /// per frame.
    trace: Vec<(usize, usize, usize)>,
}

fn settle(
    p: &ViewParams,
    frustum: &Frustum,
    occlusion: Option<TerrainOcclusionConfig>,
    fill: Fill,
    frames: usize,
    world: &mut RealWorld,
) -> Settled {
    let config = real_config();
    let mut heights = HeightTileManager::new(&config);
    let mut qt = QuadtreeManager::<Heightfield>::for_surface();
    let cam = build_camera(p);
    // The capture's LOD threshold, ceiling and fog relaxation — §7c's finding is that the
    // last of the three decides most of the far field, so a harness without it measures a
    // globe the engine never draws.
    qt.lod_factor = lod_factor_for(1.0, 256.0, p.height as f32, cam.fovy());
    qt.max_zoom = config.max_zoom;
    qt.fog_density = cesium_engine::globe::quadtree::fog_density_for(p.alt_m as f32, &config.fog);
    // **E1's half of the LOD threshold, which `d3_on_real_terrain` does not set.**
    // `wgpu_state::update_logic` calls `set_terrain_lod` right after `set_frame_params`,
    // and on the terrain arm `apply_lod` reads all three. Leaving them at their defaults
    // refines the near field considerably deeper than the renderer does — 72 tiles here
    // against the capture's 62 at `reutlingen_albtrauf` — which is a harness that has
    // quietly stopped tracking the engine since §8 E1 landed after §7c was written.
    let mgep = config.terrain.max_geometric_error_px;
    qt.terrain_lod_factor = terrain_lod_factor_for(mgep, p.height as f32, cam.fovy());
    qt.terrain_fog_policy = TerrainFogPolicy::default();
    qt.terrain_fog_sse_ratio = if mgep > 0.0 { config.fog.sse / mgep } else { 0.0 };
    qt.pipeline = match occlusion {
        Some(_) => CullPipeline::TERRAIN_DEFAULT,
        None => CullPipeline::DEFAULT,
    };
    let cam_alt = p.alt_m * 1.0e-6;
    let mut trace = Vec::new();

    for _ in 0..frames {
        match fill {
            Fill::Everything => {
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
            }
            Fill::Visible => {
                // `TileSystem::update`'s own order: heights for the visible set first,
                // then the chain. `missing_meshes` is a subset of the tiles that have
                // just entered the visible set, so requesting over the visible set is
                // the superset of both.
                let visible: Vec<TileId> = qt.get_visible_tiles().into_iter().map(|t| t.0).collect();
                let mut wanted = Vec::new();
                for id in &visible {
                    let mut curr = heights.source_tile_for(*id);
                    loop {
                        wanted.push(curr);
                        match curr.parent() {
                            Some(p) => curr = p,
                            None => break,
                        }
                    }
                }
                wanted.sort_unstable_by_key(|id| (id.z, id.x, id.y));
                wanted.dedup();
                fetch_missing(&world.dir, &wanted);
                for id in &visible {
                    fill_chain(*id, &mut heights, world);
                }
            }
        }
        qt.refresh_extras(&bounds_source(&heights));
        match &occlusion {
            Some(cfg) => qt.refresh_terrain_horizon(frustum, cam_alt, cam_alt, cfg),
            None => qt.clear_terrain_horizon(),
        }
        qt.update(frustum);

        let visible = qt.get_visible_tiles();
        let inherited = visible
            .iter()
            .filter(|(id, _, _)| heights.status_of(*id) != PatchStatus::Ready)
            .count();
        trace.push((visible.len(), heights.residency().0, inherited));
    }
    Settled {
        qt,
        heights,
        trace,
    }
}

// ── the candidates ───────────────────────────────────────────────────────────────

/// One visible tile beyond the step, with every number the verdict rests on.
struct Candidate {
    id: TileId,
    /// Ground range to the rectangle's nearest point, kilometres.
    near_km: f64,
    /// The tile's true maximum height off the DEM, metres.
    true_top_m: f64,
    /// The node's D1 interval, metres — what the box is actually fitted to.
    box_hi_m: f64,
    box_lo_m: f64,
    /// Does the node hold its **own** height tile, or is it on an inherited interval?
    ready: bool,
    /// Elevation angle of the tile's true surface, degrees — the largest over the
    /// sampled rectangle.
    theta_true_deg: f64,
    /// Elevation angle of the top of the node's D1 **box**, degrees. This is the quantity
    /// `TerrainHorizon::occludes` compares.
    theta_box_deg: f64,
    /// Is every sampled point of the tile behind real terrain?
    dem_hidden: bool,
    /// The smallest margin, in degrees, by which the DEM hides the tile. Negative where
    /// some sample is not hidden.
    dem_margin_deg: f64,
    /// Does `Stage::TerrainOcclusion` remove it?
    d3_culls: bool,
    /// Is the whole rectangle inside the oracle's prefetched fan? A tile that is not is
    /// counted and reported, never scored — an unscored tile must not be silently folded
    /// into either column.
    scored: bool,
}

/// Upper bound on the elevation angle of every point of `obb`, degrees — the same two
/// linear extremes `TerrainHorizon::occludes` takes, and the quantity it tests.
fn box_elevation_deg(obb: &cesium_engine::globe::quadtree::OrientedBoundingBox, eye: DVec3, up: DVec3) -> f64 {
    let half: [DVec3; 3] = [
        DVec3::new(obb.half_axes[0].x as f64, obb.half_axes[0].y as f64, obb.half_axes[0].z as f64),
        DVec3::new(obb.half_axes[1].x as f64, obb.half_axes[1].y as f64, obb.half_axes[1].z as f64),
        DVec3::new(obb.half_axes[2].x as f64, obb.half_axes[2].y as f64, obb.half_axes[2].z as f64),
    ];
    let v = obb.center - eye;
    let vert_c = v.dot(up);
    let horiz_c = (v - up * vert_c).length();
    let mut vert_span = 0.0;
    let mut horiz_span = 0.0;
    for h in &half {
        let a = h.dot(up);
        vert_span += a.abs();
        horiz_span += (*h - up * a).length();
    }
    let vert_max = vert_c + vert_span;
    if vert_max >= 0.0 {
        vert_max.atan2((horiz_c - horiz_span).max(0.0)).to_degrees()
    } else {
        vert_max.atan2(horiz_c + horiz_span).to_degrees()
    }
}

/// The march's own ceiling, degrees — `max` over the whole finished ridge grid, which is
/// the first thing `occludes` compares against and the cheapest thing to blame.
fn ridge_ceiling_deg(h: &TerrainHorizon, cfg: &TerrainOcclusionConfig) -> f64 {
    let mut top = f32::NEG_INFINITY;
    for a in 0..AZIMUTH_SECTORS {
        let b = (a as f64 + 0.5) * std::f64::consts::TAU / AZIMUTH_SECTORS as f64;
        let e = h.ridge_elevation(b, cfg.max_range_m as f64);
        if e > top {
            top = e;
        }
    }
    (top as f64).to_degrees()
}

/// **Where the occluder goes.** The march's guaranteed ridge along the pose bearing,
/// range by range, against the elevation angle of the terrain that is actually there.
///
/// This is the whole answer in one table when the boxes turn out to be tight: an occludee
/// bound cannot be blamed for failing to clear a ridge the occluder never built.
fn ridge_profile(
    horizon: &TerrainHorizon,
    world: &mut RealWorld,
    eye: DVec3,
    up: DVec3,
    pose: &StepPose,
) {
    println!(
        "      {:>7} {:>12} {:>12} {:>10}",
        "km", "DEM ridge", "march ridge", "lost"
    );
    let bearing = pose.bearing_deg.to_radians();
    let mut best_dem = f64::NEG_INFINITY;
    for km in [1.0, 2.0, 4.0, 6.0, 8.0, 10.0, 15.0, 20.0, 30.0, 45.0] {
        let (lo, la) = dest(pose.lon, pose.lat, pose.bearing_deg, km * 1_000.0);
        let h = dem_height_m(world, lo, la);
        let dem = elevation_deg(eye, up, ecef(lo, la, h));
        best_dem = best_dem.max(dem);
        let march = horizon.ridge_elevation(bearing, km * 1_000.0) as f64;
        let march = if march.is_finite() {
            march.to_degrees()
        } else {
            f64::NEG_INFINITY
        };
        println!(
            "      {km:>7.1} {:>12.3} {:>12.3} {:>10.3}",
            dem,
            march,
            best_dem - march
        );
    }
}

/// The node that is drawn over the crest of the step, and the occluder it contributes.
///
/// `HeightBounds::floor` is a **minimum** over the node's ground, and `floor_grid` the
/// same per 4×4 sub-cell. On an escarpment the minimum over any cell that contains the
/// crest is the valley below it — so the number to look at is not "is the crest in this
/// tile" but "how far under the crest does the tile's floor sit".
fn crest_occluder(
    settled: &Settled,
    world: &mut RealWorld,
    pose: &StepPose,
    eye: DVec3,
    up: DVec3,
) {
    let (lo, la) = dest(pose.lon, pose.lat, pose.bearing_deg, pose.crest_km * 1_000.0);
    let crest_m = dem_height_m(world, lo, la);
    let mut extras = std::collections::HashMap::new();
    collect_extras(&settled.qt, &mut extras, &settled.heights);
    // The drawn leaf over the crest: the deepest visible node whose rectangle contains it.
    let mut best: Option<(TileId, HeightBounds)> = None;
    for (id, _, _) in settled.qt.get_visible_tiles() {
        let b = tile_bounds(&id);
        if lo >= b.lon_min && lo <= b.lon_max && la >= b.lat_min && la <= b.lat_max {
            if let Some((e, _)) = extras.get(&id) {
                best = Some((id, *e));
            }
        }
    }
    let Some((id, extra)) = best else {
        println!("      no drawn leaf over the crest — nothing stamps it");
        return;
    };
    let b = tile_bounds(&id);
    let width_km = ground_range_m(b.lon_min, 0.5 * (b.lat_min + b.lat_max), b.lon_max, 0.5 * (b.lat_min + b.lat_max))
        / 1_000.0;
    let grid_max = extra
        .floor_grid
        .iter()
        .fold(f32::NEG_INFINITY, |a, b| a.max(*b)) as f64
        * 1.0e6;
    let crest_ang = elevation_deg(eye, up, ecef(lo, la, crest_m));
    let floor_ang = elevation_deg(eye, up, ecef(lo, la, grid_max));
    println!(
        "      crest tile {}/{}/{} is {:.1} km wide: crest {:.0} m, node floor {:.0} m, \
         best sub-cell floor {:.0} m — {:.0} m of ridge lost, {:.3} deg against {:.3} deg",
        id.z,
        id.x,
        id.y,
        width_km,
        crest_m,
        extra.floor * 1.0e6,
        grid_max,
        crest_m - grid_max,
        floor_ang,
        crest_ang
    );
}

/// Collects every visible leaf whose ground lies wholly beyond the step, and scores it.
#[allow(clippy::too_many_arguments)]
fn candidates(
    settled: &Settled,
    d3: &Settled,
    eye: DVec3,
    up: DVec3,
    pose: &StepPose,
    world: &mut RealWorld,
) -> Vec<Candidate> {
    let horizon = d3.qt.terrain_horizon();
    let d3_visible: std::collections::HashSet<TileId> =
        d3.qt.get_visible_tiles().into_iter().map(|t| t.0).collect();

    let mut extras: std::collections::HashMap<TileId, (HeightBounds, bool)> =
        std::collections::HashMap::new();
    collect_extras(&settled.qt, &mut extras, &settled.heights);

    let mut out = Vec::new();
    for (id, _, _) in settled.qt.get_visible_tiles() {
        let b = tile_bounds(&id);
        // Ground range to the rectangle's nearest point, from the camera's own ground
        // position — the same clamp `TerrainHorizon::extent_of` makes.
        let clamped_lat = pose.lat.clamp(b.lat_min, b.lat_max);
        let clamped_lon = pose.lon.clamp(b.lon_min, b.lon_max);
        let near_m = ground_range_m(pose.lon, pose.lat, clamped_lon, clamped_lat);
        if near_m < pose.crest_km * 1_000.0 {
            continue;
        }
        // Behind the step *in the direction the camera looks*: the rectangle's centre
        // must be within a quadrant of the pose bearing, or it is not in the shadow under
        // discussion at all.
        let bearing = initial_bearing_deg(
            pose.lon,
            pose.lat,
            0.5 * (b.lon_min + b.lon_max),
            0.5 * (b.lat_min + b.lat_max),
        );
        if wrap_deg(bearing - pose.bearing_deg).abs() > 45.0 {
            continue;
        }

        // **The oracle only answers inside the fan [`warm_dem`] prefetched.** Outside it
        // every DEM read is a fresh `curl` round trip — the first version of this
        // measurement spent twenty minutes at two seconds of CPU doing exactly that — and,
        // more to the point, a tile that reaches past the 69 km horizon is not "hidden
        // behind the step" in any sense the question is asking about. Such a tile is
        // counted and reported, never scored.
        let mut scored = true;
        let mut true_top_m = f64::NEG_INFINITY;
        let mut theta_true_deg = f64::NEG_INFINITY;
        let mut hidden = true;
        let mut margin = f64::INFINITY;
        let mut samples = Vec::with_capacity(TILE_SAMPLES * TILE_SAMPLES);
        for iy in 0..TILE_SAMPLES {
            for ix in 0..TILE_SAMPLES {
                let fx = ix as f64 / (TILE_SAMPLES - 1) as f64;
                let fy = iy as f64 / (TILE_SAMPLES - 1) as f64;
                let lon = b.lon_min + fx * (b.lon_max - b.lon_min);
                // Rows uniform in Mercator y, like the mesh's.
                let lat = mercator_lat(&b, fy);
                let r = ground_range_m(pose.lon, pose.lat, lon, lat);
                let bg = initial_bearing_deg(pose.lon, pose.lat, lon, lat);
                if r > ORACLE_FAN_M || wrap_deg(bg - pose.bearing_deg).abs() > ORACLE_FAN_DEG {
                    scored = false;
                }
                samples.push((lon, lat));
            }
        }
        if scored {
            for (lon, lat) in &samples {
                let h = dem_height_m(world, *lon, *lat);
                true_top_m = true_top_m.max(h);
                let p = ecef(*lon, *lat, h + TILE_SLACK_M);
                theta_true_deg = theta_true_deg.max(elevation_deg(eye, up, p));
                if hidden {
                    if dem_blocks(world, eye, p) {
                        // How far under the blocking ridge this sample sits, as an angle.
                        margin = margin.min(ridge_margin_deg(world, eye, up, p));
                    } else {
                        hidden = false;
                        margin = f64::NEG_INFINITY;
                    }
                }
            }
        } else {
            hidden = false;
            margin = f64::NAN;
            true_top_m = f64::NAN;
            theta_true_deg = f64::NAN;
        }

        let (extra, ready) = match extras.get(&id) {
            Some(v) => *v,
            None => continue,
        };
        let node = QuadtreeNode::<Heightfield>::for_surface_with(id, extra);
        let theta_box_deg = box_elevation_deg(&node.obb, eye, up);
        let d3_culls = !d3_visible.contains(&id)
            && horizon.map(|h| h.occludes(&node.obb, &b)).unwrap_or(false);

        out.push(Candidate {
            id,
            near_km: near_m / 1_000.0,
            true_top_m,
            box_hi_m: extra.hi * 1.0e6,
            box_lo_m: extra.lo * 1.0e6,
            ready,
            theta_true_deg,
            theta_box_deg,
            dem_hidden: hidden,
            dem_margin_deg: margin,
            d3_culls,
            scored,
        });
    }
    out.sort_by(|a, b| a.near_km.partial_cmp(&b.near_km).unwrap());
    out
}

/// By how many degrees the DEM's tallest intervening ridge stands over the ray to `p`.
fn ridge_margin_deg(world: &mut RealWorld, eye: DVec3, up: DVec3, p: DVec3) -> f64 {
    let target = elevation_deg(eye, up, p);
    let seg = p - eye;
    let mut top = f64::NEG_INFINITY;
    for i in 1..MARCH_STEPS {
        let q = eye + seg * (i as f64 / MARCH_STEPS as f64);
        let (lon, lat, _) = geodetic_of(q);
        let h = dem_height_m(world, lon, lat);
        let e = elevation_deg(eye, up, ecef(lon, lat, h));
        if e > top {
            top = e;
        }
    }
    top - target
}

fn mercator_lat(b: &cesium_engine::globe::quadtree::TileBounds, f: f64) -> f64 {
    // `TileBounds` carries the latitude span already; interpolating in Mercator y keeps
    // the samples where the mesh's rows are rather than bunching them at one edge.
    let y = |lat: f64| lat.to_radians().tan().asinh();
    let (y0, y1) = (y(b.lat_max), y(b.lat_min));
    let yy = y0 + f * (y1 - y0);
    yy.sinh().atan().to_degrees()
}

fn ground_range_m(lon0: f64, lat0: f64, lon1: f64, lat1: f64) -> f64 {
    const R: f64 = EARTH_RADIUS_A_F64 * 1.0e6;
    let dla = (lat1 - lat0).to_radians();
    let dlo = wrap_deg(lon1 - lon0).to_radians() * (0.5 * (lat0 + lat1)).to_radians().cos();
    R * (dla * dla + dlo * dlo).sqrt()
}

fn initial_bearing_deg(lon0: f64, lat0: f64, lon1: f64, lat1: f64) -> f64 {
    let dla = (lat1 - lat0).to_radians();
    let dlo = wrap_deg(lon1 - lon0).to_radians() * (0.5 * (lat0 + lat1)).to_radians().cos();
    dlo.atan2(dla).to_degrees()
}

fn wrap_deg(d: f64) -> f64 {
    let mut d = d % 360.0;
    if d > 180.0 {
        d -= 360.0;
    }
    if d <= -180.0 {
        d += 360.0;
    }
    d
}

fn collect_extras(
    qt: &QuadtreeManager<Heightfield>,
    out: &mut std::collections::HashMap<TileId, (HeightBounds, bool)>,
    heights: &HeightTileManager,
) {
    fn walk(
        node: &QuadtreeNode<Heightfield>,
        out: &mut std::collections::HashMap<TileId, (HeightBounds, bool)>,
        heights: &HeightTileManager,
    ) {
        out.insert(
            node.id,
            (node.extra, heights.status_of(node.id) == PatchStatus::Ready),
        );
        if let Some(children) = &node.children {
            for c in children.iter() {
                walk(c, out, heights);
            }
        }
    }
    for root in qt.roots.iter() {
        walk(root, out, heights);
    }
}

// ── the tests ────────────────────────────────────────────────────────────────────

/// **The pose check, and it comes first.**
///
/// `docs/terrain-plan.md` §7c records three measurement poses lost to coordinates that
/// looked like a valley on a map and were a mountainside in the DEM. Nothing below is
/// worth reading unless this passes: the camera is over the ground it claims, the horizon
/// `√(2Rh)` reaches past the far end of the plateau, and the step is the maximum of the
/// sightline profile.
#[test]
#[ignore = "measurement, needs the network for real height tiles"]
fn terrain_step_pose_is_where_it_says_it_is() {
    const R: f64 = EARTH_RADIUS_A_F64 * 1.0e6;
    let mut world = RealWorld::new();

    for pose in step_poses() {
        let ground = dem_height_m(&mut world, pose.lon, pose.lat);
        let eye_m = ground + pose.eye_agl_m;
        let horizon_km = (2.0 * R * eye_m).sqrt() / 1_000.0;
        println!(
            "  [{}] ground {:.1} m, eye {:.1} m, horizon sqrt(2Rh) = {:.1} km — {}",
            pose.name, ground, eye_m, horizon_km, pose.what
        );

        let eye = ecef(pose.lon, pose.lat, eye_m);
        let up = normal_at(eye);
        println!("    {:>6} {:>10} {:>12}", "km", "ground (m)", "sight (deg)");
        let mut best = (0.0_f64, f64::NEG_INFINITY);
        for km in [
            0.5, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 12.0, 15.0, 20.0, 25.0, 30.0,
            35.0, 40.0, 45.0, 50.0, 60.0,
        ] {
            let (lo, la) = dest(pose.lon, pose.lat, pose.bearing_deg, km * 1_000.0);
            let h = dem_height_m(&mut world, lo, la);
            let e = elevation_deg(eye, up, ecef(lo, la, h));
            if e > best.1 {
                best = (km, e);
            }
            println!("    {km:>6.1} {h:>10.1} {e:>12.3}");
        }
        println!(
            "    step crest measured at {:.1} km, {:.3} deg (pose declares {:.1} km)",
            best.0, best.1, pose.crest_km
        );
        assert!(
            (best.0 - pose.crest_km).abs() <= 2.0,
            "{}: the sightline maximum is at {:.1} km, not the declared {:.1} km",
            pose.name,
            best.0,
            pose.crest_km
        );
        assert!(
            horizon_km > 40.0,
            "{}: the horizon is only {:.1} km, the plateau is not in view at all",
            pose.name,
            horizon_km
        );

        // **And the camera really looks that way.** `ViewParams::yaw_deg` is documented
        // as "compass-style", but `camera_transform` composes it about the *nadir*-aligned
        // view axis, which is local **down**: a rotation of `+yaw` about `−up` is a
        // rotation of `−yaw` about `+up`. Checking the pose's ground profile against the
        // DEM and then handing the camera a heading that points the other way would be
        // §7c's mistake with a different cause, so the heading is measured off the built
        // camera rather than assumed from the field name.
        let p = pose.view(ground);
        let cam = build_camera(&p);
        let (cam_eye, ori) = cam.global_transform_f64();
        // The camera's forward axis is −Z in its own frame (right-handed look-at).
        let fwd = ori * DVec3::NEG_Z;
        let u = normal_at(cam_eye);
        let h = (fwd - u * fwd.dot(u)).normalize();
        let east = {
            let m = (u.x * u.x + u.z * u.z).sqrt();
            DVec3::new(u.z / m, 0.0, -u.x / m)
        };
        let north = u.cross(east).normalize();
        let heading = h.dot(east).atan2(h.dot(north)).to_degrees();
        println!(
            "    camera heading measured {:.1} deg, pitch {:.2} deg below horizontal",
            wrap_deg(heading),
            -fwd.dot(u).asin().to_degrees()
        );
        assert!(
            wrap_deg(heading - pose.bearing_deg).abs() < 1.0,
            "{}: the camera looks along {:.1} deg, the profile was taken along {:.1} deg",
            pose.name,
            wrap_deg(heading),
            pose.bearing_deg
        );
    }
}

/// **FN = 0 at the two step poses, against the DEM itself.**
///
/// `test_terrain_occlusion::d3_never_hides_a_visible_vertex` is D3's acceptance and it
/// runs on the synthetic ridge world: one Gaussian crest with a col in it. That world has
/// no escarpment in it, and an escarpment is the shape §7d found D3 does nothing about —
/// so when `ridge_safety_m` stopped charging a 250 km headroom at 2 km and the stage
/// started culling here, the sweep that proves it sound was the one sweep that does not
/// visit this shape.
///
/// This is that proof, at these two poses, with the same criterion and none of the same
/// machinery:
///
/// > A node D3 culled is a false negative if any vertex of its own mesh is inside the
/// > frustum (exact `f64`), off the ellipsoid limb (Theorem 3.1, exact), **and** not
/// > blocked by the real DEM on the straight segment from the eye — [`dem_blocks`],
/// > which marches the terrarium tiles at z14 and shaves [`ORACLE_SLACK_M`] off the
/// > terrain first, so every approximation in it pushes toward calling a vertex *visible*.
///
/// **Only nodes the stage itself removed are scored.** A node the frustum or the limb
/// culled is not D3's answer and counting it would bury the signal; `TerrainHorizon::occludes`
/// is asked directly, which is the same attribution [`terrain_step_plateau_tiles`] makes.
///
/// The fetch policy is `Fill::Visible` — the production one — because that is the arm
/// where the culls happen. A culled node requests nothing, so its own heights are filled
/// in **after** the tree has settled, purely to build the mesh whose vertices are scored;
/// the tree under test is untouched by that.
#[test]
#[ignore = "measurement, needs the network for real height tiles"]
fn terrain_step_d3_never_hides_a_visible_vertex() {
    let cfg = TerrainOcclusionConfig::default();
    let mut world = RealWorld::new();
    const FRAMES: usize = 8;

    let mut false_negatives = 0usize;
    let mut scored_nodes = 0usize;
    let mut scored_vertices = 0usize;
    let mut examples: Vec<String> = Vec::new();

    for pose in step_poses() {
        warm_dem(&mut world, &pose);
        let ground = dem_height_m(&mut world, pose.lon, pose.lat);
        let p = pose.view(ground);
        let frustum = frustum_for(&p);
        let cam = build_camera(&p);
        let oracle = VisibilityOracle::new(&cam, p.aspect());
        let limb = HorizonCamera::new(frustum.eye);
        let eye = frustum.eye;

        let mut d3 = settle(&p, &frustum, Some(cfg), Fill::Visible, FRAMES, &mut world);
        let Some(horizon) = d3.qt.terrain_horizon().cloned() else {
            panic!(
                "{}: the march is inactive, so this pose proves nothing",
                pose.name
            );
        };

        // Every node the traversal touched and removed, with its own box.
        let mut culled: Vec<(TileId, cesium_engine::globe::quadtree::OrientedBoundingBox)> =
            Vec::new();
        fn walk(
            node: &QuadtreeNode<Heightfield>,
            out: &mut Vec<(TileId, cesium_engine::globe::quadtree::OrientedBoundingBox)>,
        ) {
            if !node.visible {
                out.push((node.id, node.obb));
                return;
            }
            if let Some(children) = &node.children {
                for c in children.iter() {
                    walk(c, out);
                }
            }
        }
        for root in d3.qt.roots.iter() {
            walk(root, &mut culled);
        }

        // Only the ones **this stage** removed.
        let d3_culled: Vec<TileId> = culled
            .into_iter()
            .filter(|(id, obb)| horizon.occludes(obb, &tile_bounds(id)))
            .map(|(id, _)| id)
            .collect();

        // Heights for the meshes, after the fact — a culled node asks for nothing, so
        // without this its mesh would be built off an ancestor's texels and the vertices
        // scored would not be the ones the engine would have drawn.
        let mut wanted = Vec::new();
        for id in &d3_culled {
            let mut curr = d3.heights.source_tile_for(*id);
            loop {
                wanted.push(curr);
                match curr.parent() {
                    Some(q) => curr = q,
                    None => break,
                }
            }
        }
        wanted.sort_unstable_by_key(|i| (i.z, i.x, i.y));
        wanted.dedup();
        fetch_missing(&world.dir, &wanted);
        for id in &d3_culled {
            fill_chain(*id, &mut d3.heights, &mut world);
        }

        let mut pose_fn = 0usize;
        let mut pose_vertices = 0usize;
        for id in &d3_culled {
            let Ok(patch) = HeightPatch::sample(&mut d3.heights, *id, SEGMENTS, 1.0) else {
                continue;
            };
            let mesh = TileMesh::generate_on::<Heightfield>(id, SEGMENTS, &patch);
            let centre = DVec3::from_array(mesh.center_f64);
            scored_nodes += 1;
            for v in mesh.vertices.iter() {
                let q = centre
                    + DVec3::new(
                        v.position[0] as f64,
                        v.position[1] as f64,
                        v.position[2] as f64,
                    );
                // 1. On screen?
                let c = oracle.clip(q);
                if c.w <= 0.0 {
                    continue;
                }
                let ndc = c.truncate() / c.w;
                let out = (ndc.x.abs() - 1.0)
                    .max(ndc.y.abs() - 1.0)
                    .max(-ndc.z)
                    .max(ndc.z - 1.0);
                if out > -NDC_MARGIN {
                    continue;
                }
                // 2. Off the limb?
                let s = transform_to_scaled_space(q);
                if limb.active {
                    let h2 = limb.c2 - 1.0;
                    let d = s - limb.c;
                    let t = limb.c2 - s.dot(limb.c);
                    if t > h2 && t * t > h2 * d.dot(d) {
                        continue;
                    }
                }
                pose_vertices += 1;
                // 3. And the DEM does not block the way to it.
                if !dem_blocks(&mut world, eye, q) {
                    pose_fn += 1;
                    if examples.len() < 20 {
                        let (lon, lat, alt) = geodetic_of(q);
                        examples.push(format!(
                            "{}: z{} {}/{} culled, vertex at {lon:.4} {lat:.4} {:.0} m is on \
                             screen, off the limb and in front of the DEM",
                            pose.name,
                            id.z,
                            id.x,
                            id.y,
                            alt * 1.0e6
                        ));
                    }
                }
            }
        }
        println!(
            "  [{}] D3 removed {} nodes; {pose_vertices} of their vertices are on screen and \
             off the limb; false negatives {pose_fn}",
            pose.name,
            d3_culled.len()
        );
        false_negatives += pose_fn;
        scored_vertices += pose_vertices;
    }

    println!(
        "  [step FN] {scored_nodes} D3-culled nodes meshed, {scored_vertices} candidate \
         vertices, false negatives {false_negatives}"
    );
    assert!(
        scored_nodes > 0,
        "D3 removed nothing at either pose, so this test asserted nothing"
    );
    // **And it has to have had something to score.** A pose can remove nodes whose every
    // vertex is off-screen or behind the limb — Reutlingen does exactly that — and a
    // `false_negatives == 0` built only out of those is a green light for nothing. This is
    // the line that says the two poses together actually put the DEM oracle to work.
    assert!(
        scored_vertices > 0,
        "{scored_nodes} nodes were meshed but not one of their vertices is on screen and \
         off the limb, so the DEM oracle was never asked anything"
    );
    assert_eq!(
        false_negatives,
        0,
        "D3 hid geometry the DEM says is visible:\n  {}",
        examples.join("\n  ")
    );
}

/// **What closing it would take, priced in march resolution rather than in code.**
///
/// [`terrain_step_plateau_tiles`] shows the occludee side is already tight — a node's box
/// top sits within ten metres of the real ground — and that what fails is the *occluder*:
/// the march's guaranteed ridge never gets near the real one. The occluder is a
/// **minimum over a (sector × ring) cell**, and on an escarpment every cell that contains
/// the crest also contains the slope under it, so the minimum is the foot of the step.
/// The only lever that changes that is cell size.
///
/// This prices the lever without building it. For a grid of `s` azimuth sectors and a
/// ring ratio `r` it recomputes exactly what `TerrainHorizon::finish` computes — the
/// running maximum over rings of the elevation angle of a wall at the cell's far edge,
/// standing at the cell's minimum DEM height less [`ridge_safety_m`] — but reads the cell
/// minimum from the **DEM directly**, which is the tightest floor any occluder could ever
/// have. So the peak it reports is the *ceiling* of the whole approach at that resolution,
/// not the ceiling of one implementation of it, and the engine needs no change to run it.
///
/// The number to compare against is `theta_box` of the tiles
/// [`terrain_step_plateau_tiles`] finds hidden and kept, which that test prints.
#[test]
#[ignore = "measurement, needs the network for real height tiles"]
fn terrain_step_occluder_resolution_probe() {
    /// Sample points per cell, per axis — the cell minimum is a minimum, so a coarse
    /// sampling *over*-estimates it, i.e. flatters the finer grids. Stated because it is
    /// the direction that could manufacture a positive result.
    const CELL_SAMPLES: usize = 8;
    let mut world = RealWorld::new();

    for pose in step_poses() {
        warm_dem(&mut world, &pose);
        let ground = dem_height_m(&mut world, pose.lon, pose.lat);
        let eye_m = ground + pose.eye_agl_m;
        let eye = ecef(pose.lon, pose.lat, eye_m);
        let up = normal_at(eye);
        println!(
            "\n  [{}] the ceiling of a footprint-minimum occluder, by cell size",
            pose.name
        );
        println!(
            "    {:>8} {:>8} {:>10} {:>10} {:>12} {:>14} {:>14} {:>14}",
            "sectors",
            "rings",
            "cell @2km",
            "cell @8km",
            "peak ridge",
            "peak, no safety",
            "peak, near edge",
            "vs the real one"
        );
        // The real ridge along this bearing, for the last column.
        let mut real = f64::NEG_INFINITY;
        for i in 1..=600 {
            let d = i as f64 * 100.0;
            let (lo, la) = dest(pose.lon, pose.lat, pose.bearing_deg, d);
            let h = dem_height_m(&mut world, lo, la);
            real = real.max(elevation_deg(eye, up, ecef(lo, la, h)));
        }

        for (sectors, rings) in [
            (AZIMUTH_SECTORS, 48),
            (AZIMUTH_SECTORS * 2, 48),
            (AZIMUTH_SECTORS * 4, 96),
            (AZIMUTH_SECTORS * 8, 192),
            (AZIMUTH_SECTORS * 16, 384),
        ] {
            let sector_w = 360.0 / sectors as f64;
            let near = MIN_RANGE_M;
            let far = 120_000.0_f64;
            let step = (far / near).powf(1.0 / (rings - 1) as f64);

            let mut run = f64::NEG_INFINITY;
            let mut peak = f64::NEG_INFINITY;
            // Same grid with the safety distance set to zero, and again with the wall at
            // the cell's **near** edge. Both are diagnostics, not proposals: the second is
            // unsound for the reason `TerrainHorizon::stamp` records in full. Together
            // they split the shortfall into the part the safety distance costs, the part
            // the far-edge placement costs, and the part that is the cell minimum itself.
            // Since the distance became range- and stand-off-proportional the first two
            // columns have nearly converged, which is the whole of what that change
            // bought.
            let mut run_ns = f64::NEG_INFINITY;
            let mut peak_ns = f64::NEG_INFINITY;
            let mut run_ne = f64::NEG_INFINITY;
            let mut peak_ne = f64::NEG_INFINITY;
            let mut r0 = 0.0;
            let mut r1 = near;
            for _ in 0..rings {
                // Minimum of the DEM over the cell the pose bearing falls in.
                let mut floor = f64::INFINITY;
                for iy in 0..CELL_SAMPLES {
                    for ix in 0..CELL_SAMPLES {
                        let fb = (ix as f64 + 0.5) / CELL_SAMPLES as f64 - 0.5;
                        let fr = (iy as f64 + 0.5) / CELL_SAMPLES as f64;
                        let b = pose.bearing_deg + fb * sector_w;
                        let d = r0 + fr * (r1 - r0);
                        let (lo, la) = dest(pose.lon, pose.lat, b, d.max(1.0));
                        floor = floor.min(dem_height_m(&mut world, lo, la));
                    }
                }
                // The wall stands at the cell's far edge, as `finish` places it.
                let (lo, la) = dest(pose.lon, pose.lat, pose.bearing_deg, r1);
                let safety = ridge_safety_m(r1, eye_m - floor);
                let e = elevation_deg(eye, up, ecef(lo, la, floor - safety));
                run = run.max(e);
                peak = peak.max(run);
                let e_ns = elevation_deg(eye, up, ecef(lo, la, floor));
                run_ns = run_ns.max(e_ns);
                peak_ns = peak_ns.max(run_ns);
                // Ring 0's near edge is the camera's own feet, where the elevation angle to
                // anything is meaningless — [`MIN_RANGE_M`] exists for that reason. Skip it
                // rather than print the +16° it produces.
                if r0 >= MIN_RANGE_M {
                    let (lon_n, lat_n) = dest(pose.lon, pose.lat, pose.bearing_deg, r0);
                    let e_ne = elevation_deg(eye, up, ecef(lon_n, lat_n, floor));
                    run_ne = run_ne.max(e_ne);
                    peak_ne = peak_ne.max(run_ne);
                }
                r0 = r1;
                r1 *= step;
                if r0 > ORACLE_FAN_M {
                    break;
                }
            }
            println!(
                "    {sectors:>8} {rings:>8} {:>9.0}m {:>9.0}m {:>11.3}d {:>13.3}d \
                 {:>13.3}d {:>13.3}d",
                2_000.0 * sector_w.to_radians(),
                8_000.0 * sector_w.to_radians(),
                peak,
                peak_ns,
                peak_ne,
                real
            );
        }
        println!(
            "    the real ridge along this bearing is {:.3} deg; a cell minimum can only \
             reach it once the cell no longer straddles the slope",
            real
        );
    }
}

/// **The measurement**: how many tiles beyond the step are drawn, how many of those the
/// DEM hides outright, and what stops the stage removing them.
///
/// Run under both height-fetch policies, because the answer is not the same under both
/// and the difference is the finding — see [`Fill`].
#[test]
#[ignore = "measurement, needs the network for real height tiles"]
fn terrain_step_plateau_tiles() {
    let cfg = TerrainOcclusionConfig::default();
    let mut world = RealWorld::new();
    const FRAMES: usize = 8;

    for pose in step_poses() {
        warm_dem(&mut world, &pose);
        let ground = dem_height_m(&mut world, pose.lon, pose.lat);
        let p = pose.view(ground);
        let frustum = frustum_for(&p);
        let eye = frustum.eye;
        let up = normal_at(eye);
        println!(
            "\n  [{}] eye {:.1} m over {:.1} m of ground, bearing {:.0} deg — {}",
            pose.name, p.alt_m, ground, pose.bearing_deg, pose.what
        );

        for fill in [Fill::Visible, Fill::Everything] {
            let reference = settle(&p, &frustum, None, fill, FRAMES, &mut world);
            let d3 = settle(&p, &frustum, Some(cfg), fill, FRAMES, &mut world);

            println!(
                "    fill = {fill:?}: D1+D2 {} tiles, +D3 {} tiles ({:+.1} %)",
                reference.trace.last().unwrap().0,
                d3.trace.last().unwrap().0,
                100.0
                    * (d3.trace.last().unwrap().0 as f64
                        / reference.trace.last().unwrap().0.max(1) as f64
                        - 1.0)
            );
            println!(
                "      {:>5} {:>9} {:>9} {:>11} {:>9} {:>9} {:>11}",
                "frame", "vis D1D2", "heights", "inherited", "vis +D3", "heights", "inherited"
            );
            for i in 0..FRAMES {
                let (v0, h0, i0) = reference.trace[i];
                let (v1, h1, i1) = d3.trace[i];
                println!(
                    "      {:>5} {v0:>9} {h0:>9} {i0:>11} {v1:>9} {h1:>9} {i1:>11}",
                    i + 1
                );
            }

            let cands = candidates(&reference, &d3, eye, up, &pose, &mut world);
            let scored: Vec<&Candidate> = cands.iter().filter(|c| c.scored).collect();
            let hidden: Vec<&Candidate> = cands.iter().filter(|c| c.dem_hidden).collect();
            let removed = hidden.iter().filter(|c| c.d3_culls).count();
            println!(
                "      beyond the step: {} visible tiles ({} scored, {} reaching past the \
                 oracle's {:.0} km fan), {} of the scored hidden by the DEM, D3 removes {}",
                cands.len(),
                scored.len(),
                cands.len() - scored.len(),
                ORACLE_FAN_M / 1_000.0,
                hidden.len(),
                removed
            );

            let ceiling = d3
                .qt
                .terrain_horizon()
                .map(|h| ridge_ceiling_deg(h, &cfg))
                .unwrap_or(f64::NAN);
            println!(
                "      march: active = {}, guaranteed ridge ceiling {:.3} deg",
                d3.qt
                    .terrain_horizon()
                    .map(|h| h.is_active())
                    .unwrap_or(false),
                ceiling
            );
            if let Some(h) = d3.qt.terrain_horizon() {
                ridge_profile(h, &mut world, eye, up, &pose);
            }
            crest_occluder(&d3, &mut world, &pose, eye, up);
            println!(
                "      {:>16} {:>3} {:>7} {:>9} {:>10} {:>10} {:>9} {:>9} {:>9} {:>7} {:>5}",
                "tile",
                "z",
                "km",
                "true top",
                "box lo",
                "box hi",
                "th_true",
                "th_box",
                "dem marg",
                "ready",
                "cull"
            );
            for c in hidden.iter().take(24) {
                println!(
                    "      {:>16} {:>3} {:>7.1} {:>9.0} {:>10.0} {:>10.0} {:>9.3} {:>9.3} \
                     {:>9.3} {:>7} {:>5}",
                    format!("{}/{}/{}", c.id.z, c.id.x, c.id.y),
                    c.id.z,
                    c.near_km,
                    c.true_top_m,
                    c.box_lo_m,
                    c.box_hi_m,
                    c.theta_true_deg,
                    c.theta_box_deg,
                    c.dem_margin_deg,
                    c.ready,
                    c.d3_culls
                );
            }

            // The blame, as one line per surviving tile: which of the three bounds is
            // the one that does not clear.
            for c in hidden.iter().filter(|c| !c.d3_culls).take(12) {
                let margin_m = c.box_hi_m - c.true_top_m;
                let level_margin_m = inherit_margin_mm(c.id.z) * 1.0e6;
                let reason = if c.theta_box_deg >= ceiling {
                    "box top clears the march's whole ridge ceiling"
                } else {
                    "box top is under the ceiling; the sector/ring lookup is what fails"
                };
                println!(
                    "      why {:>14}: box hi is {:+.0} m over the real top \
                     (level margin {:.0} m, own data {}), th_box {:.3} vs ceiling {:.3} — {}",
                    format!("{}/{}/{}", c.id.z, c.id.x, c.id.y),
                    margin_m,
                    level_margin_m,
                    if c.ready { "yes" } else { "NO" },
                    c.theta_box_deg,
                    ceiling,
                    reason
                );
            }
        }
    }
}
