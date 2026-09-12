//! Measurement engine: run one camera cell through the real quadtree and score it
//! against the f64 oracle.
//!
//! # Definitions (these are the contract the whole harness is built on)
//!
//! * **False negative (FN)** — a sample point that the oracle says is
//!   unambiguously visible, but which **no tile in the visible set covers**. This
//!   is a hole in the render: the user is looking at a patch of Earth and nothing
//!   was scheduled to draw it. This is the defect that matters.
//!
//! * **False positive (FP)** — a tile in the visible set into which **no visible
//!   sample point falls**, i.e. the tile is entirely off-screen or entirely
//!   back-facing but was still scheduled. This is wasted work, not a visual bug.
//!   Bounding volumes are conservative by construction, so a non-zero FP rate is
//!   expected and is measured as a *rate*, never asserted to be zero.
//!
//! * **Marginal** — a sample within the oracle's numeric no-man's-land (see
//!   [`super::oracle`]). Excluded from both tallies, counted and reported.
//!
//! # Sampling
//!
//! Two independent sample sources feed the FN metric:
//!
//! 1. **Viewport grid.** An NDC grid is unprojected in f64 through the inverse of
//!    the camera's own view-projection matrix and intersected with the ellipsoid.
//!    This concentrates samples exactly where the camera is looking, so it stays
//!    meaningful at 10 m altitude where a geodetic grid would place zero points in
//!    view. It deliberately does **not** use `Camera::screen_to_world_ray`, which
//!    hardcodes a 45° FOV and is wrong at the screen edges.
//!
//! 2. **Geodetic grid.** A global lat/lon grid, filtered by the oracle. This is an
//!    entirely different code path from the unprojection, so a bug in one does not
//!    silently disable the FN metric, and it is what actually exercises the
//!    "camera sees most of the globe" regime.
//!
//! The FP metric never uses either grid: it samples *inside each visible tile*
//! (see [`super::geodesy::tile_sample_points`]), which makes it independent of
//! grid density — essential at zoom 18-20 where a global grid would never land a
//! point inside a tile.

use std::collections::HashSet;
use std::sync::OnceLock;

use cesium_engine::globe::quadtree::{Frustum, QuadtreeManager, TileId};
use glam::DVec3;
use rayon::prelude::*;

use super::cameras::{build_camera, ViewParams};
use super::geodesy::{dvec3_to_lat_lon, lon_lat_to_ecef, tile_sample_points, MERCATOR_LIMIT_DEG};
use super::oracle::{Verdict, VisibilityOracle};

// ─────────────────────────────────────────────────────────────────────────────
// Thread pool
// ─────────────────────────────────────────────────────────────────────────────

/// The harness's own rayon pool.
///
/// **Why not the global pool.** libtest runs test *functions* concurrently on its
/// own thread pool (as wide as the machine unless `--test-threads` says otherwise),
/// and rayon's global pool is independently sized to the machine too. Several
/// heavy sweeps running at once would therefore each try to fan out across every
/// core, and the resulting oversubscription costs more than the parallelism buys.
///
/// Every parallel section in this module runs inside this one pool via
/// `install`, so the harness's total width is bounded no matter how many sweep
/// tests libtest decides to run simultaneously. Nested `par_iter` inside the pool
/// is fine and intentional — rayon work-steals across the same worker threads
/// rather than spawning more — which lets a single-cell measurement still use the
/// whole machine.
///
/// Width defaults to `available_parallelism()` and can be overridden with
/// `CESIUM_CULLING_THREADS` for A/B timing runs.
pub fn harness_pool() -> &'static rayon::ThreadPool {
    static POOL: OnceLock<rayon::ThreadPool> = OnceLock::new();
    POOL.get_or_init(|| {
        let threads = std::env::var("CESIUM_CULLING_THREADS")
            .ok()
            .and_then(|v| v.parse::<usize>().ok())
            .filter(|n| *n > 0)
            .unwrap_or_else(|| {
                std::thread::available_parallelism()
                    .map(|n| n.get())
                    .unwrap_or(8)
            });
        rayon::ThreadPoolBuilder::new()
            .num_threads(threads)
            .thread_name(|i| format!("culling-harness-{i}"))
            .build()
            .expect("failed to build the culling harness thread pool")
    })
}

/// How many times `QuadtreeManager::update` is called before the visible set is read.
///
/// `QuadtreeNode::update` recurses into freshly created children within the same
/// call, so the tree reaches full depth after a single update. More than one
/// update matters only for the LOD hysteresis band
/// (`collapse_dist = subdivide_dist * 1.20`): once a node is subdivided it is
/// judged against the wider collapse distance, so the second update can keep nodes
/// the first one created. With a static camera the set is a fixed point from
/// update 2 onward; 4 is used for margin, and
/// `test_update_iterations_reach_fixed_point` asserts that the set stops changing
/// by then. (The superseded tests looped 30× without ever saying why.)
pub const UPDATE_ITERATIONS: usize = 4;

/// Viewport sample grid resolution (columns × rows) for the FN metric.
///
/// 257 × 145 ≈ 37 000 rays per cell: roughly one sample per 7.5 × 7.5 px block of
/// a 1920 × 1080 viewport, so a hole an eighth of the screen wide in either axis
/// cannot slip between samples. Sized for a many-core machine, not a laptop —
/// see [`harness_pool`].
pub const NDC_GRID_COLS: u32 = 257;
pub const NDC_GRID_ROWS: u32 = 145;

/// Geodetic sample grid step, in degrees, for the FN metric.
///
/// 0.5° ≈ 55 km at the equator, giving 361 × 720 ≈ 260 000 points per cell. Fine
/// enough to resolve the sub-degree limb band the defect probes care about.
pub const GEO_GRID_STEP_DEG: f64 = 0.5;

/// Per-tile sample grid used by the FP metric (`(N+1)²` points per tile).
pub const TILE_SAMPLE_STEPS: u32 = 4;

/// Hard cap on retained per-cell false-negative records.
///
/// Full per-cell records are kept rather than streamed into counters — RAM is the
/// cheap resource here and the records are what make the cluster maps useful. The
/// cap only exists so a pathologically broken build (every sample an FN) cannot
/// balloon to tens of gigabytes; truncation is reported.
pub const MAX_FN_RECORDS_PER_CELL: usize = 50_000;

/// One recorded false negative, with enough context to reproduce it by hand.
#[derive(Clone, Debug)]
pub struct FalseNegative {
    pub lat: f64,
    pub lon: f64,
    /// Which sample source found it: "viewport" or "geodetic".
    pub source: &'static str,
    /// Cosine of the limb angle; ~1 means dead centre of the visible disc.
    pub facing_cos: f64,
    pub ndc_x: f64,
    pub ndc_y: f64,
}

impl FalseNegative {
    /// Angular distance inside the visible limb, in degrees. 0° is exactly on the
    /// limb, 90° is directly beneath the camera.
    pub fn limb_deg(&self) -> f64 {
        self.facing_cos.clamp(-1.0, 1.0).asin().to_degrees()
    }
}

/// Index into [`LIMB_BUCKET_EDGES_DEG`] for a limb angle.
pub fn limb_bucket(deg: f64) -> usize {
    LIMB_BUCKET_EDGES_DEG
        .iter()
        .position(|e| deg <= *e)
        .unwrap_or(LIMB_BUCKET_EDGES_DEG.len() - 1)
}

/// Everything measured for one camera cell.
#[derive(Clone, Debug)]
pub struct CellResult {
    pub params: ViewParams,
    pub tiles: usize,
    pub min_z: u8,
    pub max_z: u8,
    pub samples_considered: usize,
    pub samples_visible: usize,
    pub samples_marginal: usize,
    pub false_negatives: usize,
    /// Every false negative found, up to [`MAX_FN_RECORDS_PER_CELL`].
    pub fn_records: Vec<FalseNegative>,
    /// True when `false_negatives` exceeded the record cap.
    pub fn_records_truncated: bool,
    /// False negatives bucketed by distance from the visible limb, using
    /// [`LIMB_BUCKET_EDGES_DEG`]. Lets the CSV say *where* the holes are without
    /// carrying every record.
    pub fn_limb_buckets: [usize; LIMB_BUCKET_EDGES_DEG.len()],
    /// Tiles with no visible interior sample at all.
    pub false_positive_tiles: usize,
    /// Tiles whose interior samples were all `Marginal` — scored neither way.
    pub marginal_tiles: usize,
}

impl CellResult {
    /// FN as a fraction of unambiguously-visible samples. 0.0 when nothing was visible.
    pub fn fn_rate(&self) -> f64 {
        if self.samples_visible == 0 {
            0.0
        } else {
            self.false_negatives as f64 / self.samples_visible as f64
        }
    }

    /// FP as a fraction of the visible tile set. 0.0 when the set is empty.
    ///
    /// Raw ratio, kept unfiltered so the CSV records what actually happened.
    /// Aggregate statistics must skip degenerate cells — see [`Self::is_degenerate`].
    pub fn fp_rate(&self) -> f64 {
        if self.tiles == 0 {
            0.0
        } else {
            self.false_positive_tiles as f64 / self.tiles as f64
        }
    }

    /// A cell in which the oracle finds no visible surface at all, so the
    /// false-positive ratio has no meaningful denominator.
    ///
    /// This is not a measurement artifact but exact geometry: for a camera at
    /// altitude 0 the front-face condition at any other surface point `p` is
    /// `n_p . (cam - p) = cos(gamma) - 1 <= 0`, with equality only at `p = cam`.
    /// Standing exactly on the ellipsoid, nothing else on it is visible; below
    /// the surface, likewise. Every tile the culler keeps there is counted as a
    /// false positive purely because no tile *could* be correct.
    ///
    /// Keeping tiles at such a pose is the conservative behaviour invariant I-6
    /// demands, so these cells are excluded from FP aggregates and reported
    /// separately. They remain fully subject to the false-negative check, which
    /// is the criterion that actually matters there.
    pub fn is_degenerate(&self) -> bool {
        self.samples_visible == 0
    }
}

/// Builds the visible tile set for a cell, exactly the way the renderer does.
pub fn visible_tiles_for(params: &ViewParams) -> (Vec<TileId>, VisibilityOracle) {
    let cam = build_camera(params);
    let aspect = params.aspect();
    let frustum_planes = cam.calculate_frustum_planes(aspect as f32);

    let (global_pos, _) = cam.global_transform_f64();
    let frustum = Frustum::new(frustum_planes, global_pos);

    let mut quadtree = QuadtreeManager::new();
    for _ in 0..UPDATE_ITERATIONS {
        quadtree.update(&frustum);
    }

    let tiles = quadtree
        .get_visible_tiles()
        .into_iter()
        .map(|(id, _, _)| id)
        .collect();

    (tiles, VisibilityOracle::new(&cam, aspect))
}

/// Measures a whole sweep, parallel across cells inside the harness pool.
///
/// This is what every sweep test should call: one `install`, one bounded pool,
/// cell-level and sample-level parallelism nested inside it. Full per-cell records
/// come back for aggregation at the end — nothing is accumulated lossily.
pub fn measure_cells(cells: &[ViewParams]) -> Vec<CellResult> {
    harness_pool().install(|| cells.par_iter().map(measure_cell_inner).collect())
}

/// Runs one cell end to end, inside the harness pool.
pub fn measure_cell(params: &ViewParams) -> CellResult {
    harness_pool().install(|| measure_cell_inner(params))
}

fn measure_cell_inner(params: &ViewParams) -> CellResult {
    let (tiles, oracle) = visible_tiles_for(params);

    let tile_set: HashSet<TileId> = tiles.iter().copied().collect();
    let mut zooms: Vec<u8> = tile_set.iter().map(|t| t.z).collect();
    zooms.sort_unstable();
    zooms.dedup();

    let min_z = zooms.first().copied().unwrap_or(0);
    let max_z = zooms.last().copied().unwrap_or(0);

    // ── FN: viewport-grid samples ────────────────────────────────────────────
    let mut ndc_samples: Vec<(f64, f64)> = Vec::new();
    for i in 0..NDC_GRID_COLS {
        let x = -1.0 + 2.0 * (i as f64 + 0.5) / NDC_GRID_COLS as f64;
        for j in 0..NDC_GRID_ROWS {
            let y = -1.0 + 2.0 * (j as f64 + 0.5) / NDC_GRID_ROWS as f64;
            ndc_samples.push((x, y));
        }
    }

    let viewport: Vec<SampleOutcome> = ndc_samples
        .par_iter()
        .map(|&(nx, ny)| match oracle.surface_point_at_ndc(nx, ny) {
            Some(p) => score_point(&oracle, &tile_set, &zooms, p, "viewport"),
            None => SampleOutcome::NotOnGlobe,
        })
        .collect();

    // ── FN: geodetic-grid samples ────────────────────────────────────────────
    let geo_points = geodetic_grid();
    let geodetic: Vec<SampleOutcome> = geo_points
        .par_iter()
        .map(|&p| score_point(&oracle, &tile_set, &zooms, p, "geodetic"))
        .collect();

    let mut samples_considered = 0usize;
    let mut samples_visible = 0usize;
    let mut samples_marginal = 0usize;
    let mut false_negatives = 0usize;
    let mut fn_records: Vec<FalseNegative> = Vec::new();
    let mut fn_records_truncated = false;
    let mut fn_limb_buckets = [0usize; LIMB_BUCKET_EDGES_DEG.len()];

    for outcome in viewport.into_iter().chain(geodetic.into_iter()) {
        match outcome {
            SampleOutcome::NotOnGlobe => {}
            SampleOutcome::Hidden => {
                samples_considered += 1;
            }
            SampleOutcome::Marginal => {
                samples_considered += 1;
                samples_marginal += 1;
            }
            SampleOutcome::Covered => {
                samples_considered += 1;
                samples_visible += 1;
            }
            SampleOutcome::Missed(fneg) => {
                samples_considered += 1;
                samples_visible += 1;
                false_negatives += 1;
                fn_limb_buckets[limb_bucket(fneg.limb_deg())] += 1;
                if fn_records.len() < MAX_FN_RECORDS_PER_CELL {
                    fn_records.push(fneg);
                } else {
                    fn_records_truncated = true;
                }
            }
        }
    }

    // ── FP: per-tile interior samples ────────────────────────────────────────
    let tile_verdicts: Vec<Verdict> = tiles
        .par_iter()
        .map(|tile| {
            let mut best = Verdict::Hidden;
            for p in tile_sample_points(tile, TILE_SAMPLE_STEPS) {
                match oracle.classify(p) {
                    Verdict::Visible => return Verdict::Visible,
                    Verdict::Marginal => best = Verdict::Marginal,
                    Verdict::Hidden => {}
                }
            }
            best
        })
        .collect();

    let false_positive_tiles = tile_verdicts
        .iter()
        .filter(|v| **v == Verdict::Hidden)
        .count();
    let marginal_tiles = tile_verdicts
        .iter()
        .filter(|v| **v == Verdict::Marginal)
        .count();

    CellResult {
        params: params.clone(),
        tiles: tiles.len(),
        min_z,
        max_z,
        samples_considered,
        samples_visible,
        samples_marginal,
        false_negatives,
        fn_records,
        fn_records_truncated,
        fn_limb_buckets,
        false_positive_tiles,
        marginal_tiles,
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Limb-band instrument
// ─────────────────────────────────────────────────────────────────────────────

/// Upper edges (degrees of limb angle) of the buckets used by [`measure_limb_band`].
///
/// "Limb angle" is `asin(facing_cos)`: 0° means the surface point sits exactly on
/// the visible edge of the disc, 90° means it is directly beneath the camera.
/// Tile-level horizon culling is at its least accurate near 0°, so the buckets are
/// packed there.
pub const LIMB_BUCKET_EDGES_DEG: [f64; 10] =
    [0.1, 0.25, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 45.0, 90.0];

/// Grid step, in degrees, for the limb-band sample grid.
///
/// 0.05° ≈ 5.5 km of ground distance: four times finer than the narrowest bucket
/// (0.1°), so the band's leading edge is resolved rather than estimated. That is
/// 3601 × 7200 ≈ 26 million points per cell — sized for a many-core machine, and
/// the single heaviest thing the harness does.
pub const LIMB_GRID_STEP_DEG: f64 = 0.05;

/// Where false negatives sit relative to the visible limb.
#[derive(Clone, Debug)]
pub struct LimbBandResult {
    pub params: ViewParams,
    /// `(bucket upper edge in degrees, visible samples, false negatives)`.
    pub buckets: Vec<(f64, usize, usize)>,
    /// Largest limb angle (degrees) at which any false negative was found.
    /// 0.0 when there were none. This is the *width of the broken band*.
    pub worst_fn_deg: f64,
    pub total_visible: usize,
    pub total_false_negatives: usize,
}

/// Measures false negatives as a function of distance from the visible limb.
///
/// This isolates the horizon/occlusion stage of `QuadtreeNode::update` from the
/// frustum stage: a hole that only ever appears within a fraction of a degree of
/// the limb is a horizon-culling conservatism, whereas one that reaches several
/// degrees inward is a frustum or LOD problem.
pub fn measure_limb_band(params: &ViewParams) -> LimbBandResult {
    harness_pool().install(|| measure_limb_band_inner(params))
}

/// Measures a set of limb-band cells, parallel across cells inside the harness pool.
pub fn measure_limb_bands(cells: &[ViewParams]) -> Vec<LimbBandResult> {
    harness_pool().install(|| cells.par_iter().map(measure_limb_band_inner).collect())
}

fn measure_limb_band_inner(params: &ViewParams) -> LimbBandResult {
    let (tiles, oracle) = visible_tiles_for(params);
    let tile_set: HashSet<TileId> = tiles.iter().copied().collect();
    let mut zooms: Vec<u8> = tile_set.iter().map(|t| t.z).collect();
    zooms.sort_unstable();
    zooms.dedup();

    // The grid is ~26M points, so it is generated row by row inside the parallel
    // reduction rather than materialised as a Vec of DVec3 (which would be ~600 MB
    // per cell for no benefit). One row per rayon task; per-row tallies are folded
    // together at the end.
    let lat_rows = ((180.0 / LIMB_GRID_STEP_DEG).round() as usize) + 1;
    let lon_cols = (360.0 / LIMB_GRID_STEP_DEG).round() as usize;

    type Tally = ([usize; LIMB_BUCKET_EDGES_DEG.len()], [usize; LIMB_BUCKET_EDGES_DEG.len()], f64, usize, usize);

    let identity: Tally = (
        [0; LIMB_BUCKET_EDGES_DEG.len()],
        [0; LIMB_BUCKET_EDGES_DEG.len()],
        0.0,
        0,
        0,
    );

    let (vis_buckets, fn_buckets, worst_fn_deg, total_visible, total_false_negatives) = (0
        ..lat_rows)
        .into_par_iter()
        .map(|row| {
            let lat = (-90.0 + row as f64 * LIMB_GRID_STEP_DEG).min(90.0);
            let mut t: Tally = identity;
            for col in 0..lon_cols {
                let lon = -180.0 + col as f64 * LIMB_GRID_STEP_DEG;
                let p = lon_lat_to_ecef(lon, lat);
                if oracle.classify(p) != Verdict::Visible {
                    continue;
                }
                let deg = oracle.facing_cos(p).clamp(-1.0, 1.0).asin().to_degrees();
                let (la, lo) = dvec3_to_lat_lon(p);
                let covered = zooms
                    .iter()
                    .any(|&z| tile_set.contains(&super::geodesy::tile_for_lat_lon(la, lo, z)));
                let idx = limb_bucket(deg);
                t.3 += 1;
                t.0[idx] += 1;
                if !covered {
                    t.1[idx] += 1;
                    t.4 += 1;
                    t.2 = t.2.max(deg);
                }
            }
            t
        })
        .reduce(
            || identity,
            |mut a, b| {
                for i in 0..LIMB_BUCKET_EDGES_DEG.len() {
                    a.0[i] += b.0[i];
                    a.1[i] += b.1[i];
                }
                a.2 = a.2.max(b.2);
                a.3 += b.3;
                a.4 += b.4;
                a
            },
        );

    let buckets: Vec<(f64, usize, usize)> = LIMB_BUCKET_EDGES_DEG
        .iter()
        .enumerate()
        .map(|(i, e)| (*e, vis_buckets[i], fn_buckets[i]))
        .collect();

    LimbBandResult {
        params: params.clone(),
        buckets,
        worst_fn_deg,
        total_visible,
        total_false_negatives,
    }
}

enum SampleOutcome {
    /// The viewport ray missed the ellipsoid entirely.
    NotOnGlobe,
    Hidden,
    Marginal,
    Covered,
    Missed(FalseNegative),
}

fn score_point(
    oracle: &VisibilityOracle,
    tile_set: &HashSet<TileId>,
    zooms: &[u8],
    p: DVec3,
    source: &'static str,
) -> SampleOutcome {
    match oracle.classify(p) {
        Verdict::Hidden => SampleOutcome::Hidden,
        Verdict::Marginal => SampleOutcome::Marginal,
        Verdict::Visible => {
            let (lat, lon) = dvec3_to_lat_lon(p);
            // A point is covered if *any* zoom level present in the visible set
            // owns it. Checking per-zoom rather than per-tile makes this O(#zooms)
            // instead of O(#tiles), and implicitly accepts coarse ancestors.
            for &z in zooms {
                if tile_set.contains(&super::geodesy::tile_for_lat_lon(lat, lon, z)) {
                    return SampleOutcome::Covered;
                }
            }
            let ndc = oracle.ndc(p).unwrap_or(DVec3::ZERO);
            SampleOutcome::Missed(FalseNegative {
                lat,
                lon,
                source,
                facing_cos: oracle.facing_cos(p),
                ndc_x: ndc.x,
                ndc_y: ndc.y,
            })
        }
    }
}

/// Global lat/lon sample grid, including both poles and the ±85.05° Mercator limit.
fn geodetic_grid() -> Vec<DVec3> {
    let mut lats: Vec<f64> = Vec::new();
    let mut lat: f64 = -90.0;
    while lat <= 90.0 + 1e-9 {
        lats.push(lat.min(90.0));
        lat += GEO_GRID_STEP_DEG;
    }
    // The Mercator truncation latitudes are where tile row 0 / row max begin their
    // stretch to the poles, and are the most likely place for a coverage hole.
    lats.push(MERCATOR_LIMIT_DEG);
    lats.push(-MERCATOR_LIMIT_DEG);
    lats.push(MERCATOR_LIMIT_DEG - 0.01);
    lats.push(-MERCATOR_LIMIT_DEG + 0.01);

    let mut lons: Vec<f64> = Vec::new();
    let mut lon: f64 = -180.0;
    while lon < 180.0 {
        lons.push(lon);
        lon += GEO_GRID_STEP_DEG;
    }
    // Straddle the antimeridian explicitly.
    lons.push(179.999);
    lons.push(-179.999);

    let mut out = Vec::with_capacity(lats.len() * lons.len());
    for &la in &lats {
        for &lo in &lons {
            out.push(lon_lat_to_ecef(lo, la));
        }
    }
    out
}
