//! Measurement engine: run one camera pose through the real quadtree, then score
//! every tile it decided to keep against the imagery-resolution metric.
//!
//! No independent oracle here, unlike [`super::super::culling::oracle`] — there is
//! no second, trusted definition of "the right LOD" to check against, which is
//! exactly why this instrument has to exist before `docs/pre-terrain-plan.md` WP3
//! touches the rule it measures.

use cesium_engine::camera::camera::Camera;
use cesium_engine::globe::geometry::lon_lat_to_ecef_f64;
use cesium_engine::globe::quadtree::{
    tile_bounds, web_mercator_y_to_lat_f64, CullPipeline, Frustum, LodDistanceMode,
    QuadtreeManager, TileBounds, TileId,
};
use glam::{DMat4, DVec3, DVec4};
use rayon::prelude::*;

use super::super::culling::bench_update::bench_cells;
use super::super::culling::cameras::{build_camera, ViewParams};
use super::super::culling::sweep::{harness_pool, UPDATE_ITERATIONS};

/// The engine's default imagery tile size — `STANDARD_IMAGERY_URL`'s `@2x` tiles
/// (`config.rs`). Deliberately a separate constant from the engine's own
/// `DEFAULT_IMAGERY_TEXTURE_SIZE_PX`: the harness must be able to state its
/// texel-density assumption independently of the engine's. Used as the *default*
/// `texture_size_px` in [`LodConfig`] and by [`measure_pose`]/[`measure_poses`];
/// every measurement function also takes an explicit `texture_size_px` (WP4/A,
/// `docs/pre-terrain-plan.md`) so `SATELLITE_IMAGERY_URL`'s 256px style can be
/// measured too — see [`ESRI_TEXTURE_SIZE_PX`].
pub const TEXTURE_SIZE_PX: u32 = 512;

/// `SATELLITE_IMAGERY_URL`'s tile size (`config.rs`) — the other real style this
/// engine serves, and the one WP4/A's live texture-size feed exists to stop
/// silently under-refining.
pub const ESRI_TEXTURE_SIZE_PX: f32 = 256.0;

/// The knobs `measure_pose_with_config` reads, bundled so a sweep can vary one,
/// several, or none without every call site growing another positional argument.
///
/// `lod_texture_size_px` and `real_texture_size_px` are split on purpose, not
/// merged into one field: `lod_texture_size_px` is what feeds `lod_factor_for` (what
/// the LOD rule *assumes* the texture size is), `real_texture_size_px` is what this
/// harness's own `texels` metric counts (the tile's *actual* decoded size). Every
/// normal measurement uses [`LodConfig::new`], which sets both to the same value —
/// that equality is WP4/A's fix (`docs/pre-terrain-plan.md`): before it, the engine's
/// LOD rule always assumed `DEFAULT_IMAGERY_TEXTURE_SIZE_PX` (512) regardless of the
/// real decoded style. [`LodConfig::uncompensated`] sets them independently, purely
/// to reproduce that historical gap for measurement — see
/// `docs/culling-baseline.md`'s WP4/A section for what it found.
#[derive(Clone, Copy, Debug)]
pub struct LodConfig {
    pub target_texel_ratio: f32,
    pub lod_texture_size_px: f32,
    pub real_texture_size_px: f32,
    /// WP4/C measurement switch — see [`LodDistanceMode`]. `Centre` (the default)
    /// in every normal measurement; `Box` only via [`LodConfig::with_distance_mode`],
    /// for the 3a-at-equal-tile-budget comparison.
    pub lod_distance_mode: LodDistanceMode,
}

impl LodConfig {
    pub fn new(target_texel_ratio: f32, texture_size_px: f32) -> Self {
        Self {
            target_texel_ratio,
            lod_texture_size_px: texture_size_px,
            real_texture_size_px: texture_size_px,
            lod_distance_mode: LodDistanceMode::default(),
        }
    }

    /// Reproduces the pre-WP4/A bug for measurement purposes only: `lod_factor_for`
    /// runs as if the texture were `assumed_texture_size_px` while the harness counts
    /// texels at the tile's `real_texture_size_px`. Not a mode the engine itself ever
    /// runs in post-WP4/A — see the struct doc comment.
    pub fn uncompensated(
        target_texel_ratio: f32,
        assumed_texture_size_px: f32,
        real_texture_size_px: f32,
    ) -> Self {
        Self {
            target_texel_ratio,
            lod_texture_size_px: assumed_texture_size_px,
            real_texture_size_px,
            lod_distance_mode: LodDistanceMode::default(),
        }
    }

    /// WP4/C only — see [`LodDistanceMode`].
    pub fn with_distance_mode(mut self, mode: LodDistanceMode) -> Self {
        self.lod_distance_mode = mode;
        self
    }
}

impl Default for LodConfig {
    /// Matches the engine's own shipped default (`target_texel_ratio = 1.0`,
    /// `texture_size_px = TEXTURE_SIZE_PX = 512`), the WP0-WP3 baseline in
    /// `docs/culling-baseline.md`.
    fn default() -> Self {
        Self::new(1.0, TEXTURE_SIZE_PX as f32)
    }
}

/// RGBA8, no mips — matches the byte accounting `config.rs`'s
/// `tile_cache_budget_bytes` doc comment already uses for the same tiles.
pub const BYTES_PER_TEXEL: u64 = 4;

/// A clip-space `w` at or below this is treated as "at or behind the eye" rather
/// than divided by, which is what a genuine near-zero or negative `w` would do to
/// a perspective divide.
const CLIP_W_EPS: f64 = 1e-9;

/// **The second metric** — a tile's geometric error, projected, in pixels.
///
/// `error · H / (dist · 2·tan(fovy/2))`: Cesium's screen-space error, evaluated on the
/// error this engine measures since E1 of `docs/terrain-plan.md` §8
/// (`SurfaceModel::geometric_error`, for `Heightfield` the tile's own deviation from the
/// DEM). It is the quantity `TerrainConfig::max_geometric_error_px` is a budget for, so a
/// sweep of that knob reads directly against it.
///
/// Lives here, in the instrument, rather than in either of the two places that call it:
/// `measure_pose_with_config` below, where it is **identically zero** because the tree is
/// an [`Ellipsoid`](cesium_engine::globe::quadtree::Ellipsoid) one, and
/// `testing::terrain::test_terrain_lod`, where it is not. One definition, two surfaces —
/// which is the whole reason this file states the metric instead of the module doc merely
/// claiming there is nothing to state.
pub fn geometric_error_px(error_mm: f64, dist_mm: f64, viewport_h: f64, fovy: f64) -> f64 {
    if dist_mm <= 0.0 {
        return f64::INFINITY;
    }
    error_mm * viewport_h / (dist_mm * 2.0 * (fovy * 0.5).tan())
}

/// The 204 poses the culling gate and `bench_update` already treat as
/// representative: the nadir ladder plus the zoom-cliff ladder. Reusing them means
/// a regression here and a regression in `bench_update`'s timings are always
/// measured on the same camera set.
pub fn bench_poses() -> Vec<ViewParams> {
    bench_cells()
}

/// One visible tile's imagery-resolution score.
#[derive(Clone, Debug)]
pub struct TileMetric {
    pub id: TileId,
    pub zoom: u8,
    /// Projected, viewport-clipped area of the drawn patch, in px². Zero when the
    /// patch clipped away entirely or every sample touching it was behind the eye.
    pub screen_px: f64,
    /// Area-weighted "effective" texel count: each of the four patch quads
    /// contributes `(texture_size_px² / 4) * (clipped_area / raw_area)` — the
    /// pose's own [`LodConfig::texture_size_px`], not always [`TEXTURE_SIZE_PX`]
    /// since WP4/A — its fair share of the tile's texel budget scaled by how much
    /// of *that quad's own* projected area actually landed onscreen. A quad
    /// skipped for being behind the eye contributes zero, same as a quad clipped
    /// away entirely — a tile clipped at the frustum boundary is not credited with
    /// texels for the part of the patch that never made it to screen. Equals the
    /// flat `texture_size_px²` only when every quad is fully onscreen and unclipped.
    pub texels: f64,
    /// `texels / screen_px`. `f64::INFINITY` when `screen_px == 0.0` — excluded
    /// from every ratio aggregate, counted separately (see [`super::report`]).
    pub ratio: f64,
    /// At least one of the four projected quads lost area to viewport clipping.
    pub partly_offscreen: bool,
    /// **The second metric**: this tile's geometric error, projected to pixels — see
    /// [`geometric_error_px`]. Zero for every tile this harness measures, because this
    /// harness measures the flat globe; not written to the CSVs for that reason, and
    /// checked rather than assumed by
    /// `test_lod_sweep::the_flat_globe_leaves_no_geometric_error_on_screen`.
    pub geom_err_px: f64,
    /// At least one of the patch's 9 samples had `w <= CLIP_W_EPS` in clip space.
    /// Any quad touching such a sample is excluded from `screen_px` rather than
    /// risking a perspective-divide blow-up, so this tile's `screen_px` may
    /// under-count its true projected area.
    pub behind_eye: bool,
}

impl TileMetric {
    pub fn has_screen_area(&self) -> bool {
        self.screen_px > 0.0
    }
}

/// Everything measured for one camera pose.
#[derive(Clone, Debug)]
pub struct PoseResult {
    pub params: ViewParams,
    pub tiles: Vec<TileMetric>,
    pub tile_count: usize,
    pub texture_bytes: u64,
    pub deepest_zoom: u8,
    /// The `texture_size_px` this pose was measured at (WP4/A) — carried per-result
    /// rather than read back out of a module constant, so a sweep that mixes styles
    /// (or a report rendering several sweeps side by side) can state honestly what
    /// each row assumed.
    pub texture_size_px: f32,
    /// No visible tile at all — a genuinely empty view (e.g. a sub-surface camera
    /// with nothing above the horizon), not a rendering defect on its own but a
    /// case the ratio aggregates must not silently absorb.
    pub degenerate: bool,
}

pub(crate) fn frustum_for(cam: &Camera, aspect: f32) -> Frustum {
    let planes = cam.calculate_frustum_planes(aspect);
    let (eye, _) = cam.global_transform_f64();
    Frustum::planes_only(planes, eye).with_corners(cam.frustum_corners_relative(aspect))
}

/// The tile's patch, sampled the same way `TileMesh::generate` builds the drawn
/// mesh (`crates/cesium-engine/src/globe/geometry.rs`): longitude linear in the
/// tile's own bounds, latitude from the shared Mercator-y definition — **not**
/// `fit_obb`'s plain latitude lerp, which only reproduces the true row positions at
/// `u ∈ {0, 1}`. `v` counts north → south, like the mesh's row index.
///
/// A 3×3 grid — `u, v ∈ {0, 0.5, 1}` — giving four quads to project and sum, per
/// `docs/pre-terrain-plan.md` WP1.
pub fn patch_grid_points(id: &TileId, b: &TileBounds) -> [[DVec3; 3]; 3] {
    const STEPS: [f64; 3] = [0.0, 0.5, 1.0];
    let max_y = (1_u32 << id.z) - 1;

    let mut pts = [[DVec3::ZERO; 3]; 3];
    for (vi, v) in STEPS.iter().enumerate() {
        let mut lat = web_mercator_y_to_lat_f64(id.y as f64 + v, id.z);
        if id.y == 0 && *v == 0.0 {
            lat = 90.0;
        }
        if id.y == max_y && *v == 1.0 {
            lat = -90.0;
        }
        for (ui, u) in STEPS.iter().enumerate() {
            let lon = b.lon_min + u * (b.lon_max - b.lon_min);
            let p = lon_lat_to_ecef_f64(lon, lat);
            pts[vi][ui] = DVec3::new(p[0], p[1], p[2]);
        }
    }
    pts
}

fn polygon_area(poly: &[(f64, f64)]) -> f64 {
    if poly.len() < 3 {
        return 0.0;
    }
    let mut sum = 0.0;
    for i in 0..poly.len() {
        let (x0, y0) = poly[i];
        let (x1, y1) = poly[(i + 1) % poly.len()];
        sum += x0 * y1 - x1 * y0;
    }
    (sum * 0.5).abs()
}

fn edge_intersection(a: (f64, f64), b: (f64, f64), nx: f64, ny: f64, offset: f64) -> (f64, f64) {
    let da = nx * a.0 + ny * a.1 - offset;
    let db = nx * b.0 + ny * b.1 - offset;
    let t = da / (da - db);
    (a.0 + t * (b.0 - a.0), a.1 + t * (b.1 - a.1))
}

/// Sutherland-Hodgman clip of a convex polygon against the viewport rectangle
/// `[0, width] x [0, height]`. Each plane is `(nx, ny, offset)` with "inside"
/// meaning `nx*x + ny*y <= offset`.
fn clip_to_viewport(poly: &[(f64, f64)], width: f64, height: f64) -> Vec<(f64, f64)> {
    const fn planes(width: f64, height: f64) -> [(f64, f64, f64); 4] {
        [
            (1.0, 0.0, width),
            (-1.0, 0.0, 0.0),
            (0.0, 1.0, height),
            (0.0, -1.0, 0.0),
        ]
    }

    let mut out = poly.to_vec();
    for &(nx, ny, offset) in &planes(width, height) {
        if out.is_empty() {
            break;
        }
        let input = std::mem::take(&mut out);
        out = Vec::with_capacity(input.len() + 1);
        for i in 0..input.len() {
            let curr = input[i];
            let prev = input[(i + input.len() - 1) % input.len()];
            let curr_in = nx * curr.0 + ny * curr.1 <= offset;
            let prev_in = nx * prev.0 + ny * prev.1 <= offset;
            if curr_in {
                if !prev_in {
                    out.push(edge_intersection(prev, curr, nx, ny, offset));
                }
                out.push(curr);
            } else if prev_in {
                out.push(edge_intersection(prev, curr, nx, ny, offset));
            }
        }
    }
    out
}

/// Projects the patch's 3x3 samples through `vp` and sums the four projected,
/// viewport-clipped quad areas.
///
/// `texels` is accumulated per quad as an area-weighted share of the tile's full
/// texel budget (`docs/pre-terrain-plan.md` WP1's follow-up fix): a quad that is
/// entirely onscreen contributes its full quarter-share, a quad half clipped away
/// contributes half that share, and a quad skipped outright (behind the eye, or
/// degenerate) contributes none — the same rule `screen_px` already follows, so a
/// tile that is mostly off-frustum no longer reports the full texel count against
/// a shrunken `screen_px` and inflates `ratio` upward.
pub(crate) fn project_patch(
    id: TileId,
    samples: &[[DVec3; 3]; 3],
    vp: &DMat4,
    width: f64,
    height: f64,
    texture_size_px: f64,
) -> TileMetric {
    let mut clip = [[DVec4::ZERO; 3]; 3];
    let mut behind_eye = false;
    for (vi, row) in samples.iter().enumerate() {
        for (ui, &p) in row.iter().enumerate() {
            let c = *vp * DVec4::new(p.x, p.y, p.z, 1.0);
            if c.w <= CLIP_W_EPS {
                behind_eye = true;
            }
            clip[vi][ui] = c;
        }
    }

    let mut screen_px = 0.0_f64;
    let mut partly_offscreen = false;
    let full_texels = texture_size_px * texture_size_px;
    let quad_texel_share = full_texels / 4.0;
    let mut effective_texels = 0.0_f64;

    for vi in 0..2 {
        for ui in 0..2 {
            let corners = [
                clip[vi][ui],
                clip[vi][ui + 1],
                clip[vi + 1][ui + 1],
                clip[vi + 1][ui],
            ];
            // A quad touching a behind-the-eye sample is excluded from screen_px
            // (and, by the same `continue`, from effective_texels) rather than
            // perspective-divided by a near-zero or negative w — see `behind_eye`
            // on TileMetric.
            if corners.iter().any(|c| c.w <= CLIP_W_EPS) {
                continue;
            }
            let poly: Vec<(f64, f64)> = corners
                .iter()
                .map(|c| {
                    let ndc_x = c.x / c.w;
                    let ndc_y = c.y / c.w;
                    (
                        (ndc_x * 0.5 + 0.5) * width,
                        (1.0 - (ndc_y * 0.5 + 0.5)) * height,
                    )
                })
                .collect();
            let raw_area = polygon_area(&poly);
            let clipped = clip_to_viewport(&poly, width, height);
            let clipped_area = polygon_area(&clipped);
            if clipped_area + 1e-6 < raw_area {
                partly_offscreen = true;
            }
            screen_px += clipped_area;

            // Guard against a degenerate (collapsed) quad rather than dividing by
            // ~0; clamp defensively since clipping can only shrink area, so the
            // fraction should never exceed 1.0 outside of float noise.
            if raw_area > 1e-9 {
                let onscreen_fraction = (clipped_area / raw_area).clamp(0.0, 1.0);
                effective_texels += quad_texel_share * onscreen_fraction;
            }
        }
    }

    let texels = effective_texels;
    let ratio = if screen_px > 0.0 {
        texels / screen_px
    } else {
        f64::INFINITY
    };

    TileMetric {
        id,
        zoom: id.z,
        screen_px,
        texels,
        ratio,
        // `Ellipsoid::HAS_GEOMETRIC_ERROR` is a compile-time `false` and
        // `Ellipsoid::geometric_error` is `0.0`: invariant I-1 says the drawn surface *is*
        // the ellipsoid, so the deviation between them is not small, it is zero. The
        // number is carried rather than omitted so the claim is a value a test can read.
        geom_err_px: 0.0,
        partly_offscreen,
        behind_eye,
    }
}

/// Builds the camera and frustum, runs the quadtree to its steady state, and
/// scores every tile it kept.
///
/// [`UPDATE_ITERATIONS`] updates, exactly as the culling sweep does: one update
/// alone reaches full tree depth, but the LOD hysteresis band
/// (`collapse_dist = subdivide_dist * 1.20`) needs the extra updates to settle,
/// and `test_update_iterations_reach_fixed_point` (in `super::super::culling`) is
/// what proves that count is enough.
pub fn measure_pose(p: &ViewParams) -> PoseResult {
    // The calibrated no-op default; no per-pose config to read.
    measure_pose_with_config(p, LodConfig::default())
}

/// As [`measure_pose`], but with an explicit `target_texel_ratio` (at the default
/// `texture_size_px`) — the WP3 follow-up fixes' own "verify by construction, not
/// by the default" check (`docs/pre-terrain-plan.md` WP3): running this at, say,
/// `4.0` and comparing the resulting `Summary::aggregate_ratio` against the
/// `target = 1.0` baseline exercises the whole pipeline (`lod_factor_for` →
/// `apply_lod` → this harness's own metric), not just the isolated formula.
pub fn measure_pose_with_target(p: &ViewParams, target_texel_ratio: f32) -> PoseResult {
    measure_pose_with_config(p, LodConfig::new(target_texel_ratio, TEXTURE_SIZE_PX as f32))
}

/// As [`measure_pose`], but with an explicit [`LodConfig`] — WP4/A
/// (`docs/pre-terrain-plan.md`): the texture size fed to `lod_factor_for` is no
/// longer implicitly [`TEXTURE_SIZE_PX`], so a sweep can measure
/// `SATELLITE_IMAGERY_URL`'s 256px style (via [`LodConfig::new`]), or reproduce the
/// pre-WP4/A engine behaviour via [`LodConfig::uncompensated`] — see
/// [`ESRI_TEXTURE_SIZE_PX`] and `docs/culling-baseline.md`'s WP4/A section.
pub fn measure_pose_with_config(p: &ViewParams, cfg: LodConfig) -> PoseResult {
    let cam = build_camera(p);
    let aspect = p.aspect();
    let frustum = frustum_for(&cam, aspect as f32);

    let mut qt = QuadtreeManager::new();
    qt.pipeline = CullPipeline::DEFAULT;
    qt.lod_distance_mode = cfg.lod_distance_mode;
    // The same derivation the real renderer runs per frame (`wgpu_state::update_logic`),
    // from the same shared function — not a copy of the arithmetic. At the 204 bench
    // poses (all `height = 1080`, all `mode = Free`) `LodConfig::default()` is
    // exactly 2.0, i.e. the old hard-coded value, so the WP0/WP1 baseline is
    // untouched. It matters once a sweep varies height, mode or texture size: without
    // it the harness would quietly stop measuring what the renderer does.
    qt.lod_factor = cesium_engine::globe::quadtree::lod_factor_for(
        cfg.target_texel_ratio,
        cfg.lod_texture_size_px,
        p.height as f32,
        cam.fovy(),
    );
    for _ in 0..UPDATE_ITERATIONS {
        qt.update(&frustum);
    }

    let visible = qt.get_visible_tiles();
    let vp = cam.get_projection_matrix_f64(aspect) * cam.get_view_matrix_f64();
    let width = p.width as f64;
    let height = p.height as f64;

    let mut tiles = Vec::with_capacity(visible.len());
    let mut deepest_zoom = 0_u8;
    for (id, _, _) in &visible {
        deepest_zoom = deepest_zoom.max(id.z);
        let bounds = tile_bounds(id);
        let samples = patch_grid_points(id, &bounds);
        tiles.push(project_patch(
            *id,
            &samples,
            &vp,
            width,
            height,
            cfg.real_texture_size_px as f64,
        ));
    }

    let tile_count = tiles.len();
    let texel_area = cfg.real_texture_size_px as u64 * cfg.real_texture_size_px as u64;
    let texture_bytes = tile_count as u64 * texel_area * BYTES_PER_TEXEL;

    PoseResult {
        params: p.clone(),
        degenerate: tile_count == 0,
        tiles,
        tile_count,
        texture_bytes,
        deepest_zoom,
        texture_size_px: cfg.real_texture_size_px,
    }
}

/// Measures every pose, inside the culling harness's own bounded rayon pool — see
/// [`super::super::culling::sweep::harness_pool`] for why the global pool is
/// deliberately not used.
pub fn measure_poses(poses: &[ViewParams]) -> Vec<PoseResult> {
    measure_poses_with_config(poses, LodConfig::default())
}

/// As [`measure_poses`], but at an explicit `target_texel_ratio` — see
/// [`measure_pose_with_target`].
pub fn measure_poses_with_target(poses: &[ViewParams], target_texel_ratio: f32) -> Vec<PoseResult> {
    measure_poses_with_config(
        poses,
        LodConfig::new(target_texel_ratio, TEXTURE_SIZE_PX as f32),
    )
}

/// As [`measure_poses`], but at an explicit [`LodConfig`] — see
/// [`measure_pose_with_config`].
pub fn measure_poses_with_config(poses: &[ViewParams], cfg: LodConfig) -> Vec<PoseResult> {
    harness_pool().install(|| {
        poses
            .par_iter()
            .map(|p| measure_pose_with_config(p, cfg))
            .collect()
    })
}

/// Binary search on `target_texel_ratio` for whatever `total_tiles_at` measures, so
/// its result lands as close as possible to `target_n` — WP4/C's "equal tile
/// budget, not equal `target_texel_ratio`" comparison
/// (`docs/pre-terrain-plan.md`), generalised so both the plain-harness and the
/// fog-aware (WP5/D) measurement paths can reuse the same search instead of each
/// carrying its own copy.
///
/// `target_texel_ratio` is continuous but tile count is a step function of it, so
/// bisection finds the closest *achievable* value, not an exact one — `total_tiles`
/// must be monotonic non-decreasing in `target_texel_ratio` over `[0.01, search_hi]`
/// for this to be meaningful (true of `lod_factor_for` post the WP3-follow-up fix:
/// higher target -> larger `lod_factor` -> equal or more subdivision).
pub fn bisect_target_for_tile_count(
    target_n: usize,
    search_hi: f32,
    total_tiles_at: impl Fn(f32) -> usize,
) -> (f32, usize) {
    let mut lo = 0.01_f32;
    let mut hi = search_hi;
    assert!(
        total_tiles_at(lo) <= target_n,
        "search_hi's lower bound must under-shoot target_n, or the bracket is wrong"
    );
    assert!(
        total_tiles_at(hi) >= target_n,
        "search_hi={search_hi} must over-shoot target_n={target_n}, widen the search range"
    );

    // 30 halvings of a [0.01, search_hi] bracket resolves target_texel_ratio to
    // better than 1e-8 — far finer than the tile-count step function can resolve,
    // so more iterations would not find a better answer.
    for _ in 0..30 {
        let mid = (lo + hi) * 0.5;
        if total_tiles_at(mid) < target_n {
            lo = mid;
        } else {
            hi = mid;
        }
    }

    // Compare both bracket ends' achieved tile counts and keep whichever is closer
    // to target_n, rather than assuming the last-moved bound is best.
    let n_lo = total_tiles_at(lo);
    let n_hi = total_tiles_at(hi);
    if n_lo.abs_diff(target_n) <= n_hi.abs_diff(target_n) {
        (lo, n_lo)
    } else {
        (hi, n_hi)
    }
}
