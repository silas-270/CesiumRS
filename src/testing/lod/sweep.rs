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
    tile_bounds, web_mercator_y_to_lat_f64, CullPipeline, Frustum, QuadtreeManager, TileBounds,
    TileId,
};
use glam::{DMat4, DVec3, DVec4};
use rayon::prelude::*;

use super::super::culling::bench_update::bench_cells;
use super::super::culling::cameras::{build_camera, ViewParams};
use super::super::culling::sweep::{harness_pool, UPDATE_ITERATIONS};

/// The engine's current default imagery tile size — `STANDARD_IMAGERY_URL`'s `@2x`
/// tiles (`config.rs`). Still frozen, and deliberately a separate constant from the
/// engine's own `DEFAULT_IMAGERY_TEXTURE_SIZE_PX`: the harness must be able to state
/// its texel-density assumption independently of the engine's. WP4 is what makes this
/// a real per-style input, fed from the texture manager rather than a constant.
///
/// (`lod_factor` is no longer frozen alongside it — WP3/3b derived it; see
/// [`cesium_engine::globe::quadtree::lod_factor_for`], which `measure_pose` calls.)
pub const TEXTURE_SIZE_PX: u32 = 512;

/// RGBA8, no mips — matches the byte accounting `config.rs`'s
/// `tile_cache_budget_bytes` doc comment already uses for the same tiles.
pub const BYTES_PER_TEXEL: u64 = 4;

/// A clip-space `w` at or below this is treated as "at or behind the eye" rather
/// than divided by, which is what a genuine near-zero or negative `w` would do to
/// a perspective divide.
const CLIP_W_EPS: f64 = 1e-9;

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
    /// contributes `(TEXTURE_SIZE_PX² / 4) * (clipped_area / raw_area)`, its fair
    /// share of the tile's texel budget scaled by how much of *that quad's own*
    /// projected area actually landed onscreen. A quad skipped for being behind the
    /// eye contributes zero, same as a quad clipped away entirely — a tile clipped
    /// at the frustum boundary is not credited with texels for the part of the
    /// patch that never made it to screen. Equals the flat `TEXTURE_SIZE_PX²` only
    /// when every quad is fully onscreen and unclipped.
    pub texels: f64,
    /// `texels / screen_px`. `f64::INFINITY` when `screen_px == 0.0` — excluded
    /// from every ratio aggregate, counted separately (see [`super::report`]).
    pub ratio: f64,
    /// At least one of the four projected quads lost area to viewport clipping.
    pub partly_offscreen: bool,
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
    /// No visible tile at all — a genuinely empty view (e.g. a sub-surface camera
    /// with nothing above the horizon), not a rendering defect on its own but a
    /// case the ratio aggregates must not silently absorb.
    pub degenerate: bool,
}

fn frustum_for(cam: &Camera, aspect: f32) -> Frustum {
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
    let full_texels = (TEXTURE_SIZE_PX as f64) * (TEXTURE_SIZE_PX as f64);
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
    let cam = build_camera(p);
    let aspect = p.aspect();
    let frustum = frustum_for(&cam, aspect as f32);

    let mut qt = QuadtreeManager::new();
    qt.pipeline = CullPipeline::DEFAULT;
    // The same derivation the real renderer runs per frame (`wgpu_state::update_logic`),
    // from the same shared function — not a copy of the arithmetic. At the 204 bench
    // poses (all `height = 1080`, all `mode = Free`) this is exactly 2.0, i.e. the value
    // this line replaced, so the WP0/WP1 baseline is untouched. It matters once WP4
    // builds a viewport ladder that actually varies height and mode: without it the
    // harness would quietly stop measuring what the renderer does.
    qt.lod_factor = cesium_engine::globe::quadtree::lod_factor_for(
        1.0, // target_texel_ratio — the calibrated no-op default; no per-pose config to read
        TEXTURE_SIZE_PX as f32,
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
        tiles.push(project_patch(*id, &samples, &vp, width, height));
    }

    let tile_count = tiles.len();
    let texture_bytes =
        tile_count as u64 * (TEXTURE_SIZE_PX as u64 * TEXTURE_SIZE_PX as u64) * BYTES_PER_TEXEL;

    PoseResult {
        params: p.clone(),
        degenerate: tile_count == 0,
        tiles,
        tile_count,
        texture_bytes,
        deepest_zoom,
    }
}

/// Measures every pose, inside the culling harness's own bounded rayon pool — see
/// [`super::super::culling::sweep::harness_pool`] for why the global pool is
/// deliberately not used.
pub fn measure_poses(poses: &[ViewParams]) -> Vec<PoseResult> {
    harness_pool().install(|| poses.par_iter().map(measure_pose).collect())
}
