//! Standalone fog-aware measurement path — WP5 of `docs/pre-terrain-plan.md`.
//!
//! Deliberately **does not** share code with [`super::sweep::measure_pose_with_config`]
//! / [`super::sweep::measure_poses_with_config`], even though the bodies are almost
//! identical. Every other LOD test in this crate trusts that
//! `sweep::measure_pose_with_config` measures the un-fogged tree — that trust is exactly
//! what makes it safe for `docs/culling-baseline.md`'s numbers to be compared against each
//! other across packages. Branching that one shared function on a "use fog or not" flag
//! would put the harness-safety property behind a conditional a future edit could flip by
//! accident. Duplicating the handful of lines that differ keeps the fogged measurement in
//! a file whose name says what it is, used from nowhere else.
//!
//! **What changed at E1c.** This module used to run `CullPipeline::DEFAULT_WITH_FOG`, and
//! that constant's whole point was to keep an unsound stage out of the harness. The stage
//! was measured to cull nothing at any camera and deleted (`docs/terrain-plan.md` §8), so
//! the pipeline here is now plain `DEFAULT` and the only difference from the baseline
//! sweep is `fog_density` — which is the only difference that ever produced a number.
//! Re-running WP5's whole suite across that deletion returned **byte-identical CSVs**,
//! which is the measurement rather than the hope.
//!
//! **Never import this module from anything that is not itself a WP5 measurement.**

use cesium_engine::camera::camera::Camera;
use cesium_engine::globe::quadtree::{
    fog_density_for, tile_bounds, CullPipeline, FogConfig, QuadtreeManager, MEGAMETERS_TO_METERS,
};
use rayon::prelude::*;

use super::super::culling::cameras::{build_camera, ViewParams};
use super::super::culling::sweep::{harness_pool, UPDATE_ITERATIONS};
use super::sweep::{frustum_for, patch_grid_points, project_patch, LodConfig, PoseResult};

/// This engine's altitude (`Camera::altitude`) is megameters; [`fog_density_for`]
/// takes metres — see `globe::quadtree::fog`'s module doc comment's Units section.
fn fog_density_at(cam: &Camera, fog_cfg: &FogConfig) -> f32 {
    fog_density_for(cam.altitude() * MEGAMETERS_TO_METERS, fog_cfg)
}

/// As [`super::sweep::measure_pose_with_config`], but with `fog_cfg`'s density for this
/// pose's camera altitude instead of no fog at all. See the module doc comment for why
/// this is not the same function with a flag.
pub fn measure_pose_with_fog(p: &ViewParams, cfg: LodConfig, fog_cfg: &FogConfig) -> PoseResult {
    let cam = build_camera(p);
    let aspect = p.aspect();
    let frustum = frustum_for(&cam, aspect as f32);

    let mut qt = QuadtreeManager::new();
    qt.pipeline = CullPipeline::DEFAULT;
    qt.lod_distance_mode = cfg.lod_distance_mode;
    qt.lod_factor = cesium_engine::globe::quadtree::lod_factor_for(
        cfg.target_texel_ratio,
        cfg.lod_texture_size_px,
        p.height as f32,
        cam.fovy(),
    );
    qt.fog_density = fog_density_at(&cam, fog_cfg);
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
    let texture_bytes = tile_count as u64 * texel_area * super::sweep::BYTES_PER_TEXEL;

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

/// As [`measure_pose_with_fog`], across poses, inside the culling harness's own
/// bounded rayon pool (see [`harness_pool`] for why).
pub fn measure_poses_with_fog(
    poses: &[ViewParams],
    cfg: LodConfig,
    fog_cfg: &FogConfig,
) -> Vec<PoseResult> {
    harness_pool().install(|| {
        poses
            .par_iter()
            .map(|p| measure_pose_with_fog(p, cfg, fog_cfg))
            .collect()
    })
}

/// The fog density this pose's camera altitude alone would produce, with
/// `fog_cfg` — exposed for measurement code that wants to report or bucket poses by
/// their fog density without re-deriving the altitude/unit-conversion dance.
pub fn fog_density_for_pose(p: &ViewParams, fog_cfg: &FogConfig) -> f32 {
    fog_density_at(&build_camera(p), fog_cfg)
}
