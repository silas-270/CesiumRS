//! LOD-quality measuring instrument, in the idiom of `src/testing/culling/`.
//!
//! `QuadtreeNode::apply_lod` decides *which zoom level a tile refines to*, using
//! `lod_factor = 2.0` — Cesium's `d < G(z)·H / (maxSSE · 2·tan(fovy/2))` collapsed
//! to one constant (`docs/culling-math.md` §8.5, `docs/pre-terrain-plan.md`
//! §Context). The culling harness in [`super::culling`] can prove the visibility
//! *decision* is right because it has an independent oracle to check against; the
//! LOD decision has no equivalent, so this module is that instrument. It is a
//! **measuring device, not a fix**: it reports numbers, it does not assert targets,
//! until `docs/pre-terrain-plan.md` WP4 picks them.
//!
//! # The metric
//!
//! With zero terrain relief the drawn geometry is always the ellipsoid, so the
//! only real per-tile error is imagery resolution:
//!
//! ```text
//! screen_px = projected area of the tile's drawn patch, in pixels²
//! texels    = texture_size²                     (512 for Carto @2x, 256 for Esri)
//! ratio     = texels / screen_px                // want ≈ 1.0
//! ```
//!
//! `ratio < 1` is blurry (under-refined: too few texels for the screen area);
//! `ratio > 1` is wasted bandwidth and memory (over-refined). `texture_size` is
//! hard-coded to the engine's current default (512, `STANDARD_IMAGERY_URL`'s `@2x`
//! tiles) — WP3/WP4 are what make it a real, per-style input; here it is frozen,
//! same as `lod_factor` itself.
//!
//! The patch is sampled the same way `TileMesh::generate` builds the mesh — see
//! [`sweep::patch_grid_points`] — not via the node's `OrientedBoundingBox`, which
//! over-states area at grazing angles (the box is fatter than the curved patch it
//! bounds).
//!
//! # Layout
//!
//! | file | concern |
//! |------|---------|
//! | [`sweep`]  | per-pose measurement: project the patch, clip to the viewport, score every visible tile |
//! | [`report`] | CSV (into the temp dir) and the human-readable summary |
//!
//! Test cases live in `test_*.rs` and contain no measurement logic of their own.
//!
//! It reuses [`super::culling::cameras`] (`ViewParams`, `build_camera`) and the 204
//! poses from `super::culling::bench_update::bench_cells` rather than inventing new
//! ones — the same poses the culling gate and the bench already treat as
//! representative.
//!
//! ## Running it
//!
//! ```text
//! cargo test --release --lib lod:: -- --test-threads=1 --nocapture
//! ```
//!
//! CSVs land in `$TMPDIR/cesium_lod_harness/`, never in the repo.

#[cfg(test)]
pub mod report;
#[cfg(test)]
pub mod sweep;

#[cfg(test)]
pub mod test_lod_sweep;
