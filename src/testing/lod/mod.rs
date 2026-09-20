//! LOD-quality measuring instrument, in the idiom of `src/testing/culling/`.
//!
//! `QuadtreeNode::apply_lod` decides *which zoom level a tile refines to*, using
//! `lod_factor` — Cesium's `d < G(z)·H / (maxSSE · 2·tan(fovy/2))` collapsed
//! to one constant (`docs/culling-math.md` §8.5, `docs/pre-terrain-plan.md`
//! §Context). That constant was the literal `2.0` when this harness was written;
//! WP3/3b unfroze it into [`cesium_engine::globe::quadtree::lod_factor_for`], which
//! [`sweep::measure_pose`] now calls with the same inputs the renderer uses, so the
//! two cannot drift apart. At these 204 poses (all `height = 1080`, `mode = Free`)
//! it still evaluates to exactly `2.0`. The culling harness in [`super::culling`] can prove the visibility
//! *decision* is right because it has an independent oracle to check against; the
//! LOD decision has no equivalent, so this module is that instrument. It is a
//! **measuring device, not a fix**: it reports numbers, it does not assert targets,
//! until `docs/pre-terrain-plan.md` WP4 picks them.
//!
//! # The metrics — there are two of them now
//!
//! ## 1. Imagery resolution, `texels / screen_px`
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
//! tiles) — WP4 is what makes it a real, per-style input; here it is still frozen.
//! (`lod_factor` no longer is: see above.)
//!
//! ## 2. Projected geometric error, in pixels — and why there had to be a second one
//!
//! This module used to say, in this position, that `texels / screen_px` was the whole
//! story *because* with zero relief the only per-tile error is imagery resolution. That
//! sentence was true when it was written and **E1 of `docs/terrain-plan.md` §8 expired
//! it**: `apply_lod`'s threshold is now `max(imagery_dist, terrain_dist)`, and the second
//! half is driven by a quantity the first metric cannot see — the deviation of the drawn
//! mesh from the real ground. A globe can be perfectly sharp and the wrong shape.
//!
//! So the harness carries [`sweep::geometric_error_px`] alongside it:
//! `error · H / (dist · 2·tan(fovy/2))`, Cesium's screen-space error evaluated on this
//! engine's own measured error. `Summary::aggregate_ratio` was a complete description of
//! LOD quality only while relief was zero; the pair is what replaces it.
//!
//! **In this harness the second metric is identically zero**, and that is the point
//! rather than a gap. Everything here runs `QuadtreeManager::new()` — an
//! `Ellipsoid` tree — whose `HAS_GEOMETRIC_ERROR` is a compile-time `false` because
//! invariant I-1 says the drawn surface *is* the ellipsoid. What used to be a claim in
//! this comment is now a value that
//! [`test_lod_sweep::the_flat_globe_leaves_no_geometric_error_on_screen`] reads and
//! asserts over all 204 poses. Where the number is *not* zero is
//! `testing::terrain::test_terrain_lod`, which measures real DEM tiles through the same
//! function — see §8's tables.
//!
//! Neither metric is written to the CSVs as a new column, deliberately: those files are
//! the byte-for-byte statement that the flat path has not moved across every package since
//! WP0, and an instrument that rewrites its own output format cannot make that statement.
//!
//! ## Known limit: flat 3×3 grid under-states curvature at coarse zoom
//!
//! `patch_grid_points` samples a fixed 3×3 grid (four flat quads) regardless of
//! zoom. A flat quad chord under-states the true curved-patch area of a coarse,
//! wide tile, exactly the effect `obb_grid_steps` in
//! `crates/cesium-engine/src/globe/quadtree/quadtree.rs` compensates for on the
//! OBB-fitting path by densifying its own grid for `z < 5`. This instrument does
//! **not** do the same thing, and the choice was measured, not assumed:
//!
//! Comparing the current 3×3/4-quad sum against a 17×17/256-quad reference grid
//! (same lon/lat/Mercator-y sampling, just denser), on real bench poses
//! (`bench_poses()`, nadir-ladder cells):
//!
//! | zoom | pose (nadir, lat=0 lon=9) | coarse (3×3) | fine (17×17) | relative gap |
//! |------|---------------------------|-------------:|-------------:|-------------:|
//! | 1    | alt = 20 Mm               | 130 762 px²  | 160 795 px²  | **18.68 %**  |
//! | 5    | alt = 1 Mm                | 1 915 048 px²| 1 933 581 px²| 0.96 %       |
//! | 8    | alt = 100 km              | 3 746 720 px²| 3 752 578 px²| 0.16 %       |
//!
//! (The two z=1 hemisphere tiles at that pose, `x=1,y=0` and `x=1,y=1`, measured
//! identically by the nadir camera's north/south symmetry — a sanity check on the
//! measurement, not a coincidence worth chasing.)
//!
//! This confirms the plan's own estimate (10-15 % at z=1, negligible by z=5) is
//! roughly right, if a bit optimistic — the measured z=1 gap is closer to 19 %.
//! **Decision: left as a flat 3×3 grid at every zoom (option 2, not the
//! `obb_grid_steps`-style zoom-dependent density of option 1).** The per-tile bias
//! is real at z=1-2, but those bands are a sliver of the pooled sample the
//! aggregates in [`report`] are built from — 52 of 3922 sampled tiles (1.3 %) in
//! the full 204-pose bench run at the time of this measurement — so it does not
//! materially move `Summary::aggregate_ratio`, the mean/median/percentiles, or the
//! per-zoom-band aggregates for z ≥ 3 that WP4 will lean on most. A future reader
//! tuning `target_texel_ratio` specifically from the z=1/z=2 `ZoomBand` rows,
//! rather than the pooled aggregate, should treat those two rows' `screen_px` (and
//! therefore `aggregate_ratio`) as **~15-20 % too low** — i.e. those bands' true
//! ratio is that much lower than reported — and either densify the grid at that
//! point or apply a correction factor before trusting them in isolation.
//!
//! # Layout
//!
//! | file | concern |
//! |------|---------|
//! | [`sweep`]  | per-pose measurement: project the patch, clip to the viewport, score every visible tile |
//! | [`report`] | CSV (into the temp dir) and the human-readable summary |
//! | [`ladder`] | WP4/B: re-labels the 204 bench pose geometries at other viewport/mode combinations |
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
pub mod fog_sweep;
#[cfg(test)]
pub mod ladder;
#[cfg(test)]
pub mod report;
#[cfg(test)]
pub mod sweep;
#[cfg(test)]
pub mod test_fog_math;

#[cfg(test)]
pub mod test_lod_sweep;
#[cfg(test)]
pub mod test_viewport_ladder;
#[cfg(test)]
pub mod test_wp4c_3a_equal_budget;
#[cfg(test)]
pub mod test_wp5_fog;
