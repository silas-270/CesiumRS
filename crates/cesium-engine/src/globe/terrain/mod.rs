//! Terrain — Phases B, C and D1/D2 of `docs/terrain-plan.md` §5, §6 and §7.
//!
//! Height tiles are fetched, decoded, cached and queried here (B), turned into the mesh
//! inputs a tile with relief is built from (C), and turned into the altitude interval and
//! bounding sphere the quadtree culls against (D1, D2).
//!
//! The occlusion march of §3.3 — culling tiles hidden behind mountains — is **D3**, and it
//! lives in [`crate::globe::quadtree::terrain_occlusion`] rather than here: its occluders
//! are the quadtree's own node floors ([`HeightBounds::floor_grid`]), so it needs nothing
//! from this module but that one number per node.
//! [`crate::globe::tiles::config::TerrainConfig::enabled`] is still `false` — that flip is
//! Phase F's, after the on-device measurements.
//!
//! - [`height_tile`] — the Terrarium decoder, the 256x256 `i16` sample grid, and the
//!   16x16 min/max mip. Metres.
//! - [`height_cache`] — the fetcher, the LRU, and `height_at` with ancestor
//!   upsampling. Megametres at its boundary.
//! - [`heightfield`] — the `Heightfield` surface model that finally consumes all of it:
//!   the pre-sampled `HeightPatch` the mesh builder takes as its input (C), and the
//!   `HeightBounds` interval plus the scaled-space bounding sphere the culler takes as
//!   its input (D1, D2). Still behind `TerrainConfig::enabled`, which is still `false`.

pub mod corridor;
pub mod height_cache;
pub mod height_tile;
pub mod heightfield;

pub use corridor::RunwayCorridor;
pub use height_cache::HeightTileManager;

pub use height_tile::{
    decode_terrarium, HeightTile, HEIGHT_DETAIL_LEVELS, HEIGHT_DETAIL_PYRAMID_CELLS,
};
pub use heightfield::{
    fallback_detail_mm, inherit_allowance_mm, inherit_margin_mm, skirt_allowance, HeightBounds,
    HeightBoundsSource, HeightPatch, Heightfield, PatchStatus, DETAIL_MAX_Z, OCCLUDER_GRID,
    OCCLUDER_GRID_CELLS,
};
