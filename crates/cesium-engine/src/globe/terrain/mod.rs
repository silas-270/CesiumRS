//! Terrain: height data, relief meshes and height-aware culling.
//!
//! Height tiles are fetched, decoded, cached and queried here, turned into the mesh
//! inputs a tile with relief is built from, and turned into the altitude interval and
//! bounding sphere the quadtree culls against.
//!
//! The occlusion march of §3.3 — culling tiles hidden behind mountains — is terrain occlusion, and it
//! lives in [`crate::globe::quadtree::terrain_occlusion`] rather than here: its occluders
//! are the quadtree's own node floors ([`HeightBounds::floor_grid`]), so it needs nothing
//! from this module but that one number per node.
//! Whether terrain is on by default is decided per platform by
//! [`crate::globe::tiles::config::TERRAIN_ENABLED_BY_DEFAULT`].
//!
//! - [`height_tile`] — the Terrarium decoder, the 256x256 `i16` sample grid, and the
//!   16x16 min/max mip. Metres.
//! - [`height_cache`] — the fetcher, the LRU, and `height_at` with ancestor
//!   upsampling. Megametres at its boundary.
//! - [`heightfield`] — the `Heightfield` surface model that finally consumes all of it:
//!   the pre-sampled `HeightPatch` the mesh builder takes as its input, and the
//!   `HeightBounds` interval plus the scaled-space bounding sphere the culler takes as
//!   its input. Still behind `TerrainConfig::enabled`, which is still `false`.

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
