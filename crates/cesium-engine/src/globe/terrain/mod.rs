//! Terrain — Phases B and C of `docs/terrain-plan.md` §5 and §6.
//!
//! Height tiles are fetched, decoded, cached and queried here (B), and turned into the
//! mesh inputs a tile with relief is built from (C). **Nothing in this module culls**:
//! the bounding volumes and the occlusion march are Phase D, which is why
//! [`crate::globe::tiles::config::TerrainConfig::enabled`] is still `false`.
//!
//! - [`height_tile`] — the Terrarium decoder, the 256x256 `i16` sample grid, and the
//!   16x16 min/max mip. Metres.
//! - [`height_cache`] — the fetcher, the LRU, and `height_at` with ancestor
//!   upsampling. Megametres at its boundary.
//! - [`heightfield`] — **Phase C**: the `Heightfield` surface model that finally
//!   consumes all of it, and the pre-sampled `HeightPatch` the mesh builder takes as
//!   its input. Still behind `TerrainConfig::enabled`, which is still `false`.

pub mod height_cache;
pub mod height_tile;
pub mod heightfield;

pub use height_cache::HeightTileManager;
pub use height_tile::{decode_terrarium, HeightTile};
pub use heightfield::{HeightPatch, Heightfield, PatchStatus};
