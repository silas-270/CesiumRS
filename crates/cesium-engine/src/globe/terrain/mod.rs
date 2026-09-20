//! Terrain height data — Phase B of `docs/terrain-plan.md` §5.
//!
//! Height tiles are fetched, decoded, cached and queryable here. **Nothing in this
//! module renders, meshes or culls**: the `Heightfield` surface model that consumes it
//! is Phase C/D, and until it exists the whole module is inert behind
//! [`crate::globe::tiles::config::TerrainConfig::enabled`], which is `false`.
//!
//! - [`height_tile`] — the Terrarium decoder, the 256x256 `i16` sample grid, and the
//!   16x16 min/max mip. Metres.
//! - [`height_cache`] — the fetcher, the LRU, and `height_at` with ancestor
//!   upsampling. Megametres at its boundary.

pub mod height_cache;
pub mod height_tile;

pub use height_cache::HeightTileManager;
pub use height_tile::{decode_terrarium, HeightTile};
