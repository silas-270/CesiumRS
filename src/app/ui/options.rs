use crate::engine::globe::io::config::TileEngineConfig;
use std::num::NonZeroUsize;

/// Configuration options for the Globe (terrain, imagery base, caching, performance)
#[derive(Clone, Debug)]
pub struct GlobeOptions {
    /// Maximum number of tiles to keep in the cache
    pub tile_cache_size: usize,
    /// Higher values lower visual fidelity but improve performance
    pub maximum_screen_space_error: f32,
    /// Whether the engine should try to fetch tiles before they are strictly needed
    pub enable_prefetch: bool,
}

impl Default for GlobeOptions {
    fn default() -> Self {
        Self {
            tile_cache_size: 2048,
            maximum_screen_space_error: 2.0,
            enable_prefetch: true,
        }
    }
}

/// Initialization options for the CesiumRS Viewer
#[derive(Clone, Debug, Default)]
pub struct ViewerOptions {
    pub globe: GlobeOptions,
    // imagery_provider: Option<...>,
    // terrain_provider: Option<...>,
}

impl ViewerOptions {
    pub(crate) fn into_tile_engine_config(self) -> TileEngineConfig {
        let mut config = TileEngineConfig::default();
        config.max_cache_size = NonZeroUsize::new(self.globe.tile_cache_size).unwrap_or(NonZeroUsize::new(1).unwrap());
        config.mesh_cache_size = NonZeroUsize::new(self.globe.tile_cache_size).unwrap_or(NonZeroUsize::new(1).unwrap());
        config.lod_factor = self.globe.maximum_screen_space_error;
        config.enable_prefetch = self.globe.enable_prefetch;
        config
    }
}
