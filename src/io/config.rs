use std::num::NonZeroUsize;
use std::time::Duration;

#[derive(Clone, Debug)]
pub struct TileEngineConfig {
    pub max_cache_size: NonZeroUsize,
    pub lod_factor: f32,
    pub prefetch_radius: u32,
    pub negative_cache_duration: Duration,
}

impl Default for TileEngineConfig {
    fn default() -> Self {
        Self {
            max_cache_size: NonZeroUsize::new(2048).unwrap(),
            lod_factor: 2.0,
            prefetch_radius: 1, // Number of tiles to prefetch in velocity direction
            negative_cache_duration: Duration::from_secs(10),
        }
    }
}
