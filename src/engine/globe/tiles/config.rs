use std::num::NonZeroUsize;
use std::time::Duration;

#[derive(Clone, Debug)]
pub struct TileEngineConfig {
    pub max_cache_size: NonZeroUsize,
    pub mesh_cache_size: NonZeroUsize,
    pub lod_factor: f32,
    pub prefetch_radius: u32,
    pub enable_prefetch: bool,
    pub negative_cache_duration: Duration,
    pub base_imagery_url: String,
    pub offline_mode: bool,
}

impl Default for TileEngineConfig {
    fn default() -> Self {
        Self {
            max_cache_size: NonZeroUsize::new(2048).unwrap(),
            mesh_cache_size: NonZeroUsize::new(512).unwrap(),
            lod_factor: 2.0,
            prefetch_radius: 1, // Number of tiles to prefetch in velocity direction
            enable_prefetch: true,
            negative_cache_duration: Duration::from_secs(10),
            base_imagery_url: "https://a.basemaps.cartocdn.com/dark_nolabels/{z}/{x}/{y}.png".to_string(),
            offline_mode: false,
        }
    }
}
