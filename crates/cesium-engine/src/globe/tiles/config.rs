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
    pub base_color: [u8; 4],
    pub offline_mode: bool,
    pub map_saturation: f32,
    pub map_contrast: f32,
    pub map_brightness: f32,
    pub transparent_background: bool,
    pub mesh_segments: u32,
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
            base_imagery_url: "https://a.basemaps.cartocdn.com/dark_nolabels/{z}/{x}/{y}.png"
                .to_string(),
            base_color: [20, 20, 20, 255],
            offline_mode: false,
            map_saturation: 0.0,
            map_contrast: 0.0,
            map_brightness: 0.0,
            transparent_background: false,
            mesh_segments: 16,
        }
    }
}
