use cesium_engine::core::app::App;
use cesium_engine::globe::tiles::config::TileEngineConfig;
use std::num::NonZeroUsize;
use winit::event_loop::{ControlFlow, EventLoop};

/// Configuration options for the Globe (terrain, imagery base, caching, performance)
#[derive(Clone, Debug)]
pub struct GlobeOptions {
    /// Maximum number of tiles to keep in the cache
    pub tile_cache_size: usize,
    /// Higher values lower visual fidelity but improve performance
    pub maximum_screen_space_error: f32,
    /// Whether the engine should try to fetch tiles before they are strictly needed
    pub enable_prefetch: bool,
    /// Image adjustment: -1.0 (grayscale) to 1.0 (oversaturated). Default 0.0.
    pub map_saturation: f32,
    /// Image adjustment: -1.0 (washed out) to 1.0 (high contrast). Default 0.0.
    pub map_contrast: f32,
    /// Image adjustment: -1.0 (pitch black) to 1.0 (bright white). Default 0.0.
    pub map_brightness: f32,
}

impl Default for GlobeOptions {
    fn default() -> Self {
        Self {
            tile_cache_size: 2048,
            maximum_screen_space_error: 2.0,
            enable_prefetch: true,
            map_saturation: 0.0,
            map_contrast: 0.0,
            map_brightness: 0.0,
        }
    }
}

/// Initialization options for the CesiumRS Viewer
#[derive(Clone, Debug, Default)]
pub struct ViewerOptions {
    pub globe: GlobeOptions,
}

impl ViewerOptions {
    pub(crate) fn into_tile_engine_config(self) -> TileEngineConfig {
        let mut config = TileEngineConfig::default();
        config.max_cache_size = NonZeroUsize::new(self.globe.tile_cache_size).unwrap_or(NonZeroUsize::new(1).unwrap());
        config.mesh_cache_size = NonZeroUsize::new(self.globe.tile_cache_size).unwrap_or(NonZeroUsize::new(1).unwrap());
        config.lod_factor = self.globe.maximum_screen_space_error;
        config.enable_prefetch = self.globe.enable_prefetch;
        config.map_saturation = self.globe.map_saturation;
        config.map_contrast = self.globe.map_contrast;
        config.map_brightness = self.globe.map_brightness;
        config
    }
}

/// The primary entry point for the CesiumRS engine.
pub struct Viewer {
    event_loop: EventLoop<()>,
    options: ViewerOptions,
}

impl Viewer {
    /// Create a new viewer with the given options.
    pub fn new(options: ViewerOptions) -> Self {
        let event_loop = EventLoop::new().unwrap();
        event_loop.set_control_flow(ControlFlow::Poll);
        
        Self {
            event_loop,
            options,
        }
    }

    /// Start the application event loop.
    /// In Rust, this takes over the main thread and does not return.
    pub fn run(self, extension: Option<Box<dyn cesium_engine::core::extension::GlobeExtension>>) {
        let config = self.options.into_tile_engine_config();
        let mut app = App::new(config, extension, None);
        self.event_loop.run_app(&mut app).unwrap();
    }
}
