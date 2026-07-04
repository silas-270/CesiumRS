pub mod options;

pub use options::{GlobeOptions, ViewerOptions};

use crate::engine::core::app::App;
use winit::event_loop::{ControlFlow, EventLoop};

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
    pub fn run(self) {
        let config = self.options.into_tile_engine_config();
        let mut app = App::new(config);
        self.event_loop.run_app(&mut app).unwrap();
    }
}
