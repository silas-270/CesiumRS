use crate::engine::core::app::App;
use crate::testing::VerifyConfig;
use crate::engine::camera::camera::CameraMode;
use winit::application::ApplicationHandler;
use winit::event::WindowEvent;
use winit::event_loop::ActiveEventLoop;
use winit::window::WindowId;
use std::sync::{Arc, Mutex};
use std::fs::File;
use std::io::Write;

pub struct FlickerTrackingApp<'a> {
    pub inner: App<'a>,
    pub config: VerifyConfig,
    pub setup_done: bool,
    pub progress: Arc<Mutex<f64>>,
    pub log_file: File,
    pub frame_count: u32,
}

impl<'a> FlickerTrackingApp<'a> {
    pub fn new(config: VerifyConfig) -> Self {
        let mut app_config = crate::engine::globe::tiles::config::TileEngineConfig::default();
        app_config.offline_mode = false; // We need actual fetching to test network glitches
        app_config.mesh_cache_size = std::num::NonZeroUsize::new(config.cache_size).unwrap();
        app_config.max_cache_size = std::num::NonZeroUsize::new(config.cache_size).unwrap();
        app_config.enable_prefetch = config.prefetch;

        let progress = Arc::new(Mutex::new(0.0));
        let mut flight_app = Box::new(crate::flight::app::FlightTrackerApp::new(progress.clone()));
        
        if let Ok(content) = std::fs::read_to_string("flight_FRA_STR.json") {
            flight_app.add_flight_path("flight_FRA_STR.json", content, false);
        } else {
            eprintln!("Warning: flight_FRA_STR.json not found. The test might not do anything useful.");
        }
        
        flight_app.is_playing = true;
        flight_app.play_speed = 0.01;
        flight_app.view_mode = CameraMode::Tracking;

        let mut log_file = File::create("flicker_metrics.csv").expect("Failed to create flicker_metrics.csv");
        writeln!(log_file, "Frame,Progress,VisibleTiles,RenderableTiles,MissingCount").unwrap();

        Self {
            inner: App::new(app_config, Some(flight_app)),
            config,
            setup_done: false,
            progress,
            log_file,
            frame_count: 0,
        }
    }
}

impl<'a> ApplicationHandler for FlickerTrackingApp<'a> {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        self.inner.resumed(event_loop);
        if !self.setup_done {
            self.setup_done = true;
        }
    }

    fn window_event(&mut self, event_loop: &ActiveEventLoop, window_id: WindowId, event: WindowEvent) {
        if let WindowEvent::RedrawRequested = event {
            self.inner.window_event(event_loop, window_id, WindowEvent::RedrawRequested);
            
            self.frame_count += 1;
            let current_progress = *self.progress.lock().unwrap();
            
            if let Some(state) = self.inner.wgpu_state_mut() {
                // Get metrics
                let requested_count = state.last_requested_tiles_count;
                let missing_count = state.last_missing_tiles_count;
                
                // Removed unexposed cache metrics
                let tile_system = &mut state.tile_system;
                
                // Let's count actually renderable tiles (has mesh AND texture)
                let visible_tiles = state.quadtree_manager.get_visible_tiles();
                let mut renderable_count = 0;
                for (id, _, _) in &visible_tiles {
                    if tile_system.get_render_data(*id).is_some() {
                        renderable_count += 1;
                    }
                }
                
                writeln!(
                    self.log_file, 
                    "{},{:.5},{},{},{}", 
                    self.frame_count, 
                    current_progress, 
                    requested_count, 
                    renderable_count, 
                    missing_count
                ).unwrap();
                
                if self.frame_count % 60 == 0 {
                    println!("Frame {}: Progress = {:.4}, Renderable = {}/{}", self.frame_count, current_progress, renderable_count, requested_count);
                }
            }
            
            if current_progress >= 0.5 {
                println!("Progress reached 0.5. Ending flicker test.");
                event_loop.exit();
            } else {
                if let Some(window) = &self.inner.window() {
                    window.request_redraw();
                }
            }
            
        } else {
            self.inner.window_event(event_loop, window_id, event);
        }
    }

    fn about_to_wait(&mut self, event_loop: &ActiveEventLoop) {
        self.inner.about_to_wait(event_loop);
    }
}
