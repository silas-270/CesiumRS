use crate::engine::core::app::App;
use crate::testing::VerifyConfig;
use winit::application::ApplicationHandler;
use winit::event::WindowEvent;
use winit::event_loop::ActiveEventLoop;
use winit::window::WindowId;
use std::fs::File;
use std::io::Write;

pub struct StressApp<'a> {
    pub inner: App<'a>,
    pub frame_count: u32,
    pub log_file: File,
    pub setup_done: bool,
    pub config: VerifyConfig,
}

impl<'a> StressApp<'a> {
    pub fn new(config: VerifyConfig) -> Self {
        let filename = format!("stress_results_{}.csv", config.stress_mode);
        let mut log_file = File::create(&filename).unwrap();
        writeln!(log_file, "frame,speed_multiplier,requested_tiles,missing_tiles").unwrap();
        Self {
            inner: App::new(crate::engine::globe::tiles::config::TileEngineConfig::default(), None),
            frame_count: 0,
            log_file,
            setup_done: false,
            config,
        }
    }
}

impl<'a> ApplicationHandler for StressApp<'a> {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        self.inner.resumed(event_loop);
        if !self.setup_done {
            if let Some(state) = self.inner.wgpu_state_mut() {
                // Apply config limits
                state.tile_system.config.enable_prefetch = self.config.prefetch;
                let cache_size = std::num::NonZeroUsize::new(self.config.cache_size).unwrap();
                state.tile_system.config.max_cache_size = cache_size;
                state.tile_system.config.mesh_cache_size = cache_size;
                state.tile_system.texture_manager.resize(cache_size);
                state.resize_tile_cache(cache_size);
                
                state.camera.set_eye(
                    glam::Vec3::new(0.0, 0.0, 8.0),
                    glam::Vec3::ZERO,
                );
            }
            self.setup_done = true;
        }
    }

    fn window_event(&mut self, event_loop: &ActiveEventLoop, window_id: WindowId, event: WindowEvent) {
        if let WindowEvent::RedrawRequested = event {
            self.frame_count += 1;
            
            // Speed starts at 0.01 and grows linearly (not quadratically to avoid insane speeds too fast)
            let time = self.frame_count as f32 / 60.0;
            let speed_multiplier = 0.01 + (time * 0.02);
            
            if let Some(state) = self.inner.wgpu_state_mut() {
                
                if self.config.stress_mode == "poi" {
                    // POI mode: Orbit a point on the surface
                    let center = glam::Vec3::new(6.378, 0.0, 0.0);
                    // Orbiting around this center at a distance of 1.0
                    let angle = time * speed_multiplier;
                    let eye_x = 6.378 + (angle.cos() * 1.0);
                    let eye_y = angle.sin() * 1.0;
                    let eye_z = 0.5; // slightly above
                    
                    state.camera.set_eye(glam::Vec3::new(eye_x, eye_y, eye_z), center);
                } else {
                    // Flight mode: Tangential flight path like a plane
                    // Radius = 6.4 (flying low over surface)
                    let angle = time * speed_multiplier * 0.5;
                    let eye_x = angle.cos() * 6.4;
                    let eye_y = angle.sin() * 6.4;
                    
                    // Look slightly ahead tangentially
                    let look_angle = angle + 0.1;
                    let look_x = look_angle.cos() * 6.4;
                    let look_y = look_angle.sin() * 6.4;
                    
                    state.camera.set_eye(glam::Vec3::new(eye_x, eye_y, 0.0), glam::Vec3::new(look_x, look_y, 0.0));
                }

                match state.render(None, false, |_, _| {}) {
                    Ok(_) => {
                        let (req, miss) = state.get_fetch_stats();
                        writeln!(self.log_file, "{},{},{},{}", self.frame_count, speed_multiplier, req, miss).unwrap();
                        
                        if self.frame_count > 1000 {
                            event_loop.exit();
                        } else {
                            if let Some(window) = self.inner.window() {
                                window.request_redraw();
                            }
                        }
                    }
                    Err(wgpu::SurfaceError::Lost) => state.resize(state.size),
                    Err(wgpu::SurfaceError::OutOfMemory) => event_loop.exit(),
                    Err(e) => log::error!("Render error: {:?}", e),
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
