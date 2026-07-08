use cesium_engine::core::app::App;
use crate::testing::harness::simulator::Simulator;
use crate::testing::VerifyConfig;
use winit::application::ApplicationHandler;
use winit::event::WindowEvent;
use winit::event_loop::ActiveEventLoop;
use winit::window::WindowId;

pub struct TestApp<'a> {
    pub inner: App<'a>,
    pub config: VerifyConfig,
    pub simulator: Simulator,
    pub frames_stable: u32,
    pub setup_done: bool,
}

impl<'a> TestApp<'a> {
    pub fn new(config: VerifyConfig) -> Self {
        let simulator = if let Some(ref actions) = config.actions {
            Simulator::parse(actions)
        } else {
            Simulator { actions: vec![] }
        };

        let mut flight_app = Box::new(cesium_flight::tracker::FlightTrackerApp::new(std::sync::Arc::new(std::sync::Mutex::new(0.0))));
        if let Ok(content) = std::fs::read_to_string("flight_FRA_STR.json") {
            flight_app.add_flight_path("flight_FRA_STR.json", content, true);
        }
        flight_app.reset_viewport = false;

        Self {
            inner: App::new(
                cesium_engine::globe::tiles::config::TileEngineConfig::default(), 
                Some(flight_app)
            ),
            config,
            simulator,
            frames_stable: 0,
            setup_done: false,
        }
    }
}

impl<'a> ApplicationHandler for TestApp<'a> {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        self.inner.resumed(event_loop);
        if !self.setup_done {
            if let Some(state) = self.inner.wgpu_state_mut() {
                let eye = glam::Vec3::new(
                    self.config.cam_x as f32,
                    self.config.cam_y as f32,
                    self.config.cam_z as f32,
                );
                
                let d = eye.length();
                let r = 6.378137f32; // Earth radius in Megameters
                let target = if d > r {
                    let normal = eye / d; // Pointing outward from center
                    // Find a tangent vector perpendicular to normal
                    let tangent = if normal.y.abs() < 0.9 {
                        normal.cross(glam::Vec3::Y).normalize()
                    } else {
                        normal.cross(glam::Vec3::X).normalize()
                    };
                    
                    // The triangle formed by Center, Horizon, Eye is right-angled at Horizon.
                    // sin(alpha) = r / d.
                    // cos(alpha) = sqrt(1.0 - sin(alpha)^2).
                    // Look direction from eye is -normal * cos(alpha) + tangent * sin(alpha).
                    let sin_alpha = r / d;
                    let cos_alpha = (1.0 - sin_alpha * sin_alpha).sqrt();
                    let look_dir = normal * (-cos_alpha) + tangent * sin_alpha;
                    eye + look_dir
                } else {
                    glam::Vec3::ZERO
                };

                state.camera.set_eye(eye, target);
            }
            self.setup_done = true;
        }
    }

    fn window_event(&mut self, event_loop: &ActiveEventLoop, window_id: WindowId, event: WindowEvent) {
        if let WindowEvent::RedrawRequested = event {
            let synthetic_events = self.simulator.pump_events();
            for syn_ev in synthetic_events {
                self.inner.window_event(event_loop, window_id, syn_ev);
            }
            
            let mut capture_path = None;
            if self.simulator.actions.is_empty() {
                if let Some(state) = self.inner.wgpu_state_mut() {
                    if state.tile_system.texture_manager.fetcher.is_loading_complete() {
                        self.frames_stable += 1;
                    } else {
                        self.frames_stable = 0;
                    }
                }
                
                if self.frames_stable > 5 {
                    capture_path = Some(self.config.out_path.as_str());
                }
            }

            if let Some(state) = self.inner.wgpu_state_mut() {
                match state.render(capture_path, false, |_, _| {}) {
                    Ok(_) => {
                        if capture_path.is_some() {
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
