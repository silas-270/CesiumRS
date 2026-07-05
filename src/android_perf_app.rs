use crate::engine::core::app::App;
use crate::engine::globe::tiles::config::TileEngineConfig;
use winit::application::ApplicationHandler;
use winit::event::WindowEvent;
use winit::event_loop::ActiveEventLoop;
use winit::window::WindowId;
use std::time::Instant;
use glam::Vec3;

pub struct AndroidPerfApp<'a> {
    pub inner: App<'a>,
    pub start_time: Option<Instant>,
}

impl<'a> AndroidPerfApp<'a> {
    pub fn new() -> Self {
        let config = TileEngineConfig::default();

        let mut flight_app = crate::flight::app::FlightTrackerApp::new();
        let flight_json = include_str!("../flight_FRA_STR.json");
        flight_app.queued_flights.push(("flight_FRA_STR.json".to_string(), flight_json.to_string()));
        
        Self {
            inner: App::new(config, Some(Box::new(flight_app))),
            start_time: None,
        }
    }
}

impl<'a> ApplicationHandler for AndroidPerfApp<'a> {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        self.inner.resumed(event_loop);
        
        if self.start_time.is_none() {
            self.start_time = Some(Instant::now());
        }
    }

    fn window_event(&mut self, event_loop: &ActiveEventLoop, window_id: WindowId, event: WindowEvent) {
        self.inner.window_event(event_loop, window_id, event);
    }

    fn about_to_wait(&mut self, event_loop: &ActiveEventLoop) {
        if let Some(start) = self.start_time {
            let elapsed = start.elapsed().as_secs_f32();
            
            if let Some(state) = self.inner.wgpu_state_mut() {
                // Animation Sequence:
                // 0 - 15s: Orbit to Germany
                // 15 - 30s: Tilt down
                // 30 - 60s: Fly forward
                
                let str_surface = crate::engine::globe::geometry::lon_lat_to_ecef(9.2, 48.7);
                let str_normal = str_surface.normalize();
                let fra_surface = crate::engine::globe::geometry::lon_lat_to_ecef(8.5, 50.0);
                let fra_normal = fra_surface.normalize();
                
                let mut eye = state.camera.local_pos;
                let mut target = eye - str_normal; // Look down by default

                if elapsed < 15.0 {
                    let progress = elapsed / 15.0;
                    let t = progress * progress * (3.0 - 2.0 * progress);
                    
                    let start_eye = Vec3::new(0.0, 0.0, 15.0);
                    let end_eye = str_surface + str_normal * 0.1; // 100km above STR
                    
                    eye = start_eye.lerp(end_eye, t);
                    // Interpolate target from pointing at origin, to pointing at surface
                    target = eye - Vec3::new(0.0, 0.0, 1.0).lerp(str_normal, t);
                } else if elapsed < 30.0 {
                    let progress = (elapsed - 15.0) / 15.0;
                    let t = progress * progress * (3.0 - 2.0 * progress);
                    
                    eye = str_surface + str_normal * 0.1;
                    
                    let start_target = eye - str_normal;
                    let dir_to_fra = (fra_surface - str_surface).normalize();
                    let end_target = eye + dir_to_fra - str_normal * 0.15; // Look ahead and slightly down
                    
                    target = start_target.lerp(end_target, t);
                } else if elapsed < 60.0 {
                    let progress = (elapsed - 30.0) / 30.0;
                    
                    let start_eye = str_surface + str_normal * 0.1;
                    let end_eye = fra_surface + fra_normal * 0.1;
                    
                    eye = start_eye.lerp(end_eye, progress);
                    let current_normal = str_normal.lerp(fra_normal, progress).normalize();
                    let dir = (end_eye - start_eye).normalize();
                    target = eye + dir - current_normal * 0.15;
                } else {
                    self.start_time = Some(Instant::now());
                }

                state.camera.set_eye(eye, target);
                
                if let Some(window) = self.inner.window() {
                    window.request_redraw();
                }
            }
        }

        self.inner.about_to_wait(event_loop);
    }
}
