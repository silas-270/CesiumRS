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
                
                let target_lon_rad = 9.2_f32.to_radians(); // Stuttgart
                let target_lat_rad = 48.7_f32.to_radians();
                let mercator_y = (std::f32::consts::PI / 4.0 + target_lat_rad / 2.0).tan().ln();
                
                let mut eye = state.camera.local_pos;
                let mut target = eye + glam::Vec3::new(0.0, 0.0, -1.0); // Look down by default

                if elapsed < 15.0 {
                    let progress = elapsed / 15.0;
                    // Ease in out
                    let t = progress * progress * (3.0 - 2.0 * progress);
                    
                    let start_eye = Vec3::new(0.0, 0.0, 8.0);
                    let end_eye = Vec3::new(target_lon_rad, mercator_y, 0.05); // Above STR
                    
                    eye = start_eye.lerp(end_eye, t);
                    target = eye + Vec3::new(0.0, 0.0, -1.0); // Look straight down
                } else if elapsed < 30.0 {
                    let progress = (elapsed - 15.0) / 15.0;
                    let t = progress * progress * (3.0 - 2.0 * progress);
                    
                    eye = Vec3::new(target_lon_rad, mercator_y, 0.05);
                    
                    let start_target = eye + Vec3::new(0.0, 0.0, -1.0);
                    // Tilt to look along the horizon (slightly down and towards FRA)
                    let fra_lon_rad = 8.5_f32.to_radians();
                    let fra_lat_rad = 50.0_f32.to_radians();
                    let fra_mercator_y = (std::f32::consts::PI / 4.0 + fra_lat_rad / 2.0).tan().ln();
                    
                    let dir = Vec3::new(fra_lon_rad - target_lon_rad, fra_mercator_y - mercator_y, -0.01).normalize();
                    let end_target = eye + dir;
                    
                    target = start_target.lerp(end_target, t);
                } else if elapsed < 60.0 {
                    let progress = (elapsed - 30.0) / 30.0;
                    
                    let fra_lon_rad = 8.5_f32.to_radians();
                    let fra_lat_rad = 50.0_f32.to_radians();
                    let fra_mercator_y = (std::f32::consts::PI / 4.0 + fra_lat_rad / 2.0).tan().ln();
                    
                    let start_eye = Vec3::new(target_lon_rad, mercator_y, 0.05);
                    let end_eye = Vec3::new(fra_lon_rad, fra_mercator_y, 0.05);
                    
                    eye = start_eye.lerp(end_eye, progress);
                    let dir = (end_eye - start_eye).normalize();
                    target = eye + dir + Vec3::new(0.0, 0.0, -0.05);
                } else {
                    // Loop animation
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
