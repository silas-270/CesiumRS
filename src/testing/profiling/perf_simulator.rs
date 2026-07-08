use std::time::Instant;
use winit::application::ApplicationHandler;
use winit::event::WindowEvent;
use winit::event_loop::ActiveEventLoop;
use winit::window::WindowId;

use cesium_engine::core::app::App;
use cesium_engine::globe::tiles::config::TileEngineConfig;
use crate::api::{CameraMode, ViewerHandle};
use cesium_flight::tracker::FlightTrackerApp;

use crate::testing::VerifyConfig;
use crate::testing::harness::simulator::Simulator;

pub struct PerfSimulatorApp<'a> {
    inner: App<'a>,
    simulator: Simulator,
    frame_count: u64,
    start_time: Option<Instant>,
    viewer_handle: Option<ViewerHandle>,
}

impl<'a> PerfSimulatorApp<'a> {
    pub fn new(_config: VerifyConfig) -> Self {
        // Build the flight app & handle
        let (flight_app, flight_handle) = FlightTrackerApp::with_handle();

        // Load the sample flight for the tracker
        if let Ok(content) = std::fs::read_to_string("flight_FRA_STR.json") {
            flight_handle.load_flight("flight_FRA_STR", content);
        }

        // Start playing immediately. Set speed so the flight lasts ~100 seconds
        // (so it's still flying when we switch modes at 20s and 40s)
        flight_handle.play();
        flight_handle.set_speed(0.01);

        // Set up the simulator script for Free Mode (1200 frames):
        // Wait 50, Zoom in 150, Wait 50, Drag 300, Wait 50, Zoom out 150, Wait 50, Drag 300, Wait 100.
        let simulator = Simulator::parse("wait:50;scroll:1.0:150;wait:50;drag:400,300->300,300:300;wait:50;scroll:-1.0:150;wait:50;drag:300,300->400,300:300;wait:100");

        let mut app_config = TileEngineConfig::default();
        app_config.enable_prefetch = true; // Make sure prefetch is on for a realistic test
        
        // We initialize the App with the FlightTracker extension.
        // We will manually inject a ViewerHandle's receiver so we can send commands.
        let (tx, rx) = std::sync::mpsc::sync_channel(64);
        
        Self {
            inner: App::new(app_config, Some(Box::new(flight_app)), Some(rx)),
            simulator,
            frame_count: 0,
            start_time: None,
            viewer_handle: Some(ViewerHandle { tx }),
        }
    }
}

impl<'a> ApplicationHandler for PerfSimulatorApp<'a> {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        self.inner.resumed(event_loop);
        
        if self.start_time.is_none() {
            self.start_time = Some(Instant::now());
            
            // Set initial position (Frankfurt, Europe)
            if let Some(handle) = &self.viewer_handle {
                handle.camera_set_position(8.68, 50.11, 20.0);
                handle.camera_set_mode(CameraMode::Free);
            }
        }
    }

    fn window_event(
        &mut self,
        event_loop: &ActiveEventLoop,
        window_id: WindowId,
        event: WindowEvent,
    ) {
        self.inner.window_event(event_loop, window_id, event);
    }

    fn about_to_wait(&mut self, event_loop: &ActiveEventLoop) {
        // Automatically exit after 3600 frames (approx 1 minute at 60 FPS)
        if self.frame_count >= 3600 {
            let elapsed = self.start_time.unwrap().elapsed().as_secs_f64();
            let avg_fps = self.frame_count as f64 / elapsed;
            println!("Profiling completed 3600 frames in {:.2}s (Avg: {:.1} FPS)", elapsed, avg_fps);
            event_loop.exit();
            return;
        }

        // --- Simulate User Input (Dragging) ---
        let mut simulated_events = self.simulator.pump_events();
        
        // Inject synthetic window events into the engine
        if let Some(window) = self.inner.window() {
            let window_id = window.id();
            for event in simulated_events.drain(..) {
                self.inner.window_event(event_loop, window_id, event);
            }
        }

        // --- Simulate Camera Mode Switches ---
        if let Some(handle) = &self.viewer_handle {
            match self.frame_count {
                1200 => {
                    println!("[Frame {}] Switching to Cockpit mode", self.frame_count);
                    handle.camera_set_mode(CameraMode::Cockpit);
                }
                2400 => {
                    println!("[Frame {}] Switching to Tracking mode", self.frame_count);
                    handle.camera_set_mode(CameraMode::Tracking);
                }
                _ => {} // Do nothing
            }
        }

        // Run the actual engine tick
        self.inner.about_to_wait(event_loop);
        
        self.frame_count += 1;
    }
}
