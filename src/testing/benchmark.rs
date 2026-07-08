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

#[derive(Default, Debug)]
pub struct BenchmarkReport {
    pub average_update_logic_us: f64,
    pub p90_update_logic_us: f64,
    pub p99_update_logic_us: f64,

    pub average_label_manager_us: f64,
    pub p90_label_manager_us: f64,
    pub p99_label_manager_us: f64,

    pub average_render_scene_us: f64,
    pub p90_render_scene_us: f64,
    pub p99_render_scene_us: f64,

    pub total_frames: usize,
}

pub struct BenchmarkApp<'a> {
    inner: App<'a>,
    frame_count: usize,
    viewer_handle: Option<ViewerHandle>,
    start_time: Option<Instant>,
    
    update_logic_samples: Vec<f64>,
    label_manager_samples: Vec<f64>,
    render_scene_samples: Vec<f64>,
}

impl<'a> BenchmarkApp<'a> {
    pub fn new(_config: VerifyConfig) -> Self {
        let (flight_app, flight_handle) = FlightTrackerApp::with_handle();

        // Load the sample flight for the tracker
        if let Ok(content) = std::fs::read_to_string("flight_FRA_JFK.json") {
            flight_handle.load_flight("flight_FRA_JFK", content);
        } else {
            eprintln!("Warning: flight_FRA_JFK.json not found!");
        }

        // Start playing immediately. Set speed higher to fly faster through it
        flight_handle.play();
        flight_handle.set_speed(0.01);

        let app_config = TileEngineConfig {
            enable_prefetch: true,
            ..TileEngineConfig::default()
        };
        
        let (tx, rx) = std::sync::mpsc::sync_channel(64);
        
        Self {
            inner: App::new(app_config, Some(Box::new(flight_app)), Some(rx)),
            frame_count: 0,
            viewer_handle: Some(ViewerHandle { tx }),
            start_time: None,
            update_logic_samples: Vec::with_capacity(3600),
            label_manager_samples: Vec::with_capacity(3600),
            render_scene_samples: Vec::with_capacity(3600),
        }
    }
}

impl<'a> ApplicationHandler for BenchmarkApp<'a> {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        self.inner.resumed(event_loop);
        
        if self.start_time.is_none() {
            self.start_time = Some(Instant::now());
            if let Some(handle) = &self.viewer_handle {
                handle.camera_set_mode(CameraMode::Tracking);
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
        // Exit after 3600 frames
        if self.frame_count >= 3600 {
            self.generate_report();
            event_loop.exit();
            return;
        }

        self.inner.about_to_wait(event_loop);

        if let Some(state) = self.inner.render_state() {
            let timings = state.last_timings;
            // Ignore the first 60 frames as warm-up
            if self.frame_count > 60 {
                self.update_logic_samples.push(timings.update_logic_us);
                self.label_manager_samples.push(timings.label_manager_us);
                self.render_scene_samples.push(timings.render_scene_us);
            }
        }
        
        self.frame_count += 1;
    }
}

impl<'a> BenchmarkApp<'a> {
    fn generate_report(&mut self) {
        if self.update_logic_samples.is_empty() {
            return;
        }
        
        let mut ul = self.update_logic_samples.clone();
        let mut lm = self.label_manager_samples.clone();
        let mut rs = self.render_scene_samples.clone();
        
        // Sort to calculate percentiles
        ul.sort_by(|a, b| a.partial_cmp(b).unwrap());
        lm.sort_by(|a, b| a.partial_cmp(b).unwrap());
        rs.sort_by(|a, b| a.partial_cmp(b).unwrap());
        
        let len = ul.len();
        let p90_idx = (len as f64 * 0.90) as usize;
        let p99_idx = (len as f64 * 0.99) as usize;
        
        let avg = |v: &[f64]| v.iter().sum::<f64>() / len as f64;
        
        let report = BenchmarkReport {
            average_update_logic_us: avg(&ul),
            p90_update_logic_us: ul[p90_idx.min(len - 1)],
            p99_update_logic_us: ul[p99_idx.min(len - 1)],

            average_label_manager_us: avg(&lm),
            p90_label_manager_us: lm[p90_idx.min(len - 1)],
            p99_label_manager_us: lm[p99_idx.min(len - 1)],

            average_render_scene_us: avg(&rs),
            p90_render_scene_us: rs[p90_idx.min(len - 1)],
            p99_render_scene_us: rs[p99_idx.min(len - 1)],

            total_frames: len,
        };
        
        let json = format!(
            r#"{{
  "average_update_logic_us": {:.2},
  "p90_update_logic_us": {:.2},
  "p99_update_logic_us": {:.2},
  "average_label_manager_us": {:.2},
  "p90_label_manager_us": {:.2},
  "p99_label_manager_us": {:.2},
  "average_render_scene_us": {:.2},
  "p90_render_scene_us": {:.2},
  "p99_render_scene_us": {:.2},
  "total_frames": {}
}}"#,
            report.average_update_logic_us,
            report.p90_update_logic_us,
            report.p99_update_logic_us,
            report.average_label_manager_us,
            report.p90_label_manager_us,
            report.p99_label_manager_us,
            report.average_render_scene_us,
            report.p90_render_scene_us,
            report.p99_render_scene_us,
            report.total_frames
        );
        
        let _ = std::fs::write("benchmark_report.json", &json);
        println!("Benchmark report generated: benchmark_report.json");
        println!("{}", json);
    }
}
