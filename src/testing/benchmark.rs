use std::time::Instant;
use winit::application::ApplicationHandler;
use winit::event::WindowEvent;
use winit::event_loop::ActiveEventLoop;
use winit::window::WindowId;

use cesium_engine::core::app::App;
use cesium_engine::globe::tiles::config::TileEngineConfig;
use cesium_engine::render::wgpu_state::SubsystemTimings;
use crate::api::{CameraMode, ViewerHandle};
use cesium_flight::tracker::FlightTrackerApp;

use crate::testing::VerifyConfig;

/// Field name/value pairs for every `SubsystemTimings` bucket, in report order.
/// Kept as one function so adding a bucket to `SubsystemTimings` only requires
/// one edit here rather than duplicating each field through percentile/JSON code.
pub(crate) fn subsystem_fields(t: &SubsystemTimings) -> [(&'static str, f64); 8] {
    [
        ("extension_update_us", t.extension_update_us),
        ("quadtree_us", t.quadtree_us),
        ("tile_streaming_us", t.tile_streaming_us),
        ("display_state_us", t.display_state_us),
        ("terrain_draw_us", t.terrain_draw_us),
        ("sky_us", t.sky_us),
        ("extension_render_us", t.extension_render_us),
        ("submit_present_us", t.submit_present_us),
    ]
}

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

pub(crate) struct Percentiles {
    pub avg: f64,
    pub p90: f64,
    pub p99: f64,
}

pub(crate) fn percentiles(samples: &[f64]) -> Percentiles {
    let mut sorted = samples.to_vec();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let len = sorted.len();
    let avg = sorted.iter().sum::<f64>() / len as f64;
    let p90_idx = ((len as f64 * 0.90) as usize).min(len - 1);
    let p99_idx = ((len as f64 * 0.99) as usize).min(len - 1);
    Percentiles {
        avg,
        p90: sorted[p90_idx],
        p99: sorted[p99_idx],
    }
}

pub struct BenchmarkApp<'a> {
    inner: App<'a>,
    frame_count: usize,
    viewer_handle: Option<ViewerHandle>,
    start_time: Option<Instant>,

    update_logic_samples: Vec<f64>,
    label_manager_samples: Vec<f64>,
    render_scene_samples: Vec<f64>,
    /// One `Vec<f64>` per `subsystem_fields` entry, same order/length.
    subsystem_samples: Vec<Vec<f64>>,
}

impl<'a> BenchmarkApp<'a> {
    pub fn new(_config: VerifyConfig) -> Self {
        let (flight_app, flight_handle) = FlightTrackerApp::with_handle();

        flight_handle.load_flight("flight_FRA_JFK", 8.5706, 50.0333, -73.7781, 40.6413, 28_800_000, None, None, Vec::new());

        // Start playing immediately. Set speed higher to fly faster through it
        flight_handle.play();
        flight_handle.set_speed(0.01);

        let app_config = TileEngineConfig {
            enable_prefetch: true,
            ..TileEngineConfig::default()
        };

        let (tx, rx) = std::sync::mpsc::sync_channel(64);
        let subsystem_count = subsystem_fields(&SubsystemTimings::default()).len();

        Self {
            inner: App::new(app_config, Some(Box::new(flight_app)), Some(rx)),
            frame_count: 0,
            viewer_handle: Some(ViewerHandle { tx }),
            start_time: None,
            update_logic_samples: Vec::with_capacity(3600),
            label_manager_samples: Vec::with_capacity(3600),
            render_scene_samples: Vec::with_capacity(3600),
            subsystem_samples: vec![Vec::with_capacity(3600); subsystem_count],
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

                for (i, (_, value)) in
                    subsystem_fields(&state.last_subsystem_timings).into_iter().enumerate()
                {
                    self.subsystem_samples[i].push(value);
                }
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

        let ul = percentiles(&self.update_logic_samples);
        let lm = percentiles(&self.label_manager_samples);
        let rs = percentiles(&self.render_scene_samples);
        let len = self.update_logic_samples.len();

        let report = BenchmarkReport {
            average_update_logic_us: ul.avg,
            p90_update_logic_us: ul.p90,
            p99_update_logic_us: ul.p99,

            average_label_manager_us: lm.avg,
            p90_label_manager_us: lm.p90,
            p99_label_manager_us: lm.p99,

            average_render_scene_us: rs.avg,
            p90_render_scene_us: rs.p90,
            p99_render_scene_us: rs.p99,

            total_frames: len,
        };

        let field_names = subsystem_fields(&SubsystemTimings::default());
        let subsystem_json: String = self
            .subsystem_samples
            .iter()
            .enumerate()
            .map(|(i, samples)| {
                let p = percentiles(samples);
                format!(
                    r#"    "{}": {{ "average_us": {:.2}, "p90_us": {:.2}, "p99_us": {:.2} }}"#,
                    field_names[i].0, p.avg, p.p90, p.p99
                )
            })
            .collect::<Vec<_>>()
            .join(",\n");

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
  "total_frames": {},
  "subsystems": {{
{}
  }}
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
            report.total_frames,
            subsystem_json,
        );

        let _ = std::fs::write("benchmark_report.json", &json);
        println!("Benchmark report generated: benchmark_report.json");
        println!("{}", json);
    }
}
