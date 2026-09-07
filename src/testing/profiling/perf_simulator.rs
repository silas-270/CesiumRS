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

use crate::testing::benchmark::{percentiles, subsystem_fields};
use crate::testing::VerifyConfig;
use crate::testing::harness::simulator::Simulator;

/// Frame ranges matching the scripted mode switches below — [start, end).
const MODE_WINDOWS: [(&str, u64, u64); 3] = [
    ("Free", 0, 1200),
    ("Cockpit", 1200, 2400),
    ("Tracking", 2400, 3600),
];

pub struct PerfSimulatorApp<'a> {
    inner: App<'a>,
    simulator: Simulator,
    frame_count: u64,
    start_time: Option<Instant>,
    viewer_handle: Option<ViewerHandle>,

    /// Per-mode-window samples, indexed the same as `MODE_WINDOWS`.
    update_logic_samples: [Vec<f64>; 3],
    render_scene_samples: [Vec<f64>; 3],
    /// `[window][subsystem_field]`.
    subsystem_samples: [Vec<Vec<f64>>; 3],
}

fn current_window(frame_count: u64) -> Option<usize> {
    MODE_WINDOWS
        .iter()
        .position(|(_, start, end)| frame_count >= *start && frame_count < *end)
}

impl<'a> PerfSimulatorApp<'a> {
    pub fn new(_config: VerifyConfig) -> Self {
        // Build the flight app & handle
        let (flight_app, flight_handle) = FlightTrackerApp::with_handle();

        flight_handle.load_flight("flight_FRA_STR", 8.5706, 50.0333, 9.2219, 48.6899, 1_800_000, None, None, Vec::new());

        // Start playing immediately. Set speed so the flight lasts ~100 seconds
        // (so it's still flying when we switch modes at 20s and 40s)
        flight_handle.play();
        flight_handle.set_speed(0.01);

        // Set up the simulator script for Free Mode (1200 frames):
        // Just wait 1200 frames so the user can perform manual camera movements
        let simulator = Simulator::parse("wait:1200");

        let app_config = TileEngineConfig {
            enable_prefetch: true,
            ..TileEngineConfig::default()
        };

        // We initialize the App with the FlightTracker extension.
        // We will manually inject a ViewerHandle's receiver so we can send commands.
        let (tx, rx) = std::sync::mpsc::sync_channel(64);
        let subsystem_count = subsystem_fields(&SubsystemTimings::default()).len();

        Self {
            inner: App::new(app_config, Some(Box::new(flight_app)), Some(rx)),
            simulator,
            frame_count: 0,
            start_time: None,
            viewer_handle: Some(ViewerHandle { tx }),
            update_logic_samples: Default::default(),
            render_scene_samples: Default::default(),
            subsystem_samples: [
                vec![Vec::with_capacity(1200); subsystem_count],
                vec![Vec::with_capacity(1200); subsystem_count],
                vec![Vec::with_capacity(1200); subsystem_count],
            ],
        }
    }
}

impl<'a> ApplicationHandler for PerfSimulatorApp<'a> {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        self.inner.resumed(event_loop);

        if self.start_time.is_none() {
            self.start_time = Some(Instant::now());

            // Set initial position (Frankfurt, Europe) at 20 Megameters altitude (20,000,000 meters)
            if let Some(handle) = &self.viewer_handle {
                handle.camera_set_position(8.68, 50.11, 20_000_000.0);
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
            self.print_and_write_breakdown();
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

        // Skip the first 60 frames of each window as mode-switch warm-up.
        if let Some(window_idx) = current_window(self.frame_count) {
            let (_, window_start, _) = MODE_WINDOWS[window_idx];
            if self.frame_count >= window_start + 60 {
                if let Some(state) = self.inner.render_state() {
                    let timings = state.last_timings;
                    self.update_logic_samples[window_idx].push(timings.update_logic_us);
                    self.render_scene_samples[window_idx].push(timings.render_scene_us);
                    for (i, (_, value)) in
                        subsystem_fields(&state.last_subsystem_timings).into_iter().enumerate()
                    {
                        self.subsystem_samples[window_idx][i].push(value);
                    }
                }
            }
        }

        self.frame_count += 1;
    }
}

impl<'a> PerfSimulatorApp<'a> {
    fn print_and_write_breakdown(&self) {
        let field_names = subsystem_fields(&SubsystemTimings::default());
        let mut windows_json = Vec::new();

        for (window_idx, (mode_name, _, _)) in MODE_WINDOWS.iter().enumerate() {
            if self.update_logic_samples[window_idx].is_empty() {
                continue;
            }
            let ul = percentiles(&self.update_logic_samples[window_idx]);
            let rs = percentiles(&self.render_scene_samples[window_idx]);

            println!("\n=== {mode_name} mode (frames {}-{}) ===", MODE_WINDOWS[window_idx].1, MODE_WINDOWS[window_idx].2);
            println!("  update_logic_us: avg={:.1} p90={:.1} p99={:.1}", ul.avg, ul.p90, ul.p99);
            println!("  render_scene_us: avg={:.1} p90={:.1} p99={:.1}", rs.avg, rs.p90, rs.p99);

            let subsystem_json: String = self.subsystem_samples[window_idx]
                .iter()
                .enumerate()
                .map(|(i, samples)| {
                    let p = percentiles(samples);
                    println!("  {}: avg={:.1} p90={:.1} p99={:.1}", field_names[i].0, p.avg, p.p90, p.p99);
                    format!(
                        r#"      "{}": {{ "average_us": {:.2}, "p90_us": {:.2}, "p99_us": {:.2} }}"#,
                        field_names[i].0, p.avg, p.p90, p.p99
                    )
                })
                .collect::<Vec<_>>()
                .join(",\n");

            windows_json.push(format!(
                r#"  "{}": {{
    "average_update_logic_us": {:.2},
    "p90_update_logic_us": {:.2},
    "p99_update_logic_us": {:.2},
    "average_render_scene_us": {:.2},
    "p90_render_scene_us": {:.2},
    "p99_render_scene_us": {:.2},
    "subsystems": {{
{}
    }}
  }}"#,
                mode_name, ul.avg, ul.p90, ul.p99, rs.avg, rs.p90, rs.p99, subsystem_json
            ));
        }

        let json = format!("{{\n{}\n}}", windows_json.join(",\n"));
        let _ = std::fs::write("perf_simulator_report.json", &json);
        println!("\nPer-mode breakdown written to perf_simulator_report.json");
    }
}
