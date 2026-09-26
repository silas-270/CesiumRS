//! Windowed visual check that city labels are not painted over the aircraft.
//!
//! The labels are an egui overlay drawn after the 3D scene, and the headless path has no
//! egui at all, so this one needs a real window: it drives the normal [`App`], frames the
//! aircraft from above with the ground (and its labels) behind it, and on each shot's
//! last frame renders with the label layer into a PNG. Not an assertion — look at the
//! pictures: no label pill should cover the aircraft.
//!
//! Run with
//! `cargo test --release --lib label_occlusion -- --nocapture --ignored`.

use cesium_engine::camera::CameraMode;
use cesium_engine::core::app::App;
use cesium_engine::globe::tiles::config::{TerrainConfig, TileEngineConfig};
use cesium_flight::flight_handle::FlightHandle;
use cesium_flight::tracker::FlightTrackerApp;
use glam::Vec3;
use winit::application::ApplicationHandler;
use winit::event::WindowEvent;
use winit::event_loop::{ActiveEventLoop, EventLoop};
use winit::platform::wayland::EventLoopBuilderExtWayland;
use winit::window::WindowId;

/// Frames rendered before each capture, for the tiles and labels to settle.
const SETTLE_FRAMES: u32 = 240;

struct Shot {
    name: &'static str,
    progress: f64,
    dist_m: f32,
    pitch_deg: f32,
    yaw_deg: f32,
}

const SHOTS: &[Shot] = &[
    // Final approach into LHR: the aircraft passes over London's label.
    Shot { name: "00_top_300km", progress: 0.985, dist_m: 300_000.0, pitch_deg: 85.0, yaw_deg: 0.0 },
    Shot { name: "01_top_600km", progress: 0.975, dist_m: 600_000.0, pitch_deg: 88.0, yaw_deg: 0.0 },
    Shot { name: "02_oblique_120km", progress: 0.98, dist_m: 120_000.0, pitch_deg: 45.0, yaw_deg: 180.0 },
    Shot { name: "03_chase_8km", progress: 0.98, dist_m: 8_000.0, pitch_deg: 25.0, yaw_deg: 180.0 },
];

struct LabelOcclusionApp<'a> {
    inner: App<'a>,
    flight: FlightHandle,
    dir: String,
    shot: usize,
    frame: u32,
}

impl<'a> ApplicationHandler for LabelOcclusionApp<'a> {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        self.inner.resumed(event_loop);
    }

    fn window_event(&mut self, event_loop: &ActiveEventLoop, window_id: WindowId, event: WindowEvent) {
        if !matches!(event, WindowEvent::RedrawRequested) {
            self.inner.window_event(event_loop, window_id, event);
            return;
        }
        let Some(shot) = SHOTS.get(self.shot) else {
            event_loop.exit();
            return;
        };

        self.flight.set_progress(shot.progress);
        let dist = shot.dist_m / 1_000_000.0;
        let (pitch, yaw) = (shot.pitch_deg.to_radians(), shot.yaw_deg.to_radians());
        let Some(state) = self.inner.wgpu_state_mut() else { return };
        state.camera.mode = CameraMode::Tracking;
        state.camera.local_pos = Vec3::new(
            dist * pitch.cos() * yaw.sin(),
            dist * pitch.sin(),
            dist * pitch.cos() * yaw.cos(),
        );
        state.camera.look_at_plane();

        self.frame += 1;
        if self.frame < SETTLE_FRAMES {
            self.inner.window_event(event_loop, window_id, event);
            return;
        }

        let out = format!("{}/{}.png", self.dir, shot.name);
        state
            .render(Some(&out), false, |ctx, s| App::render_label_indicators(ctx, s))
            .expect("render failed");
        println!(
            "[{}] visible labels {} -> {out}",
            shot.name,
            state.label_manager.visible_labels.len()
        );
        self.shot += 1;
        self.frame = 0;
    }

    fn exiting(&mut self, event_loop: &ActiveEventLoop) {
        self.inner.exiting(event_loop);
    }

    fn about_to_wait(&mut self, event_loop: &ActiveEventLoop) {
        self.inner.about_to_wait(event_loop);
        if let Some(w) = self.inner.window() {
            w.request_redraw();
        }
    }
}

#[test]
#[ignore = "opens a window and writes PNGs; run explicitly"]
fn label_occlusion_capture() {
    let dir = std::env::var("LABEL_OCCLUSION_DIR").unwrap_or_else(|_| "label_occlusion".to_string());
    std::fs::create_dir_all(&dir).unwrap();

    let (flight_app, flight) = FlightTrackerApp::with_handle();
    flight.load_route_def(&cesium_flight::preset::parse_route("JFK-LHR").unwrap());
    flight.pause();

    let config = TileEngineConfig {
        terrain: TerrainConfig { enabled: false, ..TerrainConfig::default() },
        ..TileEngineConfig::default()
    };
    let mut app = LabelOcclusionApp {
        inner: App::new(config, Some(Box::new(flight_app)), None),
        flight,
        dir,
        shot: 0,
        frame: 0,
    };
    let event_loop = EventLoop::builder().with_any_thread(true).build().unwrap();
    event_loop.run_app(&mut app).unwrap();
}
