use cesium_engine::camera::CameraMode;
use cesium_engine::core::app::App;
use cesium_engine::globe::tiles::config::{TerrainConfig, TileEngineConfig, satellite_imagery_url};
use cesium_flight::flight_handle::FlightHandle;
use cesium_flight::tracker::FlightTrackerApp;
use glam::Vec3;
use std::num::NonZeroUsize;
use std::time::Instant;
use winit::application::ApplicationHandler;
use winit::event::WindowEvent;
use winit::event_loop::ActiveEventLoop;
use winit::window::WindowId;

const LAP_SECS: f64 = 20.0;
// (name, orbit distance m, pitch deg)
const LAPS: [(&str, f64, f32); 4] = [
    ("low_lap1", 400.0, 6.0),
    ("low_lap2", 400.0, 6.0),
    ("high_lap1", 3000.0, 15.0),
    ("high_lap2", 3000.0, 15.0),
];

pub struct RevisitApp<'a> {
    inner: App<'a>,
    flight: FlightHandle,
    start: Option<Instant>,
    lap: Option<usize>,
}

impl<'a> RevisitApp<'a> {
    pub fn new(_cfg: crate::testing::VerifyConfig) -> Self {
        let (flight_app, flight) = FlightTrackerApp::with_handle();
        flight.load_route_def(&cesium_flight::preset::parse_route("STR-FRA").unwrap());
        flight.pause();
        flight.set_progress(0.0);

        let config = TileEngineConfig {
            max_cache_size: NonZeroUsize::new(2048).unwrap(),
            mesh_cache_size: NonZeroUsize::new(cesium_engine::globe::tiles::config::MESH_CACHE_ENTRIES).unwrap(),
            target_texel_ratio: 1.0,
            enable_prefetch: true,
            base_imagery_url: satellite_imagery_url(),
            terrain: TerrainConfig {
                enabled: true,
                ..TerrainConfig::default()
            },
            ..TileEngineConfig::default()
        };

        Self {
            // The camera and tile traces are what this test produces.
            inner: App::new(config, Some(Box::new(flight_app)), None).with_traces(true),
            flight,
            start: None,
            lap: None,
        }
    }
}

impl<'a> ApplicationHandler for RevisitApp<'a> {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        self.inner.resumed(event_loop);
    }

    fn window_event(&mut self, event_loop: &ActiveEventLoop, window_id: WindowId, event: WindowEvent) {
        if !matches!(event, WindowEvent::RedrawRequested) {
            self.inner.window_event(event_loop, window_id, event);
            return;
        }

        let t = self.start.get_or_insert_with(Instant::now).elapsed().as_secs_f64();
        let lap = (t / LAP_SECS) as usize;
        if lap >= LAPS.len() {
            cesium_engine::tile_event!("mark", "MARK", None, "end");
            println!("done: tile_trace.csv");
            event_loop.exit();
            return;
        }
        if self.lap != Some(lap) {
            self.lap = Some(lap);
            cesium_engine::tile_event!("mark", "MARK", None, "{}", LAPS[lap].0);
            println!("t={t:.1}s {}", LAPS[lap].0);
        }

        self.flight.set_progress(0.0);
        let (_, dist_m, pitch_deg) = LAPS[lap];
        let yaw = (std::f64::consts::TAU * (t % LAP_SECS) / LAP_SECS) as f32;
        let pitch = pitch_deg.to_radians();
        let dist = (dist_m / 1_000_000.0) as f32;
        if let Some(state) = self.inner.wgpu_state_mut() {
            state.camera.mode = CameraMode::Tracking;
            state.camera.local_pos = Vec3::new(
                dist * pitch.cos() * yaw.sin(),
                dist * pitch.sin(),
                dist * pitch.cos() * yaw.cos(),
            );
            state.camera.look_at_plane();
        }
        self.inner.window_event(event_loop, window_id, event);
    }

    // The GPU state has to go while the event loop (and with it the Wayland

    // connection its EGL surface references) still exists; dropped afterwards it

    // corrupted the heap ("corrupted size vs. prev_size") and hung on exit.

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
