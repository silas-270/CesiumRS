use cesium_engine::camera::CameraMode;
use cesium_engine::core::app::App;
use cesium_engine::globe::tiles::config::{TerrainConfig, TileEngineConfig, SATELLITE_IMAGERY_URL};
use cesium_flight::flight_handle::FlightHandle;
use cesium_flight::tracker::FlightTrackerApp;
use glam::Vec3;
use std::num::NonZeroUsize;
use std::time::Instant;
use winit::application::ApplicationHandler;
use winit::event::WindowEvent;
use winit::event_loop::ActiveEventLoop;
use winit::window::WindowId;

const DURATION: f64 = 60.0;

pub struct TrackingOrbitApp<'a> {
    inner: App<'a>,
    flight: FlightHandle,
    start: Option<Instant>,
    last_frame: Option<Instant>,
    frame_ms: Vec<(f64, f64, f64)>,
}

impl<'a> TrackingOrbitApp<'a> {
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
            base_imagery_url: SATELLITE_IMAGERY_URL.to_string(),
            terrain: TerrainConfig {
                enabled: true,
                ..TerrainConfig::default()
            },
            ..TileEngineConfig::default()
        };

        Self {
            inner: App::new(config, Some(Box::new(flight_app)), None),
            flight,
            start: None,
            last_frame: None,
            frame_ms: Vec::new(),
        }
    }

    fn summary(&self) {
        let mut dts: Vec<f64> = self.frame_ms.iter().map(|f| f.1).collect();
        dts.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let p = |q: f64| dts[((dts.len() as f64 * q) as usize).min(dts.len() - 1)];
        println!(
            "\n{} frames  avg {:.1} FPS | dt p50 {:.1} ms  p90 {:.1}  p99 {:.1}  max {:.1} | >33ms: {}  >100ms: {}",
            dts.len(),
            dts.len() as f64 / DURATION,
            p(0.5),
            p(0.9),
            p(0.99),
            p(1.0),
            dts.iter().filter(|&&d| d > 33.0).count(),
            dts.iter().filter(|&&d| d > 100.0).count(),
        );
    }
}

impl<'a> ApplicationHandler for TrackingOrbitApp<'a> {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        self.inner.resumed(event_loop);
    }

    fn window_event(&mut self, event_loop: &ActiveEventLoop, window_id: WindowId, event: WindowEvent) {
        if !matches!(event, WindowEvent::RedrawRequested) {
            self.inner.window_event(event_loop, window_id, event);
            return;
        }

        let now = Instant::now();
        let t = self.start.get_or_insert(now).elapsed().as_secs_f64();
        if t >= DURATION {
            self.summary();
            event_loop.exit();
            return;
        }

        self.flight.set_progress(0.25 * t / DURATION);

        let s = t / DURATION;
        let yaw = (4.0 * std::f64::consts::TAU * s) as f32;
        let pitch = 20f32.to_radians();
        let dist_m = 800.0 + 4200.0 * (std::f64::consts::PI * s).sin();
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

        let frame_start = Instant::now();
        self.inner.window_event(event_loop, window_id, event);
        let work_ms = frame_start.elapsed().as_secs_f64() * 1000.0;

        let dt_ms = self.last_frame.map_or(0.0, |l| (now - l).as_secs_f64() * 1000.0);
        self.last_frame = Some(now);
        self.frame_ms.push((t, dt_ms, work_ms));

        if let Some(state) = self.inner.wgpu_state_mut() {
            let (visible, missing) = state.get_fetch_stats();
            let stream = state.last_subsystem_timings.tile_streaming_us / 1000.0;
            if dt_ms > 50.0 {
                println!(
                    "t={t:5.1}s  HITCH dt={dt_ms:6.1}ms  work={work_ms:6.1}ms  stream={stream:6.1}ms  dist={dist_m:5.0}m  vis={visible} miss={missing}"
                );
            }
            if let Some(w) = self.inner.window() {
                w.set_title(&format!(
                    "tracking-orbit t={t:.1}s dt={dt_ms:.0}ms stream={stream:.1}ms dist={dist_m:.0}m vis={visible} miss={missing}"
                ));
            }
        }
    }

    fn about_to_wait(&mut self, event_loop: &ActiveEventLoop) {
        self.inner.about_to_wait(event_loop);
        if let Some(w) = self.inner.window() {
            w.request_redraw();
        }
    }
}
