use crate::testing::harness::simulator::Simulator;
use cesium_engine::core::app::App;
use cesium_engine::globe::tiles::config::{TerrainConfig, TileEngineConfig, SATELLITE_IMAGERY_URL};
use cesium_flight::flight_handle::FlightHandle;
use cesium_flight::tracker::FlightTrackerApp;
use std::num::NonZeroUsize;
use winit::application::ApplicationHandler;
use winit::event::WindowEvent;
use winit::event_loop::ActiveEventLoop;
use winit::window::WindowId;

// Aircraft parked at Innsbruck (valley floor 580 m, slopes rising within a few km). The
// camera is dragged down into the ground while orbiting — slowly, then fast — at 250 m,
// then zoomed out to ~1.7 km and ~5 km and dragged the same way, so the orbit sweeps
// across the valley sides. Everything goes through the real mouse path.
const DRAGS: &str = "drag:100,500->2500,380:600;drag:100,500->2500,380:200;";
const SHOT_EVERY: u64 = 120;

pub struct CollideApp<'a> {
    inner: App<'a>,
    flight: FlightHandle,
    sim: Simulator,
    frame: u64,
    shot_dir: String,
}

impl<'a> CollideApp<'a> {
    pub fn new(_cfg: crate::testing::VerifyConfig) -> Self {
        let (flight_app, flight) = FlightTrackerApp::with_handle();
        flight.load_route_def(&cesium_flight::preset::parse_route("47.2602,11.3440,48.3538,11.7861").unwrap());
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
        let script = format!(
            "wait:300;{DRAGS}scroll:-1:20;wait:120;{DRAGS}scroll:-1:12;wait:120;{DRAGS}wait:60"
        );
        let shot_dir = std::env::var("CESIUM_SHOT_DIR").unwrap_or_else(|_| "collide_shots".into());
        let _ = std::fs::create_dir_all(&shot_dir);

        Self {
            inner: App::new(config, Some(Box::new(flight_app)), None),
            flight,
            sim: Simulator::parse(&script),
            frame: 0,
            shot_dir,
        }
    }
}

impl<'a> ApplicationHandler for CollideApp<'a> {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        self.inner.resumed(event_loop);
    }

    fn window_event(&mut self, event_loop: &ActiveEventLoop, window_id: WindowId, event: WindowEvent) {
        if !matches!(event, WindowEvent::RedrawRequested) {
            self.inner.window_event(event_loop, window_id, event);
            return;
        }
        if self.sim.actions.is_empty() {
            println!("done: camera_trace.csv, shots in {}", self.shot_dir);
            event_loop.exit();
            return;
        }
        self.flight.set_progress(0.0);
        for ev in self.sim.pump_events() {
            self.inner.window_event(event_loop, window_id, ev);
        }

        self.frame += 1;
        let shot = (self.frame % SHOT_EVERY == 0)
            .then(|| format!("{}/frame_{:05}.png", self.shot_dir, self.frame));
        if let Some(state) = self.inner.wgpu_state_mut() {
            #[cfg(feature = "debug_panel")]
            let res = state.render(shot.as_deref(), false, |_, _| {});
            #[cfg(not(feature = "debug_panel"))]
            let res = state.render(shot.as_deref(), false);
            if let Err(e) = res {
                log::error!("render: {e:?}");
            }
        }
        if let Some(w) = self.inner.window() {
            w.request_redraw();
        }
    }

    fn about_to_wait(&mut self, event_loop: &ActiveEventLoop) {
        self.inner.about_to_wait(event_loop);
        if let Some(w) = self.inner.window() {
            w.request_redraw();
        }
    }
}
