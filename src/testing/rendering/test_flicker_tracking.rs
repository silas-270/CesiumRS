use crate::testing::VerifyConfig;
use cesium_engine::camera::camera::CameraMode;
use cesium_engine::core::app::App;
use cesium_engine::globe::quadtree::TileId;
use std::collections::HashMap;
use std::fs::File;
use std::io::Write;
use std::sync::{Arc, Mutex};
use std::time::Instant;
use winit::application::ApplicationHandler;
use winit::event::WindowEvent;
use winit::event_loop::ActiveEventLoop;
use winit::window::WindowId;

/// Tracks the last known texture assignment for a tile and when it last changed.
struct TileTextureHistory {
    texture_id: TileId,
    showing_own: bool,
    last_change_frame: u32,
    last_change_time: Instant,
    flip_count: u32,
}

pub struct FlickerTrackingApp<'a> {
    pub inner: App<'a>,
    pub config: VerifyConfig,
    pub setup_done: bool,
    pub progress: Arc<Mutex<f64>>,
    /// Per-tile aggregate log: written at the end.
    per_tile_log: File,
    /// Per-frame aggregate log.
    frame_log: File,
    pub frame_count: u32,
    /// Tracks previous texture assignment for every tile we have seen.
    texture_history: HashMap<TileId, TileTextureHistory>,
    /// Events where a tile changed texture faster than 100ms — the flicker list.
    flicker_events: Vec<String>,
}

impl<'a> FlickerTrackingApp<'a> {
    pub fn new(config: VerifyConfig) -> Self {
        let app_config = cesium_engine::globe::tiles::config::TileEngineConfig {
            offline_mode: false,
            mesh_cache_size: std::num::NonZeroUsize::new(config.cache_size).unwrap(),
            max_cache_size: std::num::NonZeroUsize::new(config.cache_size).unwrap(),
            enable_prefetch: config.prefetch,
            ..cesium_engine::globe::tiles::config::TileEngineConfig::default()
        };

        let progress = Arc::new(Mutex::new(0.0));
        let mut flight_app = Box::new(cesium_flight::tracker::FlightTrackerApp::new(
            progress.clone(),
        ));

        flight_app.add_flight_path("flight_FRA_STR", 8.5706, 50.0333, 9.2219, 48.6899, 1_800_000, false, Vec::new());

        flight_app.is_playing = true;
        flight_app.play_speed = 0.01;
        flight_app.view_mode = CameraMode::Tracking;

        let mut frame_log =
            File::create("flicker_frame_log.csv").expect("Failed to create flicker_frame_log.csv");
        writeln!(
            frame_log,
            "Frame,Progress,VisibleTiles,DisplayedTiles,TextureChanges,FastFlickers"
        )
        .unwrap();

        let per_tile_log = File::create("flicker_per_tile_log.csv")
            .expect("Failed to create flicker_per_tile_log.csv");

        Self {
            inner: App::new(app_config, Some(flight_app), None),
            config,
            setup_done: false,
            progress,
            per_tile_log,
            frame_log,
            frame_count: 0,
            texture_history: HashMap::new(),
            flicker_events: Vec::new(),
        }
    }
}

impl<'a> ApplicationHandler for FlickerTrackingApp<'a> {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        self.inner.resumed(event_loop);
        if !self.setup_done {
            self.setup_done = true;
        }
    }

    fn window_event(
        &mut self,
        event_loop: &ActiveEventLoop,
        window_id: WindowId,
        event: WindowEvent,
    ) {
        if let WindowEvent::RedrawRequested = event {
            self.inner
                .window_event(event_loop, window_id, WindowEvent::RedrawRequested);

            self.frame_count += 1;
            let frame = self.frame_count;
            let current_progress = *self.progress.lock().unwrap();
            let now = Instant::now();

            // Skip the first 60 frames — initial load noise
            let past_warmup = frame > 60;

            let mut texture_changes_this_frame: u32 = 0;
            let mut fast_flickers_this_frame: u32 = 0;
            let mut displayed_tiles: u32 = 0;
            let visible_count;

            if let Some(state) = self.inner.wgpu_state_mut() {
                let visible_tiles = state.quadtree_manager.get_visible_tiles();
                visible_count = visible_tiles.len();

                // Snapshot the current display_state (texture_id per mesh tile)
                // display_state is pub on WgpuState
                let current_assignments: Vec<(TileId, TileId, bool)> = state
                    .display_state
                    .iter()
                    .map(|(mesh_id, entry)| (*mesh_id, entry.texture_id, entry.showing_own_texture))
                    .collect();

                displayed_tiles = current_assignments.len() as u32;

                if past_warmup {
                    for (mesh_id, texture_id, showing_own) in &current_assignments {
                        if let Some(hist) = self.texture_history.get_mut(mesh_id) {
                            // Did the texture assignment change?
                            if hist.texture_id != *texture_id || hist.showing_own != *showing_own {
                                texture_changes_this_frame += 1;

                                // How long since the last change?
                                let ms_since_last = hist.last_change_time.elapsed().as_millis();
                                if ms_since_last < 100 {
                                    // This is a FLICKER: texture changed again in < 100ms
                                    fast_flickers_this_frame += 1;
                                    hist.flip_count += 1;

                                    let msg = format!(
                                        "FLICKER frame={} tile=({},{},{}) old_tex=({},{},{}) new_tex=({},{},{}) ms_since_last={} own={}->{}",
                                        frame,
                                        mesh_id.z, mesh_id.x, mesh_id.y,
                                        hist.texture_id.z, hist.texture_id.x, hist.texture_id.y,
                                        texture_id.z, texture_id.x, texture_id.y,
                                        ms_since_last,
                                        hist.showing_own, showing_own
                                    );
                                    self.flicker_events.push(msg);
                                }

                                hist.texture_id = *texture_id;
                                hist.showing_own = *showing_own;
                                hist.last_change_frame = frame;
                                hist.last_change_time = now;
                            }
                        } else {
                            // First time we see this tile
                            self.texture_history.insert(
                                *mesh_id,
                                TileTextureHistory {
                                    texture_id: *texture_id,
                                    showing_own: *showing_own,
                                    last_change_frame: frame,
                                    last_change_time: now,
                                    flip_count: 0,
                                },
                            );
                        }
                    }
                }
            } else {
                visible_count = 0;
            }

            writeln!(
                self.frame_log,
                "{},{:.5},{},{},{},{}",
                frame,
                current_progress,
                visible_count,
                displayed_tiles,
                texture_changes_this_frame,
                fast_flickers_this_frame
            )
            .unwrap();

            if frame.is_multiple_of(60) {
                println!(
                    "Frame {:4}: progress={:.4}  visible={}  displayed={}  changes={}  fast_flickers={}  total_flicker_events={}",
                    frame, current_progress, visible_count, displayed_tiles,
                    texture_changes_this_frame, fast_flickers_this_frame,
                    self.flicker_events.len()
                );
            }

            if current_progress >= 0.5 {
                // Write final per-tile summary
                writeln!(self.per_tile_log, "TileZ,TileX,TileY,FlipCount").unwrap();
                for (id, hist) in &self.texture_history {
                    if hist.flip_count > 0 {
                        writeln!(
                            self.per_tile_log,
                            "{},{},{},{}",
                            id.z, id.x, id.y, hist.flip_count
                        )
                        .unwrap();
                    }
                }

                // Print flicker event summary
                println!("\n=== FLICKER SUMMARY ===");
                if self.flicker_events.is_empty() {
                    println!("NO flicker events detected. Fix verified!");
                } else {
                    println!("{} flicker events detected:", self.flicker_events.len());
                    for ev in self.flicker_events.iter().take(30) {
                        println!("  {}", ev);
                    }
                    if self.flicker_events.len() > 30 {
                        println!("  ... and {} more", self.flicker_events.len() - 30);
                    }
                }
                println!("======================\n");

                println!("Progress reached 0.5. Ending flicker test.");
                event_loop.exit();
            } else {
                if let Some(window) = &self.inner.window() {
                    window.request_redraw();
                }
            }
        } else {
            self.inner.window_event(event_loop, window_id, event);
        }
    }

    fn about_to_wait(&mut self, event_loop: &ActiveEventLoop) {
        self.inner.about_to_wait(event_loop);
    }
}
