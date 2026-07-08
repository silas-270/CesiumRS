/// Comprehensive tile & texture monitor.
///
/// Tracks:
///   a) Every time a tile enters/leaves the visible set, or its zoom level changes
///      (indicating a resolution/LOD change).
///   b) Every time a tile's display_state changes (texture_id, uv_scale_offset, or
///      showing_own_texture flag).
///
/// Runs the flight at play_speed=0.01 (on-screen, not headless).
/// Stops when progress reaches 0.5.
/// Writes detailed CSV logs and prints a concise analysis at the end.

use cesium_engine::core::app::App;
use crate::testing::VerifyConfig;
use cesium_engine::camera::camera::CameraMode;
use cesium_engine::globe::quadtree::TileId;
use winit::application::ApplicationHandler;
use winit::event::WindowEvent;
use winit::event_loop::ActiveEventLoop;
use winit::window::WindowId;
use std::collections::{HashMap, HashSet};
use std::sync::{Arc, Mutex};
use std::fs::File;
use std::io::Write;

// ─── per-tile history structs ────────────────────────────────────────────────

/// Tracks presence in the visible set and its LOD zoom level.
struct TileVisibilityHistory {
    last_z: u8,
    last_change_frame: u32,
}

/// A point-in-time snapshot of a tile's display_state entry.
#[derive(Clone, PartialEq)]
struct TileDisplaySnapshot {
    texture_id: TileId,
    uv_key: String,
    showing_own: bool,
}

impl TileDisplaySnapshot {
    fn from_entry(texture_id: TileId, uv: [f32; 4], showing_own: bool) -> Self {
        Self {
            texture_id,
            uv_key: format!("{:.4},{:.4},{:.4},{:.4}", uv[0], uv[1], uv[2], uv[3]),
            showing_own,
        }
    }
}

// ─── rapid-change event bucket ────────────────────────────────────────────────

struct TileEventBucket {
    events: Vec<(u32, String, String)>,
}

impl TileEventBucket {
    fn new() -> Self { Self { events: Vec::new() } }
    fn push(&mut self, frame: u32, kind: &str, detail: String) {
        self.events.push((frame, kind.to_string(), detail));
    }
}

// ─── main app ────────────────────────────────────────────────────────────────

pub struct TileMonitorApp<'a> {
    pub inner: App<'a>,
    pub config: VerifyConfig,
    pub setup_done: bool,
    pub progress: Arc<Mutex<f64>>,
    pub frame_count: u32,

    visibility_log: File,
    texture_log:    File,

    vis_history:     HashMap<TileId, TileVisibilityHistory>,
    display_history: HashMap<TileId, TileDisplaySnapshot>,

    vis_buckets: HashMap<TileId, TileEventBucket>,
    tex_buckets: HashMap<TileId, TileEventBucket>,
}

impl<'a> TileMonitorApp<'a> {
    pub fn new(config: VerifyConfig) -> Self {
        let mut app_config =
            cesium_engine::globe::tiles::config::TileEngineConfig::default();
        app_config.offline_mode    = false;
        app_config.mesh_cache_size = std::num::NonZeroUsize::new(512).unwrap();
        app_config.max_cache_size  = std::num::NonZeroUsize::new(512).unwrap();
        app_config.enable_prefetch = false;

        let progress = Arc::new(Mutex::new(0.0_f64));
        let mut flight_app =
            Box::new(cesium_flight::tracker::FlightTrackerApp::new(progress.clone()));

        if let Ok(content) = std::fs::read_to_string("flight_FRA_STR.json") {
            flight_app.add_flight_path("flight_FRA_STR.json", content, false);
        } else {
            eprintln!("[TileMonitor] WARNING: flight_FRA_STR.json not found.");
        }
        flight_app.is_playing = true;
        flight_app.play_speed = 0.01;
        flight_app.view_mode  = CameraMode::Tracking;

        let mut visibility_log =
            File::create("tile_monitor_visibility.csv")
                .expect("failed to create tile_monitor_visibility.csv");
        writeln!(visibility_log, "Frame,Progress,TileZ,TileX,TileY,Event,Detail").unwrap();

        let mut texture_log =
            File::create("tile_monitor_texture.csv")
                .expect("failed to create tile_monitor_texture.csv");
        writeln!(texture_log,
            "Frame,Progress,TileZ,TileX,TileY,Event,\
             OldTexZ,OldTexX,OldTexY,OldOwnTex,\
             NewTexZ,NewTexX,NewTexY,NewOwnTex,NewUV"
        ).unwrap();

        Self {
            inner: App::new(app_config, Some(flight_app), None),
            config,
            setup_done: false,
            progress,
            frame_count: 0,
            visibility_log,
            texture_log,
            vis_history:     HashMap::new(),
            display_history: HashMap::new(),
            vis_buckets:     HashMap::new(),
            tex_buckets:     HashMap::new(),
        }
    }

    // ── logging helpers ────────────────────────────────────────────────────

    fn log_vis(&mut self, frame: u32, progress: f64,
               id: TileId, event: &str, detail: &str) {
        writeln!(self.visibility_log,
            "{},{:.5},{},{},{},{},{}",
            frame, progress, id.z, id.x, id.y, event, detail
        ).unwrap();
        self.vis_buckets
            .entry(id)
            .or_insert_with(TileEventBucket::new)
            .push(frame, event, detail.to_string());
    }

    fn log_tex(&mut self, frame: u32, progress: f64,
               id: TileId, event: &str,
               old: Option<&TileDisplaySnapshot>,
               new: &TileDisplaySnapshot) {
        let (oz, ox, oy, oown) = if let Some(o) = old {
            (o.texture_id.z, o.texture_id.x, o.texture_id.y, o.showing_own as u8)
        } else {
            (0, 0, 0, 0)
        };
        writeln!(self.texture_log,
            "{},{:.5},{},{},{},{},{},{},{},{},{},{},{},{},{}",
            frame, progress,
            id.z, id.x, id.y,
            event,
            oz, ox, oy, oown,
            new.texture_id.z, new.texture_id.x, new.texture_id.y,
            new.showing_own as u8,
            new.uv_key
        ).unwrap();
        let detail = format!(
            "tex=({},{},{}) own={} uv={}",
            new.texture_id.z, new.texture_id.x, new.texture_id.y,
            new.showing_own,
            new.uv_key
        );
        self.tex_buckets
            .entry(id)
            .or_insert_with(TileEventBucket::new)
            .push(frame, event, detail);
    }

    // ── per-frame processing — borrow-safe version ─────────────────────────
    // We snapshot all data from &mut WgpuState in one block, then drop the
    // borrow before calling self.log_* (which borrows self mutably again).

    fn process_frame(&mut self, frame: u32, progress: f64) {
        // ── collect raw data from WgpuState ────────────────────────────────
        let (visible_ids, display_entries): (
            HashSet<TileId>,
            Vec<(TileId, TileId, [f32; 4], bool)>,
        ) = if let Some(state) = self.inner.wgpu_state_mut() {
            let vis_ids = state
                .quadtree_manager
                .get_visible_tiles()
                .into_iter()
                .map(|(id, _, _)| id)
                .collect::<HashSet<_>>();

            let disp: Vec<(TileId, TileId, [f32; 4], bool)> = state
                .display_state
                .iter()
                .map(|(mesh_id, entry)| {
                    (*mesh_id,
                     entry.texture_id,
                     entry.uv_scale_offset,
                     entry.showing_own_texture)
                })
                .collect();

            (vis_ids, disp)
        } else {
            return;
        };
        // WgpuState borrow is now fully released.

        // ── a) visibility / LOD changes ────────────────────────────────────

        // newly appeared or LOD-changed tiles
        for id in &visible_ids {
            if let Some(hist) = self.vis_history.get_mut(id) {
                if hist.last_z != id.z {
                    let event = if id.z > hist.last_z { "LOD_UP" } else { "LOD_DOWN" };
                    let detail = format!("z_old={} z_new={}", hist.last_z, id.z);
                    self.log_vis(frame, progress, *id, event, &detail);
                    self.vis_history.get_mut(id).unwrap().last_z = id.z;
                    self.vis_history.get_mut(id).unwrap().last_change_frame = frame;
                }
            } else {
                let detail = format!("z={}", id.z);
                self.log_vis(frame, progress, *id, "APPEAR", &detail);
                self.vis_history.insert(
                    *id,
                    TileVisibilityHistory { last_z: id.z, last_change_frame: frame },
                );
            }
        }

        // disappeared tiles
        let disappeared: Vec<TileId> = self
            .vis_history
            .keys()
            .filter(|id| !visible_ids.contains(*id))
            .copied()
            .collect();
        for id in disappeared {
            let (z, last_f) = {
                let h = &self.vis_history[&id];
                (h.last_z, h.last_change_frame)
            };
            let detail = format!("z={} last_seen_frame={}", z, last_f);
            self.log_vis(frame, progress, id, "DISAPPEAR", &detail);
            self.vis_history.remove(&id);
        }

        // ── b) texture / display_state changes ────────────────────────────

        let current_tex_ids: HashSet<TileId> =
            display_entries.iter().map(|(id, _, _, _)| *id).collect();

        for (mesh_id, tex_id, uv, own) in &display_entries {
            let snap = TileDisplaySnapshot::from_entry(*tex_id, *uv, *own);

            // Clone the old snapshot to avoid holding a borrow on display_history
            // while calling log_tex (which borrows self mutably).
            let prev_clone: Option<TileDisplaySnapshot> =
                self.display_history.get(mesh_id).cloned();

            if let Some(prev) = prev_clone {
                if prev != snap {
                    let event = if prev.showing_own && !snap.showing_own {
                        "DOWNGRADE"
                    } else if !prev.showing_own && snap.showing_own {
                        "UPGRADE"
                    } else {
                        "CHANGED"
                    };
                    self.log_tex(frame, progress, *mesh_id, event, Some(&prev), &snap);
                    *self.display_history.get_mut(mesh_id).unwrap() = snap;
                }
            } else {
                self.log_tex(frame, progress, *mesh_id, "APPEAR", None, &snap);
                self.display_history.insert(*mesh_id, snap);
            }
        }

        // tiles removed from display_state
        let removed: Vec<(TileId, TileDisplaySnapshot)> = self
            .display_history
            .iter()
            .filter(|(id, _)| !current_tex_ids.contains(*id))
            .map(|(id, snap)| (*id, snap.clone()))
            .collect();
        for (mesh_id, prev) in removed {
            let dummy = TileDisplaySnapshot {
                texture_id: TileId { z: 0, x: 0, y: 0 },
                uv_key: String::new(),
                showing_own: false,
            };
            self.log_tex(frame, progress, mesh_id, "REMOVED", Some(&prev), &dummy);
            self.display_history.remove(&mesh_id);
        }
    }

    // ── final analysis ────────────────────────────────────────────────────
    fn run_analysis(&self) {
        const RAPID_FRAMES: u32 = 10;

        println!("\n╔══════════════════════════════════════════════════════════╗");
        println!("║         TILE MONITOR — ANALYSIS REPORT                 ║");
        println!("╚══════════════════════════════════════════════════════════╝\n");

        // ── A) Visibility rapid changes ───────────────────────────────────
        println!("── A) Visibility / LOD rapid changes (≤{} frames between events) ──", RAPID_FRAMES);

        let mut rapid_vis: Vec<(TileId, Vec<(u32, u32, String, String)>)> = Vec::new();
        for (id, bucket) in &self.vis_buckets {
            let evs = &bucket.events;
            let pairs: Vec<_> = (1..evs.len())
                .filter_map(|i| {
                    let gap = evs[i].0.saturating_sub(evs[i - 1].0);
                    if gap <= RAPID_FRAMES {
                        Some((evs[i - 1].0, evs[i].0,
                              evs[i - 1].1.clone(), evs[i].1.clone()))
                    } else { None }
                })
                .collect();
            if !pairs.is_empty() { rapid_vis.push((*id, pairs)); }
        }

        if rapid_vis.is_empty() {
            println!("  ✓ No rapid visibility/LOD changes detected.\n");
        } else {
            rapid_vis.sort_by_key(|(id, _)| (id.z, id.x, id.y));
            println!("  {} tile(s) with rapid visibility/LOD changes:\n", rapid_vis.len());
            for (id, pairs) in &rapid_vis {
                println!("  Tile z={} x={} y={} ({} rapid pair(s)):", id.z, id.x, id.y, pairs.len());
                for (f1, f2, k1, k2) in pairs {
                    println!("    frame {} [{}] → frame {} [{}]  gap={}", f1, k1, f2, k2, f2 - f1);
                }
            }
            println!();
        }

        // ── B) Texture rapid changes ──────────────────────────────────────
        println!("── B) Texture (display_state) rapid changes (≤{} frames between events) ──", RAPID_FRAMES);

        let mut rapid_tex: Vec<(TileId, Vec<(u32, u32, String, String)>)> = Vec::new();
        for (id, bucket) in &self.tex_buckets {
            let evs = &bucket.events;
            let pairs: Vec<_> = (1..evs.len())
                .filter_map(|i| {
                    let gap = evs[i].0.saturating_sub(evs[i - 1].0);
                    if gap <= RAPID_FRAMES {
                        Some((evs[i - 1].0, evs[i].0,
                              format!("{} [{}]", evs[i - 1].1, evs[i - 1].2),
                              format!("{} [{}]", evs[i].1,     evs[i].2)))
                    } else { None }
                })
                .collect();
            if !pairs.is_empty() { rapid_tex.push((*id, pairs)); }
        }

        if rapid_tex.is_empty() {
            println!("  ✓ No rapid texture changes detected.\n");
        } else {
            rapid_tex.sort_by_key(|(id, _)| (id.z, id.x, id.y));
            println!("  {} tile(s) with rapid texture changes:\n", rapid_tex.len());
            for (id, pairs) in &rapid_tex {
                println!("  Tile z={} x={} y={} ({} rapid pair(s)):", id.z, id.x, id.y, pairs.len());
                for (f1, f2, d1, d2) in pairs {
                    println!("    frame {} → frame {}  gap={}", f1, f2, f2.saturating_sub(*f1));
                    println!("      before: {}", d1);
                    println!("      after : {}", d2);
                }
            }
            println!();
        }

        // ── C) Root cause hypotheses ──────────────────────────────────────
        println!("── C) Root Cause Hypotheses ─────────────────────────────────");
        let mut hypotheses: Vec<String> = Vec::new();

        // Oscillating APPEAR/DISAPPEAR
        let osc_count = rapid_vis.iter().filter(|(_, pairs)| {
            pairs.iter().any(|(_, _, k1, k2)|
                (k1 == "APPEAR" && k2 == "DISAPPEAR") ||
                (k1 == "DISAPPEAR" && k2 == "APPEAR"))
        }).count();
        if osc_count > 0 {
            hypotheses.push(format!(
                "[VIS-OSC] {} tile(s) oscillate APPEAR↔DISAPPEAR. \
                 Root cause: the camera distance is sitting right on the \
                 quadtree subdivision threshold. The 1.05× hysteresis band \
                 (collapse_dist = subdivide_dist × 1.05) is too narrow; even \
                 tiny per-frame camera movement causes the node to flip between \
                 subdivided (children visible) and not-subdivided (parent leaf \
                 visible). Consider widening the hysteresis band to 1.15–1.25×, \
                 or adding a minimum frame hold-time before collapsing.",
                osc_count
            ));
        }

        // LOD up/down oscillation
        let lod_osc_count = rapid_vis.iter().filter(|(_, pairs)| {
            pairs.iter().any(|(_, _, k1, k2)|
                (k1.starts_with("LOD_UP") && k2.starts_with("LOD_DOWN")) ||
                (k1.starts_with("LOD_DOWN") && k2.starts_with("LOD_UP")))
        }).count();
        if lod_osc_count > 0 {
            hypotheses.push(format!(
                "[LOD-OSC] {} tile(s) rapidly flip between higher and lower \
                 zoom levels. This indicates the quadtree is subdividing and \
                 merging the same node on consecutive frames.",
                lod_osc_count
            ));
        }

        // Texture APPEAR right after REMOVED
        let tex_appear_removed = rapid_tex.iter().filter(|(_, pairs)| {
            pairs.iter().any(|(_, _, d1, d2)|
                (d1.contains("APPEAR") || d2.contains("APPEAR")) ||
                (d1.contains("REMOVED") || d2.contains("REMOVED")))
        }).count();
        if tex_appear_removed > 0 {
            hypotheses.push(format!(
                "[TEX-BLINK] {} tile(s) had APPEAR/REMOVED texture events in \
                 rapid succession. This is the direct cause of tile flickering \
                 (tile goes black/invisible for one or more frames). The tile is \
                 entering then immediately leaving display_state. This is driven \
                 by the quadtree LOD oscillation above — when the tile leaves the \
                 visible set, it is evicted from display_state; when it comes back, \
                 it starts with no texture until a parent fallback is found. \
                 Fix: add a grace period (e.g. keep tiles in display_state for at \
                 least 30 frames after they leave the visible set). This decouples \
                 transient LOD changes from texture assignment changes.",
                tex_appear_removed
            ));
        }

        // Own↔Fallback toggling
        let own_fallback_toggle = rapid_tex.iter().filter(|(_, pairs)| {
            pairs.iter().any(|(_, _, d1, d2)|
                d1.contains("UPGRADE") || d2.contains("UPGRADE") ||
                d1.contains("DOWNGRADE") || d2.contains("DOWNGRADE"))
        }).count();
        if own_fallback_toggle > 0 {
            hypotheses.push(format!(
                "[TEX-TOGGLE] {} tile(s) toggled between own hi-res texture and \
                 parent fallback rapidly (UPGRADE/DOWNGRADE). This can happen if: \
                 (1) the tile's own texture gets evicted from the LRU cache and \
                 re-fetched — increase max_cache_size; or (2) a sibling's readiness \
                 state toggled, causing the sibling-gate in update_display_state to \
                 un-latch. Since showing_own_texture should be a one-way latch, \
                 a DOWNGRADE indicates a bug in update_display_state.",
                own_fallback_toggle
            ));
        }

        if hypotheses.is_empty() {
            println!("  ✓ No flickering patterns identified — tile system looks stable.");
        } else {
            for h in &hypotheses {
                println!("  • {}", h);
                println!();
            }
        }

        // ── Summary ───────────────────────────────────────────────────────
        let total_vis: usize = self.vis_buckets.values().map(|b| b.events.len()).sum();
        let total_tex: usize = self.tex_buckets.values().map(|b| b.events.len()).sum();
        println!("── Summary ──────────────────────────────────────────────────");
        println!("  Frames processed        : {}", self.frame_count);
        println!("  Unique tiles (vis)      : {}", self.vis_buckets.len());
        println!("  Total visibility events : {}", total_vis);
        println!("  Unique tiles (tex)      : {}", self.tex_buckets.len());
        println!("  Total texture events    : {}", total_tex);
        println!("  Tiles w/ rapid vis chg  : {}", rapid_vis.len());
        println!("  Tiles w/ rapid tex chg  : {}", rapid_tex.len());
        println!("\n  CSV files written:");
        println!("    tile_monitor_visibility.csv");
        println!("    tile_monitor_texture.csv");
        println!("────────────────────────────────────────────────────────────\n");
    }
}

// ─── winit ApplicationHandler ────────────────────────────────────────────────

impl<'a> ApplicationHandler for TileMonitorApp<'a> {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        self.inner.resumed(event_loop);
        self.setup_done = true;
    }

    fn window_event(
        &mut self,
        event_loop: &ActiveEventLoop,
        window_id: WindowId,
        event: WindowEvent,
    ) {
        if let WindowEvent::RedrawRequested = event {
            self.inner.window_event(event_loop, window_id, WindowEvent::RedrawRequested);

            self.frame_count += 1;
            let frame    = self.frame_count;
            let progress = *self.progress.lock().unwrap();

            // Skip first 30 frames of startup noise.
            if frame > 30 {
                self.process_frame(frame, progress);
            }

            if frame % 120 == 0 {
                let total_vis: usize = self.vis_buckets.values().map(|b| b.events.len()).sum();
                let total_tex: usize = self.tex_buckets.values().map(|b| b.events.len()).sum();
                println!(
                    "Frame {:5}  progress={:.4}  vis_events={}  tex_events={}",
                    frame, progress, total_vis, total_tex
                );
            }

            if progress >= 0.5 {
                self.run_analysis();
                println!("TileMonitor: progress={:.4} ≥ 0.5 — exiting.", progress);
                event_loop.exit();
            } else {
                if let Some(window) = self.inner.window() {
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
