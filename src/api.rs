//! # CesiumRS Unified API
//!
//! This module is the single public entry point for the engine.
//! Consumers only need to import from this module — no internal engine types leak out.
//!
//! ## Quickstart
//!
//! ```rust,no_run
//! use cesium_rs::{CesiumViewer, CameraMode};
//!
//! fn main() {
//!     let (flight_app, flight_handle) = cesium_flight::tracker::FlightTrackerApp::with_handle();
//!
//!     let viewer = CesiumViewer::builder()
//!         .tile_cache_size(2048)
//!         .max_screen_space_error(2.0)
//!         .enable_prefetch(true)
//!         .with_extension(Box::new(flight_app))
//!         .build();
//!
//!     // The handle is Send + Sync — store it for use from JNI or other threads.
//!     let cam = viewer.handle();
//!
//!     std::thread::spawn(move || {
//!         // For example, from Frankfurt (FRA) to Stuttgart (STR)
//!         flight_handle.load_flight("my_flight", 8.5706, 50.0333, 9.2219, 48.6899, 1_800_000, None, None, Vec::new());
//!         flight_handle.play();
//!         cam.camera_set_position(8.68, 50.11, 0.5); // Frankfurt, Germany
//!     });
//!
//!     viewer.run(); // Blocks. Takes over the main thread.
//! }
//! ```

#[cfg(not(target_os = "android"))]
use cesium_engine::core::app::App;
use cesium_engine::core::command::{CameraCommandMode, ViewerCommand};
use cesium_engine::globe::tiles::config::{
    TileEngineConfig, SATELLITE_IMAGERY_URL, STANDARD_IMAGERY_URL,
};
use std::num::NonZeroUsize;
use std::sync::mpsc;
#[cfg(not(target_os = "android"))]
use winit::event_loop::{ControlFlow, EventLoop};

// ─── Public re-exports ────────────────────────────────────────────────────────

/// Engine-agnostic camera mode. No wgpu or glam types leak through this.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CameraMode {
    /// Standard globe-orbiting mode.
    Free,
    /// Camera orbits a tracked entity (e.g. an airplane).
    Tracking,
    /// First-person view locked inside the tracked entity.
    Cockpit,
}

/// Base imagery style for the globe's tile layer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MapStyle {
    /// Default dark, label-free vector-style basemap.
    Standard,
    /// Satellite aerial imagery.
    Satellite,
}

/// A snapshot of the camera's state at the time of the query.
#[derive(Debug, Clone)]
pub struct CameraState {
    /// Current altitude above the WGS84 ellipsoid in kilometres.
    pub altitude_km: f32,
    /// Current camera mode.
    pub mode: CameraMode,
}

// ─── Builder ─────────────────────────────────────────────────────────────────

/// Builder for the `CesiumViewer`. Obtain one via `CesiumViewer::builder()`.
pub struct CesiumViewerBuilder {
    tile_cache_size: usize,
    tile_cache_budget_bytes: usize,
    max_screen_space_error: f32,
    enable_prefetch: bool,
    map_saturation: f32,
    map_contrast: f32,
    map_brightness: f32,
    extension: Option<Box<dyn cesium_engine::core::extension::GlobeExtension>>,
}

impl Default for CesiumViewerBuilder {
    fn default() -> Self {
        Self {
            tile_cache_size: 2048,
            tile_cache_budget_bytes: TileEngineConfig::default().tile_cache_budget_bytes,
            max_screen_space_error: 2.0,
            enable_prefetch: true,
            map_saturation: 0.0,
            map_contrast: 0.0,
            map_brightness: 0.5,
            extension: None,
        }
    }
}

impl CesiumViewerBuilder {
    /// Upper bound on the number of tiles held in the GPU cache. This is a
    /// count, not a size — what actually bounds memory is
    /// [`tile_cache_budget_bytes`](Self::tile_cache_budget_bytes), and the
    /// smaller of the two wins.
    pub fn tile_cache_size(mut self, size: usize) -> Self {
        self.tile_cache_size = size;
        self
    }

    /// Memory budget for decoded imagery textures, in bytes (default 512MB).
    /// The entry count is derived from this once the imagery style's real tile
    /// size is known, so the ceiling holds whether a style serves 256x256 or
    /// 512x512 tiles.
    pub fn tile_cache_budget_bytes(mut self, bytes: usize) -> Self {
        self.tile_cache_budget_bytes = bytes;
        self
    }

    /// Higher values trade visual fidelity for performance.
    /// Default is `2.0`.
    pub fn max_screen_space_error(mut self, sse: f32) -> Self {
        self.max_screen_space_error = sse;
        self
    }

    /// Whether the engine should speculatively fetch tiles ahead of the camera.
    pub fn enable_prefetch(mut self, enable: bool) -> Self {
        self.enable_prefetch = enable;
        self
    }

    /// Map imagery saturation adjustment. `-1.0` = greyscale, `0.0` = neutral, `1.0` = oversaturated.
    pub fn map_saturation(mut self, value: f32) -> Self {
        self.map_saturation = value;
        self
    }

    /// Map imagery contrast adjustment. `-1.0` = flat, `0.0` = neutral, `1.0` = high contrast.
    pub fn map_contrast(mut self, value: f32) -> Self {
        self.map_contrast = value;
        self
    }

    /// Map imagery brightness adjustment. `-1.0` = black, `0.0` = neutral, `1.0` = white.
    pub fn map_brightness(mut self, value: f32) -> Self {
        self.map_brightness = value;
        self
    }

    /// Attach a `GlobeExtension` plugin (e.g. `FlightTrackerApp`).
    pub fn with_extension(
        mut self,
        extension: Box<dyn cesium_engine::core::extension::GlobeExtension>,
    ) -> Self {
        self.extension = Some(extension);
        self
    }

    /// Consume the builder and produce a `CesiumViewer`.
    pub fn build(self) -> CesiumViewer {
        let config = TileEngineConfig {
            max_cache_size: NonZeroUsize::new(self.tile_cache_size)
                .unwrap_or(NonZeroUsize::new(1).unwrap()),
            mesh_cache_size: NonZeroUsize::new(self.tile_cache_size / 4)
                .unwrap_or(NonZeroUsize::new(1).unwrap()),
            tile_cache_budget_bytes: self.tile_cache_budget_bytes,
            lod_factor: self.max_screen_space_error,
            enable_prefetch: self.enable_prefetch,
            map_saturation: self.map_saturation,
            map_contrast: self.map_contrast,
            map_brightness: self.map_brightness,
            ..TileEngineConfig::default()
        };

        let (tx, rx) = mpsc::sync_channel(128);

        #[cfg(not(target_os = "android"))]
        let event_loop = {
            let el = EventLoop::new().unwrap();
            el.set_control_flow(ControlFlow::Poll);
            el
        };

        CesiumViewer {
            #[cfg(not(target_os = "android"))]
            event_loop,
            config,
            extension: self.extension,
            command_tx: tx,
            command_rx: rx,
        }
    }
}

// ─── Viewer ──────────────────────────────────────────────────────────────────

/// The main viewer. Constructed via `CesiumViewer::builder()`.
///
/// Call `handle()` to obtain a `ViewerHandle` *before* calling `run()`,
/// since `run()` takes `self` and blocks the calling thread.
pub struct CesiumViewer {
    #[cfg(not(target_os = "android"))]
    pub(crate) event_loop: EventLoop<()>,
    pub(crate) config: TileEngineConfig,
    pub(crate) extension: Option<Box<dyn cesium_engine::core::extension::GlobeExtension>>,
    pub(crate) command_tx: mpsc::SyncSender<ViewerCommand>,
    pub(crate) command_rx: mpsc::Receiver<ViewerCommand>,
}

impl CesiumViewer {
    /// Entry point for the builder API.
    pub fn builder() -> CesiumViewerBuilder {
        CesiumViewerBuilder::default()
    }

    /// Returns a cloneable, `Send`-safe handle that can be used from any thread
    /// to control the camera and map settings at runtime.
    ///
    /// Must be called **before** `run()`.
    pub fn handle(&self) -> ViewerHandle {
        ViewerHandle {
            tx: self.command_tx.clone(),
        }
    }

    /// Start the application event loop. **Blocks the calling thread and never returns.**
    #[cfg(not(target_os = "android"))]
    pub fn run(self) {
        let mut app = App::new(self.config, self.extension, Some(self.command_rx));
        self.event_loop.run_app(&mut app).unwrap();
    }
}

// ─── Runtime Handle ───────────────────────────────────────────────────────────

/// A cloneable, `Send + Sync` handle for controlling the engine from any thread.
///
/// All methods are **non-blocking** — they enqueue a command that is applied at
/// the beginning of the next frame.
#[derive(Clone)]
pub struct ViewerHandle {
    pub(crate) tx: mpsc::SyncSender<ViewerCommand>,
}

impl ViewerHandle {
    // ── Camera ──────────────────────────────────────────────────────────────

    /// Move the camera to the given geographic position.
    ///
    /// - `lon`: longitude in decimal degrees (−180 … +180)
    /// - `lat`: latitude in decimal degrees (−90 … +90)
    /// - `alt`: altitude in kilometres above the WGS84 ellipsoid
    pub fn camera_set_position(&self, lon: f64, lat: f64, alt: f64) {
        let _ = self
            .tx
            .try_send(ViewerCommand::CameraSetPosition { lon, lat, alt });
    }

    /// Switch the camera to a different tracking mode.
    pub fn camera_set_mode(&self, mode: CameraMode) {
        let engine_mode = match mode {
            CameraMode::Free => CameraCommandMode::Free,
            CameraMode::Tracking => CameraCommandMode::Tracking,
            CameraMode::Cockpit => CameraCommandMode::Cockpit,
        };
        let _ = self.tx.try_send(ViewerCommand::CameraSetMode(engine_mode));
    }

    /// Programmatically set the camera's anchor transform.
    ///
    /// - `position`: ECEF position in kilometres `[x, y, z]`
    /// - `orientation`: unit quaternion `[x, y, z, w]`
    pub fn camera_set_anchor(&self, position: [f64; 3], orientation: [f64; 4]) {
        let _ = self.tx.try_send(ViewerCommand::CameraSetAnchor {
            position,
            orientation,
        });
    }

    /// Zoom in (`delta > 0`) or out (`delta < 0`). Scales distance by ~15% per unit.
    pub fn camera_zoom(&self, delta: f32) {
        let _ = self.tx.try_send(ViewerCommand::CameraZoom(delta));
    }

    /// Pitch the camera up (`delta > 0`) or down (`delta < 0`).
    pub fn camera_pitch(&self, delta: f32) {
        let _ = self.tx.try_send(ViewerCommand::CameraPitch(delta));
    }

    // ── Map imagery ─────────────────────────────────────────────────────────

    /// Adjust map saturation live. `-1.0` = greyscale, `0.0` = neutral, `1.0` = oversaturated.
    pub fn map_set_saturation(&self, value: f32) {
        let _ = self.tx.try_send(ViewerCommand::MapSetSaturation(value));
    }

    /// Adjust map contrast live. `-1.0` = flat, `0.0` = neutral, `1.0` = high contrast.
    pub fn map_set_contrast(&self, value: f32) {
        let _ = self.tx.try_send(ViewerCommand::MapSetContrast(value));
    }

    /// Adjust map brightness live. `-1.0` = black, `0.0` = neutral, `1.0` = white.
    pub fn map_set_brightness(&self, value: f32) {
        let _ = self.tx.try_send(ViewerCommand::MapSetBrightness(value));
    }

    /// Switch the base map imagery live (e.g. standard vs. satellite). The tile
    /// texture cache is rebuilt, so already-loaded tiles briefly show the
    /// fallback color while the new imagery re-fetches.
    pub fn map_set_style(&self, style: MapStyle) {
        let url = match style {
            MapStyle::Standard => STANDARD_IMAGERY_URL,
            MapStyle::Satellite => SATELLITE_IMAGERY_URL,
        };
        let _ = self
            .tx
            .try_send(ViewerCommand::MapSetImageryUrl(url.to_string()));
    }

    // ── Performance testing (debug-only; see tools/run_perf_scenario.sh) ──────

    /// Tags the current point in a captured Perfetto trace with a scenario id
    /// (via an ATrace instant marker) and, for the three steady-state
    /// scenarios, switches the camera to the matching mode: `2`=Free,
    /// `3`=Tracking, `4`=Cockpit. Other ids only emit the marker — sequencing
    /// (e.g. rapid mode switching) is driven externally by calling this
    /// repeatedly with a scripted delay between calls. Only built into the
    /// engine with `--features perf_trace`; a no-op without it.
    #[cfg(feature = "perf_trace")]
    pub fn run_perf_scenario(&self, scenario_id: i32) {
        let mode = match scenario_id {
            2 => Some(CameraCommandMode::Free),
            3 => Some(CameraCommandMode::Tracking),
            4 => Some(CameraCommandMode::Cockpit),
            _ => None,
        };
        if let Some(mode) = mode {
            let _ = self.tx.try_send(ViewerCommand::CameraSetMode(mode));
        }
        let _ = self
            .tx
            .try_send(ViewerCommand::PerfScenarioMarker(scenario_id));
    }
}
