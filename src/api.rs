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
//!         flight_handle.load_flight("my_flight", 8.5706, 50.0333, 9.2219, 48.6899, 1_800_000, None, None);
//!         flight_handle.play();
//!         cam.camera_set_position(8.68, 50.11, 0.5); // Frankfurt, Germany
//!     });
//!
//!     viewer.run(); // Blocks. Takes over the main thread.
//! }
//! ```

use cesium_engine::core::app::App;
use cesium_engine::core::command::{CameraCommandMode, ViewerCommand};
use cesium_engine::globe::tiles::config::TileEngineConfig;
use std::num::NonZeroUsize;
use std::sync::mpsc;
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
            max_screen_space_error: 2.0,
            enable_prefetch: true,
            map_saturation: 0.0,
            map_contrast: 0.0,
            map_brightness: 0.0,
            extension: None,
        }
    }
}

impl CesiumViewerBuilder {
    /// Maximum number of tiles held in the GPU cache.
    pub fn tile_cache_size(mut self, size: usize) -> Self {
        self.tile_cache_size = size;
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
            lod_factor: self.max_screen_space_error,
            enable_prefetch: self.enable_prefetch,
            map_saturation: self.map_saturation,
            map_contrast: self.map_contrast,
            map_brightness: self.map_brightness,
            ..TileEngineConfig::default()
        };

        let (tx, rx) = mpsc::sync_channel(128);

        let event_loop = EventLoop::new().unwrap();
        event_loop.set_control_flow(ControlFlow::Poll);

        CesiumViewer {
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
}
