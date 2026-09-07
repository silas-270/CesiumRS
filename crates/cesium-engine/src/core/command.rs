/// Commands that can be sent from any thread into the engine's main loop.
/// Drained every frame in `App::about_to_wait`.
pub enum ViewerCommand {
    // Camera
    CameraSetPosition {
        lon: f64,
        lat: f64,
        alt: f64,
    },
    CameraSetMode(CameraCommandMode),
    CameraSetAnchor {
        position: [f64; 3],
        orientation: [f64; 4],
    },
    CameraZoom(f32),
    CameraPitch(f32),
    // Map imagery adjustments
    MapSetSaturation(f32),
    MapSetContrast(f32),
    MapSetBrightness(f32),
    /// Switch the base tile layer to a new XYZ imagery URL (`{z}`/`{x}`/`{y}`
    /// placeholders). Reconstructs the tile texture cache, so already-loaded
    /// tiles briefly fall back to the base color while the new imagery loads.
    MapSetImageryUrl(String),
    /// Emits an ATrace instant marker ("cesium.scenario.<id>") on the engine
    /// thread, so a captured Perfetto trace can be auto-sliced by scenario.
    /// Processed only when built with `--features perf_trace`.
    #[cfg(feature = "perf_trace")]
    PerfScenarioMarker(i32),
}

/// Engine-agnostic camera mode enum, mirroring `camera::CameraMode` without exposing it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CameraCommandMode {
    Free,
    Tracking,
    Cockpit,
}
