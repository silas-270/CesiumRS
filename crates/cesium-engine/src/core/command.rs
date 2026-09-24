/// Commands that can be sent from any thread into the engine's main loop.
/// Drained every frame in `App::about_to_wait`.
#[derive(Debug)]
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
    ///
    /// `max_level` is the new style's own imagery depth cap — see
    /// `TileEngineConfig::imagery_max_level` — and travels with the URL so the two
    /// can never land a frame apart (one without the other briefly caps the *new*
    /// style's requests at the *old* style's depth, which is silently wrong in both
    /// directions rather than loudly broken, so this is not a case to leave to two
    /// separate commands processed back to back).
    MapSetImageryUrl { url: String, max_level: u8 },
    /// Switch the imagery source mode (e.g. to offline SVG vector tiles).
    /// Carries the URL (may be empty for SVG mode), the new source mode, and — for
    /// the same reason as `MapSetImageryUrl` — the new style's imagery depth cap.
    MapSetSourceMode {
        url: String,
        mode: crate::globe::tiles::config::TileSourceMode,
        max_level: u8,
    },
    /// Turn terrain height data on or off (`docs/terrain-plan.md` §4 A3). Rebuilds the
    /// height cache and its fetcher, or drops them entirely when turning off.
    ///
    /// Phase B: this controls whether heights are **fetched and cached**. It does not
    /// change a single rendered vertex — that is Phase C.
    TerrainSetEnabled(bool),
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
