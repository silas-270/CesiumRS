/// Commands that can be sent from any thread into the engine's main loop.
/// Drained every frame in `App::about_to_wait`.
pub enum ViewerCommand {
    // Camera
    CameraSetPosition { lon: f64, lat: f64, alt: f64 },
    CameraSetMode(CameraCommandMode),
    CameraSetAnchor { position: [f64; 3], orientation: [f64; 4] },
    CameraZoom(f32),
    CameraPitch(f32),
    // Map imagery adjustments
    MapSetSaturation(f32),
    MapSetContrast(f32),
    MapSetBrightness(f32),
}

/// Engine-agnostic camera mode enum, mirroring `camera::CameraMode` without exposing it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CameraCommandMode {
    Free,
    Tracking,
    Cockpit,
}
