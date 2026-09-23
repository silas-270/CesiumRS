//! Verification test for the 4K headless screenshot capture system.

use cesium_engine::camera::camera::CameraMode;
use cesium_engine::core::screenshot::{
    generate_screenshot_path, render_4k_screenshot_headless, LabelManagerStateSnapshot,
    ScreenshotCameraState,
};
use cesium_engine::globe::tiles::config::TileEngineConfig;
use std::path::Path;

#[test]
fn test_4k_screenshot_render() {
    let out_path = generate_screenshot_path();
    assert!(
        out_path.starts_with("screenshots/screenshot_") && out_path.ends_with(".png"),
        "Screenshot path format unexpected: {}",
        out_path
    );

    let (flight_app, _handle) = cesium_flight::tracker::FlightTrackerApp::with_handle();

    let camera_state = ScreenshotCameraState {
        anchor_pos: glam::DVec3::ZERO,
        anchor_ori: glam::DQuat::IDENTITY,
        local_pos: glam::Vec3::new(0.0, 0.0, 15.0),
        local_ori: glam::Quat::IDENTITY,
        focal_length: 35.0,
        sun_intensity: 1.0,
        mode: CameraMode::Free,
    };

    let label_snapshot = LabelManagerStateSnapshot {
        enabled: false,
        size_scale: 1.0,
        max_importance_rank: 5,
        show_anchor_dots: false,
    };

    let config = TileEngineConfig {
        offline_mode: false,
        ..TileEngineConfig::default()
    };

    let res = pollster::block_on(render_4k_screenshot_headless(
        config,
        camera_state,
        Some(Box::new(flight_app)),
        Some(label_snapshot),
        &out_path,
    ));

    assert!(res.is_ok(), "Screenshot render failed: {:?}", res);
    assert!(
        Path::new(&out_path).exists(),
        "Rendered 4K screenshot file does not exist at {}",
        out_path
    );

    let img = image::open(&out_path).expect("Failed to open 4K screenshot image");
    assert_eq!(img.width(), 3840, "Image width must be 3840 (4K)");
    assert_eq!(img.height(), 2160, "Image height must be 2160 (4K)");
}
