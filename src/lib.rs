pub mod api;
pub mod viewer;

#[cfg(not(target_os = "android"))]
pub mod testing;

// ── Primary public API ────────────────────────────────────────────────────────
pub use api::{CameraMode, CameraState, CesiumViewer, ViewerHandle};

// ── Legacy path (kept for the test harness) ───────────────────────────────────
pub use viewer::{GlobeOptions, Viewer, ViewerOptions};

use winit::event_loop::{ControlFlow, EventLoop};

#[cfg(not(target_os = "android"))]
pub fn run(config: Option<testing::VerifyConfig>) {
    if let Some(cfg) = config {
        let event_loop = EventLoop::new().unwrap();
        event_loop.set_control_flow(ControlFlow::Poll);
        if cfg.regression {
            let mut app = testing::harness::regression_app::RegressionApp::new(cfg);
            event_loop.run_app(&mut app).unwrap();
        } else if cfg.stress {
            let mut app = testing::harness::stress_app::StressApp::new(cfg);
            event_loop.run_app(&mut app).unwrap();
        } else if cfg.flicker {
            let mut app = testing::rendering::test_flicker_tracking::FlickerTrackingApp::new(cfg);
            event_loop.run_app(&mut app).unwrap();
        } else if cfg.monitor {
            let mut app = testing::rendering::test_tile_monitor::TileMonitorApp::new(cfg);
            event_loop.run_app(&mut app).unwrap();
        } else if cfg.profile {
            let mut app = testing::profiling::perf_simulator::PerfSimulatorApp::new(cfg);
            event_loop.run_app(&mut app).unwrap();
        } else if cfg.benchmark {
            let mut app = testing::benchmark::BenchmarkApp::new(cfg);
            event_loop.run_app(&mut app).unwrap();
        } else {
            let mut app = testing::harness::test_app::TestApp::new(cfg);
            event_loop.run_app(&mut app).unwrap();
        }
    } else {
        let viewer = Viewer::new(ViewerOptions::default());
        viewer.run(None);
    }
}

#[cfg(target_os = "android")]
#[no_mangle]
pub extern "C" fn android_main(app: winit::platform::android::activity::AndroidApp) {
    android_logger::init_once(
        android_logger::Config::default().with_max_level(log::LevelFilter::Info),
    );

    use winit::platform::android::EventLoopBuilderExtAndroid;
    let event_loop = EventLoop::builder().with_android_app(app).build().unwrap();
    event_loop.set_control_flow(ControlFlow::Poll);

    let mut winit_app = cesium_engine::core::app::App::new(
        cesium_engine::globe::tiles::config::TileEngineConfig::default(),
        None,
        None,
    );
    event_loop.run_app(&mut winit_app).unwrap();
}
