pub mod api;
pub mod viewer;
#[cfg(feature = "testing")]
pub mod headless;

#[cfg(all(not(target_os = "android"), feature = "testing"))]
pub mod testing;

#[cfg(target_os = "android")]
pub mod android_jni;

// ── Primary public API ────────────────────────────────────────────────────────
pub use api::{CameraMode, CameraState, CesiumViewer, ViewerHandle};

// ── Legacy path (kept for the test harness) ───────────────────────────────────
pub use viewer::{GlobeOptions, Viewer, ViewerOptions};

#[cfg(feature = "testing")]
use winit::event_loop::{ControlFlow, EventLoop};

#[cfg(all(not(target_os = "android"), feature = "testing"))]
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

#[cfg(all(not(target_os = "android"), not(feature = "testing")))]
pub fn run(config: Option<()>) {
    let viewer = Viewer::new(ViewerOptions::default());
    viewer.run(None);
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

    let (flight_app, flight_handle) = cesium_flight::tracker::FlightTrackerApp::with_handle();

    // Read the flight data Kotlin set right before launching this Activity
    if let Some(data) = android_jni::FLIGHT_DATA.lock().unwrap().take() {
        flight_handle.load_flight(
            "primary",
            data.dep_lon,
            data.dep_lat,
            data.arr_lon,
            data.arr_lat,
            data.duration_ms,
            None,
            None,
        );
    }

    let viewer = crate::api::CesiumViewer::builder()
        .with_extension(Box::new(flight_app))
        .build();

    *android_jni::VIEWER_HANDLE.lock().unwrap() = Some(viewer.handle());
    *android_jni::FLIGHT_HANDLE.lock().unwrap() = Some(flight_handle.clone());

    // The core app loop wrapper
    let mut winit_app = cesium_engine::core::app::App::new(
        cesium_engine::globe::tiles::config::TileEngineConfig::default(),
        viewer.extension,
        Some(viewer.command_rx),
    );
    event_loop.run_app(&mut winit_app).unwrap();
}
