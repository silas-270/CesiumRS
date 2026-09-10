pub mod api;
#[cfg(not(target_os = "android"))]
pub mod viewer;
pub mod headless;

#[cfg(all(not(target_os = "android"), feature = "testing"))]
pub mod testing;

#[cfg(target_os = "android")]
pub mod android_jni;

// ── Primary public API ────────────────────────────────────────────────────────
pub use api::{CameraMode, CameraState, CesiumViewer, MapStyle, ViewerHandle};

// ── Legacy path (kept for the test harness) ───────────────────────────────────
#[cfg(not(target_os = "android"))]
pub use viewer::{GlobeOptions, Viewer, ViewerOptions};

#[cfg(any(feature = "testing", target_os = "android"))]
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
        } else if cfg.cockpit {
            testing::rendering::cockpit_capture::run(cfg);
        } else if cfg.cockpit_s23 {
            testing::rendering::cockpit_s23::run(cfg);
        } else if cfg.free_routes {
            testing::rendering::free_routes::run(cfg);
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

    // Large models ship in the APK rather than baked into the .so, so point the asset
    // lookup at the AssetManager before anything can ask for one. Must happen before
    // `app` is moved into the event loop.
    let assets = app.asset_manager();
    cesium_flight::assets::set_loader(move |name| {
        let cname = std::ffi::CString::new(name).ok()?;
        let mut asset = assets.open(&cname)?;
        asset.buffer().ok().map(|b| b.to_vec())
    });

    use winit::platform::android::EventLoopBuilderExtAndroid;
    let event_loop: EventLoop<cesium_engine::core::app::EngineEvent> = EventLoop::with_user_event().with_android_app(app).build().unwrap();
    event_loop.set_control_flow(ControlFlow::Poll);

    *android_jni::EVENT_LOOP_PROXY.lock().unwrap() = Some(event_loop.create_proxy());

    let (flight_app, flight_handle) = cesium_flight::tracker::FlightTrackerApp::with_handle();
    let current_telemetry = flight_app.current_telemetry.clone();
    let current_camera_state = flight_app.current_camera_state.clone();
    let pending_camera_restore = flight_app.pending_camera_restore.clone();

    // Flight data is now loaded on-demand via nativeLoadPendingFlight() JNI call.
    // The engine starts idle and waits for the user to book a flight.

    let viewer = crate::api::CesiumViewer::builder()
        .with_extension(Box::new(flight_app))
        .build();

    *android_jni::VIEWER_HANDLE.lock().unwrap() = Some(viewer.handle());
    *android_jni::FLIGHT_HANDLE.lock().unwrap() = Some(flight_handle.clone());
    *android_jni::CURRENT_TELEMETRY.lock().unwrap() = Some(current_telemetry);
    *android_jni::CURRENT_CAMERA_STATE.lock().unwrap() = Some(current_camera_state);
    *android_jni::PENDING_CAMERA_RESTORE.lock().unwrap() = Some(pending_camera_restore);

    // If a flight was pending before we initialized (e.g. fast user click or process recreation), load it!
    if let Some(data) = android_jni::FLIGHT_DATA.lock().unwrap().take() {
        let runways = android_jni::RUNWAY_DATA.lock().unwrap().take().unwrap_or_default();
        flight_handle.load_flight(
            "primary",
            data.dep_lon,
            data.dep_lat,
            data.arr_lon,
            data.arr_lat,
            data.duration_ms,
            None,
            None,
            runways,
        );
    }

    // The core app loop wrapper
    let mut winit_app = cesium_engine::core::app::App::new(
        cesium_engine::globe::tiles::config::TileEngineConfig::default(),
        viewer.extension,
        Some(viewer.command_rx),
    );
    event_loop.run_app(&mut winit_app).unwrap();
}
