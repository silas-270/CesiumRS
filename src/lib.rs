pub mod camera;
pub mod core;
pub mod globe;
pub mod io;
pub mod render;

#[cfg(not(target_os = "android"))]
pub mod testing;

pub mod viewer;
pub use viewer::{Viewer, ViewerOptions, GlobeOptions};

use crate::core::app::App;
use winit::event_loop::{ControlFlow, EventLoop};

#[cfg(not(target_os = "android"))]
pub fn run(config: Option<testing::VerifyConfig>) {
    if let Some(cfg) = config {
        let event_loop = EventLoop::new().unwrap();
        event_loop.set_control_flow(ControlFlow::Poll);
        if cfg.stress {
            let mut app = testing::stress_app::StressApp::new(cfg);
            event_loop.run_app(&mut app).unwrap();
        } else {
            let mut app = testing::test_app::TestApp::new(cfg);
            event_loop.run_app(&mut app).unwrap();
        }
    } else {
        let viewer = Viewer::new(ViewerOptions::default());
        viewer.run();
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

    let mut winit_app = App::new(crate::io::config::TileEngineConfig::default());
    event_loop.run_app(&mut winit_app).unwrap();
}
