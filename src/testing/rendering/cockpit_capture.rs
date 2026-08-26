//! Headless capture of the in-flight camera modes.
//!
//! Renders frames from the pilot's seat mid-flight so the cockpit model's placement,
//! scale, near plane and material tints can be inspected without a window, plus the same
//! frame in tracking mode as a regression check on the exterior aircraft.
//!
//! Driven by `cesium_app --cockpit`. The cockpit GLB is read from `assets/`, so run it from
//! the repository root.

use cesium_engine::camera::camera::CameraMode;
use cesium_engine::globe::tiles::config::TileEngineConfig;
use cesium_engine::render::wgpu_state::WgpuState;

use crate::testing::VerifyConfig;

/// Metres to Megametres, the engine's world unit.
const M: f32 = 1.0 / 1_000_000.0;

/// One capture: a camera mode, an optional nudge off the seat, and an output file.
struct Shot {
    mode: CameraMode,
    /// Offset from the seat in metres, in the aircraft frame (+Y up, +Z aft). Applied
    /// after the last update pass so it survives into the render; used to pull the camera
    /// out of the cockpit to inspect the model from outside.
    offset_m: glam::Vec3,
    out_path: &'static str,
}

/// Renders one frame of the FRA→STR flight and writes it to `shot.out_path`.
async fn render_shot(shot: &Shot, progress: f64) {
    let mut flight_app = Box::new(cesium_flight::tracker::FlightTrackerApp::new(
        std::sync::Arc::new(std::sync::Mutex::new(progress)),
    ));
    // Frankfurt to Stuttgart over 30 minutes, matching the desktop viewer's default.
    flight_app.add_flight_path(
        "flight_FRA_STR",
        8.5706,
        50.0333,
        9.2219,
        48.6899,
        1_800_000,
        false,
        Vec::new(),
    );
    // `view_mode != last_view_mode` is what makes the tracker push the mode onto the
    // camera; matching them instead lets the camera's own default win the sync.
    flight_app.view_mode = shot.mode;
    flight_app.last_view_mode = if shot.mode == CameraMode::Free {
        CameraMode::Tracking
    } else {
        CameraMode::Free
    };
    flight_app.reset_viewport = true;

    let mut state = WgpuState::new(
        None,
        Some(winit::dpi::PhysicalSize::new(1280, 720)),
        TileEngineConfig::default(),
        Some(flight_app),
    )
    .await;

    let aspect_ratio = state.size.width as f32 / state.size.height as f32;

    // Several passes: the first materialises the flight and lazily loads the cockpit, the
    // rest settle the camera onto the aircraft and stream the terrain in.
    for _ in 0..10 {
        let view_proj =
            state.camera.get_projection_matrix(aspect_ratio) * state.camera.get_view_matrix();
        let visible = state.update_logic(aspect_ratio, view_proj);
        state
            .tile_system
            .texture_manager
            .fetch_and_upload_all(&state.device, &state.queue, &visible)
            .await;
        std::thread::sleep(std::time::Duration::from_millis(150));
    }

    if shot.offset_m != glam::Vec3::ZERO {
        state.camera.local_pos += shot.offset_m * M;
    }

    println!(
        "[{:?}{}] altitude {:.7} Mm, local_pos {:?}",
        state.camera.mode,
        if shot.offset_m == glam::Vec3::ZERO {
            String::new()
        } else {
            format!(" {:?}m", shot.offset_m)
        },
        state.camera.altitude(),
        state.camera.local_pos
    );

    #[cfg(feature = "debug_panel")]
    let res = state.render(Some(shot.out_path), false, |_, _| {});
    #[cfg(not(feature = "debug_panel"))]
    let res = state.render(Some(shot.out_path), false);
    res.expect("headless render failed");

    println!("wrote {}", shot.out_path);
}

pub fn run(_config: VerifyConfig) {
    let shots = [
        // From the seat: what the user actually sees.
        Shot {
            mode: CameraMode::Cockpit,
            offset_m: glam::Vec3::ZERO,
            out_path: "cockpit_view.png",
        },
        // Pulled aft and up, to check the interior's placement and scale from outside.
        Shot {
            mode: CameraMode::Cockpit,
            offset_m: glam::Vec3::new(0.0, 1.0, 6.0),
            out_path: "cockpit_behind.png",
        },
        // Pulled out to the left, to check the seat's lateral position.
        Shot {
            mode: CameraMode::Cockpit,
            offset_m: glam::Vec3::new(-6.0, 0.5, 1.0),
            out_path: "cockpit_side.png",
        },
        // Regression check on the exterior aircraft after the ModelOptions refactor.
        Shot {
            mode: CameraMode::Tracking,
            offset_m: glam::Vec3::ZERO,
            out_path: "tracking_view.png",
        },
    ];

    for shot in &shots {
        pollster::block_on(render_shot(shot, 0.5));
    }
}
