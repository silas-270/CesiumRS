//! Screenshot and high-resolution headless capture module.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Mutex;
use glam::{DQuat, DVec3, Quat, Vec3};
use crate::camera::camera::CameraMode;
use crate::core::extension::GlobeExtension;
use crate::globe::tiles::config::TileEngineConfig;

pub static SCREENSHOT_IN_PROGRESS: AtomicBool = AtomicBool::new(false);
pub static SCREENSHOT_STATUS: Mutex<Option<(String, bool)>> = Mutex::new(None);

#[derive(Clone, Debug)]
pub struct ScreenshotCameraState {
    pub anchor_pos: DVec3,
    pub anchor_ori: DQuat,
    pub local_pos: Vec3,
    pub local_ori: Quat,
    pub focal_length: f32,
    pub sun_intensity: f32,
    pub mode: CameraMode,
}

#[derive(Clone, Debug)]
pub struct LabelManagerStateSnapshot {
    pub enabled: bool,
    pub size_scale: f32,
    pub max_importance_rank: u8,
    pub show_anchor_dots: bool,
}

/// Generates a timestamped filename in the `screenshots` directory.
pub fn generate_screenshot_path() -> String {
    let _ = std::fs::create_dir_all("screenshots");
    let now = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_secs() as libc::time_t;
    let mut tm = std::mem::MaybeUninit::<libc::tm>::uninit();
    let filename = unsafe {
        libc::localtime_r(&now, tm.as_mut_ptr());
        let tm = tm.assume_init();
        format!(
            "screenshot_{:04}{:02}{:02}_{:02}{:02}{:02}.png",
            tm.tm_year + 1900,
            tm.tm_mon + 1,
            tm.tm_mday,
            tm.tm_hour,
            tm.tm_min,
            tm.tm_sec
        )
    };
    format!("screenshots/{}", filename)
}

/// Spawns a background thread to perform a 4K headless render of the exact current view.
pub fn trigger_4k_screenshot(
    base_config: TileEngineConfig,
    camera_state: ScreenshotCameraState,
    extension: Option<Box<dyn GlobeExtension>>,
    label_snapshot: Option<LabelManagerStateSnapshot>,
) {
    if SCREENSHOT_IN_PROGRESS
        .compare_exchange(false, true, Ordering::SeqCst, Ordering::SeqCst)
        .is_err()
    {
        return;
    }

    let out_path = generate_screenshot_path();
    {
        let mut status = SCREENSHOT_STATUS.lock().unwrap();
        *status = Some((format!("Capturing 4K render to {}...", out_path), false));
    }

    std::thread::spawn(move || {
        let path_clone = out_path.clone();
        let result = pollster::block_on(async move {
            render_4k_screenshot_headless(
                base_config,
                camera_state,
                extension,
                label_snapshot,
                &path_clone,
            )
            .await
        });

        match result {
            Ok(()) => {
                log::info!("Successfully saved 4K screenshot to {}", out_path);
                let mut status = SCREENSHOT_STATUS.lock().unwrap();
                *status = Some((format!("Saved: {}", out_path), false));
            }
            Err(e) => {
                log::error!("Failed to capture 4K screenshot: {}", e);
                let mut status = SCREENSHOT_STATUS.lock().unwrap();
                *status = Some((format!("Capture failed: {}", e), true));
            }
        }

        SCREENSHOT_IN_PROGRESS.store(false, Ordering::SeqCst);
    });
}

pub async fn render_4k_screenshot_headless(
    mut base_config: TileEngineConfig,
    camera_state: ScreenshotCameraState,
    extension: Option<Box<dyn GlobeExtension>>,
    label_snapshot: Option<LabelManagerStateSnapshot>,
    out_path: &str,
) -> Result<(), String> {
    const WIDTH: u32 = 3840;
    const HEIGHT: u32 = 2160;

    // High quality settings for 4K
    base_config.target_texel_ratio = 0.5;
    base_config.mesh_segments = 32;
    base_config.max_cache_size = std::num::NonZeroUsize::new(4096).unwrap();
    base_config.mesh_cache_size = std::num::NonZeroUsize::new(4096).unwrap();
    base_config.tile_cache_budget_bytes = 512 * 1024 * 1024;
    base_config.terrain.height_cache_budget_bytes = 128 * 1024 * 1024;

    let mut state = crate::render::wgpu_state::WgpuState::new(
        None,
        Some(winit::dpi::PhysicalSize::new(WIDTH, HEIGHT)),
        base_config,
        extension,
    )
    .await;

    // Restore camera
    state.camera.anchor_pos = camera_state.anchor_pos;
    state.camera.anchor_ori = camera_state.anchor_ori;
    state.camera.local_pos = camera_state.local_pos;
    state.camera.local_ori = camera_state.local_ori;
    state.camera.focal_length = camera_state.focal_length;
    state.camera.sun_intensity = camera_state.sun_intensity;
    state.camera.mode = camera_state.mode;

    if let Some(labels) = label_snapshot {
        state.label_manager.enabled = labels.enabled;
        state.label_manager.size_scale = labels.size_scale;
        state.label_manager.max_importance_rank = labels.max_importance_rank;
        state.label_manager.show_anchor_dots = labels.show_anchor_dots;
    }

    let aspect = WIDTH as f32 / HEIGHT as f32;
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(45);
    let mut quiet = 0;

    loop {
        let view_proj =
            state.camera.get_projection_matrix(aspect) * state.camera.get_view_matrix();
        let visible_tiles = state.update_logic(aspect, view_proj);
        state
            .tile_system
            .texture_manager
            .fetch_and_upload_all(&state.device, &state.queue, &visible_tiles)
            .await;

        let settled =
            state.last_missing_tiles_count == 0 && state.tile_system.is_loading_complete();
        quiet = if settled { quiet + 1 } else { 0 };
        if quiet >= 6 || std::time::Instant::now() >= deadline {
            break;
        }
        std::thread::sleep(std::time::Duration::from_millis(50));
    }

    #[cfg(feature = "debug_panel")]
    let res = state.render(Some(out_path), false, |_, _| {});
    #[cfg(not(feature = "debug_panel"))]
    let res = state.render(Some(out_path), false);

    res.map_err(|e| format!("Render error: {:?}", e))?;
    Ok(())
}
