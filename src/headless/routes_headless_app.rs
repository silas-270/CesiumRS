use cesium_engine::render::wgpu_state::WgpuState;
use cesium_engine::globe::tiles::config::TileEngineConfig;

/// How long a render waits for the last meshes before capturing what it has. Headless renders
/// rasterise the bundled vector map locally, so this is normally reached in well under a
/// second; the cap exists so a tile that never completes costs a slightly coarser image
/// instead of blocking the caller's thread forever.
const MESH_WAIT_LIMIT: std::time::Duration = std::time::Duration::from_secs(20);

/// Renders one frame to `out_path`. Returns whether the PNG was written.
pub async fn run_headless_render(
    width: u32,
    height: u32,
    config: TileEngineConfig,
    extension: Option<Box<dyn cesium_engine::core::extension::GlobeExtension>>,
    initial_cam_pos: glam::Vec3,
    initial_cam_target: glam::Vec3,
    initial_cam_up: Option<glam::Vec3>,
    out_path: &str,
) -> bool {
    let mut state = WgpuState::new(
        None,
        Some(winit::dpi::PhysicalSize::new(width, height)),
        config,
        extension,
    ).await;

    // The Hub, Onboarding and Account route maps are clean images of the routes; city
    // labels, which the scene pass now draws, have never been part of them.
    state.label_manager.enabled = false;

    // Set up camera
    if let Some(up) = initial_cam_up {
        state.camera.set_eye_with_up(initial_cam_pos, initial_cam_target, up);
    } else {
        state.camera.set_eye(initial_cam_pos, initial_cam_target);
    }

    // Calculate visible tiles for this camera
    let aspect_ratio = state.size.width as f32 / state.size.height as f32;
    let main_view_proj = state.camera.get_projection_matrix(aspect_ratio) * state.camera.get_view_matrix();
    let visible_tiles = state.update_logic(aspect_ratio, main_view_proj);

    // Wait asynchronously for all tiles to fetch
    state.tile_system.texture_manager.fetch_and_upload_all(
        &state.device,
        &state.queue,
        &visible_tiles,
    ).await;

    // We also need to wait for the mesh worker to finish generating the meshes
    let wait_started = std::time::Instant::now();
    while state.last_missing_tiles_count > 0 {
        if wait_started.elapsed() >= MESH_WAIT_LIMIT {
            log::warn!(
                "Headless render: {} tiles still missing after {:?}; capturing anyway",
                state.last_missing_tiles_count,
                MESH_WAIT_LIMIT
            );
            break;
        }
        std::thread::sleep(std::time::Duration::from_millis(50));
        let _ = state.update_logic(aspect_ratio, main_view_proj);
    }

    // Render EXACTLY one frame and capture
    #[cfg(feature = "debug_panel")]
    let render_res = state.render(Some(out_path), false, |_, _| {});
    #[cfg(not(feature = "debug_panel"))]
    let render_res = state.render(Some(out_path), false);

    // These renders are reached through `extern "C"` functions, where a panic cannot unwind
    // into the caller, so a failure is reported as `false` instead.
    match render_res {
        Ok(_) => true,
        Err(e) => {
            log::error!("Headless render failed: {e:?}");
            false
        }
    }
}
