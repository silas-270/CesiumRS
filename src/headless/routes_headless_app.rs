use cesium_engine::render::wgpu_state::WgpuState;
use cesium_engine::globe::tiles::config::TileEngineConfig;
use std::sync::Arc;

pub async fn run_headless_render(
    width: u32,
    height: u32,
    config: TileEngineConfig,
    mut extension: Option<Box<dyn cesium_engine::core::extension::GlobeExtension>>,
    initial_cam_pos: glam::Vec3,
    initial_cam_target: glam::Vec3,
    out_path: &str,
) {
    let mut state = WgpuState::new(
        None,
        Some(winit::dpi::PhysicalSize::new(width, height)),
        config,
        extension,
    ).await;

    // Set up camera
    state.camera.set_eye(initial_cam_pos, initial_cam_target);

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
    while state.last_missing_tiles_count > 0 {
        std::thread::sleep(std::time::Duration::from_millis(50));
        let visible_tiles = state.update_logic(aspect_ratio, main_view_proj);
    }

    // Render EXACTLY one frame and capture
    #[cfg(feature = "debug_panel")]
    let render_res = state.render(Some(out_path), false, |_, _| {});
    #[cfg(not(feature = "debug_panel"))]
    let render_res = state.render(Some(out_path), false);

    render_res.expect("Failed to render headless frame");
}
