use cesium_engine::render::wgpu_state::WgpuState;
use cesium_engine::globe::tiles::config::TileEngineConfig;
use winit::application::ApplicationHandler;
use winit::event::WindowEvent;
use winit::event_loop::ActiveEventLoop;
use winit::window::{Window, WindowId};
use std::sync::Arc;

pub struct RoutesHeadlessApp<'a> {
    pub wgpu_state: Option<WgpuState<'a>>,
    pub window: Option<Arc<Window>>,
    pub config: TileEngineConfig,
    pub frames_stable: u32,
    pub total_frames: u32,
    pub setup_done: bool,
    pub width: u32,
    pub height: u32,
    pub out_path: String,
    pub extension: Option<Box<dyn cesium_engine::core::extension::GlobeExtension>>,
    pub initial_cam_pos: glam::Vec3,
    pub initial_cam_target: glam::Vec3,
}

impl<'a> ApplicationHandler for RoutesHeadlessApp<'a> {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        if self.window.is_none() {
            let window_attributes = Window::default_attributes()
                .with_title("CesiumRS Headless")
                .with_inner_size(winit::dpi::PhysicalSize::new(self.width, self.height))
                .with_visible(false);

            let window = Arc::new(event_loop.create_window(window_attributes).unwrap());
            self.window = Some(window.clone());

            let mut state = pollster::block_on(WgpuState::new(
                window,
                self.config.clone(),
                self.extension.take(),
            ));

            // Force surface configuration since invisible windows don't get Resized events
            let physical_size = winit::dpi::PhysicalSize::new(self.width, self.height);
            state.resize(physical_size);

            // Set up camera
            state.camera.set_eye(self.initial_cam_pos, self.initial_cam_target);
            
            self.wgpu_state = Some(state);
        }

        if !self.setup_done {
            let state = self.wgpu_state.as_mut().unwrap();

            // Calculate visible tiles for this camera
            let aspect_ratio = state.size.width as f32 / state.size.height as f32;
            let main_view_proj = state.camera.get_projection_matrix(aspect_ratio) * state.camera.get_view_matrix();
            let visible_tiles = state.update_logic(aspect_ratio, main_view_proj);

            // Wait asynchronously for all tiles to fetch
            pollster::block_on(state.tile_system.texture_manager.fetch_and_upload_all(
                &state.device,
                &state.queue,
                &visible_tiles,
            ));

            // Render EXACTLY one frame and capture
            state
                .render(Some(self.out_path.as_str()), false, |_, _| {})
                .expect("Failed to render headless frame");

            self.setup_done = true;
            event_loop.exit();
        }
    }

    fn window_event(
        &mut self,
        event_loop: &ActiveEventLoop,
        window_id: WindowId,
        event: WindowEvent,
    ) {
        let window = self.window.as_ref().unwrap();
        if window.id() != window_id {
            return;
        }

        if let WindowEvent::CloseRequested = event {
            event_loop.exit();
        }
    }

    fn about_to_wait(&mut self, _event_loop: &ActiveEventLoop) {
        if let Some(window) = &self.window {
            window.request_redraw();
        }
    }
}
