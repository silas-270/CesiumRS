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
            self.setup_done = true;
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

        let state = self.wgpu_state.as_mut().unwrap();

        match event {
            WindowEvent::CloseRequested => {
                event_loop.exit();
            }
            WindowEvent::RedrawRequested => {
                self.total_frames += 1;
                
                if self.total_frames > 60 && state.tile_system.texture_manager.fetcher.is_loading_complete() {
                    self.frames_stable += 1;
                } else {
                    self.frames_stable = 0;
                }

                let mut capture_path = None;
                if self.frames_stable > 10 {
                    capture_path = Some(self.out_path.as_str());
                }

                match state.render(capture_path, false, |_, _| {}) {
                    Ok(_) => {
                        if capture_path.is_some() {
                            event_loop.exit();
                        } else {
                            window.request_redraw();
                        }
                    }
                    Err(wgpu::SurfaceError::Lost) => state.resize(state.size),
                    Err(wgpu::SurfaceError::OutOfMemory) => event_loop.exit(),
                    Err(e) => log::error!("Render error: {:?}", e),
                }
            }
            _ => {}
        }
    }

    fn about_to_wait(&mut self, _event_loop: &ActiveEventLoop) {
        if let Some(window) = &self.window {
            window.request_redraw();
        }
    }
}
