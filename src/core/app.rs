use std::sync::Arc;
use winit::application::ApplicationHandler;
use winit::event::{ElementState, KeyEvent, MouseButton, MouseScrollDelta, WindowEvent};
use winit::event_loop::ActiveEventLoop;
use winit::keyboard::{KeyCode, PhysicalKey};
use winit::window::{Window, WindowId};

use crate::render::wgpu_state::WgpuState;

#[derive(Default)]
pub struct App<'a> {
    window: Option<Arc<Window>>,
    wgpu_state: Option<WgpuState<'a>>,
    mouse_pressed: bool,
    last_mouse_pos: Option<(f64, f64)>,
}

impl<'a> App<'a> {
    pub fn wgpu_state_mut(&mut self) -> Option<&mut WgpuState<'a>> {
        self.wgpu_state.as_mut()
    }
    
    pub fn window(&self) -> Option<&Arc<Window>> {
        self.window.as_ref()
    }
}

impl<'a> ApplicationHandler for App<'a> {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        if self.window.is_none() {
            let window_attributes =
                Window::default_attributes().with_title("CesiumRS WGS84 Ellipsoid");

            let window = Arc::new(event_loop.create_window(window_attributes).unwrap());
            self.window = Some(window.clone());

            let state = pollster::block_on(WgpuState::new(window));
            self.wgpu_state = Some(state);
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

        let response = state.egui_state.on_window_event(window, &event);
        if response.consumed {
            return;
        }

        match event {
            WindowEvent::CloseRequested => {
                event_loop.exit();
            }
            WindowEvent::Resized(physical_size) => {
                state.resize(physical_size);
            }
            WindowEvent::RedrawRequested => match state.render(None) {
                Ok(_) => {}
                Err(wgpu::SurfaceError::Lost) => state.resize(state.size),
                Err(wgpu::SurfaceError::OutOfMemory) => event_loop.exit(),
                Err(e) => log::error!("{:?}", e),
            },
            WindowEvent::MouseInput {
                state: element_state,
                button: MouseButton::Left,
                ..
            } => {
                self.mouse_pressed = element_state == ElementState::Pressed;
                if self.mouse_pressed {
                    if let Some((x, y)) = self.last_mouse_pos {
                        let cam = if state.debug_mode {
                            &mut state.debug_camera
                        } else {
                            &mut state.camera
                        };
                        cam.begin_drag(
                            x as f32,
                            y as f32,
                            state.size.width as f32,
                            state.size.height as f32,
                        );
                    }
                } else {
                    let cam = if state.debug_mode {
                        &mut state.debug_camera
                    } else {
                        &mut state.camera
                    };
                    cam.end_drag();
                }
            }
            WindowEvent::CursorMoved { position, .. } => {
                if self.mouse_pressed {
                    let cam = if state.debug_mode {
                        &mut state.debug_camera
                    } else {
                        &mut state.camera
                    };
                    cam.drag(
                        position.x as f32,
                        position.y as f32,
                        state.size.width as f32,
                        state.size.height as f32,
                    );
                    window.request_redraw();
                }
                self.last_mouse_pos = Some((position.x, position.y));
            }
            WindowEvent::MouseWheel { delta, .. } => {
                let zoom_delta = match delta {
                    MouseScrollDelta::LineDelta(_, y) => y,
                    MouseScrollDelta::PixelDelta(pos) => (pos.y / 50.0) as f32,
                };
                let cam = if state.debug_mode {
                    &mut state.debug_camera
                } else {
                    &mut state.camera
                };
                cam.zoom(zoom_delta);
                window.request_redraw();
            }
            WindowEvent::KeyboardInput {
                event:
                    KeyEvent {
                        physical_key: PhysicalKey::Code(keycode),
                        state: ElementState::Pressed,
                        ..
                    },
                ..
            } => match keycode {
                KeyCode::ArrowUp | KeyCode::KeyW => {
                    let cam = if state.debug_mode {
                        &mut state.debug_camera
                    } else {
                        &mut state.camera
                    };
                    cam.pitch(1.0);
                    window.request_redraw();
                }
                KeyCode::ArrowDown | KeyCode::KeyS => {
                    let cam = if state.debug_mode {
                        &mut state.debug_camera
                    } else {
                        &mut state.camera
                    };
                    cam.pitch(-1.0);
                    window.request_redraw();
                }
                _ => {}
            },
            _ => {}
        }
    }

    fn about_to_wait(&mut self, _event_loop: &ActiveEventLoop) {
        if let Some(window) = &self.window {
            window.request_redraw();
        }
    }
}
