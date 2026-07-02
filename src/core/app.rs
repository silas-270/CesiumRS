use std::sync::Arc;
use std::time::Instant;
use std::collections::HashSet;
use glam::Vec3;
use winit::application::ApplicationHandler;
use winit::event::{ElementState, KeyEvent, MouseButton, MouseScrollDelta, WindowEvent};
use winit::event_loop::ActiveEventLoop;
use winit::keyboard::{KeyCode, PhysicalKey};
use winit::window::{Window, WindowId};

use crate::render::wgpu_state::WgpuState;

pub struct App<'a> {
    window: Option<Arc<Window>>,
    wgpu_state: Option<WgpuState<'a>>,
    mouse_pressed: bool,
    right_mouse_pressed: bool,
    last_mouse_pos: Option<(f64, f64)>,
    pressed_keys: HashSet<KeyCode>,
    last_frame_time: Option<Instant>,
}

impl<'a> Default for App<'a> {
    fn default() -> Self {
        Self {
            window: None,
            wgpu_state: None,
            mouse_pressed: false,
            right_mouse_pressed: false,
            last_mouse_pos: None,
            pressed_keys: HashSet::new(),
            last_frame_time: None,
        }
    }
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
                button,
                ..
            } => {
                let pressed = element_state == ElementState::Pressed;
                if button == MouseButton::Left {
                    self.mouse_pressed = pressed;
                    if !state.debug_mode {
                        if pressed {
                            if let Some((x, y)) = self.last_mouse_pos {
                                state.camera.begin_drag(
                                    x as f32,
                                    y as f32,
                                    state.size.width as f32,
                                    state.size.height as f32,
                                );
                            }
                        } else {
                            state.camera.end_drag();
                        }
                    }
                } else if button == MouseButton::Right {
                    self.right_mouse_pressed = pressed;
                }
            }
            WindowEvent::CursorMoved { position, .. } => {
                let dx = if let Some((last_x, _)) = self.last_mouse_pos {
                    position.x - last_x
                } else {
                    0.0
                };
                let dy = if let Some((_, last_y)) = self.last_mouse_pos {
                    position.y - last_y
                } else {
                    0.0
                };

                if state.debug_mode {
                    if self.right_mouse_pressed {
                        state.debug_camera.process_mouse(dx as f32, dy as f32);
                        window.request_redraw();
                    }
                } else {
                    if self.mouse_pressed {
                        state.camera.drag(
                            position.x as f32,
                            position.y as f32,
                            state.size.width as f32,
                            state.size.height as f32,
                        );
                        window.request_redraw();
                    }
                }
                self.last_mouse_pos = Some((position.x, position.y));
            }
            WindowEvent::MouseWheel { delta, .. } => {
                if !state.debug_mode {
                    let zoom_delta = match delta {
                        MouseScrollDelta::LineDelta(_, y) => y,
                        MouseScrollDelta::PixelDelta(pos) => (pos.y / 50.0) as f32,
                    };
                    state.camera.zoom(zoom_delta);
                    window.request_redraw();
                }
            }
            WindowEvent::KeyboardInput {
                event:
                    KeyEvent {
                        physical_key: PhysicalKey::Code(keycode),
                        state: element_state,
                        ..
                    },
                ..
            } => {
                if element_state == ElementState::Pressed {
                    self.pressed_keys.insert(keycode);
                } else {
                    self.pressed_keys.remove(&keycode);
                }

                if !state.debug_mode {
                    match keycode {
                        KeyCode::ArrowUp | KeyCode::KeyW => {
                            if element_state == ElementState::Pressed {
                                state.camera.pitch(1.0);
                                window.request_redraw();
                            }
                        }
                        KeyCode::ArrowDown | KeyCode::KeyS => {
                            if element_state == ElementState::Pressed {
                                state.camera.pitch(-1.0);
                                window.request_redraw();
                            }
                        }
                        _ => {}
                    }
                }
            }
            _ => {}
        }
    }

    fn about_to_wait(&mut self, _event_loop: &ActiveEventLoop) {
        if let Some(state) = &mut self.wgpu_state {
            if state.debug_mode {
                let now = Instant::now();
                let dt = if let Some(last) = self.last_frame_time {
                    now.duration_since(last).as_secs_f32()
                } else {
                    0.016
                };
                self.last_frame_time = Some(now);

                let mut movement = Vec3::ZERO;
                if self.pressed_keys.contains(&KeyCode::KeyW) {
                    movement.z += 1.0;
                }
                if self.pressed_keys.contains(&KeyCode::KeyS) {
                    movement.z -= 1.0;
                }
                if self.pressed_keys.contains(&KeyCode::KeyD) {
                    movement.x += 1.0;
                }
                if self.pressed_keys.contains(&KeyCode::KeyA) {
                    movement.x -= 1.0;
                }

                let fast = self.pressed_keys.contains(&KeyCode::ShiftLeft) || self.pressed_keys.contains(&KeyCode::ShiftRight);
                
                if self.pressed_keys.contains(&KeyCode::Space) {
                    let ctrl = self.pressed_keys.contains(&KeyCode::ControlLeft) || self.pressed_keys.contains(&KeyCode::ControlRight);
                    if ctrl {
                        movement.y -= 1.0;
                    } else {
                        movement.y += 1.0;
                    }
                }
                if movement != Vec3::ZERO {
                    state.debug_camera.update(dt, movement.normalize_or_zero(), fast);
                    if let Some(window) = &self.window {
                        window.request_redraw();
                    }
                }
            } else {
                self.last_frame_time = None;
            }
        }

        if let Some(window) = &self.window {
            window.request_redraw();
        }
    }
}
