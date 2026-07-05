use std::sync::Arc;
use std::time::Instant;
use std::collections::HashSet;
use glam::Vec3;
use winit::application::ApplicationHandler;
use winit::event::{ElementState, KeyEvent, MouseButton, MouseScrollDelta, WindowEvent};
use winit::event_loop::ActiveEventLoop;
use winit::keyboard::{KeyCode, PhysicalKey};
use winit::window::{Window, WindowId};

use crate::engine::render::wgpu_state::WgpuState;

pub struct App<'a> {
    window: Option<Arc<Window>>,
    wgpu_state: Option<WgpuState<'a>>,
    mouse_pressed: bool,
    right_mouse_pressed: bool,
    last_mouse_pos: Option<(f64, f64)>,
    pressed_keys: HashSet<KeyCode>,
    last_frame_time: Option<Instant>,
    config: crate::engine::globe::tiles::config::TileEngineConfig,
    extension: Option<Box<dyn crate::engine::core::extension::GlobeExtension>>,
}

impl<'a> App<'a> {
    pub fn new(config: crate::engine::globe::tiles::config::TileEngineConfig, extension: Option<Box<dyn crate::engine::core::extension::GlobeExtension>>) -> Self {
        Self {
            window: None,
            wgpu_state: None,
            mouse_pressed: false,
            right_mouse_pressed: false,
            last_mouse_pos: None,
            pressed_keys: HashSet::new(),
            last_frame_time: None,
            config,
            extension,
        }
    }

    fn render_ui(ctx: &egui::Context, state: &mut WgpuState) {
        egui::Window::new("Flight Tracker Debug").resizable(false).show(ctx, |ui| {
            ui.label(format!("Altitude: {:.4}", state.camera.altitude()));

            let mut is_debug = state.debug_mode;
            if ui.checkbox(&mut is_debug, "Debug Mode (Dual Camera)").changed() {
                state.debug_mode = is_debug;
                if is_debug && !state.debug_camera_initialized {
                    let (global_pos, global_ori) = state.camera.global_transform();
                    let forward = (global_ori * glam::Vec3::new(0.0, 0.0, -1.0)).normalize_or_zero();
                    let pitch = forward.y.asin();
                    let yaw = forward.x.atan2(-forward.z);
                    state.debug_camera = crate::engine::camera::GodCamera::new(
                        global_pos,
                        yaw.to_degrees(),
                        pitch.to_degrees(),
                    );
                    state.debug_camera_initialized = true;
                }
            }

            if state.debug_mode {
                ui.separator();
                ui.label("Controls: WASD to move, Right-Click to look");
                ui.label("Space / Ctrl+Space for Up / Down. Shift to boost.");
                ui.separator();
                ui.horizontal(|ui| {
                    if ui.button("Snap God Camera to Main Camera").clicked() {
                        let (global_pos, global_ori) = state.camera.global_transform();
                        let forward = (global_ori * glam::Vec3::new(0.0, 0.0, -1.0)).normalize_or_zero();
                        let pitch = forward.y.asin();
                        let yaw = forward.x.atan2(-forward.z);
                        state.debug_camera = crate::engine::camera::GodCamera::new(global_pos, yaw, pitch);
                    }
                });

                ui.separator();
                ui.label("Main Camera State:");
                ui.horizontal(|ui| {
                    ui.label("Pos:");
                    ui.add(egui::DragValue::new(&mut state.camera.local_pos.x).speed(0.1));
                    ui.add(egui::DragValue::new(&mut state.camera.local_pos.y).speed(0.1));
                    ui.add(egui::DragValue::new(&mut state.camera.local_pos.z).speed(0.1));
                });

                let (yaw, pitch, roll) = state.camera.local_ori.to_euler(glam::EulerRot::YXZ);
                let mut yaw_deg = yaw.to_degrees();
                let mut pitch_deg = pitch.to_degrees();
                let mut roll_deg = roll.to_degrees();

                ui.horizontal(|ui| {
                    ui.label("Rot:");
                    ui.add(egui::DragValue::new(&mut pitch_deg).speed(1.0).prefix("P: "));
                    ui.add(egui::DragValue::new(&mut yaw_deg).speed(1.0).prefix("Y: "));
                    ui.add(egui::DragValue::new(&mut roll_deg).speed(1.0).prefix("R: "));
                });

                ui.horizontal(|ui| {
                    ui.label("Lens:");
                    ui.add(egui::Slider::new(&mut state.camera.focal_length, 12.0..=200.0).text("Focal Length (mm)"));
                });

                if pitch_deg != pitch.to_degrees()
                    || yaw_deg != yaw.to_degrees()
                    || roll_deg != roll.to_degrees()
                {
                    state.camera.local_ori = glam::Quat::from_euler(
                        glam::EulerRot::YXZ,
                        yaw_deg.to_radians(),
                        pitch_deg.to_radians(),
                        roll_deg.to_radians(),
                    );
                }
            }

            ui.separator();
            ui.label("Caching & Performance");
            ui.checkbox(&mut state.tile_system.config.enable_prefetch, "Preload Neighboring Tiles");
            
            let mut texture_cache_size = state.tile_system.config.max_cache_size.get();
            if ui.add(egui::Slider::new(&mut texture_cache_size, 512..=8192).text("Texture Cache Size")).changed() {
                state.tile_system.config.max_cache_size = std::num::NonZeroUsize::new(texture_cache_size).unwrap();
                state.tile_system.texture_manager.resize(state.tile_system.config.max_cache_size);
            }

            let mut mesh_cache_size = state.tile_system.config.mesh_cache_size.get();
            if ui.add(egui::Slider::new(&mut mesh_cache_size, 128..=2048).text("Mesh Cache Size")).changed() {
                state.tile_system.config.mesh_cache_size = std::num::NonZeroUsize::new(mesh_cache_size).unwrap();
                state.resize_tile_cache(state.tile_system.config.mesh_cache_size);
            }

            ui.separator();
            let (req, mis) = state.get_fetch_stats();
            ui.label(format!("Missing Tiles: {} / Requested: {}", mis, req));
        });
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
                Window::default_attributes()
                    .with_title("CesiumRS WGS84 Ellipsoid")
                    .with_inner_size(winit::dpi::PhysicalSize::new(800, 600));

            let window = Arc::new(event_loop.create_window(window_attributes).unwrap());
            self.window = Some(window.clone());

            let state = pollster::block_on(WgpuState::new(window, self.config.clone(), self.extension.take()));
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
            WindowEvent::RedrawRequested => {
                match state.render(None, false, |ctx, s| Self::render_ui(ctx, s)) {
                    Ok(_) => {}
                    Err(wgpu::SurfaceError::Lost) => state.resize(state.size),
                    Err(wgpu::SurfaceError::OutOfMemory) => event_loop.exit(),
                    Err(e) => log::error!("{:?}", e),
                }
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
        let now = Instant::now();
        let dt = if let Some(last) = self.last_frame_time {
            let elapsed = now.duration_since(last);
            let target = std::time::Duration::from_secs_f32(1.0 / 60.0);
            if elapsed < target {
                std::thread::sleep(target - elapsed);
            }
            Instant::now().duration_since(last).as_secs_f32()
        } else {
            0.016
        };
        self.last_frame_time = Some(Instant::now());

        if let Some(state) = &mut self.wgpu_state {
            if state.debug_mode {
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
                }
            }
        }

        if let Some(window) = &self.window {
            window.request_redraw();
        }
    }
}
