use glam::Vec3;
use std::collections::HashSet;
use std::sync::mpsc;
use std::sync::Arc;
use std::time::Instant;
use winit::application::ApplicationHandler;
use winit::event::{ElementState, KeyEvent, MouseButton, MouseScrollDelta, WindowEvent};
use winit::event_loop::ActiveEventLoop;
use winit::keyboard::{KeyCode, PhysicalKey};
use winit::window::{Window, WindowId};

use crate::core::command::{CameraCommandMode, ViewerCommand};
use crate::render::wgpu_state::WgpuState;

pub struct App<'a> {
    window: Option<Arc<Window>>,
    wgpu_state: Option<WgpuState<'a>>,
    mouse_pressed: bool,
    right_mouse_pressed: bool,
    last_mouse_pos: Option<(f64, f64)>,
    pressed_keys: HashSet<KeyCode>,
    last_frame_time: Option<Instant>,
    config: crate::globe::tiles::config::TileEngineConfig,
    extension: Option<Box<dyn crate::core::extension::GlobeExtension>>,
    command_rx: Option<mpsc::Receiver<ViewerCommand>>,
    touch_interpreter: crate::core::touch::TouchInterpreter,
}

impl<'a> App<'a> {
    pub fn new(
        config: crate::globe::tiles::config::TileEngineConfig,
        extension: Option<Box<dyn crate::core::extension::GlobeExtension>>,
        command_rx: Option<mpsc::Receiver<ViewerCommand>>,
    ) -> Self {
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
            command_rx,
            touch_interpreter: crate::core::touch::TouchInterpreter::new(),
        }
    }

    pub fn render_state(&self) -> Option<&WgpuState<'a>> {
        self.wgpu_state.as_ref()
    }

    fn render_ui(ctx: &egui::Context, state: &mut WgpuState) {
        egui::Window::new("Flight Tracker Debug")
            .resizable(false)
            .show(ctx, |ui| {
                ui.label(format!("Altitude: {:.4}", state.camera.altitude()));

                // Sun intensity is now controlled by the flight JSON interpolation
                ui.horizontal(|ui| {
                    ui.label(format!("Sun Intensity: {:.2}", state.camera.sun_intensity));
                });

                let mut is_debug = state.debug_mode;
                if ui
                    .checkbox(&mut is_debug, "Debug Mode (Dual Camera)")
                    .changed()
                {
                    state.debug_mode = is_debug;
                    if is_debug && !state.debug_camera_initialized {
                        let (global_pos, global_ori) = state.camera.global_transform();
                        let forward =
                            (global_ori * glam::Vec3::new(0.0, 0.0, -1.0)).normalize_or_zero();
                        let pitch = forward.y.asin();
                        let yaw = forward.x.atan2(-forward.z);
                        state.debug_camera = crate::camera::GodCamera::new(global_pos, yaw, pitch);
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
                            let forward =
                                (global_ori * glam::Vec3::new(0.0, 0.0, -1.0)).normalize_or_zero();
                            let pitch = forward.y.asin();
                            let yaw = forward.x.atan2(-forward.z);
                            state.debug_camera =
                                crate::camera::GodCamera::new(global_pos, yaw, pitch);
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
                        ui.add(
                            egui::DragValue::new(&mut pitch_deg)
                                .speed(1.0)
                                .prefix("P: "),
                        );
                        ui.add(egui::DragValue::new(&mut yaw_deg).speed(1.0).prefix("Y: "));
                        ui.add(egui::DragValue::new(&mut roll_deg).speed(1.0).prefix("R: "));
                    });

                    ui.horizontal(|ui| {
                        ui.label("Lens:");
                        ui.add(
                            egui::Slider::new(&mut state.camera.focal_length, 12.0..=200.0)
                                .text("Focal Length (mm)"),
                        );
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
                ui.collapsing("Map Labels Settings", |ui| {
                    ui.checkbox(&mut state.label_manager.enabled, "Enable Labels");
                    if state.label_manager.enabled {
                        ui.add(
                            egui::Slider::new(&mut state.label_manager.size_scale, 0.5..=2.0)
                                .text("Size Scale"),
                        );
                        
                        let mut max_rank = state.label_manager.max_importance_rank;
                        if ui.add(
                            egui::Slider::new(&mut max_rank, 0..=15)
                                .text("Max Rank (0=Capitals, 15=All)")
                        ).changed() {
                            state.label_manager.max_importance_rank = max_rank;
                        }
                        
                        ui.checkbox(&mut state.label_manager.show_anchor_dots, "Show Anchor Dots");
                        ui.label(format!("Visible Labels: {}", state.label_manager.visible_labels.len()));
                    }
                });
                ui.separator();
                if let Some(ext) = &mut state.extension {
                    ext.render_ui(ctx, ui);
                }
            });
    }


    fn render_label_indicators(ctx: &egui::Context, state: &WgpuState) {
        if !state.label_manager.enabled {
            return;
        }

        let screen_rect = ctx.screen_rect();
        let width = screen_rect.width();
        let height = screen_rect.height();
        let aspect_ratio = width / height;

        let view_matrix = state.camera.get_view_matrix();
        let proj_matrix = state.camera.get_projection_matrix(aspect_ratio);
        let view_proj = proj_matrix * view_matrix;

        let altitude = state.camera.altitude().max(0.0001);
        // Max distance at which a rank-0 label is visible (Megameters)
        let max_render_dist = (altitude * 1.5 + 0.15).max(0.15);

        // Paint above the globe scene but below egui windows
        let painter = ctx.layer_painter(egui::LayerId::new(
            egui::Order::Background,
            egui::Id::new("label_layer"),
        ));

        for label in &state.label_manager.visible_labels {
            let ecef = label.ecef_pos;
            let clip_pos = view_proj * glam::Vec4::new(ecef.x, ecef.y, ecef.z, 1.0);

            // Only render points in front of the near clipping plane
            if clip_pos.w <= 0.0 {
                continue;
            }

            let ndc_x = clip_pos.x / clip_pos.w;
            let ndc_y = clip_pos.y / clip_pos.w;

            // Clip to visible viewport
            if ndc_x < -1.0 || ndc_x > 1.0 || ndc_y < -1.0 || ndc_y > 1.0 {
                continue;
            }

            let screen_x = (ndc_x + 1.0) * 0.5 * width;
            let screen_y = (1.0 - ndc_y) * 0.5 * height;

            // Compute a proximity factor [0.0 = at max range, 1.0 = very close]
            // Use clip_pos.w as a reliable depth proxy (larger = further away)
            let depth = clip_pos.w.max(0.001);
            // depth is in the same units as the scene; normalize against max_render_dist
            let proximity = 1.0 - (depth / (max_render_dist + depth)).clamp(0.0, 1.0);

            // Rank-based font size boost: capitals and major cities are larger
            let rank_scale = if label.label_rank <= 2 {
                1.3_f32
            } else if label.label_rank <= 5 {
                1.0_f32
            } else {
                0.82_f32
            };

            // Dynamic font size: ranges from 9px (distant) to 14px (near), scaled by rank and size_scale
            let font_size = (9.0 + proximity * 5.0) * rank_scale * state.label_manager.size_scale;

            // Dynamic opacity for text and backdrop
            let text_alpha = ((160.0 + proximity * 95.0) as u8).max(100);
            let bg_alpha   = ((90.0  + proximity * 90.0) as u8).max(60);
            let dot_radius = 1.5 + proximity * 1.5;

            let text_color = egui::Color32::from_white_alpha(text_alpha);
            let bg_color   = egui::Color32::from_rgba_unmultiplied(8, 12, 18, bg_alpha);

            let anchor_pos = egui::pos2(screen_x, screen_y);

            // --- Draw label text with backdrop ---
            let font_id = egui::FontId::proportional(font_size);
            let galley = ctx.fonts(|f| {
                f.layout_no_wrap(label.name.to_string(), font_id, text_color)
            });

            // Position text pill centered horizontally above the anchor dot
            let text_size = galley.size();
            let pad_x = 4.0;
            let pad_y = 2.5;
            let pill_w = text_size.x + pad_x * 2.0;
            let pill_h = text_size.y + pad_y * 2.0;
            let pill_x = screen_x - pill_w * 0.5;
            let dot_offset = if state.label_manager.show_anchor_dots { dot_radius } else { 0.0 };
            let pill_y = screen_y - dot_offset - 3.0 - pill_h;

            let bg_rect = egui::Rect::from_min_size(
                egui::pos2(pill_x, pill_y),
                egui::vec2(pill_w, pill_h),
            );

            // Backdrop pill
            painter.rect_filled(bg_rect, egui::Rounding::same(3.0), bg_color);

            // Text
            painter.galley(
                egui::pos2(pill_x + pad_x, pill_y + pad_y),
                galley,
                text_color,
            );

            // Anchor dot
            if state.label_manager.show_anchor_dots {
                painter.circle_filled(anchor_pos, dot_radius + 0.5, egui::Color32::from_black_alpha(120));
                painter.circle_filled(anchor_pos, dot_radius, egui::Color32::from_white_alpha(text_alpha));
            }
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
            let window_attributes = Window::default_attributes()
                .with_title("CesiumRS WGS84 Ellipsoid")
                .with_inner_size(winit::dpi::PhysicalSize::new(800, 600));

            let window = Arc::new(event_loop.create_window(window_attributes).unwrap());
            self.window = Some(window.clone());

            let state = pollster::block_on(WgpuState::new(
                window,
                self.config.clone(),
                self.extension.take(),
            ));
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
                match state.render(None, false, |ctx, s| { Self::render_ui(ctx, s); Self::render_label_indicators(ctx, s); }) {
                    Ok(_) => {}
                    Err(wgpu::SurfaceError::Lost) => state.resize(state.size),
                    Err(wgpu::SurfaceError::OutOfMemory) => event_loop.exit(),
                    Err(e) => log::error!("{:?}", e),
                }
            }
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
                                if state.camera.mode == crate::camera::camera::CameraMode::Free {
                                    state.camera.begin_drag(
                                        x as f32,
                                        y as f32,
                                        state.size.width as f32,
                                        state.size.height as f32,
                                    );
                                }
                            }
                        } else {
                            if state.camera.mode == crate::camera::camera::CameraMode::Free {
                                state.camera.end_drag();
                            }
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
                        match state.camera.mode {
                            crate::camera::camera::CameraMode::Free => {
                                state.camera.drag(
                                    position.x as f32,
                                    position.y as f32,
                                    state.size.width as f32,
                                    state.size.height as f32,
                                );
                            }
                            crate::camera::camera::CameraMode::Tracking => {
                                state.camera.orbit_mouse(dx as f32, dy as f32);
                            }
                            crate::camera::camera::CameraMode::Cockpit => {
                                state.camera.look_around(dx as f32, dy as f32);
                            }
                        }
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
            WindowEvent::Touch(touch) => {
                if !state.debug_mode {
                    let screen_width = state.size.width as f32;
                    let screen_height = state.size.height as f32;
                    let redrew = self.touch_interpreter.handle_touch_event(
                        &touch,
                        &mut state.camera,
                        screen_width,
                        screen_height,
                    );
                    if redrew {
                        window.request_redraw();
                    }
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
                        KeyCode::ArrowDown | KeyCode::KeyS
                            if element_state == ElementState::Pressed =>
                        {
                            state.camera.pitch(-1.0);
                            window.request_redraw();
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
            // Drain any commands submitted via ViewerHandle
            if let Some(rx) = &self.command_rx {
                while let Ok(cmd) = rx.try_recv() {
                    match cmd {
                        ViewerCommand::CameraSetPosition { lon, lat, alt } => {
                            let ecef =
                                crate::globe::geometry::lon_lat_alt_to_ecef_f64(lon, lat, alt);
                            let pos =
                                glam::Vec3::new(ecef[0] as f32, ecef[1] as f32, ecef[2] as f32);
                            state.camera.set_eye(pos, glam::Vec3::ZERO);
                        }
                        ViewerCommand::CameraSetMode(mode) => {
                            state.camera.mode = match mode {
                                CameraCommandMode::Free => crate::camera::camera::CameraMode::Free,
                                CameraCommandMode::Tracking => {
                                    crate::camera::camera::CameraMode::Tracking
                                }
                                CameraCommandMode::Cockpit => {
                                    crate::camera::camera::CameraMode::Cockpit
                                }
                            };
                        }
                        ViewerCommand::CameraSetAnchor {
                            position,
                            orientation,
                        } => {
                            let pos = glam::DVec3::from_array(position);
                            let ori = glam::DQuat::from_xyzw(
                                orientation[0],
                                orientation[1],
                                orientation[2],
                                orientation[3],
                            );
                            state.camera.set_anchor(pos, ori);
                        }
                        ViewerCommand::CameraZoom(delta) => state.camera.zoom(delta),
                        ViewerCommand::CameraPitch(delta) => state.camera.pitch(delta),
                        ViewerCommand::MapSetSaturation(v) => {
                            state.tile_system.config.map_saturation = v
                        }
                        ViewerCommand::MapSetContrast(v) => {
                            state.tile_system.config.map_contrast = v
                        }
                        ViewerCommand::MapSetBrightness(v) => {
                            state.tile_system.config.map_brightness = v
                        }
                    }
                }
            }

            if !state.debug_mode {
                let mut zoom_delta = 0.0;
                if self.pressed_keys.contains(&KeyCode::KeyI) || self.pressed_keys.contains(&KeyCode::PageUp) {
                    zoom_delta += 1.0;
                }
                if self.pressed_keys.contains(&KeyCode::KeyO) || self.pressed_keys.contains(&KeyCode::PageDown) {
                    zoom_delta -= 1.0;
                }
                if zoom_delta != 0.0 {
                    state.camera.zoom(zoom_delta * 4.0 * dt);
                }
            }

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

                let fast = self.pressed_keys.contains(&KeyCode::ShiftLeft)
                    || self.pressed_keys.contains(&KeyCode::ShiftRight);

                if self.pressed_keys.contains(&KeyCode::Space) {
                    let ctrl = self.pressed_keys.contains(&KeyCode::ControlLeft)
                        || self.pressed_keys.contains(&KeyCode::ControlRight);
                    if ctrl {
                        movement.y -= 1.0;
                    } else {
                        movement.y += 1.0;
                    }
                }
                if movement != Vec3::ZERO {
                    state
                        .debug_camera
                        .update(dt, movement.normalize_or_zero(), fast);
                }
            }
        }

        if let Some(window) = &self.window {
            window.request_redraw();
        }
    }
}
