#[cfg(feature = "debug_panel")]
use glam::Vec3;
use std::collections::HashSet;
use std::sync::mpsc;
use std::sync::Arc;
use std::sync::atomic::AtomicBool;
use std::time::Instant;

pub static RENDERING_ENABLED: AtomicBool = AtomicBool::new(false);

#[derive(Debug, Clone, Copy)]
pub enum EngineEvent {
    Suspend,
    Resume,
    /// Sent by CesiumEngineManager.onDestroy (only when NOT isChangingConfigurations).
    /// Causes the winit event loop to exit cleanly, dropping WgpuState and all Vulkan resources.
    Destroy,
}
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
    /// Whether to write `camera_trace.csv` and `tile_trace.csv`: set by the
    /// `CESIUM_TRACE` environment variable (`1`, `on`, `true`) or [`Self::with_traces`].
    #[cfg(not(target_os = "android"))]
    traces: bool,
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
            #[cfg(not(target_os = "android"))]
            traces: matches!(
                std::env::var("CESIUM_TRACE").as_deref(),
                Ok("1" | "on" | "true")
            ),
        }
    }

    /// Writes the per-frame camera trace and the tile lifecycle trace whatever
    /// `CESIUM_TRACE` says: for the test apps whose output they are.
    #[cfg(not(target_os = "android"))]
    pub fn with_traces(mut self, on: bool) -> Self {
        self.traces = on;
        self
    }

    pub fn render_state(&self) -> Option<&WgpuState<'a>> {
        self.wgpu_state.as_ref()
    }

    #[cfg(feature = "debug_panel")]
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
                ui.horizontal(|ui| {
                    ui.label("Map Style:");
                    let current_url = &state.tile_system.config.base_imagery_url;
                    let is_sat_terrain = current_url == crate::globe::tiles::config::SATELLITE_IMAGERY_URL
                        && state.tile_system.config.terrain.enabled;
                    let mut selected_sat = is_sat_terrain;
                    if ui.radio_value(&mut selected_sat, false, "Standard (Carto Dark)").changed() {
                        state.set_base_imagery_url(
                            crate::globe::tiles::config::STANDARD_IMAGERY_URL.to_string(),
                        );
                        state.set_terrain_enabled(false);
                    }
                    if ui.radio_value(&mut selected_sat, true, "Satellite + Terrain (Esri)").changed() {
                        state.set_base_imagery_url(
                            crate::globe::tiles::config::SATELLITE_IMAGERY_URL.to_string(),
                        );
                        state.set_terrain_enabled(true);
                    }
                });

                ui.separator();
                ui.collapsing("Terrain (height data)", |ui| {
                    // The map style above sets this (satellite on, standard off); the box
                    // overrides it for the style in use, e.g. relief under the dark map.
                    // The switch rebuilds the height manager, and E2 rebuilds the meshes
                    // that were flat when it was off.
                    let mut on = state.tile_system.config.terrain.enabled;
                    if ui
                        .checkbox(&mut on, "Draw the globe with relief")
                        .changed()
                    {
                        state.set_terrain_enabled(on);
                    }

                    let terrain = &state.tile_system.config.terrain;
                    ui.label(format!("Source max level: z{}", terrain.max_level));
                    ui.label(format!(
                        "Ocean: {:?}  ·  Exaggeration: {:.2}x (applied in Phase C)",
                        terrain.ocean, terrain.exaggeration
                    ));

                    // Height residency is reported separately from imagery on purpose
                    // (§5 B4): the height cache takes a declared slice of the tile byte
                    // budget, and the only way to see that it is a slice and not an
                    // addition is to show both halves.
                    let imagery_mb =
                        state.tile_system.config.imagery_cache_budget_bytes() / (1024 * 1024);
                    ui.label(format!(
                        "Imagery budget: {} MB  ·  {} entries resident",
                        imagery_mb,
                        state.tile_system.texture_manager.cache.len()
                    ));
                    match state.tile_system.height_manager.as_ref() {
                        Some(h) => {
                            let (resident, capacity) = h.residency();
                            ui.label(format!(
                                "Height budget: {} MB  ·  {resident}/{capacity} tiles, {} MB resident",
                                state.tile_system.config.terrain.height_cache_budget_bytes
                                    / (1024 * 1024),
                                h.resident_bytes() / (1024 * 1024),
                            ));
                        }
                        None => {
                            ui.label("Height budget: 0 MB · cache not built (terrain off)");
                        }
                    }
                });

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

    #[cfg(feature = "debug_panel")]
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

        let (cam_pos, _) = state.camera.global_transform();
        let altitude = state.camera.altitude().max(0.0001);
        // Max distance at which a rank-0 label is visible (Megameters)
        let max_render_dist = (altitude * 1.5 + 0.15).max(0.15);
        let r_earth = 6.378137_f32;
        let horizon_dist = (2.0 * r_earth * altitude + altitude * altitude).sqrt();
        let max_dist_rank02 = horizon_dist.max(max_render_dist);

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

            let dist = (ecef - cam_pos).length();
            let max_dist = if label.label_rank <= 2 {
                max_dist_rank02
            } else {
                max_render_dist
            };

            // Relative distance normalized across visible range [altitude .. max_dist]
            let dist_span = (max_dist - altitude).max(0.01);
            let rel_dist = ((dist - altitude) / dist_span).clamp(0.0, 1.0);

            // Proximity factor [0.0 = at max range / horizon, 1.0 = near nadir / camera]
            let proximity = 1.0 - rel_dist;

            // Continuous distance & atmospheric haze fade:
            // Labels stay at full opacity for the nearest ~35% of the range, then smoothly fade to 0
            // using a cubic Hermite smoothstep between fade_start and fade_end.
            let fade_start = 0.35_f32;
            let fade_end = 0.95_f32;
            let t = ((rel_dist - fade_start) / (fade_end - fade_start)).clamp(0.0, 1.0);
            let dist_fade = 1.0 - t * t * (3.0 - 2.0 * t);

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

            // Dynamic opacity for text, background pill, and anchor dot with continuous distance fade
            let text_alpha_f = (180.0 + proximity * 75.0) * dist_fade;
            let bg_alpha_f = (100.0 + proximity * 70.0) * dist_fade;
            let shadow_alpha_f = 120.0 * dist_fade;

            let text_alpha = text_alpha_f.round() as u8;
            let bg_alpha = bg_alpha_f.round() as u8;
            let shadow_alpha = shadow_alpha_f.round() as u8;

            // Skip rendering nearly invisible labels to save CPU font layout and GPU UI painter overhead
            if text_alpha < 3 && bg_alpha < 3 {
                continue;
            }

            let dot_radius = 1.5 + proximity * 1.5;
            let text_color = egui::Color32::from_white_alpha(text_alpha);
            let bg_color = egui::Color32::from_rgba_unmultiplied(8, 12, 18, bg_alpha);

            let screen_x = (ndc_x + 1.0) * 0.5 * width;
            let screen_y = (1.0 - ndc_y) * 0.5 * height;
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
                painter.circle_filled(anchor_pos, dot_radius + 0.5, egui::Color32::from_black_alpha(shadow_alpha));
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

#[cfg(target_os = "android")]
pub type AppUserEvent = EngineEvent;

#[cfg(not(target_os = "android"))]
pub type AppUserEvent = ();

impl<'a> ApplicationHandler<AppUserEvent> for App<'a> {
    fn user_event(&mut self, _event_loop: &ActiveEventLoop, _event: AppUserEvent) {
        #[cfg(target_os = "android")]
        match _event {
            EngineEvent::Suspend => {
                _event_loop.set_control_flow(winit::event_loop::ControlFlow::Wait);
            }
            EngineEvent::Resume => {
                _event_loop.set_control_flow(winit::event_loop::ControlFlow::Poll);
            }
            EngineEvent::Destroy => {
                // Exits the winit run_app() loop. WgpuState is dropped when App is dropped
                // immediately after, releasing the wgpu::Device and all Vulkan resources.
                event_loop.exit();
            }
        }
    }

    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        if self.window.is_none() {

            let window_attributes = Window::default_attributes()
                .with_title("CesiumRS WGS84 Ellipsoid")
                .with_inner_size(winit::dpi::PhysicalSize::new(800, 600));

            let window = Arc::new(event_loop.create_window(window_attributes).unwrap());

            if let Some(state) = &mut self.wgpu_state {
                log::info!("[WINDOW LIFECYCLE] Recreating surface for window...");
                state.recreate_surface(window.clone());
                self.window = Some(window);
            } else {
                self.window = Some(window.clone());
                log::info!("[WINDOW LIFECYCLE] Initializing WgpuState asynchronously...");
                let state = pollster::block_on(WgpuState::new(
                    Some(window.clone()),
                    None,
                    self.config.clone(),
                    self.extension.take(),
                ));
                log::info!("[WINDOW LIFECYCLE] WgpuState successfully created!");
                #[cfg(not(target_os = "android"))]
                let state = {
                    let mut state = state;
                    if self.traces {
                        state.camera_trace = crate::camera::trace::CameraTrace::create();
                        crate::globe::tiles::trace::enable();
                    }
                    state
                };
                self.wgpu_state = Some(state);
            }
        }
        event_loop.set_control_flow(winit::event_loop::ControlFlow::Poll);
    }

    fn suspended(&mut self, event_loop: &ActiveEventLoop) {
        log::info!("[WINDOW LIFECYCLE] suspended called. Dropping window and surface.");
        // On Android, this is called when the Activity is destroyed or sent to background.
        // We drop only the surface and window so we don't try to render to a dead surface,
        // but keep the WgpuState (device, pipelines, buffers) alive for fast resume.
        if let Some(state) = &mut self.wgpu_state {
            state.surface = None;
            state.window = None;
        }
        self.window = None;
        event_loop.set_control_flow(winit::event_loop::ControlFlow::Wait);
    }

    fn window_event(
        &mut self,
        event_loop: &ActiveEventLoop,
        window_id: WindowId,
        event: WindowEvent,
    ) {
        let window = match self.window.as_ref() {
            Some(w) => w,
            None => return,
        };
        if window.id() != window_id {
            return;
        }

        let state = match self.wgpu_state.as_mut() {
            Some(s) => s,
            None => return,
        };
        #[cfg(feature = "debug_panel")]
        let is_debug = state.debug_mode;
        #[cfg(not(feature = "debug_panel"))]
        let is_debug = false;

        #[cfg(feature = "debug_panel")]
        {
            let response = state.egui_state.as_mut().unwrap().on_window_event(window, &event);
            if response.consumed {
                return;
            }
        }

        match event {
            WindowEvent::CloseRequested | WindowEvent::Destroyed => {
                log::info!("[WINDOW LIFECYCLE] Event {:?} received from compositor/OS. Exiting event loop.", event);
                event_loop.exit();
            }
            WindowEvent::Resized(physical_size) => {
                state.resize(physical_size);
            }
            WindowEvent::RedrawRequested => {
                // Skip rendering when the Compose UI is covering the globe (battery saver)
                #[cfg(target_os = "android")]
                if !RENDERING_ENABLED.load(Ordering::Relaxed) {
                    return;
                }

                // Started here (not in `about_to_wait`) because this is where the 60fps
                // throttle/battery-saver sleep has already happened, so the span reflects
                // true CPU-busy work per frame rather than wall-clock-including-idle-sleep.
                let _frame_span = crate::core::trace::ScopedTrace::new("cesium.frame");

                #[cfg(feature = "debug_panel")]
                let render_result = state.render(None, false, |ctx, s| {
                    // The sliders/checkboxes window is a desktop dev tool; Android only wants
                    // the label pills it draws, not the debug chrome on top of the real UI.
                    #[cfg(not(target_os = "android"))]
                    Self::render_ui(ctx, s);
                    Self::render_label_indicators(ctx, s);
                });
                #[cfg(not(feature = "debug_panel"))]
                let render_result = state.render(None, false);

                match render_result {
                    Ok(_) => {}
                    Err(wgpu::SurfaceError::Lost) => {
                        log::warn!("GPU Surface lost! Resizing to: {:?}", state.size);
                        state.resize(state.size);
                    }
                    Err(wgpu::SurfaceError::OutOfMemory) => {
                        // Write directly to the log file — env_logger's internal pipe
                        // buffer may not be flushed before event_loop.exit() unwinds,
                        // which is why this error previously disappeared from cesium.log.
                        let unix_ms = std::time::SystemTime::now()
                            .duration_since(std::time::UNIX_EPOCH)
                            .map(|d| d.as_millis())
                            .unwrap_or(0);
                        let msg = format!(
                            "[{}.{}s UNIX ERROR cesium_engine::core::app] \
                             FATAL: GPU Surface out of memory (SurfaceError::OutOfMemory)! \
                             VRAM exhausted — likely a burst texture upload spike. \
                             Exiting event loop.\n",
                            unix_ms / 1000,
                            unix_ms % 1000,
                        );
                        eprint!("{}", msg);
                        use std::io::Write as _;
                        if let Ok(mut f) = std::fs::OpenOptions::new()
                            .append(true)
                            .open("cesium.log")
                        {
                            let _ = f.write_all(msg.as_bytes());
                            let _ = f.flush();
                        }
                        event_loop.exit();
                    }
                    Err(wgpu::SurfaceError::Timeout) => {
                        log::warn!("GPU Surface texture acquisition timeout. Skipping frame.");
                    }
                    Err(wgpu::SurfaceError::Outdated) => {
                        log::warn!("GPU Surface outdated! Reconfiguring surface.");
                        state.resize(state.size);
                    }
                }
            }
            WindowEvent::MouseInput {
                state: element_state,
                button,
                ..
            } => {
                let pressed = element_state == ElementState::Pressed;
                log::info!("[INPUT MOUSE CLICK] button={:?} state={:?} pos={:?}", button, element_state, self.last_mouse_pos);
                if button == MouseButton::Left {
                    self.mouse_pressed = pressed;
                    if !is_debug {
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

                if is_debug {
                    #[cfg(feature = "debug_panel")]
                    if self.right_mouse_pressed {
                        state.debug_camera.process_mouse(dx as f32, dy as f32);
                        window.request_redraw();
                    }
                } else {
                    if self.mouse_pressed {
                        log::info!(
                            "[INPUT MOUSE DRAG] mode={:?} pos=({:.1}, {:.1}) delta=({:.1}, {:.1})",
                            state.camera.mode,
                            position.x,
                            position.y,
                            dx,
                            dy
                        );
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
                if !is_debug {
                    let zoom_delta = match delta {
                        MouseScrollDelta::LineDelta(_, y) => y,
                        MouseScrollDelta::PixelDelta(pos) => (pos.y / 50.0) as f32,
                    };
                    log::info!(
                        "[INPUT MOUSE WHEEL] delta={:?} zoom_delta={:.2} mode={:?}",
                        delta,
                        zoom_delta,
                        state.camera.mode
                    );
                    state.camera.zoom(zoom_delta);
                    window.request_redraw();
                }
            }
            WindowEvent::Touch(touch) => {
                if !is_debug {
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
                log::info!("[INPUT KEY] key={:?} state={:?}", keycode, element_state);
                if element_state == ElementState::Pressed {
                    self.pressed_keys.insert(keycode);
                } else {
                    self.pressed_keys.remove(&keycode);
                }

                if !is_debug {
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
        // ── Battery saver: skip rendering when Compose UI is covering the globe ──
        #[cfg(target_os = "android")]
        {
            if !RENDERING_ENABLED.load(Ordering::Relaxed) {
                let _span = crate::core::trace::ScopedTrace::new("cesium.frame.idle_sleep");
                std::thread::sleep(std::time::Duration::from_millis(100));
                return;
            }
        }

        let now = Instant::now();
        let dt = if let Some(last) = self.last_frame_time {
            now.duration_since(last).as_secs_f32().clamp(0.0001, 0.1)
        } else {
            0.016
        };
        self.last_frame_time = Some(now);

        if let Some(state) = &mut self.wgpu_state {
            #[cfg(feature = "debug_panel")]
            let is_debug = state.debug_mode;
            #[cfg(not(feature = "debug_panel"))]
            let is_debug = false;

            // Drain any commands submitted via ViewerHandle
            if let Some(rx) = &self.command_rx {
                while let Ok(cmd) = rx.try_recv() {
                    log::info!("[COMMAND RECV] {:?}", cmd);
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
                        #[cfg(feature = "perf_trace")]
                        ViewerCommand::PerfScenarioMarker(scenario_id) => {
                            // A zero-width begin/end pair, immediately dropped, is
                            // ATrace's idiomatic stand-in for an instant marker — this
                            // is what an analysis script slices the trace by.
                            drop(crate::core::trace::ScopedTrace::new(&format!(
                                "cesium.scenario.{scenario_id}"
                            )));
                        }
                        ViewerCommand::MapSetImageryUrl(url) => {
                            state.set_base_imagery_url(url);
                        }
                        ViewerCommand::MapSetSourceMode { url, mode } => {
                            state.set_tile_source_mode(url, mode);
                        }
                        ViewerCommand::TerrainSetEnabled(on) => {
                            state.set_terrain_enabled(on);
                        }
                    }
                }
            }

            if !is_debug {
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
                let mut needs_redraw = false;
                if state.camera.update_inertia(dt) {
                    needs_redraw = true;
                }
                if needs_redraw {
                    if let Some(w) = &self.window {
                        w.request_redraw();
                    }
                }
            }

            if is_debug {
                #[cfg(feature = "debug_panel")]
                {
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
        }

        if let Some(window) = &self.window {
            window.request_redraw();
        }
    }

    fn exiting(&mut self, _event_loop: &ActiveEventLoop) {
        // Ensure we release any resources when the loop exits
        self.wgpu_state = None;
        self.window = None;
    }
}
