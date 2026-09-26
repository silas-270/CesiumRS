#[cfg(feature = "debug_panel")]
use glam::Vec3;
use std::collections::HashSet;
use std::sync::mpsc;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
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

                    // Determine which of the three styles is currently active.
                    use crate::globe::tiles::config::{
                        TileSourceMode, OFFLINE_IMAGERY_MAX_LEVEL, SATELLITE_IMAGERY_MAX_LEVEL,
                        satellite_imagery_url, STANDARD_IMAGERY_MAX_LEVEL, standard_imagery_url,
                    };
                    #[derive(PartialEq, Clone, Copy)]
                    enum StyleSel { Standard, Satellite, Offline }

                    let sel = match &state.tile_system.config.tile_source_mode {
                        TileSourceMode::SvgVector(_) => StyleSel::Offline,
                        TileSourceMode::HttpNetwork => {
                            if state.tile_system.config.base_imagery_url == satellite_imagery_url() {
                                StyleSel::Satellite
                            } else {
                                StyleSel::Standard
                            }
                        }
                    };
                    let mut sel = sel;

                    if ui.radio_value(&mut sel, StyleSel::Standard, "Standard").changed() {
                        state.set_base_imagery_url(standard_imagery_url(), STANDARD_IMAGERY_MAX_LEVEL);
                        state.set_terrain_enabled(false);
                    }
                    if ui.radio_value(&mut sel, StyleSel::Satellite, "Satellite + Terrain").changed() {
                        state.set_base_imagery_url(satellite_imagery_url(), SATELLITE_IMAGERY_MAX_LEVEL);
                        state.set_terrain_enabled(true);
                    }
                    if ui.radio_value(&mut sel, StyleSel::Offline, "Offline (SVG)").changed() {
                        match crate::globe::tiles::vector::bundled_world_renderer() {
                            Ok(renderer) => {
                                state.set_tile_source_mode(
                                    String::new(),
                                    TileSourceMode::SvgVector(std::sync::Arc::new(renderer)),
                                    OFFLINE_IMAGERY_MAX_LEVEL,
                                );
                                state.set_terrain_enabled(false);
                            }
                            Err(e) => {
                                log::error!("Debug panel: failed to switch to offline mode: {e}");
                            }
                        }
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
                    ui.separator();
                }

                ui.label("📸 Screenshot (4K UHD)");
                let is_capturing = crate::core::screenshot::SCREENSHOT_IN_PROGRESS
                    .load(std::sync::atomic::Ordering::Relaxed);
                ui.horizontal(|ui| {
                    if ui
                        .add_enabled(!is_capturing, egui::Button::new("📷 Capture 4K Screenshot"))
                        .clicked()
                    {
                        let camera_state = crate::core::screenshot::ScreenshotCameraState {
                            anchor_pos: state.camera.anchor_pos,
                            anchor_ori: state.camera.anchor_ori,
                            local_pos: state.camera.local_pos,
                            local_ori: state.camera.local_ori,
                            focal_length: state.camera.focal_length,
                            sun_intensity: state.camera.sun_intensity,
                            mode: state.camera.mode,
                        };
                        let label_snapshot = crate::core::screenshot::LabelManagerStateSnapshot {
                            enabled: state.label_manager.enabled,
                            size_scale: state.label_manager.size_scale,
                            max_importance_rank: state.label_manager.max_importance_rank,
                            show_anchor_dots: state.label_manager.show_anchor_dots,
                        };
                        let ext_snapshot = state
                            .extension
                            .as_ref()
                            .and_then(|e| e.snapshot_for_headless());
                        crate::core::screenshot::trigger_4k_screenshot(
                            state.tile_system.config.clone(),
                            camera_state,
                            ext_snapshot,
                            Some(label_snapshot),
                        );
                    }
                });

                if let Ok(status_guard) = crate::core::screenshot::SCREENSHOT_STATUS.lock() {
                    if let Some((msg, is_error)) = &*status_guard {
                        if *is_error {
                            ui.colored_label(egui::Color32::RED, msg);
                        } else if is_capturing {
                            ui.colored_label(egui::Color32::YELLOW, msg);
                        } else {
                            ui.colored_label(egui::Color32::GREEN, msg);
                        }
                    }
                }
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
                _event_loop.exit();
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
                let render_result = state.render(None, false, |_ctx, _s| {
                    // The sliders/checkboxes window is a desktop dev tool, kept off the
                    // real UI on Android. City labels are drawn in the scene pass
                    // (`render::label_pipeline`), not here.
                    #[cfg(not(target_os = "android"))]
                    Self::render_ui(_ctx, _s);
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
                        ViewerCommand::MapSetImageryUrl { url, max_level } => {
                            state.set_base_imagery_url(url, max_level);
                        }
                        ViewerCommand::MapSetSourceMode { url, mode, max_level } => {
                            state.set_tile_source_mode(url, mode, max_level);
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
