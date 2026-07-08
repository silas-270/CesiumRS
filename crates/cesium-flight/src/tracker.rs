use glam::DVec3;
use std::sync::mpsc;

use cesium_engine::camera::camera::CameraMode;
use cesium_engine::core::extension::GlobeExtension;
use cesium_engine::globe::geometry::lon_lat_alt_to_ecef_f64;
use cesium_engine::property::sampled::{InterpolationAlgorithm, SampledPositionProperty};
use cesium_engine::render::model_pipeline::pipeline::ModelRenderer;
use cesium_engine::render::polyline_pipeline::builder::AdaptiveSubdivisionBuilder;
use cesium_engine::render::polyline_pipeline::pipeline::{DrawParams, PolylineConfig, PolylineRenderer};
use cesium_engine::time::SimulationTime;

use crate::flight_handle::{FlightCommand, FlightHandle};

pub fn load_flight_data(
    content: &str,
) -> Result<
    (
        SampledPositionProperty,
        cesium_engine::property::sampled::SampledScalarProperty,
    ),
    Box<dyn std::error::Error>,
> {
    let waypoints: Vec<serde_json::Value> = serde_json::from_str(content)?;

    let mut property =
        SampledPositionProperty::new().with_algorithm(InterpolationAlgorithm::CatmullRom);

    let mut sun_intensity_property = cesium_engine::property::sampled::SampledScalarProperty::new()
        .with_algorithm(InterpolationAlgorithm::CatmullRom);

    for wp in waypoints {
        let time_offset_ms = wp["timeOffsetMs"].as_u64().unwrap_or(0);
        let longitude = wp["longitude"].as_f64().unwrap_or(0.0);
        let latitude = wp["latitude"].as_f64().unwrap_or(0.0);
        let altitude = wp["altitude"].as_f64().unwrap_or(0.0);

        let sun_intensity = wp["sunIntensity"].as_f64().unwrap_or(1.0);

        let ecef_array = lon_lat_alt_to_ecef_f64(longitude, latitude, altitude);
        let position = DVec3::from_array(ecef_array);
        let time = SimulationTime::new(time_offset_ms as f64 / 1000.0);
        property.add_sample(time, position);
        sun_intensity_property.add_sample(time, sun_intensity);
    }

    Ok((property, sun_intensity_property))
}

pub struct FlightEntity {
    pub id: String,
    pub renderer: PolylineRenderer,
    pub config: PolylineConfig,
    pub property: SampledPositionProperty,
    pub sun_intensity_property: cesium_engine::property::sampled::SampledScalarProperty,
    pub reference_point: glam::DVec3,
}

pub struct FlightTrackerApp {
    pub progress: std::sync::Arc<std::sync::Mutex<f64>>,
    pub pending_flights: Vec<(String, String, bool)>, // id, json_content, is_secondary
    pub flights: Vec<FlightEntity>,
    pub airplane_renderer: Option<ModelRenderer>,
    pub last_update_time: std::time::Instant,
    pub is_playing: bool,
    pub play_speed: f64,
    pub view_mode: CameraMode,
    pub last_view_mode: CameraMode,
    pub reset_viewport: bool,
    command_rx: Option<mpsc::Receiver<FlightCommand>>,
}

impl FlightTrackerApp {
    /// Constructs the app and a handle for sending commands to it from other threads.
    pub fn with_handle() -> (Self, FlightHandle) {
        let (tx, rx) = mpsc::sync_channel(64);
        let progress = std::sync::Arc::new(std::sync::Mutex::new(0.0_f64));
        let app = Self {
            progress,
            pending_flights: Vec::new(),
            flights: Vec::new(),
            airplane_renderer: None,
            last_update_time: std::time::Instant::now(),
            is_playing: false,
            play_speed: 0.1,
            view_mode: cesium_engine::camera::camera::CameraMode::Free,
            last_view_mode: cesium_engine::camera::camera::CameraMode::Free,
            reset_viewport: true,
            command_rx: Some(rx),
        };
        (app, FlightHandle::new(tx))
    }

    /// Legacy constructor kept for backwards compat with the test harness.
    pub fn new(progress: std::sync::Arc<std::sync::Mutex<f64>>) -> Self {
        Self {
            progress,
            pending_flights: Vec::new(),
            flights: Vec::new(),
            airplane_renderer: None,
            last_update_time: std::time::Instant::now(),
            is_playing: false,
            play_speed: 0.1,
            view_mode: cesium_engine::camera::camera::CameraMode::Free,
            last_view_mode: cesium_engine::camera::camera::CameraMode::Free,
            reset_viewport: true,
            command_rx: None,
        }
    }

    pub fn get_plane_state_at_time_delta(
        &self,
        progress_val: f64,
        delta_seconds: f64,
    ) -> Option<cesium_engine::math::trajectory::TransformState> {
        if let Some(flight) = self.flights.first() {
            let start_t = flight
                .property
                .start_time()
                .map(|t| t.seconds)
                .unwrap_or(0.0);
            let stop_t = flight
                .property
                .stop_time()
                .map(|t| t.seconds)
                .unwrap_or(1.0);
            let current_time_seconds = start_t + progress_val * (stop_t - start_t);
            let time =
                cesium_engine::time::SimulationTime::new(current_time_seconds + delta_seconds);

            let evaluator =
                cesium_engine::math::trajectory::TrajectoryEvaluator::new(&flight.property, 30.0);
            evaluator.evaluate(time)
        } else {
            None
        }
    }

    pub fn get_plane_state_at(
        &self,
        progress_val: f64,
    ) -> Option<cesium_engine::math::trajectory::TransformState> {
        let mut state = self.get_plane_state_at_time_delta(progress_val, 0.0);

        if let Some(ref mut s) = state {
            if progress_val > 0.999 {
                // Plane has arrived. Derive rotation robustly by looking exactly 1 second in the past.
                if let Some(prev_state) = self.get_plane_state_at_time_delta(progress_val, -1.0) {
                    s.rotation = prev_state.rotation;
                }
            }
        }

        state
    }

    pub fn get_sun_intensity_at(&self, progress_val: f64) -> Option<f64> {
        if let Some(flight) = self.flights.first() {
            let start_t = flight
                .property
                .start_time()
                .map(|t| t.seconds)
                .unwrap_or(0.0);
            let stop_t = flight
                .property
                .stop_time()
                .map(|t| t.seconds)
                .unwrap_or(1.0);
            let current_time_seconds = start_t + progress_val * (stop_t - start_t);
            let time = cesium_engine::time::SimulationTime::new(current_time_seconds);

            use cesium_engine::property::Property;
            flight.sun_intensity_property.evaluate(time)
        } else {
            None
        }
    }

    pub fn add_flight_path(&mut self, id: &str, json_content: String, is_secondary: bool) {
        self.pending_flights
            .push((id.to_string(), json_content, is_secondary));
    }
}

impl GlobeExtension for FlightTrackerApp {
    fn init(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        config: &wgpu::SurfaceConfiguration,
        camera_bind_group_layout: &wgpu::BindGroupLayout,
    ) {
        // Drain any commands that were sent before init
        if let Some(rx) = &self.command_rx {
            while let Ok(cmd) = rx.try_recv() {
                match cmd {
                    FlightCommand::LoadFlight {
                        id,
                        json,
                        is_secondary,
                    } => {
                        self.pending_flights.push((id, json, is_secondary));
                    }
                    FlightCommand::SetProgress(p) => {
                        *self.progress.lock().unwrap() = p.clamp(0.0, 1.0);
                    }
                    FlightCommand::SetSpeed(s) => {
                        self.play_speed = s;
                    }
                    FlightCommand::Play => self.is_playing = true,
                    FlightCommand::Pause => self.is_playing = false,
                }
            }
        }

        // Try loading the A350.glb model from the root directory
        match std::fs::read("A350.glb") {
            Ok(glb_bytes) => {
                match ModelRenderer::new(
                    device,
                    queue,
                    config,
                    camera_bind_group_layout,
                    &glb_bytes,
                ) {
                    Ok(renderer) => {
                        println!("A350.glb successfully loaded and renderer initialized!");
                        self.airplane_renderer = Some(renderer);
                    }
                    Err(e) => eprintln!("Failed to initialize ModelRenderer: {:?}", e),
                }
            }
            Err(e) => eprintln!("Failed to read A350.glb from disk: {:?}", e),
        }

        for (id, content, is_secondary) in self.pending_flights.drain(..) {
            if let Ok((property, sun_intensity_property)) = load_flight_data(&content) {
                use cesium_engine::property::Property;
                if let Some(start_time) = property.start_time() {
                    let reference_point = property.evaluate(start_time).unwrap_or(glam::DVec3::ZERO);
                    let builder = AdaptiveSubdivisionBuilder::new(1e-7); // High precision tolerance
                    let control_points = builder.build(&property, reference_point);
                    
                    println!("Flight path loaded: {} ({} control points)", id, control_points.len());
                    
                    let mut renderer = PolylineRenderer::new(device, config, camera_bind_group_layout);
                    // Upload geometry statically once
                    renderer.update_geometry(device, queue, &control_points);
                    
                    let mut poly_config = PolylineConfig {
                        color_end: [0.9, 0.9, 0.9, 1.0],
                        ..PolylineConfig::default()
                    };

                    if is_secondary {
                        poly_config.split_progress = 0.5;
                    }

                    self.flights.push(FlightEntity {
                        id,
                        renderer,
                        config: poly_config,
                        property,
                        sun_intensity_property,
                        reference_point,
                    });
                }
            }
        }
    }

    fn update(
        &mut self,
        _device: &wgpu::Device,
        _queue: &wgpu::Queue,
        _camera_pos_dvec3: DVec3,
        _frustum: &[(DVec3, f64); 6],
        camera: &mut cesium_engine::camera::camera::Camera,
        aspect_ratio: f32,
    ) {
        // Drain commands submitted via FlightHandle
        if let Some(rx) = &self.command_rx {
            while let Ok(cmd) = rx.try_recv() {
                match cmd {
                    FlightCommand::LoadFlight {
                        id,
                        json,
                        is_secondary,
                    } => {
                        self.pending_flights.push((id, json, is_secondary));
                    }
                    FlightCommand::SetProgress(p) => {
                        *self.progress.lock().unwrap() = p.clamp(0.0, 1.0);
                    }
                    FlightCommand::SetSpeed(s) => {
                        self.play_speed = s;
                    }
                    FlightCommand::Play => self.is_playing = true,
                    FlightCommand::Pause => self.is_playing = false,
                }
            }
        }

        let now = std::time::Instant::now();
        let dt = now.duration_since(self.last_update_time).as_secs_f64();
        self.last_update_time = now;

        if self.is_playing {
            let mut p = *self.progress.lock().unwrap();
            p += self.play_speed * dt;
            if p > 1.0 {
                p = 1.0;
                self.is_playing = false;
            } else if p < 0.0 {
                p = 0.0;
                self.is_playing = false;
            }
            *self.progress.lock().unwrap() = p;
        }

        let current_progress = *self.progress.lock().unwrap();

        if let Some(intensity) = self.get_sun_intensity_at(current_progress) {
            camera.sun_intensity = intensity as f32;
        }

        // Camera Mode two-way sync — must happen before the flight loop so that
        // mode_switched_or_reset is correct for the dirty-flag check.
        let mut mode_switched_or_reset = false;
        if self.view_mode != self.last_view_mode {
            camera.mode = self.view_mode;
            self.last_view_mode = self.view_mode;
            self.reset_viewport = true;
        } else if camera.mode != self.view_mode {
            self.view_mode = camera.mode;
            self.last_view_mode = camera.mode;
            self.reset_viewport = true;
        }
        if self.reset_viewport {
            mode_switched_or_reset = true;
            self.reset_viewport = false;
        }



        if let Some(state) = self.get_plane_state_at(current_progress) {
            match self.view_mode {
                CameraMode::Tracking => {
                    crate::camera_modes::tracking::update_tracking_mode(
                        camera,
                        &state,
                        mode_switched_or_reset,
                    );
                }
                CameraMode::Cockpit => {
                    crate::camera_modes::cockpit::update_cockpit_mode(
                        camera,
                        &state,
                        mode_switched_or_reset,
                    );
                }
                CameraMode::Free => {
                    crate::camera_modes::free::update_free_mode(
                        camera,
                        &self.flights,
                        aspect_ratio,
                        mode_switched_or_reset,
                    );
                }
            }
        } else if self.view_mode == CameraMode::Free {
            // Free mode does not require an active plane state
            crate::camera_modes::free::update_free_mode(
                camera,
                &self.flights,
                aspect_ratio,
                mode_switched_or_reset,
            );
        }
    }

    fn render<'res>(
        &'res self,
        render_pass: &mut wgpu::RenderPass<'res>,
        camera_bind_group: &'res wgpu::BindGroup,
        viewport_size: [f32; 2],
        camera_pos_f64: [f64; 3],
    ) {
        let current_progress = *self.progress.lock().unwrap();
        let airplane_state = self.get_plane_state_at(current_progress);

        for flight in &self.flights {
            let mut config = flight.config.clone();
            config.physical_half_width = 2.98 / 1_000_000.0;
            config.split_progress = current_progress as f32;

            // Compute airplane position relative to reference_point for world-space split.
            // We use the smoothed plane state to guarantee perfect alignment with the rendered model.
            let airplane_ecef: Option<glam::DVec3> = airplane_state.map(|s| s.position);
            config.airplane_pos = if let Some(ecef) = airplane_ecef {
                let rel = ecef - flight.reference_point;
                [rel.x as f32, rel.y as f32, rel.z as f32, 1.0_f32] // w=1 activates world-space split
            } else {
                [0.0, 0.0, 0.0, 0.0] // w=0 falls back to legacy progress comparison
            };

            config.airplane_forward = if let Some(state) = airplane_state {
                let cur_rot = state.rotation;
                let rot_f32 = glam::Quat::from_xyzw(
                    cur_rot.x as f32,
                    cur_rot.y as f32,
                    cur_rot.z as f32,
                    cur_rot.w as f32,
                )
                .normalize();
                let forward = rot_f32 * glam::Vec3::new(0.0, 0.0, -1.0);
                [forward.x, forward.y, forward.z, 0.0]
            } else {
                [0.0, 0.0, 0.0, 0.0]
            };

            let _cam_pos_dvec3 = glam::DVec3::from_slice(&camera_pos_f64);

            flight.renderer.draw(DrawParams {
                render_pass,
                camera_bind_group,
                viewport_size,
                camera_pos_f64,
                reference_point: [
                    flight.reference_point.x,
                    flight.reference_point.y,
                    flight.reference_point.z,
                ],
                config: &config,
            });
        }

        // Draw airplane
        if let Some(airplane) = &self.airplane_renderer {
            if let Some(state) = airplane_state {
                // Elevate 10m (0.00001 Megameters) to avoid clipping and align with ribbon elevation
                let up_dir = state.position.normalize();
                let elevated_position = state.position + up_dir * 0.00001;
                let camera_pos = glam::DVec3::from_slice(&camera_pos_f64);
                let relative_pos_f64 = elevated_position - camera_pos;
                let relative_pos = glam::Vec3::new(
                    relative_pos_f64.x as f32,
                    relative_pos_f64.y as f32,
                    relative_pos_f64.z as f32,
                );
                let translation = glam::Mat4::from_translation(relative_pos);

                let cur_rot = state.rotation;
                let rot_f32 = glam::Quat::from_xyzw(
                    cur_rot.x as f32,
                    cur_rot.y as f32,
                    cur_rot.z as f32,
                    cur_rot.w as f32,
                )
                .normalize();
                let rotation = glam::Mat4::from_quat(rot_f32);

                // Dynamic scaling based on camera distance
                let distance = relative_pos.length(); // Distance in Megameters

                // Desired length of the airplane in Megameters (now half as big as 0.0333)
                let desired_length_mm = distance * 0.01665;

                let min_length_mm = 67.0 / 1_000_000.0; // 67 meters (A350 length)
                let max_length_mm = 2000.0 * 1000.0 / 1_000_000.0; // 2000 km

                let clamped_length_mm = desired_length_mm.clamp(min_length_mm, max_length_mm);

                // Assuming the A350 model is approximately 1.0 local units long.
                let scale_factor = clamped_length_mm / 1.0;
                let scale = glam::Mat4::from_scale(glam::Vec3::splat(scale_factor));

                // Apply a constant yaw correction to align the A350.glb model with standard axes
                let model_correction = glam::Mat4::from_euler(
                    glam::EulerRot::YXZ,
                    std::f32::consts::PI, // Yaw
                    0.0,                  // Pitch
                    0.0,                  // Roll
                );

                let model_matrix = translation * rotation * scale * model_correction;

                use cesium_engine::render::model_pipeline::pipeline::ModelPushConstants;
                let push = ModelPushConstants {
                    model_matrix_0: model_matrix.x_axis.to_array(),
                    model_matrix_1: model_matrix.y_axis.to_array(),
                    model_matrix_2: model_matrix.z_axis.to_array(),
                    model_matrix_3: model_matrix.w_axis.to_array(),
                    camera_pos: [
                        camera_pos_f64[0] as f32,
                        camera_pos_f64[1] as f32,
                        camera_pos_f64[2] as f32,
                        1.0,
                    ],
                    viewport_size,
                    padding: [0.0, 0.0],
                };

                airplane.draw(render_pass, camera_bind_group, push);
            }
        }
    }

    fn render_ui(&mut self, _ctx: &egui::Context, ui: &mut egui::Ui) {
        ui.label("Flight Controls");
        let mut p = *self.progress.lock().unwrap() as f32;
        if ui
            .add(egui::Slider::new(&mut p, 0.0..=1.0).text("Flight Progress"))
            .changed()
        {
            *self.progress.lock().unwrap() = p as f64;
            self.is_playing = false; // Pause when manually dragged
        }

        ui.horizontal(|ui| {
            if ui
                .button(if self.is_playing { "Pause" } else { "Play" })
                .clicked()
            {
                self.is_playing = !self.is_playing;
            }
            ui.add(egui::Slider::new(&mut self.play_speed, -0.5..=0.5).text("Speed"));
        });

        ui.separator();
        ui.label("Camera Mode");
        ui.horizontal(|ui| {
            ui.radio_value(&mut self.view_mode, CameraMode::Free, "Free");
            ui.radio_value(&mut self.view_mode, CameraMode::Tracking, "Tracking");
            ui.radio_value(&mut self.view_mode, CameraMode::Cockpit, "Cockpit");

            if ui.button("Reset Viewport").clicked() {
                self.reset_viewport = true;
            }
        });
    }
}
