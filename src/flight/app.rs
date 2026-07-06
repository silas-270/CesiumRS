use std::path::Path;
use glam::{DVec3, Vec3, Mat4};

use crate::engine::property::sampled::{SampledPositionProperty, InterpolationAlgorithm};
use crate::engine::property::Property;
use crate::engine::time::SimulationTime;
use crate::engine::globe::geometry::lon_lat_alt_to_ecef_f64;
use crate::engine::render::polyline::bvh::PolylineBVH;
use crate::engine::render::polyline::pipeline::{PolylineRenderer, PolylineConfig};
use crate::engine::core::extension::GlobeExtension;
use crate::engine::render::model::pipeline::ModelRenderer;
use crate::engine::camera::camera::CameraMode;

pub fn load_flight_data(content: &str) -> Result<SampledPositionProperty, Box<dyn std::error::Error>> {
    let waypoints: Vec<serde_json::Value> = serde_json::from_str(content)?;

    let mut property = SampledPositionProperty::new()
        .with_algorithm(InterpolationAlgorithm::CatmullRom);

    for wp in waypoints {
        let time_offset_ms = wp["timeOffsetMs"].as_u64().unwrap_or(0);
        let longitude = wp["longitude"].as_f64().unwrap_or(0.0);
        let latitude = wp["latitude"].as_f64().unwrap_or(0.0);
        let altitude = wp["altitude"].as_f64().unwrap_or(0.0);

        let ecef_array = lon_lat_alt_to_ecef_f64(longitude, latitude, altitude);
        let position = DVec3::from_array(ecef_array);
        let time = SimulationTime::new(time_offset_ms as f64 / 1000.0);
        property.add_sample(time, position);
    }

    Ok(property)
}

pub struct FlightEntity {
    pub id: String,
    pub bvh: PolylineBVH,
    pub renderer: PolylineRenderer,
    pub config: PolylineConfig,
    pub property: SampledPositionProperty,
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
}

impl FlightTrackerApp {
    pub fn new(progress: std::sync::Arc<std::sync::Mutex<f64>>) -> Self {
        Self {
            progress,
            pending_flights: Vec::new(),
            flights: Vec::new(),
            airplane_renderer: None,
            last_update_time: std::time::Instant::now(),
            is_playing: false,
            play_speed: 0.1,
            view_mode: crate::engine::camera::camera::CameraMode::Free,
            last_view_mode: crate::engine::camera::camera::CameraMode::Free,
            reset_viewport: true,
        }
    }

    pub fn get_plane_state_at_time_delta(&self, progress_val: f64, delta_seconds: f64) -> Option<crate::engine::math::trajectory::TransformState> {
        if let Some(flight) = self.flights.first() {
            let start_t = flight.property.start_time().map(|t| t.seconds).unwrap_or(0.0);
            let stop_t = flight.property.stop_time().map(|t| t.seconds).unwrap_or(1.0);
            let current_time_seconds = start_t + progress_val * (stop_t - start_t);
            let time = crate::engine::time::SimulationTime::new(current_time_seconds + delta_seconds);
            
            let evaluator = crate::engine::math::trajectory::TrajectoryEvaluator::new(&flight.property, 30.0);
            evaluator.evaluate(time)
        } else {
            None
        }
    }

    pub fn get_plane_state_at_progress_delta(&self, progress_val: f64, delta_progress: f64) -> Option<crate::engine::math::trajectory::TransformState> {
        self.get_plane_state_at_time_delta(progress_val + delta_progress, 0.0)
    }

    pub fn get_plane_state_at(&self, progress_val: f64) -> Option<crate::engine::math::trajectory::TransformState> {
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

    pub fn add_flight_path(&mut self, id: &str, json_content: String, is_secondary: bool) {
        self.pending_flights.push((id.to_string(), json_content, is_secondary));
    }

    pub fn set_airplane_model(&mut self, _glb_bytes: Vec<u8>) {
        // In the future, this can be called via JNI to dynamically set the model
        // To be implemented: store bytes and create ModelRenderer in update()
    }

    fn reset_free_mode_camera(&self, camera: &mut crate::engine::camera::camera::Camera) {
        if let Some(flight) = self.flights.first() {
            let samples = flight.property.samples();
            if samples.len() >= 2 {
                let s = samples.first().unwrap().1;
                let e = samples.last().unwrap().1;
                
                let s_f32 = glam::Vec3::new(s.x as f32, s.y as f32, s.z as f32);
                let e_f32 = glam::Vec3::new(e.x as f32, e.y as f32, e.z as f32);
                
                let m = (s_f32 + e_f32) * 0.5;
                let n = m.normalize();
                
                let v = e_f32 - s_f32;
                let v_tangent = v - v.dot(n) * n;
                let d_vec = v_tangent.normalize_or_zero();
                
                // Rotate left by 45 degrees to find "Up" such that the flight path points top-right
                let up = glam::Quat::from_axis_angle(n, std::f32::consts::PI / 4.0) * d_vec;
                
                let l = v_tangent.length();
                let fov_y = 2.0 * (12.0 / camera.focal_length).atan();
                // 1.5 multiplier for padding
                let distance = (l * 0.707 * 1.5) / (fov_y / 2.0).tan();
                
                let center_on_earth = n * 6.378137;
                let final_cam_pos = center_on_earth + n * distance;
                
                let forward = -n;
                let right = forward.cross(up).normalize();
                let actual_up = right.cross(forward).normalize();
                let rot_mat = glam::Mat3::from_cols(right, actual_up, -forward);
                
                let q = glam::Quat::from_mat3(&rot_mat);
                camera.set_anchor(glam::Vec3::ZERO, glam::Quat::IDENTITY);
                camera.set_local_transform(final_cam_pos, q);
            }
        } else {
            // Default if no flights
            camera.set_anchor(glam::Vec3::ZERO, glam::Quat::IDENTITY);
            camera.set_local_transform(glam::Vec3::new(0.0, 0.0, 20.0), glam::Quat::IDENTITY);
        }
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
        // Try loading the A350.glb model from the root directory
        match std::fs::read("A350.glb") {
            Ok(glb_bytes) => {
                match ModelRenderer::new(device, queue, config, camera_bind_group_layout, &glb_bytes) {
                    Ok(renderer) => {
                        println!("A350.glb successfully loaded and renderer initialized!");
                        self.airplane_renderer = Some(renderer);
                    },
                    Err(e) => eprintln!("Failed to initialize ModelRenderer: {:?}", e),
                }
            },
            Err(e) => eprintln!("Failed to read A350.glb from disk: {:?}", e),
        }

        for (id, content, is_secondary) in self.pending_flights.drain(..) {
            if let Ok(property) = load_flight_data(&content) {
                if let Some(bvh) = PolylineBVH::build(&property) {
                    println!("BVH loaded: {}", id);
                    let renderer = PolylineRenderer::new(device, config, camera_bind_group_layout);
                    let mut poly_config = PolylineConfig::default();
                    poly_config.color_end = [0.9, 0.9, 0.9, 1.0];
                    
                    if is_secondary {
                        poly_config.split_progress = 0.5;
                    }

                    self.flights.push(FlightEntity {
                        id,
                        bvh,
                        renderer,
                        config: poly_config,
                        property,
                    });
                }
            }
        }
    }

    fn update(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        camera_pos_dvec3: DVec3,
        frustum: &[(DVec3, f64); 6],
        camera: &mut crate::engine::camera::camera::Camera,
    ) {
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

        for flight in &mut self.flights {
            let mut vertices = Vec::new();
            
            let visible_strips = flight.bvh.collect_visible_segments(camera_pos_dvec3, frustum, 5e-8);
            for strip in visible_strips {
                let mut strip_verts = crate::engine::render::polyline::bvh::generate_vertices(&strip, camera_pos_dvec3);
                if !vertices.is_empty() && !strip_verts.is_empty() {
                    vertices.push(*vertices.last().unwrap());
                    vertices.push(*strip_verts.first().unwrap());
                }
                vertices.append(&mut strip_verts);
            }
            flight.renderer.update_geometry(device, queue, &vertices);
        }

        // Camera Logic
        camera.mode = self.view_mode;
        
        let mut mode_switched_or_reset = false;
        if self.view_mode != self.last_view_mode || self.reset_viewport {
            mode_switched_or_reset = true;
            self.last_view_mode = self.view_mode;
            self.reset_viewport = false;
        }

        if let Some(state) = self.get_plane_state_at(*self.progress.lock().unwrap()) {
            let pos_f32 = glam::Vec3::new(state.position.x as f32, state.position.y as f32, state.position.z as f32);
            
            match self.view_mode {
                CameraMode::Tracking => {
                    // Orbit around the plane, but without banking. 
                    // We extract the forward vector and use velocity_to_orientation.
                    let forward = state.rotation * glam::DVec3::new(0.0, 0.0, -1.0);
                    let no_bank_quat = crate::engine::math::transform::velocity_to_orientation(state.position, forward);
                    let rot_f32 = glam::Quat::from_xyzw(no_bank_quat.x as f32, no_bank_quat.y as f32, no_bank_quat.z as f32, no_bank_quat.w as f32).normalize();
                    
                    camera.set_anchor(pos_f32, rot_f32);
                    
                    if mode_switched_or_reset {
                        // 50m behind, 15m up
                        camera.local_pos = glam::Vec3::new(0.0, 15.0 / 1_000_000.0, 50.0 / 1_000_000.0);
                        // Pitch down slightly to look at plane
                        camera.local_ori = glam::Quat::from_euler(glam::EulerRot::YXZ, 0.0, -0.25, 0.0);
                    }
                }
                CameraMode::Cockpit => {
                    // Cockpit is tied to the plane's exact rotation (banks and pitches with it).
                    let cur_rot = state.rotation;
                    let rot_f32 = glam::Quat::from_xyzw(cur_rot.x as f32, cur_rot.y as f32, cur_rot.z as f32, cur_rot.w as f32).normalize();
                    
                    camera.set_anchor(pos_f32, rot_f32);
                    
                    if mode_switched_or_reset {
                        camera.local_pos = glam::Vec3::new(0.0, 2.0 / 1_000_000.0, -32.0 / 1_000_000.0);
                        camera.local_ori = glam::Quat::IDENTITY;
                    }
                }
                CameraMode::Free => {
                    if mode_switched_or_reset {
                        self.reset_free_mode_camera(camera);
                    }
                }
            }
        }
    }

    fn render<'a>(
        &'a self,
        render_pass: &mut wgpu::RenderPass<'a>,
        camera_bind_group: &'a wgpu::BindGroup,
        viewport_size: [f32; 2],
        camera_pos_f64: [f64; 3],
    ) {
        let current_progress = *self.progress.lock().unwrap();

        for flight in &self.flights {
            let mut config = flight.config.clone();
            // A350 fuselage width is ~5.96 meters. Base half-width is 2.98 meters.
            // This is scaled dynamically inside the polyline.wgsl shader.
            config.physical_half_width = 2.98 / 1_000_000.0;
            
            // Make the split exactly where the plane is
            config.split_progress = current_progress as f32;

            flight.renderer.draw(
                render_pass, 
                camera_bind_group, 
                viewport_size, 
                camera_pos_f64,
                &config,
            );
        }

        // Draw airplane
        if let Some(airplane) = &self.airplane_renderer {
            if let Some(state) = self.get_plane_state_at(current_progress) {
                // Elevate 10m (0.00001 Megameters) to avoid clipping and align with ribbon elevation
                let up_dir = state.position.normalize();
                let elevated_position = state.position + up_dir * 0.00001;
                let pos_f32 = glam::Vec3::new(elevated_position.x as f32, elevated_position.y as f32, elevated_position.z as f32);
                let cam = glam::Vec3::new(camera_pos_f64[0] as f32, camera_pos_f64[1] as f32, camera_pos_f64[2] as f32);
                let relative_pos = pos_f32 - cam;
                let translation = glam::Mat4::from_translation(relative_pos);

                let cur_rot = state.rotation;
                let rot_f32 = glam::Quat::from_xyzw(cur_rot.x as f32, cur_rot.y as f32, cur_rot.z as f32, cur_rot.w as f32).normalize();
                let rotation = glam::Mat4::from_quat(rot_f32);
                
                // Dynamic scaling based on camera distance
                let distance = relative_pos.length(); // Distance in Megameters
                
                // Desired length of the airplane in Megameters (now half as big as 0.0333)
                let desired_length_mm = distance * 0.01665;
                
                let min_length_mm = 67.0 / 1_000_000.0;      // 67 meters (A350 length)
                let max_length_mm = 2000.0 * 1000.0 / 1_000_000.0; // 2000 km
                
                let clamped_length_mm = desired_length_mm.clamp(min_length_mm, max_length_mm);
                
                // Assuming the A350 model is approximately 1.0 local units long.
                let scale_factor = clamped_length_mm / 1.0; 
                let scale = glam::Mat4::from_scale(glam::Vec3::splat(scale_factor));

                // Apply a constant yaw correction to align the A350.glb model with standard axes
                let model_correction = glam::Mat4::from_euler(
                    glam::EulerRot::YXZ, 
                    std::f32::consts::PI,        // Yaw
                    0.0,                         // Pitch
                    0.0                          // Roll
                );

                let model_matrix = translation * rotation * scale * model_correction;

                use crate::engine::render::model::pipeline::ModelPushConstants;
                let push = ModelPushConstants {
                    model_matrix_0: model_matrix.x_axis.to_array(),
                    model_matrix_1: model_matrix.y_axis.to_array(),
                    model_matrix_2: model_matrix.z_axis.to_array(),
                    model_matrix_3: model_matrix.w_axis.to_array(),
                    camera_pos: [camera_pos_f64[0] as f32, camera_pos_f64[1] as f32, camera_pos_f64[2] as f32, 1.0],
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
        if ui.add(egui::Slider::new(&mut p, 0.0..=1.0).text("Flight Progress")).changed() {
            *self.progress.lock().unwrap() = p as f64;
            self.is_playing = false; // Pause when manually dragged
        }
        
        ui.horizontal(|ui| {
            if ui.button(if self.is_playing { "Pause" } else { "Play" }).clicked() {
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
