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

pub fn load_flight_path<P: AsRef<Path>>(path: P) -> Result<SampledPositionProperty, Box<dyn std::error::Error>> {
    let content = std::fs::read_to_string(path)?;
    let waypoints: Vec<serde_json::Value> = serde_json::from_str(&content)?;

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
    pub flights: Vec<FlightEntity>,
    pub airplane_renderer: Option<ModelRenderer>,
    pub start_time: std::time::Instant,
}

impl FlightTrackerApp {
    pub fn new() -> Self {
        Self {
            flights: Vec::new(),
            airplane_renderer: None,
            start_time: std::time::Instant::now(),
        }
    }

    pub fn set_airplane_model(&mut self, _glb_bytes: Vec<u8>) {
        // In the future, this can be called via JNI to dynamically set the model
        // To be implemented: store bytes and create ModelRenderer in update()
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

        if let Ok(entries) = std::fs::read_dir(".") {
            for entry in entries {
                if let Ok(entry) = entry {
                    let path = entry.path();
                    if let Some(filename) = path.file_name().and_then(|f| f.to_str()) {
                        if filename.starts_with("flight_") && filename.ends_with(".json") {
                            if let Ok(property) = load_flight_path(&path) {
                                if let Some(bvh) = PolylineBVH::build(&property) {
                                    println!("BVH loaded: {}", filename);
                                    let renderer = PolylineRenderer::new(device, config, camera_bind_group_layout);
                                    let mut poly_config = PolylineConfig::default();
                                    
                                    if filename.contains("STR") {
                                        poly_config.split_progress = 0.5;
                                        poly_config.color_end = [0.9, 0.9, 0.9, 1.0];
                                    }

                                    self.flights.push(FlightEntity {
                                        id: filename.to_string(),
                                        bvh,
                                        renderer,
                                        config: poly_config,
                                        property,
                                    });
                                }
                            }
                        }
                    }
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
    ) {
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
    }

    fn render<'a>(
        &'a self,
        render_pass: &mut wgpu::RenderPass<'a>,
        camera_bind_group: &'a wgpu::BindGroup,
        viewport_size: [f32; 2],
        camera_pos_f64: [f64; 3],
    ) {
        for flight in &self.flights {
            flight.renderer.draw(
                render_pass, 
                camera_bind_group, 
                viewport_size, 
                camera_pos_f64,
                &flight.config,
            );
        }

        // Draw airplane
        if let Some(airplane) = &self.airplane_renderer {
            if let Some(flight) = self.flights.first() {
                // Freeze the plane at the start for testing
                let time = SimulationTime::new(0.0);
                
                if let Some(pos) = flight.property.evaluate(time) {
                    let next_time = SimulationTime::new(time.seconds + 0.1);
                    if let Some(next_pos) = flight.property.evaluate(next_time) {
                        let pos_f32 = Vec3::new(pos.x as f32, pos.y as f32, pos.z as f32);
                        let next_pos_f32 = Vec3::new(next_pos.x as f32, next_pos.y as f32, next_pos.z as f32);
                        
                        let forward = (next_pos_f32 - pos_f32).normalize();
                        let up = pos_f32.normalize();
                        let right = forward.cross(up).normalize();
                        let adjusted_forward = up.cross(right).normalize();

                        let scale_factor = 1.0 / 6378137.0; // Base engine scale (meters to earth radii)
                        // If model is huge, maybe scale it down. But let's leave base scale.
                        // Wait, 1.0 engine unit = 6378137 meters. So if model is 67 meters, it becomes 67 * scale_factor = ~0.00001 engine units.
                        let scale = Mat4::from_scale(Vec3::splat(scale_factor));

                        // Pre-rotate model to fix Y-up/Z-forward mismatch (rotate -90 deg around local X)
                        let pre_rotation = Mat4::from_rotation_x(-std::f32::consts::FRAC_PI_2);

                        // Apply standard -Z forward orientation
                        let rotation = Mat4::from_cols(
                            right.extend(0.0),
                            up.extend(0.0),
                            (-adjusted_forward).extend(0.0),
                            Vec3::ZERO.extend(1.0),
                        );

                        // Position relative to camera using camera_pos_f64
                        let cam = Vec3::new(camera_pos_f64[0] as f32, camera_pos_f64[1] as f32, camera_pos_f64[2] as f32);
                        // Lift the plane significantly (100km) to ensure it doesn't clip into the ground
                        let lift = up * 0.1;
                        let relative_pos = (pos_f32 + lift) - cam;
                        
                        let translation = Mat4::from_translation(relative_pos);
                        let model_matrix = translation * rotation * pre_rotation * scale;

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
        }
    }
}
