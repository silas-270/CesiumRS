use std::path::Path;
use glam::{DVec3, DVec4};

use crate::engine::property::sampled::{SampledPositionProperty, InterpolationAlgorithm};
use crate::engine::time::SimulationTime;
use crate::engine::globe::geometry::lon_lat_alt_to_ecef_f64;
use crate::engine::render::polyline::bvh::PolylineBVH;
use crate::engine::render::polyline::pipeline::{PolylineRenderer, PolylineConfig};
use crate::engine::core::extension::GlobeExtension;

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
}

pub struct FlightTrackerApp {
    pub flights: Vec<FlightEntity>,
}

impl FlightTrackerApp {
    pub fn new() -> Self {
        Self {
            flights: Vec::new(),
        }
    }
}

impl GlobeExtension for FlightTrackerApp {
    fn init(
        &mut self,
        device: &wgpu::Device,
        config: &wgpu::SurfaceConfiguration,
        camera_bind_group_layout: &wgpu::BindGroupLayout,
    ) {
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
                                    });
                                } else {
                                    println!("Failed to build BVH for: {}", filename);
                                }
                            } else {
                                println!("Failed to load flight path: {}", filename);
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
                    // Insert degenerate vertices to break the triangle strip
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
    }
}
