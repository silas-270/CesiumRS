use std::path::Path;
use glam::DVec3;
use crate::engine::property::sampled::{SampledPositionProperty, InterpolationAlgorithm};
use crate::engine::time::SimulationTime;
use crate::engine::globe::geometry::lon_lat_alt_to_ecef_f64;

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

use crate::engine::render::polyline::bvh::PolylineBVH;
use crate::engine::render::polyline::pipeline::{PolylineRenderer, PolylineConfig};

pub struct FlightEntity {
    pub id: String,
    pub bvh: PolylineBVH,
    pub renderer: PolylineRenderer,
    pub config: PolylineConfig,
}
