use glam::DVec3;
use cesium_engine::time::{Clock, SimulationTime};
use cesium_engine::property::Property;
use cesium_engine::property::sampled::{SampledPositionProperty, InterpolationAlgorithm};
use cesium_engine::math::interpolation;
use cesium_engine::math::transform;

#[test]
fn test_clock_tick() {
    let mut clock = Clock::new(SimulationTime::new(0.0), SimulationTime::new(10.0));
    clock.multiplier = 2.0;

    clock.tick(1.0);
    assert_eq!(clock.current_time.seconds, 2.0);

    clock.tick(5.0); // +10.0 => 12.0
    assert_eq!(clock.current_time.seconds, 10.0);
    assert_eq!(clock.is_playing, false);
}

#[test]
fn test_interpolation_linear() {
    let p0 = DVec3::new(0.0, 0.0, 0.0);
    let p1 = DVec3::new(10.0, 20.0, 30.0);

    let mid = interpolation::linear_dvec3(p0, p1, 0.5);
    assert_eq!(mid, DVec3::new(5.0, 10.0, 15.0));
}

#[test]
fn test_sampled_position_property() {
    let mut prop = SampledPositionProperty::new().with_algorithm(InterpolationAlgorithm::Linear);

    prop.add_sample(SimulationTime::new(0.0), DVec3::new(0.0, 0.0, 0.0));
    prop.add_sample(SimulationTime::new(10.0), DVec3::new(10.0, 0.0, 0.0));
    prop.add_sample(SimulationTime::new(20.0), DVec3::new(10.0, 10.0, 0.0));

    // Exact samples
    assert_eq!(prop.evaluate(SimulationTime::new(0.0)).unwrap(), DVec3::new(0.0, 0.0, 0.0));
    assert_eq!(prop.evaluate(SimulationTime::new(10.0)).unwrap(), DVec3::new(10.0, 0.0, 0.0));

    // Midpoints
    assert_eq!(prop.evaluate(SimulationTime::new(5.0)).unwrap(), DVec3::new(5.0, 0.0, 0.0));
    assert_eq!(prop.evaluate(SimulationTime::new(15.0)).unwrap(), DVec3::new(10.0, 5.0, 0.0));

    // Out of bounds
    assert_eq!(prop.evaluate(SimulationTime::new(-5.0)).unwrap(), DVec3::new(0.0, 0.0, 0.0));
    assert_eq!(prop.evaluate(SimulationTime::new(25.0)).unwrap(), DVec3::new(10.0, 10.0, 0.0));
}

#[test]
fn test_enu_and_velocity() {
    // Top of the earth (North Pole)
    let ecef = cesium_engine::globe::geometry::lon_lat_to_ecef_f64(0.0, 90.0);
    let up = transform::surface_normal_ecef(DVec3::from_array(ecef));
    
    // Normal should point mostly along +Y (WGS84 orientation in this engine)
    assert!(up.y > 0.99);

    let enu = transform::enu_matrix_at_ecef(DVec3::from_array(ecef));
    // Up column (z_axis) should match up vector
    assert_eq!(enu.z_axis, up);
}

#[test]
fn test_adaptive_subdivision_stress() {
    use std::time::Instant;
    let mut prop = SampledPositionProperty::new().with_algorithm(InterpolationAlgorithm::CatmullRom);

    // Create a very long flight: New York to Singapore
    // Approximate coordinates
    let ny_lon = -74.0060;
    let ny_lat = 40.7128;
    let sin_lon = 103.8198;
    let sin_lat = 1.3521;

    prop.add_sample(SimulationTime::new(0.0), DVec3::from_array(cesium_engine::globe::geometry::lon_lat_alt_to_ecef_f64(ny_lon, ny_lat, 10000.0)));
    // Add midpoint to help the spline (flying over north pole roughly)
    prop.add_sample(SimulationTime::new(3600.0 * 8.0), DVec3::from_array(cesium_engine::globe::geometry::lon_lat_alt_to_ecef_f64(14.0, 85.0, 10000.0)));
    prop.add_sample(SimulationTime::new(3600.0 * 16.0), DVec3::from_array(cesium_engine::globe::geometry::lon_lat_alt_to_ecef_f64(sin_lon, sin_lat, 10000.0)));

    let builder = cesium_engine::render::polyline_pipeline::builder::AdaptiveSubdivisionBuilder::new(5.0);
    
    let start_time = Instant::now();
    let vertices = builder.build(&prop);
    let duration = start_time.elapsed();

    println!("Adaptive Subdivision Stress Test Results:");
    println!("Tolerance: {} meters", builder.tolerance);
    println!("Generated Vertices: {}", vertices.len());
    println!("Time Taken: {:?}", duration);

    assert!(vertices.len() > 10);
}
