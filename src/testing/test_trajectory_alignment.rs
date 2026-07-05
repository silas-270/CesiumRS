use glam::{DVec3, DQuat};
use crate::engine::time::SimulationTime;
use crate::engine::property::Property;
use crate::engine::property::sampled::{SampledPositionProperty, InterpolationAlgorithm};
use crate::engine::math::trajectory::TrajectoryEvaluator;

#[test]
fn test_plane_tangent_alignment() {
    let mut prop = SampledPositionProperty::new().with_algorithm(InterpolationAlgorithm::CatmullRom);
    
    // Add points that simulate a curved, climbing flight path to ensure pitch is non-zero
    let start_pos = crate::engine::globe::geometry::lon_lat_alt_to_ecef_f64(-74.0, 40.0, 0.0);
    let mid_pos = crate::engine::globe::geometry::lon_lat_alt_to_ecef_f64(-50.0, 45.0, 10000.0);
    let end_pos = crate::engine::globe::geometry::lon_lat_alt_to_ecef_f64(-10.0, 50.0, 10000.0);

    prop.add_sample(SimulationTime::new(0.0), DVec3::from_array(start_pos));
    prop.add_sample(SimulationTime::new(5000.0), DVec3::from_array(mid_pos));
    prop.add_sample(SimulationTime::new(10000.0), DVec3::from_array(end_pos));

    let eval = TrajectoryEvaluator::new(&prop, 2.0);
    
    // Check at an arbitrary point in the middle of the curve
    let time = SimulationTime::new(2500.0);
    
    // 1. Calculate true tangent of the polyline
    let dt = 0.001;
    let pos_next = prop.evaluate(SimulationTime::new(time.seconds + dt)).unwrap();
    let pos_prev = prop.evaluate(SimulationTime::new(time.seconds - dt)).unwrap();
    let true_tangent = (pos_next - pos_prev).normalize();
    
    // 2. Get the plane's forward vector
    let state = eval.evaluate(time).unwrap();
    let local_forward = DVec3::new(0.0, 0.0, -1.0);
    let plane_forward = state.rotation * local_forward;
    
    // Check if the true tangent matches evaluator forward (-Z)
    let distance = true_tangent.distance(plane_forward);
    
    assert!(distance < 1e-4, "Plane vector is not perfectly aligned with polyline tangent!");
}


