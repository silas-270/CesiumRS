use glam::{DVec3, DQuat};
use cesium_engine::time::SimulationTime;
use cesium_engine::property::Property;
use cesium_engine::property::sampled::{SampledPositionProperty, InterpolationAlgorithm};
use cesium_engine::math::trajectory::TrajectoryEvaluator;

#[test]
fn test_plane_tangent_alignment() {
    let mut prop = SampledPositionProperty::new().with_algorithm(InterpolationAlgorithm::CatmullRom);
    
    // Add points that simulate a curved, climbing flight path to ensure pitch is non-zero
    let start_pos = cesium_engine::globe::geometry::lon_lat_alt_to_ecef_f64(-74.0, 40.0, 0.0);
    let mid_pos = cesium_engine::globe::geometry::lon_lat_alt_to_ecef_f64(-50.0, 45.0, 10000.0);
    let end_pos = cesium_engine::globe::geometry::lon_lat_alt_to_ecef_f64(-10.0, 50.0, 10000.0);

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


use cesium_engine::render::polyline_pipeline::bvh::{generate_vertices, PolylineBVH};

#[test]
fn test_split_delta() {
    let mut prop = SampledPositionProperty::new().with_algorithm(InterpolationAlgorithm::CatmullRom);
    
    let start_pos = cesium_engine::globe::geometry::lon_lat_alt_to_ecef_f64(-74.0, 40.0, 0.0);
    let mid_pos = cesium_engine::globe::geometry::lon_lat_alt_to_ecef_f64(-50.0, 45.0, 10000.0);
    let end_pos = cesium_engine::globe::geometry::lon_lat_alt_to_ecef_f64(-10.0, 50.0, 10000.0);

    prop.add_sample(SimulationTime::new(0.0), DVec3::from_array(start_pos));
    prop.add_sample(SimulationTime::new(5000.0), DVec3::from_array(mid_pos));
    prop.add_sample(SimulationTime::new(10000.0), DVec3::from_array(end_pos));

    let eval = TrajectoryEvaluator::new(&prop, 2.0);
    let _bvh = PolylineBVH::build(&prop).unwrap();

    let check_at_time = |t: f64| {
        let time = SimulationTime::new(t);
        let state = eval.evaluate(time).unwrap();
        let airplane_pos = state.position;

        // Find the visible segment that contains airplane_pos
        // For simplicity in test, just get the whole curve points
        let mut points = Vec::new();
        for i in 0..=100 {
            let pt_time = SimulationTime::new(t - 100.0 + i as f64 * 2.0).seconds.max(0.0).min(10000.0);
            if let Some(pos) = prop.evaluate(SimulationTime::new(pt_time)) {
                points.push((pos, pt_time as f32 / 10000.0));
            }
        }
        
        let reference_point = points[0].0;
        let verts = generate_vertices(&points, airplane_pos, reference_point);

        // Find the split point on the ribbon spine
        // Spine is the average of left and right vertices.
        // Actually, let's just find the closest point on the spline to airplane_pos.
        // The delta is how far the 0-crossing on the spine is from the airplane's projected position on the spine.
        // Wait, the shader evaluates proj = dot(world_pos - airplane_pos, airplane_forward).
        // Let's find two consecutive vertices on the spine where proj crosses 0.
        // A spine vertex world_pos = model.position + elevation_offset.
        
        let mut min_delta = f64::MAX;

        for i in 0..verts.len()-1 {
            let v1 = &verts[i];
            let v2 = &verts[i+1];
            
            // Only consider top face spine (side=0 roughly, or just average)
            // Just use the 'position' which is the center of the ribbon!
            let pos1 = DVec3::new(v1.position[0] as f64, v1.position[1] as f64, v1.position[2] as f64) + reference_point;
            let pos2 = DVec3::new(v2.position[0] as f64, v2.position[1] as f64, v2.position[2] as f64) + reference_point;
            
            let elevation_offset1 = pos1.normalize() * 0.000005;
            let elevation_offset2 = pos2.normalize() * 0.000005;
            
            let wp1 = pos1 + elevation_offset1;
            let wp2 = pos2 + elevation_offset2;
            
            let to_frag1 = wp1 - airplane_pos;
            let to_frag2 = wp2 - airplane_pos;
            
            let pos_curr1 = DVec3::new(v1.position[0] as f64, v1.position[1] as f64, v1.position[2] as f64);
            let pos_prev1 = DVec3::new(v1.previous[0] as f64, v1.previous[1] as f64, v1.previous[2] as f64);
            let pos_next1 = DVec3::new(v1.next[0] as f64, v1.next[1] as f64, v1.next[2] as f64);
            let dir_prev1 = (pos_curr1 - pos_prev1).normalize_or_zero();
            let dir_next1 = (pos_next1 - pos_curr1).normalize_or_zero();
            let tangent1 = if dir_next1.length_squared() < 1e-6 {
                dir_prev1
            } else if dir_prev1.length_squared() > 1e-6 {
                (dir_prev1 + dir_next1).normalize()
            } else {
                dir_next1
            };
            let proj1 = to_frag1.dot(tangent1);
            
            let pos_curr2 = DVec3::new(v2.position[0] as f64, v2.position[1] as f64, v2.position[2] as f64);
            let pos_prev2 = DVec3::new(v2.previous[0] as f64, v2.previous[1] as f64, v2.previous[2] as f64);
            let pos_next2 = DVec3::new(v2.next[0] as f64, v2.next[1] as f64, v2.next[2] as f64);
            let dir_prev2 = (pos_curr2 - pos_prev2).normalize_or_zero();
            let dir_next2 = (pos_next2 - pos_curr2).normalize_or_zero();
            let tangent2 = if dir_next2.length_squared() < 1e-6 {
                dir_prev2
            } else if dir_prev2.length_squared() > 1e-6 {
                (dir_prev2 + dir_next2).normalize()
            } else {
                dir_next2
            };
            let proj2 = to_frag2.dot(tangent2);
            
            if proj1 * proj2 <= 0.0 && (proj1 - proj2).abs() > 1e-10 {
                // Crossing found
                let t_cross = proj1 / (proj1 - proj2);
                let cross_wp = wp1.lerp(wp2, t_cross);
                
                // Distance from airplane to the crossing point on the spine
                let delta = cross_wp.distance(airplane_pos);
                if delta < min_delta {
                    min_delta = delta;
                }
            }
        }
        min_delta
    };

    let delta_start = check_at_time(0.0);
    let delta_mid = check_at_time(5000.0);
    let delta_end = check_at_time(10000.0);

    println!("Delta Start: {} Mm", delta_start);
    println!("Delta Mid: {} Mm", delta_mid);
    println!("Delta End: {} Mm", delta_end);
    
    assert!(delta_start < 0.00005, "Delta at start too large: {}", delta_start);
    assert!(delta_end < 0.00005, "Delta at end too large: {}", delta_end);
}
