use glam::{DVec3, DQuat, Vec3, Mat4, Quat};
use crate::engine::time::SimulationTime;
use crate::engine::property::Property;
use crate::engine::property::sampled::{SampledPositionProperty, InterpolationAlgorithm};
use crate::engine::math::trajectory::TrajectoryEvaluator;
use crate::engine::render::polyline::bvh::{generate_vertices, PolylineBVH};

#[test]
fn test_split_delta() {
    let mut prop = SampledPositionProperty::new().with_algorithm(InterpolationAlgorithm::CatmullRom);
    
    let start_pos = crate::engine::globe::geometry::lon_lat_alt_to_ecef_f64(-74.0, 40.0, 0.0);
    let mid_pos = crate::engine::globe::geometry::lon_lat_alt_to_ecef_f64(-50.0, 45.0, 10000.0);
    let end_pos = crate::engine::globe::geometry::lon_lat_alt_to_ecef_f64(-10.0, 50.0, 10000.0);

    prop.add_sample(SimulationTime::new(0.0), DVec3::from_array(start_pos));
    prop.add_sample(SimulationTime::new(5000.0), DVec3::from_array(mid_pos));
    prop.add_sample(SimulationTime::new(10000.0), DVec3::from_array(end_pos));

    let eval = TrajectoryEvaluator::new(&prop, 2.0);
    let bvh = PolylineBVH::build(&prop).unwrap();

    let check_at_time = |t: f64| {
        let time = SimulationTime::new(t);
        let state = eval.evaluate(time).unwrap();
        let airplane_pos = state.position;
        let rot_f32 = glam::Quat::from_xyzw(state.rotation.x as f32, state.rotation.y as f32, state.rotation.z as f32, state.rotation.w as f32).normalize();
        let airplane_forward = rot_f32 * glam::Vec3::new(0.0, 0.0, -1.0);

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
            
            let proj1 = to_frag1.dot(DVec3::new(airplane_forward.x as f64, airplane_forward.y as f64, airplane_forward.z as f64));
            let proj2 = to_frag2.dot(DVec3::new(airplane_forward.x as f64, airplane_forward.y as f64, airplane_forward.z as f64));
            
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
