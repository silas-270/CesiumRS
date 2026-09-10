use cesium_flight::preset::PRESETS;
use cesium_flight::telemetry::generate;
use cesium_engine::globe::geometry::lon_lat_alt_to_ecef_f64;
use cesium_engine::property::sampled::{InterpolationAlgorithm, SampledPositionProperty};
use cesium_engine::time::SimulationTime;
use cesium_engine::math::trajectory::TrajectoryEvaluator;
use glam::DVec3;

#[test]
fn audit_user_experience_metrics() {
    let preset = PRESETS.iter().find(|p| p.id == "SIN-LHR").expect("SIN-LHR preset found");
    let req = preset.to_route_def();
    
    let flight_req = cesium_flight::telemetry::FlightRequest {
        departure: cesium_flight::telemetry::geo::LatLon::new(req.departure_lat, req.departure_lon),
        arrival: cesium_flight::telemetry::geo::LatLon::new(req.arrival_lat, req.arrival_lon),
        target_duration_ms: req.total_duration_ms,
        dep_heading_deg: req.dep_heading_deg,
        arr_heading_deg: req.arr_heading_deg,
        runways: Vec::new(),
        config: cesium_flight::telemetry::FlightPlanConfig::default(),
    };

    let points = generate(&flight_req);
    let mut property = SampledPositionProperty::new().with_algorithm(InterpolationAlgorithm::CatmullRom);
    for pt in &points {
        let ecef_array = lon_lat_alt_to_ecef_f64(pt.longitude, pt.latitude, pt.altitude);
        let pos = DVec3::from_array(ecef_array);
        let time = SimulationTime::new(pt.time_offset_ms as f64 / 1000.0);
        property.add_sample(time, pos);
    }

    let start_t = property.start_time().unwrap().seconds;
    let stop_t = property.stop_time().unwrap().seconds;
    let total_time = stop_t - start_t;

    let eval = TrajectoryEvaluator::new(&property, 30.0);

    let n_evals = 15000;
    let mut prev_rot: Option<glam::DQuat> = None;
    let mut prev_ang_vel = 0.0_f64;

    struct Anomaly {
        t: f64,
        ang_vel: f64,
        ang_acc: f64,
    }
    let mut high_jerk: Vec<Anomaly> = Vec::new();

    for i in 0..=n_evals {
        let frac = i as f64 / n_evals as f64;
        let t = start_t + frac * total_time;
        if let Some(state) = eval.evaluate(SimulationTime::new(t)) {
            if let Some(p_rot) = prev_rot {
                let dt = total_time / n_evals as f64;
                let dot = (state.rotation.dot(p_rot)).abs().clamp(-1.0, 1.0);
                let angle_rad = 2.0 * dot.acos();
                let ang_vel_deg_s = angle_rad.to_degrees() / dt;
                let ang_acc_deg_s2 = (ang_vel_deg_s - prev_ang_vel).abs() / dt;

                if ang_acc_deg_s2 > 0.5 {
                    high_jerk.push(Anomaly {
                        t,
                        ang_vel: ang_vel_deg_s,
                        ang_acc: ang_acc_deg_s2,
                    });
                }
                prev_ang_vel = ang_vel_deg_s;
            }
            prev_rot = Some(state.rotation);
        }
    }

    println!("=== TOTAL DETECTED ANGULAR JERK ANOMALIES (> 0.5 deg/s^2): {} ===", high_jerk.len());
    for a in &high_jerk {
        println!("t = {:.1} s ({:.2} h) | ang_vel = {:.3} deg/s | ang_acc = {:.4} deg/s^2", a.t, a.t/3600.0, a.ang_vel, a.ang_acc);
    }
}
