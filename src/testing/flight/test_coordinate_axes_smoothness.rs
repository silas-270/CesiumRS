use std::fs::File;
use std::io::Write;
use cesium_flight::preset::PRESETS;
use cesium_flight::telemetry::generate;
use cesium_engine::globe::geometry::lon_lat_alt_to_ecef_f64;
use cesium_engine::property::sampled::{InterpolationAlgorithm, SampledPositionProperty};
use cesium_engine::property::Property;
use cesium_engine::time::SimulationTime;
use glam::DVec3;

#[test]
fn audit_spatial_coordinates_xyz_enu() {
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

    let n_evals = 20000;
    let dt = 0.1; // 100 ms finite difference

    let csv_path = "/home/silas270/.gemini/antigravity-cli/brain/2de15eb8-181a-4d3f-9ca5-d350328b0865/scratch/axes_derivatives.csv";
    let mut file = File::create(csv_path).unwrap();
    writeln!(file, "t,x,y,z,vx,vy,vz,ax,ay,az,jx,jy,jz,a_tangential,a_lateral,a_vertical").unwrap();

    let mut prev_a: Option<DVec3> = None;

    for i in 0..=n_evals {
        let frac = i as f64 / n_evals as f64;
        let t = (start_t + frac * total_time).clamp(start_t + dt, stop_t - dt);

        let p_curr = property.evaluate(SimulationTime::new(t)).unwrap() * 1_000_000.0;
        let p_prev = property.evaluate(SimulationTime::new(t - dt)).unwrap() * 1_000_000.0;
        let p_next = property.evaluate(SimulationTime::new(t + dt)).unwrap() * 1_000_000.0;

        let v = (p_next - p_prev) / (2.0 * dt);
        let a = (p_next - 2.0 * p_curr + p_prev) / (dt * dt);

        // Jerk j = da/dt
        let (jx, jy, jz) = if let Some(pa) = prev_a {
            let step_dt = total_time / n_evals as f64;
            let j = (a - pa) / step_dt;
            (j.x, j.y, j.z)
        } else {
            (0.0, 0.0, 0.0)
        };
        prev_a = Some(a);

        // Local aerodynamic coordinates:
        // Forward unit vector T = v / ||v||
        // Up unit vector U = p_curr / ||p_curr|| (radial from Earth center)
        // Right / Lateral unit vector L = T x U
        // True normal vertical V = L x T
        let v_norm = v.length();
        let (a_tang, a_lat, a_vert) = if v_norm > 1e-3 {
            let t_vec = v / v_norm;
            let u_vec = p_curr.normalize();
            let l_vec = t_vec.cross(u_vec).normalize_or_zero();
            let v_vec = l_vec.cross(t_vec).normalize_or_zero();

            (a.dot(t_vec), a.dot(l_vec), a.dot(v_vec))
        } else {
            (0.0, 0.0, 0.0)
        };

        writeln!(
            file,
            "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}",
            t, p_curr.x, p_curr.y, p_curr.z,
            v.x, v.y, v.z,
            a.x, a.y, a.z,
            jx, jy, jz,
            a_tang, a_lat, a_vert
        ).unwrap();
    }
    println!("Wrote 20,000 spatial samples to {}", csv_path);
}
