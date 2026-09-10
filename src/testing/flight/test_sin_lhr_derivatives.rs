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
fn generate_sin_lhr_derivatives() {
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
    println!("Generated {} telemetry points for SIN-LHR", points.len());

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
    println!("Start time: {}, stop time: {}, duration: {} s", start_t, stop_t, total_time);

    let n_steps = 4000;
    let dt = 0.1; // 100 ms

    struct DataPoint {
        t: f64,
        s: f64, // cumulative arc length in meters
        r_prime_norm: f64, // m/s (speed)
        r_double_prime_norm: f64, // m/s^2 (acceleration)
        curvature: f64, // 1/m (geometric curvature kappa = ||r' x r''|| / ||r'||^3)
    }

    let mut data: Vec<DataPoint> = Vec::with_capacity(n_steps + 1);
    let mut cum_s = 0.0;
    let mut prev_pos: Option<DVec3> = None;

    for i in 0..=n_steps {
        let progress = i as f64 / n_steps as f64;
        // Keep interior to avoid clamped boundary artifacts
        let t = (start_t + progress * total_time).clamp(start_t + dt, stop_t - dt);
        
        let t_curr = SimulationTime::new(t);
        let t_prev = SimulationTime::new(t - dt);
        let t_next = SimulationTime::new(t + dt);

        let p_curr = property.evaluate(t_curr).unwrap();
        let p_prev = property.evaluate(t_prev).unwrap();
        let p_next = property.evaluate(t_next).unwrap();

        // Convert Mm to meters (1 Mm = 1_000_000 m)
        let p_curr_m = p_curr * 1_000_000.0;
        let p_prev_m = p_prev * 1_000_000.0;
        let p_next_m = p_next * 1_000_000.0;

        if let Some(prev_p) = prev_pos {
            cum_s += (p_curr_m - prev_p).length();
        }
        prev_pos = Some(p_curr_m);

        // First derivative r'(t) in m/s:
        let r_prime = (p_next_m - p_prev_m) / (2.0 * dt);
        let r_prime_norm = r_prime.length();

        // Second derivative r''(t) in m/s^2:
        let r_double_prime = (p_next_m - 2.0 * p_curr_m + p_prev_m) / (dt * dt);
        let r_double_prime_norm = r_double_prime.length();

        // Geometric curvature kappa = ||r' x r''|| / ||r'||^3
        let cross = r_prime.cross(r_double_prime);
        let kappa = if r_prime_norm > 1e-3 {
            cross.length() / (r_prime_norm * r_prime_norm * r_prime_norm)
        } else {
            0.0
        };

        data.push(DataPoint {
            t: start_t + progress * total_time,
            s: cum_s,
            r_prime_norm,
            r_double_prime_norm,
            curvature: kappa,
        });
    }

    let csv_path = "/home/silas270/.gemini/antigravity-cli/brain/2de15eb8-181a-4d3f-9ca5-d350328b0865/scratch/sin_lhr_derivatives.csv";
    let mut file = File::create(csv_path).unwrap();
    writeln!(file, "index,t_s,progress,s_km,r_prime_norm,r_double_prime_norm,curvature,radius_km").unwrap();

    for (idx, d) in data.iter().enumerate() {
        let radius_km = if d.curvature > 1e-12 {
            1.0 / (d.curvature * 1000.0)
        } else {
            f64::INFINITY
        };
        writeln!(
            file,
            "{},{},{},{},{},{},{},{}",
            idx,
            d.t,
            d.t / total_time,
            d.s / 1000.0,
            d.r_prime_norm,
            d.r_double_prime_norm,
            d.curvature,
            radius_km
        ).unwrap();
    }
    println!("Wrote updated CSV to {}", csv_path);
}
