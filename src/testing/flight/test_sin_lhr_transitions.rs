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
fn dump_vertical_transitions() {
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
    println!("Generated {} points", points.len());

    let csv_path = "/home/silas270/.gemini/antigravity-cli/brain/2de15eb8-181a-4d3f-9ca5-d350328b0865/scratch/telemetry_points.csv";
    let mut file = File::create(csv_path).unwrap();
    writeln!(file, "idx,t_s,lon,lat,alt_m,v_ms,pitch_deg,roll_deg").unwrap();
    for (i, p) in points.iter().enumerate() {
        writeln!(
            file,
            "{},{},{},{},{},{},{},{}",
            i,
            p.time_offset_ms as f64 / 1000.0,
            p.longitude,
            p.latitude,
            p.altitude,
            p.velocity_m_s,
            p.pitch_rad.to_degrees(),
            p.roll_rad.to_degrees()
        ).unwrap();
    }

    // Now build the SampledPositionProperty (Catmull-Rom) and evaluate fine-grained vertical speed
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

    let n_steps = 20000; // very fine sampling: every ~2.3 seconds
    let dt = 0.1;

    let fine_csv_path = "/home/silas270/.gemini/antigravity-cli/brain/2de15eb8-181a-4d3f-9ca5-d350328b0865/scratch/fine_vertical_profile.csv";
    let mut fine_file = File::create(fine_csv_path).unwrap();
    writeln!(fine_file, "t_s,progress,alt_m,vz_ms,az_ms2").unwrap();

    let mut prev_alt: Option<f64> = None;
    let mut prev_vz: Option<f64> = None;

    for i in 0..=n_steps {
        let progress = i as f64 / n_steps as f64;
        let t = (start_t + progress * total_time).clamp(start_t + dt, stop_t - dt);
        
        let p_curr = property.evaluate(SimulationTime::new(t)).unwrap();
        let p_prev = property.evaluate(SimulationTime::new(t - dt)).unwrap();
        let p_next = property.evaluate(SimulationTime::new(t + dt)).unwrap();

        // Approximate altitude above spherical reference (6371 km)
        // or using WGS84 normal
        let r_curr = p_curr.length() * 1_000_000.0;
        let r_prev = p_prev.length() * 1_000_000.0;
        let r_next = p_next.length() * 1_000_000.0;

        // Radial velocity vz = dr/dt:
        let vz = (r_next - r_prev) / (2.0 * dt);
        // Radial acceleration az = d^2 r / dt^2:
        let az = (r_next - 2.0 * r_curr + r_prev) / (dt * dt);

        fine_file.write_all(format!("{},{},{},{},{}\n", t, progress, r_curr - 6_371_000.0, vz, az).as_bytes()).unwrap();
    }
    println!("Wrote fine vertical profile to {}", fine_csv_path);
}
