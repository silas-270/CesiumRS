use cesium_flight::telemetry::generate;
use cesium_flight::telemetry::geo::LatLon;
use cesium_flight::telemetry::{FlightPlanConfig, FlightRequest};
use cesium_engine::globe::geometry::lon_lat_alt_to_ecef_f64;
use cesium_engine::property::sampled::{InterpolationAlgorithm, SampledPositionProperty};
use cesium_engine::property::Property;
use cesium_engine::time::SimulationTime;
use glam::DVec3;

struct RouteTestCase {
    name: &'static str,
    category: &'static str,
    from: (f64, f64), // lat, lon
    to: (f64, f64),   // lat, lon
    duration_min: u64,
}

#[test]
fn audit_15_routes_short_medium_long() {
    let routes = [
        // 5 Kurzstrecken (< 700 km)
        RouteTestCase { name: "FRA-STR (Frankfurt -> Stuttgart)", category: "Kurz", from: (50.0333, 8.5706), to: (48.6899, 9.2219), duration_min: 30 },
        RouteTestCase { name: "ZRH-GVA (Zuerich -> Genf)", category: "Kurz", from: (47.4581, 8.5555), to: (46.2370, 6.1092), duration_min: 35 },
        RouteTestCase { name: "LHR-CDG (London -> Paris)", category: "Kurz", from: (51.4706, -0.4619), to: (49.0097, 2.5479), duration_min: 40 },
        RouteTestCase { name: "MUC-VIE (Muenchen -> Wien)", category: "Kurz", from: (48.3538, 11.7861), to: (48.1103, 16.5697), duration_min: 50 },
        RouteTestCase { name: "GRZ-FRA (Graz -> Frankfurt)", category: "Kurz", from: (46.9911, 15.4396), to: (50.0267, 8.5584), duration_min: 70 },

        // 5 Mittelstrecken (1.000 - 4.500 km)
        RouteTestCase { name: "FRA-MAD (Frankfurt -> Madrid)", category: "Mittel", from: (50.0333, 8.5706), to: (40.4839, -3.5680), duration_min: 150 },
        RouteTestCase { name: "LHR-IST (London -> Istanbul)", category: "Mittel", from: (51.4706, -0.4619), to: (41.2753, 28.7519), duration_min: 240 },
        RouteTestCase { name: "SFO-HNL (San Francisco -> Honolulu)", category: "Mittel", from: (37.6213, -122.3790), to: (21.3187, -157.9224), duration_min: 300 },
        RouteTestCase { name: "JFK-MIA (New York -> Miami)", category: "Mittel", from: (40.6413, -73.7781), to: (25.7959, -80.2870), duration_min: 190 },
        RouteTestCase { name: "CDG-DXB (Paris -> Dubai)", category: "Mittel", from: (49.0097, 2.5479), to: (25.2532, 55.3657), duration_min: 400 },

        // 5 Langstrecken (> 5.500 km)
        RouteTestCase { name: "JFK-LHR (New York -> London)", category: "Lang", from: (40.6413, -73.7781), to: (51.4706, -0.4619), duration_min: 420 },
        RouteTestCase { name: "LHR-NRT (London -> Tokio)", category: "Lang", from: (51.4706, -0.4619), to: (35.7647, 140.3864), duration_min: 720 },
        RouteTestCase { name: "DXB-JFK (Dubai -> New York)", category: "Lang", from: (25.2532, 55.3657), to: (40.6413, -73.7781), duration_min: 840 },
        RouteTestCase { name: "DXB-SYD (Dubai -> Sydney)", category: "Lang", from: (25.2532, 55.3657), to: (-33.9399, 151.1753), duration_min: 840 },
        RouteTestCase { name: "SIN-LHR (Singapur -> London)", category: "Lang", from: (1.3644, 103.9915), to: (51.4706, -0.4619), duration_min: 780 },
    ];

    println!("\n{:=^110}", " MULTI-ROUTE AUDIT (5 KURZ, 5 MITTEL, 5 LANG) ");
    println!(
        "{:<35} | {:<7} | {:<12} | {:<12} | {:<12} | {:<12}",
        "Route", "Kat.", "Max a_tang", "Min a_tang", "Max a_lat", "Max a_vert"
    );
    println!("{:-^110}", "");

    let mut all_passed = true;

    for r in &routes {
        let req = FlightRequest {
            departure: LatLon::new(r.from.0, r.from.1),
            arrival: LatLon::new(r.to.0, r.to.1),
            target_duration_ms: r.duration_min * 60_000,
            dep_heading_deg: None,
            arr_heading_deg: None,
            runways: Vec::new(),
            config: FlightPlanConfig::default(),
        };

        let points = generate(&req);
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

        let n_evals = 3000;
        let dt = 0.1;

        let mut max_a_tang = 0.0_f64;
        let mut min_a_tang = 0.0_f64;
        let mut max_a_lat = 0.0_f64;
        let mut max_a_vert = 0.0_f64;

        for i in 0..=n_evals {
            let frac = i as f64 / n_evals as f64;
            let t = (start_t + frac * total_time).clamp(start_t + dt, stop_t - dt);

            let p_curr = property.evaluate(SimulationTime::new(t)).unwrap() * 1_000_000.0;
            let p_prev = property.evaluate(SimulationTime::new(t - dt)).unwrap() * 1_000_000.0;
            let p_next = property.evaluate(SimulationTime::new(t + dt)).unwrap() * 1_000_000.0;

            let v = (p_next - p_prev) / (2.0 * dt);
            let a = (p_next - 2.0 * p_curr + p_prev) / (dt * dt);

            let v_norm = v.length();
            if v_norm > 1e-3 {
                let t_vec = v / v_norm;
                let u_vec = p_curr.normalize();
                let l_vec = t_vec.cross(u_vec).normalize_or_zero();
                let v_vec = l_vec.cross(t_vec).normalize_or_zero();

                let a_tang = a.dot(t_vec);
                let a_lat = a.dot(l_vec).abs();
                let a_vert = a.dot(v_vec).abs();

                if a_tang > 2.6 || a_tang < -6.0 || a_vert > 3.5 {
                    if r.name.starts_with("FRA-STR") || r.name.starts_with("JFK-LHR") {
                        println!("  [OUTLIER in {}] t={:.1}s | a_tang={:+.2} | a_lat={:.2} | a_vert={:.2} | v={:.1} m/s", r.name, t, a_tang, a_lat, a_vert, v_norm);
                    }
                }

                if a_tang > max_a_tang { max_a_tang = a_tang; }
                if a_tang < min_a_tang { min_a_tang = a_tang; }
                if a_lat > max_a_lat { max_a_lat = a_lat; }
                if a_vert > max_a_vert { max_a_vert = a_vert; }
            }
        }

        // Assert sanity criteria across all dimensions
        // 1. Max positive longitudinal acceleration <= 2.5 m/s^2 (takeoff roll is ~1.65 - 1.8)
        // 2. Minimum longitudinal deceleration (braking) >= -6.0 m/s^2 (rollout braking ~1.8, taxi stops)
        // 3. Lateral turn acceleration <= 6.0 m/s^2 (bank angle <= 25-28 deg -> max ~4.7-5.0 m/s^2)
        // 4. Vertical acceleration <= 3.5 m/s^2 (flare ~1-2.5 m/s^2, rotation)
        let pass = max_a_tang <= 2.6 && min_a_tang >= -6.0 && max_a_lat <= 6.0 && max_a_vert <= 3.5;
        if !pass {
            all_passed = false;
        }

        let flag = if pass { "OK" } else { "WARN" };
        println!(
            "{:<35} | {:<7} | {:+6.2} m/s^2 | {:+6.2} m/s^2 | {:6.2} m/s^2 | {:6.2} m/s^2 | [{}]",
            r.name, r.category, max_a_tang, min_a_tang, max_a_lat, max_a_vert, flag
        );
    }
    println!("{:=^110}\n", "");
    assert!(all_passed, "One or more routes exceeded acceleration smoothness limits!");
}
