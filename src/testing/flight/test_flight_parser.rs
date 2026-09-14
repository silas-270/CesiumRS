use cesium_flight::telemetry::geo::{distance_m, LatLon};
use cesium_flight::telemetry::{generate, FlightPlanConfig, FlightRequest};

#[test]
fn test_flight_generation() {
    let departure = LatLon::new(50.0333, 8.5706);
    let arrival = LatLon::new(48.6899, 9.2219);
    let pts = generate(&FlightRequest {
        departure,
        arrival,
        target_duration_ms: 1_800_000,
        dep_heading_deg: None,
        arr_heading_deg: None,
        runways: Vec::new(),
        config: FlightPlanConfig::default(),
    });
    assert!(!pts.is_empty());

    let start = pts.first().unwrap();
    let end = pts.last().unwrap();

    // Near the airports rather than exactly on them: a flight now begins on an apron a
    // couple of kilometres back from the runway threshold and ends on another after the
    // landing rollout, instead of starting and stopping on the airport reference point.
    let tolerance_m = 6_000.0;
    assert!(distance_m(LatLon::new(start.latitude, start.longitude), departure) < tolerance_m);
    assert!(distance_m(LatLon::new(end.latitude, end.longitude), arrival) < tolerance_m);
}

#[test]
fn test_sun_intensity_never_nan_across_flight() {
    let request = FlightRequest {
        departure: LatLon::new(50.0333, 8.5706),
        arrival: LatLon::new(48.6899, 9.2219),
        target_duration_ms: 1_800_000,
        dep_heading_deg: None,
        arr_heading_deg: None,
        runways: Vec::new(),
        config: FlightPlanConfig::default(),
    };
    let points = generate(&request);
    let mut sun_intensity_property =
        cesium_engine::property::sampled::SampledScalarProperty::new()
            .with_algorithm(cesium_engine::property::sampled::InterpolationAlgorithm::CatmullRom);

    for pt in &points {
        let time = cesium_engine::time::SimulationTime::new(pt.time_offset_ms as f64 / 1000.0);
        sun_intensity_property.add_sample(time, pt.sun_intensity as f64);
    }

    use cesium_engine::property::Property;
    let start_t = points.first().unwrap().time_offset_ms as f64 / 1000.0;
    let stop_t = points.last().unwrap().time_offset_ms as f64 / 1000.0;

    for i in 0..=2000 {
        let p = (i as f64) / 2000.0;
        let time = cesium_engine::time::SimulationTime::new(start_t + p * (stop_t - start_t));
        let raw_sun = sun_intensity_property.evaluate(time).unwrap();
        let clamped_sun = raw_sun.clamp(0.0, 1.0) as f32;
        
        let cockpit_ambient = 0.05 + 0.29 * clamped_sun.powf(1.5);
        assert!(cockpit_ambient.is_finite(), "Cockpit ambient became non-finite at progress {}", p);
        assert!(cockpit_ambient >= 0.05, "Cockpit ambient dropped below minimum floor at progress {}", p);
    }
}
