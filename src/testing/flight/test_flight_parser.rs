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
