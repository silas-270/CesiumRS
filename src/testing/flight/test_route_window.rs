//! The route line's windowed mode measures in miles, but the ribbon's own `progress` is
//! a fraction of the flight's *time*. These check the arc length that bridges the two.

use cesium_engine::globe::geometry::lon_lat_alt_to_ecef_f64;
use cesium_engine::property::sampled::{InterpolationAlgorithm, SampledPositionProperty};
use cesium_engine::render::polyline_pipeline::builder::{AdaptiveSubdivisionBuilder, ControlPoint};
use cesium_engine::time::SimulationTime;
use cesium_flight::telemetry::generate;
use cesium_flight::telemetry::geo::LatLon;
use cesium_flight::telemetry::{FlightPlanConfig, FlightRequest};
use glam::DVec3;

fn control_points(from: (f64, f64), to: (f64, f64), minutes: u64) -> Vec<ControlPoint> {
    let req = FlightRequest {
        departure: LatLon::new(from.0, from.1),
        arrival: LatLon::new(to.0, to.1),
        target_duration_ms: minutes * 60_000,
        dep_heading_deg: None,
        arr_heading_deg: None,
        runways: Vec::new(),
        config: FlightPlanConfig::default(),
    };
    let mut property =
        SampledPositionProperty::new().with_algorithm(InterpolationAlgorithm::CatmullRom);
    for pt in &generate(&req) {
        let ecef = lon_lat_alt_to_ecef_f64(pt.longitude, pt.latitude, pt.altitude);
        property.add_sample(
            SimulationTime::new(pt.time_offset_ms as f64 / 1000.0),
            DVec3::from_array(ecef),
        );
    }
    AdaptiveSubdivisionBuilder::new(1e-7).build(&property, DVec3::ZERO)
}

const JFK: (f64, f64) = (40.6413, -73.7781);
const LHR: (f64, f64) = (51.4706, -0.4619);
const FRA: (f64, f64) = (50.0333, 8.5706);
const STR: (f64, f64) = (48.6899, 9.2219);

#[test]
fn control_point_distance_runs_forward_and_matches_the_route() {
    let cps = control_points(JFK, LHR, 420);
    assert!(cps.len() > 100, "expected a densely resolved route");

    assert_eq!(cps[0].distance, 0.0, "the route starts at zero distance");
    for w in cps.windows(2) {
        assert!(
            w[1].distance >= w[0].distance,
            "distance went backwards: {} then {}",
            w[0].distance,
            w[1].distance
        );
    }

    // Distances are in Megametres. JFK-LHR is about 5,600 km of track; the check is
    // loose because the exact figure moves with wind routing and the oceanic grid.
    let total_km = cps[cps.len() - 1].distance as f64 * 1000.0;
    assert!(
        (5_000.0..6_500.0).contains(&total_km),
        "route measured {total_km:.0} km, which is not a transatlantic crossing"
    );
}

#[test]
fn distance_and_progress_are_not_the_same_axis() {
    // The reason `ControlPoint::distance` exists at all. An aircraft covers ground very
    // unevenly against the clock, so a window measured in miles is not a fixed span of
    // `progress`. The gap is small in the middle of a long flight and large at its ends,
    // so this measures it where it bites: a short sector, which is almost all climb and
    // descent, and the arrival, where a breathing route line would be most obvious.
    let cps = control_points(FRA, STR, 30);
    let total = cps[cps.len() - 1].distance;

    let distance_at = |p: f32| -> f32 {
        cps.iter()
            .find(|cp| cp.progress >= p)
            .map(|cp| cp.distance)
            .unwrap_or(total)
    };

    let cruising = distance_at(0.55) - distance_at(0.45);
    let arriving = distance_at(1.0) - distance_at(0.9);
    let departing = distance_at(0.1) - distance_at(0.0);

    assert!(
        arriving < cruising * 0.75,
        "the last tenth of the flight covered {arriving} Mm against {cruising} Mm while \
         cruising. If those matched, progress would already be a distance axis and the \
         windowed route line could be sized in minutes instead."
    );
    assert!(
        departing < cruising * 0.75,
        "the first tenth covered {departing} Mm against {cruising} Mm while cruising"
    );
}
