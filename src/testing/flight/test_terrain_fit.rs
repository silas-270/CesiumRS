//! The route line and the aircraft are fitted onto the drawn terrain near the airports by
//! one rule (`cesium_flight::terrain_fit`). When only the aircraft was fitted, the line
//! hung 20-39 m above it on the runway. These check that the line lies on the ground
//! along the runway and meets the aircraft wherever the fit applies, against a synthetic
//! terrain with bumps and a slope, so neither the network nor a GPU is involved.
//! `rendering::route_line_ground` is the same thing looked at, on the real terrain.

use cesium_engine::globe::geometry::{ecef_to_lon_lat_f64, lon_lat_alt_to_ecef_f64, lon_lat_to_ecef_f64};
use cesium_engine::property::sampled::{InterpolationAlgorithm, SampledPositionProperty};
use cesium_engine::property::Property;
use cesium_engine::render::polyline_pipeline::builder::{AdaptiveSubdivisionBuilder, ControlPoint};
use cesium_engine::render::polyline_pipeline::pipeline::RIBBON_LIFT_M;
use cesium_engine::time::SimulationTime;
use cesium_flight::telemetry::geo::LatLon;
use cesium_flight::telemetry::{generate, FlightPlanConfig, FlightRequest, TelemetryPoint};
use cesium_flight::terrain_fit::{
    altitude_at, position_of, AircraftFit, GroundEnds, LineFit, LINE_GROUND_LIFT_M,
    LINE_GROUND_SPACING_M,
};
use glam::DVec3;

/// FRA-STR exactly as the viewer plans it, with the preset's headings and elevations.
fn fra_str() -> (Vec<TelemetryPoint>, SampledPositionProperty) {
    let r = cesium_flight::preset::parse_route("FRA-STR").unwrap();
    let mut config = FlightPlanConfig::default();
    config.dep_elevation_m = r.dep_elevation_m.unwrap();
    config.arr_elevation_m = r.arr_elevation_m.unwrap();
    let points = generate(&FlightRequest {
        departure: LatLon::new(r.departure_lat, r.departure_lon),
        arrival: LatLon::new(r.arrival_lat, r.arrival_lon),
        target_duration_ms: r.total_duration_ms,
        dep_heading_deg: r.dep_heading_deg,
        arr_heading_deg: r.arr_heading_deg,
        runways: Vec::new(),
        config,
    });
    let mut property =
        SampledPositionProperty::new().with_algorithm(InterpolationAlgorithm::CatmullRom);
    for p in &points {
        property.add_sample(
            SimulationTime::new(p.time_offset_ms as f64 / 1000.0),
            DVec3::from_array(lon_lat_alt_to_ecef_f64(p.longitude, p.latitude, p.altitude)),
        );
    }
    (points, property)
}

/// Terrain lower than both published elevations, as the real one is: flat at Frankfurt,
/// falling 500 m per degree eastward along the Stuttgart rollout, and bumpy at both — 2 m
/// bumps some 60 m across, the size of the source data's noise. Megametres above the
/// ellipsoid, like the engine's query.
fn terrain(pos: DVec3) -> Option<f64> {
    let (lon, lat) = ecef_to_lon_lat_f64(pos);
    let base = if lat > 49.4 { 98.0 } else { 372.0 - 500.0 * (lon - 9.2219) };
    let bumps = 2.0 * (lon * 7_500.0).sin() * (lat * 11_000.0).cos();
    Some((base + bumps) * 1.0e-6)
}

/// Height of an ECEF position above the ellipsoid, metres, by the engine's own mapping.
fn height_m(pos: DVec3) -> f64 {
    let (lon, lat) = ecef_to_lon_lat_f64(pos);
    let surface = DVec3::from_array(lon_lat_to_ecef_f64(lon, lat));
    let normal = (DVec3::from_array(lon_lat_alt_to_ecef_f64(lon, lat, 1.0)) - surface).normalize();
    (pos - surface).dot(normal) * 1.0e6
}

/// The line as the tracker builds it, as built (before any fit) and fitted to `ground`
/// until every point on the ground has been read.
fn fitted_line(
    points: &[TelemetryPoint],
    property: &SampledPositionProperty,
    ground: &dyn Fn(DVec3) -> Option<f64>,
) -> (GroundEnds, Vec<ControlPoint>, LineFit) {
    let ends = GroundEnds::new(points).unwrap();
    let mut builder = AdaptiveSubdivisionBuilder::new(1e-7);
    builder.max_segment_lengths = ends.line_spacing();
    let base = builder.build(property, DVec3::ZERO);
    let mut line = LineFit::new(base.clone(), &ends, points);
    // Some 500 points on the ground at 64 a frame; well past enough frames to read them all.
    for _ in 0..40 {
        line.sample(&ends, ground);
    }
    (ends, base, line)
}

#[test]
fn fra_str_ends_are_found() {
    let (points, _) = fra_str();
    let ends = GroundEnds::new(&points).unwrap();
    println!("  dep: field {:.2} m, airborne after {:.1} s", ends.dep.field_m, ends.dep.edge_s);
    println!("  arr: field {:.2} m, on the ground from {:.1} s", ends.arr.field_m, ends.arr.edge_s);
    // The published elevations plus the planner's 3 m render lift.
    assert!((ends.dep.field_m - 114.0).abs() < 1e-9);
    assert!((ends.arr.field_m - 392.0).abs() < 1e-9);
    assert!((60.0..90.0).contains(&ends.dep.edge_s), "lift-off at {}", ends.dep.edge_s);
    assert!((1740.0..1760.0).contains(&ends.arr.edge_s), "touchdown at {}", ends.arr.edge_s);
    assert!(ends.on_ground(0.0) && ends.on_ground(ends.end_s));
    assert!(!ends.on_ground(ends.dep.edge_s + 1.0) && !ends.on_ground(ends.arr.edge_s - 1.0));
}

#[test]
fn the_builder_holds_segments_short_only_where_asked() {
    // A straight path at 100 m/s for 20 minutes, which the tolerance alone leaves as one
    // segment per five-minute step.
    let mut property = SampledPositionProperty::new();
    for i in 0..=120 {
        let t = i as f64 * 10.0;
        property.add_sample(SimulationTime::new(t), DVec3::new(6.4, 0.0, 0.0) + DVec3::new(0.0, 1.0e-4, 0.0) * t);
    }
    let mut builder = AdaptiveSubdivisionBuilder::new(1e-7);
    builder.max_segment_lengths = vec![(100.0, 200.0, 10.0e-6)];
    let cps = builder.build(&property, DVec3::ZERO);

    let total = 1200.0;
    let (mut inside, mut outside) = (0, 0);
    for w in cps.windows(2) {
        let (t0, t1) = (w[0].progress as f64 * total, w[1].progress as f64 * total);
        let len_m = (position_of(&w[1]) - position_of(&w[0])).length() * 1.0e6;
        if t0 < 199.999 && t1 > 100.001 {
            inside += 1;
            assert!(len_m <= 10.0 + 1e-6, "segment {t0:.2}-{t1:.2} s is {len_m:.3} m");
        } else {
            outside += 1;
        }
    }
    println!("  {inside} segments inside the range, {outside} outside");
    // 10 km at 10 m apart at most, and no finer than the 0.1 s floor would force.
    assert!((1_000..=2_100).contains(&inside), "{inside} segments inside");
    // Outside, one segment per five-minute step: the range's two ends are points of their
    // own, so nothing has to be halved down to meet them.
    assert!(outside <= 6, "{outside} segments outside");
    for edge in [100.0, 200.0] {
        assert!(
            cps.iter().any(|cp| (cp.progress as f64 * total - edge).abs() < 1e-3),
            "no point at {edge} s"
        );
    }
}

#[test]
fn on_the_ground_the_line_lies_on_the_terrain() {
    let (points, property) = fra_str();
    let (ends, base, line) = fitted_line(&points, &property, &terrain);
    let total = ends.end_s - ends.start_s;

    let mut checked = 0;
    let mut worst = 0.0_f64;
    let (mut length_m, mut shortest_m) = (0.0_f64, f64::MAX);
    let mut previous: Option<(f64, DVec3)> = None;
    for (cp, built) in line.points().iter().zip(&base) {
        let t_s = ends.start_s + cp.progress as f64 * total;
        if !ends.on_ground(t_s) {
            previous = None;
            continue;
        }
        let pos = position_of(cp);
        // Where the line is drawn: its control point plus the shader's own lift.
        let drawn = height_m(pos) + RIBBON_LIFT_M;
        let ground = terrain(pos).unwrap() * 1.0e6;
        // "On the ground" takes in the first half metre of the rotation, which the line
        // (like the aircraft) keeps on top of the terrain.
        let field = if ends.is_departure(t_s) { ends.dep.field_m } else { ends.arr.field_m };
        let rotation = height_m(position_of(built)) - field;
        let err = drawn - (ground + LINE_GROUND_LIFT_M + rotation);
        worst = worst.max(err.abs());
        assert!(err.abs() < 0.01, "at {t_s:.2} s the line is {drawn:.3} m over ground {ground:.3} m");
        if let Some((t_prev, p_prev)) = previous {
            // The spacing holds for the line as built; moving points up and down onto the
            // bumps lengthens a segment a little.
            let gap_m = (position_of(built) - p_prev).length() * 1.0e6;
            assert!(
                gap_m <= LINE_GROUND_SPACING_M + 0.05,
                "{gap_m:.2} m between points at {t_prev:.2} and {t_s:.2} s"
            );
            length_m += gap_m;
            shortest_m = shortest_m.min(gap_m);
        }
        previous = Some((t_s, position_of(built)));
        checked += 1;
    }
    println!(
        "  {checked} points over {length_m:.0} m on the ground (shortest gap {shortest_m:.3} m), \
         worst {worst:.4} m off {LINE_GROUND_LIFT_M} m over the terrain"
    );
    assert!(checked > 300, "only {checked} points on the ground");
}

/// Height of the line where it passes the aircraft's profile position `raw`: the
/// segment of the line as built that passes nearest to it, at the same fraction of the
/// fitted segment.
fn line_height_at(base: &[ControlPoint], fitted: &[ControlPoint], raw: DVec3, near: usize) -> f64 {
    let lo = near.saturating_sub(40);
    let hi = (near + 40).min(base.len() - 1);
    let mut best = (f64::MAX, 0, 0.0);
    for i in lo..hi {
        let (a, b) = (position_of(&base[i]), position_of(&base[i + 1]));
        let ab = b - a;
        let f = if ab.length_squared() > 0.0 {
            ((raw - a).dot(ab) / ab.length_squared()).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let d = (a + ab * f - raw).length();
        if d < best.0 {
            best = (d, i, f);
        }
    }
    let (_, i, f) = best;
    let (a, b) = (position_of(&fitted[i]), position_of(&fitted[i + 1]));
    height_m(a + (b - a) * f)
}

#[test]
fn the_line_meets_the_aircraft_wherever_the_fit_applies() {
    let (points, property) = fra_str();
    let (ends, base, line) = fitted_line(&points, &property, &terrain);
    let total = ends.end_s - ends.start_s;

    let (mut worst_ground, mut worst_air) = (0.0_f64, 0.0_f64);
    let mut checked = 0;
    let times = (0..=320).map(|i| i as f64 * 0.5).chain((0..=520).map(|i| total - i as f64 * 0.5));
    for t_s in times {
        let progress = (t_s - ends.start_s) / total;
        let raw = property.evaluate(SimulationTime::new(t_s)).unwrap();
        let altitude = altitude_at(&points, t_s).unwrap();
        // A fresh fit snaps to its target instead of easing towards it.
        let mut fit = AircraftFit::default();
        fit.update(&ends, progress, t_s, altitude, raw, &terrain);
        if fit.weight <= 0.0 {
            continue;
        }
        let aircraft = height_m(raw + raw.normalize() * (fit.offset_m * 1.0e-6));

        let near = base.partition_point(|cp| (cp.progress as f64) < progress);
        let line_h = line_height_at(&base, line.points(), raw, near);
        // The line is meant to sit this far off the aircraft: its usual lift in the air,
        // [`LINE_GROUND_LIFT_M`] on the ground, blended by the same weight.
        let expected = (LINE_GROUND_LIFT_M - RIBBON_LIFT_M) * fit.weight;
        let err = (line_h - aircraft) - expected;
        if ends.on_ground(t_s) {
            worst_ground = worst_ground.max(err.abs());
            // Straight 10 m chords across 60 m bumps stand up to 0.27 m off them.
            assert!(err.abs() < 0.35, "on the ground at {t_s:.1} s: line off the aircraft by {err:.3} m");
        } else {
            worst_air = worst_air.max(err.abs());
            // The builder's own tolerance is 0.1 m.
            assert!(err.abs() < 0.15, "airborne at {t_s:.1} s: line off the aircraft by {err:.3} m");
        }
        checked += 1;
    }
    println!("  {checked} positions: worst {worst_ground:.3} m on the ground, {worst_air:.3} m in the air");
    assert!(checked > 600);
}

#[test]
fn without_terrain_nothing_moves() {
    let (points, property) = fra_str();
    let no_ground = |_: DVec3| None;
    let (ends, base, mut line) = fitted_line(&points, &property, &no_ground);
    let fields = |cp: &ControlPoint| (cp.pos_hi, cp.pos_lo, cp.distance, cp.progress);
    assert!(base.iter().map(fields).eq(line.points().iter().map(fields)));
    assert!(line.take_changes().iter().all(Option::is_none));

    let raw = property.evaluate(SimulationTime::new(0.0)).unwrap();
    let mut fit = AircraftFit::default();
    fit.update(&ends, 0.0, 0.0, altitude_at(&points, 0.0).unwrap(), raw, &no_ground);
    assert_eq!((fit.offset_m, fit.weight), (0.0, 0.0));
}

#[test]
fn airborne_the_fit_does_not_follow_the_ground_below() {
    // A 150 m ridge under the climb-out, a few km past the runway end. The rule reads the
    // ground at lift-off once airborne, so the ridge must not move the aircraft or the line.
    let (points, property) = fra_str();
    let ends = GroundEnds::new(&points).unwrap();
    let ridge = |pos: DVec3| -> Option<f64> {
        let (lon, _) = ecef_to_lon_lat_f64(pos);
        let flat = terrain(pos)? * 1.0e6;
        Some((flat + if lon < ends_lon(&ends) - 0.02 { 150.0 } else { 0.0 }) * 1.0e-6)
    };
    let t_s = 110.0; // ~250 m above the field, over the ridge
    let raw = property.evaluate(SimulationTime::new(t_s)).unwrap();
    let altitude = altitude_at(&points, t_s).unwrap();
    let progress = t_s / (ends.end_s - ends.start_s);
    let (mut flat, mut ridged) = (AircraftFit::default(), AircraftFit::default());
    flat.update(&ends, progress, t_s, altitude, raw, &terrain);
    ridged.update(&ends, progress, t_s, altitude, raw, &ridge);
    assert!(ecef_to_lon_lat_f64(raw).0 < ends_lon(&ends) - 0.02, "the test point is past the ridge edge");
    assert!(flat.weight > 0.0 && flat.weight < 1.0);
    assert!((flat.offset_m - ridged.offset_m).abs() < 1e-9, "{} vs {}", flat.offset_m, ridged.offset_m);
}

/// Longitude of the departure anchor, degrees.
fn ends_lon(ends: &GroundEnds) -> f64 {
    ecef_to_lon_lat_f64(ends.dep.anchor).0
}

/// STR-FRA exactly as the viewer plans it in satellite mode, with preset headings and elevations.
fn str_fra() -> (Vec<TelemetryPoint>, SampledPositionProperty) {
    let r = cesium_flight::preset::parse_route("STR-FRA").unwrap();
    let mut config = FlightPlanConfig::default();
    config.terrain_elevation = true;
    config.dep_elevation_m = r.dep_elevation_m.unwrap();
    config.arr_elevation_m = r.arr_elevation_m.unwrap();
    let points = generate(&FlightRequest {
        departure: LatLon::new(r.departure_lat, r.departure_lon),
        arrival: LatLon::new(r.arrival_lat, r.arrival_lon),
        target_duration_ms: r.total_duration_ms,
        dep_heading_deg: r.dep_heading_deg,
        arr_heading_deg: r.arr_heading_deg,
        runways: Vec::new(),
        config,
    });
    let mut property =
        SampledPositionProperty::new().with_algorithm(InterpolationAlgorithm::CatmullRom);
    for p in &points {
        property.add_sample(
            SimulationTime::new(p.time_offset_ms as f64 / 1000.0),
            DVec3::from_array(lon_lat_alt_to_ecef_f64(p.longitude, p.latitude, p.altitude)),
        );
    }
    (points, property)
}

#[test]
fn str_fra_ends_are_found() {
    let (points, _) = str_fra();
    let ends = GroundEnds::new(&points).unwrap();
    assert!((ends.dep.field_m - 392.0).abs() < 1e-9);
    assert!((ends.arr.field_m - 114.0).abs() < 1e-9);
    assert!((60.0..90.0).contains(&ends.dep.edge_s), "lift-off at {}", ends.dep.edge_s);
    assert!((1740.0..1760.0).contains(&ends.arr.edge_s), "touchdown at {}", ends.arr.edge_s);
    assert!(ends.on_ground(0.0) && ends.on_ground(ends.end_s));
    assert!(!ends.on_ground(ends.dep.edge_s + 1.0) && !ends.on_ground(ends.arr.edge_s - 1.0));
}
