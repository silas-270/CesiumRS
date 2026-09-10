//! Behavioural tests for the flight planner.
//!
//! These pin the *shape* of a flight plan, not its constants, so retuning the aircraft
//! model or the wind field does not mean rewriting them. What they assert is the set of
//! things that were wrong before and must not silently become wrong again: the route is
//! a great circle rather than a line on a flat map, it does not go the wrong way around
//! the world, the aircraft climbs at an airliner's angle and banks at an airliner's
//! angle, and it lands on the runway rather than three kilometres short of it.

use cesium_flight::flight_handle::RunwayData;
use cesium_flight::telemetry::atmosphere::m_to_feet;
use cesium_flight::telemetry::geo::{distance_m, LatLon};
use cesium_flight::telemetry::vertical::{VerticalInputs, VerticalProfile};
use cesium_flight::telemetry::{
    generate, FlightPlanConfig, FlightRequest, TelemetryPoint, WindModel,
};

const LHR: (f64, f64) = (51.4706, -0.4619);
const NRT: (f64, f64) = (35.7647, 140.3864);
const JFK: (f64, f64) = (40.6413, -73.7781);
const MEL: (f64, f64) = (-37.6733, 144.8433);
const FRA: (f64, f64) = (50.0333, 8.5706);
const STR: (f64, f64) = (48.6899, 9.2219);
const SVO: (f64, f64) = (55.9726, 37.4146);
const LED: (f64, f64) = (59.8003, 30.2625);

fn at(p: (f64, f64)) -> LatLon {
    LatLon::new(p.0, p.1)
}

fn plan(from: (f64, f64), to: (f64, f64), minutes: u64) -> Vec<TelemetryPoint> {
    plan_with(from, to, minutes, FlightPlanConfig::default())
}

fn plan_with(
    from: (f64, f64),
    to: (f64, f64),
    minutes: u64,
    config: FlightPlanConfig,
) -> Vec<TelemetryPoint> {
    generate(&FlightRequest {
        departure: at(from),
        arrival: at(to),
        target_duration_ms: minutes * 60_000,
        dep_heading_deg: None,
        arr_heading_deg: None,
        runways: Vec::new(),
        config,
    })
}

/// Still air and no closures, so the route is pure geometry.
fn geometric() -> FlightPlanConfig {
    FlightPlanConfig {
        wind: WindModel::Calm,
        avoid_closed_airspace: false,
        oceanic_tracks: false,
        ..FlightPlanConfig::default()
    }
}

fn track_length_m(points: &[TelemetryPoint]) -> f64 {
    points
        .windows(2)
        .map(|w| {
            distance_m(
                LatLon::new(w[0].latitude, w[0].longitude),
                LatLon::new(w[1].latitude, w[1].longitude),
            )
        })
        .sum()
}

fn max_latitude(points: &[TelemetryPoint]) -> f64 {
    points.iter().fold(f64::MIN, |m, p| m.max(p.latitude))
}

fn cruise_points(points: &[TelemetryPoint]) -> Vec<&TelemetryPoint> {
    let ceiling = points.iter().fold(0.0_f64, |m, p| m.max(p.altitude));
    points
        .iter()
        .filter(|p| p.altitude > ceiling * 0.95)
        .collect()
}

// ---------------------------------------------------------------------------
// Lateral geometry
// ---------------------------------------------------------------------------

#[test]
fn the_route_follows_a_great_circle_not_a_flat_projection() {
    // London to Tokyo. The great circle crosses the Arctic at about 71°N; a straight
    // line on a lon/lat plane crosses central Asia around 45°N and is a fifth longer.
    let points = plan_with(LHR, NRT, 13 * 60, geometric());
    assert!(
        max_latitude(&points) > 62.0,
        "great circle should climb far north of both endpoints, peaked at {:.1}°",
        max_latitude(&points)
    );

    let direct = distance_m(at(LHR), at(NRT));
    let flown = track_length_m(&points);
    assert!(
        flown < direct * 1.06,
        "flew {:.0} km against a {:.0} km great circle",
        flown / 1000.0,
        direct / 1000.0
    );
}

#[test]
fn routes_across_the_antimeridian_take_the_short_way() {
    // Melbourne to New York crosses 180°. Subtracting longitudes sends it the wrong way
    // round the planet, which is roughly twice as far.
    let points = plan_with(MEL, JFK, 20 * 60, geometric());
    let direct = distance_m(at(MEL), at(JFK));
    let flown = track_length_m(&points);
    assert!(
        flown < direct * 1.10,
        "flew {:.0} km against a {:.0} km great circle",
        flown / 1000.0,
        direct / 1000.0
    );

    // And no single step teleports across the seam.
    let longest = points
        .windows(2)
        .map(|w| {
            distance_m(
                LatLon::new(w[0].latitude, w[0].longitude),
                LatLon::new(w[1].latitude, w[1].longitude),
            )
        })
        .fold(0.0_f64, f64::max);
    assert!(longest < 10_000.0, "longest sample step was {longest:.0} m");
}

#[test]
fn a_short_sector_still_produces_a_sane_track() {
    let points = plan(FRA, STR, 55);
    assert!(points.len() > 50);
    let direct = distance_m(at(FRA), at(STR));
    let flown = track_length_m(&points);
    // Terminal geometry — taxi, the departure leg, a ten-mile final — adds a fixed
    // overhead that is proportionally large on a 160 km sector.
    assert!(
        flown > direct * 0.7 && flown < direct * 2.2,
        "flew {:.0} km against a {:.0} km direct distance",
        flown / 1000.0,
        direct / 1000.0
    );
}

// ---------------------------------------------------------------------------
// Wind and airspace
// ---------------------------------------------------------------------------

#[test]
fn opposite_directions_take_different_tracks() {
    // The jet stream is the reason a westbound North Atlantic crossing does not simply
    // retrace the eastbound one.
    let east = plan(JFK, LHR, 7 * 60);
    let west = plan(LHR, JFK, 8 * 60);

    let sample = |points: &[TelemetryPoint], f: f64| -> LatLon {
        let idx = ((points.len() - 1) as f64 * f) as usize;
        LatLon::new(points[idx].latitude, points[idx].longitude)
    };
    let separation = distance_m(sample(&east, 0.5), sample(&west, 0.5));
    assert!(
        separation > 40_000.0,
        "eastbound and westbound midpoints only {separation:.0} m apart"
    );
}

#[test]
fn wind_makes_the_two_directions_take_different_times_to_fly() {
    // Same route, same session length, so what differs is the *physical* time the plan
    // would take. Eastbound rides the jet and needs less of it.
    let east = plan(JFK, LHR, 7 * 60);
    let west = plan(LHR, JFK, 7 * 60);
    let mean_speed = |p: &[TelemetryPoint]| -> f64 {
        let c = cruise_points(p);
        c.iter().map(|q| q.velocity_m_s).sum::<f64>() / c.len() as f64
    };
    assert!(
        mean_speed(&east) > mean_speed(&west) + 5.0,
        "eastbound cruise {:.0} m/s should beat westbound {:.0} m/s",
        mean_speed(&east),
        mean_speed(&west)
    );
}

#[test]
fn an_atlantic_crossing_sits_on_the_organised_track_grid() {
    use cesium_flight::telemetry::airspace::AirspaceRestrictions;
    use cesium_flight::telemetry::lateral::{plan_enroute, EnrouteOptions};
    use cesium_flight::telemetry::wind::WindField;

    // Oceanic tracks are published as whole degrees of latitude on every tenth
    // meridian, which is what gives a crossing its stepped, angular look.
    let wind = WindField::annual_mean();
    let waypoints = plan_enroute(
        at(JFK),
        at(LHR),
        &EnrouteOptions {
            wind: &wind,
            airspace: &AirspaceRestrictions::none(),
            cruise_altitude_m: 11_278.0,
            cruise_tas: 250.0,
            oceanic_tracks: true,
        },
    );

    let on_grid = waypoints
        .iter()
        .filter(|p| {
            (-60.0..=-10.0).contains(&p.lon_deg)
                && (p.lon_deg / 10.0).fract().abs() < 1e-6
                && p.lat_deg.fract().abs() < 1e-6
        })
        .count();
    assert!(
        on_grid >= 4,
        "expected the crossing to sit on the track grid, found {on_grid} points"
    );
}

#[test]
fn the_westbound_track_lies_north_of_the_eastbound_one() {
    use cesium_flight::telemetry::airspace::AirspaceRestrictions;
    use cesium_flight::telemetry::lateral::{plan_enroute, EnrouteOptions};
    use cesium_flight::telemetry::wind::WindField;

    // Eastbound rides the jet; westbound climbs out of it. That is why the organised
    // track system publishes two sets of tracks and why they do not overlap.
    let wind = WindField::annual_mean();
    let opts = |a: LatLon, b: LatLon| {
        plan_enroute(
            a,
            b,
            &EnrouteOptions {
                wind: &wind,
                airspace: &AirspaceRestrictions::none(),
                cruise_altitude_m: 11_278.0,
                cruise_tas: 250.0,
                oceanic_tracks: false,
            },
        )
    };
    let east = opts(at(JFK), at(LHR));
    let west = opts(at(LHR), at(JFK));
    let mid_lat = |w: &[LatLon]| w[w.len() / 2].lat_deg;
    assert!(
        mid_lat(&west) > mid_lat(&east) + 2.0,
        "westbound crossed at {:.1}°N, eastbound at {:.1}°N",
        mid_lat(&west),
        mid_lat(&east)
    );
}

#[test]
fn closed_airspace_is_avoided() {
    let restrictions =
        cesium_flight::telemetry::airspace::AirspaceRestrictions::for_route(at(LHR), at(NRT));
    let points = plan(LHR, NRT, 15 * 60);
    let violations = points
        .iter()
        .filter(|p| restrictions.blocks(LatLon::new(p.latitude, p.longitude)))
        .count();
    assert_eq!(violations, 0, "{violations} samples inside closed airspace");

    // And avoiding it costs distance, which is the whole reason the route is longer
    // than the great circle in reality.
    let direct = plan_with(LHR, NRT, 15 * 60, geometric());
    assert!(
        track_length_m(&points) > track_length_m(&direct) * 1.05,
        "avoiding closed airspace should lengthen the route noticeably"
    );
}

#[test]
fn a_domestic_flight_may_cross_its_own_countrys_airspace() {
    // Moscow to St Petersburg. The closure is against foreign operators, not physics;
    // treating it as absolute would leave this route with no legal path at all.
    let points = plan(SVO, LED, 95);
    let direct = distance_m(at(SVO), at(LED));
    assert!(
        track_length_m(&points) < direct * 2.0,
        "a domestic route should not detour around its own country"
    );
}

#[test]
fn closed_regions_do_not_straddle_the_antimeridian() {
    // The containment test is planar in lon/lat, which is only sound while this holds.
    for region in cesium_flight::telemetry::airspace::closed_regions() {
        assert!(
            region.longitude_span() < 180.0,
            "{} spans {:.0}° of longitude",
            region.name,
            region.longitude_span()
        );
    }
}

// ---------------------------------------------------------------------------
// Vertical profile
// ---------------------------------------------------------------------------

#[test]
fn climb_and_descent_are_flown_at_airliner_angles() {
    let points = plan(FRA, JFK, 8 * 60 + 30);
    let mut steepest_climb: f64 = 0.0;
    let mut steepest_descent: f64 = 0.0;
    for w in points.windows(2) {
        let run = distance_m(
            LatLon::new(w[0].latitude, w[0].longitude),
            LatLon::new(w[1].latitude, w[1].longitude),
        );
        if run < 1.0 {
            continue;
        }
        let gradient = ((w[1].altitude - w[0].altitude) / run).atan().to_degrees();
        steepest_climb = steepest_climb.max(gradient);
        steepest_descent = steepest_descent.min(gradient);
    }
    // A jet transport leaves the ground at 10-15° and descends at about 3°.
    assert!(
        steepest_climb < 16.0,
        "steepest climb was {steepest_climb:.1}°"
    );
    assert!(
        steepest_descent > -5.0,
        "steepest descent was {steepest_descent:.1}°"
    );
}

#[test]
fn cruise_happens_at_a_flight_level_with_the_right_parity() {
    // Eastbound gets odd thousands of feet, westbound even. This is the semicircular
    // rule, and it is why the two directions are never at the same altitude.
    for (from, to, minutes, eastbound) in
        [(FRA, NRT, 13 * 60, true), (FRA, JFK, 8 * 60 + 30, false)]
    {
        let points = plan_with(from, to, minutes, geometric());
        let ceiling = points.iter().fold(0.0_f64, |m, p| m.max(p.altitude));
        // The rendered path is lifted a few metres clear of the globe surface.
        let feet = m_to_feet(ceiling - 5.0);
        let thousands = (feet / 1000.0).round();
        assert!(
            (feet - thousands * 1000.0).abs() < 60.0,
            "cruise ceiling {feet:.0} ft is not on a flight level"
        );
        let is_odd = (thousands as i64).rem_euclid(2) == 1;
        assert_eq!(
            is_odd,
            eastbound,
            "FL{:.0} has the wrong parity for this direction",
            thousands * 10.0
        );
    }
}

#[test]
fn a_long_flight_steps_up_through_the_cruise() {
    // An aircraft cannot reach its best altitude while full of fuel, so it starts lower
    // and climbs as it burns off. A flat long-haul altitude trace is wrong.
    let points = plan_with(FRA, NRT, 13 * 60, geometric());
    // Sampling by fraction of the flight rather than by altitude: a threshold on
    // altitude alone also catches the climb and the descent passing through it, which
    // makes early and late cruise look identical.
    let at_fraction = |f: f64| points[((points.len() - 1) as f64 * f) as usize].altitude;
    let early = at_fraction(0.25);
    let late = at_fraction(0.80);
    assert!(
        late > early + 400.0,
        "cruise was at {early:.0} m a quarter of the way in and {late:.0} m later — \
         no step climb"
    );

    // And every step goes up. A descent mid-cruise would mean the staircase was built
    // backwards.
    let lo = points.len() / 5;
    let hi = points.len() * 4 / 5;
    let worst_drop = points[lo..hi]
        .windows(2)
        .map(|w| w[1].altitude - w[0].altitude)
        .fold(0.0_f64, f64::min);
    assert!(worst_drop > -30.0, "cruise dropped {worst_drop:.0} m");
}

#[test]
fn a_short_sector_cruises_lower_than_a_long_one() {
    let short = plan_with(FRA, STR, 55, geometric());
    let long = plan_with(FRA, JFK, 8 * 60 + 30, geometric());
    let ceiling = |p: &[TelemetryPoint]| p.iter().fold(0.0_f64, |m, q| m.max(q.altitude));
    assert!(
        ceiling(&short) < ceiling(&long),
        "short sector reached {:.0} m, long sector {:.0} m",
        ceiling(&short),
        ceiling(&long)
    );
}

#[test]
fn the_vertical_profile_is_c1_continuous_at_all_knots() {
    for (_name, dep_elev, arr_elev, dist, track_rad) in [
        ("Short FRA-STR", 111.0, 389.0, 160_000.0, 3.14),
        ("Medium FRA-JFK", 111.0, 4.0, 6_300_000.0, 5.0),
        ("Long LHR-NRT", 25.0, 43.0, 9_600_000.0, 0.5),
        ("High dep", 2500.0, 100.0, 1_000_000.0, 1.2),
        ("High arr", 100.0, 2500.0, 1_000_000.0, 4.0),
    ] {
        let inputs = VerticalInputs {
            s_total: dist,
            s_dep_threshold: 500.0,
            s_touchdown: dist - 1500.0,
            s_rollout_end: dist - 300.0,
            dep_elevation_m: dep_elev,
            arr_elevation_m: arr_elev,
            track_rad,
            trip_distance_m: dist,
            cruise_tas_estimate: 240.0,
        };
        let profile = VerticalProfile::build(&inputs);
        let samples = profile.samples();
        let eps = 1e-4_f64;

        for (i, &(s, _alt)) in samples.iter().enumerate() {
            if i == 0 || i == samples.len() - 1 {
                continue;
            }
            let prev_s = samples[i - 1].0;
            let next_s = samples[i + 1].0;
            let h_prev = s - prev_s;
            let h_next = next_s - s;
            if h_prev < 1e-5 || h_next < 1e-5 {
                continue;
            }
            let cur_eps = eps.min(h_prev * 0.1).min(h_next * 0.1);
            let d_left = (profile.altitude_at(s) - profile.altitude_at(s - cur_eps)) / cur_eps;
            let d_right = (profile.altitude_at(s + cur_eps) - profile.altitude_at(s)) / cur_eps;
            let jump = (d_right - d_left).abs();
            assert!(
                jump < 1e-3,
                "C1 discontinuity of {:.6} at knot {} (s={:.1}m)",
                jump,
                i,
                s
            );
        }
    }
}

#[test]
fn the_monotone_limiter_prevents_altitude_overshoot() {
    for (_name, dep_elev, arr_elev, dist, track_rad) in [
        ("Short FRA-STR", 111.0, 389.0, 160_000.0, 3.14),
        ("Medium FRA-JFK", 111.0, 4.0, 6_300_000.0, 5.0),
        ("Long LHR-NRT", 25.0, 43.0, 9_600_000.0, 0.5),
        ("High dep", 2500.0, 100.0, 1_000_000.0, 1.2),
        ("High arr", 100.0, 2500.0, 1_000_000.0, 4.0),
    ] {
        let inputs = VerticalInputs {
            s_total: dist,
            s_dep_threshold: 500.0,
            s_touchdown: dist - 1500.0,
            s_rollout_end: dist - 300.0,
            dep_elevation_m: dep_elev,
            arr_elevation_m: arr_elev,
            track_rad,
            trip_distance_m: dist,
            cruise_tas_estimate: 240.0,
        };
        let profile = VerticalProfile::build(&inputs);
        let samples = profile.samples();

        // Check monotonicity on every interval
        for w in samples.windows(2) {
            let (s0, a0) = w[0];
            let (s1, a1) = w[1];
            let min_a = a0.min(a1);
            let max_a = a0.max(a1);

            for step in 1..20 {
                let frac = step as f64 / 20.0;
                let s = s0 + (s1 - s0) * frac;
                let alt = profile.altitude_at(s);
                assert!(
                    alt <= max_a + 1e-6,
                    "overshoot: alt {:.2}m > interval max {:.2}m at s={:.1}m",
                    alt,
                    max_a,
                    s
                );
                assert!(
                    alt >= min_a - 1e-6,
                    "undershoot: alt {:.2}m < interval min {:.2}m at s={:.1}m",
                    alt,
                    min_a,
                    s
                );
            }
        }

        // Check global ceiling and floor bounds
        let floor = dep_elev.min(arr_elev);
        let ceiling = profile.cruise_altitude_m;
        for step in 0..=2000 {
            let s = dist * (step as f64 / 2000.0);
            let alt = profile.altitude_at(s);
            assert!(
                alt <= ceiling + 1e-6,
                "altitude {:.2}m exceeded cruise ceiling {:.2}m at s={:.1}m",
                alt,
                ceiling,
                s
            );
            assert!(
                alt >= floor - 1e-6,
                "altitude {:.2}m fell below terrain floor {:.2}m at s={:.1}m",
                alt,
                floor,
                s
            );
        }
    }
}

// ---------------------------------------------------------------------------
// Speeds
// ---------------------------------------------------------------------------

#[test]
fn cruise_speed_is_physical_on_every_length_of_route() {
    // The old planner solved for whatever cruise speed made the session arithmetic
    // work, which put the median flight at 595 km/h and the shortest below stall speed.
    for (from, to, minutes) in [
        (FRA, STR, 55),
        (FRA, JFK, 8 * 60 + 30),
        (LHR, NRT, 15 * 60),
        (MEL, JFK, 21 * 60),
    ] {
        let points = plan(from, to, minutes);
        let cruise = cruise_points(&points);
        let mean = cruise.iter().map(|p| p.velocity_m_s).sum::<f64>() / cruise.len() as f64;
        assert!(
            (170.0..320.0).contains(&mean),
            "cruise ground speed {:.0} m/s ({:.0} km/h) is not an airliner's",
            mean,
            mean * 3.6
        );
    }
}

#[test]
fn the_flight_lasts_exactly_as_long_as_the_session() {
    for minutes in [45_u64, 150, 480, 780] {
        let points = plan(FRA, JFK, minutes);
        let actual = points.last().unwrap().time_offset_ms as f64 / 60_000.0;
        assert!(
            (actual - minutes as f64).abs() < 1.0,
            "session of {minutes} min produced a {actual:.1} min flight"
        );
    }
}

#[test]
fn time_advances_monotonically() {
    let points = plan(FRA, JFK, 8 * 60 + 30);
    for w in points.windows(2) {
        assert!(
            w[1].time_offset_ms >= w[0].time_offset_ms,
            "time went backwards"
        );
    }
}

// ---------------------------------------------------------------------------
// Attitude
// ---------------------------------------------------------------------------

#[test]
fn the_track_never_demands_more_bank_than_the_aircraft_may_use() {
    // The reported roll is clamped to the limit, so asserting on it alone would pass
    // even if the geometry called for 50°. This measures the curvature of the sampled
    // track instead and asks what bank flying it would actually require.
    use cesium_flight::telemetry::geo::{initial_bearing, wrap_pi};
    let position = |p: &TelemetryPoint| LatLon::new(p.latitude, p.longitude);

    for (from, to, minutes) in [
        (FRA, JFK, 8 * 60 + 30),
        (LHR, NRT, 15 * 60),
        (SVO, LED, 95),
        (MEL, JFK, 21 * 60),
    ] {
        let points = plan(from, to, minutes);
        let airborne: Vec<&TelemetryPoint> = points.iter().filter(|p| p.altitude > 200.0).collect();
        let mut worst: f64 = 0.0;
        for w in airborne.windows(3) {
            let d1 = distance_m(position(w[0]), position(w[1]));
            let d2 = distance_m(position(w[1]), position(w[2]));
            if d1 < 50.0 || d2 < 50.0 {
                continue;
            }
            // Both directions measured at the middle point, so meridian convergence
            // cancels. Differencing the two legs' initial bearings instead would report
            // a hard turn wherever a straight great circle passes near a pole.
            let arriving = initial_bearing(position(w[1]), position(w[0])) + std::f64::consts::PI;
            let leaving = initial_bearing(position(w[1]), position(w[2]));
            let turn = wrap_pi(leaving - arriving).abs();
            let v = w[1].velocity_m_s;
            // bank = atan(v^2 / (g * r)), with r = arc length / turn angle.
            let required = (v * v * turn / (9.806_65 * 0.5 * (d1 + d2))).atan();
            worst = worst.max(required.to_degrees());
        }
        assert!(
            worst < 30.0,
            "{:?}->{:?} needs {:.1}° of bank somewhere",
            from,
            to,
            worst
        );
    }
}

#[test]
fn bank_stays_within_an_airliners_limits() {
    let points = plan(LHR, NRT, 15 * 60);
    let steepest = points
        .iter()
        .map(|p| p.roll_rad.abs())
        .fold(0.0_f64, f64::max);
    assert!(
        steepest.to_degrees() < 26.0,
        "banked to {:.1}°",
        steepest.to_degrees()
    );
}

#[test]
fn the_aircraft_is_nose_up_on_final_approach() {
    // Pitch is the flight path angle *plus* the angle of attack. On a 3° glideslope
    // that still leaves the nose above the horizon, which is why deriving pitch from
    // the altitude gradient alone put it visibly wrong way up.
    let points = plan(FRA, JFK, 8 * 60 + 30);
    // Walking back from the end: skip the rollout and taxi, then take the final
    // approach up to about 2,000 ft.
    let approach: Vec<&TelemetryPoint> = points
        .iter()
        .rev()
        .skip_while(|p| p.altitude < 60.0)
        .take_while(|p| p.altitude < 600.0)
        .collect();
    assert!(!approach.is_empty());
    let mean_pitch = approach.iter().map(|p| p.pitch_rad).sum::<f64>() / approach.len() as f64;
    assert!(
        mean_pitch > 0.0,
        "mean pitch on final was {:.2}° — nose down",
        mean_pitch.to_degrees()
    );
    assert!(
        mean_pitch.to_degrees() < 10.0,
        "mean pitch on final was {:.2}°",
        mean_pitch.to_degrees()
    );
}

#[test]
fn the_aircraft_is_level_before_rotation() {
    // Everything from the apron to the moment the nose comes up: wheels on the runway,
    // so no bank and no pitch. Deliberately not "everything near the ground" — during
    // the flare the aircraft is a couple of metres up and several degrees nose high,
    // which is what a landing looks like.
    let points = plan(FRA, STR, 55);
    let ground_altitude = points[0].altitude;
    let before_rotation: Vec<&TelemetryPoint> = points
        .iter()
        .take_while(|p| (p.altitude - ground_altitude).abs() < 0.5)
        .collect();
    assert!(
        before_rotation.len() > 10,
        "expected a taxi and takeoff roll, got {} samples",
        before_rotation.len()
    );
    for p in before_rotation {
        assert!(
            p.roll_rad.abs().to_degrees() < 0.01,
            "banked {:.3}° on the ground",
            p.roll_rad.to_degrees()
        );
        // Not exactly zero: rotation is now a smooth pitch-up rather than a step, so the
        // first few centimetres of the climb still fall inside the "at ground level"
        // band this filters on. A regression of the kind this guards against would be
        // degrees, not hundredths of one.
        assert!(
            p.pitch_rad.abs().to_degrees() < 0.1,
            "pitched {:.3}° before rotation",
            p.pitch_rad.to_degrees()
        );
    }

    // And parked at the far end.
    let parked = points.last().unwrap();
    assert!(parked.roll_rad.abs().to_degrees() < 0.01);
    assert!(parked.pitch_rad.abs().to_degrees() < 0.01);
}

#[test]
fn the_nose_comes_up_for_landing() {
    // The flare: the aircraft arrives nose-high, not flat.
    let points = plan(FRA, STR, 55);
    let ground_altitude = points.last().unwrap().altitude;
    let flare: Vec<&TelemetryPoint> = points
        .iter()
        .rev()
        .skip_while(|p| (p.altitude - ground_altitude).abs() < 0.5)
        .take(6)
        .collect();
    assert!(!flare.is_empty());
    let highest = flare
        .iter()
        .map(|p| p.pitch_rad.to_degrees())
        .fold(f64::MIN, f64::max);
    assert!(
        (1.0..12.0).contains(&highest),
        "touchdown attitude was {highest:.1}°"
    );
}

// ---------------------------------------------------------------------------
// Runway geometry
// ---------------------------------------------------------------------------

fn runway(airport: (f64, f64), heading_deg: f64, length_m: f64) -> RunwayData {
    let end =
        cesium_flight::telemetry::geo::destination(at(airport), heading_deg.to_radians(), length_m);
    RunwayData {
        airport_id: 1,
        length_ft: (length_m / 0.3048) as f32,
        width_ft: 150.0,
        le_heading: heading_deg as f32,
        le_lat: airport.0,
        le_lon: airport.1,
        he_heading: ((heading_deg + 180.0) % 360.0) as f32,
        he_lat: end.lat_deg,
        he_lon: end.lon_deg,
    }
}

#[test]
fn touchdown_happens_past_the_threshold_not_short_of_it() {
    // The old profile put the aircraft at zero altitude three kilometres before the
    // runway and then taxied it to the threshold.
    use cesium_flight::telemetry::runway as runway_select;
    use cesium_flight::telemetry::wind::WindField;

    let arrival = STR;
    let runways = vec![runway(arrival, 70.0, 3_000.0)];
    let points = generate(&FlightRequest {
        departure: at(FRA),
        arrival: at(arrival),
        target_duration_ms: 55 * 60_000,
        dep_heading_deg: None,
        arr_heading_deg: None,
        runways: runways.clone(),
        config: FlightPlanConfig::default(),
    });

    // Measured against the end actually in use, which the wind picks — not against the
    // airport reference point, which may be at the other end of the runway.
    let threshold =
        runway_select::select(at(arrival), &runways, &WindField::annual_mean(), 0.0).threshold;

    // Find where the aircraft first reaches the ground on arrival.
    let touchdown = points
        .iter()
        .rev()
        .take_while(|p| p.altitude < 8.0)
        .last()
        .expect("should reach the ground");
    let td = LatLon::new(touchdown.latitude, touchdown.longitude);
    let from_threshold = distance_m(td, threshold);
    assert!(
        from_threshold < 1_200.0,
        "touched down {from_threshold:.0} m from the threshold"
    );

    // And it is *beyond* the threshold, on the runway, not short of it: the remaining
    // ground track from touchdown must run away from the approach direction.
    let rollout_end = points.last().unwrap();
    let rollout = distance_m(
        LatLon::new(rollout_end.latitude, rollout_end.longitude),
        threshold,
    );
    assert!(
        rollout > from_threshold,
        "the aircraft ended up closer to the threshold than it touched down"
    );
}

#[test]
fn the_runway_end_in_use_follows_the_wind() {
    use cesium_flight::telemetry::runway;
    use cesium_flight::telemetry::wind::WindField;

    // A runway aligned east/west in the mid-latitude westerlies should be used facing
    // into them — which is why Heathrow lands to the west most days.
    let rwy = runway(LHR, 90.0, 3_000.0);
    let end = runway::select(at(LHR), &[rwy], &WindField::annual_mean(), 0.0);
    let heading = end.heading_rad.to_degrees().rem_euclid(360.0);
    assert!(
        (180.0..360.0).contains(&heading),
        "chose a heading of {heading:.0}° into a prevailing westerly"
    );
}

#[test]
fn the_longer_runway_wins_when_the_wind_does_not_care() {
    use cesium_flight::telemetry::runway;
    use cesium_flight::telemetry::wind::WindField;

    let short = runway(FRA, 90.0, 1_800.0);
    let long = runway(FRA, 90.0, 4_000.0);
    let end = runway::select(at(FRA), &[short, long], &WindField::calm(), 0.0);
    assert!(end.length_m > 3_000.0, "picked the shorter runway");
}

// ---------------------------------------------------------------------------
// Terrain elevation, which stays off until the globe can render it
// ---------------------------------------------------------------------------

#[test]
fn field_elevation_is_ignored_by_default() {
    let points = plan_with(FRA, STR, 55, FlightPlanConfig::default());
    assert!(
        points[0].altitude < 20.0,
        "started at {:.0} m with terrain disabled",
        points[0].altitude
    );
}

#[test]
fn field_elevation_is_honoured_when_enabled() {
    let config = FlightPlanConfig {
        terrain_elevation: true,
        dep_elevation_m: 2_548.0, // Bogota
        arr_elevation_m: 1_640.0, // Mexico City
        ..FlightPlanConfig::default()
    };
    let points = plan_with((4.7016, -74.1469), (19.4363, -99.0721), 4 * 60 + 30, config);
    assert!(
        (points[0].altitude - 2_548.0).abs() < 30.0,
        "started at {:.0} m rather than the field elevation",
        points[0].altitude
    );
    assert!(
        (points.last().unwrap().altitude - 1_640.0).abs() < 30.0,
        "ended at {:.0} m rather than the field elevation",
        points.last().unwrap().altitude
    );
    // Cruise still has to clear both fields by a sensible margin.
    let ceiling = points.iter().fold(0.0_f64, |m, p| m.max(p.altitude));
    assert!(ceiling > 2_548.0 + 1_500.0, "cruised at {ceiling:.0} m");
}

#[test]
fn a_high_field_needs_a_longer_ground_roll() {
    use cesium_flight::telemetry::aircraft;
    // Rotation happens at a fixed calibrated airspeed, so thin air means a faster true
    // speed and a longer roll to reach it.
    assert!(
        aircraft::ground_roll_distance(2_548.0) > aircraft::ground_roll_distance(0.0) * 1.15,
        "field elevation should lengthen the takeoff roll"
    );
}

// ---------------------------------------------------------------------------
// Route Presets & Coordinate Parsing
// ---------------------------------------------------------------------------

#[test]
fn test_preset_lookup_and_case_insensitivity() {
    use cesium_flight::preset::parse_route;

    let fra_str = parse_route("FRA-STR").expect("FRA-STR should parse");
    assert_eq!(fra_str.id, "FRA-STR");
    assert!((fra_str.departure_lon - 8.5706).abs() < 0.001);
    assert!((fra_str.departure_lat - 50.0333).abs() < 0.001);

    // Case insensitive and supports underscore
    let lhr_nrt = parse_route("lhr_nrt").expect("lhr_nrt should parse");
    assert_eq!(lhr_nrt.id, "LHR-NRT");

    let jfk_lhr = parse_route("jfk-lhr").expect("jfk-lhr should parse");
    assert_eq!(jfk_lhr.id, "JFK-LHR");
}

#[test]
fn test_coordinate_parsing_lat_lon_and_duration() {
    use cesium_flight::preset::parse_route;

    // Lat, Lon, Lat, Lon format
    let res = parse_route("50.0333, 8.5706, 48.6899, 9.2219").expect("coords should parse");
    assert!((res.departure_lat - 50.0333).abs() < 1e-4);
    assert!((res.departure_lon - 8.5706).abs() < 1e-4);
    assert!((res.arrival_lat - 48.6899).abs() < 1e-4);
    assert!((res.arrival_lon - 9.2219).abs() < 1e-4);
    assert!(res.total_duration_ms >= 1_800_000); // at least 30 mins

    // Lon, Lat, Lon, Lat format (detected by abs(lon) > 90)
    let res2 = parse_route("-122.3790, 37.6213, -157.9224, 21.3187").expect("lon-first coords should parse");
    assert!((res2.departure_lon - -122.3790).abs() < 1e-4);
    assert!((res2.departure_lat - 37.6213).abs() < 1e-4);

    // Custom duration in minutes (5th parameter)
    let res3 = parse_route("50.0, 8.5, 48.6, 9.2, 45").expect("coords with duration should parse");
    assert_eq!(res3.total_duration_ms, 45 * 60 * 1000);
}

#[test]
fn test_ground_roll_straight_and_bank_zero_on_ground() {
    use cesium_flight::preset::parse_route;

    for route_id in &["ZRH-GVA", "FRA-STR"] {
        let def = parse_route(route_id).unwrap();
        let req = FlightRequest {
            departure: LatLon::new(def.departure_lat, def.departure_lon),
            arrival: LatLon::new(def.arrival_lat, def.arrival_lon),
            target_duration_ms: def.total_duration_ms,
            dep_heading_deg: def.dep_heading_deg,
            arr_heading_deg: def.arr_heading_deg,
            runways: Vec::new(),
            config: FlightPlanConfig::default(),
        };
        let points = generate(&req);
        let mut ground_heading_changes = 0;
        let mut max_ground_bank = 0.0_f64;
        let ground_pts: Vec<&TelemetryPoint> =
            points.iter().filter(|p| p.altitude <= 5.001).collect();
        for w in ground_pts.windows(2) {
            if (w[1].time_offset_ms - w[0].time_offset_ms) < 30_000 {
                let diff = (w[1].heading_rad - w[0].heading_rad).abs();
                if diff > 1e-4 {
                    ground_heading_changes += 1;
                }
            }
        }
        for p in &ground_pts {
            max_ground_bank = max_ground_bank.max(p.roll_rad.abs());
        }
        assert_eq!(
            ground_heading_changes, 0,
            "route {route_id} must have 0 heading changes while rolling on ground"
        );
        assert_eq!(
            max_ground_bank, 0.0,
            "route {route_id} must have 0 bank angle while rolling on ground"
        );
    }
}

#[test]
fn test_print_sin_lhr() {
    use cesium_flight::preset::parse_route;
    let def = parse_route("SIN-LHR").unwrap();
    let req = FlightRequest {
        departure: LatLon::new(def.departure_lat, def.departure_lon),
        arrival: LatLon::new(def.arrival_lat, def.arrival_lon),
        target_duration_ms: def.total_duration_ms,
        dep_heading_deg: def.dep_heading_deg,
        arr_heading_deg: def.arr_heading_deg,
        runways: Vec::new(),
        config: FlightPlanConfig::default(),
    };
    let wind = cesium_flight::telemetry::wind::WindField::annual_mean();
    let airspace = cesium_flight::telemetry::airspace::AirspaceRestrictions::for_route(req.departure, req.arrival);
    let enroute = cesium_flight::telemetry::lateral::plan_enroute(
        req.departure,
        req.arrival,
        &cesium_flight::telemetry::lateral::EnrouteOptions {
            wind: &wind,
            airspace: &airspace,
            cruise_altitude_m: 11_000.0,
            cruise_tas: 240.0,
            oceanic_tracks: false,
        },
    );
    println!("=== SIN-LHR enroute waypoints ({}) ===", enroute.len());
    for (i, p) in enroute.iter().enumerate() {
        if i > 0 {
            let prev = enroute[i - 1];
            let d = cesium_flight::telemetry::geo::distance_m(prev, *p);
            let b = cesium_flight::telemetry::geo::initial_bearing(prev, *p).to_degrees();
            println!("  wp[{:2}]: lat={:8.4}, lon={:8.4} | leg: {:.1} km, bearing: {:.1}°", i, p.lat_deg, p.lon_deg, d / 1000.0, b);
        } else {
            println!("  wp[{:2}]: lat={:8.4}, lon={:8.4}", i, p.lat_deg, p.lon_deg);
        }
    }

    let pts = cesium_flight::telemetry::generator::generate(&req);
    println!("=== Telemetry generated: {} points ===", pts.len());

    let mut tel_violations = 0;
    for (i, p) in pts.iter().enumerate() {
        let geo = LatLon::new(p.latitude, p.longitude);
        if airspace.blocks(geo) {
            tel_violations += 1;
            println!("  Telemetry point {} blocked: lat={:.4}, lon={:.4}", i, p.latitude, p.longitude);
        }
    }
    println!("=== Telemetry violations: {} ===", tel_violations);
    assert_eq!(tel_violations, 0, "telemetry points must not penetrate restricted airspace");

    use cesium_engine::property::Property;
    let mut prop = cesium_engine::property::sampled::SampledPositionProperty::new()
        .with_algorithm(cesium_engine::property::sampled::InterpolationAlgorithm::CatmullRom);
    for pt in &pts {
        let ecef = cesium_engine::globe::geometry::lon_lat_alt_to_ecef_f64(pt.longitude, pt.latitude, pt.altitude);
        let pos = glam::DVec3::from_array(ecef);
        let t = cesium_engine::time::SimulationTime::new(pt.time_offset_ms as f64 / 1000.0);
        prop.add_sample(t, pos);
    }
    let builder = cesium_engine::render::polyline_pipeline::builder::AdaptiveSubdivisionBuilder::new(1e-7);
    let start_t = prop.start_time().unwrap();
    let ref_pt = prop.evaluate(start_t).unwrap();
    let cps = builder.build(&prop, ref_pt);
    println!("=== Control points generated: {} cps ===", cps.len());
}
