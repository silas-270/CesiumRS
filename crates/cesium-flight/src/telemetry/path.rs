//! The ground track: a waypoint list turned into something with an arc length.
//!
//! Waypoints are joined by great-circle legs and rounded at the corners by circular
//! arcs of a radius the aircraft can actually fly. That rounding is a *fly-by* turn,
//! which is what a flight management computer does at almost every waypoint: it starts
//! turning before the fix and rolls out after it, never passing over the fix itself.
//!
//! The turn radius is what the old code got wrong, and it is worth being explicit about
//! why. A coordinated turn holds `r = v^2 / (g tan phi)`. Fixing the radius therefore
//! fixes nothing useful — it fixes the *bank angle to whatever the speed implies*, and
//! at cruise speed a four-kilometre radius implies 58° of bank. Radius follows from
//! speed and a bank limit here, never the other way round.
//!
//! The result is sampled into a dense polyline with cumulative distances, so the rest
//! of the planner can ask "where am I at 4,182 km along" without solving any spherical
//! geometry.

use super::airspace::AirspaceRestrictions;
use super::geo::{
    angular_distance, initial_bearing, interpolate, LatLon, LocalFrame, EARTH_RADIUS_M,
};

/// High-level parameters defining a complete end-to-end flight track.
pub struct FlightTrackInputs<'a> {
    pub dep_threshold: LatLon,
    pub dep_leg_end: LatLon,
    pub departure_turn_r: f64,
    pub enroute: &'a [LatLon],
    pub final_start: LatLon,
    pub final_turn_r: f64,
    pub arr_threshold: LatLon,
    pub touchdown: LatLon,
    pub rollout_end: LatLon,
    pub airspace: Option<&'a AirspaceRestrictions>,
}

/// Cumulative track distances where key flight plan events are placed.
pub struct FlightTrackPlaced {
    pub s_dep_threshold: f64,
    pub s_touchdown: f64,
    pub s_rollout_end: f64,
}

/// A point the route is planned through.
#[derive(Debug, Clone, Copy)]
pub struct Waypoint {
    pub position: LatLon,
    /// Radius of the fly-by turn here. Zero leaves the corner sharp, which is what the
    /// runway waypoints want — they are collinear anyway.
    pub turn_radius_m: f64,
}

impl Waypoint {
    pub fn new(position: LatLon, turn_radius_m: f64) -> Self {
        Self {
            position,
            turn_radius_m,
        }
    }

    pub fn sharp(position: LatLon) -> Self {
        Self {
            position,
            turn_radius_m: 0.0,
        }
    }
}

/// Largest share of a leg a single turn may consume. Two adjacent corners can therefore
/// take 80% of the leg between them and still leave a straight portion.
const MAX_TURN_LEG_FRACTION: f64 = 0.4;

/// Turns beyond this are split into two, rather than flown as one very tight arc.
///
/// The threshold is set by geometry rather than taste. A turn needs `r·tan(θ/2)` of
/// straight leg either side of it, and the departure leg is 12 km of which a turn may
/// use 40%; past about 100° that runs out, the arc tightens to fit, and the bank the
/// aircraft would need exceeds what it is allowed to use. Splitting keeps every turn
/// flyable — and a departure that reverses course really is flown as two turns joined
/// by a short leg, which is what a procedure turn is.
const SPLIT_TURN_THRESHOLD_RAD: f64 = 1.75; // 100 degrees

/// Densification. Legs get at least this many samples, and no sample is further apart
/// than the coarse limit or closer than the fine one.
const SAMPLES_PER_LEG: f64 = 16.0;
const MAX_SAMPLE_SPACING_M: f64 = 4_000.0;
const MIN_SAMPLE_SPACING_M: f64 = 25.0;

/// Arc resolution: at most this much turn, and this much distance, between samples.
///
/// Has to stay comfortably finer than the telemetry sampler's own step, which gets down
/// to about 100 m near the ground. When the two are comparable, consecutive telemetry
/// samples straddle facet junctions of the arc and read the turn as happening in
/// instalments — which looks like a turn twice as tight as the one being flown.
const ARC_SAMPLE_ANGLE_RAD: f64 = 0.0087; // 0.5 degrees
const ARC_SAMPLE_SPACING_M: f64 = 100.0;
const MIN_ARC_SAMPLES: usize = 12;
const MAX_ARC_SAMPLES: usize = 720;

/// Half-width of the window used to measure track direction.
const BEARING_WINDOW_M: f64 = 400.0;

pub struct GroundTrack {
    points: Vec<LatLon>,
    cumulative: Vec<f64>,
}

impl GroundTrack {
    /// Builds the track, and reports where each input waypoint ended up along it.
    ///
    /// The reported distances matter: the vertical profile is anchored to the runway
    /// threshold and the touchdown point, and a fly-by turn shortens the path, so the
    /// nominal leg lengths are not where those waypoints actually land.
    pub fn build(waypoints: &[Waypoint]) -> (Self, Vec<f64>) {
        let expanded = split_sharp_turns(waypoints);
        let n = expanded.len();

        let mut points: Vec<LatLon> = Vec::new();
        let mut cumulative: Vec<f64> = Vec::new();
        // Distance of each expanded waypoint, or NaN where it has not been placed yet.
        let mut placed = vec![f64::NAN; n];

        if n == 0 {
            return (Self { points, cumulative }, Vec::new());
        }
        if n == 1 {
            points.push(expanded[0].waypoint.position);
            cumulative.push(0.0);
            return (Self { points, cumulative }, vec![0.0]);
        }

        push_point(&mut points, &mut cumulative, expanded[0].waypoint.position);
        placed[0] = 0.0;

        let mut leg_start = expanded[0].waypoint.position;
        for i in 1..n - 1 {
            let arc = build_corner(
                expanded[i - 1].waypoint.position,
                expanded[i].waypoint,
                expanded[i + 1].waypoint.position,
            );
            match arc {
                Some(arc) => {
                    densify_leg(&mut points, &mut cumulative, leg_start, arc.start);
                    let mid = arc.samples.len() / 2;
                    for (k, p) in arc.samples.iter().enumerate() {
                        push_point(&mut points, &mut cumulative, *p);
                        if k == mid {
                            placed[i] = *cumulative.last().unwrap();
                        }
                    }
                    leg_start = arc.end;
                }
                None => {
                    // No turn worth rounding: the waypoint sits on the path.
                    densify_leg(
                        &mut points,
                        &mut cumulative,
                        leg_start,
                        expanded[i].waypoint.position,
                    );
                    placed[i] = *cumulative.last().unwrap();
                    leg_start = expanded[i].waypoint.position;
                }
            }
        }

        densify_leg(
            &mut points,
            &mut cumulative,
            leg_start,
            expanded[n - 1].waypoint.position,
        );
        placed[n - 1] = *cumulative.last().unwrap();

        // Map back onto the caller's waypoint list, dropping any inserted for a split.
        let mut original = vec![0.0; waypoints.len()];
        for (i, e) in expanded.iter().enumerate() {
            if let Some(orig) = e.original_index {
                original[orig] = placed[i];
            }
        }

        (Self { points, cumulative }, original)
    }

    /// Builds the flight track using a C¹ continuous spherical spline for the enroute portion
    /// and exact straight legs for runway departure, final approach, and rollout.
    pub fn build_flight_track(inputs: &FlightTrackInputs) -> (Self, FlightTrackPlaced) {
        let mut raw_wps = Vec::with_capacity(inputs.enroute.len() + 6);
        raw_wps.push(Waypoint::sharp(inputs.dep_threshold));
        raw_wps.push(Waypoint::new(inputs.dep_leg_end, inputs.departure_turn_r));
        if inputs.enroute.len() > 2 {
            for p in &inputs.enroute[1..inputs.enroute.len() - 1] {
                raw_wps.push(Waypoint::new(*p, 120_000.0));
            }
        }
        let orig_idx_final_start = raw_wps.len();
        raw_wps.push(Waypoint::new(inputs.final_start, inputs.final_turn_r));
        let orig_idx_arr_thresh = raw_wps.len();
        raw_wps.push(Waypoint::sharp(inputs.arr_threshold));
        raw_wps.push(Waypoint::sharp(inputs.touchdown));
        raw_wps.push(Waypoint::sharp(inputs.rollout_end));

        let expanded = split_sharp_turns(&raw_wps);

        let mut points: Vec<LatLon> = Vec::new();
        let mut cumulative: Vec<f64> = Vec::new();

        push_point(&mut points, &mut cumulative, inputs.dep_threshold);
        let s_dep_threshold = 0.0;

        let exp_arr_start = expanded
            .iter()
            .position(|e| e.original_index == Some(orig_idx_final_start))
            .unwrap();
        let exp_arr_thresh = expanded
            .iter()
            .position(|e| e.original_index == Some(orig_idx_arr_thresh))
            .unwrap();
        let exp_enroute_start = if inputs.enroute.len() > 2 {
            expanded
                .iter()
                .position(|e| e.original_index == Some(2))
                .unwrap_or(exp_arr_start)
        } else {
            exp_arr_start
        };

        // 1. Departure turn(s)
        let mut leg_start = inputs.dep_threshold;
        for i in 1..exp_enroute_start {
            if let Some(arc) = build_corner(
                expanded[i - 1].waypoint.position,
                expanded[i].waypoint,
                expanded[i + 1].waypoint.position,
            ) {
                densify_leg(&mut points, &mut cumulative, leg_start, arc.start);
                for p in arc.samples {
                    push_point(&mut points, &mut cumulative, p);
                }
                leg_start = arc.end;
            }
        }
        let enroute_start = leg_start;

        // 2. Pre-calculate arrival turn(s)
        let mut arr_arcs = Vec::new();
        for i in exp_arr_start..exp_arr_thresh {
            let prev = expanded[i - 1].waypoint.position;
            let next = expanded[i + 1].waypoint.position;
            let arc = build_corner(prev, expanded[i].waypoint, next);
            arr_arcs.push(arc);
        }

        let enroute_end = match arr_arcs.first() {
            Some(Some(arc)) => arc.start,
            _ => inputs.final_start,
        };

        // 3. Enroute path
        if inputs.enroute.len() <= 2 {
            densify_leg(&mut points, &mut cumulative, enroute_start, enroute_end);
        } else {
            let mut guide: Vec<LatLon> = Vec::with_capacity(inputs.enroute.len());
            guide.push(enroute_start);
            for p in &inputs.enroute[1..inputs.enroute.len() - 1] {
                guide.push(*p);
            }
            guide.push(enroute_end);
            densify_spherical_spline(&mut points, &mut cumulative, &guide, inputs.airspace);
        }

        // 4. Arrival turn(s)
        let mut arr_leg_start = enroute_end;
        for (idx, arc_opt) in arr_arcs.into_iter().enumerate() {
            let i = exp_arr_start + idx;
            match arc_opt {
                Some(arc) => {
                    densify_leg(&mut points, &mut cumulative, arr_leg_start, arc.start);
                    for p in arc.samples {
                        push_point(&mut points, &mut cumulative, p);
                    }
                    arr_leg_start = arc.end;
                }
                None => {
                    densify_leg(
                        &mut points,
                        &mut cumulative,
                        arr_leg_start,
                        expanded[i].waypoint.position,
                    );
                    arr_leg_start = expanded[i].waypoint.position;
                }
            }
        }

        // 5. Final approach, touchdown & rollout
        densify_leg(&mut points, &mut cumulative, arr_leg_start, inputs.arr_threshold);
        densify_leg(&mut points, &mut cumulative, inputs.arr_threshold, inputs.touchdown);
        let s_touchdown = *cumulative.last().unwrap_or(&0.0);
        densify_leg(&mut points, &mut cumulative, inputs.touchdown, inputs.rollout_end);
        let s_rollout_end = *cumulative.last().unwrap_or(&0.0);

        (
            Self { points, cumulative },
            FlightTrackPlaced {
                s_dep_threshold,
                s_touchdown,
                s_rollout_end,
            },
        )
    }

    pub fn total_length(&self) -> f64 {
        self.cumulative.last().copied().unwrap_or(0.0)
    }

    /// Position at a distance along the track.
    pub fn position_at(&self, s: f64) -> LatLon {
        if self.points.is_empty() {
            return LatLon::new(0.0, 0.0);
        }
        let total = self.total_length();
        let s = s.clamp(0.0, total);
        let idx = match self
            .cumulative
            .binary_search_by(|c| c.partial_cmp(&s).unwrap())
        {
            Ok(i) => return self.points[i],
            Err(i) => i,
        };
        if idx == 0 {
            return self.points[0];
        }
        if idx >= self.points.len() {
            return *self.points.last().unwrap();
        }
        let s0 = self.cumulative[idx - 1];
        let s1 = self.cumulative[idx];
        let f = if s1 > s0 { (s - s0) / (s1 - s0) } else { 0.0 };
        LatLon::from_unit(interpolate(
            self.points[idx - 1].to_unit(),
            self.points[idx].to_unit(),
            f,
        ))
    }

    /// How far the track turns over a window centred on `s`, in radians, positive to
    /// the right. This is the quantity bank angle comes from.
    ///
    /// It is *not* the change in compass heading, and the difference matters. Meridians
    /// converge, so an aircraft flying a perfectly straight great circle sees its
    /// heading change continuously — by 180° if it crosses a pole — while banking not at
    /// all. Taking both the inbound and outbound directions *at the same point* makes
    /// that convergence cancel, leaving only the geodesic curvature: zero on a great
    /// circle, at any latitude.
    pub fn turn_angle_at(&self, s: f64, window_m: f64) -> f64 {
        let total = self.total_length();
        if total <= 0.0 {
            return 0.0;
        }
        let w = window_m.min(total / 2.0);
        let here = self.position_at(s);
        let behind = self.position_at((s - w).max(0.0));
        let ahead = self.position_at((s + w).min(total));
        if w < 1.0 {
            return 0.0;
        }
        let arriving = initial_bearing(here, behind) + std::f64::consts::PI;
        let leaving = initial_bearing(here, ahead);
        super::geo::wrap_pi(leaving - arriving)
    }

    /// Track direction at a distance along the track, radians clockwise from north.
    ///
    /// Measured over a window rather than between adjacent samples, so that the
    /// piecewise densification does not show up as a staircase in the heading. This is
    /// a compass heading, for display; bank comes from `turn_angle_at` instead.
    pub fn bearing_at(&self, s: f64) -> f64 {
        let total = self.total_length();
        if total <= 0.0 {
            return 0.0;
        }
        let w = BEARING_WINDOW_M.min(total / 2.0);
        let a = self.position_at((s - w).max(0.0));
        let b = self.position_at((s + w).min(total));
        initial_bearing(a, b)
    }
}

/// A rounded corner.
struct Corner {
    start: LatLon,
    end: LatLon,
    samples: Vec<LatLon>,
}

/// Builds the fly-by arc at `waypoint`, between the legs from `prev` and to `next`.
///
/// Directions and leg lengths are measured on the sphere; only the arc itself, which
/// spans a few kilometres, is constructed on a tangent plane. Projecting the
/// neighbouring waypoints onto that plane instead would work over a short leg and fail
/// badly over a long one — and fail worst near the poles, where a plane anchored at one
/// latitude says nothing useful about a point 300 km away.
fn build_corner(prev: LatLon, waypoint: Waypoint, next: LatLon) -> Option<Corner> {
    if waypoint.turn_radius_m <= 0.0 {
        return None;
    }
    let in_len = super::geo::distance_m(prev, waypoint.position);
    let out_len = super::geo::distance_m(waypoint.position, next);
    if in_len < 1.0 || out_len < 1.0 {
        return None;
    }

    // Direction of travel arriving at and leaving the waypoint, both measured there.
    let bearing_in = initial_bearing(waypoint.position, prev) + std::f64::consts::PI;
    let bearing_out = initial_bearing(waypoint.position, next);
    // Positive to the right as a bearing; the arc below is built in a maths frame where
    // angles run the other way.
    let turn = super::geo::wrap_pi(bearing_out - bearing_in);
    if turn.abs() < 1e-4 {
        return None;
    }
    let theta = -turn;

    let frame = LocalFrame::new(waypoint.position);
    let (ax, ay) = (bearing_in.sin(), bearing_in.cos());
    let (bx, by) = (bearing_out.sin(), bearing_out.cos());

    let half = (theta.abs() / 2.0).tan();
    if half <= 1e-9 {
        return None;
    }
    let wanted = waypoint.turn_radius_m * half;
    let limit = MAX_TURN_LEG_FRACTION * in_len.min(out_len);
    let d = wanted.min(limit);
    if d < 1.0 {
        return None;
    }
    // Tightening the radius is what a real aircraft does when the turn is sharper than
    // the leg length allows: it slows down and banks harder.
    let radius = d / half;

    let start = (-ax * d, -ay * d);
    let end = (bx * d, by * d);
    // Centre lies perpendicular to the inbound track, on the inside of the turn.
    let sign = theta.signum();
    let centre = (start.0 - ay * radius * sign, start.1 + ax * radius * sign);

    let start_angle = (start.1 - centre.1).atan2(start.0 - centre.0);
    // Sampled by *angle* as well as by length. A turn resolved only by arc length
    // becomes a polygon when the radius is small, and the sampler downstream then reads
    // each facet junction as an instantaneous heading change — which looks like a turn
    // far tighter than the one actually being flown.
    let arc_len = radius * theta.abs();
    let by_angle = (theta.abs() / ARC_SAMPLE_ANGLE_RAD).ceil() as usize;
    let by_length = (arc_len / ARC_SAMPLE_SPACING_M).ceil() as usize;
    let steps = by_angle
        .max(by_length)
        .clamp(MIN_ARC_SAMPLES, MAX_ARC_SAMPLES);

    let mut samples = Vec::with_capacity(steps + 1);
    for k in 0..=steps {
        let f = k as f64 / steps as f64;
        let angle = start_angle + theta * f;
        samples.push(frame.to_geo(
            centre.0 + radius * angle.cos(),
            centre.1 + radius * angle.sin(),
        ));
    }

    Some(Corner {
        start: frame.to_geo(start.0, start.1),
        end: frame.to_geo(end.0, end.1),
        samples,
    })
}

/// Appends a point, keeping the cumulative distances in step. Points closer than a
/// micrometre to the previous one are dropped, so legs and arcs can safely repeat their
/// shared endpoints.
fn push_point(points: &mut Vec<LatLon>, cumulative: &mut Vec<f64>, p: LatLon) {
    match points.last() {
        Some(last) => {
            let d = angular_distance(last.to_unit(), p.to_unit()) * EARTH_RADIUS_M;
            if d < 1e-6 {
                return;
            }
            let acc = cumulative.last().copied().unwrap_or(0.0) + d;
            points.push(p);
            cumulative.push(acc);
        }
        None => {
            points.push(p);
            cumulative.push(0.0);
        }
    }
}

fn densify_leg(points: &mut Vec<LatLon>, cumulative: &mut Vec<f64>, from: LatLon, to: LatLon) {
    let v1 = from.to_unit();
    let v2 = to.to_unit();
    let len = angular_distance(v1, v2) * EARTH_RADIUS_M;
    if len < MIN_SAMPLE_SPACING_M {
        push_point(points, cumulative, to);
        return;
    }
    let spacing = (len / SAMPLES_PER_LEG).clamp(MIN_SAMPLE_SPACING_M, MAX_SAMPLE_SPACING_M);
    let steps = (len / spacing).ceil().max(1.0) as usize;
    for k in 0..=steps {
        let f = k as f64 / steps as f64;
        push_point(
            points,
            cumulative,
            LatLon::from_unit(interpolate(v1, v2, f)),
        );
    }
}

/// Dense C¹ spherical Catmull-Rom spline interpolation through a list of guide waypoints.
fn densify_spherical_spline(
    points: &mut Vec<LatLon>,
    cumulative: &mut Vec<f64>,
    guide: &[LatLon],
    airspace: Option<&AirspaceRestrictions>,
) {
    let n = guide.len();
    if n < 2 {
        if n == 1 {
            push_point(points, cumulative, guide[0]);
        }
        return;
    }
    if n == 2 {
        densify_leg(points, cumulative, guide[0], guide[1]);
        return;
    }

    let units: Vec<glam::DVec3> = guide.iter().map(|p| p.to_unit()).collect();

    for i in 0..n - 1 {
        let p0 = if i > 0 {
            units[i - 1]
        } else {
            (units[0] * 2.0 - units[1]).normalize()
        };
        let p1 = units[i];
        let p2 = units[i + 1];
        let p3 = if i + 2 < n {
            units[i + 2]
        } else {
            (units[n - 1] * 2.0 - units[n - 2]).normalize()
        };

        let dt01 = (p1 - p0).length().max(1e-6);
        let dt12 = (p2 - p1).length().max(1e-6);
        let dt23 = (p3 - p2).length().max(1e-6);

        let leg_m = super::geo::distance_m(guide[i], guide[i + 1]);
        const TARGET_STEP_M: f64 = 600.0;
        let steps = ((leg_m / TARGET_STEP_M).ceil() as usize).clamp(12, 500);

        for s in 1..=steps {
            let t = s as f64 / steps as f64;
            let mut pt_unit = cesium_engine::math::interpolation::catmull_rom_timed_dvec3(
                p0, p1, p2, p3, dt01, dt12, dt23, t,
            )
            .normalize();

            if let Some(airspace) = airspace {
                let mut geo = LatLon::from_unit(pt_unit);
                if airspace.blocks(geo) {
                    let chord = interpolate(p1, p2, t);
                    for pull in [0.25, 0.5, 0.75, 1.0] {
                        pt_unit = pt_unit.lerp(chord, pull).normalize();
                        geo = LatLon::from_unit(pt_unit);
                        if !airspace.blocks(geo) {
                            break;
                        }
                    }
                }
            }

            push_point(points, cumulative, LatLon::from_unit(pt_unit));
        }
    }
}

struct Expanded {
    waypoint: Waypoint,
    /// Index into the caller's list, or `None` for a waypoint inserted to split a turn.
    original_index: Option<usize>,
}

/// Splits any corner sharper than the threshold into two gentler ones.
fn split_sharp_turns(waypoints: &[Waypoint]) -> Vec<Expanded> {
    let mut out: Vec<Expanded> = Vec::with_capacity(waypoints.len());
    for (i, wp) in waypoints.iter().enumerate() {
        out.push(Expanded {
            waypoint: *wp,
            original_index: Some(i),
        });
        // Look ahead: a split inserts a waypoint *after* the sharp one.
        if i == 0 || i + 1 >= waypoints.len() || wp.turn_radius_m <= 0.0 {
            continue;
        }
        let prev = waypoints[i - 1].position;
        let next = waypoints[i + 1].position;
        let in_len = super::geo::distance_m(prev, wp.position);
        let out_len = super::geo::distance_m(wp.position, next);
        if in_len < 1.0 || out_len < 1.0 {
            continue;
        }
        let bearing_in = initial_bearing(wp.position, prev) + std::f64::consts::PI;
        let bearing_out = initial_bearing(wp.position, next);
        let turn = super::geo::wrap_pi(bearing_out - bearing_in);
        if turn.abs() <= SPLIT_TURN_THRESHOLD_RAD {
            continue;
        }
        // Fly out on the bisected heading far enough for both halves to be flyable,
        // then turn again onto the outbound leg.
        let reach = (4.0 * wp.turn_radius_m).min(0.35 * out_len);
        out.push(Expanded {
            waypoint: Waypoint::new(
                super::geo::destination(wp.position, bearing_in + turn / 2.0, reach),
                wp.turn_radius_m,
            ),
            original_index: None,
        });
    }
    out
}
