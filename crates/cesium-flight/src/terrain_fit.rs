//! Fitting the flight onto the rendered terrain near the airports.
//!
//! The flight profile puts the aircraft on the ground at the airport's published
//! elevation, which is not the height of the terrain the engine draws there. The
//! published figure is the *highest* point of the landing area, so the drawn ground along
//! a runway is usually lower: 12-16 m along the Frankfurt departure roll, 20-35 m along
//! the Stuttgart rollout. Near each airport the flight is therefore moved onto the drawn
//! ground, in full on the ground and fading out with height above the airport.
//!
//! The aircraft and its route line have to be moved by the same rule or they come apart:
//! when only the aircraft was fitted, the line stayed at the published elevation and hung
//! 20-39 m above an aircraft standing on the runway. So everything that decides the fit
//! is here, and both use it: [`AircraftFit`] for the aircraft, [`LineFit`] for the line.
//!
//! # The rule
//!
//! At time `t`, near the end whose field altitude is `field`, the profile altitude gains
//!
//! ```text
//! offset = (G - field) * weight(altitude - field)
//! ```
//!
//! [`weight`] is 1 up to [`FADE_BOTTOM_M`] above the field and 0 from [`FADE_TOP_M`]. `G`
//! is the ground under the flight position while it is on the ground, and the ground
//! under the lift-off or touchdown point once it is airborne. Following the ground under
//! an airborne flight instead made it dip into any valley beyond the runway end and rise
//! over hills under the approach, by as much as the weight allowed.

use std::ops::Range;
use std::time::Instant;

use cesium_engine::globe::geometry::lon_lat_alt_to_ecef_f64;
use cesium_engine::render::polyline_pipeline::builder::ControlPoint;
use cesium_engine::render::polyline_pipeline::pipeline::RIBBON_LIFT_M;
use glam::DVec3;

use crate::telemetry::TelemetryPoint;

/// Height above the airport below which the fit applies in full, metres.
pub const FADE_BOTTOM_M: f64 = 30.0;
/// Height above the airport from which the profile altitude is used as it is, metres.
pub const FADE_TOP_M: f64 = 600.0;
/// Time constant with which the aircraft follows the terrain (better height data
/// arriving, the aircraft rolling over a slope), seconds.
const AIRCRAFT_SMOOTHING_S: f64 = 0.15;
/// How far above the field the profile may be and still count as on the ground, metres.
/// The profile holds the field altitude exactly until rotation; this takes in the first
/// half metre of the rotation, while the main gear is still rolling.
const ON_GROUND_TOLERANCE_M: f64 = 0.5;
/// Height of the route line above the drawn terrain while the aircraft is on the ground,
/// metres.
///
/// In the air the line runs [`RIBBON_LIFT_M`] above the flight path and the aircraft's
/// model origin 2.5 m higher, which puts the line level with the bottom of the landing
/// gear, under the fuselage. On the ground the gear stands on the terrain, so the same
/// place is just above it; 1.5 m keeps the line under the belly while leaving room for the
/// drawn surface, triangulated from a coarser grid than the heights the line is placed
/// on, to stand above them. No gaps from 250 m to 3 km in `rendering::route_line_ground`.
pub const LINE_GROUND_LIFT_M: f64 = 1.5;
/// Longest segment of the route line along the ground phases, metres.
///
/// The line is straight between its control points, and a takeoff roll is straight, so
/// without this the whole Frankfurt roll was three points and the line could not follow
/// the ground between them. 10 m is about three texels of the finest height data.
pub const LINE_GROUND_SPACING_M: f64 = 10.0;
/// Longest segment of the route line where the fit fades out, from the ground phases to
/// [`FADE_TOP_M`] above the field, metres. The fit's weight is not linear in height, so
/// a line straight across a long stretch of it parts from the aircraft: by 1.4 m at
/// 250 m above Frankfurt with the builder's own points alone. At 100 m what is left is
/// the builder's own 0.1 m tolerance (0.09 m measured).
pub const LINE_FADE_SPACING_M: f64 = 100.0;
/// Ground points the route line reads per frame. Both ground phases of Frankfurt-Stuttgart
/// are some 500 points, so height data that lands is followed within a few frames while no
/// frame pays for all (the whole fit is ~10 µs a frame, dev or release).
const LINE_READS_PER_FRAME: usize = 64;
/// Change in a point's height below which the line is not re-uploaded, metres.
const LINE_UPLOAD_THRESHOLD_M: f64 = 0.01;

/// How much of the fit applies `above_field_m` above the airport: 1 up to
/// [`FADE_BOTTOM_M`], 0 from [`FADE_TOP_M`], smoothstep between.
pub fn weight(above_field_m: f64) -> f64 {
    let t = ((above_field_m - FADE_BOTTOM_M) / (FADE_TOP_M - FADE_BOTTOM_M)).clamp(0.0, 1.0);
    1.0 - t * t * (3.0 - 2.0 * t)
}

/// Profile altitude `t_s` seconds into the flight, metres: linear between telemetry
/// samples, as the telemetry the tracker reports is. `None` without telemetry.
pub fn altitude_at(points: &[TelemetryPoint], t_s: f64) -> Option<f64> {
    let t_ms = t_s * 1000.0;
    let i = points.partition_point(|p| (p.time_offset_ms as f64) < t_ms);
    if i == 0 {
        return points.first().map(|p| p.altitude);
    }
    let Some(b) = points.get(i) else {
        return points.last().map(|p| p.altitude);
    };
    let a = &points[i - 1];
    let span = (b.time_offset_ms - a.time_offset_ms) as f64;
    let f = if span > 0.0 {
        (t_ms - a.time_offset_ms as f64) / span
    } else {
        0.0
    };
    Some(a.altitude + (b.altitude - a.altitude) * f)
}

/// One end of a flight, as [`GroundEnds`] sees it.
#[derive(Clone, Copy, Debug)]
pub struct GroundEnd {
    /// Profile altitude on the ground here, metres: the published elevation plus the lift
    /// the planner gives everything it draws.
    pub field_m: f64,
    /// When the flight leaves the ground here (departure) or is back on it (arrival),
    /// seconds.
    pub edge_s: f64,
    /// Flight position at `edge_s`, ECEF Megametres. Once the flight is airborne the
    /// ground under this point stands for the ground at the airport.
    pub anchor: DVec3,
    /// When the flight is [`FADE_TOP_M`] above the field here, seconds: the fit applies
    /// between this and `edge_s`. Halfway through a flight that never gets that high.
    pub fade_s: f64,
}

/// Where a flight meets the ground at either end, found once from its profile.
#[derive(Clone, Copy, Debug)]
pub struct GroundEnds {
    pub dep: GroundEnd,
    pub arr: GroundEnd,
    /// When the flight starts and ends, seconds.
    pub start_s: f64,
    pub end_s: f64,
}

impl GroundEnds {
    /// `None` for a flight with no telemetry.
    pub fn new(points: &[TelemetryPoint]) -> Option<Self> {
        let (first, last) = (points.first()?, points.last()?);
        let start_s = first.time_offset_ms as f64 / 1000.0;
        let end_s = last.time_offset_ms as f64 / 1000.0;
        let half_s = 0.5 * (start_s + end_s);
        let time = |p: &TelemetryPoint| p.time_offset_ms as f64 / 1000.0;
        let on_ground =
            |p: &&TelemetryPoint, field: f64| p.altitude - field <= ON_GROUND_TOLERANCE_M;
        // Both at least 1: the first and last points are on the ground by definition.
        let dep_on = points.iter().take_while(|p| on_ground(p, first.altitude)).count();
        let arr_on = points.iter().rev().take_while(|p| on_ground(p, last.altitude)).count();
        let faded = |p: &&TelemetryPoint, field: f64| p.altitude - field >= FADE_TOP_M;
        let dep_fade_s = points.iter().find(|p| faded(p, first.altitude)).map_or(half_s, time);
        let arr_fade_s = points.iter().rev().find(|p| faded(p, last.altitude)).map_or(half_s, time);
        let end = |field_m: f64, p: &TelemetryPoint, fade_s: f64| GroundEnd {
            field_m,
            edge_s: time(p),
            anchor: DVec3::from_array(lon_lat_alt_to_ecef_f64(p.longitude, p.latitude, p.altitude)),
            fade_s: fade_s.clamp(start_s, end_s),
        };
        Some(Self {
            dep: end(first.altitude, &points[dep_on - 1], dep_fade_s.min(half_s)),
            arr: end(last.altitude, &points[points.len() - arr_on], arr_fade_s.max(half_s)),
            start_s,
            end_s,
        })
    }

    /// Whether `t_s` belongs to the departure end. The flight is split halfway, as the
    /// aircraft fit always has.
    pub fn is_departure(&self, t_s: f64) -> bool {
        t_s < 0.5 * (self.start_s + self.end_s)
    }

    fn end_at(&self, t_s: f64) -> &GroundEnd {
        if self.is_departure(t_s) {
            &self.dep
        } else {
            &self.arr
        }
    }

    /// Whether the flight is on the ground `t_s` seconds in.
    pub fn on_ground(&self, t_s: f64) -> bool {
        if self.is_departure(t_s) {
            t_s <= self.dep.edge_s
        } else {
            t_s >= self.arr.edge_s
        }
    }

    /// The departure and arrival ground phases, as time ranges in seconds.
    pub fn ground_phases(&self) -> [(f64, f64); 2] {
        [(self.start_s, self.dep.edge_s), (self.arr.edge_s, self.end_s)]
    }

    /// `max_segment_lengths` for the builder of a line [`LineFit`] is to move: the ground
    /// phases held to [`LINE_GROUND_SPACING_M`], so there are points along the runway to
    /// lay onto the ground, and the fades to [`LINE_FADE_SPACING_M`].
    pub fn line_spacing(&self) -> Vec<(f64, f64, f64)> {
        vec![
            (self.start_s, self.dep.edge_s, LINE_GROUND_SPACING_M * 1.0e-6),
            (self.dep.edge_s, self.dep.fade_s, LINE_FADE_SPACING_M * 1.0e-6),
            (self.arr.fade_s, self.arr.edge_s, LINE_FADE_SPACING_M * 1.0e-6),
            (self.arr.edge_s, self.end_s, LINE_GROUND_SPACING_M * 1.0e-6),
        ]
    }

    /// Where the ground that decides the fit at `t_s` is read: under `position`, the
    /// flight position then, while on the ground; under the end's anchor once airborne.
    pub fn reference(&self, t_s: f64, position: DVec3) -> DVec3 {
        if self.on_ground(t_s) {
            position
        } else {
            self.end_at(t_s).anchor
        }
    }

    /// The [`weight`] at `t_s`, where the profile is at `altitude_m`.
    pub fn weight_at(&self, t_s: f64, altitude_m: f64) -> f64 {
        weight(altitude_m - self.end_at(t_s).field_m)
    }

    /// The fit at `t_s`, where the profile is at `altitude_m` and the ground read at
    /// [`Self::reference`] is `ground_m`: metres to add to the profile altitude, and the
    /// [`weight`] they were scaled by.
    pub fn fit(&self, t_s: f64, altitude_m: f64, ground_m: f64) -> (f64, f64) {
        let w = self.weight_at(t_s, altitude_m);
        ((ground_m - self.end_at(t_s).field_m) * w, w)
    }
}

/// How the aircraft sits on the rendered terrain, refreshed every frame.
#[derive(Default)]
pub struct AircraftFit {
    /// Metres added to the profile altitude, along the local vertical.
    pub offset_m: f64,
    /// The [`weight`] at the aircraft: 1 on the ground, 0 from [`FADE_TOP_M`] above the
    /// airport up.
    pub weight: f64,
    /// Height of the (offset) flight position above the terrain under it, metres.
    pub agl_m: f64,
    /// Progress and time the fit was last computed at, to tell a jump from playback.
    at: Option<(f64, Instant)>,
}

impl AircraftFit {
    /// Refreshes the fit for the aircraft `t_s` seconds (`progress`) into the flight, at
    /// `position` on the profile, where the profile altitude is `altitude_m`. `ground` is
    /// the engine's query: terrain height above the ellipsoid, in Megametres, under an
    /// ECEF position. Keeps the previous fit while the ground it needs is not known.
    pub fn update(
        &mut self,
        ends: &GroundEnds,
        progress: f64,
        t_s: f64,
        altitude_m: f64,
        position: DVec3,
        ground: &dyn Fn(DVec3) -> Option<f64>,
    ) {
        let now = Instant::now();
        let Some(under_m) = ground(position).map(|g| g * 1.0e6) else {
            return;
        };
        let reference = ends.reference(t_s, position);
        let ground_m = if reference == position {
            under_m
        } else {
            let Some(g) = ground(reference) else {
                return;
            };
            g * 1.0e6
        };
        let (target, weight) = ends.fit(t_s, altitude_m, ground_m);

        // Follow smoothly during playback; snap after a jump in progress or on the first frame.
        let offset = match self.at {
            Some((q, then)) if (q - progress).abs() < 0.002 => {
                let dt = now.duration_since(then).as_secs_f64();
                let k = 1.0 - (-dt / AIRCRAFT_SMOOTHING_S).exp();
                self.offset_m + (target - self.offset_m) * k
            }
            _ => target,
        };
        self.offset_m = offset;
        self.weight = weight;
        self.agl_m = altitude_m + offset - under_m;
        self.at = Some((progress, now));
    }
}

/// A control point the fit can move.
struct Movable {
    /// Index into the line's control points.
    index: usize,
    t_s: f64,
    /// Profile altitude at `t_s`, metres.
    altitude_m: f64,
    /// 0 for the departure end, 1 for the arrival.
    end: usize,
    on_ground: bool,
    /// Position as built from the profile, ECEF Megametres.
    position: DVec3,
    /// Ground under it, metres, once read. Read for points on the ground only.
    ground_m: Option<f64>,
    /// How far it is currently moved along the vertical, metres.
    shift_m: f64,
}

/// A route line's control points, with the fit applied near both airports.
///
/// Only the points near an airport move: those with a nonzero [`weight`], the head and
/// tail of the line. Each is moved exactly as the aircraft would be at that point, plus
/// the difference between the line's height above the ground there and its usual
/// [`RIBBON_LIFT_M`] above the flight path, faded by the same weight: in full on the
/// ground ([`LINE_GROUND_LIFT_M`]), not at all from [`FADE_TOP_M`] up.
///
/// The ground under the points on the ground is read a share at a time
/// ([`LINE_READS_PER_FRAME`]); the anchors every frame, since every airborne point near an
/// airport hangs off them.
pub struct LineFit {
    /// As built from the profile. The fit is applied to these afresh every time, so it
    /// never accumulates.
    base: Vec<ControlPoint>,
    /// What is drawn: `base` with the fit applied.
    fitted: Vec<ControlPoint>,
    movable: Vec<Movable>,
    /// Indices into `movable` of the points on the ground, in the order they are read.
    on_ground: Vec<usize>,
    /// Next entry of `on_ground` to read.
    cursor: usize,
    /// Ground under the departure and arrival anchors, metres, once known.
    anchor_ground_m: [Option<f64>; 2],
    /// Control points moved at each end since the last [`Self::take_changes`].
    changed: [Option<Range<usize>>; 2],
}

impl LineFit {
    /// Takes the line as built from the profile: `points`, with `progress` a fraction of
    /// the flight from `ends.start_s` to `ends.end_s`. Nothing moves until
    /// [`Self::sample`] has read some ground.
    pub fn new(points: Vec<ControlPoint>, ends: &GroundEnds, telemetry: &[TelemetryPoint]) -> Self {
        let duration_s = ends.end_s - ends.start_s;
        let movable: Vec<Movable> = points
            .iter()
            .enumerate()
            .filter_map(|(index, cp)| {
                let t_s = ends.start_s + cp.progress as f64 * duration_s;
                let altitude_m = altitude_at(telemetry, t_s)?;
                (ends.weight_at(t_s, altitude_m) > 0.0).then(|| Movable {
                    index,
                    t_s,
                    altitude_m,
                    end: if ends.is_departure(t_s) { 0 } else { 1 },
                    on_ground: ends.on_ground(t_s),
                    position: position_of(cp),
                    ground_m: None,
                    shift_m: 0.0,
                })
            })
            .collect();
        let on_ground = (0..movable.len()).filter(|&i| movable[i].on_ground).collect();
        Self {
            fitted: points.clone(),
            base: points,
            movable,
            on_ground,
            cursor: 0,
            anchor_ground_m: [None; 2],
            changed: [None, None],
        }
    }

    /// The control points as they are to be drawn.
    pub fn points(&self) -> &[ControlPoint] {
        &self.fitted
    }

    /// Reads this frame's share of the ground and moves the points it changes. `ground`
    /// is the engine's query, as for [`AircraftFit::update`].
    pub fn sample(&mut self, ends: &GroundEnds, ground: &dyn Fn(DVec3) -> Option<f64>) {
        if self.movable.is_empty() {
            return;
        }
        for (slot, anchor) in [ends.dep.anchor, ends.arr.anchor].into_iter().enumerate() {
            if let Some(g) = ground(anchor) {
                self.anchor_ground_m[slot] = Some(g * 1.0e6);
            }
        }
        let n = self.on_ground.len();
        for _ in 0..LINE_READS_PER_FRAME.min(n) {
            let m = &mut self.movable[self.on_ground[self.cursor]];
            self.cursor = (self.cursor + 1) % n;
            if let Some(g) = ground(m.position) {
                m.ground_m = Some(g * 1.0e6);
            }
        }

        for m in &mut self.movable {
            // A point on the ground not read yet borrows its anchor's ground, a far better
            // guess than the published elevation it would otherwise keep.
            let anchor_ground = self.anchor_ground_m[m.end];
            let ground_m = if m.on_ground {
                m.ground_m.or(anchor_ground)
            } else {
                anchor_ground
            };
            let Some(ground_m) = ground_m else {
                continue;
            };
            let (offset, w) = ends.fit(m.t_s, m.altitude_m, ground_m);
            let shift = offset + (LINE_GROUND_LIFT_M - RIBBON_LIFT_M) * w;
            if (shift - m.shift_m).abs() < LINE_UPLOAD_THRESHOLD_M {
                continue;
            }
            m.shift_m = shift;
            let base = &self.base[m.index];
            let mut cp = ControlPoint::from_dvec3(
                m.position + m.position.normalize() * (shift * 1.0e-6),
                base.progress,
            );
            cp.distance = base.distance;
            self.fitted[m.index] = cp;
            let range = self.changed[m.end].get_or_insert(m.index..m.index + 1);
            range.start = range.start.min(m.index);
            range.end = range.end.max(m.index + 1);
        }
    }

    /// The control points moved since the last call, as up to one range per end, to be
    /// uploaded from [`Self::points`].
    pub fn take_changes(&mut self) -> [Option<Range<usize>>; 2] {
        std::mem::take(&mut self.changed)
    }
}

/// A control point's position in full precision, ECEF Megametres.
pub fn position_of(cp: &ControlPoint) -> DVec3 {
    DVec3::new(
        cp.pos_hi[0] as f64 + cp.pos_lo[0] as f64,
        cp.pos_hi[1] as f64 + cp.pos_lo[1] as f64,
        cp.pos_hi[2] as f64 + cp.pos_lo[2] as f64,
    )
}
