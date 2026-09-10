//! Assembles a flight plan and samples it into telemetry.
//!
//! The pipeline, in order, with each stage in its own module:
//!
//! 1. [`runway`] picks which end of which runway each airport is using, from the wind.
//! 2. [`lateral`] plans the enroute track — a great circle, bent by wind and by closed
//!    airspace, and gridded onto the oceanic track system where one applies.
//! 3. [`path`] joins the terminal geometry and the enroute waypoints into a ground
//!    track with fly-by turns and an arc length.
//! 4. [`vertical`] hangs an altitude profile on that arc length: climb, flight levels,
//!    step climbs, descent, flare.
//! 5. [`schedule`] works out the speeds, and fits the whole thing into the session.
//! 6. This module walks the result and emits samples.
//!
//! Attitude is derived rather than assumed. Bank comes from the turn the track is
//! actually making, and pitch is the flight path angle *plus the angle of attack* —
//! which is why the aircraft sits nose-up on a 3° approach, as a real one does, rather
//! than pointing at the runway.

use super::aircraft;
use super::airspace::AirspaceRestrictions;
use super::atmosphere::tas_from_mach;
use super::geo::{destination, distance_m, initial_bearing, LatLon};
use super::lateral::{plan_enroute, EnrouteOptions};
use super::path::{GroundTrack, Waypoint};
use super::runway;
use super::schedule::{self, ground_speed_at, SpeedSchedule, TimeContext};
use super::vertical::{semicircular_level, VerticalInputs, VerticalProfile};
use super::wind::WindField;
use crate::flight_handle::RunwayData;

#[derive(Debug, Clone, Copy)]
pub struct TelemetryPoint {
    pub time_offset_ms: u64,
    pub longitude: f64,
    pub latitude: f64,
    pub altitude: f64,
    pub velocity_m_s: f64,
    pub heading_rad: f64,
    pub pitch_rad: f64,
    pub roll_rad: f64,
    pub sun_intensity: f32,
}

/// Which wind field the plan is built against.
#[derive(Debug, Clone, Copy)]
pub enum WindModel {
    /// No wind. Routes come out as great circles, which is what the geometry tests want.
    Calm,
    /// The annual-mean jet. The default, because a focus session has no date attached.
    AnnualMean,
    /// A specific day of the year, 1–365.
    DayOfYear(f64),
}

#[derive(Debug, Clone, Copy)]
pub struct FlightPlanConfig {
    /// Whether runways sit at their true elevation.
    ///
    /// Off by default, and deliberately so: the globe currently renders without terrain,
    /// so an aircraft starting at Bogotá's 2,548 m would hang visibly above a sea-level
    /// surface. Everything downstream already handles real elevations — the ground roll
    /// lengthens in thin air, cruise levels are checked against the field below them —
    /// so turning this on is the only change needed once terrain exists.
    pub terrain_elevation: bool,
    pub dep_elevation_m: f64,
    pub arr_elevation_m: f64,
    /// Whether to route around airspace civil traffic currently avoids.
    pub avoid_closed_airspace: bool,
    /// Whether ocean crossings snap to the organised track grid.
    pub oceanic_tracks: bool,
    pub wind: WindModel,
}

impl Default for FlightPlanConfig {
    fn default() -> Self {
        Self {
            terrain_elevation: false,
            dep_elevation_m: 0.0,
            arr_elevation_m: 0.0,
            avoid_closed_airspace: true,
            oceanic_tracks: true,
            wind: WindModel::AnnualMean,
        }
    }
}

pub struct FlightRequest {
    pub departure: LatLon,
    pub arrival: LatLon,
    pub target_duration_ms: u64,
    /// Used only when the airport has no runway data at all.
    pub dep_heading_deg: Option<f64>,
    pub arr_heading_deg: Option<f64>,
    pub runways: Vec<RunwayData>,
    pub config: FlightPlanConfig,
}

/// Lifts the rendered path clear of the globe surface so it does not z-fight with it.
const RENDER_LIFT_M: f64 = 5.0;

/// Angle the taxi legs leave the runway centreline at, so they read as a parallel
/// taxiway rather than as an extension of the runway.
const TAXIWAY_SPLAY_DEG: f64 = 6.0;

/// Distance over which the nose comes up at rotation, and back down after touchdown.
const ROTATION_DISTANCE_M: f64 = 400.0;

/// Roughly how high the aircraft is at the end of the departure leg, where the first
/// turn happens. Used only to size that turn.
const DEPARTURE_TURN_HEIGHT_M: f64 = 1_400.0;

/// Half-window for the finite differences that produce pitch and bank.
const ATTITUDE_PROBE_M: f64 = 60.0;
const CURVATURE_PROBE_M: f64 = 300.0;

/// Sampling intervals by phase. Ground manoeuvres need resolving; a cruise leg does not.
const DT_TAXI_S: f64 = 4.0;
const DT_LOW_S: f64 = 1.0;
const DT_MID_S: f64 = 2.5;
const DT_TURN_S: f64 = 3.0;
const DT_CRUISE_S: f64 = 8.0;
/// Floor on the sampling interval, expressed in time rather than distance — the
/// distance between samples is free to be small, but a vanishing time interval is what
/// would make the consumer's spline ill-conditioned.
const MIN_STEP_S: f64 = 0.25;
const MAX_SAMPLE_STEP_M: f64 = 2_500.0;
/// How many steps from the end the sampler starts dividing the remainder evenly.
const TAIL_STEPS: f64 = 4.0;
const LOW_AGL_M: f64 = 1_500.0;
const MID_AGL_M: f64 = 6_000.0;
/// Bank beyond which the sampler tightens up so a turn is not chorded.
const TURN_BANK_RAD: f64 = 0.03;

/// Most the sampling interval may change between one sample and the next.
///
/// This bounds the velocity discontinuity the consumer's spline sees — see the comment
/// at the step calculation. It costs about twenty samples to migrate between the taxi
/// and cruise rates, which is a few seconds of flight.
const MAX_STEP_RATIO: f64 = 1.12;

pub fn generate(request: &FlightRequest) -> Vec<TelemetryPoint> {
    let wind = match request.config.wind {
        WindModel::Calm => WindField::calm(),
        WindModel::AnnualMean => WindField::annual_mean(),
        WindModel::DayOfYear(d) => WindField::for_day_of_year(d),
    };

    let direct_bearing = initial_bearing(request.departure, request.arrival);
    let dep_runway = runway::select(
        request.departure,
        &request.runways,
        &wind,
        request
            .dep_heading_deg
            .map(|d| d.to_radians())
            .unwrap_or(direct_bearing),
    );
    let arr_runway = runway::select(
        request.arrival,
        &request.runways,
        &wind,
        request
            .arr_heading_deg
            .map(|d| d.to_radians())
            .unwrap_or(direct_bearing),
    );

    let (dep_elev, arr_elev) = if request.config.terrain_elevation {
        (
            request.config.dep_elevation_m,
            request.config.arr_elevation_m,
        )
    } else {
        (0.0, 0.0)
    };

    // Terminal geometry, laid out along each runway's centreline.
    let splay = TAXIWAY_SPLAY_DEG.to_radians();
    let apron_out = destination(
        dep_runway.threshold,
        dep_runway.heading_rad + std::f64::consts::PI + splay,
        aircraft::TAXI_OUT_DISTANCE_M,
    );
    let dep_leg_end = destination(
        dep_runway.threshold,
        dep_runway.heading_rad,
        aircraft::DEPARTURE_LEG_M,
    );
    let final_start = destination(
        arr_runway.threshold,
        arr_runway.heading_rad + std::f64::consts::PI,
        aircraft::FINAL_APPROACH_M,
    );
    let touchdown = destination(
        arr_runway.threshold,
        arr_runway.heading_rad,
        aircraft::TOUCHDOWN_OFFSET_M,
    );
    let rollout_m = aircraft::rollout_distance(arr_elev);
    let rollout_end = destination(
        arr_runway.threshold,
        arr_runway.heading_rad,
        aircraft::TOUCHDOWN_OFFSET_M + rollout_m,
    );
    let apron_in = destination(
        rollout_end,
        arr_runway.heading_rad + splay,
        aircraft::TAXI_IN_DISTANCE_M,
    );

    // A first estimate of the cruise level, needed before the route exists because the
    // wind is sampled at cruise altitude. One pass is enough — the level depends on
    // distance only weakly, and the route length barely moves between the estimate and
    // the plan.
    let straight_m = distance_m(dep_leg_end, final_start);
    let est_level = semicircular_level(
        aircraft::optimum_cruise_altitude_m(straight_m),
        direct_bearing,
    );
    let est_altitude = super::atmosphere::feet_to_m(est_level as f64 * 100.0);
    let est_tas = tas_from_mach(aircraft::NOMINAL_CRUISE_MACH, est_altitude);

    let airspace = if request.config.avoid_closed_airspace {
        AirspaceRestrictions::for_route(request.departure, request.arrival)
    } else {
        AirspaceRestrictions::none()
    };

    let enroute = plan_enroute(
        dep_leg_end,
        final_start,
        &EnrouteOptions {
            wind: &wind,
            airspace: &airspace,
            cruise_altitude_m: est_altitude,
            cruise_tas: est_tas,
            oceanic_tracks: request.config.oceanic_tracks,
        },
    );

    // Turn radii follow the speed at each corner, which is the whole point of deriving
    // them rather than fixing one.
    // Sized for the speed at the *end* of the departure leg, not at the acceleration
    // altitude: by the time the aircraft reaches the first turn it has cleaned up and
    // is doing 250 kt, and a radius sized for the climb-out speed would put it at 47°
    // of bank.
    let departure_turn_r = aircraft::turn_radius(
        aircraft::climb_tas(dep_elev + DEPARTURE_TURN_HEIGHT_M, dep_elev),
        dep_elev + DEPARTURE_TURN_HEIGHT_M,
    );
    let cruise_turn_r = aircraft::turn_radius(est_tas, est_altitude);
    let final_turn_r = aircraft::turn_radius(
        aircraft::descent_tas(arr_elev + 1_000.0),
        arr_elev + 1_000.0,
    );

    let mut waypoints = Vec::with_capacity(enroute.len() + 8);
    waypoints.push(Waypoint::sharp(apron_out));
    let idx_dep_threshold = waypoints.len();
    waypoints.push(Waypoint::sharp(dep_runway.threshold));
    waypoints.push(Waypoint::new(dep_leg_end, departure_turn_r));
    if enroute.len() > 2 {
        for p in &enroute[1..enroute.len() - 1] {
            waypoints.push(Waypoint::new(*p, cruise_turn_r));
        }
    }
    waypoints.push(Waypoint::new(final_start, final_turn_r));
    waypoints.push(Waypoint::sharp(arr_runway.threshold));
    let idx_touchdown = waypoints.len();
    waypoints.push(Waypoint::sharp(touchdown));
    let idx_rollout_end = waypoints.len();
    waypoints.push(Waypoint::sharp(rollout_end));
    waypoints.push(Waypoint::sharp(apron_in));

    let (track, placed) = GroundTrack::build(&waypoints);
    let s_total = track.total_length();
    if s_total <= 0.0 {
        return Vec::new();
    }

    let profile = VerticalProfile::build(&VerticalInputs {
        s_total,
        s_dep_threshold: placed[idx_dep_threshold],
        s_touchdown: placed[idx_touchdown],
        s_rollout_end: placed[idx_rollout_end],
        dep_elevation_m: dep_elev,
        arr_elevation_m: arr_elev,
        track_rad: direct_bearing,
        trip_distance_m: straight_m,
        cruise_tas_estimate: est_tas,
    });

    let ctx = TimeContext {
        track: &track,
        profile: &profile,
        wind: &wind,
    };
    let fitted = schedule::fit(&ctx, request.target_duration_ms as f64 / 1000.0);

    log::info!(
        "flight plan: {:.0} km track, FL{} ({} step climbs), M{:.3}, {:.0} min physical \
         vs {:.0} min session (scale {:.2})",
        s_total / 1000.0,
        profile.cruise_flight_level,
        profile.step_count,
        fitted.schedule.cruise_mach,
        fitted.physical_duration_s / 60.0,
        request.target_duration_ms as f64 / 60_000.0,
        fitted.time_scale,
    );

    let mut points = sample(&ctx, &fitted.schedule);

    // The fit works from a coarse integration of the profile; the sampler walks it at
    // its own phase-dependent step and so lands a fraction of a percent away. Rescaling
    // against what was actually sampled makes the arrival exact, which matters because
    // playback maps session progress onto the last timestamp.
    if let Some(last) = points.last() {
        if last.time_offset_ms > 0 {
            let scale = request.target_duration_ms as f64 / last.time_offset_ms as f64;
            for p in &mut points {
                p.time_offset_ms = (p.time_offset_ms as f64 * scale).round() as u64;
            }
        }
    }
    points
}

fn sample(ctx: &TimeContext, schedule: &SpeedSchedule) -> Vec<TelemetryPoint> {
    let track = ctx.track;
    let profile = ctx.profile;
    let total = track.total_length();

    // Ground level is whichever field is nearer, so the sun curve reads 1.0 on the
    // ground whether or not real elevations are in use.
    let ground_ref = profile.dep_elevation_m.min(profile.arr_elevation_m);
    let sun_span = (profile.cruise_altitude_m - ground_ref).max(1.0);

    let mut points: Vec<TelemetryPoint> = Vec::new();
    let mut s = 0.0_f64;
    let mut t = 0.0_f64;
    let mut bank = 0.0_f64;
    let mut last_dt = 0.0_f64;

    loop {
        let position = track.position_at(s);
        let altitude = profile.altitude_at(s);
        let tas = schedule.tas_at(s, profile);
        let (ground_speed, heading_offset, _) = ground_speed_at(ctx, s, tas);
        let bearing = track.bearing_at(s);

        // Flight path angle from the profile, then body attitude on top of it.
        let lo = (s - ATTITUDE_PROBE_M).max(0.0);
        let hi = (s + ATTITUDE_PROBE_M).min(total);
        let span = (hi - lo).max(1.0);
        let fpa = ((profile.altitude_at(hi) - profile.altitude_at(lo)) / span).atan();
        let mut pitch = fpa + aircraft::angle_of_attack(tas, altitude);

        // The nose is on the ground until rotation and back down after landing, and
        // moves between the two over a few hundred metres rather than instantly.
        if s <= profile.s_rotate {
            pitch = 0.0;
        } else if s < profile.s_rotate + ROTATION_DISTANCE_M {
            pitch *= (s - profile.s_rotate) / ROTATION_DISTANCE_M;
        }
        if s >= profile.s_touchdown {
            let f = ((s - profile.s_touchdown) / ROTATION_DISTANCE_M).clamp(0.0, 1.0);
            pitch *= 1.0 - f;
        }

        // Bank from the curvature of the track the aircraft is actually flying. On the
        // ground the landing gear settles the question, so the roll dynamics below are
        // bypassed entirely rather than left to decay toward zero.
        let on_ground = profile.is_on_ground(s);
        let target_bank = if on_ground {
            0.0
        } else {
            let arc =
                ((s + CURVATURE_PROBE_M).min(total) - (s - CURVATURE_PROBE_M).max(0.0)).max(1.0);
            let turn_rate = track.turn_angle_at(s, CURVATURE_PROBE_M) / arc * ground_speed;
            let limit = aircraft::max_bank(altitude);
            (ground_speed * turn_rate / 9.806_65)
                .atan()
                .clamp(-limit, limit)
        };
        // Rolling into a turn takes a few seconds; without this the bank steps at every
        // waypoint even though the track through it is smooth.
        if on_ground {
            bank = 0.0;
        } else {
            let max_delta = aircraft::ROLL_RATE_RAD_PER_S * last_dt;
            bank += (target_bank - bank).clamp(-max_delta, max_delta);
        }

        points.push(TelemetryPoint {
            time_offset_ms: (t * 1000.0) as u64,
            longitude: position.lon_deg,
            latitude: position.lat_deg,
            altitude: altitude + RENDER_LIFT_M,
            velocity_m_s: ground_speed,
            heading_rad: bearing + heading_offset,
            pitch_rad: pitch,
            roll_rad: bank,
            sun_intensity: (1.0 - ((altitude - ground_ref) / sun_span).clamp(0.0, 1.0)) as f32,
        });

        if s >= total {
            break;
        }

        let agl = profile.height_above_field(s);
        let target_dt = if profile.is_on_ground(s) && ground_speed < 15.0 {
            DT_TAXI_S
        } else if agl < LOW_AGL_M {
            DT_LOW_S
        } else if agl < MID_AGL_M {
            DT_MID_S
        } else if bank.abs() > TURN_BANK_RAD {
            DT_TURN_S
        } else {
            DT_CRUISE_S
        };
        // The consumer interpolates these samples with a *uniform* Catmull-Rom spline,
        // whose tangent at a knot is the same vector from either side but is divided by
        // the local time interval to get a velocity. So a step change in the sampling
        // interval is a step change in the rendered aircraft's speed, in exactly that
        // ratio — and the rendered attitude is derived from that motion, not from the
        // angles in this struct. Easing between rates keeps the discontinuity bounded
        // and small; a phase-dependent interval applied directly reached 12x.
        let dt = if last_dt > 0.0 {
            target_dt.clamp(last_dt / MAX_STEP_RATIO, last_dt * MAX_STEP_RATIO)
        } else {
            target_dt
        }
        .max(MIN_STEP_S);

        let mut step = (ground_speed * dt).min(MAX_SAMPLE_STEP_M);
        // Re-imposed after the distance cap, because the time spacing is the thing the
        // spline is parameterised by and so has to take precedence. A distance floor
        // applied last is what let taxi samples drift to a 4.2 s interval while the
        // schedule was still asking for 2 s.
        if last_dt > 0.0 {
            step = step.clamp(
                ground_speed * last_dt / MAX_STEP_RATIO,
                ground_speed * last_dt * MAX_STEP_RATIO,
            );
        }
        // Spread the tail over a whole number of equal steps, so the flight lands
        // exactly on its final knot without a stub interval next to a full-length one.
        let remaining = total - s;
        if remaining <= step * TAIL_STEPS {
            let n = (remaining / step).round().max(1.0);
            step = remaining / n;
        }
        if step <= 0.0 {
            break;
        }
        last_dt = step / ground_speed.max(0.3);
        t += last_dt;
        s += step;
    }

    points
}
