//! Speed along the route, and fitting the flight into the session it has to last.
//!
//! # Why the cruise speed is not the free variable any more
//!
//! The session length is fixed by the timer, and the route is fixed by the flight the
//! pilot booked, so something has to give. Previously it was the cruise speed: a binary
//! search set it to whatever made the arithmetic work. Measured across the whole route
//! database that produced a median cruise of 595 km/h, a quarter of all flights under
//! 490, and the shortest sectors down around 180 km/h — an A350 well below its stall
//! speed, at altitude.
//!
//! The mistake was treating the scheduled time as flying time. It is *block* time, gate
//! to gate, and the difference is real: taxi, the climb, and the descent. The route
//! database shows this cleanly — median block speed runs from 292 km/h over short
//! sectors up to about 794 km/h over the longest, asymptotically approaching but never
//! reaching a true cruise speed of roughly 900.
//!
//! So the slack is spent on the things that consume it in reality, in this order:
//!
//! 1. **Taxi.** A fixed distance at each end, flown at whatever speed fills the time.
//!    Taxiing slowly in a queue is what actually happens.
//! 2. **Cruise Mach**, within 0.78–0.86. Airlines really do fly this whole band
//!    depending on how the schedule is running; it is the cost index, and it is a
//!    legitimate knob rather than a fudge.
//! 3. **A single time scale**, applied only to the time axis, as the residual.
//!
//! The scale is deliberately last and stays close to 1.0. Everything it touches is the
//! clock; the speeds reported in telemetry stay physical, so the cockpit reads a real
//! number even when the animation is running slightly fast or slow.

use super::aircraft;
use super::atmosphere::tas_from_cas;
use super::geo::LatLon;
use super::path::GroundTrack;
use super::vertical::VerticalProfile;
use super::wind::{solve_wind_triangle, WindField};

/// Height above the destination by which the aircraft is at its approach speed, and the
/// height at which it starts slowing toward it.
const APPROACH_SPEED_AGL_M: f64 = 450.0;
const DECELERATION_START_AGL_M: f64 = 3_000.0;

/// Integration steps for the duration estimate. Fine near the ground, where the speed
/// changes fastest, and coarse in the cruise, where it barely changes at all.
const TERMINAL_STEP_M: f64 = 200.0;
const ENROUTE_STEP_M: f64 = 2_000.0;
const TERMINAL_AGL_M: f64 = 3_000.0;

/// Bisection iterations for the Mach fit. Twenty halvings resolve the band to well
/// under a thousandth of a Mach number.
const MACH_FIT_ITERATIONS: usize = 20;

#[derive(Debug, Clone, Copy)]
pub struct SpeedSchedule {
    pub cruise_mach: f64,
    pub taxi_out_speed: f64,
    pub taxi_in_speed: f64,
}

impl SpeedSchedule {
    /// True airspeed commanded at a point on the route.
    ///
    /// On the ground the returned value is a ground speed — taxi speed, and the
    /// accelerating and decelerating rolls — because that is what those phases are
    /// actually governed by.
    pub fn tas_at(&self, s: f64, profile: &VerticalProfile) -> f64 {
        if s < profile.s_dep_threshold {
            return self.taxi_out_speed;
        }
        if s < profile.s_rotate {
            let rolled = (s - profile.s_dep_threshold).max(0.0);
            let v_rotate = tas_from_cas(aircraft::rotation_speed(), profile.dep_elevation_m);
            return (2.0 * aircraft::TAKEOFF_ACCEL_MPS2 * rolled)
                .sqrt()
                .clamp(1.0, v_rotate);
        }
        if s >= profile.s_rollout_end {
            return self.taxi_in_speed;
        }
        if s >= profile.s_touchdown {
            let remaining = (profile.s_rollout_end - s).max(0.0);
            let v_touchdown = tas_from_cas(aircraft::approach_speed(), profile.arr_elevation_m);
            return (2.0 * aircraft::ROLLOUT_DECEL_MPS2 * remaining)
                .sqrt()
                .clamp(self.taxi_in_speed, v_touchdown);
        }

        let alt = profile.altitude_at(s);
        if s < profile.s_top_of_climb {
            return aircraft::climb_tas(alt, profile.dep_elevation_m);
        }
        if s < profile.s_top_of_descent {
            return aircraft::cruise_tas(self.cruise_mach, alt);
        }

        // Descending: the schedule speed until the aircraft starts configuring, then a
        // blend down to the approach speed. Slowing from 250 kt to 140 kt takes several
        // thousand feet of descent, which is why the shelf in the vertical profile
        // exists at all.
        let agl = (alt - profile.arr_elevation_m).max(0.0);
        let approach = tas_from_cas(aircraft::approach_speed(), alt);
        if agl <= APPROACH_SPEED_AGL_M {
            return approach;
        }
        let clean = aircraft::descent_tas(alt);
        if agl >= DECELERATION_START_AGL_M {
            return clean;
        }
        let f = (agl - APPROACH_SPEED_AGL_M) / (DECELERATION_START_AGL_M - APPROACH_SPEED_AGL_M);
        approach + (clean - approach) * f
    }
}

/// A schedule that fits the requested duration, and how far off physical time it left.
pub struct FittedSchedule {
    pub schedule: SpeedSchedule,
    /// Multiplies the time axis so the flight lands exactly when the session ends.
    /// Speeds in telemetry are never scaled by it.
    pub time_scale: f64,
    /// What the flight would take if flown in real time.
    pub physical_duration_s: f64,
}

/// Everything the integrator needs to turn distance into time.
pub struct TimeContext<'a> {
    pub track: &'a GroundTrack,
    pub profile: &'a VerticalProfile,
    pub wind: &'a WindField,
}

/// Ground speed at a point, given a commanded airspeed.
pub fn ground_speed_at(ctx: &TimeContext, s: f64, tas: f64) -> (f64, f64, LatLon) {
    let position = ctx.track.position_at(s);
    let altitude = ctx.profile.altitude_at(s);

    if ctx.profile.is_on_ground(s) {
        // On the ground the commanded value already is a ground speed, and taxiing into
        // a headwind does not take longer.
        return (tas.max(0.3), 0.0, position);
    }

    let bearing = ctx.track.bearing_at(s);
    let (u, v) = ctx
        .wind
        .sample(position.lat_deg, position.lon_deg, altitude);
    match solve_wind_triangle(bearing, tas, u, v) {
        Some(t) => (t.ground_speed, t.heading_offset_rad, position),
        None => (tas.max(0.3), 0.0, position),
    }
}

/// Time to fly from `from` to `to` along the route.
fn integrate(ctx: &TimeContext, schedule: &SpeedSchedule, from: f64, to: f64) -> f64 {
    let mut t = 0.0;
    let mut s = from;
    while s < to {
        let agl = ctx.profile.height_above_field(s);
        let step = if agl < TERMINAL_AGL_M {
            TERMINAL_STEP_M
        } else {
            ENROUTE_STEP_M
        }
        .min(to - s);
        // Midpoint speed, so an accelerating segment is not integrated at its slowest.
        let mid = s + step * 0.5;
        let tas = schedule.tas_at(mid, ctx.profile);
        let (gs, _, _) = ground_speed_at(ctx, mid, tas);
        t += step / gs.max(0.3);
        s += step;
    }
    t
}

/// Time spent between leaving the departure threshold and stopping on the arrival
/// runway — everything except taxi.
fn air_time(ctx: &TimeContext, mach: f64) -> f64 {
    let schedule = SpeedSchedule {
        cruise_mach: mach,
        taxi_out_speed: aircraft::MAX_TAXI_SPEED_MPS,
        taxi_in_speed: aircraft::MAX_TAXI_SPEED_MPS,
    };
    integrate(
        ctx,
        &schedule,
        ctx.profile.s_dep_threshold,
        ctx.profile.s_rollout_end,
    )
}

/// Builds a schedule that lands the flight at the end of the session.
pub fn fit(ctx: &TimeContext, target_duration_s: f64) -> FittedSchedule {
    let taxi_out_m = ctx.profile.s_dep_threshold.max(1.0);
    let taxi_in_m = (ctx.profile.s_total - ctx.profile.s_rollout_end).max(1.0);

    let min_taxi =
        taxi_out_m / aircraft::MAX_TAXI_SPEED_MPS + taxi_in_m / aircraft::MAX_TAXI_SPEED_MPS;
    let max_taxi =
        taxi_out_m / aircraft::MIN_TAXI_SPEED_MPS + taxi_in_m / aircraft::MIN_TAXI_SPEED_MPS;

    // What the flight takes airborne at the nominal cruise Mach decides how much of the
    // session is left for the ground.
    let nominal_air = air_time(ctx, aircraft::NOMINAL_CRUISE_MACH);
    let ground_budget = (target_duration_s - nominal_air).clamp(min_taxi, max_taxi);

    // Split it in proportion to the distance taxied at each end.
    let share = taxi_out_m / (taxi_out_m + taxi_in_m);
    let taxi_out_time = (ground_budget * share).max(1.0);
    let taxi_in_time = (ground_budget * (1.0 - share)).max(1.0);
    let taxi_out_speed = (taxi_out_m / taxi_out_time)
        .clamp(aircraft::MIN_TAXI_SPEED_MPS, aircraft::MAX_TAXI_SPEED_MPS);
    let taxi_in_speed = (taxi_in_m / taxi_in_time)
        .clamp(aircraft::MIN_TAXI_SPEED_MPS, aircraft::MAX_TAXI_SPEED_MPS);
    let taxi_total = taxi_out_m / taxi_out_speed + taxi_in_m / taxi_in_speed;

    // Then trade cruise Mach against what is left. Faster is always less time, so a
    // plain bisection converges.
    let wanted_air = target_duration_s - taxi_total;
    let mut lo = aircraft::MIN_CRUISE_MACH;
    let mut hi = aircraft::MAX_CRUISE_MACH;
    let mut mach = aircraft::NOMINAL_CRUISE_MACH;
    if air_time(ctx, hi) > wanted_air {
        mach = hi;
    } else if air_time(ctx, lo) < wanted_air {
        mach = lo;
    } else {
        for _ in 0..MACH_FIT_ITERATIONS {
            mach = 0.5 * (lo + hi);
            if air_time(ctx, mach) > wanted_air {
                lo = mach;
            } else {
                hi = mach;
            }
        }
    }

    let schedule = SpeedSchedule {
        cruise_mach: mach,
        taxi_out_speed,
        taxi_in_speed,
    };
    let physical = integrate(ctx, &schedule, 0.0, ctx.profile.s_total);
    let time_scale = if physical > 1.0 {
        target_duration_s / physical
    } else {
        1.0
    };

    FittedSchedule {
        schedule,
        time_scale,
        physical_duration_s: physical,
    }
}
