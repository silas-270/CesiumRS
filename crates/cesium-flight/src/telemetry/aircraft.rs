//! Airbus A350-900 performance envelope.
//!
//! Every number the profile is allowed to invent lives here, so that "is this
//! realistic?" is a question about one file. The aircraft is fixed because the renderer
//! only has an A350 model; if that ever changes, this becomes a table rather than a set
//! of constants.

use super::atmosphere::{feet_to_m, knots_to_mps};

/// Ceiling used for the climb-rate decay. The certified ceiling is FL431; operationally
/// nothing is planned above FL410, and the rate near the ceiling is what shapes the
/// top of the climb.
pub const SERVICE_CEILING_M: f64 = 13_100.0;

/// Highest planned cruise level. Above FL410 the vertical separation standard changes
/// and airliners effectively never go there.
pub const MAX_CRUISE_FL: i32 = 410;

/// Lowest level that counts as cruise rather than an enroute climb.
pub const MIN_CRUISE_FL: i32 = 100;

/// Long-range cruise Mach, and the band a cost-index change can move it within.
/// Airlines really do fly the whole of this range depending on how the schedule is
/// running, which is what makes it a legitimate knob for fitting a session length.
pub const NOMINAL_CRUISE_MACH: f64 = 0.84;
pub const MIN_CRUISE_MACH: f64 = 0.78;
pub const MAX_CRUISE_MACH: f64 = 0.86;

/// Climb speed schedule: an initial segment at climb-out speed, then 250 kt below
/// 10,000 ft, then 300 kt CAS, then Mach.
pub const SPEED_LIMIT_ALT_M: f64 = 3_048.0; // 10,000 ft
pub fn speed_limit_cas() -> f64 {
    knots_to_mps(250.0)
}
pub fn climb_cas() -> f64 {
    knots_to_mps(300.0)
}
pub const CLIMB_MACH: f64 = 0.84;

/// Speed flown from rotation to the acceleration altitude, and the heights that segment
/// spans above the departure field.
///
/// This is why an airliner leaves the runway at a steep-looking attitude and then
/// visibly lowers the nose a minute later: it climbs out slow and steep on takeoff
/// flap, then accelerates and cleans up. Climbing out at the 250 kt limit instead puts
/// the initial pitch around 7° rather than the 12-13° it really is.
pub fn initial_climb_cas() -> f64 {
    knots_to_mps(170.0)
}
pub const ACCELERATION_HEIGHT_M: f64 = 460.0; // 1,500 ft
pub const CLEAN_UP_HEIGHT_M: f64 = 915.0; // 3,000 ft

/// Descent is flown slightly slower than the climb, then the same CAS schedule.
pub const DESCENT_MACH: f64 = 0.82;
pub fn descent_cas() -> f64 {
    knots_to_mps(290.0)
}

/// Reference landing speed at a typical arrival weight.
pub fn approach_speed() -> f64 {
    knots_to_mps(140.0)
}

/// Rotation speed. The ground roll is derived from this and the acceleration below.
pub fn rotation_speed() -> f64 {
    knots_to_mps(152.0)
}

/// Mean acceleration on the takeoff roll and deceleration on the landing rollout.
/// The takeoff figure puts the ground roll at roughly 2,300 m, which is what a heavy
/// A350 uses.
pub const TAKEOFF_ACCEL_MPS2: f64 = 1.65;
pub const ROLLOUT_DECEL_MPS2: f64 = 1.8;

/// Sea-level rate of climb, and the exponent on its decay toward the ceiling.
///
/// `roc(h) = ROC_SEA_LEVEL * (1 - (h/ceiling)^DECAY)` reproduces the real shape: about
/// 2,900 fpm low down, 1,200 fpm at FL350, and a few hundred approaching FL390. A
/// constant rate would put the top of climb hundreds of kilometres too early.
pub const ROC_SEA_LEVEL_MPS: f64 = 15.0;
pub const ROC_DECAY_EXPONENT: f64 = 2.5;

/// Descent gradient. Idle descent and the ILS glideslope are both very close to 3°,
/// which is where the 3:1 rule of thumb comes from — 3 nautical miles per 1,000 ft.
pub const DESCENT_ANGLE_RAD: f64 = 0.0524; // 3.0 degrees

/// Bank limits. Airliner autopilots hold 25°, and reduce it at high altitude where the
/// margin to buffet is thin.
pub const MAX_BANK_RAD: f64 = 0.436; // 25 degrees
pub const HIGH_ALT_MAX_BANK_RAD: f64 = 0.349; // 20 degrees
pub const HIGH_ALT_BANK_THRESHOLD_M: f64 = 10_400.0;

/// Roll rate. A turn does not begin at full bank; it takes a few seconds to get there,
/// and rate-limiting the bank is what stops the aircraft snapping between headings at
/// every waypoint.
pub const ROLL_RATE_RAD_PER_S: f64 = 0.0873; // 5 degrees/second

/// Taxi speed bounds. Taxi distance is fixed geometry, so the time is absorbed by
/// varying the speed within this band — which is also what happens in a real departure
/// queue.
pub const MIN_TAXI_SPEED_MPS: f64 = 1.2;
pub const MAX_TAXI_SPEED_MPS: f64 = 9.0;

/// Distance taxied at each end, along the runway.
pub const TAXI_OUT_DISTANCE_M: f64 = 2_200.0;
pub const TAXI_IN_DISTANCE_M: f64 = 1_800.0;

/// Angle of attack model.
///
/// `alpha ~ C / (rho * V^2)` is just lift equals weight rearranged, with everything
/// constant folded into `C`. Calibrated so cruise sits at about 2.5° nose-up. The flap
/// term matters as much as the rest: extending flaps increases camber, so the same lift
/// comes at a markedly lower body angle, which is why an airliner on a 3° glideslope is
/// still nose-up rather than nose-down.
pub const AOA_COEFFICIENT: f64 = 1_060.0;
pub const AOA_FLAP_RELIEF_RAD: f64 = 0.070; // 4 degrees at full flap
pub const FLAP_EXTEND_SPEED_MPS: f64 = 180.0;
pub const FLAP_FULL_SPEED_MPS: f64 = 100.0;
pub const MAX_AOA_RAD: f64 = 0.21;

/// Distance flown straight ahead on the runway heading after takeoff before the first
/// turn. Real departure procedures specify a climb to around 1,500 ft AGL before any
/// turn, and this is roughly how far that takes.
pub const DEPARTURE_LEG_M: f64 = 12_000.0;

/// Straight-in final approach length, aligned with the runway. Ten nautical miles is
/// the usual intercept point for an ILS.
pub const FINAL_APPROACH_M: f64 = 18_520.0;

/// Touchdown point past the threshold. Aiming markers are at 300 m.
pub const TOUCHDOWN_OFFSET_M: f64 = 300.0;

/// Height at which the flare begins, and the descent rate it arrests to.
pub const FLARE_HEIGHT_M: f64 = 15.0;
pub const TOUCHDOWN_SINK_MPS: f64 = 0.6;

/// Cruise altitude wanted for a given still-air trip distance, before it is quantised
/// to a usable flight level.
///
/// Short sectors never get high because the climb and descent alone consume the
/// distance; long sectors are limited by weight at the start and end up near the
/// ceiling once fuel has burned off.
pub fn optimum_cruise_altitude_m(trip_distance_m: f64) -> f64 {
    let km = trip_distance_m / 1000.0;
    let ft = if km < 200.0 {
        // Barely time to level off at all.
        16_000.0 + (km / 200.0) * 8_000.0
    } else if km < 600.0 {
        24_000.0 + ((km - 200.0) / 400.0) * 9_000.0
    } else if km < 1_500.0 {
        33_000.0 + ((km - 600.0) / 900.0) * 4_000.0
    } else {
        37_000.0 + ((km - 1_500.0) / 6_000.0).min(1.0) * 4_000.0
    };
    feet_to_m(ft)
}

/// Rate of climb at an altitude, in metres per second.
pub fn rate_of_climb(altitude_m: f64) -> f64 {
    let frac = (altitude_m / SERVICE_CEILING_M).clamp(0.0, 0.999);
    let roc = ROC_SEA_LEVEL_MPS * (1.0 - frac.powf(ROC_DECAY_EXPONENT));
    // A floor keeps the integration from stalling as the ceiling is approached; no
    // planned level is close enough for it to bind in practice.
    roc.max(0.6)
}

/// True airspeed the climb schedule commands, given the field the aircraft left.
///
/// The field elevation is needed because the first two segments are defined by height
/// above the runway, not by altitude: an aircraft leaving Bogotá is above 10,000 ft
/// before it has finished its initial climb.
pub fn climb_tas(altitude_m: f64, field_elevation_m: f64) -> f64 {
    use super::atmosphere::{tas_from_cas, tas_from_mach};
    let height = altitude_m - field_elevation_m;
    let cas = if height < ACCELERATION_HEIGHT_M {
        initial_climb_cas()
    } else if height < CLEAN_UP_HEIGHT_M {
        // Accelerating and retracting flap.
        let f = (height - ACCELERATION_HEIGHT_M) / (CLEAN_UP_HEIGHT_M - ACCELERATION_HEIGHT_M);
        initial_climb_cas() + (speed_limit_cas() - initial_climb_cas()) * f
    } else if altitude_m < SPEED_LIMIT_ALT_M {
        speed_limit_cas()
    } else {
        climb_cas()
    };
    // Whichever of the CAS and Mach schedules is currently limiting — the crossover
    // between them happens naturally around FL300.
    tas_from_cas(cas, altitude_m).min(tas_from_mach(CLIMB_MACH, altitude_m))
}

/// True airspeed the descent schedule commands at an altitude.
pub fn descent_tas(altitude_m: f64) -> f64 {
    use super::atmosphere::{tas_from_cas, tas_from_mach};
    if altitude_m < SPEED_LIMIT_ALT_M {
        tas_from_cas(speed_limit_cas(), altitude_m)
    } else {
        tas_from_cas(descent_cas(), altitude_m).min(tas_from_mach(DESCENT_MACH, altitude_m))
    }
}

/// True airspeed at cruise, respecting the low-altitude speed limit.
///
/// A short sector cruising below 10,000 ft is still bound by the 250 kt restriction, so
/// the Mach schedule cannot be applied blindly — without this an aircraft levelling at
/// FL040 would cruise at nearly 400 knots.
pub fn cruise_tas(mach: f64, altitude_m: f64) -> f64 {
    use super::atmosphere::{tas_from_cas, tas_from_mach};
    let by_mach = tas_from_mach(mach, altitude_m);
    if altitude_m < SPEED_LIMIT_ALT_M {
        by_mach.min(tas_from_cas(speed_limit_cas(), altitude_m))
    } else {
        by_mach
    }
}

/// Bank limit at an altitude.
pub fn max_bank(altitude_m: f64) -> f64 {
    if altitude_m > HIGH_ALT_BANK_THRESHOLD_M {
        HIGH_ALT_MAX_BANK_RAD
    } else {
        MAX_BANK_RAD
    }
}

/// Radius of a coordinated turn at the bank limit.
///
/// `r = v^2 / (g tan phi)`. The old fixed 4 km radius implied a 58° bank at cruise
/// speed, so this is the difference between an airliner and an aerobatic display.
pub fn turn_radius(tas: f64, altitude_m: f64) -> f64 {
    let bank = max_bank(altitude_m);
    (tas * tas / (9.80665 * bank.tan())).max(500.0)
}

/// Length of the takeoff roll from a field at this elevation.
///
/// Rotation happens at a fixed *calibrated* airspeed, so the true speed it corresponds
/// to — and the roll needed to reach it — grows as the air thins. This is the whole
/// reason high-elevation airports have such long runways.
pub fn ground_roll_distance(field_elevation_m: f64) -> f64 {
    use super::atmosphere::tas_from_cas;
    let v = tas_from_cas(rotation_speed(), field_elevation_m);
    v * v / (2.0 * TAKEOFF_ACCEL_MPS2)
}

/// Length of the landing rollout at a field of this elevation.
pub fn rollout_distance(field_elevation_m: f64) -> f64 {
    use super::atmosphere::tas_from_cas;
    let v = tas_from_cas(approach_speed(), field_elevation_m);
    v * v / (2.0 * ROLLOUT_DECEL_MPS2)
}

/// Body angle of attack for a speed and altitude.
pub fn angle_of_attack(tas: f64, altitude_m: f64) -> f64 {
    use super::atmosphere::density;
    let q = density(altitude_m) * tas * tas;
    if q < 1.0 {
        return 0.0;
    }
    let clean = AOA_COEFFICIENT / q;
    // Flaps come out as the aircraft slows, and take the body angle down with them.
    let flap = ((FLAP_EXTEND_SPEED_MPS - tas) / (FLAP_EXTEND_SPEED_MPS - FLAP_FULL_SPEED_MPS))
        .clamp(0.0, 1.0);
    (clean - flap * AOA_FLAP_RELIEF_RAD).clamp(0.0, MAX_AOA_RAD)
}
