//! International Standard Atmosphere, and the airspeed conversions built on it.
//!
//! The profile needs this because the speeds a crew actually flies are *indicated*
//! (calibrated) airspeeds and Mach numbers, not ground speeds. "250 knots below ten
//! thousand feet" is a CAS restriction; at FL350 the same 250 kt CAS is 430 kt true.
//! Converting properly is what makes the climb take its real 20 minutes instead of a
//! made-up number.

/// Sea-level ISA values.
pub const SEA_LEVEL_PRESSURE_PA: f64 = 101_325.0;
pub const SEA_LEVEL_TEMPERATURE_K: f64 = 288.15;
pub const SEA_LEVEL_DENSITY: f64 = 1.225;
pub const SEA_LEVEL_SOUND_MPS: f64 = 340.294;

const LAPSE_RATE_K_PER_M: f64 = 0.0065;
const TROPOPAUSE_M: f64 = 11_000.0;
const TROPOPAUSE_TEMPERATURE_K: f64 = 216.65;
const TROPOPAUSE_PRESSURE_PA: f64 = 22_632.06;
const GAS_CONSTANT_AIR: f64 = 287.052_87;
const GRAVITY: f64 = 9.806_65;
const GAMMA: f64 = 1.4;

pub fn temperature_k(altitude_m: f64) -> f64 {
    if altitude_m < TROPOPAUSE_M {
        SEA_LEVEL_TEMPERATURE_K - LAPSE_RATE_K_PER_M * altitude_m
    } else {
        // Isothermal through the lower stratosphere, which covers every altitude an
        // airliner cruises at above the tropopause.
        TROPOPAUSE_TEMPERATURE_K
    }
}

pub fn pressure_pa(altitude_m: f64) -> f64 {
    if altitude_m < TROPOPAUSE_M {
        let t = temperature_k(altitude_m);
        SEA_LEVEL_PRESSURE_PA
            * (t / SEA_LEVEL_TEMPERATURE_K).powf(GRAVITY / (LAPSE_RATE_K_PER_M * GAS_CONSTANT_AIR))
    } else {
        let dh = altitude_m - TROPOPAUSE_M;
        TROPOPAUSE_PRESSURE_PA
            * (-GRAVITY * dh / (GAS_CONSTANT_AIR * TROPOPAUSE_TEMPERATURE_K)).exp()
    }
}

pub fn density(altitude_m: f64) -> f64 {
    pressure_pa(altitude_m) / (GAS_CONSTANT_AIR * temperature_k(altitude_m))
}

pub fn speed_of_sound(altitude_m: f64) -> f64 {
    (GAMMA * GAS_CONSTANT_AIR * temperature_k(altitude_m)).sqrt()
}

pub fn tas_from_mach(mach: f64, altitude_m: f64) -> f64 {
    mach * speed_of_sound(altitude_m)
}

pub fn mach_from_tas(tas: f64, altitude_m: f64) -> f64 {
    tas / speed_of_sound(altitude_m)
}

/// Calibrated airspeed to true airspeed, via the compressible-flow relations.
///
/// The incompressible `TAS = CAS / sqrt(sigma)` shortcut is 8% low by FL350, which is
/// most of a climb segment's error budget, so the full form is used.
pub fn tas_from_cas(cas: f64, altitude_m: f64) -> f64 {
    // Impact pressure from CAS, evaluated in the sea-level atmosphere by definition.
    let qc =
        SEA_LEVEL_PRESSURE_PA * ((1.0 + 0.2 * (cas / SEA_LEVEL_SOUND_MPS).powi(2)).powf(3.5) - 1.0);
    let p = pressure_pa(altitude_m);
    let mach_sq = 5.0 * ((qc / p + 1.0).powf(2.0 / 7.0) - 1.0);
    mach_sq.max(0.0).sqrt() * speed_of_sound(altitude_m)
}

/// Density ratio against sea level. Drives the angle-of-attack model.
pub fn density_ratio(altitude_m: f64) -> f64 {
    density(altitude_m) / SEA_LEVEL_DENSITY
}

/// Feet to metres. Flight levels and field elevations arrive in feet.
pub fn feet_to_m(ft: f64) -> f64 {
    ft * 0.3048
}

pub fn m_to_feet(m: f64) -> f64 {
    m / 0.3048
}

/// Knots to metres per second.
pub fn knots_to_mps(kt: f64) -> f64 {
    kt * 0.514_444
}
