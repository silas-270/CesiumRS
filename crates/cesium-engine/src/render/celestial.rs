//! Where the sun and the moon are, and what colour the light from them is.
//!
//! **Nothing here reads a clock.** The sky is a function of one number — how deep into
//! the flight the aircraft is — and that number is the same climbing as descending, so
//! the whole arc is symmetric by construction:
//!
//! | depth | altitude      | sky                                 |
//! |-------|---------------|-------------------------------------|
//! | 1.0   | on the ground | high sun, bright blue               |
//! | ~0.35 | climb/descent | sun on the horizon, orange          |
//! | 0.0   | cruise        | sun down, grey and dark, moon up    |
//!
//! This is a mood, not an ephemeris. An earlier version computed the real solar position
//! from the device clock. It was accurate and completely wrong for the product: the view
//! is about the climb into focus and back out of it, and a real sun makes that depend on
//! what time the user happened to start working.
//!
//! The sun's *bearing* is fixed in the local horizon frame rather than derived from
//! anything, so it does not swing about as the aircraft turns. Only its elevation moves.

use glam::Vec3;

/// Depth at which the sun sits exactly on the horizon, and how fast it crosses.
///
/// Set against what flights actually reach rather than against the nominal range: the
/// depth scalar bottoms out around 0.3 on a short sector, because it is measured against
/// that flight's own cruise altitude and the aircraft spends little time at it. Putting
/// the crossing at 0.35 left the cruise sitting in permanent twilight. At 0.55 the sun
/// goes down through the climb and the cruise is properly night on a short hop and a
/// long one alike.
const HORIZON_DEPTH: f32 = 0.55;
/// Highest the sun gets, as a sine, and how sharply it approaches the horizon.
///
/// The curve matters as much as the endpoints. A straight line from cruise to the ground
/// runs the sun through the last few degrees above the horizon almost instantly, so the
/// sky turned red while the sun was still high — which is not what a sunset looks like.
/// An exponent above one flattens the curve about the crossing, so the sun lingers near
/// the horizon exactly where all the colour is and climbs away quickly afterwards.
const PEAK_ELEVATION: f32 = 0.85;
const HORIZON_EASING: f32 = 1.5;

/// Where the sky stops being blue and where it finishes turning to night, both as sines
/// of the sun's elevation.
///
/// `DAY_ELEVATION` is about six degrees — the top of the golden hour. Anything higher and
/// the sky reddens with the sun still well up.
pub const DAY_ELEVATION: f32 = 0.10;
pub const DUSK_ELEVATION: f32 = -0.02;
pub const NIGHT_ELEVATION: f32 = -0.22;

/// The same crossing, for the light's *hue* rather than its brightness — and deliberately
/// later.
///
/// The sky and the light run the same dusk ramp but mix toward opposite ends: the sky
/// fades orange into a dark grey, which scales both channels down together and so stays
/// visibly orange, while the light fades orange into moonlight, which is bright and
/// neutral and kills the warmth almost at once. Sharing one ramp left the aircraft pale
/// against a sky that was still burning. Holding the hue back to here keeps the two
/// together through the whole of dusk.
const HUE_DUSK_ELEVATION: f32 = -0.12;
const HUE_NIGHT_ELEVATION: f32 = -0.30;

/// A fixed phase. With no date there is no real one, and a near-full moon is both the
/// most legible and the most useful as a light source.
const MOON_ILLUMINATION: f32 = 0.9;

/// The sky at a point in the flight.
#[derive(Debug, Clone, Copy)]
pub struct Celestial {
    /// Unit vector toward the sun, in the engine's world frame.
    pub sun_dir: Vec3,
    pub moon_dir: Vec3,
    /// Lit fraction of the moon's disc.
    pub moon_illumination: f32,
    /// Sine of the sun's elevation above the horizon.
    pub sun_elevation: f32,
    /// Hue of the key light, normalised so its brightest channel is 1.0.
    pub light_color: Vec3,
    /// Strength of that key light: 0 at cruise, 1 on the ground.
    pub light_strength: f32,
}

fn smoothstep(low: f32, high: f32, x: f32) -> f32 {
    let t = ((x - low) / (high - low)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// East and north at a point on the globe, in the engine's world frame.
///
/// That frame puts the polar axis on **+Y** and runs longitude toward **-Z**, so east
/// falls out of a cross product with the pole. See `globe::geometry::lon_lat_to_ecef_f64`.
fn horizon_frame(up: Vec3) -> (Vec3, Vec3) {
    let mut east = Vec3::Y.cross(up);
    if east.length_squared() < 1e-8 {
        // Directly over a pole: any horizontal direction will do.
        east = Vec3::X;
    }
    let east = east.normalize();
    (east, up.cross(east).normalize())
}

/// The sky at `depth`, where 1.0 is on the runway and 0.0 is at cruise.
pub fn compute(depth: f32, observer_up: Vec3) -> Celestial {
    let depth = depth.clamp(0.0, 1.0);
    // Normalised either side of the crossing, then eased so the sun slows as it reaches
    // the horizon.
    let u = if depth >= HORIZON_DEPTH {
        (depth - HORIZON_DEPTH) / (1.0 - HORIZON_DEPTH)
    } else {
        (depth - HORIZON_DEPTH) / HORIZON_DEPTH
    };
    let sun_elevation = u.signum() * u.abs().powf(HORIZON_EASING) * PEAK_ELEVATION;

    let up = observer_up.normalize_or_zero();
    let up = if up == Vec3::ZERO { Vec3::Y } else { up };
    let (east, north) = horizon_frame(up);

    // Held due west and a little south, so the sun sets ahead-left of a northbound
    // aircraft and the low light rakes across the cockpit rather than sitting behind it.
    let bearing = (-east * 0.92 - north * 0.39).normalize();
    let angle = sun_elevation.clamp(-1.0, 1.0).asin();
    let sun_dir = (up * angle.sin() + bearing * angle.cos()).normalize();

    // Opposite the sun, so it climbs into the sky exactly as the sun leaves it and is well
    // up by the time the flight reaches cruise.
    let moon_dir = -sun_dir;

    // These two ramps are the same ones the sky shaders run on, and they have to stay
    // that way. Previously the light on the aircraft faded out over its own, faster
    // schedule, so the moment the sun dipped below the horizon the aeroplane went grey
    // while the sky behind it was still burning orange.
    let day_amount = smoothstep(0.0, DAY_ELEVATION, sun_elevation);
    let night_amount = smoothstep(DUSK_ELEVATION, NIGHT_ELEVATION, sun_elevation);

    // Amber as the sun reaches the horizon, near-white with it overhead.
    let horizon_hue = Vec3::new(1.0, 0.55, 0.25);
    let noon_hue = Vec3::new(1.0, 0.97, 0.92);
    // Moonlight is kept close to neutral rather than the blue a real night reads as: the
    // cruise is meant to be grey and quiet, and a blue cast fights the greyscale the rest
    // of the view drains to.
    let moon_hue = Vec3::new(0.85, 0.87, 0.92);

    let sun_hue = horizon_hue.lerp(noon_hue, day_amount);
    let hue_night = smoothstep(HUE_DUSK_ELEVATION, HUE_NIGHT_ELEVATION, sun_elevation);
    let light_color = sun_hue.lerp(moon_hue, hue_night);

    // A low sun is a dim one as well as a red one.
    let sun_strength = (1.0 - night_amount) * (0.55 + 0.45 * day_amount);
    let moon_strength = night_amount * (0.05 + 0.15 * MOON_ILLUMINATION);
    let total = sun_strength + moon_strength;
    let peak = light_color.max_element().max(1e-4);

    Celestial {
        sun_dir,
        moon_dir,
        moon_illumination: MOON_ILLUMINATION,
        sun_elevation,
        light_color: light_color / peak,
        light_strength: total.min(1.0),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Somewhere over Germany, roughly.
    fn up() -> Vec3 {
        Vec3::new(0.62, 0.74, -0.25).normalize()
    }

    #[test]
    fn the_ground_is_daylight_and_the_cruise_is_night() {
        let ground = compute(1.0, up());
        let cruise = compute(0.0, up());

        assert!(ground.sun_elevation > 0.6, "sun was low on the ground");
        assert!(cruise.sun_elevation < -0.6, "sun was still up at cruise");
        assert!(ground.light_strength > 0.95, "daylight was weak");
        assert!(cruise.light_strength < 0.25, "cruise was too bright");

        assert!(ground.sun_dir.dot(up()) > 0.6, "sun below the horizon on the ground");
        assert!(cruise.sun_dir.dot(up()) < -0.6, "sun above the horizon at cruise");
        assert!(cruise.moon_dir.dot(up()) > 0.6, "no moon at cruise");
    }

    /// A short sector's depth scalar bottoms out here rather than at zero, so this is the
    /// value that has to read as night.
    const SHALLOWEST_CRUISE_DEPTH: f32 = 0.30;

    #[test]
    fn a_short_sector_still_reaches_night_at_cruise() {
        let c = compute(SHALLOWEST_CRUISE_DEPTH, up());
        assert!(
            c.sun_elevation < NIGHT_ELEVATION,
            "sun was at {} at a short sector's cruise; it needs to be past {}",
            c.sun_elevation,
            NIGHT_ELEVATION
        );
        assert!(c.light_strength < 0.25, "short-sector cruise was not dark");
        // Fifteen degrees up. Not high, but unambiguously in the sky.
        assert!(
            c.moon_dir.dot(up()) > 0.2,
            "moon was only {} above the horizon at a short sector's cruise",
            c.moon_dir.dot(up())
        );
    }

    #[test]
    fn the_sun_crosses_the_horizon_mid_climb() {
        let at_horizon = compute(HORIZON_DEPTH, up());
        assert!(
            at_horizon.sun_elevation.abs() < 0.02,
            "sun was at {} when it should have been on the horizon",
            at_horizon.sun_elevation
        );
        let c = at_horizon.light_color;
        assert!(c.x > c.z * 1.5, "horizon light was not warm: {c:?}");
    }

    #[test]
    fn the_light_stays_warm_while_the_sky_is_still_red() {
        // Just below the horizon the sky is at its most orange. The light falling on the
        // aircraft has to still be orange there, or the aeroplane turns grey against a
        // red sky — which is what it used to do.
        let dusk = compute(HORIZON_DEPTH - 0.04, up());
        assert!(dusk.sun_elevation < 0.0, "expected the sun below the horizon");
        let c = dusk.light_color;
        assert!(
            c.x > c.z * 1.5,
            "light had already gone neutral at {:?} with the sun at {}",
            c,
            dusk.sun_elevation
        );
    }

    #[test]
    fn the_sky_only_reddens_near_the_horizon() {
        // With the sun well up the light must still be white. Six degrees is the top of
        // the golden hour; at twice that there should be no warmth left at all.
        let high = compute(0.75, up());
        assert!(
            high.sun_elevation > DAY_ELEVATION,
            "test point was not a high sun: {}",
            high.sun_elevation
        );
        let c = high.light_color;
        assert!(
            c.z > 0.85,
            "light was still warm at elevation {}: {:?}",
            high.sun_elevation,
            c
        );
    }

    #[test]
    fn the_warmth_outlasts_the_sun_going_down() {
        // The sky is still strongly orange a good way below the horizon, so the light has
        // to be too. Measured as the red-to-blue ratio of the key light.
        let warmth = |d: f32| {
            let c = compute(d, up()).light_color;
            c.x / c.z.max(1e-6)
        };
        // Just past the crossing, and a good way past it.
        assert!(warmth(0.50) > 3.0, "warmth at the horizon was {}", warmth(0.50));
        assert!(warmth(0.42) > 3.0, "warmth below the horizon was {}", warmth(0.42));
        // And it has gone by the time the sky has.
        assert!(warmth(0.30) < 1.2, "still warm at cruise: {}", warmth(0.30));
    }

    #[test]
    fn the_light_only_ever_moves_one_way_with_depth() {
        // Symmetry comes from `depth` itself being symmetric over the flight, so all this
        // has to guarantee is that nothing doubles back — otherwise the descent would not
        // retrace the climb.
        let mut last = compute(0.0, up());
        for i in 1..=50 {
            let c = compute(i as f32 / 50.0, up());
            assert!(
                c.sun_elevation >= last.sun_elevation,
                "sun elevation dipped between steps"
            );
            assert!(
                c.light_strength >= last.light_strength - 1e-6,
                "light strength dipped between steps"
            );
            last = c;
        }
    }

    #[test]
    fn the_sun_bearing_does_not_wander_with_the_aircraft() {
        // Same depth, two places on the globe: the sun must sit at the same elevation
        // above each local horizon, or it would appear to swing as the flight moves.
        let a = Vec3::new(0.62, 0.74, -0.25).normalize();
        let b = Vec3::new(-0.30, 0.80, 0.52).normalize();
        let ea = compute(0.6, a).sun_dir.dot(a);
        let eb = compute(0.6, b).sun_dir.dot(b);
        assert!((ea - eb).abs() < 1e-5, "sun elevation differed: {ea} vs {eb}");
    }
}
