//! Climatological upper-level wind.
//!
//! Real dispatch solves the route against a forecast wind grid downloaded before every
//! flight. There is no network here, so this is the next best thing: the jet streams
//! are climatologically well behaved, and a closed-form field reproduces the behaviour
//! that actually shapes routes.
//!
//! What the field has to get right, in order of importance to the route:
//!
//! 1. **Mid-latitude westerlies with a jet core.** Two Gaussian bands, one per
//!    hemisphere, centred near 40° and peaking around FL340. This is what makes
//!    eastbound and westbound flights between the same two cities take visibly
//!    different paths.
//! 2. **Jet streaks.** The core is not uniform in longitude — it is strongest off the
//!    east coasts of North America and Asia, where the land/sea temperature contrast is
//!    sharpest. Those two streaks are why the North Atlantic and North Pacific have
//!    organised track systems and nowhere else does.
//! 3. **Seasonal migration.** The winter jet is stronger and sits further toward the
//!    equator; the summer jet is weaker and further poleward.
//!
//! The meridional component is deliberately zero. Climatologically it very nearly is,
//! and inventing one would put lateral structure into routes that has no basis.

use super::geo::wrap_deg_180;

/// Altitude of the jet core, and the vertical spread around it. 250 hPa is about
/// FL340, which is where the strongest winds are found and where airliners cruise —
/// not a coincidence in either direction.
const JET_CORE_ALT_M: f64 = 10_400.0;
const JET_VERTICAL_SPREAD_M: f64 = 3_800.0;

/// Fraction of the core wind that survives all the way to the surface.
///
/// The jet is the *maximum* of the mid-latitude westerlies, not the whole of them: the
/// westerlies run through the depth of the troposphere and merely strengthen with
/// height. Modelling the vertical shape as a bare Gaussian about the core made the wind
/// vanish below about 4 km, which left the surface dead calm and runway selection with
/// nothing to go on.
const SURFACE_FRACTION: f64 = 0.35;

/// Latitudinal spread of the jet band.
const JET_LAT_SPREAD_DEG: f64 = 11.0;

/// Annual-mean core latitude and speed, and the seasonal swing about them.
const NH_CORE_LAT_DEG: f64 = 40.0;
const NH_CORE_LAT_SWING_DEG: f64 = 6.0;
const NH_CORE_SPEED_MPS: f64 = 30.0;
const NH_CORE_SPEED_SWING_MPS: f64 = 12.0;

/// The southern jet is stronger on average and far less seasonal — there is almost no
/// land in the Southern Ocean to break it up.
const SH_CORE_LAT_DEG: f64 = -45.0;
const SH_CORE_LAT_SWING_DEG: f64 = 4.0;
const SH_CORE_SPEED_MPS: f64 = 32.0;
const SH_CORE_SPEED_SWING_MPS: f64 = 7.0;

/// Jet streak centres and their strength multipliers.
const ATLANTIC_STREAK_LON_DEG: f64 = -70.0;
const PACIFIC_STREAK_LON_DEG: f64 = 145.0;
const STREAK_SPREAD_DEG: f64 = 28.0;
const STREAK_GAIN: f64 = 0.55;

/// Tropical easterlies, weak at cruise level but enough to matter on equatorial routes.
const TROPICAL_EASTERLY_MPS: f64 = 6.0;
const TROPICAL_SPREAD_DEG: f64 = 13.0;

/// A closed-form wind field.
#[derive(Debug, Clone, Copy)]
pub struct WindField {
    /// -1 at the northern summer solstice, +1 at the northern winter solstice.
    seasonal: f64,
}

impl WindField {
    /// The annual-mean field. Used when the caller has no date, which is the normal
    /// case — a focus session is not tied to a calendar day.
    pub fn annual_mean() -> Self {
        Self { seasonal: 0.0 }
    }

    /// The field for a given day of the year (1–365).
    pub fn for_day_of_year(day: f64) -> Self {
        // Peaks in mid-January, two weeks after the solstice, matching the lag between
        // minimum insolation and maximum temperature gradient.
        let phase = (day - 15.0) / 365.25 * std::f64::consts::TAU;
        Self {
            seasonal: phase.cos(),
        }
    }

    /// A still-air field. Used to check that route geometry is right before wind is
    /// allowed to bend it, and available as a configuration option.
    pub fn calm() -> Self {
        Self { seasonal: f64::NAN }
    }

    fn is_calm(&self) -> bool {
        self.seasonal.is_nan()
    }

    /// Wind at a position, as (eastward, northward) metres per second.
    pub fn sample(&self, lat_deg: f64, lon_deg: f64, altitude_m: f64) -> (f64, f64) {
        if self.is_calm() {
            return (0.0, 0.0);
        }

        // Vertical shape is shared by every term: a floor through the troposphere that
        // rises to the full core value at the jet, and a clean decay above it into the
        // stratosphere, where the westerlies genuinely do die away.
        let dz = (altitude_m - JET_CORE_ALT_M) / JET_VERTICAL_SPREAD_M;
        let gaussian = (-dz * dz).exp();
        let vertical = if altitude_m < JET_CORE_ALT_M {
            SURFACE_FRACTION + (1.0 - SURFACE_FRACTION) * gaussian
        } else {
            gaussian
        };

        let nh_core_lat = NH_CORE_LAT_DEG - NH_CORE_LAT_SWING_DEG * self.seasonal;
        let nh_speed = NH_CORE_SPEED_MPS + NH_CORE_SPEED_SWING_MPS * self.seasonal;
        // The southern hemisphere's winter is the northern hemisphere's summer.
        let sh_core_lat = SH_CORE_LAT_DEG + SH_CORE_LAT_SWING_DEG * self.seasonal;
        let sh_speed = SH_CORE_SPEED_MPS - SH_CORE_SPEED_SWING_MPS * self.seasonal;

        let band = |centre: f64| {
            let d = (lat_deg - centre) / JET_LAT_SPREAD_DEG;
            (-d * d).exp()
        };

        let streak = |centre_lon: f64| {
            let d = wrap_deg_180(lon_deg - centre_lon) / STREAK_SPREAD_DEG;
            (-d * d).exp()
        };
        let streak_gain = 1.0
            + STREAK_GAIN * streak(ATLANTIC_STREAK_LON_DEG)
            + STREAK_GAIN * streak(PACIFIC_STREAK_LON_DEG);

        let mut u = nh_speed * band(nh_core_lat) * streak_gain + sh_speed * band(sh_core_lat);

        // Easterlies over the tropics.
        let t = lat_deg / TROPICAL_SPREAD_DEG;
        u -= TROPICAL_EASTERLY_MPS * (-t * t).exp();

        (u * vertical, 0.0)
    }

    /// Surface wind, for choosing which end of a runway is in use.
    ///
    /// Derived from the gradient wind rather than modelled separately: friction slows
    /// the surface layer to roughly a third of the wind above it and backs it toward
    /// low pressure, which is the Ekman spiral. The backing is mirrored in the southern
    /// hemisphere because the Coriolis deflection is.
    ///
    /// The consequence worth knowing: with a purely zonal field aloft, this produces a
    /// prevailing westerly at the surface in mid-latitudes, so mid-latitude airports
    /// mostly use their westerly-facing runway ends. That is genuinely what happens —
    /// Heathrow lands to the west roughly seven days in ten.
    pub fn surface_wind(&self, lat_deg: f64, lon_deg: f64) -> (f64, f64) {
        if self.is_calm() {
            return (0.0, 0.0);
        }
        // Sample at the surface: the vertical profile already carries the reduction
        // through the troposphere, so the friction factor here is only the boundary
        // layer itself and must not double-count it.
        let (u, v) = self.sample(lat_deg, lon_deg, 0.0);
        const FRICTION_FACTOR: f64 = 0.6;
        let backing = (if lat_deg >= 0.0 { 25.0_f64 } else { -25.0_f64 }).to_radians();
        let (sin_b, cos_b) = backing.sin_cos();
        (
            FRICTION_FACTOR * (u * cos_b + v * sin_b),
            FRICTION_FACTOR * (-u * sin_b + v * cos_b),
        )
    }
}

/// Solving the wind triangle for an aircraft holding a given track.
///
/// The crosswind component has to be cancelled by pointing the nose off the track, so
/// only what is left of the airspeed drives the aircraft along it — hence the
/// `sqrt(V^2 - w_cross^2)` rather than a plain subtraction. Getting this right is what
/// makes an eastbound crossing genuinely faster than the westbound one, matching the
/// scheduled times the route database already carries.
#[derive(Debug, Clone, Copy)]
pub struct WindTriangle {
    pub ground_speed: f64,
    /// Added to the track to get the heading the nose actually points at. Negative in a
    /// wind from the left, which pushes the aircraft right and has to be crabbed into.
    pub heading_offset_rad: f64,
}

/// `track_rad` is the desired course, clockwise from north.
pub fn solve_wind_triangle(
    track_rad: f64,
    tas: f64,
    wind_east: f64,
    wind_north: f64,
) -> Option<WindTriangle> {
    let (sin_t, cos_t) = track_rad.sin_cos();
    let along = wind_east * sin_t + wind_north * cos_t;
    let cross = wind_east * cos_t - wind_north * sin_t;
    if tas <= 0.0 || cross.abs() >= tas {
        // Cannot hold the track — the crosswind exceeds the airspeed. Impossible for an
        // airliner, but the guard keeps the optimiser from producing a NaN cost.
        return None;
    }
    let head_component = (tas * tas - cross * cross).sqrt();
    let ground_speed = head_component + along;
    if ground_speed <= 1.0 {
        return None;
    }
    Some(WindTriangle {
        ground_speed,
        heading_offset_rad: -(cross / tas).asin(),
    })
}
