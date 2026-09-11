//! The vertical profile: altitude as a function of distance along the ground track.
//!
//! Three things here are worth knowing before changing any of it.
//!
//! **Climb and descent distances are derived, not chosen.** The climb is integrated
//! against a rate that decays toward the ceiling, and the descent is flown at the 3°
//! that both an idle descent and an ILS glideslope happen to sit at. Together they put
//! the top of climb about 240 km out and the top of descent about 200 km before the
//! runway. Those distances are outputs. Picking them, as a fixed 30 km and 50 km, is
//! what produced a 23° climb angle — steeper than a fighter leaves the runway at.
//!
//! **Cruise happens at a flight level, not an altitude.** Levels are separated by
//! 1,000 ft, and which ones are available depends on the direction of flight: the
//! semicircular rule gives odd levels to easterly tracks and even ones to westerly.
//! This is why an aircraft going one way is never at the same altitude as one coming
//! back, and it is a detail people recognise.
//!
//! **Long cruises are a staircase.** An aircraft cannot reach its best altitude while
//! full of fuel, so it starts lower and steps up as it burns off — 2,000 ft at a time,
//! because that is what keeps it on the right side of the semicircular rule. A
//! long-haul altitude trace that is flat is wrong.

use super::aircraft::{self, MAX_CRUISE_FL, MIN_CRUISE_FL};
use super::atmosphere::{feet_to_m, m_to_feet, tas_from_cas};

/// Altitude step used to integrate the climb and descent.
const INTEGRATION_STEP_M: f64 = 50.0;

/// Cruise time between step climbs, and the most steps a flight will plan.
const STEP_INTERVAL_S: f64 = 9_000.0; // 2.5 hours
const MAX_STEPS: usize = 3;
/// Each step is 2,000 ft — one level in the same direction, keeping the semicircular
/// parity intact.
const STEP_FL: i32 = 20;

/// Minimum flat cruise band held between top-of-climb and top-of-descent, so that even
/// the shortest sector levels off rather than meeting climb and descent at a point.
const MIN_CRUISE_BAND_M: f64 = 5_000.0;

/// Vertical acceleration this profile is shaped to stay under, in m/s².
///
/// The consumer interpolates the sampled positions with a spline and takes the
/// aircraft's motion from it, so a change of flight path angle is read as an
/// acceleration of `v² dγ/ds`. Every corner in the profile — rotation, levelling off,
/// starting down — is therefore spread over enough distance to keep that below this
/// figure. It sits well under the 3.5 m/s² a passenger would call a jolt because the
/// session's time scale can stretch or compress the clock the spline is walked along,
/// and that scales the acceleration by its square.
const VERTICAL_ACCEL_BUDGET: f64 = 1.6;

/// Speeds are known here only to the accuracy of the schedule; the wind and the time
/// scale move the speed the spline actually sees. Transitions are sized for a speed
/// this much higher than planned so that margin is not spent by surprise.
const TRANSITION_SPEED_MARGIN: f64 = 1.3;

/// Shortest transition worth building, and how many knots each one gets.
const MIN_TRANSITION_M: f64 = 200.0;
const TRANSITION_KNOTS: usize = 20;

/// Track distance reserved for the rotation arc when the flight levels are planned.
/// The arc gains height more slowly than the climb it replaces, so it costs a little
/// extra ground; this is a generous allowance for that.
const ROTATION_RESERVE_M: f64 = 2_000.0;

/// Where the profile is anchored on the ground track, and what it has to fit between.
pub struct VerticalInputs {
    pub s_total: f64,
    pub s_dep_threshold: f64,
    pub s_touchdown: f64,
    pub s_rollout_end: f64,
    pub dep_elevation_m: f64,
    pub arr_elevation_m: f64,
    /// Overall direction of the route, for the semicircular rule.
    pub track_rad: f64,
    pub trip_distance_m: f64,
    /// Used only to decide how many step climbs the cruise is long enough for.
    pub cruise_tas_estimate: f64,
}

pub struct VerticalProfile {
    /// Monotonic in distance; altitude is linearly interpolated between entries.
    samples: Vec<(f64, f64)>,
    tangents: Vec<f64>,
    pub s_rotate: f64,
    pub s_top_of_climb: f64,
    pub s_top_of_descent: f64,
    pub s_touchdown: f64,
    pub s_rollout_end: f64,
    pub s_dep_threshold: f64,
    pub s_total: f64,
    pub dep_elevation_m: f64,
    pub arr_elevation_m: f64,
    /// Highest level reached, and the level cruise begins at. They differ whenever the
    /// flight is long enough to step.
    pub cruise_altitude_m: f64,
    pub initial_cruise_altitude_m: f64,
    pub cruise_flight_level: i32,
    pub step_count: usize,
    pub ground_roll_m: f64,
    pub rollout_m: f64,
}

impl VerticalProfile {
    pub fn build(inputs: &VerticalInputs) -> Self {
        // Rotation and touchdown speeds are calibrated airspeeds, so the true speed —
        // and with it the length of the ground roll — grows with field elevation. This
        // is why hot-and-high airports need such long runways.
        let ground_roll_m = aircraft::ground_roll_distance(inputs.dep_elevation_m);
        let rollout_m = aircraft::rollout_distance(inputs.arr_elevation_m);
        let v_touchdown = tas_from_cas(aircraft::approach_speed(), inputs.arr_elevation_m);

        let s_rotate = inputs.s_dep_threshold + ground_roll_m;
        let flare_entry_alt = inputs.arr_elevation_m + aircraft::FLARE_HEIGHT_M;
        let flare_m = flare_distance(v_touchdown);
        let s_flare_start = inputs.s_touchdown - flare_m;

        // What the climb, the cruise and the descent have to share. The flare, the
        // rotation arc and the level band between top of climb and top of descent are
        // all taken off the top, so that a level the plan accepts really does fit.
        let available =
            (s_flare_start - s_rotate - ROTATION_RESERVE_M - MIN_CRUISE_BAND_M).max(1_000.0);

        let plan = plan_levels(inputs, available);

        let mut samples: Vec<(f64, f64)> = Vec::new();
        samples.push((0.0, inputs.dep_elevation_m));
        samples.push((s_rotate, inputs.dep_elevation_m));

        // Rotation. The runway is flat and the climb is not, so the two cannot simply
        // be joined: the flight path angle has to be brought up from zero over a
        // distance long enough that the pull-up is not felt as a corner. The gradient
        // follows a smoothstep, which is what a rotation looks like — the nose comes up
        // over several seconds, not instantly.
        let gamma_climb = climb_gradient(inputs.dep_elevation_m, inputs.dep_elevation_m)
            / plan.compression;
        let v_lift = aircraft::climb_tas(
            inputs.dep_elevation_m + aircraft::LIFTOFF_HEIGHT_M,
            inputs.dep_elevation_m,
        );
        let rotation_m = transition_length(gamma_climb, v_lift).min(available * 0.25);
        // A smoothstep gradient covers half the height a constant one would.
        let rotation_gain = 0.5 * gamma_climb * rotation_m;
        for k in 1..=TRANSITION_KNOTS {
            let w = k as f64 / TRANSITION_KNOTS as f64;
            samples.push((
                s_rotate + rotation_m * w,
                inputs.dep_elevation_m + gamma_climb * rotation_m * smoothstep_integral(w),
            ));
        }

        // Climb to the initial cruise level, picked up from where the rotation arc left
        // off so the two meet at the same gradient as well as the same height.
        let s_climb_start = s_rotate + rotation_m;
        let climb = integrate_climb(
            inputs.dep_elevation_m + rotation_gain,
            plan.initial_altitude_m,
            inputs.dep_elevation_m,
        );

        for (ds, alt) in &climb.points {
            samples.push((s_climb_start + ds * plan.compression, *alt));
        }

        let s_top_of_climb = s_climb_start + climb.distance_m * plan.compression;

        // Descent, measured back from the flare so the geometry lands on the runway.
        let descent =
            integrate_descent(plan.top_altitude_m, flare_entry_alt, inputs.arr_elevation_m);
        let s_top_of_descent = (s_flare_start - descent.distance_m * plan.compression)
            .max(s_top_of_climb + MIN_CRUISE_BAND_M);

        // Corners in the profile, collected as they are laid out and rounded off once
        // the whole thing is assembled.
        let mut corners: Vec<Corner> = Vec::new();
        corners.push(Corner {
            s: s_top_of_climb,
            delta: climb_gradient(plan.initial_altitude_m, inputs.dep_elevation_m)
                / plan.compression,
            speed: aircraft::climb_tas(plan.initial_altitude_m, inputs.dep_elevation_m),
        });

        let cruise_span = (s_top_of_descent - s_top_of_climb).max(0.0);
        let step_total: f64 = plan.step_distances.iter().sum::<f64>() * plan.compression;
        let level_total = (cruise_span - step_total).max(0.0);
        let level_each = level_total / (plan.step_distances.len() + 1) as f64;

        let mut s = s_top_of_climb;
        let mut alt = plan.initial_altitude_m;
        for (k, step_ds) in plan.step_distances.iter().enumerate() {
            // Densify level cruise if long
            let seg_steps = ((level_each / 50_000.0).ceil() as usize).max(1);
            for st in 1..=seg_steps {
                samples.push((s + level_each * (st as f64 / seg_steps as f64), alt));
            }
            s += level_each;
            let next = plan.step_altitudes_m[k];
            let step_len = step_ds * plan.compression;
            let step_climb = integrate_climb(alt, next, inputs.dep_elevation_m);
            for (ds, a) in &step_climb.points {
                samples.push((s + ds * plan.compression, *a));
            }
            // A step climb has a corner at each end, both the same size.
            let delta = climb_gradient(alt, inputs.dep_elevation_m) / plan.compression;
            corners.push(Corner {
                s,
                delta,
                speed: aircraft::cruise_tas(aircraft::NOMINAL_CRUISE_MACH, alt),
            });
            corners.push(Corner {
                s: s + step_len,
                delta,
                speed: aircraft::cruise_tas(aircraft::NOMINAL_CRUISE_MACH, next),
            });
            s += step_len;
            alt = next;
        }

        // Densify remaining cruise to top of descent
        if level_each > 0.0 {
            let seg_steps = ((level_each / 50_000.0).ceil() as usize).max(1);
            for st in 1..seg_steps {
                samples.push((s + level_each * (st as f64 / seg_steps as f64), alt));
            }
        }
        samples.push((s_top_of_descent, plan.top_altitude_m));

        // Stretched onto the distance actually left rather than laid out at the planned
        // compression: the level band above can push the top of descent later, and a
        // descent laid out from there at its own length would run past the flare and
        // fold the profile back on itself.
        let descent_scale = (s_flare_start - s_top_of_descent) / descent.distance_m.max(1.0);
        for (ds, a) in &descent.points {
            samples.push((s_top_of_descent + ds * descent_scale, *a));
        }
        samples.push((s_flare_start, flare_entry_alt));
        corners.push(Corner {
            s: s_top_of_descent,
            delta: aircraft::DESCENT_ANGLE_RAD.tan() * descent_scale.max(1e-6).recip(),
            speed: aircraft::descent_tas(plan.top_altitude_m),
        });

        // Flare: smooth C1 cubic transition matching glideslope slope at entry and touchdown sink rate
        let m0 = -aircraft::DESCENT_ANGLE_RAD.tan();
        let m1 = -(aircraft::TOUCHDOWN_SINK_MPS / v_touchdown.max(20.0)).clamp(0.001, 0.05);
        let h0 = aircraft::FLARE_HEIGHT_M;
        let h1 = 0.0;
        const FLARE_SAMPLES: usize = 16;
        for k in 1..FLARE_SAMPLES {
            let t = k as f64 / FLARE_SAMPLES as f64;
            let t2 = t * t;
            let t3 = t2 * t;
            let h00 = 2.0 * t3 - 3.0 * t2 + 1.0;
            let h10 = t3 - 2.0 * t2 + t;
            let h01 = -2.0 * t3 + 3.0 * t2;
            let h11 = t3 - t2;
            let height = h0 * h00 + flare_m * m0 * h10 + h1 * h01 + flare_m * m1 * h11;
            samples.push((s_flare_start + flare_m * t, inputs.arr_elevation_m + height.max(0.0)));
        }
        samples.push((inputs.s_touchdown, inputs.arr_elevation_m));
        samples.push((inputs.s_total, inputs.arr_elevation_m));

        // Ensure strictly sorted and deduplicated distances
        samples.retain(|(s, a)| s.is_finite() && a.is_finite());
        sort_samples(&mut samples);

        round_corners(
            &mut samples,
            &mut corners,
            s_climb_start,
            s_flare_start,
        );

        let tangents = compute_tangents(&samples, s_rotate, inputs.s_touchdown);

        Self {
            samples,
            tangents,
            s_rotate,
            s_top_of_climb,
            s_top_of_descent,
            s_touchdown: inputs.s_touchdown,
            s_rollout_end: inputs.s_rollout_end,
            s_dep_threshold: inputs.s_dep_threshold,
            s_total: inputs.s_total,
            dep_elevation_m: inputs.dep_elevation_m,
            arr_elevation_m: inputs.arr_elevation_m,
            cruise_altitude_m: plan.top_altitude_m,
            initial_cruise_altitude_m: plan.initial_altitude_m,
            cruise_flight_level: plan.top_fl,
            step_count: plan.step_distances.len(),
            ground_roll_m,
            rollout_m,
        }
    }

    pub fn altitude_at(&self, s: f64) -> f64 {
        if self.samples.is_empty() {
            return 0.0;
        }
        let idx = match self
            .samples
            .binary_search_by(|(x, _)| x.partial_cmp(&s).unwrap())
        {
            Ok(i) => return self.samples[i].1,
            Err(i) => i,
        };
        if idx == 0 {
            return self.samples[0].1;
        }
        if idx >= self.samples.len() {
            return self.samples[self.samples.len() - 1].1;
        }
        let (s0, a0) = self.samples[idx - 1];
        let (s1, a1) = self.samples[idx];
        let h = s1 - s0;
        if h <= 0.0 {
            return a0;
        }

        let m0 = self.tangents[idx - 1];
        let m1 = self.tangents[idx];

        let t = (s - s0) / h;
        let t2 = t * t;
        let t3 = t2 * t;
        let h00 = 2.0 * t3 - 3.0 * t2 + 1.0;
        let h10 = t3 - 2.0 * t2 + t;
        let h01 = -2.0 * t3 + 3.0 * t2;
        let h11 = t3 - t2;
        h00 * a0 + h10 * h * m0 + h01 * a1 + h11 * h * m1
    }

    /// Exact analytic gradient (dh/ds) of the altitude profile.
    pub fn gradient_at(&self, s: f64) -> f64 {
        if self.samples.len() < 2 {
            return 0.0;
        }
        if s <= self.s_rotate || s >= self.s_touchdown {
            return 0.0;
        }
        let idx = match self
            .samples
            .binary_search_by(|(x, _)| x.partial_cmp(&s).unwrap())
        {
            Ok(i) => return self.tangents[i],
            Err(i) => i,
        };
        if idx == 0 {
            return self.tangents[0];
        }
        if idx >= self.samples.len() {
            return self.tangents[self.samples.len() - 1];
        }
        let (s0, a0) = self.samples[idx - 1];
        let (s1, a1) = self.samples[idx];
        let h = s1 - s0;
        if h <= 0.0 {
            return 0.0;
        }
        let m0 = self.tangents[idx - 1];
        let m1 = self.tangents[idx];
        let d = (a1 - a0) / h;
        let t = (s - s0) / h;
        6.0 * t * (1.0 - t) * d + (1.0 - t) * (1.0 - 3.0 * t) * m0 + t * (3.0 * t - 2.0) * m1
    }

    /// Height above the nearer runway, used for the phase-dependent sampling rate and
    /// the sun-intensity curve.
    pub fn height_above_field(&self, s: f64) -> f64 {
        let field = if s < self.s_top_of_climb.max(self.s_total * 0.5) {
            self.dep_elevation_m
        } else {
            self.arr_elevation_m
        };
        (self.altitude_at(s) - field).max(0.0)
    }

    pub fn is_on_ground(&self, s: f64) -> bool {
        s <= self.s_rotate || s >= self.s_touchdown
    }

    pub fn samples(&self) -> &[(f64, f64)] {
        &self.samples
    }
}

/// A place where the profile's gradient changes abruptly, and what it takes to round
/// it off: how much the gradient moves, and how fast the aircraft is going through it.
struct Corner {
    s: f64,
    delta: f64,
    speed: f64,
}

/// The flight path angle, as a gradient, that the climb schedule holds at an altitude.
fn climb_gradient(altitude_m: f64, field_elevation_m: f64) -> f64 {
    let roc = aircraft::rate_of_climb(altitude_m);
    let tas = aircraft::climb_tas(altitude_m, field_elevation_m);
    let horizontal = (tas * tas - roc * roc).max(1.0).sqrt();
    roc / horizontal
}

/// Distance a gradient change of `delta` has to be spread over to stay inside the
/// vertical acceleration budget at `speed`.
///
/// The gradient is moved by a smoothstep, which is steepest at its midpoint, where
/// `dγ/ds` is `1.5 delta / L`. The acceleration that produces is `v² dγ/ds`, so the
/// length falls straight out of the budget.
fn transition_length(delta: f64, speed: f64) -> f64 {
    let v = speed * TRANSITION_SPEED_MARGIN;
    (1.5 * delta.abs() * v * v / VERTICAL_ACCEL_BUDGET).max(MIN_TRANSITION_M)
}

/// `∫₀^w 3u² - 2u³ du`, the height gained by a gradient following a smoothstep.
fn smoothstep_integral(w: f64) -> f64 {
    w * w * w * (1.0 - 0.5 * w)
}

fn sort_samples(samples: &mut Vec<(f64, f64)>) {
    samples.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
    samples.dedup_by(|a, b| (a.0 - b.0).abs() < 1e-6);
}

/// Altitude of the assembled profile read off its knots, before any tangents exist.
fn linear_altitude(samples: &[(f64, f64)], s: f64) -> f64 {
    match samples.binary_search_by(|(x, _)| x.partial_cmp(&s).unwrap()) {
        Ok(i) => samples[i].1,
        Err(0) => samples[0].1,
        Err(i) if i >= samples.len() => samples[samples.len() - 1].1,
        Err(i) => {
            let (s0, a0) = samples[i - 1];
            let (s1, a1) = samples[i];
            if s1 <= s0 {
                a0
            } else {
                a0 + (a1 - a0) * (s - s0) / (s1 - s0)
            }
        }
    }
}

/// Rounds off every corner in the profile, each over as much distance as its own size
/// and speed call for, without letting two of them run into each other or into the
/// rotation arc and the flare at either end.
fn round_corners(
    samples: &mut Vec<(f64, f64)>,
    corners: &mut Vec<Corner>,
    s_first: f64,
    s_last: f64,
) {
    corners.retain(|c| c.s > s_first && c.s < s_last && c.delta.abs() > 1e-6);
    corners.sort_by(|a, b| a.s.partial_cmp(&b.s).unwrap());
    for i in 0..corners.len() {
        let lo = if i == 0 {
            s_first
        } else {
            0.5 * (corners[i - 1].s + corners[i].s)
        };
        let hi = if i + 1 == corners.len() {
            s_last
        } else {
            0.5 * (corners[i].s + corners[i + 1].s)
        };
        let want = 0.5 * transition_length(corners[i].delta, corners[i].speed);
        let half = want.min(corners[i].s - lo).min(hi - corners[i].s);
        if half > 1.0 {
            round_corner(samples, corners[i].s, half);
        }
    }
}

/// Replaces the knots within `half` of `s_c` with a curve whose gradient moves between
/// the two half-window secants along a smoothstep.
///
/// The window's two ends keep the altitude they already had — a smoothstep gains
/// exactly the height the corner it replaces did — so this is a local operation: the
/// profile either side of it, and the altitude the aircraft reaches, are untouched.
fn round_corner(samples: &mut Vec<(f64, f64)>, s_c: f64, half: f64) {
    let s0 = s_c - half;
    let s1 = s_c + half;
    let h0 = linear_altitude(samples, s0);
    let hc = linear_altitude(samples, s_c);
    let h1 = linear_altitude(samples, s1);
    let g0 = (hc - h0) / half;
    let g1 = (h1 - hc) / half;

    samples.retain(|(s, _)| *s <= s0 || *s >= s1);
    let length = 2.0 * half;
    samples.push((s0, h0));
    samples.push((s1, h1));
    for k in 1..TRANSITION_KNOTS {
        let w = k as f64 / TRANSITION_KNOTS as f64;
        let h = h0 + g0 * length * w + (g1 - g0) * length * smoothstep_integral(w);
        samples.push((s0 + length * w, h));
    }
    sort_samples(samples);
}

fn compute_tangents(samples: &[(f64, f64)], s_rotate: f64, s_touchdown: f64) -> Vec<f64> {
    let n = samples.len();
    if n < 2 {
        return vec![0.0; n];
    }
    let secant = |i: usize| -> f64 {
        let (x0, y0) = samples[i];
        let (x1, y1) = samples[i + 1];
        if x1 > x0 {
            (y1 - y0) / (x1 - x0)
        } else {
            0.0
        }
    };
    let mut tangents = Vec::with_capacity(n);
    for i in 0..n {
        let s = samples[i].0;
        if s <= s_rotate || s >= s_touchdown {
            tangents.push(0.0);
            continue;
        }
        if i == 0 {
            tangents.push(secant(0));
        } else if i == n - 1 {
            tangents.push(secant(n - 2));
        } else {
            let prev = secant(i - 1);
            let next = secant(i);
            if prev * next <= 0.0 {
                tangents.push(0.0);
            } else {
                let (x0, _) = samples[i - 1];
                let (x1, _) = samples[i];
                let (x2, _) = samples[i + 1];
                let w1 = 2.0 * (x2 - x1) + (x1 - x0);
                let w2 = (x2 - x1) + 2.0 * (x1 - x0);
                tangents.push((w1 + w2) / (w1 / prev + w2 / next));
            }
        }
    }
    tangents
}


struct LevelPlan {
    top_fl: i32,
    top_altitude_m: f64,
    initial_altitude_m: f64,
    step_altitudes_m: Vec<f64>,
    step_distances: Vec<f64>,
    /// Applied to every climb and descent distance when even the lowest usable level
    /// will not fit. Only sectors of a couple of hundred kilometres reach for it.
    compression: f64,
}

fn fl_to_m(fl: i32) -> f64 {
    feet_to_m(fl as f64 * 100.0)
}

/// The flight level the semicircular rule allows nearest to (and not above) a target.
///
/// Easterly tracks — 000° through 179° — get odd thousands of feet, westerly tracks get
/// even. ICAO defines this on the *magnetic* track; true track is used here, which
/// differs by up to about 20° in the places magnetic variation is largest and so
/// occasionally picks the other parity. Correcting it would need a magnetic model, and
/// the visible consequence is one flight level.
pub fn semicircular_level(target_altitude_m: f64, track_rad: f64) -> i32 {
    let track_deg = track_rad.to_degrees().rem_euclid(360.0);
    let eastbound = track_deg < 180.0;
    let thousands = (m_to_feet(target_altitude_m) / 1000.0).floor() as i32;
    let correct_parity = |t: i32| (t.rem_euclid(2) == 1) == eastbound;
    let chosen = if correct_parity(thousands) {
        thousands
    } else {
        thousands - 1
    };
    (chosen * 10).clamp(10, MAX_CRUISE_FL)
}

fn plan_levels(inputs: &VerticalInputs, available: f64) -> LevelPlan {
    let optimum = aircraft::optimum_cruise_altitude_m(inputs.trip_distance_m);
    let mut fl = semicircular_level(optimum, inputs.track_rad);
    // Never plan a level the departure or arrival field is already at or above.
    let floor_alt = inputs.dep_elevation_m.max(inputs.arr_elevation_m) + feet_to_m(2_000.0);
    let floor_fl = semicircular_level(floor_alt, inputs.track_rad).max(10);

    loop {
        let top_altitude_m = fl_to_m(fl);
        let climb_to_top = integrate_climb(
            inputs.dep_elevation_m,
            top_altitude_m,
            inputs.dep_elevation_m,
        );
        let descent = integrate_descent(
            top_altitude_m,
            inputs.arr_elevation_m + aircraft::FLARE_HEIGHT_M,
            inputs.arr_elevation_m,
        );
        let spare = available - climb_to_top.distance_m - descent.distance_m;

        if spare > 0.0 {
            // How many 2,000 ft steps is the cruise long enough to be worth?
            let cruise_seconds = spare / inputs.cruise_tas_estimate.max(50.0);
            let mut steps = ((cruise_seconds / STEP_INTERVAL_S).floor() as usize).min(MAX_STEPS);
            // Steps only exist if there is room below the top level to start from.
            while steps > 0 && fl - STEP_FL * (steps as i32) < MIN_CRUISE_FL.min(fl) {
                steps -= 1;
            }

            let initial_fl = fl - STEP_FL * steps as i32;
            let initial_altitude_m = fl_to_m(initial_fl);
            let climb = integrate_climb(
                inputs.dep_elevation_m,
                initial_altitude_m,
                inputs.dep_elevation_m,
            );

            let mut step_altitudes_m = Vec::with_capacity(steps);
            let mut step_distances = Vec::with_capacity(steps);
            let mut from = initial_altitude_m;
            for k in 1..=steps {
                let to = fl_to_m(initial_fl + STEP_FL * k as i32);
                step_distances.push(integrate_climb(from, to, inputs.dep_elevation_m).distance_m);
                step_altitudes_m.push(to);
                from = to;
            }

            let needed: f64 =
                climb.distance_m + descent.distance_m + step_distances.iter().sum::<f64>();
            if needed <= available {
                return LevelPlan {
                    top_fl: fl,
                    top_altitude_m,
                    initial_altitude_m,
                    step_altitudes_m,
                    step_distances,
                    compression: 1.0,
                };
            }
        }

        if fl - STEP_FL < floor_fl {
            // Even the lowest usable level does not fit. The sector is short enough
            // that the aircraft climbs and immediately descends, so the profile is
            // compressed to whatever distance there is.
            let top_altitude_m = fl_to_m(floor_fl);
            let climb = integrate_climb(
                inputs.dep_elevation_m,
                top_altitude_m,
                inputs.dep_elevation_m,
            );
            let descent = integrate_descent(
                top_altitude_m,
                inputs.arr_elevation_m + aircraft::FLARE_HEIGHT_M,
                inputs.arr_elevation_m,
            );
            let needed = climb.distance_m + descent.distance_m;
            let compression = if needed > available {
                available / needed
            } else {
                1.0
            };
            return LevelPlan {
                top_fl: floor_fl,
                top_altitude_m,
                initial_altitude_m: top_altitude_m,
                step_altitudes_m: Vec::new(),
                step_distances: Vec::new(),
                compression,
            };
        }
        fl -= STEP_FL;
    }
}

struct Integrated {
    points: Vec<(f64, f64)>,
    distance_m: f64,
}

/// Integrates a climb between two altitudes, returning distance/altitude pairs.
///
/// The horizontal distance covered per unit of height is `TAS / ROC`, and both vary
/// with altitude — the rate of climb falls away as thrust drops and the true airspeed
/// rises. That ratio is why the last few thousand feet of a climb take so much longer
/// than the first few.
fn integrate_climb(from_alt: f64, to_alt: f64, field_elevation_m: f64) -> Integrated {
    let mut points = Vec::new();
    let mut s = 0.0;
    let mut alt = from_alt;
    points.push((0.0, alt));
    if to_alt <= from_alt {
        return Integrated {
            points,
            distance_m: 0.0,
        };
    }
    while alt < to_alt {
        let dh = INTEGRATION_STEP_M.min(to_alt - alt);
        let mid = alt + dh * 0.5;
        let roc = aircraft::rate_of_climb(mid);
        let tas = aircraft::climb_tas(mid, field_elevation_m);
        // Horizontal component of the velocity vector.
        let horizontal = (tas * tas - roc * roc).max(0.0).sqrt();
        s += horizontal * dh / roc;
        alt += dh;
        points.push((s, alt));
    }
    Integrated {
        points,
        distance_m: s,
    }
}

/// Integrates a descent from cruise down to the flare entry height.
fn integrate_descent(from_alt: f64, to_alt: f64, _field_elevation: f64) -> Integrated {
    let mut points = Vec::new();
    let mut s = 0.0;
    let mut alt = from_alt;
    points.push((0.0, alt));
    if to_alt >= from_alt {
        return Integrated {
            points,
            distance_m: 0.0,
        };
    }
    let cot = 1.0 / aircraft::DESCENT_ANGLE_RAD.tan();
    while alt > to_alt {
        let dh = INTEGRATION_STEP_M.min(alt - to_alt);
        s += dh * cot;
        alt -= dh;
        points.push((s, alt));
    }
    Integrated {
        points,
        distance_m: s,
    }
}

/// Ground distance covered during the flare.
fn flare_distance(touchdown_tas: f64) -> f64 {
    // Entering at the glideslope sink rate and leaving at the touchdown sink rate, so
    // the mean of the two sets how long the manoeuvre takes.
    let entry_sink = touchdown_tas * aircraft::DESCENT_ANGLE_RAD.tan();
    let mean_sink = 0.5 * (entry_sink + aircraft::TOUCHDOWN_SINK_MPS);
    (aircraft::FLARE_HEIGHT_M / mean_sink * touchdown_tas).clamp(150.0, 1_200.0)
}
