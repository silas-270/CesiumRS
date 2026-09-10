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
        let available = (inputs.s_touchdown - s_rotate).max(1_000.0);

        let plan = plan_levels(inputs, available);

        let mut samples: Vec<(f64, f64)> = Vec::new();
        samples.push((0.0, inputs.dep_elevation_m));
        samples.push((s_rotate, inputs.dep_elevation_m));

        // Climb to the initial cruise level.
        let climb = integrate_climb(
            inputs.dep_elevation_m,
            plan.initial_altitude_m,
            inputs.dep_elevation_m,
        );

        for (ds, alt) in &climb.points {
            samples.push((s_rotate + ds * plan.compression, *alt));
        }
        let s_top_of_climb = s_rotate + climb.distance_m * plan.compression;

        // Descent, measured back from the flare so the geometry lands on the runway.
        let flare_entry_alt = inputs.arr_elevation_m + aircraft::FLARE_HEIGHT_M;
        let descent =
            integrate_descent(plan.top_altitude_m, flare_entry_alt, inputs.arr_elevation_m);
        let flare_m = flare_distance(v_touchdown);
        let s_flare_start = inputs.s_touchdown - flare_m;
        let s_top_of_descent = s_flare_start - descent.distance_m * plan.compression;

        // Cruise, with the step climbs spread through it.
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

        for (ds, a) in &descent.points {
            samples.push((s_top_of_descent + ds * plan.compression, *a));
        }
        samples.push((s_flare_start, flare_entry_alt));

        // Flare: smooth quintic transition from glideslope to touchdown
        const FLARE_SAMPLES: usize = 16;
        for k in 1..FLARE_SAMPLES {
            let f = k as f64 / FLARE_SAMPLES as f64;
            let u = 1.0 - f;
            let ease = u * u * u * (10.0 - 15.0 * u + 6.0 * u * u);
            let height = aircraft::FLARE_HEIGHT_M * ease;
            samples.push((s_flare_start + flare_m * f, inputs.arr_elevation_m + height));
        }
        samples.push((inputs.s_touchdown, inputs.arr_elevation_m));
        samples.push((inputs.s_total, inputs.arr_elevation_m));

        // Ensure strictly sorted and deduplicated distances
        samples.retain(|(s, a)| s.is_finite() && a.is_finite());
        samples.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
        samples.dedup_by(|a, b| (a.0 - b.0).abs() < 1e-6);

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
