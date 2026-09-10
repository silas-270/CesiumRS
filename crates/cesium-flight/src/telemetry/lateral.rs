//! The enroute route: where the aircraft goes between the two terminal areas.
//!
//! The baseline is the great circle, because that is the shortest path and everything
//! else is a reason to leave it. Two reasons are modelled, and they are the two that
//! dominate real flight plans:
//!
//! - **Wind.** Formally this is Zermelo's navigation problem — the minimum-time path
//!   through a moving medium. It has no closed form over a realistic wind field, and
//!   operationally nobody tries: dispatch systems lay a lateral grid over the great
//!   circle and search it. That is exactly what happens below.
//! - **Closed airspace.** A hard constraint rather than a cost, applied by marking grid
//!   nodes and legs unusable.
//!
//! The search returns a waypoint list, not a smooth curve, and that is deliberate. Real
//! enroute tracks *are* polylines — a flight plan is a sequence of fixes joined by
//! straight legs, and the small heading changes at each one are visible on any tracker.
//! Letting the grid resolution show through produces that character honestly, instead
//! of faking kinks in a curve that would otherwise be perfectly smooth.

use super::airspace::AirspaceRestrictions;
use super::geo::{
    angular_distance, great_circle_pole, initial_bearing, interpolate, offset_toward_pole,
    unwrap_longitudes, LatLon, EARTH_RADIUS_M,
};
use super::wind::{solve_wind_triangle, WindField};

/// Below this, routes go essentially direct. Short sectors have no room to trade
/// distance for wind, and real short-haul flight plans reflect that.
const MIN_OPTIMISE_DISTANCE_M: f64 = 650_000.0;

/// Target spacing between grid stages. Roughly the spacing between enroute fixes.
const STAGE_SPACING_M: f64 = 250_000.0;
const MIN_STAGES: usize = 6;
const MAX_STAGES: usize = 48;

/// How far off the great circle the search may look.
///
/// Two budgets, because the two reasons to leave the great circle are of completely
/// different sizes. Wind is worth a few hundred kilometres — beyond that the extra
/// distance costs more time than the tailwind saves, so a larger budget would only slow
/// the search down. A closed border is worth thousands: avoiding Russian airspace moves
/// London-Tokyo about 2,900 km off its polar track and adds a fifth to the distance,
/// which is exactly what happened to that route in reality.
const WIND_OFFSET_FRACTION: f64 = 0.30;
const MIN_WIND_OFFSET_M: f64 = 300_000.0;
const MAX_WIND_OFFSET_M: f64 = 800_000.0;
const AVOIDANCE_OFFSET_FRACTION: f64 = 0.60;
const MIN_AVOIDANCE_OFFSET_M: f64 = 1_000_000.0;
const MAX_AVOIDANCE_OFFSET_M: f64 = 5_000_000.0;

/// Spacing the lateral grid aims for, and the bounds on how many nodes that implies.
/// Keeping the spacing fixed rather than the node count means the search resolves a
/// long detour as finely as a short one.
const OFFSET_SPACING_M: f64 = 150_000.0;
const MIN_OFFSET_NODES: usize = 21;
const MAX_OFFSET_NODES: usize = 81;

/// Samples used to decide whether the direct route is obstructed at all.
const DIRECT_PROBE_SAMPLES: usize = 240;

/// How many offset steps the route may move between adjacent stages.
const MAX_OFFSET_STEP: i32 = 2;

/// A small cost for changing offset to discourage chattering.
const TURN_PENALTY_FRACTION: f64 = 0.001;

/// Interior samples per leg for the airspace test. Node-only testing lets a leg clip a
/// corner of a region that neither of its endpoints is inside.
const LEG_AIRSPACE_SAMPLES: usize = 3;

pub struct EnrouteOptions<'a> {
    pub wind: &'a WindField,
    pub airspace: &'a AirspaceRestrictions,
    pub cruise_altitude_m: f64,
    pub cruise_tas: f64,
    /// Whether to snap the ocean crossing onto the organised-track grid.
    pub oceanic_tracks: bool,
}

/// Plans the enroute portion, returning waypoints from `from` to `to` inclusive.
pub fn plan_enroute(from: LatLon, to: LatLon, opts: &EnrouteOptions) -> Vec<LatLon> {
    let v_from = from.to_unit();
    let v_to = to.to_unit();

    let pole = match great_circle_pole(v_from, v_to) {
        Some(p) => p,
        // Coincident or antipodal: no unique great circle, so there is nothing to
        // optimise over. A direct leg is the only defensible answer.
        None => return vec![from, to],
    };

    let route_m = angular_distance(v_from, v_to) * EARTH_RADIUS_M;

    // Only consider avoidance when the direct route between from and to is actually blocked.
    let direct_blocked = !opts.airspace.is_empty()
        && (0..=DIRECT_PROBE_SAMPLES).any(|k| {
            let f = k as f64 / DIRECT_PROBE_SAMPLES as f64;
            opts.airspace
                .blocks(LatLon::from_unit(interpolate(v_from, v_to, f)))
        });

    if route_m < MIN_OPTIMISE_DISTANCE_M && !direct_blocked {
        return direct_waypoints(v_from, v_to, route_m);
    }

    let stages = ((route_m / STAGE_SPACING_M).round() as usize).clamp(MIN_STAGES, MAX_STAGES);

    let max_offset_m = if direct_blocked {
        (AVOIDANCE_OFFSET_FRACTION * route_m).clamp(MIN_AVOIDANCE_OFFSET_M, MAX_AVOIDANCE_OFFSET_M)
    } else {
        (WIND_OFFSET_FRACTION * route_m).clamp(MIN_WIND_OFFSET_M, MAX_WIND_OFFSET_M)
    };

    let half_nodes = ((max_offset_m / OFFSET_SPACING_M).round() as usize)
        .clamp(MIN_OFFSET_NODES / 2, MAX_OFFSET_NODES / 2);
    let offset_nodes = half_nodes * 2 + 1;
    let centre = half_nodes as i32;
    let offset_step_m = max_offset_m / centre as f64;

    // Node positions. Stage 0 and the last stage are pinned to the route ends.
    let node = |stage: usize, offset: i32| -> LatLon {
        let f = stage as f64 / stages as f64;
        let on_route = interpolate(v_from, v_to, f);
        if stage == 0 {
            return from;
        }
        if stage == stages {
            return to;
        }
        let delta = (offset - centre) as f64 * offset_step_m / EARTH_RADIUS_M;
        LatLon::from_unit(offset_toward_pole(on_route, pole, delta))
    };

    // Pre-flag blocked nodes so the transition loop does not retest them.
    let mut blocked = vec![false; (stages + 1) * offset_nodes];
    for stage in 0..=stages {
        for offset in 0..offset_nodes as i32 {
            blocked[stage * offset_nodes + offset as usize] =
                opts.airspace.blocks(node(stage, offset));
        }
    }

    const INF: f64 = f64::INFINITY;
    let mut cost = vec![INF; (stages + 1) * offset_nodes];
    let mut prev = vec![usize::MAX; (stages + 1) * offset_nodes];
    cost[centre as usize] = 0.0;

    for stage in 0..stages {
        for i in 0..offset_nodes as i32 {
            let from_idx = stage * offset_nodes + i as usize;
            let base = cost[from_idx];
            if !base.is_finite() {
                continue;
            }
            // The last stage collapses to the pinned arrival node.
            let (lo, hi) = if stage + 1 == stages {
                (centre, centre)
            } else {
                (
                    (i - MAX_OFFSET_STEP).max(0),
                    (i + MAX_OFFSET_STEP).min(offset_nodes as i32 - 1),
                )
            };
            // Stage 1 can only be reached from the pinned departure node.
            if stage == 0 && i != centre {
                continue;
            }

            let p1 = node(stage, i);
            for j in lo..=hi {
                let to_idx = (stage + 1) * offset_nodes + j as usize;
                if blocked[from_idx] || blocked[to_idx] {
                    continue;
                }
                let p2 = node(stage + 1, j);
                let leg = match leg_cost(p1, p2, opts) {
                    Some(c) => c,
                    None => continue,
                };
                let step_diff = (j - i).abs() as f64;
                let penalty = 1.0 + TURN_PENALTY_FRACTION * step_diff;
                let total = base + leg * penalty;
                if total < cost[to_idx] {
                    cost[to_idx] = total;
                    prev[to_idx] = from_idx;
                }
            }
        }
    }

    let end_idx = stages * offset_nodes + centre as usize;
    if !cost[end_idx].is_finite() {
        // Nothing reached the destination — every path was blocked. Falling back to the
        // direct route is wrong in principle but visible and finite, which beats an
        // empty flight.
        log::warn!(
            "no unobstructed route found from {:?} to {:?}; flying direct",
            from,
            to
        );
        return direct_waypoints(v_from, v_to, route_m);
    }

    let mut offsets = Vec::with_capacity(stages + 1);
    let mut idx = end_idx;
    while idx != usize::MAX {
        let stage = idx / offset_nodes;
        let offset = (idx % offset_nodes) as i32;
        offsets.push((stage, offset));
        idx = prev[idx];
    }
    offsets.reverse();

    // If every stage stayed on the centreline and not using oceanic tracks, fly purely direct.
    if !opts.oceanic_tracks && offsets.iter().all(|&(_, o)| o == centre) {
        return direct_waypoints(v_from, v_to, route_m);
    }

    let mut dists: Vec<f64> = offsets
        .iter()
        .map(|&(_, o)| (o - centre) as f64 * offset_step_m)
        .collect();

    // Multi-pass binomial filter to smooth the offset profile and eliminate grid chatter
    for _ in 0..4 {
        let mut smoothed = dists.clone();
        for s in 1..stages {
            let candidate = 0.25 * dists[s - 1] + 0.5 * dists[s] + 0.25 * dists[s + 1];
            let f = s as f64 / stages as f64;
            let on_route = interpolate(v_from, v_to, f);
            let test_pos = LatLon::from_unit(offset_toward_pole(
                on_route,
                pole,
                candidate / EARTH_RADIUS_M,
            ));
            if !opts.airspace.blocks(test_pos) {
                smoothed[s] = candidate;
            }
        }
        dists = smoothed;
    }

    let mut chain = Vec::with_capacity(stages + 1);
    for (s, &delta_m) in dists.iter().enumerate() {
        if s == 0 {
            chain.push(from);
        } else if s == stages {
            chain.push(to);
        } else {
            let f = s as f64 / stages as f64;
            let on_route = interpolate(v_from, v_to, f);
            chain.push(LatLon::from_unit(offset_toward_pole(
                on_route,
                pole,
                delta_m / EARTH_RADIUS_M,
            )));
        }
    }

    if opts.oceanic_tracks {
        chain = apply_oceanic_grid(&chain);
    } else {
        // Simplify collinear waypoints to prevent segmentation along gentle curves
        chain = simplify_waypoints(&chain, 6_000.0, opts.airspace);
    }
    chain
}

/// Simplifies a sequence of waypoints using recursive cross-track error thresholding,
/// ensuring that direct segments do not cross into restricted airspace.
fn simplify_waypoints(
    pts: &[LatLon],
    tolerance_m: f64,
    airspace: &AirspaceRestrictions,
) -> Vec<LatLon> {
    if pts.len() <= 2 {
        return pts.to_vec();
    }
    let v_first = pts[0].to_unit();
    let v_last = pts[pts.len() - 1].to_unit();
    let pole = match great_circle_pole(v_first, v_last) {
        Some(p) => p,
        None => return pts.to_vec(),
    };

    let mut max_dist = 0.0_f64;
    let mut max_idx = 0;
    for i in 1..pts.len() - 1 {
        let v_pt = pts[i].to_unit();
        let dist = (v_pt.dot(pole).abs()).clamp(0.0, 1.0).asin() * EARTH_RADIUS_M;
        if dist > max_dist {
            max_dist = dist;
            max_idx = i;
        }
    }

    let direct_blocked = !airspace.is_empty()
        && (1..=12).any(|k| {
            let f = k as f64 / 13.0;
            airspace.blocks(LatLon::from_unit(interpolate(v_first, v_last, f)))
        });

    if max_dist > tolerance_m || direct_blocked {
        let mut left = simplify_waypoints(&pts[..=max_idx], tolerance_m, airspace);
        let right = simplify_waypoints(&pts[max_idx..], tolerance_m, airspace);
        left.pop();
        left.extend(right);
        left
    } else {
        vec![pts[0], *pts.last().unwrap()]
    }
}

/// Cost of a leg, in seconds of flight time.
fn leg_cost(p1: LatLon, p2: LatLon, opts: &EnrouteOptions) -> Option<f64> {
    let v1 = p1.to_unit();
    let v2 = p2.to_unit();
    let dist = angular_distance(v1, v2) * EARTH_RADIUS_M;
    if dist < 1.0 {
        return Some(0.0);
    }

    for k in 1..=LEG_AIRSPACE_SAMPLES {
        let f = k as f64 / (LEG_AIRSPACE_SAMPLES + 1) as f64;
        if opts
            .airspace
            .blocks(LatLon::from_unit(interpolate(v1, v2, f)))
        {
            return None;
        }
    }

    let mid = LatLon::from_unit(interpolate(v1, v2, 0.5));
    let track = initial_bearing(mid, p2);
    let (u, v) = opts
        .wind
        .sample(mid.lat_deg, mid.lon_deg, opts.cruise_altitude_m);
    let triangle = solve_wind_triangle(track, opts.cruise_tas, u, v)?;
    Some(dist / triangle.ground_speed)
}

/// A direct great-circle route defined by its endpoints.
fn direct_waypoints(v_from: glam::DVec3, v_to: glam::DVec3, _route_m: f64) -> Vec<LatLon> {
    vec![LatLon::from_unit(v_from), LatLon::from_unit(v_to)]
}

/// Latitude/longitude bounds of an organised track system.
struct OceanicRegion {
    min_lat: f64,
    max_lat: f64,
    /// Longitude bounds, walking eastward from `west_lon` to `east_lon`. The Pacific
    /// region crosses the antimeridian, so `east_lon` may be numerically smaller.
    west_lon: f64,
    east_lon: f64,
}

impl OceanicRegion {
    fn contains(&self, lat: f64, lon: f64) -> bool {
        if lat < self.min_lat || lat > self.max_lat {
            return false;
        }
        if self.west_lon <= self.east_lon {
            lon >= self.west_lon && lon <= self.east_lon
        } else {
            lon >= self.west_lon || lon <= self.east_lon
        }
    }
}

/// The North Atlantic and North Pacific organised track systems.
///
/// Both exist because their crossings are long, busy, outside radar cover, and sit
/// directly under a jet streak. The tracks are rebuilt twice a day from that day's
/// winds and published as latitudes at whole meridians — which is why an oceanic
/// crossing looks stepped and angular where the rest of a route looks smooth.
const OCEANIC_REGIONS: &[OceanicRegion] = &[
    // North Atlantic (Gander/Shanwick).
    OceanicRegion {
        min_lat: 38.0,
        max_lat: 70.0,
        west_lon: -65.0,
        east_lon: -8.0,
    },
    // North Pacific (Anchorage/Oakland/Fukuoka).
    OceanicRegion {
        min_lat: 25.0,
        max_lat: 62.0,
        west_lon: 140.0,
        east_lon: -135.0,
    },
];

/// Meridian spacing of the published track points.
const OCEANIC_MERIDIAN_SPACING_DEG: f64 = 10.0;

/// Densification used to find where the route crosses each meridian.
const OCEANIC_SAMPLES_PER_LEG: usize = 24;

/// Rewrites the oceanic portion of a route onto the track grid.
///
/// Track points are published at whole degrees of latitude on every tenth meridian, so
/// that is what this produces: find where the optimised route crosses each meridian,
/// round the latitude, and use those as the waypoints for the crossing.
fn apply_oceanic_grid(waypoints: &[LatLon]) -> Vec<LatLon> {
    if waypoints.len() < 2 {
        return waypoints.to_vec();
    }

    // Densify, keeping a fractional index back into the waypoint list so gridded points
    // can be merged into the right place.
    let mut dense: Vec<(f64, LatLon)> = Vec::new();
    for i in 0..waypoints.len() - 1 {
        let v1 = waypoints[i].to_unit();
        let v2 = waypoints[i + 1].to_unit();
        for k in 0..OCEANIC_SAMPLES_PER_LEG {
            let f = k as f64 / OCEANIC_SAMPLES_PER_LEG as f64;
            dense.push((i as f64 + f, LatLon::from_unit(interpolate(v1, v2, f))));
        }
    }
    dense.push(((waypoints.len() - 1) as f64, waypoints[waypoints.len() - 1]));

    // A continuous longitude axis, so a crossing of the 180th meridian is just another
    // multiple of ten rather than a discontinuity.
    let lons = unwrap_longitudes(&dense.iter().map(|(_, p)| *p).collect::<Vec<_>>());

    let mut gridded: Vec<(f64, LatLon)> = Vec::new();
    for i in 0..dense.len() - 1 {
        let (t0, p0) = dense[i];
        let (t1, p1) = dense[i + 1];
        let (l0, l1) = (lons[i], lons[i + 1]);
        if (l1 - l0).abs() < 1e-9 {
            continue;
        }
        let step = OCEANIC_MERIDIAN_SPACING_DEG;
        let (lo, hi) = if l0 < l1 { (l0, l1) } else { (l1, l0) };
        let first = (lo / step).ceil() as i64;
        let last = (hi / step).floor() as i64;
        for m in first..=last {
            let meridian = m as f64 * step;
            let f = (meridian - l0) / (l1 - l0);
            if !(0.0..1.0).contains(&f) {
                continue;
            }
            let lat = p0.lat_deg + (p1.lat_deg - p0.lat_deg) * f;
            let wrapped = super::geo::wrap_deg_180(meridian);
            if !OCEANIC_REGIONS.iter().any(|r| r.contains(lat, wrapped)) {
                continue;
            }
            gridded.push((
                t0 + (t1 - t0) * f,
                // Whole degrees of latitude, exactly as a track is published.
                LatLon::new(lat.round(), wrapped),
            ));
        }
    }

    // Two crossings is the least that can define a track segment; anything less means
    // the route only clips the region and is better left alone.
    if gridded.len() < 2 {
        return waypoints.to_vec();
    }

    let first_t = gridded[0].0;
    let last_t = gridded[gridded.len() - 1].0;

    let mut out: Vec<(f64, LatLon)> = waypoints
        .iter()
        .enumerate()
        .map(|(i, p)| (i as f64, *p))
        .filter(|(t, _)| *t < first_t || *t > last_t)
        .collect();
    out.extend(gridded);
    out.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
    out.into_iter().map(|(_, p)| p).collect()
}
