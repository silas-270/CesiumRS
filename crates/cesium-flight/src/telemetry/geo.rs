//! Spherical geodesy.
//!
//! Every horizontal calculation in the flight planner goes through this module, and it
//! works in unit vectors rather than lat/lon deltas. That is not a style preference:
//! subtracting longitudes is what makes a path fly the wrong way around the world when
//! it crosses the antimeridian, and there is no such thing as a longitude delta here to
//! get wrong.
//!
//! The sphere is used rather than the WGS84 ellipsoid. Over a 10,000 km route the two
//! disagree by roughly 0.3%, which is far below the accuracy of anything else in the
//! plan (wind climatology, coarse airspace outlines), and the sphere keeps great-circle
//! interpolation to a single slerp.

use glam::DVec3;

/// Mean Earth radius (IUGG). Not the equatorial radius — this is the sphere whose
/// surface area matches the ellipsoid, which is the right choice for distances.
pub const EARTH_RADIUS_M: f64 = 6_371_008.8;

/// A geographic position in degrees.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LatLon {
    pub lat_deg: f64,
    pub lon_deg: f64,
}

impl LatLon {
    pub fn new(lat_deg: f64, lon_deg: f64) -> Self {
        Self { lat_deg, lon_deg }
    }

    /// The position as a unit vector on the sphere.
    pub fn to_unit(self) -> DVec3 {
        let lat = self.lat_deg.to_radians();
        let lon = self.lon_deg.to_radians();
        let (sin_lat, cos_lat) = lat.sin_cos();
        let (sin_lon, cos_lon) = lon.sin_cos();
        DVec3::new(cos_lat * cos_lon, cos_lat * sin_lon, sin_lat)
    }

    pub fn from_unit(v: DVec3) -> Self {
        let v = v.normalize();
        Self {
            lat_deg: v.z.clamp(-1.0, 1.0).asin().to_degrees(),
            lon_deg: v.y.atan2(v.x).to_degrees(),
        }
    }
}

/// Wraps a longitude (or any angle) into (-180, 180].
pub fn wrap_deg_180(deg: f64) -> f64 {
    let mut d = (deg + 180.0) % 360.0;
    if d <= 0.0 {
        d += 360.0;
    }
    d - 180.0
}

/// Wraps an angle into (-PI, PI].
pub fn wrap_pi(rad: f64) -> f64 {
    let mut r = (rad + std::f64::consts::PI) % (2.0 * std::f64::consts::PI);
    if r <= 0.0 {
        r += 2.0 * std::f64::consts::PI;
    }
    r - std::f64::consts::PI
}

/// Angular separation in radians.
///
/// Uses the `atan2(|a×b|, a·b)` form rather than `acos(a·b)`: the dot-product form
/// loses all its precision for short separations, and the terminal-area geometry is
/// full of separations measured in metres.
pub fn angular_distance(a: DVec3, b: DVec3) -> f64 {
    a.cross(b).length().atan2(a.dot(b))
}

pub fn distance_m(a: LatLon, b: LatLon) -> f64 {
    angular_distance(a.to_unit(), b.to_unit()) * EARTH_RADIUS_M
}

/// Initial true bearing from `a` to `b`, radians clockwise from north.
pub fn initial_bearing(a: LatLon, b: LatLon) -> f64 {
    let lat1 = a.lat_deg.to_radians();
    let lat2 = b.lat_deg.to_radians();
    // Wrapped before use, so an antimeridian crossing gives the short way round.
    let dlon = wrap_deg_180(b.lon_deg - a.lon_deg).to_radians();
    let y = dlon.sin() * lat2.cos();
    let x = lat1.cos() * lat2.sin() - lat1.sin() * lat2.cos() * dlon.cos();
    y.atan2(x)
}

/// The pole of the great circle through `a` and `b` — a unit vector normal to their
/// plane. Offsetting a point on the route toward this pole moves it perpendicular to
/// the track, which is how lateral route offsets are expressed.
///
/// `None` when the two points are coincident or antipodal, where the great circle
/// through them is not unique.
pub fn great_circle_pole(a: DVec3, b: DVec3) -> Option<DVec3> {
    let n = a.cross(b);
    if n.length() < 1e-9 {
        None
    } else {
        Some(n.normalize())
    }
}

/// Great-circle interpolation. `f` of 0 gives `a`, 1 gives `b`.
///
/// Antimeridian-safe by construction: it is a rotation between two vectors and never
/// sees a longitude.
pub fn interpolate(a: DVec3, b: DVec3, f: f64) -> DVec3 {
    let omega = angular_distance(a, b);
    if omega.abs() < 1e-12 {
        return a;
    }
    let sin_omega = omega.sin();
    ((a * ((1.0 - f) * omega).sin() + b * (f * omega).sin()) / sin_omega).normalize()
}

/// Moves `p` perpendicular to a great circle by `delta` radians, toward `pole`.
pub fn offset_toward_pole(p: DVec3, pole: DVec3, delta: f64) -> DVec3 {
    (p * delta.cos() + pole * delta.sin()).normalize()
}

/// The point reached by travelling `distance_m` from `p` on `bearing_rad`.
pub fn destination(p: LatLon, bearing_rad: f64, distance_m: f64) -> LatLon {
    let ang = distance_m / EARTH_RADIUS_M;
    let lat1 = p.lat_deg.to_radians();
    let lon1 = p.lon_deg.to_radians();
    let (sin_ang, cos_ang) = ang.sin_cos();
    let (sin_lat1, cos_lat1) = lat1.sin_cos();
    let lat2 = (sin_lat1 * cos_ang + cos_lat1 * sin_ang * bearing_rad.cos()).asin();
    let lon2 =
        lon1 + (bearing_rad.sin() * sin_ang * cos_lat1).atan2(cos_ang - sin_lat1 * lat2.sin());
    LatLon::new(lat2.to_degrees(), wrap_deg_180(lon2.to_degrees()))
}

/// A local east/north tangent plane, for the terminal-area geometry.
///
/// Dubins curves, runway alignment and fly-by turns are all easier to solve on a plane,
/// and within the ~100 km they span the tangent-plane error is under a metre. Nothing
/// enroute uses this — that is exactly the mistake being corrected, where a whole
/// intercontinental route was built on one of these.
#[derive(Debug, Clone, Copy)]
pub struct LocalFrame {
    origin: LatLon,
    cos_lat: f64,
}

impl LocalFrame {
    pub fn new(origin: LatLon) -> Self {
        Self {
            origin,
            // Floored so a frame centred near a pole cannot produce a singular mapping.
            cos_lat: origin.lat_deg.to_radians().cos().max(1e-6),
        }
    }

    /// Metres east and north of the frame origin.
    pub fn to_local(&self, p: LatLon) -> (f64, f64) {
        let dlon = wrap_deg_180(p.lon_deg - self.origin.lon_deg).to_radians();
        let dlat = (p.lat_deg - self.origin.lat_deg).to_radians();
        (dlon * self.cos_lat * EARTH_RADIUS_M, dlat * EARTH_RADIUS_M)
    }

    pub fn to_geo(&self, east_m: f64, north_m: f64) -> LatLon {
        let lat = self.origin.lat_deg + (north_m / EARTH_RADIUS_M).to_degrees();
        let lon = self.origin.lon_deg + (east_m / (EARTH_RADIUS_M * self.cos_lat)).to_degrees();
        LatLon::new(lat, wrap_deg_180(lon))
    }
}

/// Accumulates longitudes along a path so they increase or decrease monotonically
/// across the antimeridian instead of jumping by 360°.
///
/// Only for algorithms that genuinely need a scalar longitude axis — the oceanic track
/// grid, which is defined on whole meridians. Path geometry never needs it.
pub fn unwrap_longitudes(points: &[LatLon]) -> Vec<f64> {
    let mut out = Vec::with_capacity(points.len());
    let mut prev = 0.0;
    for (i, p) in points.iter().enumerate() {
        if i == 0 {
            prev = p.lon_deg;
        } else {
            prev += wrap_deg_180(p.lon_deg - wrap_deg_180(prev));
        }
        out.push(prev);
    }
    out
}
