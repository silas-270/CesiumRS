//! Shared ellipsoid / Web-Mercator geodesy used by the culling harness.
//!
//! Everything in here is pure f64 unless a legacy call site forces f32, and every
//! length is in **megameters** (1 unit = 1000 km), matching the engine.
//!
//! This module is deliberately free of any engine culling logic: it is one half of
//! the measuring instrument (the "where is this point" half), and must never import
//! anything from `globe::quadtree` other than the plain `TileId` address type.

use cesium_engine::globe::quadtree::TileId;
use glam::{DVec3, Vec3};

/// WGS84 semi-major axis, in megameters.
pub const A: f64 = 6.378137;
/// WGS84 semi-minor axis, in megameters.
pub const B: f64 = 6.3567523142;

/// The latitude at which the Web-Mercator projection is truncated.
pub const MERCATOR_LIMIT_DEG: f64 = 85.0511287798066;

/// Engine ECEF convention: **Y-up with negated Z**.
///
/// `x = a·cos(lat)·cos(lon)`, `y = b·sin(lat)`, `z = −a·cos(lat)·sin(lon)`.
///
/// This mirrors [`cesium_engine::globe::geometry::lon_lat_to_ecef_f64`]; it is
/// restated here only so the harness has a `DVec3`-typed entry point.
pub fn lon_lat_to_ecef(lon_deg: f64, lat_deg: f64) -> DVec3 {
    let p = cesium_engine::globe::geometry::lon_lat_to_ecef_f64(lon_deg, lat_deg);
    DVec3::new(p[0], p[1], p[2])
}

/// As [`lon_lat_to_ecef`], with an altitude **in metres** above the ellipsoid.
pub fn lon_lat_alt_to_ecef(lon_deg: f64, lat_deg: f64, alt_m: f64) -> DVec3 {
    let p = cesium_engine::globe::geometry::lon_lat_alt_to_ecef_f64(lon_deg, lat_deg, alt_m);
    DVec3::new(p[0], p[1], p[2])
}

/// Outward unit normal of the ellipsoid at (or under/above) `p`.
///
/// `normalize((p.x/a², p.y/b², p.z/a²))` — the gradient of the implicit ellipsoid,
/// which is exact regardless of how far `p` is off the surface.
pub fn ellipsoid_normal(p: DVec3) -> DVec3 {
    DVec3::new(p.x / (A * A), p.y / (B * B), p.z / (A * A)).normalize()
}

/// Nearest ray/ellipsoid intersection, in megameters.
///
/// Kept on the historical f32 signature because `testing::camera::test_drag_zoom`,
/// `testing::camera::test_z_sweep` and `testing::terrain::test_parametric_sweeps`
/// call it that way. New harness code should use [`ray_ellipsoid_f64`].
pub fn intersect_ellipsoid(ray_origin: Vec3, ray_dir: Vec3) -> Option<DVec3> {
    ray_ellipsoid_f64(
        DVec3::new(
            ray_origin.x as f64,
            ray_origin.y as f64,
            ray_origin.z as f64,
        ),
        DVec3::new(ray_dir.x as f64, ray_dir.y as f64, ray_dir.z as f64),
    )
}

/// Ray/ellipsoid intersection in full f64.
///
/// Returns the **nearest intersection with a non-negative parameter**. When the ray
/// origin is inside the ellipsoid (a sub-surface camera) the near root is negative,
/// so the far root is returned instead — otherwise sub-surface cameras would
/// silently sample nothing and the harness would under-report.
pub fn ray_ellipsoid_f64(origin: DVec3, dir: DVec3) -> Option<DVec3> {
    let ro = DVec3::new(origin.x / A, origin.y / B, origin.z / A);
    let rd = DVec3::new(dir.x / A, dir.y / B, dir.z / A);

    let qa = rd.length_squared();
    if qa <= 0.0 {
        return None;
    }
    let qb = 2.0 * ro.dot(rd);
    let qc = ro.length_squared() - 1.0;

    let disc = qb * qb - 4.0 * qa * qc;
    if disc < 0.0 {
        return None;
    }
    let sqrt_disc = disc.sqrt();
    let t0 = (-qb - sqrt_disc) / (2.0 * qa);
    let t1 = (-qb + sqrt_disc) / (2.0 * qa);

    let t = if t0 >= 0.0 {
        t0
    } else if t1 >= 0.0 {
        t1
    } else {
        return None;
    };

    let hit = ro + rd * t;
    Some(DVec3::new(hit.x * A, hit.y * B, hit.z * A))
}

/// ECEF (megameters) -> (latitude, longitude) in degrees, engine convention.
pub fn dvec3_to_lat_lon(pos: DVec3) -> (f64, f64) {
    let scaled = DVec3::new(pos.x / A, pos.y / B, pos.z / A).normalize();
    let lat = scaled.y.clamp(-1.0, 1.0).asin().to_degrees();
    let lon = (-scaled.z).atan2(scaled.x).to_degrees();
    (lat, lon)
}

/// Web-Mercator tile row for a latitude, saturating at the poles.
///
/// `tan(phi)` is infinite exactly at ±90°, and `asinh` overflows just short of it,
/// so both ends are clamped to the first/last row rather than producing NaN. This
/// is the behaviour the engine's own pole capping implies (`y == 0` and
/// `y == (1<<z)-1` are stretched to ±90°).
pub fn lat_to_web_mercator_y(lat: f64, z: u8) -> u32 {
    let n = (1_u32 << z) as f64;
    let max_y = (1_u32 << z).saturating_sub(1);

    let phi = lat.to_radians();
    let tan_phi = phi.tan();
    if !tan_phi.is_finite() {
        return if lat > 0.0 { 0 } else { max_y };
    }

    let y = (n / 2.0) * (1.0 - tan_phi.asinh() / std::f64::consts::PI);
    if y.is_nan() || y < 0.0 {
        0
    } else if y > max_y as f64 {
        max_y
    } else {
        y.floor() as u32
    }
}

/// Web-Mercator tile column for a longitude.
pub fn lon_to_web_mercator_x(lon: f64, z: u8) -> u32 {
    let n = (1_u32 << z) as f64;
    let max_x = (1_u32 << z).saturating_sub(1);
    let x = ((lon + 180.0) / 360.0) * n;
    if x.is_nan() || x < 0.0 {
        0
    } else if x > max_x as f64 {
        max_x
    } else {
        x.floor() as u32
    }
}

/// The tile at zoom `z` that owns (`lat`, `lon`).
pub fn tile_for_lat_lon(lat: f64, lon: f64, z: u8) -> TileId {
    TileId {
        z,
        x: lon_to_web_mercator_x(lon, z),
        y: lat_to_web_mercator_y(lat, z),
    }
}

/// True when `tile` is the zoom-`tile.z` tile that owns (`lat`, `lon`).
///
/// Because the owning tile is recomputed at the candidate's own zoom level, this
/// answers "does this tile cover the point" for **any** zoom — which implicitly
/// handles the ancestor case (a coarse tile still in the visible set covers points
/// that a finer tile would otherwise have claimed).
pub fn tile_contains(tile: &TileId, lat: f64, lon: f64) -> bool {
    tile_for_lat_lon(lat, lon, tile.z) == *tile
}

/// Geographic extent actually rendered for a tile: (lon_min, lon_max, lat_min, lat_max).
///
/// Mirrors the engine's pole capping in `QuadtreeNode::compute_bounding_volume`:
/// the top row is stretched to +90° and the bottom row to −90°, so the harness
/// samples the same surface the renderer claims to cover.
pub fn tile_bounds(tile: &TileId) -> (f64, f64, f64, f64) {
    let n = (1_u32 << tile.z) as f64;
    let lon_min = -180.0 + (tile.x as f64) * 360.0 / n;
    let lon_max = -180.0 + ((tile.x + 1) as f64) * 360.0 / n;

    let mut lat_max = web_mercator_y_to_lat_f64(tile.y as f64, tile.z);
    let mut lat_min = web_mercator_y_to_lat_f64((tile.y + 1) as f64, tile.z);
    if tile.y == 0 {
        lat_max = 90.0;
    }
    if tile.y == (1_u32 << tile.z) - 1 {
        lat_min = -90.0;
    }
    (lon_min, lon_max, lat_min, lat_max)
}

/// f64 twin of `cesium_engine::globe::quadtree::web_mercator_y_to_lat` (which is f32).
pub fn web_mercator_y_to_lat_f64(y: f64, z: u8) -> f64 {
    let n = (1_u32 << z) as f64;
    (std::f64::consts::PI * (1.0 - 2.0 * y / n))
        .sinh()
        .atan()
        .to_degrees()
}

/// `steps × steps` sample points spread over a tile's rendered extent, as ECEF.
///
/// Used for the false-positive measurement: a tile is a false positive only if
/// *none* of its own interior samples is visible to the oracle. Sampling inside
/// the tile (rather than relying on a global grid) makes the metric independent
/// of grid density, which matters enormously at zoom 18-20 where a global grid
/// would never place a point inside a tile at all.
pub fn tile_sample_points(tile: &TileId, steps: u32) -> Vec<DVec3> {
    let (lon_min, lon_max, lat_min, lat_max) = tile_bounds(tile);
    let mut out = Vec::with_capacity(((steps + 1) * (steps + 1)) as usize);
    for i in 0..=steps {
        let u = i as f64 / steps as f64;
        let lon = lon_min + u * (lon_max - lon_min);
        for j in 0..=steps {
            let v = j as f64 / steps as f64;
            let lat = lat_min + v * (lat_max - lat_min);
            out.push(lon_lat_to_ecef(lon, lat));
        }
    }
    out
}
