use glam::DVec3;
use crate::globe::geometry::lon_lat_to_ecef_f64;
use crate::globe::quadtree::TileBounds;

/// A linear runway corridor that flattens terrain along its centerline in terrain mode.
///
/// Within [`Self::half_width_m`] the terrain is planar along the straight line from
/// `start_elev_m` to `end_elev_m`. Between `half_width_m` and `half_width_m + blend_margin_m`
/// it smoothly blends with a cubic Hermite curve into the natural DEM, preventing cliffs.
#[derive(Clone, Debug, PartialEq)]
pub struct RunwayCorridor {
    pub start_lon: f64,
    pub start_lat: f64,
    pub end_lon: f64,
    pub end_lat: f64,
    pub start_elev_m: Option<f64>,
    pub end_elev_m: Option<f64>,
    pub half_width_m: f64,
    pub blend_margin_m: f64,
    // Precomputed for fast evaluation
    pub bbox: [f64; 4], // [min_lon, max_lon, min_lat, max_lat]
    pub start_pos: DVec3,
    pub v: DVec3,
    pub inv_v_len_sq: f64,
}

impl RunwayCorridor {
    pub fn new(
        start_lon: f64,
        start_lat: f64,
        end_lon: f64,
        end_lat: f64,
        start_elev_m: Option<f64>,
        end_elev_m: Option<f64>,
        half_width_m: f64,
        blend_margin_m: f64,
    ) -> Self {
        let min_lon = start_lon.min(end_lon);
        let max_lon = start_lon.max(end_lon);
        let min_lat = start_lat.min(end_lat);
        let max_lat = start_lat.max(end_lat);

        let total_radius = half_width_m + blend_margin_m;
        // Conservative angular margin: ~70 km per degree of longitude at mid-latitudes
        let deg_margin = (total_radius + 100.0) / 70_000.0;
        let bbox = [
            min_lon - deg_margin,
            max_lon + deg_margin,
            min_lat - deg_margin,
            max_lat + deg_margin,
        ];

        let start_pos = DVec3::from_array(lon_lat_to_ecef_f64(start_lon, start_lat));
        let end_pos = DVec3::from_array(lon_lat_to_ecef_f64(end_lon, end_lat));
        let v = end_pos - start_pos;
        let v_len_sq = v.length_squared();
        let inv_v_len_sq = if v_len_sq > 0.0 { 1.0 / v_len_sq } else { 0.0 };

        Self {
            start_lon,
            start_lat,
            end_lon,
            end_lat,
            start_elev_m,
            end_elev_m,
            half_width_m,
            blend_margin_m,
            bbox,
            start_pos,
            v,
            inv_v_len_sq,
        }
    }

    /// Whether this corridor potentially overlaps a tile bounding box.
    pub fn overlaps_bounds(&self, b: &TileBounds) -> bool {
        !(self.bbox[1] < b.lon_min
            || self.bbox[0] > b.lon_max
            || self.bbox[3] < b.lat_min
            || self.bbox[2] > b.lat_max)
    }

    /// Sets or updates the start and end elevations.
    pub fn set_elevations(&mut self, start_m: f64, end_m: f64) {
        self.start_elev_m = Some(start_m);
        self.end_elev_m = Some(end_m);
    }

    /// Filters a raw DEM elevation (in metres) at `(lon, lat)` by flattening onto the corridor.
    pub fn filter_height(&self, lon: f64, lat: f64, raw_h_m: f64) -> f64 {
        // Fast AABB check
        if lon < self.bbox[0] || lon > self.bbox[1] || lat < self.bbox[2] || lat > self.bbox[3] {
            return raw_h_m;
        }

        let (h0, h1) = match (self.start_elev_m, self.end_elev_m) {
            (Some(a), Some(b)) => (a, b),
            (Some(a), None) => (a, a),
            (None, Some(b)) => (b, b),
            (None, None) => return raw_h_m,
        };

        if self.inv_v_len_sq <= 0.0 {
            return raw_h_m;
        }

        let p = DVec3::from_array(lon_lat_to_ecef_f64(lon, lat));
        let to_p = p - self.start_pos;
        let t = to_p.dot(self.v) * self.inv_v_len_sq;
        let t_clamped = t.clamp(0.0, 1.0);
        let proj = self.start_pos + self.v * t_clamped;

        let dist_m = (p - proj).length() * 1.0e6;
        let total_radius = self.half_width_m + self.blend_margin_m;
        if dist_m >= total_radius {
            return raw_h_m;
        }

        let target_h_m = h0 + (h1 - h0) * t_clamped;

        if dist_m <= self.half_width_m {
            target_h_m
        } else {
            let u = (dist_m - self.half_width_m) / self.blend_margin_m;
            let w = u * u * (3.0 - 2.0 * u);
            target_h_m + (raw_h_m - target_h_m) * w
        }
    }
}

