//! Airspace that civil traffic routes around.
//!
//! This is the second reason real tracks diverge from the great circle, after wind, and
//! on some city pairs it is by far the larger of the two. London to Tokyo has a
//! great-circle track that crosses the Arctic at 71°N; since Russian airspace closed to
//! Western operators the flight goes south of the Caspian instead, adding hours. No
//! amount of wind modelling produces that — it is a hard constraint, not a cost.
//!
//! The outlines are deliberately coarse. They exist to answer "may this route pass
//! through here", and a polygon accurate to fifty kilometres answers that as well as
//! one accurate to five. They are national outlines rather than FIR boundaries for the
//! same reason.
//!
//! Because these encode a political situation rather than a physical one, they date.
//! The set below reflects what most Western operators avoided as of September 2026, and
//! `FlightPlanConfig::avoid_closed_airspace` turns the whole mechanism off.

use super::geo::{wrap_deg_180, LatLon};

/// A closed region, as a ring of vertices in degrees.
pub struct ClosedRegion {
    pub name: &'static str,
    vertices: &'static [(f64, f64)],
    // Bounding box, so the common case of "nowhere near this" costs four comparisons.
    min_lat: f64,
    max_lat: f64,
    min_lon: f64,
    max_lon: f64,
}

impl ClosedRegion {
    fn new(name: &'static str, vertices: &'static [(f64, f64)]) -> Self {
        let mut min_lat = f64::MAX;
        let mut max_lat = f64::MIN;
        let mut min_lon = f64::MAX;
        let mut max_lon = f64::MIN;
        for &(lat, lon) in vertices {
            min_lat = min_lat.min(lat);
            max_lat = max_lat.max(lat);
            min_lon = min_lon.min(lon);
            max_lon = max_lon.max(lon);
        }
        Self {
            name,
            vertices,
            min_lat,
            max_lat,
            min_lon,
            max_lon,
        }
    }

    /// Whether a position lies inside the region.
    ///
    /// Ray casting in lat/lon, rejected first against the bounding box so the usual
    /// answer costs four comparisons. Every region here spans well under 180° of
    /// longitude and none straddles the antimeridian, so a planar test is sound; the
    /// assertion in the test suite is what keeps that true if a region is ever added.
    pub fn contains(&self, p: LatLon) -> bool {
        if p.lat_deg < self.min_lat || p.lat_deg > self.max_lat {
            return false;
        }
        let lon = wrap_deg_180(p.lon_deg);
        if lon < self.min_lon || lon > self.max_lon {
            return false;
        }

        let mut inside = false;
        let n = self.vertices.len();
        let mut j = n - 1;
        for i in 0..n {
            let (lat_i, lon_i) = self.vertices[i];
            let (lat_j, lon_j) = self.vertices[j];
            if (lon_i > lon) != (lon_j > lon) {
                let t = (lon - lon_i) / (lon_j - lon_i);
                if p.lat_deg < lat_i + t * (lat_j - lat_i) {
                    inside = !inside;
                }
            }
            j = i;
        }
        inside
    }

    /// The widest longitude span of the region, used by the test that guards the
    /// planar containment test above.
    pub fn longitude_span(&self) -> f64 {
        self.max_lon - self.min_lon
    }
}

/// Russia. The single most consequential outline here: it sits under the great-circle
/// track of essentially every Europe-to-East-Asia route.
///
/// The southern border is the part that has to be right, because that is where the
/// re-routes go. Kazakhstan, Mongolia and northern China are all open, and a polygon
/// that swallowed them would push routes somewhere real traffic does not go.
///
/// The Pacific edge matters for the same reason and in the same way. It follows the
/// seaward boundary rather than the mainland coast, so Sakhalin, the Kuril chain,
/// Kamchatka and the whole Sea of Okhotsk are inside it — they are Russian airspace, and
/// a polygon drawn along the mainland leaves a corridor through them that a polar
/// re-route to Japan will happily cut the corner through. The Sea of Japan is left
/// outside via the notch at the Tatar Strait, and the southern end stays north of
/// Hokkaido: Sōya and Nemuro are within fifty miles of Russian territory there, and
/// swallowing either would close the approach to Japan from the north.
const RUSSIA: &[(f64, f64)] = &[
    (45.2, 37.5),
    (43.4, 47.0),
    (46.0, 48.6),
    (51.2, 55.0),
    (54.0, 62.0),
    (54.2, 72.0),
    (51.0, 80.0),
    (49.2, 87.3),
    (50.0, 90.0),
    (50.3, 100.0),
    (50.2, 115.0),
    (49.5, 120.0),
    (44.0, 131.2),
    (43.0, 132.0),
    (47.3, 138.7),
    (49.0, 140.4),
    (46.5, 141.7),
    (45.8, 142.3),
    (44.5, 146.6),
    (47.0, 152.5),
    (50.7, 156.5),
    (51.2, 158.5),
    (56.0, 163.5),
    (60.0, 166.5),
    (62.0, 170.0),
    (66.0, 179.9),
    (70.5, 179.9),
    (73.5, 140.0),
    (76.5, 110.0),
    (73.0, 80.0),
    (70.0, 60.0),
    (68.5, 40.0),
    (66.0, 30.0),
    (60.0, 28.2),
    (56.0, 28.0),
    (52.0, 32.0),
    (48.0, 39.0),
];

/// The Arctic sector north of the Russian coast.
///
/// Kept separate from the landmass because it is a different kind of thing: a flight
/// information region rather than a country, extending from the coast to the pole. It
/// has to be here, though — without it a route from Europe to East Asia simply hops
/// over the top of the landmass polygon at 88°N and carries on, which is both far
/// shorter than the real re-route and not something any airline is permitted to do.
///
/// Stops just short of the pole itself: at 90° every meridian meets, and a polygon
/// through that point has no well-defined interior in latitude/longitude.
const RUSSIAN_ARCTIC_SECTOR: &[(f64, f64)] =
    &[(68.0, 30.0), (89.9, 30.0), (89.9, 179.9), (68.0, 179.9)];

const UKRAINE: &[(f64, f64)] = &[
    (52.4, 23.6),
    (52.3, 31.8),
    (51.2, 34.4),
    (49.6, 40.2),
    (47.3, 38.3),
    (45.3, 35.0),
    (46.0, 30.5),
    (45.4, 28.2),
    (47.9, 24.9),
    (49.1, 22.1),
];

const BELARUS: &[(f64, f64)] = &[
    (56.2, 23.2),
    (56.2, 31.0),
    (53.2, 32.7),
    (51.3, 30.6),
    (51.5, 23.6),
    (54.3, 23.2),
];

const NORTH_KOREA: &[(f64, f64)] = &[
    (43.0, 129.0),
    (42.4, 130.7),
    (38.6, 128.4),
    (37.7, 126.0),
    (39.8, 124.3),
    (41.8, 126.5),
];

const LIBYA: &[(f64, f64)] = &[
    (33.2, 11.5),
    (32.9, 25.0),
    (20.0, 25.0),
    (19.5, 15.0),
    (23.5, 10.0),
    (30.2, 9.5),
];

const SYRIA: &[(f64, f64)] = &[
    (37.3, 36.7),
    (37.1, 42.4),
    (33.4, 38.8),
    (32.3, 36.0),
    (33.3, 35.6),
    (35.9, 35.9),
];

const AFGHANISTAN: &[(f64, f64)] = &[
    (38.5, 71.0),
    (37.2, 74.9),
    (35.0, 71.1),
    (29.4, 66.0),
    (30.0, 61.8),
    (35.0, 61.0),
    (37.3, 66.5),
];

const YEMEN: &[(f64, f64)] = &[
    (17.5, 43.0),
    (16.6, 53.1),
    (12.6, 53.3),
    (12.6, 43.3),
    (15.0, 42.6),
];

const SUDAN: &[(f64, f64)] = &[
    (22.0, 24.0),
    (22.0, 37.0),
    (15.0, 39.0),
    (9.5, 34.0),
    (9.5, 24.0),
];

/// The regions avoided, in rough order of how often they matter.
pub fn closed_regions() -> Vec<ClosedRegion> {
    vec![
        ClosedRegion::new("Russia", RUSSIA),
        ClosedRegion::new("Russian Arctic sector", RUSSIAN_ARCTIC_SECTOR),
        ClosedRegion::new("Ukraine", UKRAINE),
        ClosedRegion::new("Belarus", BELARUS),
        ClosedRegion::new("Afghanistan", AFGHANISTAN),
        ClosedRegion::new("Syria", SYRIA),
        ClosedRegion::new("Libya", LIBYA),
        ClosedRegion::new("Sudan", SUDAN),
        ClosedRegion::new("Yemen", YEMEN),
        ClosedRegion::new("North Korea", NORTH_KOREA),
    ]
}

/// The set of regions a particular flight has to avoid.
pub struct AirspaceRestrictions {
    regions: Vec<ClosedRegion>,
}

impl AirspaceRestrictions {
    /// Restrictions applying to a flight between two airports.
    ///
    /// A region containing either end is dropped. An airline based inside closed
    /// airspace flies over it perfectly happily — the closure is against foreign
    /// operators, not against physics — and without this a domestic Russian route would
    /// have no legal path at all.
    pub fn for_route(departure: LatLon, arrival: LatLon) -> Self {
        let regions = closed_regions()
            .into_iter()
            .filter(|r| !r.contains(departure) && !r.contains(arrival))
            .collect();
        Self { regions }
    }

    pub fn none() -> Self {
        Self {
            regions: Vec::new(),
        }
    }

    pub fn blocks(&self, p: LatLon) -> bool {
        self.regions.iter().any(|r| r.contains(p))
    }

    pub fn is_empty(&self) -> bool {
        self.regions.is_empty()
    }
}
