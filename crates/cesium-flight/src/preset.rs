//! Route presets and parsing for interactive flight tracking and CLI.

use crate::telemetry::geo::{distance_m, initial_bearing, LatLon};

#[derive(Debug, Clone, PartialEq)]
pub struct FlightRouteDef {
    pub id: String,
    pub departure_lon: f64,
    pub departure_lat: f64,
    pub arrival_lon: f64,
    pub arrival_lat: f64,
    pub total_duration_ms: u64,
    pub dep_heading_deg: Option<f64>,
    pub arr_heading_deg: Option<f64>,
    pub dep_elevation_m: Option<f64>,
    pub arr_elevation_m: Option<f64>,
}

#[derive(Debug, Clone, Copy)]
pub struct RoutePresetInfo {
    pub id: &'static str,
    pub label: &'static str,
    pub dep_lon: f64,
    pub dep_lat: f64,
    pub arr_lon: f64,
    pub arr_lat: f64,
    pub duration_ms: u64,
    pub dep_heading_deg: Option<f64>,
    pub arr_heading_deg: Option<f64>,
    pub dep_elevation_m: Option<f64>,
    pub arr_elevation_m: Option<f64>,
}

#[derive(Debug, Clone, Copy)]
pub struct AirportElevationInfo {
    pub icao: &'static str,
    pub lat: f64,
    pub lon: f64,
    pub elevation_m: f64,
}

pub const KNOWN_AIRPORTS: &[AirportElevationInfo] = &[
    AirportElevationInfo { icao: "EDDF", lat: 50.0333, lon: 8.5706, elevation_m: 111.0 }, // Frankfurt
    AirportElevationInfo { icao: "EDDS", lat: 48.6899, lon: 9.2219, elevation_m: 389.0 }, // Stuttgart
    AirportElevationInfo { icao: "EGLL", lat: 51.4706, lon: -0.4619, elevation_m: 25.0 }, // London Heathrow
    AirportElevationInfo { icao: "LFPG", lat: 49.0097, lon: 2.5479, elevation_m: 119.0 }, // Paris CDG
    AirportElevationInfo { icao: "LSZH", lat: 47.4581, lon: 8.5555, elevation_m: 432.0 }, // Zurich
    AirportElevationInfo { icao: "LSGG", lat: 46.2370, lon: 6.1092, elevation_m: 430.0 }, // Geneva
    AirportElevationInfo { icao: "LOWG", lat: 46.9911, lon: 15.4396, elevation_m: 340.0 }, // Graz
    AirportElevationInfo { icao: "KJFK", lat: 40.6413, lon: -73.7781, elevation_m: 4.0 }, // New York JFK
    AirportElevationInfo { icao: "KSFO", lat: 37.6213, lon: -122.3790, elevation_m: 4.0 }, // San Francisco
    AirportElevationInfo { icao: "PHNL", lat: 21.3187, lon: -157.9224, elevation_m: 4.0 }, // Honolulu
    AirportElevationInfo { icao: "RJAA", lat: 35.7647, lon: 140.3864, elevation_m: 43.0 }, // Tokyo Narita
    AirportElevationInfo { icao: "OMDB", lat: 25.2532, lon: 55.3657, elevation_m: 19.0 }, // Dubai
    AirportElevationInfo { icao: "YSSY", lat: -33.9399, lon: 151.1753, elevation_m: 6.0 }, // Sydney
    AirportElevationInfo { icao: "WSSS", lat: 1.3644, lon: 103.9915, elevation_m: 7.0 }, // Singapore
    AirportElevationInfo { icao: "EDDM", lat: 48.3538, lon: 11.7861, elevation_m: 453.0 }, // Munich
    AirportElevationInfo { icao: "LOWW", lat: 48.1103, lon: 16.5697, elevation_m: 183.0 }, // Vienna
    AirportElevationInfo { icao: "EDDH", lat: 53.6304, lon: 10.0077, elevation_m: 16.0 }, // Hamburg
    AirportElevationInfo { icao: "EDDB", lat: 52.3622, lon: 13.5007, elevation_m: 48.0 }, // Berlin
    AirportElevationInfo { icao: "EGKK", lat: 51.1537, lon: -0.1821, elevation_m: 62.0 }, // London Gatwick
    AirportElevationInfo { icao: "EDDL", lat: 51.2895, lon: 6.7668, elevation_m: 45.0 }, // Duesseldorf
    AirportElevationInfo { icao: "LTAI", lat: 36.8987, lon: 30.8005, elevation_m: 54.0 }, // Antalya
    AirportElevationInfo { icao: "HEGN", lat: 27.1783, lon: 33.7994, elevation_m: 16.0 }, // Hurghada
    AirportElevationInfo { icao: "LGKO", lat: 36.7933, lon: 27.0917, elevation_m: 125.0 }, // Kos
    AirportElevationInfo { icao: "LEPA", lat: 39.5517, lon: 2.7388, elevation_m: 8.0 }, // Palma de Mallorca
    AirportElevationInfo { icao: "SKBO", lat: 4.7016, lon: -74.1469, elevation_m: 2548.0 }, // Bogota
    AirportElevationInfo { icao: "MMMX", lat: 19.4363, lon: -99.0721, elevation_m: 2230.0 }, // Mexico City
    AirportElevationInfo { icao: "LOWI", lat: 47.2602, lon: 11.3440, elevation_m: 581.0 }, // Innsbruck
    AirportElevationInfo { icao: "LSZS", lat: 46.5341, lon: 9.8841, elevation_m: 1707.0 }, // Samedan
    AirportElevationInfo { icao: "VNLK", lat: 27.6881, lon: 86.7314, elevation_m: 2845.0 }, // Lukla
];

/// Looks up the nearest known airport elevation within ~50 km (0.5 degrees).
pub fn lookup_airport_elevation(lat: f64, lon: f64) -> Option<f64> {
    let mut closest_dist_sq = f64::MAX;
    let mut best_elev = None;
    let max_radius_deg = 0.5;
    let max_dist_sq = max_radius_deg * max_radius_deg;

    for airport in KNOWN_AIRPORTS {
        let d_lat = lat - airport.lat;
        let d_lon = (lon - airport.lon) * lat.to_radians().cos();
        let dist_sq = d_lat * d_lat + d_lon * d_lon;
        if dist_sq < max_dist_sq && dist_sq < closest_dist_sq {
            closest_dist_sq = dist_sq;
            best_elev = Some(airport.elevation_m);
        }
    }

    best_elev
}

impl RoutePresetInfo {
    pub fn to_route_def(&self) -> FlightRouteDef {
        FlightRouteDef {
            id: self.id.to_string(),
            departure_lon: self.dep_lon,
            departure_lat: self.dep_lat,
            arrival_lon: self.arr_lon,
            arrival_lat: self.arr_lat,
            total_duration_ms: self.duration_ms,
            dep_heading_deg: self.dep_heading_deg,
            arr_heading_deg: self.arr_heading_deg,
            dep_elevation_m: self.dep_elevation_m.or_else(|| lookup_airport_elevation(self.dep_lat, self.dep_lon)),
            arr_elevation_m: self.arr_elevation_m.or_else(|| lookup_airport_elevation(self.arr_lat, self.arr_lon)),
        }
    }
}

pub const PRESETS: &[RoutePresetInfo] = &[
    RoutePresetInfo {
        id: "STR-FRA",
        label: "Stuttgart -> Frankfurt (156 km)",
        dep_lon: 9.2219,
        dep_lat: 48.6899,
        arr_lon: 8.5706,
        arr_lat: 50.0333,
        duration_ms: 1_800_000,
        dep_heading_deg: Some(73.0),
        arr_heading_deg: Some(249.0),
        dep_elevation_m: Some(389.0),
        arr_elevation_m: Some(111.0),
    },
    RoutePresetInfo {
        id: "FRA-STR",
        label: "Frankfurt -> Stuttgart (156 km)",
        dep_lon: 8.5706,
        dep_lat: 50.0333,
        arr_lon: 9.2219,
        arr_lat: 48.6899,
        duration_ms: 1_800_000,
        dep_heading_deg: Some(249.0),
        arr_heading_deg: Some(73.0),
        dep_elevation_m: Some(111.0),
        arr_elevation_m: Some(389.0),
    },
    RoutePresetInfo {
        id: "LHR-CDG",
        label: "London Heathrow -> Paris CDG (348 km)",
        dep_lon: -0.4619,
        dep_lat: 51.4706,
        arr_lon: 2.5479,
        arr_lat: 49.0097,
        duration_ms: 2_400_000,
        dep_heading_deg: None,
        arr_heading_deg: None,
        dep_elevation_m: Some(25.0),
        arr_elevation_m: Some(119.0),
    },
    RoutePresetInfo {
        id: "ZRH-GVA",
        label: "Zurich -> Geneva (231 km)",
        dep_lon: 8.5555,
        dep_lat: 47.4581,
        arr_lon: 6.1092,
        arr_lat: 46.2370,
        duration_ms: 2_100_000,
        dep_heading_deg: None,
        arr_heading_deg: None,
        dep_elevation_m: Some(432.0),
        arr_elevation_m: Some(430.0),
    },
    RoutePresetInfo {
        id: "GRZ-FRA",
        label: "Graz -> Frankfurt (600 km)",
        dep_lon: 15.4396,
        dep_lat: 46.9911,
        arr_lon: 8.5584,
        arr_lat: 50.0267,
        duration_ms: 4_200_000,
        dep_heading_deg: None,
        arr_heading_deg: None,
        dep_elevation_m: Some(340.0),
        arr_elevation_m: Some(111.0),
    },
    RoutePresetInfo {
        id: "JFK-LHR",
        label: "New York JFK -> London Heathrow (5,550 km)",
        dep_lon: -73.7781,
        dep_lat: 40.6413,
        arr_lon: -0.4619,
        arr_lat: 51.4706,
        duration_ms: 25_200_000,
        dep_heading_deg: Some(44.0),
        arr_heading_deg: Some(272.0),
        dep_elevation_m: Some(4.0),
        arr_elevation_m: Some(25.0),
    },
    RoutePresetInfo {
        id: "SFO-HNL",
        label: "San Francisco -> Honolulu (3,860 km)",
        dep_lon: -122.3790,
        dep_lat: 37.6213,
        arr_lon: -157.9224,
        arr_lat: 21.3187,
        duration_ms: 18_000_000,
        dep_heading_deg: Some(255.0),
        arr_heading_deg: Some(83.0),
        dep_elevation_m: Some(4.0),
        arr_elevation_m: Some(4.0),
    },
    RoutePresetInfo {
        id: "LHR-NRT",
        label: "London Heathrow -> Tokyo Narita (9,600 km)",
        dep_lon: -0.4619,
        dep_lat: 51.4706,
        arr_lon: 140.3864,
        arr_lat: 35.7647,
        duration_ms: 43_200_000,
        dep_heading_deg: Some(92.0),
        arr_heading_deg: Some(157.0),
        dep_elevation_m: Some(25.0),
        arr_elevation_m: Some(43.0),
    },
    RoutePresetInfo {
        id: "DXB-SYD",
        label: "Dubai -> Sydney (12,040 km)",
        dep_lon: 55.3657,
        dep_lat: 25.2532,
        arr_lon: 151.1753,
        arr_lat: -33.9399,
        duration_ms: 50_400_000,
        dep_heading_deg: Some(122.0),
        arr_heading_deg: Some(160.0),
        dep_elevation_m: Some(19.0),
        arr_elevation_m: Some(6.0),
    },
    RoutePresetInfo {
        id: "DXB-JFK",
        label: "Dubai -> New York JFK (11,000 km)",
        dep_lon: 55.3657,
        dep_lat: 25.2532,
        arr_lon: -73.7781,
        arr_lat: 40.6413,
        duration_ms: 50_400_000,
        dep_heading_deg: Some(315.0),
        arr_heading_deg: Some(224.0),
        dep_elevation_m: Some(19.0),
        arr_elevation_m: Some(4.0),
    },
    RoutePresetInfo {
        id: "SIN-LHR",
        label: "Singapore -> London Heathrow (10,880 km)",
        dep_lon: 103.9915,
        dep_lat: 1.3644,
        arr_lon: -0.4619,
        arr_lat: 51.4706,
        duration_ms: 46_800_000,
        dep_heading_deg: Some(20.0),
        arr_heading_deg: Some(272.0),
        dep_elevation_m: Some(7.0),
        arr_elevation_m: Some(25.0),
    },
];

/// Parses a route description string.
///
/// Accepts:
/// 1. Preset names (case-insensitive, hyphens or underscores): e.g. "FRA-STR", "lhr_nrt", "JFK-LHR".
/// 2. Comma or space separated coordinates:
///    - `lat1, lon1, lat2, lon2` (default GPS format)
///    - `lon1, lat1, lon2, lat2` (detected if coordinates exceed 90 degrees latitude)
///    - Optional 5th parameter for duration in minutes: `lat1, lon1, lat2, lon2, duration_min`
pub fn parse_route(input: &str) -> Result<FlightRouteDef, String> {
    let clean = input.trim();
    if clean.is_empty() {
        return Err("Empty route string".to_string());
    }

    // Try matching presets first
    let normalized = clean.to_uppercase().replace('_', "-").replace("->", "-").replace('>', "-");
    for preset in PRESETS {
        if preset.id == normalized {
            return Ok(preset.to_route_def());
        }
    }

    // Try parsing as coordinates: split by comma, semicolon, or whitespace
    let parts: Vec<&str> = clean
        .split(|c: char| c == ',' || c == ';' || c.is_whitespace())
        .map(|s| s.trim())
        .filter(|s| !s.is_empty())
        .collect();

    if parts.len() >= 4 {
        let nums: Result<Vec<f64>, _> = parts.iter().take(5).map(|p| p.parse::<f64>()).collect();
        let nums = nums.map_err(|_| format!("Could not parse numbers from '{}'", clean))?;

        let (dep_lat, dep_lon, arr_lat, arr_lon) = if nums[0].abs() > 90.0 || nums[2].abs() > 90.0 {
            // lon1, lat1, lon2, lat2 format
            (nums[1], nums[0], nums[3], nums[2])
        } else {
            // lat1, lon1, lat2, lon2 format
            (nums[0], nums[1], nums[2], nums[3])
        };

        if dep_lat.abs() > 85.05 || arr_lat.abs() > 85.05 {
            return Err("Latitude must be within [-85.05, 85.05] (Web Mercator limit)".to_string());
        }
        if dep_lon < -180.0 || dep_lon > 180.0 || arr_lon < -180.0 || arr_lon > 180.0 {
            return Err("Longitude must be within [-180.0, 180.0]".to_string());
        }

        let p1 = LatLon::new(dep_lat, dep_lon);
        let p2 = LatLon::new(arr_lat, arr_lon);
        let dist = distance_m(p1, p2);

        let duration_ms = if nums.len() >= 5 && nums[4] > 0.0 {
            (nums[4] * 60_000.0) as u64
        } else {
            // Realistic duration: ~240 m/s (864 km/h) cruise + 20 min terminal climb/descent
            let sec = (dist / 240.0 + 1200.0).max(1800.0);
            (sec * 1000.0) as u64
        };

        let initial_brg_deg = initial_bearing(p1, p2).to_degrees();
        let dep_elevation_m = lookup_airport_elevation(dep_lat, dep_lon);
        let arr_elevation_m = lookup_airport_elevation(arr_lat, arr_lon);

        return Ok(FlightRouteDef {
            id: format!("custom_{:.2}_{:.2}", dep_lat, arr_lat),
            departure_lon: dep_lon,
            departure_lat: dep_lat,
            arrival_lon: arr_lon,
            arrival_lat: arr_lat,
            total_duration_ms: duration_ms,
            dep_heading_deg: Some(initial_brg_deg),
            arr_heading_deg: None,
            dep_elevation_m,
            arr_elevation_m,
        });
    }

    Err(format!(
        "Unknown route '{}'. Use a preset (e.g. FRA-STR, LHR-NRT, JFK-LHR) or 'lat1,lon1,lat2,lon2'",
        clean
    ))
}
