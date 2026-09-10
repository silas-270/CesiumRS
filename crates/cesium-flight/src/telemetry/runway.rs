//! Choosing which runway, and which end of it, a flight uses.
//!
//! Aircraft take off and land into wind, because it lowers the ground speed at which
//! the wings start working and shortens everything that follows. So the end in use is a
//! property of the weather, not of the airport, and it flips as the weather does.
//!
//! The previous behaviour always used the low-numbered end of whichever runway happened
//! to be last in the list, which meant half of all departures rolled the wrong way down
//! the runway and airports with several runways picked arbitrarily.
//!
//! Headings are derived from the two threshold coordinates rather than read from the
//! stored heading column. The stored values are true bearings and agree with the
//! computed ones to about a quarter of a degree for almost every runway — but a handful
//! of rows have their ends transposed, and those disagree by 180°, which is exactly the
//! error that would be least obvious and most wrong.

use crate::flight_handle::RunwayData;

use super::geo::{distance_m, initial_bearing, LatLon};
use super::wind::WindField;

/// How far from the airport reference point a runway may be and still belong to it.
const AIRPORT_RADIUS_M: f64 = 25_000.0;

/// Headwind advantage that justifies choosing a shorter runway. Within this margin the
/// longer runway wins, which is the usual tie-break.
const HEADWIND_TIE_MPS: f64 = 2.0;

/// Below this the wind is not steady enough to dictate anything, so length decides.
const CALM_WIND_MPS: f64 = 1.0;

/// The runway end a flight will actually use.
#[derive(Debug, Clone, Copy)]
pub struct RunwayEnd {
    /// Landing or departure threshold.
    pub threshold: LatLon,
    /// Direction of the runway, radians clockwise from north.
    pub heading_rad: f64,
    pub length_m: f64,
}

/// Picks the end to use at `airport`.
///
/// `fallback_heading_rad` is used when the airport has no runway data at all — the
/// route's own direction, so the aircraft at least leaves and arrives pointing sensibly.
pub fn select(
    airport: LatLon,
    runways: &[RunwayData],
    wind: &WindField,
    fallback_heading_rad: f64,
) -> RunwayEnd {
    let (wind_east, wind_north) = wind.surface_wind(airport.lat_deg, airport.lon_deg);
    let wind_speed = (wind_east * wind_east + wind_north * wind_north).sqrt();

    let mut best: Option<(RunwayEnd, f64)> = None;
    for r in runways {
        let le = LatLon::new(r.le_lat, r.le_lon);
        let he = LatLon::new(r.he_lat, r.he_lon);
        // Runways are matched to their airport by proximity rather than by id, because
        // the ids are only meaningful to the caller's database.
        if distance_m(le, airport) > AIRPORT_RADIUS_M && distance_m(he, airport) > AIRPORT_RADIUS_M
        {
            continue;
        }
        let separation = distance_m(le, he);
        let length_m = if r.length_ft > 0.0 {
            r.length_ft as f64 * 0.3048
        } else {
            separation
        };

        // Departing from each end in turn: the threshold is one end and the aircraft
        // points at the other.
        for (threshold, other, stored_heading) in
            [(le, he, r.le_heading as f64), (he, le, r.he_heading as f64)]
        {
            let heading_rad = if separation > 100.0 {
                initial_bearing(threshold, other)
            } else {
                // Degenerate or missing coordinates; the stored heading is all there is.
                stored_heading.to_radians()
            };
            let end = RunwayEnd {
                threshold,
                heading_rad,
                length_m,
            };
            let headwind = if wind_speed < CALM_WIND_MPS {
                0.0
            } else {
                // Component of the wind blowing back down the runway.
                -(wind_east * heading_rad.sin() + wind_north * heading_rad.cos())
            };
            best = Some(match best {
                None => (end, headwind),
                Some((best_end, best_headwind)) => {
                    let decisive = headwind > best_headwind + HEADWIND_TIE_MPS;
                    let comparable = (headwind - best_headwind).abs() <= HEADWIND_TIE_MPS;
                    if decisive || (comparable && end.length_m > best_end.length_m) {
                        (end, headwind)
                    } else {
                        (best_end, best_headwind)
                    }
                }
            });
        }
    }

    best.map(|(end, _)| end).unwrap_or(RunwayEnd {
        threshold: airport,
        heading_rad: fallback_heading_rad,
        length_m: 3_000.0,
    })
}
