use std::f64::consts::PI;

pub fn main() {
    let departure_lon = 9.22196; // STR
    let departure_lat = 48.689899; // STR
    let arrival_lon = 13.499078; // NBJ
    let arrival_lat = -9.050734; // NBJ
    let total_duration_ms = 600_000;

    let lat_mid = (departure_lat + arrival_lat).to_radians() / 2.0;
    let m_per_deg_lat = 111320.0;
    let m_per_deg_lon = 111320.0 * lat_mid.cos();

    println!("m_per_deg_lat: {}", m_per_deg_lat);
    println!("m_per_deg_lon: {}", m_per_deg_lon);

    let p_arr_x = (arrival_lon - departure_lon) * m_per_deg_lon;
    let p_arr_y = (arrival_lat - departure_lat) * m_per_deg_lat;

    println!("p_arr_x: {}", p_arr_x);
    println!("p_arr_y: {}", p_arr_y);

    let direct_heading = (p_arr_y).atan2(p_arr_x);
    println!("direct_heading: {}", direct_heading);
}
