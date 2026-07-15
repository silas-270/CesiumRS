fn main() {
    let p_cam_base = [7.415f32, 14.539, 1.184];
    // Frankfurt 
    let lon = 8.5706f64.to_radians();
    let lat = 50.0333f64.to_radians();
    // ECEF
    let a = 6378137.0; // Wait, maybe radius is 1.0 or 6.371?
    let r = 6.371;
    let fra_ecef = [
        r * lat.cos() * lon.cos(),
        r * lat.cos() * lon.sin(),
        r * lat.sin()
    ];
    // Actually just use vectors
}
