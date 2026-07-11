use glam::{Vec3, Quat, EulerRot};

fn lon_lat_alt_to_ecef_f64(lon: f64, lat: f64, alt: f64) -> [f64; 3] {
    let a = 6.378137;
    let b = 6.3567523142;
    let a2 = a * a;
    let b2 = b * b;

    let clat = lat.to_radians().cos();
    let slat = lat.to_radians().sin();
    let clon = lon.to_radians().cos();
    let slon = lon.to_radians().sin();

    let n = a2 / (a2 * clat * clat + b2 * slat * slat).sqrt();

    [
        (n + alt) * clat * clon,
        (n + alt) * clat * slon,
        ((b2 / a2) * n + alt) * slat,
    ]
}

fn main() {
    let p_cam = Vec3::new(7.415, 14.539, 1.184);
    let q_cam = Quat::from_euler(
        EulerRot::YXZ,
        79.24_f32.to_radians(),
        -62.75_f32.to_radians(),
        -94.11_f32.to_radians(),
    );

    let fra_ecef = lon_lat_alt_to_ecef_f64(8.5706, 50.0333, 0.0);
    let fra_pos = Vec3::new(fra_ecef[0] as f32, fra_ecef[1] as f32, fra_ecef[2] as f32);

    let up = fra_pos.normalize();
    let east = Vec3::new(0.0, 0.0, 1.0).cross(up).normalize();
    let north = up.cross(east).normalize();

    let offset_world = p_cam - fra_pos;
    let e = offset_world.dot(east);
    let n = offset_world.dot(north);
    let u = offset_world.dot(up);

    println!("East offset: {}", e);
    println!("North offset: {}", n);
    println!("Up offset: {}", u);

    // Now for rotation.
    // The camera's forward vector in world space:
    let forward = q_cam * Vec3::new(0.0, 0.0, -1.0);
    
    let f_e = forward.dot(east);
    let f_n = forward.dot(north);
    let f_u = forward.dot(up);

    println!("Forward vector in ENU: ({}, {}, {})", f_e, f_n, f_u);
    
    // Calculate the target it is looking at (say, where forward intersects the ground)
    // p_cam + forward * t = ground
    // Just estimate the distance to target (roughly height / -f_u)
    let t = u / -f_u;
    let target = p_cam + forward * t;
    
    let target_offset = target - fra_pos;
    let t_e = target_offset.dot(east);
    let t_n = target_offset.dot(north);
    let t_u = target_offset.dot(up);
    
    println!("Target offset in ENU: ({}, {}, {})", t_e, t_n, t_u);
}
