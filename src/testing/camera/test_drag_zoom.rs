use cesium_engine::camera::camera::Camera;
use glam::{DVec3, Vec3};

// Helper for the test to check where the ray ACTUALLY hits the WGS84 Ellipsoid
fn intersect_ellipsoid(ray_origin: Vec3, ray_dir: Vec3) -> Option<DVec3> {
    let a = 6.378137;
    let b = 6.3567523142;
    
    let ro = DVec3::new(ray_origin.x as f64 / a, ray_origin.y as f64 / b, ray_origin.z as f64 / a);
    let rd = DVec3::new(ray_dir.x as f64 / a, ray_dir.y as f64 / b, ray_dir.z as f64 / a);
    
    let qa = rd.length_squared();
    let qb = 2.0 * ro.dot(rd);
    let qc = ro.length_squared() - 1.0;
    
    let discriminant = qb * qb - 4.0 * qa * qc;
    if discriminant < 0.0 { return None; }
    
    let t = (-qb - discriminant.sqrt()) / (2.0 * qa);
    if t < 0.0 { return None; }
    
    Some(DVec3::new(
        ray_origin.x as f64 + ray_dir.x as f64 * t,
        ray_origin.y as f64 + ray_dir.y as f64 * t,
        ray_origin.z as f64 + ray_dir.z as f64 * t,
    ))
}

#[test]
fn test_drag_zoom() {
    let screen_w = 1920.0;
    let screen_h = 1080.0;
    
    // We will place the camera over Stuttgart (~48 deg North).
    // The radius there is somewhere between equatorial(6.378) and polar(6.356).
    // Let's position the camera precisely over the ellipsoid at 48 deg lat.
    let lat = 48.0_f64.to_radians();
    let a = 6.378137;
    let b = 6.3567523142;
    let stuttgart_surface_y = b * lat.sin();
    let stuttgart_surface_z = a * lat.cos();
    
    let altitudes = [0.02, 0.006, 0.001, 0.0001];
    
    for alt in altitudes {
        println!("========================================");
        println!("Testing Altitude: {}", alt);
        
        let start_pos = Vec3::new(0.0, stuttgart_surface_y as f32, stuttgart_surface_z as f32 + alt as f32);
        let mut cam = Camera::new(start_pos, Vec3::ZERO);
        // Look straight down at the surface
        cam.local_ori = glam::Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2);
        cam.anchor_ori = glam::DQuat::IDENTITY;
        
        // Simulating the user dragging from center (Stuttgart) to the right edge
        let paths = vec![
            ("Drag Across Screen (Ellipsoid Parallax Lag Check)", (960.0, 540.0), (1460.0, 540.0), 50),
        ];
        
        for (name, start_px, end_px, steps) in paths {
            println!("  -- Path: {} --", name);
            cam.set_eye(start_pos, Vec3::ZERO);
            cam.local_ori = glam::Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2);
            
            let start_x = start_px.0;
            let start_y = start_px.1;
            
            let (ray_origin, ray_dir) = cam.screen_to_world_ray(start_x, start_y, screen_w, screen_h);
            
            // What point on the ACTUAL ELLIPSOID is under the mouse before drag?
            let initial_earth_point = intersect_ellipsoid(ray_origin, ray_dir).unwrap();
            
            cam.begin_drag(start_x, start_y, screen_w, screen_h);
            
            let mut final_x = start_x;
            let mut final_y = start_y;

            for i in 1..=steps {
                let t = i as f32 / steps as f32;
                let cur_x = start_x + (end_px.0 - start_x) * t;
                let cur_y = start_y + (end_px.1 - start_y) * t;
                
                cam.drag(cur_x, cur_y, screen_w, screen_h);
                final_x = cur_x;
                final_y = cur_y;
            }
            
            let (new_ray_o, new_ray_d) = cam.screen_to_world_ray(final_x, final_y, screen_w, screen_h);
            
            // What point on the ACTUAL ELLIPSOID is under the mouse AFTER drag?
            if let Some(new_hit) = intersect_ellipsoid(new_ray_o, new_ray_d) {
                let diff = (initial_earth_point - new_hit).length();
                let drift_meters = diff * 1_000_000.0;
                
                println!("    Earth Point Drift on ELLIPSOID: {:.6} meters", drift_meters);
                
                if drift_meters > 1.0 {
                    println!("    [BUG!] PARALLAX LAGGING DETECTED! Map drifted away from mouse pointer.");
                } else {
                    println!("    [FIXED] Smooth exact movement on ellipsoid.");
                }
            } else {
                println!("    RAY FAILED TO HIT EARTH");
            }
            cam.end_drag();
        }
        println!();
    }
}
