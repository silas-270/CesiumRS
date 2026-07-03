use crate::camera::camera::Camera;
use crate::globe::quadtree::{QuadtreeManager, TileId};
use glam::{DVec3, Vec3, Vec4};
use rayon::prelude::*;
use std::collections::HashSet;

// --- Math Helpers ---
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

fn dvec3_to_lat_lon(point: DVec3) -> (f64, f64) {
    let a = 6.378137;
    let b = 6.3567523142;
    
    let phi = (point.y / b).asin();
    let theta = (point.z / -a).atan2(point.x / a);
    
    (phi.to_degrees(), theta.to_degrees())
}

fn lat_to_web_mercator_y(lat: f64, z: u8) -> u32 {
    let n = (1_u32 << z) as f64;
    let phi = lat.to_radians();
    
    // tan() and asinh() behave well even near poles, but exactly at poles they can hit infinity.
    let tan_phi = phi.tan();
    if tan_phi.is_infinite() {
        if lat > 0.0 {
            return 0;
        } else {
            return (1_u32 << z).saturating_sub(1);
        }
    }
    
    let y = (n / 2.0) * (1.0 - tan_phi.asinh() / std::f64::consts::PI);
    let max_y = (1_u32 << z).saturating_sub(1);
    
    if y.is_nan() || y < 0.0 {
        0
    } else if y > max_y as f64 {
        max_y
    } else {
        y.floor() as u32
    }
}

fn tile_contains(tile: &TileId, lat: f64, lon: f64) -> bool {
    let z_pow = (1_u32 << tile.z) as f64;
    
    let mut expected_x = (((lon + 180.0) / 360.0) * z_pow).floor() as u32;
    if expected_x >= (1_u32 << tile.z) {
        expected_x = (1_u32 << tile.z) - 1;
    }
    
    let expected_y = lat_to_web_mercator_y(lat, tile.z);
    
    tile.x == expected_x && tile.y == expected_y
}

// --- Test Infrastructure ---

struct CoverageResult {
    false_negatives: Vec<(f64, f64)>,
    false_positives: usize,
    rendered_tiles: usize,
    hit_tiles: usize,
}

fn check_frustum_coverage(cam: &Camera, screen_w: u32, screen_h: u32, name: &str) -> CoverageResult {
    let mut quadtree = QuadtreeManager::new();
    // Default lod_factor is 2.0, let's use it.
    
    // Force a few updates to make sure quadtree subdivides completely
    let view_proj = cam.get_projection_matrix(screen_w as f32 / screen_h as f32) * cam.get_view_matrix();
    for _ in 0..30 {
        quadtree.update(cam.global_transform().0, view_proj);
    }
    
    let visible_tiles_data = quadtree.get_visible_tiles();
    let visible_tiles: Vec<TileId> = visible_tiles_data.iter().map(|(id, _, _)| *id).collect();
    
    let rendered_tiles = visible_tiles.len();
    
    let points: Vec<(u32, u32)> = (0..screen_w).flat_map(|x| (0..screen_h).map(move |y| (x, y))).collect();
    
    // Raycast in parallel
    let results: Vec<Result<TileId, (f64, f64)>> = points.par_iter().filter_map(|&(x, y)| {
        let (ray_origin, ray_dir) = cam.screen_to_world_ray(x as f32, y as f32, screen_w as f32, screen_h as f32);
        
        if let Some(hit) = intersect_ellipsoid(ray_origin, ray_dir) {
            // Check if the hit point is actually in front of the camera using near plane
            let view_pos = cam.get_view_matrix() * Vec4::new(hit.x as f32, hit.y as f32, hit.z as f32, 1.0);
            if view_pos.z > 0.0 {
                // Point is behind the camera (Right-handed view space, -Z is forward)
                return None;
            }
            
            let (lat, lon) = dvec3_to_lat_lon(hit);
            
            for tile in &visible_tiles {
                if tile_contains(tile, lat, lon) {
                    return Some(Ok(*tile));
                }
            }
            
            // False negative! Hit the globe but no tile claimed it.
            return Some(Err((lat, lon)));
        }
        None
    }).collect();
    
    let mut hit_set = HashSet::new();
    let mut false_negatives = Vec::new();
    
    for res in results {
        match res {
            Ok(tile) => { hit_set.insert(tile); },
            Err(miss) => false_negatives.push(miss),
        }
    }
    
    let hit_tiles = hit_set.len();
    let false_positives = rendered_tiles.saturating_sub(hit_tiles);
    
    println!("--- {} ---", name);
    println!("  Camera Local Pos: x: {}, y: {}, z: {}", cam.local_pos.x, cam.local_pos.y, cam.local_pos.z);
    let (yaw, pitch, roll) = cam.local_ori.to_euler(glam::EulerRot::YXZ);
    println!("  Camera Local Rot (YXZ deg): P: {:.4}, Y: {:.4}, R: {:.4}", pitch.to_degrees(), yaw.to_degrees(), roll.to_degrees());
    println!("  Total Rendered Tiles: {}", rendered_tiles);
    println!("Unique Tiles Hit: {}", hit_tiles);
    println!("False Positives (Ghost Tiles Bound): {}", false_positives);
    println!("False Negatives (Missing Tiles): {}", false_negatives.len());
    
    if !false_negatives.is_empty() {
        println!("Sample False Negative: Lat: {}, Lon: {}", false_negatives[0].0, false_negatives[0].1);
    }
    
    CoverageResult {
        false_negatives,
        false_positives,
        rendered_tiles,
        hit_tiles,
    }
}

fn setup_camera(lat_deg: f32, lon_deg: f32, altitude: f32, pitch_deg: f32) -> Camera {
    let a = 6.378137;
    let b = 6.3567523142;
    
    let phi = lat_deg.to_radians();
    let theta = lon_deg.to_radians();
    
    let surface_x = a * phi.cos() * theta.cos();
    let surface_y = b * phi.sin();
    let surface_z = -a * phi.cos() * theta.sin();
    let surface_pos = Vec3::new(surface_x, surface_y, surface_z);
    
    let normal = Vec3::new(
        surface_x / (a*a),
        surface_y / (b*b),
        surface_z / (a*a),
    ).normalize();
    
    let pos = surface_pos + normal * altitude;
    
    let mut cam = Camera::new(pos, Vec3::ZERO);
    cam.set_eye(pos, Vec3::ZERO); // Looks straight down (-Z points to center)
    
    if pitch_deg != 0.0 {
        let pitch_quat = glam::Quat::from_axis_angle(Vec3::X, pitch_deg.to_radians());
        cam.rotate_local(pitch_quat);
    }
    
    cam
}

#[test]
fn test_equivalence_partitioning_frustum() {
    let mut failed = false;
    
    let test_cases = vec![
        ("High Altitude Overview (Straight Down)", setup_camera(48.0, 9.0, 10.0, 0.0)),
        ("Mid Altitude Angled (45 deg)", setup_camera(48.0, 9.0, 1.0, 45.0)),
        ("Low Altitude Tangent (Looking at horizon)", setup_camera(48.0, 9.0, 0.000015, 89.9)),
        ("Low Altitude Straight Down", setup_camera(48.0, 9.0, 0.000015, 0.0)),
        ("North Pole Straight Down", setup_camera(89.9, 0.0, 2.0, 0.0)),
        ("South Pole Straight Down", setup_camera(-89.9, 0.0, 2.0, 0.0)),
        ("Equator Tangent", setup_camera(0.0, 0.0, 0.01, 85.0)),
    ];
    
    let w = 200;
    let h = 150;
    
    for (name, cam) in test_cases {
        let res = check_frustum_coverage(&cam, w, h, name);
        if !res.false_negatives.is_empty() {
            failed = true;
            println!("TEST FAILED for {}", name);
        }
        
        // Let's assert a reasonable upper limit for false positives.
        // It shouldn't be rendering 500 extra tiles we don't hit at this resolution.
        if res.false_positives > 150 {
            println!("WARNING: High number of false positives ({}) for {}", res.false_positives, name);
        }
    }
    
    assert!(!failed, "One or more cases produced false negatives!");
}
