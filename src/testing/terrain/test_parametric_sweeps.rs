// Removed unused import
use cesium_engine::camera::camera::Camera;
use glam::{Vec3, Quat, EulerRot};
use super::test_frustum_coverage::{intersect_ellipsoid, dvec3_to_lat_lon, tile_contains};
use cesium_engine::globe::quadtree::{TileId, QuadtreeManager};
use std::io::Write;
use rayon::prelude::*;
use glam::Vec4;

use super::test_frustum_coverage::setup_camera;

fn setup_parametric_camera(lat: f32, lon: f32, alt: f32, pitch: f32, roll: f32) -> Camera {
    // setup_camera automatically looks straight down at the earth, and applies a pitch offset
    let mut cam = setup_camera(lat, lon, alt, pitch);
    
    // Apply roll
    if roll != 0.0 {
        let roll_quat = Quat::from_axis_angle(Vec3::Z, roll.to_radians());
        cam.rotate_local(roll_quat);
    }
    
    cam
}

fn evaluate_camera(file: &mut std::fs::File, sweep_type: &str, param_value: f32, lat: f32, lon: f32, alt: f32, pitch: f32, yaw: f32, roll: f32) {
    let cam = setup_parametric_camera(lat, lon, alt, pitch, roll);
    let w = 1920;
    let h = 1080;
    let aspect = w as f32 / h as f32;

    let mut quadtree = QuadtreeManager::new();
    let frustum_planes = cam.calculate_frustum_planes(aspect);
    let (global_pos_dvec, _) = cam.global_transform_f64();
    let global_pos_f32 = glam::Vec3::new(global_pos_dvec.x as f32, global_pos_dvec.y as f32, global_pos_dvec.z as f32);
    for _ in 0..30 {
        quadtree.update(global_pos_f32, frustum_planes);
    }
    let visible_tiles_data = quadtree.get_visible_tiles();
    let visible_tiles: Vec<TileId> = visible_tiles_data.iter().map(|(id, _, _)| *id).collect();
    let rendered_tiles = visible_tiles.len();
    
    if rendered_tiles == 0 {
        writeln!(file, "{},{},{},{},{},{},{},{},0,0,0,0", sweep_type, param_value, lat, lon, alt, pitch, yaw, roll).unwrap();
        return;
    }
    
    let step_x = (w / 100).max(1);
    let step_y = (h / 80).max(1);
    
    let mut points = Vec::new();
    for x in (0..w).step_by(step_x as usize) {
        for y in (0..h).step_by(step_y as usize) {
            points.push((x, y));
        }
    }
    
    let results: Vec<Result<TileId, (f64, f64)>> = points.par_iter().filter_map(|&(x, y)| {
        let (ray_origin, ray_dir) = cam.screen_to_world_ray(x as f32, y as f32, w as f32, h as f32);
        if let Some(hit) = intersect_ellipsoid(ray_origin, ray_dir) {
            let view_pos = cam.get_view_matrix() * Vec4::new(hit.x as f32, hit.y as f32, hit.z as f32, 1.0);
            if view_pos.z > 0.0 { return None; }
            let (hit_lat, hit_lon) = dvec3_to_lat_lon(hit);
            for tile in &visible_tiles {
                if tile_contains(tile, hit_lat, hit_lon) {
                    return Some(Ok(*tile));
                }
            }
            return Some(Err((hit_lat, hit_lon)));
        }
        None
    }).collect();
    
    let mut hit_set = std::collections::HashSet::new();
    let mut false_negatives = 0;
    
    for res in results {
        match res {
            Ok(tile) => { hit_set.insert(tile); },
            Err(_) => { false_negatives += 1; },
        }
    }
    
    let hit_tiles = hit_set.len();
    let false_positives = rendered_tiles.saturating_sub(hit_tiles);
    
    writeln!(file, "{},{},{},{},{},{},{},{},{},{},{},{}", 
        sweep_type, param_value, lat, lon, alt, pitch, yaw, roll, 
        rendered_tiles, hit_tiles, false_positives, false_negatives).unwrap();
}

#[test]
fn test_parametric_sweeps() {
    let mut file = std::fs::File::create("parametric_results.csv").unwrap();
    writeln!(file, "sweep_type,param_value,lat,lon,alt,pitch,yaw,roll,rendered,hit,false_positives,false_negatives").unwrap();
    
    // 1. Altitude Sweep (Looking straight down)
    println!("Running Altitude Sweep...");
    let num_steps = 100;
    let min_alt = 0.00001; // ~10cm
    let max_alt = 10.0;    // ~10,000km
    for i in 0..=num_steps {
        let t = i as f32 / num_steps as f32;
        // logarithmic sweep for altitude
        let alt = min_alt * (max_alt / min_alt as f32).powf(t);
        evaluate_camera(&mut file, "Altitude", alt, 0.0, 0.0, alt, -90.0, 0.0, 0.0);
    }
    
    // 2. Pitch Sweep (From straight down to looking at space)
    println!("Running Pitch Sweep...");
    let alt = 0.1; // ~100km
    for i in 0..=num_steps {
        let t = i as f32 / num_steps as f32;
        let pitch = -90.0 + t * 180.0; // -90 (down) to +90 (up)
        evaluate_camera(&mut file, "Pitch", pitch, 0.0, 0.0, alt, pitch, 0.0, 0.0);
    }
    
    // 3. Roll Sweep (Dutch Angle)
    println!("Running Roll Sweep...");
    let pitch = -45.0; // angled view
    for i in 0..=num_steps {
        let t = i as f32 / num_steps as f32;
        let roll = -180.0 + t * 360.0;
        evaluate_camera(&mut file, "Roll", roll, 0.0, 0.0, alt, pitch, 0.0, roll);
    }

    // 4. Combined Pitch & Roll
    println!("Running Combined Pitch & Roll...");
    for i in 0..=20 { // 21 steps pitch
        for j in 0..=20 { // 21 steps roll
            let p_t = i as f32 / 20.0;
            let r_t = j as f32 / 20.0;
            let pitch = -90.0 + p_t * 90.0; // -90 to 0
            let roll = -180.0 + r_t * 360.0;
            evaluate_camera(&mut file, "Combined_Pitch_Roll", pitch, 0.0, 0.0, alt, pitch, 0.0, roll);
        }
    }
}
