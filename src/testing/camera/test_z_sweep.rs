use super::test_frustum_coverage::{dvec3_to_lat_lon, tile_contains, intersect_ellipsoid};
use cesium_engine::camera::camera::Camera;
use glam::{Vec3, Vec4};
use cesium_engine::globe::quadtree::{TileId, QuadtreeManager};
use rayon::prelude::*;
use std::io::Write;

fn evaluate_direct_camera(file: &mut std::fs::File, z: f32) {
    let pos = Vec3::new(0.0, 0.0, z);
    let mut cam = Camera::new(pos, Vec3::ZERO);
    cam.set_eye(pos, Vec3::ZERO);
    
    let w = 1920;
    let h = 1080;

    let mut quadtree = QuadtreeManager::new();
        let aspect = w as f32 / h as f32;
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
        writeln!(file, "{},0,0,0,0", z).unwrap();
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
    
    let results: Vec<Result<TileId, ()>> = points.par_iter().filter_map(|&(x, y)| {
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
            return Some(Err(()));
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
    
    writeln!(file, "{},{},{},{},{}", z, rendered_tiles, hit_tiles, false_positives, false_negatives).unwrap();
}

#[test]
fn test_z_sweep() {
    let mut file = std::fs::File::create("z_sweep_results_fine.csv").unwrap();
    writeln!(file, "z,rendered,hit,false_positives,false_negatives").unwrap();
    
    println!("Running Z-Axis Sweep (Pos=0,0,Z)...");
    
    let min_z = 8.0;
    let max_z = 8.5;
    let num_steps = 100;
    
    for i in 0..=num_steps {
        let t = i as f32 / num_steps as f32;
        let z = min_z + (max_z - min_z) * t;
        evaluate_direct_camera(&mut file, z);
    }
}
