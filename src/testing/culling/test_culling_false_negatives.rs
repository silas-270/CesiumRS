use glam::{DMat4, DVec3, Vec3};
use cesium_engine::globe::quadtree::{QuadtreeManager, TileId};
use cesium_engine::camera::camera::Camera;

/// Checks if a tile is marked as visible by the quadtree.
fn is_tile_visible(visible_tiles: &[(TileId, Vec3, f32)], target_id: TileId) -> bool {
    visible_tiles.iter().any(|(id, _, _)| {
        let mut curr = *id;
        if curr == target_id { return true; }
        while let Some(p) = curr.parent() {
            if p == target_id { return true; }
            curr = p;
        }
        false
    })
}

#[test]
pub fn run_test() {
    println!("Running false-negative culling test...");
    
    let test_cases = vec![
        (DVec3::new(6378137.0 + 1000.0, 0.0, 0.0), -0.05, 0.0),
        (DVec3::new(4510000.0, 4510000.0, 10000.0), -0.5, 3.14159),
        (DVec3::new(0.0, 0.0, 6356752.0 + 500.0), -0.1, 0.0),
    ];

    let mut false_negatives_found = 0;

    for (i, (pos, pitch, yaw)) in test_cases.into_iter().enumerate() {
        let pos_v3 = Vec3::new(pos.x as f32, pos.y as f32, pos.z as f32);
        let mut camera = Camera::new(pos_v3, Vec3::ZERO);
        camera.local_pos = pos_v3;
        camera.local_ori = glam::Quat::from_euler(glam::EulerRot::YXZ, yaw as f32, pitch as f32, 0.0);

        let aspect_ratio = 16.0 / 9.0;
        let frustum = camera.calculate_frustum_planes(aspect_ratio);
        let view_proj = camera.get_projection_matrix_f64(aspect_ratio as f64) * camera.get_view_matrix_f64();
        
        let mut quadtree = QuadtreeManager::new();
        quadtree.lod_factor = 2.0;
        let pos_f32 = Vec3::new(pos.x as f32, pos.y as f32, pos.z as f32);
        quadtree.update(pos_f32, frustum);
        let visible_tiles = quadtree.get_visible_tiles();

        let a = 6378137.0;
        let b = 6356752.3142;

        for lat_deg in (-85..=85).step_by(5) {
            for lon_deg in (-180..180).step_by(5) {
                let lat = (lat_deg as f64).to_radians();
                let lon = (lon_deg as f64).to_radians();
                
                let n_x = lat.cos() * lon.cos();
                let n_y = lat.cos() * lon.sin();
                let n_z = lat.sin();
                let point = DVec3::new(a * n_x, a * n_y, b * n_z);
                let normal = DVec3::new(n_x, n_y, n_z);
                
                let cam_to_point = point - pos;
                if normal.dot(cam_to_point) > 0.0 { continue; }
                
                let clip_space = view_proj * point.extend(1.0);
                if clip_space.w <= 0.0 { continue; }
                let ndc = clip_space.truncate() / clip_space.w;
                if ndc.x >= -1.0 && ndc.x <= 1.0 && ndc.y >= -1.0 && ndc.y <= 1.0 && ndc.z >= 0.0 && ndc.z <= 1.0 {
                    let z = 4;
                    let n = 1_u32 << z;
                    let x = ((lon_deg as f32 + 180.0) / 360.0 * (n as f32)) as u32;
                    let lat_rad = (lat_deg as f32).to_radians();
                    let y_float = (1.0 - (lat_rad.tan() + 1.0 / lat_rad.cos()).ln() / std::f32::consts::PI) / 2.0 * (n as f32);
                    let mut y = y_float as u32;
                    if y >= n { y = n - 1; }
                    let id = TileId { z, x, y };
                    
                    if !is_tile_visible(&visible_tiles, id) {
                        println!("FALSE NEGATIVE DETECTED in Case {}!", i);
                        println!("Point Lat: {}, Lon: {} is ON SCREEN and front-facing.", lat_deg, lon_deg);
                        println!("But Tile {:?} is NOT in visible_tiles!", id);
                        false_negatives_found += 1;
                        break;
                    }
                }
            }
        }
    }

    if false_negatives_found > 0 {
        panic!("Found {} false negatives in culling logic!", false_negatives_found);
    } else {
        println!("No false negatives detected.");
    }
}
