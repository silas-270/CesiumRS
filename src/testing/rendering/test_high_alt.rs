use crate::engine::camera::camera::Camera;
use crate::engine::globe::quadtree::QuadtreeManager;
use glam::{Quat, Vec3};

#[test]
fn test_high_alt() {
    // Altitude 2.6219 -> z = 9.0
    let mut cam = Camera::new(Vec3::new(0.0, 0.0, 9.0), Vec3::ZERO);
    cam.set_local_transform(Vec3::new(0.0, 0.0, 9.0), Quat::IDENTITY);

    let aspect_ratio = 16.0 / 9.0;
    let frustum_planes = cam.calculate_frustum_planes(aspect_ratio);
    let (global_pos_dvec, _) = cam.global_transform_f64();
    let global_pos_f32 = glam::Vec3::new(global_pos_dvec.x as f32, global_pos_dvec.y as f32, global_pos_dvec.z as f32);
    
    let mut quadtree = QuadtreeManager::new();
    quadtree.update(global_pos_f32, frustum_planes);
    let tiles = quadtree.get_visible_tiles();

    println!("Total tiles at Altitude 20: {}", tiles.len());
    let mut ghost_count = 0;

    for (id, _, _) in tiles.iter() {

        let frustum = crate::engine::globe::quadtree::Frustum::from_planes(frustum_planes);
        let a2 = 6.378137_f32 * 6.378137_f32;
        let b2 = 6.3567524_f32 * 6.3567524_f32;

        let mut any_visible = false;

        let steps = 10;
        let z_pow = (1_u32 << id.z) as f32;
        let lon_min = -180.0 + (id.x as f32) * 360.0 / z_pow;
        let lon_max = -180.0 + ((id.x + 1) as f32) * 360.0 / z_pow;
        let mut lat_max = crate::engine::globe::quadtree::web_mercator_y_to_lat(id.y as f32, id.z);
        let mut lat_min =
            crate::engine::globe::quadtree::web_mercator_y_to_lat((id.y + 1) as f32, id.z);
        if id.y == 0 {
            lat_max = 90.0;
        }
        if id.y == (1_u32 << id.z) - 1 {
            lat_min = -90.0;
        }

        for i in 0..=steps {
            let u = i as f32 / steps as f32;
            let lon = lon_min + u * (lon_max - lon_min);
            for j in 0..=steps {
                let v = j as f32 / steps as f32;
                let lat = lat_min + v * (lat_max - lat_min);
                // get_tile_corner logic inline
                let phi = lat.to_radians();
                let theta = lon.to_radians();
                let x = 6.378137_f32 * phi.cos() * theta.cos();
                let y = 6.3567524_f32 * phi.sin();
                let z = -6.378137_f32 * phi.cos() * theta.sin();
                let p = Vec3::new(x, y, z);

                if frustum.contains_point(p) {
                    let normal = Vec3::new(p.x / a2, p.y / b2, p.z / a2).normalize();
                    if normal.dot(global_pos_f32 - p) > 0.0 {
                        any_visible = true;
                    }
                }
            }
        }
        if !any_visible {
            ghost_count += 1;
            println!("GHOST TILE: Z: {}, X: {}, Y: {}", id.z, id.x, id.y);
        }
    }
    println!("Ghost tiles: {}", ghost_count);
}
