use crate::engine::camera::camera::Camera;
use crate::engine::globe::quadtree::QuadtreeManager;
use glam::{Quat, Vec3};

#[test]
fn test_all_altitudes() {
    let mut current_alt = 20.0;
    let mut all_clear = true;

    while current_alt >= 0.01 {
        let z = current_alt + 6.378137;
        let mut cam = Camera::new(Vec3::new(0.0, 0.0, z), Vec3::ZERO);
        cam.set_local_transform(Vec3::new(0.0, 0.0, z), Quat::IDENTITY);

        let aspect_ratio = 16.0 / 9.0;
        let view_proj = cam.get_projection_matrix(aspect_ratio) * cam.get_view_matrix();
        let camera_pos = cam.global_transform().0;

        let mut quadtree = QuadtreeManager::new();
        quadtree.update(camera_pos, view_proj);
        let tiles = quadtree.get_visible_tiles();

        let mut ghost_count = 0;

        for (id, _, _) in tiles.iter() {
            let frustum = crate::engine::globe::quadtree::Frustum::from_matrix(view_proj);
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

                    let phi = lat.to_radians();
                    let theta = lon.to_radians();
                    let x = 6.378137_f32 * phi.cos() * theta.cos();
                    let y = 6.3567524_f32 * phi.sin();
                    let z = -6.378137_f32 * phi.cos() * theta.sin();
                    let p = Vec3::new(x, y, z);

                    if frustum.contains_point(p) {
                        let normal = Vec3::new(p.x / a2, p.y / b2, p.z / a2).normalize();
                        if normal.dot(camera_pos - p) > 0.0 {
                            any_visible = true;
                        }
                    }
                }
            }
            if !any_visible {
                ghost_count += 1;
            }
        }

        if ghost_count > 0 {
            println!(
                "FAILED at Altitude {:.2}: {} ghost tiles (Total tiles: {})",
                current_alt,
                ghost_count,
                tiles.len()
            );
            all_clear = false;
        }

        current_alt -= 0.1;
    }

    if all_clear {
        println!("SUCCESS! 0 ghost tiles across all tested altitudes (20.0 to 0.01).");
    }
}
