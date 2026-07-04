use crate::engine::camera::camera::Camera;
use crate::engine::globe::quadtree::QuadtreeManager;
use glam::{Quat, Vec3};

#[test]
fn test_20_tiles() {
    let mut cam = Camera::new(Vec3::new(0.0, 0.0, 9.0), Vec3::ZERO);
    cam.set_local_transform(Vec3::new(0.0, 0.0, 9.0), Quat::IDENTITY);

    // Screenshot aspect ratio: 1920 / 1000 = 1.92
    let aspect_ratio = 1.92;
    let view_proj = cam.get_projection_matrix(aspect_ratio) * cam.get_view_matrix();
    let camera_pos = cam.global_transform().0;

    let mut quadtree = QuadtreeManager::new();
    quadtree.update(camera_pos, view_proj);
    let tiles = quadtree.get_visible_tiles();

    println!("Total tiles: {}", tiles.len());
    for (id, _, _) in tiles.iter() {
        println!("  - Tile Z: {}, X: {}, Y: {}", id.z, id.x, id.y);
    }
}
