use crate::engine::camera::camera::Camera;
use crate::engine::globe::quadtree::QuadtreeManager;
use glam::{Quat, Vec3};

#[test]
fn test_obb_debug() {
    let mut cam = Camera::new(Vec3::new(0.0, 0.0, 9.0), Vec3::ZERO);
    cam.set_local_transform(Vec3::new(0.0, 0.0, 9.0), Quat::IDENTITY);

    let aspect_ratio = 16.0 / 9.0;
    let view_proj = cam.get_projection_matrix(aspect_ratio) * cam.get_view_matrix();
    let camera_pos = cam.global_transform().0;

    let mut quadtree = QuadtreeManager::new();
    quadtree.update(camera_pos, view_proj);

    // Check exactly what happens to Z=3, X=1, Y=3 (which is directly in front of the camera)
    println!("Checking Z=3, X=1, Y=3");

    // Instead of manual check, let's look at the manager's output for Z=3, 4, 5 tiles in that region
    let mut quadtree = QuadtreeManager::new();
    quadtree.update(camera_pos, view_proj);
    let tiles = quadtree.get_visible_tiles();

    println!("Tiles in the center region (Lon -135 to -45):");
    for (id, _, _) in &tiles {
        if id.z >= 3 && id.x >= (1 << (id.z - 3)) * 1 && id.x <= (1 << (id.z - 3)) * 2 {
            println!("  - Tile Z: {}, X: {}, Y: {}", id.z, id.x, id.y);
        }
    }
}
