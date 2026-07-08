use cesium_engine::camera::camera::Camera;
use cesium_engine::globe::quadtree::QuadtreeManager;
use glam::{Quat, Vec3};

#[test]
fn test_20_tiles() {
    let mut cam = Camera::new(Vec3::new(0.0, 0.0, 9.0), Vec3::ZERO);
    cam.set_local_transform(Vec3::new(0.0, 0.0, 9.0), Quat::IDENTITY);

    // Screenshot aspect ratio: 1920 / 1000 = 1.92
    let aspect_ratio = 1.92;
    let frustum_planes = cam.calculate_frustum_planes(aspect_ratio);
    let (global_pos_dvec, _) = cam.global_transform_f64();
    let global_pos_f32 = glam::Vec3::new(global_pos_dvec.x as f32, global_pos_dvec.y as f32, global_pos_dvec.z as f32);
    
    let mut quadtree = QuadtreeManager::new();
    quadtree.update(global_pos_f32, frustum_planes);
    let tiles = quadtree.get_visible_tiles();

    println!("Total tiles: {}", tiles.len());
    for (id, _, _) in tiles.iter() {
        println!("  - Tile Z: {}, X: {}, Y: {}", id.z, id.x, id.y);
    }
}
