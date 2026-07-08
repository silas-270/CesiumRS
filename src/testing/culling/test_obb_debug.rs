use cesium_engine::camera::camera::Camera;
use cesium_engine::globe::quadtree::QuadtreeManager;
use glam::{Quat, Vec3};

#[test]
fn test_obb_debug() {
    let mut cam = Camera::new(Vec3::new(0.0, 0.0, 9.0), Vec3::ZERO);
    cam.set_local_transform(Vec3::new(0.0, 0.0, 9.0), Quat::IDENTITY);

    let mut quadtree = QuadtreeManager::new();

    let frustum_planes = cam.calculate_frustum_planes(16.0 / 9.0);
    let (global_pos_dvec, _) = cam.global_transform_f64();
    let global_pos_f32 = glam::Vec3::new(
        global_pos_dvec.x as f32,
        global_pos_dvec.y as f32,
        global_pos_dvec.z as f32,
    );
    quadtree.update(global_pos_f32, frustum_planes);

    let tiles1 = quadtree.get_visible_tiles();
    println!("Tiles with OBB 1: {}", tiles1.len());

    cam.set_local_transform(Vec3::new(0.0, 0.0, 8.0), Quat::IDENTITY);
    let frustum_planes2 = cam.calculate_frustum_planes(16.0 / 9.0);
    let (global_pos_dvec2, _) = cam.global_transform_f64();
    let global_pos_f32_2 = glam::Vec3::new(
        global_pos_dvec2.x as f32,
        global_pos_dvec2.y as f32,
        global_pos_dvec2.z as f32,
    );
    quadtree.update(global_pos_f32_2, frustum_planes2);
    let tiles = quadtree.get_visible_tiles();

    println!("Tiles in the center region (Lon -135 to -45):");
    for (id, _, _) in &tiles {
        if id.z >= 3 && id.x >= (1 << (id.z - 3)) * 1 && id.x <= (1 << (id.z - 3)) * 2 {
            println!("  - Tile Z: {}, X: {}, Y: {}", id.z, id.x, id.y);
        }
    }
}
