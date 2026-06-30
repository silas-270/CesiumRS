use cesium_rs::math::quadtree::{QuadtreeNode, TileId, Frustum};
use cesium_rs::math::camera::Camera;
use glam::{Vec2, Vec3, Vec4, Quat, Mat4};

fn main() {
    let camera = Camera::new(Vec3::new(0.0, 0.0, 8.3), Vec3::ZERO);
    let aspect_ratio = 16.0 / 9.0;
    let view_proj = camera.get_projection_matrix(aspect_ratio) * camera.get_view_matrix();
    
    let mut quadtree = cesium_rs::math::quadtree::QuadtreeManager::new();
    quadtree.update(camera.global_transform().0, view_proj);
    
    println!("Visible tiles: {}", quadtree.get_visible_tiles().len());
}
