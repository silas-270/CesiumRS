//! Ad-hoc single-tile diagnostic: prints why one node is (or is not) culled.
//!
//! Not a gate — it asserts nothing. It exists so a specific tile can be walked
//! through the three culling stages by hand.

use cesium_engine::camera::camera::Camera;
use cesium_engine::globe::quadtree::{Frustum, HorizonCamera, QuadtreeNode, TileId};
use glam::{Quat, Vec3};

#[test]
fn test_point() {
    let mut cam = Camera::new(Vec3::new(0.0, 0.0, 9.0), Vec3::ZERO);
    cam.set_local_transform(Vec3::new(0.0, 0.0, 9.0), Quat::IDENTITY);
    let (eye, _) = cam.global_transform_f64();
    let aspect_ratio = 16.0 / 9.0;
    let frustum = Frustum::new(cam.calculate_frustum_planes(aspect_ratio), eye);
    let horizon = HorizonCamera::new(eye);

    for id in [TileId { z: 3, x: 3, y: 7 }, TileId { z: 5, x: 7, y: 15 }] {
        let node = QuadtreeNode::new(id);

        println!("-- tile z={} x={} y={} --", id.z, id.x, id.y);
        println!(
            "  horizon: max q.c = {:.9} (cull at <= {:.9}) -> occluded={}",
            node.patch.max_dot(&horizon),
            1.0 - horizon.eps,
            node.patch.is_occluded(&horizon)
        );

        let delta = frustum.relative(node.obb.center);
        println!("  OBB pass: {}", frustum.intersects_obb(&node.obb));
        println!("  OBB center (f64): {:?}", node.obb.center);
        println!("  OBB half-axes:    {:?}", node.obb.half_axes);
        println!("  camera-relative delta: {delta:?}");

        for (i, n) in frustum.normals.iter().enumerate() {
            let r = n.dot(node.obb.half_axes[0]).abs()
                + n.dot(node.obb.half_axes[1]).abs()
                + n.dot(node.obb.half_axes[2]).abs();
            let s = n.dot(delta);
            println!(
                "  plane {} ({}): n={:?} s={} r={} (s + r < 0: {})",
                i,
                ["Left", "Right", "Bottom", "Top"][i],
                n,
                s,
                r,
                s + r < 0.0
            );
        }
    }
}
