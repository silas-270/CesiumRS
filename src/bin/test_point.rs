use cesium_rs::camera::camera::Camera;
use glam::{Mat4, Quat, Vec3};

fn main() {
    let mut cam = Camera::new(Vec3::new(0.0, 0.0, 9.0), Vec3::ZERO);
    cam.set_local_transform(Vec3::new(0.0, 0.0, 9.0), Quat::IDENTITY);
    let camera_pos = cam.global_transform().0;
    let aspect_ratio = 16.0 / 9.0;
    let view_proj = cam.get_projection_matrix(aspect_ratio) * cam.get_view_matrix();
    let frustum = cesium_rs::globe::quadtree::Frustum::from_matrix(view_proj);

    let mut node =
        cesium_rs::globe::quadtree::QuadtreeNode::new(cesium_rs::globe::quadtree::TileId {
            z: 3,
            x: 3,
            y: 7,
        });

    let obb_pass = frustum.intersects_obb(&node.obb);
    println!("OBB pass for Z=3, X=3, Y=7: {}", obb_pass);

    let mut any_visible = false;
    for p in &node.surface_points {
        if frustum.contains_point(*p) {
            any_visible = true;
            break;
        }
    }
    println!("Surface points visible for Z=3 X=3 Y=7: {}", any_visible);

    if let Some(hcp) = node.horizon_culling_point {
        let a = 6.378137_f32;
        let b = 6.3567523142_f32;
        let cv = Vec3::new(camera_pos.x / a, camera_pos.y / b, camera_pos.z / a);
        let vh_mag_sq = cv.length_squared() - 1.0;
        let vt = hcp - cv;
        let vt_dot_vc = -vt.dot(cv);
        let is_occluded =
            vt_dot_vc > vh_mag_sq && (vt_dot_vc * vt_dot_vc) / vt.length_squared() > vh_mag_sq;
        println!("is_occluded for Z=5 X=7 Y=15: {}", is_occluded);
    }

    let mut any_visible = false;
    for p in &node.surface_points {
        if frustum.contains_point(*p) {
            any_visible = true;
            break;
        }
    }
    println!("Surface points visible for Z=5 X=7 Y=15: {}", any_visible);

    // Simulate fallback
    let mut found = false;
    let steps = 10;
    let z_pow = 32.0;
    let lon_min = -180.0 + 7.0 * 360.0 / 32.0;
    let lon_max = -180.0 + 8.0 * 360.0 / 32.0;
    let lat_max = 0.0;
    let lat_min = -11.17;
    for i in 0..=steps {
        for j in 0..=steps {
            let u = i as f32 / steps as f32;
            let v = j as f32 / steps as f32;
            let lon = lon_min + u * (lon_max - lon_min);
            let lat = lat_min + v * (lat_max - lat_min);
            let p = get_tile_corner(lon, lat, 0.0);
            if frustum.contains_point(p) {
                found = true;
                break;
            }
        }
    }
    println!("Dense fallback visible for Z=5 X=7 Y=15: {}", found);

    let obb_pass = frustum.intersects_obb(&node.obb);
    println!("OBB pass for Z=5, X=7, Y=15: {}", obb_pass);
    println!("OBB center: {:?}", node.obb.center);
    println!("OBB extents: {:?}", node.obb.half_axes);
    for (i, (n, d)) in frustum.planes.iter().enumerate() {
        let r = n.dot(node.obb.half_axes[0]).abs()
            + n.dot(node.obb.half_axes[1]).abs()
            + n.dot(node.obb.half_axes[2]).abs();
        let dist = n.dot(node.obb.center) + d;
        println!(
            "Plane {}: n={:?} d={} dist={} r={} (dist < -r: {})",
            i,
            n,
            d,
            dist,
            r,
            dist < -r
        );
    }
}
