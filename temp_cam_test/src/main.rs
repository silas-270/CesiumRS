use glam::Vec3;

fn main() {
    let fra_lon = 8.5706f32.to_radians();
    let fra_lat = 50.0333f32.to_radians();
    let r = 6.371;

    let fra_pos = Vec3::new(
        r * fra_lat.cos() * fra_lon.cos(),
        r * fra_lat.cos() * fra_lon.sin(),
        r * fra_lat.sin(),
    );

    let p_cam_base = Vec3::new(7.415, 14.539, 1.184);

    let fra_up = fra_pos.normalize();
    let fra_east = Vec3::new(0.0, 0.0, 1.0).cross(fra_up).normalize();
    let fra_north = fra_up.cross(fra_east).normalize();

    let dot_east = p_cam_base.dot(fra_east);
    let dot_north = p_cam_base.dot(fra_north);
    let dot_up = p_cam_base.dot(fra_up);

    println!("East: {}, North: {}, Up: {}", dot_east, dot_north, dot_up);
}
