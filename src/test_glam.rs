#[test]
fn test_glam_proj() {
    let p = glam::Mat4::perspective_rh(1.0, 1.0, 0.1, 100.0);
    println!("proj: {:?}", p);
    let v_near = glam::Vec4::new(0.0, 0.0, -0.1, 1.0);
    let v_far = glam::Vec4::new(0.0, 0.0, -100.0, 1.0);
    let c_near = p * v_near;
    let c_far = p * v_far;
    println!("near_z: {}, far_z: {}", c_near.z / c_near.w, c_far.z / c_far.w);
}
