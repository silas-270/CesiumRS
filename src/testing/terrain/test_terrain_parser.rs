use cesium_engine::globe::terrain_parser::parse_quantized_mesh;
use std::io::Write;

fn encode_zigzag(delta: i16) -> u16 {
    ((delta << 1) ^ (delta >> 15)) as u16
}

trait WriteLeExt: Write {
    fn write_f64_le(&mut self, val: f64) -> std::io::Result<()> {
        self.write_all(&val.to_le_bytes())
    }
    fn write_f32_le(&mut self, val: f32) -> std::io::Result<()> {
        self.write_all(&val.to_le_bytes())
    }
    fn write_u32_le(&mut self, val: u32) -> std::io::Result<()> {
        self.write_all(&val.to_le_bytes())
    }
    fn write_u16_le(&mut self, val: u16) -> std::io::Result<()> {
        self.write_all(&val.to_le_bytes())
    }
}
impl<W: Write> WriteLeExt for W {}

fn create_mock_tile() -> Vec<u8> {
    let mut buffer = Vec::new();

    // 1. Header (88 bytes)
    buffer.write_f64_le(1.0).unwrap(); // center_x
    buffer.write_f64_le(2.0).unwrap(); // center_y
    buffer.write_f64_le(3.0).unwrap(); // center_z
    buffer.write_f32_le(0.0).unwrap(); // min_height
    buffer.write_f32_le(100.0).unwrap(); // max_height
    buffer.write_f64_le(4.0).unwrap(); // bounding_sphere_center_x
    buffer.write_f64_le(5.0).unwrap(); // bounding_sphere_center_y
    buffer.write_f64_le(6.0).unwrap(); // bounding_sphere_center_z
    buffer.write_f64_le(6378137.0).unwrap(); // bounding_sphere_radius
    buffer.write_f64_le(7.0).unwrap(); // horizon_occlusion_point_x
    buffer.write_f64_le(8.0).unwrap(); // horizon_occlusion_point_y
    buffer.write_f64_le(9.0).unwrap(); // horizon_occlusion_point_z

    // 2. Vertex Data
    buffer.write_u32_le(3).unwrap(); // vertexCount = 3

    let u_vals: [u16; 3] = [0, 16384, 32767];
    let mut current = 0u16;
    for &val in &u_vals {
        let delta = val.wrapping_sub(current) as i16;
        buffer.write_u16_le(encode_zigzag(delta)).unwrap();
        current = val;
    }

    let v_vals: [u16; 3] = [0, 0, 32767];
    let mut current = 0u16;
    for &val in &v_vals {
        let delta = val.wrapping_sub(current) as i16;
        buffer.write_u16_le(encode_zigzag(delta)).unwrap();
        current = val;
    }

    let h_vals: [u16; 3] = [10, 20, 30];
    let mut current = 0u16;
    for &val in &h_vals {
        let delta = val.wrapping_sub(current) as i16;
        buffer.write_u16_le(encode_zigzag(delta)).unwrap();
        current = val;
    }

    // Alignment check: 88 + 4 + 3*6 = 110 (multiple of 2, no padding needed)

    // 3. Index Data
    buffer.write_u32_le(1).unwrap(); // triangleCount = 1

    let indices: [u16; 3] = [0, 1, 2];
    let mut highest = 0u16;
    for &idx in &indices {
        let code = highest.wrapping_sub(idx);
        buffer.write_u16_le(code).unwrap();
        if code == 0 {
            highest += 1;
        }
    }

    // 4. Edge Indices
    // West
    buffer.write_u32_le(1).unwrap();
    buffer.write_u16_le(0).unwrap();
    // South
    buffer.write_u32_le(2).unwrap();
    buffer.write_u16_le(0).unwrap();
    buffer.write_u16_le(1).unwrap();
    // East
    buffer.write_u32_le(1).unwrap();
    buffer.write_u16_le(2).unwrap();
    // North
    buffer.write_u32_le(2).unwrap();
    buffer.write_u16_le(2).unwrap();
    buffer.write_u16_le(0).unwrap();

    buffer
}

#[test]
fn test_parse_quantized_mesh_valid() {
    let buffer = create_mock_tile();
    let tile = parse_quantized_mesh(&buffer).expect("Failed to parse mock tile");

    // Check Header
    assert_eq!(tile.header.center_x, 1.0);
    assert_eq!(tile.header.min_height, 0.0);
    assert_eq!(tile.header.bounding_sphere_radius, 6378137.0);

    // Check Vertices
    assert_eq!(tile.vertices.u.len(), 3);
    assert_eq!(tile.vertices.u, vec![0, 16384, 32767]);
    assert_eq!(tile.vertices.v, vec![0, 0, 32767]);
    assert_eq!(tile.vertices.height, vec![10, 20, 30]);

    // Check Indices
    assert_eq!(tile.indices.len(), 3);
    assert_eq!(tile.indices, vec![0, 1, 2]);

    // Check Edge Indices
    assert_eq!(tile.edge_indices.west, vec![0]);
    assert_eq!(tile.edge_indices.south, vec![0, 1]);
    assert_eq!(tile.edge_indices.east, vec![2]);
    assert_eq!(tile.edge_indices.north, vec![2, 0]);
}
