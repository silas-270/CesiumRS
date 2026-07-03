use std::io::{Cursor, Read};

#[derive(Debug, Clone)]
pub struct QuantizedMeshHeader {
    pub center_x: f64,
    pub center_y: f64,
    pub center_z: f64,
    pub min_height: f32,
    pub max_height: f32,
    pub bounding_sphere_center_x: f64,
    pub bounding_sphere_center_y: f64,
    pub bounding_sphere_center_z: f64,
    pub bounding_sphere_radius: f64,
    pub horizon_occlusion_point_x: f64,
    pub horizon_occlusion_point_y: f64,
    pub horizon_occlusion_point_z: f64,
}

#[derive(Debug, Clone)]
pub struct QuantizedMeshTile {
    pub header: QuantizedMeshHeader,
    pub vertices: QuantizedVertices,
    pub indices: Vec<u32>, // Normalize to u32 for convenience
    pub edge_indices: EdgeIndices,
}

#[derive(Debug, Clone)]
pub struct QuantizedVertices {
    pub u: Vec<u16>,
    pub v: Vec<u16>,
    pub height: Vec<u16>,
}

#[derive(Debug, Clone)]
pub struct EdgeIndices {
    pub west: Vec<u32>,
    pub south: Vec<u32>,
    pub east: Vec<u32>,
    pub north: Vec<u32>,
}

#[derive(Debug)]
pub enum ParseError {
    IoError(std::io::Error),
    InvalidTileSize,
}

impl From<std::io::Error> for ParseError {
    fn from(error: std::io::Error) -> Self {
        ParseError::IoError(error)
    }
}

fn zig_zag_decode(value: u16) -> i16 {
    ((value >> 1) as i16) ^ (-((value & 1) as i16))
}

fn decode_zigzag_delta(buffer: &mut [u16]) {
    let mut current: u16 = 0;
    for val in buffer.iter_mut() {
        let decoded = zig_zag_decode(*val);
        current = current.wrapping_add(decoded as u16);
        *val = current;
    }
}

trait ReadLeExt: Read {
    fn read_f64_le(&mut self) -> std::io::Result<f64> {
        let mut buf = [0u8; 8];
        self.read_exact(&mut buf)?;
        Ok(f64::from_le_bytes(buf))
    }
    fn read_f32_le(&mut self) -> std::io::Result<f32> {
        let mut buf = [0u8; 4];
        self.read_exact(&mut buf)?;
        Ok(f32::from_le_bytes(buf))
    }
    fn read_u32_le(&mut self) -> std::io::Result<u32> {
        let mut buf = [0u8; 4];
        self.read_exact(&mut buf)?;
        Ok(u32::from_le_bytes(buf))
    }
    fn read_u16_le(&mut self) -> std::io::Result<u16> {
        let mut buf = [0u8; 2];
        self.read_exact(&mut buf)?;
        Ok(u16::from_le_bytes(buf))
    }
}
impl<R: Read> ReadLeExt for R {}

pub fn parse_quantized_mesh(buffer: &[u8]) -> Result<QuantizedMeshTile, ParseError> {
    let mut cursor = Cursor::new(buffer);

    let header = QuantizedMeshHeader {
        center_x: cursor.read_f64_le()?,
        center_y: cursor.read_f64_le()?,
        center_z: cursor.read_f64_le()?,
        min_height: cursor.read_f32_le()?,
        max_height: cursor.read_f32_le()?,
        bounding_sphere_center_x: cursor.read_f64_le()?,
        bounding_sphere_center_y: cursor.read_f64_le()?,
        bounding_sphere_center_z: cursor.read_f64_le()?,
        bounding_sphere_radius: cursor.read_f64_le()?,
        horizon_occlusion_point_x: cursor.read_f64_le()?,
        horizon_occlusion_point_y: cursor.read_f64_le()?,
        horizon_occlusion_point_z: cursor.read_f64_le()?,
    };

    let vertex_count = cursor.read_u32_le()? as usize;

    let mut u = vec![0u16; vertex_count];
    for i in 0..vertex_count { u[i] = cursor.read_u16_le()?; }

    let mut v = vec![0u16; vertex_count];
    for i in 0..vertex_count { v[i] = cursor.read_u16_le()?; }

    let mut height = vec![0u16; vertex_count];
    for i in 0..vertex_count { height[i] = cursor.read_u16_le()?; }

    decode_zigzag_delta(&mut u);
    decode_zigzag_delta(&mut v);
    decode_zigzag_delta(&mut height);

    let vertices = QuantizedVertices { u, v, height };

    let is_32_bit = vertex_count > 65536;
    let index_size = if is_32_bit { 4 } else { 2 };

    // Align to index_size
    let pos = cursor.position() as u64;
    let remainder = pos % index_size;
    if remainder != 0 {
        cursor.set_position(pos + (index_size - remainder));
    }

    let triangle_count = cursor.read_u32_le()? as usize;
    let mut indices = Vec::with_capacity(triangle_count * 3);

    if is_32_bit {
        let mut highest = 0u32;
        for _ in 0..(triangle_count * 3) {
            let code = cursor.read_u32_le()?;
            let index = highest.wrapping_sub(code);
            if code == 0 {
                highest += 1;
            }
            indices.push(index);
        }
    } else {
        let mut highest = 0u16;
        for _ in 0..(triangle_count * 3) {
            let code = cursor.read_u16_le()?;
            let index = highest.wrapping_sub(code);
            if code == 0 {
                highest += 1;
            }
            indices.push(index as u32);
        }
    }

    // Edge indices
    let west_count = cursor.read_u32_le()? as usize;
    let mut west = vec![0u32; west_count];
    if is_32_bit {
        for i in 0..west_count { west[i] = cursor.read_u32_le()?; }
    } else {
        for i in 0..west_count { west[i] = cursor.read_u16_le()? as u32; }
    }

    let south_count = cursor.read_u32_le()? as usize;
    let mut south = vec![0u32; south_count];
    if is_32_bit {
        for i in 0..south_count { south[i] = cursor.read_u32_le()?; }
    } else {
        for i in 0..south_count { south[i] = cursor.read_u16_le()? as u32; }
    }

    let east_count = cursor.read_u32_le()? as usize;
    let mut east = vec![0u32; east_count];
    if is_32_bit {
        for i in 0..east_count { east[i] = cursor.read_u32_le()?; }
    } else {
        for i in 0..east_count { east[i] = cursor.read_u16_le()? as u32; }
    }

    let north_count = cursor.read_u32_le()? as usize;
    let mut north = vec![0u32; north_count];
    if is_32_bit {
        for i in 0..north_count { north[i] = cursor.read_u32_le()?; }
    } else {
        for i in 0..north_count { north[i] = cursor.read_u16_le()? as u32; }
    }

    let edge_indices = EdgeIndices {
        west,
        south,
        east,
        north,
    };

    Ok(QuantizedMeshTile {
        header,
        vertices,
        indices,
        edge_indices,
    })
}
