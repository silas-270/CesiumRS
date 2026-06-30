use glam::Vec3;
use crate::math::quadtree::TileId;

#[repr(C)]
#[derive(Copy, Clone, Debug, bytemuck::Pod, bytemuck::Zeroable)]
pub struct Vertex {
    pub position: [f32; 3],
    pub normal: [f32; 3],
    pub color: [f32; 4],
}

impl Vertex {
    pub fn desc() -> wgpu::VertexBufferLayout<'static> {
        wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<Vertex>() as wgpu::BufferAddress,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &[
                wgpu::VertexAttribute {
                    offset: 0,
                    shader_location: 0,
                    format: wgpu::VertexFormat::Float32x3,
                },
                wgpu::VertexAttribute {
                    offset: std::mem::size_of::<[f32; 3]>() as wgpu::BufferAddress,
                    shader_location: 1,
                    format: wgpu::VertexFormat::Float32x3,
                },
                wgpu::VertexAttribute {
                    offset: (std::mem::size_of::<[f32; 3]>() * 2) as wgpu::BufferAddress,
                    shader_location: 2,
                    format: wgpu::VertexFormat::Float32x4,
                },
            ],
        }
    }
}

pub fn lon_lat_to_ecef(lon_deg: f32, lat_deg: f32) -> Vec3 {
    let a = 6.378137_f32; // Equatorial radius
    let b = 6.3567523142_f32; // Polar radius
    
    let phi = lat_deg.to_radians();
    let theta = lon_deg.to_radians();

    let x = a * phi.cos() * theta.cos();
    let y = b * phi.sin();
    let z = -a * phi.cos() * theta.sin(); // -Z to match Right-Handed +Y Up coords

    Vec3::new(x, y, z)
}

pub struct TileMesh {
    pub vertices: Vec<Vertex>,
    pub indices: Vec<u16>,
}

impl TileMesh {
    pub fn generate(id: &TileId, segments: u32) -> Self {
        let mut vertices = Vec::new();
        let mut indices = Vec::new();

        let z_pow_x = (1_u32 << (id.z + 1)) as f32; // 2^(z+1) for longitude
        let z_pow_y = (1_u32 << id.z) as f32;       // 2^z for latitude

        // Longitude spans -180 to 180 over 2^(z+1) tiles
        let lon_min = -180.0 + (id.x as f32) * 360.0 / z_pow_x;
        let lon_max = -180.0 + ((id.x + 1) as f32) * 360.0 / z_pow_x;

        // Latitude spans 90 to -90 over 2^z tiles (Y=0 is North)
        let lat_max = 90.0 - (id.y as f32) * 180.0 / z_pow_y;
        let lat_min = 90.0 - ((id.y + 1) as f32) * 180.0 / z_pow_y;

        // Deterministic pseudo-random color based on TileId
        // XORing and wrapping multiplication for simple hashing
        let hash1 = (id.z as u32).wrapping_mul(73856093) ^ id.x.wrapping_mul(19349663) ^ id.y.wrapping_mul(83492791);
        let hash2 = hash1.wrapping_mul(83492791);
        let hash3 = hash2.wrapping_mul(19349663);

        let r = ((hash1 >> 16) & 0xFF) as f32 / 255.0;
        let g = ((hash2 >> 16) & 0xFF) as f32 / 255.0;
        let b = ((hash3 >> 16) & 0xFF) as f32 / 255.0;
        
        let color = [r, g, b, 1.0];

        // Generate vertices
        for y_idx in 0..=segments {
            let v = y_idx as f32 / segments as f32;
            let lat = lat_max - v * (lat_max - lat_min); // Map v=0 to lat_max (North), v=1 to lat_min (South)

            for x_idx in 0..=segments {
                let u = x_idx as f32 / segments as f32;
                let lon = lon_min + u * (lon_max - lon_min);

                let pos = lon_lat_to_ecef(lon, lat);
                let normal = pos.normalize(); // Approximate normal for ellipsoid

                vertices.push(Vertex {
                    position: pos.into(),
                    normal: normal.into(),
                    color,
                });
            }
        }

        // Generate indices
        for y_idx in 0..segments {
            for x_idx in 0..segments {
                let current = (y_idx * (segments + 1)) + x_idx;
                let next = current + segments + 1;

                // Triangle 1
                indices.push(current as u16);
                indices.push(next as u16);
                indices.push((current + 1) as u16);

                // Triangle 2
                indices.push((current + 1) as u16);
                indices.push(next as u16);
                indices.push((next + 1) as u16);
            }
        }

        Self { vertices, indices }
    }
}
