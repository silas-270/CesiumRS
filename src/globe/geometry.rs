use crate::globe::quadtree::TileId;
use glam::Vec3;

#[repr(C)]
#[derive(Copy, Clone, Debug, bytemuck::Pod, bytemuck::Zeroable)]
pub struct Vertex {
    pub position: [f32; 3],
    pub normal: [f32; 3],
    pub color: [f32; 4],
    pub uv: [f32; 2],
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
                    offset: 12,
                    shader_location: 1,
                    format: wgpu::VertexFormat::Float32x3,
                },
                wgpu::VertexAttribute {
                    offset: 24,
                    shader_location: 2,
                    format: wgpu::VertexFormat::Float32x4,
                },
                wgpu::VertexAttribute {
                    offset: 40,
                    shader_location: 3,
                    format: wgpu::VertexFormat::Float32x2,
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

        let z_pow = (1_u32 << id.z) as f32;

        let lon_min = -180.0 + (id.x as f32) * 360.0 / z_pow;
        let lon_max = -180.0 + ((id.x + 1) as f32) * 360.0 / z_pow;

        // Base skirt height in megameters (approx 500km at z=0, scaled down)
        let skirt_height = 0.5 / 2.0_f32.powi(id.z as i32);

        let grid_size = segments + 3; // +2 for skirts

        for row in 0..grid_size {
            let is_skirt_row = row == 0 || row == grid_size - 1;
            let logical_row = (row.max(1) - 1).min(segments);
            let v = logical_row as f32 / segments as f32;

            let global_y = id.y as f32 + v;
            let mut lat = crate::globe::quadtree::web_mercator_y_to_lat(global_y, id.z);

            let is_north_pole_cap = id.y == 0 && row == 0;
            let is_south_pole_cap = id.y == (1_u32 << id.z) - 1 && row == grid_size - 1;

            if is_north_pole_cap {
                lat = 90.0;
            } else if is_south_pole_cap {
                lat = -90.0;
            }

            for col in 0..grid_size {
                let is_skirt_col = col == 0 || col == grid_size - 1;
                let logical_col = (col.max(1) - 1).min(segments);
                let u = logical_col as f32 / segments as f32;
                let lon = lon_min + u * (lon_max - lon_min);

                let is_skirt = is_skirt_row || is_skirt_col;
                let is_pole_cap = is_north_pole_cap || is_south_pole_cap;
                let alt = if is_skirt && !is_pole_cap {
                    -skirt_height
                } else {
                    0.0
                };

                let surface_pos = lon_lat_to_ecef(lon, lat);
                let normal = surface_pos.normalize();

                let pos = if alt == 0.0 {
                    surface_pos
                } else {
                    surface_pos + normal * alt
                };

                vertices.push(Vertex {
                    position: pos.into(),
                    normal: normal.into(),
                    color: [1.0, 1.0, 1.0, 1.0],
                    uv: [u, v],
                });
            }
        }

        for row in 0..(grid_size - 1) {
            for col in 0..(grid_size - 1) {
                let current = (row * grid_size) + col;
                let next = current + grid_size;

                indices.push(current as u16);
                indices.push(next as u16);
                indices.push((current + 1) as u16);

                indices.push((current + 1) as u16);
                indices.push(next as u16);
                indices.push((next + 1) as u16);
            }
        }

        Self { vertices, indices }
    }
}
