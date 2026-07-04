use crate::engine::globe::quadtree::TileId;
use glam::Vec3;

pub const EARTH_RADIUS_A_F32: f32 = 6.378137;
pub const EARTH_RADIUS_B_F32: f32 = 6.3567523142;
pub const EARTH_RADIUS_A_F64: f64 = 6.378137;
pub const EARTH_RADIUS_B_F64: f64 = 6.3567523142;
const INV_A2_F64: f64 = 1.0 / (EARTH_RADIUS_A_F64 * EARTH_RADIUS_A_F64);
const INV_B2_F64: f64 = 1.0 / (EARTH_RADIUS_B_F64 * EARTH_RADIUS_B_F64);

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
    let phi = lat_deg.to_radians();
    let theta = lon_deg.to_radians();

    let x = EARTH_RADIUS_A_F32 * phi.cos() * theta.cos();
    let y = EARTH_RADIUS_B_F32 * phi.sin();
    let z = -EARTH_RADIUS_A_F32 * phi.cos() * theta.sin(); // -Z to match Right-Handed +Y Up coords

    Vec3::new(x, y, z)
}

pub fn lon_lat_to_ecef_f64(lon_deg: f64, lat_deg: f64) -> [f64; 3] {
    let phi = lat_deg.to_radians();
    let theta = lon_deg.to_radians();

    let x = EARTH_RADIUS_A_F64 * phi.cos() * theta.cos();
    let y = EARTH_RADIUS_B_F64 * phi.sin();
    let z = -EARTH_RADIUS_A_F64 * phi.cos() * theta.sin();

    [x, y, z]
}

pub fn lon_lat_alt_to_ecef_f64(lon_deg: f64, lat_deg: f64, alt_meters: f64) -> [f64; 3] {
    let surface_pos = lon_lat_to_ecef_f64(lon_deg, lat_deg);
    
    if alt_meters == 0.0 {
        return surface_pos;
    }
    
    let nx = surface_pos[0] * INV_A2_F64;
    let ny = surface_pos[1] * INV_B2_F64;
    let nz = surface_pos[2] * INV_A2_F64;
    let len = (nx * nx + ny * ny + nz * nz).sqrt();
    
    let normal = [nx / len, ny / len, nz / len];
    let alt_megameters = alt_meters / 1_000_000.0;
    
    [
        surface_pos[0] + normal[0] * alt_megameters,
        surface_pos[1] + normal[1] * alt_megameters,
        surface_pos[2] + normal[2] * alt_megameters,
    ]
}

pub struct TileMesh {

    pub vertices: Vec<Vertex>,
    pub indices: Vec<u16>,
    pub center_f64: [f64; 3],
}

impl TileMesh {
    pub fn generate(id: &TileId, segments: u32) -> Self {
        let mut vertices = Vec::new();
        let mut indices = Vec::new();

        let z_pow = (1_u32 << id.z) as f32;

        let lon_min = -180.0 + (id.x as f32) * 360.0 / z_pow;
        let lon_max = -180.0 + ((id.x + 1) as f32) * 360.0 / z_pow;

        let mut center_lat_max = crate::engine::globe::quadtree::web_mercator_y_to_lat(id.y as f32, id.z) as f64;
        let mut center_lat_min = crate::engine::globe::quadtree::web_mercator_y_to_lat((id.y + 1) as f32, id.z) as f64;
        if id.y == 0 {
            center_lat_max = 90.0;
        }
        if id.y == (1_u32 << id.z) - 1 {
            center_lat_min = -90.0;
        }
        let center_lon = ((lon_min + lon_max) * 0.5) as f64;
        let center_lat = (center_lat_min + center_lat_max) * 0.5;
        let center_f64 = lon_lat_to_ecef_f64(center_lon, center_lat);

        // Base skirt height in megameters (approx 500km at z=0, scaled down)
        let skirt_height = 0.5 / 2.0_f32.powi(id.z as i32);

        let grid_size = segments + 3; // +2 for skirts

        for row in 0..grid_size {
            let is_skirt_row = row == 0 || row == grid_size - 1;
            let logical_row = (row.max(1) - 1).min(segments);
            let v = logical_row as f32 / segments as f32;

            let global_y = id.y as f32 + v;
            let mut lat = crate::engine::globe::quadtree::web_mercator_y_to_lat(global_y, id.z);

            let is_north_pole_cap = id.y == 0 && row == 0;
            let is_south_pole_cap = id.y == (1_u32 << id.z) - 1 && row == grid_size - 1;

            if is_north_pole_cap {
                lat = 90.0;
            } else if is_south_pole_cap {
                lat = -90.0;
            }

            let phi = (lat as f64).to_radians();
            let cos_phi = phi.cos();
            let sin_phi = phi.sin();

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

                let theta = (lon as f64).to_radians();
                let cos_theta = theta.cos();
                let sin_theta = theta.sin();

                let x = EARTH_RADIUS_A_F64 * cos_phi * cos_theta;
                let y = EARTH_RADIUS_B_F64 * sin_phi;
                let z = -EARTH_RADIUS_A_F64 * cos_phi * sin_theta;

                let surface_pos_f64 = [x, y, z];
                
                // Normal based on WGS84 ellipsoid
                let normal_f64 = {
                    let nx = x * INV_A2_F64;
                    let ny = y * INV_B2_F64;
                    let nz = z * INV_A2_F64;
                    let len = (nx * nx + ny * ny + nz * nz).sqrt();
                    [nx / len, ny / len, nz / len]
                };

                let alt_f64 = alt as f64;
                let pos_f64 = if alt_f64 == 0.0 {
                    surface_pos_f64
                } else {
                    [
                        surface_pos_f64[0] + normal_f64[0] * alt_f64,
                        surface_pos_f64[1] + normal_f64[1] * alt_f64,
                        surface_pos_f64[2] + normal_f64[2] * alt_f64,
                    ]
                };

                let relative_pos = [
                    (pos_f64[0] - center_f64[0]) as f32,
                    (pos_f64[1] - center_f64[1]) as f32,
                    (pos_f64[2] - center_f64[2]) as f32,
                ];

                let normal = [normal_f64[0] as f32, normal_f64[1] as f32, normal_f64[2] as f32];

                vertices.push(Vertex {
                    position: relative_pos,
                    normal,
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

        Self { vertices, indices, center_f64 }
    }
}
