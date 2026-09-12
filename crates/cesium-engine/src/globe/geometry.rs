use crate::globe::quadtree::TileId;

pub const EARTH_RADIUS_A_F32: f32 = 6.378137;
pub const EARTH_RADIUS_B_F32: f32 = 6.356_752_4;
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
    /// # Invariant I-1 — zero relief
    ///
    /// Every non-skirt vertex below is placed at **altitude exactly 0** on the
    /// ellipsoid, and every skirt vertex at a *negative* altitude (radially
    /// inward). Nothing this function emits is ever above the surface.
    ///
    /// The tile horizon test (`QuadtreeNode::horizon_cull`) is licensed by exactly
    /// that fact: for a point *on* the ellipsoid, occlusion collapses to the single
    /// linear inequality `q·c ≤ 1`, and interior skirt points are occluded whenever
    /// their surface neighbours are. The moment terrain relief is applied here,
    /// that collapse becomes **unsound** and the horizon test must be replaced by
    /// the scaled-space cone test (`docs/culling-math.md` §3.7, Theorem 3.7).
    ///
    /// `testing::culling::test_tile_bounds::test_generated_mesh_has_no_positive_altitude`
    /// asserts this, so whoever turns relief on gets a failing test rather than
    /// silent holes at the limb.
    pub fn generate(id: &TileId, segments: u32) -> Self {
        let mut vertices = Vec::new();
        let mut indices = Vec::new();

        // I-5: the one shared bounds function. The culling rectangle and the drawn
        // rectangle are the same four numbers, pole stretch included.
        let bounds = crate::globe::quadtree::tile_bounds(id);
        let lon_min = bounds.lon_min;
        let lon_max = bounds.lon_max;
        let center_lon = bounds.center_lon();
        let center_lat = bounds.center_lat();
        let center_f64 = lon_lat_to_ecef_f64(center_lon, center_lat);

        // Base skirt height in megameters (approx 500km at z=0, scaled down)
        let skirt_height = 0.5 / 2.0_f32.powi(id.z as i32);

        let grid_size = segments + 3; // +2 for skirts

        for row in 0..grid_size {
            let is_skirt_row = row == 0 || row == grid_size - 1;
            let logical_row = (row.max(1) - 1).min(segments);
            let v = logical_row as f32 / segments as f32;

            // Interior rows come from the same f64 Mercator definition the shared
            // bounds are built from, so `v = 0` and `v = 1` reproduce `lat_max` and
            // `lat_min` bit-for-bit and every interior row is monotonically between
            // them. Mixing an f32 row latitude into an f64 rectangle would let the
            // mesh poke a few tenths of a metre outside the culling rectangle —
            // exactly the sliver invariant I-5 exists to prevent.
            let global_y = id.y as f64 + v as f64;
            let mut lat = crate::globe::quadtree::web_mercator_y_to_lat_f64(global_y, id.z);

            let is_north_pole_cap = id.y == 0 && row == 0;
            let is_south_pole_cap = id.y == (1_u32 << id.z) - 1 && row == grid_size - 1;

            if is_north_pole_cap {
                lat = 90.0;
            } else if is_south_pole_cap {
                lat = -90.0;
            }

            let phi = lat.to_radians();
            let cos_phi = phi.cos();
            let sin_phi = phi.sin();

            for col in 0..grid_size {
                let is_skirt_col = col == 0 || col == grid_size - 1;
                let logical_col = (col.max(1) - 1).min(segments);
                let u = logical_col as f32 / segments as f32;
                // Interpolated in f64 from the shared f64 bounds. `lon_max −
                // lon_min` is exact in f64 (both operands carry ≤ 24 significant
                // bits), so `u = 1` lands on `lon_max` exactly and every vertex
                // longitude is inside `[lon_min, lon_max]`. Doing this lerp in f32
                // could overshoot the rectangle by an ulp — ~1.7 m of ground — and
                // that sliver is an FN at the limb. See I-5.
                let lon = lon_min + (u as f64) * (lon_max - lon_min);

                let is_skirt = is_skirt_row || is_skirt_col;
                let is_pole_cap = is_north_pole_cap || is_south_pole_cap;
                let alt = if is_skirt && !is_pole_cap {
                    -skirt_height
                } else {
                    0.0
                };

                let theta = lon.to_radians();
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

                let normal = [
                    normal_f64[0] as f32,
                    normal_f64[1] as f32,
                    normal_f64[2] as f32,
                ];

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

        Self {
            vertices,
            indices,
            center_f64,
        }
    }
}
