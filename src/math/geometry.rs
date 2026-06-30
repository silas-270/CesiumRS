#[repr(C)]
#[derive(Copy, Clone, Debug, bytemuck::Pod, bytemuck::Zeroable)]
pub struct Vertex {
    pub position: [f32; 3],
}

impl Vertex {
    pub fn desc() -> wgpu::VertexBufferLayout<'static> {
        wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<Vertex>() as wgpu::BufferAddress,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &[wgpu::VertexAttribute {
                offset: 0,
                shader_location: 0,
                format: wgpu::VertexFormat::Float32x3,
            }],
        }
    }
}

pub struct Ellipsoid {
    pub vertices: Vec<Vertex>,
    pub indices: Vec<u16>,
}

impl Ellipsoid {
    pub fn generate(lat_segments: u32, lon_segments: u32) -> Self {
        // WGS84 parameters downscaled by 1:1,000,000
        let a = 6.378137_f32; // Equatorial radius
        let b = 6.3567523142_f32; // Polar radius
        
        let mut vertices = Vec::new();
        let mut indices = Vec::new();

        // Generate vertices
        for y in 0..=lat_segments {
            // Latitude from -pi/2 to pi/2
            let v = y as f32 / lat_segments as f32;
            let phi = (v - 0.5) * std::f32::consts::PI;

            for x in 0..=lon_segments {
                // Longitude from -pi to pi
                let u = x as f32 / lon_segments as f32;
                let theta = u * 2.0 * std::f32::consts::PI;

                let x_pos = a * phi.cos() * theta.cos();
                let y_pos = b * phi.sin();
                let z_pos = -a * phi.cos() * theta.sin();

                vertices.push(Vertex {
                    position: [x_pos, y_pos, z_pos],
                });
            }
        }

        // Generate indices
        for y in 0..lat_segments {
            for x in 0..lon_segments {
                let current = (y * (lon_segments + 1)) + x;
                let next = current + lon_segments + 1;

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
