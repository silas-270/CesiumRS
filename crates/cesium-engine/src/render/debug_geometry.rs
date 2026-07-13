#[cfg(feature = "debug_panel")]
use glam::{Mat4, Vec3};

#[repr(C)]
#[derive(Debug, Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
pub struct DebugVertex {
    pub position: [f32; 3],
    pub color: [f32; 4],
}

impl DebugVertex {
    pub fn desc() -> wgpu::VertexBufferLayout<'static> {
        wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<DebugVertex>() as wgpu::BufferAddress,
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
                    format: wgpu::VertexFormat::Float32x4,
                },
            ],
        }
    }
}

pub fn get_frustum_corners(inv_view_proj: Mat4) -> [Vec3; 8] {
    let mut corners = [Vec3::ZERO; 8];
    let ndc_corners = [
        Vec3::new(-1.0, -1.0, 0.0), // Near
        Vec3::new(1.0, -1.0, 0.0),
        Vec3::new(1.0, 1.0, 0.0),
        Vec3::new(-1.0, 1.0, 0.0),
        Vec3::new(-1.0, -1.0, 1.0), // Far
        Vec3::new(1.0, -1.0, 1.0),
        Vec3::new(1.0, 1.0, 1.0),
        Vec3::new(-1.0, 1.0, 1.0),
    ];
    for i in 0..8 {
        corners[i] = inv_view_proj.project_point3(ndc_corners[i]);
    }
    corners
}

pub fn append_crosshair_lines(
    vertices: &mut Vec<DebugVertex>,
    center: Vec3,
    radius: f32,
    color: [f32; 4],
) {
    let p = center;
    let r = radius;
    vertices.push(DebugVertex {
        position: [p.x - r, p.y, p.z],
        color,
    });
    vertices.push(DebugVertex {
        position: [p.x + r, p.y, p.z],
        color,
    });
    vertices.push(DebugVertex {
        position: [p.x, p.y - r, p.z],
        color,
    });
    vertices.push(DebugVertex {
        position: [p.x, p.y + r, p.z],
        color,
    });
    vertices.push(DebugVertex {
        position: [p.x, p.y, p.z - r],
        color,
    });
    vertices.push(DebugVertex {
        position: [p.x, p.y, p.z + r],
        color,
    });
}

pub fn append_frustum_lines(vertices: &mut Vec<DebugVertex>, corners: &[Vec3; 8], color: [f32; 4]) {
    let indices = [
        0, 1, 1, 2, 2, 3, 3, 0, // near
        4, 5, 5, 6, 6, 7, 7, 4, // far
        0, 4, 1, 5, 2, 6, 3, 7, // connections
    ];
    for &i in &indices {
        vertices.push(DebugVertex {
            position: corners[i].into(),
            color,
        });
    }
}
