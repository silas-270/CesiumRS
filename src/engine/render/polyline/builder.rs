use glam::DVec3;
use crate::engine::property::Property;
use crate::engine::property::sampled::SampledPositionProperty;
use crate::engine::time::SimulationTime;

#[repr(C)]
#[derive(Copy, Clone, Debug, bytemuck::Pod, bytemuck::Zeroable)]
pub struct PolylineVertex {
    pub position: [f32; 3],
    pub previous: [f32; 3],
    pub next: [f32; 3],
    pub side: f32,
}

impl PolylineVertex {
    pub fn desc() -> wgpu::VertexBufferLayout<'static> {
        wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<PolylineVertex>() as wgpu::BufferAddress,
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
                    format: wgpu::VertexFormat::Float32x3,
                },
                wgpu::VertexAttribute {
                    offset: 36,
                    shader_location: 3,
                    format: wgpu::VertexFormat::Float32,
                },
            ],
        }
    }
}

pub struct AdaptiveSubdivisionBuilder {
    pub tolerance: f64,
    pub min_step: f64, // Minimum time step in seconds to avoid infinite recursion
}

impl AdaptiveSubdivisionBuilder {
    pub fn new(tolerance: f64) -> Self {
        Self {
            tolerance,
            min_step: 0.1, // 100ms
        }
    }

    pub fn build(&self, property: &SampledPositionProperty) -> Vec<PolylineVertex> {
        let samples = property.samples();
        if samples.is_empty() {
            return Vec::new();
        }

        let mut path_points: Vec<DVec3> = Vec::new();
        for (_, p) in samples {
            if path_points.is_empty() || path_points.last().unwrap().distance(*p) > 0.000001 {
                path_points.push(*p);
            }
        }

        self.generate_vertices(&path_points)
    }

    fn subdivide(
        &self,
        property: &SampledPositionProperty,
        t_start: f64,
        t_end: f64,
        p_start: DVec3,
        p_end: DVec3,
        points: &mut Vec<DVec3>
    ) {
        if (t_end - t_start) <= self.min_step {
            return;
        }

        let t_mid = (t_start + t_end) * 0.5;
        let p_mid_true = property.evaluate(SimulationTime::new(t_mid)).unwrap();

        // Calculate distance from p_mid_true to the line segment (p_start, p_end)
        let line_vec = p_end - p_start;
        let length_sq = line_vec.length_squared();
        
        let dist = if length_sq < 1e-8 {
            (p_mid_true - p_start).length()
        } else {
            let t = ((p_mid_true - p_start).dot(line_vec) / length_sq).clamp(0.0, 1.0);
            let projection = p_start + line_vec * t;
            (p_mid_true - projection).length()
        };

        if dist > self.tolerance {
            self.subdivide(property, t_start, t_mid, p_start, p_mid_true, points);
            points.push(p_mid_true);
            self.subdivide(property, t_mid, t_end, p_mid_true, p_end, points);
        }
    }

    fn generate_vertices(&self, points: &[DVec3]) -> Vec<PolylineVertex> {
        let mut vertices = Vec::with_capacity(points.len() * 2);

        for i in 0..points.len() {
            let curr = points[i];
            
            let prev = if i > 0 { points[i - 1] } else { curr + (curr - points[i + 1]).normalize_or_zero() * 1.0 };
            let next = if i < points.len() - 1 { points[i + 1] } else { curr + (curr - prev).normalize_or_zero() * 1.0 };

            let curr_f32 = [curr.x as f32, curr.y as f32, curr.z as f32];
            let prev_f32 = [prev.x as f32, prev.y as f32, prev.z as f32];
            let next_f32 = [next.x as f32, next.y as f32, next.z as f32];

            vertices.push(PolylineVertex {
                position: curr_f32,
                previous: prev_f32,
                next: next_f32,
                side: 1.0,
            });

            vertices.push(PolylineVertex {
                position: curr_f32,
                previous: prev_f32,
                next: next_f32,
                side: -1.0,
            });
        }

        vertices
    }
}
