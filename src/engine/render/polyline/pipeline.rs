use wgpu::util::DeviceExt;
use crate::engine::render::polyline::builder::PolylineVertex;

#[repr(C)]
#[derive(Debug, Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
pub struct PolylinePushConstants {
    pub camera_pos: [f32; 4],
    pub viewport_size: [f32; 2],
    pub thickness: f32,
    pub split_progress: f32,
}

pub struct PolylineRenderer {
    pub pipeline: wgpu::RenderPipeline,
    pub vertex_buffer: Option<wgpu::Buffer>,
    pub num_vertices: u32,
    pub vertex_capacity: u32,
}

impl PolylineRenderer {
    pub fn new(
        device: &wgpu::Device,
        config: &wgpu::SurfaceConfiguration,
        camera_bind_group_layout: &wgpu::BindGroupLayout,
    ) -> Self {
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Polyline Shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("polyline.wgsl").into()),
        });

        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("Polyline Pipeline Layout"),
            bind_group_layouts: &[camera_bind_group_layout],
            push_constant_ranges: &[wgpu::PushConstantRange {
                stages: wgpu::ShaderStages::VERTEX,
                range: 0..std::mem::size_of::<PolylinePushConstants>() as u32,
            }],
        });

        let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("Polyline Pipeline"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: "vs_main",
                buffers: &[PolylineVertex::desc()],
                compilation_options: Default::default(),
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: "fs_main",
                targets: &[Some(wgpu::ColorTargetState {
                    format: config.format,
                    blend: Some(wgpu::BlendState::ALPHA_BLENDING),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: Default::default(),
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleStrip,
                strip_index_format: None,
                front_face: wgpu::FrontFace::Ccw,
                cull_mode: None,
                unclipped_depth: false,
                polygon_mode: wgpu::PolygonMode::Fill,
                conservative: false,
            },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: wgpu::TextureFormat::Depth32Float,
                depth_write_enabled: true,
                depth_compare: wgpu::CompareFunction::Less,
                stencil: wgpu::StencilState::default(),
                bias: wgpu::DepthBiasState::default(),
            }),
            multisample: wgpu::MultisampleState {
                count: 1,
                mask: !0,
                alpha_to_coverage_enabled: false,
            },
            multiview: None,
            cache: None,
        });

        Self {
            pipeline,
            vertex_buffer: None,
            num_vertices: 0,
            vertex_capacity: 0,
        }
    }

    pub fn update_geometry(&mut self, device: &wgpu::Device, queue: &wgpu::Queue, vertices: &[PolylineVertex]) {
        self.num_vertices = vertices.len() as u32;
        if self.num_vertices > 0 {
            if self.num_vertices > self.vertex_capacity {
                self.vertex_capacity = self.num_vertices.max(1024); // grow with some extra padding
                self.vertex_buffer = Some(device.create_buffer(&wgpu::BufferDescriptor {
                    label: Some("Polyline Vertex Buffer"),
                    size: (self.vertex_capacity as usize * std::mem::size_of::<PolylineVertex>()) as wgpu::BufferAddress,
                    usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
                    mapped_at_creation: false,
                }));
            }
            if let Some(buffer) = &self.vertex_buffer {
                queue.write_buffer(buffer, 0, bytemuck::cast_slice(vertices));
            }
        }
    }

    pub fn draw<'a>(
        &'a self,
        render_pass: &mut wgpu::RenderPass<'a>,
        camera_bind_group: &'a wgpu::BindGroup,
        viewport_size: [f32; 2],
        thickness: f32,
        camera_pos_f64: [f64; 3],
        split_progress: f32,
    ) {
        if let Some(vertex_buffer) = &self.vertex_buffer {
            let push = PolylinePushConstants {
                camera_pos: [camera_pos_f64[0] as f32, camera_pos_f64[1] as f32, camera_pos_f64[2] as f32, 0.0],
                viewport_size,
                thickness,
                split_progress,
            };

            render_pass.set_pipeline(&self.pipeline);
            render_pass.set_bind_group(0, camera_bind_group, &[]);
            render_pass.set_push_constants(
                wgpu::ShaderStages::VERTEX,
                0,
                bytemuck::cast_slice(&[push]),
            );
            render_pass.set_vertex_buffer(0, vertex_buffer.slice(..));
            render_pass.draw(0..self.num_vertices, 0..1);
        }
    }
}
