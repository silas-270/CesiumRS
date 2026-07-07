use crate::engine::render::polyline::builder::PolylineVertex;

#[repr(C)]
#[derive(Debug, Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
pub struct PolylinePushConstants {
    pub reference_point: [f32; 4], // offset 0  (16 bytes)
    pub camera_pos: [f32; 4],      // offset 16 (16 bytes)
    pub color_start: [f32; 4],     // offset 32 (16 bytes)
    pub color_end: [f32; 4],       // offset 48 (16 bytes)
    pub viewport_size: [f32; 2],   // offset 64 (8 bytes)
    pub thickness: f32,            // offset 72 (4 bytes)
    pub split_progress: f32,       // offset 76 (4 bytes)
    pub physical_half_width: f32,  // offset 80 (4 bytes)
    pub physical_half_height: f32, // offset 84 (4 bytes)
    pub airplane_pos: [f32; 4],    // offset 88 (16 bytes)
    pub airplane_forward: [f32; 4], // offset 104 (16 bytes)
}

#[derive(Debug, Clone)]
pub struct PolylineConfig {
    pub thickness: f32,
    pub physical_half_width: f32,
    pub physical_half_height: f32,
    pub color_start: [f32; 4],
    pub color_end: [f32; 4],
    pub split_progress: f32, // -1.0 means disabled
    pub airplane_pos: [f32; 4],
    pub airplane_forward: [f32; 4],
}

impl Default for PolylineConfig {
    fn default() -> Self {
        Self {
            thickness: 4.0,
            physical_half_width: 0.00001116,  // 11.16 meters (full width 22.3m, ~1/3 of airplane)
            physical_half_height: 0.00000166, // 1.66 meters
            color_start: [1.0, 0.4, 0.0, 1.0], // Orange
            color_end: [1.0, 0.4, 0.0, 1.0],   // Orange
            split_progress: -1.0,
            airplane_pos: [0.0, 0.0, 0.0, 0.0],
            airplane_forward: [0.0, 0.0, 0.0, 0.0],
        }
    }
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
                stages: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT,
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
        camera_pos_f64: [f64; 3],
        reference_point: [f64; 3],
        config: &PolylineConfig,
    ) {
        if let Some(vertex_buffer) = &self.vertex_buffer {
            let rel_cam = [
                (camera_pos_f64[0] - reference_point[0]) as f32,
                (camera_pos_f64[1] - reference_point[1]) as f32,
                (camera_pos_f64[2] - reference_point[2]) as f32,
                0.0,
            ];
            let push = PolylinePushConstants {
                camera_pos: rel_cam,
                color_start: config.color_start,
                color_end: config.color_end,
                viewport_size,
                thickness: config.thickness,
                split_progress: config.split_progress,
                physical_half_width: config.physical_half_width,
                physical_half_height: config.physical_half_height,
                reference_point: [reference_point[0] as f32, reference_point[1] as f32, reference_point[2] as f32, 0.0],
                airplane_pos: config.airplane_pos,
                airplane_forward: config.airplane_forward,
            };

            render_pass.set_pipeline(&self.pipeline);
            render_pass.set_bind_group(0, camera_bind_group, &[]);
            render_pass.set_push_constants(
                wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT,
                0,
                bytemuck::cast_slice(&[push]),
            );
            render_pass.set_vertex_buffer(0, vertex_buffer.slice(..));
            render_pass.draw(0..self.num_vertices, 0..1);
        }
    }
}
