use crate::render::polyline_pipeline::builder::ControlPoint;

// ── Push-constant layout (unchanged from previous design) ─────────────────────
#[repr(C)]
#[derive(Debug, Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
pub struct PolylinePushConstants {
    pub reference_point: [f32; 4],  // offset   0 (16 bytes)
    pub camera_pos: [f32; 4],       // offset  16 (16 bytes)
    pub color_start: [f32; 4],      // offset  32 (16 bytes)
    pub color_end: [f32; 4],        // offset  48 (16 bytes)
    pub viewport_size: [f32; 2],    // offset  64 ( 8 bytes)
    pub thickness: f32,             // offset  72 ( 4 bytes)
    pub split_progress: f32,        // offset  76 ( 4 bytes)
    pub physical_half_width: f32,   // offset  80 ( 4 bytes)
    pub physical_half_height: f32,  // offset  84 ( 4 bytes)
    pub _padding: [f32; 2],         // offset  88 ( 8 bytes) — align to 16
    pub airplane_pos: [f32; 4],     // offset  96 (16 bytes)
    pub airplane_forward: [f32; 4], // offset 112 (16 bytes)
    // Total: 128 bytes — exactly at the guaranteed minimum device limit.
}

// ── Per-flight rendering configuration ────────────────────────────────────────
#[derive(Debug, Clone)]
pub struct PolylineConfig {
    pub thickness: f32,
    pub physical_half_width: f32,
    pub physical_half_height: f32,
    pub color_start: [f32; 4],
    pub color_end: [f32; 4],
    /// -1.0 means disabled.
    pub split_progress: f32,
    pub airplane_pos: [f32; 4],
    pub airplane_forward: [f32; 4],
}

impl Default for PolylineConfig {
    fn default() -> Self {
        Self {
            thickness: 4.0,
            physical_half_width: 0.00001116,  // 11.16 m  (~1/3 of airplane wingspan)
            physical_half_height: 0.00000166, // 1.66 m
            color_start: [1.0, 0.4, 0.0, 1.0],
            color_end: [1.0, 0.4, 0.0, 1.0],
            split_progress: -1.0,
            airplane_pos: [0.0; 4],
            airplane_forward: [0.0; 4],
        }
    }
}

// ── Renderer ──────────────────────────────────────────────────────────────────

/// Renders a thick, camera-facing ribbon from a GPU-resident array of `ControlPoint`s.
///
/// The CPU uploads only the raw control points (position + progress).  The vertex
/// shader expands each point into the required ribbon geometry using
/// `@builtin(vertex_index)` arithmetic — no CPU-side `generate_vertices` call.
pub struct PolylineRenderer {
    pub render_pipeline: wgpu::RenderPipeline,

    // Control-point storage buffer
    pub cp_buffer:   Option<wgpu::Buffer>,
    pub cp_capacity: u32,
    pub cp_count:    u32,

    // Bind group exposing the storage buffer to the shader (@group(1))
    pub cp_bind_group:  Option<wgpu::BindGroup>,
    pub cp_bind_layout: wgpu::BindGroupLayout,
}

/// Parameters for a single polyline draw call.
/// Separate lifetimes avoid invariance conflicts with `&mut RenderPass`.
pub struct DrawParams<'rp, 'res, 'cfg> {
    pub render_pass: &'rp mut wgpu::RenderPass<'res>,
    pub camera_bind_group: &'res wgpu::BindGroup,
    pub viewport_size: [f32; 2],
    pub camera_pos_f64: [f64; 3],
    pub reference_point: [f64; 3],
    pub config: &'cfg PolylineConfig,
}

impl PolylineRenderer {
    pub fn new(
        device: &wgpu::Device,
        config: &wgpu::SurfaceConfiguration,
        camera_bind_group_layout: &wgpu::BindGroupLayout,
    ) -> Self {
        // ── Storage-buffer bind-group layout ──────────────────────────────────
        let cp_bind_layout =
            device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("Polyline CP Bind Group Layout"),
                entries: &[wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::VERTEX,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Storage { read_only: true },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                }],
            });

        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Polyline Shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("polyline.wgsl").into()),
        });

        let pipeline_layout =
            device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("Polyline Pipeline Layout"),
                bind_group_layouts: &[camera_bind_group_layout, &cp_bind_layout],
                push_constant_ranges: &[wgpu::PushConstantRange {
                    stages: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT,
                    range: 0..std::mem::size_of::<PolylinePushConstants>() as u32,
                }],
            });

        let render_pipeline =
            device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label: Some("Polyline Pipeline"),
                layout: Some(&pipeline_layout),
                vertex: wgpu::VertexState {
                    module: &shader,
                    entry_point: "vs_main",
                    // No vertex buffers — geometry comes from the storage buffer.
                    buffers: &[],
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
                    depth_compare: wgpu::CompareFunction::Greater,
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
            render_pipeline,
            cp_buffer: None,
            cp_capacity: 0,
            cp_count: 0,
            cp_bind_group: None,
            cp_bind_layout,
        }
    }

    /// Upload new control points to the GPU.
    /// Reallocates the storage buffer only when the existing one is too small.
    pub fn update_geometry(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        points: &[ControlPoint],
    ) {
        self.cp_count = points.len() as u32;

        if self.cp_count == 0 {
            return;
        }

        let needs_realloc = self.cp_count > self.cp_capacity || self.cp_buffer.is_none();
        if needs_realloc {
            // Grow with some head-room to avoid frequent reallocations.
            self.cp_capacity = (self.cp_count * 2).max(1024);
            let byte_size =
                (self.cp_capacity as usize * std::mem::size_of::<ControlPoint>()) as u64;

            let buffer = device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("Polyline Control Point Buffer"),
                size: byte_size,
                usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            });

            self.cp_bind_group = Some(device.create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some("Polyline CP Bind Group"),
                layout: &self.cp_bind_layout,
                entries: &[wgpu::BindGroupEntry {
                    binding: 0,
                    resource: buffer.as_entire_binding(),
                }],
            }));

            self.cp_buffer = Some(buffer);
        }

        if let Some(buf) = &self.cp_buffer {
            queue.write_buffer(buf, 0, bytemuck::cast_slice(points));
        }
    }


    pub fn draw<'rp, 'res, 'cfg>(&'res self, params: DrawParams<'rp, 'res, 'cfg>) {
        let DrawParams {
            render_pass,
            camera_bind_group,
            viewport_size,
            camera_pos_f64,
            reference_point,
            config,
        } = params;

        let Some(cp_bind_group) = &self.cp_bind_group else {
            return;
        };
        if self.cp_count < 2 {
            return;
        }

        let rel_cam = [
            (camera_pos_f64[0] - reference_point[0]) as f32,
            (camera_pos_f64[1] - reference_point[1]) as f32,
            (camera_pos_f64[2] - reference_point[2]) as f32,
            0.0,
        ];

        let push = PolylinePushConstants {
            reference_point: [
                reference_point[0] as f32,
                reference_point[1] as f32,
                reference_point[2] as f32,
                0.0,
            ],
            camera_pos: rel_cam,
            color_start: config.color_start,
            color_end: config.color_end,
            viewport_size,
            thickness: config.thickness,
            split_progress: config.split_progress,
            physical_half_width: config.physical_half_width,
            physical_half_height: config.physical_half_height,
            _padding: [0.0; 2],
            airplane_pos: config.airplane_pos,
            airplane_forward: config.airplane_forward,
        };

        // 2 verts per control point (left + right of ribbon).
        let vertex_count = self.cp_count * 2;

        render_pass.set_pipeline(&self.render_pipeline);
        render_pass.set_bind_group(0, camera_bind_group, &[]);
        render_pass.set_bind_group(1, cp_bind_group, &[]);
        render_pass.set_push_constants(
            wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT,
            0,
            bytemuck::cast_slice(&[push]),
        );
        render_pass.draw(0..vertex_count, 0..1);
    }
}
