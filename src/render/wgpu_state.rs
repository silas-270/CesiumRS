use crate::camera::camera::Camera;
use crate::globe::geometry::{TileMesh, Vertex};
use crate::globe::quadtree::{QuadtreeManager, TileId};
use crate::io::texture_manager::TileTextureManager;
use egui_wgpu::Renderer as EguiRenderer;
use egui_winit::State as EguiState;
use glam::{EulerRot, Mat4, Quat, Vec3};
use std::collections::HashMap;
use std::sync::Arc;
use wgpu::util::DeviceExt;
use winit::window::Window;

pub struct TileBuffers {
    pub vertex_buffer: wgpu::Buffer,
    pub index_buffer: wgpu::Buffer,
    pub num_indices: u32,
    pub center_f64: [f64; 3],
}

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

fn get_frustum_corners(inv_view_proj: Mat4) -> [Vec3; 8] {
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

fn append_crosshair_lines(
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

fn append_frustum_lines(vertices: &mut Vec<DebugVertex>, corners: &[Vec3; 8], color: [f32; 4]) {
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

#[repr(C)]
#[derive(Debug, Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
struct CameraUniform {
    view_proj: [[f32; 4]; 4],
}

impl CameraUniform {
    fn new() -> Self {
        Self {
            view_proj: glam::Mat4::IDENTITY.to_cols_array_2d(),
        }
    }

    fn update_matrix(&mut self, view: Mat4, proj: Mat4) {
        self.view_proj = (proj * view).to_cols_array_2d();
    }
}

pub struct WgpuState<'a> {
    pub surface: wgpu::Surface<'a>,
    pub device: wgpu::Device,
    pub queue: wgpu::Queue,
    pub config: wgpu::SurfaceConfiguration,
    pub size: winit::dpi::PhysicalSize<u32>,
    pub window: Arc<Window>,
    solid_pipeline: wgpu::RenderPipeline,
    wireframe_pipeline: wgpu::RenderPipeline,
    depth_texture_view: wgpu::TextureView,
    tile_cache: HashMap<TileId, TileBuffers>,
    camera_uniform: CameraUniform,
    camera_buffer: wgpu::Buffer,
    camera_bind_group: wgpu::BindGroup,
    pub camera: Camera,
    pub debug_mode: bool,
    pub debug_camera: Camera,
    debug_pipeline: wgpu::RenderPipeline,
    debug_vertex_buffer: wgpu::Buffer,
    num_debug_vertices: u32,
    pub egui_ctx: egui::Context,
    pub egui_state: EguiState,
    pub egui_renderer: EguiRenderer,
    pub quadtree_manager: QuadtreeManager,
    pub texture_manager: TileTextureManager,
}

fn create_depth_texture(
    device: &wgpu::Device,
    config: &wgpu::SurfaceConfiguration,
) -> wgpu::TextureView {
    let depth_texture = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("Depth Texture"),
        size: wgpu::Extent3d {
            width: config.width.max(1),
            height: config.height.max(1),
            depth_or_array_layers: 1,
        },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: wgpu::TextureFormat::Depth32Float,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
        view_formats: &[],
    });
    depth_texture.create_view(&wgpu::TextureViewDescriptor::default())
}

fn execute_egui<'rp>(
    renderer: &'rp EguiRenderer,
    encoder: &'rp mut wgpu::CommandEncoder,
    view: &'rp wgpu::TextureView,
    paint_jobs: &[egui::ClippedPrimitive],
    screen_descriptor: &egui_wgpu::ScreenDescriptor,
) {
    let mut render_pass = encoder
        .begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("Egui Render Pass"),
            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                view,
                resolve_target: None,
                ops: wgpu::Operations {
                    load: wgpu::LoadOp::Load,
                    store: wgpu::StoreOp::Store,
                },
            })],
            depth_stencil_attachment: None,
            occlusion_query_set: None,
            timestamp_writes: None,
        })
        .forget_lifetime();
    renderer.render(&mut render_pass, paint_jobs, screen_descriptor);
}

impl<'a> WgpuState<'a> {
    pub async fn new(window: Arc<Window>) -> Self {
        let size = window.inner_size();
        let instance = wgpu::Instance::new(wgpu::InstanceDescriptor {
            backends: wgpu::Backends::all(),
            ..Default::default()
        });

        let surface = instance.create_surface(window.clone()).unwrap();

        let adapter = instance
            .request_adapter(&wgpu::RequestAdapterOptions {
                power_preference: wgpu::PowerPreference::default(),
                compatible_surface: Some(&surface),
                force_fallback_adapter: false,
            })
            .await
            .unwrap();

        let (device, queue) = adapter
            .request_device(
                &wgpu::DeviceDescriptor {
                    label: None,
                    required_features: wgpu::Features::POLYGON_MODE_LINE | wgpu::Features::PUSH_CONSTANTS,
                    required_limits: wgpu::Limits {
                        max_push_constant_size: 16,
                        ..Default::default()
                    },
                    memory_hints: wgpu::MemoryHints::default(),
                },
                None,
            )
            .await
            .unwrap();

        let surface_caps = surface.get_capabilities(&adapter);
        let surface_format = surface_caps
            .formats
            .iter()
            .copied()
            .find(|f| f.is_srgb())
            .unwrap_or(surface_caps.formats[0]);
        let config = wgpu::SurfaceConfiguration {
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            format: surface_format,
            width: size.width,
            height: size.height,
            present_mode: surface_caps.present_modes[0],
            alpha_mode: surface_caps.alpha_modes[0],
            view_formats: vec![],
            desired_maximum_frame_latency: 2,
        };

        let depth_texture_view = create_depth_texture(&device, &config);

        let camera = Camera::new(Vec3::new(0.0, 0.0, 20.0), Vec3::ZERO);
        let mut camera_uniform = CameraUniform::new();
        let mut init_view_matrix = camera.get_view_matrix();
        init_view_matrix.w_axis = glam::Vec4::new(0.0, 0.0, 0.0, 1.0); // strip translation
        camera_uniform.update_matrix(
            init_view_matrix,
            camera.get_projection_matrix(size.width as f32 / size.height as f32),
        );

        let camera_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Camera Buffer"),
            contents: bytemuck::cast_slice(&[camera_uniform]),
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });

        let camera_bind_group_layout =
            device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                entries: &[wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::VERTEX,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                }],
                label: Some("camera_bind_group_layout"),
            });

        let camera_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            layout: &camera_bind_group_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: camera_buffer.as_entire_binding(),
            }],
            label: Some("camera_bind_group"),
        });

        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("shader.wgsl").into()),
        });

        let texture_manager = TileTextureManager::new(&device);

        let push_constant_ranges = [wgpu::PushConstantRange {
            stages: wgpu::ShaderStages::VERTEX,
            range: 0..16,
        }];

        let render_pipeline_layout =
            device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("Render Pipeline Layout"),
                bind_group_layouts: &[
                    &camera_bind_group_layout,
                    &texture_manager.bind_group_layout,
                ],
                push_constant_ranges: &push_constant_ranges,
            });

        let basic_pipeline_layout =
            device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("Basic Pipeline Layout"),
                bind_group_layouts: &[&camera_bind_group_layout],
                push_constant_ranges: &push_constant_ranges,
            });

        let debug_pipeline_layout =
            device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("Debug Pipeline Layout"),
                bind_group_layouts: &[&camera_bind_group_layout],
                push_constant_ranges: &[],
            });

        // 1. Solid Pipeline
        let solid_depth_stencil = Some(wgpu::DepthStencilState {
            format: wgpu::TextureFormat::Depth32Float,
            depth_write_enabled: true,
            depth_compare: wgpu::CompareFunction::Less,
            stencil: wgpu::StencilState::default(),
            bias: wgpu::DepthBiasState::default(),
        });

        let solid_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("Solid Pipeline"),
            layout: Some(&render_pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: "vs_main",
                buffers: &[Vertex::desc()],
                compilation_options: Default::default(),
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: "fs_solid",
                targets: &[Some(wgpu::ColorTargetState {
                    format: config.format,
                    blend: Some(wgpu::BlendState::REPLACE),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: Default::default(),
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                strip_index_format: None,
                front_face: wgpu::FrontFace::Ccw,
                cull_mode: Some(wgpu::Face::Back),
                polygon_mode: wgpu::PolygonMode::Fill,
                unclipped_depth: false,
                conservative: false,
            },
            depth_stencil: solid_depth_stencil,
            multisample: wgpu::MultisampleState::default(),
            multiview: None,
            cache: None,
        });

        // 2. Wireframe Pipeline
        let wireframe_depth_stencil = Some(wgpu::DepthStencilState {
            format: wgpu::TextureFormat::Depth32Float,
            depth_write_enabled: false,
            depth_compare: wgpu::CompareFunction::LessEqual,
            stencil: wgpu::StencilState::default(),
            bias: wgpu::DepthBiasState {
                constant: -2,
                slope_scale: -2.0,
                clamp: 0.0,
            },
        });

        let wireframe_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("Wireframe Pipeline"),
            layout: Some(&basic_pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: "vs_main",
                buffers: &[Vertex::desc()],
                compilation_options: Default::default(),
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: "fs_wireframe",
                targets: &[Some(wgpu::ColorTargetState {
                    format: config.format,
                    blend: Some(wgpu::BlendState::ALPHA_BLENDING),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: Default::default(),
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                strip_index_format: None,
                front_face: wgpu::FrontFace::Ccw,
                cull_mode: Some(wgpu::Face::Back), // Fix: explicitly cull backfaces on wireframe
                polygon_mode: wgpu::PolygonMode::Line,
                unclipped_depth: false,
                conservative: false,
            },
            depth_stencil: wireframe_depth_stencil.clone(),
            multisample: wgpu::MultisampleState::default(),
            multiview: None,
            cache: None,
        });

        let debug_vertex_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Debug Vertex Buffer"),
            size: std::mem::size_of::<DebugVertex>() as u64 * 100000,
            usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let debug_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("Debug Pipeline"),
            layout: Some(&debug_pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: "vs_debug",
                buffers: &[DebugVertex::desc()],
                compilation_options: Default::default(),
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: "fs_debug",
                targets: &[Some(wgpu::ColorTargetState {
                    format: config.format,
                    blend: Some(wgpu::BlendState::ALPHA_BLENDING),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: Default::default(),
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::LineList,
                strip_index_format: None,
                front_face: wgpu::FrontFace::Cw,
                cull_mode: None,
                polygon_mode: wgpu::PolygonMode::Line,
                unclipped_depth: false,
                conservative: false,
            },
            depth_stencil: wireframe_depth_stencil.clone(),
            multisample: wgpu::MultisampleState::default(),
            multiview: None,
            cache: None,
        });

        let tile_cache = HashMap::new();

        let egui_ctx = egui::Context::default();
        let egui_state = EguiState::new(
            egui_ctx.clone(),
            egui::ViewportId::ROOT,
            &window,
            Some(window.scale_factor() as f32),
            None,
            Some(2048),
        );
        let egui_renderer = EguiRenderer::new(&device, config.format, None, 1, false);

        Self {
            surface,
            device,
            queue,
            config,
            size,
            window,
            solid_pipeline,
            wireframe_pipeline,
            depth_texture_view,
            tile_cache,
            camera_uniform,
            camera_buffer,
            camera_bind_group,
            camera,
            debug_mode: false,
            debug_camera: Camera::new(Vec3::new(0.0, 0.0, 25.0), Vec3::ZERO),
            debug_pipeline,
            debug_vertex_buffer,
            num_debug_vertices: 0,
            egui_ctx,
            egui_state,
            egui_renderer,
            quadtree_manager: QuadtreeManager::new(),
            texture_manager,
        }
    }

    pub fn resize(&mut self, new_size: winit::dpi::PhysicalSize<u32>) {
        if new_size.width > 0 && new_size.height > 0 {
            self.size = new_size;
            self.config.width = new_size.width;
            self.config.height = new_size.height;
            self.surface.configure(&self.device, &self.config);

            // Recreate depth texture
            self.depth_texture_view = create_depth_texture(&self.device, &self.config);

            let mut gpu_view_matrix = self.camera.get_view_matrix();
            gpu_view_matrix.w_axis = glam::Vec4::new(0.0, 0.0, 0.0, 1.0); // strip translation

            self.camera_uniform.update_matrix(
                gpu_view_matrix,
                self.camera
                    .get_projection_matrix(new_size.width as f32 / new_size.height as f32),
            );
            self.queue.write_buffer(
                &self.camera_buffer,
                0,
                bytemuck::cast_slice(&[self.camera_uniform]),
            );
        }
    }

    pub fn update_tile_cache(&mut self, visible_tiles: &[(TileId, Vec3, f32)]) {
        let mut active_ids = std::collections::HashSet::new();
        for (id, _, _) in visible_tiles {
            active_ids.insert(*id);
        }

        // Remove culled tiles
        self.tile_cache.retain(|id, _| active_ids.contains(id));

        // Generate missing tiles
        for (id, _, _) in visible_tiles {
            if !self.tile_cache.contains_key(id) {
                let mesh = TileMesh::generate(id, 16);

                let vertex_buffer =
                    self.device
                        .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                            label: Some(&format!("Tile Vertex Buffer {:?}", id)),
                            contents: bytemuck::cast_slice(&mesh.vertices),
                            usage: wgpu::BufferUsages::VERTEX,
                        });

                let index_buffer =
                    self.device
                        .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                            label: Some(&format!("Tile Index Buffer {:?}", id)),
                            contents: bytemuck::cast_slice(&mesh.indices),
                            usage: wgpu::BufferUsages::INDEX,
                        });

                self.tile_cache.insert(
                    *id,
                    TileBuffers {
                        vertex_buffer,
                        index_buffer,
                        num_indices: mesh.indices.len() as u32,
                        center_f64: mesh.center_f64,
                    },
                );
            }
        }
    }

    pub fn render(&mut self) -> Result<(), wgpu::SurfaceError> {
        let aspect_ratio = self.size.width as f32 / self.size.height as f32;
        let main_view_proj =
            self.camera.get_projection_matrix(aspect_ratio) * self.camera.get_view_matrix();
        let render_camera = if self.debug_mode {
            &self.debug_camera
        } else {
            &self.camera
        };

        let camera_pos = self.camera.global_transform().0;
        self.quadtree_manager.update(camera_pos, main_view_proj); // Original main_view_proj with translation is passed here!

        let mut gpu_view_matrix = render_camera.get_view_matrix();
        gpu_view_matrix.w_axis = glam::Vec4::new(0.0, 0.0, 0.0, 1.0); // Strip translation for shader

        self.camera_uniform.update_matrix(
            gpu_view_matrix,
            render_camera.get_projection_matrix(aspect_ratio),
        );
        self.queue.write_buffer(
            &self.camera_buffer,
            0,
            bytemuck::cast_slice(&[self.camera_uniform]),
        );

        let visible_tiles = self.quadtree_manager.get_visible_tiles();
        self.update_tile_cache(&visible_tiles);

        let visible_tile_ids: Vec<TileId> = visible_tiles.iter().map(|(id, _, _)| *id).collect();
        self.texture_manager.cleanup_cache(&visible_tile_ids);

        for id in &visible_tile_ids {
            self.texture_manager
                .request_tile(*id, &self.device, &self.queue);
        }

        self.texture_manager.update(&self.device, &self.queue);

        let mut debug_vertices = Vec::new();
        if self.debug_mode {
            let inv_view_proj = main_view_proj.inverse();
            let frustum_corners = get_frustum_corners(inv_view_proj);
            append_frustum_lines(&mut debug_vertices, &frustum_corners, [1.0, 1.0, 0.0, 1.0]); // Yellow

            for (_tile_id, center, radius) in &visible_tiles {
                append_crosshair_lines(&mut debug_vertices, *center, *radius, [0.0, 1.0, 0.0, 1.0]);
                // Green
            }
        }
        self.num_debug_vertices = debug_vertices.len() as u32;
        if self.num_debug_vertices > 0 {
            self.queue.write_buffer(
                &self.debug_vertex_buffer,
                0,
                bytemuck::cast_slice(&debug_vertices),
            );
        }

        let output = self.surface.get_current_texture()?;
        let view = output
            .texture
            .create_view(&wgpu::TextureViewDescriptor::default());

        let mut encoder = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("Render Encoder"),
            });

        {
            let mut render_pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("Render Pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &view,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color {
                            r: 0.1,
                            g: 0.2,
                            b: 0.3,
                            a: 1.0,
                        }),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &self.depth_texture_view,
                    depth_ops: Some(wgpu::Operations {
                        load: wgpu::LoadOp::Clear(1.0),
                        store: wgpu::StoreOp::Store,
                    }),
                    stencil_ops: None,
                }),
                occlusion_query_set: None,
                timestamp_writes: None,
            });

            let camera_pos_f64 = [
                camera_pos.x as f64,
                camera_pos.y as f64,
                camera_pos.z as f64,
            ];

            // Draw solid
            render_pass.set_pipeline(&self.solid_pipeline);
            render_pass.set_bind_group(0, &self.camera_bind_group, &[]);
            for (id, _, _) in &visible_tiles {
                if let Some((_, bind_group)) = self.texture_manager.get_texture(*id) {
                    if let Some(buffers) = self.tile_cache.get(id) {
                        let center_f64 = buffers.center_f64;
                        let relative_center = [
                            (center_f64[0] - camera_pos_f64[0]) as f32,
                            (center_f64[1] - camera_pos_f64[1]) as f32,
                            (center_f64[2] - camera_pos_f64[2]) as f32,
                            0.0, // padding
                        ];
                        render_pass.set_push_constants(wgpu::ShaderStages::VERTEX, 0, bytemuck::cast_slice(&relative_center));

                        render_pass.set_bind_group(1, bind_group, &[]);
                        render_pass.set_vertex_buffer(0, buffers.vertex_buffer.slice(..));
                        render_pass.set_index_buffer(
                            buffers.index_buffer.slice(..),
                            wgpu::IndexFormat::Uint16,
                        );
                        render_pass.draw_indexed(0..buffers.num_indices, 0, 0..1);
                    }
                }
            }

            // Draw wireframe overlay
            render_pass.set_pipeline(&self.wireframe_pipeline);
            for (id, _, _) in &visible_tiles {
                if let Some(buffers) = self.tile_cache.get(id) {
                    let center_f64 = buffers.center_f64;
                    let relative_center = [
                        (center_f64[0] - camera_pos_f64[0]) as f32,
                        (center_f64[1] - camera_pos_f64[1]) as f32,
                        (center_f64[2] - camera_pos_f64[2]) as f32,
                        0.0, // padding
                    ];
                    render_pass.set_push_constants(wgpu::ShaderStages::VERTEX, 0, bytemuck::cast_slice(&relative_center));

                    render_pass.set_vertex_buffer(0, buffers.vertex_buffer.slice(..));
                    render_pass.set_index_buffer(
                        buffers.index_buffer.slice(..),
                        wgpu::IndexFormat::Uint16,
                    );
                    render_pass.draw_indexed(0..buffers.num_indices, 0, 0..1);
                }
            }

            if self.debug_mode && self.num_debug_vertices > 0 {
                render_pass.set_pipeline(&self.debug_pipeline);
                render_pass.set_vertex_buffer(0, self.debug_vertex_buffer.slice(..));
                render_pass.draw(0..self.num_debug_vertices, 0..1);
            }
        }

        let raw_input = self.egui_state.take_egui_input(&self.window);

        let full_output = self.egui_ctx.run(raw_input, |ctx| {
            egui::Window::new("Debug").resizable(false).show(ctx, |ui| {
                ui.label(format!("Altitude: {:.4}", self.camera.altitude()));

                let mut is_debug = self.debug_mode;
                if ui
                    .checkbox(&mut is_debug, "Debug Mode (Dual Camera)")
                    .changed()
                {
                    self.debug_mode = is_debug;
                    if is_debug {
                        let main_pos = self.camera.local_pos;
                        let forward = self.camera.local_ori * Vec3::Z;
                        let up = self.camera.local_ori * Vec3::Y;
                        self.debug_camera =
                            Camera::new(main_pos - forward * 5.0 + up * 5.0, main_pos);
                    }
                }

                if self.debug_mode {
                    ui.separator();
                    ui.label("Main Camera Override:");
                    ui.horizontal(|ui| {
                        ui.label("Pos:");
                        ui.add(egui::DragValue::new(&mut self.camera.local_pos.x).speed(0.1));
                        ui.add(egui::DragValue::new(&mut self.camera.local_pos.y).speed(0.1));
                        ui.add(egui::DragValue::new(&mut self.camera.local_pos.z).speed(0.1));
                    });

                    let (yaw, pitch, roll) = self.camera.local_ori.to_euler(EulerRot::YXZ);
                    let mut yaw_deg = yaw.to_degrees();
                    let mut pitch_deg = pitch.to_degrees();
                    let mut roll_deg = roll.to_degrees();

                    ui.horizontal(|ui| {
                        ui.label("Rot:");
                        ui.add(
                            egui::DragValue::new(&mut pitch_deg)
                                .speed(1.0)
                                .prefix("P: "),
                        );
                        ui.add(egui::DragValue::new(&mut yaw_deg).speed(1.0).prefix("Y: "));
                        ui.add(egui::DragValue::new(&mut roll_deg).speed(1.0).prefix("R: "));
                    });

                    if pitch_deg != pitch.to_degrees()
                        || yaw_deg != yaw.to_degrees()
                        || roll_deg != roll.to_degrees()
                    {
                        self.camera.local_ori = Quat::from_euler(
                            EulerRot::YXZ,
                            yaw_deg.to_radians(),
                            pitch_deg.to_radians(),
                            roll_deg.to_radians(),
                        );
                    }
                }

                ui.separator();
                ui.label(format!("Visible Tiles: {}", visible_tiles.len()));

                ui.separator();
                ui.label("First 5 Visible Tiles:");
                for (tile, _, _) in visible_tiles.iter().take(5) {
                    ui.label(format!("  Z: {}, X: {}, Y: {}", tile.z, tile.x, tile.y));
                }
            });
        });
        let paint_jobs = self
            .egui_ctx
            .tessellate(full_output.shapes, self.egui_ctx.pixels_per_point());

        for (id, image_delta) in &full_output.textures_delta.set {
            self.egui_renderer
                .update_texture(&self.device, &self.queue, *id, image_delta);
        }

        {
            let screen_descriptor = egui_wgpu::ScreenDescriptor {
                size_in_pixels: [self.config.width, self.config.height],
                pixels_per_point: self.window.scale_factor() as f32,
            };
            self.egui_renderer.update_buffers(
                &self.device,
                &self.queue,
                &mut encoder,
                &paint_jobs,
                &screen_descriptor,
            );
            execute_egui(
                &self.egui_renderer,
                &mut encoder,
                &view,
                &paint_jobs,
                &screen_descriptor,
            );
        }

        for id in &full_output.textures_delta.free {
            self.egui_renderer.free_texture(id);
        }

        self.queue.submit(std::iter::once(encoder.finish()));
        output.present();

        Ok(())
    }
}
