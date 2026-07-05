use crate::engine::camera::camera::Camera;
use crate::engine::globe::quadtree::{QuadtreeManager, TileId};
use egui_wgpu::Renderer as EguiRenderer;
use egui_winit::State as EguiState;
use glam::{Mat4, Vec3};
use std::sync::Arc;
use wgpu::util::DeviceExt;
use winit::window::Window;
use lru::LruCache;

pub struct TileBuffers {
    pub vertex_buffer: wgpu::Buffer,
    pub index_buffer: wgpu::Buffer,
    pub num_indices: u32,
    pub center_f64: [f64; 3],
}

#[repr(C)]
#[derive(Debug, Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
pub struct TilePushConstants {
    pub relative_center: [f32; 4], // Padding to 16 bytes included
    pub uv_scale_offset: [f32; 4],
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
    tile_cache: LruCache<TileId, TileBuffers>,
    camera_uniform: CameraUniform,
    camera_buffer: wgpu::Buffer,
    camera_bind_group: wgpu::BindGroup,
    pub camera: Camera,
    pub debug_mode: bool,
    pub debug_camera: crate::engine::camera::GodCamera,
    pub debug_camera_initialized: bool,
    pub last_requested_tiles_count: usize,
    pub last_missing_tiles_count: usize,
    debug_pipeline: wgpu::RenderPipeline,
    debug_vertex_buffer: wgpu::Buffer,
    num_debug_vertices: u32,
    pub egui_ctx: egui::Context,
    pub egui_state: EguiState,
    pub egui_renderer: EguiRenderer,
    pub quadtree_manager: QuadtreeManager,
    pub tile_system: crate::engine::globe::tiles::system::TileSystem,
    pub extension: Option<Box<dyn crate::engine::core::extension::GlobeExtension>>,
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
    pub async fn new(
        window: Arc<Window>, 
        engine_config: crate::engine::globe::tiles::config::TileEngineConfig,
        mut extension: Option<Box<dyn crate::engine::core::extension::GlobeExtension>>
    ) -> Self {
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
                        max_push_constant_size: 128,
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
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::COPY_SRC,
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

        let config_engine = engine_config;
        let tile_system = crate::engine::globe::tiles::system::TileSystem::new(&device, config_engine);

        let (solid_pipeline, wireframe_pipeline, debug_pipeline) = crate::engine::render::pipelines::create_pipelines(
            &device,
            &config,
            &shader,
            &camera_bind_group_layout,
            &tile_system.texture_manager.bind_group_layout,
        );

        let debug_vertex_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Debug Vertex Buffer"),
            size: std::mem::size_of::<crate::engine::render::debug_geometry::DebugVertex>() as u64 * 100000,
            usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let tile_cache = LruCache::new(tile_system.config.mesh_cache_size);

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
        if let Some(ext) = &mut extension {
            ext.init(&device, &queue, &config, &camera_bind_group_layout);
        }

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
            debug_camera: crate::engine::camera::GodCamera::new(glam::Vec3::ZERO, 0.0, 0.0),
            debug_camera_initialized: false,
            last_requested_tiles_count: 0,
            last_missing_tiles_count: 0,
            debug_pipeline,
            debug_vertex_buffer,
            num_debug_vertices: 0,
            egui_ctx,
            egui_state,
            egui_renderer,
            quadtree_manager: QuadtreeManager::new(),
            tile_system,
            extension,
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

    pub fn get_fetch_stats(&self) -> (usize, usize) {
        (self.last_requested_tiles_count, self.last_missing_tiles_count)
    }

    pub fn resize_tile_cache(&mut self, size: std::num::NonZeroUsize) {
        self.tile_cache.resize(size);
    }

    pub fn update_tile_cache(&mut self, visible_tiles: &[(TileId, Vec3, f32)]) {
        // Promote all actively used tiles so they aren't evicted
        for (id, _, _) in visible_tiles {
            self.tile_cache.get(id);
        }

        // Process completed meshes
        for (id, mesh) in self.tile_system.mesh_worker.process_results() {
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

            self.tile_cache.put(
                id,
                TileBuffers {
                    vertex_buffer,
                    index_buffer,
                    num_indices: mesh.indices.len() as u32,
                    center_f64: mesh.center_f64,
                },
            );
        }
    }

    fn update_logic(&mut self, aspect_ratio: f32, main_view_proj: Mat4) -> Vec<(TileId, Vec3, f32)> {
        let (view_matrix, proj_matrix) = if self.debug_mode {
            (self.debug_camera.get_view_matrix(), self.debug_camera.get_projection_matrix(aspect_ratio))
        } else {
            (self.camera.get_view_matrix(), self.camera.get_projection_matrix(aspect_ratio))
        };

        let (camera_pos_f32, _) = self.camera.global_transform();
        let cam_pos_dvec3 = glam::DVec3::new(camera_pos_f32.x as f64, camera_pos_f32.y as f64, camera_pos_f32.z as f64);
        self.quadtree_manager.update(camera_pos_f32, main_view_proj);

        let frustum = self.camera.calculate_frustum_planes(self.config.width as f32 / self.config.height as f32);
        
        if let Some(ext) = &mut self.extension {
            ext.update(&self.device, &self.queue, cam_pos_dvec3, &frustum);
        }

        let mut gpu_view_matrix = view_matrix;
        gpu_view_matrix.w_axis = glam::Vec4::new(0.0, 0.0, 0.0, 1.0); // Strip translation for shader

        self.camera_uniform.update_matrix(
            gpu_view_matrix,
            proj_matrix,
        );
        self.queue.write_buffer(
            &self.camera_buffer,
            0,
            bytemuck::cast_slice(&[self.camera_uniform]),
        );

        let requested_tiles = self.quadtree_manager.get_visible_tiles();

        let renderable_tiles = self.quadtree_manager.get_renderable_tiles(|id| {
            self.tile_cache.peek(id).is_some()
        });

        let missing_count = requested_tiles.iter().filter(|(id, _, _)| self.tile_cache.peek(id).is_none()).count();
        self.last_requested_tiles_count = requested_tiles.len();
        self.last_missing_tiles_count = missing_count;

        let mut active_tiles = requested_tiles.clone();
        for t in &renderable_tiles {
            if !active_tiles.iter().any(|(id, _, _)| *id == t.0) {
                active_tiles.push(*t);
            }
        }

        let mut missing_meshes = Vec::new();
        for (id, _, _) in &active_tiles {
            if self.tile_cache.peek(id).is_none() {
                missing_meshes.push(*id);
            }
        }

        self.tile_system.update(&self.device, &self.queue, camera_pos_f32, &active_tiles, &missing_meshes);

        self.update_tile_cache(&active_tiles);

        renderable_tiles
    }

    fn compute_debug_vertices(&mut self, main_view_proj: Mat4, visible_tiles: &[(TileId, Vec3, f32)], camera_pos: Vec3) {
        let mut debug_vertices = Vec::new();
        if self.debug_mode {
            let inv_view_proj = main_view_proj.inverse();
            let frustum_corners = crate::engine::render::debug_geometry::get_frustum_corners(inv_view_proj);
            crate::engine::render::debug_geometry::append_frustum_lines(&mut debug_vertices, &frustum_corners, [1.0, 1.0, 0.0, 1.0]);

            for (_tile_id, center, radius) in visible_tiles {
                crate::engine::render::debug_geometry::append_crosshair_lines(&mut debug_vertices, *center, *radius, [0.0, 1.0, 0.0, 1.0]);
            }
            
            // Apply camera-relative translation to correctly position these when push constants are not available
            for vertex in &mut debug_vertices {
                vertex.position[0] -= camera_pos.x;
                vertex.position[1] -= camera_pos.y;
                vertex.position[2] -= camera_pos.z;
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
    }

    fn render_scene(
        &mut self,
        encoder: &mut wgpu::CommandEncoder,
        view: &wgpu::TextureView,
        visible_tiles: &[(TileId, Vec3, f32)],
    ) {
        let mut render_pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("Render Pass"),
            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                view,
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

        let camera_pos = if self.debug_mode {
            self.debug_camera.position
        } else {
            self.camera.global_transform().0
        };
        let camera_pos_f64 = [
            camera_pos.x as f64,
            camera_pos.y as f64,
            camera_pos.z as f64,
        ];

        // Draw solid
        render_pass.set_pipeline(&self.solid_pipeline);
        render_pass.set_bind_group(0, &self.camera_bind_group, &[]);
        for (id, _, _) in visible_tiles {
            if let Some(render_data) = self.tile_system.get_render_data(*id) {
                if let Some(buffers) = self.tile_cache.peek(id) {
                    let center_f64 = buffers.center_f64;
                    let push = TilePushConstants {
                        relative_center: [
                            (center_f64[0] - camera_pos_f64[0]) as f32,
                            (center_f64[1] - camera_pos_f64[1]) as f32,
                            (center_f64[2] - camera_pos_f64[2]) as f32,
                            0.0,
                        ],
                        uv_scale_offset: render_data.uv_scale_offset,
                    };

                    render_pass.set_push_constants(
                        wgpu::ShaderStages::VERTEX,
                        0,
                        bytemuck::cast_slice(&[push]),
                    );

                    render_pass.set_bind_group(1, render_data.bind_group, &[]);
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
        for (id, _, _) in visible_tiles {
            if let Some(buffers) = self.tile_cache.peek(id) {
                let center_f64 = buffers.center_f64;
                let push = TilePushConstants {
                    relative_center: [
                        (center_f64[0] - camera_pos_f64[0]) as f32,
                        (center_f64[1] - camera_pos_f64[1]) as f32,
                        (center_f64[2] - camera_pos_f64[2]) as f32,
                        0.0,
                    ],
                    uv_scale_offset: [1.0, 1.0, 0.0, 0.0],
                };
                render_pass.set_push_constants(
                    wgpu::ShaderStages::VERTEX,
                    0,
                    bytemuck::cast_slice(&[push]),
                );

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

        if let Some(ext) = &self.extension {
            ext.render(
                &mut render_pass,
                &self.camera_bind_group,
                [self.config.width as f32, self.config.height as f32],
                camera_pos_f64,
            );
        }
    }

    fn render_egui<F: FnMut(&egui::Context, &mut Self)>(
        &mut self,
        encoder: &mut wgpu::CommandEncoder,
        view: &wgpu::TextureView,
        mut ui_closure: F,
    ) {
        let raw_input = self.egui_state.take_egui_input(&self.window);
        let ctx = self.egui_ctx.clone();

        let full_output = ctx.run(raw_input, |ctx_ref| {
            ui_closure(ctx_ref, self);
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
                encoder,
                &paint_jobs,
                &screen_descriptor,
            );
            crate::engine::render::wgpu_state::execute_egui(
                &self.egui_renderer,
                encoder,
                view,
                &paint_jobs,
                &screen_descriptor,
            );
        }

        for id in &full_output.textures_delta.free {
            self.egui_renderer.free_texture(id);
        }
    }

    pub fn capture_pixels(&self, output_texture: &wgpu::Texture) -> Vec<u8> {
        crate::engine::render::capture::capture_pixels(&self.device, &self.queue, output_texture, &self.config)
    }

    fn capture_screenshot(&self, output_texture: &wgpu::Texture, out_path: &str) {
        crate::engine::render::capture::capture_screenshot(&self.device, &self.queue, output_texture, &self.config, out_path)
    }

    pub fn render<F: FnMut(&egui::Context, &mut Self)>(
        &mut self, 
        screenshot_out: Option<&str>, 
        capture_memory: bool,
        ui_closure: F,
    ) -> Result<Option<Vec<u8>>, wgpu::SurfaceError> {
        let aspect_ratio = self.size.width as f32 / self.size.height as f32;
        let main_view_proj =
            self.camera.get_projection_matrix(aspect_ratio) * self.camera.get_view_matrix();

        let visible_tiles = self.update_logic(aspect_ratio, main_view_proj);

        let output = self.surface.get_current_texture()?;
        let view = output
            .texture
            .create_view(&wgpu::TextureViewDescriptor::default());

        let mut encoder = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("Render Encoder"),
            });

        let camera_pos = if self.debug_mode {
            self.debug_camera.position
        } else {
            self.camera.global_transform().0
        };
        self.compute_debug_vertices(main_view_proj, &visible_tiles, camera_pos);

        self.render_scene(&mut encoder, &view, &visible_tiles);
        self.render_egui(&mut encoder, &view, ui_closure);

        self.queue.submit(std::iter::once(encoder.finish()));

        let mut captured_pixels = None;
        if capture_memory {
            captured_pixels = Some(self.capture_pixels(&output.texture));
        } else if let Some(out_path) = screenshot_out {
            self.capture_screenshot(&output.texture, out_path);
        }

        output.present();

        Ok(captured_pixels)
    }

    pub fn clear_caches(&mut self) {
        self.tile_cache.clear();
        self.tile_system.texture_manager.clear();
        self.tile_system.mesh_worker.clear();
        self.quadtree_manager = crate::engine::globe::quadtree::QuadtreeManager::new();
    }
}
