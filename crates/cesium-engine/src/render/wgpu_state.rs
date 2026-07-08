use crate::camera::camera::Camera;
use crate::globe::quadtree::{QuadtreeManager, TileId};
use crate::render::camera_uniform::CameraUniform;
use crate::render::tile_display::{TileBuffers, TileDisplayEntry, TilePushConstants};
use egui_wgpu::Renderer as EguiRenderer;
use egui_winit::State as EguiState;
use glam::{Mat4, Vec3};
use lru::LruCache;
use std::collections::{HashMap, HashSet};
use std::sync::Arc;
use std::time::Instant;
use wgpu::util::DeviceExt;
use winit::window::Window;

#[derive(Default, Clone, Copy, Debug)]
pub struct FrameTimings {
    pub update_logic_us: f64,
    pub label_manager_us: f64,
    pub render_scene_us: f64,
}

pub struct WgpuState<'a> {
    pub surface: wgpu::Surface<'a>,
    pub device: wgpu::Device,
    pub queue: wgpu::Queue,
    pub config: wgpu::SurfaceConfiguration,
    pub size: winit::dpi::PhysicalSize<u32>,
    pub window: Arc<Window>,
    solid_pipeline: wgpu::RenderPipeline,
    sky_pipeline: wgpu::RenderPipeline,
    #[allow(dead_code)]
    wireframe_pipeline: wgpu::RenderPipeline,
    depth_texture_view: wgpu::TextureView,
    tile_cache: LruCache<TileId, TileBuffers>,
    camera_uniform: CameraUniform,
    camera_buffer: wgpu::Buffer,
    camera_bind_group: wgpu::BindGroup,
    pub camera: Camera,
    pub debug_mode: bool,
    pub debug_camera: crate::camera::GodCamera,
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
    pub tile_system: crate::globe::tiles::system::TileSystem,
    pub extension: Option<Box<dyn crate::core::extension::GlobeExtension>>,
    /// Stable display state: persists across frames, only updated under controlled rules.
    pub display_state: HashMap<TileId, TileDisplayEntry>,
    /// The set of tiles that were visible last frame (for eviction of stale entries).
    pub last_visible_set: HashSet<TileId>,
    /// Fix 3: bounded LRU of tiles that have *ever* successfully shown their own
    /// hi-res texture. Used by Fix 4 to skip the sibling-gate on re-entry.
    /// Capacity 4096 to handle long Europe→US flights without unbounded growth.
    tiles_with_own_texture: LruCache<TileId, ()>,
    pub label_manager: crate::label::LabelManager,
    pub last_timings: FrameTimings,
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
        engine_config: crate::globe::tiles::config::TileEngineConfig,
        mut extension: Option<Box<dyn crate::core::extension::GlobeExtension>>,
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
                    required_features: wgpu::Features::POLYGON_MODE_LINE
                        | wgpu::Features::PUSH_CONSTANTS,
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
            camera.global_transform_f64().0,
            camera.sun_intensity,
            [
                engine_config.map_saturation,
                engine_config.map_contrast,
                engine_config.map_brightness,
            ],
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
                    visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT,
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
            source: wgpu::ShaderSource::Wgsl(include_str!("globe_pipeline/shader.wgsl").into()),
        });

        let sky_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Sky Shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("sky_pipeline/sky.wgsl").into()),
        });

        let config_engine = engine_config;
        let tile_system =
            crate::globe::tiles::system::TileSystem::new(&device, &queue, config_engine);

        let (solid_pipeline, wireframe_pipeline, debug_pipeline) =
            crate::render::globe_pipeline::pipeline::create_pipelines(
                &device,
                &config,
                &shader,
                &camera_bind_group_layout,
                &tile_system.texture_manager.bind_group_layout,
            );

        let sky_pipeline = crate::render::globe_pipeline::pipeline::create_sky_pipeline(
            &device,
            &config,
            &sky_shader,
            &camera_bind_group_layout,
        );

        let debug_vertex_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Debug Vertex Buffer"),
            size: std::mem::size_of::<crate::render::debug_geometry::DebugVertex>() as u64 * 100000,
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
            sky_pipeline,
            wireframe_pipeline,
            depth_texture_view,
            tile_cache,
            camera_uniform,
            camera_buffer,
            camera_bind_group,
            camera,
            debug_mode: false,
            debug_camera: crate::camera::GodCamera::new(glam::Vec3::ZERO, 0.0, 0.0),
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
            display_state: HashMap::new(),
            last_visible_set: HashSet::new(),
            tiles_with_own_texture: LruCache::new(std::num::NonZeroUsize::new(4096).unwrap()),
            label_manager: crate::label::LabelManager::new(),
            last_timings: FrameTimings::default(),
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
                self.camera.global_transform_f64().0,
                self.camera.sun_intensity,
                [
                    self.tile_system.config.map_saturation,
                    self.tile_system.config.map_contrast,
                    self.tile_system.config.map_brightness,
                ],
            );
            self.queue.write_buffer(
                &self.camera_buffer,
                0,
                bytemuck::cast_slice(&[self.camera_uniform]),
            );
        }
    }

    pub fn get_fetch_stats(&self) -> (usize, usize) {
        (
            self.last_requested_tiles_count,
            self.last_missing_tiles_count,
        )
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
            let vertex_buffer = self
                .device
                .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some(&format!("Tile Vertex Buffer {:?}", id)),
                    contents: bytemuck::cast_slice(&mesh.vertices),
                    usage: wgpu::BufferUsages::VERTEX,
                });

            let index_buffer = self
                .device
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

    fn update_logic(
        &mut self,
        aspect_ratio: f32,
        _main_view_proj: Mat4,
    ) -> Vec<(TileId, Vec3, f32)> {
        let (camera_pos_dvec3, _) = self.camera.global_transform_f64();

        // ALWAYS use main camera for logic and culling
        let mut frustum = self.camera.calculate_frustum_planes(aspect_ratio);

        if let Some(ext) = &mut self.extension {
            ext.update(
                &self.device,
                &self.queue,
                camera_pos_dvec3,
                &frustum,
                &mut self.camera,
                aspect_ratio,
            );
            // Recalculate frustum since the extension may have moved the camera!
            frustum = self.camera.calculate_frustum_planes(aspect_ratio);
        }

        let (view_matrix, proj_matrix) = if self.debug_mode {
            (
                self.debug_camera.get_view_matrix(),
                self.debug_camera.get_projection_matrix(aspect_ratio),
            )
        } else {
            (
                self.camera.get_view_matrix(),
                self.camera.get_projection_matrix(aspect_ratio),
            )
        };

        let (camera_pos_dvec, camera_ori_dquat) = self.camera.global_transform_f64();
        let camera_pos_f32 = glam::Vec3::new(
            camera_pos_dvec.x as f32,
            camera_pos_dvec.y as f32,
            camera_pos_dvec.z as f32,
        );
        let camera_ori_f32 = glam::Quat::from_xyzw(
            camera_ori_dquat.x as f32,
            camera_ori_dquat.y as f32,
            camera_ori_dquat.z as f32,
            camera_ori_dquat.w as f32,
        );
        self.quadtree_manager.update(camera_pos_f32, frustum);

        let altitude = self.camera.altitude();
        let zoom = ((-altitude.max(0.0001).log2() + 4.0) as isize).clamp(0, 15) as usize;
        let frustum_obj = crate::globe::quadtree::Frustum::from_planes(frustum);
        
        let label_start = Instant::now();
        self.label_manager.update(camera_pos_f32, camera_ori_f32, altitude, zoom, &frustum_obj);
        self.last_timings.label_manager_us = label_start.elapsed().as_secs_f64() * 1_000_000.0;
        


        let mut gpu_view_matrix = view_matrix;
        gpu_view_matrix.w_axis = glam::Vec4::new(0.0, 0.0, 0.0, 1.0); // Strip translation for shader

        self.camera_uniform.update_matrix(
            gpu_view_matrix,
            proj_matrix,
            camera_pos_dvec,
            self.camera.sun_intensity,
            [
                self.tile_system.config.map_saturation,
                self.tile_system.config.map_contrast,
                self.tile_system.config.map_brightness,
            ],
        );
        self.queue.write_buffer(
            &self.camera_buffer,
            0,
            bytemuck::cast_slice(&[self.camera_uniform]),
        );

        // Get the geometrically-desired visible tile set from the quadtree.
        let visible_tiles = self.quadtree_manager.get_visible_tiles();

        // Get the actually renderable set of tiles (falling back to parent meshes if children aren't ready).
        let renderable_tiles = self
            .quadtree_manager
            .get_renderable_tiles(|id| self.tile_cache.peek(id).is_some());

        let missing_count = visible_tiles
            .iter()
            .filter(|(id, _, _)| self.tile_cache.peek(id).is_none())
            .count();
        self.last_requested_tiles_count = visible_tiles.len();
        self.last_missing_tiles_count = missing_count;

        // Kick off mesh and texture fetches for everything visible.
        let mut missing_meshes = Vec::new();
        for (id, _, _) in &visible_tiles {
            if self.tile_cache.peek(id).is_none() {
                missing_meshes.push(*id);
            }
        }

        // Also kick off fetches for any fallback parent meshes we are trying to render!
        for (id, _, _) in &renderable_tiles {
            if self.tile_cache.peek(id).is_none() && !missing_meshes.contains(id) {
                missing_meshes.push(*id);
            }
        }
        self.tile_system.update(
            &self.device,
            &self.queue,
            camera_pos_f32,
            &visible_tiles,
            &missing_meshes,
        );

        // Promote both mathematically visible tiles and actively rendered fallback parents in the cache.
        self.update_tile_cache(&visible_tiles);
        for (id, _, _) in &renderable_tiles {
            self.tile_cache.get(id); // Keep fallback meshes alive!
        }

        // Update the stable display-state map. This is where texture assignment
        // decisions are made with no-downgrade and sibling-gate rules.
        // We feed it renderable_tiles so that parent fallback meshes get their textures and get drawn.
        self.update_display_state(&renderable_tiles);

        renderable_tiles
    }

    /// The core stable-texture logic. Called once per frame after the quadtree and
    /// tile fetches have been updated.
    ///
    /// Rules:
    /// 1. Tiles leaving the visible set are held in display_state for a 200 ms grace
    ///    period before eviction (Fix 2). This prevents texture blinks from transient
    ///    LOD oscillations: the tile keeps its texture assignment during brief absences.
    /// 2. New tiles enter at the best available fallback (parent/grandparent texture).
    /// 3. A tile upgrades to its own hi-res texture only when ALL 3 siblings also
    ///    have their own hi-res textures ready — OR when the tile has been waiting
    ///    more than 2 seconds (timeout, prevents permanent fallback on 404) — OR
    ///    when the tile was previously upgraded and its texture is still cached (Fix 4).
    /// 4. Once a tile is showing its own hi-res texture, it is NEVER downgraded back
    ///    to a parent fallback (prevents jitter from transient LRU evictions).
    /// 5. Tiles that have ever earned their own texture are recorded in
    ///    `tiles_with_own_texture` (Fix 3) so Fix 4 can fast-path the sibling-gate.
    fn update_display_state(&mut self, visible_tiles: &[(TileId, Vec3, f32)]) {
        const SIBLING_UPGRADE_TIMEOUT: std::time::Duration = std::time::Duration::from_secs(2);
        /// Fix 2: how long a tile can be absent from the visible set before its
        /// display_state entry is evicted. 200 ms comfortably covers the worst-case
        /// LOD oscillation period (~10 frames at 60 fps ≈ 167 ms).
        const DISPLAY_GRACE_PERIOD: std::time::Duration = std::time::Duration::from_millis(200);

        let now = Instant::now();
        let current_visible: HashSet<TileId> = visible_tiles.iter().map(|(id, _, _)| *id).collect();

        // Fix 2: grace-period eviction.
        // Update absent_since for every entry, then evict only those that have been
        // absent for longer than the grace period.
        for (id, entry) in self.display_state.iter_mut() {
            if current_visible.contains(id) {
                entry.absent_since = None; // tile is visible again — reset the timer
            } else if entry.absent_since.is_none() {
                entry.absent_since = Some(now); // first frame of absence — start the clock
            }
        }
        self.display_state.retain(|_, entry| {
            entry
                .absent_since
                .is_none_or(|t| t.elapsed() < DISPLAY_GRACE_PERIOD)
        });
        self.last_visible_set = current_visible;

        for (id, _center, _radius) in visible_tiles {
            let id = *id;

            // Check if own hi-res texture is available (non-mutating peek).
            let own_ready = self
                .tile_system
                .peek_render_data(id)
                .map(|(tex_id, _)| tex_id == id)
                .unwrap_or(false);

            if let Some(entry) = self.display_state.get_mut(&id) {
                // --- Tile already has a display entry (visible or returning from grace) ---

                if entry.showing_own_texture {
                    // Already at hi-res: never downgrade, just continue.
                    continue;
                }

                if own_ready {
                    // Own texture is ready. Check upgrade conditions.
                    let timeout_elapsed = entry.first_seen.elapsed() >= SIBLING_UPGRADE_TIMEOUT;

                    // Fix 4: skip the sibling-gate if this tile has previously earned
                    // its own texture and that texture is still cached (own_ready == true).
                    // Re-upgrades after transient evictions don't need sibling coordination.
                    let previously_upgraded = self.tiles_with_own_texture.peek(&id).is_some();

                    let all_siblings_ready = previously_upgraded
                        || if id.z == 0 {
                            true // root tiles have no siblings
                        } else {
                            // Compute sibling IDs (same parent, all 4 children)
                            let parent = id.parent().unwrap();
                            let sibling_ids = [
                                TileId {
                                    z: id.z,
                                    x: parent.x * 2,
                                    y: parent.y * 2,
                                },
                                TileId {
                                    z: id.z,
                                    x: parent.x * 2 + 1,
                                    y: parent.y * 2,
                                },
                                TileId {
                                    z: id.z,
                                    x: parent.x * 2,
                                    y: parent.y * 2 + 1,
                                },
                                TileId {
                                    z: id.z,
                                    x: parent.x * 2 + 1,
                                    y: parent.y * 2 + 1,
                                },
                            ];
                            sibling_ids.iter().all(|sib| {
                                self.tile_system
                                    .peek_render_data(*sib)
                                    .map(|(tex_id, _)| tex_id == *sib)
                                    .unwrap_or(false)
                            })
                        };

                    if all_siblings_ready || timeout_elapsed {
                        // Upgrade: switch to own hi-res texture, lock it in.
                        entry.texture_id = id;
                        entry.uv_scale_offset = [1.0, 1.0, 0.0, 0.0];
                        entry.showing_own_texture = true;
                        // Fix 3: record this tile in the bounded upgrade history.
                        self.tiles_with_own_texture.put(id, ());
                    }
                    // else: keep showing the existing parent fallback, no change.
                } else {
                    // Own texture still not ready. If we are showing the fallback color (texture_id == id),
                    // check if a parent fallback texture has become ready.
                    if let Some((parent_id, uv)) = self.tile_system.peek_render_data(id) {
                        if parent_id != id && entry.texture_id != parent_id {
                            entry.texture_id = parent_id;
                            entry.uv_scale_offset = uv;
                        }
                    }
                }
            } else {
                // --- Tile is genuinely new to the visible set ---
                let peek = self.tile_system.peek_render_data(id);
                let (tex_id, uv) = peek.unwrap_or((id, [1.0, 1.0, 0.0, 0.0]));
                let showing_own = peek.is_some_and(|(tid, _)| tid == id);

                // Fix 3: if the tile is immediately showing its own texture
                // (e.g. the texture was pre-fetched), record it now.
                if showing_own {
                    self.tiles_with_own_texture.put(id, ());
                }

                self.display_state.insert(
                    id,
                    TileDisplayEntry {
                        texture_id: tex_id,
                        uv_scale_offset: uv,
                        first_seen: now,
                        showing_own_texture: showing_own,
                        absent_since: None,
                    },
                );
            }
        }
    }

    fn compute_debug_vertices(
        &mut self,
        main_view_proj: Mat4,
        visible_tiles: &[(TileId, Vec3, f32)],
        camera_pos: Vec3,
    ) {
        let mut debug_vertices = Vec::new();
        if self.debug_mode {
            let inv_view_proj = main_view_proj.inverse();
            let frustum_corners = crate::render::debug_geometry::get_frustum_corners(inv_view_proj);
            crate::render::debug_geometry::append_frustum_lines(
                &mut debug_vertices,
                &frustum_corners,
                [1.0, 1.0, 0.0, 1.0],
            );

            for (_tile_id, center, radius) in visible_tiles {
                crate::render::debug_geometry::append_crosshair_lines(
                    &mut debug_vertices,
                    *center,
                    *radius,
                    [0.0, 1.0, 0.0, 1.0],
                );
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
        _visible_tiles: &[(TileId, Vec3, f32)],
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
                    load: wgpu::LoadOp::Clear(0.0), // Reverse-Z: clear to 0.0
                    store: wgpu::StoreOp::Store,
                }),
                stencil_ops: None,
            }),
            occlusion_query_set: None,
            timestamp_writes: None,
        });

        let camera_pos_f64 = if self.debug_mode {
            [
                self.debug_camera.position.x as f64,
                self.debug_camera.position.y as f64,
                self.debug_camera.position.z as f64,
            ]
        } else {
            let (pos_dvec, _) = self.camera.global_transform_f64();
            [pos_dvec.x, pos_dvec.y, pos_dvec.z]
        };

        // Draw solid — iterate display_state for stable per-tile texture assignments.
        // display_state was built this frame by update_display_state() with no-downgrade rules.
        render_pass.set_pipeline(&self.solid_pipeline);
        render_pass.set_bind_group(0, &self.camera_bind_group, &[]);

        // Collect display_state entries to avoid borrow conflict with tile_system.
        let draw_list: Vec<(TileId, TileId, [f32; 4])> = self
            .display_state
            .iter()
            .map(|(mesh_id, entry)| (*mesh_id, entry.texture_id, entry.uv_scale_offset))
            .collect();

        for (mesh_id, texture_id, uv_scale_offset) in &draw_list {
            // Get the GPU texture bind group for the assigned texture (LRU-promoting, correct at draw time).
            if let Some(render_data) = self.tile_system.get_render_data(*texture_id) {
                if let Some(buffers) = self.tile_cache.peek(mesh_id) {
                    let center_f64 = buffers.center_f64;
                    let push = TilePushConstants {
                        relative_center: [
                            (center_f64[0] - camera_pos_f64[0]) as f32,
                            (center_f64[1] - camera_pos_f64[1]) as f32,
                            (center_f64[2] - camera_pos_f64[2]) as f32,
                            0.0,
                        ],
                        uv_scale_offset: *uv_scale_offset,
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

        // Wireframe overlay rendering removed as per user request

        if self.debug_mode && self.num_debug_vertices > 0 {
            render_pass.set_pipeline(&self.debug_pipeline);
            render_pass.set_bind_group(0, &self.camera_bind_group, &[]);
            render_pass.set_vertex_buffer(
                0,
                self.debug_vertex_buffer
                    .slice(0..(self.num_debug_vertices as u64 * 32)),
            );
            render_pass.draw(0..self.num_debug_vertices, 0..1);
        }

        // --- SKY RENDERING ---
        // Draw the procedural sky perfectly isolated in the background!
        // Uses depth Equal 1.0, so it's perfectly rejected by any terrain already drawn.
        render_pass.set_pipeline(&self.sky_pipeline);
        render_pass.set_bind_group(0, &self.camera_bind_group, &[]);
        // Full-screen triangle is drawn with exactly 3 virtual vertices
        render_pass.draw(0..3, 0..1);

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
            crate::render::wgpu_state::execute_egui(
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
        crate::render::capture::capture_pixels(
            &self.device,
            &self.queue,
            output_texture,
            &self.config,
        )
    }

    fn capture_screenshot(&self, output_texture: &wgpu::Texture, out_path: &str) {
        crate::render::capture::capture_screenshot(
            &self.device,
            &self.queue,
            output_texture,
            &self.config,
            out_path,
        )
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

        let update_start = Instant::now();
        let visible_tiles = self.update_logic(aspect_ratio, main_view_proj);
        self.last_timings.update_logic_us = update_start.elapsed().as_secs_f64() * 1_000_000.0;

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
            let (pos_dvec, _) = self.camera.global_transform();
            glam::Vec3::new(pos_dvec.x, pos_dvec.y, pos_dvec.z)
        };
        self.compute_debug_vertices(main_view_proj, &visible_tiles, camera_pos);

        let render_start = Instant::now();
        self.render_scene(&mut encoder, &view, &visible_tiles);
        self.last_timings.render_scene_us = render_start.elapsed().as_secs_f64() * 1_000_000.0;
        
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
}
