use std::mem;

#[repr(C)]
#[derive(Debug, Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
pub struct ModelVertex {
    pub position: [f32; 3],
    pub normal: [f32; 3],
    pub uv: [f32; 2],
    pub color: [f32; 4],
    /// `1.0` for a surface that emits its own light — a cockpit display — and so should
    /// be drawn at its texture's own value, ignoring the key light and the ambient floor
    /// entirely. `0.0` is ordinary shaded geometry. Values between the two blend.
    pub unlit: f32,
}

impl ModelVertex {
    pub fn desc<'a>() -> wgpu::VertexBufferLayout<'a> {
        wgpu::VertexBufferLayout {
            array_stride: mem::size_of::<ModelVertex>() as wgpu::BufferAddress,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &[
                wgpu::VertexAttribute {
                    offset: 0,
                    shader_location: 0,
                    format: wgpu::VertexFormat::Float32x3,
                },
                wgpu::VertexAttribute {
                    offset: mem::size_of::<[f32; 3]>() as wgpu::BufferAddress,
                    shader_location: 1,
                    format: wgpu::VertexFormat::Float32x3,
                },
                wgpu::VertexAttribute {
                    offset: mem::size_of::<[f32; 6]>() as wgpu::BufferAddress,
                    shader_location: 2,
                    format: wgpu::VertexFormat::Float32x2,
                },
                wgpu::VertexAttribute {
                    offset: mem::size_of::<[f32; 8]>() as wgpu::BufferAddress,
                    shader_location: 3,
                    format: wgpu::VertexFormat::Float32x4,
                },
                wgpu::VertexAttribute {
                    offset: mem::size_of::<[f32; 12]>() as wgpu::BufferAddress,
                    shader_location: 4,
                    format: wgpu::VertexFormat::Float32,
                },
            ],
        }
    }
}

#[repr(C)]
#[derive(Debug, Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
pub struct ModelPushConstants {
    pub model_matrix_0: [f32; 4],
    pub model_matrix_1: [f32; 4],
    pub model_matrix_2: [f32; 4],
    pub model_matrix_3: [f32; 4],
    pub camera_pos: [f32; 4],
    pub viewport_size: [f32; 2],
    /// Minimum on-screen size in pixels the model is inflated to. `0.0` disables the
    /// boost entirely, so the mesh is drawn at its true world scale.
    pub min_pixel_size: f32,
    /// Clip-space depth bias applied as `z += depth_bias * w`. `0.0` disables it.
    pub depth_bias: f32,
    /// Flat lighting floor. `0.5` reproduces the old hardcoded ambient exactly.
    pub ambient_override: f32,
    /// Blinn-Phong specular highlight strength. `0.0` disables it entirely.
    pub specular_strength: f32,
    /// Triplanar procedural surface-detail strength. `0.0` disables it entirely.
    pub detail_strength: f32,
    /// Fresnel-style edge light picked up from the key light. `0.0` disables it entirely.
    pub rim_strength: f32,
    /// How much of the key light's *direction* this model feels, against its ambient.
    ///
    /// An interior wants this low. The key light has no occlusion, so inside a cockpit it
    /// happily lights the roof lining — which faces the sky and is, in reality, under a
    /// fuselage. What actually lights a flight deck is light bounced in through the
    /// windows, which is ambient. `1.0` is an object in the open.
    pub diffuse_weight: f32,
}

/// Per-model knobs for [`ModelRenderer::new_with_options`].
///
/// [`ModelOptions::default`] reproduces the behaviour tuned for the A350 exterior:
/// a mesh normalized to radius 1.0 (so the vertex shader's screen-size boost has a
/// known baseline), backface culling, and no material rewriting.
pub struct ModelOptions<'f> {
    /// Scale the assembled mesh so its farthest vertex sits at radius 1.0. Set to
    /// `false` to keep the glTF's source units (metres, for a real-scale interior).
    pub normalize_to_unit_radius: bool,
    pub cull_mode: Option<wgpu::Face>,
    /// Primitives whose final base-colour alpha is below this are dropped entirely.
    /// Useful when the whole model is one unsorted draw call that writes depth, so
    /// blended geometry would occlude whatever sits behind it.
    pub skip_alpha_below: f32,
    /// Per-material colour override, keyed by glTF material name. Applied *before*
    /// `skip_alpha_below`, so an override returning alpha 0 also removes geometry.
    pub material_override: Option<&'f dyn Fn(Option<&str>, [f32; 4]) -> [f32; 4]>,
    /// Per-primitive geometric nudge, keyed by `(mesh index, primitive index)`. Pushes
    /// every vertex of the matched primitive along its own vertex normal by the
    /// returned distance (model-space metres). Used to separate coincident duplicate
    /// geometry (e.g. an exported "fake double-sided" pair) without touching materials
    /// that are used correctly elsewhere. `None` (default) is a no-op.
    pub primitive_normal_offset: Option<&'f dyn Fn(usize, usize) -> f32>,
    /// Replacement for the model's baked-in texture (or the 1x1 white fallback):
    /// `(width, height, RGBA8 pixels, row-major, no padding)`. `None` (default)
    /// reproduces current behaviour (`images.first()`, else 1x1 white).
    pub texture_override: Option<(u32, u32, Vec<u8>)>,
    /// The point of the source mesh, in the glTF's own units, that becomes the model
    /// origin. Every vertex is translated by `-origin_offset` before anything else, so
    /// this is what the model rotates about and the point the flight path holds. Applied
    /// before `normalize_to_unit_radius`, which therefore measures its radius from here.
    /// `[0.0; 3]` (default) keeps the glTF's own origin.
    pub origin_offset: [f32; 3],
    /// Uniform scale applied after normalisation. `1.0` (default) is a no-op. This exists
    /// to hold a replacement mesh to the on-screen footprint of the one it replaces when
    /// the two have different proportions, which unit-radius normalisation alone will not
    /// do.
    pub post_scale: f32,
    /// Per-vertex UV rewrite, keyed by glTF material name.
    ///
    /// A model is drawn with one texture, so every material that has UVs shares one
    /// 0..1 space. When that texture is an atlas holding content for *some* of them —
    /// cockpit screens, say — the rest would sample it too and show smeared fragments of
    /// it. Returning a constant UV for those parks them on a single texel, which is the
    /// whole of what they then sample: keep that texel white and they render at their
    /// base colour exactly as they did with no texture at all.
    ///
    /// A constant UV also has zero screen-space derivative, so the GPU picks mip 0 and
    /// the parked material is unaffected by the rest of the atlas at any distance.
    ///
    /// `None` (default) passes the authored UVs through untouched.
    pub uv_override: Option<&'f dyn Fn(Option<&str>, [f32; 2]) -> [f32; 2]>,
    /// Marks materials as self-lit, keyed by glTF material name: `1.0` draws them at their
    /// texture's own value, `0.0` (the default for every material) shades them normally.
    ///
    /// A cockpit display is a light source, not a lit surface. Shaded like the panel
    /// around it, it renders at the ambient floor — about a quarter value — and reads as a
    /// dark grey rectangle no matter what is painted on it.
    pub material_unlit: Option<&'f dyn Fn(Option<&str>) -> f32>,
    /// Cap on the base mip level's dimensions, for a model whose authored texture carries
    /// more detail than it is ever drawn at. The chain is built from the full-size image
    /// either way and this drops the levels above the cap, so the pixels that survive have
    /// been through the same gamma-correct box filter — and every level below the cap is
    /// one that would have existed anyway. `None` (default) uploads the image as authored.
    pub max_texture_size: Option<u32>,
    /// Label used in the log line emitted once the mesh is assembled.
    pub label: &'f str,
}

impl<'f> Default for ModelOptions<'f> {
    fn default() -> Self {
        Self {
            normalize_to_unit_radius: true,
            cull_mode: Some(wgpu::Face::Back),
            skip_alpha_below: 0.0,
            material_override: None,
            primitive_normal_offset: None,
            texture_override: None,
            origin_offset: [0.0; 3],
            post_scale: 1.0,
            uv_override: None,
            material_unlit: None,
            max_texture_size: None,
            label: "Model",
        }
    }
}

pub struct ModelRenderer {
    pub pipeline: wgpu::RenderPipeline,
    pub vertex_buffer: wgpu::Buffer,
    pub index_buffer: wgpu::Buffer,
    pub num_indices: u32,
    pub bind_group: wgpu::BindGroup,
}

impl ModelRenderer {
    pub fn new(
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        config: &wgpu::SurfaceConfiguration,
        camera_bind_group_layout: &wgpu::BindGroupLayout,
        glb_bytes: &[u8],
    ) -> Result<Self, Box<dyn std::error::Error>> {
        Self::new_with_options(
            device,
            queue,
            config,
            camera_bind_group_layout,
            glb_bytes,
            ModelOptions::default(),
        )
    }

    pub fn new_with_options(
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        config: &wgpu::SurfaceConfiguration,
        camera_bind_group_layout: &wgpu::BindGroupLayout,
        glb_bytes: &[u8],
        options: ModelOptions<'_>,
    ) -> Result<Self, Box<dyn std::error::Error>> {
        // Stage timings for the one-time model load. This is a blocking spike on a
        // frame — at startup for the aircraft, on first cockpit entry for the interior —
        // so it is worth being able to read the breakdown off a device's log rather than
        // guessing at it. A handful of `Instant::now` calls per model load, twice a run.
        let t_total = std::time::Instant::now();
        let mut t = std::time::Instant::now();
        let mut stage = |name: &str| {
            log::info!("[loadtime] {:<22} {:>7.1} ms", name, t.elapsed().as_secs_f64() * 1e3);
            t = std::time::Instant::now();
        };

        // Parse the glb using the gltf crate
        let (document, buffers, images) = gltf::import_slice(glb_bytes)?;
        stage("gltf parse");

        let mut vertices = Vec::new();
        let mut indices = Vec::new();

        fn process_node(
            node: gltf::Node,
            parent_transform: glam::Mat4,
            buffers: &[gltf::buffer::Data],
            vertices: &mut Vec<ModelVertex>,
            indices: &mut Vec<u32>,
            options: &ModelOptions<'_>,
        ) {
            let local_transform = glam::Mat4::from_cols_array_2d(&node.transform().matrix());
            let transform = parent_transform * local_transform;
            let origin = glam::Vec3::from_array(options.origin_offset);

            if let Some(mesh) = node.mesh() {
                let mesh_index = mesh.index();
                for primitive in mesh.primitives() {
                    let normal_offset = options
                        .primitive_normal_offset
                        .map(|f| f(mesh_index, primitive.index()))
                        .unwrap_or(0.0);
                    let reader = primitive.reader(|buffer| Some(&buffers[buffer.index()]));

                    let positions: Vec<[f32; 3]> = reader.read_positions().unwrap().collect();
                    let normals: Vec<[f32; 3]> = reader.read_normals().unwrap().collect();
                    let mut tex_coords: Vec<[f32; 2]> = Vec::new();
                    if let Some(read_tex_coords) = reader.read_tex_coords(0) {
                        tex_coords = read_tex_coords.into_f32().collect();
                    }

                    let material = primitive.material();
                    let mut base_color = material.pbr_metallic_roughness().base_color_factor();
                    if let Some(over) = options.material_override {
                        base_color = over(material.name(), base_color);
                    }
                    if base_color[3] < options.skip_alpha_below {
                        continue;
                    }

                    let unlit = options
                        .material_unlit
                        .map(|f| f(material.name()))
                        .unwrap_or(0.0);

                    let mut vertex_colors: Vec<[f32; 4]> = Vec::new();
                    if let Some(read_colors) = reader.read_colors(0) {
                        vertex_colors = read_colors.into_rgba_f32().collect();
                    }

                    let base_index = vertices.len() as u32;

                    for (i, (pos, norm)) in positions.into_iter().zip(normals).enumerate() {
                        let uv = if i < tex_coords.len() {
                            tex_coords[i]
                        } else {
                            [0.0, 0.0]
                        };
                        let uv = match options.uv_override {
                            Some(f) => f(material.name(), uv),
                            None => uv,
                        };
                        let color = if i < vertex_colors.len() {
                            vertex_colors[i]
                        } else {
                            base_color
                        };

                        // Apply node transform
                        let world_pos = transform * glam::Vec4::new(pos[0], pos[1], pos[2], 1.0);

                        // Normal transform (inverse transpose). For uniform scales, we can just use the upper 3x3
                        let normal_matrix = glam::Mat3::from_cols(
                            transform.x_axis.truncate(),
                            transform.y_axis.truncate(),
                            transform.z_axis.truncate(),
                        )
                        .inverse()
                        .transpose();
                        let world_norm = (normal_matrix
                            * glam::Vec3::new(norm[0], norm[1], norm[2]))
                        .normalize();

                        let world_pos = world_pos + world_norm.extend(0.0) * normal_offset;
                        let local_pos = world_pos.truncate() - origin;

                        vertices.push(ModelVertex {
                            position: [local_pos.x, local_pos.y, local_pos.z],
                            normal: [world_norm.x, world_norm.y, world_norm.z],
                            uv,
                            color,
                            unlit,
                        });
                    }

                    if let Some(read_indices) = reader.read_indices() {
                        for i in read_indices.into_u32() {
                            indices.push(base_index + i);
                        }
                    }
                }
            }

            for child in node.children() {
                process_node(child, transform, buffers, vertices, indices, options);
            }
        }

        for scene in document.scenes() {
            for node in scene.nodes() {
                process_node(
                    node,
                    glam::Mat4::IDENTITY,
                    &buffers,
                    &mut vertices,
                    &mut indices,
                    &options,
                );
            }
        }

        // Normalize the entire assembled mesh so it has a radius of exactly 1.0, then
        // apply the caller's corrective scale on top.
        let mut scale = options.post_scale;
        if options.normalize_to_unit_radius {
            let mut max_extent: f32 = 0.0001;
            for v in &vertices {
                let len = (v.position[0] * v.position[0]
                    + v.position[1] * v.position[1]
                    + v.position[2] * v.position[2])
                    .sqrt();
                if len > max_extent {
                    max_extent = len;
                }
            }
            scale /= max_extent;
        }
        if scale != 1.0 {
            for v in &mut vertices {
                v.position[0] *= scale;
                v.position[1] *= scale;
                v.position[2] *= scale;
            }
        }

        stage("mesh walk");
        use wgpu::util::DeviceExt;
        let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Model Vertex Buffer"),
            contents: bytemuck::cast_slice(&vertices),
            usage: wgpu::BufferUsages::VERTEX,
        });

        let index_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Model Index Buffer"),
            contents: bytemuck::cast_slice(&indices),
            usage: wgpu::BufferUsages::INDEX,
        });

        // Pads tightly-packed RGBA8 pixels to wgpu's row alignment, for texture upload.
        fn pad_rgba(width: u32, height: u32, rgba: &[u8]) -> (Vec<u8>, u32) {
            let bytes_per_pixel = 4;
            let unpadded_bytes_per_row = width * bytes_per_pixel;
            let align = wgpu::COPY_BYTES_PER_ROW_ALIGNMENT;
            let padded_bytes_per_row = (unpadded_bytes_per_row + align - 1) & !(align - 1);

            let mut padded_data = vec![0; (padded_bytes_per_row * height) as usize];
            for y in 0..height {
                let src_offset = (y * unpadded_bytes_per_row) as usize;
                let dst_offset = (y * padded_bytes_per_row) as usize;
                padded_data[dst_offset..dst_offset + unpadded_bytes_per_row as usize]
                    .copy_from_slice(&rgba[src_offset..src_offset + unpadded_bytes_per_row as usize]);
            }
            (padded_data, padded_bytes_per_row)
        }

        // Setup Texture. Tightly-packed RGBA8 source pixels, from the caller's override,
        // the GLB's own first image, or a 1x1 white fallback.
        let (tex_width, tex_height, tex_rgba) = if let Some((width, height, rgba)) =
            options.texture_override
        {
            (width, height, rgba)
        } else if let Some(image) = images.first() {
            let width = image.width;
            let height = image.height;
            let rgba = match image.format {
                gltf::image::Format::R8G8B8 => {
                    // Convert RGB to RGBA
                    let mut data = Vec::with_capacity(image.pixels.len() / 3 * 4);
                    for chunk in image.pixels.chunks(3) {
                        data.extend_from_slice(&[chunk[0], chunk[1], chunk[2], 255]);
                    }
                    data
                }
                gltf::image::Format::R8G8B8A8 => image.pixels.clone(),
                _ => vec![255; (width * height * 4) as usize], // Fallback to white
            };
            (width, height, rgba)
        } else {
            // Fallback 1x1 white texture
            (1, 1, vec![255, 255, 255, 255])
        };

        stage("vertex/index buffers");
        let mut mips = build_mip_chain(tex_width, tex_height, tex_rgba);
        log::info!(
            "[loadtime] (texture {}x{}, {} mip levels)",
            tex_width,
            tex_height,
            mips.len()
        );
        stage("mip chain");

        // Discard the levels above the caller's cap, keeping at least the 1x1 tail.
        if let Some(cap) = options.max_texture_size {
            let skip = mips
                .iter()
                .position(|(w, h, _)| *w <= cap && *h <= cap)
                .unwrap_or(mips.len() - 1);
            mips.drain(..skip);
        }

        let (base_width, base_height, _) = &mips[0];
        let texture_size = wgpu::Extent3d {
            width: *base_width,
            height: *base_height,
            depth_or_array_layers: 1,
        };

        let texture = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("Model Texture"),
            size: texture_size,
            mip_level_count: mips.len() as u32,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Rgba8UnormSrgb,
            usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
            view_formats: &[],
        });

        for (level, (width, height, rgba)) in mips.iter().enumerate() {
            let (padded_data, padded_bytes_per_row) = pad_rgba(*width, *height, rgba);
            queue.write_texture(
                wgpu::ImageCopyTexture {
                    texture: &texture,
                    mip_level: level as u32,
                    origin: wgpu::Origin3d::ZERO,
                    aspect: wgpu::TextureAspect::All,
                },
                &padded_data,
                wgpu::ImageDataLayout {
                    offset: 0,
                    bytes_per_row: Some(padded_bytes_per_row),
                    rows_per_image: Some(*height),
                },
                wgpu::Extent3d {
                    width: *width,
                    height: *height,
                    depth_or_array_layers: 1,
                },
            );
        }

        stage("texture upload");
        let texture_view = texture.create_view(&wgpu::TextureViewDescriptor::default());
        let sampler = device.create_sampler(&wgpu::SamplerDescriptor {
            address_mode_u: wgpu::AddressMode::Repeat,
            address_mode_v: wgpu::AddressMode::Repeat,
            address_mode_w: wgpu::AddressMode::Repeat,
            mag_filter: wgpu::FilterMode::Linear,
            min_filter: wgpu::FilterMode::Linear,
            mipmap_filter: wgpu::FilterMode::Linear,
            ..Default::default()
        });

        let bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            entries: &[
                wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Texture {
                        multisampled: false,
                        view_dimension: wgpu::TextureViewDimension::D2,
                        sample_type: wgpu::TextureSampleType::Float { filterable: true },
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 1,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
                    count: None,
                },
            ],
            label: Some("Model Bind Group Layout"),
        });

        let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            layout: &bind_group_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: wgpu::BindingResource::TextureView(&texture_view),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: wgpu::BindingResource::Sampler(&sampler),
                },
            ],
            label: Some("Model Bind Group"),
        });

        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Model Shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("shader.wgsl").into()),
        });

        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("Model Pipeline Layout"),
            bind_group_layouts: &[camera_bind_group_layout, &bind_group_layout],
            push_constant_ranges: &[wgpu::PushConstantRange {
                stages: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT,
                range: 0..std::mem::size_of::<ModelPushConstants>() as u32,
            }],
        });

        let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("Model Pipeline"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: "vs_main",
                buffers: &[ModelVertex::desc()],
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
                topology: wgpu::PrimitiveTopology::TriangleList,
                strip_index_format: None,
                front_face: wgpu::FrontFace::Ccw,
                cull_mode: options.cull_mode,
                unclipped_depth: false,
                polygon_mode: wgpu::PolygonMode::Fill,
                conservative: false,
            },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: wgpu::TextureFormat::Depth32Float,
                depth_write_enabled: true,
                depth_compare: wgpu::CompareFunction::Greater, // Fix depth testing!
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

        stage("pipeline creation");
        log::info!(
            "[loadtime] TOTAL {:>20.1} ms  <- {}",
            t_total.elapsed().as_secs_f64() * 1e3,
            options.label
        );
        println!("{} mesh has {} indices", options.label, indices.len());
        Ok(Self {
            pipeline,
            vertex_buffer,
            index_buffer,
            num_indices: indices.len() as u32,
            bind_group,
        })
    }

    pub fn draw<'a>(
        &'a self,
        render_pass: &mut wgpu::RenderPass<'a>,
        camera_bind_group: &'a wgpu::BindGroup,
        push_constants: ModelPushConstants,
    ) {
        render_pass.set_pipeline(&self.pipeline);
        render_pass.set_bind_group(0, camera_bind_group, &[]);
        render_pass.set_bind_group(1, &self.bind_group, &[]);

        render_pass.set_push_constants(
            wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT,
            0,
            bytemuck::cast_slice(&[push_constants]),
        );

        render_pass.set_vertex_buffer(0, self.vertex_buffer.slice(..));
        render_pass.set_index_buffer(self.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
        render_pass.draw_indexed(0..self.num_indices, 0, 0..1);
    }
}

/// Box-filters `rgba` down to a complete mip chain — `(width, height, pixels)` per level,
/// level 0 being the input — ending at 1x1.
///
/// Without this the aircraft, a 2048² livery drawn a few hundred pixels wide, samples
/// roughly one texel in ten: every panel line and window row crawls and sparkles as it
/// moves. One texture built once at load costs a third more memory and fixes it.
///
/// The averaging runs in linear light rather than on the stored bytes. The texture is
/// uploaded as `Rgba8UnormSrgb`, so those bytes are gamma-encoded, and averaging them
/// directly makes every successive level darker than the one above it. Alpha is already
/// linear and is averaged as-is.
fn build_mip_chain(width: u32, height: u32, rgba: Vec<u8>) -> Vec<(u32, u32, Vec<u8>)> {
    // The forward direction is a 256-entry table, so the per-source-texel cost is a
    // lookup; the inverse is evaluated once per output texel.
    let to_linear: [f32; 256] = std::array::from_fn(|i| {
        let c = i as f32 / 255.0;
        if c <= 0.04045 {
            c / 12.92
        } else {
            ((c + 0.055) / 1.055).powf(2.4)
        }
    });
    fn to_srgb(c: f32) -> u8 {
        let c = c.clamp(0.0, 1.0);
        let s = if c <= 0.0031308 {
            c * 12.92
        } else {
            1.055 * c.powf(1.0 / 2.4) - 0.055
        };
        (s * 255.0 + 0.5) as u8
    }

    let mut chain = vec![(width, height, rgba)];
    loop {
        let (dw, dh, dst) = {
            let (sw, sh, src) = chain.last().expect("chain is never empty");
            let (sw, sh) = (*sw, *sh);
            if sw <= 1 && sh <= 1 {
                break;
            }
            let dw = (sw / 2).max(1);
            let dh = (sh / 2).max(1);

            let mut dst = vec![0u8; (dw * dh * 4) as usize];
            for y in 0..dh {
                // An odd source dimension repeats its last row/column into the 2x2 box
                // rather than reading past the end of the level.
                let y0 = (y * 2).min(sh - 1);
                let y1 = (y * 2 + 1).min(sh - 1);
                for x in 0..dw {
                    let x0 = (x * 2).min(sw - 1);
                    let x1 = (x * 2 + 1).min(sw - 1);

                    let mut acc = [0.0f32; 4];
                    for (sy, sx) in [(y0, x0), (y0, x1), (y1, x0), (y1, x1)] {
                        let i = ((sy * sw + sx) * 4) as usize;
                        acc[0] += to_linear[src[i] as usize];
                        acc[1] += to_linear[src[i + 1] as usize];
                        acc[2] += to_linear[src[i + 2] as usize];
                        acc[3] += src[i + 3] as f32 / 255.0;
                    }

                    let o = ((y * dw + x) * 4) as usize;
                    dst[o] = to_srgb(acc[0] / 4.0);
                    dst[o + 1] = to_srgb(acc[1] / 4.0);
                    dst[o + 2] = to_srgb(acc[2] / 4.0);
                    dst[o + 3] = ((acc[3] / 4.0).clamp(0.0, 1.0) * 255.0 + 0.5) as u8;
                }
            }
            (dw, dh, dst)
        };
        chain.push((dw, dh, dst));
    }
    chain
}
