use std::mem;

#[repr(C)]
#[derive(Debug, Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
pub struct ModelVertex {
    pub position: [f32; 3],
    pub normal: [f32; 3],
    pub uv: [f32; 2],
    pub color: [f32; 4],
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
    pub padding: [f32; 2],
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
        // Parse the glb using the gltf crate
        let (document, buffers, images) = gltf::import_slice(glb_bytes)?;

        let mut vertices = Vec::new();
        let mut indices = Vec::new();

        fn process_node(
            node: gltf::Node,
            parent_transform: glam::Mat4,
            buffers: &[gltf::buffer::Data],
            vertices: &mut Vec<ModelVertex>,
            indices: &mut Vec<u32>,
        ) {
            let local_transform = glam::Mat4::from_cols_array_2d(&node.transform().matrix());
            let transform = parent_transform * local_transform;

            if let Some(mesh) = node.mesh() {
                for primitive in mesh.primitives() {
                    let reader = primitive.reader(|buffer| Some(&buffers[buffer.index()]));
                    
                    let positions: Vec<[f32; 3]> = reader.read_positions().unwrap().collect();
                    let normals: Vec<[f32; 3]> = reader.read_normals().unwrap().collect();
                    let mut tex_coords: Vec<[f32; 2]> = Vec::new();
                    if let Some(read_tex_coords) = reader.read_tex_coords(0) {
                        tex_coords = read_tex_coords.into_f32().collect();
                    }

                    let material = primitive.material();
                    let base_color = material.pbr_metallic_roughness().base_color_factor();

                    let mut vertex_colors: Vec<[f32; 4]> = Vec::new();
                    if let Some(read_colors) = reader.read_colors(0) {
                        vertex_colors = read_colors.into_rgba_f32().collect();
                    }

                    let base_index = vertices.len() as u32;

                    for (i, (pos, norm)) in positions.into_iter().zip(normals.into_iter()).enumerate() {
                        let uv = if i < tex_coords.len() { tex_coords[i] } else { [0.0, 0.0] };
                        let color = if i < vertex_colors.len() { vertex_colors[i] } else { base_color };
                        
                        // Apply node transform
                        let world_pos = transform * glam::Vec4::new(pos[0], pos[1], pos[2], 1.0);
                        
                        // Normal transform (inverse transpose). For uniform scales, we can just use the upper 3x3
                        let normal_matrix = glam::Mat3::from_cols(
                            transform.x_axis.truncate(),
                            transform.y_axis.truncate(),
                            transform.z_axis.truncate(),
                        ).inverse().transpose();
                        let world_norm = (normal_matrix * glam::Vec3::new(norm[0], norm[1], norm[2])).normalize();

                        vertices.push(ModelVertex {
                            position: [world_pos.x, world_pos.y, world_pos.z],
                            normal: [world_norm.x, world_norm.y, world_norm.z],
                            uv,
                            color,
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
                process_node(child, transform, buffers, vertices, indices);
            }
        }

        for scene in document.scenes() {
            for node in scene.nodes() {
                process_node(node, glam::Mat4::IDENTITY, &buffers, &mut vertices, &mut indices);
            }
        }

        // Normalize the entire assembled mesh so it has a radius of exactly 1.0
        let mut max_extent: f32 = 0.0001;
        for v in &vertices {
            let len = (v.position[0]*v.position[0] + v.position[1]*v.position[1] + v.position[2]*v.position[2]).sqrt();
            if len > max_extent {
                max_extent = len;
            }
        }
        for v in &mut vertices {
            v.position[0] /= max_extent;
            v.position[1] /= max_extent;
            v.position[2] /= max_extent;
        }

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

        // Setup Texture
        let (texture_size, padded_data, padded_bytes_per_row, format) = if let Some(image) = images.first() {
            let width = image.width;
            let height = image.height;
            let mut rgba = match image.format {
                gltf::image::Format::R8G8B8 => {
                    // Convert RGB to RGBA
                    let mut data = Vec::with_capacity(image.pixels.len() / 3 * 4);
                    for chunk in image.pixels.chunks(3) {
                        data.extend_from_slice(&[chunk[0], chunk[1], chunk[2], 255]);
                    }
                    data
                },
                gltf::image::Format::R8G8B8A8 => image.pixels.clone(),
                _ => vec![255; (width * height * 4) as usize], // Fallback to white
            };

            let bytes_per_pixel = 4;
            let unpadded_bytes_per_row = width as u32 * bytes_per_pixel;
            let align = wgpu::COPY_BYTES_PER_ROW_ALIGNMENT;
            let padded_bytes_per_row = (unpadded_bytes_per_row + align - 1) & !(align - 1);

            let mut padded_data = vec![0; (padded_bytes_per_row * height as u32) as usize];
            for y in 0..height as u32 {
                let src_offset = (y * unpadded_bytes_per_row) as usize;
                let dst_offset = (y * padded_bytes_per_row) as usize;
                padded_data[dst_offset..dst_offset + unpadded_bytes_per_row as usize]
                    .copy_from_slice(&rgba[src_offset..src_offset + unpadded_bytes_per_row as usize]);
            }

            (
                wgpu::Extent3d { width, height, depth_or_array_layers: 1 }, 
                padded_data, 
                padded_bytes_per_row,
                wgpu::TextureFormat::Rgba8UnormSrgb
            )
        } else {
            // Fallback 1x1 white texture
            let padded_bytes_per_row = 256; // Minimum alignment
            let mut data = vec![0; 256];
            data[0..4].copy_from_slice(&[255, 255, 255, 255]);
            (
                wgpu::Extent3d { width: 1, height: 1, depth_or_array_layers: 1 }, 
                data, 
                padded_bytes_per_row,
                wgpu::TextureFormat::Rgba8UnormSrgb
            )
        };

        let texture = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("Model Texture"),
            size: texture_size,
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format,
            usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
            view_formats: &[],
        });

        queue.write_texture(
            wgpu::ImageCopyTexture {
                texture: &texture,
                mip_level: 0,
                origin: wgpu::Origin3d::ZERO,
                aspect: wgpu::TextureAspect::All,
            },
            &padded_data,
            wgpu::ImageDataLayout {
                offset: 0,
                bytes_per_row: Some(padded_bytes_per_row),
                rows_per_image: Some(texture_size.height),
            },
            texture_size,
        );

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
            source: wgpu::ShaderSource::Wgsl(include_str!("model.wgsl").into()),
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
                cull_mode: Some(wgpu::Face::Back), // Enable backface culling!
                unclipped_depth: false,
                polygon_mode: wgpu::PolygonMode::Fill,
                conservative: false,
            },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: wgpu::TextureFormat::Depth32Float,
                depth_write_enabled: true,
                depth_compare: wgpu::CompareFunction::Less, // Fix depth testing!
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

        println!("A350 mesh has {} indices", indices.len());
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
