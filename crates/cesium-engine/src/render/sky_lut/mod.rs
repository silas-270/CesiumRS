//! The sky LUT: the atmosphere of `render/atmosphere.wgsl`, evaluated into a small texture
//! that the sky and the globe sample instead of raymarching per pixel or per vertex.
//!
//! The LUT depends on only two things — the sun's elevation and the camera's height — so
//! it is re-rendered only when one of them has moved enough to show, which during a flight
//! is a small fraction of frames, and never at all when nothing moves; and each re-render
//! is spread over `BANDS` frames. One re-render is 128x98 texels, about 0.6% of a 1080p
//! frame's pixels but ~16 raymarch steps each. Texture layout: see the bottom of
//! atmosphere.wgsl.

pub const LUT_WIDTH: u32 = 128;
pub const LUT_HEIGHT: u32 = 98;
const LUT_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Rgba16Float;

/// Re-render thresholds. Sine of the sun's elevation (1e-3 is 0.06 degrees — far below
/// anything visible in the sky's colour), and the camera's height as a fraction of itself
/// (with a floor, so taxiing does not count). At a real flight's pace the sun crosses the
/// sun threshold about once every 40 frames.
const SUN_EPSILON: f32 = 1.0e-3;
const HEIGHT_FRACTION: f32 = 0.03;
const HEIGHT_FLOOR_MM: f32 = 0.0003;

/// A re-render is spread over this many frames, one horizontal band of the LUT each, so
/// no single frame pays for the whole texture (about 0.4ms on an integrated Vega).
const BANDS: u32 = 4;

pub struct SkyLut {
    pipeline: wgpu::RenderPipeline,
    view: wgpu::TextureView,
    /// The camera uniform alone, for the LUT pass: the main camera bind group cannot be
    /// used while the LUT is the render target.
    pass_bind_group: wgpu::BindGroup,
    /// What the sky and globe pipelines bind to read the LUT.
    pub layout: wgpu::BindGroupLayout,
    pub bind_group: wgpu::BindGroup,
    last_key: Option<(f32, f32)>,
    /// Bands still to render for the current re-render (next one is `BANDS - pending`).
    pending: u32,
}

impl SkyLut {
    pub fn new(device: &wgpu::Device, camera_buffer: &wgpu::Buffer) -> Self {
        let texture = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("Sky LUT"),
            size: wgpu::Extent3d { width: LUT_WIDTH, height: LUT_HEIGHT, depth_or_array_layers: 1 },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: LUT_FORMAT,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING,
            view_formats: &[],
        });
        let view = texture.create_view(&wgpu::TextureViewDescriptor::default());
        let sampler = device.create_sampler(&wgpu::SamplerDescriptor {
            label: Some("Sky LUT sampler"),
            address_mode_u: wgpu::AddressMode::ClampToEdge,
            address_mode_v: wgpu::AddressMode::ClampToEdge,
            mag_filter: wgpu::FilterMode::Linear,
            min_filter: wgpu::FilterMode::Linear,
            ..Default::default()
        });

        let layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("Sky LUT layout"),
            entries: &[
                wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Texture {
                        sample_type: wgpu::TextureSampleType::Float { filterable: true },
                        view_dimension: wgpu::TextureViewDimension::D2,
                        multisampled: false,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 1,
                    visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
                    count: None,
                },
            ],
        });
        let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("Sky LUT bind group"),
            layout: &layout,
            entries: &[
                wgpu::BindGroupEntry { binding: 0, resource: wgpu::BindingResource::TextureView(&view) },
                wgpu::BindGroupEntry { binding: 1, resource: wgpu::BindingResource::Sampler(&sampler) },
            ],
        });

        let pass_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("Sky LUT pass layout"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            }],
        });
        let pass_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("Sky LUT pass bind group"),
            layout: &pass_layout,
            entries: &[wgpu::BindGroupEntry { binding: 0, resource: camera_buffer.as_entire_binding() }],
        });

        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Sky LUT shader"),
            source: wgpu::ShaderSource::Wgsl(
                concat!(include_str!("../atmosphere.wgsl"), include_str!("sky_lut.wgsl")).into(),
            ),
        });
        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("Sky LUT pipeline layout"),
            bind_group_layouts: &[&pass_layout],
            push_constant_ranges: &[],
        });
        let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("Sky LUT pipeline"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: "vs_lut",
                buffers: &[],
                compilation_options: Default::default(),
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: "fs_lut",
                targets: &[Some(wgpu::ColorTargetState {
                    format: LUT_FORMAT,
                    blend: None,
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: Default::default(),
            }),
            primitive: wgpu::PrimitiveState::default(),
            depth_stencil: None,
            multisample: wgpu::MultisampleState::default(),
            multiview: None,
            cache: None,
        });

        Self { pipeline, view, pass_bind_group, layout, bind_group, last_key: None, pending: 0 }
    }

    /// Re-renders the LUT into `encoder` if the sun or the camera height has moved past
    /// the thresholds since the last time. `sun_elevation` is the sine the camera uniform
    /// carries; `camera_height_mm` is the camera's height above the ellipsoid.
    /// The camera uniform must already hold this frame's values (it is written with
    /// `queue.write_buffer`, which lands before this encoder executes).
    pub fn update(&mut self, encoder: &mut wgpu::CommandEncoder, sun_elevation: f32, camera_height_mm: f32) {
        let first = self.last_key.is_none();
        let moved = match self.last_key {
            None => true,
            Some((sun, height)) => {
                let height_tol = (height.abs() * HEIGHT_FRACTION).max(HEIGHT_FLOOR_MM);
                (sun - sun_elevation).abs() >= SUN_EPSILON || (height - camera_height_mm).abs() >= height_tol
            }
        };
        // A new re-render only starts once the previous one has finished, so a sun that
        // moves every frame still costs one band per frame at most.
        if moved && self.pending == 0 {
            self.last_key = Some((sun_elevation, camera_height_mm));
            self.pending = BANDS;
        }
        if self.pending == 0 {
            return;
        }

        // The very first time, the whole texture at once: there is nothing to show yet.
        let (band_start, band_count) = if first { (0, BANDS) } else { (BANDS - self.pending, 1) };
        self.pending -= band_count;
        let rows_per_band = LUT_HEIGHT.div_ceil(BANDS);
        let y0 = band_start * rows_per_band;
        let y1 = ((band_start + band_count) * rows_per_band).min(LUT_HEIGHT);

        let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("Sky LUT pass"),
            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                view: &self.view,
                resolve_target: None,
                ops: wgpu::Operations { load: wgpu::LoadOp::Load, store: wgpu::StoreOp::Store },
            })],
            depth_stencil_attachment: None,
            occlusion_query_set: None,
            timestamp_writes: None,
        });
        pass.set_pipeline(&self.pipeline);
        pass.set_bind_group(0, &self.pass_bind_group, &[]);
        pass.set_scissor_rect(0, y0, LUT_WIDTH, y1 - y0);
        pass.draw(0..3, 0..1);
    }
}
