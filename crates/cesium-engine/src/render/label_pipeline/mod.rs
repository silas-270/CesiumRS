//! Draws the city labels inside the scene render pass.
//!
//! The labels used to be an egui overlay painted after the whole scene, which put them
//! on top of everything, the aircraft and the cockpit included. Here they are ordinary
//! geometry in the scene pass, drawn after the world layer and before the extension's
//! foreground models. The rules this implements, for every camera mode:
//!
//! 1. Which labels are visible is decided by [`crate::label::LabelManager::update`]
//!    (rank, distance, horizon, frustum), unchanged. Terrain relief does not hide them.
//! 2. Labels cover the world layer: globe, sky, route ribbon, debug geometry.
//! 3. A label is drawn at its anchor's depth, so a 3D model drawn after it (aircraft,
//!    cockpit interior, cockpit screens) covers it wherever the model is nearer than the
//!    anchor, and is covered by it where it is farther.
//! 4. Labels are drawn far to near, so a nearer label covers a farther one.
//! 5. A label whose anchor is behind the near plane or off screen is not drawn at all.

pub mod atlas;
pub mod layout;
pub mod pipeline;

use glam::{DVec3, Mat4, Vec3};

use crate::label::style::{self, LabelStyle, StyleFrame};
use crate::label::VisibleLabel;
use atlas::{GlyphAtlas, Lookup};
use layout::LayoutCache;
use pipeline::{LabelGpu, LabelInstance, LabelUniform, KIND_DOT, KIND_GLYPH, KIND_PILL};

/// Everything one frame of labels is built from.
pub struct LabelFrameInput<'a> {
    pub labels: &'a [VisibleLabel],
    /// The position every other draw in the pass is relative to (the debug camera's
    /// when it is active), in ECEF megametres.
    pub camera_pos_f64: DVec3,
    /// The camera-relative view-projection the camera uniform carries this frame.
    pub view_proj: Mat4,
    /// Size and fade are judged from the flight camera, as they always were.
    pub style: StyleFrame,
    pub pixels_per_point: f32,
    pub viewport_px: [f32; 2],
    pub show_anchor_dots: bool,
}

pub struct LabelRenderer {
    gpu: LabelGpu,
    atlas: GlyphAtlas,
    layouts: LayoutCache,
    target_is_srgb: bool,
    instances: Vec<LabelInstance>,
    buffer: Option<wgpu::Buffer>,
    capacity: usize,
    count: u32,
}

impl LabelRenderer {
    pub fn new(
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        color_format: wgpu::TextureFormat,
        camera_bind_group_layout: &wgpu::BindGroupLayout,
    ) -> Self {
        let atlas = GlyphAtlas::new(device);
        let gpu = LabelGpu::new(device, queue, color_format, camera_bind_group_layout, &atlas.texture);
        Self {
            gpu,
            atlas,
            layouts: LayoutCache::new(),
            target_is_srgb: color_format.is_srgb(),
            instances: Vec::new(),
            buffer: None,
            capacity: 0,
            count: 0,
        }
    }

    /// Builds and uploads this frame's instances. Call before the render pass opens.
    pub fn prepare(&mut self, device: &wgpu::Device, queue: &wgpu::Queue, input: &LabelFrameInput) {
        // Rule 5 and the style cut, then far to near (rule 4).
        let mut drawn: Vec<(f32, &VisibleLabel, Vec3, LabelStyle)> = input
            .labels
            .iter()
            .filter_map(|label| {
                let rel = label.ecef_pos.as_dvec3() - input.camera_pos_f64;
                let rel = rel.as_vec3();
                let clip = input.view_proj * rel.extend(1.0);
                if clip.w <= 0.0 {
                    return None;
                }
                let (nx, ny) = (clip.x / clip.w, clip.y / clip.w);
                if !(-1.0..=1.0).contains(&nx) || !(-1.0..=1.0).contains(&ny) {
                    return None;
                }
                let s = style::label_style(&input.style, label.ecef_pos, label.label_rank)?;
                Some((rel.length(), label, rel, s))
            })
            .collect();
        drawn.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap_or(std::cmp::Ordering::Equal));

        // A full atlas empties itself mid-frame, which leaves the instances already
        // built pointing at glyphs that are gone: start over once. A screenful of names
        // always fits an empty 1024² atlas, so a second reset cannot happen.
        for _ in 0..2 {
            if self.build_instances(queue, &drawn, input) {
                break;
            }
        }

        self.count = self.instances.len() as u32;
        if self.instances.is_empty() {
            return;
        }
        if self.instances.len() > self.capacity {
            self.capacity = self.instances.len().next_power_of_two();
            self.buffer = Some(device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("Label Instances"),
                size: (self.capacity * std::mem::size_of::<LabelInstance>()) as u64,
                usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            }));
        }
        let buffer = self.buffer.as_ref().expect("allocated above");
        queue.write_buffer(buffer, 0, bytemuck::cast_slice(&self.instances));
        queue.write_buffer(
            &self.gpu.uniform_buffer,
            0,
            bytemuck::cast_slice(&[LabelUniform {
                viewport_px: input.viewport_px,
                target_is_srgb: if self.target_is_srgb { 1.0 } else { 0.0 },
                _pad: 0.0,
            }]),
        );
    }

    /// Fills `self.instances`; `false` when the atlas reset partway through.
    fn build_instances(
        &mut self,
        queue: &wgpu::Queue,
        drawn: &[(f32, &VisibleLabel, Vec3, LabelStyle)],
        input: &LabelFrameInput,
    ) -> bool {
        self.instances.clear();
        let ppp = input.pixels_per_point;
        for (_, label, anchor, s) in drawn {
            let anchor = anchor.to_array();
            // Whole pixels, as egui rasterized text.
            let font_px = (s.font_pt * ppp).round().max(1.0) as u32;
            let text = self.layouts.get(label.name, font_px);
            let (pad_x, pad_y) = (style::PAD_X_PT * ppp, style::PAD_Y_PT * ppp);
            let pill_w = text.width + 2.0 * pad_x;
            let pill_h = text.row_height + 2.0 * pad_y;
            let dot_r = s.dot_radius_pt * ppp;
            let dot_offset = if input.show_anchor_dots { dot_r } else { 0.0 };
            let pill_y1 = -(dot_offset + style::DOT_GAP_PT * ppp);
            let (pill_x0, pill_y0) = (-0.5 * pill_w, pill_y1 - pill_h);

            let [r, g, b] = style::PILL_RGB.map(|c| c as f32 / 255.0);
            self.instances.push(LabelInstance {
                anchor,
                kind: KIND_PILL,
                rect: [pill_x0, pill_y0, pill_x0 + pill_w, pill_y1],
                uv: [0.0; 4],
                color: [r, g, b, s.bg_alpha as f32 / 255.0],
                radius: style::PILL_ROUNDING_PT * ppp,
            });

            let text_alpha = s.text_alpha as f32 / 255.0;
            let baseline = pill_y0 + pad_y + text.ascent;
            for &(c, pen) in &text.glyphs {
                let e = match self.atlas.get(queue, c, font_px) {
                    Lookup::Found(Some(e)) => e,
                    Lookup::Found(None) => continue,
                    Lookup::Reset => return false,
                };
                // Snapped to whole pixels so the bitmap lands 1:1; the anchor itself is
                // snapped in the vertex shader.
                let x0 = (pill_x0 + pad_x + pen + e.offset[0]).round();
                let y0 = (baseline + e.offset[1]).round();
                self.instances.push(LabelInstance {
                    anchor,
                    kind: KIND_GLYPH,
                    rect: [x0, y0, x0 + e.size[0], y0 + e.size[1]],
                    uv: [e.uv_min[0], e.uv_min[1], e.uv_max[0], e.uv_max[1]],
                    color: [1.0, 1.0, 1.0, text_alpha],
                    radius: 0.0,
                });
            }

            if input.show_anchor_dots {
                for (radius, color) in [
                    (dot_r + 0.5 * ppp, [0.0, 0.0, 0.0, s.shadow_alpha as f32 / 255.0]),
                    (dot_r, [1.0, 1.0, 1.0, text_alpha]),
                ] {
                    let e = radius + 1.0;
                    self.instances.push(LabelInstance {
                        anchor,
                        kind: KIND_DOT,
                        rect: [-e, -e, e, e],
                        uv: [0.0; 4],
                        color,
                        radius,
                    });
                }
            }
        }
        true
    }

    /// Instances built by the last [`Self::prepare`].
    pub fn instance_count(&self) -> u32 {
        self.count
    }

    pub fn draw<'a>(&'a self, render_pass: &mut wgpu::RenderPass<'a>, camera_bind_group: &'a wgpu::BindGroup) {
        let Some(buffer) = &self.buffer else { return };
        if self.count == 0 {
            return;
        }
        render_pass.set_pipeline(&self.gpu.pipeline);
        render_pass.set_bind_group(0, camera_bind_group, &[]);
        render_pass.set_bind_group(1, &self.gpu.bind_group, &[]);
        render_pass.set_vertex_buffer(0, buffer.slice(..));
        render_pass.draw(0..4, 0..self.count);
    }
}
