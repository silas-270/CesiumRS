//! The glyph atlas the labels are drawn from.
//!
//! Glyphs are rasterized on demand at the exact integer pixel size they are drawn at,
//! and drawn 1:1 on whole pixels. That is how egui drew these labels before, and the
//! only way to match its weight: a single large raster minified through mips blurs the
//! strokes, and egui's coverage gamma then thickens the blur into visibly bolder text.
//!
//! Font sizes change continuously with camera distance, so an entry is keyed by
//! `(char, px)`. Only the handful of names on screen at once are ever in the atlas; when
//! it fills up it is simply emptied and refilled on demand.
//!
//! The font is Ubuntu-Light, egui's default proportional face.

use ab_glyph::{Font, FontRef, PxScale, ScaleFont};
use std::collections::HashMap;

pub const ATLAS_SIZE: u32 = 1024;
/// Empty texels around each glyph, so bilinear filtering never reads a neighbour.
const PADDING: u32 = 1;

/// Where one glyph sits in the atlas and how it sits on the baseline.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GlyphEntry {
    pub uv_min: [f32; 2],
    pub uv_max: [f32; 2],
    /// Top-left of the bitmap relative to the pen position on the baseline, in whole
    /// pixels (y down).
    pub offset: [f32; 2],
    /// Bitmap size in pixels.
    pub size: [f32; 2],
}

pub fn font() -> FontRef<'static> {
    FontRef::try_from_slice(epaint_default_fonts::UBUNTU_LIGHT).expect("bundled font parses")
}

/// A glyph's coverage bitmap at one pixel size: `(offset, width, height, coverage)`.
pub fn rasterize(font: &FontRef<'static>, c: char, px: u32) -> Option<([f32; 2], u32, u32, Vec<u8>)> {
    let id = font.glyph_id(c);
    if id.0 == 0 || c.is_whitespace() {
        return None;
    }
    let scale = PxScale::from(px as f32);
    let g = font.as_scaled(scale).outline_glyph(id.with_scale(scale))?;
    let b = g.px_bounds();
    let (w, h) = (b.width() as u32, b.height() as u32);
    let mut cov = vec![0u8; (w * h) as usize];
    g.draw(|x, y, v| {
        if x < w && y < h {
            cov[(y * w + x) as usize] = (v.clamp(0.0, 1.0) * 255.0 + 0.5) as u8;
        }
    });
    Some(([b.min.x, b.min.y], w, h, cov))
}

pub struct GlyphAtlas {
    font: FontRef<'static>,
    pub texture: wgpu::Texture,
    entries: HashMap<(char, u32), Option<GlyphEntry>>,
    cursor: (u32, u32),
    shelf_h: u32,
}

/// What [`GlyphAtlas::get`] did.
pub enum Lookup {
    Found(Option<GlyphEntry>),
    /// The atlas was full and has been emptied: every entry handed out before is stale.
    Reset,
}

impl GlyphAtlas {
    pub fn new(device: &wgpu::Device) -> Self {
        let texture = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("Label Atlas"),
            size: wgpu::Extent3d {
                width: ATLAS_SIZE,
                height: ATLAS_SIZE,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::R8Unorm,
            usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
            view_formats: &[],
        });
        Self {
            font: font(),
            texture,
            entries: HashMap::new(),
            cursor: (0, 0),
            shelf_h: 0,
        }
    }

    /// The glyph for `c` at `px`, rasterizing and uploading it the first time.
    /// `Found(None)` is a character with nothing to draw.
    pub fn get(&mut self, queue: &wgpu::Queue, c: char, px: u32) -> Lookup {
        if let Some(e) = self.entries.get(&(c, px)) {
            return Lookup::Found(*e);
        }
        let Some((offset, w, h, cov)) = rasterize(&self.font, c, px) else {
            self.entries.insert((c, px), None);
            return Lookup::Found(None);
        };

        let (pw, ph) = (w + 2 * PADDING, h + 2 * PADDING);
        if self.cursor.0 + pw > ATLAS_SIZE {
            self.cursor = (0, self.cursor.1 + self.shelf_h);
            self.shelf_h = 0;
        }
        if self.cursor.1 + ph > ATLAS_SIZE {
            self.entries.clear();
            self.cursor = (0, 0);
            self.shelf_h = 0;
            return Lookup::Reset;
        }
        let (x, y) = self.cursor;
        self.cursor.0 += pw;
        self.shelf_h = self.shelf_h.max(ph);

        // Upload the padding too, so whatever an earlier fill left there is cleared.
        let mut padded = vec![0u8; (pw * ph) as usize];
        for row in 0..h {
            let dst = ((row + PADDING) * pw + PADDING) as usize;
            padded[dst..dst + w as usize].copy_from_slice(&cov[(row * w) as usize..((row + 1) * w) as usize]);
        }
        queue.write_texture(
            wgpu::ImageCopyTexture {
                texture: &self.texture,
                mip_level: 0,
                origin: wgpu::Origin3d { x, y, z: 0 },
                aspect: wgpu::TextureAspect::All,
            },
            &padded,
            wgpu::ImageDataLayout {
                offset: 0,
                bytes_per_row: Some(pw),
                rows_per_image: Some(ph),
            },
            wgpu::Extent3d {
                width: pw,
                height: ph,
                depth_or_array_layers: 1,
            },
        );

        let inv = 1.0 / ATLAS_SIZE as f32;
        let (gx, gy) = (x + PADDING, y + PADDING);
        let entry = GlyphEntry {
            uv_min: [gx as f32 * inv, gy as f32 * inv],
            uv_max: [(gx + w) as f32 * inv, (gy + h) as f32 * inv],
            offset,
            size: [w as f32, h as f32],
        };
        self.entries.insert((c, px), Some(entry));
        Lookup::Found(Some(entry))
    }
}
