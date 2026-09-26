//! Single-line text layout for label names, in pixels at an integer font size.
//!
//! A name is laid out once per pixel size and cached. Metrics follow egui's so the
//! labels keep the size and placement they had when egui drew them: the font size in
//! pixels is rounded to a whole number, a row is `ascent − descent + line_gap` tall,
//! the baseline sits `ascent` below its top, and glyphs advance with pair kerning.

use ab_glyph::{Font, FontRef, PxScale, ScaleFont};
use std::collections::HashMap;

/// A laid-out name, in pixels.
#[derive(Clone, Debug, PartialEq)]
pub struct TextLayout {
    /// Each drawable character and its pen x on the baseline.
    pub glyphs: Vec<(char, f32)>,
    pub width: f32,
    pub row_height: f32,
    pub ascent: f32,
}

pub struct LayoutCache {
    font: FontRef<'static>,
    cache: HashMap<(&'static str, u32), TextLayout>,
}

impl LayoutCache {
    pub fn new() -> Self {
        Self {
            font: super::atlas::font(),
            cache: HashMap::new(),
        }
    }

    pub fn get(&mut self, text: &'static str, px: u32) -> &TextLayout {
        let font = &self.font;
        // Sizes drift with the camera; keep only what a screenful of names can use.
        if self.cache.len() > 4096 {
            self.cache.clear();
        }
        self.cache.entry((text, px)).or_insert_with(|| layout(font, text, px))
    }
}

impl Default for LayoutCache {
    fn default() -> Self {
        Self::new()
    }
}

/// Lays `text` out on one line with `font` at `px` pixels. Public for the tests in
/// `src/testing/`.
pub fn layout(font: &FontRef<'static>, text: &str, px: u32) -> TextLayout {
    let scaled = font.as_scaled(PxScale::from(px as f32));

    let mut glyphs = Vec::with_capacity(text.len());
    let mut pen = 0.0f32;
    let mut prev = None;
    for c in text.chars() {
        let id = font.glyph_id(c);
        if let Some(p) = prev {
            pen += scaled.kern(p, id);
        }
        if !c.is_whitespace() {
            glyphs.push((c, pen));
        }
        pen += scaled.h_advance(id);
        prev = Some(id);
    }

    TextLayout {
        glyphs,
        width: pen,
        row_height: scaled.ascent() - scaled.descent() + scaled.line_gap(),
        ascent: scaled.ascent(),
    }
}
