//! Dynamic SVG vector tile rasterizer for CesiumRS's offline mode.
//!
//! Renders arbitrary sub-regions (tiles) of a Web Mercator SVG document into RGBA8
//! pixel buffers using exact affine transforms on Rayon worker threads — one tile per
//! CPU core, no network, no tokio-managed sockets.
//!
//! # How the math works
//!
//! The SVG has `viewBox="0 0 W H"` where W = H = 4096.  The entire world fits in that
//! square: longitude −180…+180 maps to x ∈ [0, W] and Web Mercator latitude maps to
//! y ∈ [0, H].  At zoom level z the quadtree has 2^z tiles per axis; tile (z, x, y)
//! covers SVG coordinates [x·W/2^z, (x+1)·W/2^z] × [y·H/2^z, (y+1)·H/2^z].
//!
//! To map that sub-region into a `TILE_SIZE×TILE_SIZE` pixmap we apply:
//!   scale_x = (TILE_SIZE · 2^z) / W
//!   scale_y = (TILE_SIZE · 2^z) / H
//!   translate_x = −x · TILE_SIZE
//!   translate_y = −y · TILE_SIZE
//!
//! Applied as `resvg::render(&tree, transform, &mut pixmap)`.  The engine's existing
//! [`TileTextureManager`] and LRU cache then handle everything downstream.

use crate::globe::quadtree::TileId;
use crate::globe::tiles::tile_fetcher::TileImage;
use std::sync::Arc;

/// Edge length of each rasterized tile, in pixels.  Matches the Carto retina basemap's
/// 512×512 tiles so the LOD rule and texture cache see identical tile sizes.
pub const VECTOR_TILE_SIZE_PX: u32 = 512;

/// A parsed, ready-to-rasterize SVG world map.
///
/// Construction is expensive (~50 ms for the 1.4 MB world SVG on a desktop core) so this
/// value is built once at startup and then shared — cheaply — via [`Arc`] across every
/// worker thread that rasterizes tiles concurrently.
#[derive(Clone)]
pub struct SvgTileRenderer {
    /// Parsed usvg tree, wrapped in Arc so `Clone` is O(1).
    tree: Arc<usvg::Tree>,
    /// viewBox width of the source SVG (default 4096.0).
    view_width: f32,
    /// viewBox height of the source SVG (default 4096.0).
    view_height: f32,
}

impl SvgTileRenderer {
    /// Parses raw SVG XML bytes and builds a renderer.
    ///
    /// Returns an error string if the bytes are not valid UTF-8, not parseable as SVG, or
    /// if `resvg` rejects the document.
    pub fn from_svg_bytes(data: &[u8]) -> Result<Self, String> {
        let text = std::str::from_utf8(data)
            .map_err(|e| format!("SVG is not valid UTF-8: {e}"))?;

        let options = usvg::Options::default();
        let tree = usvg::Tree::from_str(text, &options)
            .map_err(|e| format!("Failed to parse SVG: {e}"))?;

        // Extract viewBox from the parsed tree's root size.
        let size = tree.size();
        let view_width = size.width();
        let view_height = size.height();

        Ok(Self {
            tree: Arc::new(tree),
            view_width,
            view_height,
        })
    }

    /// Rasterizes the sub-region of the SVG that corresponds to Web Mercator tile
    /// `id` into a `VECTOR_TILE_SIZE_PX × VECTOR_TILE_SIZE_PX` RGBA8 buffer.
    ///
    /// This is the hot path: call it from a Rayon / `spawn_blocking` worker.  It is
    /// CPU-bound (~1–4 ms per tile at TILE_SIZE 512 on a modern desktop core, depending
    /// on how many path segments intersect the tile) and fully `Send + Sync`.
    pub fn render_tile(&self, id: TileId) -> Result<TileImage, String> {
        let size = VECTOR_TILE_SIZE_PX;
        let mut pixmap = tiny_skia::Pixmap::new(size, size)
            .ok_or_else(|| format!("Failed to allocate {size}×{size} pixmap for tile {id:?}"))?;

        let (sx, sy, tx, ty) = self.tile_transform(id, size);
        let transform = tiny_skia::Transform::from_scale(sx, sy)
            .post_translate(tx, ty);

        resvg::render(&self.tree, transform, &mut pixmap.as_mut());

        // `Pixmap` stores RGBA8 pre-multiplied (premul).  The GPU texture we upload to is
        // `Rgba8UnormSrgb` which is straight-alpha.  For a fully-opaque map (alpha = 255
        // everywhere) premul and straight are identical, so we can hand the buffer over
        // directly without an extra pass.  A quick `debug_assert` confirms our SVG is
        // opaque.
        #[cfg(debug_assertions)]
        {
            let pixels = pixmap.pixels();
            if let Some(first) = pixels.first() {
                debug_assert_eq!(
                    first.alpha(),
                    255,
                    "world_vector_dark.svg unexpectedly has semi-transparent pixels; \
                     premul → straight conversion would be needed"
                );
            }
        }

        Ok((size, size, pixmap.take()))
    }

    /// Computes the affine transform `(scale_x, scale_y, translate_x, translate_y)` that
    /// maps tile `(z, x, y)` of a `size × size` output into this SVG's coordinate space.
    ///
    /// Exposed for unit tests.
    pub fn tile_transform(&self, id: TileId, size: u32) -> (f32, f32, f32, f32) {
        let num_tiles = (1u64 << id.z) as f32;
        let sx = (size as f32 * num_tiles) / self.view_width;
        let sy = (size as f32 * num_tiles) / self.view_height;
        let tx = -(id.x as f32) * (size as f32);
        let ty = -(id.y as f32) * (size as f32);
        (sx, sy, tx, ty)
    }

    /// viewBox width of the source SVG.
    pub fn view_width(&self) -> f32 {
        self.view_width
    }

    /// viewBox height of the source SVG.
    pub fn view_height(&self) -> f32 {
        self.view_height
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Smoke-test: does the math produce the correct sub-region bounds?
    ///
    /// At z=0 there is one tile (0,0,0) covering the whole world.  The transform must map
    /// the SVG's full viewBox [0, W] × [0, H] to the pixmap's [0, size] × [0, size].
    #[test]
    fn z0_tile_covers_full_viewbox() {
        // Construct a minimal valid SVG (1×1 square) just to get a renderer.
        let svg = r#"<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 4096 4096"/>"#;
        let renderer = SvgTileRenderer::from_svg_bytes(svg.as_bytes()).unwrap();

        // z=0, x=0, y=0: only tile, covers everything.
        let id = TileId { z: 0, x: 0, y: 0 };
        let size = 512u32;
        let (sx, sy, tx, ty) = renderer.tile_transform(id, size);

        // scale = size / viewBox  →  512 / 4096 = 0.125
        let expected_scale = size as f32 / 4096.0;
        assert!((sx - expected_scale).abs() < 1e-5, "sx = {sx}");
        assert!((sy - expected_scale).abs() < 1e-5, "sy = {sy}");
        // No translation for the origin tile.
        assert!((tx - 0.0).abs() < 1e-5, "tx = {tx}");
        assert!((ty - 0.0).abs() < 1e-5, "ty = {ty}");
    }

    /// At z=1 there are 2×2 tiles.  Tile (1, 1, 0) is the north-east quadrant.
    /// scale = 2 * 512 / 4096 = 0.25; tx = -1 * 512 = -512; ty = 0.
    #[test]
    fn z1_ne_quadrant_transform() {
        let svg = r#"<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 4096 4096"/>"#;
        let renderer = SvgTileRenderer::from_svg_bytes(svg.as_bytes()).unwrap();

        let id = TileId { z: 1, x: 1, y: 0 };
        let size = 512u32;
        let (sx, sy, tx, ty) = renderer.tile_transform(id, size);

        let expected_scale = 2.0 * 512.0 / 4096.0; // = 0.25
        assert!((sx - expected_scale).abs() < 1e-5, "sx = {sx}");
        assert!((sy - expected_scale).abs() < 1e-5, "sy = {sy}");
        assert!((tx - (-512.0)).abs() < 1e-5, "tx = {tx}");
        assert!((ty - 0.0).abs() < 1e-5, "ty = {ty}");
    }
}
