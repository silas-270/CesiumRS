//! Dynamic SVG vector tile rasterizer for CesiumRS's offline mode.
//!
//! Renders Web Mercator tiles of an SVG world map into RGBA8 pixel buffers on blocking
//! worker threads — no network, no tokio-managed sockets.
//!
//! # How the math works
//!
//! The SVG has `viewBox="0 0 W H"` (W = H; the bundled map uses 2^22 with integer
//! coordinates, exact in f32). The entire world fits in that square: longitude
//! −180…+180 maps to x ∈ [0, W] and Web Mercator latitude maps to y ∈ [0, H]. At zoom
//! level z the quadtree has 2^z tiles per axis; tile (z, x, y) covers SVG coordinates
//! [x·W/2^z, (x+1)·W/2^z] × [y·H/2^z, (y+1)·H/2^z], mapped to pixels by
//!   scale = (TILE_SIZE · 2^z) / W,   translate = −(x, y) · TILE_SIZE.
//!
//! # Map conventions (not plain SVG)
//!
//! A map is not a picture scaled up, so the renderer departs from SVG in three ways
//! (`tools/generate_world_svg.py` writes the bundled map to match):
//! - **Stroke widths and dash lengths are output pixels**, not SVG units: a 1 px border
//!   stays 1 px at every zoom, instead of growing to ~100 px at z10.
//! - A path whose `id` ends in **`.z<N>`** is only drawn on tiles of zoom ≥ N.
//! - Only solid colours are drawn (gradients and patterns are skipped); text and images
//!   are ignored.
//!
//! # Why not `resvg::render`
//!
//! Besides the stroke widths, resvg walks and rasterizes every path of the document for
//! every tile. Here the document is parsed once into flat runs (one line or ring each)
//! with bounding boxes; a tile only touches the runs that overlap it, clipped to the
//! tile before stroking, so a z14 tile does not stroke (and dash) a whole continent's
//! coastline off-screen.

use crate::globe::quadtree::TileId;
use crate::globe::tiles::tile_fetcher::TileImage;
use std::sync::Arc;
use tiny_skia::{FillRule, Paint, PathBuilder, Pixmap, Stroke, StrokeDash, Transform};

/// Edge length of each rasterized tile, in pixels.  Matches the Carto retina basemap's
/// 512×512 tiles so the LOD rule and texture cache see identical tile sizes.
pub const VECTOR_TILE_SIZE_PX: u32 = 512;

/// How far past the tile edge geometry is kept when clipping, in pixels: enough that a
/// stroke's width or round cap is never cut at the seam.
const CLIP_MARGIN_PX: f64 = 8.0;

/// Vertices closer than this (pixels, per axis) to the previous one are skipped.
const THIN_PX: f64 = 0.5;

/// Segments a quadratic or cubic curve is flattened into.
const CURVE_STEPS: usize = 8;

/// One `MoveTo…` sub-path, flattened to straight segments.
struct Run {
    /// `[min_x, min_y, max_x, max_y]` in SVG units.
    bbox: [f32; 4],
    start: u32,
    len: u32,
    closed: bool,
}

struct StrokeStyle {
    color: tiny_skia::Color,
    width_px: f32,
    dash_px: Option<Vec<f32>>,
    cap: tiny_skia::LineCap,
    join: tiny_skia::LineJoin,
}

/// One visible SVG `<path>` (or shape), ready to draw.
struct Layer {
    min_zoom: u8,
    fill: Option<(tiny_skia::Color, FillRule)>,
    stroke: Option<StrokeStyle>,
    runs: Vec<Run>,
    points: Vec<[f32; 2]>,
}

/// A parsed, ready-to-rasterize SVG world map.
///
/// Construction is expensive (the bundled 10m map is ~17 MB of SVG) so this value is
/// built once and then shared — cheaply — via [`Arc`] across every worker thread that
/// rasterizes tiles concurrently.
#[derive(Clone)]
pub struct SvgTileRenderer {
    layers: Arc<Vec<Layer>>,
    /// viewBox width of the source SVG.
    view_width: f32,
    /// viewBox height of the source SVG.
    view_height: f32,
}

impl SvgTileRenderer {
    /// Parses SVG XML bytes — or gzipped SVG, detected by its magic number — and builds
    /// a renderer.
    ///
    /// Returns an error string if the bytes do not decompress, are not valid UTF-8, or
    /// are not parseable as SVG.
    pub fn from_svg_bytes(data: &[u8]) -> Result<Self, String> {
        let unzipped;
        let data = if data.starts_with(&[0x1f, 0x8b]) {
            let mut out = Vec::new();
            std::io::Read::read_to_end(&mut flate2::read::GzDecoder::new(data), &mut out)
                .map_err(|e| format!("SVG is not valid gzip: {e}"))?;
            unzipped = out;
            &unzipped[..]
        } else {
            data
        };
        let text = std::str::from_utf8(data)
            .map_err(|e| format!("SVG is not valid UTF-8: {e}"))?;

        let options = usvg::Options::default();
        let tree = usvg::Tree::from_str(text, &options)
            .map_err(|e| format!("Failed to parse SVG: {e}"))?;

        let mut layers = Vec::new();
        collect_layers(tree.root(), &mut layers);

        let size = tree.size();
        Ok(Self {
            layers: Arc::new(layers),
            view_width: size.width(),
            view_height: size.height(),
        })
    }

    /// Rasterizes Web Mercator tile `id` into a `VECTOR_TILE_SIZE_PX ×
    /// VECTOR_TILE_SIZE_PX` RGBA8 buffer.
    ///
    /// This is the hot path: call it from a blocking worker. It is CPU-bound and fully
    /// `Send + Sync`.
    pub fn render_tile(&self, id: TileId) -> Result<TileImage, String> {
        let size = VECTOR_TILE_SIZE_PX;
        let mut pixmap = Pixmap::new(size, size)
            .ok_or_else(|| format!("Failed to allocate {size}×{size} pixmap for tile {id:?}"))?;

        let n = (1u64 << id.z) as f64;
        let sx = size as f64 * n / self.view_width as f64;
        let sy = size as f64 * n / self.view_height as f64;
        let tx = id.x as f64 * size as f64;
        let ty = id.y as f64 * size as f64;
        let to_px = |p: [f32; 2]| [p[0] as f64 * sx - tx, p[1] as f64 * sy - ty];

        let lo = -CLIP_MARGIN_PX;
        let hi = size as f64 + CLIP_MARGIN_PX;
        let clip = [lo, lo, hi, hi];
        // The clip rectangle in SVG units, for culling runs by bounding box.
        let view = [
            ((tx + lo) / sx) as f32,
            ((ty + lo) / sy) as f32,
            ((tx + hi) / sx) as f32,
            ((ty + hi) / sy) as f32,
        ];

        for layer in self.layers.iter().filter(|l| l.min_zoom as u32 <= id.z as u32) {
            let runs = || {
                layer.runs.iter().filter(|r| {
                    r.bbox[0] <= view[2]
                        && r.bbox[2] >= view[0]
                        && r.bbox[1] <= view[3]
                        && r.bbox[3] >= view[1]
                })
            };

            if let Some((color, rule)) = layer.fill {
                let mut pb = PathBuilder::new();
                for run in runs().filter(|r| r.closed) {
                    let ring = thin(layer.run_points(run).iter().map(|&p| to_px(p)));
                    let ring = clip_polygon(ring, clip);
                    if ring.len() >= 3 {
                        pb.move_to(ring[0][0] as f32, ring[0][1] as f32);
                        for p in &ring[1..] {
                            pb.line_to(p[0] as f32, p[1] as f32);
                        }
                        pb.close();
                    }
                }
                if let Some(path) = pb.finish() {
                    let mut paint = Paint::default();
                    paint.set_color(color);
                    paint.anti_alias = true;
                    pixmap.fill_path(&path, &paint, rule, Transform::identity(), None);
                }
            }

            if let Some(style) = &layer.stroke {
                let mut paint = Paint::default();
                paint.set_color(style.color);
                paint.anti_alias = true;
                let mut stroke = Stroke {
                    width: style.width_px,
                    line_cap: style.cap,
                    line_join: style.join,
                    ..Stroke::default()
                };
                // Undashed pieces share one path; dashed ones are stroked one by one so
                // each starts at its own phase and the dashes line up across tile seams.
                let mut pb = PathBuilder::new();
                for run in runs() {
                    let pts = layer.run_points(run);
                    let first = if run.closed { pts.last() } else { None };
                    let line = thin(first.into_iter().chain(pts).map(|&p| to_px(p)));
                    for piece in clip_polyline(line.into_iter(), clip) {
                        match &style.dash_px {
                            None => append_polyline(&mut pb, &piece.points),
                            Some(dash) => {
                                let mut one = PathBuilder::new();
                                append_polyline(&mut one, &piece.points);
                                stroke.dash = StrokeDash::new(dash.clone(), piece.start_len as f32);
                                if let Some(path) = one.finish() {
                                    pixmap.stroke_path(&path, &paint, &stroke, Transform::identity(), None);
                                }
                            }
                        }
                    }
                }
                if let Some(path) = pb.finish() {
                    pixmap.stroke_path(&path, &paint, &stroke, Transform::identity(), None);
                }
            }
        }

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
                    "world map SVG unexpectedly has semi-transparent pixels; \
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

    /// Number of drawable paths and of their vertices — for tests and load logging.
    pub fn stats(&self) -> (usize, usize) {
        (
            self.layers.len(),
            self.layers.iter().map(|l| l.points.len()).sum(),
        )
    }
}

impl Layer {
    fn run_points(&self, run: &Run) -> &[[f32; 2]] {
        &self.points[run.start as usize..(run.start + run.len) as usize]
    }
}

/// `.z<N>` suffix of a path id → minimum zoom; anything else draws at every zoom.
fn min_zoom_from_id(id: &str) -> u8 {
    id.rsplit_once(".z")
        .and_then(|(_, n)| n.parse().ok())
        .unwrap_or(0)
}

fn solid(paint: &usvg::Paint, opacity: f32) -> Option<tiny_skia::Color> {
    match paint {
        usvg::Paint::Color(c) => Some(tiny_skia::Color::from_rgba8(
            c.red,
            c.green,
            c.blue,
            (opacity.clamp(0.0, 1.0) * 255.0).round() as u8,
        )),
        _ => {
            log::warn!("Offline map: gradient/pattern paint is not supported, skipped");
            None
        }
    }
}

fn collect_layers(group: &usvg::Group, out: &mut Vec<Layer>) {
    for node in group.children() {
        match node {
            usvg::Node::Group(g) => collect_layers(g, out),
            usvg::Node::Path(p) if p.is_visible() => {
                if let Some(layer) = layer_from_path(p) {
                    out.push(layer);
                }
            }
            _ => {}
        }
    }
}

fn layer_from_path(path: &usvg::Path) -> Option<Layer> {
    let fill = path.fill().and_then(|f| {
        let rule = match f.rule() {
            usvg::FillRule::EvenOdd => FillRule::EvenOdd,
            usvg::FillRule::NonZero => FillRule::Winding,
        };
        solid(f.paint(), f.opacity().get()).map(|c| (c, rule))
    });
    let stroke = path.stroke().and_then(|s| {
        Some(StrokeStyle {
            color: solid(s.paint(), s.opacity().get())?,
            width_px: s.width().get(),
            dash_px: s.dasharray().map(|d| d.to_vec()),
            cap: match s.linecap() {
                usvg::LineCap::Butt => tiny_skia::LineCap::Butt,
                usvg::LineCap::Round => tiny_skia::LineCap::Round,
                usvg::LineCap::Square => tiny_skia::LineCap::Square,
            },
            join: match s.linejoin() {
                usvg::LineJoin::Round => tiny_skia::LineJoin::Round,
                usvg::LineJoin::Bevel => tiny_skia::LineJoin::Bevel,
                _ => tiny_skia::LineJoin::Miter,
            },
        })
    });
    if fill.is_none() && stroke.is_none() {
        return None;
    }

    let ts = path.abs_transform();
    let map = |p: tiny_skia::Point| {
        let mut q = [p];
        ts.map_points(&mut q);
        [q[0].x, q[0].y]
    };

    let mut layer = Layer {
        min_zoom: min_zoom_from_id(path.id()),
        fill,
        stroke,
        runs: Vec::new(),
        points: Vec::new(),
    };
    let mut start = 0usize;
    let mut last = [0.0f32; 2];
    let mut closed = false;
    let end_run = |layer: &mut Layer, start: &mut usize, closed: &mut bool| {
        let pts = &layer.points[*start..];
        if pts.len() >= 2 {
            let mut bbox = [f32::MAX, f32::MAX, f32::MIN, f32::MIN];
            for p in pts {
                bbox = [bbox[0].min(p[0]), bbox[1].min(p[1]), bbox[2].max(p[0]), bbox[3].max(p[1])];
            }
            layer.runs.push(Run {
                bbox,
                start: *start as u32,
                len: pts.len() as u32,
                closed: *closed,
            });
        } else {
            layer.points.truncate(*start);
        }
        *start = layer.points.len();
        *closed = false;
    };
    for seg in path.data().segments() {
        use tiny_skia::PathSegment as S;
        match seg {
            S::MoveTo(p) => {
                end_run(&mut layer, &mut start, &mut closed);
                last = map(p);
                layer.points.push(last);
            }
            S::LineTo(p) => {
                last = map(p);
                layer.points.push(last);
            }
            S::QuadTo(c, p) => {
                let (a, c, p) = (last, map(c), map(p));
                for i in 1..=CURVE_STEPS {
                    let t = i as f32 / CURVE_STEPS as f32;
                    let u = 1.0 - t;
                    layer.points.push([
                        u * u * a[0] + 2.0 * u * t * c[0] + t * t * p[0],
                        u * u * a[1] + 2.0 * u * t * c[1] + t * t * p[1],
                    ]);
                }
                last = p;
            }
            S::CubicTo(c1, c2, p) => {
                let (a, c1, c2, p) = (last, map(c1), map(c2), map(p));
                for i in 1..=CURVE_STEPS {
                    let t = i as f32 / CURVE_STEPS as f32;
                    let u = 1.0 - t;
                    let (k0, k1, k2, k3) = (u * u * u, 3.0 * u * u * t, 3.0 * u * t * t, t * t * t);
                    layer.points.push([
                        k0 * a[0] + k1 * c1[0] + k2 * c2[0] + k3 * p[0],
                        k0 * a[1] + k1 * c1[1] + k2 * c2[1] + k3 * p[1],
                    ]);
                }
                last = p;
            }
            S::Close => closed = true,
        }
    }
    end_run(&mut layer, &mut start, &mut closed);
    layer.runs.shrink_to_fit();
    layer.points.shrink_to_fit();
    Some(layer)
}

/// Drops vertices closer than [`THIN_PX`] to the last kept one: at low zoom a 1:10m
/// coastline has many vertices per pixel, which cost time and draw nothing.
fn thin(pts: impl Iterator<Item = [f64; 2]>) -> Vec<[f64; 2]> {
    let mut out: Vec<[f64; 2]> = Vec::new();
    let mut last = None;
    for p in pts {
        last = Some(p);
        if let Some(q) = out.last() {
            if (p[0] - q[0]).abs() < THIN_PX && (p[1] - q[1]).abs() < THIN_PX {
                continue;
            }
        }
        out.push(p);
    }
    // Keep the true end point, so lines still meet where they should.
    if let (Some(l), Some(q)) = (last, out.last()) {
        if l != *q {
            out.push(l);
        }
    }
    out
}

fn append_polyline(pb: &mut PathBuilder, pts: &[[f64; 2]]) {
    if let Some((first, rest)) = pts.split_first() {
        pb.move_to(first[0] as f32, first[1] as f32);
        for p in rest {
            pb.line_to(p[0] as f32, p[1] as f32);
        }
    }
}

/// Sutherland–Hodgman: the part of a closed ring inside `[x0, y0, x1, y1]`. The edges
/// it adds run along the rectangle, which lies outside the tile by the clip margin.
fn clip_polygon(mut ring: Vec<[f64; 2]>, rect: [f64; 4]) -> Vec<[f64; 2]> {
    for edge in 0..4 {
        let inside = |p: &[f64; 2]| match edge {
            0 => p[0] >= rect[0],
            1 => p[1] >= rect[1],
            2 => p[0] <= rect[2],
            _ => p[1] <= rect[3],
        };
        let cross = |a: &[f64; 2], b: &[f64; 2]| {
            let t = match edge {
                0 => (rect[0] - a[0]) / (b[0] - a[0]),
                1 => (rect[1] - a[1]) / (b[1] - a[1]),
                2 => (rect[2] - a[0]) / (b[0] - a[0]),
                _ => (rect[3] - a[1]) / (b[1] - a[1]),
            };
            [a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])]
        };
        let Some(&prev0) = ring.last() else { return ring };
        let mut out = Vec::with_capacity(ring.len());
        let mut prev = prev0;
        for &cur in &ring {
            match (inside(&prev), inside(&cur)) {
                (true, true) => out.push(cur),
                (true, false) => out.push(cross(&prev, &cur)),
                (false, true) => {
                    out.push(cross(&prev, &cur));
                    out.push(cur);
                }
                (false, false) => {}
            }
            prev = cur;
        }
        ring = out;
    }
    ring
}

/// A visible stretch of a clipped polyline and how far along the whole line it starts,
/// in pixels (the dash phase).
struct Piece {
    points: Vec<[f64; 2]>,
    start_len: f64,
}

/// Liang–Barsky per segment: the stretches of a polyline inside `rect`.
fn clip_polyline(pts: impl Iterator<Item = [f64; 2]>, rect: [f64; 4]) -> Vec<Piece> {
    let mut pieces: Vec<Piece> = Vec::new();
    let mut open = false;
    let mut walked = 0.0;
    let mut prev: Option<[f64; 2]> = None;
    for b in pts {
        let Some(a) = prev.replace(b) else { continue };
        let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
        let seg_len = (dx * dx + dy * dy).sqrt();
        let (mut t0, mut t1) = (0.0f64, 1.0f64);
        let mut visible = true;
        for (p, q) in [
            (-dx, a[0] - rect[0]),
            (dx, rect[2] - a[0]),
            (-dy, a[1] - rect[1]),
            (dy, rect[3] - a[1]),
        ] {
            if p == 0.0 {
                if q < 0.0 {
                    visible = false;
                    break;
                }
            } else {
                let r = q / p;
                if p < 0.0 {
                    t0 = t0.max(r);
                } else {
                    t1 = t1.min(r);
                }
            }
        }
        if visible && t0 <= t1 {
            let s = [a[0] + t0 * dx, a[1] + t0 * dy];
            let e = [a[0] + t1 * dx, a[1] + t1 * dy];
            if !open || t0 > 0.0 {
                pieces.push(Piece {
                    points: vec![s],
                    start_len: walked + t0 * seg_len,
                });
            }
            pieces.last_mut().unwrap().points.push(e);
            open = t1 >= 1.0;
        } else {
            open = false;
        }
        walked += seg_len;
    }
    pieces
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
