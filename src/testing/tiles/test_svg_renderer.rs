//! The offline map's tile rasterizer (`cesium_engine::globe::tiles::vector`): pixel-width
//! strokes, `.z<N>` minimum zooms, and what the bundled Natural Earth 1:10m map costs.
//!
//! `cargo test --lib tiles::test_svg_renderer -- --nocapture` prints the load time and
//! per-zoom tile times; `bundled_map_tiles_capture` (ignored) also writes the tiles as
//! PNGs to `$CESIUM_SHOT_DIR` for a visual check.

use cesium_engine::globe::quadtree::TileId;
use cesium_engine::globe::tiles::vector::{bundled_world_renderer, SvgTileRenderer};

/// A black world with one horizontal white line at canvas y = 2050 (viewBox 4096).
fn line_svg(id: &str, width: f32) -> String {
    format!(
        r##"<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 4096 4096" width="4096" height="4096">
<rect fill="#000000" width="4096" height="4096"/>
<path id="{id}" d="M0 2050H4096" fill="none" stroke="#ffffff" stroke-width="{width}"/>
</svg>"##
    )
}

/// Rows in pixel column 256 that the line covers by more than half.
fn lit_rows(r: &SvgTileRenderer, id: TileId) -> usize {
    let (w, _, px) = r.render_tile(id).unwrap();
    (0..w as usize)
        .filter(|&y| px[(y * w as usize + 256) * 4] > 128)
        .count()
}

#[test]
fn stroke_width_is_pixels_at_every_zoom() {
    let r = SvgTileRenderer::from_svg_bytes(line_svg("line", 2.0).as_bytes()).unwrap();
    // y = 2050 lands in tile row 0 at z0, row 2 at z2, row 32 at z6 and row 2050 at z12.
    for (z, y) in [(0u8, 0u32), (2, 2), (6, 32), (12, 2050)] {
        let rows = lit_rows(&r, TileId { z, x: 1u32 << z >> 1, y });
        assert!((1..=3).contains(&rows), "z{z}: a 2 px stroke covers {rows} rows");
    }
}

#[test]
fn id_suffix_sets_min_zoom() {
    let r = SvgTileRenderer::from_svg_bytes(line_svg("line.z3", 2.0).as_bytes()).unwrap();
    assert_eq!(lit_rows(&r, TileId { z: 2, x: 2, y: 2 }), 0, "hidden below z3");
    assert!(lit_rows(&r, TileId { z: 3, x: 4, y: 4 }) > 0, "shown from z3");
}

#[test]
fn gzipped_svg_parses_like_plain() {
    use std::io::Write;
    let svg = line_svg("line", 2.0);
    let mut gz = flate2::write::GzEncoder::new(Vec::new(), flate2::Compression::fast());
    gz.write_all(svg.as_bytes()).unwrap();
    let r = SvgTileRenderer::from_svg_bytes(&gz.finish().unwrap()).unwrap();
    assert!(lit_rows(&r, TileId { z: 0, x: 0, y: 0 }) > 0);
}

/// The tile containing (`lon`, `lat`) at zoom `z`.
fn tile_at(lon: f64, lat: f64, z: u8) -> TileId {
    let n = (1u64 << z) as f64;
    let x = ((lon + 180.0) / 360.0 * n) as u32;
    let lat = lat.to_radians();
    let y = ((1.0 - (lat.tan() + 1.0 / lat.cos()).ln() / std::f64::consts::PI) / 2.0 * n) as u32;
    TileId { z, x, y }
}

const FRA: (f64, f64) = (8.5706, 50.0333);
const ZOOMS: [u8; 7] = [2, 4, 6, 8, 10, 12, 14];

#[test]
fn bundled_map_loads_and_renders() {
    let t = std::time::Instant::now();
    let r = bundled_world_renderer().expect("bundled map parses");
    let (paths, points) = r.stats();
    println!("  bundled map: {paths} paths, {points} points, loaded in {:?}", t.elapsed());
    assert!(points > 1_000_000, "the 1:10m map has millions of points, got {points}");

    for z in ZOOMS {
        let id = tile_at(FRA.0, FRA.1, z);
        let t = std::time::Instant::now();
        let (w, h, px) = r.render_tile(id).unwrap();
        println!("  z{z:>2} {id:?}: {:?}", t.elapsed());
        assert_eq!(px.len(), (w * h * 4) as usize);
        assert!(px.chunks(4).all(|p| p[3] == 255), "z{z}: tile must be opaque");
    }
}

#[test]
#[ignore = "visual verification: writes PNGs"]
fn bundled_map_tiles_capture() {
    let dir = crate::testing::rendering::terrain_capture::shot_dir();
    let r = bundled_world_renderer().unwrap();
    for z in ZOOMS {
        let id = tile_at(FRA.0, FRA.1, z);
        let (w, h, px) = r.render_tile(id).unwrap();
        let out = dir.join(format!("svg_tile_fra_z{z:02}.png"));
        image::RgbaImage::from_raw(w, h, px).unwrap().save(&out).unwrap();
        println!("  {}", out.display());
    }
}
