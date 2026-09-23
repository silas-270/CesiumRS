pub mod svg_renderer;
pub use svg_renderer::SvgTileRenderer;

/// The bundled offline world map (Natural Earth 1:10m, dark palette), gzipped. Written by
/// `tools/generate_world_svg.py`. Embedded once here so every caller that switches to
/// offline mode shares the same bytes in the binary.
pub static WORLD_SVG: &[u8] =
    include_bytes!("../../../../../../assets/maps/world_vector_dark.svg.gz");

/// The renderer for [`WORLD_SVG`]. Parsing it takes a noticeable moment, so it is done on
/// the first call only; later calls (every switch back to offline mode) share the result.
pub fn bundled_world_renderer() -> Result<SvgTileRenderer, String> {
    static WORLD: std::sync::OnceLock<Result<SvgTileRenderer, String>> =
        std::sync::OnceLock::new();
    WORLD
        .get_or_init(|| {
            let t = std::time::Instant::now();
            let r = SvgTileRenderer::from_svg_bytes(WORLD_SVG);
            if let Ok(r) = &r {
                let (paths, points) = r.stats();
                log::info!(
                    "Offline map: {paths} paths, {points} points, parsed in {} ms",
                    t.elapsed().as_millis()
                );
            }
            r
        })
        .clone()
}
