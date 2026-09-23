pub mod svg_renderer;
pub use svg_renderer::SvgTileRenderer;

/// The bundled offline world map (Natural Earth 1:50m, dark palette). Embedded once here
/// so every caller that switches to offline mode shares the same bytes in the binary.
pub static WORLD_SVG: &[u8] = include_bytes!("../../../../../../assets/maps/world_vector_dark.svg");

/// Parses [`WORLD_SVG`] into a renderer (~50 ms; do it once per switch to offline mode).
pub fn bundled_world_renderer() -> Result<SvgTileRenderer, String> {
    SvgTileRenderer::from_svg_bytes(WORLD_SVG)
}
