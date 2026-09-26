//! GPU-free check on `ObservedTextureSize` — the live feed WP4/A wires into
//! `lod_factor_for` in place of the frozen `DEFAULT_IMAGERY_TEXTURE_SIZE_PX`
//! (`docs/pre-terrain-plan.md`). Deliberately does not construct a `TileTextureManager`
//! (needs a `wgpu::Device`); `ObservedTextureSize` was split out specifically so this
//! tracking logic is testable without one.

#[cfg(test)]
mod tests {
    use cesium_engine::globe::tiles::config::DEFAULT_IMAGERY_TEXTURE_SIZE_PX;
    use cesium_engine::globe::tiles::texture_manager::ObservedTextureSize;

    /// Before any tile has decoded, the live feed must fall back to the same
    /// constant `wgpu_state::update_logic` used unconditionally pre-WP4/A — not
    /// zero, not `NaN`, not silently wrong.
    #[test]
    fn defaults_to_the_frozen_fallback_before_any_decode() {
        let observed = ObservedTextureSize::default();
        assert_eq!(observed.current_px(), DEFAULT_IMAGERY_TEXTURE_SIZE_PX);
    }

    /// The core claim of WP4/A: this is a *live* feed, not a second frozen constant.
    /// Checked away from the bootstrap value in both directions — a decode smaller
    /// than the default (matching `satellite_imagery_url()`'s 256px tiles) and one
    /// larger than it — so a wiring bug that only ever reported the default, or that
    /// silently clamped, would fail this.
    #[test]
    fn tracks_the_most_recently_decoded_size() {
        let mut observed = ObservedTextureSize::default();

        observed.record(256, 256);
        assert_eq!(observed.current_px(), 256.0, "must move off the 512px default");

        observed.record(1024, 1024);
        assert_eq!(
            observed.current_px(),
            1024.0,
            "must keep tracking the latest decode, not stick at the first one"
        );
    }
}
