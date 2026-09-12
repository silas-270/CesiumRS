use std::num::NonZeroUsize;
use std::time::Duration;

/// Default dark, label-free vector-style basemap. The `@2x` suffix requests
/// 512x512 retina tiles, which carry genuinely twice the detail rather than
/// an upscale. Tile dimensions are derived from the decoded image, so styles
/// served at 256x256 (e.g. `SATELLITE_IMAGERY_URL`) still work unchanged.
pub const STANDARD_IMAGERY_URL: &str = "https://a.basemaps.cartocdn.com/dark_nolabels/{z}/{x}/{y}@2x.png?key=cb1_28wa_1_ff42c0a0f313514c2bdb2e7a";
/// Esri World Imagery - free, no API key required.
pub const SATELLITE_IMAGERY_URL: &str = "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}";

/// Lower bound the byte budget may never push the imagery cache below, however
/// large a single tile turns out to be. Well under any plausible working set
/// (visible tiles plus `prefetch_radius`), so it only ever acts as a guard
/// against a pathological tile size thrashing the cache down to nothing.
pub const MIN_TILE_CACHE_ENTRIES: usize = 64;

/// How many imagery tiles of `bytes_per_tile` fit in `budget_bytes`, clamped to
/// [`MIN_TILE_CACHE_ENTRIES`] and to `max_entries`.
///
/// Split out from the texture manager so the arithmetic is testable without a
/// GPU device, and so the "whichever bound is smaller wins" rule lives in one
/// place.
pub fn tile_cache_entries_for(
    budget_bytes: usize,
    bytes_per_tile: usize,
    max_entries: NonZeroUsize,
) -> NonZeroUsize {
    if bytes_per_tile == 0 {
        return max_entries;
    }
    let fits = budget_bytes / bytes_per_tile;
    // Floor first, cap second — deliberately not `clamp`, which panics when a
    // caller configures `max_entries` below MIN_TILE_CACHE_ENTRIES. An explicit
    // cap is a caller's decision and has to win over our floor.
    let clamped = fits.max(MIN_TILE_CACHE_ENTRIES).min(max_entries.get());
    NonZeroUsize::new(clamped).unwrap_or(max_entries)
}

#[derive(Clone, Debug)]
pub struct TileEngineConfig {
    /// Hard upper bound on imagery cache entries. Note this is a *count*, so on
    /// its own it doesn't bound memory: the same 2048 entries are 512MB of
    /// 256x256 tiles but 2GB of the 512x512 ones `STANDARD_IMAGERY_URL` serves.
    /// [`tile_cache_budget_bytes`](Self::tile_cache_budget_bytes) is what
    /// actually bounds it; whichever of the two is smaller wins.
    pub max_cache_size: NonZeroUsize,
    /// Memory budget for decoded imagery textures. Once the first tile of a
    /// style has been decoded its real byte size is known, and the cache is
    /// resized to `budget / bytes_per_tile` (clamped to
    /// [`MIN_TILE_CACHE_ENTRIES`] and `max_cache_size`), so switching basemap
    /// styles re-derives the entry count instead of silently changing the
    /// memory ceiling by 4x.
    pub tile_cache_budget_bytes: usize,
    pub mesh_cache_size: NonZeroUsize,
    pub lod_factor: f32,
    pub prefetch_radius: u32,
    pub enable_prefetch: bool,
    pub negative_cache_duration: Duration,
    pub base_imagery_url: String,
    pub base_color: [u8; 4],
    pub offline_mode: bool,
    pub map_saturation: f32,
    pub map_contrast: f32,
    pub map_brightness: f32,
    pub transparent_background: bool,
    pub mesh_segments: u32,
}

impl Default for TileEngineConfig {
    fn default() -> Self {
        Self {
            max_cache_size: NonZeroUsize::new(2048).unwrap(),
            // 512MB. At the 512x512 RGBA8 tiles STANDARD_IMAGERY_URL serves
            // (1MiB each, no mips) that lands on ~512 entries; at 256x256 it
            // stays at the 2048 cap, i.e. unchanged from before this budget
            // existed. Measured on a 35-minute device run: the count-only cap
            // let imagery textures climb past 1.9GB and still rising, on a
            // device that was down to 1.4GB available.
            tile_cache_budget_bytes: 512 * 1024 * 1024,
            mesh_cache_size: NonZeroUsize::new(512).unwrap(),
            lod_factor: 2.0,
            prefetch_radius: 1, // Number of tiles to prefetch in velocity direction
            enable_prefetch: true,
            negative_cache_duration: Duration::from_secs(10),
            base_imagery_url: STANDARD_IMAGERY_URL.to_string(),
            base_color: [20, 20, 20, 255],
            offline_mode: false,
            map_saturation: 0.0,
            map_contrast: 0.0,
            map_brightness: 0.5,
            transparent_background: false,
            mesh_segments: 16,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn nz(n: usize) -> NonZeroUsize {
        NonZeroUsize::new(n).unwrap()
    }

    const MIB: usize = 1024 * 1024;

    /// The case this budget exists for: `STANDARD_IMAGERY_URL`'s `@2x` tiles are
    /// 512x512 RGBA8 = 1MiB each, so the old count-only cap of 2048 meant a 2GB
    /// ceiling. The budget has to bring that down without touching the cap.
    #[test]
    fn retina_tiles_are_bounded_by_the_budget_not_the_count() {
        assert_eq!(
            tile_cache_entries_for(512 * MIB, 512 * 512 * 4, nz(2048)).get(),
            512
        );
    }

    /// A 256x256 style (e.g. `SATELLITE_IMAGERY_URL`) is 256KiB per tile, so
    /// 2048 of them fit in the same budget and the count cap stays the binding
    /// constraint — i.e. behaviour there is unchanged.
    #[test]
    fn small_tiles_still_hit_the_count_cap_first() {
        assert_eq!(
            tile_cache_entries_for(512 * MIB, 256 * 256 * 4, nz(2048)).get(),
            2048
        );
    }

    #[test]
    fn never_shrinks_below_the_floor() {
        // A 4096x4096 tile would be 64MiB; 8 of those fit in a 512MB budget,
        // which is below any workable working set.
        assert_eq!(
            tile_cache_entries_for(512 * MIB, 4096 * 4096 * 4, nz(2048)).get(),
            MIN_TILE_CACHE_ENTRIES
        );
    }

    #[test]
    fn a_budget_smaller_than_one_tile_still_yields_the_floor() {
        assert_eq!(
            tile_cache_entries_for(1024, 512 * 512 * 4, nz(2048)).get(),
            MIN_TILE_CACHE_ENTRIES
        );
    }

    /// Guards the clamp order: the floor must not be able to push the count
    /// back above an explicitly configured, smaller cap.
    #[test]
    fn cap_wins_over_the_floor_when_the_cap_is_tiny() {
        assert_eq!(tile_cache_entries_for(1024, MIB, nz(16)).get(), 16);
    }

    #[test]
    fn a_zero_tile_size_falls_back_to_the_cap() {
        assert_eq!(tile_cache_entries_for(512 * MIB, 0, nz(2048)).get(), 2048);
    }

    #[test]
    fn default_config_bounds_the_standard_basemap_at_the_budget() {
        let config = TileEngineConfig::default();
        assert!(config.base_imagery_url.contains("@2x"));
        let entries = tile_cache_entries_for(
            config.tile_cache_budget_bytes,
            512 * 512 * 4,
            config.max_cache_size,
        );
        assert_eq!(entries.get() * 512 * 512 * 4, config.tile_cache_budget_bytes);
    }
}
