use std::num::NonZeroUsize;
use std::time::Duration;

/// Default dark, label-free vector-style basemap. The `@2x` suffix requests
/// 512x512 retina tiles, which carry genuinely twice the detail rather than
/// an upscale. Tile dimensions are derived from the decoded image, so styles
/// served at 256x256 (e.g. `SATELLITE_IMAGERY_URL`) still work unchanged.
pub const STANDARD_IMAGERY_URL: &str = "https://a.basemaps.cartocdn.com/dark_nolabels/{z}/{x}/{y}@2x.png?key=cb1_28wa_1_ff42c0a0f313514c2bdb2e7a";
/// Esri World Imagery - free, no API key required.
pub const SATELLITE_IMAGERY_URL: &str = "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}";

/// Mapzen/Tilezen "Terrarium" elevation tiles on AWS Open Data — free, no API key.
///
/// 256x256 RGB(A) PNG; each texel encodes metres above the WGS-84 ellipsoid as
/// `h = R·256 + G + B/256 − 32768`. See
/// [`crate::globe::terrain::height_tile::decode_terrarium`] for the decoder and
/// [`TERRARIUM_MAX_LEVEL`] for the depth ceiling.
pub const TERRARIUM_URL: &str =
    "https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{z}/{x}/{y}.png";

/// Deepest level the Terrarium source actually serves. **Probed live, not assumed**
/// (`docs/terrain-plan.md` §2, re-checked 2026-09-20): `z15` returns a tile, `z16`
/// returns `404`.
///
/// Imagery refines to z19/z20 ([`TileEngineConfig::max_zoom`]), so for five of the
/// twenty levels there is no height tile *in principle*. Upsampling a z15 ancestor is
/// therefore the **normal** path for deep tiles, not an error path — see
/// [`crate::globe::terrain::height_cache::HeightTileManager::height_at`].
pub const TERRARIUM_MAX_LEVEL: u8 = 15;

/// Resident bytes one decoded [`crate::globe::terrain::height_tile::HeightTile`]
/// costs: 256x256 `i16` samples plus the 16x16 min and max mips.
///
/// `docs/terrain-plan.md` §5 B4 rounds this to "128 kB per height tile; 256 resident
/// = 32 MB". The real figure is 129 kB, because the mips are not free, so the 32 MiB
/// default below derives **254** entries rather than 256. The budget is the promise;
/// the entry count is derived from it, exactly as it is for imagery.
pub const HEIGHT_TILE_BYTES: usize = 256 * 256 * 2 + 2 * (16 * 16 * 2);

/// What to do with the sub-sea-level samples the Terrarium source carries.
///
/// The open ocean in Terrarium is **bathymetry**, not a flat sheet: a mid-Pacific z12
/// tile measures −4324 … −2276 m (`docs/terrain-plan.md` §2, reproduced as a pinned
/// fixture test). Rendered untreated, the sea floor *is* the sea surface and every
/// coastline becomes a multi-kilometre cliff.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum OceanPolicy {
    /// Clamp `h < 0` to `0` at decode time, so the sea is the ellipsoid.
    ///
    /// The price, stated rather than hidden: genuine below-sea-level land is flattened
    /// with it — the Dead Sea (uniformly −412 m in its z12 tile) and Death Valley
    /// (−86 m) come out at zero. That is the right trade for a flight tracker, where
    /// the ocean is most of the frame and the Dead Sea is two tiles.
    #[default]
    ClampToZero,
    /// Decode the source verbatim, bathymetry included. Kept so the choice above is
    /// visible and reversible rather than baked into the decoder.
    Raw,
}

/// Terrain height data — `docs/terrain-plan.md` §4 A3, §5, §6 and §7.
///
/// Off by default, and still off at the end of D2. The unsoundness §10 warned about is
/// **gone**: D1 fits the bounding volumes over each node's `[h_min, h_max]` and D2 runs
/// the limb test on its scaled-space bounding sphere, so turning this on no longer loses
/// geometry — the sweep in `testing::terrain::test_terrain_visibility` measures FN = 0 and
/// the headless captures over the Alps are gapless.
///
/// What is still missing is **D3**, the occlusion march of §3.3: tiles hidden behind
/// mountains are still drawn, which is a cost rather than a defect. The flip to `true`
/// belongs to §9 (Phase F), after D3 and after the on-device measurements, in its own
/// commit.
#[derive(Clone, Debug)]
pub struct TerrainConfig {
    /// Master switch. While `false` no height fetcher, no height cache and no height
    /// request exists — [`crate::globe::tiles::system::TileSystem::height_manager`] is
    /// `None` — so the flat path is byte-for-byte what it was before terrain existed.
    pub enabled: bool,
    /// XYZ template for the height source, `{z}`/`{x}`/`{y}` placeholders, same
    /// convention as [`TileEngineConfig::base_imagery_url`].
    pub source_url: String,
    /// Deepest level requested from the source. Requests for tiles below this are
    /// redirected to the ancestor at this level rather than turned into 404s.
    pub max_level: u8,
    /// Vertical exaggeration.
    ///
    /// Applied in exactly one place — [`crate::globe::terrain::HeightPatch::sample`],
    /// where a tile's heights are read out of the cache — and nowhere else
    /// (`docs/terrain-plan.md` §6 C1: "here and nowhere else"). Because that is
    /// upstream of the patch's own `[h_min, h_max]`, §3.1's boxes and §3.2's spheres
    /// inherit it automatically instead of having to remember it.
    ///
    /// [`crate::globe::terrain::HeightTileManager::height_at`] deliberately returns
    /// the **raw** field: if it exaggerated too, the factor would be applied twice.
    /// `terrain::test_heightfield::exaggeration_scales_heights_and_bounds_exactly_once`
    /// is what holds that.
    pub exaggeration: f32,
    /// How sub-sea-level samples are treated. See [`OceanPolicy`].
    pub ocean: OceanPolicy,
    /// The height cache's **declared slice** of
    /// [`TileEngineConfig::tile_cache_budget_bytes`], not an addition to it — see
    /// [`TileEngineConfig::imagery_cache_budget_bytes`]. Terrain on must not silently
    /// raise the engine's total tile-memory ceiling.
    pub height_cache_budget_bytes: usize,
}

impl Default for TerrainConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            source_url: TERRARIUM_URL.to_string(),
            max_level: TERRARIUM_MAX_LEVEL,
            exaggeration: 1.0,
            ocean: OceanPolicy::ClampToZero,
            // 32 MiB — `docs/terrain-plan.md` §5 B4. At HEIGHT_TILE_BYTES that is 254
            // resident height tiles, enough for the visible set plus its ancestor
            // chains at any camera this engine flies.
            height_cache_budget_bytes: 32 * 1024 * 1024,
        }
    }
}

/// Lower bound the byte budget may never push the imagery cache below, however
/// large a single tile turns out to be. Well under any plausible working set
/// (visible tiles plus `prefetch_radius`), so it only ever acts as a guard
/// against a pathological tile size thrashing the cache down to nothing.
pub const MIN_TILE_CACHE_ENTRIES: usize = 64;

/// Imagery tile edge length, in texels, that the LOD rule falls back to before it
/// has any better information.
///
/// Matches what [`STANDARD_IMAGERY_URL`]'s `@2x` suffix actually serves (512x512).
/// **No longer the value the LOD rule always runs at** — WP4/A
/// (`docs/pre-terrain-plan.md`) feeds the real decoded tile size through live, via
/// [`crate::globe::tiles::texture_manager::TileTextureManager::current_texture_size_px`],
/// called fresh every frame from `wgpu_state::update_logic`. This constant is now
/// only the *bootstrap* value: the texture manager doesn't know a style's real size
/// until its first tile has decoded, and this is what `lod_factor_for` runs at until
/// then (or if imagery is disabled). Before WP4/A this was the value used
/// unconditionally, silently halving effective sharpness on any 256² style
/// (`SATELLITE_IMAGERY_URL`) with no LOD compensation.
///
/// This mirrors, but is deliberately a separate constant from, the LOD test harness's
/// own `TEXTURE_SIZE_PX` in `src/testing/lod/sweep.rs` — the harness measures texel
/// density and must be able to state its assumption independently of the engine's,
/// and now takes `texture_size_px` as an explicit parameter so it can measure either
/// style rather than assuming one.
pub const DEFAULT_IMAGERY_TEXTURE_SIZE_PX: f32 = 512.0;

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
    /// Imagery texels demanded per screen pixel — the LOD target, and the WP1 LOD
    /// harness's own metric (`texels / screen_px`). `1.0` means "one texel per
    /// pixel": neither blurry nor wasteful. **Higher values demand more texels per
    /// pixel, so tiles stay sharper**: higher visual fidelity, worse performance.
    ///
    /// This is **not** a raw distance multiplier any more — it feeds into the
    /// derived `lod_factor` as `sqrt(target_texel_ratio)` rather than being it
    /// directly (the ratio above is an *area* ratio; `lod_factor` scales a *linear*
    /// distance). See [`lod_factor_for`](crate::globe::quadtree::lod_factor_for) for
    /// the formula and the exact calibration that makes the default reproduce the
    /// old hard-coded `2.0`, and `docs/pre-terrain-plan.md` WP3 for why.
    ///
    /// It is also **not** a screen-space error knob, and is not pretending to be one.
    /// Cesium's SSE bounds *geometric* error in pixels; with zero terrain relief this
    /// engine has no geometric error to bound, so texel density is the only honest
    /// thing it can target. Once terrain/relief exists, geometric error reappears and
    /// a genuine SSE metric becomes the right thing to expose — a separate knob, not a
    /// rename of this one.
    pub target_texel_ratio: f32,
    /// Atmospheric fog — WP5 of `docs/pre-terrain-plan.md`, ported from CesiumJS's
    /// `Scene/Fog.js` defaults. Consumed by `wgpu_state::update_logic` to derive
    /// this frame's fog density (`crate::globe::quadtree::fog_density_for`) from
    /// camera altitude, which drives both `Stage::Fog` (an outright cull, present
    /// only in `CullPipeline::DEFAULT_WITH_FOG` — **never** in `DEFAULT`, which the
    /// culling harness builds and every FN = 0 guarantee is proved against) and
    /// `QuadtreeNode::apply_lod`'s threshold relaxation. See
    /// `crate::globe::quadtree::fog`'s module doc comment for the full story.
    pub fog: crate::globe::quadtree::FogConfig,
    pub prefetch_radius: u32,
    pub enable_prefetch: bool,
    pub negative_cache_duration: Duration,
    pub base_imagery_url: String,
    /// Maximum quadtree zoom level supported by this imagery source.
    /// Prevents subdividing beyond the source's actual content depth, avoiding
    /// wasted fetches and cache space for flat-colour or missing tiles.
    /// Defaults to `19` for the standard basemap whose content stops at z=19.
    pub max_zoom: u8,
    pub base_color: [u8; 4],
    pub offline_mode: bool,
    pub map_saturation: f32,
    pub map_contrast: f32,
    pub map_brightness: f32,
    pub transparent_background: bool,
    pub mesh_segments: u32,
    /// Terrain height data. Off by default; see [`TerrainConfig`].
    pub terrain: TerrainConfig,
}

impl TileEngineConfig {
    /// The part of [`tile_cache_budget_bytes`](Self::tile_cache_budget_bytes) left for
    /// decoded imagery textures once terrain's declared share is taken out.
    ///
    /// `docs/terrain-plan.md` §5 B4: the height cache takes a **slice** of the existing
    /// byte budget rather than silently doubling the engine's tile-memory ceiling. With
    /// terrain off this returns `tile_cache_budget_bytes` unchanged, so
    /// `TileTextureManager` sizes itself exactly as it did before terrain existed.
    pub fn imagery_cache_budget_bytes(&self) -> usize {
        if self.terrain.enabled {
            self.tile_cache_budget_bytes
                .saturating_sub(self.terrain.height_cache_budget_bytes)
        } else {
            self.tile_cache_budget_bytes
        }
    }
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
            target_texel_ratio: 1.0,
            fog: crate::globe::quadtree::FogConfig::default(),
            prefetch_radius: 1, // Number of tiles to prefetch in velocity direction
            enable_prefetch: true,
            negative_cache_duration: Duration::from_secs(10),
            base_imagery_url: STANDARD_IMAGERY_URL.to_string(),
            max_zoom: 19,
            base_color: [20, 20, 20, 255],
            offline_mode: false,
            map_saturation: 0.0,
            map_contrast: 0.0,
            map_brightness: 0.5,
            transparent_background: false,
            mesh_segments: 16,
            terrain: TerrainConfig::default(),
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

    #[test]
    fn default_config_max_zoom_is_19() {
        let config = TileEngineConfig::default();
        assert_eq!(config.max_zoom, 19);
    }

    /// Terrain off is the whole of Phase B, and it must leave the imagery budget
    /// literally untouched — the flat path may not move.
    #[test]
    fn terrain_off_leaves_the_imagery_budget_alone() {
        let config = TileEngineConfig::default();
        assert!(!config.terrain.enabled);
        assert_eq!(
            config.imagery_cache_budget_bytes(),
            config.tile_cache_budget_bytes
        );
    }

    /// B4: a slice of the existing budget, not an addition to it.
    #[test]
    fn terrain_on_takes_its_share_out_of_the_imagery_budget() {
        let mut config = TileEngineConfig::default();
        config.terrain.enabled = true;
        assert_eq!(
            config.imagery_cache_budget_bytes() + config.terrain.height_cache_budget_bytes,
            config.tile_cache_budget_bytes
        );
    }

    /// The "256 resident = 32 MB" line of §5 B4, with the mips counted: 254.
    #[test]
    fn the_default_height_budget_lands_on_254_tiles() {
        let terrain = TerrainConfig::default();
        assert_eq!(HEIGHT_TILE_BYTES, 132_096);
        assert_eq!(terrain.height_cache_budget_bytes / HEIGHT_TILE_BYTES, 254);
    }

    #[test]
    fn terrain_defaults_are_off_terrarium_and_clamped() {
        let terrain = TerrainConfig::default();
        assert!(!terrain.enabled);
        assert_eq!(terrain.source_url, TERRARIUM_URL);
        assert_eq!(terrain.max_level, 15);
        assert_eq!(terrain.exaggeration, 1.0);
        assert_eq!(terrain.ocean, OceanPolicy::ClampToZero);
    }
}
