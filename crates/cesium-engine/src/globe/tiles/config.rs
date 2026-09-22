use std::num::NonZeroUsize;
use std::time::Duration;

/// Default dark, label-free vector-style basemap. The `@2x` suffix requests
/// 512x512 retina tiles, which carry genuinely twice the detail rather than
/// an upscale. Tile dimensions are derived from the decoded image, so styles
/// served at 256x256 (e.g. `SATELLITE_IMAGERY_URL`) still work unchanged.
pub const STANDARD_IMAGERY_URL: &str = "https://a.basemaps.cartocdn.com/dark_nolabels/{z}/{x}/{y}@2x.png?key=cb1_28wa_1_ff42c0a0f313514c2bdb2e7a";
/// Esri World Imagery - free, no API key required.
pub const SATELLITE_IMAGERY_URL: &str =
    "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}";

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
/// costs: 256x256 `i16` samples, the 16x16 min and max mips, E1's one-`i16` measured
/// geometric error ([`crate::globe::terrain::HeightTile::detail`]) and F5's 84-entry
/// pyramid of the same measurement for the descendants below it
/// ([`crate::globe::terrain::HeightTile::detail_below`]).
///
/// `docs/terrain-plan.md` §5 B4 rounds this to "128 kB per height tile; 256 resident
/// = 32 MB". The real figure is 129 kB, because the mips are not free, so a 32 MiB
/// slice derives **253** entries rather than 256 and §9 F2b's 48 MiB derives **380**.
/// The budget is the promise; the entry count is derived from it, exactly as it is for
/// imagery — see [`HEIGHT_CACHE_BUDGET_BYTES`].
///
/// E1's error term costs two bytes a tile and F5's pyramid another 168 — together **0.13 %**
/// of the entry, and one entry off the derived count: 50 331 648 / 132 266 is 380 where
/// 50 331 648 / 132 098 was 381. That is what four levels of measured shape cost in memory,
/// and it is the whole of it.
pub const HEIGHT_TILE_BYTES: usize = 256 * 256 * 2
    + 2 * (16 * 16 * 2)
    + 2
    + 2 * crate::globe::terrain::HEIGHT_DETAIL_PYRAMID_CELLS
    + 8; // the per-tile quantisation base and step

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

/// Whether [`TerrainConfig::enabled`] defaults to `true` on **this** target —
/// `docs/terrain-plan.md` §9 F4, and the one line where the platform split lives.
///
/// **Desktop: `true`.** F1 and F2 cover it. The visible set over the ten real DEM poses
/// costs 702 tiles against 483 flat, the drawn geometric error sits inside the shipped
/// 12 px budget at every pose but the Himalayan cliffs, and the memory split is measured
/// (`testing::terrain::test_mesh_density::f2_where_the_bytes_go_at_the_real_poses`):
/// 103 MiB of imagery against its share, 32 MiB of heights (48 MiB since §9 F2b), 1.6 MB
/// of vertex buffers at the heaviest pose.
///
/// **Android: `false`, and this is a deliberate hole, not an oversight.** §9 binds the
/// flip to an S23 soak with terrain on against terrain off, and that soak **has not been
/// run** — the machine Phase F was written on has no `adb` and no phone attached. Nothing
/// here is a prediction that terrain is too expensive on device; it is the statement that
/// nobody has looked. §9 F3 is the runbook that closes it, and this constant is what it
/// flips.
///
/// It is a constant rather than a per-entry-point assignment because Android reaches this
/// engine through more than one door — `android_main` in the root crate builds the live
/// viewer from `TileEngineConfig::default()` directly (the builder's own config is *not*
/// what it runs), and `headless::api`'s FFI still renderer is compiled for Android too. A
/// flag set at one door would have been silently missed at the other.
///
/// On device the soak's terrain-on arm does **not** need a rebuild to get past this: the
/// live viewer already exposes `nativeSetTerrainEnabled`, which goes through
/// `ViewerCommand::TerrainSetEnabled` and rebuilds the height manager in place.
pub const TERRAIN_ENABLED_BY_DEFAULT: bool = !cfg!(target_os = "android");

/// The height cache's declared slice of [`TileEngineConfig::tile_cache_budget_bytes`],
/// **48 MiB on desktop and 32 MiB on Android** — `docs/terrain-plan.md` §9 F2b.
///
/// # Why the number moved, and why not for the reason §9 F2 gave
///
/// F2 reported that three of the ten real poses want more distinct height sources than the
/// 32 MiB slice holds — `alps_inn_valley` 312 against 254 — and called the consequence
/// churn. `terrain::test_height_residency` went to measure that churn against the real
/// manager, a real `TileFetcher` and a real socket, and **did not find it**: F2's 312 is
/// `collect_sources` over the *whole quadtree*, and production
/// (`TileSystem::request_height_chain`) only ever asks for the visible set and its ancestor
/// chains. That set is **220** tiles at the worst pose, at either shipped imagery style,
/// and it fits in 254 with nothing evicted and nothing fetched twice.
///
/// What the same measurement did show is how little margin 220 of 254 leaves — 87 % of the
/// slice at 103 visible tiles — and the terrain LOD refinement of §9 F5 raises exactly that
/// tile count. So the slice is raised to give F5 somewhere to land, on a measured working
/// set rather than on F2's over-count, and the header above the number is the honest one:
/// it is headroom, not a fix.
///
/// # Android keeps 32 MiB
///
/// Not because 32 is right there — nobody knows — but because every Android memory decision
/// in this file is held to the soak of §9 F3, which has not been run. Android also has
/// terrain off by default ([`TERRAIN_ENABLED_BY_DEFAULT`]), so while that stands this
/// constant does not describe any memory the device actually allocates; the split exists so
/// that flipping the one constant does not silently flip this one too.
pub const HEIGHT_CACHE_BUDGET_BYTES: usize = if cfg!(target_os = "android") {
    32 * 1024 * 1024
} else {
    96 * 1024 * 1024
};

/// Tile meshes kept resident. A mesh is ~15 kB of GPU buffers (≈357 vertices of 32 bytes
/// plus indices), so this is ~60 MB on desktop.
///
/// Raised from 512: one 360° orbit near the ground draws more meshes than that, so the
/// second lap of the same orbit evicted and rebuilt 786 of them — each rebuilt first from
/// coarse ancestor data (its own height tile had been evicted too) and sharpened a moment
/// later, i.e. the ground visibly re-loaded (`testing::rendering::revisit`).
pub const MESH_CACHE_ENTRIES: usize = if cfg!(target_os = "android") { 1536 } else { 4096 };

/// Terrain height data — `docs/terrain-plan.md` §4 A3, §5, §6 and §7.
///
/// **On by default since §9 F4 — on desktop.** See [`TERRAIN_ENABLED_BY_DEFAULT`] for the
/// platform split and for why Android is not in it.
///
/// The unsoundness §10 warned about is **gone**: D1 fits the bounding volumes over each
/// node's `[h_min, h_max]` and D2 runs the limb test on its scaled-space bounding sphere,
/// so turning this on does not lose geometry — the sweep in
/// `testing::terrain::test_terrain_visibility` measures FN = 0 and the headless captures
/// over the Alps are gapless. **D3** — the occlusion march of §3.3, tiles hidden behind
/// mountains — landed too; see [`Self::occlusion`].
#[derive(Clone, Debug)]
pub struct TerrainConfig {
    /// Master switch, defaulting to [`TERRAIN_ENABLED_BY_DEFAULT`]. While `false` no
    /// height fetcher, no height cache and no height request exists —
    /// [`crate::globe::tiles::system::TileSystem::height_manager`] is `None` — so the flat
    /// path is byte-for-byte what it was before terrain existed, which is what the 204-pose
    /// LOD harness and the culling gate keep measuring after F4's flip.
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
    /// **D3** — culling tiles hidden behind mountains (`docs/terrain-plan.md` §3.3).
    ///
    /// Only consulted while [`Self::enabled`] is set: the occluders are the terrain
    /// quadtree's own node floors, and the flat quadtree has none.
    ///
    /// Unlike [`TileEngineConfig::fog`] this stage is geometrically **sound**, so it
    /// lives in `CullPipeline::TERRAIN_DEFAULT` — the pipeline the terrain harness
    /// measures — rather than being kept out of it. See
    /// [`crate::globe::quadtree::terrain_occlusion`].
    pub occlusion: crate::globe::quadtree::TerrainOcclusionConfig,
    /// **E1** — how many pixels of geometric error the drawn surface may show before
    /// the LOD refines it (`docs/terrain-plan.md` §8). Cesium's
    /// `maximumScreenSpaceError` in all but name.
    ///
    /// **12, not Cesium's 2**, and the two numbers are not comparable. Cesium budgets a
    /// *level-based estimate* of the error and pairs it with an imagery rule that refines
    /// far more eagerly than this engine's; the number here budgets the **measured**
    /// deviation of this tile's mesh from the DEM, against a `lod_factor` calibrated for a
    /// globe that draws ~50 tiles a frame. Copying Cesium's 2 across measures 3 350 tiles
    /// where the engine draws 483 — the table below.
    ///
    /// Feeds [`crate::globe::quadtree::terrain_lod_factor_for`], whose product with a
    /// node's measured error is the distance inside which that node subdivides for the
    /// sake of its *shape*. `0.0` switches the geometric term off entirely and leaves
    /// `apply_lod` refining on imagery sharpness alone, which is what every build before
    /// E1 did.
    ///
    /// Only consulted while [`Self::enabled`] is set: with no relief there is no error
    /// (I-1), and `Ellipsoid::HAS_GEOMETRIC_ERROR` is a compile-time `false`.
    ///
    /// # The unit is a **device** pixel, and Cesium's is not
    ///
    /// `wgpu_state` feeds [`terrain_lod_factor_for`] `self.size.height` — the surface's
    /// **physical** height, straight from the swapchain — with nothing dividing it. Cesium
    /// divides its screen-space error by `frameState.pixelRatio`
    /// (`Scene/QuadtreePrimitive.js`, `maxGeometricError … / frameState.pixelRatio`), so
    /// Cesium's `maximumScreenSpaceError` is in **CSS** pixels and this one is in device
    /// pixels. On a display whose pixel ratio is 1 the two units coincide, which is why
    /// nothing has noticed.
    ///
    /// **The number below is therefore tied to a viewport height, and here is the
    /// arithmetic.** `terrain_lod_factor = H / (E · 2·tan(fovy/2))`, so at a fixed `E` the
    /// threshold distance is proportional to `H` and inversely proportional to
    /// `2·tan(fovy/2)`. The 12 was picked at the cost table's own rung — **1280×720,
    /// `CameraMode::Free`** (fovy 46.40°, `2·tan(fovy/2) = 0.857`) — where it gives
    /// `720 / (12 · 0.857) = 70.0 Mm⁻¹`. Against that:
    ///
    /// | viewport | mode | H | 2·tan(fovy/2) | factor | `E` for the same threshold |
    /// |---|---|--:|--:|--:|--:|
    /// | 1280×720 (the table below) | Free | 720 | 0.857 | 1.000× | 12 px |
    /// | 1920×1080 desktop | Free | 1080 | 0.857 | **1.500×** | 18 px |
    /// | S23 **landscape** 2340×1080 | Free | 1080 | 0.857 | 1.500× | 18 px |
    /// | S23 landscape 2340×1080 | Cockpit | 1080 | 1.155 | 1.113× | 13.4 px |
    /// | S23 **portrait** 1080×2340 | Free | 2340 | 0.857 | **3.250×** | 39 px |
    /// | S23 portrait 1080×2340 | Cockpit | 2340 | 1.155 | **2.413×** | 29 px |
    ///
    /// Read against the 1080p desktop instead of the table's rung, the S23 in portrait asks
    /// for a threshold distance **2.167×** the desktop's in Free (`2340/1080`) and
    /// **1.608×** it in Cockpit — the fovy term gives back a factor 0.742 of the height
    /// term. Landscape is the flat case: the S23's landscape height *is* 1080, so in Free
    /// it is bit-identical to the desktop and in Cockpit it is 0.742× it.
    ///
    /// **So the shipped 12 is not the same configuration on a phone**, and §9 F3's soak —
    /// which runs cockpit view — would not be measuring the desktop's calibration unless
    /// the value is re-chosen there. It is left at 12 deliberately: it is calibrated
    /// against the desktop measurement below, and dividing by a pixel ratio (or by
    /// `H / 720`) would demand a new calibration that cannot be done without the device.
    /// §9 F3 carries the same table and the instruction to pick it on the phone.
    ///
    /// **[`lod_factor_for`] has the identical units question and must not be touched**: its
    /// `target_texel_ratio` default is calibrated against the hard-coded `2.0` that shipped
    /// before WP3, on the same physical height, and the LOD harness's 204-pose CSVs are
    /// pinned to it byte for byte.
    ///
    /// [`terrain_lod_factor_for`]: crate::globe::quadtree::terrain_lod_factor_for
    /// [`lod_factor_for`]: crate::globe::quadtree::lod_factor_for
    ///
    /// # The measured cost, and why the default is what it is
    ///
    /// Visible tiles summed over the ten real-DEM poses of
    /// `testing::terrain::test_terrain_lod::e1_cost_of_the_geometric_term_on_real_terrain`,
    /// and the p95 projected geometric error left on screen at `alps_inn_valley`:
    ///
    /// | budget | tiles | vs off | p95 error at `alps_inn_valley` |
    /// |--:|--:|--:|--:|
    /// | off  | 483 | — | 22.0 px |
    /// | 24 px | 493 | +2 % | 18.6 px |
    /// | 16 px | 532 | +10 % | 14.9 px |
    /// | **12 px** | **707** | **+46 %** | **10.7 px** |
    /// | 10 px | 867 | +80 % | 8.9 px |
    /// | 8 px | 1 210 | +150 % | 7.5 px |
    /// | 4 px | 3 350 | +593 % | 6.4 px |
    ///
    /// 12 is the knee, read off the *marginal* column rather than the total: 16 → 12 buys
    /// 4.2 px for 175 tiles, 12 → 10 buys 1.8 px for 160, and 10 → 8 buys 1.4 px for 343.
    /// Phase F re-measures it on device, where the answer may well differ between desktop
    /// and an S23 — the same split §9 already anticipates for `mesh_segments`.
    ///
    /// The other half of the table is the half E1 exists for: at every budget above,
    /// `po_plain_to_alps` — flat ground, same screen area — moves by **0 to 3 tiles**
    /// while `alps_inn_valley` doubles. The knob costs what the ground is worth.
    pub max_geometric_error_px: f32,
    /// **F5** — the deepest level the geometric term may demand refinement *into*
    /// (`docs/terrain-plan.md` §9 F5). At and below it a node's stored error is zero and
    /// `apply_lod` refines on imagery sharpness alone.
    ///
    /// Defaults to [`DETAIL_MAX_Z`], which F5 raised from 15 to **19**: the source tile is
    /// 256² and the mesh is 17², so a z15 tile draws a 16:1 decimation of data it already
    /// holds and four more levels of it are resolvable before the lattice reaches 1:1. E1's
    /// 15 came from Cesium, where the heightmap *is* the mesh lattice and the argument
    /// holds; see `DETAIL_MAX_Z` for why it does not hold here.
    ///
    /// A knob rather than a constant because it is the one column §9 F5's cost table sweeps,
    /// and because it is the natural thing for a device measurement to lower: it trades
    /// near-field shape for tiles one level at a time.
    pub detail_max_z: u8,
}

impl Default for TerrainConfig {
    fn default() -> Self {
        Self {
            // §9 F4. Desktop on, Android still off — see `TERRAIN_ENABLED_BY_DEFAULT`.
            enabled: TERRAIN_ENABLED_BY_DEFAULT,
            source_url: TERRARIUM_URL.to_string(),
            max_level: TERRARIUM_MAX_LEVEL,
            exaggeration: 1.0,
            ocean: OceanPolicy::ClampToZero,
            // §5 B4's slice, resized by §9 F2b's measurement of the working set it has to
            // hold: 48 MiB on desktop (381 entries against a measured worst case of 220),
            // 32 MiB on Android. See `HEIGHT_CACHE_BUDGET_BYTES`.
            height_cache_budget_bytes: HEIGHT_CACHE_BUDGET_BYTES,
            occlusion: crate::globe::quadtree::TerrainOcclusionConfig::default(),
            // E1. Cesium's own `maximumScreenSpaceError` default, kept until the cost
            // table in `docs/terrain-plan.md` §8 gives a reason to move it.
            max_geometric_error_px: 12.0,
            // F5. 19, not E1's 15 — `DETAIL_MAX_Z` carries the whole argument.
            detail_max_z: crate::globe::terrain::DETAIL_MAX_Z,
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
    /// camera altitude, which drives `QuadtreeNode::apply_lod`'s threshold relaxation —
    /// since E1c deleted `Stage::Fog`, the only consumer there is. See
    /// `crate::globe::quadtree::fog`'s module doc comment for the full story, including
    /// what that stage was measured to remove (nothing) before it went.
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
    /// Terrain height data. On by default except on Android — see [`TerrainConfig`] and
    /// [`TERRAIN_ENABLED_BY_DEFAULT`].
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
            mesh_cache_size: NonZeroUsize::new(MESH_CACHE_ENTRIES).unwrap(),
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
        assert_eq!(
            entries.get() * 512 * 512 * 4,
            config.tile_cache_budget_bytes
        );
    }

    #[test]
    fn default_config_max_zoom_is_19() {
        let config = TileEngineConfig::default();
        assert_eq!(config.max_zoom, 19);
    }

    /// Terrain off must leave the imagery budget literally untouched — the flat path may
    /// not move. Since §9 F4 that is no longer the default on desktop, so the flag is set
    /// here rather than assumed; the property is about the flag, not about the default.
    #[test]
    fn terrain_off_leaves_the_imagery_budget_alone() {
        let mut config = TileEngineConfig::default();
        config.terrain.enabled = false;
        assert_eq!(
            config.imagery_cache_budget_bytes(),
            config.tile_cache_budget_bytes
        );
    }

    /// **§9 F4** — the flip, and the hole in it, as one assertion each.
    ///
    /// Desktop ships terrain on; Android does not, because the soak §9 F3 specifies has
    /// not been run. If this test is what fails after someone runs it and flips the
    /// constant, that is the test doing its job: the Android arm of F4 is a decision, and
    /// decisions are meant to be visible when they change.
    #[test]
    fn terrain_ships_on_everywhere_except_android() {
        assert_eq!(
            TerrainConfig::default().enabled,
            TERRAIN_ENABLED_BY_DEFAULT,
            "the default must come from the one constant that states the split"
        );
        #[cfg(not(target_os = "android"))]
        assert!(
            TERRAIN_ENABLED_BY_DEFAULT,
            "desktop ships terrain on — `docs/terrain-plan.md` §9 F1/F2"
        );
        #[cfg(target_os = "android")]
        assert!(
            !TERRAIN_ENABLED_BY_DEFAULT,
            "Android stays off until the S23 soak of `docs/terrain-plan.md` §9 F3 is run"
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

    /// §5 B4's "resident = budget / tile size" line, at the budget §9 F2b resized it to.
    ///
    /// The tile-size literal was `132_096` — the figure from **before** E1 added its
    /// two-byte error term — and had been failing since, which nothing noticed because this
    /// crate's own unit tests are not in the `culling::` gate. `HEIGHT_TILE_BYTES`'s doc
    /// comment already quotes the right number, and so does
    /// `terrain::test_terrain_lod::the_error_term_costs_two_bytes_a_tile`.
    ///
    /// The entry count is platform-split now, because the budget is: F2b measured the
    /// desktop working set and Android's is still held to the unrun soak of §9 F3.
    #[test]
    fn the_default_height_budget_lands_on_the_measured_entry_count() {
        let terrain = TerrainConfig::default();
        // E1's two bytes plus F5's 168-byte pyramid, on top of the samples and the mips.
        assert_eq!(HEIGHT_TILE_BYTES, 132_274);
        assert_eq!(
            terrain.height_cache_budget_bytes, HEIGHT_CACHE_BUDGET_BYTES,
            "the default must come from the one constant that states the platform split"
        );
        #[cfg(not(target_os = "android"))]
        assert_eq!(terrain.height_cache_budget_bytes / HEIGHT_TILE_BYTES, 761);
        #[cfg(target_os = "android")]
        assert_eq!(terrain.height_cache_budget_bytes / HEIGHT_TILE_BYTES, 253);
    }

    #[test]
    fn terrain_defaults_are_terrarium_and_clamped() {
        let terrain = TerrainConfig::default();
        assert_eq!(terrain.source_url, TERRARIUM_URL);
        assert_eq!(terrain.max_level, 15);
        assert_eq!(terrain.exaggeration, 1.0);
        assert_eq!(terrain.ocean, OceanPolicy::ClampToZero);
    }
}
