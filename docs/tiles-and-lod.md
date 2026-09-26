# Tiles and level of detail

How the globe's imagery gets from a server (or from a bundled SVG) onto the screen, and
how the quadtree decides how deep to refine. Culling — whether a tile is drawn at all —
is [culling-implementation.md](culling-implementation.md); relief and the geometric half
of the LOD rule are [terrain.md](terrain.md).

Code: `crates/cesium-engine/src/globe/tiles/` (sources, fetching, caches, mesh worker),
`globe/quadtree/quadtree.rs` (`apply_lod`, `lod_factor_for`), `globe/quadtree/fog.rs`,
and the per-frame driver in `render/wgpu_state.rs`.

## The tiling

The globe is the Web Mercator XYZ tiling laid onto the WGS84 ellipsoid. A `TileId` is
`(z, x, y)` with `y` growing southward; `tile_bounds` (`quadtree/tile_id.rs`) is the only
function that turns one into a longitude/latitude rectangle, in f64, and both the culler
and the mesh builder call it (invariant I-5). Two adjustments to plain Web Mercator:

- **Pole stretch.** Mercator stops at ±85.0511°. The top and bottom tile rows are
  stretched to ±90° so the globe has no holes at the poles; the mesh pulls its outermost
  row to the pole as a cap.
- **Roots at z = 1.** The quadtree starts from the four z1 tiles, not the single z0 one,
  so the root split is the equator and the prime meridian.

`MAX_ZOOM` is 20 (the harness and fixture ceiling); `TileEngineConfig::max_zoom`, the
deepest the running quadtree refines, is 19.

Every tile is drawn as a `(segments + 3)²` vertex grid (`mesh_segments = 16`: a 17×17
surface lattice plus a skirt ring hanging inward along all four edges to hide cracks
between neighbours at different levels): 361 vertices and 1 944 16-bit indices, positions stored as f32
offsets from the tile centre (`TileMesh`, `globe/geometry.rs`).

## Imagery sources

`TileSourceMode` (`tiles/config.rs`) is either `HttpNetwork` with a `{z}/{x}/{y}` URL
template, or `SvgVector` with a parsed world map. Each map style also carries its own
**imagery depth cap**, `TileEngineConfig::imagery_max_level`, separate from the
quadtree's `max_zoom`:

| Style | Source | Tile size | Cap | Where the cap comes from |
|---|---|---|---|---|
| Standard | CARTO `dark_nolabels`, `@2x` | 512 | 20 | CARTO's documented native maximum. The service keeps answering deeper, with a vector re-render and no new content, so the cap is the engine declining to ask past the source's stated resolution. |
| Satellite | Esri World Imagery | 256 | 17 | Measured. The service advertises levels to z23, which is the tile scheme, not the photography. Past the real coverage it answers `200 OK` with a byte-identical 2 521-byte near-white placeholder that decodes like any other tile. Probed at a city, farmland in two countries, rainforest, desert, taiga and open ocean: real imagery ran to z19–z20 at the first four and only to z17 at desert, taiga and ocean. z17 was real everywhere tested. |
| Offline | Bundled Natural Earth SVG | 512 | 9 | The generator clamps every feature's minimum zoom to 9 (`tools/generate_world_svg.py`, `engine_min_zoom`); past it nothing new is drawn, only the same paths larger. |

A visible tile deeper than the cap is never requested at its own level:
`sync_imagery_requests` maps it with `TileId::ancestor_at_level(cap)` and the capped
ancestor is fetched instead. Drawing then goes through the ordinary "texture not yet
loaded" fallback, which stretches the ancestor's texture over the deeper tile. So a
capped tile looks exactly like one whose own imagery is still in flight, and terrain
geometry keeps refining below the imagery cap. The test
`the_sources_that_can_run_out_of_content_cap_shallower_than_max_zoom` holds the two
capped styles strictly below `max_zoom`; were either to reach it, the cap would silently
become a no-op.

Both online keys are read **at compile time** with `option_env!` (`CARTO_API_KEY`,
`ESRI_API_KEY`), so no key is ever in the source. Without a key CARTO still answers
`200` with every tile watermarked, and Esri falls back to its keyless non-commercial
endpoint; both log one warning.

### The offline vector map

`assets/maps/world_vector_dark.svg.gz` is Natural Earth 1:10m (land, urban areas, lakes,
rivers, coastline, state and country borders, airports) projected to Web Mercator by
`tools/generate_world_svg.py` into a `2²²`-unit square viewBox with integer coordinates,
so every vertex is exact in f32 (about 10 m at the equator). It is embedded in the binary
(`vector::WORLD_SVG`) and parsed once per process (`bundled_world_renderer`, a
`OnceLock`), then shared by `Arc` with every fetch worker.

`SvgTileRenderer` (`tiles/vector/svg_renderer.rs`) departs from plain SVG in three ways
the generator writes the file to match:

- **Stroke widths and dash lengths are output pixels**, not SVG units: a 1 px border stays
  1 px at every zoom instead of growing a hundredfold by z10.
- **A path whose id ends in `.z<N>`** is drawn only on tiles of zoom ≥ N.
- Only solid colours; gradients, patterns, text and images are ignored.

It does not use `resvg::render`, which walks and rasterises every path of the document for
every tile. The document is flattened once into runs (one ring or line each) with
bounding boxes; a tile touches only the runs overlapping it, clips them to the tile plus
an 8 px margin before stroking, drops vertices closer than half a pixel, and strokes
dashed runs one by one from their own phase so dashes line up across tile seams. A tile
rasterises in 1–4 ms on a desktop core (tiny-skia, on tokio's blocking pool).

## Fetching

`TileFetcher` (`tiles/tile_fetcher.rs`) owns a multi-thread tokio runtime, a `reqwest`
client (5 s timeout) and a priority queue. There is one fetcher for imagery and, with
terrain on, a second for heights.

**The request list is declarative.** Every frame `TileSystem` hands the fetcher its
complete wish list, `(tile, class, importance)`, through `sync`:

- new tiles are queued; queued tiles whose class changed, or whose importance moved by
  more than 10 %, get a new heap entry (a generation counter lets the stale one be
  skipped when popped);
- a queued tile absent from the list for **500 ms** (`CANCEL_GRACE`, wall time) is dropped
  and its cache placeholder forgotten, so it can be asked for again. Requests already on
  the wire finish.

A newest-first queue starves: a moving camera requests new near tiles every frame, and
anything older — including the coarse ancestors every fallback depends on — never reaches
the front. Measured before the queue was ranked, a z0 height tile requested at 0.02 s had
still not arrived 7 s later.

**Ordering.** Class first (`High` for everything the view needs, `Low` for prefetch),
then **importance**, the tile's ground width over its distance from the camera — roughly
the angle it subtends:

```text
importance = width / max(distance − 0.7·width, 0.05·width)
```

Coarse tiles are wide, so the ancestors the fallbacks rely on rank high even when far
away, and near detail ranks high by being near. Cesium orders its load queue the same way.
Ground points the camera collision will test rank above everything (`1e6`), because a
wrong ground there moves the camera.

**Concurrency.** 16 requests in flight. The worker acquires a slot *before* popping, so
the request that goes out is the most important one at the moment a slot frees, not the
one that was most important when the previous request was dispatched. PNG and JPEG
decoding runs on tokio's blocking pool; an HTTP error, a network error or a decode error
marks the tile `Failed`.

## Caches

### Imagery textures

`TileCacheManager<T>` (`tiles/tile_cache.rs`) is used for both imagery textures and
height tiles. It keeps two stores:

- **`ready`**: an LRU of loaded tiles with a **soft** capacity. Eviction only takes entries
  not used in this frame or the previous one; when every entry is in use the cache grows
  past its capacity rather than evict something on screen, and shrinks back once the view
  moves on.
- **`placeholders`**: `Fetching` and `Failed(since)` markers, outside the LRU. When they
  lived in the LRU, a view needing more tiles than the capacity evicted its own
  placeholders and re-requested them the next frame: 78 837 placeholder evictions were
  measured in a 1 620-frame run near the ground. A `Failed` marker expires after
  `negative_cache_duration` (10 s) and the tile may be requested again.

Levels z ≤ 2 (1 + 4 + 16 tiles) are **pinned** in both the imagery and the height cache,
so every point of the globe always has some ancestor to fall back to.

**The budget is bytes, not entries.** `tile_cache_budget_bytes` is 512 MiB. The entry
count is derived from it the first time a tile of the current style decodes
(`tile_cache_entries_for`): 512×512 RGBA tiles are 1 MiB each, so CARTO gets 512
entries; 256×256 Esri tiles are 256 KiB, so the 2 048-entry cap binds first. The
floor is 64 entries. A count-only cap let textures pass 1.9 GB on a device that had
1.4 GB available. With terrain on, the height cache's slice (96 MiB desktop, 32 MiB
Android) is taken out of the same budget, so turning terrain on does not raise the
engine's total ceiling (`TileEngineConfig::imagery_cache_budget_bytes`).

Uploads are capped per frame at **30 textures or 2 ms**, whichever comes first (at least
one always goes, so the backlog drains). An upload is a texture, a 256 KB–1 MB copy and a
bind group; without the cap a zoom-out landed hundreds at once and exhausted VRAM on an
integrated GPU. Textures are `Rgba8UnormSrgb` without mipmaps. A 1×1 texture in
`base_color` (20, 20, 20) is the last-resort fallback when no ancestor has imagery either.

`ObservedTextureSize` records the real decoded tile width, which the LOD rule reads; until
the first tile of a style arrives it reports 512 (`DEFAULT_IMAGERY_TEXTURE_SIZE_PX`).
Switching style rebuilds the fetcher and clears the texture cache but keeps the bind
group layout and sampler, which the compiled pipelines depend on.

### Meshes

`MeshCache` (`render/mesh_cache.rs`) holds the uploaded vertex and index buffers with the
same soft-capacity rule: 4 096 entries on desktop, 1 536 on Android (about 15 kB each).
It was raised from 512 because a single 360° orbit near the ground draws more meshes than
that, and the second lap evicted and rebuilt 786 of them.

## Meshes: worker, synchronous builds, fallback

A tile's mesh is a pure function of `(id, segments, BuildCtx)` (`TileMesh::generate_on`),
built by one of two paths:

- **`MeshWorkerPool`** (`tiles/mesh_worker.rs`) runs builds on the rayon pool and returns
  them through a `sync_channel(512)`. Requests are deduplicated on a `requested` set. The
  build input (`MeshBuild::Flat`, or `MeshBuild::Terrain` with an already sampled
  `HeightPatch`) is prepared on the main thread, because the height cache promotes in an
  LRU and cannot be touched from a worker. At most 48 finished meshes become GPU buffers
  per frame; a worker result never replaces a resident mesh built from deeper height data.
- **`build_missing_meshes_now`** builds on the main thread, within **3 ms** a frame: first
  every missing ancestor of a visible tile, coarse to fine, then the missing visible tiles
  nearest first. A flat mesh costs about 25 µs.

The synchronous path exists for the frame a tile enters the view. Before it, such a tile
had no mesh until a worker delivered one a frame or two later, and the renderable set
fell back to the nearest ancestor with a mesh — for a direction never looked at before,
a z3–z4 tile covering the whole screen, the camera's surroundings included. In a
collision run over Innsbruck that drew a "ground" up to 1.5 km above the camera in 567
frames.

`QuadtreeManager::get_renderable_tiles` makes the fallback explicit: for each visible
branch it returns the visible leaves if every one of them has a mesh, and otherwise the
branch's own node. Ancestors of visible tiles are kept resident and requested for exactly
this reason.

## Which texture a tile shows

`WgpuState::update_display_state` assigns textures to drawn tiles under rules that trade a
little sharpness for never blinking:

1. A tile new to the view starts on the best texture available — its own, or the nearest
   ancestor's with a UV window (`compute_fallback_uv`: halve the scale and add the
   quadrant offset once per level).
2. It switches to its own texture only when **all four siblings** have theirs, so a parent
   is replaced by four children at once rather than piecemeal; or after **2 s**, so a tile
   whose sibling 404s is not stuck blurred; or immediately if it has shown its own texture
   before (an LRU of 4 096 such tiles).
3. Once a tile shows its own texture it is **never downgraded** to a fallback, even if a
   transient eviction makes the fallback momentarily the only thing ready.
4. A tile that leaves the visible set keeps its entry, and is still drawn, for **200 ms**,
   which covers the longest LOD oscillation measured (about 10 frames at 60 fps).

At draw time, if the assigned texture has since been evicted, the nearest ready ancestor
stands in with a freshly computed UV window.

## Level of detail

`QuadtreeNode::apply_lod` runs for every node that survived culling:

```text
dist            = ‖centre − eye‖                         (f64 subtraction)
imagery_dist    = unstretched_radius · lod_factor · (1 − fog(d_box, density))
subdivide_dist  = imagery_dist                          (flat globe)
                = max(imagery_dist, terrain_dist)       (terrain, see terrain.md)
refine while      dist < subdivide_dist        (not yet subdivided)
keep refined while dist < 1.2 · subdivide_dist  (already subdivided)
```

- **`unstretched_radius`** is the greatest distance from the tile centre to a sampled
  point of the *un*-stretched rectangle (`tile_bounds_unstretched`): a polar row's true
  ground extent, not its pull to ±90°. Polar caps therefore refine a little late, which
  costs false positives and never a hole.
- **The 20 % hysteresis** keeps a node straddling the threshold from subdividing and
  collapsing on alternate frames. A 5 % band is about 50 m at z19 and produced visible
  appear/disappear flicker on high-detail tiles.
- **Children are reordered near to far** each update (quadrant nearest the eye first,
  judged in the tile's east/north frame), so the collected tile list is a front-to-back
  hint for early depth rejection. The reorder is keyed by quadrant identity, not array
  position, which keeps it idempotent when called every frame.
- The subtree refines within the same call, so the tree reaches full depth in one
  `update`; `max_zoom` bounds it.

### Where `lod_factor` comes from

The rule has Cesium's shape — refine while `d < G · H / (maxSSE · 2·tan(fovy/2))` — with
the imagery variables made explicit (`lod_factor_for`):

```text
lod_factor = (C / texture_size_px) · viewport_height_px · √target_texel_ratio / (2·tan(fovy/2))
C          = 256 / 315
```

- **`target_texel_ratio`** (default 1.0) is imagery texels demanded per screen pixel. It is
  an *area* ratio, and `lod_factor` scales a linear distance, so it enters as a square
  root. Higher is sharper and more expensive. It is a texel-density target, not a
  geometric screen-space error; the geometric term is separate and exists only with
  terrain.
- **`texture_size_px`** is the live decoded tile width (512 for CARTO and the offline
  map, 256 for Esri), so a 256-pixel style refines one level deeper for the same
  sharpness instead of looking half as sharp.
- **`C = 256/315`** is a calibration constant, not a geometric one. The true ratio of a
  tile's ground width to `unstretched_radius` is about √2 at every level; `C` was fitted
  so that the reference configuration reproduces the long-standing `lod_factor = 2.0`
  exactly, in rationals: with `texture_size = 512`, `H = 1080`, ratio 1 and the default
  28 mm lens on a 24 mm sensor, `tan(fovy/2) = 3/7` and
  `(256·1080·7)/(315·512·6) = 2`. The 204-pose LOD harness baselines rest on that
  identity; changing the default focal length would require re-deriving `C`.
- **`viewport_height_px`** is the swapchain height **divided by the window's scale
  factor** (`imagery_lod_height_px`), i.e. CSS-equivalent pixels, as Cesium's
  `frameState.pixelRatio` does. Undivided, a phone at scale factor 2.6–4 asked for imagery
  several levels deeper than the same framing needs on desktop — past Esri's real
  coverage, into the placeholder tiles. Headless and scale-factor-1 windows are
  unaffected, which keeps the calibration above intact.

`lod_factor` is recomputed every frame from the current viewport, field of view and
texture size; nothing is cached.

### Measuring LOD

`src/testing/lod/` scores every visible tile at 204 poses by `texels / screen_px`
(texture area over the tile's projected area; 1.0 is one texel per pixel) and, for the
terrain arm, by projected geometric error in pixels. It is a measuring instrument: it
reports, it does not assert targets. `LodDistanceMode::Box` — measuring `dist` to the
nearest point of the node's box instead of its centre — exists only as a measurement
switch: at an equal `target_texel_ratio` it drew about 70 % more tiles and changed 24 % of
subdivision decisions, so production measures to the centre.

## Atmospheric fog

`globe/quadtree/fog.rs` ports CesiumJS's fog (`Scene/Fog.js`, `CesiumMath.fog`), constants
in metres exactly as Cesium defines them:

```text
fog(d, density) = 1 − exp(−(d·density)²)
density(h)      = 0.0006 · 0.001 · max(h / 800 000 m, 1e-4)^−0.59     for h ≤ 800 km, else 0
```

Cesium's additional camera-tilt factor (fog thickening as the view tilts to the horizon)
is not ported: it depends on where the camera points, not on any tile, and the numbers
here are therefore a nadir-view lower bound on what Cesium would do.

**Fog has exactly one effect: it relaxes refinement.** In `apply_lod` the imagery term is
multiplied by `1 − fog(d)`, with `d` the distance to the nearest point of the node's box.
A tile half-hidden in fog needs half the refinement distance; a fully fogged tile stops
refining. The relaxation is applied before the hysteresis band is derived, so the band
stays a constant fraction as fog thickens.

**Fog never culls.** Cesium also culls a tile once `fog` reaches 1. That test can never
fire here: `fog` saturates at `d·density ≈ 4.16`, and at every altitude this engine
flies that distance lies beyond the horizon, which the limb test has already culled
everything past. At 10 km altitude, for instance, fog saturates about 520 km out, while
the horizon is 357 km away. Evaluated against the real tree over all 204 bench poses and
ten real-terrain poses, the predicate culls zero tiles (`testing::lod::test_wp5_fog`
reconstructs it from outside the engine and asserts exactly that, so a change of fog
constants that made it matter would be noticed).

**Fog does not relax terrain shape.** With relief, coarsening the far field has a visible
cost — distant mountains lose their silhouette — and haze hides texture, not outlines.
`TerrainFogPolicy::ImageryOnly` (the default) applies fog to the imagery term only.
Measured at an equal tile budget over ten real-DEM poses, it left 84.3 px of summed
far-field p95 geometric error against 106.3 px for applying the imagery relaxation to the
terrain term too (`Relax`) and 91.6 px for Cesium's own form, which widens the pixel
budget by `fog·sse` instead (`CesiumSse`). The other two policies remain as measurement
options; `FogConfig::sse` is read only by `CesiumSse`.

Fog in the *picture* — distant ground fading into the sky colour — is unrelated: it is the
shader's aerial perspective, measured in air mass rather than distance, and is described
in [lighting.md](lighting.md).

## Prefetch

With `enable_prefetch` (on by default), `sync_imagery_requests` estimates the camera's
velocity from its movement since the last frame. For every visible tile at z ≥ 4 whose
centre lies within about 60° of that direction (`dot > 0.5`), the four edge neighbours of
its capped imagery tile are added at `Low` priority, below everything the current view
needs. They are fetched only while connection slots are free and are dropped like any
other request once they leave the wish list for 500 ms. `TileEngineConfig::prefetch_radius`
is not read; the ring is always the four direct neighbours.

## Terrain in the same pipeline

With terrain on, the same `TileSystem::update` also sends a **height** wish list: the
source height tile of every missing mesh, of every drawn mesh built from coarser data than
its own tile, and of every ground point the camera collision will test. Meshes are built
from whatever height data is resident and rebuilt when better data lands (at most 32 per
frame). All of that is in [terrain.md](terrain.md).
