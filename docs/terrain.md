# Terrain

How the globe gets relief: where the heights come from, how they are cached, how a tile's
mesh is built from them, how culling stays correct once geometry leaves the ellipsoid,
how tiles hidden behind mountains are culled, how the LOD rule refines on shape, and how
the camera, the aircraft and the labels find the ground.

Code: `crates/cesium-engine/src/globe/terrain/` (`height_tile.rs`, `height_cache.rs`,
`heightfield.rs`, `corridor.rs`), `globe/quadtree/surface.rs`, `any.rs`,
`terrain_occlusion.rs`, `terrain_relief.rs`, and the terrain half of `globe/tiles/system.rs`.

Everything below applies only while terrain is on. With terrain off the engine is the
flat ellipsoid globe, and — the design constraint the whole subsystem is built around —
the flat path runs **the same instructions it would run if terrain did not exist**: no
height cache, no second fetcher, no per-node branch, no extra byte per node.

## Switching it on

`TerrainConfig` (`tiles/config.rs`) lives on `TileEngineConfig::terrain`:

| Field | Default | Meaning |
|---|---|---|
| `enabled` | desktop `true`, Android `false` (`TERRAIN_ENABLED_BY_DEFAULT`) | Master switch |
| `source_url` | Terrarium on AWS Open Data | `{z}/{x}/{y}` template |
| `max_level` | 15 | Deepest level the source serves |
| `exaggeration` | 1.0 | Vertical exaggeration, applied once (below) |
| `ocean` | `ClampToZero` | Sub-sea-level policy |
| `height_cache_budget_bytes` | 96 MiB desktop, 32 MiB Android | The height cache's slice of the tile budget |
| `occlusion` | see below | Culling behind mountains |
| `max_geometric_error_px` | 12 | Terrain LOD budget |
| `detail_max_z` | 19 | Deepest level the geometric LOD term may refine into |

The desktop viewer does not simply inherit the default: `CesiumViewerBuilder` lets the map
style decide (satellite-terrain on, standard off unless `.terrain(true)`, offline always
off, because heights come from the network). On Android `android_main` runs
`TileEngineConfig::default()`, so terrain starts off there, and `nativeSetTerrainEnabled`
turns it on at run time. The Android default is off because terrain has not been
measured on a device, not because it has been measured to be too expensive.

A run-time switch (`WgpuState::set_terrain_enabled`) builds or drops the height manager,
rebuilds the quadtree from its roots (the two surface models are different types),
clears every cached mesh and re-derives the imagery cache's share of the byte budget.

## The height source: Terrarium

Mapzen/Tilezen Terrarium tiles, `https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{z}/{x}/{y}.png`:
free, keyless, the same Web Mercator XYZ scheme as the imagery, 256×256 RGB PNG.

```text
h_metres = R·256 + G + B/256 − 32768
```

R and G carry whole metres, B 1/256 m. `decode_texel` forms the value in exact integer
1/256-metre units and divides once, so every decoded height is exactly representable in
f32 and the decode is bit-reproducible on every platform; that is what lets the fixture
tests pin exact extrema rather than ranges.

The source's deepest level is **z15** (z16 answers 404), while the quadtree refines to
z19. Every tile below z15 is answered from its z15 ancestor, so ancestor upsampling is
the normal path, not a fallback (below).

### The ocean

Terrarium's open ocean is **bathymetry**: a mid-Pacific z12 tile measures −4 324 m to
−2 276 m. Drawn verbatim, the sea floor becomes the sea surface and every coastline a
multi-kilometre cliff. `OceanPolicy::ClampToZero` (the default) clamps `h < 0` to 0 at
decode time, so the sea is the ellipsoid. The price is stated rather than hidden: land
genuinely below sea level is flattened with it — the Dead Sea's z12 tile is uniformly
−412 m, Death Valley −86 m — which is the right trade for a view that is mostly ocean.
`OceanPolicy::Raw` (`--terrain-ocean raw`) keeps the source verbatim.

## A decoded height tile

`HeightTile` (`terrain/height_tile.rs`) stores:

- **65 536 samples as `u16` steps across the tile's own range**, `h = base + q·scale`, with
  `scale` the smallest power-of-two multiple of 1/256 m that spans the range. Two bytes
  a texel, like whole-metre `i16`, but sub-metre: a tile spanning under 256 m keeps the
  source's full 1/256 m precision, one spanning 1 500 m of Alps is held to 1/32 m.
  Whole metres were not enough — the source steps 10–20 cm per texel over a runway, and
  rounding turned gently sloping ground into 1 m terraces about 3 m wide.
- **`h_min`/`h_max` over all 65 536 texels**, not over the 17×17 grid the mesh samples:
  a maximum taken on a subgrid misses summits between grid lines, which would put a
  summit outside its own bounding box.
- **A 16×16 min/max mip** (one cell per 16×16 texel block), rounded **outward** to whole
  metres (min down, max up), for bounds over any sub-rectangle.
- **`detail`**, the tile's measured geometric error, and **`detail_below`**, an 84-entry
  pyramid of the same measurement for descendants one to three levels down (see
  *Terrain LOD*).

The decode is about 1 ms on a desktop core (five passes over the texels for the mip and
the error pyramid) and runs on the rayon pool; a fast camera move lands a few dozen tiles
in one frame, which on the main thread was a 30 ms stall. A tile stays `Fetching` until
its decode returns, so nothing can be built from it early. One tile costs
`HEIGHT_TILE_BYTES = 132 274` bytes resident.

## The height cache

`HeightTileManager` (`terrain/height_cache.rs`) owns a `TileFetcher` pointed at the
source, a `TileCacheManager<Arc<HeightTile>>` (the same soft-capacity LRU as imagery, z ≤ 2
pinned), and every height query. It exists only while terrain is on.

**Budget.** The entry count is derived from the declared slice exactly as the imagery
count is from its budget: 96 MiB / 132 274 B = **761** tiles on desktop, 32 MiB = **253**
on Android. The slice is taken out of `tile_cache_budget_bytes`, not added to it. At the
heaviest of ten real-terrain poses the working set that production actually requests —
the visible set and its ancestor chains — is 220 tiles, so the desktop slice holds it
with a large margin for the extra tiles the terrain LOD refines.

**What is requested** (`TileSystem::sync_height_requests`), all at high priority and
ranked by the same importance as imagery:

1. the source tile of every mesh that is missing;
2. the source tile of every drawn mesh built from a coarser ancestor than its own source
   (so it can be rebuilt);
3. the z15 tile under every ground point the camera collision will test this frame, at
   an importance above any view tile.

The set is stable while the view is: a tile stays on the list until its mesh has been
built from its own data. A policy that wanted heights only while a mesh was missing made
the list flip every frame and the queue churn (9 898 height requests for 196 arrivals in
one run).

**Offline.** In `offline_mode` every request resolves synchronously to a tile of zeros,
so headless tests can run with terrain on, without a network, and see the flat globe's
geometry.

### Queries

Heights leave the cache in **megametres**; this module holds the only
metre-to-megametre factor in the terrain path.

- **`resolve_source(id)`**: the deepest *ready* tile answering for `id` — `id`'s own
  source (`id` clamped to z15) or the nearest ready ancestor.
- **`ancestor_uv`**: maps `(u, v)` in a tile into its ancestor's space. It is the same
  parent walk imagery uses for fallback textures (`compute_fallback_uv`): halve the scale
  and add the quadrant offset per level. Run against the height cache it is Cesium's
  `upsample()` as a UV transform, with no resampling pass and no extra memory. The
  offsets are dyadic rationals with at most 20 fractional bits, so they are exact.
- **`status_of(id)`**, three states: `Ready` when `id`'s own source has arrived *or failed
  with a final ancestor answering*, `Pending` while a fetch in the chain is outstanding,
  `Unavailable` when the whole chain up to z0 has failed. "Unknown" is never read as "sea
  level": a tile baked flat now has no reason ever to be rebuilt.
- **`height_at` / `peek_height_at`**: bilinear height at `(u, v)` of a tile, with or
  without LRU promotion.
- **`peek_height_at_lon_lat`**: the same, for a longitude and latitude, starting at z15
  (`tile_uv_at_lon_lat` inverts exactly the Mercator row definition the meshes are built
  from). Runway corridors are applied here.
- **`peek_mesh_height_at_lon_lat(lon, lat, z, segments)`**: the height of the *drawn
  triangle net* of a level-`z` tile, not of the bilinear field. Between two grid nodes
  the mesh is a plane, and over a valley floor the net chords above the dip, so the
  bilinear field reads lower than what is drawn. The triangulation is the mesh's own
  south-west–north-east bisection (also Cesium's `triangleInterpolateHeight`), so the
  answer is exact to curvature sagitta (about 3 mm at z12).

## The surface model: one type parameter, no per-node branch

`SurfaceModel` (`quadtree/surface.rs`) is a trait implemented by two zero-sized marker
types, and the quadtree, its nodes, patches and meshes are generic over it:

| | `Ellipsoid` (flat) | `Heightfield` (terrain) |
|---|---|---|
| `NodeExtra` (per node) | `()` | `HeightBounds` |
| `PatchExtra` (per patch and sub-patch) | `()` | `ScaledSphere` |
| `BuildCtx` (mesh input) | `()` | `HeightPatch` |
| `vertex_altitude` | 0, or −skirt | sampled height, or height − skirt |
| `vertex_normal` | ellipsoid normal | central differences on the field |
| `skirt_depth` | `0.5 / 2^z` Mm | measured from the edges |
| `is_occluded` (limb) | exact rectangle supremum | cone test on a sphere |
| `HAS_GEOMETRIC_ERROR` | `false` (compile time) | `true` |
| `occluder_floor` | −∞ | 4×4 ground floors |

**Why a type parameter and not a flag.** An `if terrain` inside the per-node loop costs a
branch in the hottest code of the engine and forces every terrain field onto every node
whether or not terrain is on. With zero-sized payloads, `QuadtreeNode<Ellipsoid>` stays
exactly 192 B (three cache lines) and `TilePatch<Ellipsoid>` 64 B
(`test_horizon_hot_structs_have_not_grown` pins both), and the flat instantiation
monomorphises to the code it was before the parameter existed. `S::HAS_GEOMETRIC_ERROR`
is a `const`, so on the flat arm `apply_lod` compiles to the imagery-only threshold with
no `max` and no second multiply.

**Where run time meets the type.** `TerrainConfig::enabled` is a run-time flag, and a value
cannot have a type chosen at run time, so something has to bridge them.
`AnyQuadtree` (`quadtree/any.rs`) is an enum of the two whole `QuadtreeManager`s; its
`match` runs once per call into the quadtree — a handful of times a frame — against tens
of thousands of nodes visited inside each call. The cost is a second monomorphisation in
the binary, not frame time. The mesh side bridges the same way: `MeshBuild` is `Flat` or
`Terrain(Box<HeightPatch>)`, matched once per mesh before the worker runs a separately
monomorphised `TileMesh::generate_on::<S>`.

## Relief meshes

### Sampling the patch

`HeightPatch::sample` runs on the main thread before a mesh is queued (the cache cannot
be touched from a worker, and the finished mesh must record which data it was built
from). It resolves the source once, then reads `(segments + 3)²` bilinear samples — 361
at the default 16 segments — on the mesh's own build grid, with vertical exaggeration
multiplied in. **This is the only place exaggeration is applied**; every bound, sphere
and occluder is derived from values that already carry it, and the raw-height queries
deliberately do not (`exaggeration_scales_heights_and_bounds_exactly_once`).

The outer ring of the grid is a **halo**: instead of repeating the edge sample it holds
the height one grid step outside the tile, read from the same source tile. Below z15 the
source is an ancestor that covers the neighbour too, so the halo is the neighbour's real
ground and the edge normals are true central differences. Where the tile *is* its own
source and sits on the source's border, the halo would fall outside the data;
`halo_valid` records that per side and the normal drops to a one-sided difference rather
than silently reporting half the slope.

**Which data a new tile may be built from.** The best resident data, even while the
tile's own height tile is still in flight — Cesium upsamples a parent's terrain the same
way — so a tile entering the view is drawn at once and rebuilt when its data lands. But
only from data at most **two levels** coarser than its own source
(`MAX_SOURCE_LEVEL_GAP`): a z12 valley tile built from z5 data is the average of 50 km of
Alps, a slab hanging a kilometre above the valley next to neighbours built from their
own data, with sky through the gaps. Until closer data arrives such a tile waits and the
renderer draws its nearest built ancestor, whose data matches its scale. If the tile's
own fetch has failed, the best ancestor is final and is used whatever the gap. If every
level up to z0 has failed, the tile is built as a flat `Ellipsoid` mesh — the same mesh
terrain-off would draw.

### Vertices, normals, skirts

- **Displacement is radial along the ellipsoid normal** at the vertex's surface point, for
  every model. A slope normal would drag vertices sideways out of their own culling
  rectangle (invariant I-5).
- **Normals** are the height field's gradient in the tile's local east/north frame:
  for `p(E,N) = p₀ + E·ê + N·n̂ + h·û` the normal is `û − h_E·ê − h_N·n̂`. Without them
  relief would show in silhouette and not in shading.
- **Skirts** hang each edge vertex inward by a depth measured from the content
  (`HeightPatch::derive_skirt`): the crack at an LOD boundary is the gap between this
  tile's edge and the straight line a coarser neighbour draws across the same edge. For
  coarsening factors k = 2 and 4 (a neighbour one or two levels up) the skirt is the
  maximum over the four edges of `|h[i] − lerp(h[i₀], h[i₁])|` plus the curvature
  sagitta `R·(1 − cos(k·δ/2))` of that chord. A neighbour three levels coarser would need
  more; adjacent visible tiles do not differ by that much. The flat globe keeps its fixed
  `0.5/2^z` Mm, which is pure sagitta. Pole caps get no skirt (it would open the cap) and
  skirt vertices take their edge vertex's normal.
- Every mesh declares the altitude interval its vertices lie in, skirts included
  (`declared_height_bounds`, invariant I-1′), and
  `test_heightfield::generated_meshes_stay_within_their_declared_height_bounds` holds the relief model to it.

### Rebuilding when better data arrives

Once relief exists, a mesh is no longer a function of its `TileId` alone: it also depends
on which height tile had arrived when it was built. `TileMesh::height_source` records
that, and the mesh cache entry carries it.

Each frame, `select_mesh_rebuilds` finds drawn meshes whose available source is
**strictly deeper** than the one they were built from (`fresher_height_source`, which asks
the same two questions the builder does, so a rebuild can never reproduce the identical
mesh forever), and queues at most **32 per frame**, nearest first, ties broken by
`(z, x, y)` so captures are reproducible. Tiles whose rebuild is already on a worker are
excluded before the budget is applied; otherwise a stale tile is re-picked every frame
until its replacement lands and blocks the tiles behind it.

- **No downgrade.** A shallower source (the deep tile evicted, a negative-cache expiry) is
  never offered: the mesh on the card is the better one, and a mesh downgrade is a
  hillside visibly dropping.
- **It terminates.** Every accepted rebuild strictly raises `height_source.z`, which is
  bounded by the source's depth, so there is nothing to oscillate and no grace period is
  needed.
- **It never shows a hole.** A rebuild replaces the cache entry only when the new buffers
  exist; the old mesh is drawn until then.

At the shipped density, sampling a patch costs 7.8 µs and building the mesh 16.1 µs off
the main thread, so the full budget of 32 is about 0.25 ms of sampling in the frames that
use it. The budget is a ceiling for bursts (a camera turning into new ground), not a rate.

## Culling with relief

The flat culler's proofs rely on one fact (invariant I-1): every drawn vertex is on the
ellipsoid or, for skirts, below it. Relief breaks it, and each test that depended on it is
replaced for `Heightfield`. The derivations are in [culling-math.md](culling-math.md) §3.7
and §4.0.

### Height-aware bounds

Each node carries a `HeightBounds` (`terrain/heightfield.rs`), all in megametres with
exaggeration applied:

| Field | What it bounds | Used by |
|---|---|---|
| `lo` | lowest point of the node's *geometry*, skirt included | the bounding box |
| `hi` | highest point of the geometry | the bounding box |
| `floor` | lowest point of the *ground* (no skirt) | the occlusion march |
| `floor_grid` | the same per cell of a 4×4 split of the tile | the occlusion march |
| `detail` | measured geometric error | the terrain LOD term |

`fit_obb` sweeps its sample grid at both `lo` and `hi`, so the node's box — and from it
the scaled-space sphere — contains the relief instead of hugging the ellipsoid under it.
Without this, geometry lifted by up to 2.9 km over the Alps sat outside boxes fitted at
altitude 0, and the tiles that should have filled the view were frustum-culled.

**Where the numbers come from** (`HeightTileManager::height_bounds_for`). Only once
`status_of(id)` is `Ready` — the same predicate the mesh builder uses, so bounds are never
taken from an ancestor while the tile's own data, which the mesh will be built from, is
still coming. The node's rectangle is mapped into its source tile, and the mip's
extrema are read over it rounded **outward** to whole cells with a one-cell halo
(bilinear reads touch a texel on each side of a cell boundary). `lo` subtracts
`skirt_allowance`: the largest height range over any aligned quarter of any of the four
edges (a linear interpolant never leaves the range of what it interpolates, and the k = 4
windows are exactly the quarters) plus the k = 4 sagitta. Bounding the skirt by the whole
tile's range instead made node intervals 1.53× the mesh interval (Everest z12: a 741 m
real skirt bounded by 4 700 m).

**Nodes without data yet.** A node is culled long before its own height tile arrives, and
a parent's interval is **not** a superset of its children's: a coarse DEM averages away a
peak a finer one resolves. So a new child inherits its parent's interval **widened by a
measured per-level margin** (`HEIGHT_INHERIT_MARGIN_M`, `Heightfield::child_extra`),
and the per-frame pass `refresh_extras` replaces the inheritance the frame real data
lands, top-down, without dropping the subtree:

| child z | worst excess in the corpus | margin |
|--:|--:|--:|
| 2 | 698 m | 20 000 m |
| 3 | 4 566 m | 20 000 m |
| 6 | 1 061 m | 8 000 m |
| 8 | 1 365 m | 8 000 m |
| 11 | 1 359 m | 6 000 m |
| 12 | 36 m | 1 500 m |
| 15 | 3 m | 200 m |

The corpus (`assets/terrain_fixtures/pyramid_extrema.csv`) holds the raw extrema of 788
tiles: z1→z15 chains over 16 regions with all four children at each step, 720
parent/child pairs. The z3 row is the argument in one number: the z3 tile over eastern
Greenland reports 7 796 m where its z2 parent reports 3 230 m, because a z2 texel (about
150 km) averages the spike away. The tightest level still has 4.4× headroom, which covers
exaggeration up to about 4. Below z15 the margin is exactly zero: child and parent read the
same tile, the child's rectangle is a dyadic sub-rectangle of the parent's, so its
covering mip cells are a subset of the parent's. `lo` additionally gets back an upper
bound on the parent's whole-tile skirt allowance (`inherit_allowance_mm`, a function of the
level only, so a cold chain cannot compound); `floor` pays the margin alone, which the
corpus measures directly (worst level 1 414 m at z9 against 6 000 m). Roots start from
the whole range the Earth's surface occupies, −11 500 m to 9 500 m.
`d1_inherit_margin_covers_the_corpus` and `d1_floor_inherit_margin_covers_the_corpus`
re-derive the tables from the CSV.

### The limb test

For a point on the ellipsoid, "below the horizon" is the linear inequality `q·c ≤ 1` in
scaled space, and the flat culler evaluates its exact supremum over the tile rectangle.
For a point 8 km above the ellipsoid that inequality says nothing: a summit can satisfy
it and stand in plain view over the limb. `Heightfield::is_occluded` therefore ignores
the rectangle and runs the cone test (Theorem 3.7) on a **scaled-space bounding sphere**
of the node's box. The sphere is fitted in scaled space around the eight vertices of the
transformed box, which contains the patch exactly with no sampling argument, and avoids
the easy mistake of dividing a real-space radius by `a` instead of `b`. The same test runs
per sub-patch. It culls nothing for an eye at or below the surface, where the cone
algebra breaks down. `test_terrain_visibility` checks it against the exact point test.

## Culling behind mountains

The limb test throws away what is behind the *planet*. A valley behind a ridge is in
front of the planet and would still be drawn. `Stage::TerrainOcclusion`
(`quadtree/terrain_occlusion.rs`) culls it, and it is sound: it removes only geometry it
has proved invisible, so it sits in the terrain arm's default pipeline
(`CullPipeline::TERRAIN_DEFAULT`) and is held to zero false negatives against the drawn
mesh (`test_terrain_occlusion::d3_never_hides_a_visible_vertex`). Cesium has no
equivalent.

### The march, once per frame

Whether a ridge hides a tile depends only on the camera, the bearing and the range, not
on the tile. So instead of marching toward each candidate, one march per frame fills a
polar grid around the camera — **96 azimuth sectors × 48 range rings**, rings
logarithmically spaced from 0.5 km to 120 km (a factor 1.12 per ring) — and each candidate
then costs a few lookups.

1. **Occluders are the quadtree's own nodes.** Each node already carries a sound lower
   bound on its ground (`floor_grid`, from the mip, rounded down). `refresh_terrain_horizon`
   walks the tree the previous frame left, top-down: a node wholly beyond 120 km is
   skipped; a node angularly larger than half the ring it sits on is descended into;
   otherwise its sixteen sub-cell floors are **stamped** into every cell their footprint
   can reach, each cell keeping the **minimum**. The nodes the walk stops at partition the
   globe, so every cell in range receives a floor and none holds an unearned one.
2. **`finish`** turns floors into the running maximum, per sector, of the elevation angle
   from the eye to each ring's floor, taken at the ring's far edge.
3. A candidate node is culled when the upper bound on its elevation angle, over its
   whole box, is below that ridge in **every** sector it spans, at a ring strictly nearer
   than the node.

Every approximation is rounded toward the weaker claim:

| Quantity | Bound | Why |
|---|---|---|
| occluder height | lower (`floor`, never `hi`) | only ground that is definitely there can definitely block |
| candidate elevation | upper (the whole box) | if the highest point is hidden, all of it is |
| a stamped node's footprint | outward | a notch drags the cell's minimum down |
| sectors a candidate spans | outward | the ridge is the minimum over them |
| rings in front of a candidate | inward | only a ridge strictly nearer can block |

Gaps in a ridge come out for free: a col inside a cell pulls the whole cell's minimum
down, so the test only fires where the ridge is continuous across the cell.

**Why it is sound.** The segment from the eye to a point `p` lies in the plane through the
Earth's centre containing both, so its ground track stays in `p`'s sector. The cull means
some ring `i` nearer than `p` has a ridge elevation above `p`'s. Two straight lines through
the eye cannot cross twice, so at ring `i`'s distance the sightline to `p` is below the
sightline to the ridge point, i.e. below altitude `floor[β][i]`, and the terrain over that
whole cell is at or above that floor: the sightline is inside the ground and cannot reach
`p`.

### Resolution, measured

A cell's floor is a minimum over its whole extent, so a cell larger than the ridge
averages the crest with the valley beside it and the ridge vanishes from the occluder.
Both axes were sized against that:

- **Rings.** 22 km from a crest about 2 km wide, 16 rings (7.2 km deep there) and 24 rings
  (5.1 km) lost the crest; 48 rings (2.4 km) kept it: floor 3 243 m against a 3 400 m crest.
- **Sectors.** At an Inn-valley pose 900 m up, with 114 tiles drawn without the stage:

  | sectors × rings | tiles removed | march |
  |---|--:|--:|
  | 24 × 48 | 7 | 460 µs |
  | 48 × 48 | 15 | 499 µs |
  | **96 × 48** | **20** | **541 µs** |
  | 96 × 96 | 21 | 651 µs |
  | 192 × 192 | 24 | 1 084 µs |

  Azimuth buys the culls and saturates around 96–144; rings add almost nothing past 48.
- **Within a node**: a single floor per node, sized by the *imagery* LOD, is the minimum
  over several kilometres and loses the ridge the same way. A 4×4 floor grid per node
  restored it (a z12 tile's whole-tile minimum 1 132 m, its southern row 3 176 m, against a
  3 400 m crest). The first version, with one floor per node, removed tiles only against
  the curvature horizon — flattening the test world's ridge changed its counts by zero.

### Placement error and the safety margin

The march bins nodes by an equirectangular angular distance in the engine's tiling
latitude and places walls with a rotation in geodetic latitude; the two metrics differ by
`κ = e²/2 = 3.35·10⁻³`. That along-track offset becomes an altitude error through the
curvature drop (`κ·s²/R`) and through the tilt of a sightline standing `Δalt` off the
floor (`κ·|Δalt|`, independent of range). Every floor is therefore lowered by

```text
ridge_safety_m = 1 m + 0.02 · (|Δalt| + s²/R)
```

A sweep over latitude, bearing, range, eye altitude and floor, plus 200 000 random draws,
measures the needed lowering at a flat ratio of 0.00336 of `|Δalt| + s²/R` (which is
`e²/2`, confirming the derivation) rising to 0.0055 at the far corner, so 0.02 is a 3.6×
reserve (`d3_ridge_safety_covers_its_placement_error`). Angular extents carry a further
10 % slack (`EXTENT_SLACK`); without it the ridge world's sweep reports 131 false
negatives.

### When the march runs

The march costs about half a millisecond whether or not it removes anything, so it only
runs where it pays:

- **Altitude gates.** Above **1 000 m above the ground** the march is not built. Measured
  on the real DEM through the real renderer at one pose:

  | height above ground | removed | march | frame time change |
  |--:|--:|--:|--:|
  | 67 m | 22 of 126 | 539 µs | −1.9 ms |
  | 317 m | 20 of 114 | 546 µs | −1.7 ms |
  | 717 m | 11 of 106 | 537 µs | −2.3 ms |
  | 1 217 m | 1 of 106 | 498 µs | noise |
  | 4 418 m | 0 of 87 | 436 µs | noise |

  The benefit falls off between 700 m and 1 200 m while the cost stays. A second gate at
  **12 km above the ellipsoid** covers the case where no ground sample exists yet and
  height above ground reads as height above the ellipsoid.
- **Relief pre-check** (`terrain_relief.rs`). Height alone does not predict the benefit:
  at some low poses nothing stands in front of the camera. Before allocating the grid,
  one walk of the *visible* leaves takes the largest elevation angle, at each leaf's
  nearest point, of the highest sub-cell floor it guarantees (not its box top, which on an
  inherited interval is kilometres of margin). Below **8°** the march is skipped. Over 34
  poses every pose where the stage removed nine or more tiles read 8.79° or more, and every
  plain, coast and above-the-relief pose read under 5°. A pre-check can only forgo a cull,
  never cause a wrong one.
- **View cone.** Only sectors within the frustum's azimuth span, widened by 30°, are
  stamped and finished; a candidate reaching into a dead sector is simply not culled.
- **Underground guard.** If the nearest ring's floor is above the camera in every live
  sector, the camera is enclosed by terrain (the sample under it can lag the mesh) and the
  march switches itself off rather than cull the whole globe.

`D3_DEBUG=1` prints the pre-check's reading and the floors.

## Terrain LOD: refining on shape

On the flat globe the only error is imagery resolution. With relief the drawn surface
also departs from the real one, so `apply_lod`'s threshold becomes

```text
subdivide_dist = max(imagery_dist, terrain_dist)
terrain_dist   = G · terrain_lod_factor
terrain_lod_factor = viewport_height_px / (max_geometric_error_px · 2·tan(fovy/2))
```

and whichever of picture and shape still wants resolution at this distance gets it. The
terrain factor shares only the projection with `lod_factor_for`: no texture size and no
calibration constant, because the shape of the ground must not depend on which imagery
style is loaded.

**`G` is measured, not estimated.** Cesium's heightmap error is `2πa / (65·2^z)`, a
function of the level alone, so a flat bay and the Karakoram refine at the same distance.
Here `HeightTile::detail` is the maximum over all texels of `|h − I(h)|`, where `I` is the
piecewise-bilinear interpolation of the field decimated 16:1 — the 17×17 lattice the mesh
actually draws. It is the drawn surface's error, measured against the finest data for the
tile, taken with `ceil` so it bounds rather than rounds.

**Below the source's deepest level.** A z15 tile's 256² samples are already drawn at 16:1,
so z16 draws its quarter at 8:1, z17 at 4:1, z18 at 2:1, and only at z19 does the lattice
land on every texel. `detail_below` stores the same measurement for every descendant one
to three levels down, each over its own window at its own lattice (4 + 16 + 64 entries);
four levels down it is exactly zero. A whole-tile maximum at the right lattice would
over-state by a median 1.25× at z16 and 2× at z17–z18. `detail_max_z` (19) bounds the
term and is the knob a device measurement could lower.

**Nodes without data** use Cesium's level rule, `616 538 m / 2^z`, until their tile lands;
the parent's measured error is not inherited, since it describes four times the ground
and would compound down an unloaded chain.

**The budget, `max_geometric_error_px = 12`.** Visible tiles summed over ten real-DEM
poses, and the p95 error left on screen at the Inn valley:

| budget | tiles | vs off | p95 error |
|--:|--:|--:|--:|
| off | 483 | — | 22.0 px |
| 24 px | 493 | +2 % | 18.6 px |
| 16 px | 532 | +10 % | 14.9 px |
| **12 px** | **707** | **+46 %** | **10.7 px** |
| 8 px | 1 210 | +150 % | 7.5 px |
| 4 px | 3 350 | +593 % | 6.4 px |

12 is the knee of the marginal column. Over the same range a flat-ground pose of the same
screen area moves by 0 to 3 tiles while the Alpine one doubles: the term spends tiles
where the ground has shape. The number is not Cesium's 2 and is not comparable to it:
Cesium budgets a level estimate and pairs it with a more eager imagery rule, and copying
its 2 costs 3 350 tiles here.

**Its unit is the device pixel.** The viewport height fed in is the swapchain's physical
height, where Cesium's `maximumScreenSpaceError` is in CSS pixels (and the imagery term
here divides by the scale factor). The value was chosen at 1280×720 in Free mode; on a
1080-high landscape screen it is 1.5× as strict, on a 2340-high portrait phone 3.25× in
Free and 2.4× in Cockpit (which has a wider field of view). It has not been re-chosen on
a device.

**Fog does not relax this term.** See [tiles-and-lod.md](tiles-and-lod.md#atmospheric-fog).

## Where the ground is

Three consumers need the ground, and they need different grounds.

- **`TileSystem::ground_height_at`** — collision and the camera's clearance. The deepest
  *resident* data under the point, bilinear, with runway corridors and exaggeration
  applied. It is view-independent on purpose: the points collision tests (under the
  camera, under the aircraft) are often off screen, where the only drawn tile is a culled
  z1–z5 whose grid spans hundreds of kilometres; at Frankfurt that read 215–325 m instead
  of 100 m, and changed as the view rotated. It never enqueues a fetch;
  `want_ground_at` registers points whose detailed tiles should be kept loaded.
- **`drawn_ground_height_at`** — things placed *on* the visible surface (labels). The
  triangle net of the tile covering the point *at the level it was drawn at last frame*
  (`DrawnMeshes`, fed from the renderable set, deepest level first), falling back to the
  bilinear field where nothing is drawn. One frame behind is the right phase: it is read at
  the top of the next `update_logic`, when those meshes are still on the card.
- **`GlobeExtension::sample_ground`** — the flight. The extension receives
  `ground_height_at`, or a constant 0 with terrain off, and fits the aircraft and the
  route line onto it near the airports (`crates/cesium-flight/src/terrain_fit.rs`).

`None` is a third state, not zero: with terrain off every query returns `None` and every
consumer runs the ellipsoid arithmetic it would run anyway, character for character.

**The camera.** `WgpuState::update_logic` samples the ground under the eye once per frame
before building the frustum; `Camera::altitude_agl` (altitude minus that ground) is what
the near plane is derived from, and `enforce_bounds_with` keeps the camera above the ground
at every position it tests. See [camera.md](camera.md).

## Runway corridors

The DEM around an airport is rarely a runway: embankments, buildings and the averaging of
a 30 m post spacing leave humps and steps along a 3 km strip. `RunwayCorridor`
(`terrain/corridor.rs`) flattens a strip onto the straight line between two threshold
elevations: exactly planar within the half-width, blended back into the DEM with a
smoothstep over a margin (35 m). The flight extension builds one per end of the flight
from the runway database (half-width = half the runway width + 5 m, at least 25 m;
known threshold elevations where the preset table has them, otherwise the field
elevation), and hands them over through `GlobeExtension::runway_corridors`. A change
clears the mesh cache so tiles are rebuilt flattened; the flattening is applied in
`HeightPatch::sample` and in every ground query, so the aircraft, the camera and the drawn
surface agree on it.

## Cost, measured

Over ten real-DEM poses at the shipped settings, terrain draws 702 tiles against 483 for
the flat globe, and at the heaviest pose holds 103 MiB of imagery, the height slice and
1.6 MB of vertex buffers. The flat path pays none of the terrain work: the bounds refresh,
the march, height requests and rebuild selection are all absent on the `Ellipsoid` arm.

## Also in the tree

`globe/terrain_parser.rs` parses Cesium's quantized-mesh format and is exercised by
`testing::terrain::test_terrain_parser`, but the engine does not use it: the source is
the Terrarium heightmap.

## Testing

`src/testing/terrain/` covers the decoder against committed fixtures
(`assets/terrain_fixtures/`: Everest, Zugspitze, the Monterey coast, the open Pacific and
the Dead Sea at z12, unmodified, so the pinned extrema are the decoder's baseline), the
cache and ancestor mapping, the height field and its normals and skirts, the height
bounds and inheritance margins against the corpus, the limb and occlusion stages against
the drawn mesh, the LOD term, mesh lifetime and rebuild budget, the ground reference
queries and the corridor. The capture harnesses in `src/testing/rendering/` (`terrain_*`)
render real DEM poses to PNGs and report tile counts and timings. See
[testing.md](testing.md).
