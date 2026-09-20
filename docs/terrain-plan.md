# Terrain plan — one engine, two surface modes

**Goal.** One engine that renders both the current flat globe and real terrain.

**The two constraints that shape every decision below:**

1. **Flat mode must not regress — at all.** Same visible set, same FP/FN, same update time,
   same memory layout. Not "close enough"; the repo already has the pins to prove it exactly,
   and they must stay green without being re-pinned.
2. **Terrain mode must cull properly**, including tiles hidden behind mountains — not just
   behind the Earth's limb.

Everything else is subordinate to those two.

---

## 1. Architecture: monomorphise, don't branch

A runtime `if terrain_enabled` in the per-node loop fails constraint 1 twice: it costs a
branch in the hottest code, and any field added for terrain grows the hot structs whether or
not terrain is on. `test_horizon_hot_structs_have_not_grown` pins
`size_of::<TilePatch>() == 64` (one cache line) and `HorizonCamera == 56`; `QuadtreeNode` is
192 B, exactly three lines.

**Solution: a `SurfaceModel` type parameter with zero-sized payloads in flat mode.**

```rust
pub trait SurfaceModel: Copy {
    type NodeExtra:  Default + Copy;   // () for flat
    type PatchExtra: Default + Copy;   // () for flat

    fn vertex_altitude(...) -> f64;
    fn vertex_normal(...) -> DVec3;
    fn obb_altitude_span(extra: &Self::NodeExtra) -> (f64, f64);
    fn is_occluded(patch: &TilePatch<Self>, cam: &HorizonCamera) -> bool;
    fn geometric_error(extra: &Self::NodeExtra, level: u8) -> f32;
}

pub struct Ellipsoid;   // NodeExtra = (), PatchExtra = ()
pub struct Heightfield; // NodeExtra = HeightBounds, PatchExtra = ScaledRadius
```

`()` is a ZST, so `QuadtreeNode<Ellipsoid>` stays **exactly 192 bytes** and
`TilePatch<Ellipsoid>` stays **exactly 64** — the existing pins prove it rather than
asserting it by hand.

**Verified before writing this, not assumed** (2026-09-20, `rustc -O`, on a standalone mirror
of `TilePatch`'s exact field layout — 8 × f64 plus the associated payload):

| | `Ellipsoid` | `Heightfield` |
|---|---:|---:|
| `TilePatch<S>` | **64 B** — the pinned value | 72 B |
| node-core mirror | 80 B | 88 B |

The zero-sized payload costs the flat path **exactly zero bytes**. This is the load-bearing
claim of the whole design; if it had failed, the plan would be a different one. Each method's `Ellipsoid` impl is today's code verbatim, inlined and
monomorphised, so the flat traversal compiles to what it compiles to today.

This is the same shape the repo already uses for `Stage::Fog` (additive, absent from the
harness pipeline) and `LodDistanceMode` (compiled in, not selected). Third time, consistent.

**The five dispatch sites**, and nothing else changes:

| site | `Ellipsoid` | `Heightfield` |
|---|---|---|
| `geometry.rs:159-163` altitude | `0.0` / `-skirt` | height sample |
| `geometry.rs:176-182` normal | ellipsoid gradient (analytic) | height gradient |
| `quadtree.rs:283-323` `fit_obb` | samples `alt = 0` | samples `alt ∈ [h_min, h_max]` |
| `horizon.rs:198` occlusion | exact rectangle supremum (Thm 3.5) | cone test (Thm 3.7) + §4 |
| `quadtree.rs:1025` `apply_lod` | imagery term only | `max(imagery, terrain)` |

**Why the horizon site must dispatch and cannot be unified.** Today's test is the exact
supremum of the linear functional `q·c` over the spherical rectangle — closed form, two
sinusoid maximisations, no `atan2`. The terrain test is a cone test on a bounding **sphere**.
Setting the sphere radius to zero does *not* recover the rectangle test: at zero relief a tile
patch still has `ρ > 0` because it is curved and extended. Unifying them would hand the flat
globe a conservative test in place of an exact one — a permanent FP tax on the mode that is
supposed to be unaffected. (An earlier draft of this document claimed `ρ = 0` recovers
today's test. It does not; it recovers the exact *point* test, Theorem 3.1.)

**Fallback if the generics get ugly.** If threading `<S>` through `quadtree.rs` turns into
turbofish soup, the escape hatch is two concrete instantiations behind a small enum at the
`QuadtreeManager` level, keeping the generic only on `QuadtreeNode`/`TilePatch`.
*(Not needed — Phase A carried the generic all the way to `QuadtreeManager<S>`.)*

### Two things Phase A found that this section had wrong

**A default type parameter does not apply in expression position.** `<S: SurfaceModel =
Ellipsoid>` makes `size_of::<TilePatch>()` and `-> QuadtreeNode` resolve as intended, but
`QuadtreeNode::new(id)` fails with **E0283**: an elided generic argument in an expression path
becomes an inference variable, never the default. The fix costs nothing and preserves every
call site — a concrete two-line wrapper in front of each generic constructor:

```rust
impl QuadtreeNode<Ellipsoid> { pub fn new(id: TileId) -> Self { Self::for_surface(id) } }
```

`TileMesh::generate` needs the same treatment for a different reason: a free generic
parameter cannot have a default at all, so `generate` wraps `generate_on::<Ellipsoid>`.

**Altitudes in the trait are megametres, and `lon_lat_alt_to_ecef_f64` is not.** That helper
(`geometry.rs:55-75`) takes **metres** and divides by 10⁶ internally, while `skirt_height` and
everything in `EARTH_RADIUS_*` are **megametres**. Reusing it inside `fit_obb` is the obvious
move and it is a 10⁶× unit trap — invisible in Phase A, where the altitude span is zero, and
detonating in Phase C/D as a mountain a million times too tall. The trait is megametres
throughout; `fit_obb` samples through a local `patch_point(lon, lat, alt_mm)` that returns
`surface_point` bit-for-bit when `alt_mm == 0.0`. **Keep every new altitude path in
megametres** and convert only at the ECEF boundary.

---

## 2. The data — what the source actually gives us

Probed live, 2026-09-20, `s3.amazonaws.com/elevation-tiles-prod/terrarium/{z}/{x}/{y}.png`
(AWS Open Data, no key):

- 256×256 RGBA PNG, `h = R·256 + G + B/256 − 32768` metres.
- **Deepest level is z15. z16 returns `404`.** Imagery refines to z19/20, so **upsampling from
  an ancestor is the normal path, not a fallback** — this drives the design of §4.2 and §5.
- Resolution at z15: **4.78 m/px at the equator, 3.07 m/px at 50° N**. The underlying DEM is
  SRTM-class (~30 m) almost everywhere, so there is no sub-5 m fidelity to chase.
- **Open ocean carries bathymetry** (Pacific tile: −4324 … −2277 m). Untreated, the sea sinks
  kilometres and the coast becomes a cliff. Inland water is encoded at surface level (Dead Sea
  tile: uniformly −412 m).
- Sanity: Everest tile z12 `max = 8740 m`, Zugspitze `2911 m`.

**Ocean decision: clamp `h < 0 → 0`.** Flattens the Dead Sea (−412 m) and Death Valley
(−86 m); correct everywhere a flight tracker looks. Keep `Raw` as a config variant so the
choice is visible and reversible.

---

## 3. Terrain culling — the "behind a mountain" problem

This is constraint 2, and it is the part with no prior art in this repo and none in Cesium
either (Cesium culls against the ellipsoid limb, the frustum and fog — never against terrain
itself). Three layers, all needed, in increasing order of novelty.

### 3.1 Height-aware bounding volumes (necessary, not sufficient)

`fit_obb` samples `alt = 0` today, so its `up` extent is the curvature sagitta. Against 9 km
of relief that is noise above ~z8. The terrain impl samples at `h_min` and `h_max`. Mechanical.

### 3.2 Horizon with relief (necessary for correctness)

A summit can satisfy `q·c ≤ 1` and still be visible over the limb. Today's test would discard
it — a false negative, and by I-7 an FN at any node kills the entire subtree, i.e. a hole in
the globe. The replacement is already derived and proved in `culling-math.md` §3.7
(Theorem 3.7, cone test on the scaled-space bounding sphere). Fit the sphere **in scaled space
at construction** — `T` is linear, so it is free there, and it avoids the wrong `ρ_real / b`
conversion.

`culling-math.md` §4 also needs its reasoning updated: it deleted back-face culling on the
strength of "below the horizon ⟺ back-facing", which holds only at zero relief. The deletion
stays correct; the stated reason does not.

### 3.3 Terrain occlusion — the actual answer to "tiles behind mountains"

**The idea.** March the view cone from the eye to the candidate tile. At each step, ask the
elevation pyramid how high the terrain is *guaranteed* to be across the cone's footprint. If
at any step that guaranteed ridge rises above the line of sight to the tile's highest possible
point, every point of the tile is behind it.

**Soundness is a matter of picking the right bound on each side, and it is easy to get
backwards:**

- The **occluder** must be a **lower** bound on the terrain — use `h_min` over the footprint.
  Something that is definitely there can definitely block.
- The **occludee** must be an **upper** bound on the tile — use `h_max`. If even its highest
  point is hidden, all of it is.

Using `h_max` for the occluder is the intuitive choice and it is wrong: it over-occludes and
produces exactly the FN this engine exists to prevent.

**Why lateral gaps are handled.** Taking the `h_min` over the *whole footprint of the cone* at
each step means a notch or valley in the ridge drives the minimum down, and no cull happens.
The test only fires when the ridge is continuous across the entire cone — which is precisely
when the tile is genuinely hidden.

**The data it needs is already required.** §4.3 builds a min/max elevation pyramid per height
tile for the bounds work anyway. A 16×16 min/max mip is ~1 kB per tile; the march is 16–32
lookups into a small array.

**Two properties worth stating up front:**

- **Unlike fog, this stage is sound.** It can live in the terrain-mode `CullPipeline::DEFAULT`
  and be verified by the harness at FN = 0, rather than being fenced out of it.
- **It pays off unevenly, and that is fine.** At 10–12 km cruise you are above almost
  everything and it will cull little. In valleys, on approach, and in cockpit view it is the
  difference between drawing a mountain range and drawing everything behind it too. Gate it on
  camera altitude so it costs nothing where it earns nothing.

---

## 4. Phase A — dual-mode skeleton, zero behaviour change

The whole point of doing this first is that constraint 1 is proved *before* any terrain code
exists, so later phases cannot quietly erode it.

- **A1.** Introduce `SurfaceModel`, `Ellipsoid`, and the associated types. Thread `<S>` through
  `QuadtreeNode`, `TilePatch`, `QuadtreeManager`, `fit_obb`, `SubGrid`, `apply_lod`,
  `TileMesh::generate`.
- **A2.** Move today's code into the `Ellipsoid` impls verbatim. No logic edits in this phase —
  if a line changes meaning, it belongs in a later phase.
- **A3.** `TerrainConfig { enabled: false, .. }` in `globe/tiles/config.rs`, wired to the
  viewer builder, CLI, debug panel and JNI the way the map-style switch already is.

**Acceptance — all of these, unchanged and un-re-pinned:**

```
cargo test --release --lib culling:: -- --test-threads=1 --nocapture   # 32 passed, 0 failed
```
- `size_of::<QuadtreeNode<Ellipsoid>>() == 192`, `TilePatch<Ellipsoid> == 64`,
  `HorizonCamera == 56`
- `test_visible_set_digest_is_stable` passes against its **existing** constants
- `bench_update`: mean ≈ 6.9 µs, 1 919 B/node, within run-to-run variance
- LOD harness CSVs byte-identical, `aggregate_ratio = 1.663`
- `culling_visual`'s 8 poses byte-identical PNGs at 1920×1080

If any of these moves in Phase A, the design is wrong and it gets fixed here, not later.

---

## 5. Phase B — height data

- **B1. Source.** Terrarium fetch + PNG decode →
  `HeightTile { data: Box<[i16; 65536]>, h_min: i16, h_max: i16 }`. `i16` metres is finer than
  the source and half of `f32`. Compute the extrema over **all** 65 536 texels, not over the
  grid the mesh will sample. Reuse `TileFetcher` (URL template, `BinaryHeap`, `Semaphore(8)`)
  and `TileCacheManager<T>` — both are already generic enough.
  Height requests for visible tiles go in at `High`: a missing texture is a blur, a missing
  height tile is the wrong shape.
- **B2. Height field.** `height_at(tile, u, v)` with bilinear sampling and ancestor fallback.
  `compute_fallback_uv` (`tiles/system.rs:112-135`) already walks the parent chain
  accumulating `scale *= 0.5` and a quadrant offset — the same walk against the height cache
  *is* Cesium's `upsample()`, done as a UV transform instead of a resample, which is cheaper
  and needs no extra memory. Returns "unknown" distinctly from "sea level" so C1 never bakes
  an unknown into a mesh it then forgets to rebuild.
- **B3. Elevation pyramid.** Per height tile, a 16×16 min/max mip (~1 kB). Feeds §3.1's bounds
  and §3.3's occlusion march.
- **B4. Budget.** 128 kB per height tile; 256 resident = 32 MB. The texture budget is a byte
  budget (`config.rs:45-59`), so the height cache takes a declared slice of it rather than
  silently doubling the total.

**Acceptance.** Decoder unit tests pin the *full-tile* extrema of committed fixtures.
`offline_mode` yields a flat zero field so every existing headless test runs with terrain on
and no network. Flat mode still passes Phase A's acceptance list.

### Three things Phase B found

**`offline_mode` cannot be shared with imagery.** `TileFetcher`'s offline stub returns an
all-255 RGBA image — white, which is a sensible fake basemap tile. Under the terrarium
encoding those same bytes decode to `255·256 + 255 + 255/256 − 32768 = +32 768 m`: a
worldwide 32 km wall, in exactly the mode meant to make headless tests run without a network.
Handled in `HeightTileManager::request_tile` (synchronous `HeightTile::flat_zero()`) rather
than by touching the shared fetcher.

**The height cache is 132 096 B per tile, not 128 kB** — 65 536 × i16 plus the 16×16 min/max
mip. At a 32 MiB slice that is 254 resident tiles, not 256.

**Decoding runs on the caller thread** (`HeightTileManager::update`), because `TileFetcher` is
shared verbatim and hands back RGBA. 65 536 texels of integer arithmetic plus the mip, a few
times per frame. Acceptable now; it is the first place to look if Phase F sees hitching during
a descent, when many height tiles land at once.

---

## 6. Phase C — terrain geometry

- **C1. Relief.** `Heightfield::vertex_altitude` returns the sampled height; it flows into the
  existing f64 `lon_lat_alt_to_ecef_f64` call, so I-2 (f64 world positions, downcast only at
  the end) is preserved for free. Skirt vertices become `edge_height − skirt_depth`.
  Apply vertical exaggeration **here and nowhere else**, before bounds are computed, so §3.1's
  boxes and §3.2's spheres stay consistent with it automatically.
- **C2. Normals.** Central differences on the height field in the tile's local east/north
  frame. Without this, relief shows in silhouette and vanishes in shading — every slope lights
  as if it were flat sphere. The lighting work on `main` consumes `out.normal` directly and
  needs no change, but it will *look* different; re-check `docs/lighting.md`'s tuning once
  terrain is on.
- **C3. Skirt depth, derived.** Today's `0.5 / 2^z` Mm was chosen against zero relief. With
  relief the crack at an LOD boundary is the disagreement between a tile's own edge heights
  and its coarser neighbour's interpolation of the same edge — **computable exactly at mesh
  build time**, because both fields are in hand. Cesium's `min(4·levelError, 1000 m)` is a
  content-blind estimate; computing the real mismatch costs one pass over four edges.
- **C4. Grid density.** `mesh_segments = 16` reads 17×17 samples out of a 256×256 tile — at
  z ≤ 15 the **grid**, not the source, is the limit, and distant mountains at z8–12 (most of a
  cruise camera's screen area) suffer most. The `u16` index buffer caps at 126.

  | `mesh_segments` | verts | vbuf | 512-tile cache |
  |--:|--:|--:|--:|
  | 16 (today) | 361 | 11.3 kB | 5.8 MB |
  | 32 | 1 225 | 38.3 kB | 19.6 MB |
  | 64 (Cesium-equivalent) | 4 489 | 140.3 kB | 71.8 MB |

  Measure at 16/32/64 and pick; starting hypothesis is 32. Note that point-sampling 17×17 out
  of 256×256 **misses summits**, so peaks grow as tiles refine — measure the popping before
  deciding whether max-filtered downsampling is worth making the mesh non-interpolating.

**New invariant, replacing I-1 (zero relief):**

> **I-1′.** Every vertex of a tile's mesh lies within that node's declared `[h_min, h_max]`,
> and the node's OBB is fitted over that interval.

I-1′ is what makes §3.1's boxes and §3.2's spheres sound, and it is checkable in exactly the
place I-1 was: `test_generated_mesh_has_no_positive_altitude` becomes
`test_generated_mesh_stays_within_declared_height_bounds`.
`test_generated_mesh_stays_inside_the_culling_rectangle` must still pass **unchanged** —
relief is radial and must not move any vertex in lon/lat.

---

## 7. Phase D — terrain culling

- **D1.** `Heightfield::obb_altitude_span` → `fit_obb` samples at `h_min`/`h_max`; `SubGrid`
  inherits through the same function.
- **D2.** Theorem 3.7 behind `SurfaceModel::is_occluded`, sphere fitted in scaled space.
- **D3.** The occlusion march of §3.3, as a new `Stage`, gated on camera altitude.

**The one genuine soundness trap, and it is in D1.** A node is culled long before its height
tile arrives, and **a parent's `[h_min, h_max]` is not a superset of its children's** — a
coarse tile smooths a peak away that a deeper tile resolves. Inheriting the parent interval
unmodified is an FN source.

| policy | sound? | cost |
|---|---|---|
| global `[−500, +9000] m` while unloaded | yes | a 9.5 km box over a 40 m tile at deep zoom |
| parent interval, unmodified | **no** | free and wrong |
| parent interval + measured per-level margin | yes | one measurement |

Take the third: measure the distribution of `child_max − parent_interpolated_max` per level
over a fixture corpus, pick a margin with headroom, record the table. The structure helps —
data stops at z15 and ancestors are already prefetched at `Low` priority
(`tiles/system.rs:98-106`), so by the time the camera is deep enough for a loose bound to
hurt, real data is there. Confirm that rather than believing it.

**Acceptance.** FN = 0 across the terrain sweep at the culling gate's sampling density — for
D1, D2 and D3 independently and together. FP recorded per stage so D3's benefit is visible
against its cost. Flat mode: Phase A's list, still unchanged.

---

## 8. Phase E — LOD and integration

- **E1. A real geometric error.** `apply_lod` keeps its shape and its 20 % hysteresis; the
  threshold becomes `max(imagery_dist, terrain_dist)`. `terrain_dist` comes from the tile's
  **measured** deviation from its parent's interpolation, computed once at decode — better
  than Cesium's heightmap path, which is level-based and content-blind, and free because the
  data is already in hand. **Clamp the terrain term at z15**: past the data ceiling the mesh
  is an interpolation and its true error stops falling, so a level-based term would buy
  nothing. Imagery still drives refinement to z19/20 — the picture keeps sharpening, the shape
  does not. `FogConfig::sse` (`fog.rs:93-101`), ported and deliberately unconsumed, finally has
  units; whether it beats WP5's shipped `subdivide_dist *= (1 − fog)` is a measurement.
- **E2. Mesh lifetime.** The mesh stops being a pure function of `TileId` — it depends on which
  height tile was available when it was built, normally an ancestor. The cache key must carry
  the height source, and `update_logic`'s `missing_meshes` pass must also collect *stale*
  meshes. Rate-limit rebuilds: a camera descending five levels over an alpine city can
  invalidate every visible mesh in one frame. `display_state`'s no-downgrade / sibling-gate /
  200 ms grace rules are the precedent — geometry needs the same discipline and is less
  forgiving, because a texture swap is a blur and a mesh swap is the ground moving.
- **E3. The rest of the engine.** In order of visibility:
  1. `FlightPlanConfig.terrain_elevation` is implemented, tested (Bogotá 2548 m, Mexico City
     1640 m) and held back by one comment — *"stays off until the globe can render terrain"*.
     Flipping it is the single most visible deliverable here.
  2. `Camera::altitude()` measures height above the *ellipsoid* and feeds `znear`/`zfar`, fog
     density and the label zoom bucket. Near a massif, `znear` is chosen for an aircraft that
     thinks it is kilometres higher — **near terrain clips**.
  3. Ground collision: `enforce_bounds` keeps the camera 2 m off the ellipsoid and will fly it
     through the Alps.
  4. Labels sit exactly on the ellipsoid; Denver's sinks 1.6 km.
  5. Picking/pan uses a closed-form ellipsoid intersection. Lowest priority — sub-pixel error
     except in mountains at low altitude, and the closed form is a fine first guess to refine.

---

## 9. Phase F — turn it on

`mesh_segments` and the D1 margin fixed at measured values. S23 soak with terrain on vs off
(`tools/phone_soak.sh`, cockpit, alpine airport and cruise), memory split recorded as imagery
vs height vs vertex bytes. `TerrainConfig.enabled` defaults to `true` in its own commit.

---

## 10. Sequencing

```
A ── B ──┬── C ──┬── D1 ── D2 ── D3 ──┬── E1 ── E2 ── F
         │       │                    │
         └───────┴────────────────────┴── E3
```

- **A gates everything.** It is where constraint 1 is proved.
- **C alone, with terrain on, is unsound** — relief with flat-mode culling is the FN the whole
  culling effort exists to prevent. C behind `enabled: false` is fine.
- **E3.1 can jump the queue** as soon as C looks right; it is one line and the most visible
  thing here.
- **D3 is the constraint-2 deliverable.** Do not let it slide to the end.

---

## 11. Open decisions

1. **Vertical exaggeration** — user knob or fixed at 1.0? Nearly free in C1, not free to
   retrofit after D1.
2. **Android in the same release?** Phase F may answer "desktop at `mesh_segments = 32`, S23 at
   16" — supported, but it means two tuning targets.
3. **Terrain shadows / AO** — out. `render_scene` is a single pass; that is a renderer change,
   not a terrain change.
