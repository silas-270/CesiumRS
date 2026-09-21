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
  *(Done — `CullPipeline::TERRAIN_DEFAULT`. And while writing it down: `Stage::Fog` is not
  merely unsound, it is **unreachable** where it currently sits. See §7b.)*
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

*(Landed. `TerrainConfig.enabled` is still `false` by default — §10: C alone, with terrain
on, is unsound. See "What Phase C found" below for the four things this section had wrong or
under-specified, and the C4 table.)*

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

**New invariant, alongside I-1 (zero relief):**

> **I-1′.** Every vertex of a tile's mesh lies within that node's declared `[h_min, h_max]`,
> and the node's OBB is fitted over that interval.

I-1′ is what makes §3.1's boxes and §3.2's spheres sound.
`test_generated_mesh_stays_inside_the_culling_rectangle` must still pass **unchanged** —
relief is radial and must not move any vertex in lon/lat.

### What Phase C found

**I-1′ does not replace I-1, it joins it.** An earlier draft of this section had
`test_generated_mesh_has_no_positive_altitude` *become*
`test_generated_mesh_stays_within_declared_height_bounds`. That is wrong twice over. The
first test says something **stronger** about `Ellipsoid` — `max = 0`, not merely "inside
its own claim" — and that stronger statement is what licenses the exact rectangle horizon
test the flat globe still runs. It is also a member of the gate, whose contract is to
report *32 passed, 0 failed, 1 ignored* unchanged across every phase; replacing or adding
to it changes the number being held fixed. So it stays exactly as it is, and I-1′ is
checked **additionally**, for both models, in `testing::terrain::test_heightfield`.

**Relief must displace along the ellipsoid normal, and shading must not.** Phase A had one
`vertex_normal` doing both jobs, because at zero relief they are the same vector. They are
not the same vector once C2 tilts the normal with the slope, and displacing along the
tilted one moves the vertex sideways out of its own culling rectangle — an I-5 violation,
i.e. a false negative at every tile edge. `VertexSample` now carries `up` (the analytic
gradient, computed once per vertex exactly as before) for displacement, and
`SurfaceModel::vertex_normal` is shading only.

**The mesh must not be built from whichever ancestor arrived first.** `TileSystem::update`
prefetches the whole height ancestor chain at `Low`, so a coarse ancestor routinely lands
before the tile's own height tile. Building from it bakes a smoothed, hundreds-of-metres-too-low
surface into a cache that — until E2 — has no reason to rebuild it. Measured, not reasoned:
the first Phase C capture over the Alps at 4.5 km had its whole foreground flattened this
way. `HeightTileManager::status_of` therefore answers `Ready` only once
`source_tile_for(id)` has arrived *or failed*; while it is in flight the mesh is deferred
and the engine draws the parent's mesh, exactly as it already draws the parent's texture.
This is not E2 — it removes the common case that would need a rebuild, and `TileMesh`
carries `height_source` for the case that still does.

**C3, derived.** The skirt is `max over k ∈ {2, 4}` of (the deviation of the tile's own edge
from that edge coarsened by `k`) + (the sagitta `R·(1 − cos(k·δ/2))` of the chord a
neighbour draws across `k` grid steps). The second term is the reassuring one: on an
all-ocean tile at z2 it lands on the same order as the hand-chosen `0.5 / 2^z`, so that
constant was never arbitrary — it was a curvature estimate, and a good one. From ~z5 down
the two diverge fast, because the constant falls as `2^-z` while the real sagitta falls as
`4^-z`; at z15 the derived skirt is four orders of magnitude smaller. Terrain is what puts
the difference back, and only where there is terrain: on the Everest fixture at z12 the
derived skirt is **741 m** against the old formula's 122 m, i.e. the old value was *six
times too small* for real relief, while on the Monterey coast tile it is 25 m, five times
too large. `Ellipsoid::skirt_depth` keeps the old expression bit-for-bit.

**C2 edge treatment: a one-texel halo, not a one-sided difference.** `HeightPatch` samples
a `(segments+3)²` grid whose outer ring lies one grid step *outside* the tile, read from
the same source tile. Whenever the source is an ancestor — the normal case, and the only
case past z15 — that halo is the real neighbour's ground, so edge vertices get a genuine
central difference and two adjacent tiles agree on their shared edge to within 0.016°.
Where the halo would fall outside the source (the tile *is* the source and sits on its
border) the patch records it per side and C2 drops to a **one-sided** difference with the
one-sided denominator. Reading the clamped value over a two-step baseline instead — the
obvious bug — would report half the true slope along that one vertex ring.

**The unsoundness §10 predicts is visible, and it is not at the limb.** With terrain on and
the quadtree still on `Ellipsoid`, the headless capture over the Alps at 4.5 km loses its
**entire near field**: the visible set is byte-identically the same 36 tiles as with terrain
off (the quadtree is untouched, as intended), but those tiles' geometry is lifted up to
2 900 m, so the near edge of the coverage rises in screen space and exposes bare background
underneath it. The tiles that *should* fill it are frustum-culled, because `fit_obb` samples
`alt = 0` and their raised geometry never enters the box being tested. This is **D1**, not
D2 — the false negative is a bounding-volume miss in the frustum stage, and it is far larger
and far more visible at low altitude than anything happening at the limb. At 400 km over the
Himalaya the limb itself looks clean; the same near-field loss appears at the bottom of that
frame too. Both go away when §7 D1 makes the box span `[h_min, h_max]`. Nothing in Phase C
touches culling to paper over it.

### C4 — grid density, measured

Max and RMS deviation of the drawn mesh from **all 65 536** source samples of each committed
fixture, in metres, with the buffer cost and the C3 skirt each density produces
(`testing::terrain::test_heightfield::c4_grid_density_error_against_the_fixtures`):

| fixture (z12) | `mesh_segments` | max err (m) | RMS err (m) | C3 skirt (m) | verts | vbuf (B) | ibuf (B) |
|---|--:|--:|--:|--:|--:|--:|--:|
| Everest | 16 | 658.5 | 75.1 | 740.6 | 361 | 11 552 | 3 888 |
| Everest | 32 | 503.8 | 33.6 | 500.5 | 1 225 | 39 200 | 13 872 |
| Everest | 64 | 274.5 | 14.9 | 207.8 | 4 489 | 143 648 | 52 272 |
| Zugspitze | 16 | 288.2 | 35.2 | 635.1 | 361 | 11 552 | 3 888 |
| Zugspitze | 32 | 193.3 | 17.3 | 152.3 | 1 225 | 39 200 | 13 872 |
| Zugspitze | 64 | 137.5 | 8.5 | 101.8 | 4 489 | 143 648 | 52 272 |
| Monterey coast | 16 | 29.3 | 3.4 | 24.8 | 361 | 11 552 | 3 888 |
| Monterey coast | 32 | 24.5 | 2.0 | 14.8 | 1 225 | 39 200 | 13 872 |
| Monterey coast | 64 | 25.4 | 1.1 | 19.0 | 4 489 | 143 648 | 52 272 |

**The default stays 16.** Phase F picks against device measurements, not against this table.
What the table says:

- **RMS falls cleanly and roughly halves per doubling** — 75 → 34 → 15 m on Everest, 35 → 17
  → 8.5 on Zugspitze, 3.4 → 2.0 → 1.1 on the coast. That is first-order convergence, which is
  what a linear interpolant over a halved spacing should give, so the mesh really is
  converging to the field.
- **Max error does not fall monotonically** — Monterey goes 29.3 → 24.5 → **25.4**. This is
  §6 C4's "point-sampling 17×17 out of 256×256 misses summits", confirmed: refining moves the
  sample points rather than averaging over them, so a peak one grid straddles the next can
  straddle almost as badly. Any decision framed on max error will be noisy; frame it on RMS.
- **Cost is the plan's table, confirmed**: 3.4× the vertex bytes per doubling. The 64 column
  is 143 kB per tile, 71.8 MB over a 512-tile cache, which is the figure §6 already quotes.
- At z12 an Everest tile is ~9.8 km wide, so `mesh_segments = 16` is a 612 m post. The RMS
  error at that density (75 m) is **larger than the ~30 m SRTM posting of the source itself**
  — the grid, not the source, is the limit, exactly as §6 C4 predicted, and it stays the limit
  through z15.

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

*(D1 and D2 landed 2026-09-20; **D3 landed 2026-09-20** together with D1's named
bounding-box follow-up. Everything below is what doing them produced or corrected.)*

### How the bounds reach a node

The quadtree has no access to the height cache and Phase D did not give it one. Three
pieces, each as narrow as it goes:

1. **`HeightTileManager::height_bounds_for(id, segments, exaggeration)`** — the only
   question the quadtree ever asks the cache. Answers `None` unless `status_of(id)` is
   `Ready`, which is deliberately the *same* predicate the mesh builder uses: the two
   disagree exactly while a coarse ancestor has landed and the tile's own height tile is
   still in flight, and bounds from the ancestor paired with geometry from the tile is
   precisely how a summit ends up outside its own box.
2. **`NodeExtraSource`**, a trait on the *outside* — `globe::terrain` implements it,
   `globe::quadtree` only calls it. Not a field on `CullContext`: that struct is `Copy`,
   is read per node per frame, and giving it a lifetime parameter for the benefit of one
   surface model is the wrong trade. Nothing implements it for `Ellipsoid`.
3. **`QuadtreeManager::refresh_extras`**, one top-down walk per frame before `update`.
   Not an argument threaded through `update → apply_lod → subdivide`, because that puts an
   extra parameter on the hottest recursion in the culler for a mode that is usually off.

A node therefore has *no* data of its own at birth — by construction, since the quadtree
decides to look at a tile before anything is fetched for it. It inherits
(`SurfaceModel::child_extra`, below), and the next frame's refresh tightens it. One frame
of looseness, which costs false positives and never a false negative.

`QuadtreeNode::set_extra` is what a tightening costs: refit the box and the `k × k`
sub-grid, once, the frame the tile's heights land. It early-outs on an unchanged payload,
and for `Ellipsoid` that comparison is `() == ()`, folds to a constant, and the body is
dead code the flat path never reaches.

### Which quadtree runs — one match per frame, at the manager boundary

`TerrainConfig::enabled` is a run-time flag and the surface model is a type, so something
has to bridge them. §1's "fallback if the generics get ugly" is taken here **deliberately
and not as a fallback**: `globe::quadtree::any::AnyQuadtree` is an enum of two whole
`QuadtreeManager`s, so the `match` runs five times a frame at the boundary, against tens of
thousands of nodes visited inside. The flat arm is a fully monomorphised
`QuadtreeManager<Ellipsoid>` — byte-for-byte the engine that shipped — and pays nothing
for the terrain arm's existence but binary size. The rejected alternative is the one §1
rejects: a `terrain_enabled` field read in the per-node loop, which costs a branch in the
hottest code *and* puts every terrain field on `QuadtreeNode` whether or not terrain is on.

### The margin, measured

`assets/terrain_fixtures/pyramid_extrema.csv` — 788 tiles, the z1…z15 chain over 16
regions with all four children at each step, 720 parent/child pairs, full-tile extrema
only (4 bytes a tile; the PNGs would have been 4.8 MB for the same numbers). The
requirement is on the *box spans* `height_bounds_for` produces, not the raw extrema, so it
covers the skirt allowance's level dependence too, and it is evaluated under **both** ocean
policies because the constant cannot know which is configured.

| child z | pairs | max needed (m) | p99 (m) | median (m) | margin (m) | headroom |
|--:|--:|--:|--:|--:|--:|--:|
| 2  | 32  | 698       | 383   | −339   | 20 000 | 28.7× |
| 3  | 72  | **4 566** | 1 668 | −903   | 20 000 | 4.4× |
| 4  | 88  | 492       | 384   | −1 052 | 20 000 | 40.7× |
| 5  | 112 | 461       | 183   | −721   | 8 000  | 17.4× |
| 6  | 120 | 1 061     | 889   | −676   | 8 000  | 7.5× |
| 7  | 120 | 1 292     | 787   | −288   | 8 000  | 6.2× |
| 8  | 128 | 1 365     | 221   | −246   | 8 000  | 5.9× |
| 9  | 128 | 166       | 155   | −304   | 6 000  | 36.1× |
| 10 | 128 | 833       | 637   | −38    | 6 000  | 7.2× |
| 11 | 128 | 1 359     | 50    | −137   | 6 000  | 4.4× |
| 12 | 128 | 36        | 25    | −113   | 1 500  | 42.1× |
| 13 | 128 | 11        | 9     | −76    | 750    | 68.2× |
| 14 | 128 | 3         | 2     | −134   | 400    | 133× |
| 15 | 128 | 3         | 3     | −29    | 200    | 66.7× |

**The z3 row is the whole argument in one number.** The z3 tile over eastern Greenland
reports a 7 796 m maximum where its z2 parent reports 3 230 m: one z2 texel is ~150 km
across and averages that spike out of existence, the z3 tile at ~75 km resolves it. A child
interval inherited unmodified would have been **4.6 km too shallow** there, and by I-7 that
is the whole subtree.

**Every median is negative.** In the typical case the parent's interval already contains
the child's and no margin is needed at all — it is the tail this table is sized for. 720
pairs bound a tail only so far, hence 4× headroom at the tightest level rather than a
fitted curve. Measured at `exaggeration = 1.0`; the height-dependent part of the
requirement scales linearly with it, so that headroom covers exaggeration up to ~4.

**What the plan asked for and what was measured instead.** §7 above says
`child_max − parent_interpolated_max`. The quantity the implemented policy actually needs
is `child_max − parent_max` with `parent_max` the parent node's *declared* interval, which
is the parent tile's whole-tile extremum, not an interpolation of it over the child's
quadrant. The tile-wide figure is the larger of the two (it covers four times the ground),
so this is the more conservative requirement, and it is the one the inheritance chain
composes over: with `extra(parent) ⊇ Ready(parent)` as the induction hypothesis,
`Ready(child) ⊆ widen(Ready(parent), M) ⊆ widen(extra(parent), M)` closes it, with the
global `[−11 500, +9 500] m` root interval as the base case.

**The margin accumulates down an unloaded chain, and that is the cold-start cost.** A z15
node under a Ready z11 ancestor inherits `1 500 + 750 + 400 + 200 = 2 850 m` of slack —
fine. A z15 node with *nothing* loaded above it inherits the sum of the whole column,
≈ 113 km, over a tile 1.2 km wide. That is sound and it is transient: `TileSystem::update`
requests the entire height ancestor chain, so the chain fills from the top within a frame
or two and each arrival restarts the accumulation from there. It is deliberately **not**
clamped to the global interval: `hi` is a real height with `TerrainConfig::exaggeration`
already multiplied in, `child_extra` is a static dispatch with no access to that factor,
and a clamp that is right at exaggeration 1.0 and wrong at 2.0 is a worse trade than a
loose box for two frames.

**The mesh can be one frame ahead of the bounds, and the margin is what covers it.**
`update_logic` refreshes the intervals, then culls, then streams — and it is the streaming
step that drains height fetches *and* builds meshes. So a height tile that lands in frame
N produces a mesh in frame N and a tightened interval in frame N+1, and for that one frame
the node was culled against its **inherited** interval while holding a **data**-derived
mesh. That pairing is exactly `Ready(child) ⊆ widen(Ready(parent), M)`, which is the
property the table above measures — it is not a separate hazard, but it is the reason the
table is measured on the full box spans rather than on the raw extrema.

**Below z15 the margin is exactly zero, and that is exact rather than optimistic.** Past
the source's deepest level a child reads the *same* height tile as its parent over a dyadic
sub-rectangle of the parent's, so its covering mip cells are a subset of the parent's and
its extrema are contained by construction. Checked on the real data path over 87 380
child/parent pairs, worst excess 0.0 m. The structural argument §7 hoped for ("data stops
at z15 and ancestors are prefetched at `Low`") turned out to be unnecessary: it is not that
the margin is *probably* small there, it is that no margin is needed at all.

### The skirt is in the box, and it is the expensive part

A node's interval is not the height field's range. C3 made the skirt content-dependent, and
a skirt vertex outside the box is a drawable point outside the box — the same false
negative as a summit outside it. So `lo = h_min − skirt_allowance`, where
`skirt_allowance = (h_max − h_min) + R·(1 − cos(2δ))` bounds C3's `max_k(edge mismatch +
sagitta)` from the two things the quadtree knows: an edge's deviation from its own
coarsening cannot exceed the tile's height range, and the sagitta is maximised at `k = 4`
with `δ` the larger of the tile's longitude and latitude grid steps (latitude matters — a
Mercator tile at low zoom is far taller than it is wide).

**Measured price: the node interval is 1.53× the mesh interval it must contain.** The
range term is the loose half — on the Everest z12 fixture C3's real skirt is 741 m against
a 4 700 m range. Bounding the mismatch by the range over the *boundary* mip cells, or over
sliding quarter-tile windows along them, would tighten it; that is a follow-up, recorded
here with its number rather than left as a surprise.

### False positives — what the bigger boxes cost

`testing::terrain::test_terrain_visibility::terrain_sweep_has_no_false_negatives`, 96 poses
over four regions × six altitudes (2 km … 2 000 km) × four pitches, both models on the same
poses with the same criterion:

| | visible leaves | FP leaves | FP rate |
|---|--:|--:|--:|
| `Ellipsoid` | 1 590 | 3 | **0.19 %** |
| `Heightfield` | 2 627 | 273 | **10.39 %** |

and the terrain model keeps **65.2 % more leaves**. Both numbers need their caveats stated:

- **This is not `culling::sweep`'s FP definition** and the two are not comparable. That one
  samples a tile's ground area against the ellipsoid oracle (2.05 % aggregate); this one
  asks whether any of a tile's own mesh vertices is unambiguously visible. The flat column
  is measured the same way in the same run precisely so the comparison has a scale.
- **Part of the +65 % is not FP at all.** Relief genuinely lifts tiles into view over the
  limb and over intervening ground; those tiles are correctly kept.
- **The sweep's field is Everest-grade everywhere**, with the relief/tile-width relation
  fitted to the real fixtures (`relief_range_m`) but applied to every tile on the globe. A
  real camera spends most of its time over terrain far gentler than that.

On real data the cost is much smaller, and the captures measure it: over the Alps at 4.5 km
the visible set goes **36 → 46 tiles** (+28 %), over the Zugspitze at 9 km 39 → 45 (+15 %),
over Everest at 11 km 33 → 42 (+27 %), at the 400 km limb 11 → 12 (+9 %). D3 is what buys
some of this back, and the number to beat is in this table.


---

## 7b. D3 — the occlusion march, as built

`globe/quadtree/terrain_occlusion.rs`, `Stage::TerrainOcclusion`,
`CullPipeline::TERRAIN_DEFAULT`. **FN = 0** over 22 poses and 1 409 culled nodes
(`testing::terrain::test_terrain_occlusion`), against an oracle that adds the third term
the D1/D2 sweep deliberately lacks.

### The march is amortised over the frame, not re-run per candidate

§3.3 describes marching the cone from the eye to each candidate. The answer does not
depend on the candidate — only on bearing and range — so the march runs **once per
frame** into a polar grid of 24 azimuth sectors × 48 range rings around the camera, and a
candidate costs a handful of lookups into it. That is §3.3's "16–32 lookups into a small
array" with the pyramid walk paid once instead of once per tile.

Each cell holds a **lower bound on the terrain** over its wedge-annulus footprint, and
`finish` converts it to the elevation angle of a wall standing at the cell's **far** edge,
accumulated as a running maximum over the nearer rings. A candidate is culled when an
**upper** bound on the elevation angle of its whole box is below the **minimum** of that
wall over every sector the candidate spans. Every bound is the one that makes the claim
weaker; the module doc has the table and the soundness proof.

### The occluders are the quadtree's own node floors

Not a second query path into the height cache. D1 already maintains a sound lower bound on
the ground per node, so the march walks the tree exactly as `refresh_extras` does and
stamps each node's floor into the cells it covers. The walk is a **partition** — it stops
at nodes it does not descend into, culled ones included — so every cell in range is
covered and its floor is the minimum over everything stamped into it. Building a second
path would have meant measuring a second margin, in the other direction, for a quantity
D1 already bounds.

### Four things this got wrong first, all of them measurable and none of them obvious

Written down because each one was *invisible in the tile counts* until the specific probe
that exposed it, and three of the four still culled plenty while being wrong.

1. **A bounding *sphere* for the occludee is useless.** A tile's box is a flat slab
   tangent to the globe; its circumsphere claims the tile could be overhead. On the
   valley pose a z12 tile 20 km out has a 4.7 km circumradius, so the sphere bound puts
   its highest point at **+11.8°** where the box bound puts it at **−1.1°**, against a
   ridge at +11.3°. Fixed by bounding `vert_max` and `horiz` separately off the box, with
   the horizontal extreme branched on the sign of the vertical one.
2. **A whole-tile minimum is not a ridge.** The LOD sizes a node for *imagery*: at the
   stand-off where D3 matters the crest lands inside a z12 tile 6.6 km across, whose
   minimum is the ground on the far side — 1 132 m against a 3 400 m crest. The ridge
   disappeared from the occluder and the stage culled only against the curvature horizon.
   **The probe that caught it: flatten the test world's ridge and the tile counts do not
   move.** Fixed by `HeightBounds::floor_grid`, a 4 × 4 minimum per node.
3. **The same failure radially.** A cell's floor is a minimum over its whole radial
   extent, so a ring deeper than the ridge is wide averages the crest with the valley in
   front of it. 16 rings (7.2 km deep at 22 km) and 24 rings (5.1 km) both lost the crest;
   48 rings (2.4 km) keep it.
4. **A stamp thinner than a ring may not claim that ring's far edge.** The wall stands at
   the far edge, so a 200 m-deep stamp writing into a 1 km-deep ring claims ground a
   kilometre beyond what it bounds. 122 false negatives, the moment the stamps got tight
   enough for it to matter.

And one plain arithmetic slip worth recording because it was the hardest to see: inverting
the ellipsoid normal to the camera's own latitude needs **one** power of the flattening
(`tan φ = (b/a)·n_y/‖n_h‖`), not two. Squaring both radii moves the camera 0.096° —
**10.6 km** on the ground — and every bearing and range in the file with it.

### Measured reduction

`testing::terrain::test_terrain_occlusion`, synthetic ridge world: a continuous
east–west crest 2.8 km above a 600 m plateau, σ ≈ 5 km, with one col through it. All poses
look due north into the crest. The last column is the **control**: the same pose over the
same world with the ridge flattened, i.e. what the stage removes against the *curvature*
horizon alone.

| pose | camera alt | D1+D2 | with D3 | delta | flat-world control |
|---|--:|--:|--:|--:|--:|
| valley (11 km out) | 1 200 m | 64 | 44 | **−31.2 %** | −20.3 % |
| cockpit (18 km out) | 2 000 m | 62 | 56 | −9.7 % | −9.7 % |
| approach (31 km out) | 3 000 m | 58 | 56 | −3.4 % | −3.4 % |
| cruise (66 km out) | 11 000 m | 45 | 45 | 0.0 % | 0.0 % |

The valley row is the deliverable: **11 points of the 31 are the mountain**, and the rest
is the terrain-aware curvature horizon that comes with it. The cruise row is the
counter-check §3.3 asks for, and it reads zero.

### The altitude gate, measured

`d3_altitude_gate_is_where_the_benefit_stops`, same stand-off geometry walked up in
altitude with the gate held open, at two crest heights:

| camera alt | 2.8 km crest | 8.0 km crest |
|--:|--:|--:|
| 800 m | −27.9 % | −27.9 % |
| 1 500 m | −16.7 % | −16.7 % |
| 3 000 m | −3.7 % | −3.7 % |
| 5 000 m | −4.3 % | −4.3 % |
| 8 000 m | −4.3 % | −4.3 % |
| 12 000 m | 0.0 % | 0.0 % |
| 20 000 m | 0.0 % | 0.0 % |
| 40 000 m | 0.0 % | 0.0 % |

Three regimes: a large benefit below ~2 km, a −4 % plateau to 8 km, and exactly zero from
12 km up. **`max_camera_altitude_m = 12 000`** — the first altitude that measures zero, not
the knee, because shutting the stage off at the knee would give up a real 4.3 % at 5 and
8 km to save a march that costs tens of microseconds once a frame. The two crest columns
agree, which is itself a finding: past ~3 km the reduction *saturates* — the shadow
lengthens but there are no tiles left in it — so the curve to read a threshold off is the
altitude one, not the relief one.

### Cost

`bench_terrain_occlusion_cost`, on a machine under load 125–169 (the same conditions that
made `bench_update` read 10–25 µs for a 6.9 µs mean, so treat these as an upper bound with
roughly 2–3× of inflation in them):

| pose | march, once/frame | `update` D1+D2 | `update` +D3 |
|---|--:|--:|--:|
| valley | 499 µs | 47 µs | 106 µs |
| approach | 479 µs | 38 µs | 81 µs |
| cockpit | 416 µs | 27 µs | 67 µs |
| cruise 11 km | 293 µs | 17 µs | 37 µs |

The march is the expensive half and it is the obvious next optimisation: it is 16 stamps
per in-range node, each costing an `acos`, an `atan2`, a `sin` and an `asin`, plus
24 × 48 elevation angles in `finish`. A node whose sub-cells are already finer than the
ring they sit on could stamp once instead of sixteen times, and `finish` could hoist the
per-ring geometry out of the sector loop. Neither was needed to make D3 correct and both
are measurements away.

*(Both measured in §7c. The second is worth 33 % and is done. **The first does not exist**:
of the 58–96 nodes stamped per frame, zero have a footprint inside a single cell, because
`classify` descends only until the *sub-cells* beat the ring. What was actually costing the
walk was 40 `web_mercator_y_to_lat_f64` calls per node for five distinct values, 8 of them
dead. These figures are also an upper bound with 2–3× of machine load in them; §7c's
before/after pairs were taken interleaved on the same loaded machine, and read 267 µs for
the valley pose rather than 499.)*

**Flat mode pays none of it**: `refresh_terrain_horizon` is never called on the
`Ellipsoid` arm, `CullContext::terrain` is `None` there, and `Stage::TerrainOcclusion` is
not in `CullPipeline::DEFAULT`.

### `MAX_STAGES` went 4 → 5

`CullPipeline::TERRAIN_DEFAULT_WITH_FOG` needs five. The flat consequences, measured:
`size_of::<QuadtreeNode<Ellipsoid>>()` is 192 B and `bench_update` reports 1 919 B/node
and a mean inside run-to-run variance (13.2 / 20.4 / 22.3 µs against a 20.4 µs baseline on
the same loaded machine). `CullPipeline::keeps` unrolls one more dead slot that `DEFAULT`
breaks out of before reaching, so the cost is code size. `CullPipeline` itself grows 4 B →
6 B and is not stored per node.

### Where the stage sits, and a structural fact worth knowing

`Stage::TerrainOcclusion` is **second**, right after `Stage::Horizon`. That is not a
preference: `Stage::NodeFrustum` answers `Keep` outright for every node without a
sub-grid and `Stage::SubPatchGrid` answers `Keep` or `Cull` for every node with one, so a
stage appended *after* those never runs. `CullPipeline::DEFAULT`'s own doc comment already
records the fact ("the final rule is never reached under `DEFAULT`") without drawing the
consequence.

**The same fact means `Stage::Fog` is unreachable in `DEFAULT_WITH_FOG`.** It is the
fourth stage of four, behind exactly those two, so `CullPipeline::keeps` returns before it
runs — for every node, at every camera. Whatever WP5's fog culling contributes today, it
comes from `apply_lod`'s `subdivide_dist` relaxation and not from the stage. **Not touched
here**: moving it would change production behaviour, and this package's contract is that
the flat path does not move. It belongs in its own commit with its own measurement.

### D1's follow-up: the loose half of the box, tightened

§7 recorded the node interval at **1.53×** the mesh interval with a named cause — the
skirt allowance bounded C3's crack by the *whole tile's* height range. C3's crack is an
**edge** property, over aligned quarter-windows of each edge, so
`HeightTile::edge_window_range` bounds it from the edges instead, with a two-texel halo in
place of `mip_extrema_over`'s deliberately generous whole-cell one.

| fixture (z12) | C3's real skirt | old bound | new bound | old inflation | new inflation |
|---|--:|--:|--:|--:|--:|
| Everest | 741 m | 3 617 m | **2 874 m** | 1.69× | **1.52×** |
| Zugspitze | 635 m | 2 099 m | **1 519 m** | 1.57× | **1.36×** |
| Monterey coast | 25 m | 146 m | **86 m** | 2.03× | **1.62×** |
| Pacific | 102 m | 2 048 m | **1 299 m** | 1.91× | **1.56×** |

On the terrain sweep's synthetic field the false-positive rate moves 10.39 % → **10.29 %**
(273/2 627 → 270/2 624), and that small number is the fixture's doing rather than the
change's: the sweep's field is incommensurable noise at every scale, which makes an edge's
range equal to the tile's and is precisely the field on which this change cannot show. The
committed PNGs are what say otherwise. **What is still loose** is now the mip's own
granularity — a quarter-edge window is 4 of 16 mip cells plus a halo, so on Everest the
bound is 2 874 m against a real 741 m. Reading the edge's texels directly would close most
of the rest and costs ~1 000 reads per node per refresh; that is the next follow-up, with
its number, rather than a surprise.

**It is not free, and the price is in the inheritance.** Tightening `lo` raises the
child's end *and* the parent's, and the corpus measured the margin table against
whole-tile spans. `Heightfield::child_extra` now hands the parent's whole-tile allowance
back — as a **level-constant** (`inherit_allowance_mm`), never as `parent.hi − parent.lo`,
which would compound down an unloaded chain and have the box larger than the solar system
by z15. The composition then closes against the numbers already in the table, with no
re-measurement the committed corpus (whole-tile extrema only) could not support. Two
consequences worth stating:

- `below_the_source_ceiling_…` no longer asserts raw containment. Past z15 a child's
  *edges* are interior lines of its parent, so its allowance can exceed its parent's —
  measured at up to 470 m on the real data path. The test now checks the inheritance
  **policy**, which is what I-7 actually depends on.
- A cold z1→z15 chain accumulates a bounded ~1 300 km of downward slack instead of the
  ~113 km §7 records. Sound, transient, and numerically ordinary; below z15 it never
  happens at all, because a z16–z20 node is `Ready` the moment its z15 source is.

### `HeightBounds` grew a third number, and a fourth

`floor` (D3's occluder, the ground minimum *without* the skirt allowance) and `floor_grid`
(the same per 4 × 4 sub-cell). `QuadtreeNode<Heightfield>` is 304 B; `QuadtreeNode<Ellipsoid>`
is **192 B**, unchanged and un-re-pinned.

**`floor`'s soundness is not covered by the corpus and needs saying.** The corpus measures
`parent.lo − child.lo`, and turning that into the bound `floor` needs —
`parent.h_min − child.h_min ≤ M` — costs the parent's whole-tile allowance back. That is
the same term the edge tightening owes, which is why one constant pays for both. Had the
margin been inherited on `floor` unmodified, it would have been the quiet kind of wrong:
sound almost everywhere, and an over-high occluder exactly where a coarse DEM had smoothed
a notch away.

### What the captures show

`rendering::terrain_capture`, 1280×720, regenerated after D3. Each pose is now rendered
**three** times — terrain off, terrain on with D1+D2, terrain on with D3 — because a tile
count cannot see a hole and three shots can.

| pose | camera alt | off | D1+D2 | +D3 | D1+D2 vs +D3 |
|---|--:|--:|--:|--:|---|
| **`alps_inn_valley`** (new) | 900 m | 34 | 49 | **48** | **pixel-identical** |
| `alps_zugspitze` | 9 km | 39 | 44 | 44 | pixel-identical |
| `alps_low` | 4.5 km | 36 | 46 | 46 | pixel-identical |
| `himalaya_everest` | 11 km | 33 | 42 | 42 | pixel-identical |
| `himalaya_limb_400km` | 400 km | 11 | 12 | 12 | pixel-identical (gate shut) |

"Pixel-identical" is a sampled RGBA comparison of the two PNGs, every second pixel in
each axis, and it comes back at **zero differing samples** on all five. At the valley pose
that is the statement worth having: D3 removed a tile and the picture did not move.

- **`alps_inn_valley`** — Innsbruck on the valley floor with the Nordkette wall rising
  behind it, which is exactly the regime §3.3 describes. Gapless from the foreground city
  to the crest line, no skirts, no break.
- The four Phase C/D1 poses are unchanged and still gapless: `alps_low`'s relief runs
  continuously from the ridge to the bottom edge with a lake basin at the left,
  `alps_zugspitze` shows ridges out to the Bavarian foreland, `himalaya_everest` has the
  massif over the Tibetan plateau, and the 400 km limb is clean.

### D3 removes almost nothing on real terrain at these poses, and that is worth stating

One tile at the valley pose, none at the other four, against **−31 %** in the synthetic
ridge world. The two numbers are both right and the gap is the finding:

- **Real relief does not stop at the ridge.** The synthetic world puts a crest over a flat
  plateau, so whole tiles behind it sit below the ridge line. In the Alps the ground behind
  a ridge is more ridges, and a tile's box is fitted over its own `h_max` — so its top
  pokes over the crest and the cull, which needs the *whole* box hidden, does not fire.
- **The far field is coarse.** At the stand-off where a ridge shadows anything, the LOD has
  already dropped to z10–z11, i.e. tiles 19–39 km across. A box that wide spans enough
  ground to contain something visible almost anywhere.

So the honest statement of the benefit is: D3 pays where the ground behind a ridge is
genuinely lower *and* flatter — a valley floor, a plain behind a range, water, a plateau —
and it is close to free elsewhere because the altitude gate shuts it off above 12 km. The
obvious next step, and it is E-shaped rather than D-shaped, is to test **per sub-patch**
the way `Stage::SubPatchGrid` does for the frustum and the limb: `SubGrid` already carries
a `k × k` decomposition, and a coarse tile half-hidden behind a ridge is exactly the case
it exists for. That is a change to the sub-patch stage's contract, it moves the flat
path's stage list, and it wants its own measurement — so it is recorded here rather than
smuggled into D3.

*(Built and measured in §7c, and **rejected**: two to four more tiles out of forty to
sixty-seven, for six to ten times the cost of the whole D1+D2 pass, and **nothing at all**
at the five capture poses. It needed neither a contract change nor a stage-list change in
the end — it fitted inside `Stage::TerrainOcclusion` — but it did not earn its place, and
the code is gone. §7c also corrects the two numbers this section reasons from: the far
field is coarse because of **fog**, not the imagery LOD.)*

**Does the sphere cull noticeably looser than the rectangle?** At these poses, no — not
measurably. The limb stage is not what is keeping the extra tiles: at 4.5 km almost nothing
in frame is anywhere near the limb, and the 400 km pose, which is the one that is, gains a
single tile. The sphere's conservatism is real and is largest at coarse zoom (a z1 tile's
bounding sphere is enormous next to its rectangle), but coarse tiles near the limb are also
the ones the frustum has already settled. `Ellipsoid::is_occluded` keeps the exact
rectangle supremum regardless — §1's rule, unchanged.

### Two things D1/D2 found

**`unstretched_radius` is deliberately left on the zero-altitude span.** Relief does grow a
tile's true extent, and feeding that into `unstretched_radius` grows `subdivide_dist` with
it, refining terrain mode deeper than flat mode at the same camera distance. That is a
real and probably desirable effect — and it is an **LOD** change, it is `apply_lod`'s
dispatch site in §1's table, and that site is E1. Smuggling it in with D1 would have made
the tile-count deltas in the table above unreadable. `fit_obb_flat` is the one-line
function that keeps it honest.

**`SubGrid`'s per-sub-patch payload rides inside the existing `obbs` vector.** D2 needs a
bounding sphere per sub-patch, and a second `Vec<S::PatchExtra>` would have cost the flat
path 24 B per gridded node for a vector that can never hold anything — visible in
`bench_update`'s bytes-per-node, which §4's acceptance list quotes. `SubPatch<S> { obb,
extra }` is layout-identical to a bare `OrientedBoundingBox` for `Ellipsoid`, so the size
and the heap accounting do not move at all.

**Sizes, for the record:** `QuadtreeNode<Ellipsoid>` 192 B and `TilePatch<Ellipsoid>` 64 B,
unchanged and un-re-pinned; `QuadtreeNode<Heightfield>` 240 B, `TilePatch<Heightfield>`
96 B. That asymmetry is the zero-sized payload doing exactly the job §1 designed it for,
and `testing::terrain::test_terrain_visibility::the_zero_sized_payload_still_costs_the_flat_node_nothing`
asserts it where it cannot change the gate's count.

---

## 7c. The sub-patch follow-up, measured and rejected — and the march made cheap

§7b closed with a proposal and a complaint. The complaint: D3 removes **one tile** at
`alps_inn_valley` and **none** at the other four capture poses, for a march costing
293–499 µs a frame. The proposal: test **per sub-patch** rather than per node box, because
`k²` small boxes follow a ridge line far more closely than one box fitted over the node's
own `h_max`.

Both were followed up. The proposal was built, measured at its ceiling, and **thrown
away**; the march is now 27–43 % cheaper. This section is the measurement, including the
part of it that says §7b was reading its own numbers through a confound.

### First: §7b's captures were measuring a globe with the far field already gone

Before any of this could be tuned it needed a harness that could be run in a loop.
`rendering::terrain_capture` measures the real-terrain reduction but only alongside a GPU
render, so `testing::terrain::test_terrain_occlusion::d3_on_real_terrain` was built to
count the same tree instead of drawing it: the real DEM off `terrarium` (cached on disk),
the capture's own poses, and the capture's own LOD threshold.

It did not agree — 100 visible tiles at `alps_inn_valley` against the capture's 49, and a
D3 reduction of −36 % against the capture's −2 %. The difference is **WP5's fog
relaxation**. `apply_lod` multiplies `subdivide_dist` by `1 − fog(distance)`, and at 900 m,
where fog is thickest, that stops the far field refining past z10/z11 *in the first place*.
The ridge-world sweep runs at `fog_density = 0` and therefore hands D3 a far field
production never draws.

So §7b's second explanation — "the far field is coarse, z10–z11" — is right, and the reason
it is coarse is not the LOD's imagery budget. It is fog. Set `fog_density` from
`fog_density_for(alt)` as `wgpu_state` does and the harness lands on the capture to a tile
(50 → 48 here, 49 → 48 there). Every number below is measured with it set.

| pose | camera alt | D1+D2 | with D3 | delta |
|---|--:|--:|--:|--:|
| `alps_inn_valley` | 900 m | 50 | 48 | −4.0 % |
| `alps_low` | 4.5 km | 46 | 46 | 0.0 % |
| `alps_zugspitze` | 9 km | 45 | 45 | 0.0 % |
| `himalaya_everest` | 11 km | 42 | 42 | 0.0 % |
| `himalaya_limb_400km` | 400 km | 12 | 12 | 0.0 % (gate shut) |
| `po_plain_to_alps` | 300 m | 64 | 63 | −1.6 % |
| `terai_to_himalaya` | 600 m | 67 | 67 | 0.0 % |
| `rhone_valley` | 900 m | 49 | 48 | −2.0 % |
| `salzach_to_alps` | 800 m | 53 | 53 | 0.0 % |
| `aosta_valley` | 900 m | 61 | 59 | −3.3 % |

**The five extra poses are placed over ground whose height was looked up first**, and that
is not fussiness. The first attempt at this list picked coordinates off a map, and three of
four put the camera *inside a mountain* — 11.75 E / 47.28 N "in the Inn valley" is 1 998 m
of Tuxer Alpen, 11.00 E / 45.60 N "the Po plain" is 499 m of Lessini hills, 86.83 E /
27.80 N "the Khumbu valley" is 5 440 m. `TerrainHorizon::finish`'s enclosure guard
correctly switched the whole march off at all three. Four poses reading a flat zero for a
reason that has nothing to do with the thing under test is how a negative result gets faked
by accident.

### The sub-patch test: built, measured at its ceiling, removed

`Stage::TerrainOcclusion` was given the node's `k × k` `SubGrid` — the same decomposition
`Stage::SubPatchGrid` has used for the frustum and the limb since the culling rework — and
culled a node only when **every** cell was individually proved hidden. Sound for the same
reason `has_surviving_sub_patch` is: the cells' union is the whole drawn patch, so it is
D3's own proof one level finer, and `d3_never_hides_a_visible_vertex` stayed at FN = 0
throughout.

It works. It is just not worth it.

| pose | D1+D2 | node | sub-patch, gated | sub-patch, ungated |
|---|--:|--:|--:|--:|
| `alps_inn_valley` | 50 | 48 (−4.0 %) | 48 (−4.0 %) | 46 (−8.0 %) |
| `alps_low` | 46 | 46 (0.0 %) | 46 (0.0 %) | 44 (−4.3 %) |
| `alps_zugspitze` | 45 | 45 (0.0 %) | 45 (0.0 %) | 44 (−2.2 %) |
| `himalaya_everest` | 42 | 42 (0.0 %) | 42 (0.0 %) | 41 (−2.4 %) |
| `himalaya_limb_400km` | 12 | 12 (0.0 %) | 12 (0.0 %) | 12 (0.0 %) |
| `po_plain_to_alps` | 64 | 63 (−1.6 %) | 63 (−1.6 %) | 58 (−9.4 %) |
| `terai_to_himalaya` | 67 | 67 (0.0 %) | 66 (−1.5 %) | 63 (−6.0 %) |
| `rhone_valley` | 49 | 48 (−2.0 %) | 47 (−4.1 %) | 46 (−6.1 %) |
| `salzach_to_alps` | 53 | 53 (0.0 %) | 53 (0.0 %) | 50 (−5.7 %) |
| `aosta_valley` | 61 | 59 (−3.3 %) | 58 (−4.9 %) | 57 (−6.6 %) |

And the price, `update` per frame on the same trees:

| pose | D1+D2 | +D3 node | +D3 sub-patch, ungated |
|---|--:|--:|--:|
| `alps_inn_valley` | 29 µs | 74 µs | **464 µs** |
| `alps_low` | 22 µs | 47 µs | 398 µs |
| `alps_zugspitze` | 21 µs | 56 µs | 460 µs |
| `himalaya_everest` | 16 µs | 24 µs | 171 µs |
| `rhone_valley` | 59 µs | 88 µs | 564 µs |
| `aosta_valley` | 30 µs | 81 µs | 523 µs |

**Three tiles across ten poses for 190 µs, or eleven tiles for 400 µs.** Either way the
cull costs six to ten times what the entire D1+D2 pass costs, to remove two to four tiles
out of forty to sixty-seven. The five capture poses — the ones this package is accepted
against — move by **nothing at all** under the gated form. That is not a marginal call, so
`OccludeeGranularity` and `SubGrid::every_sub_patch_is_occluded` were deleted rather than
shipped behind a default-off knob: a measured negative result belongs in this document, and
complex code that buys three tiles does not belong in the engine.

Three things it is worth having found out, because each of them would have to be
rediscovered by anyone who reads §7b's proposal and tries it again:

1. **The gate is not free and the ungated form is not affordable.** The cheap filter that
   makes the loop tolerable — skip a node whose box top already clears the tallest ridge
   anywhere — is exactly the filter that throws the culls away, because the nodes whose
   boxes clear everything are the *coarse* ones, and coarse nodes are where the culls are.
   Benefit and cost live in the same place.
2. **The culls are at z1–z6, and they are not mountains.** The level histogram
   (`d3_sub_patch_granularity_probe`) puts every extra cull between z1 and z6 and none at
   z7 or below. A z10 far-field tile 19–39 km across is never wholly hidden however finely
   it is cut; what a finer occludee bound removes is a *coarse leaf* in the far field, and
   what hides it is the terrain-aware **curvature** horizon, not a ridge. That is the same
   thing §7b's flat-world control column was already saying about where the ridge world's
   −31 % comes from.
3. **`SUB_BOXES_PER_AXIS` only just happens to cover it.** The table is
   `16, 16, 12, 8, 6, 4, 3, 2, 1`, so a node has a sub-grid only to `z = 7`. Had the culls
   been one level deeper the proposal would have had nothing to work with at all, and §7b's
   "`SubGrid` already carries a `k × k` decomposition" would have been simply false.

The probe that measures the ceiling stays, and it needs no engine support:
`d3_sub_patch_granularity_probe` builds each sub-rectangle's box with
`QuadtreeNode::for_surface_with` on the descendant tile id at the parent's height interval
— which is what `SubGrid::build` does one abstraction down — and reports what a 2×2, 4×4
and 8×8 division would cull, against the ridge world as a positive control. The refutation
is therefore re-runnable without the code it refutes.

### The march, made cheap

Since the stage rarely pays, it had better be cheap. §7b named two levers. One of them does
not exist; the other is worth 33 %, and a third was sitting in plain sight.

**Lever 1 — "16 stamps per node instead of one" — does not exist.** The condition under
which one stamp is equivalent to sixteen is that the node's whole footprint lands inside a
single (sector, ring) cell. Instrumented over the ten real poses: of the **58–96 nodes
stamped per frame, zero** meet it, at any pose. That is not bad luck, it is what
`TerrainHorizon::classify` is for — it descends until a node's *sub-cells* are at most half
a ring wide, which leaves the node itself up to two rings wide and always spanning several
15° sectors. The lever was measured away rather than argued away.

**What was actually costing the walk was `web_mercator_y_to_lat_f64`.** `stamp_node` called
it **40 times per node** for five distinct values: 32 inside `sub_bounds`, which re-derives
both row boundaries for every one of the sixteen sub-cells, and **8 more that were dead** —
a `lat0`/`lat1` pair computed per row and then discarded through a `let _ = (lat0, lat1);`,
left behind when the loop stopped deriving its rectangle by hand and started calling
`sub_bounds`. It is an `atan` of a `sinh` and it was the largest single line in the march.
Hoisting the five boundaries to the top of the function and building each sub-rectangle
from them — the same expressions at the same `v`, so the rectangles are bit-identical —
takes the walk from **174 µs to 115 µs**.

**Lever 2 — hoist the ring geometry out of the sector loop — is real.** `elevation_of`
opened with `gamma.sin_cos()`, and `gamma` belongs to the *ring* while the bearing belongs
to the sector, so the sector-major loop computed the same 48 sine-cosine pairs 24 times
over. Turning the loops inside out (rings outer, sectors inner, the running maximum an
array of 24 instead of a scalar) makes it 48 calls instead of 1 152 and takes `finish` from
**52 µs to 35 µs**. Every finite cell is still evaluated: the tempting skip §7b's
`RANGE_RINGS` note warns about is still wrong for the reason recorded there.

Measured end to end, `refresh_terrain_horizon` once per frame, before and after,
interleaved over two rounds to cancel the machine's drift (load 90–130 throughout):

| pose | before | after | delta |
|---|--:|--:|--:|
| `alps_inn_valley` | 346 µs | 253 µs | **−27 %** |
| `alps_low` | 271 µs | 175 µs | −35 % |
| `alps_zugspitze` | 282 µs | 203 µs | −28 % |
| `himalaya_everest` | 236 µs | 159 µs | −33 % |
| `himalaya_limb_400km` | 0.8 µs | 0.9 µs | — (gate shut) |
| `po_plain_to_alps` | 308 µs | 201 µs | −35 % |
| `terai_to_himalaya` | 344 µs | 229 µs | −33 % |
| `rhone_valley` | 333 µs | 210 µs | −37 % |
| `salzach_to_alps` | 360 µs | 207 µs | **−43 %** |
| `aosta_valley` | 355 µs | 232 µs | −35 % |

…and on the ridge world, which is the table §7b quotes:

| pose | march before | march after | `update` D1+D2 | `update` +D3 |
|---|--:|--:|--:|--:|
| valley | 267 µs | **170 µs** | 16 µs | 41 µs |
| approach | 265 µs | 193 µs | 15 µs | 34 µs |
| cockpit | 308 µs | 192 µs | 17 µs | 43 µs |
| cruise 11 km | 219 µs | 167 µs | 14 µs | 28 µs |

`update` does not move, which is the expected result: the stage itself was not touched.
`TerrainHorizon::begin` was measured too, in case the two boxed 24 × 48 grids it allocates
and fills every frame were hiding something — **0.4 µs**, so they are not.

### What did not change, and what the acceptance says

The stage's contract, the pipeline, the stage list and every flat-path number are untouched;
`Stage::TerrainOcclusion` is byte-for-byte the test it was. `refresh_terrain_horizon` is
never called on the `Ellipsoid` arm, so neither optimisation is even reachable from flat
mode.

* `cargo test --release --lib culling::` — **32 passed, 0 failed, 1 ignored**, no re-pin of
  `size_of::<QuadtreeNode<Ellipsoid>>() == 192`, `TilePatch<Ellipsoid> == 64`,
  `HorizonCamera == 56` or `test_visible_set_digest_is_stable`.
* **FN = 0**, unchanged: `d3_never_hides_a_visible_vertex` (22 poses, 1 409 culled nodes)
  and `terrain_sweep_has_no_false_negatives`.
* The ridge world's reduction table is unchanged to the tenth of a percent — valley −31.2 %,
  cockpit −9.7 %, approach −3.4 %, cruise 0.0 % — which is the check that the march still
  computes the same grid.
* **The five captures are byte-identical**, all three shots each, to the ones §7b committed.
  Not "pixel-identical to within a sampled comparison": `cmp` on the PNGs. (The
  `terrain_off` shots do differ run to run, including between two runs of the *same*
  binary — that is the satellite imagery arriving differently, on a path that has no
  terrain in it at all.)

### Where this leaves D3

Sound, cheap, and honest about how little it does. It removes one to two tiles in an Alpine
valley, occasionally two or three, nothing at altitude, and nothing at all above 12 km
because the gate shuts it off. The march is 170–250 µs once a frame and the per-node test is
some 25–45 µs inside `update`. §7b's hope that a finer occludee bound would change that is
now a measurement rather than a hope, and the measurement says no — twice over, once for
what it buys and once for what it costs.

The honest open question is no longer granularity. It is **fog**: the relaxation in
`apply_lod` has already deleted most of the far field D3 was built to remove, which is why
the synthetic world reads −31 % and the real one reads −4 %. Whether that relaxation is the
right way to spend the far field is a WP5 question with its own trade — and, since
`Stage::Fog` itself is unreachable in `DEFAULT_WITH_FOG` (§7b), the two belong in the same
commit as each other rather than in this one.

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

### E1 — what landed

Four commits, because the four pieces answer to different evidence: E1a is a feature with a
cost table, E1b is a re-measurement of a shipped decision, E1c is a production behaviour
change, and E1d is an instrument that had gone stale.

#### E1a — a measured geometric error

`apply_lod` keeps its shape and its 20 % hysteresis band. The threshold becomes

```text
subdivide_dist = max(imagery_dist, terrain_dist)
imagery_dist   = unstretched_radius · lod_factor · fog_relaxation     (unchanged)
terrain_dist   = geometric_error · terrain_lod_factor
```

`terrain_lod_factor` is `terrain_lod_factor_for(max_geometric_error_px, viewport_height,
fovy)` = `H / (max_px · 2·tan(fovy/2))` — Cesium's `d < G·H/(maxSSE·2·tan(fovy/2))` with
`G` left outside it. It is deliberately **not** derived from `lod_factor`, which carries
`LOD_CALIBRATION_CONSTANT` (a residual fitted to reproduce a hard-coded `2.0`, not a
geometric ratio), the imagery tile size and `target_texel_ratio`. Deriving one from the
other would make the shape of the ground depend on which imagery style is loaded, which is
the coupling E1 exists to break.

**The error is measured at decode, and that is the whole point.** `HeightTile::detail` is
the maximum over all 65 536 texels of `|h − I(h)|`, where `I` is the bilinear interpolation
of the same field decimated 16:1. At `mesh_segments = 16` — the shipped value — a tile's
mesh lays 17 samples across 256 texels, so the decimation lattice *is* the mesh's own
vertex lattice and this is not a proxy for the drawn surface's error, it is that error. It
costs one extra pass over a grid the decode already walks and **two bytes per resident
tile** (`HEIGHT_TILE_BYTES` 132 096 → 132 098; the derived cache entry count is still 254).

Cesium's heightmap path is `2πa / (65 · 2^z)` — level-based and content-blind. What that
costs, measured on the real DEM at five regions (metres):

| level | Inn valley | Everest | Po plain | Bay of Bengal | Amazon | level-based |
|--:|--:|--:|--:|--:|--:|--:|
| z6 | 1 520 | 3 760 | 1 520 | 802 | 204 | 9 633 |
| z8 | 1 422 | 2 200 | 1 049 | **0** | 690 | 2 408 |
| z10 | 587 | 1 206 | 10 | **0** | 104 | 602 |
| z12 | 109 | 602 | 12 | **0** | 37 | 151 |
| z13 | 42 | 610 | 4 | **0** | 32 | 75 |

At z12 the Karakoram-class number is **50× the Bay of Bengal's** and the level-based
formula cannot tell them apart. The Bengal column is exactly zero from z7 down: that ground
is flat to the metre, its mesh draws it exactly, and a level-based term would have spent
refinement on it anyway.

**The level-based formula stays, as the fallback only** (`fallback_detail_mm`), for a node
the quadtree created this frame whose data is still in flight. It is replaced by the
measurement the frame the tile lands. It is Cesium's constant unmodified, and the table
above is also its error bars: 2.6× too generous at z6, 8× too small for Everest at z13.
Too generous is the dangerous direction — a fallback that demanded refinement would
compound down a cold chain — and it is generous only where tiles are few. The *shape* is
what makes it safe either way: it halves per level exactly as `unstretched_radius` does, so
the two terms keep a fixed ratio down a chain with no data and the tree cannot run away.

**The term stops at z15** (`DETAIL_MAX_Z`), and `Heightfield::geometric_error` is where the
clamp lives rather than `HeightBounds`, because it is a statement about this engine's LOD
rule and not about the data. Past the source ceiling a node's mesh is an interpolation of
its z15 ancestor's samples; refining it converges on the DEM's resolution, not the
ground's. Imagery is untouched and still drives to z19/z20 — the picture keeps sharpening,
the surface does not.

**The flat path cannot reach any of this.** `SurfaceModel::HAS_GEOMETRIC_ERROR` is an
associated **const**, `false` for `Ellipsoid`, and `apply_lod` reads it as
`if S::HAS_GEOMETRIC_ERROR`. `QuadtreeNode<Ellipsoid>` therefore emits the
`unstretched_radius · lod_factor · fog_relaxation` line and nothing else — not a `max`
against zero, not a multiply by one. That is stronger than an arithmetic no-op, and it is
why the 204-pose LOD harness produces byte-identical CSVs (below).

##### The cost, against the knob

Visible tiles summed over ten real-DEM poses, and the p95 projected geometric error left on
screen at `alps_inn_valley` (`testing::terrain::test_terrain_lod::e1_cost_of_the_geometric_term_on_real_terrain`, at
`TerrainFogPolicy::ImageryOnly`, which E1b below selects):

| `max_geometric_error_px` | tiles | vs off | imagery bytes | p95 error at `alps_inn_valley` |
|--:|--:|--:|--:|--:|
| off | 483 | — | 121 MiB | 22.0 px |
| 24 | 493 | +2 % | 123 MiB | 18.6 px |
| 16 | 532 | +10 % | 133 MiB | 14.9 px |
| **12** | **707** | **+46 %** | **177 MiB** | **10.7 px** |
| 10 | 867 | +80 % | 217 MiB | 8.9 px |
| 8 | 1 210 | +150 % | 303 MiB | 7.5 px |
| 4 | 3 350 | +593 % | 838 MiB | 6.4 px |

**12 px**, read off the marginal column and not the total: 16 → 12 buys 4.2 px for 175
tiles, 12 → 10 buys 1.8 px for 160, and 10 → 8 buys 1.4 px for 343. Phase F re-measures it
on device; desktop and an S23 may well want different values, the same split §9 already
anticipates for `mesh_segments`.

**Cesium's own default is 2 and copying it across would be a category error.** Cesium
budgets a level-based *estimate* and pairs it with an imagery rule far more eager than this
engine's; 2 px here measures 3 350 tiles where the engine draws 483.

The other half of that table is the half E1 exists for. At **every** budget above,
`po_plain_to_alps` — flat ground, same screen area, same camera — moves by 0 to 3 tiles
while `alps_inn_valley` doubles. Bucketed by each tile's own measured error
(`e1_the_extra_tiles_land_on_the_mountains`, the same pose at 300 m):

| | tiles | flat (<10 m) | rugged (>100 m) |
|---|--:|--:|--:|
| off (pre-E1) | 63 | 43 | 20 |
| 12 px | 66 | 43 | 23 |
| **delta** | **+3** | **0** | **+3** |

Every tile the term adds lands on the mountains. None lands on the plain.

##### The picture

`rendering::terrain_e1_capture`, three poses, each rendered twice with nothing different
but the knob. Differing pixels are a sampled RGBA comparison, every second pixel in each
axis, with the frame cut into six horizontal bands:

| pose | tiles off → on | pixels differing | where |
|---|--:|--:|---|
| `e1_alps_inn_valley` | 48 → 110 | 4.30 % | **22.7 % in band 2** — the Nordkette crest — and **0.00 % in the bottom half**, which is the valley floor and the city |
| `e1_po_plain_to_alps` | 40 → 46 | 2.52 % | **15.1 % in band 3** — the Prealpine skyline — and **0.00 % in the bottom half**, which is the Po plain |
| `e1_bengal_flat` | 64 → 67 | 0.35 % | the control: a flat delta at the same altitude and pitch, where the knob is very nearly free |

That band localisation is the proof, and it is stronger than the tile counts: the frames
differ *only* where the ground has shape. On the Karwendel the crest line gains resolved
summits and notches that the coarser mesh chorded away; the city below it, already fully
refined by imagery, is bit-identical.

**The Po pose was wrong the first time and the picture is what said so.** At its original
300 m the horizon is 62 km, the Venetian Prealps begin at 40 and the Dolomites at 90 — so
the shot contained a plain and a hill line and no Alps at all. `√(2Rh)` is checkable before
rendering; it was not checked until the image came back. It is at 4 km now.

#### E1b — fog was tuned on a globe with nothing to lose

§7c named this the honest open question and it turns out to have a measurable answer.
`apply_lod` multiplies `subdivide_dist` by `1 − fog(d)`. At 900 m, where fog is thickest,
that is what stops the far field refining past z10/z11 — §7c measured the same pose at 100
tiles and −36 % D3 reduction with fog off, and 50 tiles and −4 % with it on. It was tuned
for a globe with **no relief**, where coarsening the far field is free because there is
nothing out there but texture. With terrain it is distant mountains staying coarse bumps.

Three policies (`TerrainFogPolicy`), all implemented, all re-runnable:

| | geometric term multiplied by | at `fog → 1` |
|---|---|---|
| `Relax` | `1 − fog` — WP5's, extended to the new term. What E1a shipped. | **0** |
| `ImageryOnly` | `1` | 1 |
| `CesiumSse` | `1 / (1 + fog · sse/max_px)` — Cesium's own form | ≥ `1/(1+sse/max_px)` |

`CesiumSse` is what finally gives `FogConfig::sse` units. Cesium subtracts `fog(d) · sse`
from the screen-space error *in pixels* before comparing against the budget, and refining
while `error_px − fog·sse > max_px` is refining while
`dist < error · H / (2·tan(fovy/2) · (max_px + fog·sse))` — a division, not a
multiplication, and bounded below however thick the fog gets. WP5's version is a switch
where Cesium's is a nudge.

**Measured at an equal tile budget**, which is this repo's own methodology for exactly this
situation (WP4/C: "equal tile budget, not equal `target_texel_ratio`"). The budget is
bisected per policy so all three land on the same total, and the column that decides is the
p95 projected geometric error **in the far field**, past 20 km, because fog is negligible
nearer than that by construction:

| policy | budget | tiles | Σ p95 px | **Σ far-field p95 px** | far-field tiles |
|---|--:|--:|--:|--:|--:|
| `Relax` (WP5) | 8.01 px | 897 | 205.8 | **106.3** | 368 |
| **`ImageryOnly`** | 9.71 px | 896 | 191.1 | **84.3** | 442 |
| `CesiumSse` | 9.25 px | 887 | 203.7 | **91.6** | 419 |

**`ImageryOnly` ships.** For the same tiles it leaves **21 % less geometric error in the
far field** and moves 20 % more of the budget out there, which is precisely what §7c said
was being lost. The argument matches the number: fog's case for relaxing *imagery* is as
good as it ever was — a texture you cannot see through does not need to be sharp — and it
does not transfer to silhouette, because haze does not hide an outline. The imagery term
keeps WP5's `× (1 − fog)` unchanged and every number in `docs/culling-baseline.md` stands.

**`CesiumSse` is a negative result and worth stating as one.** It loses to doing nothing at
all, by 9 %. `FogConfig::sse` therefore stays unconsumed in production — now as a
measurement rather than as a pending question, five sections after WP5 reserved it "once
terrain gives this engine a real geometric error term".

#### E1c — `Stage::Fog` was unreachable, and deleting it was the honest answer

§7b recorded the fact and did not draw the consequence: `Stage::Fog` was the fourth of four
stages in `DEFAULT_WITH_FOG`, behind `Stage::NodeFrustum` (which answers `Keep` outright
for every node without a sub-grid) and `Stage::SubPatchGrid` (which answers `Keep` or
`Cull` for every node with one), so `CullPipeline::keeps` returned before it — for every
node, at every camera, in the pipeline production actually ran.

Moved into slot 1, where a `Cull`/`Undecided` stage does execute, over all 204 bench poses:

| | visible tiles | poses that moved |
|---|--:|--:|
| shipped (fog in the dead slot) | 3 579 | — |
| fog reachable | 3 579 | **0** |

Not "almost nothing". Zero, and the reason is structural rather than a property of these
poses. `cesium_fog` saturates to exactly `1.0` in f32 at `distance · density ≈ 4.16`. A
tile that survives `Stage::Horizon` has a point the limb test could not prove hidden, and
every such point lies within `√(2Rh + h²)` of the eye — so the stage can only fire when the
saturation distance is *inside* the horizon:

| camera altitude | fog = 1 at | horizon | ratio |
|--:|--:|--:|--:|
| 10 m | 30.3 km | 11.3 km | 2.68 |
| 100 m | 34.5 km | 35.7 km | **0.97** |
| 900 m | 126.3 km | 107.1 km | 1.18 |
| 11 km | 553.1 km | 374.5 km | 1.48 |
| 400 km | 4 609 km | 2 293 km | 2.01 |

Above 1 everywhere this engine flies, dipping below only in a narrow band near 100 m where
the whole visible set is a handful of tiles 35 km out. **Fog thick enough to cull is always
further away than the planet's own edge.**

**The margin is not always wide, and that is worth writing down.** At the tightest of
the 204 poses the fog on a *surviving* tile reaches `0.999999940` — one f32 ulp under
the threshold. That is the 0.97 row of the table above seen from the other side: near
100 m the saturation distance really is just inside the horizon, and a tile out there
really does come within a rounding step of being culled. It still culls nothing, in any
frame this engine has been measured in, and the probe asserts the zero rather than
reporting it — so an ulp's worth of drift in the fog constants shows up as a red test
naming this section instead of as a silent change of behaviour.

So it was deleted rather than promoted — a stage that removes nothing still costs an
`OrientedBoundingBox::distance_to_point` and an `exp` per node per frame. With it went
`CullPipeline::DEFAULT_WITH_FOG`, `TERRAIN_DEFAULT_WITH_FOG`, and the whole apparatus that
existed to keep an unsound pipeline away from the harness. **Production now runs
`CullPipeline::DEFAULT` / `TERRAIN_DEFAULT` — the same constants the culling gate proves
FN = 0 against**, which is a better arrangement than the paragraph that used to explain why
it was safe for them to differ.

`MAX_STAGES` goes 5 → 4 with it, undoing D3's raise. §7b recorded that raise as costing
code size and not frame time; the reverse reads the same way — `bench_update` 9.3 µs before
and 8.8 µs after, on a machine whose same-code spread is wider than that, with 1 919 B/node
and `size_of::<QuadtreeNode<Ellipsoid>>() == 192` untouched.

**What did not change is the part that always did the work.** `apply_lod`'s fog relaxation
is untouched, and WP5's shipped 95 % p95 reduction in cruise came from it and only ever
came from it. The proof is byte-level: re-running the entire WP5 suite across this deletion
returns **byte-identical CSVs**, fogged ones included.

The probe survives the code it refutes, as §7c's did. `testing::lod::test_wp5_fog::the_fog_stage_never_ran_and_this_is_what_it_would_have_culled`
settles the real tree under the real fog density and then applies the deleted stage's own
predicate — `cesium_fog(obb.distance_to_point(eye) · 1e6, density) >= 1.0` — to every
surviving tile, from outside the engine, and asserts the count is zero. The day the fog
constants move far enough to make that false, something says so.

#### E1d — the LOD harness's measuring stick had expired

`src/testing/lod/mod.rs` justified `texels / screen_px` as a complete description of LOD
quality *because* with zero relief the only per-tile error is imagery resolution. True when
written, and E1a expired it: a globe can be perfectly sharp and the wrong shape.

The harness now carries a second metric, `sweep::geometric_error_px` —
`error · H / (dist · 2·tan(fovy/2))`, Cesium's screen-space error evaluated on this
engine's measured one. It is the quantity `max_geometric_error_px` budgets, so every table
above reads directly against it.

**In the LOD harness itself the second metric is identically zero**, and that is the point
rather than a gap: everything there runs an `Ellipsoid` tree, whose `HAS_GEOMETRIC_ERROR`
is a compile-time `false`. What used to be a claim in a comment is now a value
`the_flat_globe_leaves_no_geometric_error_on_screen` reads and asserts over all 204 poses
— *exactly* zero, not merely small. Where the number is not zero is
`testing::terrain::test_terrain_lod`, which calls the same function on real DEM tiles.

Neither metric is written to the CSVs as a new column, deliberately: those files are the
byte-for-byte statement that the flat path has not moved, and an instrument that rewrites
its own output format cannot make that statement.

#### E1 — acceptance

* `cargo test --release --lib culling::` — **32 passed, 0 failed, 1 ignored**, no re-pin of
  `size_of::<QuadtreeNode<Ellipsoid>>() == 192`, `TilePatch<Ellipsoid> == 64`,
  `HorizonCamera == 56` or `test_visible_set_digest_is_stable`.
* **All 14 LOD-harness CSVs byte-identical** — `cmp`, not "equivalent" — across all four
  commits, `aggregate_ratio = 1.663` and 204 poses unchanged. That includes the four
  `wp5d_*` fogged CSVs across E1c's deletion of the fog stage, which is the measurement
  that says the stage never did anything.
* The five `rendering::terrain_capture` poses: the `terrain_off` shots still read 34 / 39 /
  36 / 33 / 11 visible tiles, exactly §7b's table. The terrain-on shots refine further and
  are gapless — `alps_low` 46 → 76, `alps_inn_valley` 49 → 110, `himalaya_everest` 42 → 75,
  the 400 km limb unchanged at 12 — and were looked at, not just counted.
* One pin moved, and it is an accounting one: `resident_bytes` 132 096 → 132 098, E1's two
  bytes per height tile. The derived cache entry count stays 254.

### E2 — what landed

**The mesh cache entry carries its height source; it is not part of the key.** Phase C put
`TileMesh::height_source` in place for this and it now has two readers: `TileBuffers`
copies it on the way into `LruCache<TileId, TileBuffers>`, and `update_logic`'s
`missing_meshes` pass reads it back out. A compound `(TileId, height_source)` key was the
obvious alternative and it is the wrong shape: two meshes of the same tile never need to
coexist — the later build replaces the earlier one — so a compound key buys nothing but a
second live entry per tile for the LRU to evict, while what E2 needs is to ask an *existing*
entry which data it was built from. That is a field.

**The staleness test is the mesh builder's own two predicates, not a third one.**
`tiles::system::fresher_height_source` runs `status_of` (may a build happen at all?) and
`resolve_source` (which tile would answer?) — exactly what `HeightPatch::sample` runs. A
different question here would let a rebuild be scheduled that then produced the identical
mesh, every frame, forever.

**No downgrade, and it is what makes the loop terminate.** A rebuild is offered only when
the available source is *strictly deeper* than the recorded one. The two ways it could be
shallower are an LRU eviction of the deep tile and a negative-cache entry expiring; in both
the mesh already on the card is the better of the two, and replacing it would be a hillside
visibly dropping. `display_state`'s rule 4 refuses the same thing for textures, and §8 is
right that geometry is less forgiving. The rule also bounds the whole process: every
accepted rebuild raises `height_source.z`, which stops at `max_level`. There is nothing that
can flip back, so E2 needs no analogue of `display_state`'s 200 ms grace — that timer exists
to absorb a set that oscillates, and this one cannot.

#### The case E2 is for does not occur in normal flight, and that is a Phase C result

§8 lists two triggers. Only one of them is a fetch failure, and the measurement says it is
rare to the point of absence.

`rendering::terrain_e2_capture::capture_an_approach_into_innsbruck` flies the real descent —
eight settled steps from 6 000 m AGL to 100 m over runway 26, the same valley
`terrain_e3_capture` lands in — and reads the engine's own rebuild counter at each:

| eye AGL | 6 000 | 3 000 | 1 500 | 800 | 450 | 250 | 150 | 100 |
|---|--:|--:|--:|--:|--:|--:|--:|--:|
| visible tiles | 49 | 65 | 71 | 76 | 83 | 86 | 86 | 89 |
| mesh rebuilds | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| still-frame delta | 0.000 % | 0.000 % | 0.000 % | 0.000 % | 0.000 % | 0.000 % | 0.000 % | 0.000 % |

**Zero rebuilds over the whole approach**, and the *still-frame delta* — hold the camera
exactly where it is, render one more frame, compare pixels — is exactly zero at every step.
That column is the section's question asked directly: a non-zero value there is literally
the ground moving while nobody moved the camera.

The reason is Phase C, not luck. `status_of` answers `Ready` only once
`source_tile_for(id)` has arrived *or failed*, so an ordinary mesh is built from the deepest
data the source will ever serve for that tile and **nothing can improve on it**. A camera
descending five levels over an alpine city does not invalidate the meshes it already has; it
creates new nodes that have none, which is the `missing_meshes` path and predates terrain.
So §8's worry — "a camera descending five levels can invalidate every visible mesh in one
frame" — describes a globe where Phase C had not happened.

What is left is genuinely narrow:

1. **A tile's own height fetch fails**, the mesh comes from an ancestor, and the retry —
   after `negative_cache_duration`, 10 s — succeeds. Real, because the negative cache does
   expire and `request_height_chain` does re-queue; but not observed once in any capture in
   this section, because the Terrarium source serves every level ≤ z15 worldwide and there
   is no systematic 404 to trip over. Transient network failures are the only source, and
   they are transient.
2. **Terrain switched on at runtime** (the debug panel, `ViewerCommand::TerrainSetEnabled`),
   after which every resident mesh is a flat one carrying `height_source: None`. Before E2
   that switch did nothing to geometry already on the card; now it does. The reverse
   direction deliberately rebuilds nothing — with no `HeightTileManager` there is no
   staleness test to run, which is the same `None` arm that keeps the flat path free, and
   the relief meshes simply age out of the LRU.

Building the machinery anyway is still the right call, but the reason is (2) and the
`insert_failed` path, not a descent.

#### The budget, and what actually binds it

`MESH_REBUILD_BUDGET_PER_FRAME = 4`, and the arithmetic is *not* what sets it.
`testing::terrain::test_mesh_lifetime::e2_what_one_rebuild_costs`, measured the way
`bench_update` measures — warm up, 200 iterations, mean per call, never a single-shot clock:

| `mesh_segments` | `HeightPatch::sample` | `generate_on::<Heightfield>` | 4 × sample | of a 16.6 ms frame |
|--:|--:|--:|--:|--:|
| **16** (shipped) | **7.8 µs** | 16.1 µs | **31.3 µs** | **0.19 %** |
| 32 | 31.0 µs | 51.5 µs | 123.9 µs | 0.75 % |
| 64 | 82.4 µs | 190.2 µs | 329.8 µs | 1.99 % |

Only the first column lands on the frame — the patch is sampled on the update thread so it
can cross into a rayon worker without a borrow (§6's `BuildCtx` split), and the second column
is off it. A budget of forty would still fit in a frame. What sets the number at four is the
*visual* argument §8 states: four tiles changing shape over successive frames reads as the
surface sharpening, forty in one frame reads as a jolt.

**A budget of slots, not of offers.** The first version of this counted a tile against the
budget every frame it was stale — and a tile stays stale in the cache until its replacement
lands, several frames later. Measured on the staged burst below: **90 "rebuilds" to finish 74
tiles**, the effective budget halved, and the nearest tiles head-of-line blocking the ones
behind them. `select_mesh_rebuilds` now drops anything `MeshWorkerPool::is_requested` already
has, and the same burst takes **45**.

**Nearest to the camera first.** When a burst does land, a stale mesh 200 km out is a few
pixels of silhouette and the one under the aircraft is the ground it is about to touch. Ties
break on `(z, x, y)`, so a headless capture is reproducible frame for frame.

#### The burst, forced and photographed

A mechanism nobody has seen work is a mechanism nobody should trust, and the approach above
never triggers it. `capture_a_staged_rebuild_burst` stages the worst case the engine can
produce, at 900 m over Innsbruck: every height tile from z11 down marked failed *before the
first mesh is built* (a mesh built from good data is never downgraded — that is the rule, not
an accident), so all 74 visible meshes come from a **z10** ancestor; then all 120 real height
tiles injected in one frame through `insert_ready`, which is the same call the fetcher makes
when a retry lands, with the latency taken out.

| | |
|---|--:|
| rebuilds | 45, over 34 frames |
| worst single frame | **4**, the budget |
| total change, coarse → rebuilt | 58.5 % of the frame |
| worst single frame's share of it | 18.1 % |

The two stills are unambiguous: the near hillsides and the valley floor sit metres too high
and read as slabs in `e2_burst_0_coarse_ancestors.png`, and are the Inn valley in
`e2_burst_1_rebuilt.png`. The per-frame column is the rate limit doing its job — a 58.5 %
change delivered in twelve instalments instead of one. The worst instalment is still 18 %,
and that is worth stating rather than hiding: nearest-first means the four biggest tiles on
screen go first, so the early instalments are the large ones. The alternative orderings trade
that against fixing the far field before the ground under the aircraft, which is the wrong
trade.

#### E2 — acceptance

* `cargo test --release --lib culling::` — **32 passed, 0 failed, 1 ignored**, no re-pin of
  `size_of::<QuadtreeNode<Ellipsoid>>() == 192`, `TilePatch<Ellipsoid> == 64`,
  `HorizonCamera == 56` or `test_visible_set_digest_is_stable`.
* **All 14 LOD-harness CSVs byte-identical** (`cmp`), `aggregate_ratio = 1.663`, 204 poses.
  With terrain off `TileSystem::select_mesh_rebuilds` returns on its first `Option` test and
  `missing_meshes` is bit-for-bit the list it was before E2.
* `testing::terrain::test_mesh_lifetime`, seven tests and one `#[ignore]`d measurement. The
  rebuild is driven end to end on the committed Zugspitze fixture — a z13 tile whose own
  fetch failed, built from its z12 parent (which is that fixture box-filtered 2:1, the
  relation a real pyramid has), then rebuilt when the retry lands:

  | source | max vertex error vs the DEM | RMS | height span |
  |---|--:|--:|--:|
  | z12 ancestor | 979.0 m | 149.0 m | 1 991.8 m |
  | own z13 tile | 0.0 m | 0.0 m | 2 032.7 m |

  and the burst test drains 54 stale meshes over six levels at ≤ 4 per frame, in exactly
  `ceil(54/4) = 14` frames, nearest first, with none starved.
* The five `rendering::terrain_capture` poses are unchanged to the tile: `terrain_off`
  34 / 39 / 36 / 33 / 11, `terrain_on_d3` 110 / 52 / 75 / 75 / 12 — E1's table exactly. The
  three `terrain_e3_capture` shots likewise (Innsbruck 83 tiles, 305 m AGL). Looked at, not
  only counted.

### E3 — what landed

**1. Field elevation, flipped.** `FlightPlanConfig::terrain_elevation` now defaults to `true`.
Nothing in the planner changed; the flag simply stopped being vetoed. The default config still
carries `dep_elevation_m = arr_elevation_m = 0.0`, so a caller that supplies no elevation gets
the same sea-level plan it always got — the flip changes what happens to an elevation that *is*
supplied. `nativeSetFieldElevations` no longer has to set the flag itself, only hand over the
two numbers.

*The one thing that looks like a bug and is not.* With `TerrainConfig::enabled == false` the
globe is a sea-level sphere again, while the plan still puts the aircraft at Bogotá's 2 548 m.
It floats, by exactly the field elevation. That is a correct plan drawn on a surface that is
not there, and the fix is to switch terrain on, not to plan the flight at sea level. The
explicit `terrain_elevation: false` escape hatch is kept and tested
(`field_elevation_can_still_be_switched_off`) for a caller that deliberately runs flat.

**2. A near plane that knows where the ground is.** `Camera::altitude()` is unchanged and
still measures the ellipsoid — fog density, the label zoom bucket and `TerrainHorizon`'s own
gate genuinely want that. What was wrong is that `znear` used it too. `Camera::altitude_agl()`
is new, and both projection matrices (f32 and f64) derive `znear` from it.

The camera cannot reach the height cache, so the ground comes to it: `update_logic` samples
`TileSystem::ground_height_at(camera_position)` once per frame and writes it into a private
`Option<f32>` on the camera. Three new pieces underneath, each with one job:

| | |
|---|---|
| `geometry::ecef_to_lon_lat_f64` | inverts *this engine's* ECEF map. The `φ` in `lon_lat_to_ecef_f64` is parametric, not geodetic — 0.19° apart at 45°, which is **11 km of ground**. Inverting the textbook formula instead would have sampled the wrong valley. |
| `HeightTileManager::tile_uv_at_lon_lat` | inverts `web_mercator_y_to_lat_f64`, the expression every tile boundary and mesh row is built from, so a sample lands on the ground the mesh draws (I-5, one level down). |
| `TileSystem::ground_height_at` | `&self`, no LRU promotion, no fetch. Resolution follows what has landed: a z4-z6 continental average at cruise, the real z15 valley floor on approach. Applies `exaggeration`, which `peek_height_at` deliberately does not. |

*Why this is the flat path and not a flat-looking path.* With terrain off there is no
`HeightTileManager` at all, so `ground_height_at`'s first `?` returns `None` before any
geometry runs; the camera's field is `None` on every frame; and `altitude_agl()` is
`match None => self.altitude()` — a call to the same function, not a re-derivation of it.
`testing::terrain::test_ground_reference` pins this by recomputing the pre-E3 `znear`
expression by hand and comparing all 16 matrix elements **by bit pattern**, f32 and f64, over
five poses × two camera modes × three aspect ratios, including a camera that has been given a
ground height and then handed `None` again.

*What it buys, measured.* Standing on the Nordkette ridge at 2 045 m with 2 m of clearance,
the old near plane is `0.1 × 2 045 m = 204 m` and the new one `0.1 × 2 m = 0.2 m`. The same
pose rendered by the pre-E3 build (`2d4f2e9`) and by this one differ by a **wedge of missing
ground at the crest — 3.4 % of the lower half of the frame is empty in the old shot and 0.1 %
in the new one**. That hole is §8's "near terrain clips", photographed. Clamped at zero,
because the bilinear sample and the 16×16 drawn patch of the same field disagree by metres and
a camera can be a little under its own sampled ground.

*What it does not buy.* `znear` only enters the projection's depth row, so a pose with nothing
inside the old near plane renders identically. All fifteen `rendering::terrain_capture` shots
are **bit-identical** to `2d4f2e9` — same tile counts, zero differing pixels — which is the
strongest available statement that E3.2 changed nothing except the case it was for.

**3. The ground is a floor.** `enforce_bounds` kept the camera 2 m off the ellipsoid and flew
it through the Alps. Cesium's shape, ported: `MIN_COLLISION_TERRAIN_HEIGHT` = 15 km
(`minimumCollisionTerrainHeight`, same value), above which the terrain floor is not enforced
at all. Below it the floor is `t + ground + 2 m`, the same 2 m clearance, against a different
surface.

Two reasons for the threshold, and the first is the real one. Far from the ground the sample
comes from a z4-z6 ancestor and is a continental average, so a floor built on it would *move*
as tiles land — a wandering floor is worse than none. Second, 15 km clears Everest, so
nothing collidable is skipped at `exaggeration = 1`. (At an exaggeration high enough to lift a
summit past 15 km the camera can pass through it. That is a debug setting doing what a debug
setting does; scaling the threshold would make the flat path's constant depend on a terrain
field.)

Two refusals worth writing down:

- **Ground at or below sea level yields no floor.** The ellipsoid floor is already the higher
  of the two there, and a DEM reading of −430 m at the Dead Sea must not license the camera to
  descend *further* than it could before.
- **`set_ground_height(None)` never calls `enforce_bounds`.** An unconditional call would have
  given the flat path a per-frame distance clamp it never had, which `set_eye` (which enforces
  nothing) can be on the wrong side of. `feeding_none_every_frame_never_clamps_a_flat_camera`
  runs 240 flat frames over a camera parked outside its own `max_distance` and checks it has
  not moved by a bit.

The flat pin is `the_ellipsoid_floor_is_bitwise_unchanged_with_no_ground_known`: a camera put
5 km underground at four positions, compared against `t + 0.000002` written out on the same
operands, in `local_pos`'s own `f32` — comparing an f64 floor against an f32 field would fail
on a 0.4 m quantisation that is not a behaviour change.

**4. Labels on the ground.** `LabelManager::update` takes an `Option<&dyn GroundHeights>` — a
one-method trait in `label/mod.rs`, implemented by `TileSystem`, so the label pass does not
grow a dependency on the terrain stack in either build. Denver's label stops sitting 1.6 km
underground.

The lift happens **after** culling, at the `visible_labels.push`, and that placement is the
design and not an optimisation. §8 already observed that `label::culling` uses the exact
*point* test (Theorem 3.1), which handles points off the surface correctly, so lifting before
culling would have been sound — but it would have made the visible set depend on the height
cache, i.e. on which tiles happened to be resident. Lifting after it means what is visible is
bit-for-bit what was visible before, and only where it is drawn changed.
`the_lift_does_not_change_which_labels_are_visible` checks that at four ground heights,
including a sub-sea-level one. It also keeps the query off the hundreds of candidates about to
be rejected.

"Up" is the ellipsoid normal (`ellipsoid_up`, the normalised gradient of the implicit
function), not the radius — 0.19° apart at mid-latitude. The test that checks the lift is
vertical measures the sideways component in f64 as a rejection: an `acos` of two f32 unit
vectors near 1 has no significant digits left and reports ~2 km of drift for a pair one ulp
apart.

**5. Picking and pan — not done.** `intersect_ellipsoid` still intersects the ellipsoid, so a
drag started on a mountainside grabs the point where the ray crosses sea level instead. §8
already rated this lowest priority (sub-pixel except in mountains at low altitude), and two
things argue for leaving it rather than squeezing it in:

- **There is no green baseline to regress against.** `camera::test_touch`'s
  `test_touch_interpreter_single_finger_pan` and `test_camera_inertia_decay` fail at `2d4f2e9`,
  before any of this work. Changing what a drag ray hits, with the only two tests covering drag
  already red, means shipping a change nobody can tell is correct. Fixing those tests is its
  own piece of work and not part of E3.
- **It is not the same shape as 1-4.** Those four are a sample under a known point; this one is
  a ray-vs-heightfield march, with its own step schedule, its own miss cases (a ray that grazes
  a ridge, a ray that leaves the loaded set) and its own refinement. The closed form is the
  right first guess, as §8 says, but the refinement around it is a piece of geometry, not a
  wiring change.

The rest of E3 does not depend on it: the drag ray's error is in *where the gesture anchors*,
and nothing in 1-4 reads that anchor.

### E3's correction — the ground query was measuring a surface nobody draws

Found in the Cesium comparison after F4, and it is a bug, not a polish item. All three of
E3.2, E3.3 and E3.4 ask `TileSystem::ground_height_at`, which asked
`HeightTileManager::peek_height_at_lon_lat`, which samples the **256×256 DEM bilinearly**.
The renderer does not draw that field. `TileMesh::generate_on` draws a triangle net through a
`(segments+1)² = 17×17` sub-grid of it, bisecting each quad SW–NE, and between two grid nodes
the drawn surface is a **plane**.

The two agree at the net's own vertices and nowhere else, and the disagreement has a sign:

* over a **summit** the net chords *under* the peak, the field reads higher than what is
  drawn, and the camera's floor is merely conservative;
* over a **dip** — a valley floor, a cirque, a stream cut, anything concave inside one mesh
  cell — the net chords *over* it, the field reads **lower**, `ground_height_at` under-reports
  the ground and `enforce_bounds` parks the camera **below the visible surface**.

**Measured** (`testing::terrain::test_ground_mesh`, the committed z12 Zugspitze fixture,
`mesh_segments = 16`, 25 921 samples on a lattice 10× finer than the mesh's own):

| | |
|---|---|
| the tile's own `HeightTile::detail` | 261 m |
| worst **net above field** (the dangerous sign) | **+178.5 m** |
| worst **net below field** (conservative) | −284.2 m |
| mean \|disagreement\| | 22.1 m |

And end to end, at the pose where the dip is worst (10.968 E, 47.415 N — field 2 282.2 m,
drawn net 2 460.7 m): a camera driven onto its collision floor came to rest **176.9 m below
the triangle the renderer draws**, measured by intersecting the camera's own ray with that
triangle in ECEF. So the effect is exactly the size the reasoning predicted — the `detail` of
the tile at its drawn level, three digits of metres at z12 — and not smaller.

**The fix is exact, because the triangulation is ours.**
`HeightTileManager::peek_mesh_height_at_lon_lat` reads the four grid nodes of the cell through
the same `resolve_source` + `sample_bilinear` pair `HeightPatch::sample` built the vertices
from, and interpolates barycentrically across the SW–NE bisection — which is Cesium's
`triangleInterpolateHeight` (`Core/HeightmapTerrainData.js`, "The HeightmapTessellator bisects
the quad from southwest to northeast"), same diagonal, same expressions. It reproduces the
built `TileMesh` to **18.7 mm** over the whole tile; the residual is the curvature sagitta over
one grid step plus the f32 vertex quantisation, and it is stated rather than corrected.

**The feed was the actual work.** `peek_mesh_height_at_lon_lat` is only exact at the level the
mesh under the point was *drawn* at, and neither the height cache nor `TileSystem` knew that —
`peek_height_at_lon_lat` started at `max_level` and walked up. The coupling added is one type,
`tiles::system::DrawnMeshes`: `wgpu_state` hands it the **renderable** set once per frame (the
one with the parent-mesh fallback already applied, because a tile whose own mesh has not
arrived is drawn with its parent's grid), and it answers one question — the level of the
deepest drawn tile over a point. It is written where `renderable_tiles` exists and read at the
top of the *next* `update_logic`, which is the frame those meshes are on the card for. With
terrain off it is never written and `ground_height_at` returns `None` before reaching it.

Where nothing is drawn — behind the globe, outside the frustum, the first frame — the query
falls back to `peek_height_at_lon_lat` unchanged. There is no drawn surface there to agree
with.

*What it cost the captures.* 38 of the 40 terrain shots are **bit-identical**. The two that
moved are the two the fix is for: `skbo_bogota_sea_level`, where the camera rises 1.9 m onto
the drawn tarmac instead of standing 0.1 m above it (the shot is visually indistinguishable —
a 1.9 m lift at 2 m AGL is a parallax shift in the grazing far field, and the horizon,
skyline and tile set all read the same), and `e2_burst_0_coarse_ancestors`, 0.8 % of pixels,
whose staged rebuild shifts by one frame. `alps_zugspitze`'s height-tile count swaps 137/138
between the two arms; the images are identical.

### E3 — captures

`rendering::terrain_e3_capture` (`#[ignore]`d, needs the network) renders the two mountain
airports and the Nordkette. What the shots show, with terrain on throughout:

| shot | what it says |
|---|---|
| `lowi_innsbruck_on_ground` | 305 m over the runway 26 threshold, the Inn valley and both walls in relief, the runway markings sharp in the near field. The plan's 584 m and the DEM's 579 m agree. |
| `lowi_innsbruck_sea_level` | the old default asked for 303 m — 278 m inside the valley floor. E3.3 puts the camera on the runway at 2.1 m instead, and E3.2 draws the tarmac under it rather than clipping it. |
| `lowi_innsbruck_flat_globe` | the documented caveat: the same 884 m eye over a sea-level sphere. Dead flat horizon, the Alps a painted texture, the aircraft floating by exactly the field elevation. |
| `skbo_bogota_on_ground` | 305 m over SKBO at 2 851 m, the sabana and the Cerros Orientales standing up on the horizon. |
| `nordkette_sunk_terrain_on` | asked for 600 m, 1 400 m inside the ridge; came out at 2 045.4 m, which is the 2 043 m ridge plus the 2 m clearance. The Karwendel beyond is drawn, not rock. |

---

## 9. Phase F — turn it on

Four pieces: the density decision C4 deferred (F1), the memory split (F2), the on-device soak
(F3) and the flip (F4). Three of them landed 2026-09-21. **The fourth could not be run**, and
that is the first thing this section has to say.

### The device line, stated before anything else

**`adb` is not installed on the machine Phase F was written on, and no phone is attached.**
`tools/phone_soak.sh` cannot run here, was not run here, and none of the numbers below came off
a device. Everything in F1 and F2 is a desktop measurement over the real DEM; F3 is the runbook
that closes the gap, written so that the person with the phone can execute it cold; F4 flips the
default on desktop and leaves Android explicitly off, because the flip §9 originally described
was conditioned on exactly the measurement that is missing.

### F1 — `mesh_segments` stays 16, and here is the margin it stays on

C4 (§6) measured the error and the bytes on the committed fixtures and deliberately refused to
decide, deferring to "device measurements". There are none, so F1 decides on what exists:
`testing::terrain::test_mesh_density::f1_mesh_density_at_the_real_poses`, the same ten real-DEM
poses E1a used, at the shipped `max_geometric_error_px = 12`.

| `mesh_segments` | Σ tiles, 10 poses | verts/tile | buffer B/tile | drawn buffers at `alps_inn_valley` | 512-entry mesh cache | drawn p95 error at `alps_inn_valley` | `4 × HeightPatch::sample` per frame |
|--:|--:|--:|--:|--:|--:|--:|--:|
| **16** | **702** | **361** | **15 440** | **1.6 MB** | **7.9 MB** | **10.7 px** | **31.3 µs** |
| 32 | 703 | 1 225 | 53 072 | 5.6 MB | 27.2 MB | 7.8 px | 123.9 µs |
| 64 | 703 | 4 489 | 195 920 | 20.8 MB | 100.3 MB | 4.8 px | 329.8 µs |

The last column is E2's table (§8), repeated because it is part of the price. The mesh-cache
column counts the index buffer, which §6's 5.8 / 19.6 / 71.8 MB figures did not.

**Read the marginal column, the way E1a read its own.** 16 → 32 buys 2.9 px of shape for
+19.3 MB of resident geometry and +93 µs a frame; 32 → 64 buys 3.0 px for +73 MB and +206 µs.
Per megabyte of mesh cache that is 0.15 px, then 0.04 px. There is no knee in favour of
refining — the first step is already the expensive one, because the tile count does not fall to
pay for it.

**And the tile count really does not move: 702 → 703 → 703.** Density reaches the quadtree
through exactly one path, `heightfield::skirt_allowance`, whose sagitta term shrinks with it;
the LOD term does not see it at all. So a denser mesh is pure cost in every column except
error. This is the opposite of E1a's trade, where the knob bought error *with* tiles.

**The finding that makes this more than an arithmetic preference — the two knobs are coupled,
and the coupling is invisible at 16.** `HeightTile::detail` is documented as *the* geometric
error of the drawn surface. It is a deviation from a **16:1 decimation** (`HEIGHT_DETAIL_STEP`),
which is the mesh lattice at `mesh_segments = 16` and at no other setting. At 32 the mesh is
twice as fine and the stored number is simply stale:

| fixture (z12) | `detail()` = 16:1 | 8:1 (`= 32`) | 4:1 (`= 64`) | over-statement at 64 |
|---|--:|--:|--:|--:|
| Everest | 602 m | 494 m | 247 m | 2.4× |
| Zugspitze | 261 m | 167 m | 119 m | 2.2× |
| Monterey coast | 30 m | 24 m | 22 m | 1.4× |

(These are the **decimation** metric the engine stores, not C4's drawn-mesh error, which also
carries the map projection and the triangulation — hence 602 m here against C4's 658.5 m for
the same fixture at the same density. The three columns are comparable with each other and with
what `apply_lod` reads, which is the point of measuring them this way.)

`test_mesh_density::the_error_term_measures_a_sixteen_to_one_decimation_whatever_the_mesh_draws`
is the mechanical statement: a ridge whose crests land exactly on the 32-lattice is drawn
*exactly* by a `mesh_segments = 32` mesh, and `detail()` still reports its full 800 m. The
decode has no access to the configuration, so the number cannot follow the mesh.

What that costs is precisely E1's calibration. At 32 the engine would refine on an error it has
already removed — paying tiles for shape it is drawing — and `max_geometric_error_px = 12`,
which was fitted against the *measured* error, would no longer be measuring anything. Moving to
32 is therefore not a one-line config change; it is "make the error term follow
`mesh_segments`", which means moving the measurement out of the decode or storing it per step.
That work has a reason to exist only once someone has a device number saying 16 is not enough.

**Where 16 already is.** At the shipped budget the drawn p95 error at 16 is 10.7 px at
`alps_inn_valley`, and seven of the other nine poses land between 3.7 and 11.0 px — *inside* the
12 px budget E1a picked. The two that do not are `terai_to_himalaya` at 13.1 px and
`himalaya_everest` at 93.5 px, and neither is fixed by density (below). Refining the mesh spends memory to reduce an error that is already under budget,
while the knob that is calibrated (`max_geometric_error_px`) is the one that would spend it on
error that is over.

**One pose refuses to improve at any density, and it is worth stating**: `himalaya_everest`
reads 93.5 → 93.6 → 93.7 px across 16/32/64. Both error columns are a p95 over a per-tile
*maximum*, and a cliff deviates from its chord by roughly half its own height at every lattice
spacing — C4's "max error is not monotone" finding, at the scale of the Khumbu. A density
argument made from that pose alone would conclude, wrongly, that density does nothing anywhere.

**So: 16 stays, until someone measures on the device.** Not because 32 is wrong — its error
column is genuinely better — but because the cost is 3.4× the resident geometry and 4× the
per-frame sample cost, the benefit is below a budget that is already met, and collecting the
benefit honestly requires decoupling `detail` from `HEIGHT_DETAIL_STEP` first. The measurement
that would overturn this is in F3: if the S23's frame time is *not* the binding constraint and
its thermal headroom is comfortable, 32 becomes an argument about VRAM alone, and §11's
"desktop at 32, S23 at 16" split becomes a live option rather than a hypothetical.

### F2 — where the bytes go

§9 asks for the split as imagery vs height vs vertex bytes. B4 also makes a claim about it that
no config listing can settle — that the height cache is a *declared slice* of
`tile_cache_budget_bytes` and not an addition to it — so
`test_mesh_density::f2_where_the_bytes_go_at_the_real_poses` measures it at the **shipped**
budget, not at the 1 GiB the occlusion harness uses to keep itself from evicting.

Declared, from `TileEngineConfig::default()` with terrain on:

| | bytes | entries |
|---|--:|--:|
| `tile_cache_budget_bytes` | 512.0 MiB | — |
| ├ imagery share | 480.0 MiB | 480 at 512²×4 |
| └ height share | 32.0 MiB | 254 at 132 098 B |
| vertex buffers | 7.9 MB | 512 meshes at 15 440 B |

Measured, per pose, at the default Carto `@2x` style:

| pose | tiles | imagery | height sources wanted | height resident | height | vertex |
|---|--:|--:|--:|--:|--:|--:|
| `alps_inn_valley` | 103 | 103.0 MiB | 312 | **254 / 254** | 32.0 MiB | 1.6 MB |
| `alps_low` | 73 | 73.0 MiB | 276 | **254 / 254** | 32.0 MiB | 1.1 MB |
| `alps_zugspitze` | 47 | 47.0 MiB | 208 | 208 / 254 | 26.2 MiB | 0.7 MB |
| `himalaya_everest` | 75 | 75.0 MiB | 204 | 204 / 254 | 25.7 MiB | 1.2 MB |
| `himalaya_limb_400km` | 11 | 11.0 MiB | 52 | 52 / 254 | 6.6 MiB | 0.2 MB |
| `po_plain_to_alps` | 43 | 43.0 MiB | 204 | 204 / 254 | 25.7 MiB | 0.7 MB |
| `terai_to_himalaya` | 58 | 58.0 MiB | 200 | 200 / 254 | 25.2 MiB | 0.9 MB |
| `rhone_valley` | 65 | 65.0 MiB | 240 | 240 / 254 | 30.2 MiB | 1.0 MB |
| `salzach_to_alps` | 85 | 85.0 MiB | 268 | **254 / 254** | 32.0 MiB | 1.3 MB |
| `aosta_valley` | 52 | 52.0 MiB | 240 | 240 / 254 | 30.2 MiB | 0.8 MB |

(The same run at the Esri 256² style the §7/§8 tables use draws 701 tiles for 175.2 MiB of
imagery and the same height and vertex columns within a few percent; its `lod_factor` is twice
as eager, so it is the heavier case for tiles and the lighter one for bytes.)

Three things this says, none of which were visible from the config:

1. **The declared slice holds, and imagery is nowhere near its share.** The worst pose draws
   103 MiB of texture against a 480 MiB budget — 21 %. Terrain's 32 MiB came out of slack.
2. **The height slice is the one that binds, at three of the ten poses.** `alps_inn_valley`
   asks for 312 distinct source tiles over a settle and the cache holds 254, so tiles are
   evicted and re-fetched while the camera has not moved. That is not a correctness problem —
   `status_of` treats a missing tile as "not ready yet" and the mesh comes from an ancestor —
   but it is churn, and it is exactly where terrain matters most: low, in a valley, with both
   walls and their ancestor chains resident. The obvious remedy is to move the split (48 MiB of
   heights would cover every pose here and still leave imagery 4× its measured peak), and the
   reason this section does **not** do it is that it is a device-memory decision and F3 is where
   device memory gets measured.
3. **Vertex bytes are not in the byte budget at all.** `mesh_cache_size` is a *count* — 512 —
   so the geometry ceiling is whatever 512 meshes happen to weigh: 7.9 MB at `mesh_segments`
   16, 100.3 MB at 64. This is the same failure mode the imagery budget was introduced to fix
   (§B4's note: a count-only cap let textures climb past 1.9 GB on device), and it is a second,
   independent reason F1 does not raise the density on an unmeasured platform.

### F2b — the churn F2 predicted is not there, and the slice moves anyway

F2's second finding was that the height slice binds at three of the ten poses —
`alps_inn_valley` wants 312 distinct sources and the cache holds 254 — and that the
consequence is eviction and re-fetch "while the camera has not moved". It also named a
remedy (48 MiB) and declined to apply it. Before applying it, F2b went to measure the churn,
and found the premise wrong.

**F2's 312 is not a number production ever asks for.** It comes from
`test_terrain_occlusion::collect_sources`, which recurses the **whole quadtree** — interior
nodes, culled nodes, everything the tree holds. Production asks for height tiles in exactly
one place, `TileSystem::request_height_chain`, and it is called for the **visible set** and
for `missing_meshes`, each with its ancestor chain. Nodes that are in the tree but not drawn
never get a request: their `HeightBounds` come from D1's inheritance margin, which is what
that margin is for.

`testing::terrain::test_height_residency::double_fetches_at_the_binding_poses` replays that
real request sequence — the arriving tree frame by frame, then the settled set, 24 frames at
16 ms — against the **real** `HeightTileManager`, the real `TileCacheManager`, the real
`TileFetcher` with its real tokio worker, and a local HTTP source that counts every GET and
answers after 60 ms (the optimistic end of a real Terrarium round trip, so every number below
is a lower bound on the shipped source).

| pose | style | F2's whole-tree count | production asks | resident | evicted in flight | GETs | distinct | repeats |
|---|---|--:|--:|--:|--:|--:|--:|--:|
| `alps_inn_valley` | carto @2x | 312 | **220** | 220 / 254 | 0 | 220 | 220 | **0** |
| `alps_low` | carto @2x | 276 | 177 | 177 / 254 | 0 | 177 | 177 | 0 |
| `salzach_to_alps` | carto @2x | 268 | 179 | 179 / 254 | 0 | 179 | 179 | 0 |
| `rhone_valley` | carto @2x | 240 | 156 | 156 / 254 | 0 | 156 | 156 | 0 |
| `alps_inn_valley` | esri 256² | 316 | **220** | 220 / 254 | 0 | 220 | 220 | 0 |
| `alps_low` | esri 256² | 276 | 177 | 177 / 254 | 0 | 177 | 177 | 0 |
| `salzach_to_alps` | esri 256² | 280 | 188 | 188 / 254 | 0 | 188 | 188 | 0 |
| `rhone_valley` | esri 256² | 248 | 161 | 161 / 254 | 0 | 161 | 161 | 0 |

**Nothing is evicted, nothing is fetched twice, at either shipped imagery style.** The worst
working set is 220 against a 254-entry cache. The same table at 48 MiB is identical in every
column but the capacity. So F2's "the height slice is the one that binds" was an artefact of
the harness it was measured with, and the churn it described does not happen.

**The double-fetch window is real, and it is not reached.** Two dedup layers stand between
the request and the wire, and *neither* covers the in-flight window:

1. `HeightTileManager::request_tile` returns early on `cache.get_state(&src).is_some()`, and
   the `Fetching` placeholder that makes that work is an ordinary `LruCache` entry.
   `LruCache::put` evicts the least-recently-used entry without asking what state it is in.
   `test_height_residency::the_cache_will_evict_a_tile_whose_fetch_is_still_in_flight` is
   that, in four lines.
2. `TileFetcher::request_tile` keeps a `HashSet` of queued ids — but removes an id when the
   worker **pops** the request, i.e. when the download *starts*, not when it finishes
   (`tile_fetcher.rs`, `q.1.remove(&r.id)` inside the pop). So it dedups the queued window
   and not the downloading one.
   `the_fetcher_dedups_the_queued_window_and_not_the_downloading_one` drops the placeholder
   mid-download and watches a second GET go out for a tile still on the wire.

Cesium's counterpart is `GlobeSurfaceTile.eligibleForUnloading`
(`Scene/GlobeSurfaceTile.js:112-131`), which refuses to free a tile whose imagery is
`RECEIVING` or `TRANSFORMING`. **That guard is not built here**, because the measurement says
there is nothing yet to guard against: reaching the window needs the working set to exceed
the capacity, and at 220 of 254 it does not. The two tests above stay in the gate so that if
it ever does, the mechanism is already written down and the fix is a state check rather than
an investigation.

**The slice is raised to 48 MiB anyway, for the reason F2 did not give.** 220 of 254 is 87 %
of the slice at 103 visible tiles, and F5 below refines the tree — which raises the visible
count and therefore the height working set. The raise is headroom for measured growth, not a
repair:

| | before | after (desktop) | after (Android) |
|---|--:|--:|--:|
| `height_cache_budget_bytes` | 32 MiB | **48 MiB** | 32 MiB (unchanged) |
| entries at 132 098 B | 254 | **381** | 254 |
| imagery share of the 512 MiB budget | 480 MiB | **464 MiB** | 480 MiB |
| imagery peak measured (F2) | 103 MiB | 103 MiB | — |

Imagery keeps 4.5× its measured peak, so B4's "a slice, not an addition" still holds and the
imagery side still never binds. **Android is deliberately left at 32 MiB**
(`HEIGHT_CACHE_BUDGET_BYTES` is a `cfg!(target_os)` split, like `TERRAIN_ENABLED_BY_DEFAULT`):
every Android memory decision in this file is held to the soak of F3, which has not been run,
and Android has terrain off by default, so on that platform this constant currently describes
memory nobody allocates. The split exists so that flipping the *other* constant does not
silently flip this one too.

### F3 — the S23 soak, ready to run

Everything below is executable on the machine with the phone, in order, with no decisions left
open. It is written for someone who has not read the rest of this document.

**What is being decided.** Whether terrain can be turned on for Android. Desktop is already on
(F4); Android is off, pinned by `TERRAIN_ENABLED_BY_DEFAULT` in
`crates/cesium-engine/src/globe/tiles/config.rs`, and this run is what unpins it.

#### Before the runs — `max_geometric_error_px` is in device pixels, and the phone is not the desktop

**Read this before the runs, because it decides what they measure.** The threshold is
`terrain_lod_factor = H / (E · 2·tan(fovy/2))`, and `wgpu_state` passes `self.size.height` —
the **physical** surface height out of the swapchain — with nothing dividing it. Cesium
divides its own screen-space error by `frameState.pixelRatio`
(`Scene/QuadtreePrimitive.js`), so Cesium's `maximumScreenSpaceError` is in CSS pixels and
`TerrainConfig::max_geometric_error_px` is in **device** pixels. On a pixel-ratio-1 display
the two coincide, which is why it has never come up.

The shipped `12` was picked at the E1a cost table's own rung — **1280×720, `Free`** (fovy
46.40°, `2·tan(fovy/2) = 0.857`), giving `720 / (12 · 0.857) = 70.0 Mm⁻¹`. Everything else,
computed rather than asserted:

| viewport | mode | H | 2·tan(fovy/2) | factor vs the rung | `E` for the same threshold |
|---|---|--:|--:|--:|--:|
| 1280×720 (E1a's table) | Free | 720 | 0.857 | 1.000× | 12 px |
| 1920×1080 desktop | Free | 1080 | 0.857 | **1.500×** | 18 px |
| S23 **landscape** 2340×1080 | Free | 1080 | 0.857 | 1.500× | 18 px |
| S23 landscape 2340×1080 | Cockpit | 1080 | 1.155 | 1.113× | 13.4 px |
| S23 **portrait** 1080×2340 | Free | 2340 | 0.857 | **3.250×** | 39 px |
| S23 portrait 1080×2340 | Cockpit | 2340 | 1.155 | **2.413×** | 29 px |

Against the 1080p desktop rather than E1a's rung: the S23 in **portrait** asks for a
threshold distance **2.167×** the desktop's in Free (`2340 / 1080`) and **1.608×** it in
Cockpit — the 60° cockpit fovy returns a factor 0.742 of the height term. **Landscape is the
flat case**: the S23's landscape height *is* 1080, so in Free it is bit-identical to the
desktop and in Cockpit it is 0.742× it.

Two consequences for the runs below, and they are the reason this sits above them:

1. **The phone is not running the desktop's configuration.** The soak is a cockpit run. Held
   in portrait it asks for 1.61× the desktop's threshold distance and correspondingly more
   tiles — on exactly the device whose memory pressure F2 found binding. Held in landscape it
   asks for 0.74× it. So "terrain on, S23" is two different measurements and the orientation
   has to be recorded with every number.
2. **The value is deliberately not changed here.** 12 is calibrated against the desktop
   measurement in `TerrainConfig::max_geometric_error_px`'s table, and dividing by a pixel
   ratio — or by `H / 720` — would require a new calibration that nobody without the device
   can perform. Picking it on the phone is part of F3's job: run the soak at 12, and if the
   tile count or the memory is what fails, re-run at the parity value from the table above
   (29 px portrait cockpit, 13.4 px landscape cockpit) before concluding anything about
   terrain itself.

`lod_factor_for` has the identical units question and **must not be touched**: its
`target_texel_ratio` default is calibrated against the hard-coded `2.0` that shipped before
WP3, on the same physical height, and the LOD harness's 204-pose CSVs are pinned to it byte
for byte.

#### 0. Prerequisites

```sh
adb devices          # expect: RFCX20QDV0T   device
```

The instrumented APK on the phone predates Phase F, so it **must** be rebuilt: the `.so` on the
device knows nothing about relief. `handoff.md` §1 "If the APK needs rebuilding" is the path and
it is unchanged — build on `lxhalle` (never locally), `llvm-strip`, transfer with
`base64 | grep`, `./gradlew :app:assembleDebug -x cargoNdkBuild`, `adb install -r -d`. Copy that
block verbatim; the only thing Phase F adds is that it is now mandatory rather than optional.

#### 1. The two arms

| arm | how to get it | what it is |
|---|---|---|
| **terrain off** | the APK as built — Android's default is off | the shipping baseline, the flat globe |
| **terrain on** | call `nativeSetTerrainEnabled(true)` from the Kotlin bridge once the flight is running | the same process, same textures, relief switched on live |

The runtime switch is the honest way round: `ViewerCommand::TerrainSetEnabled` rebuilds the
height manager in place and E2 rebuilds the flat meshes four per frame, so both arms are the
same binary and the same session and nothing else can have changed between them. If the Kotlin
side has no button wired to that JNI entry, the fallback is to rebuild with
`TERRAIN_ENABLED_BY_DEFAULT` forced to `true` (one line, the `cfg!` in `config.rs`) and install
the second APK — in which case run the two arms in the **same** order on the same charge state,
because the comparison is then across builds.

#### 2. The runs

Cockpit view, foreground, screen on, the same flight in every run. **Two poses × two arms, plus
one long run** — five invocations, in this order, each from a phone that has returned to ambient:

```sh
# 1. alpine airport, terrain OFF — fly LOWI (Innsbruck), hold 300-900 m AGL in the valley.
tools/phone_soak.sh 1800 20      # 30 min, sampling every 20 s

# 2. same pose, terrain ON  (nativeSetTerrainEnabled(true), then let it settle ~30 s)
tools/phone_soak.sh 1800 20

# 3. cruise, terrain OFF — 10-11 km, mostly far field, the common case.
tools/phone_soak.sh 1800 20

# 4. same pose, terrain ON
tools/phone_soak.sh 1800 20

# 5. the long one: whichever arm looked worse in 1-4, an hour of it. Thermal behaviour
#    does not show up in thirty minutes.
tools/phone_soak.sh 3600 30
```

The alpine pose is the one that matters: it is where the desktop measurement puts the height
cache at its ceiling (F2, 312 sources wanted against 254 resident) and where the visible set
roughly doubles. Tile counts on the phone will not equal the desktop table's — the S23's
viewport gives it its own `lod_factor`, see `docs/culling-baseline.md`'s viewport ladder — so
compare arm against arm, never phone against desktop.

Each run writes `tools/soak_<HHMMSS>/samples.csv` and `logcat.txt` and prints a first-third vs
last-third trend table; label the directories by arm immediately, because they are named by
clock time only. Charging keeps the phone warm and makes throttling *more* likely, so either
run everything unplugged over `adb tcpip` or run everything plugged in — and write down which.

#### 3. What decides it

Read the trend table twice: terrain-on against terrain-off (the cost of the feature) and
last-third against first-third within each arm (degradation over time). The second is the one
the flat globe has already been through; the first is new.

| number | what it means here | abort criterion |
|---|---|---|
| frame **p50** | the steady-state cost of relief | terrain-on p50 above 16.6 ms while off is below — the feature has eaten the frame |
| frame **p90 / p99** | mesh rebuilds and fetch stalls landing on the frame | p99 more than 2× the off arm's, or rising across thirds in the on arm while flat is flat |
| **jank %** | the same, where the user feels it | more than 5 points above the off arm |
| **total PSS** | the whole process | delta over the off arm above ~150 MB, or climbing monotonically across thirds (a leak, not a working set) |
| **native heap** | where the height cache and the decoded tiles live | delta above 64 MiB — twice the declared 32 MiB slice. If the slice is not bounding, B4's claim is wrong on this platform and that is the finding |
| **battery temp** | thermal headroom | on-arm ending more than 3 °C above the off arm at the same charge state |
| **prime clock** | throttling, the mechanism behind the above | last-third prime clock dropping in the on arm while the off arm holds |
| `logcat` | correctness | any `panic`, `FATAL EXCEPTION`, `OutOfMemory` — the script greps for these already |

Expected, from the desktop numbers, so that a surprise is recognisable as one: **+32 MiB**
native heap (the height cache, at its declared ceiling in the valley), **+2 MB** of vertex
buffers, imagery *unchanged or slightly lower* (its budget shrank by the height slice), and a
visible set roughly 2× the flat one at low alpine poses (106 against 43-53) with the per-frame
mesh work bounded at 31.3 µs by E2's four-rebuild budget.

#### 4. What to do with the result

* **Passes** — flip `TERRAIN_ENABLED_BY_DEFAULT` to an unconditional `true`, delete the
  platform `cfg!`, and record the trend tables here under an "F3 — measured" heading.
* **PSS or native heap binds** — the first knob is `TerrainConfig::height_cache_budget_bytes`,
  and F2 already says which way it wants to move (up, not down: 254 entries is short at three
  poses). Moving it down instead trades churn for residency and needs the same soak again.
* **Frame time binds** — the first knob is `MESH_REBUILD_BUDGET_PER_FRAME` (E2, currently 4),
  then `max_geometric_error_px` (E1a's table gives the tile cost of every value), and only then
  `mesh_segments`, whose coupling F1 describes.
* **Thermals bind** — nothing in terrain is a per-frame CPU cost worth tuning at 0.19 % of a
  frame; look at the tile count first, which means `max_geometric_error_px`.

**The desktop comparison basis left for that run** is F1's and F2's tables above, E2's
per-rebuild costs (§8), E1a's tile-count-against-budget table (§8) and the five
`rendering::terrain_capture` poses' tile counts (§8 acceptance) — all of them re-measured on
2026-09-21 and unchanged by F4.

### F4 — the flip, and the hole in it

`TerrainConfig::enabled` now defaults to `TERRAIN_ENABLED_BY_DEFAULT`
(`crates/cesium-engine/src/globe/tiles/config.rs`), which is `true` everywhere except Android.

**Desktop, on.** Covered by F1 and F2: the visible set is measured, the geometric error is
inside its budget, the memory split is measured at the shipped budget, and the captures are
unchanged.

**Android, off, and deliberately so.** §9 bound this flip to the soak. The soak did not happen.
Turning Android on anyway would be presenting an unmeasured configuration as a measured one,
and the specific risk is not hypothetical — F2 shows the height cache at its ceiling at the
poses a flight tracker spends its interesting minutes in, on a device whose memory pressure has
already forced one budget into existence (§B4's 1.9 GB note). So Android keeps the flat globe
until the F3 run says otherwise, and the switch is one constant with the reason attached.

**Why one constant rather than a flag at the entry point.** Android reaches this engine through
more than one door. `android_main` builds its config from `TileEngineConfig::default()` directly
— *the builder's config is not what it runs*, which is worth knowing independently of terrain —
and `headless::api`'s FFI still renderer is compiled for Android too. A flag set at one door
would have been missed at the other; a constant in `Default` cannot be.

**What the flip moved, and what it did not.** Five rendering harnesses inherited
`TileEngineConfig::default()` and would have silently gained relief: the fog and haze captures,
the Free-mode route shots, the S23 cockpit shots and the flicker tracker. Each now states
`enabled: false` with its reason, because an instrument whose baseline was recorded flat has to
stay flat to be an instrument. The LOD harness and the culling gate were checked for the same
exposure and do not have it — both construct `QuadtreeManager` directly and never build a
`TileEngineConfig` — which is why the 204-pose CSVs are byte-identical across the flip.

### Phase F — acceptance

* `cargo test --release --lib culling::` — **32 passed, 0 failed, 1 ignored**, no re-pin of
  `size_of::<QuadtreeNode<Ellipsoid>>() == 192`, `TilePatch<Ellipsoid> == 64`,
  `HorizonCamera == 56` or `test_visible_set_digest_is_stable`.
* **All 14 LOD-harness CSVs byte-identical** (`cmp`) across the flip, `aggregate_ratio = 1.663`,
  204 poses — the flat path did not move when the default did.
* `testing::terrain::test_mesh_density`: two gate tests (the error term's decimation, and C4's
  buffer columns still being the buffer columns) and three `#[ignore]`d measurements.
* The five `rendering::terrain_capture` poses re-run after the flip and unchanged to the tile:
  `terrain_off` 34 / 39 / 36 / 33 / 11, `terrain_on_d3` 110 / 52 / 75 / 75 / 12. Looked at, not
  only counted — the Inn valley and the Everest massif are gapless, and the limb pose still
  shows no hole.
* **Not done, and not simulated: the S23 soak.** F3 is the runbook; the numbers it asks for do
  not exist yet, and no table in this document pretends they do.

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
  *(Closed by D1/D2, 2026-09-20. `enabled` stayed `false` afterwards for D3's sake and
  Phase F's, not for soundness; F4 flipped it on desktop 2026-09-21 and left Android off
  pending the soak — §9.)*
- **E3.1 can jump the queue** as soon as C looks right; it is one line and the most visible
  thing here.
- **D3 is the constraint-2 deliverable.** Do not let it slide to the end.
  *(Landed 2026-09-20. FN = 0, and see §7b for what it removes and — more usefully —
  where it does not. Followed up 2026-09-21 in §7c: the per-sub-patch refinement §7b
  proposed was built, measured and removed; the march is 27–43 % cheaper.)*

---

## 11. Open decisions

1. **Vertical exaggeration** — user knob or fixed at 1.0? Nearly free in C1, not free to
   retrofit after D1.
2. **Android in the same release?** *Answered, provisionally and in the other direction:
   **no**, not until the §9 F3 soak runs. F1 found the split this line anticipated is not
   available as a free config choice either — `HeightTile::detail` is a 16:1 decimation, so
   "desktop at 32" would decalibrate E1's error budget unless the error term is made to follow
   `mesh_segments` first. Both targets run 16 today.*
3. **Terrain shadows / AO** — out. `render_scene` is a single pass; that is a renderer change,
   not a terrain change.
4. **The height cache's share of the tile budget.** F2 measured 32 MiB / 254 entries binding at
   three of ten real poses, against imagery using 21 % of its 480 MiB. Moving the split is a
   two-line change and a device-memory decision; it waits on F3.
