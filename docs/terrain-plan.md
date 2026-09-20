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
  *(Closed by D1/D2, 2026-09-20. `enabled` stays `false`, now for D3's sake and Phase F's,
  not for soundness.)*
- **E3.1 can jump the queue** as soon as C looks right; it is one line and the most visible
  thing here.
- **D3 is the constraint-2 deliverable.** Do not let it slide to the end.
  *(Landed 2026-09-20. FN = 0, and see §7b for what it removes and — more usefully —
  where it does not.)*

---

## 11. Open decisions

1. **Vertical exaggeration** — user knob or fixed at 1.0? Nearly free in C1, not free to
   retrofit after D1.
2. **Android in the same release?** Phase F may answer "desktop at `mesh_segments = 32`, S23 at
   16" — supported, but it means two tuning targets.
3. **Terrain shadows / AO** — out. `render_scene` is a single pass; that is a renderer change,
   not a terrain change.
