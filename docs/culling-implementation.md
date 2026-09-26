# Culling implementation

*Implementation reference for globe visibility culling. This document describes what the
code **does**; [culling-math.md](culling-math.md) derives why the mathematics is what it is
and carries the proofs.*

*A bare section reference such as §3.4 points into `culling-math.md`; a reference to this
document always says "above" or "below".*

Read this before changing anything in `crates/cesium-engine/src/globe/quadtree/`, or to
follow the path a tile takes from construction to the visible set without first reading a
derivation. Level of detail — how deep the tree refines — is in
[tiles-and-lod.md](tiles-and-lod.md); the terrain-specific payloads and the occlusion march
are in [terrain.md](terrain.md).

---

## 0. Orientation

### 0.1 The one-paragraph version

Once per frame the engine builds a **camera-relative frustum** — four inward side-plane
normals, the eye in f64, eight frustum corners — and a **`HorizonCamera`**, the camera
reduced to scaled space where the ellipsoid is the unit sphere. It then walks a quadtree of
tiles from four z = 1 roots. Each node answers one question, *can any drawable point of my
patch be on screen?*, with a sequence of tests ordered cheapest-and-most-selective first:
the horizon test, then (terrain only) the test against guaranteed ridges, then the four
planes, then a vertex witness, then the exact separating-axis set, then the same tests
again on a `k × k` grid of sub-patches. A node that survives is marked visible and
subdivided if the LOD rule says so; a node that fails is marked invisible and its **entire
subtree is deleted**. The visible set is the set of surviving leaves. The sequence itself is
data — a `CullPipeline` of `Stage`s carried on the per-frame context (4.0 below) — so stages
can be switched off; the LOD rule deliberately is not one of them.

### 0.2 Two surface models

Everything is generic over a `SurfaceModel` (`quadtree/surface.rs`): `Ellipsoid` for the flat
globe, `Heightfield` for relief. `AnyQuadtree` (`quadtree/any.rs`) holds one
`QuadtreeManager` of either type and dispatches once per call, never per node. Where the two
differ in this document, it says so; the flat globe's per-node code is the code that would
exist if terrain did not.

| | `Ellipsoid` | `Heightfield` |
|---|---|---|
| Horizon stage | exact rectangle supremum (§3.4) | cone test on a scaled-space sphere (§3.7) |
| Box fitted over altitude | 0 | the node's `[lo, hi]` height interval |
| Pipeline | `CullPipeline::DEFAULT` | `CullPipeline::TERRAIN_DEFAULT` (adds `TerrainOcclusion`) |
| Per-frame passes before `update` | none | `refresh_extras` (height bounds), `refresh_terrain_horizon` (occlusion march) |

### 0.3 File map

| file | what lives there |
|---|---|
| `globe/quadtree/tile_id.rs` | `TileId`, `TileBounds`, `tile_bounds()` — the single source of a tile's rectangle (I-5) |
| `globe/quadtree/horizon.rs` | scaled space, `HorizonCamera`, `TilePatch`, the exact limb test, the point test, the sphere cone test |
| `globe/quadtree/bounding_volume.rs` | `OrientedBoundingBox`, `Frustum`, the four-plane test, the vertex witness |
| `globe/quadtree/slab.rs` | the separating axes the four planes leave out |
| `globe/quadtree/quadtree.rs` | OBB fitting, `SubGrid`, the `CullPipeline` of `Stage`s, `QuadtreeNode::update`, LOD |
| `globe/quadtree/surface.rs` | `SurfaceModel`, `Ellipsoid` |
| `globe/quadtree/any.rs` | `AnyQuadtree`, the run-time choice of surface model |
| `globe/quadtree/terrain_occlusion.rs`, `terrain_relief.rs` | the occlusion march and its pre-check (terrain only) |
| `globe/terrain/heightfield.rs` | `Heightfield`, `HeightBounds` |
| `camera/camera.rs` | `calculate_frustum_planes`, `frustum_corners_relative`, `global_transform_f64` |
| `render/wgpu_state.rs` | the per-frame driver (`update_logic`) |
| `globe/geometry.rs` | ellipsoid constants, `lon_lat_to_ecef_f64`, `TileMesh::generate_on` |
| `label/culling.rs` | the label path, which shares `HorizonCamera` and `Frustum` but not the tile tests |

---

## 1. Frames and units

### 1.1 Megametres

Every length in the engine is in **megametres**: 1 unit = 1 000 km. No scale factor is
applied anywhere in the culling path — the constants are simply small.

```
EARTH_RADIUS_A_F64 = 6.378137        globe/geometry.rs
EARTH_RADIUS_B_F64 = 6.3567523142
```

The f32 twins `EARTH_RADIUS_A_F32` / `EARTH_RADIUS_B_F32` exist for the mesh and shader
path. `B_F32` differs from the f64 constant by 8.6 cm; nothing in the culling path reads it,
and nothing in the culling path should start to (I-5).

A useful mental conversion while reading tolerances: **1e-9 Mm = 1 mm**, and 1 ulp of f32
at Earth scale (`2⁻²⁴ · 6.378`) is **0.38 m**. That number is the whole reason the frame
below exists.

### 1.2 ECEF: Y-up with negated Z

```rust
// globe/geometry.rs, lon_lat_to_ecef_f64
x =  a·cos φ·cos λ
y =  b·sin φ
z = −a·cos φ·sin λ
```

It is **`y`** that carries the semi-minor axis, not `z`. This is load-bearing in two places:
the scaled-space map (1.3 below) and the analytic `east` basis vector (3.2 below).
`test_scaled_space_maps_surface_to_unit_sphere` pins the convention so a move to Z-up
cannot silently invert it.

### 1.3 Scaled space

```rust
// horizon.rs, transform_to_scaled_space
T(p) = (p.x / a, p.y / b, p.z / a)
```

`T` is linear, diagonal and positive, so "the segment eye→p passes through the solid Earth"
is invariant under it — and under `T` the WGS-84 ellipsoid becomes the **unit sphere** while
a Web-Mercator tile stays an exact spherical rectangle `[λ₀,λ₁] × [φ₀,φ₁]` with the *same
numeric* λ and φ. That is what makes the horizon test a closed form rather than an
iteration. Conversion happens on the camera once per frame (`HorizonCamera::new`), on a
label point per label (`point_is_occluded`), and, for terrain, on each node's box at
construction (`ScaledSphere::around_obb`). Flat tile patches are never transformed — their
λ/φ are already the right numbers.

### 1.4 The camera-relative frame

This is the central precision decision, and it is why `Frustum` owns the eye.

All four side planes pass through the eye, so in a frame whose origin is the eye their
offset is **identically zero** — written as a literal absence, not a computed near-zero
(there is no `d` field). Every position entering a plane test goes through:

```rust
// bounding_volume.rs, Frustum::relative
pub fn relative(&self, p: DVec3) -> Vec3 {
    let d = p - self.eye;                                    // f64 subtraction
    Vec3::new(d.x as f32, d.y as f32, d.z as f32)             // then downcast
}
```

The subtraction is f64 and only the **small difference** is downcast. This turns a
distance-independent ~3 m f32 error floor into `1.5·10⁻⁷·‖Δ‖`, a constant 6·10⁻⁷ of a tile
at every zoom (§2.5). The corollary is invariant **I-2**: the OBB centre is stored in f64
or there is nothing left to subtract precisely — an f32 centre already carries ~0.5 m of
construction error that no later arithmetic can undo.

`relative_f32` exists for callers holding an f32 world point (labels): it promotes *before*
subtracting, so only the point's own quantisation survives.

### 1.5 Where each conversion happens

| conversion | site |
|---|---|
| camera local → global, f64 | `Camera::global_transform_f64` |
| view-projection → 4 side-plane normals, f64 | `Camera::calculate_frustum_planes` |
| plane normals f64 → f32 | `Frustum::planes_only` |
| NDC → world frustum corners, f64, eye subtracted before downcast | `Camera::frustum_corners_relative` |
| far-quad corners f32 → unit edge rays, f64 | `Frustum::with_corners` |
| world → camera-relative, f64 subtract then downcast | `Frustum::relative` |
| world → scaled space (camera) | `HorizonCamera::new` |
| patch λ/φ → eight trig constants, f64 | `TilePatch::for_surface` |
| OBB half-axes f64 → f32 | `fit_obb` |
| node centre → LOD distance, f64 subtract then downcast | `QuadtreeNode::apply_lod` |

---

## 2. Once per frame

Driver: `WgpuState::update_logic` (`render/wgpu_state.rs`). After the extension has moved
the camera and collision has settled it (see [architecture.md](architecture.md#one-frame)):

```rust
let frustum = self.camera.calculate_frustum_planes(aspect_ratio);
let frustum_obj = Frustum::planes_only(frustum, camera_pos_dvec)
    .with_corners(self.camera.frustum_corners_relative(aspect_ratio));
// set_frame_params / set_terrain_lod, then — terrain arm only —
// refresh_height_bounds and refresh_terrain_horizon
self.quadtree_manager.update(&frustum_obj);
// …and the same frustum_obj goes to LabelManager::update
```

One `Frustum` per frame, shared by the quadtree and the label pass, so the two cannot
disagree about where the camera is. The frustum is built from the camera's final position
after collision, with the near plane derived from the same ground sample the drawn
projection uses.

### 2.1 The four side planes

`Camera::calculate_frustum_planes`. From the rows of `P_rz · V`, all in f64:

| index | expression | constraint |
|---|---|---|
| 0 Left | `r3 + r0` | `x_c ≥ −w_c` |
| 1 Right | `r3 − r0` | `x_c ≤ +w_c` |
| 2 Bottom | `r3 + r1` | `y_c ≥ −w_c` |
| 3 Top | `r3 − r1` | `y_c ≤ +w_c` |

Each is normalised to a unit inward normal. **No offsets are returned**, because in the
camera-relative frame there are none.

### 2.2 Why near and far are absent

The projection is **reverse-Z**, and the wgpu clip volume is `0 ≤ z ≤ w`, so the depth
constraints extract as `r3 − r2` (near) and `r2` (far) — *not* the OpenGL `r3 ± r2` pair
(§2.2 shows what those two would actually be). Both are dropped from tile culling:

* **Far is vacuous.** `zfar = ‖cam‖ + 10 Mm` and every ellipsoid point is within
  `‖cam‖ + a = ‖cam‖ + 6.378 Mm` of the eye. This is invariant **I-3**, and
  `test_far_plane_is_vacuous_for_the_globe` asserts the premise rather than the
  consequence — tighten `zfar` and the far plane must come back.
* **Near is vacuous whenever `znear < distance to the nearest drawable point`**, which holds
  in Free and Cockpit and in Tracking above 5 m. Below that it is not vacuous and it is
  harmful: evaluated in the absolute frame in f32, it rejects a z = 17 tile under a camera
  5 m up in Tracking mode on 0.058 m of true clearance computed from 6.378 Mm operands, and
  the globe under the camera disappears (§2.7).
* **Nothing behind the eye survives anyway.** The Left and Right half-space values sum to
  `−2·z_eye ≥ 0` (Lemma 2.1), so a point behind the eye fails one of them.

Dropping a plane can only *add* false positives, never false negatives, so this is safe by
construction. `render::debug_geometry::get_frustum_corners` still draws all six faces; it
works from the inverse view-projection, not from here.

### 2.3 The frustum corners and edge rays

`Camera::frustum_corners_relative` unprojects the eight NDC corners in f64 (reverse-Z, so
`ndc.z = 1` is near and `0` is far), subtracts the eye in f64, and downcasts.
`Frustum::with_corners` caches `max ‖corner‖₁` for the box-axis stage's rounding bound and
normalises the **far quad** into four unit edge rays in f64.

Those rays are the edges of the infinite pyramid the four planes describe. They are what
makes the separating-axis set completable (§13.2). A `Frustum` built without `with_corners`
has `corners: None`, simply skips those stages, and gets a looser but still sound answer:
over all nine harness sweeps, 5.91 % false positives instead of 2.05 %. Everything that
culls tiles, or measures what the renderer does, uses `with_corners`.

### 2.4 The horizon constants

`CullContext::with_pipeline` builds a `HorizonCamera` from the eye:

| field | value | why |
|---|---|---|
| `c` | `T(cam)` | the camera in scaled space |
| `c2` | `c·c` | `C²`; `h² = C²−1` is the tangent length squared |
| `rho` | `hypot(c.x, c.z)` | the global maximum of `A(λ)`, hoisted out of every node |
| `eps` | `(8.9e-16 + 1e-9)·max(C,1)` | f64 rounding plus 10⁻⁹ rad (6 mm of ground) of bounds slack, in the conservative direction |
| `active` | `c2 > 1.0` | eye strictly outside the surface |

`active` gates the **point** and **sphere** tests (labels, and terrain nodes), whose
cone algebra breaks down at or inside the surface. It does **not** gate the flat tile test;
4.1 below explains why that test needs no guard and why adding one would be a hole.

### 2.5 Terrain-only passes

On the `Heightfield` arm, two passes run over the tree the previous frame left, before
`update`:

- **`refresh_extras`** gives every node its height interval from data that has since
  arrived (a new node starts with its parent's interval widened by a measured margin), and
  re-fits the node's box, radius, patch sphere and sub-grid if it changed.
- **`refresh_terrain_horizon`** builds this frame's occlusion march from the nodes' ground
  floors, or leaves it inactive (altitude gates, relief pre-check).

Both are described in [terrain.md](terrain.md). The flat arm has neither.

---

## 3. Once per node, at construction

`QuadtreeNode::for_surface_with`. Everything here is computed once, when the node is created
by `subdivide()` — and for terrain recomputed only when its height interval changes
(`set_extra`) — never per frame.

### 3.1 The tile's rectangle

```rust
let bounds = tile_bounds(&id);   // tile_id.rs
```

`tile_bounds` is the **only** place a tile's four numbers are derived (invariant **I-5**).
`TileMesh::generate_on` calls the same function, so the rectangle the horizon test evaluates
and the rectangle the renderer fills are bit-identical.

Three details that matter:

* **All f64.** `web_mercator_y_to_lat_f64` is the definition every boundary derives from. In
  f32 the longitude bound `−180 + x·360/2^z` has an ulp of 1.53·10⁻⁵° at |lon| ≈ 150° —
  **1.7 m of ground** — enough to produce false negatives at z = 19–20 with the camera
  10–100 m up (§12.3).
* **Adjacent tiles share the boundary *value*,** not merely the formula:
  `tile_bounds(x).lon_max` and `tile_bounds(x+1).lon_min` are the same expression on the
  same operands, so the tiling is an exact partition.
  `test_tile_bounds_tile_the_sphere_without_seams` guards this.
* **The polar rows are stretched** to ±90°, matching `TileMesh`'s pole-cap rows.

`tile_bounds_unstretched` is the *un*-stretched variant and is used for exactly one thing —
the LOD radius (5 below). Never for culling.

### 3.2 The tangent frame

`tangent_frame`, given the patch centre's longitude and the ellipsoid normal `up`:

```rust
east  = (−sin λ_c, 0, −cos λ_c)     // analytic, from the centre longitude
north = up × east                    // automatically unit
```

`east` is built **analytically**, not as `Y × normal`. The cross-product form divides by
`cos φ' → 0` near a pole and needs a fallback exactly at one — and any fixed fallback such
as `+X` is not orthogonal to the normal there, so the reconstructed box can fail to contain
its own samples (§6.1). The analytic form gives `‖east‖ = 1` for every `λ_c` and
`east·up = 0` identically, with no branch.

`up` is `ellipsoid_normal`, the normalised gradient of the implicit form — exact whether or
not the point is on the surface.

### 3.3 The OBB

`fit_obb` samples an `(steps+1)²` grid over the patch in f64 — at both ends of the node's
altitude span for `Heightfield`, once at altitude 0 for `Ellipsoid` — projects each sample
onto `(east, north, up)`, and takes the min/max box. It returns:

* `surface_center` — the patch centre **on the ellipsoid**, f64, stored as
  `QuadtreeNode::center` and used for the LOD distance and as the mesh's origin;
* `radius` — the greatest sample distance from that centre, f32, the per-tile bounding
  radius handed to the renderer;
* `obb` — centre in f64, three half-axes in f32.

Grid density is `obb_grid_steps`: **8 for z < 5, 2 otherwise**. §5.3 proves a 3×3 grid
captures all three extents of a lon/lat patch *exactly* on the sphere; on the **ellipsoid**
the `up` axis is the surface normal rather than the radius, which perturbs that argument
most at coarse zoom, so the coarse levels sample denser. Sampling more can only grow the
box, never shrink it — the conservative direction.

The half-axes stay f32 deliberately: they are at most half a tile across and are never
differenced against an Earth-scale quantity. `half_axis_l1 = Σ‖h_j‖₁` is cached; it bounds
the circumsphere radius and feeds every rounding tolerance, and the L1 norm is an upper
bound on the L2 norm the derivation uses, so it stays conservative and costs no square
roots per frame.

### 3.4 The patch

`TilePatch::for_surface` reduces the rectangle to eight f64 trig constants — `sin`/`cos` of
`lon_min`, `lon_max`, `lat_min`, `lat_max`. 64 bytes per node, and **no transcendentals at
run time**. For `Heightfield` it also carries the node box's scaled-space bounding sphere
(`ScaledSphere::around_obb`: the smallest sphere about `T(centre)` holding the eight
vertices of `T(obb)`, which contains the patch exactly, relief included).

### 3.5 The sub-grid

`SubGrid::build` cuts the patch into `k × k` sub-patches, each with its own OBB (fitted at
`steps = 4`) and, for terrain, its own scaled sphere, plus the `k+1` longitude and `k+1`
latitude breakpoints, stored as `(sin, cos)` pairs shared along each row and column. That
is `32·(k+1)` bytes instead of `64·k²` — at `k = 8`, 288 B rather than 4 kB.

The breakpoints come from the same expressions `sub_bounds` uses, with latitude taken in
**Mercator y** so consecutive sub-patches share an edge exactly and the pole stretch is
reapplied to the sub-patch that actually touches the pole row. **The union of the `k²`
sub-patches is exactly the drawn patch**, and that is what makes `has_surviving_sub_patch`
sound (4.7 below).

The sub-boxes are fitted at a fixed `steps = 4`, **not** at `obb_grid_steps(z)`. That is
deliberate and bit-relevant: `steps` selects which points of the sub-rectangle are sampled,
so any other value changes every sub-box's extents and moves the false-positive figures
globally. It is a recalibration, not a tidy-up.

The `k+1` latitude breakpoints are stored **increasing in φ** — index 0 is the south edge —
the same polarity as `TilePatch` and as the longitude breakpoints, so every span handed to
`lat_span_max` is `[low, high]`. The row index `vi`, which follows Mercator y, runs the
other way; `sub_patch_is_occluded` converts once, in one named place. The breakpoint values
themselves must come from `i as f64 / k as f64`: re-deriving them from the far end
(`1.0 - i/k`) is not bit-identical at k = 12, 6, 3, and one ulp there flips borderline
sub-patches.

`k` comes from the calibrated table `SUB_BOXES_PER_AXIS`:

| z | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | ≥8 |
|---|---|---|---|---|---|---|---|---|---|
| `k` | 16 | 16 | 12 | 8 | 6 | 4 | 3 | 2 | 1 |

`k = 1` means no grid at all — `SubGrid::build` returns `None` and the node's own box takes
the exact test directly. The taper is a **measurement**, not a formula: what the sub-boxes
buy is the gap between a curved patch and the single box around it, worth a lot at
z = 1..6 and very little below; cost runs the other way, because the tree holds a handful
of coarse nodes and thousands of deep ones. The two curves have no common closed form. The
comment on `SUB_BOXES_PER_AXIS` carries the full table against all nine harness sweeps; the
chosen row is the knee, and false negatives are zero at every row of it.

---

## 4. The per-node test sequence

`QuadtreeNode::update`, in execution order. Every stage either **rejects on a proof** or
defers; nothing rejects on a heuristic.

### 4.0 The stages are a list, not a cascade

The sequence is not hard-coded into `update`. It is a `CullPipeline` — an ordered, `Copy`,
list of up to `MAX_STAGES = 4` `Stage` values — so a stage can be switched off without
touching the code that runs it:

```rust
pub enum StageVerdict { Cull, Keep, Undecided }
pub enum Stage { Horizon, NodeFrustum, SubPatchGrid, TerrainOcclusion }
pub const DEFAULT: CullPipeline =
    CullPipeline::of(&[Stage::Horizon, Stage::NodeFrustum, Stage::SubPatchGrid]);
pub const TERRAIN_DEFAULT: CullPipeline = CullPipeline::of(&[
    Stage::Horizon, Stage::TerrainOcclusion, Stage::NodeFrustum, Stage::SubPatchGrid]);
```

Five things about it are load-bearing.

**`Keep` skips everything after it.** `Cull` is a proof of invisibility and `Keep` is a
decision to stop asking — so a stage may only return `Keep` where no later stage could have
culled soundly. `Undecided` is always safe.

**The final rule keeps** (`CullPipeline::keeps`): a cascade that runs out of stages returns
`true`. Every stage is therefore a pure *subtraction* from the kept set, and dropping stages
can only keep **more**, never less. Soundness never rests on a stage being present — only
on each present stage being right. That is invariant I-7 as a checkable property, and
`test_stage_prefix_only_grows_the_kept_set` checks it over every ordered stage list and
every prefix of each.

**`Stage::NodeFrustum` absorbs the grid / no-grid dichotomy internally.** Which of the two
frustum forms runs is a property of the *node* (`sub_grid.is_some()`), not of the
configuration, so the stage list stays constant across every node in a frame.
`Stage::SubPatchGrid` returns `Undecided` for a grid-less node — unreachable behind
`NodeFrustum`, which always settles such a node, but the stage is total because a pipeline
may list it alone.

**Only the first two slots can hold a stage that returns `Undecided` and still matter.**
`NodeFrustum` and `SubPatchGrid` between them settle every node, so a stage appended after
them never runs. That is why `TerrainOcclusion` sits second, behind the cheaper and more
selective horizon test.

**A stage takes `&QuadtreeNode`, never `&mut`.** LOD, hysteresis and recursion live outside
the pipeline in `apply_lod` (5 below), which takes `&mut self`. That asymmetry makes it
*structurally impossible* for a visibility test to touch `children` — the I-7 hazard —
rather than merely discouraged.

Dispatch is an enum and a `match`. Not `dyn Trait`: inlining dies at the stage boundary and
`classify_box` earns its cost only with `delta` in registers. Not static generics: every
mode combination would get its own monomorphisation and the top of the call would still
need an enum. The loop is written over `0..MAX_STAGES` with an early `break` rather than
over `&stages[..len]`, which is worth 0.4 µs per update: a constant trip count unrolls into
slot-specialised copies, a run-time one does not.

The pipeline lives on `QuadtreeManager`, where it survives frames and changes only when a
mode does, and is copied by value into the per-frame `CullContext`. It is never stored per
node.

### 4.0.1 Summary

| # | test | site | cost | rejects | exact? |
|---|---|---|---|---|---|
| 1 | horizon, whole patch | `Ellipsoid::is_occluded` → `span_is_occluded` / `Heightfield::is_occluded` → `sphere_is_occluded` | ~25 / ~30 f64 flops | everything below the limb | **exact** on zero relief; exact for the sphere with relief |
| 1b | behind terrain (terrain only) | `TerrainHorizon::occludes` | a few lookups | nodes under a guaranteed ridge | sound bound |
| 2 | camera-relative offset | `Frustum::relative` | 3 f64 subs | — | — |
| 3a | circumsphere vs 4 planes | `classify_box` / `intersects_obb` | ~20 f32 flops | boxes far from the boundary, both ways | bound |
| 3b | 4 planes vs box | `classify_box` / `intersects_obb` | ~92 f32 flops | boxes outside one plane | bound |
| 3c | vertex witness | `intersects_obb` | ~96 f32 adds | — (it *accepts*) | exact acceptance |
| 3d | box-axis slabs | `slab::separated_on_box_axes` | 3 axes × 8 corners | **compiled out** (`BOX_AXES_ENABLED = false`) | — |
| 3e | edge × edge axes | `slab::separated_on_edge_cross_axes` | ~660 f64 flops | boxes past a frustum corner or edge | completes the set |
| 4 | the same, per sub-patch | `SubGrid::has_surviving_sub_patch` | `k²` × the above | patches whose every sub-patch is dead | — |

### 4.1 Stage 1 — horizon

```rust
// Stage::Horizon
if node.patch.is_occluded(&ctx.horizon) { StageVerdict::Cull } else { StageVerdict::Undecided }
```

`TilePatch::is_occluded` dispatches to the surface model.

**Flat globe.** Let `S = max over the patch of q·c`, computed exactly by
`TilePatch::max_dot`. Cull iff `S ≤ 1 − eps` (`span_is_occluded`).

`S` is found in closed form because `q·c` is linear in `q`: maximise over λ first
(`lon_span_max`), where `A(λ) = c.x·cos λ − c.z·sin λ = ρ·cos(λ − λ_cam)` has interior
maximum `ρ`; then over φ (`lat_span_max`), where `g(φ) = A*·cos φ + c.y·sin φ` has interior
maximum `√(A*² + c.y²)`. Each is one sinusoid on an interval of width ≤ π, so "is the
maximum interior?" is the single test *derivative ≥ 0 at the low end and ≤ 0 at the high
end*. No `atan2`, no circular-clamp special case for a tile that straddles the antimeridian
relative to the camera.

*Soundness.* For a point `q` **on** the unit sphere, occlusion from `c` collapses to the
single linear inequality `q·c ≤ 1` (Theorems 3.4, 3.5): the tangent-cone condition is
automatic for surface points. So `S ≤ 1` means every point of the drawn patch satisfies it,
hence every point is beyond the polar plane and occluded. The skirts lie strictly *inside*
the ellipsoid, so their segments to an exterior eye cross the sphere too; the ellipsoid is
convex and is the only occluder, so nothing can un-occlude them. ∎

*Why it is first.* It is the cheapest *and* the most selective test in the file: roughly
half the globe is below the limb at any time, and at the coarsest level the whole back
hemisphere falls to one comparison. On zero-relief terrain it has **FP = 0 as well as
FN = 0** — it is not a bound, it is the answer.

*The interior candidate is folded in with `max`, not substituted.* `√(A*² + c.y²)` is the
*global* maximum of `g`, so admitting it when the interior test is wrong can only
over-estimate `S`, i.e. keep the tile. Conservative either way (I-6).

*There is no `active` guard here, and adding one would be a bug.* The surface-point form
stays exact for `C² ≤ 1` as well: on the sphere, every other surface point is occluded and
`q·c < 1` for every `q ≠ c`; strictly inside, every surface point is occluded and
`q·c ≤ C < 1` (§13.3). With a guard, a camera at or below the surface would cull *nothing*
and schedule the entire globe. Without it, the footpoint tile is still kept
(`S ≥ C² = 1 > 1 − eps`), which is what the camera keeps one nanometre higher up too. No
cliff at zero altitude.

**Terrain.** The rectangle supremum is unsound once geometry leaves the surface: a summit
can satisfy `q·c ≤ 1` and still be visible over the limb. `Heightfield` runs the cone test
of Theorem 3.7 on the patch's scaled-space sphere (`sphere_is_occluded`), which does need
the `active` guard — its cone algebra breaks down for an eye at or inside the surface, so
there it culls nothing. The rectangle is not consulted at all; combining the two would only
readmit the unsound one.

### 4.1b Stage 1b — behind terrain (terrain only)

`Stage::TerrainOcclusion` culls a node whose box lies below the ridge the frame's occlusion
march guarantees in front of it (`TerrainHorizon::occludes`, queried with the node's own
box and rectangle). It answers `Undecided` whenever the march is inactive, which on the flat
arm is always (`CullContext::terrain` is `None`), and never `Keep`. The march, its
soundness argument and its gates are in [terrain.md](terrain.md#culling-behind-mountains).

### 4.2 Stage 2 — the camera-relative offset

```rust
let delta = ctx.frustum.relative(node.obb.center);
```

One f64 subtraction and a downcast. This is invariant **I-2** at its point of use; 1.4
above has the reasoning. It is computed in `Stage::NodeFrustum`'s grid arm; the grid-less arm
calls `intersects_obb`, which derives its own.

### 4.3 Stage 3a — the circumsphere

`classify_box` and `intersects_obb` both open by computing the four `s_p = n_p·Δ` and
comparing them against `half_axis_l1`, which bounds the box's circumsphere radius:

* `s_p + L1 < −ε` for any `p` → **Outside**, done in 20 flops;
* `s_p − L1 ≥ ε` for all four → **Inside**, likewise.

A sub-patch is usually either well inside the frustum or well outside it, so this settles
most boxes for a fifth of the cost of the full plane test. It matters because
`has_surviving_sub_patch`'s first pass runs it `k²` times per node.

### 4.4 Stage 3b — the four planes

```
reject iff   s_p + Σ_j |n_p·h_j|  <  −ε
```

with `ε = 8u·(‖Δ‖₁ + Σ‖h_j‖₁)`, `u = 2⁻²⁴` (`FRUSTUM_EPS_COEFF`, `Frustum::eps`). The eight `u`
account for the downcast of `Δ`, the half-axes, `2√3 u` for the plane normal's components
and `3u` for accumulating each three-term dot product.

*Soundness.* The left-hand side is `sup_{p∈B} n_p·(p − cam)`. If it is negative, the whole
box lies in the open half-space outside a frustum plane and cannot meet the frustum.
Rejecting only when it is below `−ε` puts the tolerance in the **keeping** direction (I-6).

`classify_box` returns the three-way `PlaneVerdict` — `Outside` / `Inside` / `Straddling`.
Both decisive verdicts are taken on a proof: a box is called `Inside` only when
`s_p − r_p ≥ ε` for all four.

### 4.5 Stage 3c — the vertex witness

A box vertex inside all four half-spaces is a witness that the box meets the frustum, and it
costs only sign flips: the vertex's plane distance is `s_p ± r_{p,0} ± r_{p,1} ± r_{p,2}` in
quantities stage 3b already computed. About 96 adds for all eight vertices.

This stage never rejects — it only *accepts*. Its purpose is to keep the expensive stage 3e
off the common path: only a box with no vertex inside and no separating plane, one truly
wedged against a frustum **edge or corner**, reaches it. That is roughly one box in a
hundred, which is what makes an exact test affordable per node *and* per sub-box.

### 4.6 Stage 3e — the edge-cross axes

`slab::separated_on_edge_cross_axes`. With near and far dropped (I-3), the volume the four
planes describe is the infinite pyramid `P = cone(r₀..r₃)` with apex at the eye. For two
convex polyhedra the separating-axis set is complete when it holds every face normal of each
plus every cross product of an edge of one with an edge of the other:

| family | count | where |
|---|---|---|
| face normals of `P` | 4 | stage 3b |
| face normals of the box | 3 | `separated_on_box_axes`, **`BOX_AXES_ENABLED = false`** |
| edge × edge | 4 × 3 = 12 | `separated_on_edge_cross_axes`, **on** (`EDGE_CROSS_ENABLED`) |

So `four planes ‖ box axes ‖ edge-cross axes` is not a bound at all — it is disjointness,
decided.

*The mechanics.* `P` is a cone with apex at the origin of this frame, so its support along an
axis `a` is `0` when every `a·r_m ≤ 0` and `+∞` otherwise. A rejection therefore needs two
facts, both taken with the rounding bound in the conservative direction: every ray strictly
on the far side (`hi ≤ −ray_eps` or `lo ≥ ray_eps`) **and** the whole box strictly on the near
side (`c − e > box_eps` or `c + e < −box_eps`). `a = r_i × h_j` is perpendicular to `r_i`, so
`a·r_i = 0` exactly and only the other three rays are tested; a near-zero `a` (ray parallel to
a box axis) separates nothing and falls out through the same comparisons.

The arithmetic is **f64 on f32-rounded inputs**, so `EDGE_EPS_COEFF` is still 8 f32 ulps even
though the accumulation is double.

*Why it is the stage that matters.* What it catches is §5.2's corner over-report, and that
does **not** decay with subdivision: a patch grazing a frustum corner has every sub-box
grazing it too. Measured over the nine sweeps with everything else in its final form, FN zero
throughout:

| | total FP | mean update |
|---|---|---|
| neither stage | 5.91 % | 5.9 µs |
| edge-cross only | **2.05 %** | **6.7 µs** |
| edge-cross + box axes | 2.04 % | 7.2 µs |

The box's own axes are worth 0.01 points of FP for 0.5 µs, because what they would catch the
vertex witness and the edge crosses already catch. They are kept but compiled out behind a
`const` — not a runtime flag, because an `env::var_os` probe in that loop costs more than the
test it guards (+4 µs on a 10 µs update). If tile volumes ever get much larger relative to the
frustum — a tighter `zfar`, or 3D tiles — the trade flips back.

*(The FP percentages in this table were measured with the harness's per-tile FP sampling at
`N = 4`; they are valid as an A/B against each other, not as absolute FP figures. See 8.2
below.)*

### 4.7 Stage 4 — the sub-grid

`SubGrid::has_surviving_sub_patch`, reached only when the node's own box is `Straddling`. The
name is the claim: a survivor is a sub-patch no proof of invisibility reached, not one shown to
be on screen.

*The dispatch.* With a grid present, the node's own box is asked *only* the four planes:

* `Outside` → cull, no grid work;
* `Inside` → keep outright. Every sub-patch is inside the frustum too, so the grid could only
  cull if every sub-patch were behind the limb — and stage 1 already tested exactly that, on the
  whole patch;
* `Straddling` → run the grid, where the exact stages are both cheaper (smaller boxes) and far
  more selective.

Without a grid (`k = 1`) the node's box takes `intersects_obb` itself.

*The sub-patch loop, two passes.* Pass 1 asks the limb test then `classify_box` for each
sub-patch: an `Inside` sub-patch settles the node immediately, and a tile with no straddling
sub-patch at all is settled too. Pass 2 — `intersects_obb`, including the edge-cross axes —
therefore runs only for a node that has sub-patches on the frustum boundary and none strictly
within it, the thin band where the answer was ever in doubt. The frustum stage is about 7× the
first, so this ordering is worth the duplicated loop.

The limb test comes first in both passes, and on the flat globe its λ half is **hoisted out of
the inner loop** (`SubGrid::column_a_star`): every sub-patch in column `ui` has the same λ
span, and that is the expensive half. For terrain each sub-patch has its own scaled sphere and
the hoisted value is unused.

*Soundness.* A sub-patch is discarded only when it is provably invisible on its own — its
rectangle entirely behind the limb (exact, or the sphere test for terrain) or its box
separated from the frustum (exact). Discarding *every* sub-patch proves the tile invisible,
because the sub-patches' union is the whole drawn patch: any drawable point lies in some
sub-patch, and that sub-patch is invisible. Keeping the tile as soon as one survives is the
conservative direction. ∎ The union property is not incidental — it is why `sub_bounds`
parameterises latitude in Mercator y and reapplies the pole stretch. Break that and the
soundness argument goes with it.

---

## 5. LOD, subdivision and hysteresis

`QuadtreeNode::apply_lod`, reached only by a node that survived culling. It is *not* a culling
stage, and the signature says so: stages take `&QuadtreeNode`, this takes `&mut self`.

```rust
self.visible = true;                                           // in update()
let dist = (self.center - ctx.frustum.eye).length() as f32;    // f64 subtract
let imagery_dist = self.unstretched_radius * lod_factor * fog_relaxation;
let subdivide_dist = if S::HAS_GEOMETRIC_ERROR {
    imagery_dist.max(S::geometric_error(&self.extra, &self.id) * ctx.terrain_lod_factor * …)
} else { imagery_dist };
let collapse_dist = subdivide_dist * 1.20;
let should_be_subdivided = if is_subdivided { dist < collapse_dist }
                           else             { dist < subdivide_dist };
```

**The distance is an f64 subtraction**, free here because the frame is camera-relative anyway.

**`lod_factor` is derived, not hand-picked**: `lod_factor_for(target_texel_ratio,
texture_size, viewport_height, fovy)`, recomputed every frame and calibrated so the default
configuration yields exactly `2.0`. `fog_relaxation` is `1 − fog(d)` at the node's box
distance. The terrain term exists only for `Heightfield` (a compile-time `const` removes it
from the flat instantiation). All three are explained in
[tiles-and-lod.md](tiles-and-lod.md#level-of-detail) and [terrain.md](terrain.md#terrain-lod-refining-on-shape).

**The hysteresis band is 20 %.** A node subdivides at `1.0×` but does not collapse until
`1.2×`, which prevents LOD oscillation when the camera straddles the threshold. A 5 % band is
about 50 m at z = 19 and produces visible appear/disappear flicker on high-detail tiles.

**`unstretched_radius` is measured on the *un*-stretched rectangle**: a polar row's true
ground extent, not its pull to ±90°. That makes polar caps subdivide later, which is an
accepted FP source, never an FN one (§8). It is never re-fitted for terrain: relief does not
change the ground a tile covers.

### 5.1 How visibility interacts with subdivision

This is the part that bites.

* **A cull deletes the subtree.** The single cull path in `update` sets `children = None`, and
  so does falling out of the LOD band in `apply_lod`.
* **Only leaves are emitted.** `collect_visible_tiles` returns early on `!visible` and pushes
  only nodes with no children.
* **Therefore a wrong cull at *any* ancestor removes an entire subtree**, not one tile. This is
  invariant **I-7**, and it is why every test in 4 above must be sound at every level, not
  merely "sound at leaf granularity". A test that is only correct for small patches is not
  admissible here.
* **Children are updated within the same call**, so the tree reaches full depth in a single
  `update`, and are reordered near to far first. More than one update matters only for the
  hysteresis band; the harness uses four and asserts the set is a fixed point
  (`test_update_iterations_reach_fixed_point`).
* **`get_renderable_tiles`** is a *separate* traversal that falls back to an ancestor's mesh
  when a child's is not built yet. It does not re-run any visibility test — it reads the
  `visible` flags this pass set.

---

## 6. The invariants, as operational rules

[culling-math.md](culling-math.md) §10 states I-1..I-7 as properties. Here they are as rules
for someone changing the code, with the guard that catches a violation.

| | rule | guarded by |
|---|---|---|
| **I-1** | **The flat horizon test is licensed only by zero relief.** `Ellipsoid` meshes place every non-skirt vertex at altitude exactly 0 and every skirt vertex inward; the collapse to `q·c ≤ 1` depends on it. Any geometry above the surface must go through a surface model whose horizon test is the cone test (§3.7), as `Heightfield` does. | `test_generated_mesh_has_no_positive_altitude` |
| **I-1′** | **Every mesh vertex lies inside the altitude interval its surface model declares**, skirts included, and a node's box is fitted over at least that interval. | `test_heightfield::generated_meshes_stay_within_their_declared_height_bounds`; the inheritance margins against the committed corpus (`d1_inherit_margin_covers_the_corpus`) |
| **I-2** | **Keep `OrientedBoundingBox::center` and `QuadtreeNode::center` in f64.** Any `p − cam` is subtracted in f64 and only the difference downcast. Never store a world position in f32 and subtract afterwards. | `test_camera_relative_plane_error_vs_tile_size`; `test_degenerate_obb_matches_contains_point` |
| **I-3** | **Do not tighten `zfar`** below `‖cam‖ + a` without reinstating the far plane `π_far = r2`. | `test_far_plane_is_vacuous_for_the_globe` (asserts the *premise*) |
| **I-4** | **Keep the horizon tests in f64** end to end. Their conditioning near the surface scales as `1/h`; in f32 the error in `S` is ~1.2·10⁻⁷, which at 3 m altitude is 0.23° of limb angle — 26 km of ground. The test is 25 flops. | `test_horizon_closed_form_matches_brute_force`; `test_limb_band_has_no_false_negatives`; `test_horizon_hot_structs_have_not_grown` (pins `TilePatch` at 64 B and `HorizonCamera` at 56 B, so an f32 demotion of either fails at once) |
| **I-5** | **`tile_bounds()` is the only source of a tile's rectangle.** Culling and the mesh builder see bit-identical numbers, pole stretch included. Do not re-derive bounds anywhere; only the f64 Mercator inverse exists. | `test_generated_mesh_stays_inside_the_culling_rectangle`; `test_tile_bounds_tile_the_sphere_without_seams` |
| **I-6** | **Every rejection needs a strict proof, with the tolerance widening the kept set.** Write `reject iff value < −ε`, never `value < +ε`. Same for an `Inside` verdict: it must also be proved, because it *skips* later rejections. For terrain: occluders are lower bounds, occludees upper bounds, footprints rounded outward. | `test_plane_offset_partitions`; `test_degenerate_obb_matches_contains_point` (the only test that resolves the ~4.8e-7 tolerance — the sweeps cannot, see 8.5 below) |
| **I-7** | **Every test must be sound at every level of the tree**, because a cull discards the subtree. Do not add a test that is only valid for small patches. Corollary: **the final rule of `CullPipeline::keeps` must stay `true`** — it is what makes omitting a stage safe. | `test_stage_prefix_only_grows_the_kept_set` (`kept(P) ⊆ kept(P')` for every pipeline and every prefix, over the 204 bench poses; it goes red when the final rule is flipped to `false`); the sweeps collectively; `test_zoom_cliff_probe` straddles the one place the conservatism changes character |

A practical corollary of I-6 worth stating on its own: **the tolerances are all
one-directional, and all of them can safely be made larger.** Widening a tolerance costs FP;
narrowing one risks FN. If you are unsure about a bound, err wide.

---

## 7. Tests the engine deliberately does not run

Four tests an implementation would naturally include are absent. Each absence either removes a
*rejection* (which cannot create a false negative) or is covered by an exact test.

### 7.1 A per-box back-face test

A test of the form

```
keep the sub-box iff normal·(cam − centre) > −max_extent
```

is the obvious way to reject back-facing parts of a tile. Its margin is short by a factor
`h = √(C²−1)`, which makes it **unsound above 2 642 km**: an implementation using it culled
visible tiles up to **8.07° inside the limb** at 12 000 km — 305 448 misses in the harness's
limb-band probe (§4.1).

Back-face culling is not merely fixed by leaving it out; it is **subsumed**: for a point on the
ellipsoid, `n̂(p)·(cam − p)` and `q·c − 1` are the same expression up to a strictly positive
factor (Theorem 3.4). What such a test would *also* do — test occlusion per sub-box rather than
per tile — is done exactly by `SubGrid`, on the sub-patch itself. The limb band measures
**0.0000°**. With relief, a base-normal back-face test becomes a false-negative source in its
own right (§4.0).

### 7.2 The spherical-cap horizon point

Cesium's horizon-culling point replaces a tile by a spherical cap through its corners. The
reduction is sound (§3.6) but loose, costing up to 23 % of the occluded tiles at coarse zoom.
The closed form (4.1 above) is cheaper and exact, and every additional cull it makes is a proof.

### 7.3 The near and far planes

Covered in 2.2 above. Far is provably vacuous (I-3). Near is vacuous in every regime but one,
and in that one it is the defect: it blanks the globe under a camera 5 m up in Tracking mode.
Removing a plane removes rejections, so it cannot create a false negative; the cost is FP, and
there is none, because nothing behind the eye survives Left + Right anyway.

### 7.4 A guard band on the horizon test

Applying the cone test for any camera with `h² > −0.1` would run it down to 327 km **below**
the surface, where `h² < 0` makes the squared-cone condition vacuously true and the answer is
"everything is occluded". The point and sphere tests are gated by `active: C² > 1` with no band;
the flat tile test needs no gate at all, because the surface-point form is exact in all three
regimes (4.1 above).

The tangent frame likewise has no degenerate-basis guard (3.2 above), and the sub-grid is not a
fixed 8×8 at every level but the measured taper of 3.5 above.

---

## 8. How to measure

The harness is `src/testing/culling/`. It is a measuring instrument: where the engine is wrong
it records the number, it never works around it.

### 8.1 Commands

```bash
# the gate — everything must be green
cargo test --release --lib culling:: -- --test-threads=1 --nocapture

# the update-latency + memory benchmark (measures, does not assert)
cargo test --release --lib culling::bench -- --ignored --test-threads=1 --nocapture

# one sweep on its own
cargo test --release --lib culling::test_globe_sweep::test_limb_band -- --nocapture
```

`--release` matters more than core count: this is dense f64 matrix and trig work. The gate
runs in about 40–50 s on a 128-core machine and about 2½ minutes in a debug build.
`--test-threads=1` gives each sweep the whole rayon pool in turn and keeps the printed timings
meaningful; `CESIUM_CULLING_THREADS` overrides the pool width for A/B timing runs.

Per-cell CSVs land in `$TMPDIR/cesium_culling_harness/`, one per sweep, plus a
`<sweep>_false_negatives.csv` with one row per miss when a sweep is dirty.

The culling harness measures the **flat** arm. The terrain arm's height-aware stages are held
to zero false negatives against the drawn mesh by `src/testing/terrain/`
(`test_terrain_visibility`, `test_terrain_occlusion`).

### 8.2 What FN and FP mean here

Both are defined in `sweep.rs`, and they are **not** symmetric.

* **False negative** — a sample point the oracle says is unambiguously visible which **no tile
  in the visible set covers**. This is a hole in the render. It is the defect that matters, and
  its threshold is **zero** in every sweep.
* **False positive** — a tile in the visible set into which **no visible sample point falls**.
  This is wasted work, not a visual bug. Bounding volumes are conservative by construction, so a
  non-zero FP is expected; it is budgeted, never asserted to zero.
* **Marginal** — a sample inside the oracle's numeric no-man's-land. Excluded from both tallies,
  counted and reported.
* **Degenerate cell** — a pose where the oracle finds *no* visible surface at all (camera at or
  below altitude 0). FP has no denominator there, so such cells are excluded from FP aggregates
  and counted separately. They remain fully subject to the FN check.

**FP is an upper bound, not an exact figure.** A tile is scored FP when no sample inside it is
visible, so adding sample points can only *discover* visible surface, never hide it — the count
is monotonically non-increasing in the density. Quote FP as "at most", and never compare two FP
numbers measured at different densities.

### 8.3 Sampling densities

| metric | grid | points | constant |
|---|---|---|---|
| FN, viewport | 257 × 145 NDC, unprojected in f64 through the camera's own inverse VP | ~37 000/cell | `NDC_GRID_COLS`/`ROWS` |
| FN, geodetic | global lat/lon at 0.5°, plus the ±85.0511° Mercator limits and the antimeridian | ~260 000/cell | `GEO_GRID_STEP_DEG` |
| FP, per tile | `(N+1)²` points inside each visible tile, `N = 32` | 1 089/tile | `TILE_SAMPLE_STEPS` |
| limb band | global lat/lon at 0.05° | ~26 M/cell | `LIMB_GRID_STEP_DEG` |

The two FN sources are deliberately independent code paths, so a bug in one cannot silently
disable the metric. The FP metric samples *inside each tile* rather than from a global grid,
which keeps it meaningful at z = 18–20 where a global grid would never land a point inside a
tile. `TILE_SAMPLE_STEPS` is 32 because FP halved on every doubling up to there — the signature
of a boundary artefact of sparse sampling, not over-culling — and the extra samples cost nothing
measurable; any FP percentage quoted at another density is a different measurement.

### 8.4 The frozen ground truth

`src/testing/culling/{oracle,cells,geodesy}.rs` are **bit-stable by convention**: the oracle,
the parameter cells and the tile addressing do not change, so numbers measured at different
times remain comparable. A threshold, a sampling density or the engine may change — never those
three.

`test_visible_set_digest_is_stable` pins it from the other side: an FNV-1a digest of the
visible set over the nadir ladder and the zoom-cliff cells, with the cell tables' sizes asserted
first. A green digest means "these cells select the same tiles"; a refactor that claims to
preserve behaviour and moves it has a bug. When a change is *meant* to alter the visible set, the
digests are re-derived from the test's own output and the reason recorded with the change.

When comparing FP across harness configurations, aggregate over the raw CSV columns
(`false_negatives`, `samples_visible`, `tiles`, `false_positive_tiles`), never over the printed
per-cell rates — a mean of rates is not the rate of the mean — and treat degenerate cells the
same way on both sides (`CellResult::is_degenerate`).

To A/B a single stage inside the engine, flip its `const` and rerun — `slab.rs`'s
`BOX_AXES_ENABLED` and `EDGE_CROSS_ENABLED` exist for exactly that, and `SUB_BOXES_PER_AXIS` is a
table for the same reason. Every row of the tables in those comments was produced this way.

### 8.5 Does the suite have teeth?

A suite that goes green is worth exactly as much as its ability to go red, so the instrument is
validated by perturbing the *engine* one line at a time (`src/testing/culling/mod.rs`, "Does it
still have teeth?"). A negated `y` in every plane normal trips 12 guards, a sign flip in the
horizon's `A(λ)` trips 6, swapping `a` and `b` in `T` trips 9, dropping the Top plane trips 5.

One genuine limit is worth knowing: flipping the frustum tolerance to the **unsafe** direction
(`< +ε`) trips only one test. The tolerance is ~4.8e-7 relative, so the decision boundary moves
by ~1e-6 of a tile, and the oracle's own marginal band is three orders wider by design. **A
tolerance regression has to be caught by construction and by reading the code, not by the
sweeps.** That is why I-6 is stated as a rule about how to write the comparison, not as a
number.
