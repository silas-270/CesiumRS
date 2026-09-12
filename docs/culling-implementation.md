# Culling implementation

*Implementation reference for globe visibility culling. This document describes
what the code **does**; `culling-math.md` derives why the mathematics is what it
is, and carries the proofs.*

*A bare section reference such as §3.4 points into `culling-math.md`; a reference
to this document always says "above" or "below".*

Read this if you are about to change anything in
`crates/cesium-engine/src/globe/quadtree/`, or if you need to follow the path a
tile takes from construction to the visible set without first reading a
derivation.

---

## 0. Orientation

### 0.1 The one-paragraph version

Once per frame the engine builds a **camera-relative frustum** — four inward
side-plane normals, the eye in f64, eight frustum corners — and a
**`HorizonCamera`**, the camera reduced to scaled space where the ellipsoid is
the unit sphere. It then walks a quadtree of tiles from four z=1 roots. Each node
answers one question, *can any drawable point of my patch be on screen?*, with a
sequence of tests ordered cheapest-and-most-selective first: the exact horizon
test, then the four planes, then a vertex witness, then the exact
separating-axis set, then the same tests again on a `k × k` grid of sub-patches.
A node that survives is marked visible and subdivided if the LOD rule says so; a
node that fails is marked invisible and its **entire subtree is deleted**. The
visible set is the set of surviving leaves.

### 0.2 File map

| file | what lives there |
|---|---|
| `globe/quadtree/tile_id.rs` | `TileId`, `TileBounds`, and `tile_bounds()` — the single source of a tile's rectangle (I-5) |
| `globe/quadtree/horizon.rs` | scaled space, `HorizonCamera`, `TilePatch`, the exact limb test |
| `globe/quadtree/bounding_volume.rs` | `OrientedBoundingBox`, `Frustum`, the four-plane and vertex-witness tests |
| `globe/quadtree/slab.rs` | the separating axes the four planes leave out |
| `globe/quadtree/quadtree.rs` | OBB fitting, `SubGrid`, `QuadtreeNode::update`, LOD |
| `camera/camera.rs` | `calculate_frustum_planes`, `frustum_corners_relative`, `global_transform_f64` |
| `render/wgpu_state.rs` | the per-frame driver (`update_logic`) |
| `globe/geometry.rs` | ellipsoid constants, `lon_lat_to_ecef_f64`, `TileMesh::generate` |
| `label/culling.rs` | the label path, which shares `HorizonCamera` and `Frustum` but not the tile collapse |

---

## 1. Frames and units

### 1.1 Megameters

Every length in the engine is in **megameters**: 1 unit = 1000 km. There is no
scale factor applied anywhere in the culling path — the constants are simply
small.

```
EARTH_RADIUS_A_F64 = 6.378137        globe/geometry.rs:5
EARTH_RADIUS_B_F64 = 6.3567523142    globe/geometry.rs:6
```

The f32 twins `EARTH_RADIUS_A_F32` / `EARTH_RADIUS_B_F32` (`geometry.rs:3-4`)
exist for the mesh and shader path. `B_F32` differs from the f64 constant by
8.6 cm; nothing in the culling path reads it, and nothing in the culling path
should start to (I-5).

A useful mental conversion while reading tolerances: **1e-9 Mm = 1 mm**, and
1 ulp of f32 at Earth scale (`2⁻²⁴ · 6.378`) is **0.38 m**. That number is the
whole reason the frame below exists.

### 1.2 ECEF: Y-up with negated Z

```rust
// globe/geometry.rs:50
x =  a·cos φ·cos λ
y =  b·sin φ
z = −a·cos φ·sin λ
```

It is **`y`** that carries the semi-minor axis, not `z`. This trips people, and
it is load-bearing in two places: the scaled-space map (§1.3 below) and the analytic
`east` basis vector (§3.2 below). `test_scaled_space_maps_surface_to_unit_sphere` pins
the convention so a future move to Z-up cannot silently invert it.

### 1.3 Scaled space

```rust
// globe/quadtree/horizon.rs:107
T(p) = (p.x / a, p.y / b, p.z / a)
```

`T` is linear, diagonal and positive, so "the segment eye→p passes through the
solid Earth" is invariant under it — and under `T` the WGS-84 ellipsoid becomes
the **unit sphere** while a Web-Mercator tile stays an exact spherical rectangle
`[λ₀,λ₁] × [φ₀,φ₁]` with the *same numeric* λ and φ. That is what makes the
horizon test a closed form rather than an iteration. Conversion happens exactly
twice: on the camera, once per frame (`HorizonCamera::new`, `horizon.rs:87`),
and on a label point, per label (`point_is_occluded`, `horizon.rs:131`). Tile
patches are never transformed — their λ/φ are already the right numbers.

### 1.4 The camera-relative frame

This is the central precision decision, and it is why `Frustum` owns the eye.

All four side planes pass through the eye, so in a frame whose origin is the eye
their offset is **identically zero** — written as a literal absence, not a
computed near-zero (`bounding_volume.rs:82-106`; there is no `d` field). Every
position entering a plane test goes through:

```rust
// bounding_volume.rs:151
pub fn relative(&self, p: DVec3) -> Vec3 {
    let d = p - self.eye;                                    // f64 subtraction
    Vec3::new(d.x as f32, d.y as f32, d.z as f32)             // then downcast
}
```

The subtraction is f64 and only the **small difference** is downcast. This turns
a distance-independent ~3 m f32 error floor into `1.5·10⁻⁷·‖Δ‖`, i.e. a constant
6·10⁻⁷ of a tile at every zoom (§2.5). The corollary is invariant **I-2**: the
OBB centre must be stored in f64 (`bounding_volume.rs:49`) or there is nothing
left to subtract precisely — an f32 centre already carries ~0.5 m of
construction error that no later arithmetic can undo.

`relative_f32` (`bounding_volume.rs:159`) exists for callers holding an f32 world
point (labels): it promotes *before* subtracting, so only the point's own
quantisation survives.

### 1.5 Where each conversion happens

| conversion | site |
|---|---|
| camera local → global, f64 | `camera.rs:147` `global_transform_f64` |
| view-projection → 4 side-plane normals, f64 | `camera.rs:543` |
| plane normals f64 → f32 | `bounding_volume.rs:111-118` |
| NDC → world frustum corners, f64, eye subtracted before downcast | `camera.rs:576` |
| far-quad corners f32 → unit edge rays, f64 | `bounding_volume.rs:139-143` |
| world → camera-relative, f64 subtract then downcast | `bounding_volume.rs:151` |
| world → scaled space (camera) | `horizon.rs:88` |
| patch λ/φ → eight trig constants, f64 | `horizon.rs:155` |
| OBB half-axes f64 → f32 | `quadtree.rs:187-195` |
| node centre → LOD distance, f64 subtract then downcast | `quadtree.rs:510` |

---

## 2. Once per frame

Driver: `render/wgpu_state.rs:445` `update_logic`.

```rust
let mut frustum = self.camera.calculate_frustum_planes(aspect_ratio);   // :453
// … extension may move the camera; planes are recomputed at :469 …
let frustum_obj = Frustum::new(frustum, camera_pos_dvec)                // :506
    .with_corners(self.camera.frustum_corners_relative(aspect_ratio));  // :507
self.quadtree_manager.update(&frustum_obj);                             // :512
```

One `Frustum` per frame, shared by the quadtree and the label pass (`:523`), so
the two cannot disagree about where the camera is.

### 2.1 The four side planes

`camera.rs:543`. From the rows of `P_rz · V`, all in f64:

| index | expression | constraint |
|---|---|---|
| 0 Left | `r3 + r0` | `x_c ≥ −w_c` |
| 1 Right | `r3 − r0` | `x_c ≤ +w_c` |
| 2 Bottom | `r3 + r1` | `y_c ≥ −w_c` |
| 3 Top | `r3 − r1` | `y_c ≤ +w_c` |

Each is normalised to a unit inward normal. **No offsets are returned**, because
in the camera-relative frame there are none.

### 2.2 Why near and far are absent

The projection is **reverse-Z** (`camera.rs:471-478`), and the wgpu clip volume
is `0 ≤ z ≤ w`, so the depth constraints extract as `r3 − r2` (near) and `r2`
(far) — *not* the OpenGL `r3 ± r2` pair the old code emitted. Both are dropped
from tile culling:

* **Far is vacuous.** `zfar = ‖cam‖ + 10 Mm` (`camera.rs:464`, `:491`) and every
  ellipsoid point is within `‖cam‖ + a = ‖cam‖ + 6.378 Mm` of the eye. This is
  invariant **I-3**, and `test_far_plane_is_vacuous_for_the_globe` asserts the
  premise rather than the consequence — tighten `zfar` and the far plane must
  come back.
* **Near is vacuous whenever `znear < altitude`**, which holds in Free and
  Cockpit always and in Tracking above 5 m. Below that it is not vacuous and it
  is catastrophic: it is what blanked the globe at 5 m in Tracking mode,
  rejecting a z=17 tile on 0.058 m of true clearance computed from 6.378 Mm
  operands in f32.
* **Nothing behind the eye survives anyway.** The Left and Right half-space
  values sum to `−2·z_eye ≥ 0` (Lemma 2.1), so a point behind the eye fails one
  of them.

Dropping a plane can only *add* false positives, never false negatives, so this
is safe by construction. `render::debug_geometry::get_frustum_corners` still
draws all six faces; it works from the inverse view-projection, not from here.

### 2.3 The frustum corners and edge rays

`camera.rs:576` unprojects the eight NDC corners in f64 (reverse-Z, so
`ndc.z = 1` is near and `0` is far), subtracts the eye in f64, and downcasts.
`Frustum::with_corners` (`bounding_volume.rs:130`) caches `max ‖corner‖₁` for the
slab stage's rounding bound and normalises the **far quad** into four unit edge
rays in f64 (`:139-143`).

Those rays are the edges of the infinite pyramid the four planes describe. They
are what makes the separating-axis set completable (§3.6). A `Frustum` built
without `with_corners` has `corners: None` and simply skips those stages, getting
a looser but still sound answer.

### 2.4 The horizon constants

`CullContext::new` (`quadtree.rs:392`) builds a `HorizonCamera`
(`horizon.rs:87`) from the eye:

| field | value | why |
|---|---|---|
| `c` | `T(cam)` | the camera in scaled space |
| `c2` | `c·c` | `C²`; `h² = C²−1` is the tangent length squared |
| `rho` | `hypot(c.x, c.z)` | the global maximum of `A(λ)`, hoisted out of every node |
| `eps` | `(8.9e-16 + 1e-9)·max(C,1)` | rounding + bounds slack, in the conservative direction |
| `active` | `c2 > 1.0` | eye strictly outside the surface |

`active` gates **only** `point_is_occluded` (the label path), not the tile path —
see §3.1 for why the tile test needs no such guard and why adding one would be a
hole rather than a safety belt.

---

## 3. Once per node, at construction

`QuadtreeNode::new` (`quadtree.rs:416`). Everything here is computed once, when
the node is created by `subdivide()`, and never recomputed — it depends only on
the `TileId`.

### 3.1 The tile's rectangle

```rust
let bounds = tile_bounds(&id);   // quadtree.rs:418 → tile_id.rs:86
```

`tile_bounds` is the **only** place a tile's four numbers are derived
(invariant **I-5**). `TileMesh::generate` calls the same function
(`geometry.rs:112`), so the rectangle the horizon test evaluates and the
rectangle the renderer fills are bit-identical.

Three details that matter:

* **All f64.** `web_mercator_y_to_lat_f64` (`tile_id.rs:29`) is the definition
  every boundary derives from. In f32 the longitude bound `−180 + x·360/2^z` has
  an ulp of 1.53·10⁻⁵° at |lon| ≈ 150° — **1.7 m of ground** — which was
  responsible for all 7 177 residual false negatives in the 100 000-cell fuzz
  sweep, every one at z = 19–20 with the camera 10–100 m up.
* **Adjacent tiles share the boundary *value*,** not merely the formula:
  `tile_bounds(x).lon_max` and `tile_bounds(x+1).lon_min` are the same expression
  on the same operands, so the tiling is an exact partition.
  `test_tile_bounds_tile_the_sphere_without_seams` guards this.
* **The polar rows are stretched** to ±90° (`tile_id.rs:95-100`), matching
  `TileMesh::generate`'s pole-cap rows.

`tile_bounds_unstretched` (`tile_id.rs:117`) is the *un*-stretched variant and is
used for exactly one thing — the LOD radius (§5 below). Never for culling.

### 3.2 The tangent frame

`tangent_frame` (`quadtree.rs:146`), given the patch centre's longitude and the
ellipsoid normal `up`:

```rust
east  = (−sin λ_c, 0, −cos λ_c)     // analytic, from the centre longitude
north = up × east                    // automatically unit
```

`east` is built **analytically**, not as `Y × normal`. The cross-product form
divided by `cos φ' → 0` near a pole and fell back to `+X` exactly at one — and
`+X` is not orthogonal to the normal there, so the reconstructed box failed to
contain its own samples. That branch never fired (no tile centre is exactly at
±90°), but it was a loaded gun. The analytic form gives `‖east‖ = 1` for every
`λ_c` and `east·up = 0` identically, with no branch.

`up` is `ellipsoid_normal` (`quadtree.rs:123`), the normalised gradient of the
implicit form — exact whether or not the point is on the surface.

### 3.3 The OBB

`fit_obb` (`quadtree.rs:158`) samples an `(steps+1)²` grid over the patch in f64,
projects each sample onto `(east, north, up)`, and takes the min/max box. It
returns:

* `surface_center` — the patch centre **on the ellipsoid**, f64, stored as
  `QuadtreeNode::center` and used for the LOD distance and for the renderer;
* `radius` — the greatest sample distance from that centre, f32, the renderer's
  per-tile bounding radius;
* `obb` — centre in f64, three half-axes in f32 (`quadtree.rs:187-195`).

Grid density is `obb_grid_steps` (`quadtree.rs:113`): **8 for z < 5, 2
otherwise**. §5.3 proves a 3×3 grid captures all three extents of a lon/lat patch
*exactly* on the sphere; on the **ellipsoid** the `up` axis is the surface normal
rather than the radius, which perturbs that argument most at coarse zoom, so the
coarse levels sample denser. Sampling more can only grow the box, never shrink
it — the conservative direction.

The half-axes stay f32 deliberately: they are at most half a tile across and are
never differenced against an Earth-scale quantity. `half_axis_l1 = Σ‖h_j‖₁` is
cached at construction (`bounding_volume.rs:59`); it bounds the circumsphere
radius and feeds every rounding tolerance, and the L1 norm is an upper bound on
the L2 norm the derivation uses, so it stays conservative and costs no square
roots per frame.

### 3.4 The patch trig constants

`TilePatch::new` (`horizon.rs:155`) reduces the rectangle to eight f64 trig
constants — `sin`/`cos` of `lon_min`, `lon_max`, `lat_min`, `lat_max`. 64 bytes
per node, and **no transcendentals at run time**.

### 3.5 The sub-grid

`SubGrid::build` (`quadtree.rs:250`) cuts the patch into `k × k` sub-patches,
each with its own OBB (fitted at `steps = 4`) plus the `k+1` longitude and `k+1`
latitude breakpoints, stored as `(sin, cos)` pairs shared along each row and
column. That is `32·(k+1)` bytes instead of `64·k²` — at `k = 8`, 288 B rather
than 4 kB.

The breakpoints come from the same expressions `sub_bounds` (`quadtree.rs:205`)
uses, with latitude taken in **Mercator y** so consecutive cells share an edge
exactly and the pole stretch is reapplied to the cell that actually touches the
pole row. **The union of the `k²` cells is exactly the drawn patch**, and that is
what makes `any_visible` sound (§4.7 below).

`k` comes from the calibrated table (`quadtree.rs:98`):

| z | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | ≥8 |
|---|---|---|---|---|---|---|---|---|---|
| `k` | 16 | 16 | 12 | 8 | 6 | 4 | 3 | 2 | 1 |

`k = 1` means no grid at all — `SubGrid::build` returns `None` and the node's own
box takes the exact test directly. The taper is a **measurement**, not a formula:
what the sub-boxes buy is the gap between a curved patch and the single box
around it, worth a lot at z = 1..6 and very little below; cost runs the other
way, because the tree holds a handful of coarse nodes and thousands of deep ones.
The two curves have no common closed form. The comment on
`SUB_BOXES_PER_AXIS` carries the full A/B table against all nine harness sweeps;
the chosen row is the knee, and FN is zero at every row of it.

---

## 4. The per-node test sequence

`QuadtreeNode::update` (`quadtree.rs:467`), in execution order. Every stage
either **rejects on a proof** or defers; nothing rejects on a heuristic.

### 4.0 Summary

| # | test | site | cost | rejects | exact? |
|---|---|---|---|---|---|
| 1 | horizon (whole patch) | `horizon.rs:198` | ~25 f64 flops | everything below the limb | **exact** on zero relief |
| 2 | camera-relative offset | `bounding_volume.rs:151` | 3 f64 subs | — | — |
| 3a | circumsphere vs 4 planes | `bounding_volume.rs:258-269` | ~20 f32 flops | boxes far from the boundary, both ways | bound |
| 3b | 4 planes vs box | `bounding_volume.rs:272-288` | ~92 f32 flops | boxes outside one plane | bound |
| 3c | vertex witness | `bounding_volume.rs:355-369` | ~96 f32 adds | — (it *accepts*) | exact acceptance |
| 3d | box-axis slabs | `slab.rs:79` | 3 axes × 8 corners | **compiled out** (`ENABLED = false`) | — |
| 3e | edge × edge axes | `slab.rs:171` | ~660 f64 flops | boxes past a frustum corner or edge | completes the set |
| 4 | the same, per sub-cell | `quadtree.rs:322` | `k²` × the above | patches whose every cell is dead | — |

### 4.1 Stage 1 — horizon

```rust
// quadtree.rs:469
if self.patch.is_occluded(&ctx.horizon) { self.visible = false; self.children = None; return; }
```

**Condition.** Let `S = max over the patch of q·c`, computed exactly by
`TilePatch::max_dot` (`horizon.rs:184`). Cull iff `S ≤ 1 − eps`
(`span_is_occluded`, `horizon.rs:283`).

`S` is found in closed form because `q·c` is linear in `q`: maximise over λ
first (`lon_span_max`, `horizon.rs:214`), where `A(λ) = c.x·cos λ − c.z·sin λ =
ρ·cos(λ − λ_cam)` has interior maximum `ρ`; then over φ (`lat_span_max`,
`horizon.rs:235`), where `g(φ) = A*·cos φ + c.y·sin φ` has interior maximum
`√(A*² + c.y²)`. Each is one sinusoid on an interval of width ≤ π, so "is the
maximum interior?" is the single test *derivative ≥ 0 at the low end and ≤ 0 at
the high end*. No `atan2`, no circular-clamp special case for a tile that
straddles the antimeridian relative to the camera.

**Soundness.** For a point `q` **on** the unit sphere, occlusion from `c`
collapses to the single linear inequality `q·c ≤ 1` (Theorems 3.4, 3.5): the
tangent-cone condition is automatic for surface points. So `S ≤ 1` means every
point of the drawn patch satisfies it, hence every point is beyond the polar
plane and occluded. The skirts lie strictly *inside* the ellipsoid, so their
segments to an exterior eye cross the sphere too; the ellipsoid is convex and is
the only occluder, so nothing can un-occlude them. ∎

**Why it is first.** It is the cheapest *and* the most selective test in the
file: roughly half the globe is below the limb at any time, and at the coarsest
level the whole back hemisphere falls to one comparison. On zero-relief terrain
it has **FP = 0 as well as FN = 0** — it is not a bound, it is the answer.

**The interior candidate is folded in with `max`, not substituted.**
`√(A*² + c.y²)` is the *global* maximum of `g`, so admitting it when the interior
test is wrong can only over-estimate `S`, i.e. keep the tile. Conservative either
way (I-6).

**There is no `active` guard here, and adding one would be a bug.** The
surface-point form stays exact for `C² ≤ 1` as well: on the sphere, every other
surface point is occluded and `q·c < 1` for every `q ≠ c`; strictly inside, every
surface point is occluded and `q·c ≤ C < 1`. With a guard, a camera at or below
the surface would cull *nothing* and schedule the entire globe. Without it, the
footpoint tile is still kept (`S ≥ C² = 1 > 1 − eps`), which is what the camera
keeps one nanometre higher up too. No cliff at zero altitude.

### 4.2 Stage 2 — the camera-relative offset

```rust
let delta = ctx.frustum.relative(self.obb.center);   // quadtree.rs:476
```

One f64 subtraction and a downcast. This is invariant **I-2** at its point of
use; §1.4 has the reasoning.

### 4.3 Stage 3a — the circumsphere

`classify_box` / `intersects_obb` both open by computing the four
`s_p = n_p·Δ` and comparing them against `half_axis_l1`, which bounds the box's
circumsphere radius:

* `s_p + L1 < −ε` for any `p` → **Outside**, done in 20 flops;
* `s_p − L1 ≥ ε` for all four → **Inside**, likewise.

A sub-cell is usually either well inside the frustum or well outside it, so this
settles most boxes for a fifth of the cost of the full plane test. It matters
because `SubGrid::any_visible`'s first pass runs it `k²` times per node.

### 4.4 Stage 3b — the four planes

```
reject iff   s_p + Σ_j |n_p·h_j|  <  −ε        (bounding_volume.rs:227, :277, :344)
```

with `ε = 8u·(‖Δ‖₁ + ‖h‖₁)`, `u = 2⁻²⁴` (`bounding_volume.rs:38, :166`). The
eight `u` account for the downcast of `Δ`, the half-axes, `2√3 u` for the plane
normal's components and `3u` for accumulating each three-term dot product.

**Soundness.** The left-hand side is `sup_{p∈B} n_p·(p − cam)`. If it is
negative, the whole box lies in the open half-space outside a frustum plane and
cannot meet the frustum. Rejecting only when it is below `−ε` puts the tolerance
in the **keeping** direction (I-6).

`classify_box` (`bounding_volume.rs:249`) returns the three-way verdict
`Outside` / `Inside` / `Straddling`. Both decisive verdicts are taken on a proof:
a box is called `Inside` only when `s_p − r_p ≥ ε` for all four.

### 4.5 Stage 3c — the vertex witness

`bounding_volume.rs:355-369`. A box vertex inside all four half-spaces is a
witness that the box meets the frustum, and it costs only sign flips: the
vertex's plane distance is `s_p ± r_{p,0} ± r_{p,1} ± r_{p,2}` in quantities
stage 3b already computed. ~96 adds for all eight vertices.

This stage never rejects — it only *accepts*. Its whole purpose is to keep the
expensive stage 3e off the common path: only a box with no vertex inside and no
separating plane, one truly wedged against a frustum **edge or corner**, reaches
it. That is roughly one box in a hundred, which is what makes an exact test
affordable per node *and* per sub-box.

### 4.6 Stage 3e — the edge-cross axes

`slab.rs:171`. With near and far dropped (I-3), the volume the four planes
describe is the infinite pyramid `P = cone(r₀..r₃)` with apex at the eye. For two
convex polyhedra the separating-axis set is complete when it holds every face
normal of each plus every cross product of an edge of one with an edge of the
other:

| family | count | where |
|---|---|---|
| face normals of `P` | 4 | stage 3b |
| face normals of the box | 3 | `slab.rs:79`, **`ENABLED = false`** |
| edge × edge | 4 × 3 = 12 | `slab.rs:171`, **on** |

So `separated_from_box ‖ separated_on_box_axes ‖ separated_on_edge_cross_axes` is
not a bound at all — it is disjointness, decided.

**The mechanics.** `P` is a cone with apex at the origin of this frame, so its
support along an axis `a` is `0` when every `a·r_m ≤ 0` and `+∞` otherwise. A
rejection therefore needs two facts, both taken with the rounding bound in the
conservative direction: every ray strictly on the far side (`hi ≤ −ray_eps` or
`lo ≥ ray_eps`) **and** the whole box strictly on the near side
(`c − e > box_eps` or `c + e < −box_eps`). `a = r_i × h_j` is perpendicular to
`r_i`, so `a·r_i = 0` exactly and only the other three rays are tested; a
near-zero `a` (ray parallel to a box axis) separates nothing and falls out
through the same comparisons.

The arithmetic is **f64 on f32-rounded inputs**, so `EDGE_EPS_COEFF` is still
8 f32 ulps (`slab.rs:124`) even though the accumulation is double.

**Why it is the stage that matters.** What it catches is §5.2's corner
over-report, and that does **not** decay with subdivision: a patch grazing a
frustum corner has every sub-box grazing it too. `camera_modes`' one stubborn
tile survived a 16×16 grid and dies here. Measured with everything else at its
final shape, FN zero throughout:

| | total FP | mean update |
|---|---|---|
| neither stage | 5.91 % — *worse than the pre-rework baseline* | 5.9 µs |
| edge-cross only | **2.05 %** | **6.7 µs** |
| edge-cross + box axes | 2.04 % | 7.2 µs |

The box's own axes are worth 0.01 points of FP for 0.5 µs, because what they used
to catch the vertex witness and the edge crosses already catch. They are kept but
compiled out behind a `const` — not a runtime flag, because an `env::var_os`
probe in that loop costs more than the test it guards (+4 µs on a 10 µs update).
If tile volumes ever get much larger relative to the frustum — a tighter `zfar`,
or 3D tiles — the trade flips back.

*(The FP percentages in this table were measured at the harness's old
`TILE_SAMPLE_STEPS = 4`; they are valid as an A/B against each other, not as an
absolute FP figure. See §8.2 below.)*

### 4.7 Stage 4 — the sub-grid

`SubGrid::any_visible` (`quadtree.rs:322`), reached only when the node's own box
is `Straddling` (`quadtree.rs:487-499`).

**The dispatch.** With a grid present, the node's own box is asked *only* the
four planes:

* `Outside` → cull, no grid work;
* `Inside` → keep outright. Every sub-cell is inside the frustum too, so the grid
  could only cull if every sub-patch were behind the limb — and stage 1 already
  tested exactly that, on the whole patch, exactly;
* `Straddling` → run the grid, where the exact stages are both cheaper (smaller
  boxes) and far more selective.

Without a grid (`k = 1`) the node's box takes `intersects_obb` itself.

**The cell loop, two passes.** Pass 1 asks the limb test then `classify_box` for
each cell: an `Inside` cell settles the node immediately, and a tile with no
straddling cell at all is settled too. Pass 2 — `intersects_obb`, including the
edge-cross axes — therefore runs only for a node that has cells on the frustum
boundary and none strictly within it, which is the thin band where the answer was
ever in doubt. The frustum stage is ~7× the first, so this ordering is worth the
duplicated loop.

The limb test comes first in both passes, and its λ half is **hoisted out of the
inner loop** (`lon_span_max`, `quadtree.rs:361`): every cell in column `ui` has
the same λ span, and that is the expensive half.

**Soundness.** A cell is discarded only when it is provably invisible on its own —
its spherical rectangle entirely behind the limb (exact) or its box separated
from the frustum (exact). Discarding *every* cell proves the tile invisible,
because the cells' union is the whole drawn patch: any drawable point lies in
some cell, and that cell is invisible. Keeping the tile as soon as one cell
survives is the conservative direction. ∎ The union property is not incidental —
it is why `sub_bounds` parameterises latitude in Mercator y and reapplies the
pole stretch. Break that and the soundness argument goes with it.

---

## 5. LOD, subdivision and hysteresis

`quadtree.rs:506-538`, reached only by a node that survived culling.

```rust
self.visible = true;                                        // :506
let dist = (self.center - ctx.frustum.eye).length() as f32; // :510  f64 subtract
let subdivide_dist = self.lod_radius * lod_factor;          // :517  lod_factor = 2.0
let collapse_dist  = subdivide_dist * 1.20;                 // :518
let should_be_subdivided = if is_subdivided { dist < collapse_dist }
                           else             { dist < subdivide_dist };
```

**The distance is an f64 subtraction**, free here because the frame is
camera-relative anyway.

**The hysteresis band is 20 %.** A node subdivides at `1.0×` but does not
collapse until `1.2×`, which prevents LOD oscillation when the camera straddles
the threshold. The previous 1.05× band was ~50 m at z = 19 and caused visible
APPEAR/DISAPPEAR flicker on high-detail tiles.

**`lod_radius` is measured on the *un*-stretched rectangle**
(`quadtree.rs:424-425`): a polar row's true ground extent, not its pull to ±90°.
That makes polar caps subdivide later, which is an FP source, not an FN one, and
is kept deliberately (§8.4).

### 5.1 How visibility interacts with subdivision

This is the part that bites.

* **A cull deletes the subtree.** Both cull paths set `children = None`
  (`quadtree.rs:471`, `:502`), and so does falling out of the LOD band (`:537`).
* **Only leaves are emitted.** `collect_visible_tiles` (`quadtree.rs:549`)
  returns early on `!visible` and pushes only nodes with no children.
* **Therefore a wrong cull at *any* ancestor removes an entire subtree**, not one
  tile. This is invariant **I-7**, and it is why every test in §4 must be sound at
  every level, not merely "sound at leaf granularity". A test that is only
  correct for small patches is not admissible here.
* **Children are updated within the same call** (`quadtree.rs:531-535`), so the
  tree reaches full depth in a single `update`. More than one update matters only
  for the hysteresis band; the harness uses four and asserts the set is a fixed
  point (`test_update_iterations_reach_fixed_point`).
* **`get_renderable_tiles`** (`quadtree.rs:562`) is a *separate* traversal that
  falls back to an ancestor's mesh when a child is not yet cached. It does not
  re-run any visibility test — it reads the `visible` flags this pass set.

---

## 6. The invariants, as operational rules

`culling-math.md` §10.3 states I-1..I-7 as properties. Here they are as rules for
someone changing the code, with the guard that catches a violation.

| | rule | guarded by |
|---|---|---|
| **I-1** | **Do not apply terrain relief to the mesh** without replacing `TilePatch::is_occluded` with the scaled-space cone test (§3.7). The collapse to `q·c ≤ 1` is licensed *only* because every non-skirt vertex sits at altitude exactly 0 and every skirt vertex is inward. | `test_generated_mesh_has_no_positive_altitude` (measures 0.143 m worst over 200 tiles, against a 5.5 m tolerance) |
| **I-2** | **Keep `OrientedBoundingBox::center` and `QuadtreeNode::center` in f64.** Any `p − cam` must be subtracted in f64 and only the difference downcast. Never store a world position in f32 and subtract afterwards. | `test_camera_relative_plane_error_vs_tile_size`; `test_degenerate_obb_matches_contains_point` |
| **I-3** | **Do not tighten `zfar`** below `‖cam‖ + a` without reinstating the far plane `π_far = r2`. | `test_far_plane_is_vacuous_for_the_globe` (asserts the *premise*) |
| **I-4** | **Keep the horizon test in f64** end to end. Its conditioning near the surface scales as `1/h`; in f32 the error in `S` is ~1.2·10⁻⁷, which at 3 m altitude is 0.23° of limb angle — 26 km of ground. It is 25 flops. | `test_horizon_closed_form_matches_brute_force`; `test_limb_band_has_no_false_negatives` |
| **I-5** | **`tile_bounds()` is the only source of a tile's rectangle.** Culling and `TileMesh::generate` must see bit-identical numbers, pole stretch included. Do not re-derive bounds anywhere; do not use the f32 `web_mercator_y_to_lat` for a boundary. | `test_generated_mesh_stays_inside_the_culling_rectangle`; `test_tile_bounds_tile_the_sphere_without_seams` |
| **I-6** | **Every rejection needs a strict proof, with the tolerance widening the kept set.** Write `reject iff value < −ε`, never `value < +ε`. Same for an `Inside` verdict: it must also be proved, because it *skips* later rejections. | `test_plane_offset_partitions`; `test_degenerate_obb_matches_contains_point` (the only test that resolves the ~4.8e-7 tolerance — the sweeps cannot, see §7.3) |
| **I-7** | **Every test must be sound at every level of the tree**, because a cull discards the subtree. Do not add a test that is only valid for small patches. | the sweeps collectively; `test_zoom_cliff_probe` straddles the one place the conservatism changes character |

A practical corollary of I-6 worth stating on its own: **the tolerances are all
one-directional, and all of them can safely be made larger.** Widening a
tolerance costs FP; narrowing one risks FN. If you are unsure about a bound, err
wide.

---

## 7. What was deleted, and why that is safe

Four things were removed rather than repaired. Each deletion either removes a
*rejection* (which cannot create a false negative) or replaces an unsound test
with an exact one.

### 7.1 The sub-OBB back-face heuristic

The dominant false-negative source, and not the one the harness originally
suspected. The old test was

```
normal·(cam − centre) > −max_extent
```

whose margin is short by a factor `h = √(C²−1)`, making it **unsound above
2 642 km** and culling visible tiles up to **8.07° inside the limb** at
12 000 km — 305 448 misses in the limb-band probe.

Safe to delete because back-face culling is not merely fixed, it is **subsumed**:
for a point on the ellipsoid, `n̂(p)·(cam − p)` and `q·c − 1` are the same
expression up to a strictly positive factor (Theorem 3.4). What the heuristic was
*also* doing — testing occlusion per sub-box rather than per tile — is not lost
either: `SubGrid` does it exactly, on the sub-patch itself. The limb band now
measures **0.0000°**.

### 7.2 `compute_horizon_culling_point` and the spherical-cap reduction

This was **not** the limb bug — the four-corner reduction is provably sound
(§3.6) — but it is loose, costing up to 23 % of the occluded tiles at coarse
zoom. The closed form replaces it at lower cost and with zero FP for the stage.
Deleting a *loose* test in favour of an *exact* one can only cull more, never
less, and every additional cull is a proof.

### 7.3 The near and far planes

Covered in §2.2 above. Far is provably vacuous (I-3). Near was vacuous in every regime
but one, and in that one regime it was the defect: it blanked the globe at 5 m in
Tracking mode. Removing a plane removes rejections, so it cannot create a false
negative; the cost is FP, and there is none, because nothing behind the eye
survives Left + Right anyway.

### 7.4 The `vh_mag_sq > −0.1` guard band

The old horizon code allowed the test to run down to 327 km **below** the
surface, where `h² < 0` makes the squared-cone condition vacuously true and the
answer is "everything is occluded" — garbage. It is replaced by `active: C² > 1`
(`horizon.rs:83`) on the label path, and by nothing at all on the tile path,
where the surface-point form is exact in all three regimes (§4.1 above). The
replacement culls *less* below the surface, not more.

Also gone: the dead `east.length_squared() < 0.1` basis guard (§3.2 above), and the
flat 8×8 sub-OBB grid for 5 ≤ z ≤ 16 — a grid is kept, but tapered by
measurement (§3.5 above).

---

## 8. How to measure

The harness is `src/testing/culling/`. It is a measuring instrument: where the
engine is wrong it records the number, it never works around it.

### 8.1 Commands

```bash
# the gate — everything must be green
cargo test --release --lib culling:: -- --test-threads=1 --nocapture

# the update-latency + memory benchmark (measures, does not assert)
cargo test --release --lib culling::bench -- --ignored --test-threads=1 --nocapture

# one sweep on its own
cargo test --release --lib culling::test_globe_sweep::test_limb_band -- --nocapture
```

`--release` matters more than core count: this is dense f64 matrix and trig work.
The gate runs ~50 s on a 128-core box and ~2 min 30 s in a debug build.
`--test-threads=1` gives each sweep the whole rayon pool in turn and keeps the
printed timings meaningful; `CESIUM_CULLING_THREADS` overrides the pool width for
A/B timing runs.

Per-cell CSVs land in `$TMPDIR/cesium_culling_harness/`, one per sweep, plus a
`<sweep>_false_negatives.csv` with one row per miss when a sweep is dirty.

### 8.2 What FN and FP mean here

Both are defined in `sweep.rs:6-41`, and they are **not** symmetric.

* **False negative** — a sample point the oracle says is unambiguously visible
  which **no tile in the visible set covers**. This is a hole in the render. It
  is the defect that matters, and its threshold is **zero** in every sweep.
* **False positive** — a tile in the visible set into which **no visible sample
  point falls**. This is wasted work, not a visual bug. Bounding volumes are
  conservative by construction, so a non-zero FP is expected; it is budgeted,
  never asserted to zero.
* **Marginal** — a sample inside the oracle's numeric no-man's-land. Excluded
  from both tallies, counted and reported.
* **Degenerate cell** — a pose where the oracle finds *no* visible surface at all
  (camera at or below altitude 0). FP has no denominator there, so such cells are
  excluded from FP aggregates and counted separately. They remain fully subject
  to the FN check.

**FP is an upper bound, not an exact figure.** A tile is scored FP when no sample
inside it is visible, so adding sample points can only *discover* visible
surface, never hide it — the count is monotonically non-increasing in the
density. Quote FP as "at most", and never compare two FP numbers measured at
different densities.

### 8.3 Sampling densities

| metric | grid | points | site |
|---|---|---|---|
| FN, viewport | 257 × 145 NDC, unprojected in f64 through the camera's own inverse VP | ~37 000/cell | `sweep.rs:114` |
| FN, geodetic | global lat/lon at 0.5°, plus the ±85.0511° Mercator limits and the antimeridian | ~260 000/cell | `sweep.rs:121` |
| FP, per tile | `(N+1)²` points inside each visible tile, `N = 32` | 1089/tile | `sweep.rs:167` |
| limb band | global lat/lon at 0.05° | ~26 M/cell | `sweep.rs:443` |

The two FN sources are deliberately independent code paths, so a bug in one
cannot silently disable the metric. The FP metric samples *inside each tile*
rather than from a global grid, which is what keeps it meaningful at z = 18–20
where a global grid would never land a point inside a tile.

`TILE_SAMPLE_STEPS` was raised from 4 to 32 after the convergence in its doc
comment showed FP halving on every doubling — the signature of a boundary
artifact, not over-culling. Anything quoted as an FP percentage from before that
change is a different measurement.

### 8.4 The old-vs-new comparison

The comparison is valid only because the ground truth is frozen.
`src/testing/culling/{oracle,cells,geodesy}.rs` are **bit-stable by
convention**: the oracle, the parameter cells and the tile addressing must not
change, or old numbers stop being comparable. Change a threshold, a sampling
density or the engine — never those three.

The pre-rework baseline is commit **`da6573a`** — the commit that introduced the
harness, one before the rework began. Those three files are byte-identical
between `da6573a` and today (`git diff da6573a HEAD -- src/testing/culling/`
touches everything *except* them), which is what makes the two runs comparable at
all.

You cannot cherry-pick the old engine into the current tree: the old
`quadtree.rs` calls a six-plane `calculate_frustum_planes` and knows nothing of
`horizon.rs` or `slab.rs`, so it will not build here. Run each side in its own
worktree, with its own harness:

```bash
git worktree add /tmp/cull-base da6573a
cd /tmp/cull-base && cargo test --release --lib culling:: -- --test-threads=1 --nocapture
cp -r "${TMPDIR:-/tmp}/cesium_culling_harness" /tmp/baseline-csv

cd -                            # back to the working tree
cargo test --release --lib culling:: -- --test-threads=1 --nocapture
# CSVs are now in $TMPDIR/cesium_culling_harness/
```

Then diff the per-sweep CSVs by the `false_negatives`, `samples_visible`,
`tiles` and `false_positive_tiles` columns. Two rules:

* **Aggregate over the raw columns, never over the printed rates.** `fp_rate` is
  per-cell, and a mean of rates is not the rate of the mean. Sum the counts and
  divide once. Exclude cells with `samples_visible == 0` from any FP aggregate
  (see `CellResult::is_degenerate`) — or do not, but do the same on both sides.
* **FN is directly comparable; FP is not.** The FN grids are identical across the
  two commits (257 × 145, 0.5°, 0.05°, `UPDATE_ITERATIONS = 4`), but the baseline
  harness has `TILE_SAMPLE_STEPS = 4` against today's 32, and FP is a function of
  that density (§8.2 above). To compare FP, set the same `N` on both sides first —
  that is one edit in `sweep.rs`, and it is a harness edit, not an engine one.

At `da6573a` the old defect probes are `#[ignore]`d, so the baseline run reports
the five known defects rather than failing on them; use `--ignored` there to get
the limb band and the fuzz sweep.

To A/B a single stage inside the engine, flip its `const` and rerun — `slab.rs`'s
`ENABLED` and `EDGE_CROSS_ENABLED` exist for exactly that, and
`SUB_BOXES_PER_AXIS` is a table for the same reason. Every row of the tables in
those comments was produced this way.

### 8.5 Does the suite still have teeth?

A suite that goes green after a rewrite is worth exactly as much as its ability
to go red, so the instrument was re-validated by perturbing the *engine* one line
at a time (`src/testing/culling/mod.rs:82-110`). A negated `y` in every plane normal trips 12 guards,
a sign flip in the horizon's `A(λ)` trips 6, swapping `a` and `b` in `T` trips 9,
dropping the Top plane trips 5.

One genuine limit is worth knowing: flipping the frustum tolerance to the
**unsafe** direction (`< +ε`) trips only one test. The tolerance is ~4.8e-7
relative, so the decision boundary moves by ~1e-6 of a tile, and the oracle's own
marginal band is three orders wider by design. **A tolerance regression has to be
caught by construction and by reading the code, not by the sweeps.** That is why
I-6 is stated as a rule about how to write the comparison, not as a number.
