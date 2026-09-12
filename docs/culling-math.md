# Culling mathematics

*Derivation document for the visibility-culling rework. No production code is
changed by this document; it is the specification an implementer works from.*

Every claim below is either proved here or marked explicitly as an estimate.
Numbers quoted as "measured" come either from the harness in
`src/testing/culling/` (commit `da6573a`) or from throwaway numerical
experiments run while writing this; the latter are reproduced as formulas so
they can be re-derived rather than trusted.

---

## 0. Executive summary — what held, what did not

| Hypothesis (from the brief) | Verdict |
|---|---|
| Plane index 5 is really near, index 4 is degenerate, far is never enforced | **Confirmed, exactly.** Index 4's plane sits `fn/(f−2n)` *behind* the eye; predicted clearance 2.8557 Mm, measured 2.8557 Mm. |
| Culling in a camera-relative frame is the central precision fix | **Confirmed.** Side planes get `d = 0` identically; f32 error drops from a distance-independent ~3 m floor to `1.5·10⁻⁷·D`, i.e. a *constant* 6·10⁻⁷ of a tile at every zoom. |
| The 8.07° limb false negatives come from `compute_horizon_culling_point` (4-corner reduction) | **REFUTED.** The 4-corner reduction is provably sound (§3.4). The 8.07° band comes from the **sub-OBB back-face heuristic** `normal·cam_to_center > −max_extent` at `quadtree.rs:333`. Its margin is short by a factor `h = √(‖c‖²−1)`, making it unsound above **2 642 km** altitude. Simulating that one line reproduces **8.0760°** at 12 000 km against the harness's measured **8.0715°**. |
| Back-face culling is subsumed by a correct horizon test | **Confirmed and proved** (§4): for a point on the ellipsoid, `n̂(p)·(cam−p)` and `q·c − 1` are the *same expression* up to a positive factor. |
| Tracking-at-5 m returning 0 tiles is a znear/zfar conditioning failure | **Confirmed and localised.** The real near plane rejects the z=17 ancestor (signed distance −0.48 m against a projected radius 0.30 m) purely from f32 cancellation; killing z17 kills z18–20 and the branch returns nothing. |
| The plane-only SAT over-reports | Confirmed; measured 10.1 % here on the reference frustum. A 3-axis box-slab test cuts it to **2.1 %**. |
| The `east = Y × n` basis guard is dead | **Confirmed for near-polar, refuted for exactly-polar** — and the exactly-polar fallback is the *unsound* branch, not the dead one (§6). |
| 8×8 sub-OBBs for z ≤ 16 | **Refuted as a criterion.** Derived requirement: subdivision is needed only for **z ≤ 4**, with k(z) = 8, 4, 2, 1. For 5 ≤ z ≤ 16 the 64 boxes per node are pure waste. |

Headline consequences:

* **Delete** `compute_horizon_culling_point`, `horizon_culling_point`, the
  sub-OBB back-face test, the far plane, the near plane, and `tight_obbs` for
  z ≥ 5.
* **Replace** the horizon test with a 25-flop closed form that is *exact*
  (zero FN **and** zero FP for the horizon stage on zero-relief terrain).
* **Translate** the whole frustum stage into the camera-relative frame.
* Net per-tile cost for a z ≤ 16 node drops by roughly **40×** (the 64-box loop
  disappears), and per-node memory by roughly **30×**.

---

## 1. Notation and frames

### 1.1 Units, ellipsoid, ECEF convention

All lengths are **megameters** (1 Mm = 1000 km).

```
a = 6.378137          (semi-major, Mm)
b = 6.3567523142      (semi-minor, Mm)
```

ECEF is **Y-up with negated Z**:

```
p(λ, φ) = ( a·cos φ·cos λ ,  b·sin φ ,  −a·cos φ·sin λ )
```

with λ longitude and φ geodetic latitude. Note `b` appears as `6.356_752_4_f32`
in `bounding_volume.rs` — an f32 literal that differs from the f64 constant by
8.6·10⁻⁸ Mm = 8.6 cm. Harmless, but see invariant **I-5**.

The outward ellipsoid normal at any `p` (on or off the surface) is the
normalised gradient of the implicit form:

```
g(p) = ( p_x/a² , p_y/b² , p_z/a² )        n̂(p) = g(p)/‖g(p)‖            (1.1)
```

### 1.2 Scaled space

Define the linear map

```
T(p) = ( p_x/a , p_y/b , p_z/a )                                          (1.2)
```

`T` is invertible and diagonal-positive. Under `T` the WGS-84 ellipsoid becomes
the **unit sphere**. Write

```
q = T(p)      (a surface patch point)
c = T(cam)    (the camera)
C = ‖c‖ ,   C² = c·c ,   h² = C² − 1                                      (1.3)
```

`h` is the tangent length from `c` to the unit sphere in scaled space.

Three facts about `T` that the whole of §3 rests on:

* **T maps lines to lines and preserves betweenness** (it is affine). Therefore
  "the segment cam→p meets the solid Earth" is *invariant* under `T`. Occlusion
  is a purely incidence-theoretic property, so working in scaled space is exact,
  not an approximation. `T` does **not** preserve angles or lengths; no step
  below uses either.
* **The tile's lon/lat rectangle stays a lon/lat rectangle.** Substituting (1.2)
  into the ECEF parametrisation gives
  `q(λ,φ) = (cos φ cos λ, sin φ, −cos φ sin λ)` — the unit-sphere point with the
  *same numeric* λ and φ. So a Web-Mercator tile maps to an exact spherical
  rectangle `[λ₀,λ₁] × [φ₀,φ₁]`. This is the single most useful fact in the
  document.
* **The third axis scales by `a`, not `b`.** Because the engine's ECEF is Y-up,
  it is the *Y* component that carries `b`. `transform_to_scaled_space`
  already does this correctly.

### 1.3 The camera-relative frame

For a world point `p` define

```
Δ = p − cam                                                               (1.4)
```

computed in **f64** and only then downcast. Everything in the frustum stage is
expressed in `Δ`. §2.3 proves why.

### 1.4 What the renderer actually draws

`TileMesh::generate` (`globe/geometry.rs:90`) places every non-skirt vertex at
**altitude exactly 0** on the ellipsoid, and every skirt vertex at altitude
`−0.5/2^z` (radially *inward*). Terrain relief is parsed
(`globe/terrain_parser.rs`) but is not applied to geometry. Therefore:

> **Fact R.** Today the drawable surface of a tile is exactly the ellipsoid
> patch `[λ₀,λ₁] × [φ₀,φ₁]`, plus a skirt that lies strictly *inside* the
> ellipsoid.

This licenses the strongest (and cheapest) form of the horizon test. §3.7 gives
the general form to switch to the moment relief is turned on. This is
invariant **I-1**.

### 1.5 Tile bounds

For `TileId{z,x,y}` with `n = 2^z`:

```
λ₀ = −180 + 360x/n        λ₁ = −180 + 360(x+1)/n
φ₁ = mercator_lat(y)      φ₀ = mercator_lat(y+1)
if y == 0     : φ₁ = +90     (pole stretch)
if y == n − 1 : φ₀ = −90     (pole stretch)
```

`mercator_lat(y) = atan(sinh(π(1 − 2y/n)))`. The pole stretch is applied
identically by `compute_bounding_volume` (`quadtree.rs:139-144`),
`compute_geographic_corners` (`:260-265`) and `TileMesh::generate`
(`geometry.rs:103-108`), so the culling patch and the drawn patch agree. That
agreement is invariant **I-5**.

Note that the mesh interpolates *in Mercator y*, not in latitude, and
`mercator_lat` is monotone; so every mesh vertex latitude lies in `[φ₀, φ₁]`.
Longitude is interpolated linearly inside `[λ₀, λ₁]`. Hence
**mesh ⊆ rectangle**, which is what soundness needs.

### 1.6 Soundness vocabulary

A test is **sound** (FN-free) if it answers "cull" only when the tile is
provably invisible. Every section states the soundness argument in one line as
"this rejects V only if V ∩ visible-set = ∅".

---

## 2. Clip space, plane extraction, and precision

### 2.1 The projection, derived

`Camera::get_projection_matrix_f64` builds `P_rz = R · P` where
`P = glam::DMat4::perspective_rh(fovy, aspect, n, f)` and `R` is the reverse-Z
remap at `camera.rs:501-503`.

glam's `perspective_rh` is the **wgpu** convention (`z_ndc ∈ [0,1]`). Written as
rows, with `r = f/(n−f) < 0`:

```
P.row0 = ( w, 0, 0,   0 )      w = h/aspect
P.row1 = ( 0, h, 0,   0 )      h = cot(fovy/2)
P.row2 = ( 0, 0, r, r·n )
P.row3 = ( 0, 0, −1,  0 )
```

Check: `z_eye = −n → z_c = 0, w_c = n → z_ndc = 0`; `z_eye = −f → z_ndc = 1`.
So `P` alone is near→0, far→1.

`R` is given in `from_cols_array`, i.e. **column-major**. Its rows are

```
R.row0 = (1,0, 0,0)   R.row1 = (0,1, 0,0)   R.row2 = (0,0,−1,1)   R.row3 = (0,0,0,1)
```

Hence

```
P_rz.row0 = P.row0
P_rz.row1 = P.row1
P_rz.row2 = P.row3 − P.row2 = ( 0, 0, −r−1, −r·n )
P_rz.row3 = P.row3         = ( 0, 0, −1,     0   )
```

Check: `z_eye = −n → z_c = n, w_c = n → z_ndc = 1`; `z_eye = −f → z_c = 0 →
z_ndc = 0`. **Reverse-Z confirmed: near ↦ 1, far ↦ 0.**

### 2.2 The six planes, correctly

Let `M = P_rz · V` with rows `r0, r1, r2, r3`, and let
`x_c = r0·p̃`, … , `w_c = r3·p̃` for homogeneous `p̃ = (p,1)`.

The wgpu clip volume is

```
−w_c ≤ x_c ≤ w_c ,   −w_c ≤ y_c ≤ w_c ,   0 ≤ z_c ≤ w_c ,   w_c > 0
```

Each inequality rearranged to `π·p̃ ≥ 0` gives an **inward-pointing** plane
4-vector:

| plane | 4-vector | comes from |
|---|---|---|
| Left   | `r3 + r0` | `x_c ≥ −w_c` |
| Right  | `r3 − r0` | `x_c ≤ +w_c` |
| Bottom | `r3 + r1` | `y_c ≥ −w_c` |
| Top    | `r3 − r1` | `y_c ≤ +w_c` |
| **Near**  | `r3 − r2` | `z_ndc ≤ 1` (reverse-Z: 1 **is** near) |
| **Far**   | `r2`      | `z_ndc ≥ 0` (reverse-Z: 0 **is** far)  |

`calculate_frustum_planes` (`camera.rs:519-526`) currently emits
`[r3+r0, r3−r0, r3+r1, r3−r1, r3+r2, r3−r2]` labelled `[L,R,B,T,N,F]`. So:

* index 5 (`r3 − r2`), labelled "Far", **is the near plane**;
* index 4 (`r3 + r2`), labelled "Near", **is not a frustum plane at all**;
* the real far constraint (`r2`) **is missing entirely**.

**What index 4 actually is.** In eye space
`r3 + r2 = (0,0,−r−2,−r·n)`, so the constraint is
`−(r+2)z_e − r n ≥ 0`, whose boundary is

```
z_e* = − f·n / (2n − f)                                                   (2.1)
```

For the harness's reference camera (`n = 1.360001`, `f = 29.974864`),
(2.1) gives `z_e* = +1.4957` — a plane **1.4957 Mm behind the eye**, facing
forward. The near quad sits at `z_e = −1.360`, so the frustum hull clears it by
`1.4957 + 1.360 = 2.8557 Mm`. The harness measures **2.86 Mm**. Derivation and
measurement agree to five digits.

As `f ≫ n`, (2.1) → `z_e* ≈ +n`: plane 4 is always "a plane `znear` behind the
eye". It rejects only things more than `znear` behind the camera.

### 2.3 Normalisation — where it is needed and where it is not

Write a plane as `(n, d)` with `n` not necessarily unit. Two tests appear later:

* **Sign test / OBB support test.**
  `cull ⟺ n·m + d < −Σ_j |n·h_j|`.
  Scaling `(n,d) → (λn, λd)`, `λ > 0`, multiplies both sides by `λ`.
  **Scale-invariant. Normalisation is not required.**
* **Sphere test** (`label/culling.rs::intersects_sphere`), any test compared
  against an absolute length, and the rounding tolerance of §2.5, which is
  expressed in metres. **Normalisation is required.**

Recommendation: normalise once per frame anyway. It is 4 reciprocal square roots
per frame, it makes the error analysis expressible in metres, and it keeps the
label path correct. But do *not* let an implementer believe the OBB test needs
it.

### 2.4 Precision: the absolute frame is the disease

The current path evaluates, in f32,

```
s = n·m + d ,      d = −n·cam
```

with `‖m‖ ≈ ‖cam‖ ≈ 6.378 Mm. The answer `s` is the distance from a tile to a
plane through the camera — at high zoom, metres. Subtracting two f32 numbers of
magnitude 6.378 Mm (`ulp(6.378) = 2²·2⁻²³ = 4.768·10⁻⁷ Mm = 0.477 m`) to obtain
a result of order 10⁻⁵ Mm is catastrophic cancellation.

A worst-case bound. With `u = 2⁻²⁴ = 5.96·10⁻⁸`:

* rounding `d` to f32: `≤ u·|d| ≤ u‖cam‖`
* rounding `m` to f32: `≤ u‖m‖`, contributing `≤ u‖m‖` through `n·δm`
* rounding `n` to f32: `≤ 2u` per component ⇒ `≤ 2√3·u‖m‖`
* accumulating the 3-term dot product: `≤ 3u·Σ|n_i m_i| ≤ 3u‖m‖`

Total `≲ 8u·‖cam‖ ≈ 4.8·10⁻⁷ · 6.378 Mm = 3.0 m`, **independent of how close
the tile is**. A random-normal experiment over 4000 samples gives a max of
**3.3 m**; the harness's measurement on the *specific* plane normals of one
camera is **0.128 m** (a typical, not worst, case). Either number is fatal at
z ≥ 17, where the tile's own projected radius is ~0.3 m.

### 2.5 Precision: the camera-relative frame is the cure

Translate the world so the camera is the origin. For a plane through the eye
(all four side planes are), the plane becomes

```
n·(p − cam) ≥ 0            i.e.   (n, d) → (n, 0)
```

**`d = 0` holds exactly, by construction**, if the implementation *writes* zero
rather than computing `−n·cam`. (Numerically, `n·cam + d` evaluated in f64 for
the four side planes of the reference camera comes out at
`|·| ≤ 4.4·10⁻¹⁶ Mm = 0.44 nm` — i.e. it is zero to f64 rounding even when
computed. But writing the literal 0 removes even that.)

Now evaluate `s = n·Δ` with `Δ = m − cam` formed in **f64** and then downcast.
The same bound as §2.4, with `‖cam‖` replaced by `‖Δ‖`:

```
|δs| ≤ 8u·( ‖Δ‖ + Σ_j ‖h_j‖ ) ≈ 4.8·10⁻⁷ ·( ‖Δ‖ + Σ_j ‖h_j‖ )             (2.2)
```

Measured (4000 random normals, camera at 6.38 Mm):

| ‖Δ‖ | abs-frame max error | cam-relative max error | ratio |
|---|---|---|---|
| 10 m      | 2.71 m | 1.54·10⁻⁶ m | 1.8·10⁶ |
| 1 km      | 2.92 m | 1.51·10⁻⁴ m | 1.9·10⁴ |
| 100 km    | 2.80 m | 1.24·10⁻² m | 2.3·10² |
| 6 400 km  | 3.33 m | 8.63·10⁻¹ m | 3.9 |

The camera-relative error is exactly `1.5·10⁻⁷·‖Δ‖`, as (2.2) predicts.

**Why that is the right shape.** The LOD rule subdivides at
`dist < lod_radius · lod_factor` with `lod_factor = 2`. A leaf therefore has
`lod_radius ≤ D/2`, and its parent was subdivided so `lod_radius > D/4`. The
tile's half-diagonal is between `D/4` and `D/2`. Hence

```
error / tile_size  ≈  1.5·10⁻⁷·D / (D/4)  =  6·10⁻⁷                       (2.3)
```

**Constant, at every zoom, at every distance.** Compare the absolute frame at
z = 20 near the ground: `3.0 m / 20 m ≈ 15 %` of the tile (the harness measures
`0.128 m / 0.3 m = 43 %` against the *projected* radius). The fix is not an
improvement of degree; it changes the error from scale-dependent to scale-free.

**Prerequisite.** `Δ = m − cam` must be an f64 subtraction. That requires the
tile's OBB centre to be stored in f64. This is invariant **I-2**. Today
`QuadtreeNode::center` and `OrientedBoundingBox::center` are `Vec3`; the centre
alone already carries ~0.5 m of construction error (see §2.7), which
camera-relative arithmetic cannot undo.

**A caveat the implementer must know.** `Camera::local_pos` is `Vec3` (f32) and
in Free mode `anchor_pos = 0`, so the camera's *absolute* position is quantised
to 0.48 m. This does **not** break camera-relative culling — the frustum and the
tiles are both referred to the same f64 value returned by
`global_transform_f64()`, so the culling problem is internally consistent, and
the harness's oracle uses that same value. It does mean "5 m altitude in Free
mode" is a position known only to ±0.48 m. Out of scope here; listed in §11.

### 2.6 Should tile culling use near and far at all? — No.

**Far plane.** `zfar = ‖cam‖ + 10` (`camera.rs:464, 491`). For any point `p` on
the ellipsoid, `‖p − cam‖ ≤ ‖cam‖ + ‖p‖ ≤ ‖cam‖ + a = ‖cam‖ + 6.378 < zfar`.
So **no ellipsoid point is ever beyond the far plane**, and the far plane can
never reject a tile. Adding it back is exactly neutral: 0 FN change, 0 FP
change. *Invariant* **I-3**: `zfar ≥ ‖cam‖ + a`. If anyone tightens `zfar`, the
far plane (`π = r2`) must be reinstated.

**Near plane.** A conservative near-plane test rejects a tile only when its
*entire* bounding volume is nearer than `znear`. The minimum distance from the
camera to the ellipsoid is `alt`. A tile's bounding volume contains ellipsoid
points. Therefore:

> If `znear < alt`, the near plane provably rejects nothing.

* **Free**: `znear = clamp(0.1·alt, 10⁻⁷, 10)`. For `alt > 10⁻⁶ Mm = 1 m`,
  `znear = 0.1·alt < alt`. Vacuous.
* **Cockpit**: `znear = 5·10⁻⁸ Mm = 5 cm`. Vacuous for any aircraft.
* **Tracking**: `znear = clamp(0.05·‖local_pos‖, 10⁻⁸, 5·10⁻⁶)`, i.e. **≤ 5 m**,
  and pinned at 5 m whenever the anchor is the Earth centre. Vacuous whenever
  `alt > 5 m`. Below 5 m it is *not* vacuous — and that is precisely the regime
  where it destroys the globe (§2.8).

**And the near plane is not needed to reject things behind the camera.** In eye
space the left and right planes are `w·x_e − z_e ≥ 0` and `−w·x_e − z_e ≥ 0`;
their sum is `−2z_e ≥ 0`, i.e. `z_e ≤ 0`. So

> **Lemma 2.1.** The four side half-spaces already imply `z_e ≤ 0`: nothing
> behind the eye satisfies them.

Verified numerically for the reference frustum at 1 mm, 100 m, 1 km, 10 Mm and
100 Mm behind the eye: rejected in every case by the side planes alone.

**Conclusion.** Tile culling uses **four planes**. The depth range is left
entirely to the depth buffer and to near clipping, which are the mechanisms that
actually implement it. Cost drops 33 % *and* the 1.3·10⁹:1 depth ratio leaves
the tile-culling path completely.

Soundness: dropping a test can only add false positives, never false negatives.
For the far plane the addition is provably zero; for the near plane it is at
most the set of tiles entirely within `znear` of the eye, which are exactly the
tiles that near clipping discards anyway.

### 2.7 The Tracking-at-5 m failure, localised

Reproduced numerically (f32 arithmetic throughout, engine algorithms
transcribed) for the harness cell `lat −12.7, lon 147.564, alt 5 m, nadir,
Tracking`, walking the ancestor chain of the tile under the camera:

```
 z    plane-4 (deg.)   plane-5 (real NEAR)   projected radius   verdict
 16     +10.01 m           +0.00 m               0.26 m          keep
 17      +9.54 m           −0.48 m               0.30 m          CULL
 18     +10.01 m           +0.00 m               0.27 m          (unreachable)
```

The z = 17 ancestor is rejected by plane index 5 — the **real near plane** —
because the camera is at 5.058 m and `znear` is pinned at 5.000 m, so the true
signed distance is 0.058 m: eleven orders of magnitude below the 6.378 Mm
operands it is computed from. The f32 result is noise at the ±0.5 m level, and
here the noise lands at −0.48 m against a projected radius of 0.30 m.

`QuadtreeNode::update` sets `children = None` on a cull, so killing z = 17 kills
z = 18–20. The branch contributes nothing and `collect_visible_tiles` returns
zero (or, for a different roll, one surviving stale tile). This is exactly the
harness's "0–1 tiles for a screen full of ground", and the roll dependence
follows from the roll changing which of the four z17 children is tested first
against which plane.

The two fixes are independent and either one suffices:

1. **Camera-relative** (§2.5). The same quantity computed as `n·Δ − znear` with
   `Δ` from an f64 subtraction gives `−0.2686 m` exactly (the residual −0.27 m
   is the error in the f32-constructed OBB *centre*, not in the plane), against
   the exact f64 value of −0.2686 m. Error 5·10⁻⁷ m.
2. **Dropping the near plane** (§2.6). The test disappears.

Note that even the exact answer at z = 17 is `−0.27 m` against `r = 0.30 m` —
a 10 % margin, and `r` here is itself f32 noise in the box's normal extent.
Being 10 % from a cliff edge is not a design; **fix 2 is the robust answer and
fix 1 is needed anyway** for the side planes.

### 2.8 Statement of the frustum test

Per frame:

```
M   = P_rz · V                              (f64)
πL  = r3 + r0 ,  πR = r3 − r0 ,  πB = r3 + r1 ,  πT = r3 − r1
for each: n = normalize(π.xyz)  (f64), then downcast n to f32, set d := 0
```

Per node, for each candidate box `(m, h₀, h₁, h₂)` (the node's OBB, or its
sub-boxes for z ≤ 4):

```
Δ  = f32( m_f64 − cam_f64 )                 (f64 subtract, then downcast)
for each of the 4 planes:
    s = n·Δ
    r = |n·h₀| + |n·h₁| + |n·h₂|
    if s + r < −ε :  reject
```

with the **derived** tolerance from (2.2)

```
ε = 8u·( ‖Δ‖ + ‖h₀‖ + ‖h₁‖ + ‖h₂‖ ) ,   u = 2⁻²⁴                          (2.4)
```

In practice `‖Δ‖ ≤ 4r₁` at any leaf (§2.5), so
`ε ≈ 2.4·10⁻⁶ · r₁` — a *relative* widening of the box by 2.4 ppm. An
implementer may hard-code `ε = 2⁻¹⁸ · (‖Δ‖₁ + Σ‖h_j‖₁)`; that is 4× (2.4) and
still invisible. This is a derived tolerance, not a fudge: it is the f32
rounding bound of the expression being evaluated.

**Soundness.** The box `B` is rejected only when
`sup_{p∈B} (n·(p − cam)) < 0`, i.e. `B` lies strictly in the open half-space
outside one frustum plane, which is disjoint from the frustum. Hence
`B ∩ frustum = ∅`, hence `tile ⊆ B` is unseen. ∎

**Cost.** Per plane: 3 mul + 2 add (`n·Δ`), 9 mul + 6 add (`n·h_j`), 3 abs,
3 add ⇒ 12 mul, 11 add. Four planes with early-out: worst case **48 mul, 44
add ≈ 92 flops** per box. Current 6-plane version: 138 flops. Paid per frame per
node.

---

## 3. Horizon / limb occlusion

### 3.1 Exact occlusion of a point

Work in scaled space. The occluder is the unit **ball** (the ellipsoid is
convex, so it is the only occluder; there is nothing else in the scene).

**Definition.** `q` is occluded from `c` iff the open segment `(c, q)` meets the
open unit ball.

**Theorem 3.1 (exact point test).** Let `C² = c·c > 1`, `h² = C² − 1 > 0`,
`v = q − c`, and `s = −v·c = C² − q·c`. Then

```
q is occluded   ⟺   s > h²   ∧   s² > h²·‖v‖²                             (3.1)
```

*Proof.*
The first condition: `s > h² ⟺ C² − q·c > C² − 1 ⟺ q·c < 1`. The set
`{x : x·c = 1}` is the **polar plane** of `c`, which is exactly the plane of the
tangency circle of the cone from `c` to the unit sphere. So condition 1 says
"`q` is strictly beyond the horizon plane".

The second condition: the squared distance from the origin to the *infinite
line* through `c` with direction `v` is `d² = ‖c‖² − (c·v)²/‖v‖²`, and
`c·v = q·c − C² = −s`. So `d² = C² − s²/‖v‖²`, and
`s²/‖v‖² > h² = C² − 1 ⟺ d² < 1`. Condition 2 says "the line `cq` pierces the
open ball".

Now, the shadow of the unit ball seen from `c` is the intersection of the
forward nappe of the tangent cone with the half-space `x·c ≤ 1`, which is the
set of points whose segment to `c` meets the ball. Condition 2 places `q` on the
double cone; condition 1 excludes the backward nappe, because for
`q = c + t v` with `t < 0` and `v` pointing towards the ball we have
`q·c = C² + t(v·c) > C² > 1`. Conditions 1 ∧ 2 therefore characterise exactly
the forward shadow. ∎

`s > h² ≥ 0` makes `s > 0`, so squaring in condition 2 is safe and no division
is needed. (Cesium's `EllipsoidalOccluder` is (3.1) with the division left in;
the port in `bounding_volume.rs:76` and `label/culling.rs:6` is faithful to it.)

**Branch `C ≤ 1` (camera at or below the surface).** Then `h² ≤ 0` and (3.1)
degenerates: `s² > h²·‖v‖²` is vacuously true for `h² < 0`, so *everything* is
reported occluded. This is the bug behind `vh_mag_sq > -0.1`: the guard band
`h² > −0.1` corresponds to `C > 0.9487`, i.e. the camera up to
`0.0513 · 6378 km ≈ 327 km` **below** the surface — and inside that band the
test is applied and is garbage.

The geometrically correct answer for `C ≤ 1` is that the horizon test must be
**skipped** (cull nothing). From a point on or inside the sphere there is no
useful polar plane; the limit `C → 1⁺` shrinks the visible cap to a point, which
is correct but useless and numerically unusable. The branch must be
`if C² > 1 { … } else { keep }`, with no band.

### 3.2 Exact occlusion of a convex set

**Theorem 3.2.** The shadow region
`S = { x : x·c ≤ 1 } ∩ (forward tangent cone from c)` is **convex**.

*Proof.* The forward nappe of a circular cone with apex `c` is a convex solid
cone; intersecting it with a closed half-space preserves convexity. That `S`
also contains every ball point with `x·c ≤ 1` follows because the cone is
tangent to the sphere, so at every depth the cone's cross-section contains the
sphere's. ∎

**Corollary 3.3.** A *polytope* `V` is entirely occluded iff **every vertex of
`V` is occluded** by (3.1). No interior or edge extremum can escape a convex
region that contains all the vertices.

This is the exact test for an OBB: 8 vertex evaluations, no sampling, no
epsilon. It is the fall-back if §3.3 is ever not applicable.

### 3.3 The exact test for a tile, and why it collapses to one plane

**Theorem 3.4.** Let `p` lie **on** the ellipsoid and `q = T(p)`. Then

```
n̂(p)·(cam − p)  =  ( q·c − 1 ) / ‖g(p)‖ ,     ‖g(p)‖ ∈ [1/a, 1/b]         (3.2)
```

*Proof.* With `g(p) = (p_x/a², p_y/b², p_z/a²)` we have `g(p) = T(q)` because
`p_x/a² = q_x/a`. Then `T(q)·p = Σ (q_i/s_i)(s_i q_i) = ‖q‖² = 1` (as `p` is on
the ellipsoid), and `T(q)·cam = q·T(cam) = q·c`. Hence
`g(p)·(cam − p) = q·c − 1`; divide by `‖g(p)‖`. ∎

Two consequences, both central.

**(a) Back-face and horizon are the same test.** The front-face condition
`n̂(p)·(cam − p) ≥ 0` and the horizon condition `q·c ≥ 1` are the *same
inequality*. (This also proves §4.)

**(b) For surface points the cone condition is redundant.** On the unit sphere
the visible cap is exactly `{q : ‖q‖ = 1, q·c ≥ 1}`; every sphere point with
`q·c < 1` lies inside the tangent cone by construction. So condition 2 of (3.1)
is implied. **For a point on the ellipsoid, occlusion reduces to the single
linear inequality `q·c ≤ 1`.**

Therefore, using Fact R (§1.4):

> **Theorem 3.5 (exact tile horizon test).** A tile is entirely invisible to
> self-occlusion iff
> ```
>       S(tile) := max over the spherical rectangle [λ₀,λ₁]×[φ₀,φ₁] of q·c   ≤ 1
> ```
> The skirts need no separate treatment: they lie strictly inside the ellipsoid,
> so every skirt point's segment to an exterior camera crosses the sphere and is
> occluded.

Because `q·c` is a **linear functional** of `q`, `S` has a closed form.

### 3.4 Closed form for `S`

Write `c = (c_x, c_y, c_z)`, `ρ = √(c_x² + c_z²)`, and
`λ_c = atan2(−c_z, c_x)` (the camera's longitude in the engine's convention).
With `q(λ,φ) = (cos φ cos λ, sin φ, −cos φ sin λ)`:

```
q·c = A(λ)·cos φ + c_y·sin φ ,     A(λ) = c_x cos λ − c_z sin λ = ρ·cos(λ − λ_c)
```

**Step 1 — maximise over λ.** `∂(q·c)/∂A = cos φ > 0` for `|φ| < π/2` (and `= 0`
at a pole), so for every `φ` the maximiser in `λ` is the maximiser of `A`,
independent of `φ`. Hence

```
A* = ρ                                   if λ_c lies in the arc [λ₀, λ₁]
A* = max( A(λ₀), A(λ₁) )                 otherwise
```

The arc test must be **circular**, not a linear clamp: with
`d = (λ_c − λ₀) mod 2π` and `w = (λ₁ − λ₀) mod 2π`, `λ_c` is inside iff
`d ≤ w`. (A linear clamp gives the wrong endpoint when the tile straddles the
antimeridian relative to the camera; e.g. `[−180°,−90°]` with `λ_c = 170°`.)

**Step 2 — maximise over φ.** `g(φ) = A*·cos φ + c_y·sin φ` is a single
sinusoid; on an interval of length ≤ π it is unimodal with its maximum at
`φ* = atan2(c_y, A*)`. Therefore

```
S = max over φ ∈ { φ₀, φ₁ } ∪ ( {φ*} if φ* ∈ [φ₀,φ₁] )  of  g(φ)          (3.3)
  = √(A*² + c_y²)   when φ* ∈ [φ₀,φ₁]
```

**Verification.** (3.3) was compared against a 401×401 brute-force maximisation
over 4000 random (tile, camera) pairs spanning z = 1…7, all latitudes, all
longitudes (including pole rows and the antimeridian) and altitudes from 10 m to
30 Mm. Worst discrepancy `brute − closed_form = 8.88·10⁻¹⁶`, i.e. pure f64
rounding, and never positive beyond that. The closed form never
under-estimates.

**Trig-free evaluation.** Pre-compute per tile, once, at construction:

```
cos λ₀, sin λ₀, cos λ₁, sin λ₁, cos φ₀, sin φ₀, cos φ₁, sin φ₁     (8 f64)
```

Then `A(λ₀) = c_x cos λ₀ − c_z sin λ₀` (2 mul, 1 sub), likewise for `λ₁`;
`g(φ₀) = A*·cos φ₀ + c_y·sin φ₀` (2 mul, 1 add), likewise for `φ₁`. The interior
case `φ* ∈ [φ₀,φ₁]` is detected without `atan2` from the sign of the derivative
at the endpoints:

```
g'(φ) = −A*·sin φ + c_y·cos φ
interior maximum  ⟺  g'(φ₀) > 0  ∧  g'(φ₁) < 0   ⇒   S = √(A*² + c_y²)
```

The circular arc test likewise needs no `atan2`: `λ_c ∈ [λ₀,λ₁]` iff the
direction `(c_x, −c_z)` lies in the wedge spanned by `(cos λ₀, sin λ₀)` and
`(cos λ₁, sin λ₁)`, which for an arc of width ≤ π is two 2-D cross-product sign
tests. (All tiles at z ≥ 1 have arc width ≤ 180°.)

**Cost.** ≈ 25 flops, one `sqrt` in the interior branch, **no transcendentals**,
per node per frame. Storage: 8 f64 (or 8 f32; see §3.6) per node = 64 B.

### 3.5 Statement, soundness, tightness

```
per frame:   c = T(cam_f64) ;  C2 = c·c
             horizon_active = (C2 > 1)
per node:    if horizon_active and S(tile) ≤ 1 − ε_h :  cull
```

**Soundness.** If `S ≤ 1` then every point `p` of the drawn patch satisfies
`q·c ≤ 1`, hence by Theorem 3.4 `n̂(p)·(cam − p) ≤ 0`, hence `p` is on the far
side of the polar plane, hence (Theorem 3.1 with condition 2 automatic for
surface points) `p` is occluded by the ellipsoid. The skirts are interior and
also occluded. The ellipsoid is convex, so nothing else can un-occlude them.
Therefore the tile ∩ visible-set = ∅. ∎

**Tightness.** `S` is the *exact* supremum over the patch, so on zero-relief
terrain the horizon stage has **FP = 0 as well as FN = 0**. It is not a bound;
it is the answer.

**Tolerance `ε_h`.** `S` is a sum of four products of f64 quantities of
magnitude ≤ `C`; its rounding error is `≤ 8u₆₄·C` with `u₆₄ = 1.11·10⁻¹⁶`:

```
ε_h = 8.9·10⁻¹⁶ · C                                                       (3.4)
```

Converted to a limb angle: a point with `q·c = 1 + δ` has
`facing_cos ≈ δ/h`, so the angular uncertainty is `ε_h/h`. At the engine's
2 mm surface clamp (`h = 2.5·10⁻⁵`) that is `3.6·10⁻¹¹ rad ≈ 0.23 µm` of ground.
The test is well-conditioned everywhere `C > 1`.

**Do not do this in f32.** In f32 the error in `S` is `≈ 1.2·10⁻⁷`, giving an
angular uncertainty `1.2·10⁻⁷/h`: at 400 km altitude that is 2 m of ground
(fine), but at 3 m altitude (`h = 3·10⁻⁵`) it is `4·10⁻³ rad = 0.23°`, i.e.
**26 km of ground**. Since the whole test is 25 flops, run it in f64
unconditionally. This is invariant **I-4**.

### 3.6 Why the current `compute_horizon_culling_point` is *not* the bug

The 4-corner reduction was the brief's prime suspect. It is sound, and this
matters because deleting it for the wrong reason would leave the real defect in
place.

Cesium's construction replaces the tile by the **spherical cap** of angular
radius `α_max = max over the corners of angle(q_i, d̂)`, where `d̂` is the
scaled direction of the tile centre, and represents that cap by the single point
`d̂ / cos α_max`.

**Lemma 3.6.** For a lon/lat rectangle and any centre inside it, the maximum
angular distance to the centre is attained at a **corner**.

*Proof.* `cos(dist) = cos φ cos φ_c cos Δλ + sin φ sin φ_c`. In `Δλ` it is
monotonically decreasing on `[0, π]`, so the minimum over `Δλ` is at an endpoint.
In `φ` it is `C cos φ + D sin φ` with `C = cos φ_c cos Δλ ≥ 0`, `D = sin φ_c`, a
sinusoid with a single maximum; its **minimum** over an interval of length ≤ π
is therefore at an endpoint. The joint minimum of `cos(dist)` — i.e. the maximum
distance — is at a corner. ∎

So the cap contains the patch, and the HCP test is conservative. Simulated in
f64 over the harness's own limb-band grid (8 altitudes × 8 latitudes × z = 1…4,
every tile, 129×129 samples per tile), the HCP stage produced **zero** false
negatives.

Its real cost is **false positives**: a cap around a rectangle is loose.
Measured (tiles the exact test §3.4 culls but the HCP test keeps, as a fraction
of all exactly-occluded tiles):

| altitude | z = 2 | z = 4 | z = 6 |
|---|---|---|---|
| 400 km    | 5.9 % | 1.2 % | 0.1 % |
| 2 000 km  | 15.6 % | 2.8 % | 0.4 % |
| 8 700 km  | 15.4 % | 1.0 % | 0.3 % |
| 20 000 km | 22.7 % | 2.0 % | 0.4 % |

Replacing it with §3.4 removes all of that, at lower arithmetic cost
(25 flops vs ~20 flops for the test plus the construction-time cap fit) and
with 8 scalars stored instead of a `Option<Vec3>` plus a 4-corner array.

### 3.7 When terrain relief arrives

Fact R fails the moment geometry leaves the surface, and with it Theorem 3.5(b):
an elevated point can have `q·c < 1` and still be visible over the limb. The
sound general test is the **cone test on a scaled-space bounding sphere**, which
is exact for a sphere:

**Theorem 3.7.** Let the tile's scaled-space bounding sphere be `B(m, ρ)`, let
`w = m − c`, `s = C² − m·c`, `h² = C² − 1`, `C > 1`. Then `B ⊆ shadow` iff

```
s ≥ h² + ρ·C                                     (beyond the polar plane)
s ≥ ρ·C²                                         (forward of the apex)
(s − ρ·C²)² ≥ h²·( C²‖w‖² − s² )                 (inside the tangent cone)
```

*Proof sketch.* The cone has half-angle `α` with `sin α = 1/C` and axis
`û = −c/C`. Writing `t = w·û` (axial) and `r_⊥ = ‖w − tû‖` (radial), the signed
distance from `m` to the lateral surface is `t sin α − r_⊥ cos α`, which is the
true distance to the cone boundary whenever it is non-negative (the apex is
never closer, since `√(t²+r_⊥²) ≥ t ≥ t sin α − r_⊥ cos α` for `t > 0`).
Containment needs that distance `≥ ρ`. Multiplying by `C` and substituting
`tC = s`, `r_⊥C = √(C²‖w‖² − s²)`, `cos α = h/C` gives the third line; squaring
is licensed by the second. The half-space support of a sphere is
`m·c + ρC ≤ 1`, which is the first line. Setting `ρ = 0` recovers (3.1)
exactly. ∎

For the bounding radius, note that a real-space sphere of radius `ρ_real` maps
into a scaled sphere of radius `ρ_real / b` (the largest expansion factor of
`T⁻¹`… strictly, of `T`, since `b < a`). Better: fit the sphere in scaled space
directly at construction — `T` is linear, so it costs nothing.

Alternatively, use Corollary 3.3 on the 8 vertices of the scaled-space
parallelepiped `T(OBB)` (note `T(OBB)` is a parallelepiped, not a box, but the
vertex test does not care). That is exact for the box and costs ≈ 100 flops.

**Recommendation:** implement §3.4 now; put Theorem 3.7 behind the same
interface so that turning terrain on is a one-function change. Record invariant
**I-1** loudly.

### 3.8 The exact point test, for labels

`label/culling.rs::is_behind_horizon` should become, for a label at ECEF `p`:

```
q = T(p) ;  s = C² − q·c ;  v = q − c
if C² ≤ 1 : not occluded            // camera at/below the surface
occluded ⟺ s > h²  ∧  s·s > h²·(v·v)
```

which is Theorem 3.1 verbatim — division-free, no guard band, correct for labels
above *and* below the surface. Labels are not necessarily on the ellipsoid, so
the full two-condition test is required; the §3.4 collapse does not apply.

---

## 4. Back-face culling — delete it

**Claim (hypothesis 4).** On a convex ellipsoid, a surface patch is back-facing
iff it is below the horizon, so a correct horizon test subsumes back-face
culling.

**Proved** by Theorem 3.4: `n̂(p)·(cam − p)` and `q·c − 1` differ by the strictly
positive factor `1/‖g(p)‖`, so they have the same sign, pointwise. A patch is
entirely back-facing iff `max q·c ≤ 1` iff it is entirely below the horizon. ∎

Elevation breaks the equivalence in exactly one direction: an elevated point can
be *front-facing relative to its own base normal* and still visible over the
limb, so with relief, "below the horizon" ⊊ "back-facing" and only the horizon
test (§3.7) is sound. Under Fact R the two coincide exactly.

**So the back-face test at `quadtree.rs:326-336` can be deleted outright.**
And it must be, because it is the engine's dominant false-negative source.

### 4.1 The sub-OBB back-face heuristic is unsound above 2 642 km

The code is

```rust
let normal = obb.half_axes[2].normalize_or_zero();
let max_extent = obb.half_axes[0].length().max(obb.half_axes[1].length());
if normal.dot(camera_pos - obb.center) > -max_extent { /* keep */ }
```

By (3.2), with `q_m = T(obb.center_on_surface)`,

```
normal·(cam − centre)  =  ( q_m·c − 1 ) / ‖g‖ ≈ a·( q_m·c − 1 )
```

so the code culls a sub-box iff

```
q_m·c  ≤  1 − max_extent / a  =  1 − θ                                    (4.1)
```

where `θ = max_extent / a` is precisely the sub-patch's angular half-size.

The **correct** condition is `max over the sub-patch of q·c ≤ 1`. Near the limb,
`q·c = C·cos ψ` with `cos ψ = 1/C` at the limb, so a small angular displacement
`θ` changes `q·c` by

```
Δ(q·c) ≈ C·sin ψ·θ = C·(h/C)·θ = h·θ ,        h = √(C² − 1)               (4.2)
```

So the sound margin is `h·θ`, and the code uses `θ`. The heuristic is

```
sound   ⟺   h ≤ 1   ⟺   C ≤ √2   ⟺   altitude ≤ (√2 − 1)·a ≈ 2 642 km    (4.3)
```

Above that, the code culls sub-boxes whose patch still has visible points, and
the width of the wrongly-culled band, in limb angle, is

```
FN band ≈ θ·(h − 1)/h   radians                                           (4.4)
```

**Verification.** Simulating *only* the 8×8 sub-OBB back-face loop (f64; frustum
assumed to accept) over the harness's own limb-band camera set:

| altitude | worst FN limb angle | nodes wrongly culled | `h` | `(h−1)/h` |
|---|---|---|---|---|
| 400 km    | 0.0000° | 0  | 0.360 | — |
| 1 000 km  | 0.0000° | 0  | 0.582 | — |
| 2 000 km  | 0.0218° | 2  | 0.852 | — |
| 2 642 km  | 0.0000° | 0  | 1.000 | 0.000 |
| 4 000 km  | 3.5009° | 5  | 1.284 | 0.221 |
| 8 700 km  | 3.5846° | 18 | 2.142 | 0.533 |
| **12 000 km** | **8.0760°** | 8 | 2.702 | 0.630 |
| 20 000 km | 2.3297° | 10 | 4.013 | 0.751 |
| 30 000 km | 1.5619° | 15 | 5.615 | 0.822 |

The harness measures the band reaching **8.0715°**, at 12 000 km, widening with
altitude, and viewport-shape independent. This model gives **8.0760°**, at
12 000 km, widening with altitude, and it is viewport-independent by
construction. The residual 0.5 % is f32 vs f64 and the frustum stage the model
omits.

The small 0.0218° at 2 000 km, where (4.3) says the test should be sound, is the
second-order term dropped in (4.2): for a z = 4 sub-patch `θ` is not small, and
the linearisation of `cos ψ` is optimistic. That is further evidence that
patching the margin is the wrong move — **the margin cannot be repaired by a
constant; it needs the exact supremum, which §3.4 provides for free.**

The non-monotone worst-angle column (8.08° at 12 000 km, 2.33° at 20 000 km) is
not a contradiction: (4.4) is `θ·(h−1)/h`, and `θ` falls faster with altitude
than `(h−1)/h` rises, because LOD reaches shallower zooms.

### 4.2 A second, independent defect in the same three lines

`normal = obb.half_axes[2].normalize_or_zero()`. `half_axes[2] = up · ext_z`,
and `ext_z` is the sub-patch's sagitta — 0.23 mm for a z = 16 sub-box (§7). It
is computed as a difference of f32 quantities of magnitude 6.378 Mm, whose ulp
is 0.477 m. So `ext_z` at high zoom is **pure f32 noise**, and when it rounds to
exactly 0 the normal becomes `Vec3::ZERO`, `normal.dot(...) = 0 > −max_extent`
is trivially true, and the test silently becomes a no-op. That failure mode is
*conservative*, so it is not a hole — but it means the test's behaviour above
z ≈ 13 is undefined in the literal sense. Deleting the test removes this too.

---

## 5. Frustum vs bounding volume — the false-positive side

### 5.1 The correct separating-axis set

A perspective frustum is the convex hull of 8 points, not a box. The complete
SAT axis set for two convex polytopes is: face normals of A, face normals of B,
and all pairwise cross products of edge directions.

For frustum × OBB:

* **Frustum face normals:** 6 (near ∥ far, so 5 unique for a symmetric frustum).
* **Box face normals:** 3.
* **Frustum edge directions:** the near and far quads are parallel scaled
  rectangles, so their 8 edges give only **2** unique directions; plus **4**
  lateral edges ⇒ **6** unique.
* **Box edge directions:** 3.
* **Cross products:** 6 × 3 = **18**.

**Total: 27 axes** (not the box-box 15).

Cost: each axis needs 8 frustum-point projections plus one box support
evaluation (4 dots). ≈ 27 × 12 = **324 dot products ≈ 1 900 flops** per box per
frame. Out of budget by an order of magnitude.

### 5.2 Measured recovery of each option

Reproducing the harness's `test_large_box_past_corner_is_conservative` probe in
f64 (615 boxes that provably miss the frustum, placed along the outward
bisectors of each corner and edge at 7 scales × 16 offsets):

| test | over-reports | share |
|---|---|---|
| exact 27-axis SAT | 0 | 0 % |
| 6 planes (current) | 62 | **10.1 %** |
| 4 side planes only (proposed §2.6) | 76 | 12.4 % |
| 4 side planes + 3 box-axis slab test | 13 | **2.1 %** |
| 6 planes + 3 box-axis slab test | 11 | 1.8 % |

(The harness quotes 14.9 % for the 6-plane case; the difference is that it runs
in f32 and uses a slightly different probe placement. The conclusion is the
same.)

**The box-slab test** projects the 8 *frustum corners* onto the box's own three
axes and checks for a gap against the box's slab:

```
for each box axis u_j (unit):
    lo = min over the 8 corners of (corner − m)·u_j
    hi = max over the 8 corners of (corner − m)·u_j
    if hi < −‖h_j‖ or lo > ‖h_j‖ :  reject
```

With the 8 corners pre-computed **camera-relative** once per frame, this is
24 dot products (72 mul, 48 add) plus min/max ≈ **170 flops**, run only for
boxes the 4 planes already accepted.

It recovers `(12.4 − 2.1)/12.4 = 83 %` of the plane-only over-reporting and, as
a bonus, more than compensates for dropping the near and far planes.

**Soundness.** A separating axis is a separating axis: if the projections of the
two hulls onto `u_j` are disjoint, the hulls are disjoint. Adding axes can only
reject more, and every rejection is provable. FN stays 0. ∎

**Recommendation.** Ship the 4 side planes first (it is the FN fix). Add the
box-slab test as a second, independently-togglable stage once FN = 0 is
confirmed, and measure the FP delta on `test_false_positive_rate_within_budget`
and `test_fuzz_sweep_has_no_false_negatives`. It is ~170 flops for ~10 points of
FP; whether that trade is worth it depends on how expensive a wasted tile is,
which the profiler, not this document, should decide.

### 5.3 A tighter bounding volume for the patch

Work in the ENU frame at the tile centre `(λ_c, φ_c)`, on the **unit sphere**
(scaled space), then map back. With
`up = q_c`, `east = (−sin λ_c, 0, −cos λ_c)`, `north = up × east`:

```
q·east  = cos φ · sin Δλ
q·north = sin φ · cos φ_c − cos φ · sin φ_c · cos Δλ                      (5.1)
q·up    = cos φ · cos φ_c · cos Δλ + sin φ · sin φ_c = cos(angle to centre)
```

with `Δλ = λ − λ_c ∈ [−Δ, +Δ]`, `Δ = (λ₁ − λ₀)/2`.

**East extent (closed form, exact).** The rectangle is a product set, so
`sup (cos φ · sin Δλ) = (max cos φ)(max sin Δλ)`:

```
e_east = c_max · sin Δ ,      c_max = 1 if 0 ∈ [φ₀,φ₁] else max(cos φ₀, cos φ₁)
```

symmetric about 0. (For Web Mercator, `φ = 0` is always a row boundary, so
`c_max` is always attained at an endpoint.)

**Up extent (closed form, exact).** `q·up = cos(dist to centre)`; the maximum is
`1` (at the centre, which is in the rectangle) and the minimum is
`cos θ_max` at a corner (Lemma 3.6). So the up-slab is `[cos θ_max, 1]`:

```
centre offset = −(1 − cos θ_max)/2 ,     e_up = (1 − cos θ_max)/2 = sagitta/2
```

**North extent.** `q·north` is monotone in `cos Δλ` (coefficient
`−cos φ sin φ_c`, constant sign) and monotone in `φ` on the relevant range
(`∂/∂φ = cos φ cos φ_c + sin φ sin φ_c cos Δλ > 0` whenever the patch does not
span from one pole to the other). Hence the extrema are among the **four**
combinations `φ ∈ {φ₀, φ₁} × cos Δλ ∈ {1, cos Δ}` — all of which are grid points
of a 3×3 sample.

**Consequence for the existing code.** A 3×3 sample grid over
`{λ₀, λ_c, λ₁} × {φ₀, φ_c, φ₁}` captures *all three* extrema exactly:
east from `(λ₀ or λ₁, φ endpoint)`, up-min from a corner, up-max from the
centre, north from the four combinations above. So `compute_bounding_volume`'s
`steps = 2` (a 3×3 grid) is **already mathematically sound**; its problems are
f32 and, at z ≤ 4, looseness — not under-coverage. `steps = 8` for z < 5 adds
nothing to soundness. This is worth stating because "the OBB does not contain
the tile" would have been a far worse bug than the one that is actually there.

**Recommendation:** keep the 3×3 sampling if that is simpler, but compute it in
**f64** and store the centre in f64. Or use the closed forms above (cheaper and
exact). Either is sound. Do **not** reduce to fewer than the 3×3 grid.

### 5.4 The residual false-positive floor

Even with §3.4 exact and §5.2 near-exact, a tile can be a false positive when
*part* of it is off-screen and the *rest* is back-facing, without being entirely
either. Catching that needs a joint test — e.g. clipping the OBB against the
polar plane `q·c = 1` before the frustum test, which is the "capped bounding
volume" idea. It is implementable but fiddly. It is the remaining FP floor and
is deliberately out of scope here.

---

## 6. The tangent-frame construction near the poles

### 6.1 What the current code does

```rust
let mut east = Vec3::new(0.0, 1.0, 0.0).cross(normal).normalize_or_zero();
if east.length_squared() < 0.1 { east = Vec3::new(1.0, 0.0, 0.0); }
let north = normal.cross(east).normalize();
```

`Y × n = (n_z, 0, −n_x)`, whose length is `√(n_x² + n_z²) = cos φ'` (with `φ'`
the geocentric latitude of the normal). After `normalize_or_zero` the result has
length **exactly 1 or exactly 0**, so `east.length_squared() < 0.1` can only be
true in the `0` case — the guard is dead for every near-polar tile, exactly as
the brief says.

Two distinct problems, with opposite severities:

* **Near-polar (length tiny but non-zero):** `normalize_or_zero` divides by a
  tiny length, amplifying the ~6·10⁻⁸ component error of `n` to a relative error
  of `6·10⁻⁸ / cos φ'`. For the northernmost tile row the centre latitude is
  `(85.05 + 90)/2 = 87.5°`, giving `1.4·10⁻⁶`. **This is harmless**: the frame
  stays orthonormal to rounding, and the box is *built by projecting samples
  onto that frame*, so a slightly rotated frame yields a slightly less tight box
  — never an incorrect one.
* **Exactly polar (length 0):** the fallback `east = X` fires, and `X` is **not
  orthogonal to `normal`**. `north = normal × east` is then normalised and is
  orthogonal to both, but `east ⟂ normal` fails, so the "coordinates"
  `(rel·east, rel·north, rel·up)` are *oblique*. Reconstructing
  `centre + east·off_x + north·off_y + up·off_z` and
  `half_axes = [east·ext_x, …]` is then **not** a bounding box of the samples:
  the reconstructed box can fail to contain them. **That branch is unsound.**
  It does not currently fire (no tile or sub-tile centre is exactly at ±90°,
  because centres are midpoints of `[85.05°, 90°]`-type intervals), but it is a
  loaded gun.

### 6.2 The robust construction

Do not use a cross product at all. Build `east` analytically from the centre
longitude:

```
east = ( −sin λ_c , 0 , −cos λ_c )                                         (6.1)
up   = n̂( p(λ_c, φ_c) )              (eq. 1.1)
north = normalize( up × east )
```

**Properties.**

* `‖east‖ = √(sin²λ_c + cos²λ_c) = 1` for every `λ_c`, including at the poles.
  No degeneracy, no branch, no normalisation of a near-zero vector.
* `east · up = 0` **identically**: writing `up ∝ (k cos λ_c, ·, −k sin λ_c)`,
  `east·up = (−sin λ_c)(k cos λ_c) + (−cos λ_c)(−k sin λ_c) = 0`. It is exactly
  orthogonal analytically and orthogonal to ~1 ulp numerically.
* `north = up × east` is automatically unit (both factors unit and orthogonal);
  normalise anyway to absorb rounding.
* Cost: one `sin_cos(λ_c)` at construction. Amortised; free per frame.

At a pole `east` is an arbitrary but well-defined unit direction — which is
exactly right, since "east" is genuinely undefined there and any orthonormal
frame is an acceptable box frame.

**On Duff et al.'s branchless ONB** (`b1, b2` from `copysign`): it is
numerically excellent and would also remove the degeneracy, but it produces a
frame with no relation to the tile's lon/lat directions, so the resulting box is
substantially looser for a lon/lat rectangle. Use (6.1) as the primary. Duff is
the right tool only if a frame is needed from an arbitrary normal with no
parametrisation available — which is not the case here.

### 6.3 The pole stretch

`quadtree.rs:139-144` forces `lat_max = 90` for row `y = 0` and
`lat_min = −90` for the bottom row. `TileMesh::generate` does the same
(`geometry.rs:103-108`), so the culling patch and the drawn patch agree. Folding
it into the above:

* §3.4 handles `φ₁ = π/2` with no special case: `cos φ₁ = 0, sin φ₁ = 1`, so
  `g(φ₁) = c_y`. Correct — at the pole `q·c` is independent of longitude.
* §5.3's `c_max` is unaffected (`cos φ` is still maximised at a row endpoint).
* Lemma 3.6 holds (the "corners" of a polar row include the two collapsed
  pole points).
* `lod_radius` is deliberately computed from the *un*-stretched bounds
  (`quadtree.rs:171-183`). That makes polar rows subdivide *later* than their
  true extent warrants (smaller `lod_radius` ⇒ smaller `subdivide_dist`), so the
  polar caps stay coarse and their boxes stay loose. It is an FP source, not an
  FN source, and it is a LOD decision — flagged in §8, not changed here.

---

## 7. The z ≤ 16 cliff — deriving the subdivision criterion

### 7.1 Sagitta as a function of zoom

A Web-Mercator tile at zoom `z` and latitude `φ` is approximately square on the
ground with side `L = 2πa cos φ / 2^z`. Its half-diagonal subtends, from the
Earth centre,

```
θ_max(z, φ) ≈ √2 · π · cos φ / 2^z                                        (7.1)
```

and its sagitta (the depth of the box in the "up" direction — exactly `2·e_up`
from §5.3) is

```
sagitta(z, φ) = a·( 1 − cos θ_max )   →   a·θ_max²/2 for small θ          (7.2)
```

At the equator:

| z | tile side | θ_max (rad) | sagitta | sagitta / side |
|---|---|---|---|---|
| 1 | 20 037 km | 2.221 | 10 241 km | 5.1·10⁻¹ |
| 2 | 10 019 km | 1.111 | 3 546 km | 3.5·10⁻¹ |
| 3 | 5 009 km | 0.555 | 959 km | 1.9·10⁻¹ |
| 4 | 2 505 km | 0.278 | 244 km | 9.8·10⁻² |
| 5 | 1 252 km | 0.139 | 61.4 km | 4.9·10⁻² |
| 6 | 626 km | 0.069 | 15.4 km | 2.5·10⁻² |
| 8 | 157 km | 0.0174 | 961 m | 6.1·10⁻³ |
| 10 | 39.1 km | 0.00434 | 60.0 m | 1.5·10⁻³ |
| 12 | 9.78 km | 1.09·10⁻³ | 3.75 m | 3.8·10⁻⁴ |
| 14 | 2.45 km | 2.71·10⁻⁴ | 0.234 m | 9.6·10⁻⁵ |
| **16** | **612 m** | 6.78·10⁻⁵ | **1.47 cm** | 2.4·10⁻⁵ |
| 18 | 153 m | 1.70·10⁻⁵ | 0.9 mm | 6.0·10⁻⁶ |
| 20 | 38.2 m | 4.24·10⁻⁶ | 0.06 mm | 1.5·10⁻⁶ |

Note `sagitta/side = θ_max/(2√2)` to first order: the ratio halves per zoom
level.

### 7.2 The criterion

What the subdivision is for: making the box a tight proxy for the patch, so the
frustum test does not accept a tile whose surface is off-screen. The relevant
error is the box's excess *as an angle at the eye*. Under LOD (§2.5) a leaf sits
at `D ≈ 2 · half_diagonal ≈ 2 a θ_max`, so the angular excess is

```
sagitta / D  ≈  ( a θ_max²/2 ) / ( 2 a θ_max )  =  θ_max / 4   radians     (7.3)
```

Pick a screen-space budget. With `fovy ≈ 0.81 rad` across 1080 px, one pixel is
`7.5·10⁻⁴ rad`. Allowing the box to overhang the tile by 5 % of the screen
height (≈ 54 px, generous for a *bounding*-volume slop) gives
`θ_max/4 ≤ 0.04`, i.e.

```
θ* = 0.16 rad                                                             (7.4)
```

Subdivide a node into `k × k` sub-boxes with

```
k(z) = ceil( θ_max(z) / θ* )                                              (7.5)
```

| θ* | z = 1 | z = 2 | z = 3 | z = 4 | z = 5 | z ≥ 6 |
|---|---|---|---|---|---|---|
| 0.30 | 8 | 4 | 2 | 1 | 1 | 1 |
| **0.16** | 14 | 7 | 4 | 2 | 1 | 1 |
| 0.08 | 28 | 14 | 7 | 4 | 2 | 1 |

**Therefore: sub-boxes are needed only for z ≤ 4.** For 5 ≤ z ≤ 16 the engine
currently allocates 64 boxes per node to bound a patch whose sagitta is between
61 km (z = 5) and 1.5 cm (z = 16) — and at z = 16 the sub-patch sagitta is
0.23 mm, which is a factor 2000 *below* the f32 resolution of the quantities it
is computed from (§4.2). It is not merely wasted; it is not even computable.

Conversely, at z = 1 even 8×8 is not enough: `θ_max = 2.22 rad` is a patch
covering most of a hemisphere, for which no box is a useful proxy.

### 7.3 Two ways to spend the result

**Option A — raise the root level.** Start the quadtree at `z = 4` (256 roots)
instead of `z = 1` (4 roots) and delete `tight_obbs`, `compute_sub_obb` and the
whole sub-box loop. At z = 4, `sagitta/side = 9.8·10⁻²` and `θ_max/4 = 0.069`
rad — within budget without any subdivision.
*Cost:* 256 always-tested nodes per frame (≈ 33 kflops — free) and 256 × ~100 B
≈ 25 kB. *Risk:* at 30 000 km altitude the minimum visible set becomes ~100 z=4
tiles instead of ~10 z=1/z=2 tiles: more draw calls and more texture fetches at
the very top of the zoom range. That is a rendering-budget decision, not a
mathematical one.

**Option B — keep z = 1 roots, use the derived k(z).** Sub-boxes for z ≤ 4 only,
with `k = 8, 4, 2, 1` (θ* = 0.30) or `14, 7, 4, 2` (θ* = 0.16). Globally that is
`4·64 + 16·16 + 64·4 + 256·1 = 1024` sub-boxes (θ* = 0.30), allocated once at
tree construction and never again. Compare today: **every** node with z ≤ 16
allocates 64 boxes.

**Recommendation: Option B**, because it changes nothing about draw counts and
still deletes the z ≥ 5 machinery entirely. Option A is the cleaner code and
should be revisited if high-altitude draw counts turn out not to matter.

### 7.4 Memory and cost delta

Per node, today, for z ≤ 16: `Box<Vec<OrientedBoundingBox>>` with 64 entries ×
48 B = 3 072 B plus `Vec` and `Box` overhead (the brief's ~4.6 kB).

Per node, proposed: OBB 48 B + f64 centre 24 B + 8 horizon scalars 64 B ≈
**136 B**, with sub-boxes only at z ≤ 4.

Per-node per-frame flops, today (z ≤ 16, worst case):
`20 (HCP) + 138 (6-plane on the loose OBB) + 64 × (138 + 10) ≈ 9 630`.
Proposed: `25 (horizon) + 92 (4 planes) + 10 (LOD) ≈ 130`, plus 170 if the
box-slab test is enabled. **≈ 40× cheaper** for the z ≤ 16 population, which is
most of the tree.

---

## 8. LOD couplings (flagged, not redesigned)

1. **Cull-kills-subtree.** `QuadtreeNode::update` sets `children = None` on a
   cull, and `collect_visible_tiles` emits **leaves only**. So a wrong cull at
   *any* ancestor removes an entire subtree. This is why the sub-OBB back-face
   defect (§4.1) at z ≤ 16 produced holes at every zoom below it, and it is why
   every test in the pipeline must be sound at every level, not just at the
   leaves. The proposed tests are exact (§3) or provably conservative (§2.8,
   §5.2) at every level, so soundness follows by induction over the tree.

2. **`dist` in the absolute frame.** `dist = (self.center − camera_pos).length()`
   with both operands f32 at 6.378 Mm has ~0.8 m of error. At z = 20
   (`lod_radius ≈ 26 m`) that is 3 % of the subdivision threshold — swamped by
   the 1.20 hysteresis band, so not a bug today, but it should be moved to the
   camera-relative f64 subtraction with the rest (free, since `Δ` is computed
   anyway).

3. **Roots at z = 1, MAX_ZOOM = 20.** Option A in §7.3 changes the root level;
   nothing else here depends on it.

4. **`lod_radius` uses un-stretched bounds** while the bounding volume uses
   stretched ones (§6.3). Polar caps therefore stay coarse and loose: an FP
   contribution, not an FN one. Leave it; note it.

5. **No screen-space error.** Nothing in this document depends on the LOD metric
   being distance-based. If a screen-space-error metric is introduced later,
   §7.2's budget `θ*` should be re-derived against *its* target error rather
   than against 5 % of screen height.

---

## 9. Consolidated proposed algorithm

### 9.1 Per frame, once

```
(cam, ori) = camera.global_transform_f64()                       // DVec3, DQuat
M   = proj_f64(aspect) * view_f64()                              // DMat4
r0..r3 = rows of M

// four side planes, camera-relative: d ≡ 0 by construction
for (raw) in [ r3+r0, r3−r0, r3+r1, r3−r1 ]:
    n_f64 = normalize(raw.xyz)
    plane[i] = Vec3::from(n_f64)                                 // f32, no offset

// horizon constants, f64
c   = ( cam.x/a , cam.y/b , cam.z/a )
C2  = c·c
horizon_active = C2 > 1.0
rho = hypot(c.x, c.z)

// optional: 8 frustum corners for the box-slab test, camera-relative f32
corners[k] = Vec3::from( unproject(ndc_k) − cam )                // f64 subtract
```

Deliberately **absent**: `znear`, `zfar`, plane indices 4 and 5.

### 9.2 Per node, in order

```
1. HORIZON  (exact, f64, ~25 flops)                         [strongest, cheapest]
   if horizon_active:
       A* = (λ_c inside arc) ? rho : max( c.x·cosλ₀ − c.z·sinλ₀ ,
                                          c.x·cosλ₁ − c.z·sinλ₁ )
       S  = max over φ ∈ {φ₀, φ₁} of ( A*·cosφ + c_y·sinφ )
       if g'(φ₀) > 0 and g'(φ₁) < 0 :  S = max(S, sqrt(A*² + c_y²))
       if S ≤ 1 − 8.9e−16·sqrt(C2) :  cull, return

2. Δ = Vec3::from( node.center_f64 − cam )                  // f64 subtract, downcast

3. FRUSTUM  (4 side planes, camera-relative, ~92 flops)
   candidates = (z ≤ 4) ? node.sub_boxes : [ node.obb ]
   if no candidate passes all 4 planes :  cull, return
       // per candidate, per plane:
       //   s = n·(Δ + box_offset) ;  r = Σ|n·h_j|
       //   reject if s + r < −ε      with ε from (2.4)

4. BOX-SLAB  (optional, ~170 flops, only for survivors)
   for each box axis u_j:
       project the 8 camera-relative frustum corners onto u_j
       if the interval misses [−‖h_j‖, +‖h_j‖] :  cull, return

5. visible = true
   dist = ‖Δ‖
   … existing LOD / hysteresis, unchanged …
```

**Ordering rationale.** The horizon test goes first because it is both the
cheapest (25 flops vs 92) and the most selective (roughly half the globe is
below the horizon at any time, and the whole back hemisphere is rejected by one
comparison at the coarsest level). The frustum test second. The box-slab test
last, because it is the most expensive and only tightens FP.

### 9.3 Node construction (amortised, once per node)

```
λ₀, λ₁, φ₀, φ₁  (f64, with the pole stretch, from the SHARED bounds function)
cos/sin of all four                                          → 8 f64 (64 B)
centre_f64 = p(λ_c, φ_c)                                     → DVec3 (24 B)
east = (−sin λ_c, 0, −cos λ_c) ;  up = n̂(centre) ;  north = normalize(up × east)
OBB from the closed forms of §5.3, or a 3×3 grid in f64
sub-boxes only if z ≤ 4, k(z) from (7.5)
```

---

## 10. Implementation notes

### 10.1 Replaced

| existing | becomes |
|---|---|
| `Camera::calculate_frustum_planes` (`camera.rs:512`) | returns **4** side planes (`r3±r0`, `r3±r1`) with `d = 0`, plus the camera position; the depth planes are gone. If the 6-plane signature must stay for the god camera, fix the depth entries to `r3 − r2` (near) and `r2` (far) and document the order. |
| `Frustum` / `Frustum::from_planes` (`bounding_volume.rs:9,14`) | `Frustum { normals: [Vec3; 4] }`, built from f64 normals; no offsets. |
| `Frustum::intersects_obb` (`:38`) | takes `Δ = centre − cam` (already camera-relative) and applies (2.4). |
| `Frustum::contains_point` (`:29`) | takes a camera-relative point. |
| `compute_horizon_culling_point` (`:76`) | **deleted**; replaced by the 8 per-tile sin/cos scalars and the §3.4 evaluation. |
| `QuadtreeNode::horizon_culling_point` field | **deleted**; replaced by `cos_lon: [f64;2], sin_lon: [f64;2], cos_lat: [f64;2], sin_lat: [f64;2]`. |
| the horizon block at `quadtree.rs:293-314` | §3.4/§3.5, in f64, with the branch `C² > 1` and **no** `-0.1` band. |
| the sub-OBB loop at `quadtree.rs:322-344` | frustum-only, and only for `z ≤ 4`. |
| basis construction at `quadtree.rs:83-86` and `:195-198` | (6.1). |
| `QuadtreeNode::center`, `OrientedBoundingBox::center` | `DVec3` (f64). Half-axes stay `Vec3`. |
| `label/culling.rs::is_behind_horizon` | Theorem 3.1 verbatim (§3.8), f64, no guard band. |
| `get_tile_corner` (`bounding_volume.rs:51`) | f64. |

### 10.2 Deleted outright

* `compute_horizon_culling_point` and its call site.
* The sub-OBB **back-face** test (`quadtree.rs:326-336`) — §4.
* `tight_obbs` for `z ≥ 5` — §7.
* Frustum plane indices 4 and 5 as currently defined — §2.2, §2.6.
* The `vh_mag_sq > -0.1` band — §3.1.
* The dead `east.length_squared() < 0.1` guard — §6.1.

### 10.3 Invariants the implementer must preserve

* **I-1 — Zero relief.** §3.4's collapse to a single plane test is licensed by
  Fact R (all non-skirt geometry is at altitude 0, all skirts are inward). If
  terrain heights are ever applied to the mesh, §3.4 becomes **unsound** and
  must be replaced by Theorem 3.7. Put this in a comment on the function and in
  a test that asserts `TileMesh::generate` produces no vertex with positive
  altitude.
* **I-2 — f64 tile centres.** `Δ = centre − cam` must be an f64 subtraction
  followed by a downcast. Storing the centre in f32 reintroduces ~0.5 m of error
  that camera-relative arithmetic cannot remove.
* **I-3 — `zfar ≥ ‖cam‖ + a`.** The far plane is omitted because it is provably
  vacuous under the current `zfar = ‖cam‖ + 10`. Any tightening of `zfar`
  requires reinstating `π_far = r2`.
* **I-4 — Horizon in f64.** The horizon test's conditioning near the surface
  scales as `1/h`; f32 gives 0.23° of angular slop at 3 m altitude. It is 25
  flops; keep it in f64.
* **I-5 — One source of tile bounds.** The `(λ₀, λ₁, φ₀, φ₁)` used for culling
  must be bit-identical to those used by `TileMesh::generate`, including the
  pole stretch and including whether `web_mercator_y_to_lat` runs in f32 or f64.
  Extract one shared function and call it from both. A mismatch is a
  metre-scale sliver at every tile edge, i.e. a false negative.
* **I-6 — Conservative direction.** Every test must reject only on a *strict*
  proof of invisibility, with the rounding tolerance widening the kept set, not
  the culled set: `reject iff  value < −ε`, never `value < +ε`.
* **I-7 — Soundness at every level.** Because a cull discards the subtree, an
  ancestor's test must be sound, not merely "sound at leaf granularity".

### 10.4 How to verify against the existing harness

The harness already encodes the right contract. In order:

1. `test_analytic_planes::test_far_plane_is_enforced` — un-`#[ignore]` it. With
   §2.2 the plane set has no redundant entry. If the 4-plane version is adopted,
   the test's `PLANE_NAMES` and `redundant` check need updating to the new
   4-entry contract; the "point at 2× zfar" assertion should become a
   documented `assert!(true_by_I-3)` or be dropped with a comment pointing at
   I-3.
2. `test_limb_band_has_no_false_negatives` — should go to **0** and the band to
   **0.0000°**. This is the single most informative check; run it first after
   §4's deletion, before anything else, because §4 alone should close it.
3. `test_near_ground_high_zoom_has_no_false_negatives` — should go to **0**,
   driven by §2.5 + §2.6.
4. `test_fuzz_sweep_has_no_false_negatives` (100 000 cells) — should go to
   **0**.
5. `test_false_positive_rate_within_budget` and the per-cell FP rates in
   `test_zoom_cliff_probe` / `test_axis_sweep_has_no_false_negatives` — expected
   to *improve*; tighten the thresholds only after measuring.
6. `test_update_iterations_reach_fixed_point` must keep passing: none of the
   proposed changes alters the recursion structure.

Add one new analytic test that the document makes cheap: assert the closed form
(3.3) against a brute-force maximisation over a dense grid, for a few thousand
random (tile, camera) pairs. That pins the one piece of nontrivial trigonometry.

---

## 11. Expected outcome, open questions, risks

### 11.1 Expected FN

**Zero**, and for a reason rather than by measurement:

* The horizon stage is *exact* (§3.5), with a rounding tolerance derived from
  the f64 bound (3.4) applied in the conservative direction.
* The frustum stage rejects only on a strict separating half-space, with a
  rounding tolerance derived from (2.2) applied in the conservative direction.
* Both are sound at every tree level, so §8.1's subtree-discard cannot
  manufacture holes.

The two measured defect families both vanish by construction: the 8.07° limb
band is §4's deleted heuristic (305 448 misses), and the Tracking-5 m blackout
is §2.6's deleted near plane plus §2.5's camera-relative arithmetic (223 592
misses).

### 11.2 Expected FP

An estimate, not a proof. Current whole-sweep FP is **5.44 %**. Contributions
and expected movement:

| source | today | after |
|---|---|---|
| horizon stage (cap ⊋ rectangle) | 0.3 %–23 % of occluded tiles, worst at coarse z | **0** (exact) |
| plane-only SAT over-report | 10–15 % of near-miss boxes | 12.4 % with 4 planes; **2.1 %** with the box-slab test |
| OBB ⊋ patch, z ≥ 5 | small (`sagitta/side ≤ 5·10⁻²`) | unchanged |
| OBB ⊋ patch, z ≤ 4 | large | reduced by the derived `k(z)` |
| partly-off-screen ∧ partly-back-facing | — | unchanged (the §5.4 floor) |

Estimate: **5.44 % → ~2 %** with the box-slab test, **→ ~3 %** without it. Both
are guesses with the right sign; the harness will give the real number in one
run.

### 11.3 Open questions and risks

1. **Option A vs B in §7.3** is a rendering-budget question I cannot settle from
   the mathematics. *Conservative fallback:* Option B, which changes no draw
   counts.
2. **Is the box-slab test worth 170 flops?** It recovers 83 % of the plane-only
   over-reporting. Whether that beats the cost of the wasted tiles depends on
   the tile pipeline's marginal cost, which the profiler knows and I do not.
   *Conservative fallback:* ship without it, measure, add it if FP matters.
3. **`Camera::local_pos` is f32** (§2.5). In Free mode the camera's absolute
   position is quantised to 0.48 m. Culling stays self-consistent, but "5 m
   altitude" is only meaningful to ±0.48 m. If sub-metre free-flight ever
   matters, `local_pos` must become `DVec3`. Out of scope; flagged because the
   harness's near-ground cells live entirely inside that quantum.
4. **`b` as an f32 literal.** `6.356_752_4_f32` vs the f64
   `6.3567523142` differ by 8.6 cm. Once the culling path is f64 it should use
   the f64 constant — but then I-5 requires `TileMesh::generate` to use the same
   one. It already uses `EARTH_RADIUS_B_F64 = 6.3567523142`. So moving culling to
   f64 *fixes* a latent 8.6 cm inconsistency rather than creating one.
5. **`web_mercator_y_to_lat` is f32** (`tile_id.rs:3`) and is used by both the
   mesh and the quadtree. Promoting it to f64 is correct but changes tile bounds
   by ~1 m; both call sites must move together (I-5). *Conservative fallback:*
   leave it f32 and have the culling code consume its f32 output promoted to
   f64, so the two agree exactly.
6. **`b` vs `a` in the third scaled axis.** `T` divides `z` by `a`, not `b`,
   because ECEF is Y-up. Every existing scaled-space site already does this; a
   future refactor to a Z-up convention would silently invert it. Worth a unit
   test asserting `‖T(p(λ,φ))‖ = 1` for random λ, φ.
7. **`h²` as an interface.** I have deliberately removed `h²` from the tile
   path; it survives only in the *label* path (§3.8, Theorem 3.1), where points
   can be off the surface. If someone reintroduces it into tile culling to
   "share the constant", the `C ≤ 1` branch and the 1/h conditioning come back
   with it.
8. **The sub-box count `k(z)` rests on the empirical budget (7.4)** — 5 % of
   screen height. That is the one surviving free parameter in this document.
   It is calibrated by: rendering at a fixed pose, measuring the FP rate as a
   function of `θ*`, and picking the knee. `θ* = 0.30` and `θ* = 0.16` both give
   `k = 1` for `z ≥ 5`, so the choice only affects z ≤ 4, where the cost is 1024
   boxes allocated once either way. A safe default is `θ* = 0.16`.
