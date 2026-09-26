# Culling mathematics

*The derivations and proofs behind the globe's visibility culling.
[culling-implementation.md](culling-implementation.md) describes what the code does, in
execution order; this document proves why each test is sound, how tight it is, and where
each tolerance comes from.*

Every claim below is either proved here or marked explicitly as an estimate. Numbers quoted
as "measured" come from the culling harness in `src/testing/culling/` or from numerical
experiments reproduced here as formulas, so they can be re-derived rather than trusted.
Several sections analyse a natural design the engine does **not** use — six frustum planes,
an absolute-frame evaluation, a per-box back-face test, a cross-product tangent frame, a
spherical-cap horizon point — because the reason each is wrong is the reason the engine's
test has the shape it has.

---

## 0. Summary

| Result | Where |
|---|---|
| Under wgpu's clip volume and reverse-Z, an OpenGL-style six-plane extraction gets both depth planes wrong: its "near" plane lies `fn/(f−2n)` *behind* the eye, and the real far plane is missing. Predicted clearance on the reference camera 2.8557 Mm; measured 2.8557 Mm. | §2.2 |
| Tile culling needs only the four side planes. Far is vacuous for the globe; near is vacuous above a few metres and harmful below. | §2.6 |
| In a frame centred on the eye the side planes have `d = 0` exactly, and the f32 plane error falls from a distance-independent ~3 m floor to `1.5·10⁻⁷·D`, a constant 6·10⁻⁷ of a tile at every zoom. | §2.5 |
| For a point on the ellipsoid, occlusion collapses to the linear inequality `q·c ≤ 1` in scaled space, whose supremum over a tile has a ~25-flop closed form: the horizon test is **exact** (zero FN and zero FP) on zero relief. | §3.3–§3.5 |
| With relief the collapse is unsound; the cone test on a scaled-space bounding sphere is exact for the sphere. | §3.7 |
| Back-face culling is the same inequality up to a positive factor, so it adds nothing. The margin-based per-box form of it is unsound above 2 642 km, and reproduces a measured 8.07° band of limb false negatives to four digits. | §4 |
| The four planes are an incomplete separating-axis set. With near and far dropped, the frustum is a four-edged pyramid and the complete set is 19 axes; with a vertex witness in front, the exact test costs about +1.6 µs per update. | §5, §13 |
| A tangent frame built as `Y × n` has an unsound branch at the pole; the analytic frame has none. | §6 |
| The sagitta argument says sub-boxes are needed only for z ≤ 4; the measured need is a taper to z = 7, because sub-boxes buy the gap between patch and box, not the sagitta. | §7, §12.2, §13 |

With all of it in place the culling harness measures **zero** false negatives over every
sweep (1.08·10⁹ visible samples in the 100 000-cell fuzz sweep alone) and 2.05 % false
positives, against 5.39 % false positives and 286 176 false negatives for a culler built
from the rejected alternatives (§13.4).

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

with λ longitude and φ the engine's tiling latitude (the angle that names a Web
Mercator row; see [architecture.md](architecture.md#units-and-frames)). The f32 twin
`EARTH_RADIUS_B_F32 = 6.356_752_4` differs from the f64 constant by 8.6·10⁻⁸ Mm = 8.6 cm;
the culling path reads only the f64 constants (invariant **I-5**).

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
  it is the *Y* component that carries `b`, and `transform_to_scaled_space` divides it by
`b`.

### 1.3 The camera-relative frame

For a world point `p` define

```
Δ = p − cam                                                               (1.4)
```

computed in **f64** and only then downcast. Everything in the frustum stage is
expressed in `Δ`. §2.3 proves why.

### 1.4 What the renderer actually draws

For the flat surface model, `TileMesh::generate_on::<Ellipsoid>` (`globe/geometry.rs`)
places every non-skirt vertex at **altitude exactly 0** on the ellipsoid, and every skirt
vertex at altitude `−0.5/2^z` (radially *inward*). Therefore:

> **Fact R.** On the flat globe the drawable surface of a tile is exactly the ellipsoid
> patch `[λ₀,λ₁] × [φ₀,φ₁]`, plus a skirt that lies strictly *inside* the ellipsoid.

This licenses the strongest (and cheapest) form of the horizon test. It is invariant
**I-1**. The terrain surface model (`Heightfield`) displaces vertices radially by the
sampled relief and breaks Fact R; it uses the general form of §3.7 instead.

### 1.5 Tile bounds

For `TileId{z,x,y}` with `n = 2^z`:

```
λ₀ = −180 + 360x/n        λ₁ = −180 + 360(x+1)/n
φ₁ = mercator_lat(y)      φ₀ = mercator_lat(y+1)
if y == 0     : φ₁ = +90     (pole stretch)
if y == n − 1 : φ₀ = −90     (pole stretch)
```

`mercator_lat(y) = atan(sinh(π(1 − 2y/n)))`. These four numbers, pole stretch included,
come from one function, `tile_bounds` (`quadtree/tile_id.rs`), which both the culler and
the mesh builder call, so the culling patch and the drawn patch agree bit for bit. That
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
remap in `Camera::get_projection_matrix_f64`.

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

An extraction written for OpenGL's clip volume emits
`[r3+r0, r3−r0, r3+r1, r3−r1, r3+r2, r3−r2]` labelled `[L,R,B,T,N,F]`. Under wgpu's
clip volume with the reverse-Z remap:

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

The engine normalises once per frame anyway: it is 4 reciprocal square roots per frame,
it makes the error analysis expressible in metres, and the label path needs it. The OBB
test on its own would not.

### 2.4 Precision: the absolute frame is the disease

Evaluated in the absolute frame, in f32,

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
tile's OBB centre to be stored in f64. This is invariant **I-2**, and both
`QuadtreeNode::center` and `OrientedBoundingBox::center` are `DVec3`: an f32 centre alone
would already carry ~0.5 m of construction error (see §2.7), which camera-relative
arithmetic cannot undo.

**A caveat.** `Camera::local_pos` is `Vec3` (f32) and
in Free mode `anchor_pos = 0`, so the camera's *absolute* position is quantised
to 0.48 m. This does **not** break camera-relative culling — the frustum and the
tiles are both referred to the same f64 value returned by
`global_transform_f64()`, so the culling problem is internally consistent, and
the harness's oracle uses that same value. It does mean "5 m altitude in Free
mode" is a position known only to ±0.48 m (§11.3).

### 2.6 Should tile culling use near and far at all? — No.

**Far plane.** `zfar = ‖cam‖ + 10` (`Camera::get_projection_matrix`). For any point `p` on
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

* **Free**: `znear = clamp(0.1·alt, 10⁻⁷, 10)`, with `alt` the height above the ground
  under the eye (the ellipsoid when terrain is off). For `alt > 10⁻⁶ Mm = 1 m`,
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

### 2.7 What the near plane does at 5 m

Evaluated numerically for a six-plane culler in the absolute frame (f32 arithmetic
throughout) at the harness cell `lat −12.7, lon 147.564, alt 5 m, nadir,
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

`QuadtreeNode::update` sets `children = None` on a cull, so killing z = 17 kills z = 18–20.
The branch contributes nothing and `collect_visible_tiles` returns zero (or, for a different
roll, one surviving stale tile): 0–1 tiles for a screen full of ground, which is what the
harness measures for such a culler, and the roll dependence
follows from the roll changing which of the four z17 children is tested first
against which plane.

Two remedies, independent, either one sufficient:

1. **Camera-relative** (§2.5). The same quantity computed as `n·Δ − znear` with
   `Δ` from an f64 subtraction gives `−0.2686 m` exactly (the residual −0.27 m
   is the error in the f32-constructed OBB *centre*, not in the plane), against
   the exact f64 value of −0.2686 m. Error 5·10⁻⁷ m.
2. **Dropping the near plane** (§2.6). The test disappears.

Note that even the exact answer at z = 17 is `−0.27 m` against `r = 0.30 m` —
a 10 % margin, and `r` here is itself f32 noise in the box's normal extent.
Being 10 % from a cliff edge is not a design; **remedy 2 is the robust answer and remedy
1 is needed anyway** for the side planes. The engine does both.

### 2.8 Statement of the frustum test

Per frame:

```
M   = P_rz · V                              (f64)
πL  = r3 + r0 ,  πR = r3 − r0 ,  πB = r3 + r1 ,  πT = r3 − r1
for each: n = normalize(π.xyz)  (f64), then downcast n to f32, set d := 0
```

Per node, for each candidate box `(m, h₀, h₁, h₂)` (the node's OBB, or its sub-boxes;
§7, §12, §13):

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

In practice `‖Δ‖ ≤ 4r₁` at any leaf (§2.5), so `ε ≈ 2.4·10⁻⁶ · r₁` — a *relative*
widening of the box by 2.4 ppm. The implementation evaluates (2.4) with L1 norms in place
of L2 (`8u · (‖Δ‖₁ + Σ‖h_j‖₁)`, `bounding_volume.rs`), which is larger and therefore still
conservative. This is a derived tolerance, not a fudge: it is the f32 rounding bound of the
expression being evaluated.

**Soundness.** The box `B` is rejected only when
`sup_{p∈B} (n·(p − cam)) < 0`, i.e. `B` lies strictly in the open half-space
outside one frustum plane, which is disjoint from the frustum. Hence
`B ∩ frustum = ∅`, hence `tile ⊆ B` is unseen. ∎

**Cost.** Per plane: 3 mul + 2 add (`n·Δ`), 9 mul + 6 add (`n·h_j`), 3 abs,
3 add ⇒ 12 mul, 11 add. Four planes with early-out: worst case **48 mul, 44
add ≈ 92 flops** per box, against 138 for six planes. Paid per frame per node.

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
`horizon::point_is_occluded` is (3.1) without it.)

**Branch `C ≤ 1` (camera at or below the surface).** Then `h² ≤ 0` and (3.1)
degenerates: `s² > h²·‖v‖²` is vacuously true for `h² < 0`, so *everything* is
reported occluded. A guard band such as `h² > −0.1` (`vh_mag_sq > −0.1` in a port of
the division form) corresponds to `C > 0.9487`, i.e. the camera up to
`0.0513 · 6378 km ≈ 327 km` **below** the surface — and inside that band the test is
applied and is garbage.

The geometrically correct answer for `C ≤ 1` is that the horizon test must be
**skipped** (cull nothing). From a point on or inside the sphere there is no
useful polar plane; the limit `C → 1⁺` shrinks the visible cap to a point, which
is correct but useless and numerically unusable. The branch must be
`if C² > 1 { … } else { keep }`, with no band. (For points **on** the surface there is a
stronger statement: §13.3.)

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
per node per frame. Storage: 8 f64 per node = 64 B (f64 by I-4).

### 3.5 Statement, soundness, tightness

```
per frame:   c = T(cam_f64) ;  C2 = c·c
             horizon_active = (C2 > 1)
per node:    if horizon_active and S(tile) ≤ 1 − ε_h :  cull
             (the tile test drops `horizon_active`, §13.3, and adds 10⁻⁹ of
              bounds slack to ε_h, I-5)
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
`facing_cos ≈ δ/h`, so the angular uncertainty is `ε_h/h`. Even 2 mm above the surface
(`h = 2.5·10⁻⁵`) that is `3.6·10⁻¹¹ rad ≈ 0.23 µm` of ground.
The test is well-conditioned everywhere `C > 1`.

**Do not do this in f32.** In f32 the error in `S` is `≈ 1.2·10⁻⁷`, giving an
angular uncertainty `1.2·10⁻⁷/h`: at 400 km altitude that is 2 m of ground
(fine), but at 3 m altitude (`h = 3·10⁻⁵`) it is `4·10⁻³ rad = 0.23°`, i.e.
**26 km of ground**. Since the whole test is 25 flops, run it in f64
unconditionally. This is invariant **I-4**.

### 3.6 The spherical-cap reduction is sound but loose

Cesium reduces a tile to a single horizon-culling point built from its four corners. The
reduction is sound, which matters because an unsound horizon test is the obvious suspect
for limb false negatives, and the real cause lies elsewhere (§4.1).

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

The closed form of §3.4 removes all of that, at lower arithmetic cost
(25 flops vs ~20 flops for the test plus the construction-time cap fit) and
with 8 scalars stored instead of a `Option<Vec3>` plus a 4-corner array.

### 3.7 Terrain relief: the cone test

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

Both forms sit behind one interface, `SurfaceModel::is_occluded`: `Ellipsoid` runs §3.4's
exact rectangle supremum and `Heightfield` runs Theorem 3.7 (`horizon::sphere_is_occluded`).
Invariant **I-1** is what decides which a surface model may use.

#### Implementation notes

1. **The sphere is fitted from the node's OBB, not from a sampled patch.** `T` is
   linear, so `T(obb)` is the parallelepiped spanned by the three transformed
   half-axes and its convex hull is the eight sign combinations of them. A sphere
   about `T(centre)` through the farthest of those eight contains `T(obb)`
   exactly, with no sampling argument to get wrong — and since `fit_obb` already
   spans the node's `[h_min, h_max]`, it contains the relief too. This also
   sidesteps the `ρ_real / b` conversion this section offers as the alternative,
   which is correct only because `b < a` and is the obvious thing for a later
   reader to "simplify" into `ρ_real / a`, i.e. into a sphere that does not
   contain its own patch.
2. **The three inequalities are evaluated in the order written.** The second is
   what licenses the third's squaring, so reordering them for an early-out would
   make the third meaningless on exactly the inputs the second excludes.
3. **`C² ≤ 1` must return "cull nothing" here**, the opposite convention to
   `span_is_occluded`, which deliberately has no such guard. The surface-point
   form `q·c ≤ 1` stays exact for an eye at or inside the surface (see that
   function's comment); the cone form does not — `h² < 0` makes the third line
   vacuously true and reports the whole globe occluded — §3.1's guard-band failure in
another form.

Checked by `testing::terrain::test_terrain_visibility`: 72 000 points confirm the
`ρ = 0` reduction to Theorem 3.1, and 11 017 spheres that the test culls contain
no point the exact point test calls visible.

### 3.8 The exact point test, for labels

`label/culling.rs::is_behind_horizon` is, through `horizon::point_is_occluded`, for a
label at ECEF `p`:

```
q = T(p) ;  s = C² − q·c ;  v = q − c
if C² ≤ 1 : not occluded            // camera at/below the surface
occluded ⟺ s > h²  ∧  s·s > h²·(v·v)
```

which is Theorem 3.1 verbatim — division-free, no guard band, correct for labels
above *and* below the surface. Labels are not necessarily on the ellipsoid, so
the full two-condition test is required; the §3.4 collapse does not apply.

---

## 4. Back-face culling is subsumed

**Claim.** On a convex ellipsoid, a surface patch is back-facing
iff it is below the horizon, so a correct horizon test subsumes back-face
culling.

**Proved** by Theorem 3.4: `n̂(p)·(cam − p)` and `q·c − 1` differ by the strictly
positive factor `1/‖g(p)‖`, so they have the same sign, pointwise. A patch is
entirely back-facing iff `max q·c ≤ 1` iff it is entirely below the horizon. ∎

**So a separate back-face test adds nothing.** The margin-based per-box form of it that an
implementation reaches for is worse than redundant: it is unsound (§4.1).

### 4.0 The claim above depends on Fact R, and terrain relief ended Fact R

The claim is a statement about points **on** the ellipsoid, because Theorem
3.4 is. Once `TileMesh` displaces vertices radially (terrain relief) it stops applying,
and it fails in **both** directions, not one.

**Direction 1 — the base normal, which is what a node-level test uses.** For a
point `p = p₀ + a·n̂(p₀)` at altitude `a`,

```
n̂(p₀)·(cam − p) = n̂(p₀)·(cam − p₀) − a                                   (4.0)
```

so elevation drives the back-face quantity **down**. A summit whose base point
sits just inside the horizon — `n̂·(cam − p₀)` small and positive — is classified
*back-facing* as soon as `a` exceeds it, while the summit itself is in plain
sight over the limb. Base-normal back-face culling is therefore not merely
redundant at non-zero relief; it is a **false-negative source in its own right**,
and it discards exactly the geometry terrain culling exists to keep.

**Direction 2 — the true surface normal, which no node-level test has.** For a
closed solid, a facet whose outward normal faces away from the eye is reached by
a ray that was inside the solid the instant before, so it is occluded by that
solid:

```
back-facing  ⊆  occluded            for any watertight surface             (4.0′)
```

The inclusion is strict once there is relief — a front-facing slope behind a
ridge is occluded and not back-facing — and collapses to equality exactly under
Fact R, which is the claim above. So even in the favourable direction, back-face
culling can never prove anything a correct occlusion test would not: it is a
subset of occlusion, never an addition to it. And a node-level test has one
normal for a whole tile rather than one per facet, so it cannot evaluate (4.0′)
in any case.

**Why the engine has no back-face test**, for three reasons that are each independently
sufficient, none of which is "relief makes it redundant":

1. At zero relief it is subsumed by the horizon test — the claim above, proved.
2. Its margin is wrong by a factor of `h`, making it **unsound above 2 642 km**
   even at zero relief — §4.1.
3. Its normal is f32 noise above z ≈ 13 — §4.2.

At non-zero relief reason 1 is replaced by something stronger: the test would
have to go even if it had never been unsound, because (4.0) makes it a hole in
the globe. The sound test with relief is §3.7's cone test on the scaled-space
bounding sphere, and nothing in this document replaces that.

### 4.1 The sub-OBB back-face heuristic is unsound above 2 642 km

The heuristic is

```rust
let normal = obb.half_axes[2].normalize_or_zero();
let max_extent = obb.half_axes[0].length().max(obb.half_axes[1].length());
if normal.dot(camera_pos - obb.center) > -max_extent { /* keep */ }
```

By (3.2), with `q_m = T(obb.center_on_surface)`,

```
normal·(cam − centre)  =  ( q_m·c − 1 ) / ‖g‖ ≈ a·( q_m·c − 1 )
```

so it culls a sub-box iff

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

So the sound margin is `h·θ`, and the heuristic uses `θ`. The heuristic is

```
sound   ⟺   h ≤ 1   ⟺   C ≤ √2   ⟺   altitude ≤ (√2 − 1)·a ≈ 2 642 km    (4.3)
```

Above that, it culls sub-boxes whose patch still has visible points, and
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

The culling harness, run on an implementation that used this heuristic, measures the band
reaching **8.0715°**, at 12 000 km, widening with
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
is trivially true, and the test silently becomes a no-op. That failure mode is *conservative*, so it is not a hole — but it means the test's behaviour above z ≈ 13
is undefined in the literal sense. Leaving the test out removes this too.

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
| 6 planes | 62 | **10.1 %** |
| 4 side planes only (§2.6) | 76 | 12.4 % |
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

On synthetic boxes this is ~170 flops for ~10 points of FP. On real tiles it is worth far
less: tile boxes are tiny next to the frustum, and once the vertex witness and the
edge-cross axes are in place (§13) it adds 0.01 points for 0.5 µs, so the engine keeps it
compiled out (`slab::BOX_AXES_ENABLED`, §12.2).

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

**Consequence.** A 3×3 sample grid over
`{λ₀, λ_c, λ₁} × {φ₀, φ_c, φ₁}` captures *all three* extrema exactly:
east from `(λ₀ or λ₁, φ endpoint)`, up-min from a corner, up-max from the
centre, north from the four combinations above. So `fit_obb`'s 3×3 grid (`steps = 2`) is
**exactly sound** on the sphere. The 9×9 grid it uses for z < 5 covers the ellipsoid's
perturbation of the `up` axis at coarse zoom and adds nothing on the sphere. A box that
did not contain its tile would be a hole; this one provably does. The sampling runs in
**f64** and the centre is stored in f64 (I-2); fewer than 3×3 samples would be unsound.

### 5.4 The residual false-positive floor

Even with §3.4 exact and §5.2 near-exact, a tile can be a false positive when
*part* of it is off-screen and the *rest* is back-facing, without being entirely
either. Catching that needs a joint test — e.g. clipping the OBB against the
polar plane `q·c = 1` before the frustum test, which is the "capped bounding
volume" idea. It is implementable but fiddly. It is the remaining FP floor and
is deliberately out of scope here.

---

## 6. The tangent-frame construction near the poles

### 6.1 The cross-product construction

```rust
let mut east = Vec3::new(0.0, 1.0, 0.0).cross(normal).normalize_or_zero();
if east.length_squared() < 0.1 { east = Vec3::new(1.0, 0.0, 0.0); }
let north = normal.cross(east).normalize();
```

`Y × n = (n_z, 0, −n_x)`, whose length is `√(n_x² + n_z²) = cos φ'` (with `φ'`
the geocentric latitude of the normal). After `normalize_or_zero` the result has
length **exactly 1 or exactly 0**, so `east.length_squared() < 0.1` can only be
true in the `0` case — the guard is dead for every near-polar tile.

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
  the reconstructed box can fail to contain them. **That branch is unsound.** It would not
fire for a real tile (no tile or sub-tile centre is exactly at ±90°, because centres are
midpoints of `[85.05°, 90°]`-type intervals), but it is a loaded gun.

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
frame is an acceptable box frame. The engine uses (6.1) (`quadtree::tangent_frame`).

**On Duff et al.'s branchless ONB** (`b1, b2` from `copysign`): it is
numerically excellent and would also remove the degeneracy, but it produces a
frame with no relation to the tile's lon/lat directions, so the resulting box is
substantially looser for a lon/lat rectangle. Duff is the right tool only if a frame is needed from an arbitrary normal with no
parametrisation available — which is not the case here.

### 6.3 The pole stretch

`tile_bounds` forces `lat_max = 90` for row `y = 0` and `lat_min = −90` for the bottom
row, and the mesh builder uses `tile_bounds`, so the culling patch and the drawn patch
agree. Folding it into the above:

* §3.4 handles `φ₁ = π/2` with no special case: `cos φ₁ = 0, sin φ₁ = 1`, so
  `g(φ₁) = c_y`. Correct — at the pole `q·c` is independent of longitude.
* §5.3's `c_max` is unaffected (`cos φ` is still maximised at a row endpoint).
* Lemma 3.6 holds (the "corners" of a polar row include the two collapsed
  pole points).
* `unstretched_radius`, the LOD radius, is deliberately computed from the *un*-stretched
  bounds (`tile_bounds_unstretched`). That makes polar rows subdivide *later* than their
  drawn extent would suggest (smaller radius ⇒ smaller `subdivide_dist`), so the polar caps
  stay coarse and their boxes stay loose. It is an FP source, not an FN source, and it is a
  LOD decision (§8).

---

## 7. Sub-box subdivision: the sagitta criterion

This section derives how finely a node's box must be subdivided from the box's
**sagitta** — how far the box overhangs the curved patch. §12.2 shows that the sagitta is
not what sub-boxes buy in practice, and the engine's per-zoom table is measured (§13); the
derivation is kept because its numbers bound the question and its conclusion about deep
zoom holds.

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

**By this criterion, sub-boxes are needed only for z ≤ 4.** A flat 8×8 grid of sub-boxes
on every node down to z16 would bound patches whose sagitta is between 61 km (z = 5) and
1.5 cm (z = 16) — and at z = 16 the sub-patch sagitta is 0.23 mm, a factor 2000 *below* the
f32 resolution of the quantities it is computed from (§4.2). It is not merely wasted; it is
not even computable. (The measured table does use sub-boxes to z = 7 for a different reason,
§12.2; below that it agrees with this conclusion.)

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
`4·64 + 16·16 + 64·4 + 256·1 = 1024` sub-boxes (θ* = 0.30), allocated once at tree
construction, against 64 for **every** node to z16 in a flat grid.

The engine takes **Option B**'s shape — roots stay at z = 1 and draw counts are unchanged —
with the per-zoom counts measured rather than derived (§13).

### 7.4 Memory and cost delta

A flat 8×8 grid costs, per node to z16, a `Box<Vec<OrientedBoundingBox>>` of 64 × 48 B =
3 072 B plus `Vec` and `Box` overhead, about 4.6 kB.

The derived design: OBB 48 B + f64 centre 24 B + 8 horizon scalars 64 B ≈ **136 B**, with
sub-boxes only at z ≤ 4.

Per-node per-frame flops, worst case, for the flat grid with a cap-point horizon test:
`20 (HCP) + 138 (6-plane on the loose OBB) + 64 × (138 + 10) ≈ 9 630`. For the derived
design: `25 (horizon) + 92 (4 planes) + 10 (LOD) ≈ 130`, plus 170 if the box-slab test is
enabled — **≈ 40× cheaper** for the z ≤ 16 population, which is most of the tree. The
measured implementation, with its sub-box table and the exact separating axes, averages
1 919 B per node and 6.7 µs per update over the 204 bench poses (§13.4).

---

## 8. LOD couplings

1. **Cull-kills-subtree.** `QuadtreeNode::update` sets `children = None` on a
   cull, and `collect_visible_tiles` emits **leaves only**. So a wrong cull at
   *any* ancestor removes an entire subtree. That is why an unsound per-box test (§4.1)
produces holes at every zoom below the node it fails at, and why every test in the
pipeline must be sound at every level, not just at the leaves. The engine's tests are exact
(§3) or provably conservative (§2.8, §5.2, §13) at every level, so soundness follows by
induction over the tree.

2. **`dist` is an f64 subtraction.** Computed from two f32 operands at 6.378 Mm it would
   carry ~0.8 m of error — at z = 20 (`lod_radius ≈ 26 m`) 3 % of the subdivision
   threshold, inside the 1.20 hysteresis band. `apply_lod` subtracts in f64, which is free
   since the frame is camera-relative anyway.

3. **Roots at z = 1, MAX_ZOOM = 20.** Option A in §7.3 changes the root level;
   nothing else here depends on it.

4. **`lod_radius` uses un-stretched bounds** while the bounding volume uses
   stretched ones (§6.3). Polar caps therefore stay coarse and loose: an FP contribution,
not an FN one. It is kept deliberately.

5. **The LOD metric.** On the flat globe the metric is distance-based with a derived
   constant, `lod_factor_for(target_texel_ratio, texture_size, viewport_height, fovy)`,
   calibrated to exactly `2.0` at the default configuration: an imagery texel-density
   target, because with zero relief there is no geometric error. With terrain the threshold
   is the larger of that and a measured geometric screen-space error
   ([tiles-and-lod.md](tiles-and-lod.md), [terrain.md](terrain.md)). Nothing in this
   document depends on which metric is used; §7.2's budget `θ*` is expressed against 5 % of
   screen height, and the sub-box table that replaced it is measured (§13).

---

## 9. The algorithm

### 9.1 Per frame, once

```
(cam, ori) = camera.global_transform_f64()                       // DVec3, DQuat
M   = proj_f64(aspect) * view_f64()                              // DMat4
r0..r3 = rows of M

// four side planes, camera-relative: d ≡ 0 by construction
for (raw) in [ r3+r0, r3−r0, r3+r1, r3−r1 ]:
    plane[i] = f32( normalize(raw.xyz) )                         // no offset

// horizon constants, f64
c   = ( cam.x/a , cam.y/b , cam.z/a )
C2  = c·c ;  rho = hypot(c.x, c.z)
eps = (8.9e-16 + 1e-9) · max(sqrt(C2), 1)
active = C2 > 1.0                                                // point and sphere tests only

// frustum corners, camera-relative (f64 subtract, then f32), and the four far-quad
// directions as unit edge rays (f64)
```

Deliberately **absent**: `znear`, `zfar`, and any depth plane.

### 9.2 Per node, in order

```
1. HORIZON (exact, f64)
   flat:    S = max over the rectangle of q·c   (§3.4, λ half then φ half)
            if S ≤ 1 − eps : cull                // no `active` guard, §13.3
   terrain: if active and sphere_in_shadow(T(box) sphere) : cull     // §3.7

1b. TERRAIN OCCLUSION (terrain only): cull if the box is below the frame's guaranteed ridge

2. Δ = f32( centre_f64 − cam )

3. FRUSTUM on the node's own box
   circumsphere outside a plane           → cull
   circumsphere inside all four            → keep
   box outside a plane (s + Σ|n·h| < −ε)   → cull
   box inside all four                     → keep
   node has a sub-grid                     → go to 4 (straddling)
   a box vertex inside all four            → keep
   separated on an edge-cross axis (§13.2) → cull
   otherwise                               → keep

4. SUB-GRID (k × k, k from the measured table)
   pass 1: for each sub-patch not behind the limb, four planes:
           any Inside → keep ; none straddling → cull
   pass 2: for each sub-patch not behind the limb, the full test of 3 → first survivor keeps
   none survives → cull

5. visible = true ; LOD (distance to the f64 centre, hysteresis 1.2) ; recurse
```

**Ordering rationale.** The horizon test goes first because it is both the cheapest
(25 flops against 92) and the most selective (roughly half the globe is below the horizon
at any time, and the whole back hemisphere is rejected by one comparison at the coarsest
level). The frustum test comes second, cheap verdicts before expensive ones, and the exact
separating axes only for the roughly one box in a hundred wedged against a frustum edge or
corner.

### 9.3 Node construction (amortised, once per node)

```
λ₀, λ₁, φ₀, φ₁  (f64, with the pole stretch, from tile_bounds)
cos/sin of all four                                          → 8 f64 (64 B)
centre_f64 = p(λ_c, φ_c)                                     → DVec3 (24 B)
east = (−sin λ_c, 0, −cos λ_c) ;  up = n̂(centre) ;  north = up × east
OBB from a 3×3 grid in f64 (9×9 for z < 5), swept over the node's altitude span
sub-grid of k × k boxes (k from the table), each fitted on a 5×5 grid
terrain: scaled-space sphere around T(OBB), per node and per sub-patch
```

---

## 10. The design, stated as properties

### 10.1 Components

| component | derived in | code |
|---|---|---|
| four side planes, camera-relative, `d = 0` | §2.2, §2.5 | `Camera::calculate_frustum_planes`, `Frustum` |
| f64 OBB centre, f32 half-axes, L1 tolerance | §2.5, §2.8 | `OrientedBoundingBox` |
| exact tile horizon test | §3.4, §3.5, §13.3 | `TilePatch::max_dot`, `span_is_occluded` |
| cone test on a scaled sphere (relief) | §3.7 | `sphere_is_occluded`, `ScaledSphere` |
| exact point test (labels) | §3.1, §3.8 | `point_is_occluded` |
| analytic tangent frame | §6.2 | `tangent_frame` |
| edge-cross separating axes | §13.2 | `slab::separated_on_edge_cross_axes` |
| sub-patch grid with exact limb test per sub-patch | §13.3 | `SubGrid` |

### 10.2 Deliberately absent

* The near and far planes (§2.6).
* A guard band on the horizon test (§3.1), and any `active` guard on the flat tile test (§13.3).
* A per-box back-face test (§4).
* A spherical-cap horizon-culling point (§3.6).
* A degenerate-basis fallback in the tangent frame (§6.1).
* The box-axis slab stage, compiled out behind `slab::BOX_AXES_ENABLED` (§12.2, §13.5).

### 10.3 Invariants

* **I-1 — Zero relief licenses the flat horizon test.** §3.4's collapse to a single plane
  test is licensed by Fact R (all non-skirt geometry at altitude 0, all skirts inward).
  Geometry with relief must use Theorem 3.7, as the `Heightfield` surface model does.
  `test_generated_mesh_has_no_positive_altitude` holds the flat mesh to it.
* **I-1′ — Declared height bounds.** Every mesh vertex lies inside the altitude interval its
  surface model declares, and a node's box is fitted over at least that interval. Relief
  above the box is a false negative exactly like a summit outside it.
* **I-2 — f64 tile centres.** `Δ = centre − cam` is an f64 subtraction followed by a
  downcast. An f32 centre reintroduces ~0.5 m of error that camera-relative arithmetic
  cannot remove.
* **I-3 — `zfar ≥ ‖cam‖ + a`.** The far plane is omitted because it is provably vacuous under
  `zfar = ‖cam‖ + 10`. Any tightening of `zfar` requires reinstating `π_far = r2`.
* **I-4 — Horizon in f64.** The horizon test's conditioning near the surface scales as `1/h`;
  f32 gives 0.23° of angular slop at 3 m altitude. It is 25 flops.
* **I-5 — One source of tile bounds.** The `(λ₀, λ₁, φ₀, φ₁)` used for culling are
  bit-identical to those used by the mesh builder, including the pole stretch, from one f64
  function. A mismatch is a metre-scale sliver at every tile edge, i.e. a false negative.
* **I-6 — Conservative direction.** Every test rejects only on a *strict* proof of
  invisibility, with the rounding tolerance widening the kept set, not the culled set:
  `reject iff value < −ε`, never `value < +ε`.
* **I-7 — Soundness at every level.** Because a cull discards the subtree, an ancestor's
  test must be sound, not merely "sound at leaf granularity".

### 10.4 Verification

The harness encodes the contract directly:

1. `test_analytic_planes` pins the plane set, including that it has no dead entry and that
   the far plane is vacuous for the globe.
2. `test_limb_band_has_no_false_negatives` — the band measures **0.0000°**; it is the single
   most informative check of the horizon stage.
3. `test_near_ground_high_zoom_has_no_false_negatives` — the 5 m regime of §2.7.
4. `test_fuzz_sweep_has_no_false_negatives` — 100 000 random cells.
5. `test_false_positive_rate_within_budget` and the per-cell FP rates of the other sweeps.
6. `test_update_iterations_reach_fixed_point` — the recursion structure.
7. `test_horizon_closed_form_matches_brute_force` — the closed form (3.3) against a
   brute-force maximisation over a dense grid, for thousands of random (tile, camera) pairs.

---

## 11. Soundness, and what remains open

### 11.1 False negatives

**Zero**, and for a reason rather than by measurement:

* The horizon stage is *exact* (§3.5), with a rounding tolerance derived from the f64 bound
  (3.4) applied in the conservative direction.
* The frustum stage rejects only on a strict separating axis, with a rounding tolerance
  derived from (2.2) applied in the conservative direction.
* Both are sound at every tree level, so §8's subtree-discard cannot manufacture holes.

The two failure families the rejected designs show both vanish by construction: the 8.07°
limb band is §4's per-box back-face heuristic (305 448 misses), and the empty view at 5 m is
§2.6's near plane plus the absolute frame of §2.4 (223 592 misses).

### 11.2 False positives

What remains is not a defect of the frustum stage, which is exact: it is the gap between a
patch and the box around it, which the sub-grid trades against cost, and the configuration
of §5.4 — part of a tile off screen and the rest behind the limb, without either being
total. Measured totals are in §13.4.

### 11.3 Residual risks and settled questions

1. **Root level (Option A vs B, §7.3).** Settled: roots stay at z = 1.
2. **The box-slab test.** Settled by measurement: worth 0.01 points of FP for 0.5 µs on real
   tiles once the edge-cross axes exist; compiled out (§12.2).
3. **`Camera::local_pos` is f32** (§2.5). In Free mode the camera's absolute position is
   quantised to 0.48 m. Culling stays self-consistent — frustum, tiles and oracle all refer
   to the same f64 value — but "5 m altitude" is only meaningful to ±0.48 m, and the
   harness's near-ground cells live inside that quantum. Sub-metre free flight would need
   `local_pos` in f64.
4. **`b` as an f32 literal.** The f32 constant differs from the f64 one by 8.6 cm. The culling
   path uses only the f64 constants and so does the mesh builder's geometry (I-5).
5. **The Mercator inverse must be f64.** An f32 `web_mercator_y_to_lat` keeps the tiling a
   partition (both sides of an edge evaluate the same expression) but displaces every edge
   by up to 1.7 m, a real fraction of a z = 19–20 tile, and was the last false-negative
   source (§12.3). Only the f64 function exists.
6. **`b` vs `a` in the third scaled axis.** `T` divides `z` by `a`, not `b`, because ECEF is
   Y-up. A refactor to a Z-up convention would silently invert it;
   `test_scaled_space_maps_surface_to_unit_sphere` asserts `‖T(p(λ,φ))‖ = 1` for random λ, φ
   (measured 3.3·10⁻¹⁶ over 20 000 points).
7. **`h²` stays out of the tile path.** It survives only in the point and sphere tests
   (§3.1, §3.7), where points can be off the surface. Reintroducing it into the flat tile
   test to "share the constant" would bring back the `C ≤ 1` branch and the `1/h`
   conditioning with it.
8. **The sub-box budget.** §7.2's `θ*` was the one free parameter of the derivation. It is
   replaced by a table measured against all nine sweeps and the bench (§13); the table, not
   the budget, is what the code uses.

---

## 12. Measured results

Numbers are from the harness in `src/testing/culling/` unless stated otherwise.

### 12.1 Predictions confirmed

| claim | predicted | measured |
|---|---|---|
| FN = 0 by construction (§11.1) | 0 | **0**, over 1 079 616 535 visible samples in the 100 000-cell fuzz sweep, and 664 097 246 in the limb band |
| the 8.07° limb band is §4's back-face heuristic | band → 0.0000° without it | **0.0000°** |
| the closed form (3.3) never under-estimates | ≤ 8.88·10⁻¹⁶ | **8.882·10⁻¹⁶**, from an independent 257×257 brute force over 3 120 (tile, camera) pairs |
| camera-relative error is scale-free (2.3) | ≈ 6·10⁻⁷ of a tile at every zoom | **1.23·10⁻⁶**, flat from z = 4 to z = 20 |
| plane-only SAT over-reports (§5.2) | 12.4 % | **13.8 %** on the 658-box corner probe |
| the box-slab test recovers most of it | → 2.1 % | **3.6 %** |
| `‖T(p)‖ = 1` (§11.3 item 6) | — | 3.3·10⁻¹⁶ over 20 000 surface points |
| all four side planes pass through the eye | `d ≡ 0` | `max |n·eye + d| = 0` exactly |

### 12.2 Where the derivation was wrong

**§7.2's subdivision criterion bounds the wrong quantity.** It bounds the box's *sagitta*
overhang and concludes `k = 1` for `z ≥ 5`. That costs **3.1 points of false positives**.
Sub-boxes buy a fix for §5.2's corner over-report, and *that* does not decay with zoom:
distance LOD keeps every leaf at roughly the same angular size (≈26° half-diagonal, so a
screen holds ~17 tiles), and "straddling a frustum corner" is as common at z = 20 as at
z = 5. Measured against a flat floor for `z ≥ 5`, FN = 0 throughout:

| k | 1 (derived) | 2 | 3 | **4** | 5 | 8 |
|---|---|---|---|---|---|---|
| FP | 7.30 % | 5.92 % | 4.67 % | **4.18 %** | 4.06 % | 4.39 % |
| mean update | 2.5 µs | 3.2 µs | 3.7 µs | **4.3 µs** | 4.9 µs | 7.8 µs |

A flat floor calibrated this way on the fuzz sweep alone is still the wrong shape: it
improves three sweeps and worsens five others (§13.1). The corner over-report is closed
exactly by completing the separating-axis set (§13.2), after which the sub-boxes buy only
the patch/box gap, which *does* decay with zoom — hence the taper of §13.

**§11.2's false-positive forecast (~2 %) was right in the end but for the wrong reason.**
Removing unsound culling necessarily raises FP (the four-plane, exact-horizon, no-sub-box
configuration measures 7.30 %); the sub-box calibration above brings it to 4.18 %; the box-slab
test contributes only 0.2 points of the rest; the edge-cross axes of §13 bring it to 2.05 %.

**The box-slab test is not worth its cost on real tiles.** 4.18 % → 3.98 % for 4.3 → 7.0 µs.
Tile OBBs are tiny next to a frustum reaching `‖cam‖ + 10 Mm`, so their own axes rarely
separate anything the four planes did not already reject. It is implemented, measured, and
compiled out behind `slab::BOX_AXES_ENABLED`.

### 12.3 One thing the derivation did not reach

An f32 `web_mercator_y_to_lat` was **the last false-negative source**. The f32 longitude bound
`−180 + x·360/2^z` has an ulp of 1.53·10⁻⁵° ≈ **1.7 m of ground**. The tiling stays a partition
— both sides of an edge evaluate the same expression — but every tile edge sits up to 1.7 m
from its true Web-Mercator position, which at z = 19–20 (76 m and 38 m tiles) is a real
fraction of a tile. Every one of 7 177 residual fuzz misses and all 4 near-ground misses had
that signature. With one shared bounds function (I-5) there is a single call site, and the
f64 form costs nothing.

---

## 13. Completing the separating-axis set

*Everything here is measured over all nine sweeps of `src/testing/culling/`, aggregated from
the raw per-cell CSV columns with no exclusions, and against `bench_update`'s 204 poses.*

### 13.1 Why subdivision alone cannot close the corner over-report

A flat sub-box floor of `k ≥ 4` at every zoom, calibrated against `fuzz_sweep` alone,
improves `fuzz_sweep`, `near_ground_high_zoom` and `zoom_cliff`, and makes five other sweeps
worse than the rejected baseline culler — `horizon_pitch` 1.24 % → 1.62 %, `axis_sweep`
1.56 % → 4.16 %, `aspect_extremes` 1.76 % → 6.70 %, `nadir_ladder` and `fp_budget`
0.94 % → 3.05 % — all sweeps where the baseline already had FN = 0, so "its number was bought
with false negatives" does not explain them.

The regressions sit at ~1 000 km and ~5 000 km altitude, at z = 3..6, and are all the same
defect: a tile that grazes a frustum **corner**. Subdivision attacks it but never *closes*
it. Refining the grid refines the box, but every sub-box near the corner still straddles the
corner; `camera_modes`' single false positive — one z = 5 tile whose nearest point is 0.012
of half-screen outside the frustum — survives a 16 × 16 grid unchanged. §7.2 fixed `k` by a
derivation from the wrong quantity, and a flat floor fixes it by a measurement on the wrong
sample; neither is the shape of the answer.

### 13.2 The complete axis set is cheap

§5.2 prices the complete set at "27 axes, about 1 900 flops, out of budget". Both halves of
that change once the depth planes are gone.

**It is 19 axes, not 27.** With near and far dropped (I-3), the volume the test models is not
a box-shaped frustum but the infinite pyramid `P = cone(r₀..r₃)` with apex at the eye. `P`
has four faces and **four edges**, so the complete set is 4 face normals of `P`, 3 face
normals of the box, and 4 × 3 = 12 edge crosses.

**The cost is not what it costs when you always pay it.** Three witnesses settle a box before
any of that, in the order they are cheap:

1. the box's circumsphere is outside a plane, or inside all four — 20 flops;
2. the box is outside a plane, or inside all four — one pass, ~92 flops;
3. **a box vertex is inside all four** — the vertex's distance to plane `p` is
   `s_p ± r_{p,0} ± r_{p,1} ± r_{p,2}` in quantities pass 2 already computed, so all eight
   vertices cost ~96 adds, and a witness of intersection ends the question.

Only a box wedged against an edge or a corner — no vertex inside, no plane separating —
reaches the 12 crosses. Measured over the bench poses, that is about one box in a hundred.
The exact test therefore costs **+1.6 µs of a 7.8 µs budget**, not a tenfold blowup, and it
is affordable per node *and* per sub-box.

Because `P` is a cone with its apex at the origin of the camera-relative frame, its support
along an axis `a` is `0` when every `a·rₘ ≤ 0` and `+∞` otherwise, which makes each
cross-product axis ~50 flops rather than a projection of a polytope. `a = r_i × h_j ⊥ r_i`,
so only the other three rays are tested. Implementation:
`slab::separated_on_edge_cross_axes`.

### 13.3 Two more places where an exact test was already available

**Per-sub-patch occlusion.** A per-box back-face test tests the limb per *sub-box*, unsoundly
(§4). The tile-level limb test is exact but coarser: a tile whose in-frustum part is behind
the limb and whose visible part is off-screen passes both stages and is scheduled. `SubGrid`
restores the granularity soundly — a cell is discarded if its own spherical rectangle is
entirely below the limb (3.4, exact) *or* its box misses the frustum, and discarding every
cell proves the tile invisible because the cells tile the patch. The rectangles cost
`32·(k+1)` bytes, not `64·k²`, because λ and φ separate: `lon_span_max` is hoisted out of the
row loop. Worth 1.2 points of FP on the fuzz sweep on its own.

**The eye at or below the surface.** §3.1 argues that from inside the sphere there is no
useful polar plane and the only correct answer is to cull nothing. That is true of the
*cone* form (`point_is_occluded`, `sphere_is_occluded`) and false of the **surface-point**
form. For `q` on the unit sphere, `q·c ≤ 1` decides occlusion in all three regimes:
`C² > 1` is Theorem 3.5; at `C² = 1` the open chord `(c, q)` lies strictly inside the ball
for every `q ≠ c`, and `q·c < 1` exactly there; at `C² < 1` every chord starts inside, and
`q·c ≤ C < 1` for every `q`. So `TilePatch::is_occluded` has no `active` guard. It is not a
special case bolted on — it is one inequality doing the work of three, and a guard would be a
hole: with one, a camera at altitude 0 schedules 15 tiles of which the oracle calls 14
invisible, and a camera 50 m *under* the ground schedules 14 of 14. The footpoint tile is
still kept (`S ≥ C² = 1 > 1 − eps`), so there is no cliff at zero altitude — which matters,
because the camera's collision floor holds it 2 m above the surface, where the limb is about
5 km away and the kept set is little more than the ground underfoot. The harness's own
`CellResult::is_degenerate` documents the same geometry from the oracle's side.

### 13.4 Result

Same suite, same cells, same oracle, raw aggregation, FN **0** everywhere. The baseline is a
culler built from the rejected alternatives of §2.4, §3.6 and §4 (six planes in the absolute
frame, the spherical-cap horizon point, the per-box back-face heuristic):

| sweep | FP, baseline | FP, engine |
|---|---|---|
| `fuzz_sweep` | 5.440 % (73 640) | 2.078 % (26 855) |
| `near_ground_high_zoom` | 12.407 % (603) | 3.572 % (154) |
| `zoom_cliff` | 0.540 % (18) | 0.061 % (2) |
| `camera_modes` | 0.568 % (1) | 0.000 % (0) |
| `horizon_pitch` | 1.245 % (207) | 0.463 % (76) |
| `axis_sweep` | 1.564 % (23) | 0.958 % (14) |
| `aspect_extremes` | 1.765 % (3) | 1.183 % (2) |
| `fp_budget` / `nadir_ladder` | 0.938 % (6) | 0.000 % (0) |
| **total** | **5.393 % (74 507)** | **2.054 % (27 103)** |

The baseline also carries 286 176 false negatives (0.0251 % of visible samples); the engine
has none. Mean `QuadtreeManager::update` 7.8 → 6.7 µs, worst 12.8 → 16.8 µs, footprint
2 966 → 1 919 B/node.

### 13.5 What this leaves open

* The **worst-case** update is 2.5× the mean. It is a nadir view at 1 000 km, where z = 3..5
  nodes with the widest `k` all straddle at once. A per-frame budget on the sub-grid, or a
  hierarchy over the cells rather than a flat `k × k`, would cap it; neither is needed at
  17 µs.
* `slab::BOX_AXES_ENABLED` (the box's own three axes) is worth 0.01 points of FP for 0.5 µs
  and is off. It is the stage to revisit first if bounding volumes ever grow relative to the
  frustum.
* Remaining false positives are not a culling defect in the frustum stage, which is exact.
  They are the gap between a **patch and the box around it**, which `SUB_BOXES_PER_AXIS`
  trades against cost, and the harness's own finite per-tile sampling, which calls a tile
  invisible when its visible sliver misses every sample.
