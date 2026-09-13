# Pre-Terrain Plan — landing the culling branch, then unfreezing LOD

**Scope:** from merging `refactor/culling-stages` up to, but not including, any terrain mode.
**Estimated shape:** 7 work packages, WP0 in an hour, WP1–WP5 the bulk, WP7 a document.

---

## Context

`refactor/culling-stages` is six commits ahead of `main`, finished and unmerged. A comparison of this engine against CesiumJS (`packages/engine/Source`) produced two findings that set this plan's direction.

**The culling maths is ahead of the reference and must not be touched.** The limb test takes an exact closed-form supremum over the spherical rectangle where Cesium reduces a tile to one conservative occludee point. `slab.rs` carries a complete separating-axis set where `CullingVolume.computeVisibility` tests plane normals only — measured at 5.91 % false positives without the edge-cross family, 2.05 % with. Nothing in this plan modifies `horizon.rs`, `slab.rs`, `bounding_volume.rs`, or the stage pipeline.

**The LOD rule is Cesium's formula with its variables frozen.** Cesium refines while `d < G(z)·H / (maxSSE · 2·tan(fovy/2))`. `QuadtreeNode::apply_lod` refines while `d < r(z) · 2.0`. These are the same rule: `G(z)` and `r(z)` both halve per level, so `lod_factor = 2.0` is the whole formula collapsed to one constant. Measured against Cesium's defaults it lands within 17 % at every zoom — equivalent to **maxSSE ≈ 1.7 px at 1080p and 46° FOV**. The rule is not wrong, it is frozen, and four things it should depend on are currently constants:

| Frozen input | Consequence today |
|---|---|
| Viewport height | A 2340-tall S23 and a 1080-tall window get identical tile density. `cockpit_s23.rs` says the S23 is a target. |
| Vertical FOV | Cockpit mode is pinned at 60°, Free/Tracking at 46.4° — a factor of 1.35 in `2·tan(fovy/2)` that nothing accounts for. |
| Imagery texture size | Carto `@2x` serves 512², Esri serves 256². Switching to `SATELLITE_IMAGERY_URL` halves effective texel density with zero LOD compensation. |
| Distance measure | `(center − eye).length()` over-estimates badly at grazing angles — exactly the cockpit and low-altitude geometry. |

Terrain lands on top of this rule. Unfreezing it first means tuning once instead of twice, and every item below is independently worth shipping regardless of whether terrain ever happens.

`docs/culling-math.md` §8.5 already records the gap in one line — *"No screen-space error. Nothing in this document depends on the LOD metric being distance-based."* That line is what WP3 changes.

---

## Ground rules for every work package

These are not suggestions; several are load-bearing for guarantees the repo already proves.

1. **Do not touch the culling maths.** `horizon.rs`, `slab.rs`, `bounding_volume.rs`, and the `Stage` / `CullPipeline` machinery in `quadtree.rs` are out of scope. The only parts of `quadtree.rs` in play are `QuadtreeNode::apply_lod`, node lifetime, and the collection traversals. Additive helpers on `OrientedBoundingBox` are fine; changes to existing behaviour are not.

2. **Respect the structural split (invariant I-7).** Culling stages take `&QuadtreeNode`; `apply_lod` takes `&mut self`. That borrow asymmetry is what makes it *impossible* for a visibility test to delete a subtree. Do not turn LOD into a `Stage`, and do not give a `Stage` access to `children`.

3. **Invariant I-2 — world positions stay f64.** Any `p − cam` is an f64 subtraction with only the difference downcast. This applies to the new OBB distance in WP3.

4. **`src/testing/culling/{oracle,cells,geodesy}.rs` are bit-stable by convention.** Editing them invalidates every historical measurement in `docs/culling-math.md`. The new LOD harness *reads* from the culling harness; it does not modify it. `cameras.rs` and `bench_update.rs` are **not** in the frozen set and may be edited (e.g. to make `bench_cells()` `pub(crate)`).

5. **The culling gate stays green at every commit:**
   ```
   cargo test --release --lib culling:: -- --test-threads=1 --nocapture
   ```

6. **Per `AGENTS.md`:** test code only under `src/testing/`; commit as soon as a feature works; anything touching rendering, geometry, or camera gets a headless visual check before it is called done.

7. **Docs are part of the deliverable.** `docs/culling-math.md` §8 (LOD couplings) and `docs/culling-implementation.md` §5 (LOD, subdivision and hysteresis) both describe the current rule literally, including line numbers. WP3 and WP4 must update them or they become lies.

---

## WP0 — Land the culling branch

**Goal.** `main` contains the stage-pipeline work, and a recorded baseline exists to measure everything else against.

**Why first.** Every package below edits `quadtree.rs`. Stacking a second refactor on an unmerged one invites a merge conflict in the most delicate file in the repo.

**Done when**
- `refactor/culling-stages` is merged to `main`.
- A baseline run of the culling gate and `bench_update` is captured — mean update µs, B/node, per-sweep FP/FN — and stored somewhere durable (a section in `docs/`, or a committed CSV). The harness writes to `$TMPDIR/cesium_culling_harness/`, which does not survive.
- Housekeeping: `culling-pipeline.html` is untracked at the repo root and looks like a debug artifact. Either commit it deliberately under `docs/` or delete it; `AGENTS.md` is explicit about not leaving clutter in the main directory.

**Files.** Git only, plus wherever the baseline is recorded.

---

## WP1 — The LOD harness

**Goal.** A measuring instrument for "is this tile the right level of detail", in the idiom of `src/testing/culling/`, with no GPU in the loop.

**Why.** `lod_factor = 2.0` is a number someone picked by eye. The only reason the culling maths can be argued superior to the reference is that the harness can prove it. The LOD path has no equivalent, so tuning it is the same eyeballing that produced 2.0 in the first place. This package builds the instrument *before* anything changes, so WP3's no-op claim is provable and WP4's tuning is measured.

**The metric.** With zero relief the geometry is always the ellipsoid, so the only real error is imagery resolution. For each visible tile at each pose:

```
screen_px = projected area of the tile patch, in pixels²
texels    = texture_size²                      (512 for Carto @2x, 256 for Esri)
ratio     = texels / screen_px                 // want ≈ 1.0
```

`ratio < 1` is blurry (under-refined), `ratio > 1` is wasted bandwidth and memory (over-refined). Project the tile's 3×3 patch samples through the camera's own f64 view-projection, clip to the viewport, and sum the four projected quad areas — do not use the OBB, which over-states area at grazing angles.

**Also record the cost side.** Per pose: total tile count, total texture bytes, deepest zoom reached. WP4 is a trade between the ratio and these numbers and cannot be made without both.

**Done when**
- A new `src/testing/lod/` module exists (`mod.rs`, `sweep.rs`, `report.rs`, plus `test_*.rs` holding no measurement logic of their own — mirroring the culling harness layout).
- It reuses `src/testing/culling/cameras.rs` (`ViewParams`, `build_camera`, `Lcg`) and the 204 poses from `bench_update.rs::bench_cells()` rather than inventing new ones.
- CSVs land in `$TMPDIR/cesium_lod_harness/`, never in the repo.
- Reports mean, p95, worst-tile and per-zoom-band aggregates, and handles the awkward cases explicitly: tiles partly off-screen, tiles behind the eye, and degenerate poses where no surface is visible.
- It **measures and does not assert** at this stage. It becomes a regression guard only once WP4 has picked the targets.
- Aggregation follows the culling harness's rule: sum raw counts and divide once, never average per-cell rates.

**Files.** `src/testing/lod/*` (new), `src/testing/mod.rs`, `src/testing/culling/bench_update.rs` (visibility of `bench_cells`).

---

## WP2 — Vertex format and draw order

**Goal.** Two independent free wins that touch nothing about the LOD metric, so they can land in parallel with WP1.

### 2a. Delete the dead vertex attribute

`geometry::Vertex` is 48 bytes: `position` 12, `normal` 12, `color` 16, `uv` 8. `vs_main` passes `color` through to the fragment stage and **`fs_solid` never reads it** — its output is built from the texture sample and the light uniform alone. Every vertex is `[1,1,1,1]`. At `mesh_segments = 16` that is a 19×19 grid, 361 vertices, **5.8 kB of constant white per tile** — a third of the vertex buffer, roughly 3 MB across a 512-mesh cache, on a platform that already needed a 512 MB texture budget.

Confirm first that no other pipeline shares `geometry::Vertex` (check `debug_geometry.rs`, `model_pipeline`, `polyline_pipeline`); if one does, decide whether to split the type or leave it.

### 2b. Order the draw list

`render_scene` draws by iterating `display_state`, a `HashMap` — so the order is arbitrary **and changes between frames**. Two fixes, both small:

- Order child recursion in `apply_lod` by the camera's quadrant relative to the tile rectangle, mirroring Cesium's `visitVisibleChildrenNearToFar`. Two comparisons per node; the collection traversals then emit roughly front-to-back.
- Draw from that ordered list instead of from the map. `render_scene` already takes a `_visible_tiles` parameter it ignores — the plumbing exists.

Beyond early-Z, this removes a plausible source of frame-to-frame z-fighting flicker between coplanar skirts under reverse-Z.

**Done when**
- The vertex struct, `TileMesh::generate`, `Vertex::desc()` and `shader.wgsl` agree on the reduced format.
- The visible tile **set** is provably unchanged — only order differs. The culling gate proves this.
- A headless capture before and after shows no visual difference.

**Files.** `crates/cesium-engine/src/globe/geometry.rs`, `render/globe_pipeline/shader.wgsl`, `globe/quadtree/quadtree.rs`, `render/wgpu_state.rs`.

---

## WP3 — Unfreeze the LOD rule, calibrated as a no-op

**Goal.** The four frozen inputs become real inputs, with the default configuration reproducing today's behaviour exactly.

**Why calibrated as a no-op.** It separates the mechanism from the tuning. If the knobs land and nothing changes on screen, then any change in WP4 is attributable to a deliberate choice rather than to a refactor bug.

### 3a. Distance to the box, not to the centre

Replace `(self.center − ctx.frustum.eye).length()` with the distance to the nearest point of the node's `OrientedBoundingBox`. The box is already on the node; this is an additive `distance_squared_to`-style helper plus a one-line substitution. Keep the subtraction in f64 (I-2).

Of everything in this plan, this is the change most likely to be visible in a screenshot — it directly fixes under-refinement at grazing angles.

> **Refuted, moved to WP4 (2026-09-13).** *The text above is left exactly as originally specified, as the record of what was planned. It did not survive measurement.*
>
> 3a was specified as part of a package "calibrated as a no-op", and those two requirements are **mutually exclusive**. The subsection above says so itself, one sentence apart: this is "the change most likely to be visible in a screenshot" *and* WP3's Done-when demands "no tile changed level at any of the 204 poses". Both cannot hold.
>
> Measured, holding everything else fixed, 3a alone:
> - moves **24.3 %** of subdivision decisions across the 204 bench poses;
> - changes the visible tile count from **3 922 to 6 685**;
> - breaks `test_visible_set_digest_is_stable`.
>
> **No recalibration can absorb this.** The ratio between the old (centre) distance and the new (box) distance is not a constant to be divided out: it runs from ~1.12 at the 5th percentile to **unbounded** — it diverges as the camera approaches the box — and it varies with the camera's angle to the patch, which is *exactly* the dependence 3a exists to introduce. A scalar `GROUND_PER_RADIUS` or `target_texel_ratio` adjustment is one number; it cannot cancel a per-pose, angle-dependent factor uniformly. Any single value that restored the old tile count at one pose would overshoot at another.
>
> So 3a does not land here. It moves to **WP4**, reframed as the deliberate tuning trade it actually is (see WP4's "3a returns, as a paired change"). Only **3b** landed in WP3.
>
> `bounding_volume.rs::distance_to_point` was still written and committed (additive, documented, **unused**) so WP4 starts with the helper in place; nothing in WP3 calls it.

### 3b. Derive `lod_factor` instead of hard-coding it

The rule keeps its shape. Only the constant becomes a function:

```
lod_factor = (GROUND_PER_RADIUS / texture_size)
           · viewport_height
           / (target_texel_ratio · 2·tan(fovy/2))
```

`GROUND_PER_RADIUS` is a geometry constant relating `unstretched_radius` to tile ground width. `texture_size`, `viewport_height` and `fovy` are all already known to the engine — they are simply not consulted. Pick `GROUND_PER_RADIUS` and the default `target_texel_ratio` so that at `H = 1080`, `fovy = 46.4°` (`focal_length = 28`) and `texture_size = 512` the expression evaluates to exactly `2.0`.

Note the naming: `TileEngineConfig.lod_factor` stops being the knob and `target_texel_ratio` becomes it. The doc comments in `config.rs` and `culling-implementation.md` §5 both call `lod_factor` "the LOD knob" and will need to change with it.

**Done when** *(as landed: 3b only — see the Refuted note above)*
- The culling gate is green and the WP1 harness reports a distribution identical to its WP0/WP1 baseline on the default config.

  With 3a deferred, this clause is **stronger than originally written, and for a dull reason**. The original text settled for "bit-identical is not required; 'no tile changed level' is" because 3a was expected to perturb the metric and need empirical recalibration. 3b alone perturbs nothing: `apply_lod`'s `dist` is *untouched* — still the distance to the node's centre — and the only change is that `lod_factor` is computed rather than typed. The calibration is exact **in rationals**, not merely to float tolerance: `fovy/2 = atan(24/56) = atan(3/7)`, so `tan(fovy/2) = 3/7` exactly, `2·tan(fovy/2) = 6/7`, and with `GROUND_PER_RADIUS = 256/315` the expression is `(256/315 / 512)·1080 / (1 · 6/7) = (1/630)·1080 / (6/7) = (12/7)/(6/7) = 2`. Confirmed to the last representable bit in f32 (`0x40000000`).

  So the observed result is not "no tile changed level" but the full harness output — all 204 pose rows and all 3 922 tile rows of the CSVs, every aggregate, every per-zoom band — **byte-identical**, and `test_visible_set_digest_is_stable` passing against its existing pin with no re-pinning.

  The credit belongs to `dist` being unchanged, not to anything clever in the formula. A future reader should not read this as evidence that the formula is well-chosen — only that it reproduces the old constant. Whether the constant itself is *right* is WP4's question, and is still open.
- `docs/culling-math.md` §8 item 5 and `docs/culling-implementation.md` §5 reflect the new rule (the metric is still distance-based; only the constant's provenance changed).
- A headless capture shows no visual change on the default config. All 8 `culling_visual` poses are **byte-identical PNGs** at a 1920×1080 capture.

  **But the no-op is scoped to `viewport_height = 1080`, and this is not a caveat — it is the feature.** `viewport_height` is now a live input, so `lod_factor` scales with it: the same captures at the harness's usual 1280×**720** differ on 5 of 8 poses, because `lod_factor` is `2 · 720/1080 = 1.333` there rather than `2.0`. Inspected by eye, the difference is exactly and only one LOD step coarser — identical framing, identical coverage, no holes, no popping. That is the correct behaviour: a 720-tall viewport genuinely needs less detail than a 1080-tall one, and the old hard-coded `2.0` was over-refining it. WP4's "feed the real viewport height … expect the S23 to ask for meaningfully more detail" is therefore **already live** as of this package; what remains for WP4 is FOV, texture size, and choosing the ratio.

  Anyone re-running a before/after visual check on this commit must do it at 1080 height, or they will be looking at this intended change and mistaking it for a regression.

**Files** *(as landed, 3b only)*.
- `globe/quadtree/quadtree.rs` — new `pub fn lod_factor_for` + `GROUND_PER_RADIUS`, carrying the derivation; exported from `globe/quadtree/mod.rs`. `apply_lod`'s `dist` line deliberately untouched.
- `globe/tiles/config.rs` — `lod_factor` → `target_texel_ratio` (default `2.0` → `1.0`), plus the new `DEFAULT_IMAGERY_TEXTURE_SIZE_PX = 512.0`.
- `camera/camera.rs` — `fovy()` / `fovy_f64()` accessors; both projection builders now call them instead of recomputing the `atan` inline. (`fovy_f64` is not `fovy() as f64`: narrowing would perturb every f64 projection.)
- `render/wgpu_state.rs` — `update_logic` sets `quadtree_manager.lod_factor` from `lod_factor_for` fresh every frame, unconditionally. `self.size.height` is already current, so resize-correctness comes free with no cache to invalidate.
- `src/testing/lod/sweep.rs` — `measure_pose` calls the *same* function, so the harness cannot drift from the renderer once WP4 varies height and mode. Evaluates to `2.0` at all 204 poses (`height = 1080`, `mode = Free`), hence the byte-identical baseline.
- `src/viewer.rs`, `src/api.rs`, `src/headless/api.rs`, `src/main.rs`, `README.md`, `src/testing/rendering/culling_visual.rs` — the public rename `maximum_screen_space_error`/`max_screen_space_error` → `target_texel_ratio`, matching the internal name 1:1. Both were dead before this package (`TileEngineConfig.lod_factor` was never wired to `QuadtreeManager` until now), so the rename was free; the SSE name was a category error, since with zero relief there is no geometric error to bound.
- `docs/culling-implementation.md` §5, `docs/culling-math.md` §8.

**Not touched.** `globe/quadtree/bounding_volume.rs::distance_to_point` was committed ahead of this package (additive, documented, unused) and stays uncalled — it is WP4's.

---

## WP4 — Spend the knobs

**Goal.** Actually close the gaps the knobs were unfrozen for. This package changes on-screen behaviour deliberately.

**Work**
- ~~Feed the **real viewport height** through on construction and on resize.~~ **Done in WP3/3b**: `update_logic` recomputes `lod_factor` from `self.size.height` every frame, so resize is covered with no cached value to invalidate. What is left here is to *measure* the consequence — the S23 should now ask for meaningfully more detail than the desktop default, and that wants confirming rather than assuming.
- Feed the **per-mode FOV**. Cockpit's `2·tan(30°) = 1.155` against Free's `0.857` means cockpit should refine ~35 % less in distance terms — currently it refines identically, carrying roughly 1.8× the tiles it can resolve.
- Feed the **imagery texture size** from the texture manager, so switching between the 512² Carto basemap and the 256² Esri satellite layer adjusts LOD instead of silently halving sharpness.
- Use the WP1 harness to pick `target_texel_ratio`. The theoretical answer is 1.0; measure it rather than assume it, and report the cost curve (tile count and texture bytes against ratio) the way `SUB_BOXES_PER_AXIS`'s doc comment reports its trade.

### 3a returns, as a paired change

Inherited from WP3, where it was refuted as a no-op (see the callout there). The mistake was framing it as a refactor. The question to ask instead is:

> **Is box-distance at a higher `target_texel_ratio` better than centre-distance at a lower one, at a fixed tile/texture budget?**

Box-distance can only *decrease* `dist` (the nearest point of a box is never further than its centre), so it can only trigger **more** subdivision, never less — measured at +70 % tiles (3 922 → 6 685) if adopted alone. It is therefore not separable from the ratio: adopting it while holding tile count roughly constant means **raising `target_texel_ratio` to compensate**, and the two must be tuned as one change.

That makes it the same cost-vs-quality trade this WP already runs for `target_texel_ratio` itself, and it should be measured the same way — tile count and texture bytes against the ratio distribution, across the 204 poses, both variants on one table. The thing to look for is whether box-distance buys a *better shape*: it fixes under-refinement at grazing angles specifically, so the honest comparison is not the aggregate but the tails — does it lift the worst under-refined tiles (`ratio << 1`) at a given budget more than simply lowering the ratio uniformly does? If it does not, it is not worth the +70 % it costs to re-spend.

`bounding_volume.rs::distance_to_point` is already in the tree, unused, waiting for this.

**Procedural note.** Whenever this lands, **re-pinning `test_visible_set_digest_is_stable`'s constants is expected and correct** — the visible set is supposed to change, that is the point of the package. This follows the WP2 draw-order precedent; that test's own doc comment anticipates and welcomes a deliberate digest change. Make the re-pin **its own commit**, with the reason and the before/after digests in the message, rather than folding it silently into a larger one. A digest change buried in a commit that also moves other things is indistinguishable from an accident.

**Done when**
- Defaults are changed and justified by a table in the docs, in the style the culling docs already use.
- `bench_update` and the WP1 harness are re-run and the new numbers recorded.
- Headless captures at desktop and S23 viewports, in Free and Cockpit modes, confirm the intended change.
- The WP1 harness gains its first regression guard now that a target exists.

**Files.** As WP3, plus `render/wgpu_state.rs` resize path and `globe/tiles/texture_manager.rs`.

---

## WP5 — Fog

**Goal.** Cull and de-refine tiles the atmosphere already hides. For an engine whose product sits at 10–12 km with the horizon permanently in frame, this is the largest single reduction in tile count available.

**Two distinct uses**, both from Cesium's `Fog.js` and `screenSpaceError`:
- **Cull outright** when `fog(distance, density) ≥ 1.0`.
- **Relax the threshold**: subtract `fog(distance, density) · fog.sse` from the error, so near-horizon tiles refine less without disappearing.

Port defaults with the maths: `density = 0.0006`, `sse = 2.0`, `maxHeight = 800 km` (fog disabled above), `heightScalar = 0.001`, `heightFalloff = 0.59`.

**The important constraint.** Fog culling is *not* geometrically sound — it deliberately discards visible geometry. Adding it to the pipeline the harness measures would make FN non-zero by design and turn every sweep red. **The stage list the branch in WP0 just introduced is exactly what makes this landable:** add fog as a `Stage`, and have production run `DEFAULT + Fog` while the harness continues to run `CullPipeline::DEFAULT`. `test_stage_prefix_only_grows_the_kept_set` still holds, because fog only ever subtracts.

Write that down where someone will find it. A future reader who adds the fog stage to the harness's pipeline will get a wall of red and no explanation.

**Done when**
- Fog is a `Stage`, never present in the pipeline the harness builds.
- Tile-count reduction at horizon poses is measured with the WP1 harness.
- Headless captures at cruise altitude confirm nothing pops visibly at the fog boundary.

**Files.** `globe/quadtree/quadtree.rs` (a new `Stage` variant), `globe/tiles/config.rs`, `docs/culling-implementation.md` §4.

---

## WP6 — Node lifetime and load priority

**Goal.** Two ports that are independently worth having and are also the last things that want doing before terrain.

### 6a. Stop freeing subtrees on cull

`QuadtreeNode::update` sets `children = None` on a cull, so a panning camera destroys and rebuilds exactly the tiles at the frustum edge — the ones most likely to come straight back. Rebuilding runs `fit_obb` plus `SubGrid::build`; at z = 3 that is 144 sub-OBBs at 25 surface samples each.

At 6.7 µs mean update this is **not a problem today**, and the package should open by confirming that with `bench_update` rather than assuming it. It becomes one with terrain, when a node also owns height data and a vertex buffer. Cesium's shape is the right one: keep the node, clear `visible`, and free from an LRU keyed on last-rendered frame against a node budget (`TileReplacementQueue`, `tileCacheSize`).

Note there are **two** sites that clear `children` — the cull path and the LOD-collapse path in `apply_lod`. Decide both deliberately; they are not the same question.

### 6b. Continuous, view-aware load priority

`TilePriority::{High, Low}` means every visible tile competes equally for the eight semaphore permits. Cesium's `computeTileLoadPriority` returns `(1 − dot(tileDirection, cameraDirection)) · distance` — one float that sorts tiles you are flying *toward* ahead of tiles at the edge of vision, then by distance. The `BinaryHeap` in `tile_fetcher.rs` already takes an ordering; only the key changes.

The hand-rolled velocity prefetch in `TileSystem::update` is reaching for the same idea. Once the dot product applies to every tile, decide whether the prefetch block still earns its place.

**Done when**
- `bench_update` shows mean update µs and B/node before and after 6a, and the result is recorded whether or not it is an improvement.
- The culling gate is green (6a must not change the visible set at all).
- Load latency at a few representative flight poses is compared before and after 6b.

**Files.** `globe/quadtree/quadtree.rs`, `globe/tiles/tile_fetcher.rs`, `globe/tiles/system.rs`, `src/testing/culling/bench_update.rs`.

---

## WP7 — Terrain decision brief

**Goal.** A short document, no code, that closes this plan and opens the next one.

Cesium runs its globe quadtree on the *terrain provider's* tiling scheme — `GeographicTilingScheme` for Cesium World Terrain — and reprojects Web-Mercator imagery onto it with a GPU pass per imagery tile. This engine is Mercator to its bones: `AGENTS.md` pins it, invariant I-5 makes `tile_bounds` the single source, pole rows are stretched to ±90° in both the mesh and the culling rectangle, and `compute_fallback_uv` assumes imagery and geometry share a tile ID.

The brief should answer one question — **is Cesium ion / Cesium World Terrain a product requirement?** — and record the recommendation with the evidence WP1–WP6 produced.

The standing recommendation is **stay Mercator, use a heightmap source** (Terrain-RGB / terrarium). It keeps every culling invariant, the pole handling and the whole imagery-fallback machinery intact, and reduces `TileMesh::generate` to "the same grid, plus a height lookup". One measured argument supports it: `G(z)/r(z)` is constant in latitude for Mercator terrain but rises from 0.00272 at the equator to 0.00472 at 55° N for geographic terrain — so with a Mercator source the distance rule keeps the right shape everywhere, and with a geographic one Europe silently gets ~1.7× too little detail relative to the tropics. The cost is that `terrain_parser.rs`, which already decodes quantized-mesh correctly, goes unused for now.

Going geographic is the right answer only if ion terrain is specifically required.

---

## Verification

**At every commit**
```
cargo test --release --lib culling:: -- --test-threads=1 --nocapture
```
Never `cargo test` bare — `AGENTS.md` is explicit, and the full suite is minutes of GPU and network work.

**Per package**

| WP | Gate | Measurement | Visual |
|----|------|-------------|--------|
| 0 | green | baseline recorded | — |
| 1 | green | harness self-check on known-good poses | — |
| 2 | green, set unchanged | — | headless, no diff |
| 3 | green | LOD harness identical to baseline | headless, no diff |
| 4 | green | LOD harness + `bench_update`, recorded | headless: desktop + S23 × Free + Cockpit |
| 5 | green (fog stage excluded from harness pipeline) | tile-count delta at horizon poses | headless at cruise |
| 6 | green, set unchanged | `bench_update` before/after | — |

**Benchmark**
```
cargo test --release --lib culling::bench -- --ignored --test-threads=1 --nocapture
```

**Headless visual check** — per `AGENTS.md`, render to PNG and inspect. `src/testing/rendering/light_audit.rs` is the pattern to copy: it builds `WgpuState::new` with no surface and captures via `render/capture.rs`. `cockpit_s23.rs` already encodes the S23 viewport.

---

## Sequencing

```
WP0 ──┬── WP1 ──┬── WP3 ── WP4 ── WP5 ──┐
      │         │                        ├── WP7
      └── WP2 ──┘                   WP6 ─┘
```

- **WP1 and WP2 are independent** and can run in parallel after WP0.
- **WP3 requires WP1**, because its no-op claim is only provable with the instrument.
- **WP5 requires WP3**, because fog modifies the error expression WP3 introduces.
- **WP6 is independent of WP1–WP5** but touches `quadtree.rs`, so it should not run concurrently with WP3 or WP5.
- **WP7 requires everything**, and is a document.

A reasonable stopping point if time runs short is after WP4: the knobs are unfrozen, the S23 and cockpit gaps are closed, and the instrument exists for whoever picks it up next.
