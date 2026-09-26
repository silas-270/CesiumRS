# Culling baseline — post `refactor/culling-stages`

*Recorded once, at the point `refactor/culling-stages` lands on `main`
(commit `845d040`, merged as a fast-forward). This is the number every later
package in `docs/pre-terrain-plan.md` is measured against — WP2's vertex/draw
changes, WP3's calibrated-no-op claim, WP6's node-lifetime work.*

## Culling gate

```
cargo test --release --lib culling:: -- --test-threads=1 --nocapture
```

32 passed, 0 failed, 1 ignored (`bench_quadtree_update`, run separately below).
~51 s wall clock on this machine.

Notable numbers from the gate's own output, for reference:

- `test_stage_prefix_only_grows_the_kept_set`: 204 cells × 16 pipelines, 6732
  prefix pairs checked, 5439 strictly larger (I-7 holds and is non-vacuous).
- `test_is_behind_horizon_matches_exact_convexity`: 0 over-cull, 10 under-cull
  worst 0.014876° over 651 600 samples (label culling, unaffected by this
  branch — recorded here only as a snapshot of the gate's state at merge time).
- `test_horizon_closed_form_matches_brute_force`: worst over-estimate 9.522e-5°
  across 3120 (tile, camera) pairs.
- `test_tile_bounds_tile_the_sphere_without_seams`, `test_scaled_space_maps_surface_to_unit_sphere`,
  `test_generated_mesh_has_no_positive_altitude`, `test_generated_mesh_stays_inside_the_culling_rectangle`:
  all green, values unchanged in shape from the pre-refactor baseline in
  `docs/culling-math.md` / `docs/culling-implementation.md`.

## `bench_quadtree_update`

```
cargo test --release --lib culling::bench -- --ignored --test-threads=1 --nocapture
```

204 camera poses (nadir ladder + zoom-cliff ladder), single-threaded, 200
iterations per pose after an 8-iteration warm-up.

| metric | value |
|---|---|
| mean `QuadtreeManager::update` | 6.9 µs |
| worst `QuadtreeManager::update` | 17.0 µs — lat=60.0 alt=1 000 000 m pitch=0 mode=Free |
| quadtree footprint | 18 100 nodes, 33 918.1 kB total |
| bytes/node | 1 919 B |
| `size_of::<QuadtreeNode>()` | 192 B |

This matches the `SUB_BOXES_PER_AXIS` calibration table in
`crates/cesium-engine/src/globe/quadtree/quadtree.rs` (6.7 µs / 1 919 B/node
measured there) within normal run-to-run variance — same machine class, same
204 poses, same chosen row of the table.

## LOD harness — `target_texel_ratio = 1.0` (default)

*Recorded 2026-09-13, before the WP3/3b-follow-up fixes to `lod_factor_for`
(direction, exponent, `GROUND_PER_RADIUS` → calibration-constant rename). Those
three fixes are provable no-ops at this default config — see the "invisible at
`target_texel_ratio = 1.0`" note on each in `docs/pre-terrain-plan.md` WP3 — so
this is also the number after them, byte-identical CSVs included. This is the
number WP4 tunes `target_texel_ratio` against.*

```
cargo test --release --lib lod:: -- --test-threads=1 --nocapture
```

204 poses (the same nadir/zoom-cliff ladder as above), 3922 visible tiles
sampled, 0 degenerate poses, 0 offscreen/clipped-away, 0 behind-eye. Deepest
zoom reached: 20.

| stat | ratio (texels / screen_px, want ≈ 1.0) |
|---|---|
| `aggregate_ratio` (Σtexels / Σscreen_px, pooled) | **1.663** |
| mean | 56.876 (dominated by near-zero-area sliver tiles — see the harness's own NOTE; not representative on its own) |
| median | 3.161 |
| p5 | 0.273 |
| p95 | 219.469 |

Worst under-refined: `z=20 x=550502 y=364500` ratio `0.0135`, zoom-cliff pose
(`lat=48 lon=9 alt=10m`). Worst over-refined: `z=13 x=4301 y=2844` ratio
`8913.8`, same pose family.

Per-zoom-band `aggregate_ratio`:

| z | tiles | aggregate_ratio | z | tiles | aggregate_ratio |
|--:|--:|--:|--:|--:|--:|
| 1 | 32 | 2.4672 | 11 | 256 | 2.8846 |
| 2 | 20 | 0.9920 | 12 | 235 | 0.9510 |
| 3 | 83 | 1.1569 | 13 | 399 | 2.7803 |
| 4 | 32 | 0.9041 | 14 | 463 | 3.4409 |
| 5 | 42 | 0.4755 | 15 | 359 | 1.9743 |
| 6 | 65 | 0.5537 | 16 | 400 | 2.1583 |
| 7 | 86 | 0.8649 | 17 | 256 | 1.5198 |
| 8 | 175 | 5.5291 | 18 | 277 | 1.6683 |
| 9 | 177 | 2.0183 | 19 | 180 | 0.7312 |
| 10 | 226 | 2.3953 | 20 | 159 | 0.4325 |

z=1/z=2 read ~15-20 % low per the harness's own documented flat-3×3-grid bias
(`src/testing/lod/mod.rs`); treat those two rows accordingly rather than as
literal.

## WP4 / A — live imagery texture size

*Recorded 2026-09-13. `lod_factor_for`'s `texture_size_px` input is now fed live from
`TileTextureManager::current_texture_size_px()` (the real decoded tile size, via the
new `ObservedTextureSize`) instead of the frozen `DEFAULT_IMAGERY_TEXTURE_SIZE_PX`
(512) `wgpu_state::update_logic` used unconditionally before this. Not a no-op:
`satellite_imagery_url()` serves 256×256 tiles, so this changes what the engine
actually renders for that style. Measured with
`cargo test --release --lib lod::test_lod_sweep::test_wp4a_esri_texture_size_compensated_vs_uncompensated -- --nocapture`,
same 204 bench poses, `target_texel_ratio = 1.0` throughout.*

| scenario | `lod_factor_for` assumes | real texels counted | `aggregate_ratio` | tiles | texture bytes |
|---|---|---|---|---|---|
| 512px baseline (Carto, `standard_imagery_url()`) | 512px | 512px | **1.6625** | 3 922 | 3922.0 MiB |
| **compensated 256px (Esri, post-WP4/A, live)** | 256px | 256px | **1.7367** | 12 363 | 3090.8 MiB |
| uncompensated 256px (Esri, pre-WP4/A bug, reproduced for comparison) | *512px (frozen, wrong)* | 256px | **0.4156** | 3 922 | 980.5 MiB |

**Reading this**: before WP4/A, Esri's `lod_factor` was computed assuming 512px
texels it never actually had, so its measured sharpness sat at `aggregate_ratio ≈
0.42` — roughly a **quarter** of Carto's 1.66, i.e. visibly blurrier, silently, with
no LOD compensation. This is not a coincidence: the uncompensated run's `lod_factor`
is *identical* to the 512px baseline's (same target, same assumed texture size), so
it reaches the *identical* tile set and `screen_px` per tile — only the real texel
count differs, by exactly `(256/512)² = 0.25`, so `aggregate_ratio_uncompensated =
aggregate_ratio_512_baseline × 0.25` **exactly** (`test_wp4a_esri_texture_size_compensated_vs_uncompensated`
checks this to `< 1e-6` relative error — it is a provable-by-construction relation,
not a measured one).

After WP4/A, Esri correctly refines further out to compensate (256px tiles now
common at deeper zooms than 512px ones needed for the same ground coverage — 12 363
visible tiles at 256px vs 3 922 at 512px for the same 204 poses, 3.15× more, each a
quarter the texels), landing `aggregate_ratio` at **1.74** — close to Carto's 1.66,
the two styles now delivering comparable sharpness instead of Esri running at a
silent quarter of it. `test_lod_factor_scales_inversely_with_texture_size` pins the
underlying `lod_factor_for` relationship (halving `texture_size_px` doubles
`lod_factor`) at a second target and viewport away from this calibration point.

## WP4 / B — viewport and mode ladder

*Recorded 2026-09-13. `src/testing/lod/ladder.rs` re-labels the same 204 bench pose
geometries at other viewport/mode combinations (`bench_cells()` itself is frozen and
untouched). Measured with
`cargo test --release --lib lod::test_viewport_ladder:: -- --test-threads=1 --nocapture`,
`target_texel_ratio = 1.0`, `texture_size_px = 512` throughout — only viewport and
mode vary.*

| rung | tiles | aggregate_ratio | median | p5 | p95 | texture MiB | deepest z |
|---|--:|--:|--:|--:|--:|--:|--:|
| 1920x1080 Free (desktop default) | 3 922 | 1.6625 | 3.1610 | 0.2728 | 219.47 | 3922.0 | 20 |
| 1280x720 Free | 2 406 | 1.8411 | 2.7981 | 0.2275 | 274.40 | 2406.0 | 20 |
| S23 landscape (2340x1080) Free | 4 382 | 1.5835 | 3.0621 | 0.2469 | 208.59 | 4382.0 | 20 |
| S23 landscape Cockpit | 3 391 | 1.1822 | 1.6160 | 0.1330 | 240.44 | 3391.0 | 20 |
| S23 portrait (1080x2340) Free | 5 640 | 1.8361 | 5.1167 | 0.3766 | 362.58 | 5640.0 | 20 |
| S23 portrait Cockpit | 4 942 | 1.5939 | 3.1854 | 0.3631 | 313.35 | 4942.0 | 20 |

**What this confirms, and one thing it refutes.** `lod_factor_for` depends on
viewport *height* and mode's `fovy` — never width — which
`test_lod_factor_matches_hand_derivation_at_every_rung` checks bit-for-bit
(1280x720's `lod_factor` is exactly `desktop × 720/1080`; S23 landscape's is
*bit-identical* to the desktop default's, since both are 1080 tall and `Free`;
Cockpit vs Free at equal height differs by exactly `2·tan(fovy_free/2) /
2·tan(fovy_cockpit/2) ≈ 0.742`, i.e. Cockpit refines **~25.8 %** less in distance
terms — measurably different from `docs/pre-terrain-plan.md`'s original ~35 %
estimate, which this now supersedes with an exact figure).

`docs/pre-terrain-plan.md` WP3/3b's own text says *"the S23 should now ask for
meaningfully more detail than the desktop default"* — **true for portrait
(height 2340, aggregate_ratio 1.84 and 5 640 tiles vs the desktop's 3 922), false
for landscape** (height 1080, identical to desktop — 1.58 aggregate_ratio, and the
tile-count difference that does show up, 4 382 vs 3 922, comes entirely from the
wider frustum's aspect ratio letting more tiles pass culling, not from any change in
`lod_factor`). The formula is purely height-driven; it has no notion of physical
pixel density, so a phone held sideways at the same logical height as a desktop
window gets the desktop's detail level, not the phone's. Worth knowing before WP4/C
and D pick numbers that assume otherwise — the cockpit interior is normally viewed
in landscape, which is the orientation this formula does *not* sharpen.

Cockpit mode's lower `aggregate_ratio` at both S23 orientations (1.18 landscape,
1.59 portrait, both below their Free counterparts) is the wider Cockpit FOV working
as intended — fewer, coarser tiles for the same viewport, matching the plan's
"cockpit should refine less" expectation, now measured rather than assumed.

## WP4 / C — 3a at equal tile budget (measurement only — not adopted)

*Recorded 2026-09-13. Measured with
`cargo test --release --lib lod::test_wp4c_3a_equal_budget:: -- --test-threads=1 --nocapture`.
**This section reports a finding. It does not adopt anything** — no default changed,
`test_visible_set_digest_is_stable` was not re-pinned (it doesn't need to be: the
`LodDistanceMode` switch this measurement uses is off by default and unreachable
from `wgpu_state.rs`), and whether to spend the +70% tile cost 3a requires is a
product decision this document does not make. See WP4/D in
`docs/pre-terrain-plan.md` for what happens after that decision, whichever way it
goes.*

**The question**, per WP3's refutation and WP4's reframing: not "is box-distance a
free upgrade" (WP3 already answered that — no, it costs +70% tiles at equal
`target_texel_ratio`), but *"is box-distance at a higher `target_texel_ratio` better
than centre-distance at a lower one, at the **same** tile budget?"*

**Method.** `N` is centre-distance's own tile count at `target_texel_ratio = 1.0`
(3 922 — "today"). A bisection search
(`tune_target_for_tile_count`) finds the `target_texel_ratio` at which box-distance
also produces `≈ N` tiles: `0.42844`, landing at 3 918 tiles (−0.10% off `N`).
`test_box_distance_matches_wp3s_refutation_measurement_at_equal_target` first
confirms the `LodDistanceMode` plumbing itself reproduces WP3's +70% figure exactly
(+70.4% measured here vs. +70.5% in the WP3 refutation) before trusting anything
built on top of it.

| | centre-distance (today) | box-distance (equal budget) |
|---|---:|---:|
| `target_texel_ratio` | 1.00000 | 0.42844 |
| tiles | 3 922 | 3 918 |
| `aggregate_ratio` | 1.6625 | 1.6659 |
| p5 (blurry tail) | **0.2728** | **0.2003** |
| p25 | 0.8030 | 0.9892 |
| median | 3.1610 | 4.7787 |
| p95 (wasteful tail) | 219.47 | 375.86 |

**Finding: box-distance does not lift the blurry tail at equal budget — if
anything it is measurably worse.** `p5` is `0.2003` for box vs. `0.2728` for
centre: *lower*, not higher. The single worst 20 under-refined tiles are listed in
both variants' test output; **the 12 single worst ranks are bit-identical between
them**, both centre and box putting the single worst tile at exactly `z=20
x=550502 y=364500`, ratio `0.0135` — because that tile is already at `MAX_ZOOM =
20`, the hard subdivision ceiling neither distance metric can refine past. The
grazing-angle cases 3a exists to fix are real (WP3 measured the centre/box distance
ratio diverging without bound as the camera nears the box), but at *this* bench
pose set, the very worst offenders are already zoom-capped, not distance-starved —
so 3a has nothing left to spend its budget on there. What it *does* buy: `p25`
improves (`0.80 → 0.99`, genuinely closer to the "ideal" 1.0), at the cost of a
markedly worse `p95` (`219 → 376`) and median (`3.16 → 4.78`) — box-distance's
extra subdivision (visible in the per-zoom table: box shifts tiles from z=2,4-8,11
toward z=3,9-10,12-15) buys some mid-distribution improvement by making the
already-over-refined near-limb tail *more* over-refined, which is exactly the tail
WP5's fog work owns, not this one.

**Recommendation: do not adopt 3a as specified, at least not as a blanket
distance-metric swap.** It does not solve the problem it was proposed for (the
worst blurry tiles are zoom-capped, not distance-metric-limited) and it makes the
already-known-bad wasteful tail worse for a `p25` improvement that a uniformly
lower `target_texel_ratio` — the "simply lowering the ratio uniformly" comparison
WP4's own text asks for — might buy just as well without the redistribution cost.
That further comparison (box-distance's *shape* vs. a plain lower-ratio
centre-distance's *shape*, both at equal `N`) is the natural next measurement if
this finding needs more confidence before WP4/D acts on it, and is not included
here — this section reports what was asked for, not everything that could be asked
next.

## WP4 / E — does the default basemap actually serve z=20?

*Checked 2026-09-13, live against the real tile server, per WP5/E in
`docs/pre-terrain-plan.md`.* WP4/C found the 12 single worst under-refined tiles
bit-identical across both LOD distance metrics, all at `MAX_ZOOM = 20` — a zoom
ceiling, not an LOD-rule problem. The question worth checking cheaply: does
`standard_imagery_url()` (Carto `dark_nolabels` `@2x`) even have real content at
z=20, or is the engine dutifully refining toward data that doesn't exist?

Fetched the exact worst-case tile from the WP0-WP4 measurements, `z=20 x=550502
y=364500` (the zoom-cliff pose at `lat=48 lon=9 alt=10m`), plus its z=19 parent and
a z=20 neighbour:

| tile | HTTP | size | unique colours |
|---|---|---|---|
| z=19 (275251, 182250) — parent | 200 | 566 B | 6 |
| **z=20 (550502, 364500) — the worst tile itself** | 200 | **126 B** | **1** (flat `#090909`) |
| z=20 (550500, 364498) — a neighbour | 200 | 245 B | 6, but 99.8% one colour — anti-aliased oversampling of z=19's edges, not new content |

**The server returns `200 OK` at z=20, not `404` — but the worst-case tile is a
single flat colour, and its neighbour is a z=19 edge oversampled up, not genuine
z=20-resolution detail.** This style's real cartographic content tops out around
z=19; z=20 is served (so nothing breaks), but it is not *data*, so nothing the LOD
rule does can make that specific tile sharper — `p5`'s worst offenders are below
the data floor, not just below `MAX_ZOOM`. Worth knowing before spending more
effort chasing this specific tail: even a hypothetical `MAX_ZOOM = 21` would not
fix it for this basemap.

## WP5 — Fog

*Recorded 2026-09-13. `Stage::Fog` and `apply_lod`'s relaxation exist only in
`CullPipeline::DEFAULT_WITH_FOG`, which only `wgpu_state.rs` ever selects; every
number below comes from `src/testing/lod/fog_sweep.rs`'s standalone measurement
path (never `sweep::measure_pose_with_config`, which stays `CullPipeline::DEFAULT`
throughout — confirmed by the 204-pose CSVs staying byte-identical to the WP0-WP4
baseline and the culling gate staying 32/32 green with this landed). Measured with
`cargo test --release --lib lod::test_wp5_fog:: -- --test-threads=1 --nocapture`.*

### A/B — landed

`Stage::Fog` culls a tile outright when `cesium_fog(dist, density) >= 1.0`.
`apply_lod` relaxes `subdivide_dist *= (1 - cesium_fog(dist, density))`, derived
(not copied) from Cesium's `error -= fog·sse` — see `fog.rs` and `apply_lod`'s own
doc comments for the full derivation and why `FogConfig::sse` is ported but not
consumed by this relaxation. `CullPipeline::DEFAULT` (what the culling harness
builds) is untouched; `test_fog_stage_is_not_in_the_default_pipeline` pins that
directly, on top of the culling gate itself staying green.

### C — fog vs. the WP0-WP4 baseline

| population | poses | tiles (baseline → fogged) | Δtiles | aggregate | median | p95 (baseline → fogged) |
|---|--:|---|--:|---|---|---|
| all 204 poses | 204 | 3922 → 3579 | −8.7% | 1.663 → 1.431 | 3.161 → 2.314 | 219.47 → **107.14** |
| horizon poses (`pitch ≥ 85°`) | 44 | 1723 → 1519 | −11.8% | 5.781 → 4.648 | 12.209 → 7.265 | 312.32 → **162.01** |
| **cruise altitude (9-15km)** | 21 | 284 → 236 | **−16.9%** | 1.041 → 0.777 | 0.934 → 0.799 | 70.15 → **3.46** |

**Cruise altitude — this product's actual 10-12km operating envelope — sees the
largest effect on every axis**: the biggest tile-count reduction (−16.9%, vs
−8.7% overall) and by far the biggest `p95` collapse (70.15 → 3.46, a **95%**
reduction) of the three populations. `aggregate_ratio` at cruise also drops just
under 1.0 (1.041 → 0.777) — cruise poses were already close to the target
resolution before fog, so fog is now trimming genuinely wasteful over-refinement
there rather than fixing under-refinement. Horizon poses (pitch ≥ 85°, the
near-limb population `p95` is nominally about) see a smaller but still large `p95`
drop (312 → 162, −48%) and the largest relative tile-count cut among the three
(−11.8%… second to cruise's −16.9%). `Stage::Fog` and `apply_lod`'s relaxation
only ever remove or shrink — `test_wp5c_fog_vs_baseline` asserts fog never
increases tile count, texture bytes, or `p95` on any of the three populations,
and it holds everywhere measured.

Across the full WP4/B viewport/mode ladder the reduction holds throughout —
4-18% fewer tiles, `p95` roughly halved in most rungs (S23 portrait's is cut by
**82%**, 362.6 → 63.6, the deepest reduction of the six):

| rung | tiles (baseline → fogged) | Δtiles | p95 (baseline → fogged) |
|---|---|--:|---|
| 1920x1080 Free (desktop) | 3922 → 3579 | −8.7% | 219.47 → 107.14 |
| 1280x720 Free | 2406 → 2309 | −4.0% | 274.40 → 222.22 |
| S23 landscape Free | 4382 → 3952 | −9.8% | 208.59 → 101.93 |
| S23 landscape Cockpit | 3391 → 3074 | −9.3% | 240.44 → 122.51 |
| S23 portrait Free | 5640 → 4630 | −17.9% | 362.58 → **63.62** |
| S23 portrait Cockpit | 4942 → 4261 | −13.8% | 313.35 → 70.74 |

### D — 3a re-run post-fog (measurement only, still not adopted)

The scheduled re-run of WP4/C's equal-tile-budget comparison, now with fog active
for both variants — not a retry, since WP4/C's own finding predicted the direction
this should move: box-distance's WP4/C budget was spent almost entirely on the
near-limb tail fog now removes.

| | centre-distance @ target=1.0 (today) | box-distance @ equal budget |
|---|---|---|
| tiles | 3579 | 3582 (target=0.46929) |
| aggregate | 1.4309 | 1.4320 |
| p5 | 0.2391 | 0.2001 |
| p25 | 0.7422 | 0.7918 |
| median | 2.3139 | 3.1849 |
| p95 | 107.1398 | 139.9669 |

The predicted shrinkage happened, but did not evaporate. **`box/centre p95`
ratio: pre-fog 1.713× → post-fog 1.306×** — box-distance still makes the wasteful
tail meaningfully worse, just less so. **The `p25` gain shrank even more
sharply**: pre-fog box beat centre by +23.2% (0.9892 vs 0.8030); post-fog the gap
is +6.7% (0.7918 vs 0.7422) — most of 3a's one selling point in WP4/C was itself
sitting in the population fog now thins out. `p5` is essentially unchanged in
absolute terms and box is still not better than centre there (0.2001 vs 0.2391 —
box is *worse*, same as WP4/C found).

**Recommendation unchanged: do not adopt 3a.** Post-fog, its cost (a worse `p95`
tail, 3582 vs 3579 tiles to get there) buys even less than it did pre-fog (a `p25`
gain roughly a third the size), and it still does nothing for `p5`, which WP4/C
and WP4/E both traced to a `MAX_ZOOM`/data-floor ceiling neither distance metric
can touch. Fog was the right fix for `p95`; box-distance was not, before or after.

### B — hysteresis and the `maxHeight = 800km` boundary

`apply_lod` applies fog relaxation *before* deriving `collapse_dist =
subdivide_dist × 1.20`, so the hysteresis band stays exactly 20% of whatever the
(possibly fog-shrunk) threshold is, at any relaxation factor —
`test_wp5b_hysteresis_band_stays_proportional_under_fog` checks this algebraically
at five relaxation factors from `1.0` down to `0.01`.

**The `maxHeight` cutoff step is real — first mismeasured as absent, then
corrected.** `fog_density_for`'s cutoff is hard, per Cesium's own
`if (height > maxHeight): disabled` — density *at* `max_height_m` is `6.0×10⁻⁷/m`
(not `0`), so crossing the boundary drops it to exactly `0` in one step, not a
fade. A first pass measured this with an altitude ladder built from round numbers
(799,999 / 800,000 / 800,001m) and found **no step at all** — but `Camera::altitude()`
(what `fog_density_for` is actually driven by every frame) reads a few metres
*higher* than the analytic `alt_m` a pose is built from, so all three "round"
altitudes landed on the same side of the true cutoff without anyone noticing.
Locating the true crossing by bisecting on the camera's own altitude (rather than
assuming the nominal constant) and rebuilding the ladder around *that* found a
real step:

| view | tiles just below the true crossing | tiles just above | step |
|---|--:|--:|--:|
| nadir | 8 | 12 | **+50.0%** |
| grazing (pitch=80°) | 7 | 10 | **+42.9%** |

A headless capture at the corrected altitudes
(`src/testing/rendering/fog_capture.rs`, `11_just_below_crossing_799995m.png` vs
`12_just_above_crossing_799997m.png`) shows **no perceptible visual pop** despite
the step — at 800km the newly-un-fogged tiles are coarse (z=4-6) and either
peripheral to the framed nadir view or too far to read as a distinct change by
eye. The discontinuity is real in the underlying computation, not a measurement
artifact, but its practical consequence is bounded by two things: it requires a
camera literally at ~800km altitude, far outside this product's 10-12km cruise
envelope, and where measured, it did not translate into a visible pop.

## Housekeeping done alongside this baseline

- `culling-pipeline.html` (untracked at the repo root) was **deleted**, not
  committed. It diagrammed the pre-refactor five-step cascade and the old
  `BoxVerdict` enum name — both superseded by this branch's `Stage` /
  `CullPipeline` list — so keeping it under `docs/` would have shipped
  actively wrong documentation rather than merely stale clutter.
- `docs/pre-terrain-plan.md` was added: the 7-package plan this baseline is
  step WP0 of, copied in from the local (unversioned) plans directory so it
  sits next to the culling docs it depends on.
