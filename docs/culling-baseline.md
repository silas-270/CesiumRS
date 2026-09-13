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
`SATELLITE_IMAGERY_URL` serves 256×256 tiles, so this changes what the engine
actually renders for that style. Measured with
`cargo test --release --lib lod::test_lod_sweep::test_wp4a_esri_texture_size_compensated_vs_uncompensated -- --nocapture`,
same 204 bench poses, `target_texel_ratio = 1.0` throughout.*

| scenario | `lod_factor_for` assumes | real texels counted | `aggregate_ratio` | tiles | texture bytes |
|---|---|---|---|---|---|
| 512px baseline (Carto, `STANDARD_IMAGERY_URL`) | 512px | 512px | **1.6625** | 3 922 | 3922.0 MiB |
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

## Housekeeping done alongside this baseline

- `culling-pipeline.html` (untracked at the repo root) was **deleted**, not
  committed. It diagrammed the pre-refactor five-step cascade and the old
  `BoxVerdict` enum name — both superseded by this branch's `Stage` /
  `CullPipeline` list — so keeping it under `docs/` would have shipped
  actively wrong documentation rather than merely stale clutter.
- `docs/pre-terrain-plan.md` was added: the 7-package plan this baseline is
  step WP0 of, copied in from the local (unversioned) plans directory so it
  sits next to the culling docs it depends on.
