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

## Housekeeping done alongside this baseline

- `culling-pipeline.html` (untracked at the repo root) was **deleted**, not
  committed. It diagrammed the pre-refactor five-step cascade and the old
  `BoxVerdict` enum name — both superseded by this branch's `Stage` /
  `CullPipeline` list — so keeping it under `docs/` would have shipped
  actively wrong documentation rather than merely stale clutter.
- `docs/pre-terrain-plan.md` was added: the 7-package plan this baseline is
  step WP0 of, copied in from the local (unversioned) plans directory so it
  sits next to the culling docs it depends on.
