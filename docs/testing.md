# Testing

What is tested, how, and how to run it. The short version is in the top-level README;
this document explains the structure, because the engine's tests are not one kind of
thing.

## Four kinds of test

| Kind | Where | Asserts? | Needs |
|---|---|---|---|
| **Unit and property tests** | `crates/*/src` (`#[cfg(test)]`), `crates/cesium-flight/tests/flight_plan.rs`, most of `src/testing/{camera,flight,tiles,terrain}` | yes | nothing |
| **Measuring instruments** | `src/testing/culling/`, `src/testing/lod/`, parts of `src/testing/terrain/` | the culling gate and terrain soundness tests assert; the rest report | CPU; many cores help |
| **Capture harnesses** | `src/testing/rendering/` | no — they write PNGs and print numbers | a GPU, usually network |
| **Interactive harness apps** | `src/testing/harness/`, `profiling/`, `benchmark.rs`, selected with `cesium_app` flags | no | a window |

`src/testing/` is compiled only with the `testing` feature (on by default) and only on
desktop. The engine's tests live there rather than in the engine crate because most of
them need the root crate's API, the flight extension, or both.

## Flight planning

```bash
cargo test -p cesium-flight
```

Pure computation, no GPU, no network. `crates/cesium-flight/tests/flight_plan.rs` pins the
*shape* of a plan — great-circle routes that do not go the wrong way round the world,
closed airspace avoided at a cost, airliner climb and descent angles, flight levels of
the right parity, step climbs, physical cruise speeds, a flight that lasts exactly the
session, nose-up on final, touchdown past the threshold — rather than its constants, so
retuning the aircraft or wind model does not mean rewriting the tests. The acceleration
audit over fifteen routes (`src/testing/flight/test_multi_route_suite.rs`) and the
route-line distance tests are in the root crate. See [flight-plan.md](flight-plan.md#testing).

## The culling gate

```bash
cargo test --release --lib culling:: -- --test-threads=1 --nocapture
cargo test --release --lib culling::bench -- --ignored --test-threads=1 --nocapture   # latency + memory
```

`src/testing/culling/` measures the visible set against an **independent f64 oracle**
(`oracle.rs`: is this surface point visible from this camera, computed without any of the
engine's code) and an exact convex-hull intersection (`sat.rs`), over nine sweeps of
camera poses from a 2 m taxi to 30 000 km, nadir to horizon, every aspect ratio. False
negatives (a visible point no drawn tile covers) must be zero everywhere; false positives
are reported and budgeted. It is the gate for any change under `globe/quadtree/`. The
FN/FP definitions, sampling densities, the frozen ground truth and the visible-set digest
are in [culling-implementation.md](culling-implementation.md#8-how-to-measure).

## The LOD harness

```bash
cargo test --release --lib lod:: -- --test-threads=1 --nocapture
```

`src/testing/lod/` scores every visible tile at the 204 bench poses by imagery texels per
screen pixel and, for the terrain arm, by projected geometric error, and writes CSVs to
`$TMPDIR/cesium_lod_harness/`. It reports rather than asserts; the few assertions pin
properties such as the exact `lod_factor = 2.0` calibration and the flat globe leaving no
geometric error on screen. See [tiles-and-lod.md](tiles-and-lod.md#measuring-lod).

## Terrain

```bash
cargo test --release --lib terrain::
```

The decoder is pinned against five committed Terrarium tiles
(`assets/terrain_fixtures/`); the inheritance margins are re-derived from the committed
788-tile extrema corpus; the height-aware limb and occlusion stages are held to zero false
negatives against the drawn mesh; mesh lifetime, the rebuild budget, the ground reference
queries and the runway corridor have their own files. None of it touches the network.
See [terrain.md](terrain.md#testing).

## Capture harnesses

Almost everything in `src/testing/rendering/` renders headlessly with `WgpuState` and
writes PNGs, and is `#[ignore]`d because it needs a GPU and usually network imagery.
Rendering changes are verified by looking at these, and by sampling pixels: judged by eye,
dim surfaces read far brighter than they measure.

| Harness | What it shows |
|---|---|
| `light_audit` | the flight view across progress × camera mode; dark versus satellite imagery |
| `sunset_capture` | six fixed views at six sun elevations, for tuning twilight |
| `haze_capture` | a nadir ladder from 10 km to 30 000 km plus grazing views; the ground must stay visible |
| `sky_perf` | GPU timestamps around the scene, for comparing two builds within 1 % |
| `terrain_capture`, `terrain_*_capture`, `terrain_step_capture` | real-DEM poses: relief, bounds, LOD, rebuilds, occlusion |
| `terrain_balance` | frame time with and without culling behind mountains at the same pose |
| `terrain_rapid_pan` | streaming cost of fast camera movement over terrain |
| `route_line_ground` | the route line against the terrain on the ground phases |
| `label_capture` | label placement and occlusion by the aircraft |
| `fog_capture`, `culling_visual` | the visible set, drawn |
| `offline_switch`, `headless_offline` | the offline map, including switching to it at run time |
| `cockpit_capture`, `cockpit_s23` | the flight deck, at desktop and phone resolutions (also `cesium_app --cockpit`, `--cockpit-s23`) |

Run one with, for example:

```bash
cargo test --release --lib haze_capture -- --ignored --nocapture
```

Each file's module comment names its output directory and environment variables.

## Harness apps

`cesium_app` (the default binary) accepts flags that run a windowed harness instead of the
viewer (`src/lib.rs::run`, `src/main.rs`):

| Flag | App | Purpose |
|---|---|---|
| `--verify [--actions …] [--out …]` | `TestApp` | scripted drags, scrolls and waits (`Simulator`), then a screenshot once the tiles settle |
| `--regression` | `RegressionApp` | a fixed list of poses, hashed with SHA-256 |
| `--stress [--stress-mode …]` | `StressApp` | fast camera motion, requested/missing tile counts to `stress_results_<mode>.csv` |
| `--flicker`, `--monitor` | | tile flicker in Tracking mode; a live tile-state monitor |
| `--profile`, `--benchmark` | `PerfSimulatorApp`, `BenchmarkApp` | scripted Free/Cockpit/Tracking segments with per-subsystem timings |
| `--collide`, `--revisit`, `--tracking-orbit` | | the camera driven into the ground at Innsbruck; a repeated orbit (cache churn); a tracking orbit capture |
| `--render-hub`, `--routes-test` | | the headless route renderer from the command line |

`CESIUM_TRACE=1` adds the per-frame camera trace and the tile lifecycle trace to any
windowed run; `tools/camera_jumps.py` and `tools/tile_churn.py` analyse them.
`tools/phone_soak.sh` drives a long on-device run.

## Shaders

WGSL is compiled when a pipeline is created, so a shader error does not fail `cargo build`;
it panics when the pipeline is built. Any headless test that creates a `WgpuState` catches
it on desktop, but a shader should also be validated against naga for the wgpu version in
use before it is trusted on a device.
