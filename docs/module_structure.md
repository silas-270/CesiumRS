# Proposed Module Structure

This document defines the target module layout for CesiumRS after restructuring. The primary goal is:

1. **Core Engine is self-contained** — the `engine/` tree compiles and runs without any knowledge of `flight/`.
2. **Flight is a pure plugin** — it only consumes `engine/` public APIs. Remove the `flight/` folder and nothing in `engine/` breaks.
3. **Self-documenting paths** — any developer can infer file contents from its directory path + filename alone.

---

## Design Principles

- A folder describes **domain** (what area of the problem it addresses).
- A file describes a **concrete thing** (one struct, one pipeline, one system — not a catch-all).
- No file is named `mod.rs` at the leaf level when there is only one logical concept in it. `mod.rs` is reserved for re-export/glue only.
- Testing files are organized by **what they test**, not randomly lumped together.

---

## Proposed Tree

```
src/
+-- lib.rs                        # Crate root — platform entry points (android_main, run)
+-- main.rs                       # Desktop binary entry point — CLI parsing only
¦
+-- engine/                       # Self-contained globe rendering engine
¦   +-- mod.rs                    # Re-exports the engine public surface
¦   ¦
¦   +-- core/                     # Engine lifecycle and extension contract
¦   ¦   +-- mod.rs
¦   ¦   +-- app.rs                # GlobeApp — main event loop, orchestrates all systems
¦   ¦   +-- extension.rs          # GlobeExtension trait — plugin contract for Flight etc.
¦   ¦
¦   +-- camera/                   # All camera math and interaction modes
¦   ¦   +-- mod.rs
¦   ¦   +-- camera.rs             # GlobeCamera — view/projection, orbit, zoom, drag
¦   ¦
¦   +-- globe/                    # Earth geometry, tile hierarchy, terrain data
¦   ¦   +-- mod.rs
¦   ¦   +-- geometry.rs           # WGS84/ECEF/Mercator coordinate conversion utilities
¦   ¦   +-- quadtree.rs           # QuadtreeNode, Frustum, OBB culling
¦   ¦   +-- terrain_parser.rs     # Quantized-mesh binary format parser
¦   ¦   +-- tiles/                # Tile streaming pipeline (fetch -> decode -> upload)
¦   ¦       +-- mod.rs
¦   ¦       +-- config.rs         # TileSystemConfig — cache sizes, fetch limits
¦   ¦       +-- tile_cache.rs     # In-memory tile cache with negative-cache support
¦   ¦       +-- tile_fetcher.rs   # Async HTTP tile fetch with priority queue
¦   ¦       +-- mesh_worker.rs    # Background thread: decode terrain mesh
¦   ¦       +-- texture_manager.rs# GPU texture atlas — upload, evict, fallback
¦   ¦       +-- system.rs         # TileSystem — top-level coordinator for the above
¦   ¦
¦   +-- render/                   # All wgpu rendering — pipelines, buffers, state
¦   ¦   +-- mod.rs
¦   ¦   +-- wgpu_state.rs         # WgpuState — device, queue, per-frame rendering logic
¦   ¦   +-- capture.rs            # Off-screen render-to-texture and PNG screenshot export
¦   ¦   +-- debug_geometry.rs     # DebugVertex, frustum wireframe and crosshair helpers
¦   ¦   +-- globe_pipeline/       # Terrain tile rendering pipeline
¦   ¦   ¦   +-- mod.rs
¦   ¦   ¦   +-- pipeline.rs       # Tile render pipeline — shaders, bind groups, draw calls
¦   ¦   ¦   +-- shader.wgsl       # Tile vertex + fragment shader
¦   ¦   +-- sky_pipeline/         # Atmospheric scattering / sky dome pipeline
¦   ¦   ¦   +-- mod.rs
¦   ¦   ¦   +-- sky.wgsl          # Sky vertex + fragment shader
¦   ¦   +-- model_pipeline/       # glTF 3D model rendering pipeline
¦   ¦   ¦   +-- mod.rs
¦   ¦   ¦   +-- pipeline.rs       # ModelRenderer — load glTF, GPU buffers, draw
¦   ¦   ¦   +-- shader.wgsl       # Model vertex + fragment shader
¦   ¦   +-- polyline_pipeline/    # Great-circle polyline rendering pipeline
¦   ¦       +-- mod.rs
¦   ¦       +-- builder.rs        # AdaptiveSubdivisionBuilder — densify position samples
¦   ¦       +-- bvh.rs            # PolylineBVH — spatial culling for polyline segments
¦   ¦       +-- pipeline.rs       # PolylineRenderer — GPU buffers and draw calls
¦   ¦       +-- shader.wgsl       # Polyline vertex + fragment shader
¦   ¦
¦   +-- math/                     # Pure math utilities — no engine state, no wgpu
¦   ¦   +-- mod.rs
¦   ¦   +-- interpolation.rs      # Hermite, Catmull-Rom curve interpolation
¦   ¦   +-- trajectory.rs         # TransformState, spline sampling along a flight path
¦   ¦   +-- transform.rs          # ENU frame, surface tangent basis, ECEF transforms
¦   ¦
¦   +-- property/                 # Time-sampled scalar and positional property system
¦   ¦   +-- mod.rs                # Property trait definition
¦   ¦   +-- sampled.rs            # SampledPositionProperty, SampledScalarProperty
¦   ¦
¦   +-- time/                     # Simulation time and clock abstraction
¦       +-- mod.rs                # SimulationTime, Clock
¦
+-- flight/                       # Flight tracker plugin — depends ONLY on engine::*
¦   +-- mod.rs                    # Re-exports FlightTrackerApp and helpers
¦   +-- tracker.rs                # FlightTrackerApp — implements GlobeExtension
¦   +-- loader.rs                 # load_flight_data — parse JSON waypoints into properties
¦   +-- camera_modes/             # Flight-specific camera behaviours
¦       +-- mod.rs
¦       +-- tracking.rs           # Tracking mode — camera follows the aircraft
¦       +-- cockpit.rs            # Cockpit mode — first-person, locked to fuselage
¦       +-- free.rs               # Free mode — user-controlled orbit while flight plays
¦
+-- testing/                      # All headless test apps and unit tests (never in src/)
    +-- mod.rs                    # Declares all test modules, exports VerifyConfig
    +-- harness/                  # Shared test infrastructure
    ¦   +-- mod.rs
    ¦   +-- simulator.rs          # Simulator — replay action scripts (drag, wait, ...)
    ¦   +-- regression_app.rs     # RegressionApp — runs a scenario, captures PNG output
    ¦   +-- stress_app.rs         # StressApp — high-load sustained rendering
    ¦   +-- test_app.rs           # TestApp — general single-scenario test runner
    +-- camera/                   # Camera behaviour tests
    ¦   +-- test_drag_zoom.rs     # Drag and zoom gesture correctness
    ¦   +-- test_tracking_camera.rs # Tracking-mode camera orbit and zoom
    ¦   +-- test_z_sweep.rs       # Altitude sweep, LOD transition smoothness
    +-- culling/                  # Frustum and visibility culling tests
    ¦   +-- test_frustum_coverage.rs # Frustum coverage equivalence + fuzz
    ¦   +-- test_culling_false_negatives.rs # Tiles incorrectly culled as invisible
    ¦   +-- test_obb_debug.rs     # OBB visualisation and boundary correctness
    +-- tiles/                    # Tile streaming pipeline tests
    ¦   +-- test_tile_cache.rs    # Cache insert, evict, negative-cache behaviour
    ¦   +-- test_tile_fetcher.rs  # Fetch priority ordering, invalid tile handling
    ¦   +-- test_tile_system_stress.rs # Throughput, eviction under load
    ¦   +-- test_mesh_worker.rs   # Worker deduplication and concurrency
    ¦   +-- tile_system_tests.rs  # Fallback UV computation unit tests
    +-- terrain/                  # Terrain parsing and geometry tests
    ¦   +-- test_terrain_parser.rs # Quantized-mesh parse validity
    ¦   +-- test_parametric_sweeps.rs # Parametric sweep over geometry parameters
    +-- flight/                   # Flight-layer specific tests
    ¦   +-- test_flight_parser.rs # JSON waypoint parse correctness
    ¦   +-- test_trajectory_alignment.rs # Plane tangent/delta alignment math
    ¦   +-- test_entity.rs        # Property sampling, ENU velocity, interpolation
    +-- rendering/                # Visual regression and rendering tests
    ¦   +-- test_flicker_tracking.rs # Tile flicker during camera tracking
    ¦   +-- test_tile_monitor.rs  # Tile load/unload monitor over a flight
    ¦   +-- test_20_tiles.rs      # Exactly 20 tiles visible at reference pose
    ¦   +-- test_all_altitudes.rs # LOD correctness across altitude range
    ¦   +-- test_high_alt.rs      # High-altitude rendering, pole geometry
    ¦   +-- test_point.rs         # Single-point camera pose rendering
    +-- misc/                     # Miscellaneous / utility tests
        +-- test_glam.rs          # glam projection matrix sanity check
        +-- test_winit.rs         # winit event loop bootstrap smoke test
```

---

## Key Structural Changes vs. Current Layout

| Area | Current | Proposed | Reason |
| :--- | :--- | :--- | :--- |
| `engine/render/pipelines.rs` | Single file for globe + sky pipeline | Split into `globe_pipeline/` and `sky_pipeline/` subdirs | Each pipeline is a distinct, large concept |
| `engine/render/model/pipeline.rs` | Named `pipeline.rs` inside `model/` | Renamed to `model_pipeline/` at the same render level | Consistent naming with other pipelines |
| `engine/render/polyline/` | Pipeline, builder, BVH mixed | Renamed to `polyline_pipeline/` for consistency | Uniform naming across all pipelines |
| `flight/app.rs` | One monolithic file with tracker + loader + mode dispatch | Split into `tracker.rs` and `loader.rs` | Single responsibility per file |
| `flight/modes/` | `modes/` folder | Renamed to `camera_modes/` | Immediately obvious what the folder contains |
| `testing/` | 30 files flat in one folder | Grouped into `harness/`, `camera/`, `culling/`, `tiles/`, `terrain/`, `flight/`, `rendering/`, `misc/` | Self-documenting by domain |
| `engine/entity/` | Exists as near-empty module | Merged into `engine/property/` or removed | Avoids empty/near-empty directories |

---

## Separation Guarantee: Engine vs. Flight

The `engine/` tree defines only one point of integration with any plugin:

```
engine::core::extension::GlobeExtension  (trait)
```

The `flight/` tree implements this trait inside `flight::tracker::FlightTrackerApp`.

Deleting the entire `flight/` folder leaves the engine compiling cleanly because:
- No file in `engine/` imports from `flight::`.
- `flight/` is only wired in at the binary level (`main.rs` / `lib.rs`).
