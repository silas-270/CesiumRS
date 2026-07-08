# Large File Split Analysis

Files currently at 400+ lines and whether/how they should be split.

---

## 1. `src/engine/render/wgpu_state.rs` — 778 lines ❌ SPLIT

**What the name implies:** GPU device/queue state holder.  
**What it actually contains:** GPU state *and* tile display logic *and* camera uniform management *and* egui rendering *and* screenshot capture *and* the entire per-frame render loop.

This is the most overloaded file in the project. It should be split into:

| New File | Content | Justification |
| :--- | :--- | :--- |
| `wgpu_state.rs` | `WgpuState` struct + device/queue init + resize + buffer creation | Name matches: pure GPU state |
| `tile_display.rs` | `TileBuffers`, `TilePushConstants`, `TileDisplayEntry`, `update_display_state()`, `update_logic()` | Tile display bookkeeping is a distinct concern |
| `camera_uniform.rs` | `CameraUniform` struct + `update_matrix()` | Manages the GPU-side camera buffer — one focused struct |
| `render_loop.rs` | `render()`, `render_scene()`, `render_egui()`, `execute_egui()`, `compute_debug_vertices()` | The actual frame rendering pipeline — clearly named |

`capture.rs` already exists as a separate file (✅ done correctly). `debug_geometry.rs` too (✅).

---

## 2. `src/engine/camera/camera.rs` — 573 lines ⚠️ SPLIT (partial)

**What the name implies:** One camera.  
**What it actually contains:** Two completely different cameras — `Camera` (globe orbit camera with ellipsoid intersection, drag, projection) and `GodCamera` (a free-fly debug camera with WASD movement).

These are separate structs with no shared logic at all. They should live in separate files:

| New File | Content | Justification |
| :--- | :--- | :--- |
| `camera.rs` | `Camera`, `CameraMode` — the main globe orbit camera | Matches name exactly |
| `god_camera.rs` | `GodCamera` — free-fly debug/dev camera | It's a distinct camera type with distinct behaviour |

The `camera/mod.rs` re-exports both.

---

## 3. `src/engine/globe/quadtree.rs` — 503 lines ⚠️ SPLIT (partial)

**What the name implies:** Quadtree node structure.  
**What it actually contains:** Quadtree nodes + `TileId` + `Frustum` + `OrientedBoundingBox` + horizon culling math + `QuadtreeManager`.

The culling geometry types (`Frustum`, `OrientedBoundingBox`) and the horizon culling math are clearly separate concerns from the quadtree data structure:

| New File | Content | Justification |
| :--- | :--- | :--- |
| `tile_id.rs` | `TileId` + `web_mercator_y_to_lat()` | Tile addressing is its own concept |
| `bounding_volume.rs` | `OrientedBoundingBox`, `Frustum`, `get_tile_corner()`, `transform_to_scaled_space()`, `compute_horizon_culling_point()`, `compute_bounding_volume()`, `compute_sub_obb()` | All bounding volume / culling geometry — one clear concept |
| `quadtree.rs` | `QuadtreeNode`, `QuadtreeManager` + subdivision + LOD update | Now purely the tree structure and traversal |

---

## 4. `src/testing/rendering/test_tile_monitor.rs` — 478 lines ✅ KEEP

**What the name implies:** A test app that monitors tile visibility during a flight.  
**What it actually contains:** Exactly that — `TileMonitorApp`, `TileDisplaySnapshot`, `TileVisibilityHistory`, `TileEventBucket`. All types are internal to the test and exist to support tile monitoring analysis.

This is a test file, not production code. Its complexity is justified — it needs supporting structs to do detailed tile event tracking. No split needed.

---

## Summary

| File | Action | Reason |
| :--- | :--- | :--- |
| `wgpu_state.rs` (778 lines) | **Split into 4** | Mixes GPU init, tile display, camera uniform, and render loop |
| `camera/camera.rs` (573 lines) | **Split into 2** | Two completely different camera types in one file |
| `globe/quadtree.rs` (503 lines) | **Split into 3** | Mixes tile IDs, bounding volumes, and the actual quadtree |
| `rendering/test_tile_monitor.rs` (478 lines) | **Keep as-is** | Test file; all internal structs serve one test purpose |
