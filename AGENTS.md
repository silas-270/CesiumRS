# CesiumRS - Agent Context

**Project:** CesiumRS
**Type:** 3D virtual globe / terrain renderer
**Language:** Rust
**Graphics API:** WebGPU (via `wgpu` crate)
**Windowing/UI:** `winit`, `egui` (for debugging/UI)

## Architecture Overview
Modular structure designed for scalability and future caching implementations.

*   **`src/core/`**: App entry point and window lifecycle management (`winit` EventLoop integration).
    *   `app.rs`: Main `App` struct implementing `winit::application::ApplicationHandler`.
*   **`src/camera/`**: View/Projection logic.
    *   `camera.rs`: Contains `Camera` struct, handles view/projection matrices, global transforms, frustum calculations.
*   **`src/globe/`**: Core terrain generation and LOD mechanics.
    *   `geometry.rs`: `TileMesh` generation. Creates vertex grids for tiles. **Important:** Implements "skirts" (edges pointing inwards to center of globe) to hide LOD cracks. Implements "integrated pole caps" (collapsing skirt rows at lat = ±90.0) to plug holes inherent to Web Mercator.
    *   `quadtree.rs`: LOD quadtree management (`QuadtreeManager`, `QuadtreeNode`). Handles Web Mercator projection math (`web_mercator_y_to_lat`). Culls nodes outside camera frustum.
*   **`src/io/`**: Networking and I/O.
    *   `texture_manager.rs`: Fetches map tiles via HTTP. Prepares tiles for GPU upload. Setup for future local caching.
*   **`src/render/`**: Graphics pipeline and GPU state.
    *   `wgpu_state.rs`: `WgpuState` struct. Manages `wgpu` device, queue, surface, render passes, buffers, and shader bindings. Integrates `egui`.
    *   `shader.wgsl`: WGSL shader code for rendering the globe.

## Technical Details / Quirks
*   **Projection:** Web Mercator (EPSG:3857). Max latitude is mathematically ~±85.05°. 
*   **Hole Fixing:** The poles are explicitly capped in `geometry.rs` by intercepting boundary tiles (`y == 0` or `y == (1<<z)-1`) and forcing the skirt vertices to exactly 90 or -90 degrees latitude.
*   **Coordinate System:** WGS84 Ellipsoid.
*   **Style:** No logic mixed between modules. If adding features (e.g., caching), isolate them in `src/io/` or equivalent new modules.
*   **Agent Instructions:** Prioritize modularity. Do not lump unrelated logic into single files. Use targeted CLI tools and `grep_search`.
