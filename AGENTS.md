# CesiumRS - Agent Rules & Context

**Project:** CesiumRS  
**Language:** Rust  
**Graphics API:** WebGPU (via `wgpu` crate)  
**Windowing/UI:** `winit`, `egui` (for debugging/UI)

---

## Agent Mandates & Workflow Rules

> [!IMPORTANT]
> **Git Commits are Mandatory**  
> You MUST commit your changes using `git` as soon as a feature or change is fully working as expected. Do not leave changes uncommitted.

> [!IMPORTANT]
> **No Testing Code in Main Directory**  
> All testing code, test files, and test helpers must be placed inside the `src/testing/` directory to keep the main application clean and free from test clutter. No test files or binaries should exist in `src/` or `src/bin/`.

---

## Technical Details & Quirks
*   **Projection:** Web Mercator (EPSG:3857). Max latitude is mathematically ~±85.05°. 
*   **Hole Fixing:** The poles are explicitly capped in `geometry.rs` by intercepting boundary tiles (`y == 0` or `y == (1<<z)-1`) and forcing the skirt vertices to exactly 90 or -90 degrees latitude.
*   **Coordinate System:** WGS84 Ellipsoid.
*   **Style:** No logic mixed between modules. If adding features, isolate them in `src/io/` or equivalent new modules. Prioritize modularity. Do not lump unrelated logic into single files. Use targeted CLI tools and `grep_search`.

---

## Visual Testing & Verification
> [!IMPORTANT]
> If you make any changes that could affect rendering, geometry, camera positioning, or UI, you MUST verify your changes using the headless test mode.

You can take a screenshot of the engine state from a specific coordinate using the following CLI command:
```bash
cargo run -- --verify --cam-x 0.0 --cam-y 0.0 --cam-z 8.0 --out test.png
```

To test the interaction layer, you can simulate raw user mouse inputs over a sequence of frames by passing the `--actions` argument:
```bash
cargo run -- --verify --cam-x 0.0 --cam-y 0.0 --cam-z 8.0 --actions "drag:0,0->500,500:20;wait:5" --out test_drag.png
```

### Action Syntax:
*   `drag:x1,y1->x2,y2:frames` - Simulates a mouse drag from pixel (x1, y1) to (x2, y2) over the specified number of frames.
*   `wait:frames` - Idles for a number of frames.

Once the command finishes, use your `view_file` tool on the output `.png` to visually inspect the globe and ensure your fix works correctly and no visual artifacts were introduced.
