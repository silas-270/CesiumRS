# Map Labels Subsystem

The Map Labels subsystem provides high-performance rendering of populated place markers (cities, towns, and capitals) directly onto the 3D globe. It leverages spatial partitioning, aggressive CPU-side culling, and efficient UI-batching to draw thousands of labels at 60+ FPS with zero GPU state changes or extra render passes.

---

## 1. Architecture Overview

### Data Storage & Format
The subsystem pre-compiles a subset of Natural Earth's populated places into a packed binary database (`populated_places.bin` included in `mod.rs`):
* **Zero Allocations:** Places are parsed directly as static byte slices.
* **Pre-sorted by Importance:** Labels are grouped and ordered by their visual scale rank.

### Spatial Partitioning
The globe is divided into a 2.5D grid of 648 cells (10° latitude × 10° longitude bins). Each cell stores:
* The subset of places within its boundaries.
* Pre-calculated bounding sphere center and radius.
* Index offsets pointing to LOD boundaries.

---

## 2. Performance Culling Pipeline

To achieve sub-millisecond culling times on the CPU over a database of ~30,000 cities, the update loop implements a multi-stage culling pipeline:

1. **Temporal Throttling:**
   Label updates are throttled to run at **10Hz** (once every 6 frames) or on-demand if the camera undergoes major translation or rotation.
2. **Coarse Hierarchical Frustum Check:**
   Each grid cell's bounding sphere is first tested against the camera's view frustum. If a cell is completely out of view, all of its labels are discarded instantly in one check.
3. **Horizon-Clutter & Distance-Rank Filter:**
   To prevent labels from stacking up at the earth's horizon, the engine scales the culling range with camera altitude:
   $$\text{max\_local\_dist} = \text{altitude} \times 1.5 + 0.15\text{ Megameters}$$
   If a place is further than this distance and its `label_rank` is greater than `2` (not a major capital or city), it is skipped.
4. **Branchless Horizon Culling:**
   Remaining places are checked against the earth's ellipsoid horizon using branchless vector math. This discards ~50% of candidate places (those on the far side of the planet) extremely efficiently.
5. **Precise Frustum Culling:**
   Places passing the horizon check are tested against the 6 frustum planes of the camera.

---

## 3. Visual Representation

Labels are projected into screen-space and rendered on the egui `Background` layer:
* **Pill Backdrop:** A dark, semi-transparent rounded rectangle (`rgba(8, 12, 18, alpha)`) is drawn behind text for high legibility on any satellite or terrain background.
* **Dynamic Proximity Scaling:** Label scale, text opacity, backdrop opacity, and dot radius are dynamically interpolated based on depth. Near labels appear larger and brighter; distant labels fade and shrink gracefully.
* **Anchor Dots:** A white pinprick dot with a dark halo marks the exact coordinate of the place.

---

## 4. API Configuration

The label system behavior can be adjusted dynamically via the `LabelManager` configuration properties:

| Property | Type | Default | Description |
| :--- | :---: | :---: | :--- |
| `enabled` | `bool` | `true` | Toggles the entire culling and rendering pipeline on/off. When disabled, zero CPU/GPU resources are consumed. |
| `size_scale` | `f32` | `1.0` | Multiplier for label font size, allowing overall label scaling. |
| `max_importance_rank` | `u8` | `15` | Filters out places with a `label_rank` greater than this value. (0: Capitals only, 15: Show all cities & towns). |
| `show_anchor_dots` | `bool` | `true` | Toggles rendering of the white 2D screen anchor dots. |

---

## 5. UI Controls

Controls for all these API parameters are exposed under the **"Map Labels Settings"** section in the **Flight Tracker Debug** UI window.
