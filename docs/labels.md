# City labels

Which place names are shown, where they stand, how they look, and how they are drawn
among the other things in the scene. Code: `crates/cesium-engine/src/label/`
(selection and style) and `render/label_pipeline/` (text and drawing).

## The database

`label/populated_places.bin` is compiled into the engine. Its format (`LabelDatabase::load`):

| Bytes | Content |
|---|---|
| 0–3 | magic `CLBL` |
| 4–7 | version, `1` |
| 8–11 | record count |
| 12–75 | sixteen `u32` offsets: the number of records with `scale_rank ≤ z`, for z = 0…15 |
| 76… | 32-byte `PackedLabel` records: ECEF position and normal (f32, megametres), name offset and length, `scale_rank`, `label_rank` |
| rest | UTF-8 string table |

Records are sorted by `scale_rank`, the zoom at which a place starts to be shown, so "every
label up to zoom z" is a prefix of the array. `label_rank` is its importance (0 for
capitals). At start-up the records are bucketed into a 10°×10° grid, 648 cells, each with
its own sorted list and prefix offsets and a bounding sphere.

## Selection

`LabelManager::update` runs in `update_logic` after the quadtree, using the same
camera-relative `Frustum`:

1. **Zoom bucket** from altitude: `z = clamp(4 − log₂(altitude in Mm), 0, 15)`, so each
   halving of altitude admits one more `scale_rank`.
2. **Grid cells** whose bounding sphere misses the frustum are skipped.
3. Per candidate: drop it if its `label_rank` exceeds the debug panel's rank limit; drop
   ranks above 2 farther than `1.5 × altitude + 150 km` (so the horizon does not fill with
   small towns); drop it if it is **behind the Earth's limb** (the exact point test,
   Theorem 3.1 of [culling-math.md](culling-math.md#31-exact-occlusion-of-a-point), in
   f64 — labels are points that need not lie on the ellipsoid, so the tile test's
   surface-point shortcut does not apply); drop it if it is outside the four side planes.
4. **Terrain lift.** A survivor is moved up along the ellipsoid normal by the height of the
   *drawn* surface under it (`TileSystem::drawn_ground_height_at`, the triangle net at the
   level it was drawn last frame), so a label stands on the ground that is visible rather
   than inside it — Denver's sat 1.6 km underground before. The lift is applied only to
   labels that already passed every test, and only with terrain on; the selection itself is
   computed on the ellipsoid position and is identical with terrain on and off.

The list is recomputed at most every six frames unless the camera moves more than about
10 km or turns; a stationary camera does not re-run the loop.

## Style

`label/style.rs` decides each label's look from its distance and rank, in points
(multiplied by the window's pixels-per-point when drawn):

- **Range** is `1.5 × altitude + 150 km` for ordinary labels and the horizon distance
  (at least that range) for ranks 0–2. Relative distance across `[altitude, range]` drives
  everything below.
- **Size** 9 pt far to 14 pt near, ×1.3 for ranks 0–2 and ×0.82 above rank 5, times the
  user's size scale.
- **Fade**: full opacity over the nearest 35 % of the range, smoothstep to zero by 95 %.
  Text alpha 180–255, pill alpha 100–170, both times the fade; a label under 3/255 on
  both is dropped.
- A dark pill (sRGB 8, 12, 18) with 3 pt corner radius behind the text, and an anchor dot
  with a soft shadow, 1.5–3 pt radius, 3 pt below the pill.

## Drawing

`LabelRenderer` draws the labels inside the scene render pass as instanced screen-space
quads hung off a 3D anchor — one instance per pill, per glyph and per dot, a four-vertex
triangle strip each. The rules it implements, in every camera mode:

1. Labels cover the whole world layer: globe, sky, route ribbon, debug geometry.
2. Each quad keeps its anchor's clip-space depth, and the pipeline writes that depth with
   compare `Always`. So the aircraft or cockpit drawn *after* the labels
   (`GlobeExtension::render_foreground`) covers a label exactly where the model is nearer
   than the anchor, and is covered by it where it is farther.
3. Labels are sorted far to near, so a nearer label covers a farther one.
4. A label whose anchor is behind the eye or outside the viewport is not drawn at all.
5. Fully transparent texels (pill corners, gaps between glyphs) are discarded so they write
   no depth.

**Text** matches what egui drew before labels moved into the scene: Ubuntu Light (egui's
default proportional face, from `epaint_default_fonts`), laid out with egui's metrics and
pair kerning (`layout.rs`, cached per name and pixel size), each glyph rasterised with
`ab_glyph` at the exact integer pixel size it is drawn at and placed on whole pixels, the
anchor snapped to a whole pixel in the vertex shader. A single large raster minified
through mips would blur the strokes and, after egui's coverage gamma, read visibly bolder.
Coverage goes through the same gamma, and colour is premultiplied in gamma space before
the sRGB decode, as egui-wgpu does.

The **glyph atlas** (`atlas.rs`) is 1 024², keyed by `(character, pixel size)`, filled on
demand; when it is full it is emptied and the frame's instances are rebuilt once. A
screenful of names always fits an empty atlas, so a second reset within one frame cannot
happen.

Headless route renders switch labels off (`run_headless_render`); the debug panel exposes
the enable switch, the size scale, the rank limit and the anchor dots.
