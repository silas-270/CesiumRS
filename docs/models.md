# Aircraft and cockpit models

How the two glTF models — the Airbus A350-1000 seen from outside and the Boeing 787 flight
deck seen from the seat — are loaded, scaled, placed on the flight path and drawn. The
lighting they receive is in [lighting.md](lighting.md#objects); the cameras that look at
them in [camera.md](camera.md).

Code: `crates/cesium-engine/src/render/model_pipeline/` (a generic glTF renderer),
`crates/cesium-flight/src/aircraft_model.rs`, `cockpit_model.rs`, `cockpit_screens.rs`,
`tracker.rs` (placement), `terrain_fit.rs` (ground contact). Licences:
[MODEL_LICENSES.md](../MODEL_LICENSES.md).

## One generic renderer, model quirks outside it

`ModelRenderer` loads a GLB, walks its node tree, bakes node transforms into one vertex
buffer and draws the whole model in **one draw call** with one texture. Everything
specific to a particular file — its axis convention, pivot, scale, which materials to
recolour or hide — is passed in through `ModelOptions` from the flight crate, so the
engine stays a generic glTF renderer:

| Option | A350 exterior | 787 cockpit |
|---|---|---|
| `normalize_to_unit_radius` | yes (then scaled with distance) | no — true metres |
| `cull_mode` | back faces | none (every material is double-sided and the camera is inside) |
| `skip_alpha_below` | 0 | 0.5 — the model is one unsorted depth-writing draw, so blended glass would occlude what is behind it |
| `material_override` | textured materials forced to white (below) | white placeholders tinted, display material to full value |
| `texture_override` | — | the procedurally painted display atlas |
| `uv_override` | — | parks seven other materials on a white texel |
| `material_unlit` | — | the displays are self-lit |
| `max_texture_size` | 1 024 | — |

Per draw, push constants carry the camera-relative model matrix, the viewport,
and the lighting dials (`ambient_override`, `specular_strength`, `detail_strength`,
`rim_strength`, `diffuse_weight`), plus `min_pixel_size` and `depth_bias`.

## The A350-1000 exterior

`assets/A350-1000.glb` is compiled into the binary. It is Y-up with **+Z forward** (nose at
z = +35.65 m, fin tip at −37.93 m), so it is yawed 180° into the aircraft frame
(`YAW_CORRECTION`). Its units are the real airframe's metres: 64.74 m span, 74.12 m long.

- **Pivot.** `ORIGIN_OFFSET_M` moves the model origin to 50 % of the span, 21.5 % up and
  55.8 % from tail to nose — the point the flight path holds and the aircraft rotates
  about. A wrong pivot swings the whole airframe around in a turn.
- **Scale.** The mesh is normalised to a bounding radius of 1 and then scaled by
  `POST_SCALE` (1.029): the uniform scale that best fits the bounding box the route line
  and the camera framing were tuned against.
- **Livery.** Both textured materials are authored with base colour 0.588 over a fully
  painted 2048² atlas, and the shader multiplies the two; they are forced to 1.0 so the
  livery shows as painted instead of at 59 %. The atlas is capped at 1 024²: rendered at
  both sizes, the tracking view is bit-identical across the light audit, and the cap saves
  about 16 MB of texture memory with mips.
- The file carries 137 mesh-less nodes (`nav_l`, `beacon`, `landing_r`, …), attachment
  points for lights; they cost nothing and are kept as exact positions for lights later.

### Size on screen

The aircraft is drawn larger than life when far away, so it stays legible as a symbol.
`FlightTrackerApp::airplane_model_matrix` sets the model's radius to
`0.008325 × distance to the camera`, clamped to 33.5 m … 1 000 km, and the vertex shader
enlarges it further until it covers at least `AIRCRAFT_MIN_PIXEL_SIZE` = 100 px.

### Height and ground contact

Airborne, the model origin sits 7.5 m above the flight position (the ribbon is 5 m up, so
the line runs under the belly). On the ground the gear rests on the terrain: the lift
blends to `gear depth × current scale + 0.2 m` by the terrain-fit weight, and is never
less than what keeps the lowest point above the ground under the aircraft, however large
the zoom has made the model.

`terrain_fit.rs` moves the flight onto the *drawn* ground near both airports. The planner
puts the aircraft at the published field elevation, which is the highest point of the
landing area, so the drawn runway is usually lower (12–16 m along the Frankfurt departure
roll, 20–35 m along the Stuttgart rollout). Near each end the profile altitude gains
`(G − field) · weight(height above field)`, with `weight` 1 up to 30 m above the field
and 0 from 600 m (smoothstep between). `G` is the ground under the flight while it is on
the ground and the ground under the lift-off or touchdown point once airborne, so an
airborne aircraft does not dip into every valley past the runway end. The aircraft
follows the fitted height with a 0.15 s time constant; the route line is fitted by the
same rule (`LineFit`), with control points every 10 m on the ground and every 100 m through
the fade, 1.5 m above the ground on the runway, and at most 64 re-sampled points a frame.
Fitting only the aircraft left the line hanging 20–39 m above an aircraft on the runway.

## The 787 flight deck

`Boeing787Cockpit.glb` is not compiled in: it is far larger than the exterior and most
sessions never enter cockpit mode. `cesium_flight::assets::load` reads it from
`assets/` on desktop and from the APK on Android, **the first time cockpit mode is
entered**; that frame pays the read and parse. A missing file is logged once and cockpit
mode then shows no interior.

- It is Y-up and **−Z forward**, the aircraft frame's own convention, in metres
  (3.06 × 2.04 × 3.03 m). It is drawn at true scale, `MODEL_SCALE` = 10⁻⁶ (metres to
  megametres).
- **Eye point.** `EYE_LOCAL_M` = (−0.55, 1.16, −1.28) m, measured off the captain's seat
  (cushion top at y = 0.383, seat centre-line x = −0.55) and checked against the HUD
  combiners, which land 0.31 m ahead at eye height. The model is placed so this point
  coincides with the cockpit camera's position in the aircraft frame
  (`CAMERA_LOCAL_MM`); placement is recomputed from the aircraft state, not from the
  camera, so the interior stays put under the debug God camera too.
- **Materials.** The file's only image is a 1×1 white pixel, so every textured material
  would resolve to white. `COCKPIT_TINTS` recolours materials authored white; with the
  current GLB every material is authored between 0.02 and 0.22 and the table is inert,
  kept as the mechanism for a model with white placeholders. The HUD combiners are
  flagged as blended with an opaque factor and would render as solid rectangles 30 cm in
  front of the eyes; the tint table gives them alpha 0 and `skip_alpha_below` drops them,
  along with the 10 % alpha window glass.

### The displays

The four forward displays are single quads whose UVs occupy four bands of one 0–1 space:
the GLB was authored against an atlas and lost the image in conversion.
`cockpit_screens::build_atlas` paints that missing image once at load — primary flight
displays outboard, navigation displays inboard — into a 3072×512 texture (each screen
gets 580×434 texels, square on the panel; about 8.4 MB with mips). Seven other materials
share the same UV space and would sample fragments of it; `uv_for_material` parks them on
a single white texel, where a constant UV also pins them to mip 0. The display material is
authored at 0.02 (a blank screen) and is lifted to full value so the painted panels
survive the multiply, and `unlit_for_material` marks the displays self-lit: they ignore
the flight deck's shading and keep their painted colours in daylight and at night. The
atlas is static; nothing updates per frame.

## Draw order

The route ribbon is part of the world layer (`GlobeExtension::render`); the aircraft or
the interior are drawn in `render_foreground`, **after** the city labels, so a model
nearer than a label's anchor covers the label and one farther away does not. See
[labels.md](labels.md).
