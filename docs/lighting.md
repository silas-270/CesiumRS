# Lighting and sky

The flight view lights itself from one number: **how deep into the flight the aircraft
is**. There is no clock anywhere in it.

## Why it works that way

CesiumRS renders for **FocusFlight**, a productivity app. A session runs for hours in the
background, which rules out shadow maps, per-pixel scattering and any full-screen post
pass — all the power, for an effect nobody is looking directly at.

More importantly, the view is telling a story about the session rather than about the
world. Climbing to cruise is the metaphor for descending into deep focus; the descent is
waking up and the light coming back. So the whole arc is driven by the altitude scalar,
which is symmetric over the flight by construction:

| depth | altitude      | sky                                    |
|-------|---------------|----------------------------------------|
| 1.0   | on the ground | high sun, bright blue                  |
| ~0.55 | climb/descent | sun on the horizon, orange             |
| 0.0   | cruise        | sun down, grey and dark, moon and stars|

Departure and arrival are bit-identical because depth is.

An earlier version of this computed the **real** solar position from the device's wall
clock — subsolar point, Greenwich sidereal time, a truncated lunar ephemeris, the lot. It
was accurate, tested against equinoxes and solstices, and completely wrong for the
product: it made the arc depend on what time the user happened to start working, and a
terminator crossing is an event with its own clock, which is precisely what a focus app
must not have. It was removed rather than left switched off. Do not re-add it without
reading this paragraph twice.

## The two axes

`sun_params.x` is named for the sun for historical reasons and has nothing to do with it.
It is the depth scalar, 1 on the runway and 0 at cruise, computed in
`telemetry/generator.rs`. **Daylight is a separate axis**, derived from depth in
`render/celestial.rs`.

They own different things:

- **Depth** owns saturation and the flattening of form. As the flight climbs, the map
  drains toward greyscale and the terrain's directional shading gives way to flat ambient,
  so the world becomes shapeless — there is deliberately nothing to look at at cruise.
- **Daylight** owns brightness, light direction and colour.

Contrast is deliberately *not* driven by depth. The colour grading pulls contrast toward a
mid-grey pivot, and against a basemap that is almost entirely near-black that *raises* the
dark pixels: "flatter" came out as a washed-out, brighter map, the opposite of the intent.

## The sun and the moon

`render/celestial.rs` maps depth to a sun elevation and produces directions and a key
light colour. Three things in it are load-bearing:

- **The elevation curve is eased, not linear.** A straight line runs the sun through the
  last few degrees above the horizon almost instantly, so the sky turned red while the sun
  was still high. An exponent of 1.5 about the crossing makes the sun linger near the
  horizon, which is where all the colour is, and climb away quickly afterwards.
- **Reddening starts at about six degrees** (`DAY_ELEVATION`, a sine of 0.10) — the top of
  the golden hour. Any higher and the sky reddens with the sun well up.
- **The bearing is fixed in the local horizon frame**, held west and a little south, so the
  sun does not swing about as the aircraft turns. Only its elevation moves. The world frame
  is Y-up with longitude toward -Z, so east comes from a cross product with the pole — see
  `globe::geometry::lon_lat_to_ecef_f64`, which this must agree with.

The moon sits exactly opposite the sun, so it rises as the sun sets and is well up at
cruise. That also makes it permanently full, which is the phase worth having.

### Hue outlasts brightness

The sky and the light run the same dusk ramp but mix toward opposite ends. The sky fades
orange into a **dark grey**, and scaling both channels down together preserves the
red/blue ratio, so it stays visibly orange. The light fades orange into **moonlight**,
which is bright and neutral, and that kills the ratio almost at once.

Sharing one ramp therefore left the aircraft pale against a sky that was still burning.
The hue has its own, later crossover (`HUE_DUSK_ELEVATION` to `HUE_NIGHT_ELEVATION`) while
brightness keeps the earlier one. Two tests pin it: warm below the horizon, neutral with
the sun well up.

### Two lights, never a switch

The moon is exactly opposite the sun, so **choosing** between them on a threshold swings
the key light through 180° the instant the sun crosses it, and the lit side of the aircraft
swaps between one frame and the next. This was visible as a step of +16 luminance between
two adjacent progress values.

Both lights now contribute, weighted by the dusk ramp and summed — which is what a pair of
light sources does, and it has no edge to fall off. The specular highlight is blended the
same way. The globe carries the identical code; it had the same switch and would have
swung the whole landscape's shading round at the same instant.

## Objects

In `model_pipeline/shader.wgsl`, ambient is a **floor the key light fills up to**:

```wgsl
light_intensity = ambient + (1.0 - ambient) * lit
```

The two therefore always sum to exactly 1. Adding them instead ran to 1.2 and clipped, so
nothing could be darker than the floor and every surface facing the light blew out flat
white.

Two per-model knobs matter:

- **`diffuse_weight`** — how much of the key light's *direction* a model feels. The key
  light has no occlusion, so inside a cockpit it happily lights the roof lining, which
  faces the sky and is in reality under a fuselage. An interior wants this low (0.35) and
  leans on ambient, which is what light bouncing in through the windows actually is.
- **`rim_strength`** — a Fresnel edge term standing for light wrapping around a silhouette
  against open sky. The aircraft wants it (0.30); it is nearly always seen against sky and
  the edge is what separates it. **The cockpit must not have it.** A window post seen from
  the seat is at a grazing angle to the eye, so the rim term added a flat 0.10 to exactly
  the thing that should read as a silhouette, and made the frames glow. Measured: the posts
  carried +0.102 of it while the panel carried 0.000.

Note that `COCKPIT_TINTS` in `cockpit_model.rs` is currently **inert**. It only applies to
materials authored white, and every material in the present GLB is 0.02–0.22. The table is
kept because it is the right mechanism if the model is replaced with one using white
placeholders, which is what it was written for.

## The sky

`sky_pipeline/mod.rs` owns both `sky.wgsl` and the pipeline that runs it — a full-screen
triangle that reconstructs a world-space ray and ray-marches an atmosphere shell with a
Beer-Lambert opacity. Added to it:

- **A shared palette function.** `sky_palette(sun_elevation)` returns a `[zenith, horizon]`
  pair, mixed from four anchors — `NOON_ZENITH`/`NOON_HORIZON`/`NIGHT_ZENITH`/
  `NIGHT_HORIZON` — on the same day/night ramp as `celestial.rs`, plus a twilight-only violet
  cast on the zenith (`TWILIGHT_ZENITH_VIOLET`) so a clear dusk zenith reads as a saturated
  blue-violet rather than just fading toward the near-black night colour.
  `sky_hue_rotation(sun_elevation, toward_sun)` (renamed from `sky_warm_tint`) is a separate
  multiplicative tint on the horizon colour, bidirectional: on the sun's side of the sky
  (`toward_sun` near 1) it warms, derived from the Rayleigh channel weights used only as a
  relative hue (not a real extinction term — seeing what a literal Beer-Lambert transmittance
  does to a horizon sunset was the reason this wasn't attempted: it computes to a near-black
  smear, not a glow, because a real sunset's brightness is dominated by inscattered light
  along the view path, which this engine doesn't integrate); on the antisolar side
  (`toward_sun` near 0) it instead cools toward a hand-picked saturated blue — the "Earth's
  shadow" band a clear dusk sky shows opposite the sun. `sky.wgsl` builds `toward_sun` from
  `cos_sun`, so the warm half of the sky is the half the sun is in — without that a sunset is
  an even orange band all the way round, which is the giveaway of a faked sky.
  `globe_pipeline/shader.wgsl` used to apply the old tint *unconditionally*, on the claim that
  a per-pixel terrain fragment has no clean "toward the sun" direction the way the sky dome's
  view ray does. That claim doesn't hold: `-to_camera` (already computed there for the
  horizon-haze grazing test) *is* that fragment's view ray, so `cos_sun_terrain =
  dot(-to_camera, sun_dir)` gives the same signal, and the terrain's horizon ring now gets the
  same direction-aware hue rotation the sky dome does. Both files' `toward_sun` remap
  (`-0.2, 0.9`) is now part of the byte-identical-function contract below, not just the
  function bodies themselves — it's a local at each call site, not inside the shared function.

  `sky.wgsl` alone (not shared — `fs_solid` only ever needs a horizon colour, never a mid-sky
  one) also overlays a twilight-only "glow" band (`TWILIGHT_GLOW_PEACH` on the sun's side,
  `BELT_OF_VENUS_PINK` on the antisolar side) between zenith and horizon, for the pale
  peach/gold and pink Belt-of-Venus colours a clear sunset shows that a plain 2-stop gradient
  can't represent. It's a smooth `rise * (1 - fall)` bump (two smoothsteps meeting at
  `GLOW_BAND_POSITION`) laid additively over the always-computed `mix(zenith, horizon,
  color_mix)`, not a piecewise split of `color_mix` itself. An earlier version *did* split
  `color_mix` into two separately-interpolated segments; the colour was continuous at the
  join but its slope wasn't (the two segments aim at very different anchors), which read as a
  distinct bright line hovering in the sky parallel to the horizon — caught by eye in
  `light_audit_sweep`'s `02_sunset` frames. A smoothstep's derivative is exactly zero at both
  of its own edges, so the bump is flat (no kink) exactly at its peak. Being an overlay with
  zero weight at twilight=0 also makes "noon and full-night renders are unaffected" automatic
  and independent of the bump's shape, rather than relying on an algebraic identity between
  two interpolation curves.

  The same "bright line hanging in the air" came back a second time anyway, and the fix for
  it is the rule worth remembering: **the glow band changes the sky's hue, never its
  brightness.** A line in the sky is not a kink in a curve — it is a local *maximum* of
  luminance, which the eye reads as an edge however smoothly the ramp leads into it, and
  laying hand-picked constants over the gradient at their own brightness put one right in
  the middle of the sky. Altitude made it worse (the gradient is dimmed by `* 0.12` at the
  zenith and `* 0.25` at the horizon, the constants were not), so the higher the flight
  climbed the more the band detached from the horizon it belongs to.

  Dimming the anchors on the same altitude schedule was the first attempt and only shrank
  the peak — measured on `02_sunset`, a `+55` bump above the surrounding sky became `+14`,
  which is still a line. The anchor is now rescaled to the exact luminance of the pixel it
  is replacing before being mixed in, so the sky's luminance profile from zenith to horizon
  stays monotonic *by construction*, at any altitude and for any anchor colour anyone picks
  later. Same measurement: `+2` or less, which is dither. What survives is a band of colour
  — peach on the sun's side, Belt-of-Venus pink opposite — which is what was wanted in the
  first place.
- **A sun disc**, its angular radius a named, derived constant (`SUN_ANGULAR_RADIUS`,
  mirroring the already-documented `MOON_ANGULAR_RADIUS` below it) rather than a pair of
  unexplained cosine thresholds, with a two-lobe forward-scatter halo, drawn *before* the
  atmosphere is composited so a low sun is reddened and dimmed by the air it is seen
  through.
- **A moon** with a procedural surface: value-noise maria, finer speckle for craters, and a
  touch of limb darkening so it does not read as a sticker. Procedural rather than a
  texture because this pipeline binds nothing but the camera uniform — an image would mean
  a new bind-group layout, an asset, and a load path that can fail at runtime.
  `MOON_ANGULAR_RADIUS` is deliberately about five times life size; at the real quarter of
  a degree it is a handful of pixels and reads as a stray dot.
- **Stars**, hashed off the world-space ray so the field is pinned to the celestial sphere
  and stays put as the aircraft flies and turns.
- **A hash-based dither**, about one 8-bit ULP, added just before the final return. The
  gradient is smooth and mostly monochrome and this renders for hours in the background, so
  banding gets more visible the longer a session runs, not less. Screen-space rather than
  per-frame, so it doesn't flicker over a session.

**The sky must agree with the globe at the horizon.** This used to be enforced only by a
comment in each file asking whoever edited one to remember the other — and it had already
failed, with the two horizon colours drifted apart by up to 0.05 per channel. Now it's
enforced by construction: `sky_palette`/`sky_hue_rotation` are textually identical functions
in both files (there is no shared-WGSL-include mechanism in this codebase, so "identical"
means copy-pasted, not `#include`d — edit all copies in the same commit), called with the
same `camera.sun_dir.w` input and, now, matching `toward_sun`/`toward_sun_terrain` remap
constants at each call site, so the horizon colour they compute cannot drift apart without
the source itself drifting, which is visible in a diff. Both also carry the same
`EARTH_RADIUS_MM`/`ATMOSPHERE_THICKNESS_MM` constants and the same altitude dimming.

`globe_pipeline/shader.wgsl`'s side of the seam — the haze near the terrain's visual
horizon — used to be detected with an `fwidth`-based screen-space heuristic (how fast
distance-from-Earth-centre changes per pixel), a resolution- and FOV-sensitive proxy. It's
now a direct geometric measure: `grazing_cos = dot(normalize(in.normal), to_camera)` is
exactly 0 at the true visual horizon of a smooth sphere (the same fact the culling
subsystem's own horizon test relies on — zero terrain relief today), so no per-pixel
derivative is needed. The blended colour is named `horizon_haze_color` and the blend weight
`horizon_blend` — this is unrelated to `Stage::Fog` in `quadtree.rs`, a culling/LOD
relaxation that shares only the English word "fog" and touches no colour.

That grazing-angle ring alone left distant-but-not-silhouette terrain crisp until a sudden
fog wall right at the edge — terrain is visible 50-370km away at cruise (horizon distance
from ~10.7km altitude), well beyond where the ring has any effect. `aerial_blend`, a
distance-based fade, is layered on top of `horizon_blend` via `max()` so a fragment that's
both far away and near the silhouette gets one full haze blend rather than a stacked
double-fade.

**Haze is measured in air, not in distance.** This is the one rule that matters here, and
two earlier versions broke it and had to be replaced, each time with the same symptom: zoom
out, and the entire globe turns into a flat, featureless sheet of `horizon_haze_color` —
which at that altitude is the space colour, so the map simply disappears.

- The first version used the raw camera-to-fragment distance, which from orbit is thousands
  of kilometres of mostly vacuum.
- The second bounded that to the part of the ray inside the 150km atmosphere shell (via
  `ray_sphere_intersect`, shared with `sky.wgsl`). That is *also* unbounded in the only way
  that counts: a ray grazing the shell crosses far more of it than the shell is thick, and
  even a vertical look crosses the whole 150km. It made the sums smaller without making them
  scale-invariant, so the whiteout came back a little further out — a Free-camera view from
  ~110km, which is where that camera sits by default, was already a solid grey wash.

What is actually bounded is the *air*. Density falls off exponentially with height on a
scale height of 8km (`HAZE_SCALE_HEIGHT_MM`), so `air_path_length` integrates
`exp(-h / 8km)` along the ray analytically (the integral is exact for a height that varies
linearly between the endpoints; the Earth's curvature lifts the middle of a long ray by a
couple of kilometres, which makes it a slight over-estimate at the horizon, where the haze
is saturated anyway) and returns a length of *sea-level-density* air. Straight down from any
altitude, that is one scale height — 8km, clear — however far out the camera is zoomed.
Along the ground it is the full ray length. The onset/full constants (0.03 / 0.17 Mm of air,
tuned by eye against `light_audit_sweep` and `haze_capture_sweep`, not derived) are ~0.57x
the raw-distance numbers they replaced, that being the fraction a ray from 10km down to the
horizon works out to, so the Tracking and Cockpit frames the effect was originally tuned
against are unchanged to within a couple of 8-bit levels.

One trap worth keeping in mind if you touch this: the heights must come from
`height_above_ellipsoid`, not from `length(p) - EARTH_RADIUS_MM`. The WGS84 axes differ by
21km — more than two scale heights — so at European latitudes the spherical version puts
both the camera and the ground below the surface, every height clamps to zero, every ray
comes out sea-level dense along its whole length, and the haze is far too strong. Only
`space_fade`, measured in hundreds of kilometres, can afford to ignore the difference.

### Stars, in detail

Three things keep them from looking cheap:

- **Never sub-pixel.** A star smaller than a pixel crawls and flickers as the camera moves,
  and that single artefact is what gives most procedural skies away. The radius is derived
  from `fwidth` of the cell coordinate, so it is at least about a pixel at any resolution
  or field of view.
- **Cube-face projection**, so cells stay square everywhere instead of crowding at the
  poles as a latitude/longitude grid would.
- **A power law on brightness.** A flat distribution reads as noise; a few bright and many
  faint reads as sky.

They appear **brightest-first** as the sky darkens, which is what dusk does, and means they
arrive gradually rather than switching on.

They are added *after* the atmosphere is composited, with their own far gentler extinction
curve. Compositing them behind it — the obvious thing, and what a sun disc wants —
extinguished them completely: the sky's opacity describes how much light the atmosphere
*adds*, which at three kilometres is already 90%, not how much it absorbs from a point
source. The long slant path near the horizon still puts them out, which is what you see.

A real star catalogue was considered and rejected. Without a clock the sky's orientation is
arbitrary, so a recognisable Orion would be in the wrong place — and a wrong Orion is worse
than no Orion to anyone who would notice.

## Uniforms

Everything global lives on `CameraUniform` (`render/camera_uniform.rs`), bound at
`@group(0)` for all four pipelines, so one addition reaches every shader:

| field | contents |
|---|---|
| `sun_params` | `[depth, saturation, contrast, brightness]` |
| `sun_dir` | `xyz` toward the sun, `w` = sine of elevation |
| `moon_dir` | `xyz` toward the moon, `w` = lit fraction |
| `light_color` | `rgb` key light hue (normalised), `w` = strength |

It is an ordinary uniform buffer and can grow. The **polyline** push-constant block cannot:
it is exactly 128 bytes, the guaranteed device minimum requested in `wgpu_state.rs`.

The struct is mirrored in four WGSL files — `sky.wgsl`, `globe_pipeline/shader.wgsl`,
`model_pipeline/shader.wgsl`, `polyline_pipeline/polyline.wgsl` — which must be kept
byte-identical.

## Testing

Shaders only compile when their pipeline is created, so a syntax or type error does not
fail `cargo build` — it panics at runtime on the device. **Validate every edited `.wgsl`
against naga** (matching the wgpu version) before believing it works. This caught a real
error during development.

Two WGSL traps already hit here:

- `smoothstep(low, high, x)` is **undefined when `low >= high`**. Write the edges low-first
  and invert with `1.0 - …` instead. It happens to work on desktop and may not elsewhere.
- Semi-transparent geometry fights `depth_write_enabled: true`; prefer `discard` below an
  alpha threshold.

`render::celestial` has eight unit tests covering the arc, its symmetry, the horizon
crossing, and the warmth thresholds.

### Looking at it

`src/testing/rendering/light_audit.rs` (`#[ignore]`d) renders a sweep of progress values ×
camera modes to PNGs and prints the depth scalar for each frame:

```
cargo test --lib light_audit_sweep -- --ignored --nocapture
```

`src/testing/rendering/haze_capture.rs` (also `#[ignore]`d) is the zoom-out counterpart —
the sweep `light_audit_sweep` does not cover, and the one that catches the haze failure
described above, since every camera the flight itself uses sits at flight altitudes:

```
cargo test --release --lib haze_capture -- --ignored --nocapture
```

It shoots a nadir ladder from 10km out to 30 000km (`Camera::max_distance`, as far as the
user can ever zoom) plus grazing looks at three altitudes. The map must stay legible in
every frame of the nadir ladder; the grazing frames are the ones that must keep their haze.

The measurement behind "there is no fog on the surface from space": render the ladder
twice, once normally and once with `final_color = shaded_color` (haze disabled), and diff.
Over 21 altitudes from 10km to 30 000km, nadir and 45°, every ground pixel is bit-identical
except two places — the top ~16 rows of the 300km tilted frame, which is the atmospheric
limb seen edge-on and *should* haze, and a 1-2px rim on the globe's outline past ~7 500km,
which is `horizon_blend`, not `aerial_blend`. Do this diff again after any change here; a
veil that creeps back in is invisible frame-by-frame and obvious in the difference.

**Use it.** This work was reported as finished twice on the strength of compiling cleanly
and passing tests, and both times it was visibly wrong. Rendering frames and measuring
pixels found every real defect; reading the code found none of them.

Two cautions from experience:

- **Eyeball judgement of these renders is unreliable.** Window posts read as "near-white
  0.85" when they measured 0.30; a night cockpit looked blown out when its brightest pixel
  was 0.23. Sample pixels.
- **Sample carefully.** The brightest pixel on the aircraft is a clipped specular highlight
  and always reads 255. The top row of a tracking frame is terrain, not sky. The route line
  passes straight through the middle of the aircraft and is not lit at all. All three
  produced wrong conclusions before being noticed.
