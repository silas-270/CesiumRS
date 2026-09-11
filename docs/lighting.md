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

`sky_pipeline/sky.wgsl` is a full-screen triangle that reconstructs a world-space ray and
ray-marches an atmosphere shell with a Beer-Lambert opacity. Added to it:

- **A three-way palette** — day, dusk, night — ramped on sun elevation, plus azimuthal
  warmth toward the sun during twilight. Without that last part a sunset is an even orange
  band all the way round, which is the giveaway of a faked sky.
- **A sun disc** with a two-lobe forward-scatter halo, drawn *before* the atmosphere is
  composited so a low sun is reddened and dimmed by the air it is seen through.
- **A moon** with a procedural surface: value-noise maria, finer speckle for craters, and a
  touch of limb darkening so it does not read as a sticker. Procedural rather than a
  texture because this pipeline binds nothing but the camera uniform — an image would mean
  a new bind-group layout, an asset, and a load path that can fail at runtime.
  `MOON_ANGULAR_RADIUS` is deliberately about five times life size; at the real quarter of
  a degree it is a handful of pixels and reads as a stray dot.
- **Stars**, hashed off the world-space ray so the field is pinned to the celestial sphere
  and stays put as the aircraft flies and turns.

**The sky must agree with the globe at the horizon.** Both carry the same altitude dimming
and the same day/dusk/night ramp, with a comment in each saying so. They used to agree for
free by both keying off the depth scalar; once the sky moved to a time-of-day ramp, any
difference between them showed up as a hard line drawn across the whole view.

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
