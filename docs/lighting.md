# Lighting and sky

The flight view lights itself from one number: **how deep into the flight the aircraft
is**. There is no clock anywhere in it.

Code: `crates/cesium-engine/src/render/celestial.rs` (sun, moon, key light),
`render/camera_uniform.rs` (what every shader receives), `render/atmosphere.wgsl` (the
scattering model), `render/sky_lut/` (its lookup texture), `render/sky_pipeline/sky.wgsl`,
`render/globe_pipeline/shader.wgsl`, `render/model_pipeline/shader.wgsl`.

## Why it works that way

CesiumRS renders for **Blocktime**, a productivity app. A session runs for hours in the
background, which rules out shadow maps, per-pixel scattering and any full-screen post
pass — all that power for an effect nobody is looking at directly.

More importantly, the view tells a story about the session rather than about the world.
Climbing to cruise is the metaphor for descending into deep focus; the descent is waking
up and the light coming back. So the whole arc is driven by the flight's altitude scalar,
which is symmetric over the flight by construction:

| depth | altitude      | sky                                    |
|-------|---------------|----------------------------------------|
| 1.0   | on the ground | high sun, bright blue                  |
| ~0.55 | climb/descent | sun on the horizon, orange             |
| 0.0   | cruise        | sun down, grey and dark, moon and stars|

Departure and arrival are bit-identical because depth is.

The sun is deliberately **not** computed from a date and time. A real solar position
(subsolar point, sidereal time, a lunar ephemeris) is straightforward, and wrong for this
product: it makes the arc depend on what time the user happened to start working, and a
terminator crossing is an event with its own clock, which is exactly what a focus app
must not have.

## The two axes

The depth scalar (1 on the runway, 0 at cruise) is computed per telemetry sample in
`telemetry/generator.rs`, interpolated by the flight extension into
`Camera::sun_intensity`, and reaches the shaders as `sun_params.x` — named for the sun,
but it has nothing to do with it. **Daylight is a separate axis**, derived from depth in
`celestial.rs`.

They own different things:

- **Depth** owns the flattening of form. As the flight climbs, the terrain's directional
  shading gives way to flat ambient, so the world becomes shapeless — there is
  deliberately nothing to look at at cruise — and the sky darkens toward space
  (`× mix(0.35, 1, depth)`).
- **Daylight** owns brightness, light direction and colour. The map's drain toward
  greyscale also follows the light, not depth (`camera_uniform.rs`): full colour while
  the sun is up and through sunset, fading to grey between sun elevations of about −1° and
  −11.5°. Running it on depth pulls half the colour out of the map by the time the sun
  touches the horizon, and golden hour comes out a grey-orange wash.

Contrast is deliberately *not* driven by either. The colour grading pulls contrast toward
a mid-grey pivot, and against a basemap that is almost entirely near-black that *raises*
the dark pixels: "flatter" comes out as a washed-out, brighter map.

## The sun and the moon

`celestial::compute` maps depth to a sun elevation and produces directions and a key light.
Three things in it are load-bearing:

- **The crossing is at depth 0.55** (`HORIZON_DEPTH`). The depth scalar bottoms out around
  0.3 on a short sector, because it is measured against that flight's own cruise, so a
  crossing at 0.35 leaves a short hop's cruise in permanent twilight. At 0.55 the sun goes
  down during the climb on a short hop and a long one alike.
- **The elevation curve is eased, not linear**: `sin(elevation) = sign(u)·|u|^1.5·0.85`
  with `u` depth normalised either side of the crossing. A straight line runs the sun
  through the last few degrees above the horizon almost instantly, so the sky reddens
  while the sun is still high. The exponent makes the sun linger near the horizon, which
  is where all the colour is.
- **Reddening starts at about six degrees** (`DAY_ELEVATION`, a sine of 0.10) — the top of
  the golden hour. Any higher and the sky reddens with the sun well up.
- **The bearing is fixed in the local horizon frame**, held west and a little south, so the
  sun does not swing about as the aircraft turns. Only its elevation moves. The world frame
  is Y-up with longitude toward −Z, so east comes from a cross product with the pole — see
  `globe::geometry::lon_lat_to_ecef_f64`, which this must agree with.

The moon sits exactly opposite the sun, so it rises as the sun sets and is well up at
cruise, and it is always drawn as a full disc. Its lit fraction is a fixed 0.9
(`MOON_ILLUMINATION`), used as its brightness.

### Hue outlasts brightness

The sky and the light run the same dusk ramp but mix toward opposite ends. The sky fades
orange into a **dark grey**, and scaling both channels down together preserves the
red/blue ratio, so it stays visibly orange. The light fades orange into **moonlight**,
which is bright and neutral, and that kills the ratio almost at once. With one shared
ramp the aircraft turns pale against a sky that is still burning.

So the hue has its own, later crossover (`HUE_DUSK_ELEVATION` −0.12 to
`HUE_NIGHT_ELEVATION` −0.30, in sine) while brightness keeps the earlier one (−0.02 to
−0.22). Unit tests pin it: warm below the horizon, neutral with the sun well up.

The key light's strength is `(1 − night)·(0.85 + 0.15·day)` from the sun plus
`night·(0.05 + 0.15·0.9)` from the moon, capped at 1, so a low sun stays an effective
directional light through the golden hour.

### Two lights, never a switch

The moon is exactly opposite the sun, so **choosing** between them on a threshold swings
the key light through 180° the instant the sun crosses it, and the lit side of the aircraft
swaps between one frame and the next — measured as a step of +16 luminance between two
adjacent progress values. Instead both lights contribute, weighted by the dusk ramp and
summed, which is what a pair of light sources does and has no edge to fall off. The
specular highlight is blended the same way, and the globe, the route line and the models
all use the identical weighting.

## Objects

`model_pipeline/shader.wgsl` lights the aircraft and the cockpit:

```wgsl
total_light = ambient_override · ambient_factor + lit · key_colour
shaded      = texture · vertex_colour · total_light · detail + key_colour · (spec + rim)
```

clamped to 1. `lit` is the sun and moon terms summed by the dusk weights, times the key
light's strength and the model's `diffuse_weight`. The dials come from the flight
extension per draw:

| | aircraft | cockpit |
|---|---|---|
| `ambient_override` | `0.03 + 0.15·depth` | `0.05 + 0.29·depth^1.5` |
| `diffuse_weight` | 1.0 | 0.35 |
| `specular_strength` | 0.35 | 0.15 |
| `rim_strength` | `0.08 + 0.17·depth` | 0 |
| `detail_strength` | 0 | 0.13 (triplanar surface grain, about 4.5 mm cells) |

- **`diffuse_weight`** is how much of the key light's *direction* a model feels. The key
  light has no occlusion, so inside a cockpit it happily lights the roof lining, which
  faces the sky and is in reality under a fuselage. An interior wants this low and leans
  on ambient, which is what light coming in through the windows is.
- **`rim_strength`** is a clearcoat Fresnel edge: Schlick's `F0 + (1 − F0)(1 − N·V)⁵` with
  `F0 = 0.04`, the reflectance of polyurethane gloss paint, standing for light wrapping
  round a silhouette against open sky. The aircraft wants it; it is nearly always seen
  against sky. **The cockpit must not have it**: a window post seen from the seat is at a
  grazing angle to the eye, so the rim term adds a flat 0.10 to exactly the thing that
  should read as a silhouette, and the frames glow (measured: +0.102 on the posts, 0 on the
  panel).
- **Specular** is Blinn-Phong with exponent 64, scaled by the same Schlick term at the
  half-vector and normalised by `F0` so it is 1 at normal incidence and brightens toward
  grazing, as a clearcoat does; masked to surfaces facing the light.
- **Hemispherical ambient.** For exterior models the ambient is split by
  `dot(normal, up)`, with up the fragment's radial direction: full on surfaces facing the
  sky, 0.40 of it on surfaces facing the ground. The switch is `rim_strength > 0`, so the
  cockpit, with its rim at zero, keeps uniform ambient — an interior has no ground a few
  centimetres away to bounce off. That is a real coupling: a model wanting a rim on an
  interior part, or none on an exterior one, gains or loses ground-bounce shading with it.
- **Self-lit surfaces** (the cockpit displays) take none of the lighting: the final colour
  is mixed toward the plain texture by the vertex's `unlit` value.

Model materials are covered in [models.md](models.md).

## The sky

`sky_pipeline/mod.rs` draws one full-screen triangle at depth 0 behind the globe, and
`sky.wgsl` reconstructs the world-space view ray per pixel. The colour along that ray
comes from `render/atmosphere.wgsl`, a physically based single-scattering atmosphere
shared with the globe.

**Nothing is raymarched per pixel or per vertex.** The model is evaluated by
`render/sky_lut/` into one 128×98 `Rgba16Float` texture: 96 rows of the sky from the
camera (over azimuth from the sun, squared toward the sun where the aureole needs the
resolution, and view zenith angle with Hillaire's mapping, squeezed toward the horizon,
whose position moves with the camera's height), plus one row of skylight and one of
direct sunlight on horizontal ground, against the sun's cos-zenith there. The LUT is
re-rendered every frame in a single pass before the scene. The sky pixel does one fetch;
the globe vertex two (the ground rows); the globe fragment one, and only where there is
haze to show. `src/testing/rendering/sky_perf.rs` measures the GPU cost.

- **Rayleigh + Mie + ozone, one raymarch per LUT texel.** 16 quadratically spaced samples
  along the view ray, dense near the eye. The sunlight reaching each sample is attenuated
  along its own path analytically with the Chapman function (the same one the haze uses),
  so there is no inner loop, and the planet's shadow is a ray/sphere test on that sun ray.
  Everything a sunset is made of comes out of this rather than being painted: the reddened
  sun, the yellow-orange band along the horizon under it, the aureole (Mie,
  Cornette-Shanks with g = 0.72 — at 0.8 the aureole tone-maps to a white blob that
  swallows the disc), the blue-grey Earth's shadow rising from the antisolar horizon after
  sunset with the pink Belt of Venus on top of it, and the deep blue twilight zenith.
- **The twilight zenith is blue because of ozone.** Light still reaching the upper air
  after sunset crosses the ozone layer almost edge-on, and the Chappuis band takes its
  orange out (Hulburt 1953). The ozone path is the exact chord through a tent-shaped layer
  from 10 to 40 km. With textbook 680/550/440 nm coefficients the zenith comes out
  mauve-brown; the red coefficient is raised (a display's red primary sits near the
  Chappuis peak) and the set doubled to stand in for the aerosol extinction single
  scattering leaves out.
- **Multiple scattering is a stand-in**: an isotropic term lit by the sunlight at a point
  high enough that its sun ray skims the planet no lower than 25 km — the blue,
  ozone-filtered light of the upper twilight sky, which is what lights the air inside the
  Earth's shadow and keeps it blue-grey instead of black. It is weighted by each sample's
  own sunlight with a floor of 0.08; letting the bluish skylight swamp the low, long,
  red-lit horizon paths washes the sunset band out to lavender.
- **Exposure and tone curve.** `atmo_exposure` opens up by up to 7 stops as the sun goes
  down (about 5.5 by the end of civil twilight at −6°, when a real sky is still clearly
  lit): the eye adapting, but not all the way, so it still gets darker. Then a per-channel
  `1 − exp(−x)` shoulder, which rolls the sun and aureole off to white instead of clipping
  them, and a 1.3 display saturation (the shoulder alone leaves the noon sky pale cyan).
- **The sun disc and glow**, evaluated only within about 14° of the sun. The disc's colour
  is the transmittance of its own line of sight (power 0.7, normalised to its brightest
  channel), so it goes white, yellow, orange as it sets. A tight glow (strength 1.6,
  e-folding 2°) and the limb-darkened disc (0.49° radius, about twice life size, with a
  white-hot core) are screen-blended over the sky *after* the altitude darkening —
  darkened with the sky, the disc reads as a grey dot. The wide aureole is left to the
  Mie term. Atmospheric refraction (Bennett's formula) is undone on the view ray before
  the disc test, which lifts the sun just after geometric sunset and flattens it into an
  oval at the horizon.
- **A moon** with a procedural surface: value-noise maria, finer speckle for craters, and a
  touch of limb darkening so it does not read as a sticker. Procedural because a texture
  would need its own binding, an asset and a load path that can fail, for a disc a degree
  across. `MOON_ANGULAR_RADIUS` is about 4.7 times life size; at the real quarter of a
  degree it is a handful of pixels and reads as a stray dot.
- **A night glow** (airglow and starlight, the source single scattering lacks once the sun
  is far down), graded from zenith to horizon.
- **Stars**, hashed off the world-space ray so the field is pinned to the celestial sphere
  and stays put as the aircraft flies and turns. They start to appear with the sun about
  7° down and are complete by about 11.5°: at the end of civil twilight the sky is still
  lit.
- **A hash-based dither**, about one 8-bit step, added last. The gradient is smooth and
  mostly monochrome and this renders for hours, so banding gets more visible the longer a
  session runs, not less. Screen-space rather than per-frame, so it does not flicker.

**The sky must agree with the globe at the horizon.** Both shader modules — and the LUT
pass — are built by prepending `atmosphere.wgsl` to their own source (`concat!` in
`wgpu_state.rs` and `sky_lut/mod.rs`), so there is one model and one LUT
parametrisation. The globe's haze reads the *same* LUT in the direction of each fragment:
the LUT's rows below the horizon are exactly the air in front of the ground, so distant
terrain fades into exactly the sky above it. Both apply the same altitude darkening,
`mix(0.35, 1.0, depth)`.

### Stars, in detail

Three things keep them from looking cheap:

- **Never sub-pixel.** A star smaller than a pixel crawls and flickers as the camera moves,
  and that single artefact is what gives most procedural skies away. The radius is derived
  from `fwidth` of the cell coordinate, so it is at least about a pixel at any resolution
  or field of view.
- **Cube-face projection**, so cells stay square everywhere instead of crowding at the
  poles as a latitude/longitude grid would.
- **A power law on brightness.** A flat distribution reads as noise; a few bright and many
  faint reads as sky. Tints run from blue-white to amber.

They appear **brightest first** as the sky darkens, which is what dusk does, so they
arrive gradually rather than switching on.

They are added *after* the atmosphere is composited, extinguished by the local sky
luminance and toward the horizon. Compositing them behind the atmosphere — the obvious
thing, and what a sun disc wants — extinguishes them completely: the sky's opacity
describes how much light the atmosphere *adds*, which a few kilometres up is already most
of it, not how much it absorbs from a point source.

A real star catalogue is deliberately not used. Without a clock the sky's orientation is
arbitrary, so a recognisable Orion would be in the wrong place — and a wrong Orion is worse
than no Orion to anyone who would notice.

## The ground

### Light arriving at the ground

The globe takes its light from the same atmosphere, per vertex, from the LUT's two ground
rows: direct sun (transmittance, zero once that ground is in the Earth's shadow) and
skylight (three short raymarches — zenith, low toward and low away from the sun — done in
the LUT pass).

- **Colour.** The light's colour tints the ground, pulled halfway to white
  (`GROUND_TINT_STRENGTH` 0.5) because the eye white-balances: golden hour reads warm, blue
  hour cool, neither dyed. A horizontal field gets the sun's `max(n·s, 0) + 0.25`, the
  extra standing for everything on real ground that faces a low sun — trees, walls — which
  a flat satellite image cannot show and which is most of why a golden-hour landscape
  reads warm.
- **Ambient brightness** mixes a day floor (`0.70` at cruise to `0.58` on the ground) and a
  night floor (`0.18` to `0.26`) by a dusk ramp from +2.3° to −10.4° of sun elevation, and
  golden hour is 10 % dimmer than noon. The ground darkens with the sky instead of staying
  at noon brightness under a dusk sky.
- **Ambient tint** is the white-balanced light by day and a cool moonlight
  (0.80, 0.86, 0.98) at night, crossing over on a *scotopic* ramp from −4.6° to −14.5°:
  the eye keeps colour well into civil twilight, and starting the grey moonlit look at
  sunset drains the ground to grey while the sky is still on fire.
- **Directional light.** The sun term uses a slight wrap, `max((n·s + 0.12)/1.12, 0)`,
  scaled by the square root of the transmitted sunlight's luminance and tinted by the
  white-balanced sun colour; by day it is at most `0.38 · mix(0.2, 1, depth)` —
  effectively off at cruise and full on the ground, the flattening-with-depth rule. The
  moon adds a flat `0.08 · max(n·m, 0)` at night.

The ambient colour does **not** come from the view-dependent horizon haze colour, which is
strongly orange toward the sun and blue away from it: tinting the ground with it makes the
same field change colour with the camera heading.

### The texture: highlights, night and grading

`fs_solid` then treats the imagery itself:

- **Highlight roll-off.** Texture values above 0.75 lose up to 45 % of the excess, scaled by
  `day_amount`, so runway concrete and pale roofs keep surface detail instead of clipping to
  white. It rides the day ramp, so it fades through twilight rather than snapping off.
- **Mesopic night curve.** Night vision is rod-dominated: colour discrimination collapses
  and the eye reads mostly luminance. `photo_gate = smoothstep(0.04, 0.35, luminance)` opens
  only for bright pixels, and scaled by `scotopic · photo_gate` two things ride it: a gamma
  push to 1.35 that darkens the tile, and an 80 % blend toward its own luminance. Gating on
  the *texture's own brightness* rather than on which basemap is loaded keeps this
  model-agnostic: the dark CARTO basemap is near-black everywhere its roads are not, so the
  gate stays shut and its roads stay legible, while photographic imagery gets the full
  moonlit treatment with no per-basemap branch.
- **Grading** (saturation, contrast, brightness from `TileEngineConfig`, saturation
  drained at night as described above) is skipped when all three are zero, and the result
  is **always** clamped to [0, 1]: the roll-off and the night curve can push a channel out
  of range on their own, and an unclamped value shows up only as a wrong pixel later.

### Haze

The haze near the terrain's visual horizon is one term, `aerial_blend`, mixing the shaded
ground toward `horizon_haze_color` (the sky LUT in the fragment's direction, tone-mapped
and darkened like the sky). It is unrelated to the atmospheric *fog* that relaxes LOD in
the quadtree ([tiles-and-lod.md](tiles-and-lod.md#atmospheric-fog)), which shares only the
word and touches no colour.

**Haze is measured in air, not in distance.** Every alternative fails the same way — zoom
out and the whole globe turns into a flat sheet of haze colour and the map disappears:

- The raw camera-to-fragment distance is, from orbit, thousands of kilometres of mostly
  vacuum.
- The part of the ray inside a 150 km atmosphere shell is unbounded in the only way that
  counts: a ray grazing the shell crosses far more of it than it is thick, and even a
  vertical look crosses all 150 km. At the Free camera's default distance of about 110 km
  it is a solid grey wash.
- An exponential density integrated along a **flat slab**, with height varying linearly
  between the two endpoints, is right for the flight's own views and wrong the other way
  everywhere else: a ray that grazes the planet spends most of its length near its lowest
  point, so from 100 km up the terrain stays crisp to a hard silhouette against a sky that
  is already white with haze.
- A grazing-angle term (`n·to_camera`, zero at the true visual horizon of a sphere) is a
  proxy for "this ray crosses a lot of air"; the quantity itself is available directly,
  and two mechanisms combined with `max`, one a proxy for the other, is one too many.

What is actually bounded is the *air*, and the correct measure of it has a name. For an
exponential atmosphere the optical depth from a point out to space along a ray is
`n(P) · H · Ch(r/H, χ)`, where `Ch` is the **Chapman function** — the curved-atmosphere
generalisation of the schoolbook `1 / cos χ` air mass. The two differ only near the
horizon, and that is the entire subject: `1 / cos χ` diverges there, while `Ch` tops out at
`√(π·X/2)`, about 35 vertical columns or roughly 280 km of sea-level air. A horizon looks
right when that number is large but finite.

`air_path_length` is `column(fragment) − column(camera)` along the shared ray, in Mm of
sea-level-density air, with an 8 km scale height. It needs no raymarching — two `exp`s and
two `sqrt`s — and it is correct at both ends of the range: straight down from any altitude
it returns one scale height (8 km, clear) however far the camera is zoomed out; along a
horizon ray it returns the full ~280 km whatever the altitude. Checked against numerical
integration from ground level to 2 000 km altitude and from nadir to the horizon: worst
case 1.2 %.

Two details in it are load-bearing:

- **The scaled error function must be scaled.** `Ch` needs `exp(z²)·erfc(z)` for arguments
  up to about 20. The textbook Abramowitz & Stegun 7.1.26 polynomial bounds its error on
  `erfc` *absolutely*, so dividing out the vanishing `exp(−z²)` leaves it 38 % wrong in
  exactly this regime. `erfcx` uses Numerical Recipes' fit kept in scaled form instead:
  fractional error below 1.2·10⁻⁷.
- **Negative `cos χ` is not an error to clamp away.** It means the ray's lowest point lies
  *between* camera and fragment — a camera near the ground looking at distant ground. The
  formula continues analytically to that case (`erfcx(−z) = 2·exp(z²) − erfcx(z)`), and the
  continuation is what makes a taxi view agree with numerical integration to 1 %.

The onset and saturation constants, 0.035 and 0.160 Mm of air (`AERIAL_HAZE_ONSET_MM`,
`AERIAL_HAZE_FULL_MM`), are tuned by eye against the light audit, not derived.

One trap: heights come from `ellipsoid_frame`, not from `length(p) − EARTH_RADIUS_MM`. The
WGS84 axes differ by 21 km — more than two scale heights — so at European latitudes a
spherical height puts both camera and ground below the surface, every height clamps to
zero, every ray comes out sea-level dense along its whole length, and the haze is far too
strong. `ellipsoid_frame` also returns the local *up* (the ellipsoid normal, which differs
from the direction to the Earth's centre by up to 11 arcminutes), which the zenith angles
are measured against.

**There is no haze on the ground seen from space.** Rendered with and without haze over 21
altitudes from 10 km to 30 000 km, nadir and 45°, every ground pixel looking down is
bit-identical up to 3 000 km; beyond that, with the whole planet in frame, the only
difference is a rim at the edge of the disc — 32 px wide at 5 000 km, thinning to 7 px at
30 000 km — which is the atmosphere seen edge-on. Repeating this diff is the way to catch a
veil creeping back in, which is invisible frame by frame and obvious in the difference.

## Uniforms

Everything global lives on `CameraUniform` (`render/camera_uniform.rs`), bound at
`@group(0)` in every pipeline, so one addition reaches every shader:

| field | contents |
|---|---|
| `view_proj`, `inv_view_proj` | camera-relative matrices |
| `camera_pos` | f64 camera position narrowed to f32 |
| `sun_params` | `[depth, saturation, contrast, brightness]` |
| `sun_dir` | `xyz` toward the sun, `w` = sine of elevation |
| `moon_dir` | `xyz` toward the moon, `w` = lit fraction |
| `light_color` | `rgb` key light hue (normalised), `w` = strength |

It is an ordinary uniform buffer and can grow. The **polyline** push-constant block cannot:
it is exactly 128 bytes, the guaranteed device minimum requested in `wgpu_state.rs`.

The struct is mirrored in six WGSL files — `sky_pipeline/sky.wgsl`, `sky_lut/sky_lut.wgsl`,
`globe_pipeline/shader.wgsl`, `model_pipeline/shader.wgsl`,
`polyline_pipeline/polyline.wgsl`, `label_pipeline/shader.wgsl` — which must stay
byte-identical with the Rust layout.

## Testing

Shaders compile when their pipeline is created, so a syntax or type error does not fail
`cargo build`: it panics at run time, on the device. Any headless test that builds a
`WgpuState` compiles every pipeline; an edited `.wgsl` should also be validated against
naga for the wgpu version in use.

Two WGSL traps worth knowing:

- `smoothstep(low, high, x)` is **undefined when `low >= high`**. Write the edges low-first
  and invert with `1.0 − …` instead. It happens to work on desktop and may not elsewhere.
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

`light_audit_dark_vs_satellite_sweep` runs six progress values (0.0 … 1.0, taxi to cruise
and back) in Tracking mode twice, once on the default dark basemap and once on Esri
imagery (`shoot_with_config` threads the tile configuration through). The night curve's
gate opens only on bright photographic pixels, so a defect in it is invisible on the dark
map, and this pair is the frame that shows it.

`src/testing/rendering/sunset_capture.rs` (`#[ignore]`d) is the one to tune the sunset
with: six fixed views (toward the sun, away from it, the zenith, from 3 km both ways, and
the ground from 9 km) at sun elevations +10, +3, 0, −3, −6 and −10 degrees, each turned back
into the depth that produces it, on satellite imagery:

```
SUNSET_DIR=out cargo test --lib sunset_capture -- --ignored --nocapture
SUNSET_ELEVS=3,-3 SUNSET_VIEWS=g_sun,air_anti ...   # narrow it
```

`src/testing/rendering/sky_perf.rs` (`#[ignore]`d) is the performance instrument: the real
flight view (tracking at Frankfurt; dark and satellite with terrain; noon and sunset)
offscreen at 1080p, with GPU timestamps around each frame's scene
(`WgpuState::scene_timestamps`; the device requests timestamp features only when
`CESIUM_GPU_TIMING` is set, which the test does). Frames are queued back to back so the
GPU stays at its top clock — waiting per frame lets an integrated GPU drop clocks between
frames and the numbers swing by 30 %. To compare two builds, build both lib test binaries,
run them alternately on the same machine, and compare medians:

```
cargo test --release --lib --no-run   # then run the binary:
./cesium_rs-<hash> sky_perf --ignored --nocapture   # SKY_PERF_SCENES/FRAMES/BATCHES
```

`src/testing/rendering/haze_capture.rs` (also `#[ignore]`d) is the zoom-out counterpart —
the range the light audit does not cover, and the one that catches the haze failure above,
since every camera the flight itself uses sits at flight altitudes:

```
cargo test --release --lib haze_capture -- --ignored --nocapture
```

It shoots a nadir ladder from 10 km out to 30 000 km (`Camera::max_distance`, as far as
the user can zoom) plus grazing looks at three altitudes. The map must stay legible in every
frame of the ladder; the grazing frames are the ones that must keep their haze.

Rendering frames and measuring pixels finds the defects in this code; reading it does not.
Two cautions:

- **Judgement by eye is unreliable.** Window posts read as "near-white 0.85" when they
  measured 0.30; a night cockpit looked blown out when its brightest pixel was 0.23.
  Sample pixels.
- **Sample carefully.** The brightest pixel on the aircraft is a clipped specular highlight
  and always reads 255. The top row of a tracking frame is terrain, not sky. The route line
  passes straight through the middle of the aircraft and is lit differently.
