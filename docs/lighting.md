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

- **Depth** owns the flattening of form. As the flight climbs, the terrain's directional
  shading gives way to flat ambient, so the world becomes shapeless — there is deliberately
  nothing to look at at cruise. The map still drains toward greyscale on the way there, but
  on the *light's* schedule (`camera_uniform.rs`): full colour while the sun is up and
  through sunset, fading out across civil and nautical twilight. Running it on depth pulled
  half the colour out of the map by the time the sun touched the horizon.
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

Two more effects sit on top of that base model:

- **Clearcoat Fresnel.** `rim` is no longer a hand-picked `pow(1 - N·V, 3)` edge falloff; it
  is a Schlick Fresnel reflectance for a dielectric clearcoat, `F0 + (1 - F0) * (1 - N·V)^5`
  with `F0 = 0.04` — the textbook reflectance of polyurethane gloss paint at normal
  incidence, which is what every aircraft surface here is modelled as wearing. The same term
  (evaluated at the half-vector instead, `F0 + (1 - F0) * (1 - V·H)^5`, normalised by `F0` so
  it equals 1 at normal incidence and only grows from there) now also scales the specular
  catch-light, so the highlight itself brightens toward grazing angles the way a clearcoat's
  does, rather than staying a fixed-intensity Blinn-Phong spot. The gloss exponent went from
  32 to 64 in the same change, for a tighter, sleeker glint instead of a soft blob. None of
  this touches `push.rim_strength`'s meaning from the reader's point of view — it is still
  the per-model dial for how much edge/grazing sheen a surface gets, and the cockpit still
  keeps it at 0 for the reason above.
- **Hemispherical ambient occlusion / ground bounce.** Ambient used to be perfectly
  isotropic: a belly panel facing straight down at the ground received exactly as much
  ambient as the top of the fuselage facing open sky — backwards, since the ground is
  exactly the thing an ambient *sky* term should not count as a light source. `world_up =
  normalize(push.camera_pos.xyz + in.view_pos)` — the fragment's own radial direction, a
  fine stand-in for "up" at aircraft scale, unlike the sphere-vs-ellipsoid error that
  matters for terrain (`ellipsoid_frame`, discussed under haze below) — feeds
  `dot(normal, world_up)` to split ambient between a `ground_ratio` of 0.40 for
  downward-facing surfaces and the full floor for upward-facing ones.
  **This reuses `push.rim_strength` as the exterior/interior switch**:
  `is_exterior = step(0.001, push.rim_strength)`, so the cockpit interior — which already
  sets `rim_strength = 0.0` to keep the window posts from glowing — rides that same zero to
  fall back to old uniform, direction-independent ambient, because an interior has no
  "ground" a few centimetres away to bounce off; it has a floor lit by whatever comes
  through the windows. That is a real coupling, not a coincidence: a future model that wants
  a nonzero rim on an interior part, or a zero rim on an exterior one, silently gains or
  loses ground-bounce shading along with it.

## The sky

`sky_pipeline/mod.rs` owns both `sky.wgsl` and the pipeline that runs it — a full-screen
triangle that reconstructs a world-space ray. The colour along that ray comes from
`render/atmosphere.wgsl`, a physically based single-scattering atmosphere shared with the
globe (see "The sky must agree with the globe" below).

**Performance rule: nothing is raymarched per pixel or per vertex.** The model is baked by
`render/sky_lut/` into one 128x98 `Rgba16Float` texture — the sky from the camera over
(azimuth from the sun, view zenith angle with Hillaire's horizon-squeezed mapping), plus two
rows of ground skylight and ground sunlight against the sun's cos-zenith — and re-rendered
only when the sun's elevation (1e-3 in sine, 0.06°) or the camera height (3%) has moved,
and then spread over four frames, one band each (a whole LUT is ~0.4ms on the laptop's
Vega 8, which rendered every frame was +4% frame time). The sky
pixel does one fetch; the terrain vertex two; the terrain fragment one, and only where there
is haze to show. The user's budget for this was "no more than 1% slower than before"; the
`sky_perf` harness (`src/testing/rendering/sky_perf.rs`) measures it with GPU timestamps.
Measured on the laptop's Vega 8 the LUT version is at or below the old painted gradient.

- **Rayleigh + Mie + ozone, one raymarch per LUT texel.** 16 quadratically spaced samples
  along the view ray; the sunlight reaching each sample is attenuated along its own path
  analytically with the Chapman function (the same one the haze uses), so there is no inner
  loop. The planet's shadow is a ray/sphere test on that sun ray. Everything a sunset is
  made of comes out of this rather than being painted: the reddened sun, the yellow-orange
  band along the horizon under it, the aureole (Mie, g = 0.72), the blue-grey Earth's shadow
  rising from the antisolar horizon after sunset with the pink Belt of Venus on top of it,
  and the deep blue twilight zenith.
- **The twilight zenith is blue because of ozone.** Light still reaching the upper air
  after sunset crosses the ozone layer almost edge-on, and the Chappuis band takes its
  orange out (Hulburt 1953). The ozone path is the exact chord through the 10–40km layer.
  With textbook 680/550/440nm coefficients the zenith came out mauve-brown; the red ozone
  coefficient is raised (a display's red primary sits near the Chappuis peak) and the set
  doubled to stand in for the aerosol extinction single scattering leaves out.
- **Multiple scattering is a stand-in.** An isotropic term lit by the sunlight at a point
  high enough that its sun ray skims the planet no lower than 25km — the blue,
  ozone-filtered light of the upper twilight sky, which is what lights the air inside the
  Earth's shadow and keeps it blue-grey instead of black. Weighted by each sample's own
  sunlight (floor 0.15), or it swamps the red sunset band into lavender.
- **Exposure and tone curve.** `atmo_exposure` opens up by up to 10 stops through twilight
  (~6.5 at -6°, when a real sky is still clearly lit; the eye adapting, but not all the
  way — it still gets darker), then a per-channel
  `1 - exp(-x)` shoulder and a 1.3 display saturation (the shoulder alone left the noon sky
  pale cyan).
- **A sun disc and glow**, evaluated only within ~14° of the sun. Colour is the
  transmittance of its own line of sight (power 0.7, normalised), so it goes white, yellow,
  orange as it sets; a two-part exponential glow (0.8 at 1.1°, 0.3 at 4.6° e-folding) and
  the limb-darkened disc are screen-blended over the sky *after* the altitude darkening —
  darkened with the sky, the disc read as a grey dot. Atmospheric refraction (Bennett's
  formula) is undone on the view ray before the disc test, which lifts the sun just after
  geometric sunset and flattens it into an oval at the horizon.
- **A moon** with a procedural surface: value-noise maria, finer speckle for craters, and a
  touch of limb darkening so it does not read as a sticker. Procedural rather than a
  texture because this pipeline binds nothing but the camera uniform — an image would mean
  a new bind-group layout, an asset, and a load path that can fail at runtime.
  `MOON_ANGULAR_RADIUS` is deliberately about five times life size; at the real quarter of
  a degree it is a handful of pixels and reads as a stray dot.
- **Stars**, hashed off the world-space ray so the field is pinned to the celestial sphere
  and stays put as the aircraft flies and turns. Gated to a sun more than ~7° down: at the
  end of civil twilight the sky is still lit.
- **A hash-based dither**, about one 8-bit ULP, added just before the final return. The
  gradient is smooth and mostly monochrome and this renders for hours in the background, so
  banding gets more visible the longer a session runs, not less. Screen-space rather than
  per-frame, so it doesn't flicker over a session.

**The sky must agree with the globe at the horizon.** It used to be enforced by
copy-pasted palette functions in both shaders, and had already drifted once. Now both shader
modules are built by prepending `atmosphere.wgsl` to their own source (`concat!` in
`wgpu_state.rs`), and the globe's haze reads the *same* sky LUT in the direction of each
fragment — the LUT's rows below the horizon are exactly the air in front of the ground.
Distant terrain fades into exactly the sky above it. Both apply the
same altitude darkening, `mix(0.35, 1.0, depth)`.

### Ground light at sunset

The ground takes its light from the same atmosphere, per vertex, from the LUT's two ground
rows: the direct sun (transmittance, zero once that ground is in the Earth's shadow) and
skylight (three short raymarches — zenith, low toward and low away from the sun — done in
the LUT pass). Their
*colour* tints the ground, pulled halfway to white (`GROUND_TINT_STRENGTH`) because the eye
white-balances — golden hour reads warm, blue hour cool, neither dyed. Brightness follows
its own ramp from +2° to -10°, so the ground darkens with the sky instead of staying at
noon brightness under a dusk sky.

What this replaced was the cause of the grey-orange ground: (1) the map's saturation was
drained linearly with *depth*, so it was already ~45% grey when the sun reached the
horizon; (2) the ambient light was multiplied by the chroma of the *view-dependent* horizon
haze colour (including the warm hue rotation and the sun glow), which was very saturated
orange toward the sun and blue away from it, so the same field changed colour with the
camera heading and turned orange-on-grey at dusk; and (3) the moonlit grey conversion
started at sunset, on `night_amount`.

### Terrain lighting: the mesopic night curve and the daytime highlight roll-off

The ambient/diffuse split in `fs_solid` (`globe_pipeline/shader.wgsl`) is keyed off
`day_amount` / `night_amount` — `smoothstep(0.0, 0.10, sun_elevation)` and
`smoothstep(-0.02, -0.22, sun_elevation)`, the identical six-degree `DAY_ELEVATION`
crossing and dusk-ramp thresholds already documented under "The sun and the moon" — so the
ground's day/night transition and the sky's reddening are pinned to the same moment by
construction, not by two constants that happen to agree today.

- **Ambient now depends on day/night, not only on depth.** It used to be
  `ambient = mix(1.0, 0.8, altitude_scalar)` — full daylight and a clear night sky lit the
  ground through the same 0.8–1.0 floor, depth (climb/cruise/descent) being the only thing
  it heard from. `base_ambient` now mixes a `day_ambient` (`0.70` at cruise down to `0.58`
  on the ground) against a `night_ambient` (`0.18` at cruise up to `0.26` on the ground) by
  `night_amount`. Diffuse follows the same split: by day it caps at
  `0.38 * mix(0.2, 1.0, altitude_scalar)` — effectively off at cruise (`≈0.076`) and full
  on the ground, the existing "flatten toward ambient as the flight climbs" rule from
  depth, just applied inside the terrain's own lighting model instead of overriding it from
  outside — while by night it is a flat `0.08` regardless of altitude, because moonlight
  is a much weaker direct source than sunlight and has no reason to track the flight's
  depth the way the sun's diffuse term does. In full daylight the two floors sum to at most
  `0.58 + 0.38 = 0.96`, always under 1 — headroom the highlight roll-off below depends on.
  (The code's own comment claims a `~0.12` night-ambient floor at cruise; the constant it
  sits next to is `0.18`. Read the constant, not the comment.)
- **Daytime highlight roll-off.** Texture values above `0.75` have up to 45% of the excess
  subtracted (`* 0.45 * day_amount`) before grading, so runway concrete and light-coloured
  roofs — which would otherwise clip straight to white — keep some visible surface detail.
  It rides `day_amount` rather than a flat constant, so it fades through the same twilight
  window as everything else instead of snapping off at some elevation.
- **Mesopic night curve.** Real night vision is rod-dominated: colour discrimination
  collapses at low light and the eye reads mostly luminance, with what tint survives a cool
  Purkinje-shifted blue rather than the daylight white point. `night_lum` — the luminance of
  the texture *after* the highlight roll-off above — feeds
  `photo_gate = smoothstep(0.04, 0.35, night_lum)`, which only opens for bright pixels.
  Scaled by `night_amount * photo_gate`, two things ride that gate: a gamma push
  (`pow(color, mix(1.0, 1.35, …))`) that darkens the tile, and a 65%-weighted blend toward
  `luminance * vec3(0.75, 0.82, 0.95)` — grey tinted cold blue, not black. Gating on the
  *texture's own brightness*, rather than on which basemap is configured, is what keeps this
  model-agnostic: the default "Dark Matter" vector basemap (`STANDARD_IMAGERY_URL`) is
  already near-black everywhere its roads aren't, so `night_lum` never clears 0.04,
  `photo_gate` stays at 0, and the curve leaves it alone — which is the entire point, since
  crushing an already-dark map toward monochrome would erase the roads it exists to keep
  legible. Point the engine at photographic imagery (`SATELLITE_IMAGERY_URL`) instead and
  the same code now has bright pixels to gate on, and the curve applies without any
  per-basemap branch.
- **The final `clamp` moved outside the colour-grading `if`.** It used to run only when
  saturation/contrast/brightness were non-default, which was fine while nothing upstream
  could push a channel out of `[0, 1]` on its own. The highlight roll-off and the mesopic
  gamma/tint both can, independent of any user grading, so the clamp is now unconditional.
  Skipping it would not fail loudly — it would produce an out-of-range colour that only
  shows up as a wrong pixel later, with nothing near the actual cause.

`globe_pipeline/shader.wgsl`'s side of the seam — the haze near the terrain's visual
horizon — is one term, `aerial_blend`, and there is a long history of it being two. The
blended colour is named `horizon_haze_color` and the blend weight `aerial_blend`; both are
unrelated to `Stage::Fog` in `quadtree.rs`, a culling/LOD relaxation that shares only the
English word "fog" and touches no colour.

The term that was deleted, `horizon_blend`, hazed the terrain by its grazing angle,
`dot(normalize(in.normal), to_camera)`, which is exactly 0 at the true visual horizon of a
smooth sphere — sound geometry, and it replaced an even worse `fwidth` screen-space
heuristic. But its `to_camera` was built as `camera_pos - world_pos` from a
camera-**relative** `world_pos`, which is `2*camera - fragment`, not a direction to the
camera at all. Measured: at the true visual horizon seen from 10km up it returns 0.996
where the real grazing cosine is 0. The term never fired at any altitude a flight reaches
— rendering the sweep with it forced to zero changed nothing below 10 000km — and past
~7500km it woke up and drew a 1-2px ring of space colour around the globe. That ring was
the only thing it ever did.

Fixing the vector would have been the wrong repair. A grazing-angle ring is a *proxy* for
"this ray crosses a lot of air", and the rule below measures that quantity directly, and
correctly for exactly the grazing geometry the proxy existed to catch. Two mechanisms
combined with `max()`, one a proxy for the other, is one mechanism too many. The same
reasoning retired `space_fade`, which pulled `horizon_haze_color` toward the colour of
space as the camera climbed: it existed because the haze used to saturate over the whole
globe when zoomed out and *something* had to stop that being a pale blue disc. What is
left now is a thin rim at the limb, and the limb is exactly where this colour must agree
with the sky behind it — which is atmosphere, not space.

**Haze is measured in air, not in distance.** This is the one rule that matters here, and
three versions of this code broke it, each with the same symptom: zoom out, and the whole
globe turns into a flat sheet of `horizon_haze_color` and the map disappears.

- The first used the raw camera-to-fragment distance, which from orbit is thousands of
  kilometres of mostly vacuum.
- The second bounded that to the part of the ray inside the 150km atmosphere shell. That is
  *also* unbounded in the only way that counts: a ray grazing the shell crosses far more of
  it than the shell is thick, and even a vertical look crosses the whole 150km. The
  whiteout came back a little further out — the Free camera's default ~110km view was a
  solid grey wash.
- The third integrated a real exponential density profile, but along a **flat slab**: the
  height taken to vary linearly between the two endpoints. That is right for the flight's
  own views, and wrong in the opposite direction everywhere else — a ray that grazes the
  planet spends most of its length near its lowest point, not halfway between its ends, so
  from 100km up the terrain stayed crisp all the way to a hard silhouette against a sky
  that was already white with haze.

What is actually bounded is the *air*, and the correct measure of it has a name. For an
exponential atmosphere the optical depth from a point out to space along a ray is
`n(P) * H * Ch(r/H, chi)`, where `Ch` is the **Chapman function** — the curved-atmosphere
generalisation of the schoolbook `1 / cos(chi)` air mass. The two differ only near the
horizon, and that is the entire subject: `1 / cos(chi)` diverges there, while `Ch` tops out
at `sqrt(pi * X / 2)`, about 35 vertical columns or ~280km of sea-level air. A horizon
looks right when that number is large but finite.

`air_path_length` is then just `column(fragment) - column(camera)` along the shared ray,
in Mm of sea-level-density air. It needs no ray-marching — two `exp`s and two `sqrt`s — and
it is correct by construction at both ends of the range that broke every earlier version:
straight down from any altitude it returns one scale height (8km, clear) however far the
camera is zoomed; along a horizon ray it returns the full ~280km whatever the altitude.
Checked against brute-force numerical integration from ground level to 2000km altitude and
from nadir to the horizon: worst case 1.2%.

Two details in there are load-bearing, and both were got wrong on the way:

- **The scaled error function must be scaled.** `Ch` needs `exp(z^2) * erfc(z)` for
  arguments up to ~20. The textbook Abramowitz & Stegun 7.1.26 polynomial bounds its error
  on `erfc` *absolutely*, so dividing out the vanishing `exp(-z^2)` leaves it 38% wrong in
  exactly this regime. `erfcx` uses Numerical Recipes' fit kept in scaled form instead:
  fractional error below 1.2e-7, verified against `math.erfc` across the range.
- **Negative `cos(chi)` is not an error to clamp away.** It means the ray's lowest point
  lies *between* camera and fragment rather than at one of them — a camera near the ground
  looking at distant ground. The same formula analytically continues to that case
  (`erfcx(-z) = 2*exp(z^2) - erfcx(z)`), and the continuation is what makes a taxi view
  agree with numerical integration to 1%.

The onset/full constants (0.03 / 0.19 Mm of air) are tuned by eye against
`light_audit_sweep`, not derived: from 10km up, haze starts about 50km out (29km of air)
and saturates about 300km out (198km of air), which is what the Tracking and Cockpit frames
were already set to. Every flight frame in the sweep survived the switch to the Chapman
integral within 17/255, mean under 0.2.

One trap if you touch this: heights come from `ellipsoid_frame`, not from
`length(p) - EARTH_RADIUS_MM`. The WGS84 axes differ by 21km — more than two scale heights
— so at European latitudes the spherical version puts both camera and ground below the
surface, every height clamps to zero, every ray comes out sea-level dense along its whole
length, and the haze is far too strong. `ellipsoid_frame` also returns the local *up* (the
ellipsoid normal, not the direction to the Earth's centre — they differ by up to 11
arcminutes), which is what the zenith angles are measured against.

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

`src/testing/rendering/sunset_capture.rs` (`#[ignore]`d) is the one to tune the sunset
with: six fixed views (toward the sun, away from it, the zenith, from 3km both ways, and the
ground from 9km) at fixed sun elevations (+10, +3, 0, -3, -6, -10 degrees), each turned
back into the depth that produces it, on satellite imagery:

```
SUNSET_DIR=out cargo test --lib sunset_capture -- --ignored --nocapture
SUNSET_ELEVS=45,0 SUNSET_VIEWS=g_sun,g_anti ...   # narrow it
```

`src/testing/rendering/sky_perf.rs` (`#[ignore]`d) is the performance gate: the real flight
view (tracking at FRA; Carto dark and satellite+terrain; noon and sunset) offscreen at
1080p, with GPU timestamps around each frame's scene (`WgpuState::scene_timestamps`; the
device only asks for timestamp features when `CESIUM_GPU_TIMING` is set, which the test
does). Frames are queued back to back so the GPU stays at its top clock — waiting per frame
let an APU drop clocks between frames and the numbers swung by 30%. Build the lib test
binary for both commits, run them alternately on the target machine, compare medians:

```
cargo test --release --lib --no-run   # then run the binary:
./cesium_rs-<hash> sky_perf --ignored --nocapture   # SKY_PERF_SCENES/FRAMES/BATCHES
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

`light_audit_sweep` and `haze_capture` both vary progress and camera/altitude, but neither
ever changes which basemap is loaded — every frame renders the default "Dark Matter" vector
tiles. That was fine until the terrain shader grew a mesopic night curve gated on the
sampled texture's own brightness (`photo_gate`, see "The sky" above): a bug in that gate
would be invisible to both sweeps, since Dark Matter's near-black pixels never open it.
`light_audit_dark_vs_satellite_sweep` closes that gap by adding a second dimension — the
imagery source — instead of another altitude or camera angle: it runs the same six
progress values (`0.0, 0.2, … 1.0`, spanning the full depth arc, taxi to cruise and back) in
`Tracking` mode twice, once against `TileEngineConfig::default()` and once with
`base_imagery_url` swapped to `SATELLITE_IMAGERY_URL` (Esri World Imagery, photographic),
via a new `shoot_with_config` that threads the tile config through instead of always taking
the default. The output pairs up as `dark_*.png` / `sat_*.png` per progress value, so a
regression that only shows up on bright photographic tiles at night — the exact failure
mode `photo_gate` exists to prevent — has a frame that will actually show it.

The measurement behind "there is no fog on the surface from space": render the ladder
twice, once normally and once with `final_color = shaded_color` (haze disabled), and diff.
Over 21 altitudes from 10km to 30 000km, nadir and 45°, the result is that haze appears
*only* where the limb is in frame. Looking down, every ground pixel is bit-identical to the
haze-free render at every altitude up to 3000km; past that the whole planet fits on screen
and what shows up is a rim at the edge of the disc — 32px wide at 5000km, thinning to 7px
at 30 000km — which is the atmosphere seen edge-on, the thing that is bright in every
photograph of the Earth. Nothing veils the surface at any altitude. Do this diff again
after any change here: a veil that creeps back in is invisible frame-by-frame and obvious
in the difference.

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
