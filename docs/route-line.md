# The route line

The ribbon drawn along the flight's ground track, the three ways it can be shown, and
how it is kept on the terrain near the airports.

## Modes

`RouteLineMode` (`crates/cesium-flight/src/flight_handle.rs`):

- **`Full`** — the whole route, departure to arrival. The default.
- **`Window { behind_m, ahead_m }`** — only the stretch around the aircraft, fading to
  transparent at both outer ends.
- **`Hidden`** — no line at all. The aircraft is still drawn.

Showing the whole route from the first second of a long-haul flight both gives the route
away and fills the screen with a line that is nowhere near the aircraft, which is at odds
with what Blocktime is for.

The two distances are set by the host app and are not exposed to the end user; they
cross the FFI as parameters so a change of taste does not need a new engine build.

## Setting it

`FlightHandle::set_route_line_mode`, which applies from the next frame to every loaded
flight. From Kotlin, following the shape of `nativeSetCameraMode`:

```
nativeSetRouteLineMode(mode: Int, behindNm: Double, aheadNm: Double)
```

`0` full, `1` window, `2` hidden. The distances are **nautical miles** and are read only
for mode 1; they are converted to metres at the boundary, as the planner works in metres
throughout. No struct crosses JNI — everything is primitives, as with every other entry
point.

Nothing persists across restarts, so the host owns the setting and re-pushes it on engine
start, the same as the camera-pose restore.

The debug panel offers the three modes with two drag values in NM (40 behind, 150 ahead
to start with), so the window can be exercised on desktop without an Android build.

## The ribbon

`PolylineRenderer` (`crates/cesium-engine/src/render/polyline_pipeline/`) draws a flat
ribbon, parallel to the ground, expanded in the vertex shader from a storage
buffer of control points: two vertices per point, extruded sideways along
`up × tangent`, 5 m above the path (`RIBBON_LIFT_M`, which must equal the shader's
`elevation`). Positions are stored as high/low f32 pairs and differenced against the
camera's high/low pair on the GPU, so the line does not jitter at Earth scale.

- **Control points** come from `AdaptiveSubdivisionBuilder`: the interpolated path is
  subdivided wherever the chord deviates from the curve by more than 10 cm
  (`tolerance = 1e-7` Mm), with a five-minute maximum step, plus a maximum segment length
  along the stretches the terrain fit moves (below).
- **Width** is a 1.49 m half-width within about 1.3 km of the camera, then grows in
  proportion to distance (the shader multiplies by `clamp(0.05 · distance, 67 m, 3 000 km) / 67 m`),
  so the line stays legible from orbit and never outgrows the aircraft up close.
- **Colour** is split at the aircraft: orange behind, near-white ahead. Near the aircraft
  the split is decided per fragment by position along the tangent from the aircraft's
  camera-relative position, so it lands exactly under the model; elsewhere by progress.
- **Lighting** follows the scene's key light (sun, then moon), with an ambient floor of
  0.22–0.45 and 65 % of the key light's tint, so the line warms at sunset and dims at
  night with everything else.

## How the window works

The window is measured **along the route**, which is harder than it looks.

`ControlPoint::progress` is normalised *time* — `(t − start) / (stop − start)` — and a
flight covers ground very unevenly against the clock. Equal steps of progress are not
equal distances, and a window expressed in miles cannot be a constant progress delta. The
error is small through the cruise of a long flight and large at both ends: on a short
sector the last tenth of the flight covers under three-quarters of the ground a cruising
tenth does.

So each control point also carries its **cumulative arc length** (`ControlPoint::distance`,
megametres, in the f32 slot beside the high position). The length is summed over chords
between consecutive points, which are placed by the 10 cm chord-deviation test above, so
it is exact to far better than anything drawn from it could show.

The window reaches the shader as **two absolute distances along the route** rather than a
centre and a radius. That is not a stylistic choice: the polyline push-constant block is
exactly 128 bytes, the guaranteed device minimum, and two floats is precisely what was
left. Each frame the CPU computes `aircraft distance ± the configured window` from a
progress→distance table held on `FlightEntity` (`route_distances`).

Fading is a `smoothstep` over the outer quarter of the window at each end, with a
`discard` below 1 % alpha so fully-outside fragments cost nothing and write no depth. The
pipeline blends with standard alpha blending.

`Hidden` skips the draw call entirely rather than drawing a transparent line.

## On the ground

Near the airports the flight is moved onto the *drawn* terrain, and the line is moved by
exactly the same rule as the aircraft (`LineFit` in `crates/cesium-flight/src/terrain_fit.rs`);
fitting only the aircraft left the line hanging 20–39 m above it on the runway. Along the
ground phases the builder places a control point at least every 10 m (a takeoff roll is
straight, and three points could not follow the ground), and every 100 m through the fade
up to 600 m above the field; on the ground the line runs 1.5 m above the terrain, under
the belly. The fit re-samples at most 64 points a frame as height data arrives and
re-uploads only the ranges that moved by more than a centimetre
(`PolylineRenderer::write_points`). See [models.md](models.md#height-and-ground-contact).

## Things to know before changing it

- **The window is deliberately not clamped to the route's start.** Letting it hang off the
  front keeps the ribbon solid at the departure airport instead of fading it in over the
  first forty miles. "No window" is therefore spelled as an empty range
  (`window_end > window_start` failing; the tracker sets both to −1), not as a negative
  start.
- **Geometry is uploaded once per flight load**; after that only the terrain fit rewrites
  the ranges it moves. Anything else per-frame has to happen in the shader.
- `src/headless/route_builder.rs` builds control points itself and fills the same
  distance field.

## Testing

`src/testing/flight/test_route_window.rs` checks that arc length runs forward and matches
the route, and — the point of the whole exercise — that distance and progress are
genuinely different axes. That second test is measured on a short sector, where climb and
descent dominate; on a seven-hour flight the climb is only about 3 % of the clock and the
two track each other closely enough to hide the problem.
`src/testing/rendering/route_line_ground.rs` renders the line against the runway from
250 m to 3 km.
