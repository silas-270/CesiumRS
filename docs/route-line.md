# The route line

The polyline drawn along the flight's ground track, and the three ways it can be shown.

## Modes

`RouteLineMode` (`crates/cesium-flight/src/flight_handle.rs`):

- **`Full`** — the whole route, departure to arrival. The default.
- **`Window { behind_m, ahead_m }`** — only the stretch around the aircraft, fading to
  transparent at both outer ends.
- **`Hidden`** — no line at all. The aircraft is still drawn.

Showing the whole route from the first second of a long-haul flight both gives the route
away and fills the screen with a line that is nowhere near the aircraft, which is at odds
with what FocusFlight is for.

The two distances are set by the Android side and are not exposed to the end user — they
cross the FFI as parameters so a change of taste does not need a new engine build.

## Setting it

JNI, following the shape of `nativeSetCameraMode`:

```
nativeSetRouteLineMode(mode: Int, behindNm: Double, aheadNm: Double)
```

`0` full, `1` window, `2` hidden. The distances are **nautical miles** and are read only
for mode 1; they are converted to metres at the boundary, as the rest of the planner works
in metres throughout. No struct crosses JNI — everything is primitives, as with every other
entry point.

Nothing persists across restarts, so Kotlin owns the setting and re-pushes it on engine
start, the same as the camera-pose restore.

The mode is also in the egui debug panel, with two drag values in NM, so it can be
exercised on desktop without an Android build.

## How the window works

The window is measured **along the route**, which is harder than it looks.

`ControlPoint::progress` is normalised *time* — `(t - start) / (stop - start)` — and a
flight covers ground very unevenly against the clock. So equal steps of progress are not
equal distances, and a window expressed in miles cannot be a constant progress delta. The
error is small through the cruise of a long flight and large at both ends: on a short
sector the last tenth of the flight covers under three-quarters of the ground a cruising
tenth does.

So the control points carry **cumulative arc length** alongside progress, in the `f32` slot
that used to be padding (`polyline_pipeline/builder.rs`). The length is summed over chords
between consecutive points, which are placed by a chord-deviation test with a tolerance
measured in centimetres, so it is exact to far better than anything drawn from it could
show.

The window then reaches the shader as **two absolute distances along the route** rather
than a centre and a radius. That is not a stylistic choice: the polyline push-constant
block is exactly 128 bytes, the guaranteed device minimum, and two floats is precisely
what was left. The CPU computes `aircraft_distance ± the configured window` each frame,
from a progress→distance table held on `FlightEntity`.

Fading is a `smoothstep` over the outer quarter of each end, with a `discard` below a small
alpha threshold so fully-outside fragments cost nothing and write no depth. Alpha blending
was already enabled on this pipeline; no pipeline change was needed.

`Hidden` skips the draw call entirely rather than drawing a transparent line.

## Things to know before changing it

- **The window is deliberately not clamped to the route's start.** Letting it hang off the
  front keeps the ribbon solid at the departure airport instead of fading it in over the
  first forty miles. The "disabled" sentinel is therefore `window_end > window_start`
  failing, not a negative start.
- **Geometry is uploaded once per flight load and never rewritten.** Anything per-frame has
  to happen in the shader, not by rebuilding buffers.
- `src/headless/route_builder.rs` builds control points directly and must fill the same
  distance field.

## Testing

`src/testing/flight/test_route_window.rs` checks that arc length runs forward and matches
the route, and — the point of the whole exercise — that distance and progress are genuinely
different axes. That second test is measured on a short sector, where climb and descent
dominate; on a seven-hour flight the climb is only about 3% of the clock and the two track
each other closely enough to hide the problem.
