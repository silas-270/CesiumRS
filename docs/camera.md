# Camera

The camera's structure, its three modes, the projection and near plane, input, and how it
is kept out of the ground. Code: `crates/cesium-engine/src/camera/camera.rs` (the camera),
`core/touch.rs` and `core/app.rs` (input), `crates/cesium-flight/src/camera_modes/` (what
each mode does every frame).

## Anchor and local transform

A camera is two transforms composed:

```text
global_position    = anchor_pos + anchor_ori · local_pos
global_orientation = anchor_ori · local_ori
```

The **anchor** (`DVec3`, `DQuat`) is the thing the camera is attached to; the **local**
transform (`Vec3`, `Quat`) is the offset from it. Every mode is a choice of anchor:

| Mode | Anchor | Local transform |
|---|---|---|
| Free | Earth's centre, identity | the camera's full position and orientation |
| Tracking | the aircraft, 7.5 m above its flight position, oriented along its track **without bank** | an orbit: yaw, pitch and distance around the aircraft |
| Cockpit | the aircraft's exact position and rotation (it banks and pitches with it) | the pilot's eye point; orientation is the head |

`global_transform_f64` composes in f64 and is what culling, the camera uniform and every
camera-relative draw use. The anchor is f64 because it can be anywhere on Earth; the
local offset is f32 because in Tracking and Cockpit it is metres to kilometres. In Free
mode the local position is the whole ECEF position in f32, so the camera's absolute
position there is quantised to about 0.5 m; culling stays consistent because the frustum
and the tiles are referred to the same f64 value.

## The three modes

The engine owns the mode enum; what a mode *does* each frame is the flight extension's
job (`camera_modes/`), called from `FlightTrackerApp::update`:

- **Free** (`free.rs`). On entry or reset, frames the whole route: the midpoint of
  departure and arrival, a basis rotated so the route runs along the screen diagonal,
  the extents of every sample projected on it, and the distance at which they fit with a
  5 % margin. Afterwards the user owns the camera: drag rotates the globe under the
  pointer, wheel/pinch zooms, keys pitch.
- **Tracking** (`tracking.rs`). Every frame re-anchors on the aircraft with a no-bank
  orientation (`math::transform::velocity_to_orientation`), so turns do not roll the
  view. On entry the orbit is 250 m out, 22° up, 45° off the tail on the left side. Drag
  orbits (pitch clamped to −80°…85°), zoom scales the distance by 10 % per unit within
  20 m – 30 000 km.
- **Cockpit** (`cockpit.rs`). Anchored to the aircraft's full rotation; the local position
  is re-applied every frame (`CAMERA_LOCAL_MM`, 44 m forward and 17 m up of the model
  origin, plus a small aft-and-up pull-back), so the seat cannot drift out of the
  interior. On entry the view looks 8° below the nose so the displays sit along the
  bottom of the frame. Drag turns the head within ±100° yaw and ±34° pitch; zoom does
  nothing (it would slide the eye through the walls).

The aircraft's orientation, which Tracking and Cockpit are anchored to, is not the
planner's telemetry attitude: `math::trajectory::TrajectoryEvaluator` derives it from the
interpolated path — nose along the spline tangent, and "up" opposing gravity minus the
path's centripetal acceleration averaged over a 30 s window (three quarters look-ahead),
scaled by 2.5/3 — then smooths the quaternion over ±0.8 s. See [flight-plan.md](flight-plan.md#attitude).

A mode switch (from the host, the debug panel or a key) or a newly loaded flight triggers
the mode's default framing once; `FlightTrackerApp::pending_camera_restore` can then
override it with a saved pose (the Android app restores the user's view this way after
the activity is recreated).

## Projection

Reverse-Z perspective, recomputed every frame:

| | Field of view | `znear` | `zfar` |
|---|---|---|---|
| Free | `2·atan(24 / (2·focal_length))`, 46.4° at the default 28 mm | `0.1 × altitude above ground`, clamped to 10 cm … 10 Mm | `‖eye‖ + 10 Mm` |
| Tracking | same | `0.05 × orbit distance`, clamped to 1 cm … 5 m | same |
| Cockpit | fixed 60° | 5 cm | same |

- **The near plane follows height above the ground**, not above the ellipsoid
  (`altitude_agl`, the ellipsoid altitude minus the terrain height sampled under the eye
  that frame). Flying up the Inn valley at 900 m above the ellipsoid, the camera is about
  330 m above the valley floor, and a wall 400 m ahead sat inside a near plane of
  `0.1 × 900 m = 90 m`. With terrain off, `altitude_agl` is `altitude` exactly.
- **The far plane is never binding for the globe**: every ellipsoid point is within
  `‖eye‖ + a` of the eye, and `zfar` is `‖eye‖ + 10 Mm` (invariant I-3 in
  [culling-implementation.md](culling-implementation.md)).
- **Cockpit uses a wider, fixed field of view** because the lens-derived 46° is about 22°
  horizontally in a portrait phone, a letterbox onto the panel.
- `fovy()` is a single function read by both the projection and the LOD rule, so the two
  cannot disagree about the frustum. The f32 and f64 projections compute `znear` from the
  same f32 altitude so the drawn and culling frusta match to the bit.

Tile culling uses only the four side planes of this projection; see
[culling-implementation.md](culling-implementation.md).

## Input

**Desktop** (`core/app.rs`): left drag — Free: rotate the globe under the pointer;
Tracking: orbit; Cockpit: look around. Wheel, `I`/`O`, `PageUp`/`PageDown`: zoom. `W`/`S`
or arrows: pitch. With the debug panel's God camera active, `WASD`/`Space` fly it and
right drag looks around; the main camera, which culling always uses, stays where it was.

**Touch** (`core/touch.rs`, `TouchInterpreter`): one-finger pan (with a dead band, and
routed like a mouse drag per mode), double tap zooms in, double-tap-and-drag zooms
continuously, two-finger tap zooms out, and a two-finger gesture decides after its first
movement between pinch-and-twist (zoom) and parallel vertical drag (pitch, or orbit
pitch, or head pitch).

**Programmatic**: `ViewerHandle` (`camera_set_mode`, `camera_zoom`, `camera_pitch`,
`camera_set_position`, `camera_set_anchor`), and on Android the JNI camera exports.

**Free-mode drag** is exact rather than incremental: on press the ray under the pointer is
intersected with the ellipsoid (or, past the limb, the closest point of the ray is
projected onto it), and each move re-derives the rotation that carries that start point to
the point under the pointer now, applied to the camera's pose at the start of the drag.
The grabbed point stays under the finger. Release with enough speed starts **inertia**:
the rotation continues about the last axis and decays by 0.92 per sixtieth of a second
until it falls below 0.05 rad/s.

**Zoom** in Free mode moves the camera along its view direction by 15 % of its altitude per
unit, with a 2 m floor on the step size; in Tracking it scales the orbit distance.

## Keeping the camera out of the ground

`Camera::enforce_bounds_with(ground)` is the one place collision is decided. It runs once
per frame in `update_logic`, after the extension has put the camera where it wants it,
with `ground` being `TileSystem::ground_height_at` — the deepest resident height data,
view-independent — sampled **at every position it tests** rather than once under the
camera. With terrain off it is called with no ground and keeps the camera 2 m off the
ellipsoid.

- **The floor** at a position is the ellipsoid radius along it, plus the terrain height
  there, plus 2 m. Terrain is consulted only below **15 km above the ellipsoid**
  (Cesium's `minimumCollisionTerrainHeight`): higher up, the deepest resident data is a
  z4–z6 tile whose height is a continental average, and a floor that jumps by hundreds of
  metres as tiles stream in is worse than none; and nothing on Earth reaches 15 km.
- **Free** mode: a camera below its floor is moved radially up to it; beyond
  `max_distance` (30 000 km above the equatorial radius) it is pulled back.
- **Tracking** mode keeps the orbit and lifts only its pitch: the lowest pitch at or above
  the requested one at which the camera is above its floor **and** the terrain does not
  block the line of sight to the aircraft (sampled every 25 m, 12–96 samples, over the
  first 90 % of the distance — the rest is the ground the aircraft stands on). The search
  scans upward in 3° steps, because a hill can make validity non-monotone in pitch, then
  bisects the last step ten times. Without the line-of-sight half, an orbit swinging
  behind a ridge left the camera in the valley behind it, looking at grass.
- **Cockpit** mode is never clamped: the seat is bolted to the aircraft, and the aircraft
  is fitted to the ground by the flight extension.

To keep the collision on the ground that is drawn, `update_logic` registers the points it
will test next (`TileSystem::want_ground_at`): the camera, the anchor, and in Tracking a
ring around the aircraft at the orbit radius (8–48 points, one per 400 m) and at half of
it. Their z15 height tiles are requested above everything in view.

## Traces

On desktop, `CESIUM_TRACE=1` makes `camera/trace.rs` write one row per frame to
`camera_trace.csv`: mode, position and attitude, orbit parameters, altitude above the
ellipsoid and above the ground, the ground under the camera and under the anchor, how far
collision moved the camera that frame, the height above the *drawn* ground, and the
lowest clearance along the line of sight.
`tools/camera_jumps.py` finds frames where the camera moved discontinuously.
