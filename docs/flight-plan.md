# Flight plan generation

How `cesium-flight` turns two airports and a duration into telemetry, and where every
number in it comes from.

The goal is that a route drawn on the globe should be the route an airline would
actually file, and the altitude and speed traces should be the ones a crew would
actually fly — using nothing but local computation. There is no waypoint database, no
wind download and no navigation data of any kind. Everything below is either geometry,
published aircraft performance, or a rule of the air.

## The pipeline

Each stage is one module under `crates/cesium-flight/src/telemetry/`.

| Stage | Module | Produces |
|---|---|---|
| Runway selection | `runway.rs` | Which end of which runway each airport is using |
| Enroute route | `lateral.rs` | Waypoints from departure to arrival |
| Ground track | `path.rs` | Those waypoints joined into a path with an arc length |
| Vertical profile | `vertical.rs` | Altitude as a function of distance along that path |
| Speeds | `schedule.rs` | Airspeed at each point, and the fit to the session length |
| Sampling | `generator.rs` | `TelemetryPoint`s, with attitude derived from the path |

Supporting these: `geo.rs` (spherical geodesy), `atmosphere.rs` (ISA and airspeed
conversions), `aircraft.rs` (the A350-900 envelope), `wind.rs` (jet stream
climatology), `airspace.rs` (regions civil traffic avoids).

`aircraft.rs` is the only place a performance number may be invented. If you are asking
"is this realistic?", that is the file to read.

## Everything horizontal is spherical

`geo.rs` works in unit vectors and never subtracts two longitudes. This is a rule, not
a preference — a longitude delta is exactly what sends a route the wrong way around the
world when it crosses the antimeridian, and there is no such quantity here to get wrong.
389 routes in the bundled database cross 180°, including Melbourne–New York.

The one place a flat tangent plane is used is `LocalFrame`, and it is confined to
geometry within a few kilometres: the arc of a single fly-by turn, and the runway
layout. Anything spanning a leg uses the sphere. Projecting a whole intercontinental
route onto one tangent plane makes the route both the wrong shape and up to 35% too
long — London–Tokyo peaks at 71°N on the great circle and at about 45°N on a flat map.

## The route is a great circle, and then two reasons to leave it

### Wind

This is the larger of the two effects on most routes, and it is why the two directions
of the same city pair do not retrace each other.

Formally it is **Zermelo's navigation problem** — the minimum-time path through a moving
medium. It has no closed form over a real wind field, and operationally nobody looks for
one: dispatch systems lay a lateral grid over the great circle and search it. That is
what `lateral.rs` does, with a dynamic program over lateral offsets, costing each leg by
`distance / ground speed` from the wind triangle.

The wind field itself is climatological rather than forecast, because there is no
network. `wind.rs` models what actually shapes routes:

- Mid-latitude westerlies with a jet core near 40° and 250 hPa (about FL340).
- **Jet streaks** off the east coasts of North America and Asia, where the land/sea
  temperature contrast is sharpest. These are why the North Atlantic and North Pacific
  have organised track systems and nowhere else does.
- Seasonal migration: the winter jet is stronger and further toward the equator.

The vertical shape has a floor rather than being a bare Gaussian about the core. The jet
is the *maximum* of the westerlies, not the whole of them — they run through the depth of
the troposphere. A field that dies away below 4 km leaves the surface dead calm, which
in turn leaves runway selection with nothing to work from.

The meridional component is deliberately zero. Climatologically it very nearly is, and
inventing one would put lateral structure into routes with no basis for it.

### Closed airspace

A hard constraint rather than a cost. `airspace.rs` holds coarse national outlines for
the regions most Western operators avoided as of September 2026, and grid nodes and legs
inside them are simply unusable.

Two details matter more than the outlines themselves:

- **The Russian Arctic sector is a separate region**, running from the north coast to
  89.9°N. Without it a Europe–Asia route hops over the top of the landmass polygon at
  88°N and carries on, which is both far shorter than the real re-route and not
  something anyone is permitted to do.
- **A region containing either endpoint is dropped.** The closure is against foreign
  operators, not against physics; an airline based inside it overflies it perfectly
  happily, and without this a Moscow–St Petersburg flight would have no legal path.

Because these encode a political situation rather than a physical one, they date.
`FlightPlanConfig::avoid_closed_airspace` turns the whole mechanism off.

The effect is large and correct: London–Tokyo comes out 20% longer than its great circle
and crosses the Arctic at 88°N, descending through Alaska. That is the real post-2022
re-route, and it is why that flight gained roughly three hours.

### Oceanic tracks

Where a route crosses the North Atlantic or North Pacific, the crossing is snapped onto
the organised track grid: whole degrees of latitude on every tenth meridian, which is how
tracks are published. A crossing therefore looks stepped and angular where the rest of a
route looks smooth — a distinctive and recognisable shape.

### Why the route is a polyline

Real enroute tracks are not smooth curves. A flight plan is a sequence of fixes joined by
straight legs, with a few degrees of turn at each. Letting the grid resolution show
through produces that character honestly, rather than faking kinks into a curve that
would otherwise be perfectly smooth.

## Turns

Corners are rounded by **fly-by turns**: the aircraft starts turning before the fix and
rolls out after it, never passing over the fix itself. That is what a flight management
computer does at almost every waypoint, and it replaced a Dubins solver that only ever
applied to the two terminal corners.

The radius is the thing to get right. A coordinated turn holds

```
r = v² / (g · tan φ)
```

so fixing the radius does not fix the geometry — it fixes the *bank angle to whatever the
speed implies*. The previous fixed 4 km radius implied 58° of bank at cruise speed.
Radius here follows from speed and a bank limit (25°, reduced to 20° above FL340 where
the margin to buffet is thin), never the other way round.

Three consequences worth knowing:

- **Turn radius is per-corner.** The departure turn is sized for the speed at the *end*
  of the departure leg, not at the acceleration altitude — by the time the aircraft gets
  there it has cleaned up and is doing 250 kt.
- **Turns beyond 100° are split into two.** A turn needs `r·tan(θ/2)` of straight leg
  either side; past about 100° the departure leg runs out, and the arc would otherwise
  tighten until the bank exceeded what the aircraft may use. A departure that reverses
  course really is flown as two turns joined by a short leg.
- **Arcs are sampled by angle, not just by length.** They must stay finer than the
  telemetry sampler's own step, which gets down to about 100 m near the ground.
  Otherwise consecutive samples straddle facet junctions and read the turn as happening
  in instalments, which looks like a turn twice as tight as the one being flown.

## The vertical profile

### Climb and descent distances are derived, not chosen

The climb is integrated against a rate of climb that decays toward the ceiling; the
descent is flown at the 3° that both an idle descent and an ILS glideslope sit at — the
origin of the 3:1 rule, three nautical miles per thousand feet. Together they put the top
of climb about 240 km out and the top of descent about 200 km before the runway.

Those distances are *outputs*. Choosing them, as a fixed 30 km and 50 km, is what
produced a 23° climb angle — steeper than a fighter leaves the runway at.

If the pair does not fit the distance available, the planned level steps down until it
does. On a sector too short for even the lowest usable level, the profile is compressed:
the aircraft climbs and immediately descends, which is what a short hop actually does.

### Cruise happens at a flight level

Levels are 1,000 ft apart, and which are available depends on direction: the
**semicircular rule** gives odd thousands to easterly tracks (000–179°) and even to
westerly. This is why an aircraft going one way is never at the same altitude as one
coming back.

ICAO defines the rule on *magnetic* track. True track is used here, which differs by up
to about 20° where magnetic variation is largest and so occasionally picks the other
parity. Correcting it would need a magnetic model; the visible consequence is one flight
level.

### Long cruises are a staircase

An aircraft cannot reach its best altitude while full of fuel, so it starts lower and
**steps up** as it burns off — 2,000 ft at a time, because that is what keeps it on the
right side of the semicircular rule. Steps are placed roughly every two and a half hours
of cruise, and each is a short climb of twenty-odd kilometres, not an instant jump.

A long-haul altitude trace that is flat is wrong.

### The rest of the vertical shape

- A **deceleration shelf**: a shallower segment around 10,000 ft where the aircraft slows
  to the 250 kt limit before descending below it.
- A **flare**: the sink rate is arrested over the last 15 m rather than the aircraft
  simply arriving at the ground still on the glideslope.
- **Touchdown 300 m past the threshold**, on the aiming markers, followed by the rollout
  along the runway. The previous profile put the aircraft at zero altitude three
  kilometres *before* the runway and then taxied it to the threshold.

## Speeds, and fitting the session

### Cruise speed is not the free variable

The session length is fixed by the timer and the route by the flight the pilot booked, so
something has to give. It used to be the cruise speed: a binary search set it to whatever
made the arithmetic work. Measured across the whole route database, that produced a
median cruise of 595 km/h, a quarter of all flights below 490, and the shortest sectors
down around 180 km/h — an A350 well below its stall speed, at altitude.

The mistake was treating the scheduled time as flying time. It is **block time**, gate to
gate, and the difference is taxi, climb and descent. The route database shows this
cleanly:

| Sector length | Median block speed |
|---|---|
| 0–500 km | 292 km/h |
| 1,500–2,000 km | 605 km/h |
| 8,000+ km | 794 km/h |

That curve rises toward, but never reaches, a true cruise speed of roughly 900 km/h. The
gap is exactly the overhead the old model ignored.

So the slack is now spent on the things that consume it in reality, in order:

1. **Taxi.** A fixed distance at each end, flown at whatever speed fills the time.
   Taxiing slowly in a queue is what actually happens.
2. **Cruise Mach**, within 0.78–0.86. Airlines fly this whole band depending on how the
   schedule is running — it is the cost index, and a legitimate knob rather than a fudge.
3. **A single time scale**, as the residual.

The scale is last and stays close to 1.0. It touches only the clock: the speeds in
telemetry stay physical, so the cockpit reads a real number even when the animation runs
slightly fast or slow. Because a final rescale is applied against what was actually
sampled, the flight lands exactly when the session ends.

### The speed schedule

Climb-out at 170 kt to the acceleration altitude, then 250 kt below 10,000 ft, then 300
kt, then Mach — with the crossover between the last two falling naturally around FL300.
Descent mirrors it. The low-altitude speed limit applies to cruise too, which matters for
short sectors: without it an aircraft levelling at FL040 cruises at nearly 400 knots.

This schedule is also why the initial climb looks right. Climbing out at 250 kt puts the
pitch attitude around 7°; at 170 kt on takeoff flap it is 12–13°, which is what an
airliner actually rotates to before lowering the nose a minute later to accelerate.

## Attitude

Derived from the path, never assumed.

**Pitch is the flight path angle plus the angle of attack.** The angle of attack model is
lift-equals-weight rearranged, with a flap term: extending flaps increases camber, so the
same lift comes at a markedly lower body angle. That term is what puts an airliner on a
3° glideslope *nose-up* rather than nose-down, and its absence is why deriving pitch from
the altitude gradient alone had the aircraft pointing visibly the wrong way on approach.

**Bank comes from the geodesic curvature of the track, not from the rate of change of
heading.** The distinction is not academic. Meridians converge, so an aircraft flying a
perfectly straight great circle sees its heading change continuously — by 180° if it
crosses a pole — while banking not at all. `GroundTrack::turn_angle_at` measures the
inbound and outbound directions *at the same point* so the convergence cancels, leaving
zero on a great circle at any latitude. Bank is then rate-limited to 5°/s, because
rolling into a turn takes a few seconds.

**Heading is track plus drift.** With a wind the nose points off the track, so the
aircraft visibly crabs — most noticeably in a crosswind on approach.

On the ground, bank is zero outright and pitch is zero until rotation, blended over a few
hundred metres at each end.

## Runway selection

Aircraft take off and land into wind, so the end in use is a property of the weather, not
of the airport, and it flips as the weather does. Each end is scored on its headwind
component, with runway length as the tie-break.

Headings are derived from the two threshold coordinates rather than read from the stored
heading column. The stored values are true bearings and agree with the computed ones to
about a quarter of a degree for almost every runway — but a handful of rows have their
ends transposed, and those disagree by 180°, which is the error that would be least
obvious and most wrong.

One emergent behaviour worth recognising: with a predominantly westerly surface field in
mid-latitudes, mid-latitude airports mostly use their westerly-facing ends. That is
genuinely what happens — Heathrow lands to the west roughly seven days in ten.

## Field elevation

**Off by default**, via `FlightPlanConfig::terrain_elevation`.

Everything downstream already handles real elevations: the takeoff roll lengthens in thin
air because rotation happens at a fixed *calibrated* airspeed, and cruise levels are
checked against the ground beneath them. The switch is off only because the globe
currently renders without terrain, so an aircraft sitting at Bogotá's 2,548 m would hang
visibly above a sea-level surface.

To turn it on once terrain exists: call `nativeSetFieldElevations` before loading a
flight (the export is present and unused), or `FlightHandle::set_plan_config` directly.
Nothing else needs to change.

## Testing

`crates/cesium-flight/tests/flight_plan.rs` pins the *shape* of a plan rather than its
constants, so retuning the aircraft model or the wind field does not mean rewriting the
tests. Run them with `cargo test -p cesium-flight`.

What they guard is, deliberately, the set of things that were once wrong:

- The route is a great circle, not a line on a flat map, and does not go the wrong way
  round the world.
- Closed airspace is avoided, and avoiding it costs distance.
- Climb and descent are at airliner angles; cruise is on a flight level with the right
  parity; long flights step up.
- Cruise speed is physical on every length of route, and the flight lasts exactly as long
  as the session.
- The aircraft is nose-up on final and level before rotation.
- Touchdown is past the threshold, not short of it.

Two are worth understanding before weakening them:

**`the_track_never_demands_more_bank_than_the_aircraft_may_use`** measures the curvature
of the sampled track and asks what bank flying it would require. Asserting on the
*reported* roll would pass even if the geometry called for 50°, because the reported value
is clamped to the limit. It is the clamp this test exists to catch.

**`closed_regions_do_not_straddle_the_antimeridian`** guards the containment test, which
is planar in latitude/longitude and only sound while no region wraps. Adding a region that
does would silently break avoidance rather than fail loudly.

## Known limits

- Airspace outlines are national rather than FIR boundaries, and coarse. The Russian
  Arctic sector stops at 179.9°E, so the Chukotka sliver east of the antimeridian is not
  covered.
- The wind field is climatology, not a forecast, and has no meridional component. Routes
  therefore differ by direction and season but not by day.
- Terrain is not consulted for anything but field elevation: there is no check that a
  cruise level clears the ground between the airports.
- ETOPS is not modelled. For an A350-900, certified to ETOPS-370, it would bend almost
  nothing outside the South Pacific.
- The semicircular rule uses true rather than magnetic track, as above.
