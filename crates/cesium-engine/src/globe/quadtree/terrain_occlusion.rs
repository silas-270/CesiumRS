//! Culling tiles that are hidden *behind mountains*, not merely behind the
//! Earth's limb.
//!
//! This is constraint 2 of the terrain plan and the one piece of it with no prior art:
//! Cesium culls against the ellipsoid limb, the frustum and fog, never against terrain
//! itself. Height-aware bounds gave nodes height-aware boxes and the relief-aware horizon test gave them a relief-safe limb test;
//! both throw away what is behind the *planet*. A valley behind a ridge is in front of
//! the planet and was still drawn in full.
//!
//! # The idea, and where the march went
//!
//! §3.3 describes marching the view cone from the eye to each candidate tile and asking
//! the elevation pyramid, at every step, how high the terrain is *guaranteed* to be
//! across the cone's footprint. That is exactly what happens here — but the march is
//! **amortised over the frame** rather than re-run per candidate, because the answer
//! does not depend on the candidate at all: it depends only on the camera, the bearing
//! and the range. So the march runs once, into a polar grid of
//! [`AZIMUTH_SECTORS`] × [`RANGE_RINGS`] cells around the camera, and each candidate
//! then costs a handful of lookups into that grid — the "16–32 lookups into a small
//! array" §3.3 budgets, with the pyramid walk paid once instead of once per tile.
//!
//! # Where the guaranteed ridge comes from, and why it is not the height cache
//!
//! The occluder needs a **lower bound on the terrain surface** over a whole footprint.
//! The quadtree already maintains exactly that, per node:
//! [`HeightBounds::floor`] is the minimum of the node's own height field over its own
//! ground, taken from the 16×16 min/max mip with the cell range rounded *outward*, and
//! widened downward by the measured per-level margin whenever the node is still on an
//! inherited interval. Nothing else in the engine is a sound floor for a node's ground,
//! and building a second, parallel query path into the height cache would have meant
//! measuring a second margin (§7's table, in the other direction) for a quantity height-aware bounds
//! already bounds.
//!
//! So the march reads the **tree**, not the cache: `QuadtreeManager::refresh_terrain_horizon`
//! walks the quadtree exactly as `refresh_extras` does, one pass per frame, and stamps
//! each node's floor into every cell its footprint can touch. The walk is a **partition**
//! of the globe — it descends from the roots and stops at nodes it does not descend into
//! — so every cell is covered by at least one node, and a cell's floor is the minimum
//! over every node stamped into it.
//!
//! # The direction of every bound, which is the only way this goes wrong
//!
//! The rule is easy to get backwards. Written out for this implementation:
//!
//! | quantity | bound | why |
//! |---|---|---|
//! | occluder height (`floor`) | **lower** | only terrain that is *definitely* there can definitely block |
//! | occludee elevation (`θ_max`) | **upper** | if even the tile's highest point is hidden, all of it is |
//! | footprint of a stamped node | **outward** | a node's low floor must reach every cell it could touch, so a notch drags the cell down |
//! | sectors a candidate spans | **outward** | the ridge is the **min** over them, so more sectors is weaker |
//! | rings a candidate is behind | **inward** | only a ridge strictly nearer than the tile can block it |
//!
//! Every one of those is "the choice that makes the claim weaker". Taking `h_max` for
//! the occluder is the intuitive choice and is precisely the false negative this engine
//! exists to prevent: by I-7 it deletes a whole subtree.
//!
//! **Lateral gaps come out for free.** A cell's floor is the minimum over its *entire*
//! wedge-annulus footprint, so a col or a notch in a ridge pulls the whole cell down and
//! no cull happens. The test fires only where the ridge is continuous across the cell —
//! which is exactly when the tile behind it is genuinely hidden.
//!
//! # Why it is sound, in one paragraph
//!
//! Fix a candidate point `p` of a node whose bounding sphere the test culled, and let
//! `β` be `p`'s bearing from the camera. The segment `eye → p` lies in the plane through
//! the Earth's centre spanned by `eye` and `p` (a plane through the origin containing
//! both points contains the segment), so its ground track is the great circle of initial
//! bearing `β` — i.e. it never leaves sector `β`. The test culled only because
//! `θ_max < ridge[β][j]`, and `ridge[β][j]` is the running maximum over rings `0..=j`,
//! so some ring `i ≤ j` has `elev(γ_far(i), floor[β][i]) > θ_max ≥ θ_p`. Two straight
//! lines through the eye cannot cross again, so the lower-elevation ray (`p`'s) is below
//! the higher one (the ridge point's) at every angular distance ahead, and in particular
//! at `γ_far(i)`, which lies between the eye and `p` because `γ_far(i) ≤ γ_far(j) ≤ γ_p`.
//! There the ray is below altitude `floor[β][i]`, and the terrain over that cell is
//! *everywhere* at or above `floor[β][i]`, so the ray is inside solid ground. It cannot
//! reach `p`. ∎
//!
//! # Unlike fog, this stage is sound — and that difference is the point
//!
//! A fog culling stage would **discard geometry that is genuinely visible**, which is why
//! [`super::fog`] only relaxes refinement and is not a stage at all: it would turn every
//! sweep in the culling harness red. [`super::quadtree::Stage::TerrainOcclusion`] discards
//! only geometry it has *proved* invisible, exactly like the limb and frustum stages. It
//! therefore lives in the terrain-mode default pipeline
//! ([`super::quadtree::CullPipeline::TERRAIN_DEFAULT`]) and is verified at FN = 0 by
//! `testing::terrain::test_terrain_occlusion`, rather than being kept out of the thing
//! that measures it.
//!
//! # Units
//!
//! **Megametres** for every length, like the rest of `SurfaceModel` and the quadtree;
//! **radians** for every angle. The two `*_m` fields on [`TerrainOcclusionConfig`] are
//! metres, because that is the unit a person picks a camera-altitude threshold in, and
//! they are converted once, in [`TerrainHorizon::begin`].

use glam::DVec3;

use super::bounding_volume::Frustum;

use crate::globe::geometry::{EARTH_RADIUS_A_F64, EARTH_RADIUS_B_F64};

/// Azimuth sectors the march is cut into — **96 since §7f**, i.e. 3.75° each.
///
/// The sector is the resolution at which a *notch* in a ridge is still allowed to let
/// sight through: a gap narrower than a sector does not prevent a cull on its own, it
/// only pulls that sector's floor down (which prevents the cull anyway, because the
/// floor is a minimum over the whole cell). Finer sectors therefore buy tighter floors,
/// not soundness, and cost a linear amount of both build time and per-candidate lookups.
///
/// # 24 was the wrong number, and only a balance could show it
///
/// §7d priced a finer grid as `96 × 96` and put it **second** on its list of three
/// levers, behind the safety constant, on the strength of how far it moved the *ridge
/// ceiling* — a necessary condition, not a sufficient one. Measured on what actually
/// matters instead, at `rendering::terrain_balance`'s Inn-valley pose (900 m, 114 tiles
/// drawn without terrain occlusion), the two axes are not comparable at all:
///
/// | sectors × rings | tiles removed | march | net at 146 µs a tile |
/// |---|--:|--:|--:|
/// | 24 × 48 (§7b–§7e) | 7 | 460 µs | +562 µs |
/// | 48 × 48 | 15 | 499 µs | +1 691 µs |
/// | 48 × 96 | 16 | 593 µs | +1 743 µs |
/// | **96 × 48** | **20** | **541 µs** | **+2 379 µs** |
/// | 96 × 96 | 21 | 651 µs | +2 415 µs |
/// | 144 × 48 | 21 | 572 µs | +2 421 µs |
/// | 192 × 192 | 24 | 1 084 µs | +2 420 µs |
///
/// **It is the azimuth that buys the culls, and it saturates at 96–144.** Rings buy
/// almost nothing past 48 — one tile from 48 to 96 — which is the opposite of what §7b's
/// `RANGE_RINGS` note found when the grid was 24 wide, and not a contradiction of it:
/// the radial resolution had already been fixed there, and what was left binding was the
/// lateral one. [`TerrainHorizon::occludes`] takes the **minimum** of the ridge over
/// every sector a candidate spans, so a 15° sector 10 km out is 2.6 km of ground, and one
/// side valley inside it takes the whole candidate's cull with it.
///
/// 96 rather than 144: one tile is not worth 31 µs a frame, and the cells are already
/// 55 kB of grid.
pub const AZIMUTH_SECTORS: usize = 96;

/// Range rings, logarithmically spaced between [`MIN_RANGE_M`] and
/// `TerrainOcclusionConfig::max_range_m`.
///
/// Logarithmic because the quantity that matters — the elevation angle to a ridge —
/// changes fast near the camera and slowly far from it. 48 rings over 0.5 km … 120 km is
/// a ratio of 1.12 per ring.
///
/// # The ring depth is what decides whether a ridge exists at all
///
/// A cell's floor is the minimum over its **whole radial extent**, so a ring deeper than
/// the ridge is wide averages the crest together with the valley in front of it and the
/// ridge disappears from the occluder. Measured on the ridge world, 22 km from a crest
/// whose usable width is about 2 km:
///
/// | rings | ratio | depth at 22 km | crest survives? |
/// |--:|--:|--:|---|
/// | 16 | 1.49 | 7.2 km | no — floor is the valley |
/// | 24 | 1.30 | 5.1 km | no |
/// | **48** | **1.12** | **2.4 km** | **yes — floor 3 243 m against a 3 400 m crest** |
///
/// This is the same failure the sub-cell floor grid fixes in the *lateral* direction
/// (`HeightBounds::floor_grid`), one axis over, and it went unnoticed for the same reason:
/// the stage still culls plenty against the curvature horizon while the mountains do
/// nothing at all.
///
/// # 48 survived §7f's sweep and [`AZIMUTH_SECTORS`]' 24 did not
///
/// The same grid sweep that took the azimuth from 24 to 96 tried the rings at 96 and 192
/// and kept 48: at the Inn-valley pose `96 × 48` removes 20 tiles for a 541 µs march and
/// `96 × 96` removes 21 for 651 µs. One tile is not worth 110 µs a frame. The radial
/// resolution was already the one that had been measured — by the table above — and it
/// was already enough; the lateral one had never been measured against anything but a
/// ridge ceiling.
pub const RANGE_RINGS: usize = 48;

/// Nearest range the march resolves, metres. Inside this the camera is effectively on
/// top of the ground and the elevation angle to it is meaningless.
pub const MIN_RANGE_M: f64 = 500.0;

/// Metres of safety per metre of the two lengths the march's placement error acts on —
/// the slope of [`ridge_safety_m`].
///
/// # What the safety distance covers, and why a flat 100 m was the wrong shape
///
/// [`TerrainHorizon::finish`] places a cell's wall with the *normal*-space rotation
/// `n = up·cos γ + b̂·sin γ`, which moves the **geodetic** latitude by `γ`. The `γ` it is
/// handed is the one `TerrainHorizon::extent_of` bins nodes and candidates with, and that
/// one is an equirectangular distance in the engine's **parametric** latitude (the `φ` of
/// `geometry::lon_lat_to_ecef_f64`, which names a tile row). The two latitudes differ by
/// up to 0.19°, and — what actually matters — their *metrics* differ: one parametric
/// radian is not one geodetic radian of ground. The meridian arc per parametric radian is
/// `√(a²sin²β + b²cos²β)` against the geodetic `M(φ)`, whose ratio runs from `1 + e²/2` at
/// the equator to `1 − e²/2` at the pole, so the relative discrepancy is
/// **`κ = e²/2 = 3.35·10⁻³`** — half the squared eccentricity, one power of the flattening,
/// and the sweep below measures exactly that number falling out of the two constructions.
///
/// So the wall stands an **along-track** distance of order `κ·s` from the ground its cell
/// actually bounds. (Across track the two constructions agree to first order — a purely
/// lateral offset `δ` changes the range by `δ²/2s`, which is second order and is not what
/// this constant is paying for.) That along-track offset turns into an *altitude* error
/// through two channels, and the second one is easy to miss:
///
/// * **Through the curvature drop.** An offset `κ·s` under the drop rate `s/R` is
///   `κ·s²/R` of altitude — the term that grows with range, and the one
///   that was predicted.
/// * **Through the tilt of a sightline that stands off the floor.** With the eye `Δalt`
///   above (or below) the cell floor, an along-track offset `δ` at horizontal distance `s`
///   moves the elevation angle by `δ·Δalt/(s²+Δalt²)`, and the metres of wall that buys
///   that tilt back is `δ·Δalt/s` — so with `δ = κ·s`, exactly `κ·|Δalt|`,
///   **independent of range**. This term is not in §7d's sketch and it is not negligible:
///   at the altitude gate it is 40 m. Shipping a range-only law would have relaxed the
///   occluder bound, silently, at precisely the poses a range-only sweep does not visit.
///
/// Together: **`Δh ≲ κ·(|Δalt| + s²/R)`**, which is what [`ridge_safety_m`] is. (The
/// curvature term's first-order coefficient is really `κ/2`; carrying the full `s²/R` is
/// the cheap conservative choice and is where a slice of the reserve below comes from.)
///
/// # The measurement, and the reserve
///
/// `test_terrain_occlusion::d3_ridge_safety_covers_its_placement_error` sweeps the two
/// constructions against each other over latitude × bearing × range × eye altitude × cell
/// floor — ±85° of latitude, every 5° of bearing, `MIN_RANGE_M` … `max_range_m` of range,
/// an eye from 2 m to 40 km (3.3× the altitude gate), floors from −60 km to +20 km (what a
/// node deep on an inherited interval can carry) — plus 200 000 off-grid draws from the
/// same box, and solves for the exact metres of lowering each case needs. The ratio it
/// reports is flat:
///
/// | where | `Δh` needed | `|Δalt| + s²/R` | ratio |
/// |---|--:|--:|--:|
/// | 2 km, the worst of the box | 67.26 m | 19 999 m | **0.003363** |
/// | 32 km | 207.37 m | 60 163 m | **0.003447** |
/// | 120 km (the worst corner of the box) | 562.02 m | 102 258 m | **0.005496** |
///
/// `0.003363` is `e²/2` to three digits, which is the check that the derivation above is
/// the right one rather than a curve that happens to fit; `0.005496` at the far corner is
/// the second-order term the first-order derivation leaves out.
///
/// **`2·10⁻²` is 3.6× the worst measured need over that whole box** (3.9× on the off-grid
/// pass), and 6× the first-order `κ`. That is the reserve, and it is **empirical, not
/// proved**: a dense grid plus a pseudo-random pass over a smooth five-parameter box is
/// evidence, not a bound. What *is* derived is the law's shape — a constant times
/// `|Δalt| + s²/R` — and the sweep's flat ratio is the evidence that the shape is right,
/// which is worth more than the factor on the front of it.
///
/// # The error this does **not** cover, and why relaxing was still safe
///
/// A wall is built on its sector's *centre* bearing and has to hold against a ray up to
/// half a sector — 7.5° — away. Over that spread the ellipsoid's Euler radius moves by a
/// quarter of `κ`, which the reserve absorbs; but `TerrainHorizon::begin` takes `up` from
/// `ellipsoid_normal_at`, the **confocal** normal rather than the geodetic one, so the
/// eye's own up-axis misses the ground point `cam_lon`/`cam_lat` names by 7 mm at eye
/// height and **40 m at the 12 km gate**. Swing a ground point half a sector round and it
/// is that much further from the eye's axis — worth `|Δalt|/s` metres of wall each.
///
/// That error has the wrong *shape* for this constant: it grows as the range falls, where
/// both of `ridge_safety_m`'s terms shrink, and no affordable distance covers it (at 40 km
/// of altitude the worst case asks for 6 km of wall). It is also **pre-existing** — it is a
/// property of the 24-sector grid and of `ellipsoid_normal_at`, not of this constant.
/// `test_terrain_occlusion::d3_ridge_safety_never_falls_below_the_constant_it_replaced`
/// measures it and pins the two things that make relaxing the constant safe anyway:
///
/// * **under the shipped altitude gate the law covers it too**, over the whole box with
///   the bearing spread included — 0 cases short against the flat 100 m's 124 169; and
/// * **there is no case anywhere in the box where this law is short and the flat 100 m was
///   not.** The relaxation strictly shrinks the set of geometries where the grid's wall can
///   stand over its ground; it does not trade one hole for another. (Over that box the
///   flat constant's own worst reserve is **0.017×**.)
///
/// # What it buys
///
/// The old constant was a flat 100 m at every range and every stand-off. §7d measured what
/// that cost where terrain occlusion was specified to pay: **0.55° of occluder at Reutlingen and 2.23° at
/// Stuttgart**, thrown away before a single candidate was tested. At those two poses this
/// law reads **8 m and 6 m**.
pub const RIDGE_SAFETY_RATE: f64 = 2.0e-2;

/// Range- and stand-off-independent part of [`ridge_safety_m`], metres.
///
/// Covers what neither term of [`RIDGE_SAFETY_RATE`]'s law scales with:
/// `TerrainHorizon::elevation_along` measuring the altitude radially rather than along the
/// normal (sub-centimetre at 3 km of relief), and the `f32` the finished
/// `TerrainHorizon::ridge` grid is stored in (a relative 6·10⁻⁸ of a radian, i.e. tens of
/// microns of wall at the nearest ring). 1 m is two orders on the sum of those, and it
/// keeps the safety distance from collapsing to nothing for a camera standing on its own
/// cell floor.
pub const RIDGE_SAFETY_FLOOR_M: f64 = 1.0;

/// Metres subtracted from a cell floor before its elevation angle is taken: ground range
/// `range_m` to the cell, `alt_diff_m` the eye's height above or below that floor.
///
/// See [`RIDGE_SAFETY_RATE`] for the derivation and the measured reserve. In
/// [`TerrainHorizon::finish`] the `s²/R` half is hoisted per ring and only an `abs`, an
/// add and a multiply are left per cell — 1 152 of them a frame, which is why
/// `bench_terrain_occlusion_cost` cannot see it.
#[inline]
#[cfg_attr(not(feature = "testing"), allow(dead_code))]
pub fn ridge_safety_m(range_m: f64, alt_diff_m: f64) -> f64 {
    const R_M: f64 = EARTH_RADIUS_A_F64 * 1.0e6;
    RIDGE_SAFETY_FLOOR_M + RIDGE_SAFETY_RATE * (alt_diff_m.abs() + range_m * range_m / R_M)
}

/// Sectors of slop added on each side of the view cone before the march decides a sector
/// is dead — see [`TerrainHorizon::live`].
///
/// Eight sectors. Not for soundness (a dead sector is `−∞` and culls nothing) but for the
/// culls at the edge of the frame: [`TerrainHorizon::occludes`] takes the **minimum** over
/// every sector a candidate spans, so a tile straddling the frustum edge would lose its
/// cull to a neighbouring dead sector. A coarse candidate spans several sectors, and the
/// margin is what keeps the ones just inside the frame whole.
pub const LIVE_SECTOR_MARGIN: usize = 8;

/// Fractional slack applied to every angular extent — see
/// [`TerrainHorizon::extent_of`].
pub const EXTENT_SLACK: f64 = 0.10;

/// The knobs terrain occlusion is gated by. Lives on
/// [`TerrainConfig`](crate::globe::tiles::config::TerrainConfig) as `occlusion`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TerrainOcclusionConfig {
    /// Master switch for the stage. `false` leaves the terrain pipeline at height-aware bounds and the relief-aware horizon test.
    pub enabled: bool,
    /// Camera altitude above the ellipsoid, in **metres**, above which the march is not
    /// built at all and [`super::quadtree::Stage::TerrainOcclusion`] answers `Undecided`
    /// on its first comparison.
    ///
    /// Measured, not guessed — see
    /// `testing::terrain::test_terrain_occlusion::d3_altitude_gate_is_where_the_benefit_stops`,
    /// which sweeps the reduction against altitude at two crest heights and prints a
    /// table. It has three regimes: **−28 %** at 800 m and
    /// **−17 %** at 1.5 km — the valley, which is what this feature exists for — a **−4 %**
    /// plateau from 3 km to 8 km, and **exactly zero** from 12 km up, on an Alpine and a
    /// Himalayan crest alike.
    ///
    /// The gate was put at the first altitude that measures zero rather than at the knee,
    /// on the argument that shutting the stage off at the knee would give up a real 4.3 %
    /// at 5 and 8 km to save a march that runs once a frame. **§7f retired that
    /// argument**: a 4.3 % reduction is not a benefit until it is worth more than the
    /// march, and measured against the frame it is not. [`Self::max_camera_agl_m`] is now
    /// the threshold that decides, and it is written in the variable that matters —
    /// height above the **ground**, not above the ellipsoid, because over a 2 000 m
    /// valley floor a camera 1 000 m up reads 3 000 m here and it is the 1 000 m that
    /// decides what a ridge can hide.
    ///
    /// This one stays as the **outer guard**, unchanged at 12 000 m: `altitude_agl`
    /// degrades to `altitude` when there is no ground sample yet, so something has to
    /// shut the march off over an ocean at 400 km on a cold cache.
    pub max_camera_altitude_m: f32,
    /// Camera altitude above the **ground**, in metres, above which the march is not
    /// built at all — **§7f's gate, and the one the balance picks**.
    ///
    /// [`Self::max_camera_altitude_m`] is a ceiling on the altitude above the
    /// *ellipsoid*, put at 12 000 m because that is the first altitude at which §7b's
    /// synthetic ladder measured the stage removing **zero** tiles. Under the balance
    /// rule that is the wrong question: the march costs what it costs whether or not it
    /// removes anything, so the threshold has to be where the tiles it removes stop
    /// **paying for it**. It also measured the wrong variable — over a 2 000 m valley
    /// floor a camera 1 000 m up reads 3 000 m of ellipsoid altitude, and it is the
    /// 1 000 m that decides what a ridge can hide.
    ///
    /// `rendering::terrain_balance::terrain_balance_altitude_ladder` walks the real DEM
    /// through the real renderer at one horizontal pose and reads both halves at once —
    /// at the shipped `96 × 48` grid, and with the frame time the two arms actually took:
    ///
    /// | AGL | tiles without occlusion | with | removed | march | frame |
    /// |--:|--:|--:|--:|--:|--:|
    /// | 67 m | 126 | 104 | **22** | 539 µs | **−1 877 µs** |
    /// | 317 m | 114 | 94 | **20** | 546 µs | **−1 734 µs** |
    /// | 717 m | 106 | 95 | **11** | 537 µs | **−2 290 µs** |
    /// | 1 217 m | 106 | 105 | 1 | 498 µs | +825 µs |
    /// | 1 917 m | 102 | 101 | 1 | 475 µs | −277 µs |
    /// | 2 917 m | 94 | 93 | 1 | 461 µs | +40 µs |
    /// | 4 418 m | 87 | 87 | **0** | 436 µs | +1 066 µs |
    ///
    /// The benefit falls off a cliff between 717 m and 1 217 m — eleven tiles to one —
    /// while the march keeps costing half a millisecond, so **1 000 m** is where the
    /// threshold goes. The three rungs under it are also the only ones whose frame-time
    /// column clears the machine's noise floor (±1.5 ms, calibrated on the cruise pose,
    /// where terrain occlusion removes nothing and the frame time still moves by that much); above the
    /// cliff the column is noise around zero, which is exactly what one tile against half
    /// a millisecond should look like.
    ///
    /// [`Self::max_camera_altitude_m`] is kept as the outer guard, for the case this one
    /// cannot answer: `Camera::altitude_agl` falls back to `altitude` when no ground
    /// sample exists, so on the first frames of a cold start, or with the height cache
    /// empty, AGL reads as ellipsoid altitude and the 12 km ceiling is what shuts the
    /// march off over an ocean at 400 km.
    pub max_camera_agl_m: f32,
    /// How far out the march looks, **metres** — where *occluders* are looked for, not how
    /// far an occludee may be. A tile 500 km out behind a ridge 11 km out is exactly what
    /// terrain occlusion is for, and it is tested against the ridge, not skipped.
    ///
    /// Beyond this the cells grow deep enough that a ridge's crest is averaged with the
    /// ground in front of it and the floor stops being a ridge at all (see
    /// [`RANGE_RINGS`]), so extending it buys nothing and costs rings. 120 km is also past
    /// the Earth's own horizon for any camera under the altitude gate — 39 km at 120 m,
    /// 124 km at 1.2 km, 390 km at 12 km — so the band where terrain rather than the limb
    /// is the occluder is comfortably inside it.
    pub max_range_m: f32,
    /// **§7g's pre-check**: how much provable relief has to stand above the eye, as an
    /// **elevation angle in degrees**, before the march is built at all.
    ///
    /// [`Self::max_camera_agl_m`] is §7f's gate and it measures the wrong thing for four
    /// of the five poses it lets through: §7f's own closing section says "the
    /// discriminator is not height, it is **relief in view**", and its table has
    /// `alps_approach` at 584 m AGL and `alps_cockpit` at 2 m each spending ~460 µs to
    /// remove nothing beside `alps_inn_valley` at 317 m removing twenty.
    ///
    /// The statistic is
    /// [`ReliefProbe`](super::terrain_relief::ReliefProbe)'s: the largest elevation angle,
    /// taken at each visible leaf's **nearest** point, of the highest altitude height-aware bounds
    /// *guarantees* over it. Measured over 34 poses in six families (`terrain::
    /// test_terrain_relief::terrain_relief_statistic_separates_the_family`) it tracks the
    /// removal closely — the nine poses where terrain occlusion removes nine tiles or more all read
    /// **8.79° or above**, they carry 158 of the family's 168 removed tiles between the
    /// seventeen poses above the threshold, and every plain, every coast and every
    /// above-the-relief pose reads under 5°.
    ///
    /// **8.0° is where the threshold goes**, and it is a keep-side choice inside a band
    /// the data says nobody can split: the lowest-reading pose at which terrain occlusion still removes
    /// a paying number of tiles is the Inn valley at 700 m AGL — §7f's own ladder rung,
    /// **11 tiles** — at 8.79°, and the two non-paying poses just above it read 9.37° and
    /// 9.72°. Anything from 8° to 10° is worth the same to within a hundred microseconds
    /// over the family; 8° is the end that keeps the paying pose rather than the end that
    /// trades it for two break-evens.
    ///
    /// `f32::NEG_INFINITY` disables the pre-check and restores §7f's behaviour exactly.
    /// Setting it *above* the relief of a pose can only cost tiles that would have been
    /// culled, never show one that should have been hidden — see
    /// [`super::terrain_relief`]'s module doc.
    pub min_relief_deg: f32,
}

impl TerrainOcclusionConfig {
    /// The altitude gates, as one predicate — §7f's pair, in the one place both
    /// [`TerrainHorizon::begin`] and the pre-check's caller read them from.
    #[inline]
    pub fn gate_shut(&self, cam_alt: f64, cam_agl: f64) -> bool {
        !self.enabled
            || cam_agl * 1.0e6 > self.max_camera_agl_m as f64
            || cam_alt * 1.0e6 > self.max_camera_altitude_m as f64
    }
}

impl Default for TerrainOcclusionConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            max_camera_altitude_m: 12_000.0,
            max_camera_agl_m: 1_000.0,
            max_range_m: 120_000.0,
            min_relief_deg: 8.0,
        }
    }
}

/// What [`TerrainHorizon::classify`] says about one node of the occluder walk.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OccluderStep {
    /// Entirely beyond the march's range. Neither it nor its subtree can touch a cell.
    Skip,
    /// Small enough that descending would not refine any cell it touches. Stamp it.
    Stamp,
    /// Big enough that its children have tighter floors over the cells it covers.
    /// Descend if it has any; otherwise stamp it.
    Descend,
}

/// The per-frame result of the march: a guaranteed ridge elevation angle per
/// (sector, ring), already accumulated so one lookup answers "is there anything nearer
/// than ring `j`, in sector `a`, that reaches above angle θ".
///
/// Built once per frame by `QuadtreeManager::refresh_terrain_horizon` and handed to the
/// stages through `CullContext::terrain` as a shared reference — deliberately **not** by
/// value: `CullContext` is `Copy` and is built fresh every frame on both arms, so 6 kB of
/// grid in it would be a memcpy the flat path pays for a structure it can never use.
#[derive(Clone, Debug)]
pub struct TerrainHorizon {
    /// `false` disables every query. Set when terrain occlusion is switched off, when
    /// the camera is above the altitude gate, and when the camera is below the
    /// guaranteed ground in every direction (see [`Self::finish`]).
    active: bool,
    eye: DVec3,
    /// Ellipsoid normal at the eye — the "up" every elevation angle is measured against.
    up: DVec3,
    east: DVec3,
    north: DVec3,
    /// Camera altitude above the ellipsoid, megametres.
    cam_alt: f64,
    /// Camera ground position, degrees — the apex every bearing and range is measured
    /// from, and what a tile's lon/lat rectangle is clamped against.
    cam_lon: f64,
    cam_lat: f64,
    /// Lowest guaranteed terrain altitude over each cell, megametres.
    /// `f64::INFINITY` means "no node has been stamped here yet".
    floor: Box<[[f64; RANGE_RINGS]; AZIMUTH_SECTORS]>,
    /// `max` over rings `0..=r` of the elevation angle to the top of that ring's
    /// guaranteed ridge, radians. `f32::NEG_INFINITY` where nothing is guaranteed.
    ridge: Box<[[f32; RANGE_RINGS]; AZIMUTH_SECTORS]>,
    /// Bitmask of the sectors the march actually builds — **§7f's view cone**.
    ///
    /// # Why most of the grid was never readable
    ///
    /// [`Self::occludes`] reads `ridge[a][j]` only for the sectors a *candidate* spans,
    /// and every candidate that a cull could save work on is one the renderer would
    /// otherwise draw, i.e. one inside the frustum. The march nonetheless stamped and
    /// finished all 24 sectors, so at a 76° horizontal field of view roughly **two thirds
    /// of the polar grid was built for nobody**: the occluders behind the camera were
    /// walked, their sub-cells' bearings and ranges were computed, and
    /// [`Self::finish`] took an elevation angle for every one of their cells.
    ///
    /// A sector outside this mask keeps `f32::NEG_INFINITY` in [`Self::ridge`] for every
    /// ring, and `occludes` takes the **minimum** over the sectors a candidate spans — so
    /// a candidate that reaches into a dead sector is simply never culled. That is the
    /// same weakening a missing occluder is, and it needs no separate soundness argument:
    /// nothing the mask does can make a wall stand higher.
    ///
    /// The mask is derived from the frustum's four edge rays in [`live_sectors`], widened
    /// by [`LIVE_SECTOR_MARGIN`], and falls back to *all sectors* whenever the view cone's
    /// azimuth cannot be bounded — a camera looking at the nadir or the zenith, a
    /// frustum built without corners, or a spread of half a turn or more.
    ///
    /// An array rather than a bitmask so that [`AZIMUTH_SECTORS`] can be swept past 32
    /// without the mask silently truncating; it is one per *manager*, not per node.
    live: [bool; AZIMUTH_SECTORS],
    /// `1 / ln(step)` for [`Self::ring_of`]'s closed form.
    inv_ln_step: f64,
    /// Angular distance (radians) of each ring's **far** edge, increasing.
    ring_far: [f64; RANGE_RINGS],
    /// `ring_far[RANGE_RINGS - 1]`, hoisted: the prune radius of the walk.
    max_ang: f64,
    /// The highest elevation angle any sector reaches at any range — `max` over the whole
    /// [`Self::ridge`] grid, hoisted.
    ///
    /// The hot per-node filter. A candidate whose *upper* bound already sits at or above
    /// this cannot be under any ridge in any sector, so it is settled before a single
    /// angular quantity is computed. It is the right way round to filter: the naive
    /// alternative — rejecting candidates beyond the march's range — is **wrong**, and
    /// measurably so. Beyond that range is exactly where a ridge's shadow lands: the
    /// march limits where occluders are looked for, not how far an occludee may be, and a
    /// tile 500 km out behind a ridge 11 km out is precisely the case terrain occlusion exists for. A
    /// far-field cut-off cost the ridge world's valley pose its entire 20 % reduction.
    ridge_ceiling: f32,
}

/// Point of the ellipsoid whose outward unit normal is `n`.
///
/// From the implicit form alone, so it is independent of which latitude convention
/// `lon_lat_to_ecef_f64` happens to use: a surface point satisfies
/// `p = λ·(a²n_x, b²n_y, a²n_z)` (the normal is the gradient), and substituting into
/// `p_x²/a² + p_y²/b² + p_z²/a² = 1` gives `λ = 1/√(a²(n_x²+n_z²) + b²n_y²)`. The
/// engine's polar axis is **y** (`geometry::lon_lat_to_ecef_f64`).
#[inline]
pub(crate) fn surface_point_for_normal(n: DVec3) -> DVec3 {
    const A2: f64 = EARTH_RADIUS_A_F64 * EARTH_RADIUS_A_F64;
    const B2: f64 = EARTH_RADIUS_B_F64 * EARTH_RADIUS_B_F64;
    let lambda = 1.0 / (A2 * (n.x * n.x + n.z * n.z) + B2 * (n.y * n.y)).sqrt();
    DVec3::new(A2 * n.x, B2 * n.y, A2 * n.z) * lambda
}

/// The camera's local ground frame: the ellipsoid normal at the eye, the east and north
/// axes every bearing is measured against, and the eye's own longitude and latitude.
///
/// Lifted out of [`TerrainHorizon::begin`] **unchanged** — the same expressions in the
/// same order, so the march's numbers are the ones it always had — because
/// [`super::terrain_relief::ReliefProbe`] needs exactly this and needs it *before*
/// `begin` allocates the grid. A second, parallel derivation of the camera's ground
/// position is the one thing this file cannot afford: getting it wrong by one power of
/// the flattening moved the camera 10.6 km and cost the sweep 122 false negatives (see
/// [`Self::at`]).
#[derive(Clone, Copy, Debug)]
pub struct GroundFrame {
    /// Outward ellipsoid normal at the eye — the "up" every elevation angle is measured
    /// against.
    pub up: DVec3,
    pub east: DVec3,
    pub north: DVec3,
    /// The eye's own ground longitude, degrees.
    pub cam_lon: f64,
    /// The eye's own ground latitude, degrees.
    pub cam_lat: f64,
}

impl GroundFrame {
    pub fn at(eye: DVec3) -> Self {
        let up = ellipsoid_normal_at(eye);
        // East analytically from the normal's horizontal part, the same construction
        // `quadtree::tangent_frame` uses and for the same reason: no `cos φ → 0` division
        // and no non-orthogonal fallback at a pole. `up ∝ (cos λ, ·, −sin λ)` in this
        // engine's frame, so `(u_z, 0, −u_x)` normalised is due east.
        let m = (up.x * up.x + up.z * up.z).sqrt();
        let east = if m > 1.0e-12 {
            DVec3::new(up.z / m, 0.0, -up.x / m)
        } else {
            DVec3::X
        };
        let north = up.cross(east).normalize();
        // The camera's own ground position, inverted from the ellipsoid normal.
        //
        // `geometry::lon_lat_to_ecef_f64` parameterises the surface as
        // `(a cos φ cos θ, b sin φ, −a cos φ sin θ)`, whose gradient is
        // `(cos φ cos θ / a, sin φ / b, −cos φ sin θ / a)`. So the horizontal part gives
        // `θ` directly, and `n_y / ‖n_h‖ = (a/b)·tan φ`, i.e. **`tan φ = (b/a)·n_y/‖n_h‖`**
        // — one power of the flattening, not two. Squaring both radii here (the obvious
        // slip, and the one that was made) moves the camera 0.096° of latitude, **10.6 km**
        // on the ground, which puts every bearing and every range in this file at the
        // wrong place and cost the sweep 122 false negatives.
        let cam_lon = (-up.z).atan2(up.x).to_degrees();
        let cam_lat = (up.y * EARTH_RADIUS_B_F64)
            .atan2(m * EARTH_RADIUS_A_F64)
            .to_degrees();
        Self {
            up,
            east,
            north,
            cam_lon,
            cam_lat,
        }
    }
}

impl TerrainHorizon {
    /// A horizon that never occludes anything — the flat arm's value, and the terrain
    /// arm's whenever the gate is shut.
    pub fn inactive() -> Self {
        Self {
            active: false,
            eye: DVec3::ZERO,
            up: DVec3::Y,
            east: DVec3::X,
            north: DVec3::Z,
            cam_alt: 0.0,
            cam_lon: 0.0,
            cam_lat: 0.0,
            floor: Box::new([[f64::INFINITY; RANGE_RINGS]; AZIMUTH_SECTORS]),
            ridge: Box::new([[f32::NEG_INFINITY; RANGE_RINGS]; AZIMUTH_SECTORS]),
            live: [false; AZIMUTH_SECTORS],
            inv_ln_step: 1.0,
            ring_far: [0.0; RANGE_RINGS],
            max_ang: 0.0,
            ridge_ceiling: f32::NEG_INFINITY,
        }
    }

    /// Starts a march for this frame's camera, or returns an inactive horizon when the
    /// altitude gate is shut.
    ///
    /// `frustum` supplies the eye and — through its four edge rays — the azimuth band
    /// the march is built over (see [`Self::live`]); `cam_alt` is the camera's altitude
    /// above the ellipsoid in megametres, and `cam_agl` its altitude above the **ground**,
    /// which is what the gate is measured in.
    pub fn begin(
        frustum: &Frustum,
        cam_alt: f64,
        cam_agl: f64,
        cfg: &TerrainOcclusionConfig,
    ) -> Self {
        let eye = frustum.eye;
        if cfg.gate_shut(cam_alt, cam_agl) {
            return Self::inactive();
        }

        let GroundFrame {
            up,
            east,
            north,
            cam_lon,
            cam_lat,
        } = GroundFrame::at(eye);

        // Log-spaced ring far edges, as angular distances. `EARTH_RADIUS_A_F64` rather
        // than a mean radius: a *larger* radius makes each ring's angular extent
        // *smaller*, so a candidate needs to be slightly further out before a given ring
        // counts as being in front of it — the conservative direction.
        let near = MIN_RANGE_M * 1.0e-6 / EARTH_RADIUS_A_F64;
        let far = cfg.max_range_m as f64 * 1.0e-6 / EARTH_RADIUS_A_F64;
        let step = (far / near).powf(1.0 / (RANGE_RINGS - 1) as f64);
        let mut ring_far = [0.0; RANGE_RINGS];
        let mut g = near;
        for slot in ring_far.iter_mut() {
            *slot = g;
            g *= step;
        }

        Self {
            active: true,
            eye,
            up,
            east,
            north,
            cam_alt,
            cam_lon,
            cam_lat,
            floor: Box::new([[f64::INFINITY; RANGE_RINGS]; AZIMUTH_SECTORS]),
            ridge: Box::new([[f32::NEG_INFINITY; RANGE_RINGS]; AZIMUTH_SECTORS]),
            live: live_sectors(up, east, north, frustum),
            inv_ln_step: 1.0 / step.ln(),
            ring_far,
            max_ang: ring_far[RANGE_RINGS - 1],
            ridge_ceiling: f32::NEG_INFINITY,
        }
    }

    /// Is the march live at all? A `false` here is what makes the stage free.
    #[inline]
    pub fn is_active(&self) -> bool {
        self.active
    }

    /// Where a tile's **ground rectangle** sits relative to the camera: the angular
    /// distance to its centre, an upper bound on its angular radius, and the angular
    /// distance to its *nearest* point.
    ///
    /// # Why a rectangle and not a bounding sphere
    ///
    /// Measured, after a version that used the box's circumradius as an angular footprint
    /// produced a march in which every cell of the grid held the same number. A tile's
    /// box is a chord-bounded solid, and for a coarse tile its circumradius is most of the
    /// Earth: a z1 root reports an angular radius of 1.2 rad, so *every* root "covers" the
    /// camera's neighbourhood, and a root that the walk cannot descend into (its children
    /// were culled, or it is the first frame) stamps its whole-quadrant minimum over the
    /// entire march. Sound — that minimum really is a lower bound on its ground — and
    /// completely useless: on the ridge world it flattened every cell to the 600 m plateau
    /// and left terrain occlusion culling against nothing but the curvature horizon.
    ///
    /// The rectangle answers the question that was actually being asked. `near` clamps the
    /// camera's own ground position into the tile's lon/lat span, so a root in the next
    /// hemisphere is 860 km away rather than zero, and the walk prunes it.
    ///
    /// All three are on the small-angle spherical approximation with the **polar** radius,
    /// i.e. rounded outward: `near` comes out too small and the radius too large, which is
    /// the conservative direction on both sides of this file (see [`Self::stamp`] and
    /// [`Self::occludes`]).
    pub fn extent_of_pub(&self, b: &super::tile_id::TileBounds) -> (f64, f64, f64) {
        self.extent_of(b)
    }

    #[inline]
    fn extent_of(&self, b: &super::tile_id::TileBounds) -> (f64, f64, f64) {
        extent_of_at(self.cam_lon, self.cam_lat, b)
    }
}

/// [`TerrainHorizon::extent_of`]'s body, as a free function of the camera's ground
/// position — moved here **verbatim** so [`super::terrain_relief::ReliefProbe`] can ask
/// the same question of the same rectangle without a second approximation and without a
/// second `EXTENT_SLACK` to keep in step with this one.
pub(crate) fn extent_of_at(
    cam_lon: f64,
    cam_lat: f64,
    b: &super::tile_id::TileBounds,
) -> (f64, f64, f64) {
    let lat_c = 0.5 * (b.lat_min + b.lat_max);
    let lon_c = 0.5 * (b.lon_min + b.lon_max);
    let dlat = (b.lat_max - b.lat_min).to_radians() * 0.5;
    // The east-west half-extent shrinks with latitude; take it at whichever edge is
    // nearer the equator, which is the wider one.
    let cos_lat = b
        .lat_min
        .abs()
        .min(b.lat_max.abs())
        .to_radians()
        .cos()
        .max(0.0);
    let dlon = (b.lon_max - b.lon_min).to_radians() * 0.5 * cos_lat;
    let gr = (dlat * dlat + dlon * dlon).sqrt();

    let ang = |lon: f64, lat: f64| -> f64 {
        let (dlo, dla) = (
            wrap_deg(lon - cam_lon).to_radians(),
            (lat - cam_lat).to_radians(),
        );
        let e = dlo * (0.5 * (lat + cam_lat)).to_radians().cos();
        (dla * dla + e * e).sqrt()
    };

    let gamma = ang(lon_c, lat_c);
    // Nearest point of the rectangle: the camera's own position clamped into it.
    let clamped_lat = cam_lat.clamp(b.lat_min, b.lat_max);
    let clamped_lon = {
        let d = wrap_deg(cam_lon - lon_c);
        let half = 0.5 * (b.lon_max - b.lon_min);
        lon_c + d.clamp(-half, half)
    };
    // Both ends rounded outward. The equirectangular `ang` above and the
    // clamped-point construction of `near` are small-angle approximations of a
    // great-circle distance on an ellipsoid, and their error goes the wrong way on
    // both counts: a `near` that is too large lets a ring that is not actually in
    // front of the tile cull it, and a `gr` that is too small narrows the sector range
    // the ridge is minimised over. `EXTENT_SLACK` is what keeps the approximation from
    // being load-bearing — measured, not assumed: without it the ridge world's sweep
    // reports 131 false negatives, with it none.
    let near =
        (ang(clamped_lon, clamped_lat) * (1.0 - EXTENT_SLACK) - gr * EXTENT_SLACK).max(0.0);
    (gamma, gr * (1.0 + EXTENT_SLACK), near)
}

impl TerrainHorizon {
    /// Bearing of `center` from the camera, radians clockwise from north.
    #[inline]
    fn bearing(&self, center: DVec3) -> f64 {
        let v = center - self.eye;
        let h = v - self.up * v.dot(self.up);
        h.dot(self.east).atan2(h.dot(self.north))
    }

    /// The half-open sector index range `[lo, hi)` (taken modulo [`AZIMUTH_SECTORS`])
    /// a disc of angular radius `gr` at angular distance `gamma` and bearing `beta`
    /// can touch, rounded **outward**.
    ///
    /// `None` means "every sector": the disc either contains the camera's ground point or
    /// is wide enough that no bearing excludes it.
    #[inline]
    fn sector_range(&self, gamma: f64, gr: f64, beta: f64) -> Option<(isize, isize)> {
        if gamma <= gr {
            return None;
        }
        let sin_g = gamma.sin();
        if sin_g <= 1.0e-12 {
            return None;
        }
        let ratio = gr.sin() / sin_g;
        if ratio >= 1.0 {
            return None;
        }
        let half = ratio.asin();
        let scale = AZIMUTH_SECTORS as f64 / std::f64::consts::TAU;
        let lo = ((beta - half) * scale).floor() as isize;
        let hi = ((beta + half) * scale).ceil() as isize;
        if hi - lo >= AZIMUTH_SECTORS as isize {
            return None;
        }
        Some((lo, hi))
    }

    /// How to treat one node of the occluder walk.
    ///
    /// `Descend` whenever the node's angular radius is larger than half the *radial*
    /// extent of the rings it sits on: below that, its children's footprints all land in
    /// the same cells and refining buys nothing. Combined with `Skip`, this is what keeps
    /// the walk proportional to the tree that actually exists near the camera rather than
    /// to the whole tree.
    /// Is this sector one the march builds? See [`Self::live`].
    #[inline]
    fn is_live(&self, a: usize) -> bool {
        self.live[a]
    }

    /// How many sectors this march builds — a read-only window for tests and debug
    /// readouts. [`AZIMUTH_SECTORS`] means the view cone could not be bounded.
    #[inline]
    pub fn live_sector_count(&self) -> usize {
        self.live.iter().filter(|b| **b).count()
    }

    pub fn classify(&self, bounds: &super::tile_id::TileBounds, center: DVec3) -> OccluderStep {
        if !self.active {
            return OccluderStep::Skip;
        }
        let (gamma, gr, near) = self.extent_of(bounds);
        if near > self.max_ang {
            return OccluderStep::Skip;
        }
        // **§7f: out of the view cone, out of the walk.** A node whose footprint misses
        // every live sector cannot write a cell that will ever be read, and neither can
        // anything under it — its descendants' footprints are subsets of its own. This is
        // where the mask pays: not in the cells it leaves unfinished, but in the subtrees
        // it never descends into. `sector_range` rounds outward, so a node that touches a
        // live sector at all is kept.
        if let Some((lo, hi)) = self.sector_range(gamma, gr, self.bearing(center)) {
            let mut touches = false;
            for k in lo..=hi {
                if self.is_live(k.rem_euclid(AZIMUTH_SECTORS as isize) as usize) {
                    touches = true;
                    break;
                }
            }
            if !touches {
                return OccluderStep::Skip;
            }
        }
        // Radial extent of the ring the node's near edge falls in.
        let j = self.ring_of(near);
        let width = match j {
            0 => self.ring_far[0],
            j => self.ring_far[j] - self.ring_far[j - 1],
        };
        // A stamped node contributes an `OCCLUDER_GRID × OCCLUDER_GRID` grid, not one
        // floor, so the resolution that has to beat the cell is the *sub-cell's*.
        let sub_gr = gr / crate::globe::terrain::heightfield::OCCLUDER_GRID as f64;
        if sub_gr > 0.5 * width {
            OccluderStep::Descend
        } else {
            OccluderStep::Stamp
        }
    }

    /// Index of the ring whose far edge is the first at or beyond `gamma`.
    ///
    /// # Closed form, then corrected — §7f
    ///
    /// This was a linear scan of up to 48 comparisons, and it is called three times per
    /// stamped sub-cell (twice here, once from [`Self::classify`]) on 58–96 nodes a
    /// frame, i.e. some five thousand times. The rings are geometric, so the index is
    /// `ceil(ln(γ/near) / ln(step))` — but a `ln` and a rounding are not bit-identical to
    /// the scan at a ring boundary, and which side of a boundary a stamp lands on decides
    /// whether it may claim that ring's far edge at all (see [`Self::stamp`]'s note on the
    /// 122 false negatives). So the closed form is used as a **seed** and then walked to
    /// the exact answer against `ring_far` itself, which is at most a step or two and
    /// leaves the predicate — `the first i with γ ≤ ring_far[i]` — identical by
    /// construction rather than by argument.
    #[inline]
    fn ring_of(&self, gamma: f64) -> usize {
        if gamma <= self.ring_far[0] {
            return 0;
        }
        let mut i = ((gamma / self.ring_far[0]).ln() * self.inv_ln_step).ceil();
        if !(i >= 0.0) {
            i = 0.0;
        }
        let mut i = (i as usize).min(RANGE_RINGS - 1);
        while i > 0 && gamma <= self.ring_far[i - 1] {
            i -= 1;
        }
        while i + 1 < RANGE_RINGS && gamma > self.ring_far[i] {
            i += 1;
        }
        i
    }

    /// Records one occluder: terrain over this node's ground is guaranteed to be at or
    /// above `floor` megametres.
    ///
    /// The node's footprint is rounded outward to whole cells, and each cell keeps the
    /// **minimum** floor stamped into it. Because the walk that calls this is a partition
    /// of the globe, every cell within range ends up with a floor that is a lower bound
    /// on the terrain over the whole cell.
    pub fn stamp(&mut self, center: DVec3, gamma: f64, gr: f64, near: f64, floor: f64) {
        if !self.active || near > self.max_ang {
            return;
        }
        // **Only rings whose far edge this stamp's ground actually reaches.**
        //
        // A cell's floor is read at its ring's *far* edge — that is where the wall stands
        // in the soundness argument — so a stamp may only constrain a ring if its own
        // ground covers that edge. Writing into `ring_of(near) ..= ring_of(far)` instead,
        // as a first version did, lets a stamp 200 m deep claim a wall at the far edge of
        // a ring 1 km deep, a kilometre beyond the ground it actually bounds. That is an
        // occluder that is too high, i.e. the false negative this whole file is arranged
        // to prevent, and it showed up as 122 of them the moment the stamps got tight
        // enough to be thinner than a ring.
        //
        // Coverage survives it: the walk partitions the globe and each node's sub-cells
        // partition the node radially, so whichever sub-cell's `[near, far]` contains a
        // given ring's far edge is the one that constrains it.
        let far = gamma + gr;
        let r0 = self.ring_of(near);
        if self.ring_far[r0] > far {
            return;
        }
        let mut r1 = r0;
        while r1 + 1 < RANGE_RINGS && self.ring_far[r1 + 1] <= far {
            r1 += 1;
        }

        // Dead sectors are never written and never read (§7f, [`Self::live`]). Skipping
        // them here is not an approximation of the coverage argument above: the argument
        // is about the cells a candidate can be tested against, and a candidate that
        // reaches a dead sector is not culled at all.
        match self.sector_range(gamma, gr, self.bearing(center)) {
            None => {
                for a in 0..AZIMUTH_SECTORS {
                    if !self.is_live(a) {
                        continue;
                    }
                    for r in r0..=r1 {
                        let cell = &mut self.floor[a][r];
                        if floor < *cell {
                            *cell = floor;
                        }
                    }
                }
            }
            Some((lo, hi)) => {
                for s in lo..=hi {
                    let a = s.rem_euclid(AZIMUTH_SECTORS as isize) as usize;
                    if !self.is_live(a) {
                        continue;
                    }
                    for r in r0..=r1 {
                        let cell = &mut self.floor[a][r];
                        if floor < *cell {
                            *cell = floor;
                        }
                    }
                }
            }
        }
    }

    /// Turns the stamped floors into the cumulative ridge elevation angles the stage
    /// reads, and decides whether the march is usable at all.
    ///
    /// # The underground guard
    ///
    /// The camera's ground collision only knows the terrain that has loaded under it, so
    /// the camera can legitimately end up inside a mountain whose tile arrives later. Every cell's ridge
    /// then towers over the eye, the test culls the entire globe, and the screen goes
    /// black — a correct answer to the wrong question. When the nearest ring's floor is
    /// above the camera in *every* sector the camera is enclosed, and the march is
    /// switched off instead. A weakening, so it cannot introduce a false negative.
    pub fn finish(&mut self) {
        if !self.active {
            return;
        }

        let mut enclosed = true;
        for a in 0..AZIMUTH_SECTORS {
            if !self.is_live(a) {
                continue;
            }
            let f = self.floor[a][0];
            if !f.is_finite() || f <= self.cam_alt {
                enclosed = false;
                break;
            }
        }
        // What the enclosure guard just decided, and the two numbers it decided it from.
        // Not decoration: a camera placed a few hundred metres over what a map calls a
        // valley floor but what the DEM calls a mountainside switches the entire march off
        // here, silently and correctly, and §7c lost three measurement poses to exactly
        // that before this line existed. `D3_DEBUG` is the same switch the floor dump
        // below uses.
        if std::env::var_os("D3_DEBUG").is_some() {
            let mut lo = f64::INFINITY;
            let mut hi = f64::NEG_INFINITY;
            let mut inf = 0;
            for a in 0..AZIMUTH_SECTORS {
                let f = self.floor[a][0];
                if f.is_finite() {
                    lo = lo.min(f);
                    hi = hi.max(f);
                } else {
                    inf += 1;
                }
            }
            eprintln!(
                "D3 finish: cam_alt {:.0} m, ring0 floor min {:.0} max {:.0} m, {inf} unstamped, enclosed {enclosed}",
                self.cam_alt * 1.0e6,
                lo * 1.0e6,
                hi * 1.0e6
            );
        }
        if enclosed {
            self.active = false;
            return;
        }

        if std::env::var_os("D3_DEBUG").is_some() {
            for a in [0usize, 2, 6] {
                let row: Vec<String> = (0..RANGE_RINGS)
                    .map(|r| {
                        let f = self.floor[a][r];
                        if f.is_finite() {
                            format!("{:.0}", f * 1.0e6)
                        } else {
                            "-".into()
                        }
                    })
                    .collect();
                eprintln!("sector {a}: {}", row.join(" "));
            }
        }
        let scale = std::f64::consts::TAU / AZIMUTH_SECTORS as f64;

        // Sector centre bearings, once. The elevation angle to a ridge depends on the
        // bearing only through the ellipsoid's local radius, which is why the direction is
        // still needed per sector — but the `sin_cos` that builds it is not a function of
        // the ring, so it belongs outside.
        let mut bearing_dir = [DVec3::ZERO; AZIMUTH_SECTORS];
        for (a, dir) in bearing_dir.iter_mut().enumerate() {
            let (sin_b, cos_b) = ((a as f64 + 0.5) * scale).sin_cos();
            *dir = self.east * sin_b + self.north * cos_b;
        }

        // **Rings outside, sectors inside** — §7c's second optimisation, and the only
        // reason the loops are this way round. `elevation_of` opens with `gamma.sin_cos()`
        // and `gamma` is the ring's, not the sector's, so the sector-major order computed
        // the same 48 sine-cosine pairs 24 times over. Hoisting them turns 1 152 into 48
        // and takes `finish` from 52 µs to 35 µs. The running maximum is per sector, so it
        // becomes an array rather than a scalar; nothing else about the accumulation
        // changes, and in particular each sector still sees its rings in increasing order.
        //
        // Every finite cell is still evaluated. A tempting optimisation — skip a ring that
        // is farther than one already seen and no higher — is **wrong**, and it was
        // measured to be: the elevation angle to a fixed altitude below the eye is not
        // monotone in range. It climbs steeply from −90° just under the camera toward
        // its maximum and only then falls away with the curvature drop, so the ring
        // that matters is usually one whose floor equals the near field's. Skipping on
        // that basis left `run` pinned at the near field's −54° and cost the ridge
        // world's valley pose its entire 20 % reduction.
        let mut run = [f64::NEG_INFINITY; AZIMUTH_SECTORS];
        for r in 0..RANGE_RINGS {
            let gamma = self.ring_far[r];
            let (sin_g, cos_g) = gamma.sin_cos();
            let up_term = self.up * cos_g;
            // **The safety distance is the cell's, not the grid's.** It covers a
            // placement error proportional to the range (see [`RIDGE_SAFETY_RATE`]),
            // reaching the wall's altitude through the curvature drop `s²/R` and through
            // the tilt of a sightline that stands `Δalt` off the floor. A flat 100 m
            // charged the first term's 250 km value at the 2 km ring, which is where §7d
            // measured it throwing away 2.23° of Stuttgart's escarpment for a quantity
            // that is a centimetre there.
            //
            // Megametres, like every altitude in this struct: `gamma * gamma * R` **is**
            // `s²/R` once `s = gamma·R`, so the radius carries the unit and no metre ever
            // appears. Hoisted here because it is the ring's; only the `Δalt` term below
            // is the cell's.
            let drop = gamma * gamma * EARTH_RADIUS_A_F64;
            let safety_base = RIDGE_SAFETY_FLOOR_M * 1.0e-6 + RIDGE_SAFETY_RATE * drop;
            for a in 0..AZIMUTH_SECTORS {
                if !self.is_live(a) {
                    continue;
                }
                let f = self.floor[a][r];
                if f.is_finite() {
                    let n = up_term + bearing_dir[a] * sin_g;
                    let safety = safety_base + RIDGE_SAFETY_RATE * (self.cam_alt - f).abs();
                    let e = self.elevation_along(n, f - safety);
                    if e > run[a] {
                        run[a] = e;
                    }
                }
                self.ridge[a][r] = run[a] as f32;
            }
        }
        for a in 0..AZIMUTH_SECTORS {
            if !self.is_live(a) {
                continue;
            }
            let top = self.ridge[a][RANGE_RINGS - 1];
            if top > self.ridge_ceiling {
                self.ridge_ceiling = top;
            }
        }
    }

    /// Elevation angle, from the eye, of the ridge point whose outward ellipsoid normal
    /// is `n`, at altitude `alt` megametres.
    ///
    /// Exact on the ellipsoid up to the altitude being measured radially rather than
    /// along the normal (a sub-centimetre difference at 3 km of relief), which is what
    /// [`RIDGE_SAFETY_FLOOR_M`] covers along with the rest of the range-independent
    /// small-angle slop.
    ///
    /// # Why it takes the normal and not `(bearing, gamma)`
    ///
    /// It used to take the bearing direction and the angular distance and open with
    /// `gamma.sin_cos()`. `gamma` is the **ring's** and the bearing is the sector's, so
    /// [`Self::finish`]'s sector-major loop recomputed the same 48 sine-cosine pairs 24
    /// times over. Taking the finished normal moves that line up one loop level — 1 152
    /// `sin_cos` calls become 48 — and is the whole of §7c's second optimisation.
    #[inline]
    fn elevation_along(&self, n: DVec3, alt: f64) -> f64 {
        let p = surface_point_for_normal(n) + n * alt;
        let v = p - self.eye;
        let vert = v.dot(self.up);
        let horiz = (v - self.up * vert).length();
        vert.atan2(horiz)
    }

    /// The guaranteed ridge elevation angle, in radians, that a candidate `range_m` away
    /// on compass bearing `bearing_rad` would be tested against.
    ///
    /// A read-only window on the finished march, for tests and debug readouts — it is the
    /// same `ridge[sector][ring]` lookup [`Self::occludes`] makes, minus the candidate's
    /// own extent. `f32::NEG_INFINITY` when the march is inactive or nothing is guaranteed
    /// in that direction.
    ///
    /// It exists because the claim about **lateral gaps**
    /// — that a col in a ridge pulls its sector's floor down to the valley and stops the
    /// cull — is a statement about this grid, and checking it through tile counts instead
    /// confounds it with the LOD's own choices about where to refine.
    pub fn ridge_elevation(&self, bearing_rad: f64, range_m: f64) -> f32 {
        if !self.active {
            return f32::NEG_INFINITY;
        }
        let gamma = range_m * 1.0e-6 / EARTH_RADIUS_A_F64;
        let r = self.ring_of(gamma);
        let scale = AZIMUTH_SECTORS as f64 / std::f64::consts::TAU;
        let a = ((bearing_rad * scale).floor() as isize).rem_euclid(AZIMUTH_SECTORS as isize);
        self.ridge[a as usize][r]
    }

    /// **The stage's whole question**: is every drawable point of this box provably
    /// behind guaranteed terrain?
    ///
    /// The box is the node's height-aware box, which by I-1′ contains its mesh and its skirts.
    ///
    /// # Why the occludee's elevation is bounded from the box and not from a sphere
    ///
    /// A sphere is the obvious bound and it is far too weak to be worth having. A tile's
    /// box is a *flat slab tangent to the globe*: its two big extents are horizontal and
    /// barely change the elevation angle at all, while its small one is the relief. The
    /// circumsphere throws that shape away and claims the tile could be anywhere within
    /// its radius — including directly overhead. Measured on the ridge world's valley
    /// pose: a z12 tile 20 km out has a 4.7 km circumradius, so the sphere bound puts its
    /// highest possible point at **+11.8°** where the box bound puts it at **−1.1°**,
    /// against a ridge at +11.3°. The sphere culls nothing; the box culls it comfortably.
    ///
    /// The box bound is two linear extremes, both exact:
    ///
    /// * `vert_max = (c − eye)·û + Σ|h_j·û|` — the highest the box reaches along the
    ///   eye's up axis. `vert` is linear in the point, so its maximum over a box is
    ///   attained at a vertex and this expression *is* that maximum.
    /// * `horiz` — the distance from the eye's vertical axis — is bounded on whichever
    ///   side makes the angle larger, and that side depends on the sign of `vert_max`:
    ///   `atan2(v, h)` grows as `h` shrinks when `v ≥ 0` and as `h` grows when `v < 0`.
    ///   Taking the wrong one is an under-estimate of the occludee's elevation, i.e. a
    ///   false negative, which is why the sign is branched on rather than assumed.
    ///
    /// Maximising the two independently is conservative (the true maximum is over the
    /// joint set), and unlike a vertex-wise maximum of the angle itself it is *sound*:
    /// the superlevel sets of the elevation angle are convex cones, so a box can poke
    /// into one without any of its eight vertices being inside it.
    pub fn occludes(
        &self,
        obb: &super::bounding_volume::OrientedBoundingBox,
        bounds: &super::tile_id::TileBounds,
    ) -> bool {
        if !self.active {
            return false;
        }
        let half: [DVec3; 3] = [
            DVec3::new(
                obb.half_axes[0].x as f64,
                obb.half_axes[0].y as f64,
                obb.half_axes[0].z as f64,
            ),
            DVec3::new(
                obb.half_axes[1].x as f64,
                obb.half_axes[1].y as f64,
                obb.half_axes[1].z as f64,
            ),
            DVec3::new(
                obb.half_axes[2].x as f64,
                obb.half_axes[2].y as f64,
                obb.half_axes[2].z as f64,
            ),
        ];
        let center = obb.center;

        // Upper bound on the elevation angle of every point of the **box**. See this
        // method's doc comment for why the two extremes are taken separately and why the
        // horizontal one is branched on the sign of the vertical one.
        let v = center - self.eye;
        let vert_c = v.dot(self.up);
        let horiz_c = (v - self.up * vert_c).length();
        let mut vert_span = 0.0;
        let mut horiz_span = 0.0;
        for h in &half {
            let a = h.dot(self.up);
            vert_span += a.abs();
            horiz_span += (*h - self.up * a).length();
        }
        let vert_max = vert_c + vert_span;
        let theta_max = if vert_max >= 0.0 {
            vert_max.atan2((horiz_c - horiz_span).max(0.0))
        } else {
            vert_max.atan2(horiz_c + horiz_span)
        } as f32;

        // The cheap filter, and it comes first because everything below costs four more
        // transcendental functions. Nothing can be culled whose highest point already
        // clears the tallest ridge the march found anywhere.
        if theta_max >= self.ridge_ceiling {
            return false;
        }

        let (gamma, gr, near) = self.extent_of(bounds);
        // Nothing can be nearer than the first ring's far edge, so there is no ridge in
        // front of this candidate to test against.
        if near <= self.ring_far[0] {
            return false;
        }
        // The last ring whose far edge is strictly nearer than the candidate's near edge.
        let mut j = None;
        for (i, far) in self.ring_far.iter().enumerate() {
            if *far <= near {
                j = Some(i);
            } else {
                break;
            }
        }
        let Some(j) = j else { return false };

        let ridge_min = match self.sector_range(gamma, gr, self.bearing(center)) {
            None => {
                let mut m = f32::INFINITY;
                for a in 0..AZIMUTH_SECTORS {
                    m = m.min(self.ridge[a][j]);
                }
                m
            }
            Some((lo, hi)) => {
                let mut m = f32::INFINITY;
                for s in lo..=hi {
                    let a = s.rem_euclid(AZIMUTH_SECTORS as isize) as usize;
                    m = m.min(self.ridge[a][j]);
                }
                m
            }
        };

        theta_max < ridge_min
    }
}

/// The sectors of the polar grid the frustum's azimuth band can reach — **§7f**.
///
/// The four rays of [`Frustum::rays`] are the *edges* of the pyramid the side planes
/// model, so every direction in the view volume is a non-negative combination of them.
/// Project them onto the local horizontal plane: a non-negative combination of vectors
/// whose bearings all lie inside an arc shorter than half a turn lies in that same arc,
/// so the minimal arc enclosing the four bearings bounds every azimuth in view.
///
/// Three things break that argument, and each falls back to **every sector**, which is
/// the pre-§7f behaviour and cannot lose a cull:
///
/// * `rays` is `None` — a frustum built by `Frustum::planes_only`, which the renderer
///   never uses but `Frustum::intersects_sphere` callers do.
/// * a ray is (near-)vertical, so its bearing does not exist.
/// * the enclosing arc is half a turn or more, so the horizontal cone is not salient and
///   no arc bounds it. A camera pointed at the nadir is the ordinary case of this: its
///   four rays fan out around the whole compass.
fn live_sectors(
    up: DVec3,
    east: DVec3,
    north: DVec3,
    frustum: &Frustum,
) -> [bool; AZIMUTH_SECTORS] {
    const ALL: [bool; AZIMUTH_SECTORS] = [true; AZIMUTH_SECTORS];
    let Some(rays) = frustum.rays else {
        return ALL;
    };

    let mut beta = [0.0f64; 4];
    for (i, r) in rays.iter().enumerate() {
        let h = *r - up * r.dot(up);
        // A ray within ~0.06° of vertical: its bearing is numerical noise, and a camera
        // that has one is a camera looking very nearly straight up or down.
        if h.length_squared() < 1.0e-6 {
            return ALL;
        }
        beta[i] = h.dot(east).atan2(h.dot(north));
    }
    beta.sort_by(|a, b| a.partial_cmp(b).unwrap());

    // The enclosing arc is the complement of the widest gap between consecutive bearings.
    let mut gap_at = 0usize;
    let mut widest = f64::NEG_INFINITY;
    for i in 0..4 {
        let g = if i == 3 {
            beta[0] + std::f64::consts::TAU - beta[3]
        } else {
            beta[i + 1] - beta[i]
        };
        if g > widest {
            widest = g;
            gap_at = i;
        }
    }
    let span = std::f64::consts::TAU - widest;
    if !(span < std::f64::consts::PI) {
        return ALL;
    }
    // The arc runs forward from the bearing just after the widest gap.
    let start = beta[(gap_at + 1) % 4];

    let scale = AZIMUTH_SECTORS as f64 / std::f64::consts::TAU;
    let lo = (start * scale).floor() as isize - LIVE_SECTOR_MARGIN as isize;
    let hi = ((start + span) * scale).ceil() as isize + LIVE_SECTOR_MARGIN as isize;
    if hi - lo >= AZIMUTH_SECTORS as isize {
        return ALL;
    }
    let mut mask = [false; AZIMUTH_SECTORS];
    for k in lo..=hi {
        mask[k.rem_euclid(AZIMUTH_SECTORS as isize) as usize] = true;
    }
    mask
}

/// `d` folded into `(-180, 180]`.
#[inline]
fn wrap_deg(d: f64) -> f64 {
    let mut d = d % 360.0;
    if d > 180.0 {
        d -= 360.0;
    }
    if d <= -180.0 {
        d += 360.0;
    }
    d
}

/// Outward unit normal of the ellipsoid at `p` — the same expression
/// `quadtree::ellipsoid_normal` uses, kept here so this module does not have to be
/// `pub(super)`-coupled to the traversal for one four-line function.
#[inline]
fn ellipsoid_normal_at(p: DVec3) -> DVec3 {
    const INV_A2: f64 = 1.0 / (EARTH_RADIUS_A_F64 * EARTH_RADIUS_A_F64);
    const INV_B2: f64 = 1.0 / (EARTH_RADIUS_B_F64 * EARTH_RADIUS_B_F64);
    DVec3::new(p.x * INV_A2, p.y * INV_B2, p.z * INV_A2).normalize()
}
