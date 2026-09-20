//! Atmospheric fog: the distance-based imagery falloff CesiumJS ships in
//! `Scene/Fog.js` and `Core/Math.js`'s `CesiumMath.fog` — WP5 of
//! `docs/pre-terrain-plan.md`.
//!
//! # Where this is (and is not) wired in
//!
//! **Fog culling is not geometrically sound.** It deliberately discards tiles that
//! are genuinely visible — that is the entire point, and it is why the pure math
//! lives in this module, isolated from anything the culling harness's FN = 0
//! guarantee depends on. Two, and only two, places consume it:
//!
//! * [`super::quadtree::Stage::Fog`] — a `Stage`, `&self`, outright culls a node
//!   when [`cesium_fog`] reaches `1.0`. Present **only** in
//!   [`super::quadtree::CullPipeline::DEFAULT_WITH_FOG`], which is what
//!   `wgpu_state.rs` actually runs. **Never** present in
//!   [`super::quadtree::CullPipeline::DEFAULT`], which is what the culling harness
//!   builds and what every FN = 0 guarantee in `docs/culling-math.md` is proved
//!   against. If a future change adds `Stage::Fog` to `CullPipeline::DEFAULT` or to
//!   `test_stage_pipeline.rs`'s `all_pipelines`, every sweep in the culling gate
//!   turns red — by design, not by accident, because fog subtracts real visible
//!   geometry and the gate has no way to know that subtraction was intentional.
//! * `QuadtreeNode::apply_lod` — relaxes `subdivide_dist` (shrinks the threshold a
//!   node refines within) as fog thickens, so a tile buried in fog stops demanding
//!   full resolution shortly before the `Stage` above culls it outright, rather
//!   than staying maximally refined right up to the frame it vanishes.
//!
//! # The port
//!
//! [`cesium_fog`] is `CesiumMath.fog` verbatim: `scalar = distance · density`,
//! `fog = 1 - exp(-scalar²)`.
//!
//! [`fog_density_for`] is `Fog.prototype.update`'s height-based term:
//!
//! ```text
//! density(h) = base_density · height_scalar · max(h / max_height, EPSILON4) ^ -height_falloff
//! ```
//!
//! for `h <= max_height`, and `0` above it (fog disabled in space). `EPSILON4 =
//! 1e-4` is `CesiumMath.EPSILON4`, and it is what stops the negative exponent from
//! diverging as `h -> 0`.
//!
//! **Deliberately not ported**: Cesium's `Fog.update` also multiplies `density` by
//! `(1 - |dot(cameraDirection, positionNormal)|)`, fading fog in as the camera tilts
//! toward the horizon and out as it looks straight down. That term modulates
//! *visual* fog density based on which way the camera happens to be pointed, not on
//! any per-tile quantity, and plumbing a camera forward vector into
//! [`super::quadtree::CullContext`] for it would add real surface area for a term
//! orthogonal to what WP5 is scoped to (tile-count reduction from distance falloff,
//! not view-angle-dependent haze). Every tile-count and quality number this package
//! reports is therefore a **worst case for a nadir-ish view** relative to real
//! Cesium — a horizon-tilted camera would fog (and therefore cull/relax) somewhat
//! more than these numbers show, never less.
//!
//! # Units
//!
//! Every constant here is in **metres**, exactly as CesiumJS defines them —
//! `density`, `max_height_m` etc. are not renormalised. This engine's world frame
//! is **megameters** (`globe/geometry.rs`'s `alt_meters / 1_000_000.0`, and every
//! `QuadtreeNode::center`/`OrientedBoundingBox` built from it), so every call site
//! in `quadtree.rs` and `wgpu_state.rs` converts its megameter distance/altitude to
//! metres — `* MEGAMETERS_TO_METERS` — right before calling into this module, never
//! the other way around. Keeping the constants in their original, citable units
//! means they can be checked against the CesiumJS source by eye; rescaling them
//! into engine units would hide that comparison behind arithmetic a reader would
//! have to redo.

/// `CesiumMath.EPSILON4`.
const EPSILON4: f32 = 1e-4;

/// This engine's world frame is megameters; Cesium's fog constants are metres. See
/// the module doc comment's Units section.
pub const MEGAMETERS_TO_METERS: f32 = 1_000_000.0;

/// Fog constants, ported from CesiumJS `Scene/Fog.js`'s field defaults (all in
/// metres — see the module doc comment). Lives on
/// [`crate::globe::tiles::config::TileEngineConfig`] as `fog`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FogConfig {
    /// `Fog.density`. "A scalar that determines the density of the fog... The more
    /// dense the fog is, the more aggressively the terrain is culled."
    pub density: f32,
    /// `Fog.heightScalar`. "A scalar used in the function to adjust density based
    /// on the height of the camera above the terrain."
    pub height_scalar: f32,
    /// `Fog.heightFalloff`. "Exponent factor used in the function to adjust how
    /// density changes based on the height of the camera above the ellipsoid.
    /// Smaller values produce a more gradual transition as camera height
    /// increases."
    pub height_falloff: f32,
    /// `Fog.maxHeight`, in metres. "The maximum height fog is applied. If the
    /// camera is above this height fog will be disabled."
    pub max_height_m: f32,
    /// `Fog.screenSpaceErrorFactor`. Ported for parity with the CesiumJS defaults, and
    /// reserved since WP5 "once terrain gives this engine a real geometric error term".
    ///
    /// **E1 gave it one, and E1b measured whether Cesium's form beats WP5's. It does
    /// not.** Cesium subtracts `fog(d) · sse` from the screen-space error *in pixels*
    /// before comparing against the budget, which in this engine's distance form is
    /// `terrain_dist /= 1 + fog · (sse / max_geometric_error_px)`; that is
    /// [`super::quadtree::TerrainFogPolicy::CesiumSse`], it is implemented, and at an
    /// equal tile budget over ten real-terrain poses it left **91.6** pixels of summed
    /// far-field p95 geometric error against **84.3** for simply not relaxing the
    /// geometric term at all. So the shipped policy is
    /// [`super::quadtree::TerrainFogPolicy::ImageryOnly`] and this field is still not
    /// consumed in production — now as a measured result rather than as a pending
    /// question. `docs/terrain-plan.md` §8 has the table.
    ///
    /// The structural reason, which is worth more than the seven pixels: Cesium's form is
    /// bounded below by `1/(1 + sse/max_px)` however thick the fog gets, so with this
    /// engine's shipped budget it can only ever shorten the geometric refinement distance
    /// by a fifth. It is a nudge where WP5's `× (1 − fog)` is a switch, and the question
    /// E1b was actually asking — may fog coarsen a *mountain* — is answered "no" by both
    /// of them far better than by WP5's.
    pub sse: f32,
}

impl Default for FogConfig {
    /// CesiumJS `Fog.js`'s own field defaults, verbatim.
    fn default() -> Self {
        Self {
            density: 0.0006,
            height_scalar: 0.001,
            height_falloff: 0.59,
            max_height_m: 800_000.0,
            sse: 2.0,
        }
    }
}

/// `CesiumMath.fog(distanceToCamera, density)`, verbatim: the fraction of a tile at
/// `distance_m` that atmospheric fog obscures, given the frame's current `density`.
/// `0.0` is clear air, `>= 1.0` is fully obscured (the outright-cull threshold).
/// Monotonically increasing in both arguments; `0.0` at `distance_m == 0.0` or
/// `density == 0.0` (including the `density == 0.0` [`fog_density_for`] returns
/// above `max_height_m`, which is how "fog disabled in space" actually disables
/// it — nothing downstream needs a separate enabled flag).
///
/// Both arguments are in **metres** — see the module doc comment's Units section.
pub fn cesium_fog(distance_m: f32, density: f32) -> f32 {
    let scalar = distance_m * density;
    1.0 - (-(scalar * scalar)).exp()
}

/// `Fog.prototype.update`'s height-based density term: how thick the fog is this
/// frame, from camera altitude alone (the camera-orientation term Cesium also
/// applies is deliberately not ported — see the module doc comment).
///
/// `0.0` above `cfg.max_height_m` ("fog disabled in space"); below it, density
/// grows as altitude falls, following `(altitude / max_height) ^ -height_falloff`,
/// floored at [`EPSILON4`] so the negative exponent cannot diverge as altitude
/// approaches the surface.
///
/// `altitude_m` is in **metres**, camera altitude above the ellipsoid — see the
/// module doc comment's Units section.
pub fn fog_density_for(altitude_m: f32, cfg: &FogConfig) -> f32 {
    if altitude_m > cfg.max_height_m {
        return 0.0;
    }
    let ratio = (altitude_m / cfg.max_height_m).max(EPSILON4);
    cfg.density * cfg.height_scalar * ratio.powf(-cfg.height_falloff.max(0.0))
}
