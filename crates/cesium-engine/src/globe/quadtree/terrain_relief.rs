//! **The pre-check** — one number, read off the visible set, that decides whether the terrain occlusion
//! march is worth running at all.
//!
//! # Why there is a pre-check
//!
//! Profiling put terrain occlusion's two halves on one clock and found the shape of the answer: over six poses
//! the stage removed twenty tiles at **one** of them and nothing at four others, while
//! charging 300–500 µs of march at every pose the altitude gate let through. The analysis also
//! named the discriminator it could not build — "the discriminator is not height, it is
//! **relief in view**" — and recorded that the quantity is the one the march itself
//! computes, "which is the shape of the problem and why it is recorded here rather than
//! built".
//!
//! It is not quite the shape of the problem, and this module is the part that can be
//! built. The march's answer depends on the terrain at a resolution of 96 × 48 cells; the
//! *question* "could there be a wall in front of the camera at all" depends only on how
//! high the nearest provable ground stands above the eye, and every node already carries
//! that number. Reading it off the **visible leaves** costs one walk of the set the
//! renderer is about to draw — 59–341 nodes — against a march that walks the whole tree
//! and stamps sixteen sub-cells per node.
//!
//! # The quantity, derived
//!
//! Not "relief above eye height". That was the first candidate and it is the wrong one,
//! measured: `max(node.hi) − eye` reads 3 539 m at the Inn valley (twenty tiles removed)
//! and 4 025 m at the Albtrauf (one), because a height says nothing without the distance
//! it stands at. A wall 400 m high at 800 m casts a longer shadow than a wall 4 km high
//! at 80 km.
//!
//! The quantity that carries both is the **elevation angle** — height with the distance
//! divided back out — and two further choices make it the right angle rather than an
//! angle:
//!
//! * **At the node's nearest point, not its centre.** [`extent_of_at`] is the march's own
//!   rectangle geometry, shared rather than re-derived, so the pre-check and the march
//!   measure the same tile the same way.
//! * **Off `floor_grid`, not off the box.** The box's top is `HeightBounds::hi`, which on
//!   an inherited interval is kilometres of margin rather than terrain; at the
//!   Jungfraujoch it reads **+36.7°** of relief where nothing at all stands above the eye.
//!   The sixteen sub-cell floors are what height-aware bounds can *prove* is there, which is exactly what
//!   the march would stamp, and the same pose reads **−1.8°** off them. Highest sub-cell,
//!   because one sub-cell of guaranteed crest is a wall; the scalar `floor` is a minimum
//!   over the whole tile and would average that crest away (`HeightBounds::floor_grid`'s
//!   own measurement, in the other direction).
//!
//! Nodes nearer than [`MIN_RANGE_M`] are skipped. That is not a tuning knob: the march's
//! first ring starts there, so nothing nearer can be an occluder at all — and without the
//! cut the tile the camera is *standing on* reports +90° at every pose that has any relief
//! under it.
//!
//! # It cannot cost a tile, and that is the whole of its correctness
//!
//! Not marching is not culling. A pre-check that says "no" leaves occlusion culling at height-aware bounds and the relief-aware horizon test, which is
//! the arm the flat globe has always run and which the culling gate proves sound
//! independently. The only mistake this file can make is to leave a cull on the table —
//! a **balance** error, not a correctness one — so nothing here is rounded outward, no
//! margin is measured for it, and `testing::terrain::test_terrain_relief` scores it
//! against the removal it is predicting rather than against an oracle.
//!
//! # Units
//!
//! Megametres and radians, like the rest of the quadtree. The threshold is in degrees
//! because that is the unit a person picks an elevation angle in, and it is converted
//! once, into a tangent, in [`ReliefProbe::new`].

use glam::DVec3;

use super::terrain_occlusion::{extent_of_at, surface_point_for_normal, GroundFrame, MIN_RANGE_M};
use super::tile_id::TileBounds;

/// The running maximum of the relief angle over the visible set, and the threshold it is
/// being compared against.
///
/// Built once per frame, fed one visible leaf at a time, and asked
/// [`Self::clears`](ReliefProbe::clears) once. It keeps a **tangent** rather than an
/// angle: the comparison is monotone, `atan2` is the most expensive thing in the loop,
/// and nothing downstream needs the angle itself except [`Self::relief_deg`], which
/// exists for `D3_DEBUG` and for the harness.
#[derive(Clone, Debug)]
pub struct ReliefProbe {
    cam_lon: f64,
    cam_lat: f64,
    /// Geocentric radius of the ellipsoid under the camera, megametres.
    r_ground: f64,
    /// Geocentric radius of the eye, megametres.
    r_eye: f64,
    /// [`MIN_RANGE_M`] as an angular distance — the march's first ring.
    min_ang: f64,
    /// The threshold, as a tangent.
    tan_gate: f64,
    /// Largest `tan(elevation)` any visible leaf's provable ground has reached.
    best_tan: f64,
    /// `true` once [`Self::best_tan`] has passed [`Self::tan_gate`], so the walk can stop.
    cleared: bool,
}

impl ReliefProbe {
    /// A probe at `eye`, gated at `min_relief_deg` degrees of elevation.
    ///
    /// The local sphere is the camera's own: `r_ground` is the ellipsoid's geocentric
    /// radius under the eye and every wall is placed on that same sphere. The ellipsoid's
    /// radius does drift along the ground — some 230 m per degree of latitude at mid
    /// latitudes — so a wall 120 km away is placed with up to a couple of hundred metres
    /// of radial error. That is deliberate and not a corner cut: it is the same local
    /// sphere `TerrainHorizon::finish`'s curvature drop `γ²R` works on, the statistic's
    /// maximum is in practice a wall between 0.7 and 11 km out (measured, §7g's table),
    /// and an error in a *predictor* costs at most a march.
    pub fn new(eye: DVec3, min_relief_deg: f32) -> Self {
        let GroundFrame {
            up,
            cam_lon,
            cam_lat,
            ..
        } = GroundFrame::at(eye);
        let r_ground = surface_point_for_normal(up).length();
        // **The threshold has to survive its own switch-off value.** `tan` is only
        // monotone on `(−90°, 90°)` and `(−∞).to_radians().tan()` is `NaN`, against which
        // every comparison is false — so `min_relief_deg = −∞`, the documented way to
        // disable the pre-check, would have disabled the *march* instead. It is the kind
        // of mistake that passes every test that does not try the off switch, and it did:
        // `terrain_relief_pre_check_only_ever_removes_culls` caught it on its control arm.
        //
        // At or below −90° the gate is open and the probe says so before the walk starts,
        // which is what makes the off switch cost nothing rather than one visible-set
        // walk. At or above +90° it is shut. `NaN` reads as "open", i.e. §7f's engine.
        let tan_gate = if !(min_relief_deg > -90.0) {
            f64::NEG_INFINITY
        } else if !(min_relief_deg < 90.0) {
            f64::INFINITY
        } else {
            (min_relief_deg as f64).to_radians().tan()
        };
        Self {
            cam_lon,
            cam_lat,
            r_ground,
            r_eye: eye.length(),
            min_ang: MIN_RANGE_M * 1.0e-6 / r_ground,
            tan_gate,
            best_tan: f64::NEG_INFINITY,
            cleared: tan_gate == f64::NEG_INFINITY,
        }
    }

    /// Offers one visible leaf: its ground rectangle and the highest altitude, in
    /// megametres, that height-aware bounds guarantees over any part of it.
    ///
    /// Returns `true` once the threshold has been cleared, which is the walk's cue to
    /// stop — the poses that clear it are the ones about to pay for a march anyway, and
    /// the poses that do not are the ones this file exists to make cheap.
    #[inline]
    pub fn consider(&mut self, bounds: &TileBounds, ground_top: f64) -> bool {
        if self.cleared {
            return true;
        }
        let (_, _, near) = extent_of_at(self.cam_lon, self.cam_lat, bounds);
        if near < self.min_ang {
            return false;
        }
        // Elevation angle on the local sphere: the wall's top sits at radius
        // `r_ground + ground_top`, an angular distance `near` around from the eye.
        let (sin_g, cos_g) = near.sin_cos();
        let r = self.r_ground + ground_top;
        let horiz = r * sin_g;
        if horiz <= 0.0 {
            return false;
        }
        let tan = (r * cos_g - self.r_eye) / horiz;
        if tan > self.best_tan {
            self.best_tan = tan;
            self.cleared = tan >= self.tan_gate;
        }
        self.cleared
    }

    /// Is there enough relief in view for the march to be worth building?
    #[inline]
    pub fn clears(&self) -> bool {
        self.cleared
    }

    /// The statistic itself, in degrees — `−∞` when no visible leaf was far enough out to
    /// be an occluder. For `D3_DEBUG` and for the harness; nothing in the engine reads it.
    ///
    /// Note that the walk **stops early** once the threshold is cleared, so this is the
    /// maximum over the leaves actually visited, not over the whole visible set. It is a
    /// lower bound on the true statistic and it is on the same side of the threshold.
    pub fn relief_deg(&self) -> f64 {
        self.best_tan.atan().to_degrees()
    }
}
