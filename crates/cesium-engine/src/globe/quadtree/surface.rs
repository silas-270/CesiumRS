//! The **surface model**: what shape the globe's skin has, as a type parameter.
//!
//! Phase A of `docs/terrain-plan.md` §4. This module introduces the seam along
//! which a second surface mode (terrain) will later be added, and nothing else:
//! every line of [`Ellipsoid`]'s impl below is today's code, moved verbatim from
//! `geometry.rs`, `quadtree.rs` and `horizon.rs`. **No behaviour changes here.**
//!
//! # Why a type parameter and not a runtime flag
//!
//! `docs/terrain-plan.md` §1. A `if terrain_enabled` in the per-node loop costs a
//! branch in the hottest code *and* grows the hot structs whether or not terrain is
//! on — `test_horizon_hot_structs_have_not_grown` pins
//! `size_of::<TilePatch>() == 64` (one cache line) and `QuadtreeNode` is 192 B,
//! exactly three lines. A type parameter with **zero-sized** payloads in flat mode
//! costs the flat path exactly zero bytes and monomorphises to today's code.
//!
//! The parameter carries a `= Ellipsoid` default, so every existing call site and
//! every existing test keeps compiling unchanged and the pins above stay literally
//! the same expressions. Note that Rust applies a type-parameter default only in
//! *type* position, never in an expression path, so the constructors keep a
//! concrete `impl …<Ellipsoid>` wrapper (`QuadtreeNode::new`,
//! `QuadtreeManager::new`, `TilePatch::new`, `TileMesh::generate`) in front of the
//! generic `…for_surface` / `…generate_on` form.
//!
//! # Units
//!
//! Every altitude in this trait is in **megametres**, the engine's world unit — the
//! same unit `EARTH_RADIUS_A_F64` and `TileMesh`'s skirt depth are already in. It is
//! deliberately *not* the metres `lon_lat_alt_to_ecef_f64` takes: mixing the two is
//! the obvious way for a later phase to put a mountain 10⁶ times too high.

use super::horizon::{span_is_occluded, HorizonCamera, TilePatch};
use super::tile_id::TileId;
use crate::globe::geometry::{INV_A2_F64, INV_B2_F64};

/// One mesh vertex's inputs, as `TileMesh::generate` has them in hand.
///
/// A bag of already-computed values, not a computation: it exists so the two
/// per-vertex dispatch sites ([`SurfaceModel::vertex_altitude`] and
/// [`SurfaceModel::vertex_normal`]) can be given everything a height-field model
/// will need without the mesh loop having to know which of them it uses.
#[derive(Clone, Copy, Debug)]
pub struct VertexSample {
    /// The tile being meshed.
    pub id: TileId,
    /// Vertex longitude in degrees. A skirt vertex carries its *edge's* longitude,
    /// not one outside the tile — skirts hang inward, they do not widen the patch
    /// (invariant I-5).
    pub lon_deg: f64,
    /// Vertex latitude in degrees, same convention, pole caps forced to ±90.
    pub lat_deg: f64,
    /// Tile-local texture coordinates, clamped into `[0,1]` on the skirt
    /// rows/columns — the same `u`/`v` written into the vertex.
    pub u: f32,
    pub v: f32,
    /// The point on the ellipsoid at altitude 0 for `(lon_deg, lat_deg)`.
    pub surface_pos: [f64; 3],
    /// This vertex belongs to a skirt row or column.
    pub is_skirt: bool,
    /// …and it is one of the two pole caps, which are pulled to ±90° and get **no**
    /// skirt (they would tear the cap open).
    pub is_pole_cap: bool,
    /// The tile's skirt depth, `0.5 / 2^z` **megametres**, as the f32 the mesh
    /// computes it in.
    pub skirt_height: f32,
}

/// What shape the globe's skin has.
///
/// The five dispatch sites of `docs/terrain-plan.md` §1 — four of them here, the
/// fifth (`apply_lod`'s geometric error) deliberately left for Phase E, since
/// there is nothing to dispatch on it yet and Phase A adds no dead code.
///
/// `Copy + 'static` because the implementors are markers: a `SurfaceModel` value is
/// never constructed at run time, the type is the whole content. `Debug` because
/// [`TilePatch`] derives it.
pub trait SurfaceModel: Copy + std::fmt::Debug + 'static {
    /// Per-node payload — `()` for [`Ellipsoid`], a height interval for terrain.
    ///
    /// Zero-sized in flat mode, which is what keeps `QuadtreeNode` at 192 B.
    type NodeExtra: Default + Copy + std::fmt::Debug;

    /// Per-patch payload — `()` for [`Ellipsoid`].
    ///
    /// Zero-sized in flat mode, which is what keeps `TilePatch` at 64 B.
    type PatchExtra: Default + Copy + std::fmt::Debug;

    /// Altitude of one mesh vertex above the ellipsoid, in **megametres**.
    fn vertex_altitude(sample: &VertexSample) -> f64;

    /// Outward unit normal at one mesh vertex.
    fn vertex_normal(sample: &VertexSample) -> [f64; 3];

    /// The `[min, max]` altitude interval, in **megametres**, that a node's bounding
    /// box must be fitted over (`fit_obb`).
    ///
    /// `min == max` means "one altitude", and `fit_obb` then samples each grid point
    /// exactly once — which is what makes the flat path's work identical to today's.
    fn obb_altitude_span(extra: &Self::NodeExtra) -> (f64, f64);

    /// Is every drawable point of this patch hidden behind the limb?
    ///
    /// **This site cannot be unified across models** (`docs/terrain-plan.md` §1):
    /// the flat test is the *exact* supremum of `q·c` over the spherical rectangle,
    /// the terrain test is a cone test on a bounding sphere, and setting that
    /// sphere's radius to zero does not recover the rectangle test. Unifying them
    /// would hand the flat globe a conservative test in place of an exact one.
    fn is_occluded(patch: &TilePatch<Self>, cam: &HorizonCamera) -> bool;
}

/// Today's globe: the bare WGS-84 ellipsoid, zero relief (invariant I-1).
///
/// Both payloads are `()`, so `QuadtreeNode<Ellipsoid>` and `TilePatch<Ellipsoid>`
/// are byte-for-byte the structs that existed before the type parameter did.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Ellipsoid;

impl SurfaceModel for Ellipsoid {
    type NodeExtra = ();
    type PatchExtra = ();

    /// Moved verbatim from `TileMesh::generate`: zero everywhere, except the skirt
    /// rows and columns, which hang `skirt_height` radially *inward*. The pole caps
    /// are excluded — they are pulled to ±90° and a skirt there would open the cap.
    ///
    /// Computed in f32 and widened, exactly as before: `skirt_height` is an f32
    /// and the old code's `alt` was too.
    #[inline]
    fn vertex_altitude(sample: &VertexSample) -> f64 {
        let alt = if sample.is_skirt && !sample.is_pole_cap {
            -sample.skirt_height
        } else {
            0.0
        };
        alt as f64
    }

    /// Moved verbatim from `TileMesh::generate`: the analytic ellipsoid gradient at
    /// the vertex's **surface** point, normalised. Independent of altitude, because
    /// the vertex is displaced *along* this normal.
    #[inline]
    fn vertex_normal(sample: &VertexSample) -> [f64; 3] {
        let [x, y, z] = sample.surface_pos;
        let nx = x * INV_A2_F64;
        let ny = y * INV_B2_F64;
        let nz = z * INV_A2_F64;
        let len = (nx * nx + ny * ny + nz * nz).sqrt();
        [nx / len, ny / len, nz / len]
    }

    /// Zero relief: one altitude, and it is 0. `fit_obb` therefore samples each of
    /// its grid points once, at altitude 0 — today's loop exactly.
    #[inline]
    fn obb_altitude_span(_extra: &()) -> (f64, f64) {
        (0.0, 0.0)
    }

    /// Moved verbatim from `TilePatch::is_occluded`.
    ///
    /// Soundness (§3.5): `S ≤ 1` means every point `p` of the drawn patch has
    /// `q·c ≤ 1`, hence `n̂(p)·(cam − p) ≤ 0` (Theorem 3.4), hence `p` is beyond the
    /// polar plane and — the cone condition being automatic for surface points — is
    /// occluded by the ellipsoid. The skirts lie strictly *inside* the ellipsoid, so
    /// their segments to an exterior eye cross the sphere too. The ellipsoid is
    /// convex and is the only occluder, so nothing can un-occlude them. ∎
    ///
    /// The closed form itself ([`TilePatch::max_dot`] and the two `*_span_max`
    /// helpers) stays where it is: it is the exact supremum over a spherical
    /// rectangle, a fact about the rectangle rather than about the surface, and
    /// `SubGrid` shares its two halves per sub-patch column.
    #[inline]
    fn is_occluded(patch: &TilePatch<Self>, cam: &HorizonCamera) -> bool {
        span_is_occluded(cam, patch.max_dot(cam))
    }
}
