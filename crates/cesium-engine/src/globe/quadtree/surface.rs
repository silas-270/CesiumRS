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

use super::bounding_volume::OrientedBoundingBox;
use super::horizon::{lat_span_max, span_is_occluded, HorizonCamera, TilePatch};
use super::tile_id::TileId;

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
    /// This vertex's row in the `(segments+3)²` build grid, skirt ring included, so
    /// row 0 and row `segments+2` are the two skirt rows.
    ///
    /// Phase C: a height-field model needs to find *this* vertex in its pre-sampled
    /// [`SurfaceModel::BuildCtx`] and to walk to its four neighbours for a central
    /// difference. Both are grid-index questions, not `(lon, lat)` questions, so the
    /// index is what the sample carries. [`Ellipsoid`] ignores it.
    pub row: u32,
    /// This vertex's column in the same grid. See [`Self::row`].
    pub col: u32,
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
    /// The **outward ellipsoid normal** at [`Self::surface_pos`] — the analytic WGS-84
    /// gradient, normalised.
    ///
    /// This is the direction the mesh displaces the vertex along, for **every** model,
    /// and it is deliberately not [`SurfaceModel::vertex_normal`]'s job: relief is
    /// radial (I-5, `test_generated_mesh_stays_inside_the_culling_rectangle`), whereas
    /// a terrain normal tilts with the slope and would drag the vertex sideways out of
    /// its own culling rectangle. The two coincide only for [`Ellipsoid`], which is
    /// why they used to be one value.
    pub up: [f64; 3],
    /// This vertex belongs to a skirt row or column.
    pub is_skirt: bool,
    /// …and it is one of the two pole caps, which are pulled to ±90° and get **no**
    /// skirt (they would tear the cap open).
    pub is_pole_cap: bool,
    /// The tile's skirt depth in **megametres**, as the f32 the mesh computes it in —
    /// [`SurfaceModel::skirt_depth`], called once per tile before the vertex loop.
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
    ///
    /// `PartialEq` because Phase D1's bounds arrive *after* the node does (see
    /// [`Self::child_extra`]) and `QuadtreeNode::set_extra` must be able to ask
    /// "has this changed?" before paying to refit a box and a `k × k` sub-grid.
    /// `()` compares equal to itself, so the flat path's answer is a compile-time
    /// `true` and nothing is ever refitted.
    type NodeExtra: Default + Copy + PartialEq + std::fmt::Debug;

    /// Per-patch payload — `()` for [`Ellipsoid`].
    ///
    /// Zero-sized in flat mode, which is what keeps `TilePatch` at 64 B.
    type PatchExtra: Default + Copy + std::fmt::Debug;

    /// Everything `TileMesh::generate_on` needs to know about *this* tile's surface
    /// data, gathered **before** the mesh worker starts — `()` for [`Ellipsoid`], a
    /// pre-sampled height patch for terrain.
    ///
    /// # Why the mesh builder takes its inputs rather than fetching them
    ///
    /// `generate_on` runs on a rayon worker and must stay a **pure function of
    /// `(id, segments, ctx)`**. Two reasons, and the second is the load-bearing one:
    ///
    /// 1. The height cache is `&mut` (it promotes in an LRU) and lives on the update
    ///    thread. Reaching into it from the worker would need a lock in the middle of
    ///    a per-vertex loop.
    /// 2. Phase E2 (`docs/terrain-plan.md` §8) makes the mesh *stale* when a better
    ///    height tile arrives — the mesh stops being a function of `TileId` alone. A
    ///    builder that sampled the cache itself would have no record of *which* data
    ///    it used; one that is handed a `BuildCtx` does, and
    ///    [`Self::height_source`] hands it straight back out into the finished
    ///    `TileMesh`.
    type BuildCtx: Send + 'static;

    /// The tile's skirt depth, in **megametres**, computed once per tile before the
    /// vertex loop and handed to every vertex as [`VertexSample::skirt_height`].
    ///
    /// C3 of `docs/terrain-plan.md` §6: with relief the crack at an LOD boundary is a
    /// property of the *content*, not of the level, so this is a dispatch site rather
    /// than the one formula it used to be.
    fn skirt_depth(id: &TileId, segments: u32, ctx: &Self::BuildCtx) -> f32;

    /// The `[min, max]` altitude interval, in **megametres**, that every vertex of
    /// the finished mesh is promised to lie inside — skirts included, which is why it
    /// takes the skirt depth [`Self::skirt_depth`] just returned.
    ///
    /// **Invariant I-1′** (`docs/terrain-plan.md` §6). This is the number Phase D's
    /// bounding boxes are fitted over, and
    /// `testing::culling::test_tile_bounds::test_generated_mesh_stays_within_declared_height_bounds`
    /// is what makes the promise checkable for both models.
    fn declared_height_bounds(ctx: &Self::BuildCtx, skirt: f32) -> [f64; 2];

    /// The height tile this mesh was built from, or `None` when the model has no
    /// height data at all ([`Ellipsoid`], always).
    ///
    /// Carried through into `TileMesh` so Phase E2 can key the mesh cache on it; it
    /// costs one `Option<TileId>` per mesh and is the whole of what E2 needs from C.
    fn height_source(ctx: &Self::BuildCtx) -> Option<TileId>;

    /// Altitude of one mesh vertex above the ellipsoid, in **megametres**.
    fn vertex_altitude(sample: &VertexSample, ctx: &Self::BuildCtx) -> f64;

    /// Outward unit normal at one mesh vertex.
    fn vertex_normal(sample: &VertexSample, ctx: &Self::BuildCtx) -> [f64; 3];

    /// Does this surface model have a geometric error for `apply_lod` to refine against?
    ///
    /// **E1 of `docs/terrain-plan.md` §8, and the fifth dispatch site of §1** — the one
    /// Phase A deliberately left out because there was nothing to dispatch on yet.
    ///
    /// `false` for [`Ellipsoid`], and that is a *compile-time* false: `apply_lod` reads it
    /// as `if S::HAS_GEOMETRIC_ERROR`, so the flat arm monomorphises to the single
    /// `unstretched_radius · lod_factor · fog_relaxation` expression it always was, with
    /// no `max`, no second multiply and nothing for a float to round differently. That is
    /// stronger than returning a zero error and relying on `max(x, 0.0) == x`: the
    /// threshold is not merely equal, it is the same instruction sequence, which is what
    /// "the flat path does not move" has meant at every previous dispatch site.
    const HAS_GEOMETRIC_ERROR: bool;

    /// How far the drawn surface of this node departs from the real one, in
    /// **megametres** — E1's LOD term, and `0.0` wherever [`Self::HAS_GEOMETRIC_ERROR`]
    /// is false.
    ///
    /// Read by `apply_lod` as `terrain_dist = geometric_error · terrain_lod_factor`,
    /// which is Cesium's `d < G·H / (maxSSE · 2·tan(fovy/2))` with `G` finally being a
    /// real quantity rather than a constant folded into `lod_factor`. The threshold is
    /// then `max(imagery_dist, terrain_dist)`: whichever of the picture and the shape
    /// still wants resolution at this distance gets it.
    ///
    /// `id` is passed because the term has to **stop** at the data ceiling — see
    /// `DETAIL_MAX_Z` in [`crate::globe::terrain::heightfield`].
    fn geometric_error(extra: &Self::NodeExtra, id: &TileId) -> f32;

    /// The `[min, max]` altitude interval, in **megametres**, that a node's bounding
    /// box must be fitted over (`fit_obb`).
    ///
    /// `min == max` means "one altitude", and `fit_obb` then samples each grid point
    /// exactly once — which is what makes the flat path's work identical to today's.
    fn obb_altitude_span(extra: &Self::NodeExtra) -> (f64, f64);

    /// The interval a **child** node starts life with, given its parent's.
    ///
    /// # The one genuine soundness trap in Phase D1
    ///
    /// A node is culled long before its height tile arrives, and a parent's
    /// `[h_min, h_max]` is **not** a superset of its children's: a coarse DEM smooths
    /// away a peak that a deeper one resolves. Inheriting the parent interval
    /// *unmodified* is therefore a false-negative source, and by I-7 a false negative
    /// at one node costs the whole subtree.
    ///
    /// `docs/terrain-plan.md` §7 lists three policies and this is the third: the
    /// parent's interval widened by a **measured** per-level margin. The measurement
    /// lives with the implementation
    /// ([`Heightfield::child_extra`](crate::globe::terrain::Heightfield)) and is
    /// checked against a committed corpus by
    /// `testing::terrain::test_terrain_visibility::d1_inherit_margin_covers_the_corpus`.
    ///
    /// [`Ellipsoid`] has nothing to inherit and returns `()`.
    fn child_extra(parent: &Self::NodeExtra, child: &TileId) -> Self::NodeExtra;

    /// A **lower** bound on the terrain surface over each sub-cell of a
    /// `4 × 4` division of this node's ground, in megametres — **D3**'s occluder, and
    /// the only thing the occlusion march reads off a node.
    ///
    /// Row-major, `u` along the row and `v` (Mercator y, north first) down the column,
    /// matching `sub_bounds`' parameterisation.
    ///
    /// # Why a grid and not one number
    ///
    /// Measured, after shipping one number and finding it did nothing. A node's footprint
    /// is sized by the *imagery* LOD, and at the stand-off where D3 matters that is a tile
    /// several kilometres across — wider than the ridge it is supposed to represent. The
    /// minimum over the whole tile is then the valley on the far side of the crest, the
    /// ridge vanishes from the occluder, and what is left culls only against the curvature
    /// horizon. Flattening the test world's ridge changed the tile counts by **zero**,
    /// which is what a "behind mountains" culler must not do. See
    /// [`HeightBounds::floor_grid`](crate::globe::terrain::HeightBounds::floor_grid).
    ///
    /// # This is not `obb_altitude_span().0`
    ///
    /// The box's lower end is the lowest point of the *drawn geometry*, skirts
    /// included, and a skirt hangs well below the ground it belongs to
    /// ([`skirt_allowance`](crate::globe::terrain::skirt_allowance) bounds C3's
    /// content-derived skirt by the tile's whole height range, which measures 1.53× the
    /// mesh interval). Using it here would sink every ridge by that much and quietly
    /// throw most of D3's benefit away. What the march needs is the lowest point of the
    /// *ground*, which is a separate, tighter number the node already carries.
    ///
    /// # It must be a lower bound, and getting that backwards is the failure mode
    ///
    /// `docs/terrain-plan.md` §3.3: only terrain that is definitely there can definitely
    /// block. An occluder taken from `h_max` over-occludes and produces exactly the false
    /// negative this engine exists to prevent — and by I-7 it deletes the whole subtree
    /// behind the ridge, not one tile.
    ///
    /// [`Ellipsoid`] returns `-∞` in every cell: the flat globe has no relief, so
    /// nothing on it can ever occlude anything the limb test has not already discarded.
    /// The march is never built on the flat arm at all, so this is unreachable there
    /// rather than merely cheap.
    fn occluder_floor(
        extra: &Self::NodeExtra,
    ) -> [f32; crate::globe::terrain::heightfield::OCCLUDER_GRID_CELLS];

    /// The per-patch payload, derived from the node's **already fitted** box.
    ///
    /// Called once per node and once per sub-patch, at construction. Phase D2's
    /// payload is the scaled-space bounding sphere of `obb`, which is why this takes
    /// the box rather than the rectangle: `T(obb)` is a parallelepiped whose eight
    /// vertices bound the patch exactly, with no sampling argument (see
    /// [`ScaledSphere::around_obb`](super::horizon::ScaledSphere::around_obb)).
    fn patch_extra(obb: &OrientedBoundingBox) -> Self::PatchExtra;

    /// Is every drawable point of this patch hidden behind the limb?
    ///
    /// **This site cannot be unified across models** (`docs/terrain-plan.md` §1):
    /// the flat test is the *exact* supremum of `q·c` over the spherical rectangle,
    /// the terrain test is a cone test on a bounding sphere, and setting that
    /// sphere's radius to zero does not recover the rectangle test. Unifying them
    /// would hand the flat globe a conservative test in place of an exact one.
    fn is_occluded(patch: &TilePatch<Self>, cam: &HorizonCamera) -> bool;

    /// The same question for one sub-patch of a node's `k × k` grid.
    ///
    /// Separate from [`Self::is_occluded`] because a sub-patch is not a
    /// [`TilePatch`]: the grid stores its `k+1` latitude and longitude breakpoints
    /// once per row and column rather than eight trig constants per sub-patch (32·(k+1)
    /// bytes against 64·k²), so the flat test is handed the φ span and the shared
    /// `A*` for the column instead of a struct.
    ///
    /// Phase C left `SubGrid::sub_patch_is_occluded` calling `span_is_occluded`
    /// directly, which was correct exactly as long as `Heightfield`'s node-level test
    /// was also still the flat one. D2 closes it: with relief, the *sub*-patch test
    /// is the same false negative as the node-level one, one level finer.
    fn sub_patch_is_occluded(
        cam: &HorizonCamera,
        a_star: f64,
        sin_lat: &[f64; 2],
        cos_lat: &[f64; 2],
        extra: &Self::PatchExtra,
    ) -> bool;
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
    type BuildCtx = ();

    /// `0.5 / 2^z` megametres — today's formula, moved here **as the same
    /// expression**, so the flat mesh is bit-for-bit what it was.
    ///
    /// C3 replaces this for terrain with a measured edge mismatch. It deliberately
    /// does **not** replace it here: with zero relief the crack at an LOD boundary is
    /// pure curvature sagitta, this constant has been chosen against exactly that, and
    /// the flat path may not move (`docs/terrain-plan.md` §4 acceptance).
    #[inline]
    fn skirt_depth(id: &TileId, _segments: u32, _ctx: &()) -> f32 {
        0.5 / 2.0_f32.powi(id.z as i32)
    }

    /// I-1 as an interval: nothing above the ellipsoid, nothing below the skirt.
    ///
    /// The two invariants agree where they overlap — I-1′ with `max = 0` *is* I-1 —
    /// which is why `test_generated_mesh_has_no_positive_altitude` stays exactly as it
    /// is rather than being replaced.
    #[inline]
    fn declared_height_bounds(_ctx: &(), skirt: f32) -> [f64; 2] {
        [-(skirt as f64), 0.0]
    }

    /// No height data exists in flat mode, so there is nothing for Phase E2 to key on.
    #[inline]
    fn height_source(_ctx: &()) -> Option<TileId> {
        None
    }

    /// Moved verbatim from `TileMesh::generate`: zero everywhere, except the skirt
    /// rows and columns, which hang `skirt_height` radially *inward*. The pole caps
    /// are excluded — they are pulled to ±90° and a skirt there would open the cap.
    ///
    /// Computed in f32 and widened, exactly as before: `skirt_height` is an f32
    /// and the old code's `alt` was too.
    #[inline]
    fn vertex_altitude(sample: &VertexSample, _ctx: &()) -> f64 {
        let alt = if sample.is_skirt && !sample.is_pole_cap {
            -sample.skirt_height
        } else {
            0.0
        };
        alt as f64
    }

    /// The analytic ellipsoid gradient at the vertex's **surface** point, normalised —
    /// the same value, from the same expression, the mesh loop already computes as
    /// [`VertexSample::up`] to displace the vertex along.
    ///
    /// Phase A had this expression here and the mesh used its result for both jobs.
    /// Phase C splits the two jobs (displacement is radial for every model, shading is
    /// not) but not the arithmetic: flat mode still evaluates the gradient exactly
    /// once per vertex and still gets the identical bits out of it.
    #[inline]
    fn vertex_normal(sample: &VertexSample, _ctx: &()) -> [f64; 3] {
        sample.up
    }

    /// Invariant I-1 restated for E1: the drawn surface **is** the ellipsoid, so the
    /// deviation between them is not small, it is identically zero — there is no
    /// geometric error here to bound and never was.
    ///
    /// This is what `src/testing/lod/`'s `texels / screen_px` metric has always rested on,
    /// and the flat half of it stays true: with this constant `false`, `apply_lod`
    /// compiles to the imagery-only threshold and the LOD harness's CSVs do not move by a
    /// byte.
    const HAS_GEOMETRIC_ERROR: bool = false;

    /// Unreachable rather than merely zero: `HAS_GEOMETRIC_ERROR` is `false`, so
    /// `apply_lod`'s `if` is a compile-time constant and this call site is not emitted.
    #[inline]
    fn geometric_error(_extra: &(), _id: &TileId) -> f32 {
        0.0
    }

    /// Zero relief: one altitude, and it is 0. `fit_obb` therefore samples each of
    /// its grid points once, at altitude 0 — today's loop exactly.
    #[inline]
    fn obb_altitude_span(_extra: &()) -> (f64, f64) {
        (0.0, 0.0)
    }

    /// Nothing to inherit: the flat globe's altitude interval is `(0, 0)` at every
    /// node of every level, known statically.
    #[inline]
    fn child_extra(_parent: &(), _child: &TileId) {}

    /// Zero relief: nothing on the flat globe occludes anything the limb test has not
    /// already thrown away, so this node can never be a D3 occluder.
    #[inline]
    fn occluder_floor(
        _extra: &(),
    ) -> [f32; crate::globe::terrain::heightfield::OCCLUDER_GRID_CELLS] {
        [f32::NEG_INFINITY; crate::globe::terrain::heightfield::OCCLUDER_GRID_CELLS]
    }

    /// No per-patch payload. The exact rectangle supremum below needs the eight trig
    /// constants and the camera, and nothing else.
    #[inline]
    fn patch_extra(_obb: &OrientedBoundingBox) {}

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

    /// Moved verbatim from `SubGrid::sub_patch_is_occluded`, whose body this was.
    /// Same expressions, same order, same arguments — the λ half is still hoisted to
    /// the column by the caller.
    #[inline]
    fn sub_patch_is_occluded(
        cam: &HorizonCamera,
        a_star: f64,
        sin_lat: &[f64; 2],
        cos_lat: &[f64; 2],
        _extra: &(),
    ) -> bool {
        span_is_occluded(cam, lat_span_max(cam, a_star, sin_lat, cos_lat))
    }
}
