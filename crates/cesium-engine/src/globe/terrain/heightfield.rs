//! The `Heightfield` surface model — C1, C2 and C3 of `docs/terrain-plan.md` §6.
//!
//! This is the second [`SurfaceModel`] implementation and the first real one: the
//! globe's skin stops being the bare ellipsoid and becomes the ellipsoid displaced
//! radially by a sampled elevation field.
//!
//! # What lives here and what does not
//!
//! Everything in this file is about turning *height samples* into *mesh inputs*.
//! Fetching, decoding and caching those samples is [`super::height_cache`]'s job, and
//! building the vertices out of the inputs is `geometry::TileMesh::generate_on`'s. The
//! seam between the first two is [`HeightPatch::sample`], which runs on the update
//! thread with `&mut HeightTileManager` in hand; the seam to the third is the
//! finished, owned [`HeightPatch`], which is `Send` and carries no borrow, so the mesh
//! build can go to a rayon worker unchanged.
//!
//! # Units
//!
//! **Megametres**, like the rest of `SurfaceModel` (see `quadtree/surface.rs`'s module
//! doc). [`HeightTileManager::height_at`] hands out megametres already; the one
//! metres-to-megametres conversion in the engine stays where Phase B put it.
//!
//! # Vertical exaggeration is applied exactly once, here
//!
//! `TerrainConfig::exaggeration` is multiplied in during [`HeightPatch::sample`] and
//! *nowhere else*. Phase B deliberately left it unused (`height_at` returns raw
//! heights), and Phase D reads [`HeightPatch::height_bounds`] — or, for a node whose
//! mesh does not exist yet, [`HeightTileManager::height_bounds_for`], which applies the
//! same factor to the same data — rather than the source tile's raw `h_min`/`h_max`, so
//! the boxes and spheres of §3.1/§3.2 inherit the exaggeration automatically instead of
//! having to remember it. One multiplication, upstream of every bound derived from it.

use crate::globe::geometry::{lon_lat_to_ecef_f64, EARTH_RADIUS_A_F64};
use crate::globe::quadtree::bounding_volume::OrientedBoundingBox;
use crate::globe::quadtree::horizon::{sphere_is_occluded, ScaledSphere};
use crate::globe::quadtree::surface::{SurfaceModel, VertexSample};
use crate::globe::quadtree::{
    tile_bounds, web_mercator_y_to_lat_f64, HorizonCamera, NodeExtraSource, TileId,
};
use crate::globe::terrain::height_cache::HeightTileManager;

/// How many levels of LOD jump across a tile edge the skirt is derived against.
///
/// C3 computes the crack as the deviation of a tile's own edge from that edge
/// *coarsened by a factor `k`*, which is exactly what a neighbour `log2(k)` levels up
/// interpolates across it. `k ∈ {2, 4}` covers a one- and a two-level jump; the
/// quadtree refines one level at a time and screen-space LOD does not produce deeper
/// steps between adjacent visible tiles in practice. A tile next to a neighbour three
/// levels coarser would need a bigger skirt than this yields — stated so that whoever
/// sees a crack there knows where to look.
const SKIRT_COARSENINGS: [u32; 2] = [2, 4];

/// Which of the four halo edges of a [`HeightPatch`] carry real data.
const HALO_LEFT: usize = 0;
const HALO_RIGHT: usize = 1;
const HALO_TOP: usize = 2;
const HALO_BOTTOM: usize = 3;

/// Whether a tile's height data is usable *yet*.
///
/// The distinction Phase B's `height_at` makes between "unknown" and "sea level" only
/// pays off if the mesh builder acts on it, which is what this enum is for: a tile
/// whose heights have not arrived is **deferred**, not flattened.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PatchStatus {
    /// A height tile — `id`'s own or an ancestor's — is resident and was sampled.
    Ready,
    /// Nothing resident yet, but a fetch for `id` or one of its ancestors is still
    /// outstanding. The caller must **retry**, not substitute zero: baking sea level
    /// into the mesh now means a flat tile that nothing has a reason to rebuild.
    Pending,
    /// Every ancestor up to the root has failed. There is no data and none is coming,
    /// so a flat mesh is the honest answer and the caller falls back to [`Ellipsoid`].
    ///
    /// [`Ellipsoid`]: crate::globe::quadtree::surface::Ellipsoid
    Unavailable,
}

/// One tile's height samples, pre-sampled on the mesh's own build grid.
///
/// # The grid, and why it has a halo
///
/// `TileMesh::generate_on` walks a `(segments+3)²` grid whose outer ring is the skirt.
/// This patch uses the **same indexing** — `grid[row * (segments+3) + col]` — but
/// gives the outer ring a different meaning: instead of repeating the edge sample, it
/// holds the height one grid step *outside* the tile. That is:
///
/// ```text
/// row r  ->  v = (r - 1) / segments        (so r = 0 is v = -1/segments)
/// col c  ->  u = (c - 1) / segments        (so c = segments+2 is u = 1 + 1/segments)
/// ```
///
/// The interior `rows/cols 1..=segments+1` are the tile's own `(segments+1)²`
/// samples; the ring around them is a **halo**, and it exists for C2. A central
/// difference at a tile's edge vertex needs the neighbour's height, which the tile
/// itself does not have. Sampling the halo out of the *same source tile* supplies it
/// exactly whenever the source is an ancestor — which is the normal case (always,
/// below z15) — because an ancestor covers the neighbour too. Where it is not
/// (`id`'s own tile is the source and `id` sits on that tile's border), the halo
/// column would fall outside the source and [`Self::halo_valid`] records the fact so
/// C2 can drop to a one-sided difference instead of silently halving the slope.
///
/// The skirt ring's *altitude* still comes from the edge, not the halo — see
/// [`Heightfield::vertex_altitude`]: skirts hang inward from the edge, they do not
/// reach outside the tile (invariant I-5).
/// How many levels coarser than a tile's own height source the data it is built from
/// may be while that source is still in flight; see [`HeightPatch::sample`]. Two levels
/// is a 4x coarser sample spacing — smoothed, but at the right height.
pub const MAX_SOURCE_LEVEL_GAP: u8 = 2;

#[derive(Clone, Debug)]
pub struct HeightPatch {
    segments: u32,
    /// `(segments+3)²` heights in **megametres**, exaggeration applied, row-major.
    grid: Vec<f64>,
    /// Extrema over the **interior** only — the halo is another tile's ground and is
    /// not part of this mesh.
    h_min: f64,
    h_max: f64,
    /// The height tile these samples came from: `id` itself, or the ancestor that
    /// answered for it. Phase E2's cache key.
    source: TileId,
    /// `1 / (2 · east step)` per row, in `Mm⁻¹`, for the central difference. Zero
    /// where the east step degenerates (at a pole).
    inv_2ds_east: Vec<f64>,
    /// `1 / (2 · north step)` per row, same convention.
    inv_2ds_north: Vec<f64>,
    /// Per-side halo validity, indexed by `HALO_*`.
    halo_valid: [bool; 4],
    /// C3's derived skirt depth, megametres.
    skirt: f32,
}

impl HeightPatch {
    /// Samples `id`'s heights out of the height cache, or says why it could not.
    ///
    /// Runs on the update thread, before the mesh is handed to a worker — that is the
    /// whole point of the split (see [`SurfaceModel::BuildCtx`]). Costs
    /// `(segments+3)²` bilinear lookups into one resident tile: 361 at the default
    /// `mesh_segments = 16`.
    pub fn sample(
        heights: &mut HeightTileManager,
        id: TileId,
        segments: u32,
        exaggeration: f32,
    ) -> Result<Self, PatchStatus> {
        // The best data resident now, even if `id`'s own tile is still in flight — the
        // way Cesium upsamples a parent's terrain for a child that has not loaded. The
        // mesh records `source`, and E2 rebuilds it once the tile's own data lands
        // (`tiles::system::fresher_height_source`). Waiting instead left the tile without
        // a mesh, and the renderer's fallback then climbed to whatever ancestor *had* one
        // — a single new tile at the edge of the view could swap the whole screen for a
        // z4 mesh whose 16x16 grid sits hundreds of metres off the real ground.
        //
        // …but only from data at most `MAX_SOURCE_LEVEL_GAP` levels coarser than the
        // tile's own source. A z12 valley tile built from z5 data is the average of 50 km
        // of Alps — a slab floating a kilometre above the valley next to neighbours built
        // from their own data, with sky showing through the gaps. Until closer data is in,
        // the tile waits and the renderer draws its nearest built ancestor instead, whose
        // own data matches its scale. When the tile's own fetch has *failed*
        // (`status_of` is `Ready` with an ancestor answering) the best ancestor is final
        // and is used whatever the gap.
        let (source, tile) = match heights.source_for(id) {
            Some(found) => found,
            None => {
                return Err(match heights.status_of(id) {
                    PatchStatus::Unavailable => PatchStatus::Unavailable,
                    _ => PatchStatus::Pending,
                })
            }
        };
        if source.z + MAX_SOURCE_LEVEL_GAP < heights.source_tile_for(id).z
            && heights.status_of(id) != PatchStatus::Ready
        {
            return Err(PatchStatus::Pending);
        }

        let grid_size = (segments + 3) as usize;
        let inv_seg = 1.0 / segments as f64;
        let exaggeration = exaggeration as f64;

        // The affine map from this tile's (u, v) into the source tile's, evaluated
        // **without** the [0,1] clamp `ancestor_uv` applies: the halo ring lives just
        // outside [0,1] by construction and clamping it would collapse it onto the
        // edge, which is precisely the degenerate central difference C2 is avoiding.
        let uv = |u: f64, v: f64| HeightTileManager::ancestor_uv_unclamped(id, source, u, v);

        let mut grid = vec![0.0; grid_size * grid_size];
        let (mut h_min, mut h_max) = (f64::INFINITY, f64::NEG_INFINITY);
        for r in 0..grid_size {
            let v = (r as f64 - 1.0) * inv_seg;
            for c in 0..grid_size {
                let u = (c as f64 - 1.0) * inv_seg;
                let (su, sv) = uv(u, v);
                // `sample_bilinear` is metres and clamps its own arguments into the
                // source tile; the clamp is what makes an invalid halo degrade to the
                // edge value rather than read out of bounds.
                let h = tile.sample_bilinear(su, sv) * 1.0e-6 * exaggeration;
                grid[r * grid_size + c] = h;

                let interior = (1..=(segments as usize + 1)).contains(&r)
                    && (1..=(segments as usize + 1)).contains(&c);
                if interior {
                    h_min = h_min.min(h);
                    h_max = h_max.max(h);
                }
            }
        }

        // A halo edge is real only if it maps strictly inside the source tile. Both
        // the map and the ring are affine, so one test per side settles the whole side.
        let (left, _) = uv(-inv_seg, 0.0);
        let (right, _) = uv(1.0 + inv_seg, 0.0);
        let (_, top) = uv(0.0, -inv_seg);
        let (_, bottom) = uv(0.0, 1.0 + inv_seg);
        let halo_valid = [left >= 0.0, right <= 1.0, top >= 0.0, bottom <= 1.0];

        let (inv_2ds_east, inv_2ds_north, step_angle) = Self::grid_metrics(id, segments);

        let mut patch = Self {
            segments,
            grid,
            h_min,
            h_max,
            source,
            inv_2ds_east,
            inv_2ds_north,
            halo_valid,
            skirt: 0.0,
        };
        patch.skirt = patch.derive_skirt(step_angle);
        Ok(patch)
    }

    /// The local east/north step lengths per grid row, and the angular size of one
    /// grid step — all pure geometry, independent of the heights.
    ///
    /// Returned as chord lengths in megametres, measured between the two points a
    /// central difference actually straddles, so the denominators below are the real
    /// distances rather than a small-angle approximation of them.
    fn grid_metrics(id: TileId, segments: u32) -> (Vec<f64>, Vec<f64>, f64) {
        let b = tile_bounds(&id);
        let grid_size = (segments + 3) as usize;
        let inv_seg = 1.0 / segments as f64;
        let dlon = (b.lon_max - b.lon_min) * inv_seg;
        let center_lon = b.center_lon();

        // Row latitudes on the same f64 Mercator definition the mesh loop uses, the
        // halo rows included (they simply fall one step outside the tile).
        let lat: Vec<f64> = (0..grid_size)
            .map(|r| {
                let v = (r as f64 - 1.0) * inv_seg;
                web_mercator_y_to_lat_f64(id.y as f64 + v, id.z)
            })
            .collect();

        let dist = |a: [f64; 3], c: [f64; 3]| {
            let d = [a[0] - c[0], a[1] - c[1], a[2] - c[2]];
            (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt()
        };

        let mut inv_2ds_east = vec![0.0; grid_size];
        let mut inv_2ds_north = vec![0.0; grid_size];
        let mut max_step = 0.0_f64;
        for r in 0..grid_size {
            let ds_e = dist(
                lon_lat_to_ecef_f64(center_lon - dlon, lat[r]),
                lon_lat_to_ecef_f64(center_lon + dlon, lat[r]),
            );
            let ds_n = if r == 0 || r + 1 == grid_size {
                0.0
            } else {
                dist(
                    lon_lat_to_ecef_f64(center_lon, lat[r - 1]),
                    lon_lat_to_ecef_f64(center_lon, lat[r + 1]),
                )
            };
            // A pole row has no east extent at all; a zero denominator there would
            // hand the shader a NaN normal. No east gradient is the right answer.
            inv_2ds_east[r] = if ds_e > 1.0e-12 { 1.0 / ds_e } else { 0.0 };
            inv_2ds_north[r] = if ds_n > 1.0e-12 { 1.0 / ds_n } else { 0.0 };
            max_step = max_step.max(ds_e.max(ds_n) * 0.5);
        }

        (inv_2ds_east, inv_2ds_north, max_step / EARTH_RADIUS_A_F64)
    }

    /// C3 — the skirt depth, measured instead of assumed.
    ///
    /// The crack at an LOD boundary is the disagreement between this tile's edge and
    /// the straight line a coarser neighbour draws across the same edge. Both are in
    /// hand here, so the disagreement is a maximum over four edges rather than a
    /// content-blind estimate like Cesium's `min(4·levelError, 1000 m)`.
    ///
    /// Two terms, both derived:
    ///
    /// 1. **Height mismatch.** For a coarsening factor `k`, the neighbour joins every
    ///    `k`-th of this edge's vertices with a straight line, so the crack is
    ///    `max_i |h[i] − lerp(h[i₀], h[i₁], …)|` over that edge.
    /// 2. **Curvature sagitta.** Even at zero relief the neighbour's edge is a *chord*
    ///    of this one's arc, `R·(1 − cos(k·δ/2))` deep for an angular grid step `δ`.
    ///    Sub-metre at every level, and it is what keeps an all-ocean tile's skirt from
    ///    collapsing to nothing.
    ///
    /// Taken over `k ∈ ` [`SKIRT_COARSENINGS`] and maximised.
    fn derive_skirt(&self, step_angle: f64) -> f32 {
        let n = self.segments as usize;
        let g = self.segments as usize + 1; // last interior index along an edge
        let edge_h = |e: usize, i: usize| -> f64 {
            match e {
                0 => self.at(1, i + 1), // north edge
                1 => self.at(g, i + 1), // south edge
                2 => self.at(i + 1, 1), // west edge
                _ => self.at(i + 1, g), // east edge
            }
        };

        let mut worst = 0.0_f64;
        for &k in &SKIRT_COARSENINGS {
            let k = k as usize;
            let mut mismatch = 0.0_f64;
            for e in 0..4 {
                for i in 0..=n {
                    let i0 = (i / k) * k;
                    let i1 = (i0 + k).min(n);
                    if i1 == i0 {
                        continue;
                    }
                    let t = (i - i0) as f64 / (i1 - i0) as f64;
                    let interp = edge_h(e, i0) + (edge_h(e, i1) - edge_h(e, i0)) * t;
                    mismatch = mismatch.max((edge_h(e, i) - interp).abs());
                }
            }
            let sagitta = EARTH_RADIUS_A_F64 * (1.0 - (k as f64 * step_angle * 0.5).cos()).max(0.0);
            worst = worst.max(mismatch + sagitta);
        }
        worst as f32
    }

    /// One grid sample, by `(row, col)` in the build grid.
    #[inline]
    fn at(&self, row: usize, col: usize) -> f64 {
        self.grid[row * (self.segments as usize + 3) + col]
    }

    /// `(row, col)` clamped into the **interior** — the edge sample a skirt or
    /// pole-cap vertex inherits its altitude from.
    #[inline]
    fn edge_clamped(&self, row: u32, col: u32) -> (usize, usize) {
        let last = self.segments + 1;
        (row.clamp(1, last) as usize, col.clamp(1, last) as usize)
    }

    /// The `[min, max]` height interval of this patch's interior, **megametres**,
    /// exaggeration already applied. Phase D reads this, not the source tile's raw
    /// `h_min`/`h_max`.
    pub fn height_bounds(&self) -> [f64; 2] {
        [self.h_min, self.h_max]
    }

    /// The height tile these samples came from.
    pub fn source(&self) -> TileId {
        self.source
    }

    /// C3's derived skirt depth, megametres.
    pub fn skirt(&self) -> f32 {
        self.skirt
    }

    /// Whether the halo on one side of the patch holds real neighbour data. See the
    /// struct doc; `HALO_LEFT` … `HALO_BOTTOM`.
    pub fn halo_valid(&self) -> [bool; 4] {
        self.halo_valid
    }

    /// A patch of constant height, for tests and for a tile whose data will never
    /// arrive. Not used by the engine's normal path — `PatchStatus::Unavailable`
    /// falls back to `Ellipsoid` rather than to a flat height field, so the flat mesh
    /// stays bit-for-bit the flat mesh.
    pub fn flat(id: TileId, segments: u32, height_mm: f64) -> Self {
        let grid_size = (segments + 3) as usize;
        let (inv_2ds_east, inv_2ds_north, step_angle) = Self::grid_metrics(id, segments);
        let mut patch = Self {
            segments,
            grid: vec![height_mm; grid_size * grid_size],
            h_min: height_mm,
            h_max: height_mm,
            source: id,
            inv_2ds_east,
            inv_2ds_north,
            halo_valid: [true; 4],
            skirt: 0.0,
        };
        patch.skirt = patch.derive_skirt(step_angle);
        patch
    }
}

/// The altitude interval, in **megametres**, a node's bounding volumes are fitted
/// over — [`SurfaceModel::NodeExtra`] for [`Heightfield`], and the whole of Phase D1.
///
/// # This is the *box* span, not the height field's range
///
/// `hi` is the highest sample the node's mesh can reach, but `lo` is **not** the
/// lowest: it is the lowest sample minus the deepest skirt that mesh can hang, because
/// a skirt vertex outside the box is a drawable point outside the box, which is a false
/// negative in the frustum stage exactly like a summit outside it. C3 made the skirt
/// content-dependent (`docs/terrain-plan.md` §6), so it is no longer a number the
/// quadtree could hard-code; [`skirt_allowance`] bounds it from the same two things the
/// patch derives it from, and [`HeightTileManager::height_bounds_for`] folds it in
/// before the interval ever reaches a node.
///
/// # Units
///
/// Megametres, exaggeration already applied — the contract `quadtree/surface.rs`'s
/// module doc states for every altitude in the trait, and the reason the conversion
/// happens once, at the height cache's boundary, rather than here.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HeightBounds {
    /// Lowest altitude the node's geometry can reach, skirt included.
    pub lo: f64,
    /// Highest altitude the node's geometry can reach.
    pub hi: f64,
    /// Lowest altitude the node's **ground** can reach — `lo` *without* the skirt
    /// allowance. **D3's occluder**, and the reason this is a third number rather than
    /// a derived one.
    ///
    /// `lo` is the right bound for a bounding volume, because a skirt vertex outside the
    /// box is as much a false negative as a summit outside it. It is the wrong bound for
    /// an occluder: an occluder is a claim about where solid ground *is*, and the skirt
    /// hangs into empty space below the ground precisely so that a crack at an LOD
    /// boundary is covered. [`skirt_allowance`] bounds C3's content-derived skirt by the
    /// tile's entire height range (measured: the node interval is 1.53× the mesh
    /// interval it contains), so an occluder taken from `lo` would sit kilometres below
    /// the ridge it is supposed to represent and D3 would cull almost nothing.
    ///
    /// Always `lo <= floor <= hi`. On an inherited interval it widens downward by
    /// [`inherit_margin_mm`] — the table's per-level margin and **nothing else**, unlike
    /// `lo`, which also hands back the parent's whole-tile skirt allowance. That
    /// asymmetry is measured rather than assumed; see [`Heightfield::child_extra`]. Loose
    /// in the safe direction either way: a floor that is too low occludes too little,
    /// which costs false positives and never a subtree. The reason it is not *allowed* to
    /// be arbitrarily loose is that a march cell's floor is a **minimum** over everything
    /// stamped into it, so a single bottomless node takes its whole cell with it.
    pub floor: f64,
    /// [`Self::floor`] again, but per **sub-cell** of a
    /// [`OCCLUDER_GRID`]×[`OCCLUDER_GRID`] division of the tile — row-major, `u` along the
    /// row, `v` (Mercator y, so north first) down the column. Megametres, as `f32`.
    ///
    /// # Why the whole-tile minimum is not enough, measured
    ///
    /// This is the difference between D3 culling *behind mountains* and D3 culling behind
    /// the local curvature horizon, and the first implementation did the latter without
    /// anybody noticing until the ridge was flattened and the tile counts did not move.
    ///
    /// A node's occluder has to be a minimum over its footprint, and the LOD sizes that
    /// footprint for *imagery*, not for ridges. On the ridge-world valley pose the crest
    /// lands near the southern edge of a z12 tile 6.6 km across; the minimum over that
    /// whole tile is the ground 6 km away on the far side, 1 132 m, against a crest of
    /// 3 400 m. The ridge simply disappears from the occluder. Split the same tile into
    /// 4×4 and the southern row's minimum is 3 176 m — the ridge, back.
    ///
    /// Each entry is a minimum over its own sub-rectangle taken from the same 16×16 mip,
    /// so it is sound for exactly the reason [`Self::floor`] is, and it inherits the same
    /// way: a child that has no data of its own gets its inherited scalar floor in every
    /// cell, because a parent's sub-cells are not a child's.
    pub floor_grid: [f32; OCCLUDER_GRID_CELLS],
    /// The node's **measured geometric error** — megametres, exaggeration applied, the
    /// deviation of the drawn mesh from the DEM ([`HeightTile::detail`]). E1 of
    /// `docs/terrain-plan.md` §8, and the only field here that is not about a bounding
    /// volume.
    ///
    /// It rides in `HeightBounds` rather than in a payload of its own because the
    /// quadtree has exactly one per-node channel out of the height cache
    /// ([`NodeExtraSource`]), and a second one would mean a second per-frame walk of the
    /// tree for four bytes.
    ///
    /// # Loose in the *upward* direction, unlike everything above it
    ///
    /// `lo`, `hi` and `floor` are bounds and a wrong one is a hole in the globe. This is
    /// a **tuning input**: too large refines early (tiles, bandwidth), too small refines
    /// late (a coarse mountain). Nothing about soundness passes through it, which is why
    /// the inheritance fallback below is allowed to be a level-based guess where the
    /// others must be measured margins.
    ///
    /// [`HeightTile::detail`]: crate::globe::terrain::HeightTile::detail
    /// [`NodeExtraSource`]: crate::globe::quadtree::NodeExtraSource
    pub detail: f32,
}

/// Sub-cells per axis in [`HeightBounds::floor_grid`].
///
/// 4 rather than 2 or 8, by the two things it trades. Up: a sub-cell's minimum is the
/// resolution at which a ridge survives being averaged with the ground beside it, and a
/// quarter of a tile is the coarsest division that resolves an Alpine crest inside the z12
/// tile the LOD gives it at typical stand-off. Down: it is 16 extra `mip_extrema_over`
/// queries per node per bounds refresh and 16 extra stamps per node per march, and the mip
/// is 16×16 — at `OCCLUDER_GRID = 8` a sub-cell is two mip cells wide and the halo makes
/// it four, so the resolution stops improving while the cost keeps going up.
pub const OCCLUDER_GRID: usize = 4;

/// `OCCLUDER_GRID²`.
pub const OCCLUDER_GRID_CELLS: usize = OCCLUDER_GRID * OCCLUDER_GRID;

/// The deepest trench the source can report, metres — Challenger Deep is −10 924 m.
///
/// Only reachable under [`OceanPolicy::Raw`](crate::globe::tiles::config::OceanPolicy);
/// the default `ClampToZero` never goes below 0. Used for the root fallback, which must
/// be sound under **either** policy because the quadtree does not know which is
/// configured.
const GLOBAL_H_MIN_M: f64 = -11_500.0;

/// The highest ground the source can report, metres — Everest is 8 849 m, and the
/// source's own z12 Everest tile peaks at 8 740 m.
const GLOBAL_H_MAX_M: f64 = 9_500.0;

impl Default for HeightBounds {
    /// The whole range the Earth's solid surface occupies — the interval a **root**
    /// node starts with, and the only interval available before anything has been
    /// fetched.
    ///
    /// `docs/terrain-plan.md` §7's first policy row, used exactly where that row is
    /// cheap: at z1 a 21-km-tall box over a 10 000-km tile is nothing. Every node below
    /// a root either has its own data or inherits through [`Heightfield::child_extra`],
    /// which is the third row.
    fn default() -> Self {
        Self {
            lo: GLOBAL_H_MIN_M * 1.0e-6,
            hi: GLOBAL_H_MAX_M * 1.0e-6,
            // The deepest the Earth's solid surface goes: a root guarantees nothing, and
            // a floor at the bottom of the Challenger Deep occludes nothing, which is
            // the right answer before any data has arrived.
            floor: GLOBAL_H_MIN_M * 1.0e-6,
            floor_grid: [(GLOBAL_H_MIN_M * 1.0e-6) as f32; OCCLUDER_GRID_CELLS],
            // A root has no data and no parent, so it gets the level-based fallback at
            // the coarsest level this table defines. The engine's roots are at z1 and
            // subdivide at every camera anyway, so which end of `z ∈ {0, 1}` this reads
            // has never decided anything; z0 is the conservative one.
            detail: fallback_detail_mm(0) as f32,
        }
    }
}

impl HeightBounds {
    /// Widened by `margin` megametres on both ends.
    #[inline]
    pub fn widened(self, margin: f64) -> Self {
        Self {
            lo: self.lo - margin,
            hi: self.hi + margin,
            // Downward, like `lo`: a child's ground can dip below what its parent's
            // coarser DEM resolved, and an occluder that is too high is the one error
            // this whole file is arranged to prevent.
            floor: self.floor - margin,
            // A widened interval has lost its sub-cell structure: the caller is about to
            // apply it to different ground. The scalar floor is the only thing still true
            // of every part of it.
            floor_grid: [(self.floor - margin) as f32; OCCLUDER_GRID_CELLS],
            // Not a bound and not widened by one. `margin` answers "how far outside its
            // parent's interval can a child's ground be?", which says nothing about how
            // rough that ground is; the error term's own inheritance rule is
            // `Heightfield::child_extra`'s level-based fallback.
            detail: self.detail,
        }
    }

    /// Does this interval contain `other`?
    #[inline]
    pub fn contains(&self, other: &HeightBounds) -> bool {
        self.lo <= other.lo && other.hi <= self.hi
    }
}

/// How far a child node's height interval may fall outside its parent's, per level,
/// in **metres** — the measured margin of `docs/terrain-plan.md` §7's third policy.
///
/// # What is being measured, and why it is not zero
///
/// A node is culled long before its own height tile arrives, so while it waits it must
/// use its parent's interval. That interval is **not** a superset of the child's: the
/// parent's DEM covers four times the ground at half the resolution, so it smooths away
/// peaks and fills in notches that the child's own tile resolves. Inheriting unmodified
/// is therefore an FN source, and by I-7 an FN at one node deletes its whole subtree.
///
/// The quantity is `max(child.hi − parent.hi, parent.lo − child.lo)` over every
/// parent/child pair in the committed corpus
/// (`assets/terrain_fixtures/pyramid_extrema.csv` — 16 regions, chains from z1 to z15
/// with all four children at each step), computed on the *box spans*
/// [`HeightTileManager::height_bounds_for`] produces rather than on the raw extrema, so
/// it covers [`skirt_allowance`]'s level dependence too.
///
/// # Measured (2026-09-20, 788 tiles, 720 parent/child pairs)
///
/// | child z | pairs | max needed (m) | p99 (m) | margin here (m) | headroom |
/// |--:|--:|--:|--:|--:|--:|
/// | 2  | 16 | 698    | 383  | 20 000 | 28× |
/// | 3  | 36 | **4 566** | 1 668 | 20 000 | 4.4× |
/// | 4  | 44 | 492    | 384  | 20 000 | 41× |
/// | 5  | 56 | 461    | 183  | 8 000  | 17× |
/// | 6  | 60 | 1 061  | 889  | 8 000  | 7.5× |
/// | 7  | 60 | 1 292  | 787  | 8 000  | 6.2× |
/// | 8  | 64 | 1 365  | 221  | 8 000  | 5.9× |
/// | 9  | 64 | 166    | 155  | 6 000  | 36× |
/// | 10 | 64 | 833    | 637  | 6 000  | 7.2× |
/// | 11 | 64 | 1 359  | 50   | 6 000  | 4.4× |
/// | 12 | 64 | 36     | 25   | 1 500  | 42× |
/// | 13 | 64 | 11     | 9    | 750    | 68× |
/// | 14 | 64 | 3      | 2    | 400    | 133× |
/// | 15 | 64 | 3      | 3    | 200    | 67× |
///
/// The z3 row is the whole argument in one number: the z3 tile over eastern Greenland
/// reports a 7 796 m maximum where its z2 parent reports 3 230 m. One z2 texel is ~150 km
/// across, which averages that spike out of existence; the z3 tile at ~75 km resolves it.
/// A child interval inherited unmodified would have been **4.6 km too shallow** there.
///
/// The medians are all *negative* (−29 m to −1 052 m), i.e. in the typical case the
/// parent's interval already contains the child's and no margin is needed at all. It is
/// the tail this table is sized for, and a sample of 720 pairs bounds a tail only so
/// far — hence headroom of 4× at the tightest level rather than a fitted curve.
///
/// # Exaggeration
///
/// Measured at `exaggeration = 1.0`. The height-dependent part of the requirement scales
/// linearly with it, so the 4.4× headroom at the binding levels covers exaggeration up to
/// ~4; past that the table needs re-measuring. Stated rather than asserted because
/// `child_extra` is a static dispatch with no access to the config.
///
/// # Below the source's deepest level the margin is exactly zero
///
/// Past this table's length (z ≥ 16) it reads `0.0`, and that is exact rather than
/// optimistic: there the child's interval is read from the *same* height tile as its
/// parent's, over a dyadic sub-rectangle of the parent's, so its covering mip cells are a
/// subset of the parent's and its extrema are contained by construction. See
/// [`HeightTileManager::height_bounds_for`] and `HeightTile::mip_extrema_over`.
///
/// Indexed by the **child's** level; levels 0 and 1 are roots or their children.
/// `testing::terrain::test_terrain_visibility::d1_inherit_margin_covers_the_corpus` re-derives
/// the middle column from `assets/terrain_fixtures/pyramid_extrema.csv` and fails if any
/// entry here stops covering it.
const HEIGHT_INHERIT_MARGIN_M: [f64; 16] = [
    // z0    z1      z2      z3      z4     z5     z6     z7
    20_000.0, 20_000.0, 20_000.0, 20_000.0, 20_000.0, 8_000.0, 8_000.0, 8_000.0,
    // z8   z9     z10    z11    z12     z13    z14    z15
    8_000.0, 6_000.0, 6_000.0, 6_000.0, 1_500.0, 750.0, 400.0, 200.0,
];

/// Mesh density the inheritance allowance is evaluated at.
///
/// The shipped default (`docs/terrain-plan.md` §6 C4 keeps 16). [`skirt_allowance`]'s
/// sagitta term *falls* with density, so evaluating at 16 is an upper bound for every
/// mesh built at 16 or finer. A configuration that lowered `mesh_segments` below 16 would
/// need this raised with it — stated here because [`Heightfield::child_extra`] is a static
/// dispatch and has no way to read the configuration.
const INHERIT_SEGMENTS: u32 = 16;

/// Widest height range a **Ready** node's data can report, metres — the whole span the
/// Earth's solid surface occupies, times headroom for vertical exaggeration.
///
/// `GLOBAL_H_MAX_M − GLOBAL_H_MIN_M` is 21 km; the factor of 4 matches the exaggeration
/// range [`HEIGHT_INHERIT_MARGIN_M`] already declares its table to cover, and is here for
/// the same reason: [`Heightfield::child_extra`] is a static dispatch with no access to
/// `TerrainConfig::exaggeration`, and the quantity being bounded scales linearly with it.
const INHERIT_RANGE_M: f64 = 4.0 * (GLOBAL_H_MAX_M - GLOBAL_H_MIN_M);

/// An upper bound, **megametres**, on the *whole-tile* skirt allowance the **parent** of
/// `child` could have had — what [`Heightfield::child_extra`] must hand back when it
/// widens **the interval**. See that function for why it is owed there, and for why
/// [`HeightBounds::floor`] does not owe it at all.
///
/// `skirt_allowance(parent, 16, INHERIT_RANGE_M)` is exactly `range + sagitta(parent)`
/// with `range` at its global maximum, which bounds `allow_whole(parent)` for any data the
/// source can return. It depends only on the parent's level and tile shape, so unlike the
/// parent's own interval it cannot compound down an unloaded chain.
#[inline]
pub fn inherit_allowance_mm(child: &TileId) -> f64 {
    match child.parent() {
        Some(p) => skirt_allowance(p, INHERIT_SEGMENTS, INHERIT_RANGE_M * 1.0e-6),
        // A root has no parent to owe anything to; it starts from `HeightBounds::default`.
        None => 0.0,
    }
}

/// Level-zero geometric error, **metres** — the fallback [`fallback_detail_mm`] halves
/// per level, and the one number in E1 that is *not* measured off the data.
///
/// Cesium's `getEstimatedLevelZeroGeometricErrorForAHeightmap(ellipsoid, tileWidth,
/// tilesAtLevelZero)` is `maximumRadius · 2π / (tileWidth · tilesAtLevelZero)`. For this
/// engine's scheme — Web Mercator, **one** tile at z0 — and Cesium's own shipped
/// `tileWidth = 65`, that is `2π · 6 378 137 / 65 = 616 538 m`, which is the value used
/// here. The table it generates, and the measured errors it stands in for, are in
/// `docs/terrain-plan.md` §8.
const LEVEL_ZERO_DETAIL_M: f64 = 616_538.0;

/// The deepest level the terrain LOD term is allowed to demand refinement *into* — the
/// default of `TerrainConfig::detail_max_z`, and **19 since §9 F5**, not 15.
///
/// # What F5 corrected
///
/// E1 set this to `TERRARIUM_MAX_LEVEL` with Cesium's argument: *"past the source's deepest
/// level a node's mesh is an interpolation of its z15 ancestor's samples, so the refinement
/// it would buy is arithmetic and not shape"*. That argument is sound **for Cesium**, where
/// `HeightmapTerrainData`'s `width × height` *is* the mesh lattice and `upsample` resamples
/// the parent's mesh, so a descendant genuinely holds nothing new.
///
/// It does not hold here. The source tile is 256 × 256 and the mesh is 17 × 17, so a z15
/// tile already decimates its own data **16:1**. A z16 node draws the same lattice over a
/// quarter of that tile — 8:1 — z17 draws 4:1, z18 draws 2:1, and only at **z19** does the
/// lattice land on every texel and the mesh become exact. Four levels of resolved,
/// already-fetched shape were being reported as zero error.
///
/// # Why 19 is a fact and not a taste
///
/// `HeightTile::detail_below` returns exactly zero for a descendant four or more levels
/// below its source, because at that depth the mesh reproduces the field. So the clamp is
/// redundant with the data at 19 and the constant is kept for two narrower jobs: it bounds
/// [`fallback_detail_mm`], the level-based stand-in for a node whose tile has not arrived
/// and which therefore has no data to be bounded by; and it is the knob §9 F5's cost table
/// sweeps and a device measurement could lower. `mesh_segments = 16` is baked into the 4 —
/// at 32 the ladder would reach 1:1 one level earlier.
pub const DETAIL_MAX_Z: u8 = 19;

/// The level-based geometric error for a node at level `z`, **megametres** — E1's
/// fallback for a node whose own height tile has not landed.
///
/// `LEVEL_ZERO_DETAIL_M / 2^z`, which is Cesium's heightmap rule exactly: content-blind,
/// halving per level. It is kept for the one case where the measured number cannot exist —
/// a node the quadtree created this frame, whose data is still in flight — and is replaced
/// by [`HeightTile::detail`](crate::globe::terrain::HeightTile::detail) the frame it
/// arrives.
///
/// Zero at and below [`DETAIL_MAX_Z`], for that constant's reason.
#[inline]
pub fn fallback_detail_mm(z: u8) -> f64 {
    if z >= DETAIL_MAX_Z {
        return 0.0;
    }
    LEVEL_ZERO_DETAIL_M * 1.0e-6 / (1u64 << z.min(40)) as f64
}

/// The inheritance margin for a node at level `z`, **megametres**.
#[inline]
pub fn inherit_margin_mm(z: u8) -> f64 {
    let m = HEIGHT_INHERIT_MARGIN_M
        .get(z as usize)
        .copied()
        .unwrap_or(0.0);
    m * 1.0e-6
}

/// An upper bound, in **megametres**, on the skirt [`HeightPatch::derive_skirt`] can
/// produce for a tile at level `z` with a height range of `range` megametres.
///
/// C3's skirt is `max over k ∈ {2, 4}` of (the edge's deviation from its own
/// `k`-coarsening) + (the curvature sagitta of the chord a neighbour draws across `k`
/// grid steps). Both terms are bounded here from things the quadtree knows:
///
/// 1. The deviation of an edge from a linear interpolation *of that same edge* cannot
///    exceed the edge's own range, which cannot exceed the tile's.
/// 2. The sagitta is `R·(1 − cos(k·δ/2))` for an angular grid step `δ`, maximised at
///    `k = 4`. `δ` is taken as the larger of the tile's longitude and latitude steps —
///    latitude matters, because a Mercator tile at low zoom is far taller than it is
///    wide and the pole row is stretched to ±90°.
///
/// Both are upper bounds, so the result is one, which is what I-6 needs: a box that is
/// too deep costs false positives, a box that is too shallow costs the subtree.
///
/// `segments` is the mesh density (`TileEngineConfig::mesh_segments`); the sagitta falls
/// with it, so a caller that passes the shipped 16 while the mesh is built at 32 is
/// still conservative.
pub fn skirt_allowance(id: TileId, segments: u32, range_mm: f64) -> f64 {
    let b = tile_bounds(&id);
    let inv_seg = 1.0 / segments.max(1) as f64;
    let dlon = (b.lon_max - b.lon_min).to_radians() * inv_seg;
    let dlat = (b.lat_max - b.lat_min).to_radians() * inv_seg;
    let delta = dlon.max(dlat);
    // k = 4, so the chord spans 4 steps and the half-angle is 2δ.
    let sagitta = EARTH_RADIUS_A_F64 * (1.0 - (2.0 * delta).cos()).max(0.0);
    range_mm + sagitta
}

/// The quadtree's view of the height cache — Phase D1's feed, and the only coupling
/// between the two.
///
/// Exists because the interval a node needs is not a property of the cache alone: it
/// depends on the mesh density and the vertical exaggeration the *geometry* will be
/// built at, and a bound derived at different settings from the mesh it is supposed to
/// contain is no bound at all. Carrying both here means the quadtree side cannot forget
/// either one.
///
/// Borrows the manager immutably, so the per-frame refresh walk cannot reorder the LRU
/// by looking — see [`NodeExtraSource`].
pub struct HeightBoundsSource<'a> {
    pub heights: &'a HeightTileManager,
    /// `TileEngineConfig::mesh_segments`, as the mesh will be built.
    pub segments: u32,
    /// `TerrainConfig::exaggeration`, as [`HeightPatch::sample`] will apply it.
    pub exaggeration: f32,
    /// `TerrainConfig::detail_max_z` — **F5**. The deepest level the geometric term may
    /// demand refinement into; at and below it a node's stored error is zero. It rides here
    /// rather than in [`Heightfield::geometric_error`] because that is a static dispatch
    /// with no access to the configuration, which is the same reason `segments` rides here.
    pub detail_max_z: u8,
}

impl NodeExtraSource<Heightfield> for HeightBoundsSource<'_> {
    #[inline]
    fn extra_for(&self, id: &TileId) -> Option<HeightBounds> {
        self.heights
            .height_bounds_for(*id, self.segments, self.exaggeration, self.detail_max_z)
    }
}

/// The globe with relief: the ellipsoid displaced radially by a sampled height field.
///
/// Phase C gave this model its geometry; **Phase D gives it its culling**. The two
/// payloads below are D1 and D2 of `docs/terrain-plan.md` §7:
///
/// * [`HeightBounds`] per node — the altitude interval `fit_obb` sweeps, so a node's
///   box contains the relief inside it instead of hugging the ellipsoid under it. This
///   is what the Phase C captures were missing: the visible set over the Alps at 4.5 km
///   was byte-identically the flat one, its geometry was lifted by up to 2.9 km, and the
///   tiles that should have filled the gap underneath were frustum-culled against boxes
///   fitted at `alt = 0`.
/// * [`ScaledSphere`] per patch — Theorem 3.7's cone test, which stays sound when a
///   summit satisfies `q·c ≤ 1` and is visible over the limb anyway.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Heightfield;

impl SurfaceModel for Heightfield {
    type NodeExtra = HeightBounds;
    type PatchExtra = ScaledSphere;
    type BuildCtx = HeightPatch;

    /// C3's measured value. See [`HeightPatch::derive_skirt`].
    #[inline]
    fn skirt_depth(_id: &TileId, _segments: u32, ctx: &HeightPatch) -> f32 {
        ctx.skirt()
    }

    /// I-1′: the patch's own interior extrema, widened downward by the skirt, which is
    /// the only thing in the mesh that goes below the sampled field.
    #[inline]
    fn declared_height_bounds(ctx: &HeightPatch, skirt: f32) -> [f64; 2] {
        let [lo, hi] = ctx.height_bounds();
        [lo - skirt as f64, hi]
    }

    #[inline]
    fn height_source(ctx: &HeightPatch) -> Option<TileId> {
        Some(ctx.source())
    }

    /// C1 — the sampled height, and for a skirt vertex that height minus the skirt.
    ///
    /// Structurally identical to `Ellipsoid`'s `0` / `-skirt`, with the zero replaced
    /// by the field. The skirt row reads the **edge** sample rather than its own halo
    /// slot: a skirt hangs inward from the edge it belongs to, it does not reach into
    /// the neighbour (I-5). The pole caps keep the edge height with no skirt at all,
    /// exactly as in flat mode — a skirt there would open the cap.
    #[inline]
    fn vertex_altitude(sample: &VertexSample, ctx: &HeightPatch) -> f64 {
        let (r, c) = ctx.edge_clamped(sample.row, sample.col);
        let h = ctx.at(r, c);
        if sample.is_skirt && !sample.is_pole_cap {
            h - sample.skirt_height as f64
        } else {
            h
        }
    }

    /// C2 — central differences on the height field in the tile's local east/north
    /// frame.
    ///
    /// Without this the relief is visible in silhouette and invisible in shading: the
    /// fragment shader takes `out.normal` at face value, so every slope would light as
    /// if it were smooth sphere.
    ///
    /// # Edges
    ///
    /// The stencil reaches one step outside the tile, where the tile has no data of
    /// its own. It reads the patch's **halo** (see [`HeightPatch`]), which is sampled
    /// from the same source tile and is therefore the true neighbour height whenever
    /// the source is an ancestor — the normal case, and the only case below z15.
    /// Where the halo is not real ([`HeightPatch::halo_valid`]: `id` *is* the source
    /// and lies on its border), the difference drops to **one-sided**, with the
    /// one-sided denominator. Using the halo there anyway would read the clamped edge
    /// value over a two-step baseline and report half the true slope — a visible
    /// flattening along that one vertex ring, and the subtler of the two failures,
    /// which is why it is detected rather than tolerated.
    ///
    /// Skirt vertices inherit their edge vertex's normal, so the skirt is shaded like
    /// the ground it hangs from.
    #[inline]
    fn vertex_normal(sample: &VertexSample, ctx: &HeightPatch) -> [f64; 3] {
        let (r, c) = ctx.edge_clamped(sample.row, sample.col);
        let last = ctx.segments as usize + 1;
        let valid = ctx.halo_valid;

        // ∂h/∂east, megametres per megametre. One-sided where the halo is not real.
        let d_east = if c == 1 && !valid[HALO_LEFT] {
            (ctx.at(r, c + 1) - ctx.at(r, c)) * 2.0 * ctx.inv_2ds_east[r]
        } else if c == last && !valid[HALO_RIGHT] {
            (ctx.at(r, c) - ctx.at(r, c - 1)) * 2.0 * ctx.inv_2ds_east[r]
        } else {
            (ctx.at(r, c + 1) - ctx.at(r, c - 1)) * ctx.inv_2ds_east[r]
        };

        // ∂h/∂north. Row index grows southward (v grows with latitude decreasing), so
        // the northward difference is `row-1` minus `row+1`.
        let d_north = if r == 1 && !valid[HALO_TOP] {
            (ctx.at(r, c) - ctx.at(r + 1, c)) * 2.0 * ctx.inv_2ds_north[r]
        } else if r == last && !valid[HALO_BOTTOM] {
            (ctx.at(r - 1, c) - ctx.at(r, c)) * 2.0 * ctx.inv_2ds_north[r]
        } else {
            (ctx.at(r - 1, c) - ctx.at(r + 1, c)) * ctx.inv_2ds_north[r]
        };

        // The local frame. `east` is the normalised ∂p/∂λ of the engine's Y-up,
        // negated-Z ECEF frame; `north = up × east` completes it, and is exactly unit
        // because the ellipsoid normal lies in the meridian plane and so is orthogonal
        // to east.
        let theta = sample.lon_deg.to_radians();
        let (sin_t, cos_t) = theta.sin_cos();
        let east = [-sin_t, 0.0, -cos_t];
        let up = sample.up;
        let north = [
            up[1] * east[2] - up[2] * east[1],
            up[2] * east[0] - up[0] * east[2],
            up[0] * east[1] - up[1] * east[0],
        ];

        // For a surface p(E, N) = p₀ + E·ê + N·n̂ + h(E, N)·û the tangents are
        // ê + h_E·û and n̂ + h_N·û, whose cross product is û − h_E·ê − h_N·n̂.
        let mut n = [
            up[0] - d_east * east[0] - d_north * north[0],
            up[1] - d_east * east[1] - d_north * north[1],
            up[2] - d_east * east[2] - d_north * north[2],
        ];
        let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
        if len > 0.0 && len.is_finite() {
            n[0] /= len;
            n[1] /= len;
            n[2] /= len;
            n
        } else {
            up
        }
    }

    /// **D1** — the node's own interval, so `fit_obb` sweeps its grid at both ends and
    /// the box spans the relief instead of hugging the ellipsoid under it.
    ///
    /// Always a non-degenerate interval ([`skirt_allowance`]'s sagitta term is strictly
    /// positive at every level), so `fit_obb`'s `alt_max != alt_min` guard always takes
    /// the two-sample branch here and never takes it for [`Ellipsoid`].
    ///
    /// [`Ellipsoid`]: crate::globe::quadtree::Ellipsoid
    #[inline]
    fn obb_altitude_span(extra: &HeightBounds) -> (f64, f64) {
        (extra.lo, extra.hi)
    }

    /// **E1** — this globe has relief, so it has an error to refine against.
    const HAS_GEOMETRIC_ERROR: bool = true;

    /// **E1, as F5 left it** — the node's measured deviation from the DEM, read and
    /// nothing else.
    ///
    /// E1 clamped here, at `id.z >= DETAIL_MAX_Z`, on the argument that the ceiling is a
    /// statement about the LOD rule rather than about the data. F5 moved it, because that
    /// stopped being true: below the source ceiling the error is
    /// `HeightTile::detail_below`'s per-window, per-lattice measurement, which reaches
    /// exactly zero on its own four levels down, and the remaining ceiling is a
    /// configuration value (`TerrainConfig::detail_max_z`) this static dispatch cannot see.
    /// Both now happen in `HeightTileManager::height_bounds_for`, which has the tile and the
    /// config in hand, so there is one site rather than two that could disagree.
    #[inline]
    fn geometric_error(extra: &HeightBounds, _id: &TileId) -> f32 {
        extra.detail
    }

    /// **D3** — the node's ground floor, not its box floor. See
    /// [`HeightBounds::floor`] for why those are two different numbers and
    /// [`SurfaceModel::occluder_floor`] for why this side must be the lower bound.
    #[inline]
    fn occluder_floor(extra: &HeightBounds) -> [f32; OCCLUDER_GRID_CELLS] {
        extra.floor_grid
    }

    /// **D1's soundness trap** — the parent's interval, widened by the measured
    /// per-level margin of [`HEIGHT_INHERIT_MARGIN_M`].
    ///
    /// Not the parent's interval unmodified: a coarse DEM smooths a peak away that a
    /// deeper one resolves, so the parent's interval is not a superset of the child's
    /// and copying it is a false negative waiting for the first mountain
    /// (`docs/terrain-plan.md` §7).
    ///
    /// # Not clamped to the global interval
    ///
    /// Down a chain with nothing loaded the margins add up — ≈113 km by z15, over a tile
    /// 1.2 km wide. Capping `hi` at [`GLOBAL_H_MAX_M`] would bound that, and is
    /// deliberately **not** done: `hi` is a real height with `TerrainConfig::exaggeration`
    /// already multiplied in, this is a static dispatch with no access to that factor, and
    /// a cap that is right at exaggeration 1.0 would cut a real summit at 2.0. A loose box
    /// for the frame or two it takes the prefetched ancestor chain to fill is the better
    /// trade: it costs false positives, and the alternative costs the subtree.
    ///
    /// # The extra downward term, and why the corpus does not need re-measuring
    ///
    /// The margin table was measured on box spans whose skirt allowance was the *whole
    /// tile's* height range. **D1's follow-up** replaced that with the tight edge-window
    /// bound ([`HeightTile::edge_window_range`](crate::globe::terrain::HeightTile::edge_window_range)),
    /// which raises the child's `lo` — that helps — *and* the parent's, which does not.
    /// With `allow` for the skirt allowance and `M` for the table's margin, the corpus
    /// measured
    ///
    /// ```text
    ///   child.h_min − allow_whole(child) ≥ parent.h_min − allow_whole(parent) − M
    /// ```
    ///
    /// and what is needed now is the same line with `allow_edge` on both sides.
    /// `allow_edge ≤ allow_whole` gives the child's side away, and
    /// `allow_edge(parent) ≥ sagitta(parent)` gives the parent's for one extra
    /// `range(parent)` of downward slack.
    ///
    /// So the *interval* wants the parent's whole-tile allowance returned, and
    /// [`inherit_allowance_mm`] is an upper bound on it that depends only on the parent's
    /// level. The composition then closes against the numbers already in
    /// [`HEIGHT_INHERIT_MARGIN_M`], rather than against a re-measurement the committed
    /// corpus (whole-tile extrema only) could not support.
    ///
    /// # [`HeightBounds::floor`] does **not** pay that allowance, and this is why
    ///
    /// D3's occluder was widened by the same `w` as `lo` on the argument that the corpus
    /// measures `parent.lo − child.lo` and that turning it into the statement `floor`
    /// needs — `parent.h_min − child.h_min ≤ M` — costs `allow_whole(parent)` back. The
    /// argument is sound and the premise is wrong: `pyramid_extrema.csv` holds
    /// **`h_min_m` and `h_max_m` raw**, one row per tile per level, so the relation
    /// `floor` needs is not a derivation from the interval's — it is a *direct*
    /// measurement on the same corpus, and
    /// `testing::terrain::test_terrain_visibility::d1_floor_inherit_margin_covers_the_corpus`
    /// is it:
    ///
    /// | z | worst `parent.h_min − child.h_min` | [`HEIGHT_INHERIT_MARGIN_M`] | headroom |
    /// |--:|--:|--:|--:|
    /// | 2 | 1 350 m | 20 000 m | 14.8× |
    /// | 8 | 1 517 m | 8 000 m | 5.3× |
    /// | **9** | **1 414 m** | **6 000 m** | **4.2×** |
    /// | 12 | 16 m | 1 500 m | 94× |
    /// | 15 | 2 m | 200 m | 100× |
    ///
    /// The worst level clears the table by 4.2×, against the 4.4× the *interval* relation
    /// clears it by at z3 and z11 — i.e. the floor needs no more margin than the two ends
    /// the table was measured for, and the allowance was never buying soundness on this
    /// side. It was buying **84 km a level**, compounding, and a cell floor is a minimum
    /// over everything stamped into it, so one node on an inherited interval anywhere near
    /// the camera took its whole polar cell to the bottom of the march. That is why
    /// `rendering::terrain_step_capture` read 62 → 62 and 71 → 71 while the counting
    /// harness, whose fetch policy leaves no such node in range, read 80 → 70
    /// (`docs/terrain-plan.md` §7e, "Where the renderer still reads 62 → 62").
    ///
    /// **A relaxation of the occluder bound is the error class that opens holes**, which
    /// is why it is a measurement rather than an argument, and why `floor`'s new value is
    /// still a *widening* — `parent.floor − M`, never `parent.floor`.
    ///
    /// **A level-constant, not `parent.hi − parent.lo`.** The parent's own interval is the
    /// obvious source for its range and it is a trap: down an unloaded chain the widening
    /// would feed on itself, roughly tripling per level, and by z15 the box would be
    /// larger than the solar system and its f32 half-axes would be infinities. A constant
    /// per level cannot compound — a cold z1→z15 chain accumulates a bounded ~1 300 km,
    /// which is loose, sound, numerically ordinary, and gone the frame the prefetched
    /// ancestor chain lands.
    ///
    /// `hi` is untouched: nothing about the skirt moves it, so the table covers it as
    /// measured.
    #[inline]
    fn child_extra(parent: &HeightBounds, child: &TileId) -> HeightBounds {
        let m = inherit_margin_mm(child.z);
        let w = m + inherit_allowance_mm(child);
        // **The floor pays the table and nothing else.** See the section above for why
        // this is not `parent.floor - w`.
        let floor = parent.floor - m;
        HeightBounds {
            lo: parent.lo - w,
            hi: parent.hi + m,
            floor,
            // A parent's sub-cells are not a child's, and the child covers one quadrant of
            // the parent rather than a scaled copy of it. The scalar floor is what is still
            // true everywhere in that quadrant.
            floor_grid: [floor as f32; OCCLUDER_GRID_CELLS],
            // **E1: the level-based formula, and only here.** The parent's *measured*
            // error is the wrong number to inherit — it is the error of the parent's mesh
            // over four times the ground, which is systematically larger than the child's
            // and would compound a demand for refinement down a chain that has no data at
            // all. The level-based fallback halves per level exactly as the imagery term's
            // `unstretched_radius` does, so a cold chain refines at the same rate it always
            // did rather than running away. It is replaced by the measurement the frame the
            // child's own tile lands.
            detail: fallback_detail_mm(child.z) as f32,
        }
    }

    /// **D2** — the scaled-space bounding sphere of the node's box, fitted at
    /// construction where `T`'s linearity makes it free and exact.
    #[inline]
    fn patch_extra(obb: &OrientedBoundingBox) -> ScaledSphere {
        ScaledSphere::around_obb(obb)
    }

    /// **D2** — Theorem 3.7's cone test on that sphere.
    ///
    /// The flat model's exact rectangle supremum is *unsound* here and this is the
    /// whole reason the site dispatches: `q·c ≤ 1` says a point is below the polar
    /// plane, which for a point **on** the ellipsoid means occluded (Theorem 3.5) and
    /// for a point 8 km above it means nothing at all. A summit can satisfy it and be
    /// in plain view over the limb.
    ///
    /// The rectangle is not consulted at all — `patch.max_dot` is never called on this
    /// model. The sphere already contains the rectangle *and* its relief, and combining
    /// the two tests would only re-admit the unsound one.
    #[inline]
    fn is_occluded(patch: &crate::globe::quadtree::TilePatch<Self>, cam: &HorizonCamera) -> bool {
        sphere_is_occluded(cam, &patch.extra)
    }

    /// **D2**, one level finer. See [`SurfaceModel::sub_patch_is_occluded`].
    ///
    /// `a_star` and the φ span are the flat test's inputs and go unread here, which is
    /// ~25 f64 flops per sub-patch column that the terrain path computes and throws
    /// away. Left that way on purpose: hoisting `column_a_star` behind a model-dependent
    /// condition would put a branch on the flat path's hottest loop to save work only
    /// the terrain path does, which is the trade `docs/terrain-plan.md` §1 exists to
    /// refuse.
    #[inline]
    fn sub_patch_is_occluded(
        cam: &HorizonCamera,
        _a_star: f64,
        _sin_lat: &[f64; 2],
        _cos_lat: &[f64; 2],
        extra: &ScaledSphere,
    ) -> bool {
        sphere_is_occluded(cam, extra)
    }
}
