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
        let (source, tile) = match heights.status_of(id) {
            PatchStatus::Ready => heights.source_for(id).ok_or(PatchStatus::Pending)?,
            other => return Err(other),
        };

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
}

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
}

impl NodeExtraSource<Heightfield> for HeightBoundsSource<'_> {
    #[inline]
    fn extra_for(&self, id: &TileId) -> Option<HeightBounds> {
        self.heights
            .height_bounds_for(*id, self.segments, self.exaggeration)
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
    #[inline]
    fn child_extra(parent: &HeightBounds, child: &TileId) -> HeightBounds {
        parent.widened(inherit_margin_mm(child.z))
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
