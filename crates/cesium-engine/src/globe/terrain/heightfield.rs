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
//! heights), and Phase D will read [`HeightPatch::height_bounds`] rather than the
//! source tile's `h_min`/`h_max`, so the boxes and spheres of §3.1/§3.2 inherit the
//! exaggeration automatically instead of having to remember it. One multiplication,
//! upstream of every bound derived from it.

use crate::globe::geometry::{lon_lat_to_ecef_f64, EARTH_RADIUS_A_F64};
use crate::globe::quadtree::surface::{SurfaceModel, VertexSample};
use crate::globe::quadtree::{tile_bounds, web_mercator_y_to_lat_f64, TileId};
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

/// The globe with relief: the ellipsoid displaced radially by a sampled height field.
///
/// The node and patch payloads are still `()` in this phase — **Phase C changes the
/// geometry, not the culling**. `docs/terrain-plan.md` §10 says so explicitly: "C
/// alone, with terrain on, is unsound", and D1/D2 are where
/// [`SurfaceModel::obb_altitude_span`] and [`SurfaceModel::is_occluded`] get their
/// terrain forms. Giving them terrain bounds here without the cone test of §3.2 would
/// mix a half-done culling change into a geometry change and make neither reviewable.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Heightfield;

impl SurfaceModel for Heightfield {
    // Phase D's payloads. Deliberately still `()`: see the type's doc comment.
    type NodeExtra = ();
    type PatchExtra = ();
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

    /// **Phase D.** Still the flat span, and deliberately so — see the type's doc.
    #[inline]
    fn obb_altitude_span(_extra: &()) -> (f64, f64) {
        (0.0, 0.0)
    }

    /// **Phase D.** Still the flat-mode rectangle test, which relief makes *unsound*
    /// (`docs/terrain-plan.md` §3.2: a summit can satisfy `q·c ≤ 1` and still be
    /// visible over the limb). Known, scoped to D2, and the reason terrain stays
    /// `enabled: false` by default at the end of Phase C.
    #[inline]
    fn is_occluded(
        patch: &crate::globe::quadtree::TilePatch<Self>,
        cam: &crate::globe::quadtree::HorizonCamera,
    ) -> bool {
        crate::globe::quadtree::horizon::span_is_occluded(cam, patch.max_dot(cam))
    }
}
