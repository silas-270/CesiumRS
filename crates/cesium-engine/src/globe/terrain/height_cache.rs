//! Fetching, caching and querying height tiles — B1, B2 and B4 of
//! `docs/terrain-plan.md` §5.
//!
//! Nothing here renders. The manager below owns a [`TileFetcher`] pointed at the
//! Terrarium source, a [`TileCacheManager`] of decoded [`HeightTile`]s, and the one
//! query the rest of the engine will ever ask it: [`HeightTileManager::height_at`].
//!
//! # Units
//!
//! [`HeightTile`] is metres. [`HeightTileManager::height_at`] returns
//! **megametres**, the unit `SurfaceModel` and `EARTH_RADIUS_A_F64` are in
//! (`quadtree/surface.rs`'s module doc). This module is the only place the factor
//! appears, so there is exactly one line to get wrong and it is under test.

use std::num::NonZeroUsize;
use std::sync::Arc;

use tokio::sync::mpsc;

use crate::globe::quadtree::TileId;
use crate::globe::terrain::height_tile::{decode_terrarium, HeightTile};
use crate::globe::terrain::heightfield::{
    skirt_allowance, HeightBounds, PatchStatus, OCCLUDER_GRID, OCCLUDER_GRID_CELLS,
};
use crate::globe::tiles::config::{
    tile_cache_entries_for, OceanPolicy, TileEngineConfig, HEIGHT_TILE_BYTES,
};
use crate::globe::tiles::system::TileSystem;
use crate::globe::tiles::tile_cache::{TileCacheManager, TileState};
use crate::globe::tiles::tile_fetcher::{TileFetcher, TileImage, TilePriority};

/// Metres to megametres. The engine's world unit is the megametre; the DEM's is the
/// metre. Converting in the wrong direction here puts a mountain 10^6 times too high
/// — the exact trap `quadtree/surface.rs` warns about.
const METRES_TO_MEGAMETRES: f64 = 1.0e-6;

/// Hard cap on resident height tiles, independent of the byte budget, mirroring
/// [`TileEngineConfig::max_cache_size`]'s role for imagery. Height tiles are a fixed
/// size so the byte budget is always the binding constraint in practice; this is only
/// a guard against a pathological budget.
const MAX_HEIGHT_CACHE_ENTRIES: usize = 4096;

/// The latitude Web Mercator stops at, `atan(sinh(π))` in degrees — the value that makes
/// the projection square. Beyond it there is no tile row and therefore no DEM sample; see
/// [`HeightTileManager::peek_height_at_lon_lat`]'s "Poles" note.
const MERCATOR_LAT_LIMIT_DEG: f64 = 85.051_128_779_806_59;

/// Owns the height tiles: fetch, decode, cache, query.
///
/// Constructed **only** when [`crate::globe::tiles::config::TerrainConfig::enabled`]
/// is set. While terrain is off this type is never instantiated, so no second tokio
/// runtime, no second cache and no height request exists — that is what keeps the flat
/// path unchanged rather than merely unaffected.
pub struct HeightTileManager {
    /// Decoded tiles, `Arc` so a query can hand one out without copying 128 kB.
    /// `Fetching` / `Failed(Instant)` and the negative-cache expiry come with
    /// [`TileCacheManager`] for free.
    pub cache: TileCacheManager<Arc<HeightTile>>,
    rx: mpsc::UnboundedReceiver<(TileId, Result<TileImage, String>)>,
    fetcher: TileFetcher,
    ocean: OceanPolicy,
    max_level: u8,
    /// No network: every request resolves immediately to a flat zero field. See
    /// [`Self::request_tile`].
    offline_mode: bool,
    /// Entries the byte budget allows — reported by the debug panel, so terrain's
    /// share of the tile budget is visible rather than inferred.
    capacity: NonZeroUsize,
}

impl HeightTileManager {
    pub fn new(config: &TileEngineConfig) -> Self {
        let (tx, rx) = mpsc::unbounded_channel();

        // B4: entries derived from the *declared slice* of the tile budget, by the same
        // arithmetic that sizes the imagery cache. Height tiles never change size, so
        // unlike imagery this is derived once and stays put.
        let capacity = tile_cache_entries_for(
            config.terrain.height_cache_budget_bytes,
            HEIGHT_TILE_BYTES,
            NonZeroUsize::new(MAX_HEIGHT_CACHE_ENTRIES).expect("non-zero"),
        );

        log::info!(
            "Height cache: {}KiB per tile, {}MB budget -> {} entries (source {}, max level {})",
            HEIGHT_TILE_BYTES / 1024,
            config.terrain.height_cache_budget_bytes / (1024 * 1024),
            capacity.get(),
            config.terrain.source_url,
            config.terrain.max_level,
        );

        Self {
            cache: TileCacheManager::new(capacity, config.negative_cache_duration),
            rx,
            fetcher: TileFetcher::new(
                tx,
                config.terrain.source_url.clone(),
                // Deliberately false: `TileFetcher`'s own offline stub returns an
                // all-`255` RGBA image, which under the Terrarium encoding decodes to
                // +32768 m — a globe-wide wall, not a flat field. Offline is handled in
                // `request_tile` instead, where the zero field can be stated directly.
                false,
            ),
            ocean: config.terrain.ocean,
            max_level: config.terrain.max_level,
            offline_mode: config.offline_mode,
            capacity,
        }
    }

    /// The tile actually fetched for `id`: `id` itself, or its ancestor at
    /// [`max_level`](Self::max_level) when `id` is deeper than the source goes.
    ///
    /// Not an error path. The source stops at z15 and imagery refines to z19/z20, so
    /// for a quarter of the level range this redirect is the *only* thing that happens
    /// (`docs/terrain-plan.md` §2).
    pub fn source_tile_for(&self, id: TileId) -> TileId {
        clamp_to_level(id, self.max_level)
    }

    /// Queues `id`'s height tile if it is not already known.
    ///
    /// Visible tiles come in at [`TilePriority::High`] — a missing texture is a blur,
    /// a missing height tile is the wrong shape, so heights must not queue behind
    /// prefetched imagery.
    pub fn request_tile(&mut self, id: TileId, priority: TilePriority) {
        let src = self.source_tile_for(id);
        if self.cache.get_state(&src).is_some() {
            return;
        }

        if self.offline_mode {
            // A flat zero field, resolved synchronously: no network, no fetcher, and a
            // globe whose geometry is identical to flat mode, so every existing headless
            // test runs unchanged with terrain switched on.
            self.cache
                .mark_ready(src, Arc::new(HeightTile::flat_zero()));
            return;
        }

        self.cache.mark_fetching(src);
        self.fetcher.request_tile(src, priority);
    }

    /// Drains finished fetches and decodes them.
    ///
    /// Decoding runs here, on the caller's thread, rather than in the fetch worker:
    /// `TileFetcher` is shared verbatim with imagery and hands back RGBA. 65 536 texels
    /// of integer arithmetic plus the mip is tens of microseconds, and it happens at
    /// most a handful of times per frame.
    pub fn update(&mut self) {
        while let Ok((id, result)) = self.rx.try_recv() {
            // Same guard as the texture manager's: if the entry was evicted while the
            // fetch was in flight, the result is stale and re-inserting it would
            // resurrect a tile nothing asked for.
            if !matches!(self.cache.get_state(&id), Some(TileState::Fetching)) {
                continue;
            }

            match result.and_then(|(w, h, rgba)| decode_terrarium(w, h, &rgba, self.ocean)) {
                Ok(tile) => self.cache.mark_ready(id, Arc::new(tile)),
                Err(e) => {
                    log::warn!(
                        "Failed to load height tile z:{} x:{} y:{}: {}",
                        id.z,
                        id.x,
                        id.y,
                        e
                    );
                    self.cache.mark_failed(id);
                }
            }
        }
    }

    /// Height above the ellipsoid at tile-local `(u, v)` of `id`, in **megametres**,
    /// or `None` when no ancestor's data has arrived yet.
    ///
    /// # `None` is not zero
    ///
    /// "Unknown" has to be distinguishable from "sea level" (`docs/terrain-plan.md` §5
    /// B2): a caller that reads an unknown as `0.0` bakes a flat tile into a mesh and
    /// then has no reason to rebuild it when the real data lands. Phase C must skip the
    /// tile, not flatten it.
    ///
    /// # Ancestor upsampling is the normal path
    ///
    /// The source stops at z15. Every tile from z16 to z20 is answered from its z15
    /// ancestor, and the transform that does it is
    /// [`TileSystem::compute_fallback_uv`] — the same parent walk imagery already uses
    /// for fallback textures, accumulating `scale *= 0.5` and a quadrant offset. Run
    /// against the height cache, that walk *is* Cesium's `upsample()`: same result, as
    /// a UV transform instead of a resample, so it costs no extra memory and no
    /// resampling pass.
    ///
    /// The returned scale/offset are exact dyadic rationals with at most
    /// [`crate::globe::quadtree::MAX_ZOOM`] = 20 fractional bits, so the `f32` they come
    /// back in is exact and widening to `f64` here loses nothing.
    ///
    /// Promotes the tile it reads in the LRU, so the ancestor a deep tile depends on
    /// cannot be evicted by the very traversal that is using it.
    pub fn height_at(&mut self, id: TileId, u: f64, v: f64) -> Option<f64> {
        let src = self.resolve_source(id)?;
        let (su, sv) = Self::ancestor_uv(id, src, u, v);
        // Promote, then read. `get_state` is what keeps the ancestor alive; `peek_state`
        // deliberately would not (`tile_cache.rs:45-47`).
        match self.cache.get_state(&src)? {
            TileState::Ready(tile) => Some(tile.sample_bilinear(su, sv) * METRES_TO_MEGAMETRES),
            _ => None,
        }
    }

    /// [`Self::height_at`] without the LRU promotion — for readiness checks, debug
    /// readouts and tests, where the act of looking must not evict anything.
    pub fn peek_height_at(&self, id: TileId, u: f64, v: f64) -> Option<f64> {
        let src = self.resolve_source(id)?;
        let (su, sv) = Self::ancestor_uv(id, src, u, v);
        match self.cache.peek_state(&src)? {
            TileState::Ready(tile) => Some(tile.sample_bilinear(su, sv) * METRES_TO_MEGAMETRES),
            _ => None,
        }
    }

    /// Terrain height in **megametres** under a geodetic position, from whatever height
    /// data is already resident — `None` when nothing in the ancestor chain has arrived.
    ///
    /// The geodetic entry point, for the parts of the engine that know where they are on
    /// the globe rather than which tile they are in: the camera's ground clearance
    /// (`Camera::altitude_agl`), its collision floor, and label placement. It is
    /// [`Self::peek_height_at`] with the Web-Mercator forward map in front of it, so it
    /// promotes nothing in the LRU and can be called from a `&self` frame path.
    ///
    /// **Raw height — [`crate::globe::tiles::config::TerrainConfig::exaggeration`] is
    /// *not* applied**, exactly as [`Self::height_at`] leaves it off. The caller that
    /// wants the height of the surface the renderer actually draws multiplies it in;
    /// [`crate::globe::tiles::system::TileSystem::ground_height_at`] is the one that does.
    ///
    /// # Resolution follows what has landed, and that is the point
    ///
    /// The query starts at [`Self::max_level`] (z15 for Terrarium) and
    /// [`Self::resolve_source`] walks up to the deepest ancestor that is ready. At
    /// cruise altitude that is a z4-z6 tile and the answer is a continent-scale average;
    /// on final approach the z15 tile under the aircraft is resident because it is being
    /// drawn, and the answer is the real valley floor. Both are the best available, and
    /// neither needs a fetch of its own — this query never enqueues anything, so it
    /// cannot make the camera path compete with the tiles being drawn.
    ///
    /// # Poles
    ///
    /// Latitude is clamped to the Web-Mercator limit (±85.051 13°). [`tile_bounds`]
    /// stretches the top and bottom tile *rows* to ±90° to cap the globe, but that
    /// stretch is a property of the drawn rectangle, not of the DEM inside it; there is
    /// no sample beyond the Mercator limit to return. The clamp yields the polar row's
    /// edge height, which over the Arctic ocean and the Antarctic coast is the right
    /// answer to within the relief this query is used to resolve.
    ///
    /// [`tile_bounds`]: crate::globe::quadtree::tile_bounds
    pub fn peek_height_at_lon_lat(&self, lon_deg: f64, lat_deg: f64) -> Option<f64> {
        let (id, u, v) = Self::tile_uv_at_lon_lat(lon_deg, lat_deg, self.max_level);
        self.peek_height_at(id, u, v)
    }

    /// The tile of level `z` containing `(lon, lat)`, and the position inside it.
    ///
    /// The inverse of the map `HeightPatch::grid_metrics` and `TileMesh::generate`
    /// build their rows from: `lat = web_mercator_y_to_lat_f64(y + v, z)`, `lon`
    /// linear in `x + u`. Written as the inverse of *that* expression and not of a
    /// textbook Web-Mercator formula, so a sample lands on the same ground the mesh
    /// puts there — invariant I-5's concern, one level down.
    ///
    /// `u`, `v` come back in `[0, 1]`, `v` measured downward from the tile's north
    /// edge, which is [`Self::peek_height_at`]'s convention.
    pub fn tile_uv_at_lon_lat(lon_deg: f64, lat_deg: f64, z: u8) -> (TileId, f64, f64) {
        let n = (1_u64 << z) as f64;

        let fx = ((lon_deg + 180.0) / 360.0 * n).clamp(0.0, n - f64::EPSILON);
        // `web_mercator_y_to_lat_f64` is `atan(sinh(π(1 − 2y/n)))`; inverted,
        // `y = n/2 · (1 − asinh(tan φ)/π)`. `asinh` is the exact inverse of the `sinh`
        // that function applies, so the round trip is an identity to within an ulp.
        let lat = lat_deg.clamp(-MERCATOR_LAT_LIMIT_DEG, MERCATOR_LAT_LIMIT_DEG);
        let fy = 0.5 * n * (1.0 - lat.to_radians().tan().asinh() / std::f64::consts::PI);
        let fy = fy.clamp(0.0, n - f64::EPSILON);

        let (x, y) = (fx.floor(), fy.floor());
        (
            TileId {
                z,
                x: x as u32,
                y: y as u32,
            },
            fx - x,
            fy - y,
        )
    }

    /// The tile that answers for `id`: the deepest ready ancestor (or `id` itself),
    /// starting from the source's depth ceiling.
    pub fn resolve_source(&self, id: TileId) -> Option<TileId> {
        let mut curr = self.source_tile_for(id);
        loop {
            if matches!(self.cache.peek_state(&curr), Some(TileState::Ready(_))) {
                return Some(curr);
            }
            curr = curr.parent()?;
        }
    }

    /// Maps `(u, v)` in `child`'s tile space into `ancestor`'s.
    ///
    /// Split out from [`Self::height_at`] so the quadrant accumulation — the one piece
    /// of B2 whose off-by-one is invisible except as a landscape displaced by hundreds
    /// of metres — is testable on its own.
    pub fn ancestor_uv(child: TileId, ancestor: TileId, u: f64, v: f64) -> (f64, f64) {
        Self::ancestor_uv_unclamped(child, ancestor, u.clamp(0.0, 1.0), v.clamp(0.0, 1.0))
    }

    /// [`Self::ancestor_uv`] with the `[0,1]` clamp on its **input** left off.
    ///
    /// The map is affine, so it is perfectly well defined outside the child's own
    /// rectangle, and Phase C's mesh patch needs exactly that: its gradient halo asks
    /// for the height one grid step *outside* the tile, which is inside the ancestor
    /// whenever the ancestor is a real ancestor. Clamping the input would collapse the
    /// halo onto the tile edge and turn every edge normal into a half-slope.
    ///
    /// The *output* is not clamped either. `HeightTile::sample_bilinear` clamps into
    /// its own tile, which is the right degradation: a halo that falls outside the
    /// source reads the source's border sample, and the caller
    /// (`HeightPatch::halo_valid`) knows it happened.
    pub fn ancestor_uv_unclamped(child: TileId, ancestor: TileId, u: f64, v: f64) -> (f64, f64) {
        if child == ancestor {
            return (u, v);
        }
        let [scale_x, scale_y, offset_x, offset_y] =
            TileSystem::compute_fallback_uv(child, ancestor);
        (
            offset_x as f64 + u * scale_x as f64,
            offset_y as f64 + v * scale_y as f64,
        )
    }

    /// Whether `id`'s heights are usable yet, and if not, whether waiting will help.
    ///
    /// The three-way answer is what lets Phase C's mesh builder honour §5 B2's
    /// "unknown is not sea level": [`PatchStatus::Pending`] means retry next frame,
    /// [`PatchStatus::Unavailable`] means the whole ancestor chain has failed and a
    /// flat mesh is the honest answer. Only the second is a terminating condition, so
    /// a stalled fetch can never be mistaken for flat ground.
    ///
    /// Non-promoting, like every readiness check in this engine.
    /// # Why an already-loaded ancestor is not good enough
    ///
    /// [`Self::resolve_source`] answers with the *deepest ready* ancestor, which is the
    /// right answer for a query. It is the wrong answer for a **mesh**, because a mesh
    /// is built once and Phase E2 — the rebuild-on-better-data pass — does not exist
    /// yet. `TileSystem::update` prefetches the whole ancestor chain at `Low`, so a
    /// coarse ancestor routinely lands before the tile's own height tile does; building
    /// from it bakes a smoothed, hundreds-of-metres-too-low surface into the cache with
    /// nothing to correct it. Measured, not reasoned: the first Phase C capture over
    /// the Alps at 4.5 km had its whole foreground flattened this way.
    ///
    /// So the answer is `Ready` only once `source_tile_for(id)` — the deepest level the
    /// source actually serves for this tile — has arrived, or has **failed**, in which
    /// case the best available ancestor is genuinely the best there will ever be. While
    /// it is still in flight the tile is `Pending` and the engine draws the parent's
    /// mesh, exactly as it already draws the parent's texture: coarser geometry, not a
    /// hole, and not a wrong shape that sticks.
    ///
    /// This is **not** E2. It removes the common case that would need a rebuild; a
    /// tile that falls back to an ancestor because its own fetch failed still wants one,
    /// which is why the mesh records [`crate::globe::geometry::TileMesh::height_source`].
    pub fn status_of(&self, id: TileId) -> PatchStatus {
        let mut curr = self.source_tile_for(id);
        loop {
            match self.cache.peek_state(&curr) {
                Some(TileState::Ready(_)) => return PatchStatus::Ready,
                // Fetching, or never requested at all: something better is still coming.
                None | Some(TileState::Fetching) => return PatchStatus::Pending,
                // Failed: this level will not answer. Fall back to the parent.
                _ => {}
            }
            match curr.parent() {
                Some(p) => curr = p,
                None => return PatchStatus::Unavailable,
            }
        }
    }

    /// The altitude interval `id`'s bounding volumes must be fitted over — **Phase
    /// D1's feed**, and the only thing the quadtree ever asks the height cache.
    ///
    /// `None` means "nothing better than inheritance is known yet", and the node keeps
    /// the widened interval it got from its parent ([`Heightfield::child_extra`]).
    ///
    /// # Why the readiness test is `status_of` and not `resolve_source`
    ///
    /// The two disagree exactly when a coarse ancestor has landed and the tile's own
    /// height tile is still in flight, and that gap is a false-negative source. In that
    /// window `resolve_source` would answer with the ancestor, whose extrema over this
    /// tile's ground are a *smoothed* version of the truth and can be hundreds of metres
    /// too low; the mesh, meanwhile, is deferred by [`Self::status_of`] until the tile's
    /// own data arrives. Bounds taken from the ancestor and geometry built from the tile
    /// is precisely the pairing that puts a summit outside its own box. Using the same
    /// predicate as the mesh builder keeps the two in lockstep: while the answer here is
    /// `None` no mesh exists either, and the frame the mesh becomes buildable is the
    /// frame this starts answering — from the same tile.
    ///
    /// # Why the mip, and why whole cells
    ///
    /// Past the source's deepest level (z15 for Terrarium) `id` is answered by an
    /// ancestor and covers only a sub-rectangle of it, so the ancestor's whole-tile
    /// extrema would be wildly loose — a z20 tile is 1/32 768 of its z15 source by area.
    /// B3's 16×16 min/max mip is exactly the structure for that, and rounding the
    /// sub-rectangle *outward* to whole mip cells keeps the answer an upper bound on the
    /// samples the mesh will bilinearly interpolate, which is the direction I-6 needs.
    ///
    /// It also makes the margin of [`HEIGHT_INHERIT_MARGIN_M`] exactly zero below z15:
    /// child and parent read the same tile, and the child's rectangle is a **dyadic
    /// sub-rectangle** of the parent's, so its covering cell set is a subset of the
    /// parent's and its extrema are contained by construction.
    ///
    /// # The skirt
    ///
    /// `lo` is the lowest sample minus [`skirt_allowance`], because the mesh's skirt
    /// hangs below the field and a skirt vertex outside the box is as much a false
    /// negative as a summit outside it.
    ///
    /// [`Heightfield::child_extra`]: crate::globe::terrain::heightfield::Heightfield
    /// [`HEIGHT_INHERIT_MARGIN_M`]: crate::globe::terrain::heightfield::skirt_allowance
    pub fn height_bounds_for(
        &self,
        id: TileId,
        segments: u32,
        exaggeration: f32,
    ) -> Option<HeightBounds> {
        if self.status_of(id) != PatchStatus::Ready {
            return None;
        }
        let src = self.resolve_source(id)?;
        let tile = match self.cache.peek_state(&src)? {
            TileState::Ready(tile) => tile,
            _ => return None,
        };

        // `id`'s own [0,1]² mapped into the source tile. Affine, so the corners settle
        // the whole rectangle.
        let (u0, v0) = Self::ancestor_uv(id, src, 0.0, 0.0);
        let (u1, v1) = Self::ancestor_uv(id, src, 1.0, 1.0);
        let (h_min_m, h_max_m) = tile.mip_extrema_over(u0, v0, u1, v1);
        // D1's follow-up: the skirt is an *edge* property, so it is bounded from the
        // edges. See `HeightTile::edge_window_range` and the "The skirt is in the box"
        // note in `docs/terrain-plan.md` §7 for the 1.53× this replaces.
        let edge_range_m = tile.edge_window_range(u0, v0, u1, v1) as f64;

        let exaggeration = exaggeration as f64;
        let lo_m = h_min_m as f64 * METRES_TO_MEGAMETRES * exaggeration;
        let hi_m = h_max_m as f64 * METRES_TO_MEGAMETRES * exaggeration;
        let edge_range = edge_range_m * METRES_TO_MEGAMETRES * exaggeration;
        // D3's occluder, per sub-cell. Each entry is the minimum over its own
        // sub-rectangle of the same mip, which is what keeps a ridge from being averaged
        // away against the ground on the far side of the tile — see
        // `HeightBounds::floor_grid` for the measurement that made this a grid.
        let mut floor_grid = [0.0f32; OCCLUDER_GRID_CELLS];
        let n = OCCLUDER_GRID as f64;
        for j in 0..OCCLUDER_GRID {
            let (a, b) = (j as f64 / n, (j + 1) as f64 / n);
            let (sv0, sv1) = (v0 + (v1 - v0) * a, v0 + (v1 - v0) * b);
            for i in 0..OCCLUDER_GRID {
                let (c, d) = (i as f64 / n, (i + 1) as f64 / n);
                let (su0, su1) = (u0 + (u1 - u0) * c, u0 + (u1 - u0) * d);
                let (cell_min, _) = tile.mip_extrema_over(su0, sv0, su1, sv1);
                floor_grid[j * OCCLUDER_GRID + i] =
                    (cell_min as f64 * METRES_TO_MEGAMETRES * exaggeration) as f32;
            }
        }

        Some(HeightBounds {
            lo: lo_m - skirt_allowance(id, segments, edge_range),
            hi: hi_m,
            // D3's occluder: the ground's own minimum over this tile, *without* the
            // skirt allowance. See [`HeightBounds::floor`].
            floor: lo_m,
            floor_grid,
            // **E1** — the measured geometric error, read off the tile that answers for
            // `id`. Whole-tile, not per sub-rectangle, and that is exact rather than
            // approximate wherever it matters: `status_of` above only answers `Ready`
            // once `source_tile_for(id)` itself has landed, so below the source's ceiling
            // `src == id` and this is `id`'s own error. Past the ceiling, and in the rare
            // case where `id`'s own tile 404s and an ancestor answers, it is the
            // ancestor's error over four or more times the ground — an over-statement,
            // which refines early and costs tiles rather than shape, and which
            // `Heightfield::geometric_error` clamps to zero past the ceiling anyway.
            detail: (tile.detail() as f64 * METRES_TO_MEGAMETRES * exaggeration) as f32,
        })
    }

    /// The decoded tile that answers for `id`, together with its id, promoted in the
    /// LRU so the ancestor a mesh is being built from cannot be evicted by the build.
    ///
    /// Phase C samples a whole `(segments+3)²` grid at once; resolving the source once
    /// and sampling the `Arc` directly is what keeps that from being 361 walks up the
    /// ancestor chain.
    pub fn source_for(&mut self, id: TileId) -> Option<(TileId, Arc<HeightTile>)> {
        let src = self.resolve_source(id)?;
        match self.cache.get_state(&src)? {
            TileState::Ready(tile) => Some((src, Arc::clone(tile))),
            _ => None,
        }
    }

    /// Tiles currently held, and the byte-budget-derived ceiling on them. Reported
    /// separately from imagery in the debug panel, per §5 B4.
    pub fn residency(&self) -> (usize, usize) {
        (self.cache.len(), self.capacity.get())
    }

    /// Bytes currently resident in height tiles.
    pub fn resident_bytes(&self) -> usize {
        self.cache.len() * HEIGHT_TILE_BYTES
    }

    pub fn is_loading_complete(&self) -> bool {
        !self.cache.has_fetching()
    }

    pub fn clear(&mut self) {
        self.cache.clear();
        while self.rx.try_recv().is_ok() {}
    }

    /// Inserts a tile directly, bypassing the fetcher. The seam the unit tests use to
    /// drive the query path against fixture data without touching the network.
    pub fn insert_ready(&mut self, id: TileId, tile: Arc<HeightTile>) {
        self.cache.mark_ready(id, tile);
    }

    /// Marks a tile as having failed, bypassing the fetcher — [`Self::insert_ready`]'s
    /// counterpart, and the seam **Phase E2** needs.
    ///
    /// A failed own-level fetch is the one state in which a mesh gets built from an
    /// ancestor and then wants rebuilding later (see
    /// [`crate::globe::tiles::system::fresher_height_source`]). It cannot be reached
    /// through [`Self::insert_ready`], and reaching it through the network would make
    /// the test depend on a 404 the source does not reliably serve, so it is reachable
    /// here instead.
    pub fn insert_failed(&mut self, id: TileId) {
        self.cache.mark_failed(id);
    }
}

/// `id`'s ancestor at `level`, or `id` when it is already at or above it.
fn clamp_to_level(id: TileId, level: u8) -> TileId {
    if id.z <= level {
        return id;
    }
    let drop = id.z - level;
    TileId {
        z: level,
        x: id.x >> drop,
        y: id.y >> drop,
    }
}
