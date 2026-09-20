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
use crate::globe::terrain::heightfield::PatchStatus;
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
