use crate::globe::quadtree::TileId;
use lru::LruCache;
use std::collections::HashMap;
use std::num::NonZeroUsize;
use std::time::{Duration, Instant};

pub enum TileState<T> {
    Fetching,
    Ready(T),
    Failed(Instant),
}

/// Tile data plus the bookkeeping of what is still coming.
///
/// Two stores, on purpose:
///
/// - **`ready`**, an LRU of loaded data with a *soft* capacity. Eviction only takes
///   entries not used in this or the previous frame ([`Self::begin_frame`]); when every
///   entry is in use the cache grows past its capacity instead of evicting something the
///   current view needs, and shrinks back once the view moves on.
/// - **`placeholders`**, `Fetching` / `Failed` markers, outside the LRU. They used to be
///   LRU entries themselves, so a view wanting more tiles than the capacity evicted its
///   own placeholders, re-requested them the next frame and evicted others — 78 837
///   placeholder evictions in a 1 620-frame manual run near the ground.
pub struct TileCacheManager<T> {
    ready: LruCache<TileId, (TileState<T>, u64)>,
    placeholders: HashMap<TileId, TileState<T>>,
    capacity: usize,
    frame: u64,
    negative_cache_duration: Duration,
    /// Tile-trace kind (`tex`, `hgt`); see [`crate::globe::tiles::trace`].
    label: &'static str,
    /// Entries at or above this level (`z <=`) are never evicted.
    pinned_max_z: Option<u8>,
}

impl<T> TileCacheManager<T> {
    pub fn new(capacity: NonZeroUsize, negative_cache_duration: Duration) -> Self {
        Self {
            ready: LruCache::unbounded(),
            placeholders: HashMap::new(),
            capacity: capacity.get(),
            frame: 0,
            negative_cache_duration,
            label: "cache",
            pinned_max_z: None,
        }
    }

    /// Never evict tiles with `z <= max_z`.
    pub fn pin_levels(&mut self, max_z: u8) {
        self.pinned_max_z = Some(max_z);
    }

    pub fn labeled(mut self, label: &'static str) -> Self {
        self.label = label;
        self
    }

    /// Starts a new frame: entries used from now on, or in the frame that just ended,
    /// are protected from eviction.
    pub fn begin_frame(&mut self) {
        self.frame += 1;
        self.trim();
    }

    /// Evicts least-recently-used entries until back at capacity, stopping at the first
    /// one that is still in use.
    fn trim(&mut self) {
        let mut pinned_seen = 0;
        while self.ready.len() > self.capacity && pinned_seen < self.ready.len() {
            match self.ready.peek_lru() {
                Some((_, (_, used))) if self.frame > 0 && *used + 1 >= self.frame => break,
                Some((id, _)) if self.pinned_max_z.is_some_and(|z| id.z <= z) => {
                    let id = *id;
                    self.ready.promote(&id);
                    pinned_seen += 1;
                }
                Some(_) => {
                    if let Some((old, _)) = self.ready.pop_lru() {
                        crate::tile_event!(self.label, "EVICT", Some(old), "ready");
                    }
                }
                None => break,
            }
        }
    }

    fn is_expired(&self, state: &TileState<T>) -> bool {
        matches!(state, TileState::Failed(t) if t.elapsed() >= self.negative_cache_duration)
    }

    /// The state of `id`, marking it as used this frame (and most recently used).
    pub fn get_state(&mut self, id: &TileId) -> Option<&TileState<T>> {
        let frame = self.frame;
        if self.ready.contains(id) {
            return self.ready.get_mut(id).map(|entry| {
                entry.1 = frame;
                &entry.0
            });
        }
        if self.placeholders.get(id).is_some_and(|s| self.is_expired(s)) {
            self.placeholders.remove(id);
            crate::tile_event!(self.label, "EXPIRE", Some(*id));
            return None;
        }
        self.placeholders.get(id)
    }

    /// Like `get_state` but does NOT promote the entry or mark it used.
    /// Use this for readiness checks / traversal so that the act of checking
    /// does not keep anything alive.
    pub fn peek_state(&self, id: &TileId) -> Option<&TileState<T>> {
        if let Some((state, _)) = self.ready.peek(id) {
            return Some(state);
        }
        let state = self.placeholders.get(id)?;
        if self.is_expired(state) {
            return None; // expired — treat as absent (cleaned by the next get_state)
        }
        Some(state)
    }

    pub fn mark_fetching(&mut self, id: TileId) {
        crate::tile_event!(self.label, "REQ", Some(id));
        self.placeholders.insert(id, TileState::Fetching);
    }

    pub fn mark_ready(&mut self, id: TileId, data: T) {
        crate::tile_event!(self.label, "READY", Some(id));
        self.placeholders.remove(&id);
        self.ready.put(id, (TileState::Ready(data), self.frame));
        self.trim();
    }

    pub fn mark_failed(&mut self, id: TileId) {
        crate::tile_event!(self.label, "FAIL", Some(id));
        if self.ready.pop(&id).is_some() {
            crate::tile_event!(self.label, "EVICT", Some(id), "ready");
        }
        let now = Instant::now();
        let ttl = self.negative_cache_duration;
        self.placeholders
            .retain(|_, s| !matches!(s, TileState::Failed(t) if now.duration_since(*t) >= ttl));
        self.placeholders.insert(id, TileState::Failed(now));
    }

    /// Drops `id`'s placeholder if it is still `Fetching`, so a later request goes out
    /// again. The counterpart of a cancelled fetch; a `Ready` or `Failed` entry is kept.
    pub fn forget_fetching(&mut self, id: &TileId) {
        if matches!(self.placeholders.get(id), Some(TileState::Fetching)) {
            self.placeholders.remove(id);
            crate::tile_event!(self.label, "CANCEL", Some(*id));
        }
    }

    pub fn resize(&mut self, new_capacity: NonZeroUsize) {
        crate::tile_event!(self.label, "RESIZE", None, "{}", new_capacity);
        self.capacity = new_capacity.get();
        self.trim();
    }

    /// Tiles with data. Placeholders are not counted.
    pub fn len(&self) -> usize {
        self.ready.len()
    }

    pub fn is_empty(&self) -> bool {
        self.ready.is_empty()
    }

    pub fn has_fetching(&self) -> bool {
        self.placeholders
            .values()
            .any(|state| matches!(state, TileState::Fetching))
    }

    pub fn clear(&mut self) {
        self.ready.clear();
        self.placeholders.clear();
    }
}
