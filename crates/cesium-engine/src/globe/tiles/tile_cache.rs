use crate::globe::quadtree::TileId;
use lru::LruCache;
use std::num::NonZeroUsize;
use std::time::{Duration, Instant};

pub enum TileState<T> {
    Fetching,
    Ready(T),
    Failed(Instant),
}

pub struct TileCacheManager<T> {
    cache: LruCache<TileId, TileState<T>>,
    negative_cache_duration: Duration,
    /// Tile-trace kind (`tex`, `hgt`); see [`crate::globe::tiles::trace`].
    label: &'static str,
}

fn state_name<T>(s: &TileState<T>) -> &'static str {
    match s {
        TileState::Fetching => "fetching",
        TileState::Ready(_) => "ready",
        TileState::Failed(_) => "failed",
    }
}

impl<T> TileCacheManager<T> {
    pub fn new(capacity: NonZeroUsize, negative_cache_duration: Duration) -> Self {
        Self {
            cache: LruCache::new(capacity),
            negative_cache_duration,
            label: "cache",
        }
    }

    pub fn labeled(mut self, label: &'static str) -> Self {
        self.label = label;
        self
    }

    fn insert(&mut self, id: TileId, state: TileState<T>, event: &str) {
        crate::tile_event!(self.label, event, Some(id));
        if let Some((old, old_state)) = self.cache.push(id, state) {
            if old != id {
                crate::tile_event!(self.label, "EVICT", Some(old), "{}", state_name(&old_state));
            }
        }
    }

    pub fn get_state(&mut self, id: &TileId) -> Option<&TileState<T>> {
        let is_expired_failure = if let Some(state) = self.cache.peek(id) {
            if let TileState::Failed(timestamp) = state {
                timestamp.elapsed() >= self.negative_cache_duration
            } else {
                false
            }
        } else {
            return None;
        };

        if is_expired_failure {
            self.cache.pop(id);
            crate::tile_event!(self.label, "EXPIRE", Some(*id));
            return None;
        }

        // Now we know it's not an expired failure and it exists. Update LRU and return.
        self.cache.get(id)
    }

    /// Like `get_state` but does NOT promote the entry in the LRU order.
    /// Use this for readiness checks / traversal so that the act of checking
    /// does not silently evict other entries.
    pub fn peek_state(&self, id: &TileId) -> Option<&TileState<T>> {
        let state = self.cache.peek(id)?;
        if let TileState::Failed(timestamp) = state {
            if timestamp.elapsed() >= self.negative_cache_duration {
                return None; // expired — treat as absent (will be cleaned next get_state call)
            }
        }
        Some(state)
    }

    pub fn mark_fetching(&mut self, id: TileId) {
        self.insert(id, TileState::Fetching, "REQ");
    }

    pub fn mark_ready(&mut self, id: TileId, data: T) {
        self.insert(id, TileState::Ready(data), "READY");
    }

    pub fn mark_failed(&mut self, id: TileId) {
        self.insert(id, TileState::Failed(Instant::now()), "FAIL");
    }

    /// Drops `id`'s placeholder if it is still `Fetching`, so a later request goes out
    /// again. The counterpart of a cancelled fetch; a `Ready` or `Failed` entry is kept.
    pub fn forget_fetching(&mut self, id: &TileId) {
        if matches!(self.cache.peek(id), Some(TileState::Fetching)) {
            self.cache.pop(id);
            crate::tile_event!(self.label, "CANCEL", Some(*id));
        }
    }

    pub fn resize(&mut self, new_capacity: NonZeroUsize) {
        crate::tile_event!(self.label, "RESIZE", None, "{}", new_capacity);
        while self.cache.len() > new_capacity.get() {
            if let Some((old, old_state)) = self.cache.pop_lru() {
                crate::tile_event!(self.label, "EVICT", Some(old), "{}", state_name(&old_state));
            }
        }
        self.cache.resize(new_capacity);
    }

    /// Entries currently held, `Fetching` and `Failed` placeholders included.
    /// Read-only and non-promoting; exists so the debug panel can report imagery and
    /// height residency separately (`docs/terrain-plan.md` §5 B4).
    pub fn len(&self) -> usize {
        self.cache.len()
    }

    pub fn is_empty(&self) -> bool {
        self.cache.is_empty()
    }

    pub fn has_fetching(&self) -> bool {
        self.cache
            .iter()
            .any(|(_, state)| matches!(state, TileState::Fetching))
    }

    pub fn clear(&mut self) {
        self.cache.clear();
    }
}
