use lru::LruCache;
use std::num::NonZeroUsize;
use std::time::{Duration, Instant};
use crate::globe::quadtree::TileId;

pub enum TileState<T> {
    Fetching,
    Ready(T),
    Failed(Instant),
}

pub struct TileCacheManager<T> {
    cache: LruCache<TileId, TileState<T>>,
    negative_cache_duration: Duration,
}

impl<T> TileCacheManager<T> {
    pub fn new(capacity: NonZeroUsize, negative_cache_duration: Duration) -> Self {
        Self {
            cache: LruCache::new(capacity),
            negative_cache_duration,
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
            return None;
        }

        // Now we know it's not an expired failure and it exists. Update LRU and return.
        self.cache.get(id).map(|s| &*s)
    }

    pub fn mark_fetching(&mut self, id: TileId) {
        self.cache.put(id, TileState::Fetching);
    }

    pub fn mark_ready(&mut self, id: TileId, data: T) {
        self.cache.put(id, TileState::Ready(data));
    }

    pub fn mark_failed(&mut self, id: TileId) {
        self.cache.put(id, TileState::Failed(Instant::now()));
    }

    pub fn resize(&mut self, new_capacity: NonZeroUsize) {
        self.cache.resize(new_capacity);
    }
}
