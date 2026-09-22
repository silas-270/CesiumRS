use crate::globe::quadtree::TileId;
use lru::LruCache;
use std::num::NonZeroUsize;

/// LRU of tile meshes with a soft capacity: entries used in this or the previous frame
/// are never evicted; the cache grows past `capacity` instead and shrinks back once the
/// view moves on. Same rule as [`crate::globe::tiles::tile_cache::TileCacheManager`].
pub struct MeshCache<V> {
    lru: LruCache<TileId, (V, u64)>,
    capacity: usize,
    frame: u64,
}

impl<V> MeshCache<V> {
    pub fn new(capacity: NonZeroUsize) -> Self {
        Self {
            lru: LruCache::unbounded(),
            capacity: capacity.get(),
            frame: 0,
        }
    }

    pub fn begin_frame(&mut self) {
        self.frame += 1;
        self.trim();
    }

    fn trim(&mut self) {
        while self.lru.len() > self.capacity {
            match self.lru.peek_lru() {
                Some((_, (_, used))) if self.frame > 0 && *used + 1 >= self.frame => break,
                Some(_) => {
                    if let Some((old, _)) = self.lru.pop_lru() {
                        crate::tile_event!("mesh", "EVICT", Some(old));
                    }
                }
                None => break,
            }
        }
    }

    /// Marks `id` used this frame.
    pub fn get(&mut self, id: &TileId) -> Option<&V> {
        let frame = self.frame;
        self.lru.get_mut(id).map(|e| {
            e.1 = frame;
            &e.0
        })
    }

    pub fn peek(&self, id: &TileId) -> Option<&V> {
        self.lru.peek(id).map(|e| &e.0)
    }

    pub fn put(&mut self, id: TileId, v: V) {
        self.lru.put(id, (v, self.frame));
        self.trim();
    }

    pub fn resize(&mut self, capacity: NonZeroUsize) {
        self.capacity = capacity.get();
        self.trim();
    }

    pub fn clear(&mut self) {
        self.lru.clear();
    }

    pub fn len(&self) -> usize {
        self.lru.len()
    }

    pub fn is_empty(&self) -> bool {
        self.lru.is_empty()
    }

    pub fn cap(&self) -> usize {
        self.capacity
    }
}
