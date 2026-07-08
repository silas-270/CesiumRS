#[cfg(test)]
mod tests {
    use crate::engine::globe::tiles::tile_cache::{TileCacheManager, TileState};
    use crate::engine::globe::quadtree::TileId;
    use std::num::NonZeroUsize;
    use std::time::Duration;
    use std::thread;

    #[test]
    fn test_tile_cache_basic() {
        let capacity = NonZeroUsize::new(2).unwrap();
        let negative_cache = Duration::from_secs(10);
        let mut cache = TileCacheManager::<Vec<u8>>::new(capacity, negative_cache);

        let id1 = TileId { z: 0, x: 0, y: 0 };
        let id2 = TileId { z: 1, x: 0, y: 0 };
        let id3 = TileId { z: 2, x: 0, y: 0 };

        assert!(cache.get_state(&id1).is_none());

        cache.mark_fetching(id1);
        if let Some(TileState::Fetching) = cache.get_state(&id1) {
        } else {
            panic!("Expected Fetching state");
        }

        cache.mark_ready(id1, vec![1, 2, 3]);
        if let Some(TileState::Ready(data)) = cache.get_state(&id1) {
            assert_eq!(data, &vec![1, 2, 3]);
        } else {
            panic!("Expected Ready state");
        }

        // Test LRU eviction
        cache.mark_ready(id2, vec![4, 5, 6]);
        cache.mark_ready(id3, vec![7, 8, 9]);

        // capacity is 2, id1 should be evicted
        assert!(cache.get_state(&id1).is_none());
        assert!(cache.get_state(&id2).is_some());
        assert!(cache.get_state(&id3).is_some());
    }

    #[test]
    fn test_tile_cache_negative_caching() {
        let capacity = NonZeroUsize::new(10).unwrap();
        let negative_cache = Duration::from_millis(50); // short duration for test
        let mut cache = TileCacheManager::<Vec<u8>>::new(capacity, negative_cache);

        let id1 = TileId { z: 0, x: 0, y: 0 };

        cache.mark_failed(id1);
        if let Some(TileState::Failed(_)) = cache.get_state(&id1) {
        } else {
            panic!("Expected Failed state");
        }

        // Wait for negative cache to expire
        thread::sleep(Duration::from_millis(100));

        // Now it should return None because the failed state expired
        assert!(cache.get_state(&id1).is_none());
    }
}
