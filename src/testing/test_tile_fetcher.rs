#[cfg(test)]
mod tests {
    use crate::io::tile_fetcher::{TilePriority, TileFetcher};
    use crate::globe::quadtree::TileId;
    use std::sync::mpsc;
    use std::time::Duration;

    #[test]
    fn test_priority_ordering() {
        // TilePriority High > Low
        assert!(TilePriority::High > TilePriority::Low);
    }

    // Since TileFetcher actually makes network requests,
    // we'll just test that it can fetch a known tile,
    // and that invalid URLs correctly return errors.
    #[test]
    fn test_fetch_valid_tile() {
        let (tx, rx) = mpsc::channel();
        let fetcher = TileFetcher::new(tx, "https://tile.openstreetmap.org/{z}/{x}/{y}.png".to_string());
        
        let valid_tile = TileId { z: 0, x: 0, y: 0 };
        fetcher.request_tile(valid_tile, TilePriority::High);
        
        // It might take a moment to fetch
        let (id, result) = rx.recv_timeout(Duration::from_secs(10)).expect("Timeout waiting for tile");
        assert_eq!(id, valid_tile);
        assert!(result.is_ok(), "Failed to fetch valid tile");
    }

    #[test]
    fn test_fetch_invalid_tile() {
        let (tx, rx) = mpsc::channel();
        let fetcher = TileFetcher::new(tx, "https://tile.openstreetmap.org/{z}/{x}/{y}.png".to_string());
        
        // Invalid tile (Z=20 out of bounds or x/y out of bounds for OSM, which causes 404/400)
        let invalid_tile = TileId { z: 20, x: 9999999, y: 9999999 };
        fetcher.request_tile(invalid_tile, TilePriority::Low);
        
        let (id, result) = rx.recv_timeout(Duration::from_secs(10)).expect("Timeout waiting for tile");
        assert_eq!(id, invalid_tile);
        assert!(result.is_err(), "Invalid tile should return error");
    }
}
