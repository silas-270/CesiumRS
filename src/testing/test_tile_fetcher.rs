#[cfg(test)]
mod tests {
    use crate::globe::quadtree::TileId;
    use crate::io::tile_fetcher::{TileFetcher, TilePriority};
    use std::sync::mpsc;
    use std::time::Duration;
    use crate::io::providers::OpenStreetMapImageryProvider;
    use std::sync::Arc;

    #[test]
    fn test_tile_fetcher_single_request() {
        let (tx, rx) = mpsc::channel();
        let reqwest_client = reqwest::Client::new();
        let provider = Arc::new(OpenStreetMapImageryProvider::new(reqwest_client));
        
        let fetcher = TileFetcher::new(tx, provider);
        let id = TileId { z: 0, x: 0, y: 0 };
        
        fetcher.request_tile(id, TilePriority::High);
        
        let msg = rx.recv_timeout(Duration::from_secs(10));
        assert!(msg.is_ok(), "Should receive a response within 10 seconds");
        
        let (received_id, result) = msg.unwrap();
        assert_eq!(received_id, id);
        assert!(result.is_ok(), "Image fetch and decode should succeed");
        
        let image_data = result.unwrap();
        assert!(!image_data.is_empty(), "Image data should not be empty");
    }

    #[test]
    fn test_tile_fetcher_priority_ordering() {
        let (tx, rx) = mpsc::channel();
        let reqwest_client = reqwest::Client::new();
        let provider = Arc::new(OpenStreetMapImageryProvider::new(reqwest_client));

        let fetcher = TileFetcher::new(tx, provider);
        
        fetcher.request_tile(TileId { z: 1, x: 0, y: 0 }, TilePriority::Low);
        fetcher.request_tile(TileId { z: 1, x: 1, y: 0 }, TilePriority::Low);
        
        fetcher.request_tile(TileId { z: 2, x: 0, y: 0 }, TilePriority::High);
        
        let mut high_count = 0;
        
        for _ in 0..3 {
            if let Ok((id, _)) = rx.recv_timeout(Duration::from_secs(10)) {
                if id.z == 2 {
                    high_count += 1;
                }
            }
        }
        
        assert_eq!(high_count, 1, "High priority tile should have been processed");
    }
}
