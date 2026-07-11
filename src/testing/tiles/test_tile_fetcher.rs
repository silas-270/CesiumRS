#[cfg(test)]
mod tests {
    use cesium_engine::globe::quadtree::TileId;
    use cesium_engine::globe::tiles::tile_fetcher::{TileFetcher, TilePriority};
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
    #[tokio::test]
    async fn test_fetch_valid_tile() {
        let (tx, mut rx) = tokio::sync::mpsc::unbounded_channel();
        let fetcher = TileFetcher::new(
            tx,
            "https://a.basemaps.cartocdn.com/dark_nolabels/{z}/{x}/{y}.png".to_string(),
            false,
        );

        let valid_tile = TileId { z: 0, x: 0, y: 0 };
        fetcher.request_tile(valid_tile, TilePriority::High);

        // It might take a moment to fetch
        let (id, result) = rx
            .recv()
            .await
            .expect("Timeout waiting for tile");
        assert_eq!(id, valid_tile);
        assert!(result.is_ok(), "Failed to fetch valid tile");
    }

    #[tokio::test]
    async fn test_fetch_invalid_tile() {
        let (tx, mut rx) = tokio::sync::mpsc::unbounded_channel();
        let fetcher = TileFetcher::new(
            tx,
            "https://a.basemaps.cartocdn.com/dark_nolabels/{z}/{x}/{y}.png".to_string(),
            false,
        );

        // Invalid tile (Z=20 out of bounds or x/y out of bounds for OSM, which causes 404/400)
        let invalid_tile = TileId {
            z: 20,
            x: 9999999,
            y: 9999999,
        };
        fetcher.request_tile(invalid_tile, TilePriority::Low);

        let (id, result) = rx
            .recv()
            .await
            .expect("Timeout waiting for tile");
        assert_eq!(id, invalid_tile);
        assert!(result.is_err(), "Invalid tile should return error");
    }
}
