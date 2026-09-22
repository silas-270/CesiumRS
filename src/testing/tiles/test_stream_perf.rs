//! Streaming costs under rapid camera movement — what the update thread pays when a
//! burst of tiles lands, and whether the fetch queue lets go of tiles the camera has
//! already left.

use cesium_engine::globe::terrain::height_tile::decode_terrarium;
use cesium_engine::globe::tiles::config::OceanPolicy;
use std::time::Instant;

/// A synthetic Terrarium tile with real relief in it, so `from_samples`' detail passes
/// do the same arithmetic they do on mountains rather than short-circuiting on a flat
/// field.
fn synthetic_terrarium() -> Vec<u8> {
    let mut rgba = vec![0u8; 256 * 256 * 4];
    for y in 0..256usize {
        for x in 0..256usize {
            let metres = 1500.0
                + 800.0 * ((x as f64 * 0.11).sin() * (y as f64 * 0.07).cos())
                + 30.0 * ((x * 7 + y * 13) % 17) as f64;
            let v = ((metres + 32768.0) * 256.0) as u32;
            let i = (y * 256 + x) * 4;
            rgba[i] = (v >> 16) as u8;
            rgba[i + 1] = (v >> 8) as u8;
            rgba[i + 2] = v as u8;
            rgba[i + 3] = 255;
        }
    }
    rgba
}

/// How long one height tile's decode costs. This used to run on the update thread for
/// every tile that arrived in a frame, uncapped — a burst after a fast pan is dozens.
#[test]
fn decode_cost_per_height_tile() {
    let rgba = synthetic_terrarium();
    for _ in 0..5 {
        decode_terrarium(256, 256, &rgba, OceanPolicy::ClampToZero).unwrap();
    }
    const N: u32 = 50;
    let start = Instant::now();
    for _ in 0..N {
        std::hint::black_box(decode_terrarium(256, 256, &rgba, OceanPolicy::ClampToZero).unwrap());
    }
    let per = start.elapsed().as_secs_f64() * 1000.0 / N as f64;
    println!("decode_terrarium: {per:.3} ms per tile; a 32-tile burst = {:.1} ms", per * 32.0);
}

/// A loopback source that accepts every connection and never answers, so the first 16
/// requests occupy every fetch slot and everything after them stays queued.
fn silent_source() -> (std::net::TcpListener, String) {
    let listener = std::net::TcpListener::bind("127.0.0.1:0").expect("bind loopback");
    let url = format!("http://{}/{{z}}/{{x}}/{{y}}.png", listener.local_addr().unwrap());
    (listener, url)
}

/// The fast-pan case: forty height tiles requested, the camera moves on, and only the
/// even ones are still wanted. Every queued odd one must be dropped from the fetcher
/// *and* forgotten by the cache — otherwise it is `Fetching` forever and can never be
/// requested again — while the ones already on the wire are left to finish.
#[test]
fn a_camera_that_moves_on_cancels_its_queued_height_fetches() {
    use cesium_engine::globe::quadtree::TileId;
    use cesium_engine::globe::terrain::HeightTileManager;
    use cesium_engine::globe::tiles::config::{TerrainConfig, TileEngineConfig};
    use cesium_engine::globe::tiles::tile_cache::TileState;
    use cesium_engine::globe::tiles::tile_fetcher::TilePriority;
    use std::collections::HashSet;
    use std::time::Duration;

    let (_listener, url) = silent_source();
    let config = TileEngineConfig {
        terrain: TerrainConfig {
            enabled: true,
            source_url: url,
            ..TerrainConfig::default()
        },
        ..TileEngineConfig::default()
    };
    let mut heights = HeightTileManager::new(&config);

    let ids: Vec<TileId> = (0..40).map(|x| TileId { z: 10, x, y: 300 }).collect();
    for id in &ids {
        heights.request_tile(*id, TilePriority::High);
    }

    // Let the worker fill its 16 slots.
    let deadline = std::time::Instant::now() + Duration::from_secs(5);
    while heights.fetcher_queued_len() > 24 && std::time::Instant::now() < deadline {
        std::thread::sleep(Duration::from_millis(5));
    }
    assert_eq!(heights.fetcher_queued_len(), 24, "16 in flight, 24 queued");

    let wanted: HashSet<TileId> = ids.iter().copied().filter(|id| id.x % 2 == 0).collect();
    let cancelled = heights.cancel_unwanted(&wanted);
    println!("cancelled {cancelled} of 24 queued");
    assert!(cancelled > 0 && cancelled <= 20);
    assert_eq!(heights.fetcher_queued_len(), 24 - cancelled);

    let mut forgotten = 0;
    for id in &ids {
        match heights.cache.peek_state(id) {
            None => {
                assert!(id.x % 2 == 1, "a wanted tile was forgotten: {id:?}");
                forgotten += 1;
            }
            Some(TileState::Fetching) => {}
            _ => panic!("nothing can have arrived from a silent source"),
        }
    }
    assert_eq!(forgotten, cancelled, "every cancelled fetch leaves the cache");

    // A cancelled tile the camera comes back to goes out again.
    let back = ids.iter().copied().find(|id| heights.cache.peek_state(id).is_none()).unwrap();
    heights.request_tile(back, TilePriority::High);
    assert!(matches!(heights.cache.peek_state(&back), Some(TileState::Fetching)));
    assert_eq!(heights.fetcher_queued_len(), 24 - cancelled + 1);
}
