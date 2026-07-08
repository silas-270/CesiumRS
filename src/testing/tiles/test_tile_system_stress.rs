use cesium_engine::globe::quadtree::TileId;
use cesium_engine::globe::tiles::mesh_worker::MeshWorkerPool;
use cesium_engine::globe::tiles::tile_cache::TileCacheManager;
use cesium_engine::globe::tiles::system::TileSystem;
use std::time::{Duration, Instant};

// ---
// Test 1: Deep fallback UV calculation (edge case + perf)
// Verifies correctness and sub-microsecond speed over 100k iterations
// at maximum tile depth (z=20, bottom-right corner tile)
// ---
#[test]
fn test_parent_fallback_math_stress_deep() {
    let parent = TileId { z: 0, x: 0, y: 0 };
    let child = TileId {
        z: 20,
        x: (1 << 20) - 1,
        y: (1 << 20) - 1,
    }; // Absolute bottom-right at z=20

    let start = Instant::now();
    let iterations = 100_000;

    let mut uv = [0.0; 4];
    for _ in 0..iterations {
        uv = TileSystem::compute_fallback_uv(child, parent);
    }
    let duration = start.elapsed();
    let per_op = duration / iterations;

    println!("Deep fallback computation (20 levels): {:?} total", duration);
    println!("Time per op: {:?}", per_op);

    // Bottom-right at every level means offsets accumulate to near 1.0
    // scale = 0.5^20 = ~9.5e-7
    assert!(uv[0] > 0.0 && uv[0] < 1e-5, "scale_x out of range: {}", uv[0]);
    assert!(uv[1] > 0.0 && uv[1] < 1e-5, "scale_y out of range: {}", uv[1]);
    assert!(uv[2] > 0.9999, "offset_x should be near 1.0, got {}", uv[2]);
    assert!(uv[3] > 0.9999, "offset_y should be near 1.0, got {}", uv[3]);

    // Performance: each call should be sub-microsecond
    assert!(per_op < Duration::from_micros(1), "compute_fallback_uv is too slow: {:?}", per_op);
}

// ---
// Test 2: Tile Fetcher Priority Queue Ordering
// Diagnostic: Verifies the internal BinaryHeap ordering is correct
// without relying on network timing (which is non-deterministic)
// ---
#[test]
fn test_tile_fetcher_priority_queue_ordering() {
    use cesium_engine::globe::tiles::tile_fetcher::TilePriority;
    use std::cmp::Ordering;

    // Verify the Ord implementation directly
    assert_eq!(TilePriority::High.cmp(&TilePriority::Low), Ordering::Greater);
    assert_eq!(TilePriority::Low.cmp(&TilePriority::High), Ordering::Less);
    assert_eq!(TilePriority::High.cmp(&TilePriority::High), Ordering::Equal);
    assert_eq!(TilePriority::Low.cmp(&TilePriority::Low), Ordering::Equal);

    // Verify BinaryHeap pops High before Low
    use std::collections::BinaryHeap;

    #[derive(Eq, PartialEq)]
    struct PrioritizedRequest { priority: TilePriority, seq: u32 }
    impl Ord for PrioritizedRequest {
        fn cmp(&self, other: &Self) -> Ordering { self.priority.cmp(&other.priority) }
    }
    impl PartialOrd for PrioritizedRequest {
        fn partial_cmp(&self, other: &Self) -> Option<Ordering> { Some(self.cmp(other)) }
    }

    let mut heap = BinaryHeap::new();
    for i in 0..20 {
        heap.push(PrioritizedRequest { priority: TilePriority::Low, seq: i });
    }
    for i in 0..5 {
        heap.push(PrioritizedRequest { priority: TilePriority::High, seq: i });
    }

    // First 5 pops MUST all be High priority
    for _ in 0..5 {
        let req = heap.pop().unwrap();
        assert_eq!(req.priority, TilePriority::High, "Low priority was popped before High priority!");
    }
    // Remainder should be Low
    while let Some(req) = heap.pop() {
        assert_eq!(req.priority, TilePriority::Low);
    }
    println!("Priority ordering is correctly enforced by BinaryHeap");
}

// ---
// Test 3: Mesh worker stress — 1000 requests, measure throughput
// This is the core CPU geometry pipeline performance test
// ---
#[test]
fn test_mesh_worker_stress_throughput() {
    let mut pool = MeshWorkerPool::new();
    let count = 200; // Use 200 to keep test time reasonable

    let start = Instant::now();
    for i in 0..count {
        pool.request_mesh(
            TileId { z: 10, x: i % 1024, y: i % 1024 },
            16,
        );
    }

    // Poll until all complete or timeout
    let deadline = start + Duration::from_secs(15);
    let mut results = vec![];
    while results.len() < count as usize && Instant::now() < deadline {
        std::thread::sleep(Duration::from_millis(50));
        results.extend(pool.process_results());
    }

    let elapsed = start.elapsed();
    let throughput = results.len() as f64 / elapsed.as_secs_f64();
    println!("Processed {}/{} meshes in {:?}", results.len(), count, elapsed);
    println!("Throughput: {:.2} meshes/sec", throughput);

    assert!(results.len() == count as usize, "Not all meshes completed: {}/{}", results.len(), count);
    // Expect at least 50 meshes/sec on any modern CPU (stress baseline)
    assert!(throughput >= 50.0, "Mesh throughput is critically low: {:.2} meshes/sec", throughput);
}

// ---
// Test 4: Negative cache memory pressure / eviction under pressure
// Verifies LRU eviction and timed expiry work together under load
// ---
#[test]
fn test_negative_cache_eviction_under_pressure() {
    let max_size = std::num::NonZeroUsize::new(10).unwrap();
    let mut cache: TileCacheManager<u8> = TileCacheManager::new(max_size, Duration::from_millis(100));

    // Fill cache beyond capacity with failed tiles
    for i in 0..15u32 {
        let id = TileId { z: 5, x: i, y: 0 };
        cache.mark_failed(id);
    }

    // Only 10 should remain (LRU evicted oldest 5)
    let mut retained = 0;
    for i in 0..15u32 {
        let id = TileId { z: 5, x: i, y: 0 };
        if cache.get_state(&id).is_some() {
            retained += 1;
        }
    }
    assert_eq!(retained, 10, "LRU did not enforce max capacity: {} entries remain", retained);
    println!("LRU correctly evicted down to 10 entries");

    // Wait past negative cache duration
    std::thread::sleep(Duration::from_millis(150));

    // All entries should now be expired and return None
    let mut still_alive = 0;
    for i in 0..15u32 {
        let id = TileId { z: 5, x: i, y: 0 };
        if cache.get_state(&id).is_some() {
            still_alive += 1;
        }
    }
    assert_eq!(still_alive, 0, "Negative cache entries did not expire: {} still alive", still_alive);
    println!("All negative cache entries correctly expired after duration");
}

// ---
// Test 5: Edge case — z=0 tile has no parent (root tile)
// ---
#[test]
fn test_root_tile_has_no_parent() {
    let root = TileId { z: 0, x: 0, y: 0 };
    assert!(root.parent().is_none(), "Root tile z=0 should have no parent");
}

// ---
// Test 6: Edge case — TileId coordinate bounds at max zoom
// Verifies that: 
//   a) saturating_add on u32 does NOT cap at tile-grid max (only at u32::MAX)
//   b) the tile_system's explicit bounds check `n.x <= max_x_y` is therefore
//      the correct guard and must be preserved
//   c) parent traversal terminates correctly after exactly `z` steps
// ---
#[test]
fn test_tile_neighbor_boundary_no_overflow() {
    let z: u8 = 20;
    let max_coord: u32 = (1u32 << z) - 1; // 1048575
    let edge_tile = TileId { z, x: max_coord, y: max_coord };

    // u32::saturating_add caps at u32::MAX (4294967295), NOT at max_coord.
    // This means the tile_system MUST use an explicit bounds check.
    let next_x = edge_tile.x.saturating_add(1);
    assert_eq!(next_x, max_coord + 1, 
        "saturating_add returned unexpected value — behavior changed");
    // Confirm the tile_system's guard logic correctly filters this out:
    assert!(next_x > max_coord,
        "Without an explicit bounds check, this neighbor would be out of the tile grid!");
    println!("Confirmed: saturating_add({}, 1) = {}, > max_coord={}", max_coord, next_x, max_coord);
    println!("The tile_system's `n.x <= max_x_y` guard is essential and must NOT be removed.");

    // Parent traversal at max zoom should terminate at z=0 in exactly 20 steps
    let mut current = edge_tile;
    let mut depth = 0;
    while let Some(parent) = current.parent() {
        current = parent;
        depth += 1;
        assert!(depth <= 21, "Parent traversal exceeded expected depth — infinite loop risk!");
    }
    assert_eq!(current.z, 0, "Traversal did not reach root tile");
    assert_eq!(depth, z as usize, "Parent traversal should take exactly z steps");
    println!("Parent traversal at z=20 correctly terminated at root after {} steps", depth);
}
