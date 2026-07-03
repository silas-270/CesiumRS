use std::time::Duration;
use crate::globe::quadtree::TileId;
use crate::io::mesh_worker::MeshWorkerPool;

#[test]
fn test_mesh_worker_spawns_and_returns() {
    let mut pool = MeshWorkerPool::new();
    let id = TileId { z: 0, x: 0, y: 0 };

    pool.request_mesh(id, 16);

    // Poll until result is ready (rayon is sync, but result is on a thread)
    let deadline = std::time::Instant::now() + Duration::from_secs(5);
    let mut results = vec![];
    while results.is_empty() && std::time::Instant::now() < deadline {
        std::thread::sleep(Duration::from_millis(20));
        results = pool.process_results();
    }

    assert_eq!(results.len(), 1);
    let (received_id, mesh) = &results[0];
    assert_eq!(*received_id, id);
    assert!(!mesh.vertices.is_empty());
    assert!(!mesh.indices.is_empty());
}

#[test]
fn test_mesh_worker_deduplicates_requests() {
    let mut pool = MeshWorkerPool::new();
    let id = TileId { z: 1, x: 0, y: 0 };

    pool.request_mesh(id, 16);
    pool.request_mesh(id, 16);
    pool.request_mesh(id, 16);

    let deadline = std::time::Instant::now() + Duration::from_secs(5);
    let mut results = vec![];
    while results.is_empty() && std::time::Instant::now() < deadline {
        std::thread::sleep(Duration::from_millis(20));
        results = pool.process_results();
    }

    // Should only get 1 result even though we requested 3 times
    assert_eq!(results.len(), 1);
}

#[test]
fn test_mesh_worker_multiple_concurrent_requests() {
    let mut pool = MeshWorkerPool::new();

    for i in 0..10 {
        pool.request_mesh(TileId { z: 2, x: i, y: 0 }, 16);
    }

    let deadline = std::time::Instant::now() + Duration::from_secs(10);
    let mut results = vec![];
    while results.len() < 10 && std::time::Instant::now() < deadline {
        std::thread::sleep(Duration::from_millis(50));
        results.extend(pool.process_results());
    }

    assert_eq!(results.len(), 10);

    let mut found_ids = std::collections::HashSet::new();
    for (id, _) in results {
        found_ids.insert(id.x);
    }
    for i in 0..10 {
        assert!(found_ids.contains(&i));
    }
}
