use std::time::Duration;
use crate::globe::quadtree::TileId;
use crate::io::mesh_worker::MeshWorkerPool;

#[tokio::test]
async fn test_mesh_worker_spawns_and_returns() {
    let mut pool = MeshWorkerPool::new();
    let id = TileId { z: 0, x: 0, y: 0 };

    // Request a mesh
    pool.request_mesh(id, 16);

    // Wait a bit for the blocking task to complete
    tokio::time::sleep(Duration::from_millis(500)).await;

    // Process results
    let results = pool.process_results();
    assert_eq!(results.len(), 1);
    
    let (received_id, mesh) = &results[0];
    assert_eq!(*received_id, id);
    assert!(!mesh.vertices.is_empty());
    assert!(!mesh.indices.is_empty());
}

#[tokio::test]
async fn test_mesh_worker_deduplicates_requests() {
    let mut pool = MeshWorkerPool::new();
    let id = TileId { z: 1, x: 0, y: 0 };

    // Request the same mesh multiple times
    pool.request_mesh(id, 16);
    pool.request_mesh(id, 16);
    pool.request_mesh(id, 16);

    // Wait for the blocking task to complete
    tokio::time::sleep(Duration::from_millis(500)).await;

    let results = pool.process_results();
    
    // We should only get 1 result back since the subsequent requests were deduplicated
    assert_eq!(results.len(), 1);
}

#[tokio::test]
async fn test_mesh_worker_multiple_concurrent_requests() {
    let mut pool = MeshWorkerPool::new();
    
    // Request multiple distinct meshes
    for i in 0..10 {
        pool.request_mesh(TileId { z: 2, x: i, y: 0 }, 16);
    }

    // Wait for tasks to complete
    tokio::time::sleep(Duration::from_millis(1000)).await;

    let results = pool.process_results();
    
    // We should get 10 results back (order is not guaranteed)
    assert_eq!(results.len(), 10);
    
    // Ensure all requested IDs are present
    let mut found_ids = std::collections::HashSet::new();
    for (id, _) in results {
        found_ids.insert(id.x);
    }
    for i in 0..10 {
        assert!(found_ids.contains(&i));
    }
}
