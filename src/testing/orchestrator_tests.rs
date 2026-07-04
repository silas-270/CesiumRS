use crate::engine::globe::quadtree::TileId;
use crate::engine::globe::io::orchestrator::TileOrchestrator;

#[test]
fn test_compute_fallback_uv_1_level() {
    let parent = TileId { z: 1, x: 0, y: 0 };
    
    // Top-left child
    let child_tl = TileId { z: 2, x: 0, y: 0 };
    let uv = TileOrchestrator::compute_fallback_uv(child_tl, parent);
    assert_eq!(uv, [0.5, 0.5, 0.0, 0.0]);

    // Top-right child
    let child_tr = TileId { z: 2, x: 1, y: 0 };
    let uv = TileOrchestrator::compute_fallback_uv(child_tr, parent);
    assert_eq!(uv, [0.5, 0.5, 0.5, 0.0]);

    // Bottom-left child
    let child_bl = TileId { z: 2, x: 0, y: 1 };
    let uv = TileOrchestrator::compute_fallback_uv(child_bl, parent);
    assert_eq!(uv, [0.5, 0.5, 0.0, 0.5]);

    // Bottom-right child
    let child_br = TileId { z: 2, x: 1, y: 1 };
    let uv = TileOrchestrator::compute_fallback_uv(child_br, parent);
    assert_eq!(uv, [0.5, 0.5, 0.5, 0.5]);
}

#[test]
fn test_compute_fallback_uv_2_levels() {
    let parent = TileId { z: 1, x: 0, y: 0 };
    
    // Child of bottom-right child (so it's z=3, x=3, y=3)
    let child_br_br = TileId { z: 3, x: 3, y: 3 };
    let uv = TileOrchestrator::compute_fallback_uv(child_br_br, parent);
    // It should be 1/4th scale, offset by 3/4th
    assert_eq!(uv, [0.25, 0.25, 0.75, 0.75]);

    // Child of top-left child of top-right child (z=3, x=2, y=0)
    // parent -> tr (z=2,x=1,y=0) -> tl (z=3,x=2,y=0)
    let child_custom = TileId { z: 3, x: 2, y: 0 };
    let uv = TileOrchestrator::compute_fallback_uv(child_custom, parent);
    // tr is offset x=0.5. then tl adds 0 offset, but scaled by 0.5, so x offset is 0.5.
    assert_eq!(uv, [0.25, 0.25, 0.5, 0.0]);
}

#[test]
fn test_compute_fallback_uv_3_levels() {
    let parent = TileId { z: 0, x: 0, y: 0 };
    let child = TileId { z: 3, x: 5, y: 2 };
    
    // Z=1: x=0, y=0
    // Z=2: x=1, y=0
    // Z=3: x=2, y=1 (Wait, 5/2 = 2. 2/2 = 1. So Z=2 is x=2, y=1. Z=1 is x=1, y=0. Wait, parent is z=0,x=0,y=0)
    // Let's trace it:
    // z=3, x=5, y=2
    // z=2, x=2, y=1 -> is_right=1, is_bottom=0
    // z=1, x=1, y=0 -> is_right=0, is_bottom=1
    // z=0, x=0, y=0 -> is_right=1, is_bottom=0
    
    let uv = TileOrchestrator::compute_fallback_uv(child, parent);
    // Expected Scale: 0.125 (1/8)
    // Offset X: 5 * 0.125 = 0.625
    // Offset Y: 2 * 0.125 = 0.25
    assert_eq!(uv, [0.125, 0.125, 0.625, 0.25]);
}
