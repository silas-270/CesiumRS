use cesium_engine::globe::quadtree::TileId;
use cesium_engine::globe::tiles::system::TileSystem;

#[test]
fn test_compute_fallback_uv_1_level() {
    let parent = TileId { z: 1, x: 0, y: 0 };

    // Top-left child
    let child_tl = TileId { z: 2, x: 0, y: 0 };
    let uv = TileSystem::compute_fallback_uv(child_tl, parent);
    assert_eq!(uv, [0.5, 0.5, 0.0, 0.0]);

    // Top-right child
    let child_tr = TileId { z: 2, x: 1, y: 0 };
    let uv = TileSystem::compute_fallback_uv(child_tr, parent);
    assert_eq!(uv, [0.5, 0.5, 0.5, 0.0]);

    // Bottom-left child
    let child_bl = TileId { z: 2, x: 0, y: 1 };
    let uv = TileSystem::compute_fallback_uv(child_bl, parent);
    assert_eq!(uv, [0.5, 0.5, 0.0, 0.5]);

    // Bottom-right child
    let child_br = TileId { z: 2, x: 1, y: 1 };
    let uv = TileSystem::compute_fallback_uv(child_br, parent);
    assert_eq!(uv, [0.5, 0.5, 0.5, 0.5]);
}

#[test]
fn test_compute_fallback_uv_2_levels() {
    let parent = TileId { z: 1, x: 0, y: 0 };

    // Child of bottom-right child (so it's z=3, x=3, y=3)
    let child_br_br = TileId { z: 3, x: 3, y: 3 };
    let uv = TileSystem::compute_fallback_uv(child_br_br, parent);
    // It should be 1/4th scale, offset by 3/4th
    assert_eq!(uv, [0.25, 0.25, 0.75, 0.75]);

    // Child of top-left child of top-right child (z=3, x=2, y=0)
    // parent -> tr (z=2,x=1,y=0) -> tl (z=3,x=2,y=0)
    let child_custom = TileId { z: 3, x: 2, y: 0 };
    let uv = TileSystem::compute_fallback_uv(child_custom, parent);
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

    let uv = TileSystem::compute_fallback_uv(child, parent);
    // Expected Scale: 0.125 (1/8)
    // Offset X: 5 * 0.125 = 0.625
    // Offset Y: 2 * 0.125 = 0.25
    assert_eq!(uv, [0.125, 0.125, 0.625, 0.25]);
}

// `TileId::ancestor_at_level` — the imagery-depth-cap fix. `sync_imagery_requests`
// maps every visible tile through this before requesting a texture for it, so the
// cache never holds (and the fetcher never asks for) a tile deeper than a style's
// `imagery_max_level`; the existing `compute_fallback_uv` tests above are what proves
// the *display* half (an ungranted deep tile stretches its ancestor's texture) — this
// is the *request* half.

#[test]
fn ancestor_at_level_is_a_no_op_at_or_above_the_cap() {
    let shallow = TileId { z: 5, x: 3, y: 7 };
    assert_eq!(shallow.ancestor_at_level(17), shallow);
    assert_eq!(shallow.ancestor_at_level(5), shallow);
}

#[test]
fn ancestor_at_level_climbs_to_exactly_the_cap() {
    // z=20 tile, capped to 17 — three levels up, halving x/y each time (same
    // arithmetic `TileId::parent` uses, just three steps at once).
    let deep = TileId { z: 20, x: 308_729, y: 394_244 }; // NYC-ish, from the live
                                                          // SATELLITE_IMAGERY_MAX_LEVEL probe
    let capped = deep.ancestor_at_level(17);
    assert_eq!(capped.z, 17);
    assert_eq!(capped.x, 308_729 >> 3);
    assert_eq!(capped.y, 394_244 >> 3);

    // Walking `.parent()` three times from the deep tile must land on the same id —
    // `ancestor_at_level` is a shortcut for that walk, not a different rule.
    let walked = deep.parent().unwrap().parent().unwrap().parent().unwrap();
    assert_eq!(capped, walked);
}

#[test]
fn ancestor_at_level_matches_repeated_parent_calls_at_every_depth() {
    let mut id = TileId { z: 9, x: 511, y: 3 };
    for level in (0..=9).rev() {
        assert_eq!(id.ancestor_at_level(level), id, "level {level}");
        if let Some(p) = id.parent() {
            id = p;
        }
    }
}
