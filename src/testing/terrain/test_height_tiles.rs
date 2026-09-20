//! Phase B acceptance for `docs/terrain-plan.md` §5: the Terrarium decoder, the
//! min/max pyramid, and `height_at`'s ancestor upsampling.
//!
//! **These tests never touch the network.** Every elevation number below comes from a
//! tile committed under `assets/terrain_fixtures/` (see the README there), decoded by
//! the same `decode_terrarium` the engine runs. A test that fetched its own input
//! would pin whatever the source served that morning, which is not a regression
//! baseline.

use std::sync::Arc;

use cesium_engine::globe::quadtree::TileId;
use cesium_engine::globe::terrain::height_cache::HeightTileManager;
use cesium_engine::globe::terrain::height_tile::{
    HEIGHT_MIP_BLOCK, HEIGHT_MIP_DIM, HEIGHT_TILE_DIM, HEIGHT_TILE_TEXELS,
};
use cesium_engine::globe::terrain::{decode_terrarium, HeightTile};
use cesium_engine::globe::tiles::config::{OceanPolicy, TerrainConfig, TileEngineConfig};

// ── Fixtures ─────────────────────────────────────────────────────────────────

const EVEREST: &str = "everest_z12_3037_1716.png";
const ZUGSPITZE: &str = "zugspitze_z12_2172_1433.png";
const COAST: &str = "monterey_coast_z12_661_1599.png";
const PACIFIC: &str = "pacific_z12_341_2048.png";
const DEAD_SEA: &str = "dead_sea_z12_2451_1670.png";

fn fixture(name: &str, ocean: OceanPolicy) -> HeightTile {
    let path = format!(
        "{}/assets/terrain_fixtures/{name}",
        env!("CARGO_MANIFEST_DIR")
    );
    let bytes = std::fs::read(&path).unwrap_or_else(|e| panic!("reading {path}: {e}"));
    let img = image::load_from_memory(&bytes)
        .unwrap_or_else(|e| panic!("decoding {path}: {e}"))
        .to_rgba8();
    let (w, h) = (img.width(), img.height());
    decode_terrarium(w, h, &img.into_raw(), ocean)
        .unwrap_or_else(|e| panic!("terrarium decode of {path}: {e}"))
}

// ── B1: the decoder, pinned against committed tiles ──────────────────────────

/// The five fixtures' **exact** full-tile extrema, `OceanPolicy::Raw`.
///
/// These are the regression baseline. They were computed independently of the
/// decoder under test (a standalone Python pass over the same PNGs) before being
/// written down here, so this asserts agreement between two implementations rather
/// than merely recording what the Rust happened to print.
///
/// The plan's §2 numbers (Everest 8740 m, Zugspitze 2911 m) are **not** pinned here:
/// they were sampled with stride 4, so they are lower bounds on the true tile
/// maximum. They appear below as plausibility assertions instead, which is all a
/// subsampled figure can honestly support.
#[test]
fn raw_decode_reproduces_the_fixture_extrema() {
    for (name, min, max) in [
        (EVEREST, 5136, 8753),
        (ZUGSPITZE, 848, 2947),
        (COAST, -23, 123),
        (PACIFIC, -4324, -2276),
        (DEAD_SEA, -412, -412),
    ] {
        let tile = fixture(name, OceanPolicy::Raw);
        assert_eq!(
            (tile.h_min, tile.h_max),
            (min, max),
            "raw extrema of {name}"
        );
    }
}

/// The same tiles under the default policy. Only the negative side moves — clamping
/// is `max(h, 0)` and nothing else.
#[test]
fn clamped_decode_lifts_only_the_sub_sea_level_side() {
    for (name, min, max) in [
        (EVEREST, 5136, 8753),
        (ZUGSPITZE, 848, 2947),
        (COAST, 0, 123),
        (PACIFIC, 0, 0),
        (DEAD_SEA, 0, 0),
    ] {
        let tile = fixture(name, OceanPolicy::ClampToZero);
        assert_eq!(
            (tile.h_min, tile.h_max),
            (min, max),
            "clamped extrema of {name}"
        );
    }
}

/// §2's sanity figures, used as the lower bounds they actually are.
#[test]
fn the_plans_subsampled_summits_are_lower_bounds_on_the_real_ones() {
    assert!(fixture(EVEREST, OceanPolicy::Raw).h_max >= 8740);
    assert!(fixture(ZUGSPITZE, OceanPolicy::Raw).h_max >= 2911);
}

/// The case `OceanPolicy::ClampToZero` exists for: an open-ocean tile is entirely
/// *below* sea level in the source, by kilometres. Left raw, this tile alone would
/// drop a 4 km hole in the Pacific.
#[test]
fn the_open_ocean_carries_bathymetry_not_a_flat_sheet() {
    let raw = fixture(PACIFIC, OceanPolicy::Raw);
    assert!(raw.h_max < 0, "every Pacific sample is below sea level");
    assert!(raw.h_min < -4000, "and some of it is 4 km down");

    let clamped = fixture(PACIFIC, OceanPolicy::ClampToZero);
    assert!(clamped.data.iter().all(|&h| h == 0));
}

/// And the documented price of that choice, asserted rather than left implicit: the
/// Dead Sea is uniformly −412 m in the source and comes out flat at zero.
#[test]
fn clamping_flattens_the_dead_sea_as_documented() {
    let raw = fixture(DEAD_SEA, OceanPolicy::Raw);
    assert!(raw.data.iter().all(|&h| h == -412));
    assert!(fixture(DEAD_SEA, OceanPolicy::ClampToZero)
        .data
        .iter()
        .all(|&h| h == 0));
}

/// A coastal tile is the one fixture where the policy matters *within* a tile rather
/// than to the whole of it — the land side is untouched, the water side lifts.
#[test]
fn a_coastal_tile_keeps_its_land_and_lifts_its_water() {
    let raw = fixture(COAST, OceanPolicy::Raw);
    let clamped = fixture(COAST, OceanPolicy::ClampToZero);
    assert_eq!(raw.h_max, clamped.h_max);
    assert!(raw.h_min < 0 && clamped.h_min == 0);
    for (r, c) in raw.data.iter().zip(clamped.data.iter()) {
        assert_eq!(*c, (*r).max(0));
    }
}

/// The encoding itself, forwards. `h = R·256 + G + B/256 − 32768`.
#[test]
fn the_terrarium_encoding_round_trips_through_the_decoder() {
    for h in [0i32, 1, -1, 8848, -412, -4324, 2947] {
        let biased = (h + 32768) as u32;
        let (r, g) = ((biased >> 8) as u8, (biased & 0xff) as u8);
        let mut rgba = vec![0u8; HEIGHT_TILE_TEXELS * 4];
        for px in rgba.chunks_exact_mut(4) {
            px[0] = r;
            px[1] = g;
            px[2] = 0;
            px[3] = 255;
        }
        let tile = decode_terrarium(256, 256, &rgba, OceanPolicy::Raw).unwrap();
        assert_eq!((tile.h_min, tile.h_max), (h as i16, h as i16), "h = {h}");
    }
}

#[test]
fn a_tile_that_is_not_256_square_is_rejected_rather_than_misread() {
    let rgba = vec![0u8; 512 * 512 * 4];
    assert!(decode_terrarium(512, 512, &rgba, OceanPolicy::Raw).is_err());
}

// ── B1/B3: extrema over all 65 536 texels, not over a subgrid ────────────────

/// The load-bearing half of B1. A summit that falls between the 17x17 points
/// `TileMesh` samples must still reach `h_max`, because Phase D fits the node's
/// bounding box to `h_max` and a box that misses the summit is a false negative,
/// which by I-7 kills the whole subtree.
///
/// Texel (37, 101) is on no plausible subsampling grid: it is not a multiple of 4, 8,
/// 16 or 128, and 256/17 does not land on it either.
#[test]
fn a_summit_between_grid_lines_still_reaches_h_max() {
    let mut data = Box::new([0i16; HEIGHT_TILE_TEXELS]);
    data[101 * HEIGHT_TILE_DIM + 37] = 9000;
    data[200 * HEIGHT_TILE_DIM + 173] = -450;
    let tile = HeightTile::from_samples(data);

    assert_eq!(tile.h_max, 9000);
    assert_eq!(tile.h_min, -450);
    // …and it reaches the mip cell that contains it, and only that one.
    assert_eq!(
        tile.mip_max(37 / HEIGHT_MIP_BLOCK, 101 / HEIGHT_MIP_BLOCK),
        9000
    );
    assert_eq!(tile.mip_max(0, 0), 0);
    assert_eq!(
        tile.mip_min(173 / HEIGHT_MIP_BLOCK, 200 / HEIGHT_MIP_BLOCK),
        -450
    );
}

/// B3, checked exhaustively against the samples rather than against itself: every
/// mip cell's interval really does bound its 16x16 block, and the tile extrema are
/// the extrema of the mip.
///
/// Min and max are stored separately on purpose. In Phase D the occluder reads
/// `mip_min` (a *lower* bound — only what is definitely there can definitely block)
/// and the occludee reads `mip_max` (an *upper* bound). Swapping them over-occludes,
/// which is exactly the false negative the culling work exists to prevent, so the two
/// arrays are asserted independently here.
#[test]
fn the_min_max_pyramid_bounds_every_block_it_covers() {
    let tile = fixture(ZUGSPITZE, OceanPolicy::Raw);

    let mut global_min = i16::MAX;
    let mut global_max = i16::MIN;
    for cy in 0..HEIGHT_MIP_DIM {
        for cx in 0..HEIGHT_MIP_DIM {
            let (lo, hi) = (tile.mip_min(cx, cy), tile.mip_max(cx, cy));
            assert!(lo <= hi, "cell ({cx},{cy}) has an inverted interval");

            let (mut block_min, mut block_max) = (i16::MAX, i16::MIN);
            for y in cy * HEIGHT_MIP_BLOCK..(cy + 1) * HEIGHT_MIP_BLOCK {
                for x in cx * HEIGHT_MIP_BLOCK..(cx + 1) * HEIGHT_MIP_BLOCK {
                    let h = tile.sample(x, y);
                    block_min = block_min.min(h);
                    block_max = block_max.max(h);
                }
            }
            assert_eq!(lo, block_min, "mip_min of cell ({cx},{cy})");
            assert_eq!(hi, block_max, "mip_max of cell ({cx},{cy})");
            global_min = global_min.min(lo);
            global_max = global_max.max(hi);
        }
    }
    assert_eq!((tile.h_min, tile.h_max), (global_min, global_max));
}

// ── B2: bilinear sampling ────────────────────────────────────────────────────

/// Texel centres sit at `(i + 0.5)/256`, so sampling one returns that texel exactly —
/// no half-texel drift, which would shift every mountain by ~19 m of ground at z12.
#[test]
fn sampling_a_texel_centre_returns_that_texel() {
    let tile = fixture(EVEREST, OceanPolicy::Raw);
    let n = HEIGHT_TILE_DIM as f64;
    for (x, y) in [(0usize, 0usize), (1, 1), (37, 101), (128, 128), (255, 255)] {
        let u = (x as f64 + 0.5) / n;
        let v = (y as f64 + 0.5) / n;
        assert_eq!(tile.sample_bilinear(u, v), tile.sample(x, y) as f64);
    }
}

/// Edges clamp to the edge sample rather than extrapolating past it.
#[test]
fn the_tile_border_clamps_instead_of_extrapolating() {
    let tile = fixture(EVEREST, OceanPolicy::Raw);
    assert_eq!(tile.sample_bilinear(0.0, 0.0), tile.sample(0, 0) as f64);
    assert_eq!(tile.sample_bilinear(1.0, 1.0), tile.sample(255, 255) as f64);
}

// ── B2: the ancestor walk — the off-by-one this section exists to catch ───────

/// A z20 tile, five levels below the source's z15 ceiling, built from a hand-chosen
/// quadrant path so the sub-rectangle it occupies in its ancestor can be written down
/// independently of the code under test.
///
/// Quadrant bits, coarse to fine (z15→z16 first, z19→z20 last):
///
/// | step | right? | bottom? |
/// |---|---|---|
/// | z15→z16 | 1 | 1 |
/// | z16→z17 | 0 | 1 |
/// | z17→z18 | 1 | 0 |
/// | z18→z19 | 0 | 0 |
/// | z19→z20 | 1 | 1 |
///
/// so `x20 − x15·32 = 0b10101 = 21` and `y20 − y15·32 = 0b11001 = 25`, i.e. the z20
/// tile is the sub-rectangle `[21/32, 22/32] × [25/32, 26/32]` of the z15 tile.
const ANCESTOR_Z15: TileId = TileId {
    z: 15,
    x: 17_361,
    y: 11_269,
};
const DESCENDANT_Z20: TileId = TileId {
    z: 20,
    x: 17_361 * 32 + 21,
    y: 11_269 * 32 + 25,
};
/// `0b10101 / 32`, from the table above.
const EXPECTED_OFFSET_U: f64 = 21.0 / 32.0;
/// `0b11001 / 32`.
const EXPECTED_OFFSET_V: f64 = 25.0 / 32.0;
const EXPECTED_SCALE: f64 = 1.0 / 32.0;

/// The quadrant accumulation, against the sub-rectangle derived by hand above.
///
/// This is the test the plan asks for by name. The failure mode it guards is an
/// off-by-one *level* in the accumulation — dropping the first or last quadrant bit —
/// which is silent: no panic, no wrong type, just terrain fetched from the wrong half
/// of its ancestor. At z20 half a z15 tile is ~600 m of ground, so the symptom is a
/// landscape shifted by hundreds of metres and nothing else.
#[test]
fn the_ancestor_walk_lands_on_the_hand_computed_sub_rectangle() {
    // The identity is derived independently of the walk: a descendant `k` levels down
    // occupies `[(x_d − x_a·2^k)/2^k, +2^-k]` of its ancestor.
    let k = DESCENDANT_Z20.z - ANCESTOR_Z15.z;
    let span = (1u32 << k) as f64;
    assert_eq!(
        (DESCENDANT_Z20.x - ANCESTOR_Z15.x * (1 << k)) as f64 / span,
        EXPECTED_OFFSET_U
    );
    assert_eq!(
        (DESCENDANT_Z20.y - ANCESTOR_Z15.y * (1 << k)) as f64 / span,
        EXPECTED_OFFSET_V
    );

    for (u, v) in [(0.0, 0.0), (1.0, 1.0), (0.5, 0.25), (0.125, 0.875)] {
        let (su, sv) = HeightTileManager::ancestor_uv(DESCENDANT_Z20, ANCESTOR_Z15, u, v);
        // Exact equality, not approximate: every value here is a dyadic rational with
        // at most 20 fractional bits, so the f32 the walk accumulates in is exact and
        // the widening to f64 is lossless. If this ever needs a tolerance, something
        // has started rounding and the walk is no longer exact.
        assert_eq!(su, EXPECTED_OFFSET_U + u * EXPECTED_SCALE, "u = {u}");
        assert_eq!(sv, EXPECTED_OFFSET_V + v * EXPECTED_SCALE, "v = {v}");
    }
}

/// The corners of the descendant must be the corners of its sub-rectangle, in order —
/// a transposed or mirrored quadrant offset passes the scale check above and fails
/// this one.
#[test]
fn the_sub_rectangle_is_oriented_the_same_way_as_the_tile() {
    let (u0, v0) = HeightTileManager::ancestor_uv(DESCENDANT_Z20, ANCESTOR_Z15, 0.0, 0.0);
    let (u1, v1) = HeightTileManager::ancestor_uv(DESCENDANT_Z20, ANCESTOR_Z15, 1.0, 1.0);
    assert_eq!((u0, v0), (21.0 / 32.0, 25.0 / 32.0));
    assert_eq!((u1, v1), (22.0 / 32.0, 26.0 / 32.0));
}

/// A tile at or above the source's ceiling answers from itself, unscaled.
#[test]
fn a_tile_at_the_source_depth_is_its_own_ancestor() {
    assert_eq!(
        HeightTileManager::ancestor_uv(ANCESTOR_Z15, ANCESTOR_Z15, 0.3, 0.7),
        (0.3, 0.7)
    );
}

// ── B2: the query, end to end ────────────────────────────────────────────────

fn terrain_config(enabled: bool, offline: bool) -> TileEngineConfig {
    TileEngineConfig {
        offline_mode: offline,
        terrain: TerrainConfig {
            enabled,
            ..TerrainConfig::default()
        },
        ..TileEngineConfig::default()
    }
}

/// `h(x, y) = x` metres. Bilinear interpolation of a function linear in `x` is exact,
/// so the expected height at any `u` is a closed form and a displaced quadrant shows
/// up as a plain numeric mismatch instead of a plausible-looking number.
fn x_ramp() -> Arc<HeightTile> {
    let mut data = Box::new([0i16; HEIGHT_TILE_TEXELS]);
    for y in 0..HEIGHT_TILE_DIM {
        for x in 0..HEIGHT_TILE_DIM {
            data[y * HEIGHT_TILE_DIM + x] = x as i16;
        }
    }
    Arc::new(HeightTile::from_samples(data))
}

/// `h(x, y) = y` metres.
fn y_ramp() -> Arc<HeightTile> {
    let mut data = Box::new([0i16; HEIGHT_TILE_TEXELS]);
    for y in 0..HEIGHT_TILE_DIM {
        for x in 0..HEIGHT_TILE_DIM {
            data[y * HEIGHT_TILE_DIM + x] = y as i16;
        }
    }
    Arc::new(HeightTile::from_samples(data))
}

/// The whole of B2 in one assertion: a z20 query, answered from a z15 ancestor,
/// returning megametres.
///
/// With `h = x` the bilinear result at ancestor coordinate `su` is exactly
/// `clamp(su·256 − 0.5, 0, 255)` metres. At `su = 21/32 + u/32` that is
/// `167.5 + 8u` metres. Hand-computed, and far enough from what a mis-accumulated
/// quadrant would give (`0.328125·256 − 0.5 ≈ 83.5 m`, an 84 m error) that the test
/// cannot pass by coincidence.
#[test]
fn a_z20_query_is_answered_from_its_z15_ancestor_in_megametres() {
    let mut heights = HeightTileManager::new(&terrain_config(true, false));
    heights.insert_ready(ANCESTOR_Z15, x_ramp());

    for u in [0.0, 0.25, 0.5, 1.0] {
        let got = heights.height_at(DESCENDANT_Z20, u, 0.5).unwrap();
        let expected_metres = 167.5 + 8.0 * u;
        assert!(
            (got - expected_metres * 1e-6).abs() < 1e-12,
            "u = {u}: got {got} Mm, expected {} Mm",
            expected_metres * 1e-6
        );
    }

    // …and the v axis, from the same sub-rectangle: `25/32 + v/32` → `199.5 + 8v` m.
    heights.insert_ready(ANCESTOR_Z15, y_ramp());
    for v in [0.0, 0.5, 1.0] {
        let got = heights.height_at(DESCENDANT_Z20, 0.5, v).unwrap();
        let expected_metres = 199.5 + 8.0 * v;
        assert!(
            (got - expected_metres * 1e-6).abs() < 1e-12,
            "v = {v}: got {got} Mm, expected {} Mm",
            expected_metres * 1e-6
        );
    }
}

/// Megametres, not metres. The unit trap `quadtree/surface.rs` names: a factor of
/// 10^6 here is a mountain a million times too tall and nothing catches it downstream.
#[test]
fn height_at_returns_megametres() {
    let mut heights = HeightTileManager::new(&terrain_config(true, false));
    let mut data = Box::new([0i16; HEIGHT_TILE_TEXELS]);
    data.fill(8848);
    heights.insert_ready(ANCESTOR_Z15, Arc::new(HeightTile::from_samples(data)));

    let got = heights.height_at(ANCESTOR_Z15, 0.5, 0.5).unwrap();
    assert!((got - 0.008848).abs() < 1e-12, "got {got}");
    // Sanity against the engine's world unit: Everest is ~0.14% of an Earth radius.
    assert!(got < 0.01);
}

/// "Unknown" must be distinguishable from "sea level" (§5 B2). A `0.0` here would let
/// Phase C bake a flat mesh for a tile whose data has not arrived and then never
/// rebuild it.
#[test]
fn an_unloaded_tile_is_unknown_rather_than_sea_level() {
    let mut heights = HeightTileManager::new(&terrain_config(true, false));
    assert_eq!(heights.height_at(DESCENDANT_Z20, 0.5, 0.5), None);

    // A tile that really is at sea level answers 0.0 — the distinction is not
    // theoretical.
    heights.insert_ready(
        ANCESTOR_Z15,
        Arc::new(HeightTile::from_samples(Box::new(
            [0i16; HEIGHT_TILE_TEXELS],
        ))),
    );
    assert_eq!(heights.height_at(DESCENDANT_Z20, 0.5, 0.5), Some(0.0));
}

/// The walk stops at the *deepest* ready ancestor, not the first one it can find from
/// the root — otherwise a loaded z15 tile would be ignored in favour of its z8 parent.
#[test]
fn the_deepest_ready_ancestor_wins() {
    let mut heights = HeightTileManager::new(&terrain_config(true, false));
    let z8 = TileId {
        z: 8,
        x: ANCESTOR_Z15.x >> 7,
        y: ANCESTOR_Z15.y >> 7,
    };

    let mut coarse = Box::new([0i16; HEIGHT_TILE_TEXELS]);
    coarse.fill(1000);
    heights.insert_ready(z8, Arc::new(HeightTile::from_samples(coarse)));
    assert_eq!(heights.resolve_source(DESCENDANT_Z20), Some(z8));

    let mut fine = Box::new([0i16; HEIGHT_TILE_TEXELS]);
    fine.fill(2000);
    heights.insert_ready(ANCESTOR_Z15, Arc::new(HeightTile::from_samples(fine)));
    assert_eq!(heights.resolve_source(DESCENDANT_Z20), Some(ANCESTOR_Z15));
    assert_eq!(
        heights.height_at(DESCENDANT_Z20, 0.5, 0.5),
        Some(2000.0 * 1e-6)
    );
}

// ── B1/B4: source ceiling, offline mode, budget ──────────────────────────────

/// z16 and below do not exist at the source (§2, probed live). Requests for them are
/// redirected to the z15 ancestor rather than turned into 404s and a negative-cache
/// entry per deep tile.
#[test]
fn requests_below_the_source_ceiling_are_redirected_not_fetched() {
    let heights = HeightTileManager::new(&terrain_config(true, false));
    assert_eq!(heights.source_tile_for(DESCENDANT_Z20), ANCESTOR_Z15);
    assert_eq!(heights.source_tile_for(ANCESTOR_Z15), ANCESTOR_Z15);
    let z9 = TileId { z: 9, x: 5, y: 7 };
    assert_eq!(heights.source_tile_for(z9), z9);
}

/// `offline_mode` yields a flat zero field, so every existing headless test can run
/// with terrain on and no network and see the same geometry it sees today (§5
/// acceptance).
#[test]
fn offline_mode_yields_a_flat_zero_field_without_the_network() {
    let mut heights = HeightTileManager::new(&terrain_config(true, true));
    heights.request_tile(
        DESCENDANT_Z20,
        cesium_engine::globe::tiles::tile_fetcher::TilePriority::High,
    );

    // Resolved synchronously — no `update()` call, no worker, no socket.
    assert_eq!(heights.resolve_source(DESCENDANT_Z20), Some(ANCESTOR_Z15));
    assert_eq!(heights.height_at(DESCENDANT_Z20, 0.5, 0.5), Some(0.0));
    assert!(heights.is_loading_complete());
}

/// B4: the height cache reports its own residency, and its capacity comes from its
/// declared slice of the tile byte budget.
#[test]
fn the_height_cache_reports_its_share_of_the_budget() {
    let config = terrain_config(true, false);
    let mut heights = HeightTileManager::new(&config);

    let (resident, capacity) = heights.residency();
    assert_eq!(resident, 0);
    assert_eq!(capacity, 254, "32 MiB / 129 kB per tile");

    heights.insert_ready(ANCESTOR_Z15, x_ramp());
    assert_eq!(heights.residency().0, 1);
    // 256² i16 samples + the two 16² mips + E1's one-i16 measured geometric error. The
    // last term is two bytes and the derived capacity above does not move with it.
    assert_eq!(heights.resident_bytes(), 132_098);

    // …and it is a slice of the imagery budget, not an addition to it.
    assert_eq!(
        config.imagery_cache_budget_bytes() + config.terrain.height_cache_budget_bytes,
        config.tile_cache_budget_bytes
    );
}

/// The one test here that hits the network, and therefore the one that is
/// `#[ignore]`d: it proves the fetch path end to end — URL template, worker, decode —
/// against the live source, and it is run by hand
/// (`cargo test --release --lib live_terrarium -- --ignored`), never by the gate.
///
/// Everything else in this file reads committed fixtures, so the decoder's pins stay
/// a regression baseline instead of a snapshot of what AWS served this morning.
#[test]
#[ignore = "hits the network; run by hand to check the live source, not in the gate"]
fn live_terrarium_fetch_decodes_the_zugspitze_tile() {
    use cesium_engine::globe::tiles::tile_fetcher::TilePriority;

    let mut heights = HeightTileManager::new(&terrain_config(true, false));
    let zugspitze = TileId {
        z: 12,
        x: 2172,
        y: 1433,
    };
    heights.request_tile(zugspitze, TilePriority::High);

    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(20);
    while std::time::Instant::now() < deadline {
        heights.update();
        if heights.resolve_source(zugspitze).is_some() {
            break;
        }
        std::thread::sleep(std::time::Duration::from_millis(50));
    }

    let src = heights
        .resolve_source(zugspitze)
        .expect("tile never arrived");
    assert_eq!(src, zugspitze);
    // Same clamped extrema as the committed fixture of the same tile.
    let sampled = heights.height_at(zugspitze, 0.5, 0.5).unwrap();
    assert!(sampled > 0.0 && sampled < 0.004, "{sampled} Mm");
}
