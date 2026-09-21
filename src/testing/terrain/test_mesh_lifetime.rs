//! Phase E2 acceptance for `docs/terrain-plan.md` §8 — **mesh lifetime**: a mesh is
//! rebuilt when better height data arrives, without the ground visibly jumping and
//! without thrashing.
//!
//! **Nothing here touches the network or a GPU.** The whole policy lives in two free
//! functions in `globe::tiles::system` — [`fresher_height_source`] (is this mesh out of
//! date?) and [`select_mesh_rebuilds`] (which ones, this frame?) — precisely so it can
//! be driven against a bare [`HeightTileManager`] with fixture data, frame by frame,
//! and so the number in [`MESH_REBUILD_BUDGET_PER_FRAME`] is something a test can hold
//! rather than something a capture has to be squinted at to confirm.
//!
//! # What E2 is actually for, and how rare it is
//!
//! Phase C already removed the common case: `HeightTileManager::status_of` answers
//! `Ready` only once the tile's *own* source tile has arrived or failed, so the
//! ordinary mesh is built from the deepest data that will ever exist for it and cannot
//! be improved on. A camera descending five levels creates *new* nodes with no mesh at
//! all — that is the `missing_meshes` path, which predates terrain.
//!
//! What is left, and what this file drives, is the case Phase C wrote down and
//! deferred: a tile whose **own height fetch failed**, whose mesh therefore came from
//! an ancestor, and whose retry later succeeds once the negative cache expires. Plus
//! one case Phase C did not anticipate — switching terrain on at runtime, after which
//! every resident mesh is a flat one with no height source at all.

use std::sync::Arc;

use cesium_engine::globe::geometry::TileMesh;
use cesium_engine::globe::quadtree::TileId;
use cesium_engine::globe::terrain::height_tile::{HEIGHT_TILE_DIM, HEIGHT_TILE_TEXELS};
use cesium_engine::globe::terrain::{HeightPatch, HeightTile, HeightTileManager, Heightfield};
use cesium_engine::globe::tiles::config::{OceanPolicy, TerrainConfig, TileEngineConfig};
use cesium_engine::globe::tiles::system::{
    fresher_height_source, select_mesh_rebuilds, MESH_REBUILD_BUDGET_PER_FRAME,
};
use glam::Vec3;

const SEGMENTS: u32 = 16;

fn terrain_manager() -> HeightTileManager {
    HeightTileManager::new(&TileEngineConfig {
        terrain: TerrainConfig {
            enabled: true,
            ..TerrainConfig::default()
        },
        ..TileEngineConfig::default()
    })
}

/// The committed Zugspitze tile, decoded — real DEM, 1 300 m of relief across it.
fn zugspitze() -> Arc<HeightTile> {
    let path = format!(
        "{}/assets/terrain_fixtures/zugspitze_z12_2172_1433.png",
        env!("CARGO_MANIFEST_DIR")
    );
    let bytes = std::fs::read(&path).unwrap_or_else(|e| panic!("reading {path}: {e}"));
    let img = image::load_from_memory(&bytes).unwrap().to_rgba8();
    let (w, h) = (img.width(), img.height());
    Arc::new(
        cesium_engine::globe::terrain::decode_terrarium(
            w,
            h,
            &img.into_raw(),
            OceanPolicy::ClampToZero,
        )
        .unwrap(),
    )
}

/// `fine`, box-filtered 2:1 into the **top-left quadrant** of a tile one level up.
///
/// This is the relation a real tile pyramid has between a level and its parent, and
/// the centres line up exactly: a parent texel covers two child texels in each axis,
/// and the mean of those two sits at the parent texel's own centre, so the coarse tile
/// is a genuine decimation of the fine one over the same ground rather than a shifted
/// resample of it. The parent's other three quadrants are filled with the same
/// decimation so the tile is not half empty; nothing in this file reads them.
fn decimate_into_parent_quadrant(fine: &HeightTile) -> Arc<HeightTile> {
    let mut data = Box::new([0i16; HEIGHT_TILE_TEXELS]);
    let half = HEIGHT_TILE_DIM / 2;
    for y in 0..half {
        for x in 0..half {
            let mean = (fine.sample(2 * x, 2 * y) as i32
                + fine.sample(2 * x + 1, 2 * y) as i32
                + fine.sample(2 * x, 2 * y + 1) as i32
                + fine.sample(2 * x + 1, 2 * y + 1) as i32)
                / 4;
            let h = mean as i16;
            // All four quadrants, so the coarse tile covers its whole rectangle.
            for (oy, ox) in [(0, 0), (0, half), (half, 0), (half, half)] {
                data[(y + oy) * HEIGHT_TILE_DIM + (x + ox)] = h;
            }
        }
    }
    Arc::new(HeightTile::from_samples(data))
}

/// The heights of a mesh's interior (non-skirt) vertices, in metres above the
/// ellipsoid, read back out of the vertices themselves rather than out of the patch —
/// the drawn geometry, not the intent.
fn interior_vertex_heights_m(mesh: &TileMesh, segments: u32) -> Vec<f64> {
    use crate::testing::culling::geodesy;
    use glam::DVec3;
    let n = segments as usize;
    let grid = n + 3;
    let centre = DVec3::from_array(mesh.center_f64);
    let mut out = Vec::with_capacity((n + 1) * (n + 1));
    for r in 0..=n {
        for c in 0..=n {
            let v = &mesh.vertices[(r + 1) * grid + (c + 1)];
            let p = centre
                + DVec3::new(
                    v.position[0] as f64,
                    v.position[1] as f64,
                    v.position[2] as f64,
                );
            // Altitude above the ellipsoid, dropped along the gradient at the point —
            // the same measurement `test_heightfield`'s `altitude_mm` makes, converted
            // from megametres to metres here because the DEM this is compared against
            // is in metres.
            let (lat, lon) = geodesy::dvec3_to_lat_lon(p);
            let foot = geodesy::lon_lat_to_ecef(lon, lat);
            out.push((p - foot).dot(geodesy::ellipsoid_normal(p)) * 1.0e6);
        }
    }
    out
}

/// A `z13` tile and its `z12` parent, the child in the parent's top-left quadrant.
fn child_and_parent() -> (TileId, TileId) {
    let parent = TileId {
        z: 12,
        x: 2172,
        y: 1433,
    };
    let child = TileId {
        z: 13,
        x: parent.x * 2,
        y: parent.y * 2,
    };
    (child, parent)
}

// ── The rebuild itself ───────────────────────────────────────────────────────

/// **E2's first acceptance**: build a mesh from a coarse source, let the better source
/// arrive, and show that the engine notices and that what comes out is closer to the
/// ground.
///
/// The staging is the real failure mode and not a contrivance: the child's own height
/// tile **fails**, so `status_of` walks up to the parent and the mesh is built from a
/// 2:1 decimation of the child's own ground. Later the retry succeeds — the negative
/// cache expires after `TileEngineConfig::negative_cache_duration`, `request_height_chain`
/// re-queues it, and this time it lands. `fresher_height_source` is what has to see the
/// difference.
///
/// "Finer" is measured as the deviation of the drawn vertices from the DEM the source
/// actually holds, which is the same quantity C4's grid-density table reports.
#[test]
fn a_better_height_tile_rebuilds_the_mesh_and_the_ground_gets_finer() {
    let (child, parent) = child_and_parent();
    let fine = zugspitze();
    let coarse = decimate_into_parent_quadrant(&fine);

    let mut heights = terrain_manager();
    // The child's own fetch failed; the parent's landed.
    heights.insert_failed(child);
    heights.insert_ready(parent, coarse);

    let patch = HeightPatch::sample(&mut heights, child, SEGMENTS, 1.0).expect("parent answers");
    let from_ancestor = TileMesh::generate_on::<Heightfield>(&child, SEGMENTS, &patch);
    assert_eq!(
        from_ancestor.height_source,
        Some(parent),
        "with its own tile failed, the mesh must come from the parent"
    );

    // Nothing better is available yet, so nothing is stale — this is the steady state
    // a failed tile sits in, and it must not spin.
    assert_eq!(
        fresher_height_source(&heights, child, from_ancestor.height_source),
        None,
        "an ancestor-built mesh is not stale while the ancestor is still the best there is"
    );

    // The retry succeeds.
    heights.insert_ready(child, Arc::clone(&fine));

    let now = fresher_height_source(&heights, child, from_ancestor.height_source)
        .expect("the tile's own data is strictly better than its parent's");
    assert_eq!(now, child);

    let patch = HeightPatch::sample(&mut heights, child, SEGMENTS, 1.0).expect("own data answers");
    let rebuilt = TileMesh::generate_on::<Heightfield>(&child, SEGMENTS, &patch);
    assert_eq!(rebuilt.height_source, Some(child));

    // …and it is now up to date, so the rebuild is not offered again. This is what
    // makes the loop terminate rather than re-queue the same tile every frame.
    assert_eq!(
        fresher_height_source(&heights, child, rebuilt.height_source),
        None
    );

    // How much finer, against the DEM the fine tile holds, at the vertices the mesh
    // actually draws.
    let before = interior_vertex_heights_m(&from_ancestor, SEGMENTS);
    let after = interior_vertex_heights_m(&rebuilt, SEGMENTS);
    assert_eq!(before.len(), after.len());

    let n = SEGMENTS as usize;
    let mut truth = Vec::with_capacity(before.len());
    for r in 0..=n {
        for c in 0..=n {
            truth.push(fine.sample_bilinear(c as f64 / n as f64, r as f64 / n as f64));
        }
    }

    let stats = |h: &[f64]| {
        let mut max = 0.0f64;
        let mut sq = 0.0f64;
        for (a, t) in h.iter().zip(truth.iter()) {
            let e = (a - t).abs();
            max = max.max(e);
            sq += e * e;
        }
        (max, (sq / h.len() as f64).sqrt())
    };
    let (max_before, rms_before) = stats(&before);
    let (max_after, rms_after) = stats(&after);

    let span = |h: &[f64]| {
        h.iter().cloned().fold(f64::NEG_INFINITY, f64::max)
            - h.iter().cloned().fold(f64::INFINITY, f64::min)
    };

    println!(
        "\n  E2 rebuild on the Zugspitze fixture, z13 child of z12, {SEGMENTS} segments:\n  \
         | source | max vertex error (m) | RMS (m) | height span (m) |\n  |---|--:|--:|--:|\n  \
         | z12 ancestor | {max_before:6.1} | {rms_before:6.1} | {:7.1} |\n  \
         | own z13 tile | {max_after:6.1} | {rms_after:6.1} | {:7.1} |",
        span(&before),
        span(&after),
    );

    // The rebuilt mesh reads the same field the truth is taken from, so its error is
    // bilinear-round-trip noise; the assertion that matters is that the ancestor's is
    // not, by a wide margin.
    assert!(
        max_before > 10.0,
        "the decimated ancestor should visibly smooth this terrain, got {max_before:.2} m"
    );
    assert!(
        max_after < max_before * 0.25 && rms_after < rms_before * 0.25,
        "the rebuild must be markedly closer to the ground: \
         max {max_before:.2} -> {max_after:.2} m, RMS {rms_before:.2} -> {rms_after:.2} m"
    );
    assert!(
        span(&after) > span(&before),
        "decimation clips summits, so the rebuilt mesh should span more relief: \
         {:.1} -> {:.1} m",
        span(&before),
        span(&after)
    );
}

/// The steady state, which is almost every mesh in almost every frame: a tile built
/// from its **own** source tile can never be bettered, so it is never stale and the E2
/// pass costs it one `status_of` walk and nothing else.
#[test]
fn a_mesh_built_from_its_own_source_is_never_stale() {
    let (_, id) = child_and_parent();
    let mut heights = terrain_manager();
    heights.insert_ready(id, zugspitze());

    let patch = HeightPatch::sample(&mut heights, id, SEGMENTS, 1.0).unwrap();
    let mesh = TileMesh::generate_on::<Heightfield>(&id, SEGMENTS, &patch);
    assert_eq!(mesh.height_source, Some(id));
    assert_eq!(
        fresher_height_source(&heights, id, mesh.height_source),
        None
    );
}

/// **No downgrade** — the geometry half of `display_state`'s rule 4.
///
/// If the deep tile a mesh was built from is evicted, the best *available* source
/// becomes coarser than the one already on the card. Rebuilding then would replace
/// good geometry with worse: the hillside would visibly drop. The rule refuses it, and
/// that refusal is also why the rebuild chain terminates — every accepted rebuild
/// strictly raises `height_source.z`.
#[test]
fn a_coarser_available_source_is_never_a_reason_to_rebuild() {
    let (child, parent) = child_and_parent();
    let mut heights = terrain_manager();
    // Only the parent is resident, but the mesh on the card was built from the child's
    // own tile, before it was evicted. The child's own entry is gone, so `status_of`
    // has to see something terminal at that level for a build to be allowed at all.
    heights.insert_failed(child);
    heights.insert_ready(parent, decimate_into_parent_quadrant(&zugspitze()));

    assert_eq!(
        fresher_height_source(&heights, child, Some(child)),
        None,
        "a mesh built from the child's own tile must not be rebuilt from the parent's"
    );
    // And the other direction still works, so the test is not passing vacuously.
    assert_eq!(
        fresher_height_source(&heights, child, Some(parent.parent().unwrap())),
        Some(parent)
    );
}

/// While the tile's own source is still in flight, **no** rebuild is offered — the same
/// gate `HeightPatch::sample` runs, for the same reason.
///
/// If this asked a different question than the builder does, a rebuild could be
/// scheduled that then produced the identical mesh, every frame, forever.
#[test]
fn a_source_still_in_flight_offers_no_rebuild() {
    let (child, parent) = child_and_parent();
    let mut heights = terrain_manager();
    heights.insert_ready(parent, decimate_into_parent_quadrant(&zugspitze()));
    // `request_tile` on a live manager would mark the child `Fetching`; with no entry
    // at all `status_of` answers `Pending` for the same reason — something better is
    // still coming.
    assert_eq!(fresher_height_source(&heights, child, None), None);

    // The moment the child's own fetch resolves — here, by failing — the parent
    // becomes the honest answer and a flat mesh would be stale against it.
    heights.insert_failed(child);
    assert_eq!(fresher_height_source(&heights, child, None), Some(parent));
}

/// Switching terrain on at runtime (the debug panel, `ViewerCommand::TerrainSetEnabled`)
/// leaves every resident mesh flat, with no height source at all. E2 is what makes that
/// switch do something to the geometry already on the card.
///
/// The reverse — switching terrain **off** — deliberately rebuilds nothing: with no
/// `HeightTileManager` there is no staleness test to run, which is the same `None` arm
/// that keeps the flat path free. The relief meshes stay until the LRU retires them.
#[test]
fn a_flat_mesh_becomes_stale_the_moment_terrain_is_switched_on() {
    let (_, id) = child_and_parent();
    let mut heights = terrain_manager();
    heights.insert_ready(id, zugspitze());

    let flat = TileMesh::generate(&id, SEGMENTS);
    assert_eq!(flat.height_source, None);
    assert_eq!(
        fresher_height_source(&heights, id, flat.height_source),
        Some(id)
    );
}

// ── The rate limit ───────────────────────────────────────────────────────────

/// A descent over five levels above an alpine city, with every tile's own height fetch
/// having failed and every retry landing in the same frame — the worst burst the
/// engine can produce — and the rebuilds still come out at
/// [`MESH_REBUILD_BUDGET_PER_FRAME`] per frame or fewer.
///
/// This is E2's thrashing acceptance. The burst is staged rather than waited for
/// because the thing under test is the *policy*, not the fetcher: what matters is that
/// when 126 meshes go stale at once, no frame is asked to sample and upload more than a
/// handful of them.
///
/// It also checks the two properties that make the drain safe rather than merely
/// bounded: the queue **drains** (every stale mesh is eventually rebuilt — a budget
/// that starved some tile forever would be a permanent coarse patch), and each frame
/// takes the **nearest** stale tiles first, because a stale mesh under the aircraft is
/// the ground it is about to touch while one on the horizon is a few pixels of
/// silhouette.
#[test]
fn a_five_level_descent_cannot_rebuild_more_than_the_budget_in_one_frame() {
    // Innsbruck, the same ground `rendering::terrain_e3_capture` flies into.
    let (lon, lat) = (11.3439, 47.2602);
    let fine = zugspitze();
    let coarse = decimate_into_parent_quadrant(&fine);

    let mut heights = terrain_manager();

    /// The level everything in this test falls back to: one coarse tile that did
    /// arrive, standing in for whatever the camera had loaded before the descent began.
    const ANCHOR_Z: u8 = 10;
    /// The deepest level the descent reaches. Past `max_level` (z15) a tile is answered
    /// by its z15 ancestor, so z16 is included to keep that redirect in the picture.
    const DEEPEST_Z: u8 = 16;

    fn ancestor_at(id: TileId, z: u8) -> TileId {
        let drop = id.z - z;
        TileId {
            z,
            x: id.x >> drop,
            y: id.y >> drop,
        }
    }

    // Six levels of descent, z11 through z16, and a 3x3 block of tiles around the city
    // at each: the set a camera sinking over it has drawn and still holds meshes for.
    let ids: Vec<TileId> = (ANCHOR_Z + 1..=DEEPEST_Z)
        .flat_map(|z| {
            let (centre, _, _) = HeightTileManager::tile_uv_at_lon_lat(lon, lat, z);
            (0..3u32).flat_map(move |dy| {
                (0..3u32).map(move |dx| TileId {
                    z,
                    x: centre.x + dx,
                    y: centre.y + dy,
                })
            })
        })
        .collect();

    // Staged in two passes, and the order is load-bearing: the anchors go in first and
    // the failures second, because the levels nest — one tile's own source tile is
    // another tile's ancestor, and marking them ready in the same pass would quietly
    // un-fail half of them.
    for id in &ids {
        heights.insert_ready(ancestor_at(*id, ANCHOR_Z), Arc::clone(&coarse));
    }
    for id in &ids {
        heights.insert_failed(heights.source_tile_for(*id));
    }

    // Every mesh was therefore built from the anchor: `status_of` walks its own failed
    // source up through the failed levels to the one tile that answered.
    let mut drawn: Vec<(TileId, Vec3, Option<TileId>)> = ids
        .iter()
        .map(|id| {
            let anchor = ancestor_at(*id, ANCHOR_Z);
            assert_eq!(
                heights.resolve_source(*id),
                Some(anchor),
                "staging: {id:?} should fall back to its z{ANCHOR_Z} ancestor"
            );
            (*id, tile_centre_world(*id), Some(anchor))
        })
        .collect();
    let total = drawn.len();
    assert!(
        total > 4 * MESH_REBUILD_BUDGET_PER_FRAME,
        "{total} tiles staged"
    );

    // The eye: 1 200 m over the deepest tile in the stack, i.e. over the city itself,
    // so "nearest first" has a real ordering to get right.
    let eye = tile_centre_world(drawn[drawn.len() - 1].0) * 1.000_188;
    assert!(
        select_mesh_rebuilds(&heights, eye, &drawn, MESH_REBUILD_BUDGET_PER_FRAME).is_empty(),
        "a mesh built from the best available source is not stale"
    );

    // The burst: every retry lands in the same frame.
    for (id, _, _) in &drawn {
        heights.insert_ready(heights.source_tile_for(*id), Arc::clone(&fine));
    }

    let mut frames = 0usize;
    let mut rebuilt = std::collections::HashSet::new();
    let mut worst = 0usize;
    loop {
        let picked = select_mesh_rebuilds(&heights, eye, &drawn, MESH_REBUILD_BUDGET_PER_FRAME);
        if picked.is_empty() {
            break;
        }
        assert!(
            picked.len() <= MESH_REBUILD_BUDGET_PER_FRAME,
            "frame {frames} asked for {} rebuilds, budget is {MESH_REBUILD_BUDGET_PER_FRAME}",
            picked.len()
        );
        worst = worst.max(picked.len());

        // Nearest first: this frame's furthest pick must be no further than anything
        // still waiting.
        let d = |id: TileId| (tile_centre_world(id) - eye).length_squared();
        let furthest_picked = picked.iter().map(|id| d(*id)).fold(0.0f32, f32::max);
        for (id, _, src) in &drawn {
            if picked.contains(id) || rebuilt.contains(id) {
                continue;
            }
            if fresher_height_source(&heights, *id, *src).is_some() {
                assert!(
                    d(*id) >= furthest_picked - 1.0e-9,
                    "frame {frames} skipped a nearer stale tile {:?}",
                    id
                );
            }
        }

        // Apply the rebuilds: each one now carries its own source.
        for id in &picked {
            let src = heights.source_tile_for(*id);
            for entry in drawn.iter_mut() {
                if entry.0 == *id {
                    entry.2 = Some(src);
                }
            }
            rebuilt.insert(*id);
        }
        frames += 1;
        assert!(frames < 10_000, "the rebuild queue did not drain");
    }

    println!(
        "\n  E2 burst: {total} stale meshes over 6 levels drained in {frames} frames, \
         at most {worst} per frame (budget {MESH_REBUILD_BUDGET_PER_FRAME})"
    );
    assert_eq!(
        rebuilt.len(),
        total,
        "every stale mesh must eventually be rebuilt — a starved tile is a permanent \
         coarse patch"
    );
    assert_eq!(frames, total.div_ceil(MESH_REBUILD_BUDGET_PER_FRAME));
}

/// A budget of zero rebuilds nothing, and the empty drawn set costs nothing — the two
/// degenerate inputs the frame loop can hand this on a frame with no terrain in view.
#[test]
fn the_rebuild_selection_handles_its_degenerate_inputs() {
    let (_, id) = child_and_parent();
    let mut heights = terrain_manager();
    heights.insert_ready(id, zugspitze());
    let drawn = [(id, tile_centre_world(id), None)];
    assert!(select_mesh_rebuilds(&heights, Vec3::ZERO, &drawn, 0).is_empty());
    assert!(
        select_mesh_rebuilds(&heights, Vec3::ZERO, &[], MESH_REBUILD_BUDGET_PER_FRAME).is_empty()
    );
    assert_eq!(
        select_mesh_rebuilds(&heights, Vec3::ZERO, &drawn, MESH_REBUILD_BUDGET_PER_FRAME),
        vec![id]
    );
}

/// The world-frame centre of a tile at sea level, in megametres — the same quantity
/// the quadtree hands the renderer as a visible tile's centre.
fn tile_centre_world(id: TileId) -> Vec3 {
    let b = cesium_engine::globe::quadtree::tile_bounds(&id);
    let p = cesium_engine::globe::geometry::lon_lat_to_ecef_f64(b.center_lon(), b.center_lat());
    Vec3::new(p[0] as f32, p[1] as f32, p[2] as f32)
}

// ── The measurement behind the budget ────────────────────────────────────────

/// What one rebuild costs, and therefore what
/// [`MESH_REBUILD_BUDGET_PER_FRAME`] is allowed to be.
///
/// Measured the way `culling::bench_update` measures: warm up, then 200 iterations,
/// report the mean per call. Never a single-shot clock — this machine's same-code
/// spread is wider than most of the differences being quoted.
///
/// The split is the point. `HeightPatch::sample` runs on the **update thread**, once
/// per rebuild, inside the frame; `TileMesh::generate_on` is handed to a rayon worker
/// by `MeshWorkerPool` and does not. So the frame cost of a full budget is
/// `budget × sample`, and the number to compare it against is a 16.6 ms frame.
///
/// ```text
/// cargo test --release --lib terrain::test_mesh_lifetime::e2_what -- --ignored --nocapture
/// ```
#[test]
#[ignore = "measurement, not a gate: prints the per-rebuild cost the frame budget is set against"]
fn e2_what_one_rebuild_costs() {
    use std::time::Instant;

    const ITERS: u32 = 200;
    let (_, id) = child_and_parent();

    println!("\n  | mesh_segments | HeightPatch::sample (us) | generate_on::<Heightfield> (us) | budget x sample (us) | % of a 16.6 ms frame |");
    println!("  |--:|--:|--:|--:|--:|");

    for segments in [16u32, 32, 64] {
        let mut heights = terrain_manager();
        heights.insert_ready(id, zugspitze());

        // Warm-up: first touch pulls the tile through the LRU and the caches.
        for _ in 0..8 {
            let _ = HeightPatch::sample(&mut heights, id, segments, 1.0).unwrap();
        }

        let t0 = Instant::now();
        for _ in 0..ITERS {
            let p = HeightPatch::sample(&mut heights, id, segments, 1.0).unwrap();
            std::hint::black_box(&p);
        }
        let sample_us = t0.elapsed().as_secs_f64() * 1.0e6 / ITERS as f64;

        let patch = HeightPatch::sample(&mut heights, id, segments, 1.0).unwrap();
        for _ in 0..8 {
            std::hint::black_box(TileMesh::generate_on::<Heightfield>(&id, segments, &patch));
        }
        let t0 = Instant::now();
        for _ in 0..ITERS {
            std::hint::black_box(TileMesh::generate_on::<Heightfield>(&id, segments, &patch));
        }
        let gen_us = t0.elapsed().as_secs_f64() * 1.0e6 / ITERS as f64;

        let budget_us = sample_us * MESH_REBUILD_BUDGET_PER_FRAME as f64;
        println!(
            "  | {segments} | {sample_us:8.1} | {gen_us:8.1} | {budget_us:8.1} | {:5.2} % |",
            budget_us / 16_600.0 * 100.0
        );
    }
}
