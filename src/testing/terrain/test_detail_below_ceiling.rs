//! **Deep detail acceptance** — the four levels `DETAIL_MAX_Z` throws away, and what it would take to get them
//! back.
//!
//! `heightfield.rs`'s `DETAIL_MAX_Z = 15` makes `Heightfield::geometric_error` return zero
//! at and below the source's deepest level, so the terrain LOD term stops refining there.
//! Its stated reason is Cesium's: *"past the source's deepest level a node's mesh is an
//! interpolation of its z15 ancestor's samples, so the refinement it would buy is arithmetic
//! and not shape."*
//!
//! # That reason is true for Cesium and false here
//!
//! In Cesium a `HeightmapTerrainData`'s `width × height` **is** the mesh lattice, and
//! `HeightmapTerrainData.upsample` (`Core/HeightmapTerrainData.js`) resamples the parent's
//! *mesh*. A descendant there genuinely cannot show anything its parent did not.
//!
//! Here the source tile is **256 × 256** (`HEIGHT_TILE_DIM`) and the mesh is **17 × 17**
//! (`mesh_segments = 16`). A z15 tile is a 16:1 decimation of data it already holds. A z16
//! node draws the same 17 × 17 lattice over a quarter of that tile — an 8:1 decimation — z17
//! draws 4:1, z18 draws 2:1, and z19 lands on every texel and is exact. Four levels of
//! resolved, already-fetched, already-resident shape, and the LOD term reports zero error
//! for all of them.
//!
//! # What this file measures, in order
//!
//! 1. [`the_source_has_four_more_levels_of_lattice_below_the_ceiling`] — the arithmetic,
//!    with no network: the effective decimation per level, and the level it reaches 1:1.
//! 2. [`f5_which_error_term_tracks_the_truth_below_the_ceiling`] — the decision. For every
//!    visible node below the ceiling at the ten real poses it computes the **true** drawn
//!    error (the windowed deviation at the lattice the mesh really lays down) and scores
//!    three candidate terms against it, in the direction that matters: an error that is too
//!    small costs shape, silently.
//! 3. [`f5_the_cost_table`] — the cost table, at the shipped knob, with the term switched off
//!    and on below the ceiling.
//!
//! # Why `culling` is not in this path
//!
//! `cargo test --release --lib culling::` must keep reporting 32/0/1 and libtest's filter is
//! a plain substring match on the full test path. Same reason as its four siblings.

use std::collections::HashMap;

use cesium_engine::globe::quadtree::{tile_bounds, Frustum, TileId};
use cesium_engine::globe::terrain::height_tile::{HeightTile, HEIGHT_DETAIL_STEP, HEIGHT_TILE_DIM};
use cesium_engine::globe::terrain::{fallback_detail_mm, HeightTileManager, DETAIL_MAX_Z};
use cesium_engine::globe::tiles::config::{
    TerrainConfig, TileEngineConfig, HEIGHT_TILE_BYTES, TERRARIUM_MAX_LEVEL,
};
use glam::DVec3;

use super::test_mesh_density::{
    deviation_at_step, deviation_over, frustum_of, mesh_bytes, mesh_step_on_source, p95,
    settled_at_density, source_window,
};
use super::test_terrain_occlusion::{real_poses, RealWorld};
use crate::testing::culling::cameras::{build_camera, ViewParams};
use crate::testing::lod::sweep::geometric_error_px;
use cesium_engine::camera::camera::CameraMode;

/// Deepest level at which a `mesh_segments = 16` mesh still leaves error on a 256² source:
/// z19 samples every texel, so z18 is the last level with anything to resolve.
const LAST_LEVEL_WITH_DETAIL: u8 = 18;

/// **Approach poses**, and why this evaluation needs its own set rather than the forward ten.
///
/// The ten `real_poses` are all *forward* views: pitch 1-18° below the horizontal from
/// 300 m to 400 km, chosen for terrain occlusion, which is a question about the far field. At 1.5° below
/// horizontal from 900 m the nearest ground in frame is kilometres away, so those trees
/// bottom out around z16 and the deep detail reserve levels are barely reached — [`f5_which…`]
/// measured 88 nodes at z15 and **two** at z16 across all ten.
///
/// A flight tracker on approach is the opposite view: a few hundred metres over the ground,
/// looking down the slope at terrain 1-3 km ahead, which is where z17-z19 live. These four
/// are that case, and they are **added here rather than to `real_poses`** on purpose: every
/// table in previous benchmarks is measured on that list, and changing it would silently
/// move numbers this posting is not about.
///
/// Each one is placed over ground whose height was looked up in the DEM first
/// ([`the_approach_poses_are_above_their_own_ground`] is that check, in the gate), because
/// earlier tests record what happens when they are not: the camera ends up inside a mountain and the
/// measurement reports a flat zero for a reason that has nothing to do with the thing under
/// test.
///
/// | pose | ground (DEM) | camera | AGL | look-down | horizon `√(2Rh)` |
/// |---|--:|--:|--:|--:|--:|
/// | `lowi_final` | 579 m | 1 000 m | 421 m | 12° | 113 km |
/// | `samedan_final` | 1 700 m | 2 200 m | 500 m | 14° | 167 km |
/// | `lukla_final` | 2 843 m | 3 400 m | 557 m | 16° | 208 km |
/// | `aosta_final` | 541 m | 1 100 m | 559 m | 14° | 118 km |
fn approach_poses() -> Vec<(&'static str, ViewParams, f64)> {
    let p = |name: &'static str, lon: f64, lat: f64, alt_m: f64, below: f64, yaw: f64, agl: f64| {
        (
            name,
            ViewParams {
                sweep: "f5",
                lat_deg: lat,
                lon_deg: lon,
                alt_m,
                pitch_deg: 90.0 - below,
                yaw_deg: yaw,
                roll_deg: 0.0,
                width: 1280,
                height: 720,
                mode: CameraMode::Free,
            },
            agl,
        )
    };
    vec![
        // Innsbruck, down the Inn towards the Karwendel wall.
        p("lowi_final", 11.35, 47.255, 1_000.0, 12.0, 90.0, 421.0),
        // Samedan, Upper Engadin, towards Piz Nair.
        p("samedan_final", 9.884, 46.534, 2_200.0, 14.0, 0.0, 500.0),
        // Lukla, into the Dudh Kosi gorge.
        p("lukla_final", 86.731, 27.687, 3_400.0, 16.0, 0.0, 557.0),
        // Aosta, towards the Grand Combin side.
        p("aosta_final", 7.368, 45.738, 1_100.0, 14.0, 0.0, 559.0),
    ]
}

/// The pose check §7b's list had to learn the hard way, run in the gate rather than by eye.
///
/// Every approach pose must sit above its own ground by the AGL its table row claims, and
/// its horizon `√(2Rh)` must be long enough that the shot is terrain rather than sky. No
/// network: it reads the same on-disk DEM cache the measurements do, and skips if a tile is
/// not there.
#[test]
fn the_approach_poses_are_above_their_own_ground() {
    let mut world = RealWorld::new();
    let shipped = TileEngineConfig {
        terrain: TerrainConfig {
            enabled: true,
            ..TerrainConfig::default()
        },
        ..TileEngineConfig::default()
    };
    let mut heights = HeightTileManager::new(&shipped);

    for (name, p, claimed_agl) in approach_poses() {
        let (id, _, _) = HeightTileManager::tile_uv_at_lon_lat(p.lon_deg, p.lat_deg, 15);
        heights.insert_ready(id, world.tile(id));
        let ground_mm = heights
            .peek_height_at_lon_lat(p.lon_deg, p.lat_deg)
            .unwrap_or_else(|| panic!("{name}: no DEM under the pose"));
        let ground_m = ground_mm * 1.0e6;
        let agl = p.alt_m - ground_m;
        assert!(
            agl > 150.0,
            "{name}: camera at {:.0} m over ground at {ground_m:.0} m is {agl:.0} m AGL — \
             §7b's trap, a pose inside a mountain",
            p.alt_m
        );
        assert!(
            (agl - claimed_agl).abs() < 25.0,
            "{name}: the table in `approach_poses` claims {claimed_agl:.0} m AGL and the DEM \
             says {agl:.0} m"
        );
        // √(2Rh) with R the mean Earth radius: how far the shot can possibly see.
        let horizon_km = (2.0 * 6_371_000.0 * p.alt_m).sqrt() / 1000.0;
        println!(
            "    {name:<16} ground {ground_m:>7.0} m  camera {:>7.0} m  AGL {agl:>6.0} m  \
             horizon {horizon_km:>5.0} km",
            p.alt_m
        );
        assert!(
            horizon_km > 50.0,
            "{name}: a {horizon_km:.0} km horizon is too short for the far field to be \
             terrain at all"
        );
    }
}

/// **The arithmetic behind deep detail below the ceiling, with no network and no DEM.**
///
/// The engine's own [`mesh_step_on_source`] is what says how many texels the drawn mesh
/// skips at each level below the ceiling. If this table ever reads `1` before z19 the source
/// or the density has changed and the whole premise needs re-deriving.
#[test]
fn the_source_has_four_more_levels_of_lattice_below_the_ceiling() {
    let src = TileId {
        z: TERRARIUM_MAX_LEVEL,
        x: 17_000,
        y: 11_500,
    };

    // A descendant of `src` at each level: the first child corner every time, which is the
    // one whose window starts at texel 0 and therefore the easiest to check by eye.
    let mut id = src;
    let mut steps = vec![(src.z, mesh_step_on_source(src, src, 16))];
    for _ in 0..5 {
        id = TileId {
            z: id.z + 1,
            x: id.x * 2,
            y: id.y * 2,
        };
        steps.push((id.z, mesh_step_on_source(id, src, 16)));
    }

    assert_eq!(
        steps,
        vec![(15, 16), (16, 8), (17, 4), (18, 2), (19, 1), (20, 1)],
        "the decimation the drawn mesh applies to its z15 source, per level"
    );

    // And the ceiling now sits where that ladder reaches 1:1, not where the source stops.
    assert_eq!(
        DETAIL_MAX_Z,
        TERRARIUM_MAX_LEVEL + 4,
        "§9 F5: four levels of lattice below the source ceiling, so the term stops at z19"
    );
    // The level-based fallback — the cold path, for a node whose tile has not arrived —
    // follows the same ceiling, and is the only thing `DETAIL_MAX_Z` still bounds.
    assert!(fallback_detail_mm(15) > 0.0);
    assert!(fallback_detail_mm(18) > 0.0);
    assert_eq!(fallback_detail_mm(19), 0.0);
}

// ── the candidates ──────────────────────────────────────────────────────────────────

/// The exact error of the drawn mesh over one node below the ceiling: the deviation of the
/// source field from its own decimation, at the lattice this node's mesh really lays down,
/// over this node's own share of the source's texels. Metres.
///
/// This is the number every candidate below is scored against, and it is not itself a
/// candidate: answering it per node at run time means scanning up to 128 × 128 texels inside
/// `refresh_extras`, every frame, for every node.
fn true_drawn_error(tile: &HeightTile, id: TileId, src: TileId, segments: u32) -> f64 {
    deviation_over(
        tile,
        mesh_step_on_source(id, src, segments),
        source_window(id, src),
    )
}

/// **Candidate A** — the first-order interpolant: halve the source tile's stored error once
/// per level below the ceiling.
///
/// Costs nothing: no new bytes, no new decode pass, one shift in `height_bounds_for`. It is
/// the convergence grid density measurements showed — RMS halves per doubling of the density — applied to a
/// number the earlier measurements did not measure it on.
fn candidate_first_order(tile: &HeightTile, id: TileId, src: TileId) -> f64 {
    let k = id.z.saturating_sub(src.z) as u32;
    if k >= 4 {
        return 0.0;
    }
    tile.detail() as f64 / (1u32 << k) as f64
}

/// **Candidate B** — the whole source tile's deviation measured at the level's own lattice,
/// as if it were stored per step at decode.
///
/// Costs three extra passes over 65 536 texels at decode and six bytes a tile. Exact in the
/// *lattice* and still whole-tile in the *window*: a z18 node in a flat corner is charged
/// for the icefall on the other side of its z15 ancestor.
fn candidate_per_step(tile: &HeightTile, id: TileId, src: TileId, segments: u32) -> f64 {
    deviation_at_step(tile, mesh_step_on_source(id, src, segments))
}

/// **Candidate C** — per step *and* per window: the pyramid of 4 + 16 + 64 sub-tile maxima
/// that makes the stored number exactly [`true_drawn_error`].
///
/// Costs the same three decode passes and 168 bytes a tile. Scored here only to confirm it
/// is exact, which it is by construction — it is in the table so the cost column has
/// something to sit next to.
fn candidate_pyramid(tile: &HeightTile, id: TileId, src: TileId, segments: u32) -> f64 {
    true_drawn_error(tile, id, src, segments)
}

/// Ratio statistics of a candidate against the truth.
#[derive(Default)]
struct Score {
    n: usize,
    /// `candidate / truth` per node, for the nodes where the truth is non-zero.
    ratios: Vec<f64>,
    /// Nodes where the candidate is **below** the truth by more than 1 % — shape the LOD
    /// rule would not know it was losing.
    under: usize,
    /// The worst under-statement seen, as `truth / candidate`.
    worst_under: f64,
}

impl Score {
    fn push(&mut self, cand: f64, truth: f64) {
        self.n += 1;
        if truth <= 0.5 {
            // Flat ground: every candidate is right and the ratio is 0/0.
            return;
        }
        self.ratios.push(cand / truth.max(1.0e-9));
        if cand < truth * 0.99 {
            self.under += 1;
            let r = truth / cand.max(1.0e-9);
            if r > self.worst_under {
                self.worst_under = r;
            }
        }
    }

    fn median(&mut self) -> f64 {
        let mut v = self.ratios.clone();
        v.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        if v.is_empty() {
            0.0
        } else {
            v[v.len() / 2]
        }
    }

    fn p5(&mut self) -> f64 {
        let mut v = self.ratios.clone();
        v.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        if v.is_empty() {
            0.0
        } else {
            v[((v.len() as f64 * 0.05) as usize).min(v.len().saturating_sub(1))]
        }
    }
}

/// A settled tree at one pose at the shipped config, plus a live manager to read heights
/// from. `max_zoom` is the shipped 19, so the deep nodes are in the tree.
fn settled_shipped_tree(
    p: &ViewParams,
    world: &mut RealWorld,
) -> (Vec<TileId>, HeightTileManager, Frustum) {
    let shipped = TileEngineConfig {
        terrain: TerrainConfig {
            enabled: true,
            ..TerrainConfig::default()
        },
        ..TileEngineConfig::default()
    };
    let frustum = frustum_of(p);
    let (qt, heights, _) = settled_at_density(p, &frustum, &shipped, 512.0, world, None);
    let ids = qt
        .get_visible_tiles()
        .into_iter()
        .map(|(id, _, _)| id)
        .collect();
    (ids, heights, frustum)
}

/// **Evaluating deep detail below the ceiling** — which stored error term tracks the drawn mesh below the ceiling.
///
/// One row per level below the source ceiling, over every visible node at the ten real
/// poses. The truth column is what the mesh really leaves on screen; the three candidates
/// are scored as `candidate / truth`, and the column that decides is **under** — how often a
/// candidate reports *less* error than the mesh actually has. Too large costs tiles; too
/// small costs shape, silently (`HeightTile::detail`'s own doc comment).
///
/// ```text
/// cargo test --release --lib terrain::test_detail_below_ceiling::f5_which -- --ignored --nocapture
/// ```
#[test]
#[ignore = "measurement, needs the network for real height tiles"]
fn f5_which_error_term_tracks_the_truth_below_the_ceiling() {
    let mut world = RealWorld::new();
    let segments = 16u32;

    // `deviation_over` walks up to 65 536 texels and the same source answers for many nodes.
    let mut memo: HashMap<(TileId, usize, [usize; 4]), f64> = HashMap::new();
    let mut memo_step: HashMap<(TileId, usize), f64> = HashMap::new();

    // Per level offset k = z − 15: truths, and one Score per candidate.
    let mut truths: Vec<Vec<f64>> = vec![Vec::new(); 6];
    let mut a: Vec<Score> = (0..6).map(|_| Score::default()).collect();
    let mut b: Vec<Score> = (0..6).map(|_| Score::default()).collect();
    let mut c: Vec<Score> = (0..6).map(|_| Score::default()).collect();

    // The ten forward poses plus the four approaches: the first set says what the term
    // does to the globe as measured everywhere else, the second is where its reserve lives.
    let mut all: Vec<(&'static str, ViewParams)> = real_poses();
    all.extend(approach_poses().into_iter().map(|(n, p, _)| (n, p)));

    let mut levels: HashMap<u8, usize> = HashMap::new();
    let mut sources: std::collections::HashSet<TileId> = std::collections::HashSet::new();
    for (_, p) in all {
        let (ids, mut heights, _) = settled_shipped_tree(&p, &mut world);
        for id in &ids {
            *levels.entry(id.z).or_insert(0) += 1;
        }
        // Every distinct z15 source the settled tree rests on, and **all** of its
        // descendants down to z19 — not just the nodes the shipped tree happens to hold.
        //
        // That is deliberate, and it is the only honest way to decide this before building
        // it: with `DETAIL_MAX_Z = 15` the terrain term is switched off below the ceiling,
        // so the shipped tree reaches z16 at two nodes in fourteen poses. Scoring the
        // candidates only where the current rule already refines would be scoring them on
        // the one level where they cannot differ. The set below is exactly the set a tree
        // with the term switched on could contain, weighted per node rather than per pixel.
        for id in ids {
            if id.z < TERRARIUM_MAX_LEVEL {
                continue;
            }
            let Some((src, tile)) = heights.source_for(id) else {
                continue;
            };
            if !sources.insert(src) {
                continue;
            }

            for k in 0..=4usize {
                let mut descendants = vec![src];
                for _ in 0..k {
                    descendants = descendants
                        .iter()
                        .flat_map(|d| {
                            [
                                TileId {
                                    z: d.z + 1,
                                    x: d.x * 2,
                                    y: d.y * 2,
                                },
                                TileId {
                                    z: d.z + 1,
                                    x: d.x * 2 + 1,
                                    y: d.y * 2,
                                },
                                TileId {
                                    z: d.z + 1,
                                    x: d.x * 2,
                                    y: d.y * 2 + 1,
                                },
                                TileId {
                                    z: d.z + 1,
                                    x: d.x * 2 + 1,
                                    y: d.y * 2 + 1,
                                },
                            ]
                        })
                        .collect();
                }
                for d in descendants {
                    let step = mesh_step_on_source(d, src, segments);
                    let window = source_window(d, src);
                    let truth = *memo
                        .entry((src, step, window))
                        .or_insert_with(|| deviation_over(&tile, step, window));
                    let per_step = *memo_step
                        .entry((src, step))
                        .or_insert_with(|| candidate_per_step(&tile, d, src, segments));

                    truths[k].push(truth);
                    a[k].push(candidate_first_order(&tile, d, src), truth);
                    b[k].push(per_step, truth);
                    c[k].push(candidate_pyramid(&tile, d, src, segments), truth);
                }
            }
        }
    }

    let mut zs: Vec<_> = levels.iter().map(|(z, n)| (*z, *n)).collect();
    zs.sort();
    println!("\n  [F5] visible nodes by level over all poses (shipped DETAIL_MAX_Z = 15)");
    print!("   ");
    for (z, n) in &zs {
        print!(" z{z}:{n}");
    }
    println!();

    println!("\n  [F5] the drawn error below the z15 source ceiling");
    println!(
        "    {:<6} {:>7} {:>10} {:>10} {:>10}",
        "level", "nodes", "step", "p50 err m", "p95 err m"
    );
    for k in 0..6 {
        if truths[k].is_empty() {
            continue;
        }
        let mut t = truths[k].clone();
        let p50 = {
            let mut v = t.clone();
            v.sort_by(|x, y| x.partial_cmp(y).unwrap());
            v[v.len() / 2]
        };
        println!(
            "    z{:<5} {:>7} {:>10} {:>10.1} {:>10.1}",
            15 + k,
            truths[k].len(),
            (256usize / (16usize << k.min(4))).max(1),
            p50,
            p95(&mut t),
        );
    }

    println!("\n  [F5] candidate / truth, and how often each reports LESS error than there is");
    println!(
        "    {:<6} {:<26} {:>8} {:>8} {:>10} {:>14}",
        "level", "candidate", "p50", "p5", "under", "worst under"
    );
    for k in 0..6 {
        if truths[k].is_empty() {
            continue;
        }
        for (label, s) in [
            ("A first-order (detail/2^k)", &mut a[k]),
            ("B per step, whole tile", &mut b[k]),
            ("C per step, per window", &mut c[k]),
        ] {
            let (p50, p5, under, worst) = (s.median(), s.p5(), s.under, s.worst_under);
            println!(
                "    z{:<5} {label:<26} {p50:>8.2} {p5:>8.2} {:>10} {:>14}",
                15 + k,
                format!("{under}/{}", s.n),
                if worst > 0.0 {
                    format!("{worst:.2}x")
                } else {
                    "—".to_string()
                },
            );
        }
    }
    println!(
        "    (A costs nothing; B costs three decode passes and 6 B a tile; C costs the same \
         three passes and 168 B a tile — 4 + 16 + 64 sub-tile maxima — and is exact by \
         construction.)"
    );
}

/// **The cost table for detail below the ceiling** — what carrying the error below the ceiling costs in
/// tiles, height-cache pressure and vertex bytes, and what it buys in p95 drawn error.
///
/// Read as the marginal column, not the total. `terrain_max_z` is the level
/// the geometric term is allowed to demand refinement into, i.e. `DETAIL_MAX_Z`, swept from
/// the shipped 15 up to 19.
///
/// ```text
/// cargo test --release --lib terrain::test_detail_below_ceiling::f5_the_cost -- --ignored --nocapture
/// ```
#[test]
#[ignore = "measurement, needs the network for real height tiles"]
fn f5_the_cost_table() {
    let mut world = RealWorld::new();
    let segments = 16u32;
    let mut memo: HashMap<(TileId, usize, [usize; 4]), f64> = HashMap::new();

    // Two pose sets, because they answer different halves of the question. The ten
    // forward poses of §7b/§8/§9 are the globe everything else in this document is
    // measured on — what the deep detail term does *there* is the regression column. The four approaches of
    // `approach_poses` are the near field it exists for.
    for (set_name, poses, at) in [
        (
            "the ten forward poses (§7b/§8/§9)",
            real_poses(),
            "alps_inn_valley",
        ),
        (
            "the four approaches (F5)",
            approach_poses()
                .into_iter()
                .map(|(n, p, _)| (n, p))
                .collect::<Vec<_>>(),
            "lowi_final",
        ),
    ] {
        println!("\n  [F5] the cost of carrying the geometric term below the z15 source ceiling");
        println!("    — {set_name}, columns marked * at `{at}` —");
        println!(
            "    {:<10} {:>9} {:>9} {:>12} {:>14} {:>12} {:>16}",
            "max z", "Σ tiles", "vs 15", "tiles*", "vertex MB*", "deepest z*", "drawn p95 px*"
        );

        let (_, vbuf, ibuf) = mesh_bytes(segments);
        let mut base_total = 0usize;

        for max_z in [15u8, 16, 17, 18, 19] {
            let mut total = 0usize;
            let mut at_alps = (0usize, 0usize, 0u8, 0.0f64);
            for (name, p) in &poses {
                let shipped = TileEngineConfig {
                    terrain: TerrainConfig {
                        enabled: true,
                        detail_max_z: max_z,
                        ..TerrainConfig::default()
                    },
                    ..TileEngineConfig::default()
                };
                let frustum = frustum_of(p);
                let (qt, mut heights, asked) =
                    settled_at_density(p, &frustum, &shipped, 512.0, &mut world, None);
                let visible = qt.get_visible_tiles();
                total += visible.len();

                if *name == at {
                    let cam = build_camera(p);
                    let fovy = cam.fovy() as f64;
                    let mut drawn: Vec<f64> = Vec::with_capacity(visible.len());
                    let mut deepest = 0u8;
                    for (id, _, _) in &visible {
                        deepest = deepest.max(id.z);
                        let bounds = tile_bounds(id);
                        let q = cesium_engine::globe::geometry::lon_lat_to_ecef_f64(
                            0.5 * (bounds.lon_min + bounds.lon_max),
                            0.5 * (bounds.lat_min + bounds.lat_max),
                        );
                        let dist = (DVec3::new(q[0], q[1], q[2]) - frustum.eye).length();
                        let mm = match heights.source_for(*id) {
                            Some((src, tile)) => {
                                let step = mesh_step_on_source(*id, src, segments);
                                let window = source_window(*id, src);
                                *memo
                                    .entry((src, step, window))
                                    .or_insert_with(|| deviation_over(&tile, step, window))
                                    * 1.0e-6
                            }
                            None => fallback_detail_mm(id.z),
                        };
                        drawn.push(geometric_error_px(mm, dist, p.height as f64, fovy));
                    }
                    let _ = asked;
                    at_alps = (visible.len(), visible.len(), deepest, p95(&mut drawn));
                }
            }
            if max_z == 15 {
                base_total = total;
            }
            println!(
                "    {:<10} {:>9} {:>9} {:>12} {:>14.1} {:>12} {:>16.1}",
                max_z,
                total,
                format!(
                    "{:+.0}%",
                    100.0 * (total as f64 - base_total as f64) / base_total.max(1) as f64
                ),
                at_alps.1,
                (at_alps.0 * (vbuf + ibuf)) as f64 / 1.0e6,
                format!("z{}", at_alps.2),
                at_alps.3,
            );
        }
    }
    println!(
        "\n    (the drawn error is the windowed deviation at the lattice the mesh really lays \
         down — the same metric §9 F1's `drawn` column uses — so it keeps falling past z15, \
         which is the whole of F5. Height cache: {} entries at {HEIGHT_TILE_BYTES} B.)",
        TerrainConfig::default().height_cache_budget_bytes / HEIGHT_TILE_BYTES
    );

    // ── why the ladder stops where it does ──────────────────────────────────────────
    //
    // The table above says the tree gains one level and no more. That is either the
    // reserve being smaller than it looked, or the measurement not reaching it, and the
    // two are told apart by one number per node: how far inside its own terrain threshold
    // the node actually sits. `ratio = terrain_dist / dist`; above 1 the shape term is
    // asking for this node's children, below 1 it is not, and whichever of the two terms
    // is larger is the one that decided.
    println!(
        "\n  [F5] why the ladder stops — per level at the approach poses, `detail_max_z = 19`"
    );
    println!(
        "    {:<8} {:>7} {:>12} {:>12} {:>14} {:>14}",
        "level", "nodes", "p50 err m", "p95 err m", "p95 terr/dist", "p95 img/dist"
    );
    let shipped = TileEngineConfig {
        terrain: TerrainConfig {
            enabled: true,
            ..TerrainConfig::default()
        },
        ..TileEngineConfig::default()
    };
    let mut per_level: HashMap<u8, (Vec<f64>, Vec<f64>, Vec<f64>)> = HashMap::new();
    for (_, p, _) in approach_poses() {
        let frustum = frustum_of(&p);
        let cam = build_camera(&p);
        let (qt, heights, _) = settled_at_density(&p, &frustum, &shipped, 512.0, &mut world, None);
        let lodf = cesium_engine::globe::quadtree::lod_factor_for(
            shipped.target_texel_ratio,
            512.0,
            p.height as f32,
            cam.fovy(),
        );
        let tlodf = cesium_engine::globe::quadtree::terrain_lod_factor_for(
            shipped.terrain.max_geometric_error_px,
            p.height as f32,
            cam.fovy(),
        );
        for (id, _, radius) in qt.get_visible_tiles() {
            let b = tile_bounds(&id);
            let q = cesium_engine::globe::geometry::lon_lat_to_ecef_f64(
                0.5 * (b.lon_min + b.lon_max),
                0.5 * (b.lat_min + b.lat_max),
            );
            let dist = (DVec3::new(q[0], q[1], q[2]) - frustum.eye).length();
            let detail_m = heights
                .height_bounds_for(id, segments, 1.0, shipped.terrain.detail_max_z)
                .map(|hb| hb.detail as f64 * 1.0e6)
                .unwrap_or(0.0);
            let e = per_level.entry(id.z).or_default();
            e.0.push(detail_m);
            e.1.push(detail_m * 1.0e-6 * tlodf as f64 / dist.max(1.0e-12));
            e.2.push(radius as f64 * lodf as f64 / dist.max(1.0e-12));
        }
    }
    let mut ks: Vec<u8> = per_level.keys().copied().collect();
    ks.sort();
    for z in ks {
        if z < 13 {
            continue;
        }
        let (mut d, mut t, mut i) = per_level.remove(&z).expect("level");
        let n = d.len();
        let mut d2 = d.clone();
        println!(
            "    z{:<7} {:>7} {:>12.1} {:>12.1} {:>14.2} {:>14.2}",
            z,
            n,
            {
                d.sort_by(|a, b| a.partial_cmp(b).unwrap());
                d[n / 2]
            },
            p95(&mut d2),
            p95(&mut t),
            p95(&mut i),
        );
    }
    println!(
        "    (a level subdivides while max(img, terr)/dist > 1. Where the terrain column \
         falls under 1 the shape term has stopped asking — and F5's reserve is however many \
         levels it keeps asking for, not however many the lattice could in principle resolve.)"
    );

    // ── the reserve against the budget that spends it ───────────────────────────────
    //
    // The two tables above, together, say the reserve is four levels of *lattice* and one
    // level of *demand*: at `max_geometric_error_px = 12` the measured error has already
    // fallen under the budget by z16, so the term stops asking. That is a statement about
    // the budget, not about the data — and the way to tell the two apart is to move the
    // budget and watch what each ceiling does with it.
    //
    // This is the table the detail ceiling is actually decided on. With the earlier ceiling the rows below z15 are
    // flat by construction: no budget, however tight, can buy a level the clamp forbids.
    // With the deeper ceiling, the same budget keeps buying.
    println!("\n  [F5] the reserve against the budget — Σ tiles / deepest z / drawn p95 px");
    println!("    — the four approach poses —");
    println!(
        "    {:<12} {:>26} {:>26}",
        "max err px", "detail_max_z = 15 (E1)", "detail_max_z = 19 (F5)"
    );
    for px in [12.0f32, 8.0, 6.0, 4.0] {
        print!("    {px:<12.0}");
        for max_z in [15u8, 19] {
            let mut total = 0usize;
            let mut deepest = 0u8;
            let mut drawn: Vec<f64> = Vec::new();
            for (_, p, _) in approach_poses() {
                let cfg = TileEngineConfig {
                    terrain: TerrainConfig {
                        enabled: true,
                        detail_max_z: max_z,
                        max_geometric_error_px: px,
                        ..TerrainConfig::default()
                    },
                    ..TileEngineConfig::default()
                };
                let frustum = frustum_of(&p);
                let cam = build_camera(&p);
                let fovy = cam.fovy() as f64;
                let (qt, mut heights, _) =
                    settled_at_density(&p, &frustum, &cfg, 512.0, &mut world, None);
                let visible = qt.get_visible_tiles();
                total += visible.len();
                for (id, _, _) in &visible {
                    deepest = deepest.max(id.z);
                    let b = tile_bounds(id);
                    let q = cesium_engine::globe::geometry::lon_lat_to_ecef_f64(
                        0.5 * (b.lon_min + b.lon_max),
                        0.5 * (b.lat_min + b.lat_max),
                    );
                    let dist = (DVec3::new(q[0], q[1], q[2]) - frustum.eye).length();
                    let mm = match heights.source_for(*id) {
                        Some((src, tile)) => {
                            let step = mesh_step_on_source(*id, src, segments);
                            let window = source_window(*id, src);
                            *memo
                                .entry((src, step, window))
                                .or_insert_with(|| deviation_over(&tile, step, window))
                                * 1.0e-6
                        }
                        None => fallback_detail_mm(id.z),
                    };
                    drawn.push(geometric_error_px(mm, dist, p.height as f64, fovy));
                }
            }
            print!(
                "{:>26}",
                format!("{total} / z{deepest} / {:.1}", p95(&mut drawn))
            );
        }
        println!();
    }
    println!(
        "    (the left column is E1's ceiling: below z15 no budget buys anything, so tightening \
         `max_geometric_error_px` past the point where z15 is already met spends tiles on the \
         far field and nothing on the near one. The right column is what F5 unlocks.)"
    );
}

/// The lattice the mesh lands on at z19 is 1:1 with the source, so the error there is
/// **exactly** zero and the ceiling at z19 is a fact about the data rather than a choice.
///
/// Gate, not a measurement: it runs on a synthetic tile with no network.
#[test]
fn the_drawn_error_reaches_exactly_zero_at_z19_and_not_before() {
    // A field that is rough at every scale: no decimation of it is exact until the lattice
    // lands on every texel.
    let mut data = Box::new([0i16; HEIGHT_TILE_DIM * HEIGHT_TILE_DIM]);
    for y in 0..HEIGHT_TILE_DIM {
        for x in 0..HEIGHT_TILE_DIM {
            let v = ((x * 37 + y * 53) % 17) as i16 * 40 + ((x % 3) as i16) * 111;
            data[y * HEIGHT_TILE_DIM + x] = v;
        }
    }
    let tile = HeightTile::from_samples(data);

    let src = TileId {
        z: TERRARIUM_MAX_LEVEL,
        x: 17_000,
        y: 11_500,
    };
    let mut id = src;
    let mut errs = Vec::new();
    for _ in 0..=4 {
        errs.push((
            id.z,
            deviation_over(
                &tile,
                mesh_step_on_source(id, src, 16),
                source_window(id, src),
            ),
        ));
        id = TileId {
            z: id.z + 1,
            x: id.x * 2,
            y: id.y * 2,
        };
    }

    for (z, e) in &errs {
        if *z <= LAST_LEVEL_WITH_DETAIL {
            assert!(
                *e > 0.0,
                "z{z} still has shape to resolve on a field that is rough at every scale, \
                 and the shipped DETAIL_MAX_Z reports zero for it"
            );
        } else {
            assert_eq!(
                *e, 0.0,
                "z{z} samples every texel of its source, so its mesh is the data"
            );
        }
    }

    // And the 16:1 number the engine stores is the z15 entry of that ladder.
    assert_eq!(errs[0].1, deviation_at_step(&tile, HEIGHT_DETAIL_STEP));
}
