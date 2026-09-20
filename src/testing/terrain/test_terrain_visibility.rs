//! Phase D acceptance for `docs/terrain-plan.md` §7 — D1 (height-aware bounding
//! volumes) and D2 (the horizon with relief). **D3, the occlusion march, is not here.**
//!
//! **Nothing in this file touches the network.** The soundness sweep runs against a
//! synthetic elevation field built in this file; the margin measurement runs against
//! `assets/terrain_fixtures/pyramid_extrema.csv`, committed for exactly this purpose.
//!
//! # Why this is not in `culling::`, and why it is not *called* `…culling` either
//!
//! The same reason Phase C's tests are not (`test_heightfield`'s module doc): the gate's
//! contract is that `cargo test --release --lib culling::` reports **32 passed, 0 failed,
//! 1 ignored** unchanged across every phase of this plan, because that is how "flat mode
//! must not regress, and must not be re-pinned" is enforced. Adding a test to it changes
//! the number being held fixed. So `culling::` keeps saying exactly what it said about
//! `Ellipsoid`, and everything terrain is checked here.
//!
//! The name matters as much as the location, and that is not obvious. libtest's filter is
//! a plain **substring** match on the full test path, so a module named
//! `test_terrain_culling` puts `testing::terrain::test_terrain_culling::…` in the gate's
//! results — it contains the substring `culling::`. Measured, not reasoned: the gate went
//! from 32 to 39 the first time this file ran. Anything added under `testing::terrain`
//! must keep `culling` out of its path.
//!
//! # Why the oracle is not reused, and what replaces it
//!
//! `culling::oracle` answers "is this point on the **ellipsoid** visible" and is frozen.
//! With relief the ellipsoid is not the surface, so that question no longer has the right
//! subject. The truth here is **the mesh the engine actually draws**:
//!
//! > A node is a false negative if it was culled while some vertex of its own mesh is
//! > inside the frustum (exact f64, [`VisibilityOracle`]'s own matrices) and not occluded
//! > by the ellipsoid (exact, Theorem 3.1).
//!
//! That is strictly harder on the engine than the ellipsoid oracle would be — it counts
//! the relief the oracle cannot see — and strictly easier than the truth, because a vertex
//! hidden behind a *mountain* still counts as visible here. Closing that last gap is D3's
//! job, and until D3 exists the gap can only make this test over-report, never under-.

use std::sync::Arc;

use cesium_engine::camera::camera::CameraMode;
use cesium_engine::globe::geometry::TileMesh;
use cesium_engine::globe::quadtree::{
    point_is_occluded, sphere_is_occluded, transform_to_scaled_space, Ellipsoid, Frustum,
    HorizonCamera, QuadtreeManager, QuadtreeNode, ScaledSphere, TileId, TilePatch,
};
use cesium_engine::globe::terrain::height_tile::{HEIGHT_TILE_DIM, HEIGHT_TILE_TEXELS};
use cesium_engine::globe::terrain::{
    skirt_allowance, HeightBounds, HeightBoundsSource, HeightPatch, HeightTile, HeightTileManager,
    Heightfield,
};
use cesium_engine::globe::tiles::config::{OceanPolicy, TerrainConfig, TileEngineConfig};
use glam::DVec3;

use crate::testing::culling::cameras::{build_camera, ViewParams};
use crate::testing::culling::oracle::{VisibilityOracle, NDC_MARGIN};

/// Mesh density the sweep builds at — the shipped default, so the geometry being checked
/// is the geometry that ships (`docs/terrain-plan.md` §6 C4 keeps 16).
const SEGMENTS: u32 = 16;

/// How many times `update` is run per pose before the tree is read.
///
/// The same count `culling::sweep` uses, and for the same reason: `apply_lod`'s 20 %
/// hysteresis and `reorder_children_near_to_far` both need a few ticks to settle. Here it
/// matters twice over, because D1's bounds refresh only tightens nodes that already exist
/// — the first tick creates them with inherited intervals, the later ones re-derive them
/// from data.
const UPDATE_ITERATIONS: usize = 4;

/// Deepest level the sweep's quadtree refines to.
///
/// Well below `MAX_ZOOM`: at 2 km altitude an uncapped tree refines to z20 and spends the
/// whole test budget building meshes for tiles a metre across, which exercises nothing
/// this file is about.
const SWEEP_MAX_ZOOM: u8 = 14;

/// Radial band, in scaled-space units, a mesh vertex must clear the limb by before this
/// file will call it visible. `1e-9` of a unit sphere is 6.4 mm of ground — the same order
/// as the engine's own `HORIZON_EPS_BOUNDS_RAD`, and five orders above the f64 rounding of
/// either test. Vertices inside the band are reported, not scored.
const LIMB_BAND_SCALED: f64 = 1.0e-9;

// ── fixtures ─────────────────────────────────────────────────────────────────────

fn sweep_config() -> TileEngineConfig {
    TileEngineConfig {
        mesh_segments: SEGMENTS,
        terrain: TerrainConfig {
            enabled: true,
            exaggeration: 1.0,
            ocean: OceanPolicy::Raw,
            // The source "serves" every level, so a node's interval is its own tile's
            // whole-tile extrema — which is the regime the real engine is in for every
            // level up to z15, i.e. for every level this sweep reaches.
            max_level: SWEEP_MAX_ZOOM,
            // 1 365 tiles' worth of entries. They are all clones of ~20 `Arc`s, so this
            // buys LRU slots, not memory; without it the cache evicts the coarse levels
            // out from under the tree it is supposed to be bounding.
            height_cache_budget_bytes: 512 * 1024 * 1024,
            ..TerrainConfig::default()
        },
        ..TileEngineConfig::default()
    }
}

/// How much relief a tile of ground width `w_m` metres carries, in metres — the shape the
/// real source has, fitted to it.
///
/// Anchored on two measured points from `assets/terrain_fixtures`: a z12 Everest tile is
/// 9.8 km wide and spans 4 700 m, a z15 one is 1.2 km wide and spans ~590 m. Those give a
/// relation that is very nearly linear in the tile's width, capped where the Earth runs
/// out of relief. A test field with a *constant* range at every level would be the wrong
/// instrument entirely: it would put 9 km of relief inside a 600-m tile and report a
/// false-positive rate that says more about the fixture than about D1.
fn relief_range_m(id: TileId) -> f64 {
    let w_m = 40_075_017.0 / (1_u32 << id.z) as f64;
    (0.48 * w_m).min(9_000.0)
}

/// A deterministic, deliberately rough elevation tile whose range is `range_m`.
///
/// Rough on purpose: a gentle field would let every box contain its mesh by accident. Two
/// incommensurable frequencies plus a diagonal ridge, so no grid spacing in this file
/// lands on a stationary point of it. The mean is offset per level so the levels are not
/// nested copies of one another.
fn rough_field_with_range(range_m: f64, seed: f64) -> Arc<HeightTile> {
    let mut data = Box::new([0i16; HEIGHT_TILE_TEXELS]);
    let amp = range_m * 0.5;
    for y in 0..HEIGHT_TILE_DIM {
        for x in 0..HEIGHT_TILE_DIM {
            let a = (x as f64 * 0.31 + seed).sin() * (y as f64 * 0.17 + seed).cos();
            let b = (x as f64 * 0.041 + y as f64 * 0.067 + seed).sin();
            let h = amp * (0.62 * a + 0.38 * b);
            data[y * HEIGHT_TILE_DIM + x] = h as i16;
        }
    }
    Arc::new(HeightTile::from_samples(data))
}

/// One field per level, built once and shared by every tile at that level.
///
/// Per level rather than per tile because a per-tile field would need 132 kB each and the
/// sweep touches ten thousand of them; per level keeps the memory at one tile per level
/// while still giving each level its own relief scale, which is the property
/// [`relief_range_m`] exists for.
struct LevelFields(Vec<Arc<HeightTile>>);

impl LevelFields {
    fn new() -> Self {
        Self(
            (0..=SWEEP_MAX_ZOOM)
                .map(|z| {
                    let id = TileId { z, x: 0, y: 0 };
                    rough_field_with_range(relief_range_m(id), 0.7 * z as f64)
                })
                .collect(),
        )
    }

    fn get(&self, z: u8) -> Arc<HeightTile> {
        self.0[(z as usize).min(self.0.len() - 1)].clone()
    }
}

/// Makes every node currently in the tree `Ready`, so the sweep measures the **data**
/// path: bounds from the tile's mip, mesh from the same tile.
///
/// The inheritance path is deliberately *not* what this sweep measures — a node on an
/// inherited interval has no mesh yet, so "the drawn mesh" has no referent for it. That
/// half of D1 is [`d1_inherit_margin_covers_the_corpus`]'s job, on real data.
fn fill_cache(node: &QuadtreeNode<Heightfield>, heights: &mut HeightTileManager, f: &LevelFields) {
    let src = heights.source_tile_for(node.id);
    if heights.status_of(node.id) != cesium_engine::globe::terrain::PatchStatus::Ready {
        heights.insert_ready(src, f.get(src.z));
    }
    if let Some(children) = &node.children {
        for c in children.iter() {
            fill_cache(c, heights, f);
        }
    }
}

fn bounds_source<'a>(heights: &'a HeightTileManager) -> HeightBoundsSource<'a> {
    HeightBoundsSource {
        heights,
        segments: SEGMENTS,
        exaggeration: 1.0,
    }
}

fn frustum_for(p: &ViewParams) -> (Frustum, VisibilityOracle) {
    let cam = build_camera(p);
    let aspect = p.aspect();
    let (global_pos, _) = cam.global_transform_f64();
    let frustum = Frustum::planes_only(cam.calculate_frustum_planes(aspect as f32), global_pos)
        .with_corners(cam.frustum_corners_relative(aspect as f32));
    (frustum, VisibilityOracle::new(&cam, aspect))
}

/// The poses the sweep runs, chosen around what Phase C's captures showed.
///
/// `alps_low`'s regime — a few kilometres up, looking along the ground — is where the
/// Phase C hole was, and where a box fitted at `alt = 0` loses the near field. The high
/// cells are where D2 has to matter instead: at 400 km and above the limb is in frame and
/// the question stops being the frustum and becomes the horizon.
fn sweep_poses() -> Vec<ViewParams> {
    let mut out = Vec::new();
    for &(lat, lon) in &[(47.1, 11.0), (27.9, 86.9), (0.0, -150.0), (-33.9, 18.4)] {
        for &alt in &[2_000.0, 4_500.0, 11_000.0, 40_000.0, 400_000.0, 2_000_000.0] {
            for &pitch in &[0.0, 60.0, 82.0, 95.0] {
                out.push(ViewParams {
                    sweep: "terrain_d",
                    lat_deg: lat,
                    lon_deg: lon,
                    alt_m: alt,
                    pitch_deg: pitch,
                    yaw_deg: 20.0,
                    roll_deg: 0.0,
                    width: 1920,
                    height: 1080,
                    mode: CameraMode::Free,
                });
            }
        }
    }
    out
}

// ── the vertex verdict ───────────────────────────────────────────────────────────

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum VertexVerdict {
    /// Unambiguously on screen and clear of the limb. A culled node holding one of
    /// these is a false negative.
    Visible,
    /// In the numeric no-man's-land of the viewport edge or the limb. Scored neither way,
    /// counted so the exclusion stays visible.
    Marginal,
    Hidden,
}

/// Theorem 3.1 in scaled space, so the limb band can be applied radially without a second
/// round trip through `T`.
fn scaled_point_is_occluded(cam: &HorizonCamera, q: DVec3) -> bool {
    if !cam.active {
        return false;
    }
    let h2 = cam.c2 - 1.0;
    let v = q - cam.c;
    let s = cam.c2 - q.dot(cam.c);
    s > h2 && s * s > h2 * v.dot(v)
}

/// Classifies one mesh vertex, in world megametres.
///
/// The limb band is applied by pulling the point **inward** by [`LIMB_BAND_SCALED`], which
/// is the conservative direction: a point closer to the centre is more occluded, so a
/// pulled-in point that is still unoccluded is unoccluded with room to spare. Only that
/// stronger statement is allowed to produce a false negative.
fn classify_vertex(oracle: &VisibilityOracle, cam: &HorizonCamera, p: DVec3) -> VertexVerdict {
    let c = oracle.clip(p);
    if c.w <= 0.0 {
        return VertexVerdict::Hidden;
    }
    if c.w < 1.0e-12 * c.truncate().length().max(1.0) {
        return VertexVerdict::Marginal;
    }
    let ndc = c.truncate() / c.w;
    let out = (ndc.x.abs() - 1.0)
        .max(ndc.y.abs() - 1.0)
        .max(-ndc.z)
        .max(ndc.z - 1.0);
    if out > NDC_MARGIN {
        return VertexVerdict::Hidden;
    }
    if out > -NDC_MARGIN {
        return VertexVerdict::Marginal;
    }

    let q = transform_to_scaled_space(p);
    if scaled_point_is_occluded(cam, q) {
        return VertexVerdict::Hidden;
    }
    if scaled_point_is_occluded(cam, q * (1.0 - LIMB_BAND_SCALED)) {
        return VertexVerdict::Marginal;
    }
    VertexVerdict::Visible
}

/// World-space positions of a mesh's vertices, in megametres (I-2: the f32 offsets are
/// added to the f64 centre, never the other way round).
fn mesh_points(mesh: &TileMesh) -> Vec<DVec3> {
    let c = DVec3::from_array(mesh.center_f64);
    mesh.vertices
        .iter()
        .map(|v| {
            c + DVec3::new(
                v.position[0] as f64,
                v.position[1] as f64,
                v.position[2] as f64,
            )
        })
        .collect()
}

#[derive(Default, Debug, Clone)]
struct PoseTally {
    nodes_visited: usize,
    nodes_culled: usize,
    /// Culled nodes holding at least one unambiguously visible mesh vertex.
    false_negatives: usize,
    /// Vertices that landed in the viewport or limb band, over all culled nodes.
    marginal_vertices: usize,
    visible_leaves: usize,
    /// Visible leaves with no unambiguously visible mesh vertex at all.
    false_positive_leaves: usize,
}

/// Walks every node the traversal touched, scoring culled nodes for false negatives and
/// kept leaves for false positives.
///
/// The recursion mirrors `collect_visible_tiles`: a node with `visible == false` had its
/// subtree deleted (I-7), so its own mesh is the finest statement available about what was
/// thrown away, and that is what gets checked.
fn walk<F>(node: &QuadtreeNode<Heightfield>, tally: &mut PoseTally, score: &mut F)
where
    F: FnMut(TileId, bool, &mut PoseTally),
{
    tally.nodes_visited += 1;
    if !node.visible {
        tally.nodes_culled += 1;
        score(node.id, true, tally);
        return;
    }
    match &node.children {
        Some(children) => {
            for c in children.iter() {
                walk(c, tally, score);
            }
        }
        None => {
            tally.visible_leaves += 1;
            score(node.id, false, tally);
        }
    }
}

// ── D2: the cone test itself ─────────────────────────────────────────────────────

/// Theorem 3.7 at `ρ = 0` **is** Theorem 3.1 — `culling-math.md` §3.7 says so in one line
/// of proof sketch, and this is that line as a test.
///
/// It is worth pinning because the two are written from different algebra: 3.1 compares
/// `s²` against `h²‖v‖²`, 3.7 against `h²(C²‖w‖² − s²)`, and they agree only after
/// substituting `C² = 1 + h²`. A transcription slip in either one would not show up at
/// any single camera, which is why this sweeps a few thousand.
#[test]
fn theorem_37_reduces_to_the_point_test_at_zero_radius() {
    let mut disagreements = 0usize;
    let mut checked = 0usize;

    for alt_km in [1.0, 10.0, 400.0, 2_000.0, 36_000.0, 380_000.0] {
        let cam = HorizonCamera::new(crate::testing::culling::geodesy::lon_lat_alt_to_ecef(
            12.0,
            41.0,
            alt_km * 1000.0,
        ));
        for i in 0..60 {
            for j in 0..40 {
                let lon = -180.0 + 360.0 * i as f64 / 60.0;
                let lat = -85.0 + 170.0 * j as f64 / 40.0;
                for h_m in [-400.0, 0.0, 2_000.0, 8_800.0, 100_000.0] {
                    let p = crate::testing::culling::geodesy::lon_lat_alt_to_ecef(lon, lat, h_m);
                    let sphere = ScaledSphere {
                        m: transform_to_scaled_space(p),
                        rho: 0.0,
                    };
                    // `sphere_is_occluded` inflates rho by its own epsilon, so exact
                    // agreement is only expected away from the boundary: a point the
                    // point test calls occluded by less than that epsilon may legally
                    // come back "not occluded" here. Pull it in by ten times the epsilon
                    // and the two must agree.
                    let inflated = ScaledSphere {
                        m: sphere.m * (1.0 - 1.0e-8),
                        rho: 0.0,
                    };
                    checked += 1;
                    if point_is_occluded(&cam, p) != sphere_is_occluded(&cam, &sphere)
                        && point_is_occluded(&cam, p * (1.0 - 1.0e-8))
                            != sphere_is_occluded(&cam, &inflated)
                    {
                        disagreements += 1;
                    }
                }
            }
        }
    }

    println!("  [thm 3.7 @ rho=0] {checked} points, {disagreements} disagreements");
    assert_eq!(
        disagreements, 0,
        "Theorem 3.7 must reduce to Theorem 3.1 when the bounding sphere is a point"
    );
}

/// Theorem 3.7 never claims a sphere is occluded while a point of it is not.
///
/// The soundness direction, sampled rather than proved: the proof is in
/// `culling-math.md` §3.7, this catches a transcription error in it. Spheres are placed
/// across the whole limb region at radii from a tile's to a continent's, and every sphere
/// the engine would cull is checked against a grid of its own surface and interior points
/// under the exact point test.
#[test]
fn theorem_37_never_occludes_a_sphere_holding_a_visible_point() {
    let mut rng = crate::testing::culling::cameras::Lcg::new(0x7E44_0137);
    let mut culled = 0usize;
    let mut violations = 0usize;

    for alt_km in [8.0, 400.0, 3_000.0, 20_000.0] {
        let cam_pos =
            crate::testing::culling::geodesy::lon_lat_alt_to_ecef(-71.0, 5.0, alt_km * 1000.0);
        let cam = HorizonCamera::new(cam_pos);

        for _ in 0..4_000 {
            let lon = rng.range(-180.0, 180.0);
            let lat = rng.range(-89.0, 89.0);
            let h = rng.range(-2_000.0, 9_000.0);
            let centre = crate::testing::culling::geodesy::lon_lat_alt_to_ecef(lon, lat, h);
            let sphere = ScaledSphere {
                m: transform_to_scaled_space(centre),
                rho: rng.range(1.0e-5, 0.3),
            };
            if !sphere_is_occluded(&cam, &sphere) {
                continue;
            }
            culled += 1;

            // A cheap cover of the ball: its six axis extremes plus random interior
            // points. Every one of them must be occluded, or the theorem over-claims.
            let mut probes: Vec<DVec3> = Vec::new();
            for d in [DVec3::X, DVec3::Y, DVec3::Z] {
                probes.push(sphere.m + d * sphere.rho);
                probes.push(sphere.m - d * sphere.rho);
            }
            for _ in 0..24 {
                let dir = DVec3::new(
                    rng.range(-1.0, 1.0),
                    rng.range(-1.0, 1.0),
                    rng.range(-1.0, 1.0),
                );
                let dir = dir.normalize_or_zero();
                probes.push(sphere.m + dir * sphere.rho * rng.range(0.0, 1.0));
            }
            for q in probes {
                if !scaled_point_is_occluded(&cam, q) {
                    violations += 1;
                }
            }
        }
    }

    println!("  [thm 3.7 soundness] {culled} spheres culled, {violations} escaping points");
    assert_eq!(
        violations, 0,
        "Theorem 3.7 called a sphere occluded while a point inside it was visible"
    );
}

// ── the flat model's pins, restated where they cannot move the gate ──────────────

/// The pins of `docs/terrain-plan.md` §4, checked for **both** models.
///
/// `culling::test_stage_pipeline::test_horizon_hot_structs_have_not_grown` already pins
/// `TilePatch<Ellipsoid>` and `HorizonCamera` and is not touched. What it cannot say —
/// because saying it would mean adding an assertion to the gate — is that `QuadtreeNode`
/// is still 192 B after Phase D put a height interval and a bounding sphere on the
/// *terrain* node, and that the terrain node is the only one that grew. That is the whole
/// claim the zero-sized payload exists to make, so it is asserted here, and the terrain
/// sizes are printed rather than pinned: they are allowed to move.
#[test]
fn the_zero_sized_payload_still_costs_the_flat_node_nothing() {
    let flat_node = std::mem::size_of::<QuadtreeNode<Ellipsoid>>();
    let terrain_node = std::mem::size_of::<QuadtreeNode<Heightfield>>();
    let flat_patch = std::mem::size_of::<TilePatch<Ellipsoid>>();
    let terrain_patch = std::mem::size_of::<TilePatch<Heightfield>>();
    println!(
        "  QuadtreeNode: Ellipsoid {flat_node} B, Heightfield {terrain_node} B   \
         TilePatch: Ellipsoid {flat_patch} B, Heightfield {terrain_patch} B"
    );
    assert_eq!(flat_node, 192, "QuadtreeNode<Ellipsoid> must stay 192 B");
    assert_eq!(flat_patch, 64, "TilePatch<Ellipsoid> must stay 64 B");
    assert_eq!(
        std::mem::size_of::<HorizonCamera>(),
        56,
        "HorizonCamera must stay 56 B"
    );
    assert!(
        terrain_node > flat_node && terrain_patch > flat_patch,
        "the terrain payloads are supposed to be real — if they are zero-sized too, \
         the surface model is not carrying anything and D1/D2 are not wired up"
    );
}

// ── D1: the margin ───────────────────────────────────────────────────────────────

/// Reproduces [`HeightTileManager::height_bounds_for`]'s box span from a corpus row, for a
/// tile at or above the source's deepest level (where the mip covers the whole tile and
/// the extrema *are* the whole-tile extrema).
fn corpus_span(id: TileId, h_min_m: i32, h_max_m: i32, clamp: bool) -> HeightBounds {
    let (lo_m, hi_m) = if clamp {
        (h_min_m.max(0), h_max_m.max(0))
    } else {
        (h_min_m, h_max_m)
    };
    let lo = lo_m as f64 * 1.0e-6;
    let hi = hi_m as f64 * 1.0e-6;
    HeightBounds {
        lo: lo - skirt_allowance(id, SEGMENTS, hi - lo),
        hi,
    }
}

/// **The one genuine soundness trap in D1** (`docs/terrain-plan.md` §7), measured.
///
/// For every parent/child pair in the committed corpus, the child's own interval must fit
/// inside its parent's widened by `HEIGHT_INHERIT_MARGIN_M[child.z]`. If it does not, a
/// node culled before its height tile arrives is culled against a box that does not
/// contain the geometry it will draw — a false negative, and by I-7 a hole.
///
/// Both ocean policies, because the constant cannot know which one is configured.
///
/// Prints the table that `docs/terrain-plan.md` §7 and `HEIGHT_INHERIT_MARGIN_M` quote.
#[test]
fn d1_inherit_margin_covers_the_corpus() {
    let csv = std::fs::read_to_string("assets/terrain_fixtures/pyramid_extrema.csv")
        .expect("assets/terrain_fixtures/pyramid_extrema.csv — see the README there");

    let mut rows: std::collections::HashMap<(u8, u32, u32), (i32, i32)> =
        std::collections::HashMap::new();
    for line in csv.lines().skip(1).filter(|l| !l.trim().is_empty()) {
        let f: Vec<&str> = line.split(',').collect();
        assert_eq!(f.len(), 5, "malformed corpus row: {line}");
        rows.insert(
            (
                f[0].parse().unwrap(),
                f[1].parse().unwrap(),
                f[2].parse().unwrap(),
            ),
            (f[3].parse().unwrap(), f[4].parse().unwrap()),
        );
    }
    assert!(rows.len() > 500, "corpus is too small to bound a tail");

    // Per child level: (pairs, worst needed margin in metres, the pair that needed it).
    let mut per_level: Vec<(usize, f64, Option<(TileId, bool)>)> = vec![(0, f64::MIN, None); 32];
    let mut failures: Vec<String> = Vec::new();

    for (&(z, x, y), &(lo, hi)) in &rows {
        if z < 2 {
            continue;
        }
        let parent = (z - 1, x / 2, y / 2);
        let Some(&(plo, phi)) = rows.get(&parent) else {
            continue;
        };
        let child_id = TileId { z, x, y };
        let parent_id = TileId {
            z: parent.0,
            x: parent.1,
            y: parent.2,
        };

        for clamp in [true, false] {
            let c = corpus_span(child_id, lo, hi, clamp);
            let p = corpus_span(parent_id, plo, phi, clamp);
            // What `child_extra` would have to add to the parent's interval for it to
            // contain the child's, on whichever side is worse.
            let needed_m = (c.hi - p.hi).max(p.lo - c.lo) * 1.0e6;

            let slot = &mut per_level[z as usize];
            slot.0 += 1;
            if needed_m > slot.1 {
                slot.1 = needed_m;
                slot.2 = Some((child_id, clamp));
            }

            let widened = p.widened(cesium_engine::globe::terrain::inherit_margin_mm(z));
            if !widened.contains(&c) {
                failures.push(format!(
                    "z{z} {x}/{y} (clamp={clamp}): needs {needed_m:.0} m, \
                     HEIGHT_INHERIT_MARGIN_M gives {:.0} m",
                    cesium_engine::globe::terrain::inherit_margin_mm(z) * 1.0e6
                ));
            }
        }
    }

    println!("  [D1 inherit margin] from assets/terrain_fixtures/pyramid_extrema.csv");
    println!(
        "    {:>3} {:>7} {:>15} {:>13} {:>9}",
        "z", "pairs", "max needed (m)", "margin (m)", "headroom"
    );
    for (z, (pairs, worst, who)) in per_level.iter().enumerate() {
        if *pairs == 0 {
            continue;
        }
        let margin_m = cesium_engine::globe::terrain::inherit_margin_mm(z as u8) * 1.0e6;
        let headroom = if *worst > 0.0 {
            format!("{:.1}x", margin_m / worst)
        } else {
            "n/a".to_string()
        };
        println!(
            "    {z:>3} {pairs:>7} {worst:>15.0} {margin_m:>13.0} {headroom:>9}   worst: {:?}",
            who.map(|(id, c)| (id.z, id.x, id.y, c))
        );
    }

    assert!(
        failures.is_empty(),
        "the inherited height interval is not a superset of the child's at {} pair(s) — \
         D1's margin is too small and every one of these is a false negative waiting to \
         happen:\n  {}",
        failures.len(),
        failures.join("\n  ")
    );
}

/// The zero margin below the source's deepest level is exact, not optimistic.
///
/// Past z15 a child reads the *same* height tile as its parent over a dyadic
/// sub-rectangle, so its covering mip cells are a subset of the parent's. This checks the
/// claim on the real data path rather than on the argument: real tiles, real
/// `height_bounds_for`, every child of a deep node.
#[test]
fn below_the_source_ceiling_a_child_interval_is_contained_without_a_margin() {
    let root = TileId { z: 4, x: 8, y: 5 };
    let config = TileEngineConfig {
        mesh_segments: SEGMENTS,
        terrain: TerrainConfig {
            enabled: true,
            exaggeration: 1.0,
            ocean: OceanPolicy::Raw,
            // The regime this test is about: the source stops here and every deeper tile
            // is answered by this one over a shrinking sub-rectangle.
            max_level: 4,
            ..TerrainConfig::default()
        },
        ..TileEngineConfig::default()
    };
    let mut heights = HeightTileManager::new(&config);
    heights.insert_ready(root, rough_field_with_range(relief_range_m(root), 0.0));

    let mut checked = 0usize;
    let mut worst_excess_m = f64::NEG_INFINITY;
    let mut parents = vec![root];
    for _ in 0..8 {
        let mut next = Vec::new();
        for p in parents.drain(..) {
            let Some(pb) = heights.height_bounds_for(p, SEGMENTS, 1.0) else {
                continue;
            };
            for (dx, dy) in [(0, 0), (1, 0), (0, 1), (1, 1)] {
                let c = TileId {
                    z: p.z + 1,
                    x: p.x * 2 + dx,
                    y: p.y * 2 + dy,
                };
                let Some(cb) = heights.height_bounds_for(c, SEGMENTS, 1.0) else {
                    continue;
                };
                checked += 1;
                worst_excess_m = worst_excess_m.max((cb.hi - pb.hi).max(pb.lo - cb.lo) * 1.0e6);
                assert!(
                    pb.contains(&cb),
                    "z{} {}/{}: child interval [{:.3}, {:.3}] km escapes its parent's \
                     [{:.3}, {:.3}] km with no margin",
                    c.z,
                    c.x,
                    c.y,
                    cb.lo * 1000.0,
                    cb.hi * 1000.0,
                    pb.lo * 1000.0,
                    pb.hi * 1000.0
                );
                next.push(c);
            }
        }
        parents = next;
    }

    println!(
        "  [D1 zero margin below the z4 ceiling] {checked} child/parent pairs, \
         worst excess {worst_excess_m:.1} m"
    );
    assert!(checked > 100, "the descent did not actually happen");
}

/// **I-1′ meets D1**: the interval a node's box is fitted over contains the interval the
/// mesh for that node declares.
///
/// Phase C made `TileMesh::height_bounds` a promise
/// (`test_heightfield::generated_meshes_stay_within_their_declared_height_bounds`); this
/// is the other half of the join. Without it, the mesh could be inside its own claim and
/// the box fitted to a different one, and every sweep below would be measuring the wrong
/// pair.
#[test]
fn a_node_interval_contains_the_mesh_interval_it_is_fitted_against() {
    let config = sweep_config();
    let fields = LevelFields::new();
    let mut heights = HeightTileManager::new(&config);

    let mut checked = 0usize;
    let mut worst_slack_m = f64::INFINITY;
    // How much taller the node's box interval is than the mesh interval it must contain —
    // the price of `skirt_allowance` bounding C3's content-derived skirt by the tile's
    // whole height range. Reported, not asserted: it is a cost, not a defect.
    let mut inflation = 0.0_f64;
    for z in 2..=SWEEP_MAX_ZOOM {
        let n = 1_u32 << z;
        for (x, y) in [
            (0, 0),
            (n - 1, 0),
            (0, n - 1),
            (n / 2, n / 2),
            (n / 3, n / 2),
        ] {
            let id = TileId { z, x, y };
            heights.insert_ready(id, fields.get(z));
            let node_bounds = heights
                .height_bounds_for(id, SEGMENTS, 1.0)
                .expect("just inserted");
            let patch = HeightPatch::sample(&mut heights, id, SEGMENTS, 1.0).expect("Ready");
            let mesh = TileMesh::generate_on::<Heightfield>(&id, SEGMENTS, &patch);
            let [mlo, mhi] = mesh.height_bounds;

            checked += 1;
            worst_slack_m =
                worst_slack_m.min((mlo - node_bounds.lo).min(node_bounds.hi - mhi) * 1.0e6);
            inflation += (node_bounds.hi - node_bounds.lo) / (mhi - mlo);
            assert!(
                node_bounds.lo <= mlo && mhi <= node_bounds.hi,
                "z{z} {x}/{y}: the mesh declares [{:.1}, {:.1}] m but the node's box is \
                 fitted over [{:.1}, {:.1}] m — the box does not contain its own geometry",
                mlo * 1.0e6,
                mhi * 1.0e6,
                node_bounds.lo * 1.0e6,
                node_bounds.hi * 1.0e6
            );
        }
    }
    println!(
        "  [D1 vs I-1'] {checked} tiles, tightest slack {worst_slack_m:.1} m, \
         node interval {:.2}x the mesh interval",
        inflation / checked as f64
    );
}

// ── the sweep ────────────────────────────────────────────────────────────────────

/// **The acceptance criterion for D1 and D2**: no node is culled while its own mesh has a
/// vertex on screen and off the limb.
///
/// Also the file's false-positive measurement. Both models are run over the same poses so
/// the cost of the bigger boxes is a delta rather than a number with no scale; the flat
/// column is measured with the *flat* mesh, i.e. it is what this engine already ships.
///
/// Note the FP definition here is not `culling::sweep`'s: that one samples a tile's
/// ground area against an ellipsoid oracle, this one asks whether any of the tile's own
/// mesh vertices is visible. They are not comparable numbers, which is exactly why the
/// flat model is measured the same way in the same run.
#[test]
fn terrain_sweep_has_no_false_negatives() {
    let config = sweep_config();
    let fields = LevelFields::new();
    let poses = sweep_poses();

    let mut terrain = PoseTally::default();
    let mut flat = PoseTally::default();
    let mut fn_examples: Vec<String> = Vec::new();

    for p in &poses {
        let (frustum, oracle) = frustum_for(p);
        let cam = HorizonCamera::new(frustum.eye);

        // ── terrain ──
        //
        // Fill, refresh, update — in that order and once more at the end, so that by the
        // time the tree is read every node in it has been fitted against data rather than
        // against the interval it inherited at birth. See `fill_cache`.
        let mut heights = HeightTileManager::new(&config);
        let mut qt = QuadtreeManager::<Heightfield>::for_surface();
        qt.max_zoom = SWEEP_MAX_ZOOM;
        for _ in 0..UPDATE_ITERATIONS {
            for root in qt.roots.iter() {
                fill_cache(root, &mut heights, &fields);
            }
            qt.refresh_extras(&bounds_source(&heights));
            qt.update(&frustum);
        }
        for root in qt.roots.iter() {
            fill_cache(root, &mut heights, &fields);
        }
        qt.refresh_extras(&bounds_source(&heights));

        let mut ids: Vec<(TileId, bool)> = Vec::new();
        let mut collect = |id: TileId, culled: bool, _t: &mut PoseTally| ids.push((id, culled));
        for root in qt.roots.iter() {
            walk(root, &mut terrain, &mut collect);
        }
        drop(collect);

        for (id, culled) in ids {
            let Ok(patch) = HeightPatch::sample(&mut heights, id, SEGMENTS, 1.0) else {
                continue;
            };
            let mesh = TileMesh::generate_on::<Heightfield>(&id, SEGMENTS, &patch);
            let mut visible = 0usize;
            let mut marginal = 0usize;
            for q in mesh_points(&mesh) {
                match classify_vertex(&oracle, &cam, q) {
                    VertexVerdict::Visible => visible += 1,
                    VertexVerdict::Marginal => marginal += 1,
                    VertexVerdict::Hidden => {}
                }
            }
            if culled {
                terrain.marginal_vertices += marginal;
                if visible > 0 {
                    terrain.false_negatives += 1;
                    if fn_examples.len() < 20 {
                        fn_examples.push(format!(
                            "{} lat={:.1} lon={:.1} alt={:.0}m pitch={:.0}: z{} {}/{} culled \
                             with {visible} visible mesh vertices",
                            p.sweep, p.lat_deg, p.lon_deg, p.alt_m, p.pitch_deg, id.z, id.x, id.y
                        ));
                    }
                }
            } else if visible == 0 && marginal == 0 {
                terrain.false_positive_leaves += 1;
            }
        }

        // ── flat, same poses, same measurement ──
        let mut fqt = QuadtreeManager::<Ellipsoid>::new();
        fqt.max_zoom = SWEEP_MAX_ZOOM;
        for _ in 0..UPDATE_ITERATIONS {
            fqt.update(&frustum);
        }
        for (id, _, _) in fqt.get_visible_tiles() {
            flat.visible_leaves += 1;
            let mesh = TileMesh::generate(&id, SEGMENTS);
            let any = mesh_points(&mesh)
                .into_iter()
                .any(|q| !matches!(classify_vertex(&oracle, &cam, q), VertexVerdict::Hidden));
            if !any {
                flat.false_positive_leaves += 1;
            }
        }
    }

    let fp_terrain = terrain.false_positive_leaves as f64 / terrain.visible_leaves.max(1) as f64;
    let fp_flat = flat.false_positive_leaves as f64 / flat.visible_leaves.max(1) as f64;

    println!("  [D1/D2 sweep] {} poses", poses.len());
    println!(
        "    terrain: {} nodes visited, {} culled, {} visible leaves",
        terrain.nodes_visited, terrain.nodes_culled, terrain.visible_leaves
    );
    println!(
        "    false negatives: {}   (marginal vertices excluded: {})",
        terrain.false_negatives, terrain.marginal_vertices
    );
    println!(
        "    false positives: terrain {}/{} = {:.2} %   flat {}/{} = {:.2} %   delta {:+.2} pts",
        terrain.false_positive_leaves,
        terrain.visible_leaves,
        100.0 * fp_terrain,
        flat.false_positive_leaves,
        flat.visible_leaves,
        100.0 * fp_flat,
        100.0 * (fp_terrain - fp_flat),
    );
    println!(
        "    visible-leaf count: terrain {} vs flat {} ({:+.1} %)",
        terrain.visible_leaves,
        flat.visible_leaves,
        100.0 * (terrain.visible_leaves as f64 / flat.visible_leaves.max(1) as f64 - 1.0)
    );

    assert!(
        terrain.nodes_culled > 1_000 && terrain.visible_leaves > 1_000,
        "the sweep did not exercise the culler: {} culled, {} kept",
        terrain.nodes_culled,
        terrain.visible_leaves
    );
    assert_eq!(
        terrain.false_negatives,
        0,
        "D1/D2 lost geometry that is on screen and off the limb:\n  {}",
        fn_examples.join("\n  ")
    );
}
