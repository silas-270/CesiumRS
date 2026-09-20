//! Phase C acceptance for `docs/terrain-plan.md` §6: relief (C1), normals (C2),
//! the derived skirt (C3), and the grid-density measurement (C4).
//!
//! **Nothing here touches the network.** The relief and normal tests run on a
//! synthetic field built in this file; the C4 measurement runs on the tiles committed
//! under `assets/terrain_fixtures/` (see the README there and
//! [`super::test_height_tiles`]).
//!
//! # Why the I-1′ guard lives here and not in `culling::`
//!
//! `culling::` is the gate, and the gate's contract is that it reports **32 passed, 0
//! failed, 1 ignored** unchanged across every phase of this plan — that is how "flat
//! mode must not regress, and must not be re-pinned" is enforced. Adding a test to it
//! changes the number that is being held fixed. So `test_generated_mesh_has_no_positive_altitude`
//! stays exactly as it is, in the gate, saying the stronger and still-true thing about
//! `Ellipsoid`; the weaker statement that survives relief is checked here, for both
//! models, and is what Phase D will build its bounding volumes on.

use std::sync::Arc;

use cesium_engine::globe::geometry::TileMesh;
use cesium_engine::globe::quadtree::{Ellipsoid, TileId};
use cesium_engine::globe::terrain::height_tile::{HEIGHT_TILE_DIM, HEIGHT_TILE_TEXELS};
use cesium_engine::globe::terrain::{
    HeightPatch, HeightTile, HeightTileManager, Heightfield, PatchStatus,
};
use cesium_engine::globe::tiles::config::{OceanPolicy, TerrainConfig, TileEngineConfig};
use glam::DVec3;

use crate::testing::culling::geodesy;

/// `8·2⁻²⁴`, the f32 rounding bound — the same constant, for the same reason, as in
/// `culling::test_tile_bounds`: mesh vertices are f32 offsets from an f64 centre, so
/// no agreement finer than this is observable.
const F32_BOUND: f64 = 8.0 * 5.960_464_477_539_063e-8;

const SEGMENTS: u32 = 8;

fn terrain_manager() -> HeightTileManager {
    HeightTileManager::new(&TileEngineConfig {
        terrain: TerrainConfig {
            enabled: true,
            ..TerrainConfig::default()
        },
        ..TileEngineConfig::default()
    })
}

/// A deterministic, deliberately rough synthetic elevation tile, peaking near 8 800 m.
///
/// Rough on purpose: a flat field would let every bounds assertion below pass
/// vacuously. Two incommensurable frequencies plus a diagonal ridge, so no grid
/// spacing this test uses can land on a stationary point of it.
fn rough_field() -> Arc<HeightTile> {
    let mut data = Box::new([0i16; HEIGHT_TILE_TEXELS]);
    for y in 0..HEIGHT_TILE_DIM {
        for x in 0..HEIGHT_TILE_DIM {
            let a = (x as f64 * 0.31).sin() * (y as f64 * 0.17).cos();
            let b = (x as f64 * 0.041 + y as f64 * 0.067).sin();
            data[y * HEIGHT_TILE_DIM + x] = (4400.0 + 2800.0 * a + 1600.0 * b) as i16;
        }
    }
    Arc::new(HeightTile::from_samples(data))
}

/// A field that is exactly linear in `x` (`h = 4·x` metres), so its east gradient has
/// a closed form and a wrong sign or a halved slope is a plain numeric mismatch.
fn x_ramp(scale: f64) -> Arc<HeightTile> {
    let mut data = Box::new([0i16; HEIGHT_TILE_TEXELS]);
    for y in 0..HEIGHT_TILE_DIM {
        for x in 0..HEIGHT_TILE_DIM {
            data[y * HEIGHT_TILE_DIM + x] = (x as f64 * scale) as i16;
        }
    }
    Arc::new(HeightTile::from_samples(data))
}

/// The tiles the invariants are checked over: both pole rows, the antimeridian
/// columns, the middle, at every zoom. The same spread `culling::test_tile_bounds`
/// uses, restated here so this file does not reach into the gate's internals.
fn sample_tiles() -> Vec<TileId> {
    let mut out = Vec::new();
    for z in 1..=20u8 {
        let n = 1_u32 << z;
        let mid = n / 2;
        for (x, y) in [
            (0, 0),
            (n - 1, 0),
            (0, n - 1),
            (n - 1, n - 1),
            (mid, 0),
            (mid, n - 1),
            (0, mid),
            (n - 1, mid),
            (mid, mid),
            (mid.saturating_sub(1), mid.saturating_sub(1)),
        ] {
            out.push(TileId { z, x, y });
        }
    }
    out
}

/// Altitude of `p` above the ellipsoid, in **megametres**.
///
/// Drops the point onto the ellipsoid along the gradient at the point itself. That
/// footpoint is not exactly the one the mesh displaced from — the ellipsoid normal
/// turns slightly along its own ray — but the discrepancy is second order in the
/// altitude and is under a tenth of a millimetre at 9 km, three orders below the f32
/// quantum of the vertex it is measuring.
fn altitude_mm(p: DVec3) -> f64 {
    let (lat, lon) = geodesy::dvec3_to_lat_lon(p);
    let foot = geodesy::lon_lat_to_ecef(lon, lat);
    (p - foot).dot(geodesy::ellipsoid_normal(p))
}

/// Builds both meshes for `id` over the given field.
fn both_meshes(
    heights: &mut HeightTileManager,
    id: TileId,
    field: &Arc<HeightTile>,
    exaggeration: f32,
) -> (TileMesh, TileMesh, HeightPatch) {
    heights.insert_ready(heights.source_tile_for(id), field.clone());
    let patch = HeightPatch::sample(heights, id, SEGMENTS, exaggeration)
        .unwrap_or_else(|s| panic!("z={} x={} y={}: patch not ready: {s:?}", id.z, id.x, id.y));
    let flat = TileMesh::generate(&id, SEGMENTS);
    let relief = TileMesh::generate_on::<Heightfield>(&id, SEGMENTS, &patch);
    (flat, relief, patch)
}

// ── I-1′ ─────────────────────────────────────────────────────────────────────

/// **Invariant I-1′** (`docs/terrain-plan.md` §6). Every vertex of a tile's mesh lies
/// within the `[h_min, h_max]` interval that mesh declares — for **both** surface
/// models.
///
/// This is the bridge that makes Phase D provable at all. D1 fits a node's oriented
/// box over the declared interval and D2 fits a bounding sphere over the same; both
/// are sound only if the geometry actually drawn stays inside it. Until Phase C
/// nothing could check it, because the interval was the constant `[−skirt, 0]` and
/// I-1 was the whole statement.
///
/// The `Ellipsoid` half is deliberately redundant with the gate's
/// `test_generated_mesh_has_no_positive_altitude`: the point of I-1′ is that it is
/// *one* statement covering both models, so a future third model cannot quietly
/// declare bounds it does not honour.
#[test]
fn generated_meshes_stay_within_their_declared_height_bounds() {
    let mut heights = terrain_manager();
    let field = rough_field();

    let mut failures: Vec<String> = Vec::new();
    let mut checked = 0usize;
    let mut worst_excess_mm = f64::NEG_INFINITY;
    let mut widest_span_m = 0.0_f64;

    for id in sample_tiles() {
        let (flat, relief, _) = both_meshes(&mut heights, id, &field, 1.0);

        let [lo, hi] = relief.height_bounds;
        widest_span_m = widest_span_m.max((hi - lo) * 1.0e6);
        assert!(
            hi > lo,
            "z={} x={} y={}: the relief mesh declared the degenerate interval \
             [{lo}, {hi}] — every assertion below would pass vacuously",
            id.z,
            id.x,
            id.y
        );

        for (model, mesh) in [("Ellipsoid", &flat), ("Heightfield", &relief)] {
            let [lo, hi] = mesh.height_bounds;
            let center = DVec3::from_array(mesh.center_f64);
            for v in &mesh.vertices {
                let rel = DVec3::new(
                    v.position[0] as f64,
                    v.position[1] as f64,
                    v.position[2] as f64,
                );
                let alt = altitude_mm(center + rel);
                let tol = F32_BOUND * (center.length() + rel.length());

                checked += 1;
                let excess = (lo - alt).max(alt - hi);
                worst_excess_mm = worst_excess_mm.max(excess);
                if excess > tol {
                    failures.push(format!(
                        "{model} z={} x={} y={}: vertex at {:.3} m is outside the \
                         declared [{:.3}, {:.3}] m by {:.3} m (tolerance {:.3} m)",
                        id.z,
                        id.x,
                        id.y,
                        alt * 1.0e6,
                        lo * 1.0e6,
                        hi * 1.0e6,
                        excess * 1.0e6,
                        tol * 1.0e6
                    ));
                }
            }
        }
    }

    println!(
        "  I-1': {checked} vertices over {} tiles x 2 models; worst excess \
         {:.6} m, widest declared span {widest_span_m:.1} m",
        sample_tiles().len(),
        worst_excess_mm * 1.0e6
    );
    assert!(
        failures.is_empty(),
        "{} vertices fall outside the height interval their own mesh declares. \
         I-1' is broken, and with it every bounding volume Phase D fits over that \
         interval.\n  {}",
        failures.len(),
        failures
            .iter()
            .take(5)
            .cloned()
            .collect::<Vec<_>>()
            .join("\n  ")
    );
}

/// Relief is **radial**: it may move a vertex up or down and must not move it in
/// longitude or latitude.
///
/// `culling::test_tile_bounds::test_generated_mesh_stays_inside_the_culling_rectangle`
/// asserts this for the flat mesh and keeps passing unchanged. This is the same
/// statement for the relief mesh, phrased as a direct flat-versus-relief comparison so
/// that the obvious way to get it wrong — displacing along the *terrain* normal, which
/// tilts with the slope — is caught as a sideways component of the displacement
/// rather than as a near-miss against a rectangle.
///
/// "Radial" here means **along the ellipsoid normal**, which is the direction the
/// existing skirts already use and is *not* the direction to the ellipsoid's centre:
/// the two differ by up to 0.19° at mid-latitude, which is why the displacement is
/// measured against the normal at the undisplaced vertex rather than against its
/// position vector.
#[test]
fn relief_moves_vertices_only_radially() {
    let mut heights = terrain_manager();
    let field = rough_field();

    let mut worst_ratio = 0.0_f64;
    let mut worst_lateral_m = 0.0_f64;
    let mut worst_ctx = String::new();
    let mut failures = 0usize;

    for id in sample_tiles() {
        let (flat, relief, _) = both_meshes(&mut heights, id, &field, 1.0);
        assert_eq!(flat.vertices.len(), relief.vertices.len());
        assert_eq!(flat.center_f64, relief.center_f64);
        let center = DVec3::from_array(flat.center_f64);

        for (a, b) in flat.vertices.iter().zip(relief.vertices.iter()) {
            let ra = DVec3::new(
                a.position[0] as f64,
                a.position[1] as f64,
                a.position[2] as f64,
            );
            let rb = DVec3::new(
                b.position[0] as f64,
                b.position[1] as f64,
                b.position[2] as f64,
            );
            let (pa, pb) = (center + ra, center + rb);
            let d = pb - pa;
            if d.length() < 1.0e-15 {
                continue;
            }
            // The ellipsoid normal at the **surface** point, which is what the mesh
            // displaces along — and which the flat mesh already carries as its vertex
            // normal. Deliberately not `ellipsoid_normal(pa)`: the normal turns along
            // its own ray, and a z1 skirt vertex is 250 km down it.
            let up =
                DVec3::new(a.normal[0] as f64, a.normal[1] as f64, a.normal[2] as f64).normalize();
            let lateral = (d - up * d.dot(up)).length();

            // Both endpoints are f32-quantised offsets from the same f64 centre, and
            // `up` is an f32 direction, so the sideways component cannot be resolved
            // below the f32 quantum of either. Neither term is negligible: a pole-cap
            // vertex sits ~550 km from its tile centre at any zoom, and a coarse
            // tile's skirt displacement is hundreds of kilometres long.
            let tol = F32_BOUND * (center.length() + ra.length() + rb.length() + d.length());
            let ratio = lateral / tol;
            if ratio > worst_ratio {
                worst_ratio = ratio;
                worst_lateral_m = lateral * 1.0e6;
                worst_ctx = format!("z={} x={} y={}", id.z, id.x, id.y);
            }
            if lateral > tol {
                failures += 1;
            }
        }
    }

    println!(
        "  worst lateral component of a relief displacement: {worst_lateral_m:.4} m, \
         {worst_ratio:.3} of its own f32 floor  [{worst_ctx}]"
    );
    assert_eq!(
        failures, 0,
        "relief moved {failures} vertices sideways by more than the f32 quantum \
         (worst {worst_lateral_m:.4} m, {worst_ratio:.3}x the floor): the displacement \
         is not purely radial, so the drawn mesh has left the rectangle the culler \
         tests. {worst_ctx}"
    );
}

// ── C1: relief, and the exaggeration applied exactly once ────────────────────

/// The whole of C1 in one assertion: the mesh's vertices sit at the sampled height.
#[test]
fn a_relief_vertex_sits_at_the_sampled_height() {
    let mut heights = terrain_manager();
    let id = TileId {
        z: 12,
        x: 2172,
        y: 1433,
    };
    let field = rough_field();
    let (_, relief, patch) = both_meshes(&mut heights, id, &field, 1.0);

    let grid = SEGMENTS as usize + 3;
    let center = DVec3::from_array(relief.center_f64);
    // An interior vertex, away from every edge and skirt.
    for (r, c) in [(1usize, 1usize), (3, 5), (SEGMENTS as usize + 1, 4)] {
        let v = &relief.vertices[r * grid + c];
        let p = center
            + DVec3::new(
                v.position[0] as f64,
                v.position[1] as f64,
                v.position[2] as f64,
            );
        let expected = heights
            .peek_height_at(
                id,
                (c as f64 - 1.0) / SEGMENTS as f64,
                (r as f64 - 1.0) / SEGMENTS as f64,
            )
            .unwrap();
        let got = altitude_mm(p);
        assert!(
            (got - expected).abs() < 1.0e-6,
            "vertex ({r},{c}): {:.3} m, height field says {:.3} m",
            got * 1.0e6,
            expected * 1.0e6
        );
    }
    // …and the bounds the mesh declares really are the patch's own.
    assert_eq!(relief.height_bounds[1], patch.height_bounds()[1]);
}

/// §6 C1: "apply vertical exaggeration **here and nowhere else**".
///
/// Doubling the exaggeration must double every height *and every bound derived from
/// it*, exactly — which is the property that lets Phase D stay consistent with the
/// knob without knowing it exists. A factor applied twice (once in `height_at`, once
/// in the model) would show as 4x here; one applied to the vertices but not to the
/// bounds would break the equality on the second line.
#[test]
fn exaggeration_scales_heights_and_bounds_exactly_once() {
    let mut heights = terrain_manager();
    let id = TileId {
        z: 12,
        x: 2172,
        y: 1433,
    };
    let field = rough_field();
    heights.insert_ready(id, field.clone());

    let one = HeightPatch::sample(&mut heights, id, SEGMENTS, 1.0).unwrap();
    let two = HeightPatch::sample(&mut heights, id, SEGMENTS, 2.0).unwrap();

    for (a, b) in one.height_bounds().iter().zip(two.height_bounds().iter()) {
        assert!((b - 2.0 * a).abs() < 1.0e-15, "{b} != 2 * {a}");
    }

    // Phase B must not be exaggerating as well: `height_at` is the raw field.
    let raw = heights.peek_height_at(id, 0.5, 0.5).unwrap();
    let mesh = TileMesh::generate_on::<Heightfield>(&id, SEGMENTS, &two);
    let center = DVec3::from_array(mesh.center_f64);
    let grid = SEGMENTS as usize + 3;
    let mid = (SEGMENTS as usize / 2) + 1;
    let v = &mesh.vertices[mid * grid + mid];
    let p = center
        + DVec3::new(
            v.position[0] as f64,
            v.position[1] as f64,
            v.position[2] as f64,
        );
    let expected = 2.0 * heights.peek_height_at(id, 0.5, 0.5).unwrap();
    assert!((altitude_mm(p) - expected).abs() < 1.0e-6);
    assert!(raw > 0.0, "the fixture field must not be flat here");
}

/// Skirts hang inward from the **edge** height, not from the halo outside the tile,
/// and the pole caps get no skirt at all — the flat mesh's two rules, preserved.
#[test]
fn skirts_hang_from_the_edge_height_and_poles_keep_none() {
    let mut heights = terrain_manager();
    let field = rough_field();
    // A north-pole-row tile, so the cap rule is exercised.
    let id = TileId { z: 4, x: 8, y: 0 };
    let (_, relief, patch) = both_meshes(&mut heights, id, &field, 1.0);

    let grid = SEGMENTS as usize + 3;
    let center = DVec3::from_array(relief.center_f64);
    let alt_at = |r: usize, c: usize| {
        let v = &relief.vertices[r * grid + c];
        altitude_mm(
            center
                + DVec3::new(
                    v.position[0] as f64,
                    v.position[1] as f64,
                    v.position[2] as f64,
                ),
        )
    };

    let skirt = patch.skirt() as f64;
    // The floor: vertex positions are f32 offsets from an f64 centre at ~6.4 Mm.
    let tol = F32_BOUND * 2.0 * cesium_engine::globe::geometry::EARTH_RADIUS_A_F64;
    for c in 1..=(SEGMENTS as usize + 1) {
        // Row 0 is the north pole cap for y == 0: same height as the edge, no skirt.
        assert!(
            (alt_at(0, c) - alt_at(1, c)).abs() < tol,
            "pole cap at col {c} is not at the edge height"
        );
        // The south skirt row hangs exactly one skirt below its edge.
        let edge = alt_at(SEGMENTS as usize + 1, c);
        let hang = alt_at(SEGMENTS as usize + 2, c);
        assert!(
            (edge - hang - skirt).abs() < tol,
            "south skirt at col {c}: {edge} - {hang} != {skirt}"
        );
    }
}

// ── C2: normals ──────────────────────────────────────────────────────────────

/// A flat height field must give back exactly the ellipsoid normal — the central
/// difference has to vanish, not merely be small.
#[test]
fn a_flat_field_reproduces_the_ellipsoid_normal() {
    let mut heights = terrain_manager();
    let id = TileId { z: 6, x: 33, y: 22 };
    let flat_field = Arc::new(HeightTile::from_samples(Box::new(
        [1234i16; HEIGHT_TILE_TEXELS],
    )));
    let (flat, relief, _) = both_meshes(&mut heights, id, &flat_field, 1.0);

    let mut worst = 0.0_f64;
    for (a, b) in flat.vertices.iter().zip(relief.vertices.iter()) {
        for k in 0..3 {
            worst = worst.max((a.normal[k] as f64 - b.normal[k] as f64).abs());
        }
    }
    assert!(
        worst < 1.0e-12,
        "a constant height field perturbed the normal by {worst:.3e}"
    );
}

/// The east gradient, against a closed form.
///
/// The field is `h = scale·x` metres over a 256-texel tile, so on a tile spanning
/// `W` metres of ground east-west the slope is `256·scale / W`. The normal must tilt
/// **west** by `atan(slope)` — uphill is east, so the normal leans away from it.
/// A halved slope (the failure mode a clamped gradient halo produces) or a flipped
/// sign both show up as a plain numeric mismatch here.
#[test]
fn the_east_gradient_matches_a_closed_form_slope() {
    let mut heights = terrain_manager();
    // A low-latitude tile so east/north are well conditioned, and one whose own data
    // is the source, so the halo is exercised at its hardest.
    let id = TileId {
        z: 8,
        x: 128,
        y: 128,
    };
    let field = x_ramp(4.0);
    heights.insert_ready(id, field);
    let patch = HeightPatch::sample(&mut heights, id, SEGMENTS, 1.0).unwrap();
    let mesh = TileMesh::generate_on::<Heightfield>(&id, SEGMENTS, &patch);

    let b = cesium_engine::globe::quadtree::tile_bounds(&id);
    let grid = SEGMENTS as usize + 3;
    let mid_row = SEGMENTS as usize / 2 + 1;

    // Ground width of the tile at the sampled row's latitude, in metres.
    let lat = cesium_engine::globe::quadtree::web_mercator_y_to_lat_f64(
        id.y as f64 + (mid_row as f64 - 1.0) / SEGMENTS as f64,
        id.z,
    );
    let width_m = (b.lon_max - b.lon_min).to_radians()
        * cesium_engine::globe::geometry::EARTH_RADIUS_A_F64
        * 1.0e6
        * lat.to_radians().cos();
    // 256 texels of ramp across the tile, 4 m per texel, and the bilinear sampler's
    // half-texel border means the ramp actually spans 255 texels of rise.
    let slope = 255.0 * 4.0 / width_m;
    let expected_tilt_deg = slope.atan().to_degrees();

    // An interior column, where the difference is genuinely central.
    let c = SEGMENTS as usize / 2 + 1;
    let v = &mesh.vertices[mid_row * grid + c];
    let n = DVec3::new(v.normal[0] as f64, v.normal[1] as f64, v.normal[2] as f64).normalize();

    let lon = b.lon_min + (c as f64 - 1.0) / SEGMENTS as f64 * (b.lon_max - b.lon_min);
    let theta = lon.to_radians();
    let east = DVec3::new(-theta.sin(), 0.0, -theta.cos());
    let up = geodesy::ellipsoid_normal(geodesy::lon_lat_to_ecef(lon, lat));

    let tilt_deg = n.dot(up).clamp(-1.0, 1.0).acos().to_degrees();
    assert!(
        (tilt_deg - expected_tilt_deg).abs() < 0.02 * expected_tilt_deg.max(1.0e-6),
        "normal tilt {tilt_deg:.6} deg, closed form {expected_tilt_deg:.6} deg \
         (slope {slope:.3e} over {width_m:.0} m)"
    );
    assert!(
        n.dot(east) < 0.0,
        "the normal leans east on an east-uphill slope — the gradient sign is flipped"
    );
}

/// C2's edge treatment, stated as a test.
///
/// The gradient stencil reaches one grid step outside the tile. When the height source
/// is an **ancestor** — the normal case, and the only case below z15 — that halo is
/// real neighbour data, so the edge vertex gets a genuine central difference and its
/// normal must agree with the one the neighbouring tile computes for the *same* ground
/// position. Without the halo the edge would read its own clamped value over a
/// two-step baseline and report half the slope, which is exactly the visible seam this
/// test exists to prevent.
#[test]
fn edge_normals_agree_across_a_shared_tile_boundary() {
    let mut heights = terrain_manager();
    // Two horizontally adjacent z18 tiles under one z15 ancestor: below the source's
    // depth ceiling, so that ancestor *is* the preferred source for both, the shared
    // edge is interior to it, and the halo is the real neighbour's ground. This is the
    // normal case, not a contrived one — everything past z15 is answered this way.
    let ancestor = TileId {
        z: 15,
        x: 17_361,
        y: 11_269,
    };
    let left = TileId {
        z: 18,
        x: 17_361 * 8 + 3,
        y: 11_269 * 8 + 5,
    };
    let right = TileId {
        z: 18,
        x: 17_361 * 8 + 4,
        y: 11_269 * 8 + 5,
    };
    heights.insert_ready(ancestor, rough_field());

    let mesh_of = |h: &mut HeightTileManager, id: TileId| {
        let patch = HeightPatch::sample(h, id, SEGMENTS, 1.0).unwrap();
        assert_eq!(patch.source(), ancestor, "the ancestor must be the source");
        assert_eq!(
            patch.halo_valid(),
            [true; 4],
            "an ancestor source must make every halo side real"
        );
        TileMesh::generate_on::<Heightfield>(&id, SEGMENTS, &patch)
    };
    let lm = mesh_of(&mut heights, left);
    let rm = mesh_of(&mut heights, right);

    let grid = SEGMENTS as usize + 3;
    let mut worst = 0.0_f64;
    for r in 1..=(SEGMENTS as usize + 1) {
        // `left`'s east edge and `right`'s west edge are the same ground line.
        let a = &lm.vertices[r * grid + (SEGMENTS as usize + 1)];
        let b = &rm.vertices[r * grid + 1];
        let na = DVec3::new(a.normal[0] as f64, a.normal[1] as f64, a.normal[2] as f64);
        let nb = DVec3::new(b.normal[0] as f64, b.normal[1] as f64, b.normal[2] as f64);
        worst = worst.max(na.dot(nb).clamp(-1.0, 1.0).acos().to_degrees());
    }
    println!("  worst normal disagreement across a shared edge: {worst:.4} deg");
    assert!(
        worst < 0.05,
        "edge normals disagree by {worst:.4} deg across a shared boundary — the \
         gradient halo is not supplying the neighbour's heights"
    );
}

// ── C3: the derived skirt ────────────────────────────────────────────────────

/// The flat model's skirt is untouched: `0.5 / 2^z`, bit for bit.
#[test]
fn the_flat_skirt_formula_is_unchanged() {
    use cesium_engine::globe::quadtree::SurfaceModel;
    for z in 0..=20u8 {
        let id = TileId { z, x: 0, y: 0 };
        assert_eq!(
            Ellipsoid::skirt_depth(&id, 16, &()),
            0.5 / 2.0_f32.powi(z as i32),
            "z = {z}"
        );
    }
}

/// C3's claim: the skirt is at least as deep as the crack it has to hide.
///
/// The crack at an LOD boundary is the gap between this tile's edge and the straight
/// line a coarser neighbour draws across the same edge. Measured here independently of
/// the derivation — by coarsening the tile's own edge heights by 2 and by 4 and taking
/// the worst deviation — and compared against what the patch actually chose.
#[test]
fn the_derived_skirt_covers_the_edge_mismatch_it_is_derived_from() {
    let mut heights = terrain_manager();
    let field = rough_field();

    for id in [
        TileId { z: 6, x: 33, y: 22 },
        TileId {
            z: 12,
            x: 2172,
            y: 1433,
        },
        TileId {
            z: 15,
            x: 17_361,
            y: 11_269,
        },
    ] {
        heights.insert_ready(heights.source_tile_for(id), field.clone());
        let patch = HeightPatch::sample(&mut heights, id, SEGMENTS, 1.0).unwrap();

        // The edge heights, straight out of the height field rather than out of the
        // patch, so the two derivations are independent.
        let n = SEGMENTS as usize;
        let mut worst = 0.0_f64;
        for edge in 0..4 {
            let h: Vec<f64> = (0..=n)
                .map(|i| {
                    let t = i as f64 / n as f64;
                    let (u, v) = match edge {
                        0 => (t, 0.0),
                        1 => (t, 1.0),
                        2 => (0.0, t),
                        _ => (1.0, t),
                    };
                    heights.peek_height_at(id, u, v).unwrap()
                })
                .collect();
            for k in [2usize, 4] {
                for i in 0..=n {
                    let i0 = (i / k) * k;
                    let i1 = (i0 + k).min(n);
                    if i1 == i0 {
                        continue;
                    }
                    let t = (i - i0) as f64 / (i1 - i0) as f64;
                    let interp = h[i0] + (h[i1] - h[i0]) * t;
                    worst = worst.max((h[i] - interp).abs());
                }
            }
        }

        let skirt = patch.skirt() as f64;
        println!(
            "  z={:2} skirt {:8.1} m   measured crack {:8.1} m   (flat formula {:8.1} m)",
            id.z,
            skirt * 1.0e6,
            worst * 1.0e6,
            (0.5 / 2.0_f64.powi(id.z as i32)) * 1.0e6
        );
        assert!(
            skirt >= worst * (1.0 - 1.0e-9),
            "z={}: skirt {:.3} m is shallower than the {:.3} m crack it must hide",
            id.z,
            skirt * 1.0e6,
            worst * 1.0e6
        );
    }
}

/// Even with no relief at all the skirt must not collapse to zero: a coarser
/// neighbour's edge is still a *chord* of this one's arc, and the sagitta of that
/// chord is a real crack.
///
/// It is also the term that dominates at coarse levels, which is the reassuring half
/// of the derivation: at z2 it lands on the same order as the hand-chosen
/// `0.5 / 2^z`, so that constant was never arbitrary — it was a curvature estimate,
/// and it was a good one. From about z5 down the two diverge fast, because the
/// constant falls as `2^-z` while the real sagitta falls as `4^-z`: at z15 the derived
/// skirt is four orders of magnitude smaller. Terrain is what puts the difference
/// back, and only where there is terrain.
#[test]
fn an_ocean_tile_keeps_a_curvature_only_skirt() {
    let mut heights = terrain_manager();
    let ocean = Arc::new(HeightTile::from_samples(Box::new(
        [0i16; HEIGHT_TILE_TEXELS],
    )));

    println!("  all-ocean tiles, segments = {SEGMENTS}:");
    for z in [2u8, 5, 8, 12, 15] {
        let id = TileId {
            z,
            x: 1 << (z - 1),
            y: 1 << (z - 1),
        };
        heights.insert_ready(id, ocean.clone());
        let patch = HeightPatch::sample(&mut heights, id, SEGMENTS, 1.0).unwrap();
        let skirt_m = patch.skirt() as f64 * 1.0e6;
        let flat_m = 0.5 / 2.0_f64.powi(z as i32) * 1.0e6;
        println!(
            "    z={z:2}: derived {skirt_m:10.3} m   flat formula {flat_m:10.1} m   \
             ratio {:.3}",
            skirt_m / flat_m
        );
        assert!(
            skirt_m > 0.0 && skirt_m.is_finite(),
            "z={z}: a flat tile still has a curvature crack"
        );
        if z >= 5 {
            assert!(
                skirt_m < flat_m,
                "z={z}: the derived skirt {skirt_m} m is not below the flat formula's \
                 {flat_m} m on a tile with no relief at all"
            );
        }
    }
}

// ── B2 x C1: unknown heights must defer, never flatten ───────────────────────

/// §5 B2's "unknown is not sea level", enforced at the place it matters.
///
/// A tile whose heights have not arrived yields `Pending`, and the tile system's mesh
/// loop skips it rather than building a flat mesh nothing would later rebuild. A tile
/// whose whole ancestor chain has failed yields `Unavailable`, which *is* a
/// terminating answer — otherwise a dead source would stall the mesh pipeline forever.
#[test]
fn an_unloaded_tile_defers_the_mesh_instead_of_flattening_it() {
    let mut heights = terrain_manager();
    let id = TileId {
        z: 12,
        x: 2172,
        y: 1433,
    };
    assert_eq!(
        HeightPatch::sample(&mut heights, id, SEGMENTS, 1.0).unwrap_err(),
        PatchStatus::Pending
    );

    heights.insert_ready(id, rough_field());
    assert!(HeightPatch::sample(&mut heights, id, SEGMENTS, 1.0).is_ok());
}

/// The mesh records which height tile it was built from, which is the whole of what
/// Phase E2 needs out of Phase C: past z15 the answer is an ancestor, and the mesh is
/// therefore *not* a pure function of its own `TileId`.
#[test]
fn a_relief_mesh_records_the_height_tile_it_was_built_from() {
    let mut heights = terrain_manager();
    let ancestor = TileId {
        z: 15,
        x: 17_361,
        y: 11_269,
    };
    let deep = TileId {
        z: 20,
        x: 17_361 * 32 + 21,
        y: 11_269 * 32 + 25,
    };
    heights.insert_ready(ancestor, rough_field());

    let patch = HeightPatch::sample(&mut heights, deep, SEGMENTS, 1.0).unwrap();
    let mesh = TileMesh::generate_on::<Heightfield>(&deep, SEGMENTS, &patch);
    assert_eq!(mesh.height_source, Some(ancestor));

    // …and the flat mesh has none, so the key is unambiguous.
    assert_eq!(TileMesh::generate(&deep, SEGMENTS).height_source, None);
}

// ── C4: grid density, measured ───────────────────────────────────────────────

/// The mesh's own interpolated height at tile-local `(u, v)`, in metres.
///
/// Reproduces the triangulation `TileMesh::generate_on` emits — each grid cell split
/// by the diagonal from `(row+1, col)` to `(row, col+1)` — rather than bilinearly
/// interpolating the cell, because the error being measured is the error of the
/// surface that is actually drawn.
fn mesh_height_m(grid_h: &[f64], segments: u32, u: f64, v: f64) -> f64 {
    let n = segments as usize;
    let stride = n + 1;
    let gx = (u.clamp(0.0, 1.0) * n as f64).min(n as f64 - 1.0e-12);
    let gy = (v.clamp(0.0, 1.0) * n as f64).min(n as f64 - 1.0e-12);
    let (c, r) = (gx.floor() as usize, gy.floor() as usize);
    let (s, t) = (gx - c as f64, gy - r as f64);
    let at = |rr: usize, cc: usize| grid_h[rr.min(n) * stride + cc.min(n)];
    let (p00, p10, p01, p11) = (at(r, c), at(r, c + 1), at(r + 1, c), at(r + 1, c + 1));
    if s + t <= 1.0 {
        p00 + s * (p10 - p00) + t * (p01 - p00)
    } else {
        p11 + (1.0 - s) * (p01 - p11) + (1.0 - t) * (p10 - p11)
    }
}

/// **C4** — how much geometric error `mesh_segments` actually costs, measured over
/// the committed fixtures, against all 65 536 source samples of each tile.
///
/// `docs/terrain-plan.md` §6 C4 asks for this and explicitly does **not** ask for a
/// decision: the default stays 16 and the choice is made in Phase F against device
/// measurements, not here. What this produces is the table that Phase F will argue
/// from — max and RMS deviation in metres, with the vertex and index bytes each
/// density costs.
///
/// The one assertion is that **RMS** falls monotonically with refinement: if it did
/// not, the mesh would not be converging to the height field and the whole table would
/// be meaningless. The **max** is deliberately not asserted to fall, and the reason is
/// itself a C4 finding: the mesh point-samples the field, so refining moves the sample
/// points rather than averaging over them, and a summit that one grid straddles the
/// next can straddle almost as badly. On the Monterey fixture the max goes 29.3 → 24.5
/// → 25.4 m while the RMS goes 3.4 → 2.0 → 1.1 m.
#[test]
fn c4_grid_density_error_against_the_fixtures() {
    const FIXTURES: [(&str, TileId); 3] = [
        (
            "everest_z12_3037_1716.png",
            TileId {
                z: 12,
                x: 3037,
                y: 1716,
            },
        ),
        (
            "zugspitze_z12_2172_1433.png",
            TileId {
                z: 12,
                x: 2172,
                y: 1433,
            },
        ),
        (
            "monterey_coast_z12_661_1599.png",
            TileId {
                z: 12,
                x: 661,
                y: 1599,
            },
        ),
    ];
    const DENSITIES: [u32; 3] = [16, 32, 64];

    println!(
        "\n  | fixture | segments | max err (m) | RMS err (m) | skirt (m) | verts | \
         vbuf (B) | ibuf (B) |"
    );
    println!("  |---|--:|--:|--:|--:|--:|--:|--:|");

    for (name, id) in FIXTURES {
        let path = format!(
            "{}/assets/terrain_fixtures/{name}",
            env!("CARGO_MANIFEST_DIR")
        );
        let bytes = std::fs::read(&path).unwrap_or_else(|e| panic!("reading {path}: {e}"));
        let img = image::load_from_memory(&bytes).unwrap().to_rgba8();
        let (w, h) = (img.width(), img.height());
        let tile = Arc::new(
            cesium_engine::globe::terrain::decode_terrarium(
                w,
                h,
                &img.into_raw(),
                OceanPolicy::ClampToZero,
            )
            .unwrap(),
        );

        let mut prev: Option<(f64, f64)> = None;
        for segments in DENSITIES {
            let mut heights = terrain_manager();
            heights.insert_ready(id, tile.clone());
            let patch = HeightPatch::sample(&mut heights, id, segments, 1.0).unwrap();

            // The mesh's interior heights, in metres, exactly as the vertices carry
            // them: the patch interior is `vertex_altitude` for every non-skirt vertex.
            let n = segments as usize;
            let stride = n + 1;
            let grid_size = n + 3;
            let mut grid_h = vec![0.0f64; stride * stride];
            {
                // Rebuild the interior out of the mesh itself rather than trusting the
                // patch, so the measurement is of the drawn geometry.
                let mesh = TileMesh::generate_on::<Heightfield>(&id, segments, &patch);
                let center = DVec3::from_array(mesh.center_f64);
                for r in 0..=n {
                    for c in 0..=n {
                        let v = &mesh.vertices[(r + 1) * grid_size + (c + 1)];
                        let p = center
                            + DVec3::new(
                                v.position[0] as f64,
                                v.position[1] as f64,
                                v.position[2] as f64,
                            );
                        grid_h[r * stride + c] = altitude_mm(p) * 1.0e6;
                    }
                }
            }

            let mut max_err = 0.0f64;
            let mut sq = 0.0f64;
            for y in 0..HEIGHT_TILE_DIM {
                let v = (y as f64 + 0.5) / HEIGHT_TILE_DIM as f64;
                for x in 0..HEIGHT_TILE_DIM {
                    let u = (x as f64 + 0.5) / HEIGHT_TILE_DIM as f64;
                    let err = mesh_height_m(&grid_h, segments, u, v) - tile.sample(x, y) as f64;
                    max_err = max_err.max(err.abs());
                    sq += err * err;
                }
            }
            let rms = (sq / HEIGHT_TILE_TEXELS as f64).sqrt();

            let verts = grid_size * grid_size;
            let vbuf = verts * std::mem::size_of::<cesium_engine::globe::geometry::Vertex>();
            let ibuf = (n + 2) * (n + 2) * 6 * 2;
            println!(
                "  | {name} | {segments} | {max_err:.1} | {rms:.1} | {:.1} | {verts} | \
                 {vbuf} | {ibuf} |",
                patch.skirt() as f64 * 1.0e6
            );

            if let Some((_, pr)) = prev {
                assert!(
                    rms < pr,
                    "{name}: refining to {segments} segments did not reduce the RMS \
                     error ({rms:.1} m against {pr:.1} m) — the mesh is not converging \
                     to the height field"
                );
            }
            prev = Some((max_err, rms));
        }
    }
    println!();
}
