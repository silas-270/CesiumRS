//! Guards on the three assumptions the tile culling path is *licensed* by, rather
//! than on its output.
//!
//! The sweeps in [`super::test_globe_sweep`] measure what the culler does. These
//! measure whether it is still allowed to do it. Each one corresponds to a named
//! invariant in `docs/culling-math.md` §10.3, and each is written so that the person
//! who breaks the premise gets a failing test instead of a silent hole in the globe.

use cesium_engine::globe::geometry::{lon_lat_to_ecef_f64, TileMesh};
use cesium_engine::globe::quadtree::{
    tile_bounds, transform_to_scaled_space, HorizonCamera, TileId, TilePatch,
};
use glam::DVec3;

/// `8·2⁻²⁴` — the f32 rounding bound on a quantity of the given magnitude.
///
/// `TileMesh` stores vertex positions as f32 offsets from an f64 tile centre, so
/// this is the floor below which no amount of agreement between the culler and the
/// mesh can be observed. Every tolerance in this file is this bound times the
/// magnitude actually involved; none of them is a fudge factor.
const F32_BOUND: f64 = 8.0 * 5.960_464_477_539_063e-8;

use super::cameras::Lcg;
use super::geodesy::lon_lat_alt_to_ecef;

/// A spread of tiles: both pole rows, the antimeridian columns, the equator, and a
/// scattering of ordinary ones at every zoom.
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

// ─────────────────────────────────────────────────────────────────────────────
// I-1 — zero relief
// ─────────────────────────────────────────────────────────────────────────────

/// **Invariant I-1.** No vertex `TileMesh::generate` emits may sit above the
/// ellipsoid.
///
/// This is the premise for the whole horizon stage. For a point *on* the ellipsoid,
/// occlusion collapses from Theorem 3.1's two conditions to the single linear
/// inequality `q·c ≤ 1` (Theorem 3.5), and that collapse is what makes the tile
/// horizon test exact in 25 flops. It is also what makes back-face culling redundant
/// rather than merely cheap. An elevated point can have `q·c < 1` and still be
/// visible over the limb, so **the moment terrain relief is applied to the mesh, the
/// horizon test becomes unsound** and must be replaced by the scaled-space cone test
/// (§3.7, Theorem 3.7), which `horizon::point_is_occluded` is already the `ρ = 0`
/// special case of.
///
/// If you are reading this because you just turned terrain on: that is the fix. Do
/// not delete this test, and do not widen its tolerance.
///
/// Altitude is measured as `‖T(p)‖ − 1` in scaled space, which is exactly zero on the
/// ellipsoid regardless of latitude. The tolerance is the f32 quantum of the vertex
/// positions themselves — `TileMesh` stores them as f32 offsets from an f64 tile
/// centre — and nothing else; any real elevation model clears it by orders of
/// magnitude.
#[test]
fn test_generated_mesh_has_no_positive_altitude() {
    let mut worst_m = f64::NEG_INFINITY;
    let mut worst_ctx = String::new();
    let mut failures: Vec<String> = Vec::new();

    for id in sample_tiles() {
        let mesh = TileMesh::generate(&id, 8);
        let center = DVec3::from_array(mesh.center_f64);
        for v in &mesh.vertices {
            let rel = DVec3::new(v.position[0] as f64, v.position[1] as f64, v.position[2] as f64);
            let p = center + rel;
            // Radial excess in scaled space, converted back to metres of altitude.
            let alt_m = (transform_to_scaled_space(p).length() - 1.0)
                * cesium_engine::globe::geometry::EARTH_RADIUS_B_F64
                * 1.0e6;
            let tol_m = F32_BOUND * (center.length() + rel.length()) * 1.0e6;

            if alt_m > worst_m {
                worst_m = alt_m;
                worst_ctx = format!("z={} x={} y={} (tolerance {tol_m:.3} m)", id.z, id.x, id.y);
            }
            if alt_m > tol_m {
                failures.push(format!(
                    "z={} x={} y={}: vertex {alt_m:.3} m above the ellipsoid \
                     (tolerance {tol_m:.3} m)",
                    id.z, id.x, id.y
                ));
            }
        }
    }

    println!("  worst vertex altitude over {} tiles: {worst_m:.3} m  [{worst_ctx}]", sample_tiles().len());
    assert!(
        failures.is_empty(),
        "TileMesh::generate emitted {} vertices above the ellipsoid. Invariant I-1 \
         is broken, and with it the tile horizon test — see the doc comment on this \
         test and docs/culling-math.md §3.7.\n  {}",
        failures.len(),
        failures.iter().take(5).cloned().collect::<Vec<_>>().join("\n  ")
    );
}

// ─────────────────────────────────────────────────────────────────────────────
// I-5 — one source of tile bounds
// ─────────────────────────────────────────────────────────────────────────────

/// **Invariant I-5.** Every vertex the renderer draws for a tile must lie inside the
/// lon/lat rectangle the culler tests for that tile.
///
/// A mismatch is not a rounding curiosity. The horizon test culls a tile when the
/// *rectangle* is entirely below the limb; if the mesh reaches outside the rectangle,
/// that sliver is drawn nowhere and culled anyway — a false negative at every tile
/// edge. This test converts each mesh vertex back to (lon, lat) and checks it against
/// `tile_bounds`.
///
/// It is what caught the two things that made the bounds agree only approximately:
/// the mesh lerping longitude in f32 (up to an ulp, ~1.7 m, past `lon_max`), and the
/// bounds being derived in f32 at all.
///
/// Skirt vertices are exempt in *altitude* only — they are pushed radially inward,
/// never sideways — so their lon/lat must still be inside the rectangle.
#[test]
fn test_generated_mesh_stays_inside_the_culling_rectangle() {
    let mut worst_deg = 0.0_f64;
    let mut worst_ctx = String::new();
    let mut failures: Vec<String> = Vec::new();

    for id in sample_tiles() {
        let b = tile_bounds(&id);
        let mesh = TileMesh::generate(&id, 8);
        let center = DVec3::from_array(mesh.center_f64);

        for v in &mesh.vertices {
            let rel = DVec3::new(v.position[0] as f64, v.position[1] as f64, v.position[2] as f64);
            let p = center + rel;

            // Tolerance: the f32 quantum of the vertex position, expressed as an
            // angle. The vertex is stored as an f32 offset from an f64 centre, so
            // this is the floor no amount of agreement can get below.
            let tol_deg = (F32_BOUND * (center.length() + rel.length())
                / cesium_engine::globe::geometry::EARTH_RADIUS_A_F64)
                .to_degrees();

            // Skirt vertices are displaced along the **ellipsoid normal**, not toward
            // the centre, so their geodetic latitude is not their parametric latitude
            // — at z=2 the 125 km skirt shifts it by 2.8e-3 deg. They are deliberately
            // outside this rectangle's surface and are excluded here; §3.5 covers them
            // separately, and by a stronger argument: they lie strictly *inside* the
            // ellipsoid, so any segment from an exterior eye to a skirt point crosses
            // the sphere and is occluded whenever the patch is.
            //
            // Pole-cap vertices sit at altitude 0 and so are *not* excluded — they are
            // exactly what checks the pole stretch.
            let alt = transform_to_scaled_space(p).length() - 1.0;
            if alt < -tol_deg.to_radians() {
                continue;
            }

            let (lat, lon) = super::geodesy::dvec3_to_lat_lon(p);
            // Both terms are measured as **degrees of great-circle arc**, so they
            // are comparable and so the pole needs no special case: a longitude
            // error there spans no ground, and `cos(lat)` says so.
            let d_lat = (b.lat_min - lat).max(lat - b.lat_max);

            // Longitude is circular: `dvec3_to_lat_lon` returns (-180, 180], so a
            // vertex at exactly -180 comes back as +180 and a naive difference reads
            // 360 deg outside. Measure the offset from `lon_min` modulo a full turn
            // and take whichever representative sits closer to the rectangle —
            // picking one a priori is ambiguous for a z=1 tile, whose width is
            // exactly 180 deg.
            let width = b.lon_max - b.lon_min;
            let excess_of = |o: f64| (-o).max(o - width);
            let off = (lon - b.lon_min).rem_euclid(360.0);
            let d_lon = excess_of(off).min(excess_of(off - 360.0)) * lat.to_radians().cos();

            let excess = d_lat.max(d_lon);

            if excess > worst_deg {
                worst_deg = excess;
                worst_ctx = format!(
                    "z={} x={} y={} d_lat={:.3e} d_lon={:.3e} tol={tol_deg:.3e}",
                    id.z, id.x, id.y, d_lat, d_lon
                );
            }
            if excess > tol_deg {
                failures.push(format!(
                    "z={} x={} y={}: vertex {excess:.3e} deg outside the culling \
                     rectangle (tolerance {tol_deg:.3e} deg)",
                    id.z, id.x, id.y
                ));
            }
        }
    }

    println!("  worst mesh excess over the culling rectangle: {worst_deg:.3e} deg  [{worst_ctx}]");
    assert!(
        failures.is_empty(),
        "{} mesh vertices fall outside their tile's culling rectangle. Invariant I-5 \
         is broken: that sliver is drawn by nobody and culled anyway.\n  {}",
        failures.len(),
        failures.iter().take(5).cloned().collect::<Vec<_>>().join("\n  ")
    );
}

/// Adjacent tiles must share a boundary *value*, not merely a boundary formula —
/// otherwise the tiling has seams at every edge no matter how exact each tile is.
#[test]
fn test_tile_bounds_tile_the_sphere_without_seams() {
    for z in 1..=20u8 {
        let n = 1_u32 << z;
        for (x, y) in [(0u32, 0u32), (n / 2, n / 2), (n.saturating_sub(2), n.saturating_sub(2))] {
            if x + 1 > n - 1 || y + 1 > n - 1 {
                continue;
            }
            let a = tile_bounds(&TileId { z, x, y });
            let right = tile_bounds(&TileId { z, x: x + 1, y });
            let below = tile_bounds(&TileId { z, x, y: y + 1 });
            assert_eq!(
                a.lon_max, right.lon_min,
                "z={z} x={x}: longitude seam between columns {x} and {}",
                x + 1
            );
            // The pole stretch deliberately overrides the outer edge of row 0 and row
            // n-1 to +/-90; the *shared* edges between any two real rows must still
            // agree exactly.
            assert_eq!(
                a.lat_min, below.lat_max,
                "z={z} y={y}: latitude seam between rows {y} and {}",
                y + 1
            );
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// The scaled-space map
// ─────────────────────────────────────────────────────────────────────────────

/// `‖T(p(λ,φ))‖ = 1` for every surface point.
///
/// `T` divides **x and z** by `a` and **y** by `b`, because this ECEF frame is Y-up
/// with negated Z — it is `y` that carries the semi-minor axis. Every scaled-space
/// site in the engine already does this, but the pairing is invisible at a glance and
/// a future move to a Z-up convention would silently invert it, turning the exact
/// horizon test into a subtly wrong one. `docs/culling-math.md` §11.3 item 6 asks for
/// exactly this test.
#[test]
fn test_scaled_space_maps_surface_to_unit_sphere() {
    let mut rng = Lcg::new(0x5CA1_ED00);
    let mut worst = 0.0_f64;

    for i in 0..20_000 {
        let (lon, lat) = if i < 8 {
            // The corners of the domain, explicitly.
            [
                (-180.0, -90.0),
                (-180.0, 90.0),
                (180.0, -90.0),
                (180.0, 90.0),
                (0.0, 0.0),
                (0.0, 90.0),
                (90.0, 0.0),
                (-90.0, -0.0),
            ][i]
        } else {
            (
                -180.0 + 360.0 * rng.next_f64(),
                -90.0 + 180.0 * rng.next_f64(),
            )
        };

        let p = DVec3::from_array(lon_lat_to_ecef_f64(lon, lat));
        let err = (transform_to_scaled_space(p).length() - 1.0).abs();
        if err > worst {
            worst = err;
        }
        assert!(
            err < 1.0e-14,
            "|T(p({lon}, {lat}))| = 1 + {err:.3e}; the scaled-space axis pairing is \
             wrong (x,z ÷ a and y ÷ b, for a Y-up, negated-Z ECEF frame)"
        );
    }
    println!("  worst | |T(p)| - 1 | over 20 000 surface points: {worst:.3e}");
}

// ─────────────────────────────────────────────────────────────────────────────
// The horizon closed form
// ─────────────────────────────────────────────────────────────────────────────

/// The one piece of nontrivial trigonometry in the culling path, pinned against
/// brute force.
///
/// `TilePatch::max_dot` claims to return the **exact** supremum of `q·c` over the
/// tile's spherical rectangle, via two closed-form maximisations (§3.4). This
/// compares it against a dense grid search over the same rectangle for a few thousand
/// random (tile, camera) pairs, including pole rows, the antimeridian, cameras
/// directly over a pole (where `ρ = 0` and the longitude maximisation degenerates),
/// and the case the arc test exists for: a tile that straddles the antimeridian
/// *relative to the camera*, where a linear clamp on longitude picks the wrong
/// endpoint.
///
/// The closed form must never **under**-estimate — under-estimating `S` is what would
/// cull a visible tile — and must not over-estimate by more than f64 rounding, or the
/// horizon stage stops being exact and starts being a bound.
#[test]
fn test_horizon_closed_form_matches_brute_force() {
    /// Grid resolution per axis for the reference maximisation.
    const STEPS: usize = 257;

    let mut rng = Lcg::new(0xB0_1DFACE);
    let mut worst_under = 0.0_f64; // brute − closed: positive means the closed form missed a point
    let mut worst_over = 0.0_f64;
    let mut worst_ctx = String::new();

    // Deterministic hard cases first, then random ones.
    let mut cases: Vec<(TileId, DVec3)> = Vec::new();
    for z in 1..=6u8 {
        let n = 1_u32 << z;
        for (x, y) in [(0, 0), (n - 1, 0), (0, n - 1), (n - 1, n - 1), (n / 2, n / 2)] {
            // A camera at the antipodal longitude, which is what the circular arc
            // test exists for.
            cases.push((TileId { z, x, y }, lon_lat_alt_to_ecef(170.0, 0.0, 400_000.0)));
            cases.push((TileId { z, x, y }, lon_lat_alt_to_ecef(-170.0, 0.0, 400_000.0)));
            // Cameras over each pole: rho = 0.
            cases.push((TileId { z, x, y }, lon_lat_alt_to_ecef(0.0, 90.0, 2_000_000.0)));
            cases.push((TileId { z, x, y }, lon_lat_alt_to_ecef(0.0, -90.0, 2_000_000.0)));
        }
    }
    for _ in 0..3000 {
        let z = 1 + (rng.next_f64() * 7.0) as u8;
        let n = 1_u32 << z;
        let x = ((rng.next_f64() * n as f64) as u32).min(n - 1);
        let y = ((rng.next_f64() * n as f64) as u32).min(n - 1);
        let cam = lon_lat_alt_to_ecef(
            -180.0 + 360.0 * rng.next_f64(),
            -90.0 + 180.0 * rng.next_f64(),
            10.0 * 10f64.powf(6.5 * rng.next_f64()),
        );
        cases.push((TileId { z, x, y }, cam));
    }

    for (id, cam_pos) in &cases {
        let b = tile_bounds(id);
        let patch = TilePatch::new(&b);
        let cam = HorizonCamera::new(*cam_pos);
        let closed = patch.max_dot(&cam);

        let mut brute = f64::NEG_INFINITY;
        for i in 0..STEPS {
            let u = i as f64 / (STEPS - 1) as f64;
            let lon = (b.lon_min + u * (b.lon_max - b.lon_min)).to_radians();
            let (sin_lon, cos_lon) = lon.sin_cos();
            for j in 0..STEPS {
                let v = j as f64 / (STEPS - 1) as f64;
                let lat = (b.lat_min + v * (b.lat_max - b.lat_min)).to_radians();
                let (sin_lat, cos_lat) = lat.sin_cos();
                // q(λ,φ) on the unit sphere, in the engine's Y-up, negated-Z frame.
                let q = DVec3::new(cos_lat * cos_lon, sin_lat, -cos_lat * sin_lon);
                brute = brute.max(q.dot(cam.c));
            }
        }

        let under = brute - closed;
        let over = closed - brute;
        if under > worst_under {
            worst_under = under;
            worst_ctx = format!(
                "z={} x={} y={} closed={closed:.15} brute={brute:.15}",
                id.z, id.x, id.y
            );
        }
        worst_over = worst_over.max(over);
    }

    println!(
        "  closed form vs {STEPS}x{STEPS} brute force over {} (tile, camera) pairs:",
        cases.len()
    );
    println!("    worst under-estimate (brute - closed): {worst_under:.3e}  [{worst_ctx}]");
    println!("    worst over-estimate  (closed - brute): {worst_over:.3e}");

    // Under-estimating is the dangerous direction: it culls a tile that has a visible
    // point. f64 rounding of a four-term expression of magnitude <= C <= ~6 is
    // ~1e-15; the grid itself cannot resolve better than that either.
    assert!(
        worst_under < 1.0e-14,
        "TilePatch::max_dot under-estimated the supremum of q.c by {worst_under:.3e} \
         — the horizon test is not exact and will cull visible tiles. {worst_ctx}"
    );
    // Over-estimating is safe but means the closed form is a bound, not the answer,
    // and the FP = 0 claim for the horizon stage no longer holds. The brute-force
    // grid samples the rectangle, so it can legitimately miss the true maximum by the
    // curvature over one grid cell; at 257 steps on a z=1 tile that is ~1e-4.
    assert!(
        worst_over < 1.0e-3,
        "TilePatch::max_dot over-estimated by {worst_over:.3e}, far more than the \
         grid's own resolution — the closed form is not tracking the true supremum"
    );
}
