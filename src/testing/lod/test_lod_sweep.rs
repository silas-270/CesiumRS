//! Harness self-checks, and the measurement run itself.
//!
//! Per `docs/pre-terrain-plan.md` WP1, this module **measures and does not assert**
//! a target yet — there is no calibrated `target_texel_ratio` until WP4. What it
//! does assert are properties of the *instrument*: that sampling is faithful to the
//! shared tile-bounds definition, and that the measurement run itself behaves (no
//! NaNs leaking into aggregates, every counted tile lands in exactly one bucket).

use cesium_engine::globe::quadtree::{tile_bounds, TileId};
use glam::{DMat4, DVec3};

use super::report;
use super::sweep::{self, bench_poses, measure_poses, patch_grid_points, project_patch, TEXTURE_SIZE_PX};

fn to_dvec3(p: [f64; 3]) -> DVec3 {
    DVec3::new(p[0], p[1], p[2])
}

/// `patch_grid_points`'s four corners (`u, v ∈ {0, 1}`) must land exactly on the
/// same rectangle `tile_bounds` defines — the shared I-5 source the drawn mesh, the
/// culling rectangle and this harness are all required to agree on.
#[test]
fn test_patch_grid_corners_match_tile_bounds() {
    let ids = [
        TileId { z: 0, x: 0, y: 0 },
        TileId { z: 1, x: 0, y: 0 }, // touches the north pole cap
        TileId { z: 1, x: 1, y: 1 }, // touches the south pole cap
        TileId { z: 5, x: 10, y: 12 },
        TileId { z: 12, x: 2000, y: 1500 },
        TileId { z: 20, x: 3, y: 3 },
    ];

    for id in ids {
        let bounds = tile_bounds(&id);
        let grid = patch_grid_points(&id, &bounds);

        let expect = |lon: f64, lat: f64| to_dvec3(cesium_engine::globe::geometry::lon_lat_to_ecef_f64(lon, lat));

        assert_eq!(grid[0][0], expect(bounds.lon_min, bounds.lat_max), "NW corner, {id:?}");
        assert_eq!(grid[0][2], expect(bounds.lon_max, bounds.lat_max), "NE corner, {id:?}");
        assert_eq!(grid[2][0], expect(bounds.lon_min, bounds.lat_min), "SW corner, {id:?}");
        assert_eq!(grid[2][2], expect(bounds.lon_max, bounds.lat_min), "SE corner, {id:?}");

        // The centre sample must be strictly between the two latitude extremes
        // (or exactly the pole itself for a polar tile's opposite-of-the-cap row),
        // and it must not silently collapse onto an edge.
        let center = grid[1][1];
        assert!(center != grid[0][1] && center != grid[2][1], "{id:?}: centre row collapsed");
    }
}

/// Every visible tile lands in exactly one of "has screen area" / "offscreen or
/// clipped away entirely", and every ratio that enters an aggregate is finite and
/// positive. A harness that let a NaN or an infinity leak into `Summary::mean()`
/// would make every later WP3/WP4 comparison meaningless.
#[test]
fn test_lod_sweep_produces_sane_aggregates() {
    let poses = bench_poses();
    assert_eq!(poses.len(), 204, "expected the same 204 bench poses bench_update uses");

    let results = measure_poses(&poses);
    assert_eq!(results.len(), poses.len());

    let mut total_tiles = 0usize;
    for r in &results {
        assert_eq!(r.degenerate, r.tile_count == 0, "degenerate must track tile_count == 0");
        total_tiles += r.tile_count;
        assert_eq!(
            r.texture_bytes,
            r.tile_count as u64 * (sweep::TEXTURE_SIZE_PX as u64).pow(2) * sweep::BYTES_PER_TEXEL,
            "texture_bytes must be tile_count x texel area x bytes-per-texel"
        );
        for t in &r.tiles {
            assert!(t.screen_px >= 0.0 && t.screen_px.is_finite(), "{:?}: screen_px must be finite and non-negative", t.id);
            if t.has_screen_area() {
                assert!(t.ratio.is_finite() && t.ratio > 0.0, "{:?}: a tile with screen area must have a finite positive ratio", t.id);
            } else {
                assert!(t.ratio.is_infinite(), "{:?}: a tile with no screen area must report ratio = inf, not silently drop out", t.id);
            }
        }
    }
    assert!(total_tiles > 0, "the nadir/zoom-cliff bench poses must produce visible tiles");

    let text = report::emit("bench_poses", &results);

    let s = report::summarize(&results);
    assert!(s.mean().is_finite(), "mean ratio must be finite when any tile was sampled:\n{text}");
    assert!(s.sampled_tiles() > 0);
    assert_eq!(
        s.offscreen_tiles + s.sampled_tiles(),
        total_tiles,
        "every tile must be counted exactly once, in sampled or offscreen:\n{text}"
    );
}

/// Regression guard on the area-weighted `texels` fix: a tile whose patch is
/// entirely onscreen, entirely unclipped, and entirely in front of the eye must
/// still report the full `TEXTURE_SIZE_PX²` texel count — the area-weighting only
/// ever discounts a quad, never inflates it, so the base case (today's flat
/// behaviour) has to fall out of the general formula exactly.
///
/// Built directly against `project_patch` with a synthetic flat 3×3 grid and a
/// hand-built orthographic view-projection (`w = 1` for every point, by
/// construction) rather than a real camera/tile, so the test exercises the
/// area-weighting formula in isolation instead of real tile-bounds/camera
/// machinery that could hide a broken formula behind an unrelated pass.
#[test]
fn test_project_patch_fully_onscreen_quad_gets_full_texel_count() {
    // A flat 3x3 grid spanning [-1, 1] x [-1, 1] at z = 0.
    let samples: [[DVec3; 3]; 3] = [
        [DVec3::new(-1.0, 1.0, 0.0), DVec3::new(0.0, 1.0, 0.0), DVec3::new(1.0, 1.0, 0.0)],
        [DVec3::new(-1.0, 0.0, 0.0), DVec3::new(0.0, 0.0, 0.0), DVec3::new(1.0, 0.0, 0.0)],
        [DVec3::new(-1.0, -1.0, 0.0), DVec3::new(0.0, -1.0, 0.0), DVec3::new(1.0, -1.0, 0.0)],
    ];

    // Orthographic: w = 1 identically (the last row of the matrix is (0,0,0,1)),
    // so there is no behind-eye sample by construction. The [-4, 4] extent maps
    // the grid's [-1, 1] span to NDC [-0.25, 0.25] — comfortably inside [-1, 1]
    // with margin, so nothing clips against the viewport either.
    let vp = DMat4::orthographic_rh(-4.0, 4.0, -4.0, 4.0, -1.0, 1.0);
    let width = 800.0_f64;
    let height = 600.0_f64;

    let id = TileId { z: 3, x: 1, y: 1 };
    let metric = project_patch(id, &samples, &vp, width, height);

    assert!(!metric.behind_eye, "synthetic grid must not report any behind-eye sample");
    assert!(!metric.partly_offscreen, "synthetic grid must not be clipped at all");
    assert!(metric.screen_px > 0.0, "synthetic grid must project to nonzero area");

    let texels_full = (TEXTURE_SIZE_PX as f64) * (TEXTURE_SIZE_PX as f64);
    assert!(
        (metric.texels - texels_full).abs() < 1e-6,
        "a fully onscreen, unclipped, in-front-of-eye tile must report the full texel \
         count: got {}, expected {}",
        metric.texels,
        texels_full,
    );
}
