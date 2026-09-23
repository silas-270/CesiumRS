use cesium_engine::globe::quadtree::{tile_bounds, TileId};
use cesium_engine::globe::terrain::RunwayCorridor;

#[test]
fn test_runway_corridor_flat() {
    // A corridor at Stuttgart (EDDS) runway 07/25
    let start_lon = 9.200130;
    let start_lat = 48.685699;
    let end_lon = 9.243800;
    let end_lat = 48.694000;
    let elev = 389.0;
    let half_width = 25.0;
    let blend_margin = 35.0;

    let corridor = RunwayCorridor::new(
        start_lon,
        start_lat,
        end_lon,
        end_lat,
        Some(elev),
        Some(elev),
        half_width,
        blend_margin,
    );

    // Center of runway
    let mid_lon = (start_lon + end_lon) * 0.5;
    let mid_lat = (start_lat + end_lat) * 0.5;

    // Centerline height should be flattened to 389.0 regardless of raw height
    let filtered_start = corridor.filter_height(start_lon, start_lat, 450.0);
    assert!((filtered_start - 389.0).abs() < 1e-4, "got {filtered_start}");

    let filtered_mid = corridor.filter_height(mid_lon, mid_lat, 350.0);
    assert!((filtered_mid - 389.0).abs() < 1e-4, "got {filtered_mid}");

    let filtered_end = corridor.filter_height(end_lon, end_lat, 360.0);
    assert!((filtered_end - 389.0).abs() < 1e-4, "got {filtered_end}");

    // Far away point (e.g. Frankfurt) should be untouched
    let far_h = corridor.filter_height(8.57, 50.03, 111.0);
    assert_eq!(far_h, 111.0);
}

#[test]
fn test_runway_corridor_slope_and_blend() {
    // Sloping runway: 386.18 m at start down to 359.97 m at end
    let start_lon = 9.200130;
    let start_lat = 48.685699;
    let end_lon = 9.243800;
    let end_lat = 48.694000;
    let h0 = 386.18;
    let h1 = 359.97;
    let half_width = 25.0;
    let blend_margin = 35.0;

    let corridor = RunwayCorridor::new(
        start_lon,
        start_lat,
        end_lon,
        end_lat,
        Some(h0),
        Some(h1),
        half_width,
        blend_margin,
    );

    // Exact threshold evaluations
    let start_h = corridor.filter_height(start_lon, start_lat, 400.0);
    assert!((start_h - h0).abs() < 1e-3);

    let end_h = corridor.filter_height(end_lon, end_lat, 340.0);
    assert!((end_h - h1).abs() < 1e-3);

    // Midpoint should be exactly average
    let mid_lon = (start_lon + end_lon) * 0.5;
    let mid_lat = (start_lat + end_lat) * 0.5;
    let mid_expected = (h0 + h1) * 0.5;
    let mid_h = corridor.filter_height(mid_lon, mid_lat, 400.0);
    assert!((mid_h - mid_expected).abs() < 0.05, "got {mid_h}, expected {mid_expected}");

    // Lateral displacement:
    // ~1 degree of latitude is ~111,000 m.
    // 10 m north is ~ 10 / 111_000 ≈ 9.0e-5 degrees.
    let lat_offset_10m = 9.0e-5;
    let h_10m = corridor.filter_height(mid_lon, mid_lat + lat_offset_10m, 400.0);
    // 10m is <= half_width (25m), so must be exactly flattened to mid_expected
    assert!((h_10m - mid_expected).abs() < 0.05, "got {h_10m}, expected {mid_expected}");

    // 100m north: 100 / 111_000 ≈ 9.0e-4 degrees.
    // 100m is > half_width + blend_margin (25 + 35 = 60m), so must return raw_h untouched!
    let lat_offset_100m = 9.0e-4;
    let raw_test = 500.0;
    let h_100m = corridor.filter_height(mid_lon, mid_lat + lat_offset_100m, raw_test);
    assert_eq!(h_100m, raw_test);

    // In blend zone: 40m offset (between 25m and 60m)
    let lat_offset_40m = 40.0 / 111_000.0;
    let h_40m = corridor.filter_height(mid_lon, mid_lat + lat_offset_40m, raw_test);
    assert!(h_40m > mid_expected && h_40m < raw_test, "blended {h_40m} between {mid_expected} and {raw_test}");
}

#[test]
fn test_runway_corridor_tile_overlap() {
    let start_lon = 9.200130;
    let start_lat = 48.685699;
    let end_lon = 9.243800;
    let end_lat = 48.694000;

    let corridor = RunwayCorridor::new(
        start_lon,
        start_lat,
        end_lon,
        end_lat,
        Some(386.18),
        Some(359.97),
        25.0,
        35.0,
    );

    // Stuttgart tile at z12
    // Let's find the TileId for Stuttgart:
    let n = 1u32 << 12;
    let x = ((start_lon + 180.0) / 360.0 * n as f64).floor() as u32;
    let y = ((1.0 - (start_lat.to_radians().tan() + 1.0 / start_lat.to_radians().cos()).ln() / std::f64::consts::PI) * 0.5 * n as f64).floor() as u32;
    let id_str = TileId { z: 12, x, y };
    let bounds_str = tile_bounds(&id_str);
    assert!(corridor.overlaps_bounds(&bounds_str));

    // A tile in London or Pacific Ocean should not overlap
    let id_pacific = TileId { z: 12, x: 100, y: 100 };
    let bounds_pacific = tile_bounds(&id_pacific);
    assert!(!corridor.overlaps_bounds(&bounds_pacific));
}

