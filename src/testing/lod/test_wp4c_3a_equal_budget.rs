//! The 3a evaluation, reframed per the refutation
//! as the deliberate trade it actually is — *"Is box-distance at a
//! higher `target_texel_ratio` better than centre-distance at a lower one, at a
//! fixed tile/texture budget?"* — rather than the no-op originally hoped for.
//!
//! **Measurement only.** Nothing here is adopted: `wgpu_state.rs` never sets
//! `LodDistanceMode::Box`, `QuadtreeManager::new()` still defaults to `Centre`, and
//! this file changes no shipped behaviour. It exists to produce the report
//! and then stop — whether to spend the finding is gated on
//! a product decision this file does not make.

use super::report::{self, Summary};
use super::sweep::{bench_poses, measure_poses_with_config, LodConfig, PoseResult};
use cesium_engine::globe::quadtree::LodDistanceMode;
use super::super::culling::cameras::ViewParams;

/// Total visible tiles across all poses at `cfg`.
fn total_tiles(poses: &[ViewParams], cfg: LodConfig) -> usize {
    measure_poses_with_config(poses, cfg)
        .iter()
        .map(|r| r.tile_count)
        .sum()
}

/// Sanity check on the `LodDistanceMode` plumbing itself, before trusting anything
/// downstream of it: an earlier refutation measured box-distance alone, at equal `target_texel_ratio = 1.0`,
/// moving tile count from 3 922 to 6 685 — a +70.5% change. If this harness's `Box`
/// mode were wired to the wrong field, gave the same answer as `Centre`, or used a
/// different box than the one previously measured, this would catch it before the equal-
/// budget search (which depends on `Box` actually being more aggressive than
/// `Centre`, never less) runs on top of a silently broken switch.
#[test]
fn test_box_distance_matches_wp3s_refutation_measurement_at_equal_target() {
    let poses = bench_poses();

    let centre_tiles = total_tiles(&poses, LodConfig::default());
    let box_tiles = total_tiles(
        &poses,
        LodConfig::default().with_distance_mode(LodDistanceMode::Box),
    );

    assert_eq!(
        centre_tiles, 3922,
        "centre-distance tile count at target=1.0 must match the recorded WP0-WP3 baseline"
    );
    let pct_change = (box_tiles as f64 - centre_tiles as f64) / centre_tiles as f64 * 100.0;
    eprintln!(
        "centre={centre_tiles} box={box_tiles} ({pct_change:+.1}%), WP3 refutation measured 3922 -> 6685 (+70.5%)"
    );
    assert!(
        (50.0..90.0).contains(&pct_change),
        "box-distance at equal target should move tile count by roughly +70% (WP3's own \
         measurement), got {pct_change:+.1}% ({centre_tiles} -> {box_tiles}) — the \
         LodDistanceMode plumbing may not match what WP3 measured"
    );
}

/// As [`super::sweep::bisect_target_for_tile_count`], specialised to the plain
/// (non-fog) measurement path with an explicit `distance_mode` and
/// `texture_size_px`. `target_texel_ratio`'s relationship to `total_tiles` is
/// monotonic non-decreasing (higher target -> larger `lod_factor` -> equal or more
/// subdivision), never the reverse, which is what makes bisection meaningful here.
fn tune_target_for_tile_count(
    poses: &[ViewParams],
    distance_mode: LodDistanceMode,
    texture_size_px: f32,
    target_n: usize,
    search_hi: f32,
) -> (f32, usize) {
    super::sweep::bisect_target_for_tile_count(target_n, search_hi, |t| {
        total_tiles(
            poses,
            LodConfig::new(t, texture_size_px).with_distance_mode(distance_mode),
        )
    })
}

/// One row of the comparison table.
struct Row {
    label: &'static str,
    target_texel_ratio: f32,
    tiles: usize,
    summary_text: String,
}

fn measure_row(
    label: &'static str,
    poses: &[ViewParams],
    distance_mode: LodDistanceMode,
    target_texel_ratio: f32,
) -> (Row, Summary, Vec<PoseResult>) {
    let cfg = LodConfig::new(target_texel_ratio, 512.0).with_distance_mode(distance_mode);
    let results = measure_poses_with_config(poses, cfg);
    let s = report::summarize(&results);
    let tiles: usize = results.iter().map(|r| r.tile_count).sum();
    let row = Row {
        label,
        target_texel_ratio,
        tiles,
        summary_text: format!(
            "aggregate={:.4} median={:.4} p5={:.4} p25={:.4} p95={:.4}",
            s.aggregate_ratio,
            s.median(),
            s.p5(),
            s.p25(),
            s.p95(),
        ),
    };
    (row, s, results)
}

/// The real work: compares centre-distance (today, at its own `target=1.0`, which
/// already defines the reference tile budget `N`) against box-distance tuned to the
/// *same* `N` — not to the same `target_texel_ratio`, which is exactly what made
/// the earlier "no-op" framing wrong ("Refuted, moved to follow-up"). Ends in a printed report
/// (and this doc comment / the assertions below pin the qualitative finding); no
/// production code is changed by this test, and none of its numbers are adopted.
#[test]
fn test_wp4c_box_distance_vs_centre_distance_at_equal_tile_budget() {
    let poses = bench_poses();

    // N is centre-distance's own tile count at its natural default — "today", not a
    // separately chosen number.
    let n = total_tiles(&poses, LodConfig::default());
    assert_eq!(n, 3922);

    // Box-distance over-produces at equal target (see the sanity test above), so its
    // matching target must be *below* 1.0 — search comfortably includes 1.0 as the
    // upper bound since we already know target=1.0 overshoots N for Box.
    let (box_target, box_n) =
        tune_target_for_tile_count(&poses, LodDistanceMode::Box, 512.0, n, 1.0);

    eprintln!(
        "WP4/C equal-budget search: N={n}, box-distance matched at target_texel_ratio={box_target:.5} (tiles={box_n}, {:+.2}% off N)",
        (box_n as f64 - n as f64) / n as f64 * 100.0
    );
    let off_frac = (box_n as i64 - n as i64).unsigned_abs() as f64 / (n as f64);
    assert!(
        off_frac < 0.01,
        "the bisection search should match N to within 1%: N={n} achieved={box_n}"
    );

    let (centre_row, centre_summary, centre_results) =
        measure_row("centre-distance @ target=1.0 (today)", &poses, LodDistanceMode::Centre, 1.0);
    let (box_row, box_summary, box_results) =
        measure_row("box-distance @ equal budget", &poses, LodDistanceMode::Box, box_target);

    eprintln!("\n=== WP4/C: centre-distance vs box-distance at equal tile budget (N={n}) ===");
    for row in [&centre_row, &box_row] {
        eprintln!(
            "{:<32} target_texel_ratio={:<8.5} tiles={:<6} {}",
            row.label, row.target_texel_ratio, row.tiles, row.summary_text
        );
    }

    // The tails, which is what this comparison is actually about:
    // "does box-distance lift the worst under-refined tiles at a given budget more
    // than simply lowering the ratio uniformly does?".
    let worst_n = 20;
    let mut centre_worst: Vec<f64> = centre_results
        .iter()
        .flat_map(|r| r.tiles.iter())
        .filter(|t| t.has_screen_area())
        .map(|t| t.ratio)
        .collect();
    let mut box_worst: Vec<f64> = box_results
        .iter()
        .flat_map(|r| r.tiles.iter())
        .filter(|t| t.has_screen_area())
        .map(|t| t.ratio)
        .collect();
    centre_worst.sort_by(|a, b| a.partial_cmp(b).unwrap());
    box_worst.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let centre_tail = &centre_worst[..worst_n.min(centre_worst.len())];
    let box_tail = &box_worst[..worst_n.min(box_worst.len())];
    eprintln!(
        "worst {worst_n} under-refined ratios, centre: {:?}",
        centre_tail.iter().map(|r| format!("{r:.4}")).collect::<Vec<_>>()
    );
    eprintln!(
        "worst {worst_n} under-refined ratios, box:    {:?}",
        box_tail.iter().map(|r| format!("{r:.4}")).collect::<Vec<_>>()
    );
    let shared_worst = centre_tail
        .iter()
        .zip(box_tail.iter())
        .take_while(|(c, b)| (**c - **b).abs() < 1e-9)
        .count();
    eprintln!(
        "worst {worst_n}: the {shared_worst} single worst ranks are bit-identical between \
         variants (both hit MAX_ZOOM=20 — box-distance cannot refine past the cap either)"
    );

    // Pins the measured, qualitative finding rather than a formula-derived one:
    // at equal tile budget, box-distance does not lift the blurry tail (p5) — if
    // anything it is measurably worse here, because the true worst cases are capped
    // by MAX_ZOOM (see `shared_worst` above) and the +70%-at-equal-target budget it
    // would otherwise spend there gets redirected elsewhere by the lower
    // target_texel_ratio needed to hold the budget. A future change to the bench
    // poses, MAX_ZOOM, or the LOD rule could legitimately flip this; if it does,
    // the finding below needs
    // re-deriving, not just re-pinning.
    assert!(
        box_summary.p5() <= centre_summary.p5(),
        "measured finding: box-distance's p5 should not improve on centre-distance's \
         at equal tile budget — centre_p5={:.4} box_p5={:.4}. If this now fails, the \
         qualitative conclusion needs \
         re-checking, not just this assertion re-pinning.",
        centre_summary.p5(),
        box_summary.p5()
    );

    eprintln!("\n-- per-zoom aggregate_ratio, centre vs box --");
    let zooms: std::collections::BTreeSet<u8> = centre_summary
        .by_zoom
        .keys()
        .chain(box_summary.by_zoom.keys())
        .copied()
        .collect();
    for z in zooms {
        let c = centre_summary.by_zoom.get(&z);
        let b = box_summary.by_zoom.get(&z);
        eprintln!(
            "  z={z:<3} centre: tiles={:<6} ratio={:<8}  |  box: tiles={:<6} ratio={:<8}",
            c.map(|x| x.tile_count.to_string()).unwrap_or_default(),
            c.map(|x| format!("{:.4}", x.aggregate_ratio())).unwrap_or_default(),
            b.map(|x| x.tile_count.to_string()).unwrap_or_default(),
            b.map(|x| format!("{:.4}", x.aggregate_ratio())).unwrap_or_default(),
        );
    }

    report::emit("wp4c_centre_at_budget", &centre_results);
    report::emit("wp4c_box_at_budget", &box_results);
}
