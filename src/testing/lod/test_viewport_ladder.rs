//! Confirms `lod_factor_for`'s live viewport
//! height and mode-dependent `fovy` — shipped (commit `d866274`) on the
//! strength of one headless capture — actually behave as intended across a ladder of
//! viewport/mode combinations, with the 204-pose distribution as the evidence rather
//! than a screenshot.

use cesium_engine::camera::camera::{Camera, CameraMode};
use cesium_engine::globe::quadtree::lod_factor_for;

use super::ladder::{at_rung, rungs, S23_LANDSCAPE, S23_PORTRAIT};
use super::report;
use super::sweep::{bench_poses, measure_poses};

fn fovy_for(mode: CameraMode) -> f32 {
    let mut cam = Camera::new(glam::Vec3::ZERO, glam::Vec3::ZERO);
    cam.mode = mode;
    cam.fovy()
}

/// Pure-function check, away from the calibration point (1080p, `Free`): every
/// rung's `lod_factor_for` output must match hand-derived expectations from the
/// formula alone — `lod_factor` scales *linearly* with viewport height (it is a bare
/// multiplicative term, unlike `target_texel_ratio`'s `sqrt` or `texture_size_px`'s
/// inverse), and `Free` vs `Cockpit` at the *same* height must differ only by the
/// `2·tan(fovy/2)` term. If `wgpu_state::update_logic`'s live feed were wired to the
/// wrong field (e.g. width instead of height, or a mode that doesn't route through
/// `Camera::fovy()`), this is what would catch it — a screenshot at one viewport
/// cannot distinguish "correct" from "coincidentally looks fine here".
#[test]
fn test_lod_factor_matches_hand_derivation_at_every_rung() {
    let free_fovy = fovy_for(CameraMode::Free);
    let cockpit_fovy = fovy_for(CameraMode::Cockpit);
    assert!(
        (free_fovy - cockpit_fovy).abs() > 0.01,
        "sanity: Free and Cockpit fovy must actually differ, or this test proves nothing"
    );

    let desktop = lod_factor_for(1.0, 512.0, 1080.0, free_fovy);

    // Linear in height, Free throughout: 720p must be exactly desktop * (720/1080).
    let small_desktop = lod_factor_for(1.0, 512.0, 720.0, free_fovy);
    let rel_err = (small_desktop - desktop * (720.0 / 1080.0)).abs() / desktop;
    assert!(rel_err < 1e-5, "1280x720 Free lod_factor should be desktop * 720/1080: rel_err={rel_err}");

    // S23 landscape height (1080) equals the desktop's: lod_factor_for depends only
    // on height, not width, so these must be *identical* in Free mode — landscape
    // alone buys the S23 nothing extra from this formula, only portrait does.
    let s23_landscape_free = lod_factor_for(1.0, 512.0, S23_LANDSCAPE.1 as f32, free_fovy);
    assert_eq!(
        s23_landscape_free.to_bits(),
        desktop.to_bits(),
        "S23 landscape Free must be bit-identical to the 1920x1080 Free desktop default \
         (same height, same mode) — width plays no part in lod_factor_for"
    );

    // S23 portrait height (2340) is what actually asks for more detail.
    let s23_portrait_free = lod_factor_for(1.0, 512.0, S23_PORTRAIT.1 as f32, free_fovy);
    let rel_err = (s23_portrait_free - desktop * (S23_PORTRAIT.1 as f32 / 1080.0)).abs() / desktop;
    assert!(
        rel_err < 1e-5,
        "S23 portrait Free lod_factor should be desktop * {}/1080: rel_err={rel_err}",
        S23_PORTRAIT.1
    );
    assert!(
        s23_portrait_free > desktop * 2.0,
        "S23 portrait must ask for meaningfully (>2x) more detail than the desktop default: \
         desktop={desktop} s23_portrait={s23_portrait_free}"
    );

    // Cockpit vs Free at the *same* height (S23 landscape, 1080): must differ by
    // exactly the 2*tan(fovy/2) ratio, nothing else — same target, same texture size,
    // same height, only mode changes.
    let s23_landscape_cockpit = lod_factor_for(1.0, 512.0, S23_LANDSCAPE.1 as f32, cockpit_fovy);
    let expected_ratio =
        (2.0 * (free_fovy * 0.5).tan()) / (2.0 * (cockpit_fovy * 0.5).tan());
    let measured_ratio = s23_landscape_cockpit / s23_landscape_free;
    let rel_err = (measured_ratio - expected_ratio).abs() / expected_ratio;
    assert!(
        rel_err < 1e-5,
        "Cockpit/Free lod_factor ratio at equal height should be 2tan(fovy_free/2) / \
         2tan(fovy_cockpit/2) = {expected_ratio:.4}, measured {measured_ratio:.4}"
    );
    assert!(
        s23_landscape_cockpit < s23_landscape_free,
        "Cockpit's wider FOV must make it refine less than Free at the same height: \
         free={s23_landscape_free} cockpit={s23_landscape_cockpit}"
    );
}

/// The distribution-based confirmation: re-runs the 204 bench pose
/// geometries at every ladder rung and reports `Summary` side by side. Measures and records —
/// no pass/fail target here beyond the instrument behaving (handled by
/// `test_lod_sweep_produces_sane_aggregates` already covering the degenerate/NaN
/// cases at the default rung); the numbers themselves are the deliverable.
#[test]
fn test_viewport_ladder_distributions() {
    let base_poses = bench_poses();

    for rung in rungs() {
        let poses = at_rung(&base_poses, &rung);
        let results = measure_poses(&poses);
        let s = report::summarize(&results);
        eprintln!(
            "{:<28} tiles={:<6} aggregate={:<8.4} median={:<8.4} p5={:<8.4} p95={:<9.4} \
             texture_MiB={:<9.1} deepest_z={}",
            rung.name,
            s.total_tiles,
            s.aggregate_ratio,
            s.median(),
            s.p5(),
            s.p95(),
            s.total_texture_bytes as f64 / (1024.0 * 1024.0),
            s.max_deepest_zoom,
        );
        assert!(s.total_tiles > 0, "{}: must produce visible tiles", rung.name);
        assert!(s.aggregate_ratio.is_finite(), "{}: aggregate_ratio must be finite", rung.name);
    }
}
