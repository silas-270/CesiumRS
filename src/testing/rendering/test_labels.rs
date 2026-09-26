//! Unit tests for the city-label renderer's CPU side: style, layout and rasterization
//! (`cesium_engine::label::style`, `cesium_engine::render::label_pipeline`).
//!
//! Run with `cargo test --lib test_labels`.

use cesium_engine::label::style::{label_style, StyleFrame};
use cesium_engine::render::label_pipeline::{atlas, layout};
use glam::Vec3;

/// A camera `altitude` megametres straight above a label on the equator.
fn frame_and_label(altitude: f32) -> (StyleFrame, Vec3) {
    let ground = Vec3::new(6.378137, 0.0, 0.0);
    let cam = Vec3::new(6.378137 + altitude, 0.0, 0.0);
    (StyleFrame::new(cam, altitude, 1.0), ground)
}

#[test]
fn nadir_label_is_largest_and_opaque() {
    let (frame, ground) = frame_and_label(0.01);
    let s = label_style(&frame, ground, 4).expect("a label straight below is drawn");
    assert!((s.font_pt - 14.0).abs() < 1e-4, "{s:?}");
    assert_eq!(s.text_alpha, 255);
    assert_eq!(s.bg_alpha, 170);
    assert!((s.dot_radius_pt - 3.0).abs() < 1e-4);
}

#[test]
fn rank_scales_font_size() {
    let (frame, ground) = frame_and_label(0.01);
    let capital = label_style(&frame, ground, 1).unwrap().font_pt;
    let town = label_style(&frame, ground, 4).unwrap().font_pt;
    let village = label_style(&frame, ground, 8).unwrap().font_pt;
    assert!((capital / town - 1.3).abs() < 1e-4);
    assert!((village / town - 0.82).abs() < 1e-4);
}

#[test]
fn labels_past_their_range_are_not_drawn() {
    let (frame, _) = frame_and_label(0.01);
    // Far beyond max_render_dist for a rank > 2 label.
    let far = Vec3::new(6.378137, 0.5, 0.0);
    assert!(label_style(&frame, far, 6).is_none());
}

#[test]
fn fade_is_monotone_with_distance() {
    let (frame, _) = frame_and_label(0.05);
    let mut last = u8::MAX;
    for i in 0..20 {
        let p = Vec3::new(6.378137, i as f32 * 0.01, 0.0);
        let a = label_style(&frame, p, 4).map_or(0, |s| s.text_alpha);
        assert!(a <= last, "alpha rose from {last} to {a} at step {i}");
        last = a;
    }
}

#[test]
fn layout_widths_and_metrics() {
    let font = atlas::font();
    let short = layout::layout(&font, "Lond", 16);
    let full = layout::layout(&font, "London", 16);
    assert!(full.width > short.width);
    assert!(full.row_height > full.ascent && full.ascent > 0.0);
    assert_eq!(full.glyphs.len(), 6);

    // Whitespace advances the pen but is not drawn.
    let two = layout::layout(&font, "Le Havre", 16);
    assert_eq!(two.glyphs.len(), 7);
    assert!(two.width > layout::layout(&font, "LeHavre", 16).width);

    // Pixel metrics scale with the size.
    let big = layout::layout(&font, "London", 32);
    assert!((big.width / full.width - 2.0).abs() < 0.05, "{} vs {}", big.width, full.width);
}

#[test]
fn rasterize_draws_glyphs_and_skips_blanks() {
    let font = atlas::font();
    let (_, w, h, cov) = atlas::rasterize(&font, 'A', 20).expect("'A' has an outline");
    assert!(w > 0 && h > 0);
    assert!(cov.iter().any(|&c| c > 200), "'A' should have solid coverage somewhere");
    assert!(atlas::rasterize(&font, ' ', 20).is_none());
    // Ubuntu covers the accents city names use.
    assert!(atlas::rasterize(&font, 'ü', 20).is_some());
}
