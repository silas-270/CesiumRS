//! How a visible label looks: size, opacity and the dimensions of its pill and dot.
//!
//! Pure functions of the label and the camera, so the renderer only places what this
//! module decides. Sizes are in **points**; the renderer multiplies by the window's
//! pixels-per-point, as egui did when it painted these labels.

use glam::Vec3;

/// Horizontal padding between the text and the pill's edge.
pub const PAD_X_PT: f32 = 4.0;
/// Vertical padding between the text and the pill's edge.
pub const PAD_Y_PT: f32 = 2.5;
/// Gap between the top of the anchor dot and the bottom of the pill.
pub const DOT_GAP_PT: f32 = 3.0;
/// Corner radius of the pill.
pub const PILL_ROUNDING_PT: f32 = 3.0;
/// The pill's colour, sRGB; its alpha comes from [`LabelStyle::bg_alpha`].
pub const PILL_RGB: [u8; 3] = [8, 12, 18];

/// Per-frame values every label's style is computed against.
pub struct StyleFrame {
    cam_pos: Vec3,
    altitude: f32,
    /// Max distance at which a rank > 2 label is drawn (megametres).
    max_render_dist: f32,
    /// Max distance for ranks 0–2: the horizon, or `max_render_dist` if further.
    max_dist_rank02: f32,
    size_scale: f32,
}

impl StyleFrame {
    /// `cam_pos` in ECEF megametres, `altitude` above the ellipsoid in megametres.
    pub fn new(cam_pos: Vec3, altitude: f32, size_scale: f32) -> Self {
        let altitude = altitude.max(0.0001);
        let max_render_dist = (altitude * 1.5 + 0.15).max(0.15);
        let r_earth = 6.378137_f32;
        let horizon_dist = (2.0 * r_earth * altitude + altitude * altitude).sqrt();
        Self {
            cam_pos,
            altitude,
            max_render_dist,
            max_dist_rank02: horizon_dist.max(max_render_dist),
            size_scale,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LabelStyle {
    pub font_pt: f32,
    pub text_alpha: u8,
    pub bg_alpha: u8,
    pub shadow_alpha: u8,
    pub dot_radius_pt: f32,
}

/// The style of a label at `ecef_pos` with `label_rank`, or `None` when it would be
/// too faint to be worth drawing.
pub fn label_style(frame: &StyleFrame, ecef_pos: Vec3, label_rank: u8) -> Option<LabelStyle> {
    let dist = (ecef_pos - frame.cam_pos).length();
    let max_dist = if label_rank <= 2 {
        frame.max_dist_rank02
    } else {
        frame.max_render_dist
    };

    // Relative distance normalized across visible range [altitude .. max_dist]
    let dist_span = (max_dist - frame.altitude).max(0.01);
    let rel_dist = ((dist - frame.altitude) / dist_span).clamp(0.0, 1.0);

    // Proximity factor [0.0 = at max range / horizon, 1.0 = near nadir / camera]
    let proximity = 1.0 - rel_dist;

    // Continuous distance & atmospheric haze fade: full opacity for the nearest ~35% of
    // the range, then a smoothstep down to 0 by 95%.
    let fade_start = 0.35_f32;
    let fade_end = 0.95_f32;
    let t = ((rel_dist - fade_start) / (fade_end - fade_start)).clamp(0.0, 1.0);
    let dist_fade = 1.0 - t * t * (3.0 - 2.0 * t);

    // Rank-based font size boost: capitals and major cities are larger
    let rank_scale = if label_rank <= 2 {
        1.3_f32
    } else if label_rank <= 5 {
        1.0_f32
    } else {
        0.82_f32
    };

    // 9pt (distant) to 14pt (near), scaled by rank and the user's size setting
    let font_pt = (9.0 + proximity * 5.0) * rank_scale * frame.size_scale;

    let text_alpha = ((180.0 + proximity * 75.0) * dist_fade).round() as u8;
    let bg_alpha = ((100.0 + proximity * 70.0) * dist_fade).round() as u8;
    let shadow_alpha = (120.0 * dist_fade).round() as u8;

    if text_alpha < 3 && bg_alpha < 3 {
        return None;
    }

    Some(LabelStyle {
        font_pt,
        text_alpha,
        bg_alpha,
        shadow_alpha,
        dot_radius_pt: 1.5 + proximity * 1.5,
    })
}
