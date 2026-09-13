//! Viewport/mode ladder for the LOD harness — WP4/B (`docs/pre-terrain-plan.md`).
//!
//! [`super::sweep::bench_poses`] is frozen at one viewport (1920x1080, `Free`) baked
//! into `src/testing/culling/cells.rs`, which is bit-stable by convention and out of
//! scope to edit. This module does not add poses there — it re-labels the *same* 204
//! pose geometries (lat/lon/alt/pitch/yaw/roll) at different viewport/mode
//! combinations. That is safe because `cameras::camera_transform` (which
//! `build_camera` calls) only ever reads geometry — it never reads
//! `ViewParams::width`/`height`/`mode` — so overriding those three fields cannot move
//! a single camera; it only changes what each camera is asked to project through and
//! which `fovy` it uses.
//!
//! This validates something WP3/3b shipped live but only ever checked with one
//! headless capture (commit `d866274`): `lod_factor_for` now reads real viewport
//! height and mode-dependent `fovy` every frame, but nothing had re-run the 204-pose
//! distribution at anything other than the harness's own 1080p/Free default until
//! this package.

use cesium_engine::camera::camera::CameraMode;

use super::super::culling::cameras::ViewParams;

/// The Samsung S23's physical landscape resolution — matches
/// `src/testing/rendering/cockpit_s23.rs`'s `Shot { width: 2340, height: 1080, .. }`.
pub const S23_LANDSCAPE: (u32, u32) = (2340, 1080);
/// The S23 in portrait — matches `cockpit_s23.rs`'s `Shot { width: 1080, height: 2340, .. }`.
pub const S23_PORTRAIT: (u32, u32) = (1080, 2340);

/// One named viewport/mode combination to re-run the 204 bench pose geometries at.
#[derive(Clone, Copy, Debug)]
pub struct Rung {
    pub name: &'static str,
    pub width: u32,
    pub height: u32,
    pub mode: CameraMode,
}

/// The ladder WP4/B asks for, plus the S23 in portrait alongside the requested
/// landscape rungs: `lod_factor_for` depends on viewport *height* alone (not width,
/// not physical DPI), and the S23's landscape height (1080) is identical to the
/// desktop default's — so landscape alone cannot show whether "the S23 asks for more
/// detail" (`docs/pre-terrain-plan.md` WP4's original text) actually holds. Portrait
/// (height 2340) is the rung that tests it.
pub fn rungs() -> Vec<Rung> {
    vec![
        Rung { name: "1920x1080 Free (desktop default)", width: 1920, height: 1080, mode: CameraMode::Free },
        Rung { name: "1280x720 Free", width: 1280, height: 720, mode: CameraMode::Free },
        Rung { name: "S23 landscape Free", width: S23_LANDSCAPE.0, height: S23_LANDSCAPE.1, mode: CameraMode::Free },
        Rung { name: "S23 landscape Cockpit", width: S23_LANDSCAPE.0, height: S23_LANDSCAPE.1, mode: CameraMode::Cockpit },
        Rung { name: "S23 portrait Free", width: S23_PORTRAIT.0, height: S23_PORTRAIT.1, mode: CameraMode::Free },
        Rung { name: "S23 portrait Cockpit", width: S23_PORTRAIT.0, height: S23_PORTRAIT.1, mode: CameraMode::Cockpit },
    ]
}

/// Re-labels `poses`' viewport and mode, leaving every geometric field untouched —
/// see the module doc comment for why that is sound.
pub fn at_rung(poses: &[ViewParams], rung: &Rung) -> Vec<ViewParams> {
    poses
        .iter()
        .map(|p| ViewParams {
            width: rung.width,
            height: rung.height,
            mode: rung.mode,
            ..p.clone()
        })
        .collect()
}
