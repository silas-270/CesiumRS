//! Everything specific to the Boeing 787 interior GLB drawn in cockpit mode.
//!
//! Keeping the model's quirks here leaves `cesium-engine` a generic glTF renderer and
//! `camera_modes::cockpit` purely about the camera.
//!
//! ## Model conventions
//! The GLB is Y-up with **-Z forward** — the same convention as the aircraft frame — so
//! unlike `A350.glb` it needs no yaw correction. Its units are metres, spanning
//! 3.06 x 2.04 x 3.03 m, with the windshield at `z = -3.03` and the rear bulkhead at `z = 0`.

use cesium_engine::render::model_pipeline::pipeline::{ModelOptions, ModelRenderer};
use glam::Vec3;

use crate::camera_modes::cockpit::CAMERA_LOCAL_MM;

/// File name looked up through [`crate::assets`].
pub const ASSET_NAME: &str = "Boeing787Cockpit.glb";

/// Metres to Megametres, the engine's world unit.
const M_TO_MM: f32 = 1.0 / 1_000_000.0;

/// The captain's eye reference point in model space, in metres.
///
/// Measured off the left seat in the model: the cushion top sits at `y = 0.383` and the
/// seat back spans `x` in `[-0.723, -0.365]`, `z` in `[-1.11, -0.84]`. That puts a seated
/// pilot on the seat centreline (`x = -0.55`), roughly 0.78 m above the cushion
/// (`y = 1.16`) and a little forward of the backrest (`z = -1.28`). Cross-checked against
/// the HUD combiners (`HMD`), two flat quads at `z = -1.595`, `y` in `[1.138, 1.334]`,
/// which land 0.31 m ahead of that point and level with the eye line, as they should.
pub const EYE_LOCAL_M: Vec3 = Vec3::new(-0.55, 1.16, -1.28);

/// Position of the model's origin in the aircraft's local frame, in Megametres.
///
/// Offsetting by the eye point puts [`EYE_LOCAL_M`] exactly where the camera sits.
pub fn model_origin_offset_mm() -> Vec3 {
    CAMERA_LOCAL_MM - EYE_LOCAL_M * M_TO_MM
}

/// Uniform scale taking the model's metres to engine Megametres.
pub const MODEL_SCALE: f32 = M_TO_MM;

/// Base colours for materials the GLB leaves white.
///
/// The file's only image is a 1x1 white pixel, so every material that references a texture
/// resolves to white, as do the materials that omit `baseColorFactor` entirely. These are
/// the replacements; anything not listed falls back to [`FALLBACK_TINT`]. Materials that do
/// carry a real colour (`Pedestal_Black`, `Pedestal_Red`, ...) are left alone.
const COCKPIT_TINTS: &[(&str, [f32; 4])] = &[
    // Instrument screens read as dark glass, not white panels.
    ("Main_Display", [0.05, 0.07, 0.09, 1.0]),
    ("Side_Display", [0.05, 0.07, 0.09, 1.0]),
    ("ModeControl_Panel1", [0.22, 0.23, 0.24, 1.0]),
    ("Console_Glay", [0.42, 0.42, 0.43, 1.0]),
    ("material_0", [0.55, 0.55, 0.56, 1.0]),
    ("Overhed_Panel", [0.60, 0.60, 0.61, 1.0]),
    ("SidePanel", [0.55, 0.55, 0.56, 1.0]),
    ("pedestal_main", [0.62, 0.62, 0.63, 1.0]),
    ("pedestal_01", [0.62, 0.62, 0.63, 1.0]),
    ("DreamLiner_LOGO1", [0.55, 0.55, 0.56, 1.0]),
    ("Interior_White", [0.80, 0.80, 0.80, 1.0]),
    ("Pedestal_White", [0.78, 0.78, 0.78, 1.0]),
    // The HUD combiners are flagged as blended but their factor is opaque, so they would
    // render as solid rectangles 0.30 m in front of the eyes. Alpha 0 drops them.
    ("HMD", [0.0, 0.0, 0.0, 0.0]),
];

const FALLBACK_TINT: [f32; 4] = [0.62, 0.62, 0.63, 1.0];

/// Anything below this stays as authored.
const WHITE_THRESHOLD: f32 = 0.99;

fn material_tint(name: Option<&str>, base: [f32; 4]) -> [f32; 4] {
    let is_white =
        base[0] >= WHITE_THRESHOLD && base[1] >= WHITE_THRESHOLD && base[2] >= WHITE_THRESHOLD;
    if !is_white {
        return base;
    }

    let tint = name
        .and_then(|n| COCKPIT_TINTS.iter().find(|(key, _)| *key == n))
        .map(|(_, tint)| *tint)
        .unwrap_or(FALLBACK_TINT);

    // Keep the authored alpha unless the table deliberately zeroes it to drop the mesh.
    [tint[0], tint[1], tint[2], tint[3].min(base[3])]
}

/// Loads the interior, or `None` if the asset is missing or malformed.
pub fn load(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    config: &wgpu::SurfaceConfiguration,
    camera_bind_group_layout: &wgpu::BindGroupLayout,
) -> Option<ModelRenderer> {
    let bytes = match crate::assets::load(ASSET_NAME) {
        Some(bytes) => bytes,
        None => {
            log::error!("Cockpit model '{}' not found in assets", ASSET_NAME);
            return None;
        }
    };

    let options = ModelOptions {
        // Real-scale interior: keep the GLB's metres and skip the screen-size boost.
        normalize_to_unit_radius: false,
        // Every material in this model is double sided, and we are inside the mesh.
        cull_mode: None,
        // The whole model is one unsorted draw call that writes depth, so blended
        // geometry would occlude what sits behind it. This drops `_787_Glass`
        // (alpha 0.098) and, via the tint table, the HUD combiners.
        skip_alpha_below: 0.5,
        material_override: Some(&material_tint),
        label: "787 cockpit",
    };

    match ModelRenderer::new_with_options(
        device,
        queue,
        config,
        camera_bind_group_layout,
        &bytes,
        options,
    ) {
        Ok(renderer) => {
            log::info!("Cockpit model '{}' loaded", ASSET_NAME);
            Some(renderer)
        }
        Err(e) => {
            log::error!("Failed to build cockpit ModelRenderer: {:?}", e);
            None
        }
    }
}
