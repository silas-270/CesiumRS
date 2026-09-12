//! Everything specific to the Airbus A350-1000 GLB drawn as the exterior aircraft.
//!
//! Keeping the model's quirks here leaves `cesium-engine` a generic glTF renderer, the
//! same way [`crate::cockpit_model`] does for the interior.
//!
//! ## Model conventions
//! The GLB is Y-up with **+Z forward** — the nose is at `z = +35.65`, the fin tip at
//! `z = -37.93` — which is the opposite of the aircraft frame, hence
//! [`YAW_CORRECTION`]. Its units are metres, and they are the real airframe's:
//! 64.74 m span, 74.12 m long, 16.50 m tall.
//!
//! It carries 137 nodes with no mesh attached — `nav_l`, `beacon`, `landing_r`,
//! `taxi_nosecam_r` and friends, the light and camera attachment points of the flight-sim
//! model it was converted from. The mesh walk in `model_pipeline` only emits geometry for
//! nodes that have a mesh, so they cost nothing. They are worth keeping: they are exact
//! positions to hang real nav, strobe and landing lights off later.
//!
//! The mesh this replaced, `A350.glb`, is still in the repository and nothing references
//! it. It is kept deliberately, as something to fall back to.

use cesium_engine::render::model_pipeline::pipeline::{ModelOptions, ModelRenderer};

/// The model, baked into the binary.
const GLB: &[u8] = include_bytes!("../../../A350-1000.glb");

/// Yaw applied to turn the model's +Z nose into the aircraft frame's forward axis.
pub const YAW_CORRECTION: f32 = std::f32::consts::PI;

/// The point of the source mesh, in its own metres, that becomes the model origin —
/// i.e. what the aircraft rotates about and the point the flight path holds.
///
/// The mesh's own origin is not this point, and the difference is not cosmetic: it is
/// the pivot, so getting it wrong swings the whole airframe around during a turn. These
/// numbers place the origin at the same fraction of the bounding box as the A350.glb this
/// model replaces (50.0% across the span, 21.5% up, 55.8% of the way from tail to nose),
/// so the aircraft keeps rotating about the same point on its own body as before.
const ORIGIN_OFFSET_M: [f32; 3] = [0.0, -0.235_659, 2.876_352];

/// Corrective scale applied after the mesh is normalised to unit radius.
///
/// Unit-radius normalisation alone does not preserve on-screen size across a model swap,
/// because the radius depends on where the origin sits and on the airframe's proportions,
/// and this is a -1000: 8.8% longer than the roughly -900-shaped mesh it replaces, at the
/// same span. Left alone it came out 6.6% narrower across the wings.
///
/// This is the uniform scale that fits the new bounding box to the old one as closely as
/// one can — the geometric mean of the three per-axis ratios — leaving the aircraft
/// 3.9% narrower, 0.6% shorter in height and 4.7% longer than before, with its true
/// proportions intact. It lands the bounding radius at 1.029 rather than exactly 1.0,
/// which the vertex shader's `min_pixel_size` boost assumes; at 2.9% that is under a
/// third of a pixel on the 100 px floor the tracker asks for.
const POST_SCALE: f32 = 1.029_333;

/// Materials whose base colour the texture alone should decide.
///
/// Both of the model's materials are `baseColorFactor` 0.588 grey over a fully painted
/// 2048² atlas, and the shader multiplies the two together — so the livery would render
/// at 59% of its authored value before any light reached it, and the aircraft would read
/// as grubby rather than white. Forcing these to 1.0 lets the baked texture through as
/// authored. Keyed by name so a future re-export that adds an untextured material does
/// not get blown out to white along with them.
const TEXTURED_MATERIALS: &[&str] = &["material00", "material01"];

/// Cap on the livery atlas, which is authored at 2048².
///
/// The tracking camera holds the aircraft at a floor of 100 px and rarely past a few
/// hundred, so the full-size level is never the one the GPU samples: rendering the flight
/// at both caps gives bit-identical frames across all six phases of the light audit, not
/// merely similar ones. The saving is worth having on Android — with its mip chain a
/// 2048² atlas is ~22 MB of texture memory against ~5.6 MB at 1024². The GLB still
/// carries the full-size image, so raising this is a one-line change if the aircraft is
/// ever drawn much larger.
const MAX_TEXTURE_SIZE: u32 = 1024;

fn material_tint(name: Option<&str>, base: [f32; 4]) -> [f32; 4] {
    match name {
        Some(n) if TEXTURED_MATERIALS.contains(&n) => [1.0, 1.0, 1.0, base[3]],
        _ => base,
    }
}

/// Builds the exterior aircraft renderer.
pub fn load(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    config: &wgpu::SurfaceConfiguration,
    camera_bind_group_layout: &wgpu::BindGroupLayout,
) -> Result<ModelRenderer, Box<dyn std::error::Error>> {
    let options = ModelOptions {
        normalize_to_unit_radius: true,
        cull_mode: Some(wgpu::Face::Back),
        skip_alpha_below: 0.0,
        material_override: Some(&material_tint),
        primitive_normal_offset: None,
        // The model brings its own baked livery, which is the entire point of it.
        texture_override: None,
        origin_offset: ORIGIN_OFFSET_M,
        post_scale: POST_SCALE,
        // The livery is a single atlas the whole airframe already addresses correctly, and
        // nothing on the exterior is self-lit — the nav and strobe lights are not modelled.
        uv_override: None,
        material_unlit: None,
        max_texture_size: Some(MAX_TEXTURE_SIZE),
        label: "A350-1000",
    };

    ModelRenderer::new_with_options(
        device,
        queue,
        config,
        camera_bind_group_layout,
        GLB,
        options,
    )
}
