// MUST match the other file exactly — see docs/lighting.md.
const EARTH_RADIUS_MM: f32 = 6.378137;      // Mm
const ATMOSPHERE_THICKNESS_MM: f32 = 0.15;  // Mm; 150km shell for the sky raymarch

// ── Shared sky palette ──────────────────────────────────────────────────
// MUST match sky_pipeline/sky.wgsl / globe_pipeline/shader.wgsl exactly (this
// is the other one). See docs/lighting.md, "The sky must agree with the
// globe at the horizon." Both files call these with the same sun_elevation
// (camera.sun_dir.w) and, for sky_hue_rotation, the same toward_sun remap
// constants (-0.2, 0.9) at each call site — that remap is a local, not part
// of the shared function body, so it has to be kept in sync by hand too.
// Edit all copies in the same commit; verify with light_audit_sweep before
// trusting.

const NOON_ZENITH: vec3<f32>   = vec3<f32>(0.15, 0.35, 0.75);
const NOON_HORIZON: vec3<f32>  = vec3<f32>(0.70, 0.80, 0.90);
const NIGHT_ZENITH: vec3<f32>  = vec3<f32>(0.012, 0.012, 0.014);
const NIGHT_HORIZON: vec3<f32> = vec3<f32>(0.055, 0.057, 0.062);

/// Deep blue-violet the zenith picks up during civil twilight, instead of
/// just fading toward the near-black NIGHT_ZENITH — a clear dusk zenith
/// stays saturated blue-violet for a while after sunset, which a straight
/// day-to-night fade can't show. Hand-picked from clear dusk reference
/// photographs (Wikimedia Commons, inspected during this work).
const TWILIGHT_ZENITH_VIOLET: vec3<f32> = vec3<f32>(0.10, 0.08, 0.28);
/// How far toward TWILIGHT_ZENITH_VIOLET the zenith moves at full twilight.
/// Not 1.0: a full replacement read as an unmotivated colour swap against
/// light_audit_sweep — starting point for tuning, not a derived number.
const TWILIGHT_ZENITH_MIX: f32 = 0.6;

// Relative Rayleigh scattering weight per channel (R,G,B), normalised to
// green = 1, from inverse-4th-power-of-wavelength coefficients for
// 680/550/440nm air. Used only as a relative HUE weight below, not as a real
// extinction term — see the plan's "design decision" note for why a literal
// Beer-Lambert transmittance here would darken toward black instead of glow.
const RAYLEIGH_WEIGHT: vec3<f32> = vec3<f32>(0.43, 1.0, 2.45);

/// `sun_elevation` is sin(elevation), carried on camera.sun_dir.w
/// (render/celestial.rs), not radians. Returns [zenith, horizon].
fn sky_palette(sun_elevation: f32) -> array<vec3<f32>, 2> {
    let day_amount = smoothstep(0.0, 0.10, sun_elevation); // must match celestial.rs
    let night_amount = smoothstep(-0.02, -0.22, sun_elevation); // must match celestial.rs
    var zenith = mix(NOON_ZENITH, NIGHT_ZENITH, night_amount);
    let horizon = mix(NOON_HORIZON, NIGHT_HORIZON, night_amount);

    // Twilight-only violet cast — see TWILIGHT_ZENITH_VIOLET above. Same
    // "not day AND not night" trapezoid sky_hue_rotation gates on below, so
    // it appears and disappears on the same schedule as the rest of the
    // twilight-only colour and vanishes at noon and at full night.
    let twilight = (1.0 - day_amount) * (1.0 - night_amount);
    zenith = mix(zenith, TWILIGHT_ZENITH_VIOLET, twilight * TWILIGHT_ZENITH_MIX);

    return array<vec3<f32>, 2>(zenith, horizon);
}

/// Multiplicative hue rotation on the horizon colour, split by which side of
/// the sky a fragment is on relative to the sun (`toward_sun`: 1 = the sun's
/// half, 0 = the antisolar half).
///
/// Replaces the old direction-blind `sky_warm_tint`, which only ever warmed
/// the sun's side and left the antisolar side untouched. A real dusk sky
/// also cools toward a saturated blue opposite the sun — the "Earth's
/// shadow" band sitting under the pink "Belt of Venus" the glow band puts
/// higher in the sky (see fs_sky's glow-band code below). Both sides fade to
/// neutral (1,1,1) outside the twilight window via the same trapezoid the
/// old function used — verified against light_audit_sweep to reproduce the
/// old dusk timing exactly on the sun side.
///
/// MUST match the other file's copy of this function, AND the `-0.2, 0.9`
/// remap used to build `toward_sun` at every call site (sky.wgsl's
/// `toward_sun`, globe_pipeline/shader.wgsl's `toward_sun_terrain`) — see
/// docs/lighting.md.
fn sky_hue_rotation(sun_elevation: f32, toward_sun: f32) -> vec3<f32> {
    let day_amount = smoothstep(0.0, 0.10, sun_elevation); // must match celestial.rs
    let night_amount = smoothstep(-0.02, -0.22, sun_elevation); // must match celestial.rs
    let twilight = (1.0 - day_amount) * (1.0 - night_amount);
    // 1/RAYLEIGH_WEIGHT rescaled so the reddest channel lands on the old
    // hand-picked tint's peak (1.35) — the one number here still tuned by
    // eye, down from a whole extra hand-authored RGB key.
    let warm = (1.0 / RAYLEIGH_WEIGHT) * (1.35 / 2.3256);
    // Earth's-shadow blue — hand-tuned from reference photographs, not
    // derived from RAYLEIGH_WEIGHT: the shadow band is the daytime sky's own
    // colour seen through the Earth's shadow, not a scattering-hue
    // relationship, so there's no channel weight to invert here.
    let shadow = vec3<f32>(0.55, 0.62, 0.95);
    let side_tint = mix(shadow, warm, toward_sun);
    return mix(vec3<f32>(1.0), side_tint, twilight);
}

struct CameraUniform {
    view_proj: mat4x4<f32>,
    inv_view_proj: mat4x4<f32>,
    camera_pos: vec4<f32>,
    sun_params: vec4<f32>,   // [altitude_scalar, saturation, contrast, brightness]
    sun_dir: vec4<f32>,      // xyz toward the sun, w = sin(elevation)
    moon_dir: vec4<f32>,     // xyz toward the moon, w = lit fraction
    light_color: vec4<f32>,  // rgb key light hue, w = strength
};
@group(0) @binding(0)
var<uniform> camera: CameraUniform;

struct VertexInput {
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) uv: vec2<f32>,
};

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) normal: vec3<f32>,
    @location(1) uv: vec2<f32>,
    @location(2) world_pos: vec3<f32>,
};

struct PushConstants {
    relative_center: vec3<f32>,
    uv_scale_offset: vec4<f32>,
}
var<push_constant> push_constants: PushConstants;

@vertex
fn vs_main(model: VertexInput) -> VertexOutput {
    var out: VertexOutput;
    let world_pos = model.position + push_constants.relative_center;
    out.clip_position = camera.view_proj * vec4<f32>(world_pos, 1.0);
    out.normal = model.normal;
    out.uv = model.uv * push_constants.uv_scale_offset.xy + push_constants.uv_scale_offset.zw;
    out.world_pos = world_pos;
    return out;
}

@group(1) @binding(0)
var t_diffuse: texture_2d<f32>;
@group(1) @binding(1)
var s_diffuse: sampler;

@fragment
fn fs_solid(in: VertexOutput) -> @location(0) vec4<f32> {
    // This is the flight's depth, not daylight: 1 on the runway, 0 at cruise. It flattens
    // the terrain out as the aircraft climbs — all ambient and no shading — which is the
    // deliberate "deep focus" look. Daylight is a separate axis entirely, below.
    let altitude_scalar = camera.sun_params.x;

    let key_color = camera.light_color.rgb;
    let key_strength = camera.light_color.a;

    // Sun and moon summed with smooth weights, never switched between — see the note in
    // model_pipeline/shader.wgsl. The moon is exactly opposite the sun, so a threshold
    // here would swing the terrain's shading right round in a single frame.
    let night_key = smoothstep(-0.02, -0.22, camera.sun_dir.w);
    let from_sun = max(dot(in.normal, camera.sun_dir.xyz), 0.0) * (1.0 - night_key);
    let from_moon = max(dot(in.normal, camera.moon_dir.xyz), 0.0) * night_key;

    let ambient = mix(1.0, 0.8, altitude_scalar);
    let diffuse = (from_sun + from_moon) * mix(0.0, 0.4, altitude_scalar) * key_strength;
    // Tinting only the directional term keeps a shaded slope neutral while a sunlit one
    // goes warm — the ground then agrees with the sky instead of fighting it.
    let key_tint = mix(vec3<f32>(1.0, 1.0, 1.0), key_color, diffuse);
    
    let tex_color_raw = textureSample(t_diffuse, s_diffuse, in.uv);
    
    // Extract map color grading parameters from the uniform (-1.0 to 1.0)
    let saturation_adj = camera.sun_params.y;
    let contrast_adj = camera.sun_params.z;
    let brightness_adj = camera.sun_params.w;
    
    var tex_color_rgb = tex_color_raw.rgb;
    
    // Performance optimization: skip color grading entirely if all adjustments are 0.0
    if (saturation_adj != 0.0 || contrast_adj != 0.0 || brightness_adj != 0.0) {
        
        // 1. Brightness (-1 to 1) -> multiplicative scaling
        if (brightness_adj != 0.0) {
            let multiplier = max(1.0 + brightness_adj * 2.0, 0.0);
            tex_color_rgb = tex_color_rgb * multiplier;
        }
        
        // 2. Contrast (-1 to 1) 
        // We map -1 to 1 into a multiplier: >0 scales up (e.g., 1.0 -> factor 2.0), <0 scales down.
        if (contrast_adj != 0.0) {
            let contrast_factor = max(1.0 + contrast_adj, 0.0);
            tex_color_rgb = (tex_color_rgb - 0.5) * contrast_factor + 0.5;
        }
        
        // 3. Saturation (-1 to 1)
        // Convert to grayscale using luminance weights, then interpolate based on saturation factor.
        if (saturation_adj != 0.0) {
            let luminance = dot(tex_color_rgb, vec3<f32>(0.299, 0.587, 0.114));
            let saturation_factor = max(1.0 + saturation_adj, 0.0);
            tex_color_rgb = mix(vec3<f32>(luminance), tex_color_rgb, saturation_factor);
        }
        
        tex_color_rgb = clamp(tex_color_rgb, vec3<f32>(0.0), vec3<f32>(1.0));
    }
    
    let shaded_color = tex_color_rgb * (ambient + diffuse) * key_tint;
    
    // Grazing angle between the view ray and the local surface normal: 1.0
    // looking straight down, 0.0 exactly edge-on at the true visual horizon of a
    // smooth sphere (same geometric fact culling's own horizon test relies on —
    // zero terrain relief today). Two dot products, resolution/FOV-independent,
    // replacing a screen-space-derivative heuristic.
    let cam_to_frag = camera.camera_pos.xyz - in.world_pos;
    let frag_dist = length(cam_to_frag);
    let to_camera = cam_to_frag / max(frag_dist, 1e-6);
    let grazing_cos = max(dot(normalize(in.normal), to_camera), 0.0);

    let HORIZON_HAZE_LOWER: f32 = 0.03; // full haze below this — tune against light_audit_sweep
    let HORIZON_HAZE_UPPER: f32 = 0.20; // no haze above this
    // smoothstep needs low < high (docs/lighting.md's WGSL gotchas) — invert with
    // 1.0 - … rather than swapping the arguments.
    var horizon_blend = 1.0 - smoothstep(HORIZON_HAZE_LOWER, HORIZON_HAZE_UPPER, grazing_cos);

    // `in.world_pos` is already camera-relative (vs_main adds push_constants.relative_center,
    // which wgpu_state.rs sets to tile_center - camera_pos — see TilePushConstants), so it
    // IS the true camera-to-fragment vector directly. This is deliberately a second,
    // separate pair from `cam_to_frag`/`frag_dist`/`to_camera` above: those mix that
    // camera-relative position with the *absolute* `camera.camera_pos.xyz`, which is only
    // meaningful as a direction (for the grazing test above, which only ever normalizes it)
    // — their `frag_dist` is actually close to the camera's own distance from the Earth's
    // centre, not the distance to this fragment, so it can't be reused for a real
    // distance-based effect. Used below for both the aerial haze and the terrain's
    // sun-relative direction.
    let true_frag_dist = length(in.world_pos);
    let view_dir_terrain = in.world_pos / max(true_frag_dist, 1e-6);

    // Aerial perspective: distance haze independent of viewing angle, layered on top of
    // the grazing-angle ring above, which alone left far terrain crisp until a sudden
    // fog wall right at the silhouette — terrain is visible 50-370km away at cruise
    // (horizon distance from ~10.7km altitude), so that ring alone isn't enough.
    // true_frag_dist is in Mm (1.0 = 1000km); onset/full below are tuned to real haze
    // becoming noticeable over tens of km, not to ATMOSPHERE_THICKNESS_MM (that's the
    // sky dome's raymarch shell, a different scale).
    let AERIAL_HAZE_ONSET_MM: f32 = 0.05; // 50km — tune against light_audit_sweep
    let AERIAL_HAZE_FULL_MM: f32 = 0.30;  // 300km
    let aerial_blend = smoothstep(AERIAL_HAZE_ONSET_MM, AERIAL_HAZE_FULL_MM, true_frag_dist);

    let earth_radius = EARTH_RADIUS_MM;
    let r_cam = max(length(camera.camera_pos.xyz), earth_radius);
    let altitude = max(r_cam - earth_radius, 0.0);

    // The haze at the limb has to agree with the sky drawn behind it, so it follows the
    // sun's elevation on the same ramp rather than the altitude scalar.
    let sun_elevation = camera.sun_dir.w;

    // `view_dir_terrain` (computed above) is the camera's true view ray toward this
    // fragment, so this is exactly the "which half of the sky is the sun in" signal
    // sky.wgsl's `cos_sun`/`toward_sun` give the dome. MUST match sky.wgsl's toward_sun
    // remap (-0.2, 0.9) — see docs/lighting.md.
    let cos_sun_terrain = dot(view_dir_terrain, camera.sun_dir.xyz);
    let toward_sun_terrain = smoothstep(-0.2, 0.9, cos_sun_terrain);
    var horizon_haze_color = sky_palette(sun_elevation)[1]
        * sky_hue_rotation(sun_elevation, toward_sun_terrain);
    // MUST match sky.wgsl exactly. The terrain's limb haze and the sky behind it meet at
    // the horizon, so any difference between them shows up as a hard line across the
    // whole view. They used to agree for free by both keying off the altitude scalar;
    // once the sky moved to a time-of-day ramp, this had to be dimmed the same way.
    horizon_haze_color = mix(horizon_haze_color * 0.25, horizon_haze_color, altitude_scalar);

    let space_color = vec3<f32>(0.02, 0.02, 0.04);
    let space_fade = clamp(
        (altitude - ATMOSPHERE_THICKNESS_MM / 3.0) / (ATMOSPHERE_THICKNESS_MM * 3.0),
        0.0, 1.0);
    horizon_haze_color = mix(horizon_haze_color, space_color, space_fade);

    // max(), not add or sequential mix: a fragment that's both far away AND near the
    // grazing-angle silhouette gets one full haze blend, not a stacked double-fade.
    let final_color = mix(shaded_color, horizon_haze_color, max(horizon_blend, aerial_blend));

    return vec4<f32>(final_color, tex_color_raw.a);
}

@fragment
fn fs_wireframe(in: VertexOutput) -> @location(0) vec4<f32> {
    // Light gray color for the wireframe overlay
    return vec4<f32>(0.7, 0.7, 0.7, 0.5);
}

// --- DEBUG LINE RENDERING ---

struct DebugVertexInput {
    @location(0) position: vec3<f32>,
    @location(1) color: vec4<f32>,
};

struct DebugVertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) color: vec4<f32>,
};

@vertex
fn vs_debug(model: DebugVertexInput) -> DebugVertexOutput {
    var out: DebugVertexOutput;
    out.clip_position = camera.view_proj * vec4<f32>(model.position, 1.0);
    out.color = model.color;
    return out;
}

@fragment
fn fs_debug(in: DebugVertexOutput) -> @location(0) vec4<f32> {
    return in.color;
}
