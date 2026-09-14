// MUST match the other file exactly — see docs/lighting.md.
const EARTH_RADIUS_MM: f32 = 6.378137;      // Mm
const ATMOSPHERE_THICKNESS_MM: f32 = 0.15;  // Mm; 150km shell for the sky raymarch

// ── Shared sky palette ──────────────────────────────────────────────────
// MUST match sky_pipeline/sky.wgsl / globe_pipeline/shader.wgsl exactly (this
// is the other one). See docs/lighting.md, "The sky must agree with the
// globe at the horizon." Both files call these with the same sun_elevation
// (camera.sun_dir.w) so the seam is enforced by construction. Edit both
// copies in the same commit; verify with light_audit_sweep before trusting.

const NOON_ZENITH: vec3<f32>   = vec3<f32>(0.15, 0.35, 0.75);
const NOON_HORIZON: vec3<f32>  = vec3<f32>(0.70, 0.80, 0.90);
const NIGHT_ZENITH: vec3<f32>  = vec3<f32>(0.012, 0.012, 0.014);
const NIGHT_HORIZON: vec3<f32> = vec3<f32>(0.055, 0.057, 0.062);

// Relative Rayleigh scattering weight per channel (R,G,B), normalised to
// green = 1, from inverse-4th-power-of-wavelength coefficients for
// 680/550/440nm air. Used only as a relative HUE weight below, not as a real
// extinction term — see the plan's "design decision" note for why a literal
// Beer-Lambert transmittance here would darken toward black instead of glow.
const RAYLEIGH_WEIGHT: vec3<f32> = vec3<f32>(0.43, 1.0, 2.45);

/// `sun_elevation` is sin(elevation), carried on camera.sun_dir.w
/// (render/celestial.rs), not radians. Returns [zenith, horizon].
fn sky_palette(sun_elevation: f32) -> array<vec3<f32>, 2> {
    let night_amount = smoothstep(-0.02, -0.22, sun_elevation); // must match celestial.rs
    let zenith = mix(NOON_ZENITH, NIGHT_ZENITH, night_amount);
    let horizon = mix(NOON_HORIZON, NIGHT_HORIZON, night_amount);
    return array<vec3<f32>, 2>(zenith, horizon);
}

/// The dusk glow as a multiplicative tint on the horizon colour.
///
/// **Deviation from the plan as written:** the plan's snippet computed
/// `twilight` as `day_amount * (1-day_amount) * 4`, a bell curve entirely
/// inside `sun_elevation` ∈ [0, DAY_ELEVATION]. That's zero for the whole
/// negative-elevation dusk-to-night band (DUSK_ELEVATION..NIGHT_ELEVATION in
/// celestial.rs) — exactly the regime the old hand-authored
/// `dusk_horizon_color` used to dominate — so light_audit_sweep's
/// `02_sunset` frames rendered with no warmth at all (flat blue). The
/// trapezoid below is "not day AND not night", which is what the old
/// sequential day/dusk/night mix actually produced as its dusk weight; at
/// full weight (twilight=1) it reproduces the old dusk_horizon_color to
/// within ~0.005 per channel against NOON_HORIZON, confirming the 1.35/2.3256
/// scale below was tuned assuming this shape. Verified against
/// light_audit_sweep. MUST match the other file's copy of this function.
fn sky_warm_tint(sun_elevation: f32) -> vec3<f32> {
    let day_amount = smoothstep(0.0, 0.10, sun_elevation); // must match celestial.rs
    let night_amount = smoothstep(-0.02, -0.22, sun_elevation); // must match celestial.rs
    let twilight = (1.0 - day_amount) * (1.0 - night_amount);
    // 1/RAYLEIGH_WEIGHT rescaled so the reddest channel lands on the old
    // hand-picked tint's peak (1.35) — the one number here still tuned by
    // eye, down from a whole extra hand-authored RGB key.
    let warm = (1.0 / RAYLEIGH_WEIGHT) * (1.35 / 2.3256);
    return mix(vec3<f32>(1.0), warm, twilight);
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

    let earth_radius = EARTH_RADIUS_MM;
    let r_cam = max(length(camera.camera_pos.xyz), earth_radius);
    let altitude = max(r_cam - earth_radius, 0.0);

    // The haze at the limb has to agree with the sky drawn behind it, so it follows the
    // sun's elevation on the same ramp rather than the altitude scalar.
    let sun_elevation = camera.sun_dir.w;
    var horizon_haze_color = sky_palette(sun_elevation)[1] * sky_warm_tint(sun_elevation);
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

    let final_color = mix(shaded_color, horizon_haze_color, horizon_blend);

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
