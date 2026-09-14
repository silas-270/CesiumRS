// MUST match the other file exactly — see docs/lighting.md.
const EARTH_RADIUS_MM: f32 = 6.378137;      // Mm

const PI: f32 = 3.14159265;

/// Semi-minor (polar) axis of the WGS84 ellipsoid, in Mm. `y` is the polar axis in this
/// engine's world space — see `Camera::altitude`, whose formula `ellipsoid_frame`
/// reproduces. Not part of the must-match constant above: the sky dome is a shell around
/// the whole planet and has no use for a local surface height.
const EARTH_RADIUS_B_MM: f32 = 6.3567523142;

/// Scale height of the air, in Mm: the height over which density falls by a factor of e.
/// 8km is the standard figure for Earth's troposphere, and it is the ONLY length that
/// sets how much air a view ray crosses. See `air_path_length`.
const HAZE_SCALE_HEIGHT_MM: f32 = 0.008;

/// Where a point sits relative to the ellipsoid: `xyz` is the local up (the ellipsoid
/// normal, which is NOT the direction to the Earth's centre — they differ by up to 11
/// arcminutes), `w` is the height above the surface in Mm.
///
/// A sphere of EARTH_RADIUS_MM is not good enough here: the two axes differ by 21km,
/// which is more than two scale heights of air, so at European latitudes a spherical
/// height puts both the camera and the ground *below* the surface — every height clamps
/// to zero, every view ray comes out sea-level dense along its whole length, and the
/// haze is far too strong.
fn ellipsoid_frame(p: vec3<f32>) -> vec4<f32> {
    let inv_a2 = 1.0 / (EARTH_RADIUS_MM * EARTH_RADIUS_MM);
    let inv_b2 = 1.0 / (EARTH_RADIUS_B_MM * EARTH_RADIUS_B_MM);
    let r = length(p);
    let dir = p / max(r, 1e-6);
    let surface_radius = 1.0
        / sqrt(dir.x * dir.x * inv_a2 + dir.y * dir.y * inv_b2 + dir.z * dir.z * inv_a2);
    let up = normalize(vec3<f32>(p.x * inv_a2, p.y * inv_b2, p.z * inv_a2));
    return vec4<f32>(up, r - surface_radius);
}

/// `exp(z^2) * erfc(z)`, the scaled complementary error function.
///
/// Numerical Recipes' `erfcc` rational-exponential fit, kept in scaled form: fractional
/// error below 1.2e-7 over the whole range, one `exp` and a Horner chain. The scaling
/// matters — the textbook Abramowitz & Stegun 7.1.26 polynomial bounds its error on
/// `erfc` itself, which is ABSOLUTE, so dividing by the vanishing `exp(-z^2)` leaves it
/// 38% wrong for the large arguments this is called with.
///
/// Negative `z` is the analytic continuation `2*exp(z^2) - erfcx(-z)`, and it is not an
/// edge case to be clamped away: it is exactly what `air_path_length` needs when the ray
/// dips to its lowest point *between* the camera and the fragment rather than at one of
/// them (a camera near the ground looking at distant ground). Verified against numerical
/// integration for that case too. `|z|` never exceeds ~2 for any real view.
fn erfcx(z: f32) -> f32 {
    let a = abs(z);
    let t = 1.0 / (1.0 + 0.5 * a);
    let poly = -1.26551223 + t * (1.00002368 + t * (0.37409196 + t * (0.09678418
             + t * (-0.18628806 + t * (0.27886807 + t * (-1.13520398 + t * (1.48851587
             + t * (-0.82215223 + t * 0.17087277))))))));
    let scaled = t * exp(poly);
    if (z >= 0.0) {
        return scaled;
    }
    return 2.0 * exp(min(z * z, 60.0)) - scaled;
}

/// Chapman function: how many *vertical* columns of air a ray leaving a point at radius
/// `radius_mm` and local zenith angle `chi` crosses on its way out of the atmosphere.
///
/// This is the curved-atmosphere generalisation of the schoolbook `1 / cos(chi)` air
/// mass, and the whole reason it is here is that `1 / cos(chi)` diverges at the horizon
/// while the real answer does not: at `chi = 90 degrees` it is `sqrt(pi * X / 2)` — about
/// 35 columns, ~280km of sea-level air. Everything that looks right about a horizon is in
/// that number being large but finite.
fn chapman(radius_mm: f32, cos_chi: f32) -> f32 {
    let x = radius_mm / HAZE_SCALE_HEIGHT_MM;
    return sqrt(0.5 * PI * x) * erfcx(sqrt(0.5 * x) * cos_chi);
}

/// Length of air, in Mm at sea-level density, between the camera and a fragment — the
/// only quantity distance haze may be driven by.
///
/// Air density falls off exponentially with height, so the air along a ray is bounded in
/// a way the ray's *length* is not: straight down from any altitude this returns one
/// scale height (8km — clear) however far out the camera is zoomed, while a look along
/// the ground accumulates hundreds of kilometres. Two earlier versions measured geometry
/// instead — first the raw camera-to-fragment distance, then the part of it inside a
/// 150km shell — and both grew without limit as the camera pulled back, painting the
/// whole globe with `horizon_haze_color` (which at that altitude was the space colour, so
/// the map simply vanished) the moment the user zoomed out.
///
/// The integral is `column(fragment) - column(camera)`, each column being the optical
/// depth from that point out of the atmosphere along the same line, which is what the
/// Chapman function gives. Exact for a spherical exponential atmosphere, so it holds for
/// the grazing rays a flat-slab approximation gets badly wrong: checked against numerical
/// integration from ground level to 2000km and from nadir to the horizon, worst case
/// 1.2%. Cost is two `exp`s and two `sqrt`s, no loop and no raymarch.
fn air_path_length(camera_pos: vec3<f32>, frag_pos: vec3<f32>) -> f32 {
    let to_frag = frag_pos - camera_pos;
    let ray = normalize(to_frag);

    let cam = ellipsoid_frame(camera_pos);
    let frag = ellipsoid_frame(frag_pos);
    let cam_height = max(cam.w, 0.0);
    let frag_height = max(frag.w, 0.0);

    // Both angles are measured on the ray pointing back up out of the atmosphere, so
    // both are zenith angles of an outgoing ray, which is what `chapman` expects.
    let column_frag = exp(-frag_height / HAZE_SCALE_HEIGHT_MM)
        * chapman(length(frag_pos), dot(-ray, frag.xyz));
    let column_cam = exp(-cam_height / HAZE_SCALE_HEIGHT_MM)
        * chapman(length(camera_pos), dot(-ray, cam.xyz));

    return HAZE_SCALE_HEIGHT_MM * max(column_frag - column_cam, 0.0);
}

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
    
    let sun_elevation = camera.sun_dir.w;
    let day_amount = smoothstep(0.0, 0.10, sun_elevation);
    let night_amount = smoothstep(-0.02, -0.22, sun_elevation);
    let twilight = (1.0 - day_amount) * (1.0 - night_amount);

    let frag_pos = camera.camera_pos.xyz + in.world_pos;
    let true_frag_dist = length(in.world_pos);
    let view_dir_terrain = in.world_pos / max(true_frag_dist, 1e-6);

    let cos_sun_terrain = dot(view_dir_terrain, camera.sun_dir.xyz);
    let toward_sun_terrain = smoothstep(-0.2, 0.9, cos_sun_terrain);
    var horizon_haze_color = sky_palette(sun_elevation)[1]
        * sky_hue_rotation(sun_elevation, toward_sun_terrain);
    horizon_haze_color = mix(horizon_haze_color * 0.25, horizon_haze_color, altitude_scalar);

    let zenith_color = mix(sky_palette(sun_elevation)[0] * 0.12, sky_palette(sun_elevation)[0], altitude_scalar);
    let sky_irradiance = mix(zenith_color, horizon_haze_color, 0.4);

    // Ground ambient receives subtle twilight illumination matching the sky dome
    let twilight_ambient = sky_irradiance * twilight * mix(0.4, 0.8, altitude_scalar);
    let ambient_vec = vec3<f32>(ambient) + twilight_ambient * 2.2;
    let twilight_floor = sky_irradiance * twilight * 0.06 * mix(0.5, 1.0, altitude_scalar);

    let shaded_color = tex_color_rgb * (ambient_vec + vec3<f32>(diffuse)) * key_tint + twilight_floor;

    // ── Aerial perspective ───────────────────────────────────────────────────────
    let haze_path_length = air_path_length(camera.camera_pos.xyz, frag_pos);

    // haze_path_length is in Mm of sea-level-density air (1.0 = 1000km).
    // Tuned so distant terrain smoothly and completely blends into horizon_haze_color near
    // the horizon without a razor-sharp cutoff edge, while keeping vertical views clear.
    let AERIAL_HAZE_ONSET_MM: f32 = 0.015;
    let AERIAL_HAZE_FULL_MM: f32 = 0.085;
    let aerial_blend = smoothstep(AERIAL_HAZE_ONSET_MM, AERIAL_HAZE_FULL_MM, haze_path_length);

    let final_color = mix(shaded_color, horizon_haze_color, aerial_blend);

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
