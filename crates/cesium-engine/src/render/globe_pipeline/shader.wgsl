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
    // Light arriving at this piece of ground, from the sky LUT: direct sun through its
    // own air column (zero in the Earth's shadow), and skylight, both per unit of solar
    // irradiance.
    @location(3) sun_light: vec3<f32>,
    @location(4) sky_light: vec3<f32>,
};

@group(2) @binding(0)
var sky_lut: texture_2d<f32>;
@group(2) @binding(1)
var sky_lut_sampler: sampler;

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

    // Two texture fetches; the LUT holds these against the sun's cos-zenith over the
    // ground (atmosphere.wgsl, "Sky LUT").
    let up = normalize(atmo_position(camera.camera_pos.xyz + world_pos));
    let mu = dot(up, camera.sun_dir.xyz);
    out.sky_light = textureSampleLevel(sky_lut, sky_lut_sampler, ground_lut_uv(mu, 0.0), 0.0).rgb;
    out.sun_light = textureSampleLevel(sky_lut, sky_lut_sampler, ground_lut_uv(mu, 1.0), 0.0).rgb;
    return out;
}

@group(1) @binding(0)
var t_diffuse: texture_2d<f32>;
@group(1) @binding(1)
var s_diffuse: sampler;

const LUMA: vec3<f32> = vec3<f32>(0.299, 0.587, 0.114);

/// How much of the light's physical colour the ground is allowed to take on. The eye
/// (and every camera) white-balances toward the light it is in, so a golden-hour field
/// reads warm, not orange, and a blue-hour one cool, not blue. 1.0 would be the raw
/// physical ratio, which makes the ground look dyed.
const GROUND_TINT_STRENGTH: f32 = 0.5;

/// Normalises a light colour to unit luminance, then pulls it toward white.
fn white_balanced(light: vec3<f32>, strength: f32) -> vec3<f32> {
    let lum = dot(light, LUMA);
    if (lum < 1e-6) {
        return vec3<f32>(1.0);
    }
    let hue = clamp(light / lum, vec3<f32>(0.0), vec3<f32>(3.0));
    return mix(vec3<f32>(1.0), hue, strength);
}

@fragment
fn fs_solid(in: VertexOutput) -> @location(0) vec4<f32> {
    // This is the flight's depth, not daylight: 1 on the runway, 0 at cruise. It flattens
    // the terrain out as the aircraft climbs — all ambient and no shading — which is the
    // deliberate "deep focus" look. Daylight is a separate axis entirely, below.
    let altitude_scalar = camera.sun_params.x;

    let sun_elevation = camera.sun_dir.w;
    let day_amount = smoothstep(0.0, 0.10, sun_elevation);
    let night_amount = smoothstep(-0.02, -0.22, sun_elevation);
    // The eye keeps colour well into civil twilight; the grey, moonlit look only takes
    // over once it is properly dark. Starting it at sunset (with `night_amount`) is what
    // used to drain the ground to grey while the sky was still on fire.
    let scotopic = smoothstep(-0.08, -0.25, sun_elevation);

    // ── Light arriving at the ground ──────────────────────────────────────────
    //
    // Both lights come from the same atmosphere the sky is drawn with (vs_main). The
    // direct sun is `in.sun_light` — reddened and dimmed by its own air column, and zero
    // once this ground is in the Earth's shadow; the skylight is `in.sky_light`.
    let n_dot_sun = dot(in.normal, camera.sun_dir.xyz);
    let n_dot_moon = dot(in.normal, camera.moon_dir.xyz);
    let sun_lum = dot(in.sun_light, LUMA);

    // Horizontal-ground irradiance, per unit of solar irradiance. The +0.25 on the sun
    // stands for everything on real ground that faces a low sun — trees, walls, hedges —
    // which a flat satellite image cannot show but which is most of why a golden-hour
    // landscape reads warm.
    let up_local = normalize(camera.camera_pos.xyz + in.world_pos);
    let sun_up = max(dot(up_local, camera.sun_dir.xyz), 0.0) + 0.25;
    let e_direct = in.sun_light * sun_up;
    let e_sky = in.sky_light;
    let light_tint = white_balanced(e_direct + e_sky, GROUND_TINT_STRENGTH);
    let sun_tint = white_balanced(in.sun_light, GROUND_TINT_STRENGTH);

    // Brightness keeps the long-standing schedule (flat and bright in daylight, a soft
    // moonlit floor at night), with golden hour a little dimmer than noon.
    let day_ambient = mix(0.70, 0.58, altitude_scalar);
    let night_ambient = mix(0.18, 0.26, altitude_scalar);
    // The ground darkens with the light, from just before sunset to the end of civil
    // twilight — not on `night_amount`, which kept it at full daylight brightness under
    // a dusk sky.
    let dusk = smoothstep(0.04, -0.18, sun_elevation);
    let base_ambient = mix(day_ambient, night_ambient, dusk) * mix(0.9, 1.0, day_amount);
    let moon_tint = vec3<f32>(0.80, 0.86, 0.98);
    let ambient_tint = mix(light_tint, moon_tint, scotopic);
    let ambient_rgb = base_ambient * ambient_tint;

    // Directional sun, strength following how much sunlight actually gets through.
    let wrap_sun = max((n_dot_sun + 0.12) / 1.12, 0.0);
    let sun_strength = sqrt(clamp(sun_lum, 0.0, 1.0));
    let from_sun = wrap_sun * sun_strength;
    let from_moon = max(n_dot_moon, 0.0) * night_amount;
    let day_diffuse_max = 0.38 * mix(0.2, 1.0, altitude_scalar);
    let night_diffuse_max = 0.08;
    let diffuse_rgb = from_sun * day_diffuse_max * sun_tint
        + vec3<f32>(from_moon * night_diffuse_max);

    let tex_color_raw = textureSample(t_diffuse, s_diffuse, in.uv);

    // Extract map color grading parameters from the uniform (-1.0 to 1.0)
    let saturation_adj = camera.sun_params.y;
    let contrast_adj = camera.sun_params.z;
    let brightness_adj = camera.sun_params.w;

    var tex_color_rgb = tex_color_raw.rgb;

    // 1. Daytime highlight compression: gently roll off extreme whites (runway concrete/roofs)
    // so textures preserve surface detail without blowing out into blinding white patches.
    let highlight_excess = max(tex_color_rgb - vec3<f32>(0.75), vec3<f32>(0.0));
    tex_color_rgb = tex_color_rgb - highlight_excess * 0.45 * day_amount;

    // 2. Night tone curve (mesopic response): daytime sun-patches in photographic tiles
    // are suppressed and the landscape takes on a soft, dark, monotone moonlit presence.
    // Gated to brighter textures (photographic satellite tiles) so dark vector basemaps
    // (Dark Matter) stay legible.
    let night_lum = dot(tex_color_rgb, LUMA);
    let photo_gate = smoothstep(0.04, 0.35, night_lum);
    let night_gamma = mix(1.0, 1.35, scotopic * photo_gate);
    tex_color_rgb = pow(tex_color_rgb, vec3<f32>(night_gamma));
    let moonlit_gray = vec3<f32>(dot(tex_color_rgb, LUMA));
    tex_color_rgb = mix(tex_color_rgb, moonlit_gray, scotopic * 0.80 * photo_gate);

    // Performance optimization: skip explicit color grading entirely if all adjustments are 0.0
    if (saturation_adj != 0.0 || contrast_adj != 0.0 || brightness_adj != 0.0) {
        if (brightness_adj != 0.0) {
            let multiplier = max(1.0 + brightness_adj * 2.0, 0.0);
            tex_color_rgb = tex_color_rgb * multiplier;
        }
        if (contrast_adj != 0.0) {
            let contrast_factor = max(1.0 + contrast_adj, 0.0);
            tex_color_rgb = (tex_color_rgb - 0.5) * contrast_factor + 0.5;
        }
        if (saturation_adj != 0.0) {
            let luminance = dot(tex_color_rgb, LUMA);
            let saturation_factor = max(1.0 + saturation_adj, 0.0);
            tex_color_rgb = mix(vec3<f32>(luminance), tex_color_rgb, saturation_factor);
        }
    }
    tex_color_rgb = clamp(tex_color_rgb, vec3<f32>(0.0), vec3<f32>(1.0));

    let shaded_color = tex_color_rgb * (ambient_rgb + diffuse_rgb);

    // ── Aerial perspective ───────────────────────────────────────────────────────
    let frag_pos = camera.camera_pos.xyz + in.world_pos;

    // haze_path_length is in Mm of sea-level-density air (1.0 = 1000km).
    // Pushed further out so near and mid-distance terrain stays crisp and clear,
    // while distant terrain smoothly transitions into horizon_haze_color at the horizon.
    let haze_path_length = air_path_length(camera.camera_pos.xyz, frag_pos);
    let AERIAL_HAZE_ONSET_MM: f32 = 0.035;
    let AERIAL_HAZE_FULL_MM: f32 = 0.160;
    let aerial_blend = smoothstep(AERIAL_HAZE_ONSET_MM, AERIAL_HAZE_FULL_MM, haze_path_length);

    // The colour it fades into is the sky the terrain hides: the sky LUT in the same
    // direction, whose rows below the horizon are exactly the air in front of the ground.
    // Only looked up where there is haze to show.
    var horizon_haze_color = vec3<f32>(0.0);
    if (aerial_blend > 0.0) {
        let cam = atmo_position(camera.camera_pos.xyz);
        let r_cam = length(cam);
        let view = sky_lut_view(normalize(in.world_pos), cam / r_cam, camera.sun_dir.xyz);
        let radiance = textureSampleLevel(sky_lut, sky_lut_sampler,
            sky_lut_uv(view.x, view.y, r_cam), 0.0).rgb;
        // MUST match sky.wgsl's altitude darkening.
        horizon_haze_color = atmo_tonemap(radiance, sun_elevation) * mix(0.35, 1.0, altitude_scalar);
    }

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
