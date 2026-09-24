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

struct SkyOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) clip_pos_xy: vec2<f32>,
};

@vertex
fn vs_sky(@builtin(vertex_index) vertex_index: u32) -> SkyOutput {
    var out: SkyOutput;
    
    // Generate full-screen triangle:
    let x = f32(i32(vertex_index) == 1) * 4.0 - 1.0;
    let y = f32(i32(vertex_index) == 2) * 4.0 - 1.0;
    
    // Set z to 0.0 to push it to the far clipping plane (Reverse-Z)
    out.clip_position = vec4<f32>(x, y, 0.0, 1.0);
    out.clip_pos_xy = vec2<f32>(x, y);
    
    return out;
}


/// Angular radius of the moon's disc, in radians.
///
/// The real one is 0.00454 (a quarter of a degree). This is that, enlarged — first to
/// about 0.81 degrees so it was visible at all, and then half again on top, which is
/// roughly what it takes for an aircraft to read as crossing it rather than passing a dot.
const MOON_ANGULAR_RADIUS: f32 = 0.0212;

/// Angular radius of the sun's disc, in radians. The real value is 0.00466
/// (~0.267°, i.e. 0.53° diameter); enlarged to ~0.49° radius for visibility —
/// smaller than the moon's enlargement since the sun is already the
/// brightest thing in frame.
const SUN_ANGULAR_RADIUS: f32 = 0.00863;
const SUN_DISC_SOFT_EDGE: f32 = 0.00004;

/// The night sky's own faint glow (airglow, starlight, light pollution), which the
/// single-scattering atmosphere has no source for once the sun is far enough down.
const NIGHT_ZENITH: vec3<f32>  = vec3<f32>(0.002, 0.002, 0.004);
const NIGHT_HORIZON: vec3<f32> = vec3<f32>(0.008, 0.009, 0.014);

/// Refraction lifts the sun near the horizon (Bennett's formula, apparent elevation in
/// degrees in, lift in degrees out). The lift is larger at the lower limb than the upper,
/// which is what squashes a setting sun into an oval.
fn refraction_lift_deg(apparent_deg: f32) -> f32 {
    let a = max(apparent_deg, -1.5);
    return (1.0 / tan(radians(a + 7.31 / (a + 4.4)))) / 60.0;
}

// ── Procedural lunar surface ──────────────────────────────────────────────────
//
// Generated rather than sampled. The sky pipeline binds nothing but the camera uniform,
// so a real texture would mean a new bind-group layout, an image in `assets/`, and a
// loading path that can fail at runtime — for a disc barely a degree across.

fn hash2(p: vec2<f32>) -> f32 {
    return fract(sin(dot(p, vec2<f32>(127.1, 311.7))) * 43758.5453123);
}

fn value_noise_2d(p: vec2<f32>) -> f32 {
    let i = floor(p);
    let f = fract(p);
    let u = f * f * (3.0 - 2.0 * f);
    let a = hash2(i);
    let b = hash2(i + vec2<f32>(1.0, 0.0));
    let c = hash2(i + vec2<f32>(0.0, 1.0));
    let d = hash2(i + vec2<f32>(1.0, 1.0));
    return mix(mix(a, b, u.x), mix(c, d, u.x), u.y);
}

fn fbm2(p: vec2<f32>) -> f32 {
    var total = 0.0;
    var amplitude = 0.5;
    var freq = p;
    for (var i = 0; i < 4; i = i + 1) {
        total = total + value_noise_2d(freq) * amplitude;
        freq = freq * 2.03;
        amplitude = amplitude * 0.5;
    }
    return total;
}

// ── Stars ─────────────────────────────────────────────────────────────────────
//
// Fixed to the celestial sphere, because the ray being hashed is already in world space:
// the field stays put as the aircraft flies and turns, which is the whole trick.

/// Cells per unit of cube face. With six faces this is a few thousand stars over the
/// whole sky, which is about what a good night gives the naked eye.
const STAR_GRID: f32 = 60.0;
/// Fraction of cells holding a star at all.
const STAR_CHANCE: f32 = 0.07;
/// Radius of a star's core, in pixels. Never allowed to fall below a pixel — a
/// sub-pixel star crawls and flickers as the camera moves, and that single artefact is
/// what makes most procedural skies look cheap.
const STAR_PIXEL_RADIUS: f32 = 1.1;

fn star_field(dir: vec3<f32>, cutoff: f32) -> vec3<f32> {
    // Cube-face projection. Cells stay roughly square everywhere, with none of the
    // crowding at the poles that a latitude/longitude grid would give.
    let a = abs(dir);
    var uv = dir.yz / max(a.x, 1e-6);
    var face = 0.0;
    if a.y >= a.x && a.y >= a.z {
        uv = dir.xz / max(a.y, 1e-6);
        face = 2.0;
    } else if a.z >= a.x && a.z >= a.y {
        uv = dir.xy / max(a.z, 1e-6);
        face = 4.0;
    }

    let g = uv * STAR_GRID;
    // How much of a cell a single pixel covers. Sizing the star against this is what
    // keeps it from ever going sub-pixel. Clamped because the value jumps at the seams
    // between cube faces.
    let footprint = clamp(max(fwidth(g.x), fwidth(g.y)), 1e-4, 0.4);

    let cell = floor(g) + vec2<f32>(face * 37.0, face * 17.0);
    let h_present = hash2(cell + vec2<f32>(0.5, 0.5));
    let h_mag = hash2(cell + vec2<f32>(3.7, 9.1));
    let h_hue = hash2(cell + vec2<f32>(6.3, 2.9));
    let jitter = vec2<f32>(
        hash2(cell + vec2<f32>(1.3, 7.7)),
        hash2(cell + vec2<f32>(8.1, 4.2))
    );

    // Brightest first as the sky darkens: `cutoff` walks down from 1 in daylight to 0 at
    // night, and only stars above it are showing. A flat distribution reads as noise, so
    // actual luminance follows a power law — a few bright ones, many faint.
    let present = step(1.0 - STAR_CHANCE, h_present);
    let shown = smoothstep(cutoff, cutoff + 0.25, h_mag);
    let magnitude = pow(h_mag, 2.2);

    let centre = clamp(jitter, vec2<f32>(0.25), vec2<f32>(0.75));
    let d = length(fract(g) - centre);
    let radius = STAR_PIXEL_RADIUS * footprint;
    let disc = smoothstep(radius, radius * 0.25, d);

    // Real stars run blue-white through amber. Subtle, but a monochrome field looks
    // printed on.
    let tint = mix(vec3<f32>(0.78, 0.85, 1.0), vec3<f32>(1.0, 0.86, 0.68), h_hue * h_hue);
    return tint * (present * shown * magnitude * disc);
}

@fragment
fn fs_sky(in: SkyOutput) -> @location(0) vec4<f32> {
    let clip_pos = vec4<f32>(in.clip_pos_xy, 1.0, 1.0);
    let world_pos = camera.inv_view_proj * clip_pos;
    let world_pos_xyz = world_pos.xyz / world_pos.w;
    let view_dir = normalize(world_pos_xyz);

    // Not daylight: this runs 1 on the runway to 0 at cruise, and is what thins the sky
    // out as the flight climbs. Daylight is `camera.sun_dir.w`.
    let altitude_scalar = camera.sun_params.x;
    let sun_dir = camera.sun_dir.xyz;
    let sun_elevation = camera.sun_dir.w;
    let cos_sun = dot(view_dir, sun_dir);

    let day_amount = smoothstep(0.0, 0.10, sun_elevation);
    let night_amount = smoothstep(-0.02, -0.22, sun_elevation);

    // ── The air ──────────────────────────────────────────────────────────────
    let origin = atmo_position(camera.camera_pos.xyz);
    let scatter = atmo_integrate(origin, view_dir, sun_dir, 1.0e9, ATMO_VIEW_STEPS);
    var radiance = atmo_radiance(scatter, cos_sun);

    // ── The sun's disc ───────────────────────────────────────────────────────
    //
    // Seen through the same air as everything else, so a low sun is reddened and dimmed
    // by exactly the transmittance of its own line of sight. Undo refraction on the view
    // ray before testing the disc: that both lifts the sun (it is still visible just
    // after it has geometrically set) and flattens it near the horizon.
    let up = normalize(origin);
    let view_elev = asin(clamp(dot(view_dir, up), -1.0, 1.0));
    let true_elev = view_elev - radians(refraction_lift_deg(degrees(view_elev)));
    let horiz = view_dir - up * dot(view_dir, up);
    let horiz_len = length(horiz);
    var unrefracted = view_dir;
    if (horiz_len > 1e-4) {
        unrefracted = horiz / horiz_len * cos(true_elev) + up * sin(true_elev);
    }
    let cos_disc = dot(unrefracted, sun_dir);
    let cos_sun_edge = cos(SUN_ANGULAR_RADIUS);
    let disc = smoothstep(cos_sun_edge - SUN_DISC_SOFT_EDGE, cos_sun_edge, cos_disc);
    let celestial_fade = mix(0.55, 1.0, altitude_scalar);
    var base_color = atmo_tonemap(radiance, sun_elevation);
    // Added over the tone-mapped sky rather than into its radiance, and on its own
    // brightness scale: a disc tens of thousands of times brighter than the sky would
    // clip white at any exposure. Its colour is the transmittance of its own line of
    // sight, normalised against the red channel so it stays a bright disc: white high
    // up, yellow, orange, then red at the horizon.
    let t_sun = scatter.transmittance;
    let disc_color = vec3<f32>(1.0) - exp(-t_sun * (6.0 / pow(max(t_sun.r, 1e-3), 0.6)));
    // Screen blend: always brighter than the sky behind it, never clipped flat.
    base_color = mix(base_color, vec3<f32>(1.0) - (vec3<f32>(1.0) - base_color) * (vec3<f32>(1.0) - disc_color), disc);

    // Altitude is a second, independent axis: the flight climbing into cruise drains the
    // sky toward space regardless of the hour — a cruise at noon must not look like a
    // taxi at noon. MUST match the globe's haze (globe_pipeline/shader.wgsl).
    base_color *= mix(0.35, 1.0, altitude_scalar);

    // The night's own faint glow, graded from zenith to horizon by the air column.
    let view_up = clamp(dot(view_dir, up), 0.0, 1.0);
    base_color += mix(NIGHT_HORIZON, NIGHT_ZENITH, sqrt(view_up)) * night_amount;

    // ── Stars ────────────────────────────────────────────────────────────────
    //
    // Attenuated by the air column and extinguished by the background sky luminance: as
    // the local sky darkens, the brightest stars emerge first at the zenith and fill the
    // sky down to the horizon.
    let sky_luminance = dot(base_color, vec3<f32>(0.2126, 0.7152, 0.0722));
    let lum_factor = smoothstep(0.005, 0.08, sky_luminance);
    // Never before the sun is ~4 degrees down, whatever the sky's brightness says: the
    // first stars come out around the middle of civil twilight.
    let star_extinction = dot(scatter.transmittance, vec3<f32>(0.2126, 0.7152, 0.0722))
        * (1.0 - lum_factor) * smoothstep(-0.07, -0.12, sun_elevation);
    base_color += star_field(view_dir, clamp(lum_factor, 0.0, 1.0)) * star_extinction;

    // The moon, with a face on it. Sitting exactly opposite the sun it is always at full,
    // which is the phase worth having: a full disc is what an aircraft crosses in the
    // photograph everyone has seen.
    //
    // Deliberately larger than life. The real moon is about a quarter of a degree in
    // radius, which at this field of view is a handful of pixels and reads as a stray
    // bright dot rather than as the moon.
    let moon_dir = camera.moon_dir.xyz;
    let cos_moon = dot(view_dir, moon_dir);
    if cos_moon > 0.999 {
        // A frame on the moon's disc, to sample its surface across.
        var tangent = cross(moon_dir, vec3<f32>(0.0, 1.0, 0.0));
        if length(tangent) < 1e-4 {
            tangent = vec3<f32>(1.0, 0.0, 0.0);
        }
        tangent = normalize(tangent);
        let bitangent = cross(moon_dir, tangent);

        let disc = vec2<f32>(dot(view_dir, tangent), dot(view_dir, bitangent))
            / MOON_ANGULAR_RADIUS;
        let r = length(disc);
        let edge = smoothstep(1.0, 0.97, r);

        if edge > 0.0 {
            // Maria: the big dark seas, smooth and low-contrast.
            let maria = smoothstep(0.44, 0.60, fbm2(disc * 1.5 + 4.7));
            // Craters and highlands: fine bright speckle over the top.
            let craters = fbm2(disc * 9.0 + 13.1);
            var surface = mix(1.0, 0.58, maria);
            surface = surface * (0.86 + 0.28 * craters);
            // The moon barely limb-darkens in reality, but a touch of it is what stops
            // the disc reading as a flat sticker.
            let limb = sqrt(max(1.0 - r * r, 0.0));
            surface = surface * mix(0.80, 1.0, pow(limb, 0.35));

            let moon_visible = camera.moon_dir.w * (1.0 - day_amount * 0.85);
            base_color += vec3<f32>(0.96, 0.95, 0.90)
                * surface * edge * moon_visible * celestial_fade;
        }
    }

    // Hash-based dither, about one 8-bit ULP. The sky's gradient is smooth and
    // mostly monochrome, and this runs for hours in the background — banding
    // gets more obvious the longer it's on screen, not less. Screen-space
    // (not per-frame) so it doesn't flicker over a session; reuses hash2 already
    // written for the star field.
    let dither = (hash2(in.clip_pos_xy * 0.5 + vec2<f32>(17.0, 41.0)) - 0.5) / 255.0;
    base_color = max(base_color + vec3<f32>(dither), vec3<f32>(0.0));
    return vec4<f32>(base_color, 1.0);
}
