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

fn ray_sphere_intersect(r0: vec3<f32>, rd: vec3<f32>, radius: f32) -> vec2<f32> {
    let b = 2.0 * dot(rd, r0);
    let c = dot(r0, r0) - radius * radius;
    let d = b * b - 4.0 * c;
    if (d < 0.0) {
        return vec2<f32>(-1.0, -1.0);
    }
    let d_sqrt = sqrt(d);
    return vec2<f32>((-b - d_sqrt) / 2.0, (-b + d_sqrt) / 2.0);
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
    
    let origin = camera.camera_pos.xyz;
    // Not daylight: this runs 1 on the runway to 0 at cruise, and is what thins the sky
    // out as the flight climbs. Daylight is `camera.sun_dir.w`.
    let altitude_scalar = camera.sun_params.x;
    let sun_dir = camera.sun_dir.xyz;
    let sun_elevation = camera.sun_dir.w;
    let cos_sun = dot(view_dir, sun_dir);
    
    let earth_radius = 6.378137;
    let atmosphere_thickness = 0.15; // 150km boundary for the ray marcher
    let atmosphere_radius = earth_radius + atmosphere_thickness;
    
    let t_atm = ray_sphere_intersect(origin, view_dir, atmosphere_radius);
    
    var dist_in_atm = 0.0;
    if (t_atm.y > 0.0) {
        let t_start = max(0.0, t_atm.x);
        let t_stop = t_atm.y; 
        dist_in_atm = max(0.0, t_stop - t_start);
    }
    
    let t_closest = -dot(origin, view_dir);
    let t_min = max(0.0, t_closest);
    let p_closest = origin + view_dir * t_min;
    let d = length(p_closest);
    
    let h_normalized = clamp((d - earth_radius) / atmosphere_thickness, 0.0, 1.0);
    let density_at_d = exp(-h_normalized * 10.0);
    let boundary_softener = smoothstep(1.0, 0.8, h_normalized);
    let optical_depth = density_at_d * dist_in_atm * boundary_softener * 2.0;
    
    let space_color = vec3<f32>(0.02, 0.02, 0.04);

    // Three daylight palettes, chosen by how high the sun is rather than by a clock.
    let day_horizon_color   = vec3<f32>(0.70, 0.80, 0.90);
    let day_zenith_color    = vec3<f32>(0.15, 0.35, 0.75);
    let dusk_horizon_color  = vec3<f32>(0.95, 0.45, 0.22);
    let dusk_zenith_color   = vec3<f32>(0.18, 0.20, 0.42);
    // Grey rather than blue: the cruise is the quiet part and must not read as a colour.
    let night_horizon_color = vec3<f32>(0.055, 0.057, 0.062);
    let night_zenith_color  = vec3<f32>(0.012, 0.012, 0.014);

    // Below the horizon the sky fades to night; above it, the warmth burns off as the sun
    // climbs. Twilight is the band in between and is where all the colour lives.
    // Must match celestial.rs: DAY_ELEVATION / DUSK / NIGHT.
    let day_amount   = smoothstep(0.0, 0.10, sun_elevation);
    let night_amount = smoothstep(-0.02, -0.22, sun_elevation);

    var horizon_color = mix(dusk_horizon_color, day_horizon_color, day_amount);
    var zenith_color  = mix(dusk_zenith_color, day_zenith_color, day_amount);
    horizon_color = mix(horizon_color, night_horizon_color, night_amount);
    zenith_color  = mix(zenith_color, night_zenith_color, night_amount);

    // The warm half of the sky is the half the sun is in. Without this a sunset is an
    // even orange band all the way round, which is the giveaway of a faked sky.
    let toward_sun = smoothstep(-0.2, 0.9, cos_sun);
    let twilight = day_amount * (1.0 - day_amount) * 4.0; // peaks mid-twilight
    horizon_color = mix(
        horizon_color,
        horizon_color * vec3<f32>(1.35, 0.95, 0.75),
        toward_sun * twilight
    );

    // Altitude is a second, independent axis: the flight climbing into cruise drains the
    // sky toward space regardless of the hour. Dropping this broke the "deep dive" the
    // whole view is built around — a cruise at noon must not look like a taxi at noon.
    horizon_color = mix(horizon_color * 0.25, horizon_color, altitude_scalar);
    zenith_color = mix(zenith_color * 0.12, zenith_color, altitude_scalar);

    var base_color = space_color;
    if (optical_depth > 0.0) {
        // The bright band tightens toward the horizon as the air thins, which is what
        // the sky actually does from the flight levels.
        // The band tightens with altitude, but only a little: taken too far the gradient
        // collapses into a visible edge, which looks like a seam rather than a horizon.
        let band_low  = mix(1.8, 1.5, altitude_scalar);
        let band_high = mix(2.7, 2.7, altitude_scalar);
        let color_mix = smoothstep(band_low, band_high, optical_depth);
        let atmosphere_color = mix(zenith_color, horizon_color, color_mix);

        // True optical absorption/scattering (Beer-Lambert law approximation)
        let opacity = 1.0 - exp(-optical_depth * 10.0); 
        
        base_color = mix(space_color, atmosphere_color, opacity);
    }

    // ── Stars ────────────────────────────────────────────────────────────────
    //
    // Attenuated by the air, but on a far gentler curve than the sky's own opacity: that
    // figure describes how much scattered light the atmosphere *adds*, which at three
    // kilometres is already 90% and would extinguish every star. What matters to a point
    // source is how much it absorbs, which is much less. The long slant path near the
    // horizon still puts them out, which is what you actually see.
    let star_extinction = exp(-optical_depth * 1.5);
    base_color += star_field(view_dir, clamp(day_amount, 0.0, 1.0)) * star_extinction;

    // ── The sun and the moon themselves ──────────────────────────────────────
    //
    // Both are drawn before the atmosphere is layered over them, so a low sun is
    // reddened and dimmed by the air it is seen through, as it should be.

    // The sun's disc is about half a degree across, so its cosine sits very close to 1.
    let sun_disc = smoothstep(0.99993, 0.99997, cos_sun);
    // A wide, faint forward-scatter halo. This is most of what sells a sun in a sky.
    let sun_glow = pow(max(cos_sun, 0.0), 350.0) * 0.6
                 + pow(max(cos_sun, 0.0), 12.0) * 0.05;
    let sun_tint = mix(vec3<f32>(1.0, 0.45, 0.2), vec3<f32>(1.0, 0.96, 0.9),
                       smoothstep(0.0, 0.25, sun_elevation));
    let celestial_fade = mix(0.55, 1.0, altitude_scalar);
    base_color += sun_tint * (sun_disc + sun_glow)
        * smoothstep(-0.08, 0.02, sun_elevation) * celestial_fade;

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

    return vec4<f32>(base_color, 1.0);
}
