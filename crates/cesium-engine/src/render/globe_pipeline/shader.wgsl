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
    @location(2) color: vec4<f32>,
    @location(3) uv: vec2<f32>,
};

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) normal: vec3<f32>,
    @location(1) color: vec4<f32>,
    @location(2) uv: vec2<f32>,
    @location(3) world_pos: vec3<f32>,
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
    out.color = model.color;
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
    
    let pixel_dist = length(in.world_pos);
    let dist_dx = dpdx(pixel_dist);
    let dist_dy = dpdy(pixel_dist);
    let dist_grad = sqrt(dist_dx * dist_dx + dist_dy * dist_dy);
    
    let horizon_metric = dist_grad / sqrt(max(pixel_dist, 0.0001));
    
    let HORIZON_BLUR_WIDTH = 2.0; 
    let lower_thresh = 0.015 / HORIZON_BLUR_WIDTH;
    let upper_thresh = 0.06 / HORIZON_BLUR_WIDTH;
    
    var blur_factor = smoothstep(lower_thresh, upper_thresh, horizon_metric);
    
    let dist_fade = smoothstep(0.0, 0.0001, pixel_dist);
    blur_factor = blur_factor * dist_fade;
    
    let earth_radius = 6.378137;
    let r_cam = max(length(camera.camera_pos.xyz), earth_radius);
    let altitude = max(r_cam - earth_radius, 0.0);
    
    // The haze at the limb has to agree with the sky drawn behind it, so it follows the
    // sun's elevation on the same ramp rather than the altitude scalar.
    let sun_elevation = camera.sun_dir.w;
    // Must match celestial.rs: DAY_ELEVATION / DUSK / NIGHT.
    let day_amount = smoothstep(0.0, 0.10, sun_elevation);
    let night_amount = smoothstep(-0.02, -0.22, sun_elevation);
    let day_horizon_color = vec3<f32>(0.65, 0.75, 0.85);
    let dusk_horizon_color = vec3<f32>(0.92, 0.44, 0.22);
    let night_horizon_color = vec3<f32>(0.055, 0.057, 0.062);
    var horizon_color = mix(dusk_horizon_color, day_horizon_color, day_amount);
    horizon_color = mix(horizon_color, night_horizon_color, night_amount);
    // MUST match sky.wgsl exactly. The terrain's limb haze and the sky behind it meet at
    // the horizon, so any difference between them shows up as a hard line across the
    // whole view. They used to agree for free by both keying off the altitude scalar;
    // once the sky moved to a time-of-day ramp, this had to be dimmed the same way.
    horizon_color = mix(horizon_color * 0.25, horizon_color, altitude_scalar);
    
    let space_color = vec3<f32>(0.02, 0.02, 0.04);
    let space_fade = clamp((altitude - 0.05) / 0.45, 0.0, 1.0);
    let current_fog_color = mix(horizon_color, space_color, space_fade);
    
    let final_color = mix(shaded_color, current_fog_color, blur_factor);
    
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
