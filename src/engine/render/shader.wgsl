struct CameraUniform {
    view_proj: mat4x4<f32>,
    inv_view_proj: mat4x4<f32>,
    camera_pos: vec4<f32>,
    sun_params: vec4<f32>,
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
    let sun_intensity = camera.sun_params.x;
    let sun_pos_rad = radians(camera.sun_params.y);
    let light_dir = normalize(vec3<f32>(cos(sun_pos_rad), 0.0, sin(sun_pos_rad)));
    
    // When sun_intensity is 0.0 (Night Mode with Dark Map): Ambient is 1.0 (show the dark map naturally), Diffuse is 0.0.
    // When sun_intensity is 1.0 (Day Mode with Light Map): Ambient is 0.3 (dark side of the earth is shaded), Diffuse is 0.9 (sun side is 1.2x bright).
    let ambient = mix(1.0, 0.3, sun_intensity); 
    let diffuse = max(dot(in.normal, light_dir), 0.0) * mix(0.0, 0.9, sun_intensity);
    
    let tex_color = textureSample(t_diffuse, s_diffuse, in.uv);
    let shaded_color = tex_color.rgb * (ambient + diffuse);
    
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
    
    let day_horizon_color = vec3<f32>(0.65, 0.75, 0.85); 
    let night_horizon_color = vec3<f32>(0.02, 0.02, 0.03);
    let horizon_color = mix(night_horizon_color, day_horizon_color, sun_intensity);
    
    let space_color = vec3<f32>(0.02, 0.02, 0.04);
    let space_fade = clamp((altitude - 0.05) / 0.45, 0.0, 1.0);
    let current_fog_color = mix(horizon_color, space_color, space_fade);
    
    let final_color = mix(shaded_color, current_fog_color, blur_factor);
    
    return vec4<f32>(final_color, tex_color.a);
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
