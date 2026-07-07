struct CameraUniform {
    view_proj: mat4x4<f32>,
    inv_view_proj: mat4x4<f32>,
    camera_pos: vec4<f32>,
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
    // model.uv * scale + offset
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
    let light_dir = normalize(vec3<f32>(1.0, 1.0, 1.0));
    let ambient = 0.8; // Much brighter base
    let diffuse = max(dot(in.normal, light_dir), 0.0) * 0.4; // Softer shadows
    
    let tex_color = textureSample(t_diffuse, s_diffuse, in.uv);
    let shaded_color = tex_color.rgb * (ambient + diffuse);
    
    // --- HORIZON FOG ---
    // in.world_pos is already relative to the camera due to RTE rendering!
    let pixel_dist = length(in.world_pos);
    let earth_radius = 6.378137;
    let r = max(length(camera.camera_pos.xyz), earth_radius);
    
    // Minimum 1km horizon distance to prevent divide by zero
    let horizon_dist = sqrt(max(r * r - earth_radius * earth_radius, 0.000001)); 
    let fog_ratio = pixel_dist / horizon_dist;
    
    let altitude = max(r - earth_radius, 0.0);
    // Smoothly push fog_start closer to the horizon as you climb to space
    let fog_start = mix(0.5, 0.95, clamp(altitude / 0.02, 0.0, 1.0));
    
    // Smoothstep creates the beautiful blur effect
    let fog_factor = smoothstep(fog_start, 1.0, fog_ratio);
    let horizon_color = vec3<f32>(0.65, 0.75, 0.85); // Must perfectly match sky.wgsl
    
    let final_color = mix(shaded_color, horizon_color, fog_factor);
    
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
