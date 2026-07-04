struct CameraUniform {
    view_proj: mat4x4<f32>,
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
    return out;
}

@group(1) @binding(0)
var t_diffuse: texture_2d<f32>;
@group(1) @binding(1)
var s_diffuse: sampler;

@fragment
fn fs_solid(in: VertexOutput) -> @location(0) vec4<f32> {
    let light_dir = normalize(vec3<f32>(1.0, 1.0, 1.0));
    let ambient = 0.4;
    let diffuse = max(dot(in.normal, light_dir), 0.0) * 0.6;
    
    let tex_color = textureSample(t_diffuse, s_diffuse, in.uv);
    let final_color = tex_color.rgb * (ambient + diffuse);
    
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
