struct CameraUniform {
    view_proj: mat4x4<f32>,
    inv_view_proj: mat4x4<f32>,
    camera_pos: vec4<f32>,
    sun_params: vec4<f32>,
};

@group(0) @binding(0)
var<uniform> camera: CameraUniform;

@group(1) @binding(0) var t_diffuse: texture_2d<f32>;
@group(1) @binding(1) var s_diffuse: sampler;

struct ModelPushConstants {
    model_matrix_0: vec4<f32>,
    model_matrix_1: vec4<f32>,
    model_matrix_2: vec4<f32>,
    model_matrix_3: vec4<f32>,
    camera_pos: vec4<f32>,
    viewport_size: vec2<f32>,
    padding: vec2<f32>,
}

var<push_constant> push: ModelPushConstants;

struct VertexInput {
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) uv: vec2<f32>,
    @location(3) color: vec4<f32>,
}

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) normal: vec3<f32>,
    @location(1) uv: vec2<f32>,
    @location(2) color: vec4<f32>,
}

@vertex
fn vs_main(model: VertexInput) -> VertexOutput {
    var out: VertexOutput;
    
    let model_matrix = mat4x4<f32>(
        push.model_matrix_0,
        push.model_matrix_1,
        push.model_matrix_2,
        push.model_matrix_3,
    );

    let center_world = model_matrix * vec4<f32>(0.0, 0.0, 0.0, 1.0);
    let dist_to_cam = max(length(center_world.xyz), 0.000001);

    // `world_scale` is the size of the mesh in engine units, since the mesh is normalized to radius 1.0
    let world_scale = length(model_matrix[0].xyz);
    let physical_size_engine = 2.0 * world_scale; // diameter

    let fov_factor = 1.5;
    let pixels_per_engine_unit = (1.0 / dist_to_cam) * push.viewport_size.y * fov_factor;
    
    let size_pixels = max(physical_size_engine * pixels_per_engine_unit, 0.00001);
    let target_pixels = 100.0;
    
    let needed_scale = target_pixels / size_pixels;
    
    // We want the plane to never be smaller than 1.0 (its true size)
    // and never larger than some huge factor (e.g., to prevent it from covering the globe)
    let max_scale = max(1.0, 4000000.0 / (6378137.0 * max(physical_size_engine, 0.000001)));
    
    let scale_multiplier = clamp(needed_scale, 1.0, max_scale);

    // Apply scaling
    let scaled_pos = model.position * scale_multiplier;
    
    // Apply model matrix (which already has translation and rotation)
    let final_world_pos = model_matrix * vec4<f32>(scaled_pos, 1.0);

    out.clip_position = camera.view_proj * vec4<f32>(final_world_pos.xyz, 1.0);
    // Apply a slight depth bias to prevent the airplane from clipping into the earth's surface
    out.clip_position.z = out.clip_position.z + 0.005 * out.clip_position.w;

    // Transform normal to world space (ignoring non-uniform scaling for now)
    let normal_matrix = mat3x3<f32>(
        model_matrix[0].xyz,
        model_matrix[1].xyz,
        model_matrix[2].xyz
    );
    out.normal = normalize(normal_matrix * model.normal);
    out.uv = model.uv;
    out.color = model.color;
    
    return out;
}

@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
    let light_dir = normalize(vec3<f32>(0.5, 1.0, 0.3));
    let normal = normalize(in.normal);
    
    let diffuse = max(dot(normal, light_dir), 0.0);
    let ambient = 0.5;
    let light_intensity = diffuse * 0.7 + ambient;
    
    // Sample texture
    let tex_color = textureSample(t_diffuse, s_diffuse, in.uv).rgb;
    let color = tex_color * in.color.rgb * light_intensity;
    
    return vec4<f32>(color, in.color.a);
}
