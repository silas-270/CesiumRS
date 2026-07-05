struct CameraUniform {
    view_proj: mat4x4<f32>,
}

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
}

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) normal: vec3<f32>,
    @location(1) uv: vec2<f32>,
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

    // Dynamic screen-space scaling
    // 1. Where is the center of the model? (model_matrix translation)
    let center_world = model_matrix * vec4<f32>(0.0, 0.0, 0.0, 1.0);
    
    // 2. Distance to camera
    // `center_world` is relative to camera if we're using RTC, 
    // but we can also just use length(center_world.xyz) since the origin IS the camera.
    // However, if it's absolute, we do center_world - camera_pos.
    // Since we know the engine uses RTC, `center_world` length is the distance!
    let dist_to_cam = max(length(center_world.xyz), 0.000001);

    // 3. Airplane roughly fits in a 67-meter sphere. Let's say physical size is 67.0.
    // Wait, the model_matrix already contains a scale factor. 
    // If the matrix scales it to 67 meters in engine units:
    // Engine units = meters / 6378137.0.
    // Let's extract the scale from the matrix (length of X axis)
    let world_scale = length(model_matrix[0].xyz);

    // Let's say the base model is 67 units long in its own space (if it's in meters).
    // The physical size in engine units is 67.0 * world_scale.
    let physical_size_engine = 67.0 * world_scale;

    // 4. Calculate approximate pixel size
    let fov_factor = 1.5; // Approximation of projection scaling
    let pixels_per_engine_unit = (1.0 / dist_to_cam) * push.viewport_size.y * fov_factor;
    let size_pixels = physical_size_engine * pixels_per_engine_unit;

    // 5. Enforce minimum pixel size (e.g. 500 pixels for testing)
    let min_pixels = 500.0;
    var scale_multiplier = 1.0;
    if size_pixels > 0.00001 {
        scale_multiplier = max(1.0, min_pixels / size_pixels);
    }

    // Apply scale multiplier to local position
    let scaled_pos = model.position * scale_multiplier;

    let world_position = model_matrix * vec4<f32>(scaled_pos, 1.0);
    out.clip_position = camera.view_proj * world_position;

    // Transform normal to world space (ignoring non-uniform scaling for now)
    let normal_matrix = mat3x3<f32>(
        model_matrix[0].xyz,
        model_matrix[1].xyz,
        model_matrix[2].xyz
    );
    out.normal = normalize(normal_matrix * model.normal);
    out.uv = model.uv;
    
    return out;
}

@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
    let light_dir = normalize(vec3<f32>(0.5, 1.0, 0.3));
    let normal = normalize(in.normal);
    
    let diffuse = max(dot(normal, light_dir), 0.0);
    let ambient = 0.5;
    let light_intensity = diffuse * 0.7 + ambient;
    
    // Sample texture and add a prominent red tint so it can't be missed
    let tex_color = textureSample(t_diffuse, s_diffuse, in.uv).rgb;
    let color = (tex_color + vec3<f32>(0.5, 0.0, 0.0)) * light_intensity;
    
    return vec4<f32>(color, 1.0);
}
