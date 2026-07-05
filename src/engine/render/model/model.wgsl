struct CameraUniform {
    view_proj: mat4x4<f32>,
    view: mat4x4<f32>,
    proj: mat4x4<f32>,
    position: vec4<f32>,
}

@group(0) @binding(0)
var<uniform> camera: CameraUniform;

struct ModelPushConstants {
    model_matrix_0: vec4<f32>,
    model_matrix_1: vec4<f32>,
    model_matrix_2: vec4<f32>,
    model_matrix_3: vec4<f32>,
}

var<push_constant> push: ModelPushConstants;

struct VertexInput {
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
}

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) normal: vec3<f32>,
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

    let world_position = model_matrix * vec4<f32>(model.position, 1.0);
    out.clip_position = camera.view_proj * world_position;

    // Transform normal to world space (ignoring non-uniform scaling for now)
    let normal_matrix = mat3x3<f32>(
        model_matrix[0].xyz,
        model_matrix[1].xyz,
        model_matrix[2].xyz
    );
    out.normal = normalize(normal_matrix * model.normal);
    
    return out;
}

@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
    let light_dir = normalize(vec3<f32>(0.5, 1.0, 0.3));
    let normal = normalize(in.normal);
    
    // Very simple Lambertian lighting
    let diffuse = max(dot(normal, light_dir), 0.0);
    let ambient = 0.3;
    let light_intensity = diffuse * 0.7 + ambient;
    
    // Base color of the airplane (white)
    let base_color = vec3<f32>(0.9, 0.9, 0.95);
    let color = base_color * light_intensity;
    
    return vec4<f32>(color, 1.0);
}
