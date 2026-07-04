struct CameraUniform {
    view_proj: mat4x4<f32>,
};

@group(0) @binding(0)
var<uniform> camera: CameraUniform;

struct VertexInput {
    @location(0) position: vec3<f32>,
    @location(1) previous: vec3<f32>,
    @location(2) next: vec3<f32>,
    @location(3) side: f32, // 1.0 or -1.0
};

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) color: vec4<f32>,
};

struct PushConstants {
    camera_pos: vec4<f32>,
    viewport_size: vec2<f32>,
    thickness: f32,
    padding: f32,
};
var<push_constant> push_constants: PushConstants;

@vertex
fn vs_main(model: VertexInput) -> VertexOutput {
    var out: VertexOutput;

    let rel_curr = model.position - push_constants.camera_pos.xyz;
    let rel_prev = model.previous - push_constants.camera_pos.xyz;
    let rel_next = model.next - push_constants.camera_pos.xyz;
    
    let clip_curr = camera.view_proj * vec4<f32>(rel_curr, 1.0);
    let clip_prev = camera.view_proj * vec4<f32>(rel_prev, 1.0);
    let clip_next = camera.view_proj * vec4<f32>(rel_next, 1.0);

    let ndc_curr = clip_curr.xy / clip_curr.w;
    let ndc_prev = clip_prev.xy / clip_prev.w;
    let ndc_next = clip_next.xy / clip_next.w;

    let raw_dir1 = ndc_curr - ndc_prev;
    let raw_dir2 = ndc_next - ndc_curr;
    
    var dir1 = vec2<f32>(1.0, 0.0);
    if length(raw_dir1) > 0.00001 {
        dir1 = normalize(raw_dir1);
    }
    
    var dir2 = vec2<f32>(1.0, 0.0);
    if length(raw_dir2) > 0.00001 {
        dir2 = normalize(raw_dir2);
    }

    // Average direction (tangent)
    var tangent = normalize(dir1 + dir2);
    if length(tangent) < 0.1 {
        tangent = vec2<f32>(1.0, 0.0);
    }

    let normal = vec2<f32>(-tangent.y, tangent.x);

    // Extrude in screen space
    let extrusion_pixels = normal * model.side * (push_constants.thickness / 2.0);
    
    // Convert pixel extrusion back to NDC (-1 to 1)
    let extrusion_ndc = extrusion_pixels / push_constants.viewport_size * 2.0;

    let final_clip_xy = clip_curr.xy + extrusion_ndc * clip_curr.w;

    out.clip_position = vec4<f32>(final_clip_xy, clip_curr.z, clip_curr.w);
    
    // Simple glowing orange color
    out.color = vec4<f32>(1.0, 0.5, 0.0, 1.0);

    return out;
}

@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
    return in.color;
}
