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
    @location(4) progress: f32,
};

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) color: vec4<f32>,
    @location(1) progress: f32,
};

struct PushConstants {
    camera_pos: vec4<f32>,
    viewport_size: vec2<f32>,
    thickness: f32,
    split_progress: f32,
};
var<push_constant> push_constants: PushConstants;

@vertex
fn vs_main(model: VertexInput) -> VertexOutput {
    var out: VertexOutput;

    let rel_curr = model.position - push_constants.camera_pos.xyz;
    let clip_curr = camera.view_proj * vec4<f32>(rel_curr, 1.0);

    // 1. Calculate 3D tangent
    var tangent_3d = model.next - model.previous;
    if length(tangent_3d) < 0.000001 {
        tangent_3d = vec3<f32>(1.0, 0.0, 0.0);
    }
    tangent_3d = normalize(tangent_3d);

    // 2. Earth surface normal (straight up) at current position
    var up_3d = model.position;
    if length(up_3d) < 0.000001 {
        up_3d = vec3<f32>(0.0, 0.0, 1.0);
    }
    up_3d = normalize(up_3d);

    // 3. Horizontal normal perpendicular to tangent and up vector
    var normal_3d = cross(up_3d, tangent_3d);
    if length(normal_3d) < 0.00001 {
        normal_3d = cross(tangent_3d, vec3<f32>(0.0, 0.0, 1.0));
    }
    normal_3d = normalize(normal_3d);

    // 4. Robust Edge-of-Screen Extrusion (fixes distortion when w changes rapidly)
    let physical_half_width = 0.0001; // 100 meters in Megameters (so 200m total width)
    let extruded_3d = rel_curr + normal_3d * physical_half_width;
    let clip_extruded = camera.view_proj * vec4<f32>(extruded_3d, 1.0);
    
    let ndc_curr = clip_curr.xy / max(clip_curr.w, 0.000001);
    let ndc_extruded = clip_extruded.xy / max(clip_extruded.w, 0.000001);
    let offset_pixels = (ndc_extruded - ndc_curr) * push_constants.viewport_size * 0.5;
    
    let physical_pixels = length(offset_pixels);
    let min_pixels = push_constants.thickness / 2.0;
    let final_pixels = max(physical_pixels, min_pixels);

    // 5. Extrude in screen space along the projected normal direction
    var normal_ndc = vec2<f32>(1.0, 0.0);
    if physical_pixels > 0.00001 {
        normal_ndc = offset_pixels / physical_pixels;
    } else {
        let normal_clip = camera.view_proj * vec4<f32>(normal_3d, 0.0);
        if length(normal_clip.xy) > 0.00001 {
            normal_ndc = normalize(normal_clip.xy);
        }
    }

    let extrusion_pixels = normal_ndc * model.side * final_pixels;
    let extrusion_ndc = extrusion_pixels / push_constants.viewport_size * 2.0;

    let final_clip_xy = clip_curr.xy + extrusion_ndc * clip_curr.w;

    out.clip_position = vec4<f32>(final_clip_xy, clip_curr.z, clip_curr.w);
    
    // Pass progress forward
    out.progress = model.progress;

    return out;
}

@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
    // If split_progress < 0, entirely orange. Else split based on progress.
    if push_constants.split_progress >= 0.0 {
        if in.progress < push_constants.split_progress {
            return vec4<f32>(1.0, 0.5, 0.0, 1.0); // Orange
        } else {
            return vec4<f32>(1.0, 1.0, 1.0, 1.0); // White
        }
    }
    
    return vec4<f32>(1.0, 0.5, 0.0, 1.0); // Orange (Default)
}
