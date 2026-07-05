struct CameraUniform {
    view_proj: mat4x4<f32>,
};

@group(0) @binding(0)
var<uniform> camera: CameraUniform;

struct VertexInput {
    @location(0) position: vec3<f32>,
    @location(1) previous: vec3<f32>,
    @location(2) next: vec3<f32>,
    @location(3) side: f32,
    @location(4) v_side: f32,
    @location(5) face: f32,
    @location(6) progress: f32,
};

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) color: vec4<f32>,
};

struct PushConstants {
    camera_pos_high: vec3<f32>,
    camera_pos_low: vec3<f32>,
    viewport_size: vec2<f32>,
    thickness: f32,
    split_progress: f32,
};
var<push_constant> push_constants: PushConstants;

@vertex
fn vs_main(model: VertexInput) -> VertexOutput {
    var out: VertexOutput;

    // Relative to camera position
    let cam_pos = vec3<f32>(push_constants.camera_pos_high) + push_constants.camera_pos_low;
    
    let rel_curr = model.position - cam_pos;
    let rel_prev = model.previous - cam_pos;
    let rel_next = model.next - cam_pos;

    // Extrude direction (horizontal)
    let dir_prev = normalize(rel_curr - rel_prev);
    let dir_next = normalize(rel_next - rel_curr);
    
    var tangent = dir_prev + dir_next;
    let tangent_len = length(tangent);
    if tangent_len > 0.001 {
        tangent = tangent / tangent_len;
    } else {
        tangent = dir_next;
        if length(tangent) < 0.001 {
            tangent = vec3<f32>(1.0, 0.0, 0.0);
        }
    }

    // Up vector is the normal to the globe surface
    let up_3d = normalize(model.position);
    
    // Horizontal extrusion vector (cross product)
    var normal_3d = cross(up_3d, tangent);
    if length(normal_3d) < 0.001 {
        normal_3d = vec3<f32>(0.0, 1.0, 0.0);
    }
    normal_3d = normalize(normal_3d);

    // 4. Robust Edge-of-Screen Extrusion (fixes distortion when w changes rapidly)
    let physical_half_width = 0.0001; // 100 meters in Megameters
    let physical_half_height = 0.000001; // 1 meter (2m total height)

    // Calculate how big it is on screen
    let clip_center = camera.view_proj * vec4<f32>(rel_curr, 1.0);
    let clip_right = camera.view_proj * vec4<f32>(rel_curr + normal_3d * physical_half_width, 1.0);
    
    let ndc_center = clip_center.xy / max(clip_center.w, 0.000001);
    let ndc_right = clip_right.xy / max(clip_right.w, 0.000001);
    
    let width_pixels = length((ndc_right - ndc_center) * push_constants.viewport_size * 0.5);
    
    let min_pixels = push_constants.thickness / 2.0;
    var scale_multiplier = 1.0;
    if width_pixels > 0.00001 {
        scale_multiplier = max(1.0, min_pixels / width_pixels);
    }
    
    let final_half_width = physical_half_width * scale_multiplier;
    let final_half_height = physical_half_height * scale_multiplier;
    
    let corner_offset_3d = normal_3d * final_half_width * model.side + up_3d * final_half_height * model.v_side;
    let extruded_3d = rel_curr + corner_offset_3d;
    let clip_final = camera.view_proj * vec4<f32>(extruded_3d, 1.0);

    out.clip_position = clip_final;
    
    // Calculate color based on split_progress and face lighting
    var base_color = vec3<f32>(1.0, 0.4, 0.0); // Orange
    if push_constants.split_progress >= 0.0 && model.progress > push_constants.split_progress {
        base_color = vec3<f32>(0.9, 0.9, 0.9); // White
    }
    
    // Add shading depending on face
    var face_shade = 1.0;
    if model.face == 0.0 { // Top
        face_shade = 1.0;
    } else if model.face == 1.0 { // Bottom
        face_shade = 0.4;
    } else { // Sides (Left 2.0, Right 3.0)
        face_shade = 0.7;
    }
    
    base_color = base_color * face_shade;

    out.color = vec4<f32>(base_color, 1.0);
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
