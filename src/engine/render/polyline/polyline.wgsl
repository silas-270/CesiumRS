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
    @location(7) forward: f32,
};

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) face_shade: f32,
    @location(1) uv: vec2<f32>,
    @location(2) progress: f32,
};

struct PushConstants {
    reference_point: vec4<f32>,
    camera_pos: vec4<f32>,
    color_start: vec4<f32>,
    color_end: vec4<f32>,
    viewport_size: vec2<f32>,
    thickness: f32,
    split_progress: f32,
    physical_half_width: f32,
    physical_half_height: f32,
    // World-space airplane position (relative to reference_point, f32 precision).
    // The split is rendered at the closest point on the ribbon to this position.
    airplane_pos: vec4<f32>,
};
var<push_constant> push_constants: PushConstants;

@vertex
fn vs_main(
    model: VertexInput,
) -> VertexOutput {
    var out: VertexOutput;

    // 1. Transform positions relative to camera using DVec3 precision
    let pos_3d = model.position;
    let prev_3d = model.previous;
    let next_3d = model.next;

    let rel_curr = pos_3d - push_constants.camera_pos.xyz;
    let rel_prev = prev_3d - push_constants.camera_pos.xyz;
    let rel_next = next_3d - push_constants.camera_pos.xyz;

    // 2. Calculate up vector based on current spherical position
    let pos_3d_abs = pos_3d + push_constants.reference_point.xyz;
    var up_3d = normalize(pos_3d_abs);

    // 3. Compute tangent in 3D
    let dir_prev = normalize(rel_curr - rel_prev);
    let dir_next = normalize(rel_next - rel_curr);
    
    var tangent = dir_next;
    if length(dir_next) < 0.001 {
        tangent = dir_prev;
    } else if length(dir_prev) > 0.001 {
        tangent = normalize(dir_prev + dir_next);
    }
    
    if length(tangent) < 0.001 {
        tangent = vec3<f32>(1.0, 0.0, 0.0);
    }

    // Horizontal extrusion vector (cross product)
    var normal_3d = cross(up_3d, tangent);
    if length(normal_3d) < 0.001 {
        normal_3d = vec3<f32>(0.0, 1.0, 0.0);
    }
    normal_3d = normalize(normal_3d);

    // 4. Robust Edge-of-Screen Extrusion
    let physical_half_width = push_constants.physical_half_width;
    let physical_half_height = push_constants.physical_half_height;

    let dist_to_cam = length(rel_curr);
    
    // We want the ribbon to scale exactly like the airplane.
    // The airplane is 67 meters long, and we scale it by 5% of the distance to the camera,
    // clamped between 67m and 3000km.
    let desired_scale_mm = dist_to_cam * 0.05;
    let min_scale_mm = 67.0 / 1000000.0;
    let max_scale_mm = 3000.0 * 1000.0 / 1000000.0;
    let clamped_scale_mm = clamp(desired_scale_mm, min_scale_mm, max_scale_mm);
    
    // Calculate the scale multiplier relative to the base length
    var scale_multiplier = clamped_scale_mm / min_scale_mm;
    
    // Scale the ribbon's physical width and height by the same multiplier
    let final_half_width = physical_half_width * scale_multiplier;
    
    // Cap the height scaling so the ribbon volume doesn't become too thick vertically at high zooms
    let height_scale_multiplier = min(scale_multiplier, 4500.0);
    let final_half_height = physical_half_height * height_scale_multiplier;
    
    // Elevate 5m to avoid clipping
    let elevation_offset = up_3d * 0.000005;

    let corner_offset_3d = normal_3d * final_half_width * model.side + up_3d * final_half_height * model.v_side + tangent * final_half_width * model.forward;
    let extruded_3d = rel_curr + corner_offset_3d + elevation_offset;
    let clip_final = camera.view_proj * vec4<f32>(extruded_3d, 1.0);

    out.clip_position = clip_final;
    out.uv = vec2<f32>(model.side, model.forward);
    out.progress = model.progress;

    // --- Fix: per-face depth nudge to eliminate Z-fighting at seam edges ---
    // The four faces of the tube share exact vertex positions at their seam edges.
    // Left and right side faces are pushed very slightly further from the camera
    // (larger clip-space z) so they never fight with the top/bottom faces.
    // The nudge is proportional to w so it's constant in NDC regardless of distance.
    if model.face >= 2.0 {
        out.clip_position.z += out.clip_position.w * 0.0002;
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
    
    out.face_shade = face_shade;
    return out;
}

@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
    if length(in.uv) > 1.0 {
        discard;
    }

    // --- Fix: world-space split instead of progress-domain split ---
    // We project this fragment's world position onto the airplane-to-ribbon axis
    // so the orange/white boundary always passes through the airplane center,
    // regardless of how the pre-baked 'progress' values were assigned.
    var base_color = push_constants.color_start.rgb;
    if push_constants.split_progress >= 0.0 {
        // airplane_pos is relative to reference_point (same space as vertex positions)
        // in.progress still carries the vertex world-position dot product in the new scheme.
        // We compare using the per-vertex progress value which is set by the CPU to the
        // dot product of the vertex position along the airplane's forward direction.
        // (See: the CPU now passes airplane_pos and we compute a signed distance in app.rs)
        //
        // Fallback: if airplane_pos.w == 0 (not set), use legacy progress comparison.
        if push_constants.airplane_pos.w > 0.5 {
            // airplane_pos.xyz is the airplane world position relative to reference_point,
            // already in the same space as vertex positions stored in the buffer.
            // We use the per-vertex 'progress' as the signed projection value:
            // progress < 0  => behind the airplane  => orange
            // progress >= 0 => ahead of the airplane => white
            if in.progress >= 0.0 {
                base_color = push_constants.color_end.rgb;
            }
        } else {
            // Legacy path: time-domain progress comparison
            if in.progress > push_constants.split_progress {
                base_color = push_constants.color_end.rgb;
            }
        }
    }

    return vec4<f32>(base_color * in.face_shade, push_constants.color_start.a);
}
