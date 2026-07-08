// ── Uniforms & push constants ─────────────────────────────────────────────────

struct CameraUniform {
    view_proj:     mat4x4<f32>,
    inv_view_proj: mat4x4<f32>,
    camera_pos:    vec4<f32>,
    sun_params:    vec4<f32>,
};

@group(0) @binding(0)
var<uniform> camera: CameraUniform;

struct PushConstants {
    reference_point:    vec4<f32>,  // offset   0
    camera_pos:         vec4<f32>,  // offset  16
    color_start:        vec4<f32>,  // offset  32
    color_end:          vec4<f32>,  // offset  48
    viewport_size:      vec2<f32>,  // offset  64
    thickness:          f32,        // offset  72
    split_progress:     f32,        // offset  76
    physical_half_width:  f32,      // offset  80
    physical_half_height: f32,      // offset  84
    _padding:           vec2<f32>,  // offset  88
    airplane_pos:       vec4<f32>,  // offset  96
    airplane_forward:   vec4<f32>,  // offset 112
    // Total: 128 bytes
};
var<push_constant> pc: PushConstants;

// ── GPU-resident control points ───────────────────────────────────────────────

struct ControlPoint {
    position: vec3<f32>, // relative to reference_point
    progress:  f32,
};

@group(1) @binding(0)
var<storage, read> control_points: array<ControlPoint>;

// ── Vertex shader output ──────────────────────────────────────────────────────

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) face_shade: f32,
    @location(1) uv:         vec2<f32>,
    @location(2) progress:   f32,
    @location(3) world_pos:  vec3<f32>,
    @location(4) tangent:    vec3<f32>,
};

// ── Vertex shader ─────────────────────────────────────────────────────────────
//
// Layout (2 verts per control point — 2-D ribbon):
//
//   vertex_index  →  cp_idx = vid / 2,  corner = vid % 2
//   corner 0 = left  (side = -1)
//   corner 1 = right (side = +1)
//
// This produces a triangle-strip ribbon where every pair of verts straddles
// one control point.  Degenerate segments (zero-length) are used between
// disconnected strips and produce zero-area triangles discarded by the GPU.

@vertex
fn vs_main(@builtin(vertex_index) vid: u32) -> VertexOutput {
    var out: VertexOutput;

    let total = arrayLength(&control_points);
    let cp_idx = vid / 2u;
    let corner = vid % 2u;   // 0 = left, 1 = right

    // Guard against out-of-range (should not happen in normal use)
    if cp_idx >= total {
        out.clip_position = vec4<f32>(0.0, 0.0, 0.0, 1.0);
        return out;
    }

    let cp      = control_points[cp_idx];
    let cp_prev = control_points[select(cp_idx - 1u, 0u, cp_idx == 0u)];
    let cp_next = control_points[select(cp_idx + 1u, total - 1u, cp_idx + 1u >= total)];

    let pos  = cp.position;
    let prev = cp_prev.position;
    let next = cp_next.position;

    // Camera-relative positions (subtract camera so we work near the origin)
    let cam      = pc.camera_pos.xyz;
    let rel_curr = pos  - cam;
    let rel_prev = prev - cam;
    let rel_next = next - cam;

    // Up vector: outward normal from the sphere at this point
    let pos_abs = pos + pc.reference_point.xyz;
    let up_3d   = normalize(pos_abs);

    // Tangent: average of incoming and outgoing directions
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

    // Horizontal extrusion vector
    var normal_3d = cross(up_3d, tangent);
    if length(normal_3d) < 0.001 {
        normal_3d = vec3<f32>(0.0, 1.0, 0.0);
    }
    normal_3d = normalize(normal_3d);

    // Distance-based physical scale (matches airplane model scale)
    let dist_to_cam     = length(rel_curr);
    let desired_scale   = dist_to_cam * 0.05;
    let min_scale       = 67.0 / 1000000.0;   // 67 m in Mm
    let max_scale       = 3000.0 * 1000.0 / 1000000.0;
    let clamped_scale   = clamp(desired_scale, min_scale, max_scale);
    let scale_mult      = clamped_scale / min_scale;

    let final_half_width  = pc.physical_half_width  * scale_mult;
    let height_scale_mult = min(scale_mult, 4500.0);
    let final_half_height = pc.physical_half_height * height_scale_mult;

    // Slight elevation to avoid z-fighting with the globe surface
    let elevation = up_3d * 0.000005;

    // Side sign: corner 0 → -1 (left), corner 1 → +1 (right)
    let side = select(-1.0, 1.0, corner == 1u);
    let corner_offset = normal_3d * final_half_width * side + elevation;
    let extruded = rel_curr + corner_offset;

    out.clip_position = camera.view_proj * vec4<f32>(extruded, 1.0);
    out.uv         = vec2<f32>(side, 0.0);
    out.progress   = cp.progress;
    out.world_pos  = pos + corner_offset;
    out.tangent    = tangent;
    out.face_shade = 1.0; // ribbon is always top-face

    return out;
}

// ── Fragment shader ───────────────────────────────────────────────────────────

@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
    // Soft round-cap discard at the ribbon tips (|uv.x| > 1 is outside the ribbon)
    if abs(in.uv.x) > 1.0 {
        discard;
    }

    // Color split: orange (behind plane) / white-ish (ahead of plane)
    var base_color = pc.color_start.rgb;
    if pc.split_progress >= 0.0 {
        if pc.airplane_pos.w > 0.5 {
            // World-space proximity split
            let to_frag = in.world_pos - pc.airplane_pos.xyz;
            let dist    = length(to_frag);
            var is_ahead = false;
            if dist > 0.001 {
                is_ahead = in.progress >= pc.split_progress;
            } else {
                let tgt = normalize(in.tangent);
                is_ahead = dot(to_frag, tgt) >= 0.0;
            }
            if is_ahead {
                base_color = pc.color_end.rgb;
            }
        } else {
            // Legacy progress-domain split
            if in.progress > pc.split_progress {
                base_color = pc.color_end.rgb;
            }
        }
    }

    return vec4<f32>(base_color * in.face_shade, pc.color_start.a);
}
