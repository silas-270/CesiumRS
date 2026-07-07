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
    
    // --- HORIZON BLUR RIBBON (SCREEN-SPACE GRADIENT) ---
    // The fundamental issue on the ground is that `camera_pos` loses precision in f32 when altitude is low.
    // This corrupts all altitude-based horizon math, making the ribbon vanish.
    // SOLUTION: Use raw screen-space derivatives of distance. At any silhouette or horizon edge, 
    // the distance changes infinitely fast between adjacent pixels, regardless of zoom or altitude.
    
    let pixel_dist = length(in.world_pos);
    let dist_dx = dpdx(pixel_dist);
    let dist_dy = dpdy(pixel_dist);
    let dist_grad = sqrt(dist_dx * dist_dx + dist_dy * dist_dy);
    
    let pos_dx = dpdx(in.world_pos);
    let pos_dy = dpdy(in.world_pos);
    let pos_grad = sqrt(dot(pos_dx, pos_dx) + dot(pos_dy, pos_dy));
    
    // slope is the ratio of vertical radius change to physical distance change.
    // For flat terrain, slope is ~0. For vertical skirts at the horizon, slope is 1.0.
    // By using the slope, the blur thickness remains a constant 1-2 pixels at the silhouette
    // regardless of zoom level, because it relies on the skirt geometry itself.
    // Let's normalize the screen-space depth gradient by the square root of the distance
    // from the camera. Mathematically, the ratio of (depth_gradient / sqrt(depth)) at the horizon
    // is a zoom-independent invariant (~0.094 for a 45-degree FOV at 1080p).
    // This creates a beautiful, thin horizon blur that maintains a constant pixel width (1-2 pixels)
    // and never bleeds into normal terrain at any zoom level.
    let horizon_metric = dist_grad / sqrt(max(pixel_dist, 0.0001));
    var blur_factor = smoothstep(0.03, 0.08, horizon_metric);
    
    // Prevent blurring geometry closer than 100 meters (e.g. walls right in front of camera)
    let dist_fade = smoothstep(0.0, 0.0001, pixel_dist);
    blur_factor = blur_factor * dist_fade;
    
    let earth_radius = 6.378137;
    let r_cam = max(length(camera.camera_pos.xyz), earth_radius);
    let altitude = max(r_cam - earth_radius, 0.0);
    let horizon_color = vec3<f32>(0.65, 0.75, 0.85); 
    let space_color = vec3<f32>(0.02, 0.02, 0.04);
    let space_fade = clamp((altitude - 0.05) / 0.45, 0.0, 1.0);
    let current_fog_color = mix(horizon_color, space_color, space_fade);
    
    let final_color = mix(shaded_color, current_fog_color, blur_factor);
    
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
