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
    
    // --- RENEWED EXPONENTIAL HEIGHT FOG ---
    let pixel_dist = length(in.world_pos);
    let earth_radius = 6.378137;
    let r_cam = length(camera.camera_pos.xyz);
    
    // 1. Calculate camera and fragment altitudes above the ellipsoid
    let h_cam = max(r_cam - earth_radius, 0.0);
    
    // For fragment position, world_pos = in.world_pos + camera_pos
    // (since in.world_pos is relative to camera)
    let frag_world_pos = in.world_pos + camera.camera_pos.xyz;
    let h_frag = max(length(frag_world_pos) - earth_radius, 0.0);
    
    // 2. Average height along the view ray
    let h_avg = (h_cam + h_frag) * 0.5;
    
    // 3. Physical constants (in Megameter units)
    let scale_height = 0.008; // 8 km atmosphere scale height
    let base_density = 35.0;  // Adjusts base visibility at sea level
    
    // 4. Calculate optical depth along the ray
    let density = base_density * exp(-h_avg / scale_height);
    let optical_depth = pixel_dist * density;
    
    // 5. Compute exponential fog factor
    var fog_factor = 1.0 - exp(-optical_depth);
    
    // 6. Smoothly fade fog out completely when looking from deep space
    // Start fading at 100km, fully gone at 250km
    let view_fade = clamp((0.25 - h_cam) / 0.15, 0.0, 1.0);
    fog_factor = fog_factor * view_fade;
    
    // 7. Match color and space-fade transition of sky.wgsl
    let horizon_color = vec3<f32>(0.65, 0.75, 0.85); 
    let space_color = vec3<f32>(0.02, 0.02, 0.04);
    let space_fade = clamp((h_cam - 0.05) / 0.45, 0.0, 1.0);
    let current_fog_color = mix(horizon_color, space_color, space_fade);
    
    let final_color = mix(shaded_color, current_fog_color, fog_factor);
    
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
