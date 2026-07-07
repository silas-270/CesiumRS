struct CameraUniform {
    view_proj: mat4x4<f32>,
    inv_view_proj: mat4x4<f32>,
    camera_pos: vec4<f32>,
};

@group(0) @binding(0)
var<uniform> camera: CameraUniform;

struct SkyOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) view_dir: vec3<f32>,
};

@vertex
fn vs_sky(@builtin(vertex_index) vertex_index: u32) -> SkyOutput {
    var out: SkyOutput;
    
    // Generate full-screen triangle:
    // v0 = (-1, -1)
    // v1 = ( 3, -1)
    // v2 = (-1,  3)
    let x = f32(i32(vertex_index) == 1) * 4.0 - 1.0;
    let y = f32(i32(vertex_index) == 2) * 4.0 - 1.0;
    
    // Set z to 1.0 to push it to the far clipping plane
    out.clip_position = vec4<f32>(x, y, 1.0, 1.0);
    
    // Reconstruct world-space view direction.
    // Because inv_view_proj has no translation, world_pos IS the view direction!
    let clip_pos = vec4<f32>(x, y, 1.0, 1.0);
    let world_pos = camera.inv_view_proj * clip_pos;
    out.view_dir = normalize(world_pos.xyz / world_pos.w);
    
    return out;
}

@fragment
fn fs_sky(in: SkyOutput) -> @location(0) vec4<f32> {
    let view_dir = normalize(in.view_dir);
    let zenith = normalize(camera.camera_pos.xyz);
    let cos_angle = dot(view_dir, zenith);
    
    let earth_radius = 6378137.0;
    let r = max(length(camera.camera_pos.xyz), earth_radius);
    
    // True mathematical horizon dips below perfectly horizontal due to earth curvature
    let true_horizon_cos = -sqrt(max(1.0 - (earth_radius * earth_radius) / (r * r), 0.0));
    
    // --- DEBUG LINES ---
    if (abs(cos_angle) < 0.0015) {
        return vec4<f32>(1.0, 0.0, 0.0, 1.0); // Red: perfectly horizontal
    }
    if (abs(cos_angle - true_horizon_cos) < 0.0015) {
        return vec4<f32>(0.0, 1.0, 0.0, 1.0); // Green: true earth curvature horizon
    }
    
    let horizon_color = vec3<f32>(0.65, 0.75, 0.85); // Hazy horizon
    let zenith_color = vec3<f32>(0.15, 0.35, 0.75);  // Deep blue
    let space_color = vec3<f32>(0.02, 0.02, 0.04);   // Dark space
    
    // Calculate elevation relative to the true horizon
    let elevation = max(cos_angle - true_horizon_cos, 0.0);
    
    // smoothstep creates a beautiful S-curve. 
    // Reaching 1.0 at elevation 0.4 means the dark blue fully kicks in around 23 degrees above the horizon!
    let gradient_factor = smoothstep(0.0, 0.4, elevation);
    var base_color = mix(horizon_color, zenith_color, gradient_factor);
    
    // Fade to space as altitude increases
    let altitude = max(r - earth_radius, 0.0);
    
    // Start fading at 20km, fully dark by 100km
    let space_fade = clamp((altitude - 20000.0) / 80000.0, 0.0, 1.0); 
    base_color = mix(base_color, space_color, space_fade);
    
    return vec4<f32>(base_color, 1.0);
}
