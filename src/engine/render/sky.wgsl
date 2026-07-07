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
    let x = f32(i32(vertex_index) == 1) * 4.0 - 1.0;
    let y = f32(i32(vertex_index) == 2) * 4.0 - 1.0;
    
    // Set z to 1.0 to push it to the far clipping plane
    out.clip_position = vec4<f32>(x, y, 1.0, 1.0);
    
    // Reconstruct world-space view direction.
    let clip_pos = vec4<f32>(x, y, 1.0, 1.0);
    let world_pos = camera.inv_view_proj * clip_pos;
    
    // DO NOT NORMALIZE HERE! Linear interpolation of normalized vectors causes spherical skew!
    out.view_dir = world_pos.xyz / world_pos.w;
    
    return out;
}

fn ray_sphere_intersect(r0: vec3<f32>, rd: vec3<f32>, radius: f32) -> vec2<f32> {
    let b = 2.0 * dot(rd, r0);
    let c = dot(r0, r0) - radius * radius;
    let d = b * b - 4.0 * c;
    if (d < 0.0) {
        return vec2<f32>(-1.0, -1.0);
    }
    let d_sqrt = sqrt(d);
    return vec2<f32>((-b - d_sqrt) / 2.0, (-b + d_sqrt) / 2.0);
}

@fragment
fn fs_sky(in: SkyOutput) -> @location(0) vec4<f32> {
    let view_dir = normalize(in.view_dir);
    let origin = camera.camera_pos.xyz;
    
    let earth_radius = 6.378137;
    let atmosphere_thickness = 0.15; // 150km boundary for the ray marcher
    let atmosphere_radius = earth_radius + atmosphere_thickness;
    
    let t_atm = ray_sphere_intersect(origin, view_dir, atmosphere_radius);
    
    // We intentionally do NOT use t_earth to cut off the atmosphere depth.
    // The ray marcher will naturally handle rays that hit the earth by accumulating density.
    // The earth mesh renders over this, perfectly hiding the hidden parts and filling any gaps!
    
    var dist_in_atm = 0.0;
    if (t_atm.y > 0.0) {
        let t_start = max(0.0, t_atm.x);
        let t_stop = t_atm.y; 
        dist_in_atm = max(0.0, t_stop - t_start);
    }
    
    // --- ANALYTICAL OPTICAL DEPTH (ZERO LOOPS) ---
    // For mobile GPU performance, we use a loopless analytical approximation of the atmospheric 
    // scattering integral (Chapman function approximation). This is virtually free to compute!
    
    // 1. Find the closest point of the ray to the center of the earth
    let t_closest = -dot(origin, view_dir);
    let t_min = max(0.0, t_closest);
    let p_closest = origin + view_dir * t_min;
    let d = length(p_closest);
    
    // 2. Compute exponential density at that closest point
    let h_normalized = clamp((d - earth_radius) / atmosphere_thickness, 0.0, 1.0);
    // 10 scale heights (e^-10) falloff to space
    let density_at_d = exp(-h_normalized * 10.0);
    
    // 3. Soften the harsh geometric boundary
    // The dist_in_atm calculation has an infinite slope at the exact outer edge, which causes 
    // the "sharp ribbon" visual artifact. We perfectly crush this slope using a smoothstep.
    let boundary_softener = smoothstep(1.0, 0.8, h_normalized);
    
    // 4. Combine for final optical depth
    let optical_depth = density_at_d * dist_in_atm * boundary_softener * 2.0;
    
    let horizon_color = vec3<f32>(0.7, 0.8, 0.9); // Hazy white
    let zenith_color = vec3<f32>(0.15, 0.35, 0.75);  // Deep blue
    let space_color = vec3<f32>(0.02, 0.02, 0.04);   // Dark space
    
    var base_color = space_color;
    if (optical_depth > 0.0) {
        // We push the white haze down towards the horizon by requiring a much higher optical depth
        // before mixing in the horizon color. This keeps the upper sky saturated deep blue.
        let color_mix = smoothstep(1.5, 2.7, optical_depth);
        let atmosphere_color = mix(zenith_color, horizon_color, color_mix);
        
        // True optical absorption/scattering (Beer-Lambert law approximation)
        let opacity = 1.0 - exp(-optical_depth * 10.0); 
        
        base_color = mix(space_color, atmosphere_color, opacity);
    }
    
    return vec4<f32>(base_color, 1.0);
}
