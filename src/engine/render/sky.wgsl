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
    let atmosphere_thickness = 0.15; // 150km for a softer fade
    let atmosphere_radius = earth_radius + atmosphere_thickness;
    
    let t_atm = ray_sphere_intersect(origin, view_dir, atmosphere_radius);
    let t_earth = ray_sphere_intersect(origin, view_dir, earth_radius);
    
    var dist_in_atm = 0.0;
    if (t_atm.y > 0.0) {
        let t_start = max(0.0, t_atm.x);
        var t_stop = t_atm.y;
        if (t_earth.y > 0.0 && t_earth.x > 0.0) {
            t_stop = min(t_stop, t_earth.x);
        }
        dist_in_atm = max(0.0, t_stop - t_start);
    }
    
    let max_dist = sqrt(atmosphere_radius * atmosphere_radius - earth_radius * earth_radius);
    let depth = dist_in_atm / max_dist;
    
    let horizon_color = vec3<f32>(0.65, 0.75, 0.85); // Hazy horizon
    let zenith_color = vec3<f32>(0.15, 0.35, 0.75);  // Deep blue
    let space_color = vec3<f32>(0.02, 0.02, 0.04);   // Dark space
    
    var base_color = space_color;
    if (depth > 0.0) {
        let atmosphere_color = mix(zenith_color, horizon_color, smoothstep(0.05, 1.0, depth));
        base_color = mix(space_color, atmosphere_color, smoothstep(0.0, 0.15, depth));
    }
    
    return vec4<f32>(base_color, 1.0);
}
