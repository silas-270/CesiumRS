struct CameraUniform {
    view_proj: mat4x4<f32>,
    inv_view_proj: mat4x4<f32>,
    camera_pos: vec4<f32>,
    sun_params: vec4<f32>,   // [altitude_scalar, saturation, contrast, brightness]
    sun_dir: vec4<f32>,      // xyz toward the sun, w = sin(elevation)
    moon_dir: vec4<f32>,     // xyz toward the moon, w = lit fraction
    light_color: vec4<f32>,  // rgb key light hue, w = strength
};

@group(0) @binding(0)
var<uniform> camera: CameraUniform;

@group(1) @binding(0) var t_diffuse: texture_2d<f32>;
@group(1) @binding(1) var s_diffuse: sampler;

struct ModelPushConstants {
    model_matrix_0: vec4<f32>,
    model_matrix_1: vec4<f32>,
    model_matrix_2: vec4<f32>,
    model_matrix_3: vec4<f32>,
    camera_pos: vec4<f32>,
    viewport_size: vec2<f32>,
    // Minimum on-screen size in pixels the model is inflated to. 0.0 = true world scale.
    min_pixel_size: f32,
    // Clip-space depth bias applied as z += depth_bias * w. 0.0 = none.
    depth_bias: f32,
    // Flat lighting floor. 0.5 reproduces the old hardcoded ambient exactly.
    ambient_override: f32,
    // Blinn-Phong specular highlight strength. 0.0 disables it entirely.
    specular_strength: f32,
    // Triplanar procedural surface-detail strength. 0.0 disables it entirely.
    detail_strength: f32,
    // Fresnel-style edge light from the key light. 0.0 disables it entirely.
    rim_strength: f32,
    // How much of the key light's direction this model feels. Low for interiors.
    diffuse_weight: f32,
}

var<push_constant> push: ModelPushConstants;

struct VertexInput {
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) uv: vec2<f32>,
    @location(3) color: vec4<f32>,
}

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) normal: vec3<f32>,
    @location(1) uv: vec2<f32>,
    @location(2) color: vec4<f32>,
    // Raw, pre-transform vertex attributes for the triplanar detail sampler below. Object-
    // local, not camera-relative world space, so the noise stays put as the camera moves.
    @location(3) local_pos: vec3<f32>,
    @location(4) local_normal: vec3<f32>,
    // Camera-relative position, for the specular view direction (rendering is already
    // camera-relative, so `-view_pos` is a free, correct direction back to the camera).
    @location(5) view_pos: vec3<f32>,
}

@vertex
fn vs_main(model: VertexInput) -> VertexOutput {
    var out: VertexOutput;
    
    let model_matrix = mat4x4<f32>(
        push.model_matrix_0,
        push.model_matrix_1,
        push.model_matrix_2,
        push.model_matrix_3,
    );

    // Distant models (the aircraft) are inflated so they stay legible on screen. A model
    // drawn at true world scale — the cockpit interior, which surrounds the camera —
    // passes min_pixel_size = 0.0 and skips the boost entirely.
    var scale_multiplier = 1.0;
    if (push.min_pixel_size > 0.0) {
        let center_world = model_matrix * vec4<f32>(0.0, 0.0, 0.0, 1.0);
        let dist_to_cam = max(length(center_world.xyz), 0.000001);

        // `world_scale` is the size of the mesh in engine units, since the mesh is normalized to radius 1.0
        let world_scale = length(model_matrix[0].xyz);
        let physical_size_engine = 2.0 * world_scale; // diameter

        let fov_factor = 1.5;
        let pixels_per_engine_unit = (1.0 / dist_to_cam) * push.viewport_size.y * fov_factor;

        let size_pixels = max(physical_size_engine * pixels_per_engine_unit, 0.00001);

        let needed_scale = push.min_pixel_size / size_pixels;

        // We want the plane to never be smaller than 1.0 (its true size)
        // and never larger than some huge factor (e.g., to prevent it from covering the globe)
        let max_scale = max(1.0, 4000000.0 / (6378137.0 * max(physical_size_engine, 0.000001)));

        scale_multiplier = clamp(needed_scale, 1.0, max_scale);
    }

    // Apply scaling
    let scaled_pos = model.position * scale_multiplier;
    
    // Apply model matrix (which already has translation and rotation)
    let final_world_pos = model_matrix * vec4<f32>(scaled_pos, 1.0);

    out.clip_position = camera.view_proj * vec4<f32>(final_world_pos.xyz, 1.0);
    // Apply a slight depth bias to prevent the airplane from clipping into the earth's surface
    out.clip_position.z = out.clip_position.z + push.depth_bias * out.clip_position.w;

    // Transform normal to world space (ignoring non-uniform scaling for now)
    let normal_matrix = mat3x3<f32>(
        model_matrix[0].xyz,
        model_matrix[1].xyz,
        model_matrix[2].xyz
    );
    out.normal = normalize(normal_matrix * model.normal);
    out.uv = model.uv;
    out.color = model.color;
    out.local_pos = model.position;
    out.local_normal = model.normal;
    out.view_pos = final_world_pos.xyz;

    return out;
}

// Cheap integer hash (wang-hash style), matching the one used to generate the model's
// procedural grain texture on the Rust side (see cesium-flight/src/cockpit_texture.rs),
// extended to a third argument so each triplanar projection plane gets a decorrelated
// pattern instead of visibly repeating at object bounds where two planes meet.
fn hash3(x: u32, y: u32, z: u32) -> u32 {
    var h = x * 374761393u + y * 668265263u + z * 2147483647u;
    h = (h ^ (h >> 13u)) * 1274126177u;
    return h ^ (h >> 16u);
}

fn hash_to_float(h: u32) -> f32 {
    return f32(h & 0x00FFFFFFu) / f32(0x01000000u);
}

// Bilinear value noise on one 2D plane, decorrelated per plane_id.
fn value_noise(p: vec2<f32>, plane_id: u32) -> f32 {
    // Offset well away from zero so floor()/cast to u32 never has to handle negative
    // cockpit-local coordinates.
    let pp = p + vec2<f32>(100000.0, 100000.0);
    let cell = floor(pp);
    let f = fract(pp);
    let x0 = u32(cell.x);
    let y0 = u32(cell.y);

    let h00 = hash_to_float(hash3(x0, y0, plane_id));
    let h10 = hash_to_float(hash3(x0 + 1u, y0, plane_id));
    let h01 = hash_to_float(hash3(x0, y0 + 1u, plane_id));
    let h11 = hash_to_float(hash3(x0 + 1u, y0 + 1u, plane_id));

    let sx = smoothstep(0.0, 1.0, f.x);
    let sy = smoothstep(0.0, 1.0, f.y);
    let top = mix(h00, h10, sx);
    let bottom = mix(h01, h11, sx);
    return mix(top, bottom, sy);
}

// Object-space triplanar surface grain. UV-independent (fine here since ~63% of the
// cockpit's primitives have no UVs at all) and stable under camera/aircraft motion since
// it's keyed on the model's own local coordinates, not the camera-relative world position.
fn triplanar_detail(local_pos: vec3<f32>, local_normal: vec3<f32>) -> f32 {
    // Cycles per metre. The cockpit is drawn at true world scale (no unit-radius
    // normalisation), so this is a real physical grain size (~4.5mm cells), not a guess.
    let freq = 220.0;

    let weights_raw = pow(abs(local_normal), vec3<f32>(4.0));
    let weights = weights_raw / max(weights_raw.x + weights_raw.y + weights_raw.z, 0.0001);

    let nx = value_noise(local_pos.yz * freq, 0u);
    let ny = value_noise(local_pos.xz * freq, 1u);
    let nz = value_noise(local_pos.xy * freq, 2u);

    return nx * weights.x + ny * weights.y + nz * weights.z;
}

@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
    let normal = normalize(in.normal);
    // `view_pos` is camera-relative world space, so this is the direction back to the eye
    // and `normal` is already in the same frame the sun direction is given in.
    let view_dir = normalize(-in.view_pos);

    let key_color = camera.light_color.rgb;
    let key_strength = camera.light_color.a;

    // Two lights, weighted, rather than one that switches between them. The moon sits
    // exactly opposite the sun, so picking whichever is higher on a threshold teleports
    // the light through 180° the instant the sun crosses it, and the lit side of the
    // aircraft swaps between one frame and the next. Summing both contributions is what a
    // pair of light sources actually does, and it has no edge to fall off. The weights run
    // on the same dusk-to-night ramp as the sky.
    let night_key = smoothstep(-0.02, -0.22, camera.sun_dir.w);
    let from_sun = max(dot(normal, camera.sun_dir.xyz), 0.0) * (1.0 - night_key);
    let from_moon = max(dot(normal, camera.moon_dir.xyz), 0.0) * night_key;
    let lit = (from_sun + from_moon) * key_strength * push.diffuse_weight;
    // Ambient is a floor the key light fills up to, so the two always sum to exactly 1.
    // Adding them instead ran to 1.2 and clipped: nothing could be darker than the floor,
    // and every surface facing the light blew out to flat white. That is what made the
    // cockpit window frames glow.
    let light_intensity = push.ambient_override + (1.0 - push.ambient_override) * lit;

    // Sample texture
    let tex_color = textureSample(t_diffuse, s_diffuse, in.uv).rgb;

    // Subtle procedural surface grain, independent of UVs (push.detail_strength = 0.0
    // disables this entirely at zero extra cost).
    let detail_noise = triplanar_detail(in.local_pos, normalize(in.local_normal));
    let detail = 1.0 + (detail_noise - 0.5) * push.detail_strength;

    // Soft Blinn-Phong catch-light (push.specular_strength = 0.0 disables it), blended
    // across the same two lights so the highlight does not jump either.
    let half_sun = normalize(camera.sun_dir.xyz + view_dir);
    let half_moon = normalize(camera.moon_dir.xyz + view_dir);
    let spec = (pow(max(dot(normal, half_sun), 0.0), 28.0) * (1.0 - night_key)
        + pow(max(dot(normal, half_moon), 0.0), 28.0) * night_key)
        * push.specular_strength * key_strength;

    // Rim light: surfaces turning away from the eye catch the sky behind them. Costs one
    // dot product and is most of what makes an aircraft read as lit from outside rather
    // than painted, especially with a low sun.
    let rim = pow(1.0 - max(dot(normal, view_dir), 0.0), 3.0) * push.rim_strength * key_strength;

    // Everything is tinted by the key light, which is near-white by day, amber at sunset
    // and blue by moonlight. The hue is normalised so it never adds brightness of its own.
    let color = tex_color * in.color.rgb * light_intensity * detail * key_color
        + key_color * (spec + rim);

    return vec4<f32>(clamp(color, vec3<f32>(0.0), vec3<f32>(1.0)), in.color.a);
}
