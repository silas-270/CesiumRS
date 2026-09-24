// The sky LUT pass: evaluates the atmosphere (atmosphere.wgsl, prepended) once per LUT
// texel. The layout of the texture is documented at the bottom of atmosphere.wgsl.

struct CameraUniform {
    view_proj: mat4x4<f32>,
    inv_view_proj: mat4x4<f32>,
    camera_pos: vec4<f32>,
    sun_params: vec4<f32>,
    sun_dir: vec4<f32>,
    moon_dir: vec4<f32>,
    light_color: vec4<f32>,
};

@group(0) @binding(0)
var<uniform> camera: CameraUniform;

/// Steps for the skylight rows. Short, mostly steep rays.
const SKYLIGHT_STEPS: i32 = 8;

@vertex
fn vs_lut(@builtin(vertex_index) vertex_index: u32) -> @builtin(position) vec4<f32> {
    let x = f32(i32(vertex_index) == 1) * 4.0 - 1.0;
    let y = f32(i32(vertex_index) == 2) * 4.0 - 1.0;
    return vec4<f32>(x, y, 0.0, 1.0);
}

/// Skylight on horizontal ground from three directions (zenith, and low toward and away
/// from the sun), per unit of solar irradiance.
fn ground_skylight(mu: f32) -> vec3<f32> {
    let r = ATMO_R + 0.002;
    let origin = vec3<f32>(0.0, 0.0, r);
    let up = vec3<f32>(0.0, 0.0, 1.0);
    let sun = vec3<f32>(sqrt(max(1.0 - mu * mu, 0.0)), 0.0, mu);
    let low_sun = normalize(vec3<f32>(1.0, 0.0, 0.4));
    let low_anti = normalize(vec3<f32>(-1.0, 0.0, 0.4));
    let zen = atmo_integrate(origin, up, sun, 1.0e9, SKYLIGHT_STEPS);
    let ls = atmo_integrate(origin, low_sun, sun, 1.0e9, SKYLIGHT_STEPS);
    let la = atmo_integrate(origin, low_anti, sun, 1.0e9, SKYLIGHT_STEPS);
    let radiance = 0.5 * atmo_radiance(zen, mu)
        + 0.25 * atmo_radiance(ls, dot(low_sun, sun))
        + 0.25 * atmo_radiance(la, dot(low_anti, sun));
    return radiance * (ATMO_PI / ATMO_SUN_E);
}

@fragment
fn fs_lut(@builtin(position) frag: vec4<f32>) -> @location(0) vec4<f32> {
    let col = floor(frag.x);
    let row = floor(frag.y);
    let u = col / (SKY_LUT_W - 1.0);

    if (row >= SKY_LUT_SKY_ROWS) {
        let mu = mix(GROUND_MU_MIN, 1.0, u);
        if (row < SKY_LUT_SKY_ROWS + 1.0) {
            return vec4<f32>(ground_skylight(mu), 1.0);
        }
        return vec4<f32>(atmo_sun_transmittance(ATMO_R + 0.002, mu), 1.0);
    }

    // The camera, in a local frame with +z up and the sun in the x-z plane.
    let p = atmo_position(camera.camera_pos.xyz);
    let r = length(p);
    let up = p / r;
    let mu_s = clamp(dot(up, camera.sun_dir.xyz), -1.0, 1.0);
    let sun = vec3<f32>(sqrt(max(1.0 - mu_s * mu_s, 0.0)), 0.0, mu_s);

    let v = row / (SKY_LUT_SKY_ROWS - 1.0);
    let ang = sky_lut_angles(u, v, r);
    let st = sin(ang.x);
    let dir = vec3<f32>(st * cos(ang.y), st * sin(ang.y), cos(ang.x));
    let s = atmo_integrate(vec3<f32>(0.0, 0.0, r), dir, sun, 1.0e9, ATMO_VIEW_STEPS);
    return vec4<f32>(atmo_radiance(s, dot(dir, sun)), 1.0);
}
