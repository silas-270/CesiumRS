// City labels: instanced screen-space quads hung off a 3D anchor.
//
// Every quad is positioned in pixels relative to its anchor's projected position, and
// keeps the anchor's clip-space depth, so the whole label sits at the depth of the
// point it names. The pipeline writes that depth with compare `Always`: labels always
// cover the world drawn before them, and any model drawn after them (aircraft, cockpit)
// covers them exactly where it is nearer than the anchor.

struct CameraUniform {
    view_proj: mat4x4<f32>,
};

@group(0) @binding(0)
var<uniform> camera: CameraUniform;

struct LabelUniform {
    viewport_px: vec2<f32>,
    // 1.0 when the target is an sRGB format (the hardware encodes, so write linear);
    // 0.0 when it is not (write the sRGB values as they are).
    target_is_srgb: f32,
    _pad: f32,
};

@group(1) @binding(0)
var<uniform> params: LabelUniform;
@group(1) @binding(1)
var atlas: texture_2d<f32>;
@group(1) @binding(2)
var atlas_sampler: sampler;

const KIND_PILL: u32 = 0u;
const KIND_GLYPH: u32 = 1u;
const KIND_DOT: u32 = 2u;

struct Instance {
    // Anchor, camera-relative (megametres).
    @location(0) anchor: vec3<f32>,
    @location(1) kind: u32,
    // Quad in pixels relative to the anchor's screen position, y down: min.xy, max.xy.
    @location(2) rect: vec4<f32>,
    @location(3) uv: vec4<f32>,
    // sRGB colour, straight alpha.
    @location(4) color: vec4<f32>,
    // Pill: corner radius in pixels. Dot: radius in pixels.
    @location(5) radius: f32,
};

struct VertexOutput {
    @builtin(position) clip: vec4<f32>,
    @location(0) local: vec2<f32>,
    @location(1) half_size: vec2<f32>,
    @location(2) uv: vec2<f32>,
    @location(3) color: vec4<f32>,
    @location(4) @interpolate(flat) kind: u32,
    @location(5) @interpolate(flat) radius: f32,
};

@vertex
fn vs_main(@builtin(vertex_index) vi: u32, inst: Instance) -> VertexOutput {
    // Triangle strip corners: (0,0) (1,0) (0,1) (1,1).
    let corner = vec2<f32>(f32(vi & 1u), f32((vi >> 1u) & 1u));
    let px = mix(inst.rect.xy, inst.rect.zw, corner);

    var clip = camera.view_proj * vec4<f32>(inst.anchor, 1.0);
    // The anchor snapped to a whole pixel, so glyph bitmaps (whole-pixel offsets from
    // it) land 1:1 on the screen's pixel grid.
    let vp = params.viewport_px;
    let anchor_px = round((clip.xy / clip.w * vec2<f32>(0.5, -0.5) + 0.5) * vp);
    let screen_px = anchor_px + px;
    let ndc = (screen_px / vp - 0.5) * vec2<f32>(2.0, -2.0);
    clip = vec4<f32>(ndc * clip.w, clip.z, clip.w);

    var out: VertexOutput;
    out.clip = clip;
    let center = 0.5 * (inst.rect.xy + inst.rect.zw);
    out.half_size = 0.5 * (inst.rect.zw - inst.rect.xy);
    out.local = px - center;
    out.uv = mix(inst.uv.xy, inst.uv.zw, corner);
    out.color = inst.color;
    out.kind = inst.kind;
    out.radius = inst.radius;
    return out;
}

fn srgb_to_linear(c: vec3<f32>) -> vec3<f32> {
    let lo = c / 12.92;
    let hi = pow((c + 0.055) / 1.055, vec3<f32>(2.4));
    return select(hi, lo, c <= vec3<f32>(0.04045));
}

@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
    // Sampled unconditionally: textureSample must be in uniform control flow.
    // Glyph coverage goes through egui's coverage gamma (epaint `srgba_pixels`), so text
    // keeps the weight it had when egui drew it.
    var coverage = pow(textureSample(atlas, atlas_sampler, in.uv).r, 0.55);
    if (in.kind == KIND_PILL) {
        let r = min(in.radius, min(in.half_size.x, in.half_size.y));
        let q = abs(in.local) - (in.half_size - vec2<f32>(r));
        let d = length(max(q, vec2<f32>(0.0))) + min(max(q.x, q.y), 0.0) - r;
        coverage = clamp(0.5 - d, 0.0, 1.0);
    } else if (in.kind == KIND_DOT) {
        coverage = clamp(0.5 - (length(in.local) - in.radius), 0.0, 1.0);
    }

    let a = coverage * in.color.a;
    // Fully transparent texels (pill corners, glyph gaps) must not write depth.
    if (a < 1.0 / 255.0) {
        discard;
    }
    // Premultiplied in gamma space and only then decoded, as egui-wgpu does for an sRGB
    // target: it blends as if in gamma space, which is what makes the text read thin.
    let premul_gamma = in.color.rgb * a;
    if (params.target_is_srgb > 0.5) {
        return vec4<f32>(srgb_to_linear(premul_gamma), a);
    }
    return vec4<f32>(premul_gamma, a);
}
