//! The flight deck's display atlas: one static image, four panels, painted in code.
//!
//! ## Why an atlas rather than a texture per screen
//! A [`ModelRenderer`](cesium_engine::render::model_pipeline::pipeline::ModelRenderer)
//! binds one texture for the whole model and draws it in one call, so "give this surface
//! an image" — the way you would do it in Blender — is not something the model can carry.
//! It does not need to. The four forward displays are each a single four-vertex quad whose
//! UVs already occupy their own band of one 0..1 space:
//!
//! | quad | x in the cockpit | u band |
//! |---|---|---|
//! | `group_18.026` | −0.574 m | 0.0509 – 0.2396 |
//! | `group_18.022` | −0.229 m | 0.2635 – 0.4522 |
//! | `group_18.023` | +0.226 m | 0.5432 – 0.7319 |
//! | `group_18.027` | +0.572 m | 0.7558 – 0.9445 |
//!
//! all spanning v 0.0828 – 0.9302, with u running left to right and v downwards — plain
//! image convention. That is an atlas layout: the GLB was authored against one and lost
//! the image in conversion (`images: 0`), keeping the coordinates that addressed it. So
//! this module paints the image that was missing, and nothing about the geometry, the UVs
//! or the draw call has to change.
//!
//! ## Why the rest of the cockpit has to be parked
//! Seven other materials also have UVs in that same 0..1 space — `pedestal_01`,
//! `ModeControl_Panel1`, `Side_Display` and friends — and would sample this atlas too,
//! smearing fragments of a navigation display across the pedestal. [`uv_for_material`]
//! collapses every one of them onto a single white texel, which leaves them rendering at
//! their base colour exactly as they did when the model had no texture at all. A constant
//! UV also has zero screen-space derivative, so the GPU selects mip 0 and they are immune
//! to the rest of the atlas at any distance.
//!
//! ## Static
//! The atlas is built once at load. Nothing here updates per frame, so the cost at runtime
//! is exactly the texture memory — the fragment shader already sampled a texture for every
//! cockpit pixel, it was just a 1x1 white one.

use std::f32::consts::PI;

/// Atlas dimensions.
///
/// Deliberately very wide. Each screen is 0.307 m across and 0.233 m down, so 1.32:1
/// landscape, but its UV band is 0.189 wide by 0.847 tall — the reverse. Square texels on
/// the panel therefore need `W/H = (0.847/0.189) * 1.32 ≈ 5.9`, and 3072x512 is 6.0, a
/// 1.3% aspect error that no one will see. It gives each screen 580x434 texels, a little
/// over one texel per pixel at the size the displays occupy from the seat, and costs
/// 6.3 MB (8.4 MB once mipped).
pub const ATLAS_W: u32 = 3072;
pub const ATLAS_H: u32 = 512;

/// The only material whose primitives are allowed to address the atlas.
pub const SCREEN_MATERIAL: &str = "Main_Display";

/// What each panel shows, left to right as the pilot sees them.
#[derive(Clone, Copy)]
enum Role {
    /// Primary flight display: attitude, speed, altitude, heading.
    Pfd,
    /// Navigation display: compass arc, range rings, track.
    Nd,
}

/// One screen's rectangle in UV space, measured off the GLB's own vertices.
struct Band {
    u0: f32,
    u1: f32,
    v0: f32,
    v1: f32,
    role: Role,
}

const BANDS: [Band; 4] = [
    // Captain's outboard and inboard, then first officer's inboard and outboard. These are
    // the quads' own corner values to full f32 precision, not rounded: see BAND_EPSILON.
    Band { u0: 0.050_939_33, u1: 0.239_630_80, v0: V0, v1: V1, role: Role::Pfd },
    Band { u0: 0.263_536_01, u1: 0.452_227_41, v0: V0, v1: V1, role: Role::Nd },
    Band { u0: 0.543_178_98, u1: 0.731_870_41, v0: V0, v1: V1, role: Role::Nd },
    Band { u0: 0.755_775_63, u1: 0.944_467_13, v0: V0, v1: V1, role: Role::Pfd },
];

/// All four quads share these, top and bottom.
const V0: f32 = 0.082_822_20;
const V1: f32 = 0.930_173_70;

/// Slack on the band test in [`uv_for_material`].
///
/// Worth understanding, because without it the screens are spectacularly broken rather
/// than slightly off. The test runs per vertex, and a quad's corners sit exactly on its
/// band's edge — so if one corner falls outside by a rounding error it gets parked while
/// the opposite corner keeps its coordinate, and the UV then interpolates from one to the
/// other, smearing the entire atlas across that screen. Writing the bounds to four decimal
/// places was enough to do it: `group_18.026` ends at u = 0.23963080, which is greater
/// than 0.2396.
///
/// The exact values above make that impossible today; this makes it impossible to
/// reintroduce. The ceiling is 0.0119 — half the 0.0239 gap between two bands — and the
/// tightest clearance to a non-screen primitive is 0.0134, so 0.004 has a wide margin at
/// both ends while being four orders of magnitude above f32 rounding.
const BAND_EPSILON: f32 = 0.004;

/// Texels of each band's own edge colour extended outwards, so that minified mip levels
/// average a screen against more of itself rather than against its neighbour or the
/// surrounding panel. The narrowest gap between two bands is 73 texels, so 16 a side
/// leaves a clear margin; bleed is held off until roughly mip 4, by which point a whole
/// screen is 36 texels wide and the aircraft is nowhere near the flight deck.
const PAD: i32 = 16;

/// Where the parked materials land, in texels.
///
/// Both sit in gaps between bands that no primitive addresses — the model's own UVs skip
/// them — and outside the padding, so nothing painted here can bleed into a screen.
/// `WHITE` leaves a material's base colour untouched; `GLASS` is for the few
/// [`SCREEN_MATERIAL`] primitives that are bezel and standby glass rather than screens,
/// whose base colour this module forces to white along with the real displays.
const PARK_WHITE_TEXEL: (u32, u32) = (772, 256);
const PARK_GLASS_TEXEL: (u32, u32) = (2285, 256);
/// Size of the flat block stamped at each parked texel, so bilinear filtering at the
/// parked coordinate has nothing but that colour to interpolate between.
const PARK_BLOCK: i32 = 8;

// ── Palette ───────────────────────────────────────────────────────────────────
// Final on-screen values. The display material is drawn unlit, so what is painted here is
// what reaches the eye, unscaled by the flight deck's ambient floor.

const SKY: [u8; 3] = [40, 118, 194];
const GROUND: [u8; 3] = [132, 88, 44];
const GLASS: [u8; 3] = [16, 18, 23];
const TAPE: [u8; 3] = [28, 30, 36];
const WHITE: [u8; 3] = [235, 238, 242];
const GREY: [u8; 3] = [150, 155, 162];
const GREEN: [u8; 3] = [64, 226, 108];
const MAGENTA: [u8; 3] = [226, 92, 226];
const YELLOW: [u8; 3] = [236, 206, 60];
const CYAN: [u8; 3] = [96, 222, 238];

// ── What the panels read ──────────────────────────────────────────────────────
// Static, by design. One place to change them.

const IAS_KT: i32 = 280;
const ALT_FT: i32 = 35_000;
const HDG_DEG: i32 = 160;
const GS_KT: i32 = 480;
const ND_RANGE_NM: i32 = 40;

/// Rewrites a vertex's UV so that only the display quads read the atlas.
///
/// See the module docs: everything that is not a screen is collapsed onto a single texel.
pub fn uv_for_material(name: Option<&str>, uv: [f32; 2]) -> [f32; 2] {
    let park = |t: (u32, u32)| {
        [
            (t.0 as f32 + 0.5) / ATLAS_W as f32,
            (t.1 as f32 + 0.5) / ATLAS_H as f32,
        ]
    };

    if name != Some(SCREEN_MATERIAL) {
        return park(PARK_WHITE_TEXEL);
    }

    // `Main_Display` covers eleven primitives but only four are screens. The other seven —
    // bezel, the standby instrument, a clock — sit in the gaps between the bands, where
    // the original atlas presumably had content this one does not. Keying on the UV rather
    // than the material name is what separates them, which is why this hook sees both.
    let inside = BANDS.iter().any(|b| {
        uv[0] >= b.u0 - BAND_EPSILON
            && uv[0] <= b.u1 + BAND_EPSILON
            && uv[1] >= b.v0 - BAND_EPSILON
            && uv[1] <= b.v1 + BAND_EPSILON
    });
    if inside {
        uv
    } else {
        park(PARK_GLASS_TEXEL)
    }
}

/// A display is a light source, not a lit surface, so it takes none of the flight deck's
/// shading. Without this it renders at the ambient floor — around a quarter value — and
/// reads as a dark grey rectangle whatever is painted on it.
pub fn unlit_for_material(name: Option<&str>) -> f32 {
    if name == Some(SCREEN_MATERIAL) {
        1.0
    } else {
        0.0
    }
}

/// Paints the atlas: `(width, height, RGBA8, row-major, no padding)`.
pub fn build_atlas() -> (u32, u32, Vec<u8>) {
    let mut atlas = Canvas::new(ATLAS_W as i32, ATLAS_H as i32, GLASS);
    for band in &BANDS {
        let x0 = (band.u0 * ATLAS_W as f32).round() as i32;
        let x1 = (band.u1 * ATLAS_W as f32).round() as i32;
        let y0 = (band.v0 * ATLAS_H as f32).round() as i32;
        let y1 = (band.v1 * ATLAS_H as f32).round() as i32;
        let (w, h) = (x1 - x0, y1 - y0);

        let mut panel = Canvas::new(w, h, GLASS);
        match band.role {
            Role::Pfd => paint_pfd(&mut panel),
            Role::Nd => paint_nd(&mut panel),
        }
        atlas.blit(&panel, x0, y0);
        atlas.extend_edges(x0, y0, w, h, PAD);
    }

    stamp_park(&mut atlas, PARK_WHITE_TEXEL, [255, 255, 255]);
    stamp_park(&mut atlas, PARK_GLASS_TEXEL, GLASS);

    (ATLAS_W, ATLAS_H, atlas.into_rgba())
}

fn stamp_park(atlas: &mut Canvas, texel: (u32, u32), color: [u8; 3]) {
    let r = PARK_BLOCK / 2;
    atlas.rect(
        texel.0 as i32 - r,
        texel.1 as i32 - r,
        PARK_BLOCK,
        PARK_BLOCK,
        color,
    );
}

// ── Primary flight display ────────────────────────────────────────────────────

fn paint_pfd(c: &mut Canvas) {
    let (w, h) = (c.w, c.h);
    c.rect(0, 0, w, h, GLASS);

    // Column layout, as fractions of the panel width: speed tape, attitude, altitude tape,
    // vertical speed. The proportions are Boeing-ish rather than measured.
    let tape_w = (w as f32 * 0.125) as i32;
    let spd_x = (w as f32 * 0.045) as i32;
    let ai_x = spd_x + tape_w + (w as f32 * 0.035) as i32;
    let alt_x = (w as f32 * 0.735) as i32;
    let vs_x = alt_x + tape_w + (w as f32 * 0.020) as i32;

    let top = (h as f32 * 0.10) as i32;
    let band_h = (h as f32 * 0.62) as i32;
    let ai_w = alt_x - ai_x - (w as f32 * 0.030) as i32;

    // One text size for the whole panel, in multiples of the 5x7 cell. What fixes it is
    // the altitude readout: five digits at scale n are `29n` texels wide, and the box they
    // sit in is a tape plus its overhang, 118 across. Scale 3 fits, 4 does not — so the
    // tick labels, a size down at 2, follow from that rather than the other way round.
    let s = (h / 200).max(1);

    paint_fma(c, ai_x, (h as f32 * 0.015) as i32, ai_w, s);
    paint_attitude(c, ai_x, top, ai_w, band_h, s);
    paint_speed_tape(c, spd_x, top, tape_w, band_h, s);
    paint_alt_tape(c, alt_x, top, tape_w, band_h, s);
    paint_vs_scale(c, vs_x, top, (w as f32 * 0.055) as i32, band_h);
    paint_heading_strip(c, ai_x, top + band_h + (h as f32 * 0.045) as i32, ai_w, (h as f32 * 0.15) as i32, s);
}

/// How far a boxed readout overhangs its tape on each side, so five digits fit.
const READOUT_OVERHANG: i32 = 23;

/// The flight mode annunciator: the green strip of engaged modes across the top.
fn paint_fma(c: &mut Canvas, x: i32, y: i32, w: i32, s: i32) {
    let h = (w as f32 * 0.085) as i32;
    c.rect(x, y, w, h, TAPE);
    let third = w / 3;
    c.text(x + third / 2, y + h / 2 - 3 * s, "LNAV", s, GREEN, Anchor::Center);
    c.text(x + third + third / 2, y + h / 2 - 3 * s, "VNAV", s, GREEN, Anchor::Center);
    c.text(x + 2 * third + third / 2, y + h / 2 - 3 * s, "CMD", s, GREEN, Anchor::Center);
    // The dividers between the three columns.
    c.rect(x + third, y + 2, 1, h - 4, GREY);
    c.rect(x + 2 * third, y + 2, 1, h - 4, GREY);
}

fn paint_attitude(c: &mut Canvas, x: i32, y: i32, w: i32, h: i32, s: i32) {
    let cx = x + w / 2;
    let cy = y + h / 2;

    // Level flight: sky above the centre line, ground below, horizon straight across.
    c.rect(x, y, w, h / 2, SKY);
    c.rect(x, cy, w, h - h / 2, GROUND);
    c.rect(x, cy - 1, w, 3, WHITE);

    // Pitch ladder. 26 degrees across the full height puts the 10-degree bars comfortably
    // inside the sky and ground fields.
    let deg_px = h as f32 / 26.0;
    for step in 1..=4 {
        let deg = step * 5;
        let long = deg % 10 == 0;
        let half = if long { w / 6 } else { w / 12 };
        for sign in [-1i32, 1] {
            let ly = cy + (sign as f32 * deg as f32 * deg_px) as i32;
            if ly < y + 4 || ly > y + h - 4 {
                continue;
            }
            c.rect(cx - half, ly, half * 2, 2, WHITE);
            if long {
                let label = deg.to_string();
                c.text(cx - half - 4 * s, ly - 3 * s, &label, s, WHITE, Anchor::Right);
                c.text(cx + half + 4 * s, ly - 3 * s, &label, s, WHITE, Anchor::Left);
            }
        }
    }

    // Bank scale: an arc of ticks over the top of the ball with a pointer at its apex.
    let r = (h as f32 * 0.44) as i32;
    for deg in [-45i32, -30, -20, -10, 10, 20, 30, 45] {
        let a = deg as f32 * PI / 180.0;
        let len = if deg.abs() >= 30 { 10 } else { 6 };
        let (sx, sy) = (cx as f32 + r as f32 * a.sin(), cy as f32 - r as f32 * a.cos());
        let (ex, ey) = (
            cx as f32 + (r + len) as f32 * a.sin(),
            cy as f32 - (r + len) as f32 * a.cos(),
        );
        c.line(sx as i32, sy as i32, ex as i32, ey as i32, 2, WHITE);
    }
    // Wings level, so the pointer sits on the centreline.
    c.tri(
        [(cx, cy - r + 2), (cx - 7, cy - r + 14), (cx + 7, cy - r + 14)],
        YELLOW,
    );

    // Aircraft reference symbol.
    c.rect(cx - 5, cy - 4, 10, 8, YELLOW);
    c.rect(cx - w / 4, cy - 2, w / 8, 5, YELLOW);
    c.rect(cx + w / 8, cy - 2, w / 8, 5, YELLOW);
}

fn paint_speed_tape(c: &mut Canvas, x: i32, y: i32, w: i32, h: i32, s: i32) {
    c.rect(x, y, w, h, TAPE);
    let px_per_kt = h as f32 / 80.0;
    let cy = y + h / 2;

    let lo = IAS_KT - 40;
    let hi = IAS_KT + 40;
    for v in (lo..=hi).filter(|v| v % 10 == 0) {
        let ty = cy - ((v - IAS_KT) as f32 * px_per_kt) as i32;
        if ty < y + 2 || ty > y + h - 2 {
            continue;
        }
        let long = v % 20 == 0;
        let len = if long { w / 4 } else { w / 8 };
        c.rect(x + w - len, ty, len, 2, WHITE);
        if long {
            c.text(x + w - w / 4 - 3 * s, ty - 3 * s, &v.to_string(), s, WHITE, Anchor::Right);
        }
    }

    paint_readout(
        c,
        x - READOUT_OVERHANG,
        cy,
        w + 2 * READOUT_OVERHANG,
        12 * s + 6,
        &IAS_KT.to_string(),
        s + 1,
    );
}

fn paint_alt_tape(c: &mut Canvas, x: i32, y: i32, w: i32, h: i32, s: i32) {
    c.rect(x, y, w, h, TAPE);
    let px_per_ft = h as f32 / 1600.0;
    let cy = y + h / 2;

    let lo = ALT_FT - 800;
    let hi = ALT_FT + 800;
    for v in (lo..=hi).filter(|v| v % 100 == 0) {
        let ty = cy - ((v - ALT_FT) as f32 * px_per_ft) as i32;
        if ty < y + 2 || ty > y + h - 2 {
            continue;
        }
        let long = v % 200 == 0;
        let len = if long { w / 4 } else { w / 8 };
        c.rect(x, ty, len, 2, WHITE);
        if long {
            // Hundreds only, the way an altitude tape is labelled.
            c.text(x + w / 4 + 3 * s, ty - 3 * s, &format!("{}", v / 100), s, WHITE, Anchor::Left);
        }
    }

    paint_readout(
        c,
        x - READOUT_OVERHANG,
        cy,
        w + 2 * READOUT_OVERHANG,
        12 * s + 6,
        &ALT_FT.to_string(),
        s + 1,
    );
    // Selected altitude, in the magenta the autopilot's targets are drawn in.
    c.text(x + w / 2, y - 9 * s, &ALT_FT.to_string(), s, MAGENTA, Anchor::Center);
}

/// The boxed current value that sits over the middle of a tape.
fn paint_readout(c: &mut Canvas, x: i32, cy: i32, w: i32, h: i32, text: &str, s: i32) {
    c.rect(x, cy - h / 2, w, h, [10, 11, 14]);
    c.outline(x, cy - h / 2, w, h, 2, WHITE);
    c.text(x + w / 2, cy - 3 * s, text, s, WHITE, Anchor::Center);
}

fn paint_vs_scale(c: &mut Canvas, x: i32, y: i32, w: i32, h: i32) {
    c.rect(x, y, w, h, [20, 21, 26]);
    let cy = y + h / 2;
    // Ticks at 1000 and 2000 feet per minute either side of level.
    for frac in [0.22f32, 0.42] {
        for sign in [-1.0f32, 1.0] {
            let ty = cy + (sign * frac * h as f32) as i32;
            c.rect(x + 2, ty, w / 2, 2, GREY);
        }
    }
    // Level, so the needle is horizontal.
    c.rect(x + 2, cy - 1, w - 4, 3, GREEN);
}

fn paint_heading_strip(c: &mut Canvas, x: i32, y: i32, w: i32, h: i32, s: i32) {
    c.rect(x, y, w, h, TAPE);
    let cx = x + w / 2;
    let px_per_deg = w as f32 / 60.0;

    for d in (HDG_DEG - 30)..=(HDG_DEG + 30) {
        if d % 5 != 0 {
            continue;
        }
        let tx = cx + ((d - HDG_DEG) as f32 * px_per_deg) as i32;
        if tx < x + 2 || tx > x + w - 2 {
            continue;
        }
        let long = d % 10 == 0;
        let len = if long { h / 3 } else { h / 6 };
        c.rect(tx, y, 2, len, WHITE);
        if long {
            let n = ((d % 360) + 360) % 360;
            c.text(tx, y + h / 3 + 2 * s, &format!("{:02}", n / 10), s, WHITE, Anchor::Center);
        }
    }

    c.tri([(cx, y - 2), (cx - 6, y - 12), (cx + 6, y - 12)], WHITE);
}

// ── Navigation display ────────────────────────────────────────────────────────

fn paint_nd(c: &mut Canvas) {
    let (w, h) = (c.w, c.h);
    c.rect(0, 0, w, h, [10, 12, 17]);

    // Same 5x7 cell as the flight display next to it, so the two panels read as a set.
    // `s + 1` for the compass labels, which sit alone inside the arc and can carry it.
    let s = (h / 200).max(1);
    let s_rose = s + 1;
    let margin = 4 * s;

    // The aircraft sits low and centred, looking up the compass arc.
    let cx = w / 2;
    let cy = (h as f32 * 0.88) as i32;
    let r = (h as f32 * 0.72) as i32;

    // Range rings at a half and a full range, dashed the way a real one is.
    for (frac, label) in [(0.5f32, ND_RANGE_NM / 2), (1.0, ND_RANGE_NM)] {
        let rr = (r as f32 * frac) as i32;
        c.dashed_arc(cx, cy, rr, -70.0, 70.0, 2, GREY, 9, 7);
        if frac < 1.0 {
            // Tucked just inside the left-hand end of the ring itself. Placing it level
            // with the aircraft instead left it floating in open space, because the ring
            // is only a 140-degree arc and has nothing down there.
            let a = -70.0f32 * PI / 180.0;
            let lx = cx as f32 + rr as f32 * a.sin();
            let ly = cy as f32 - rr as f32 * a.cos();
            c.text(lx as i32 + 3 * s, ly as i32 - 3 * s, &label.to_string(), s, GREY, Anchor::Left);
        }
    }

    // Compass arc: ticks every 5 degrees, labels every 30.
    c.arc(cx, cy, r, -70.0, 70.0, 2, WHITE);
    let mut d = HDG_DEG - 70;
    while d <= HDG_DEG + 70 {
        if d % 5 == 0 {
            let rel = (d - HDG_DEG) as f32 * PI / 180.0;
            let long = d % 10 == 0;
            let len = if long { 12 } else { 7 };
            let (sx, sy) = (cx as f32 + r as f32 * rel.sin(), cy as f32 - r as f32 * rel.cos());
            let (ex, ey) = (
                cx as f32 + (r - len) as f32 * rel.sin(),
                cy as f32 - (r - len) as f32 * rel.cos(),
            );
            c.line(sx as i32, sy as i32, ex as i32, ey as i32, 2, WHITE);

            if d % 30 == 0 {
                let n = ((d % 360) + 360) % 360;
                let label = match n {
                    0 => "N".to_string(),
                    90 => "E".to_string(),
                    180 => "S".to_string(),
                    270 => "W".to_string(),
                    _ => format!("{:02}", n / 10),
                };
                let lr = (r - len) as f32 - 6.0 * s_rose as f32;
                let lx = cx as f32 + lr * rel.sin();
                let ly = cy as f32 - lr * rel.cos();
                c.text(lx as i32, ly as i32 - 3 * s_rose, &label, s_rose, WHITE, Anchor::Center);
            }
        }
        d += 1;
    }

    // Track: straight up, since the static heading and track agree.
    c.dashed_line(cx, cy - 6, cx, cy - r + 4, 2, MAGENTA, 12, 8);

    // Heading bug at the top of the arc.
    c.tri([(cx, cy - r - 2), (cx - 7, cy - r - 13), (cx + 7, cy - r - 13)], MAGENTA);

    // Aircraft symbol.
    c.tri([(cx, cy - 14), (cx - 9, cy + 6), (cx + 9, cy + 6)], WHITE);
    c.rect(cx - 1, cy - 14, 3, 22, WHITE);

    // Corner data blocks. Laid out in explicit texels rather than multiples of the text
    // size: the two left labels and the two right ones have to clear each other across the
    // full width, and tying that to the glyph scale is how they ended up overlapping.
    let row0 = margin;
    let row1 = margin + 10 * s;
    c.text(margin, row0, "GS", s, GREY, Anchor::Left);
    c.text(margin + 18 * s, row0, &GS_KT.to_string(), s, WHITE, Anchor::Left);
    c.text(margin, row1, "HDG", s, GREY, Anchor::Left);
    c.text(margin + 26 * s, row1, &format!("{:03}", HDG_DEG), s, GREEN, Anchor::Left);
    c.text(w - margin, row0, &format!("{} NM", ND_RANGE_NM), s, CYAN, Anchor::Right);
    c.text(w - margin, row1, "VOR/ARC", s, CYAN, Anchor::Right);
}

// ── Drawing ───────────────────────────────────────────────────────────────────

#[derive(Clone, Copy, PartialEq)]
enum Anchor {
    Left,
    Center,
    Right,
}

struct Canvas {
    px: Vec<[u8; 3]>,
    w: i32,
    h: i32,
}

impl Canvas {
    fn new(w: i32, h: i32, fill: [u8; 3]) -> Self {
        Self { px: vec![fill; (w * h) as usize], w, h }
    }

    fn put(&mut self, x: i32, y: i32, color: [u8; 3]) {
        if x >= 0 && y >= 0 && x < self.w && y < self.h {
            self.px[(y * self.w + x) as usize] = color;
        }
    }

    fn get(&self, x: i32, y: i32) -> [u8; 3] {
        self.px[(y.clamp(0, self.h - 1) * self.w + x.clamp(0, self.w - 1)) as usize]
    }

    fn rect(&mut self, x: i32, y: i32, w: i32, h: i32, color: [u8; 3]) {
        for yy in y..y + h {
            for xx in x..x + w {
                self.put(xx, yy, color);
            }
        }
    }

    fn outline(&mut self, x: i32, y: i32, w: i32, h: i32, t: i32, color: [u8; 3]) {
        self.rect(x, y, w, t, color);
        self.rect(x, y + h - t, w, t, color);
        self.rect(x, y, t, h, color);
        self.rect(x + w - t, y, t, h, color);
    }

    /// A thick line, stepped along whichever axis it covers more of.
    fn line(&mut self, x0: i32, y0: i32, x1: i32, y1: i32, t: i32, color: [u8; 3]) {
        let steps = (x1 - x0).abs().max((y1 - y0).abs()).max(1);
        for i in 0..=steps {
            let f = i as f32 / steps as f32;
            let x = x0 as f32 + (x1 - x0) as f32 * f;
            let y = y0 as f32 + (y1 - y0) as f32 * f;
            self.rect(x as i32 - t / 2, y as i32 - t / 2, t, t, color);
        }
    }

    fn dashed_line(&mut self, x0: i32, y0: i32, x1: i32, y1: i32, t: i32, color: [u8; 3], on: i32, off: i32) {
        let len = ((x1 - x0).pow(2) as f32 + (y1 - y0).pow(2) as f32).sqrt();
        let mut travelled = 0.0;
        while travelled < len {
            let a = travelled / len;
            let b = ((travelled + on as f32) / len).min(1.0);
            let lerp = |p0: i32, p1: i32, f: f32| p0 as f32 + (p1 - p0) as f32 * f;
            self.line(
                lerp(x0, x1, a) as i32,
                lerp(y0, y1, a) as i32,
                lerp(x0, x1, b) as i32,
                lerp(y0, y1, b) as i32,
                t,
                color,
            );
            travelled += (on + off) as f32;
        }
    }

    /// An arc about `(cx, cy)`, angles in degrees clockwise from straight up.
    fn arc(&mut self, cx: i32, cy: i32, r: i32, a0: f32, a1: f32, t: i32, color: [u8; 3]) {
        let steps = ((a1 - a0).abs() * r as f32 / 40.0).max(24.0) as i32;
        for i in 0..=steps {
            let a = (a0 + (a1 - a0) * i as f32 / steps as f32) * PI / 180.0;
            let x = cx as f32 + r as f32 * a.sin();
            let y = cy as f32 - r as f32 * a.cos();
            self.rect(x as i32 - t / 2, y as i32 - t / 2, t, t, color);
        }
    }

    fn dashed_arc(&mut self, cx: i32, cy: i32, r: i32, a0: f32, a1: f32, t: i32, color: [u8; 3], on: i32, off: i32) {
        // Dash lengths given in texels along the arc, converted to degrees for this radius.
        let per_deg = r as f32 * PI / 180.0;
        let (on_deg, off_deg) = (on as f32 / per_deg, off as f32 / per_deg);
        let mut a = a0;
        while a < a1 {
            self.arc(cx, cy, r, a, (a + on_deg).min(a1), t, color);
            a += on_deg + off_deg;
        }
    }

    fn tri(&mut self, p: [(i32, i32); 3], color: [u8; 3]) {
        let min_y = p.iter().map(|q| q.1).min().unwrap();
        let max_y = p.iter().map(|q| q.1).max().unwrap();
        let min_x = p.iter().map(|q| q.0).min().unwrap();
        let max_x = p.iter().map(|q| q.0).max().unwrap();
        let edge = |a: (i32, i32), b: (i32, i32), x: i32, y: i32| {
            (b.0 - a.0) * (y - a.1) - (b.1 - a.1) * (x - a.0)
        };
        for y in min_y..=max_y {
            for x in min_x..=max_x {
                let e0 = edge(p[0], p[1], x, y);
                let e1 = edge(p[1], p[2], x, y);
                let e2 = edge(p[2], p[0], x, y);
                if (e0 >= 0 && e1 >= 0 && e2 >= 0) || (e0 <= 0 && e1 <= 0 && e2 <= 0) {
                    self.put(x, y, color);
                }
            }
        }
    }

    fn text(&mut self, x: i32, y: i32, s: &str, scale: i32, color: [u8; 3], anchor: Anchor) {
        let advance = (GLYPH_W + 1) * scale;
        let width = s.chars().count() as i32 * advance - scale;
        let mut cursor = match anchor {
            Anchor::Left => x,
            Anchor::Center => x - width / 2,
            Anchor::Right => x - width,
        };
        for ch in s.chars() {
            self.glyph(cursor, y, ch, scale, color);
            cursor += advance;
        }
    }

    fn glyph(&mut self, x: i32, y: i32, ch: char, scale: i32, color: [u8; 3]) {
        let rows = match glyph_rows(ch) {
            Some(rows) => rows,
            None => return,
        };
        for (ry, bits) in rows.iter().enumerate() {
            for rx in 0..GLYPH_W {
                if bits & (1 << (GLYPH_W - 1 - rx)) != 0 {
                    self.rect(x + rx * scale, y + ry as i32 * scale, scale, scale, color);
                }
            }
        }
    }

    fn blit(&mut self, src: &Canvas, x: i32, y: i32) {
        for sy in 0..src.h {
            for sx in 0..src.w {
                self.put(x + sx, y + sy, src.px[(sy * src.w + sx) as usize]);
            }
        }
    }

    /// Smears the border of the rectangle `pad` texels outwards, so a minified mip level
    /// averages a screen against more of itself instead of against its neighbours.
    fn extend_edges(&mut self, x: i32, y: i32, w: i32, h: i32, pad: i32) {
        for d in 1..=pad {
            for sx in (x - d)..(x + w + d) {
                let top = self.get(sx.clamp(x, x + w - 1), y);
                let bottom = self.get(sx.clamp(x, x + w - 1), y + h - 1);
                self.put(sx, y - d, top);
                self.put(sx, y + h - 1 + d, bottom);
            }
            for sy in (y - d)..(y + h + d) {
                let left = self.get(x, sy.clamp(y, y + h - 1));
                let right = self.get(x + w - 1, sy.clamp(y, y + h - 1));
                self.put(x - d, sy, left);
                self.put(x + w - 1 + d, sy, right);
            }
        }
    }

    fn into_rgba(self) -> Vec<u8> {
        let mut out = Vec::with_capacity(self.px.len() * 4);
        for p in self.px {
            out.extend_from_slice(&[p[0], p[1], p[2], 255]);
        }
        out
    }
}

// ── A 5x7 bitmap font ─────────────────────────────────────────────────────────
// Only what the panels spell. Each row is five bits, most significant on the left.

const GLYPH_W: i32 = 5;

fn glyph_rows(ch: char) -> Option<[u8; 7]> {
    let rows = match ch.to_ascii_uppercase() {
        '0' => [0b01110, 0b10001, 0b10011, 0b10101, 0b11001, 0b10001, 0b01110],
        '1' => [0b00100, 0b01100, 0b00100, 0b00100, 0b00100, 0b00100, 0b01110],
        '2' => [0b01110, 0b10001, 0b00001, 0b00010, 0b00100, 0b01000, 0b11111],
        '3' => [0b11111, 0b00010, 0b00100, 0b00010, 0b00001, 0b10001, 0b01110],
        '4' => [0b00010, 0b00110, 0b01010, 0b10010, 0b11111, 0b00010, 0b00010],
        '5' => [0b11111, 0b10000, 0b11110, 0b00001, 0b00001, 0b10001, 0b01110],
        '6' => [0b00110, 0b01000, 0b10000, 0b11110, 0b10001, 0b10001, 0b01110],
        '7' => [0b11111, 0b00001, 0b00010, 0b00100, 0b01000, 0b01000, 0b01000],
        '8' => [0b01110, 0b10001, 0b10001, 0b01110, 0b10001, 0b10001, 0b01110],
        '9' => [0b01110, 0b10001, 0b10001, 0b01111, 0b00001, 0b00010, 0b01100],
        'A' => [0b01110, 0b10001, 0b10001, 0b11111, 0b10001, 0b10001, 0b10001],
        'C' => [0b01110, 0b10001, 0b10000, 0b10000, 0b10000, 0b10001, 0b01110],
        'D' => [0b11110, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b11110],
        'E' => [0b11111, 0b10000, 0b10000, 0b11110, 0b10000, 0b10000, 0b11111],
        'G' => [0b01110, 0b10001, 0b10000, 0b10111, 0b10001, 0b10001, 0b01111],
        'H' => [0b10001, 0b10001, 0b10001, 0b11111, 0b10001, 0b10001, 0b10001],
        'L' => [0b10000, 0b10000, 0b10000, 0b10000, 0b10000, 0b10000, 0b11111],
        'M' => [0b10001, 0b11011, 0b10101, 0b10101, 0b10001, 0b10001, 0b10001],
        'N' => [0b10001, 0b11001, 0b10101, 0b10011, 0b10001, 0b10001, 0b10001],
        'O' => [0b01110, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b01110],
        'R' => [0b11110, 0b10001, 0b10001, 0b11110, 0b10100, 0b10010, 0b10001],
        'S' => [0b01111, 0b10000, 0b10000, 0b01110, 0b00001, 0b00001, 0b11110],
        'T' => [0b11111, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100],
        'V' => [0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b01010, 0b00100],
        'W' => [0b10001, 0b10001, 0b10001, 0b10101, 0b10101, 0b11011, 0b10001],
        '/' => [0b00001, 0b00010, 0b00010, 0b00100, 0b01000, 0b01000, 0b10000],
        '-' => [0b00000, 0b00000, 0b00000, 0b11111, 0b00000, 0b00000, 0b00000],
        ' ' => [0; 7],
        _ => return None,
    };
    Some(rows)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every character the panels actually spell has to exist in the font, or it silently
    /// renders as a gap.
    #[test]
    fn font_covers_every_label() {
        let mut used = String::from("LNAVVNAVCMDGSHDGNMVOR/ARCNESW0123456789");
        used.push_str(&IAS_KT.to_string());
        used.push_str(&ALT_FT.to_string());
        used.push_str(&GS_KT.to_string());
        for ch in used.chars() {
            assert!(glyph_rows(ch).is_some(), "no glyph for {ch:?}");
        }
    }

    /// The parked texels must sit in gaps no display primitive addresses, and clear of the
    /// padding smeared out from the bands either side of them.
    #[test]
    fn parked_texels_are_clear_of_every_band() {
        for texel in [PARK_WHITE_TEXEL, PARK_GLASS_TEXEL] {
            let u = (texel.0 as f32 + 0.5) / ATLAS_W as f32;
            let v = (texel.1 as f32 + 0.5) / ATLAS_H as f32;
            for b in &BANDS {
                let clear_u = u < b.u0 || u > b.u1;
                let clear_v = v < b.v0 || v > b.v1;
                assert!(clear_u || clear_v, "parked texel {texel:?} lands inside a band");
                if !clear_v {
                    // Sharing rows with the bands, so it must clear their padding too.
                    let pad_u = (PAD + PARK_BLOCK) as f32 / ATLAS_W as f32;
                    assert!(
                        u < b.u0 - pad_u || u > b.u1 + pad_u,
                        "parked texel {texel:?} is inside the padding of a band"
                    );
                }
            }
        }
    }

    /// Everything that is not a screen has to come back as one of the two parked
    /// coordinates, or it reads the atlas and shows a slice of a flight display.
    #[test]
    fn only_display_quads_keep_their_uvs() {
        let park_white = uv_for_material(Some("pedestal_01"), [0.4, 0.6]);
        assert_eq!(uv_for_material(Some("Side_Display"), [0.9, 0.2]), park_white);
        assert_eq!(uv_for_material(None, [0.1, 0.1]), park_white);

        // A UV inside a band, on the screen material, is the one case that passes through.
        let inside = [0.15, 0.5];
        assert_eq!(uv_for_material(Some(SCREEN_MATERIAL), inside), inside);

        // The same material in the gap between bands is bezel, and gets parked instead.
        let gap = [0.50, 0.5];
        assert_ne!(uv_for_material(Some(SCREEN_MATERIAL), gap), gap);
        assert_ne!(uv_for_material(Some(SCREEN_MATERIAL), gap), park_white);
    }

    /// Writes the atlas out so the artwork can be looked at without launching the app.
    /// `COCKPIT_ATLAS_PPM=/path/atlas.ppm cargo test -p cesium-flight dump_atlas -- --ignored`
    #[test]
    #[ignore = "writes a file; run explicitly"]
    fn dump_atlas() {
        let path = std::env::var("COCKPIT_ATLAS_PPM").unwrap_or_else(|_| "atlas.ppm".into());
        let (w, h, rgba) = build_atlas();
        let mut out = format!("P6\n{w} {h}\n255\n").into_bytes();
        for px in rgba.chunks(4) {
            out.extend_from_slice(&px[..3]);
        }
        std::fs::write(&path, out).unwrap();
        println!("wrote {path} ({w}x{h})");
    }

    /// The exact corner UVs of the four display quads, read out of the GLB. All eight
    /// corners of each have to survive [`uv_for_material`] untouched: park even one of
    /// them and that screen interpolates from the parked texel to the opposite corner,
    /// which draws the whole atlas across it.
    const QUAD_CORNERS: [[f32; 2]; 8] = [
        [0.050_939_33, 0.082_822_20],
        [0.239_630_80, 0.930_173_70],
        [0.263_536_01, 0.082_822_20],
        [0.452_227_41, 0.930_173_70],
        [0.543_178_98, 0.082_822_20],
        [0.731_870_41, 0.930_173_70],
        [0.755_775_63, 0.082_822_20],
        [0.944_467_13, 0.930_173_70],
    ];

    #[test]
    fn every_display_corner_survives_untouched() {
        for uv in QUAD_CORNERS {
            assert_eq!(
                uv_for_material(Some(SCREEN_MATERIAL), uv),
                uv,
                "corner {uv:?} was parked; that screen will show the whole atlas"
            );
        }
    }

    /// The slack must not be so wide that a band swallows its neighbour's content or a
    /// bezel primitive, which would put screen imagery where it does not belong.
    #[test]
    fn band_slack_cannot_reach_anything_it_should_not() {
        // Every gap between two adjacent bands has to be wider than twice the slack.
        for pair in BANDS.windows(2) {
            let gap = pair[1].u0 - pair[0].u1;
            assert!(
                gap > 2.0 * BAND_EPSILON,
                "bands {:.4} and {:.4} are only {gap:.4} apart",
                pair[0].u1,
                pair[1].u0
            );
        }

        // The `Main_Display` primitives that are bezel rather than screen, at their
        // closest approach to a band edge. Read out of the GLB alongside the corners.
        for u in [0.036_071_f32, 0.468_239, 0.529_788, 0.963_639] {
            let uv = [u, 0.5];
            assert_ne!(
                uv_for_material(Some(SCREEN_MATERIAL), uv),
                uv,
                "bezel UV {u} was let through as screen"
            );
        }
    }

    #[test]
    fn atlas_is_the_declared_size() {
        let (w, h, px) = build_atlas();
        assert_eq!((w, h), (ATLAS_W, ATLAS_H));
        assert_eq!(px.len(), (ATLAS_W * ATLAS_H * 4) as usize);

        // The white park texel has to be white, or every non-screen material in the
        // flight deck is tinted by whatever landed there.
        let i = ((PARK_WHITE_TEXEL.1 * ATLAS_W + PARK_WHITE_TEXEL.0) * 4) as usize;
        assert_eq!(&px[i..i + 3], &[255, 255, 255]);
    }
}
