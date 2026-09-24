// ── Physically based atmosphere ──────────────────────────────────────────────
//
// Shared by the sky (sky_pipeline/sky.wgsl) and the globe (globe_pipeline/shader.wgsl):
// both shader modules are built by prepending this file to their own source (the
// `concat!` in `render/wgpu_state.rs`), so the sky at the horizon and the haze over distant ground
// come out of the same function and cannot drift apart. See docs/lighting.md.
//
// Single scattering by air (Rayleigh) and aerosol (Mie), with ozone absorption, along a
// short non-uniform raymarch. The light reaching each sample is attenuated along its own
// path to the sun analytically — the Chapman function, the exact column of an
// exponential atmosphere over a sphere — so there is no inner loop and no lookup table.
// Everything a sunset is made of falls out of this without being painted on:
//
// - the sun and the light near it redden, because the sunlight crosses ~35 air masses;
// - the horizon toward the sun glows yellow-orange-red (forward Mie + long paths);
// - opposite the sun, the planet's own shadow is cast into the air: the dark blue-grey
//   Earth's-shadow band rises from the antisolar horizon after sunset, with the pink
//   Belt of Venus (backscattered reddened sunlight) sitting on top of it;
// - the zenith stays blue, then turns deep blue/violet in twilight, because the
//   sunlight lighting it crossed the ozone layer at grazing incidence and lost its
//   orange (Chappuis band) — without ozone a twilight zenith comes out grey.
//
// Multiple scattering is approximated by an isotropic term lit by the sunlight a little
// above each sample, which is what keeps the Earth's shadow blue-grey instead of black.
//
// Units in here are kilometres; world space is megametres, converted at the boundary.

const ATMO_PI: f32 = 3.14159265;
/// Mean planet radius and the top of the modelled air, km. The sky is spherical; the
/// camera's height above the WGS84 ellipsoid is what places it in this sphere.
const ATMO_R: f32 = 6371.0;
const ATMO_TOP: f32 = 6471.0;
/// Scale heights, km.
const ATMO_HR: f32 = 8.0;
const ATMO_HM: f32 = 1.2;
/// Sea-level coefficients, per km, for 680/550/440 nm (Bruneton 2017 / Hillaire 2020).
const ATMO_BETA_R: vec3<f32> = vec3<f32>(5.802e-3, 13.558e-3, 33.1e-3);
const ATMO_BETA_M_SCA: f32 = 3.996e-3;
const ATMO_BETA_M_EXT: f32 = 4.440e-3;
/// Ozone absorption at the peak of its layer, per km, and the layer's shape: a tent
/// centred at 25km, 15km either side.
///
/// Not the textbook 680/550/440nm values (0.65, 1.88, 0.085)e-3. A display's red
/// primary sits much nearer the Chappuis band's 600nm peak than 680nm does, so the red
/// channel is raised to 1.6e-3; and the whole set is doubled, standing in for the
/// aerosol extinction and higher-order scattering that single scattering leaves out.
/// Both were tuned against the headless sunset sweep (src/testing/rendering/
/// sunset_capture.rs): with the textbook values the twilight zenith came out
/// mauve-brown, where a real one is deep blue.
const ATMO_BETA_O3: vec3<f32> = vec3<f32>(3.2e-3, 3.76e-3, 0.17e-3);
const ATMO_O3_CENTER: f32 = 25.0;
const ATMO_O3_HALF: f32 = 15.0;
/// Aerosol forward-scattering asymmetry. 0.8 is a clear continental day; it is what
/// makes the bright aureole hugging the sun.
const ATMO_MIE_G: f32 = 0.80;
/// Strength of the multiple-scattering stand-in, and how far above each sample it looks
/// for the sunlight feeding it (km).
const ATMO_MS_STRENGTH: f32 = 4.0;
const ATMO_MS_LIFT: f32 = 8.0;
/// Lowest height (km) the sun's ray may skim for the light feeding multiple scattering.
const ATMO_MS_TANGENT: f32 = 25.0;
/// Share of the multiple-scattering light a sample gets even when the sun itself does
/// not reach it. The rest follows the sample's own sunlight: low, long horizon paths at
/// sunset are lit by a dim red sun, and letting the (bluish) skylight term swamp them
/// is what washed the sunset band out to lavender.
const ATMO_MS_FLOOR: f32 = 0.15;
/// Solar irradiance in display units before exposure.
const ATMO_SUN_E: f32 = 24.0;
/// Radiance of the sun's disc relative to ATMO_SUN_E; the tone curve clips it white at
/// noon and leaves it a coloured disc once the air has reddened and dimmed it.
const ATMO_SUN_DISC: f32 = 60.0;

const ATMO_VIEW_STEPS: i32 = 16;

fn atmo_erfcx(z: f32) -> f32 {
    // Scaled complementary error function, Numerical Recipes' erfcc fit (see the globe
    // shader's `erfcx` for the derivation); negative z is the analytic continuation.
    let a = abs(z);
    let t = 1.0 / (1.0 + 0.5 * a);
    let poly = -1.26551223 + t * (1.00002368 + t * (0.37409196 + t * (0.09678418
             + t * (-0.18628806 + t * (0.27886807 + t * (-1.13520398 + t * (1.48851587
             + t * (-0.82215223 + t * 0.17087277))))))));
    let scaled = t * exp(poly);
    if (z >= 0.0) {
        return scaled;
    }
    return 2.0 * exp(min(z * z, 60.0)) - scaled;
}

/// Air columns (in scale heights) from radius `r` along a ray of cos-zenith `mu` to space.
fn atmo_chapman(x: f32, mu: f32) -> f32 {
    return sqrt(0.5 * ATMO_PI * x) * atmo_erfcx(sqrt(0.5 * x) * mu);
}

/// Length of the ray from radius `r` with cos-zenith `mu` that lies inside a sphere of
/// radius `radius`.
fn atmo_chord(r: f32, mu: f32, radius: f32) -> f32 {
    let disc = r * r * (mu * mu - 1.0) + radius * radius;
    if (disc <= 0.0) {
        return 0.0;
    }
    let s = sqrt(disc);
    if (r < radius) {
        return -r * mu + s;
    }
    if (mu >= 0.0) {
        return 0.0;
    }
    return 2.0 * s;
}

/// Transmittance from a point at radius `r` (km) toward the sun at cos-zenith `mu`,
/// including the planet's shadow.
fn atmo_sun_transmittance(r: f32, mu: f32) -> vec3<f32> {
    let h = max(r - ATMO_R, 0.0);
    let rr = ATMO_R + h;
    // The ray to the sun is blocked by the ground: the Earth's shadow.
    // Lowest point of the ray: the start itself when it climbs, the tangent when it dips.
    var r_low = rr;
    if (mu < 0.0) {
        r_low = rr * sqrt(max(1.0 - mu * mu, 0.0));
    }
    let lit = smoothstep(ATMO_R - 1.0, ATMO_R, r_low);
    if (lit <= 0.0) {
        return vec3<f32>(0.0);
    }
    let col_r = ATMO_HR * exp(-h / ATMO_HR) * atmo_chapman(rr / ATMO_HR, mu);
    let col_m = ATMO_HM * exp(-h / ATMO_HM) * atmo_chapman(rr / ATMO_HM, mu);
    // Ozone: the exact length of the sun ray inside the layer (10-40km), at the tent's
    // mean density. This path is what turns the twilight zenith blue — the light that
    // still reaches the upper air after sunset has crossed the layer almost edge-on,
    // hundreds of kilometres of it, and the Chappuis band has taken its orange out.
    let col_o = 0.5 * (atmo_chord(rr, mu, ATMO_R + ATMO_O3_CENTER + ATMO_O3_HALF)
        - atmo_chord(rr, mu, ATMO_R + ATMO_O3_CENTER - ATMO_O3_HALF));
    let tau = ATMO_BETA_R * col_r + vec3<f32>(ATMO_BETA_M_EXT * col_m) + ATMO_BETA_O3 * col_o;
    return exp(-min(tau, vec3<f32>(80.0))) * lit;
}

/// Sunlight feeding the multiple-scattering term at a sample: what reaches the air some
/// way above it. After sunset that has to come from high enough that the sun's ray
/// skims the planet no lower than ATMO_MS_TANGENT — the blue, ozone-filtered light of
/// the upper twilight sky, which is what actually lights the air inside the Earth's
/// shadow — and it is weighted by how little air there is up there to scatter it.
fn atmo_ms_source(r: f32, mu: f32) -> vec3<f32> {
    var r_lift = r + ATMO_MS_LIFT;
    let mu_down = min(mu, 0.0);
    r_lift = max(r_lift, (ATMO_R + ATMO_MS_TANGENT) / sqrt(max(1.0 - mu_down * mu_down, 1e-4)));
    let above = max(r_lift - r - ATMO_MS_LIFT, 0.0);
    return atmo_sun_transmittance(r_lift, mu) * exp(-above / ATMO_HR);
}

fn atmo_ray_sphere(o: vec3<f32>, d: vec3<f32>, radius: f32) -> vec2<f32> {
    let b = dot(o, d);
    let c = dot(o, o) - radius * radius;
    let disc = b * b - c;
    if (disc < 0.0) {
        return vec2<f32>(-1.0, -1.0);
    }
    let s = sqrt(disc);
    return vec2<f32>(-b - s, -b + s);
}

/// Where a world-space point (Mm) sits in the atmosphere's sphere (km): along its
/// ellipsoid normal, at its height above the WGS84 ellipsoid.
fn atmo_position(p_mm: vec3<f32>) -> vec3<f32> {
    let a = 6.378137;
    let b = 6.3567523142;
    let inv_a2 = 1.0 / (a * a);
    let inv_b2 = 1.0 / (b * b);
    let r = length(p_mm);
    let dir = p_mm / max(r, 1e-6);
    let surface = 1.0 / sqrt(dir.x * dir.x * inv_a2 + dir.y * dir.y * inv_b2 + dir.z * dir.z * inv_a2);
    let up = normalize(vec3<f32>(p_mm.x * inv_a2, p_mm.y * inv_b2, p_mm.z * inv_a2));
    let h_km = (r - surface) * 1000.0;
    return up * (ATMO_R + max(h_km, 0.002));
}

/// Light scattered toward the eye along a ray, split so the phase functions (which vary
/// fast near the sun) can be applied late — per pixel even when this ran per vertex.
struct AtmoScatter {
    rayleigh: vec3<f32>,
    mie: vec3<f32>,
    /// Isotropic multiple-scattering stand-in, phase already applied.
    ms: vec3<f32>,
    /// Transmittance along the ray from the origin to its end.
    transmittance: vec3<f32>,
};

fn atmo_integrate(origin: vec3<f32>, dir: vec3<f32>, sun_dir: vec3<f32>, t_limit: f32, steps: i32) -> AtmoScatter {
    var out: AtmoScatter;
    out.rayleigh = vec3<f32>(0.0);
    out.mie = vec3<f32>(0.0);
    out.ms = vec3<f32>(0.0);
    out.transmittance = vec3<f32>(1.0);

    let top = atmo_ray_sphere(origin, dir, ATMO_TOP);
    if (top.y <= 0.0) {
        return out;
    }
    let t0 = max(top.x, 0.0);
    var t1 = top.y;
    let ground = atmo_ray_sphere(origin, dir, ATMO_R);
    if (ground.x > 0.0) {
        t1 = min(t1, ground.x);
    }
    t1 = min(t1, t0 + t_limit);
    let len = t1 - t0;
    if (len <= 0.0) {
        return out;
    }

    var tau = vec3<f32>(0.0);
    let n = f32(steps);
    for (var i = 0; i < steps; i = i + 1) {
        // Quadratic spacing: fine near the eye where the air is dense, coarse far out.
        let a = f32(i) / n;
        let b = f32(i + 1) / n;
        let dt = len * (b * b - a * a);
        let t = t0 + len * 0.5 * (a * a + b * b);
        let p = origin + dir * t;
        let r = length(p);
        let h = max(r - ATMO_R, 0.0);
        let mu_s = dot(p / r, sun_dir);

        let d_r = exp(-h / ATMO_HR);
        let d_m = exp(-h / ATMO_HM);
        let d_o = max(0.0, 1.0 - abs(h - ATMO_O3_CENTER) / ATMO_O3_HALF);
        let ext = ATMO_BETA_R * d_r + vec3<f32>(ATMO_BETA_M_EXT * d_m) + ATMO_BETA_O3 * d_o;

        let t_view = exp(-(tau + ext * (0.5 * dt)));
        let t_sun = atmo_sun_transmittance(r, mu_s);
        out.rayleigh += (d_r * dt) * t_view * t_sun;
        out.mie += (d_m * dt) * t_view * t_sun;

        let local = min(dot(t_sun, vec3<f32>(0.3, 0.6, 0.1)), 1.0);
        let ms_light = atmo_ms_source(r, mu_s) * mix(ATMO_MS_FLOOR, 1.0, local);
        out.ms += (ATMO_BETA_R * d_r + vec3<f32>(ATMO_BETA_M_SCA * d_m)) * (dt * t_view) * ms_light;

        tau += ext * dt;
    }
    out.rayleigh *= ATMO_BETA_R;
    out.mie *= ATMO_BETA_M_SCA;
    out.ms *= ATMO_MS_STRENGTH / (4.0 * ATMO_PI);
    out.transmittance = exp(-tau);
    return out;
}

fn atmo_phase_rayleigh(c: f32) -> f32 {
    return 3.0 / (16.0 * ATMO_PI) * (1.0 + c * c);
}

/// Cornette-Shanks, the physically better-behaved Henyey-Greenstein.
fn atmo_phase_mie(c: f32) -> f32 {
    let g = ATMO_MIE_G;
    let g2 = g * g;
    let k = 3.0 / (8.0 * ATMO_PI) * (1.0 - g2) / (2.0 + g2);
    return k * (1.0 + c * c) / pow(max(1.0 + g2 - 2.0 * g * c, 1e-4), 1.5);
}

/// Radiance (pre-exposure) for a scatter result seen at angle cos `c` from the sun.
fn atmo_radiance(s: AtmoScatter, c: f32) -> vec3<f32> {
    return ATMO_SUN_E * (s.rayleigh * atmo_phase_rayleigh(c) + s.mie * atmo_phase_mie(c) + s.ms);
}

/// Exposure: the eye opens up as the light goes, but not all the way — the scene still
/// has to get darker through twilight into night. `sun_elevation` is sin(elevation).
fn atmo_exposure(sun_elevation: f32) -> f32 {
    // In stops: none with the sun up, ~5 by the end of civil twilight, 7 at night.
    return exp2(7.0 * smoothstep(0.04, -0.18, sun_elevation));
}

/// Saturation applied after the tone curve. The per-channel shoulder desaturates
/// everything bright, which left the daytime sky a pale cyan next to the rich blue the
/// old painted palette had; this puts the colour back, uniformly, without touching hue.
const ATMO_DISPLAY_SATURATION: f32 = 1.3;

/// Radiance to display colour: a per-channel soft shoulder, which rolls the sun and its
/// aureole off to white instead of clipping them into flat discs.
fn atmo_tonemap(radiance: vec3<f32>, sun_elevation: f32) -> vec3<f32> {
    let c = vec3<f32>(1.0) - exp(-radiance * atmo_exposure(sun_elevation));
    let lum = dot(c, vec3<f32>(0.2126, 0.7152, 0.0722));
    return clamp(vec3<f32>(lum) + (c - vec3<f32>(lum)) * ATMO_DISPLAY_SATURATION,
        vec3<f32>(0.0), vec3<f32>(1.0));
}

