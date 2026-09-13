#![allow(clippy::type_complexity)]
//! The tile quadtree and its per-node visibility decision.
//!
//! Derivation: `docs/culling-math.md` §9 is the consolidated algorithm this file
//! implements, §13 the recalibration that produced its present shape. Per node, in
//! order — the first three as a [`CullPipeline`] of switchable [`Stage`]s, the
//! fourth deliberately outside it:
//!
//! 1. [`Stage::Horizon`] — exact, f64, ~25 flops ([`super::horizon`]). Cheapest
//!    *and* most selective: roughly half the globe is below the limb at any time,
//!    and the whole back hemisphere falls to one comparison at the coarsest level.
//! 2. [`Stage::NodeFrustum`] — the four side planes, camera-relative, ~92 flops
//!    ([`super::bounding_volume`]), and where they leave the answer open, the
//!    separating axes that close it ([`super::slab`]).
//! 3. [`Stage::SubPatchGrid`] ([`SubGrid`]) — both tests again, per sub-patch of a
//!    `k × k` subdivision of the tile, with `k` from [`SUB_BOXES_PER_AXIS`].
//! 4. LOD / hysteresis ([`QuadtreeNode::apply_lod`]), unchanged. Not a stage: it is
//!    not a visibility question, and it needs `&mut self` where a stage gets
//!    `&self` — which is what makes it *structurally* impossible for a culling
//!    stage to delete a subtree (see I-7 below).
//!
//! Deliberately absent, and deleted rather than repaired:
//!
//! * `compute_horizon_culling_point` and the spherical-cap reduction. It was *not*
//!   the limb bug — it is provably sound (§3.6) — but it is loose, costing up to
//!   23 % of the occluded tiles at coarse zoom, and the closed form replaces it at
//!   lower cost.
//! * The **sub-OBB back-face heuristic**, which was the dominant false-negative
//!   source. `normal·(cam − centre) > −max_extent` uses a margin short by a factor
//!   `h = √(C²−1)`, making it unsound above 2 642 km and culling visible tiles up
//!   to 8.07° inside the limb at 12 000 km (§4.1). Back-face culling is not merely
//!   fixed here, it is *subsumed*: for a surface point `n̂(p)·(cam − p)` and
//!   `q·c − 1` are the same expression up to a positive factor (Theorem 3.4). What
//!   that heuristic was *also* doing — testing occlusion per sub-box rather than per
//!   tile — is not lost: [`SubGrid`] does it exactly, on the sub-patch itself.
//! * The near and far planes, and the `vh_mag_sq > −0.1` sub-surface band.
//! * The flat 8×8 sub-OBB grid for `5 ≤ z ≤ 16`. A grid is kept, but tapered by
//!   measurement rather than fixed by assertion — see [`SUB_BOXES_PER_AXIS`].
//!
//! # Invariant I-7 — soundness at every level
//!
//! [`QuadtreeNode::update`] sets `children = None` on a cull and
//! `collect_visible_tiles` emits leaves only, so a wrong cull at *any* ancestor
//! deletes an entire subtree. Every test above is exact (§3) or provably
//! conservative (§2.8), at every level, so soundness follows by induction over the
//! tree. Do not add a test here that is only "sound at leaf granularity".
//!
//! Two things now hold the line rather than this paragraph alone. A [`Stage`] takes
//! `&QuadtreeNode`, so it *cannot* reach `children` — only `apply_lod` can. And
//! [`CullPipeline`]'s final rule keeps, which makes every stage a pure subtraction
//! from the kept set: dropping stages can only keep more, never less, and
//! `test_stage_prefix_only_grows_the_kept_set` asserts exactly that over every
//! pipeline and every prefix of one.

use glam::{DVec3, Vec3};

use super::bounding_volume::{Frustum, OrientedBoundingBox, PlaneVerdict};
use super::fog::{cesium_fog, MEGAMETERS_TO_METERS};
use super::horizon::{HorizonCamera, TilePatch};
use super::tile_id::{
    tile_bounds, tile_bounds_unstretched, web_mercator_y_to_lat_f64, TileBounds, TileId, MAX_ZOOM,
};
use crate::globe::geometry::{lon_lat_to_ecef_f64, EARTH_RADIUS_A_F64, EARTH_RADIUS_B_F64};

/// Sub-boxes per axis, by zoom — the calibrated subdivision rule.
///
/// Index by zoom level, last entry repeated: `k = 1` means "no grid, the node's own
/// box takes the exact test". Read [`SubGrid`] first for what a sub-patch buys.
///
/// # Why a table and not a formula
///
/// `docs/culling-math.md` §7.2 derived `k(z) = ceil(θ_max(z)/θ*)` from the box's
/// **sagitta** and concluded sub-boxes are waste above z = 4. §12 recorded that the
/// conclusion was wrong, because the sagitta is not what sub-boxes buy — they
/// attack §5.2's corner over-report, which does not decay with zoom. The first fix
/// for that was a flat floor, `k ≥ 4` at every zoom, calibrated against
/// `fuzz_sweep` alone; it improved `fuzz_sweep` and `near_ground_high_zoom` and
/// made **five** other sweeps worse than the code it replaced.
///
/// Neither shape was right, because neither quantity is what the taper tracks. The
/// corner over-report is now *solved*, exactly, by
/// [`super::slab::separated_on_edge_cross_axes`], and what the sub-boxes are left
/// buying is the gap between a patch and the single box around it — worth a lot at
/// z = 1..6, where a tile still spans degrees, and very little below that. Cost
/// runs the other way: the quadtree holds a handful of nodes per coarse level and
/// thousands of deep ones, so a sub-patch at z = 3 is nearly free and one at z = 12
/// is not. The taper is the measured crossing of those two curves, and the two
/// curves have no common closed form, so it is a table.
///
/// # The measurement
///
/// All nine sweeps of the harness, aggregated over the raw per-cell CSV columns,
/// against `QuadtreeManager::update` over the 204 bench poses. False negatives are
/// **zero at every point** in this table.
///
/// | k(z) | total FP | worst sweep vs the pre-rework baseline | mean update | B/node |
/// |------|----------|----------------------------------------|-------------|--------|
/// | `1` (no grid anywhere)      | 3.87 % | `axis_sweep` 123 vs 23 — far worse | 3.4 µs | 192 |
/// | `16,16,8,8,4,2,1`           | 2.39 % | `aspect_extremes` 9 vs 3 — worse   | 6.0 µs | 1 487 |
/// | **`16,16,12,8,6,4,3,2,1`**  | **2.05 %** | **all nine better**            | **6.7 µs** | **1 919** |
/// | `16,16,12,12,8,6,4,2,1`     | 1.97 % | all nine better                    | 7.5 µs | 2 362 |
/// | `16,16,12,12,12,8,4,2,1`    | 1.93 % | all nine better                    | 8.1 µs | 2 691 |
/// | flat `k = 8` from z = 2     | 1.84 % | all nine better                    | 17.7 µs | 6 015 |
///
/// The pre-rework baseline this must beat is 5.39 % FP at 7.8 µs and 2 966 B/node
/// (measured on the same 204 poses, `main` @ da6573a). The chosen row is the knee:
/// one step richer costs 0.8 µs for 0.08 points of FP, one step poorer gives back
/// 0.34 points and loses `aspect_extremes`. It leaves 1.1 µs of the old budget
/// unspent and a third of the old memory free.
const SUB_BOXES_PER_AXIS: [u32; 9] = [16, 16, 12, 8, 6, 4, 3, 2, 1];

fn sub_boxes_per_axis(z: u8) -> u32 {
    SUB_BOXES_PER_AXIS[(z as usize).min(SUB_BOXES_PER_AXIS.len() - 1)]
}

/// Sample grid used to fit a node's OBB, as `(steps+1)²` points.
///
/// §5.3 proves a 3×3 grid captures all three extents of a lon/lat patch *exactly*
/// on the sphere — east at a λ endpoint, up-min at a corner, up-max at the centre,
/// north among four λ×φ combinations, all of them grid points. The denser grid at
/// `z < 5` is kept because on the **ellipsoid** the `up` axis is the surface normal
/// rather than the radius, which perturbs that argument, and a coarse tile is where
/// the perturbation is largest. Sampling more can only grow the box, never shrink
/// it, so this is the conservative direction.
fn obb_grid_steps(z: u8) -> u32 {
    if z < 5 {
        8
    } else {
        2
    }
}

/// Calibration constant, *not* a geometry constant, despite the name this constant
/// had before the pre-WP4 fixes documented on [`lod_factor_for`] (`GROUND_PER_RADIUS`).
///
/// The quantity that name claimed — tile ground width over the sphere-fitted
/// [`QuadtreeNode::unstretched_radius`] — is a real, level-independent geometric
/// ratio, but it measures **≈ 1.415** (essentially `√2 ≈ 1.4142`, the corner-to-centre
/// half-diagonal relation for a roughly square patch; both halve per zoom level, so
/// the ratio is level-independent — verified numerically at z = 8, 11, 14, 18 by
/// `test_true_ground_per_radius_is_not_the_calibration_constant` in
/// `src/testing/lod/test_lod_sweep.rs`, converging as the flat-chord approximation's
/// curvature error shrinks; coarser tiles, z ≤ 5, read measurably lower for the same
/// reason the WP1 harness's own 3×3-grid curvature note describes). `256.0 / 315.0 ≈
/// 0.8127` is not that number — it is ~1.74× off — because it was reverse-engineered
/// to reproduce the old hard-coded `lod_factor = 2.0` at one reference configuration,
/// not derived from the tile geometry. It is kept as a residual, not replaced with
/// the true geometric
/// ratio, specifically so [`lod_factor_for`]'s no-op calibration at
/// `target_texel_ratio = 1.0` survives untouched — see that function's doc comment.
///
/// Written as the literal fraction it was fitted as, not as a decimal, so the
/// calibration can still be checked by hand — see [`lod_factor_for`].
const LOD_CALIBRATION_CONSTANT: f32 = 256.0 / 315.0;

/// The LOD constant, derived instead of hand-picked — WP3/3b of `docs/pre-terrain-plan.md`,
/// with the direction/exponent/naming fixes from the follow-up pass documented in that
/// file's WP3 section.
///
/// `QuadtreeNode::apply_lod` refines while `dist < unstretched_radius · lod_factor`.
/// That is Cesium's rule `d < G(z)·H / (maxSSE · 2·tan(fovy/2))` with every variable
/// frozen into one number, historically the literal `2.0`. This function unfreezes
/// them, keeping the rule's shape:
///
/// ```text
/// lod_factor = (LOD_CALIBRATION_CONSTANT / texture_size)
///            · viewport_height
///            · sqrt(target_texel_ratio)
///            / (2·tan(fovy/2))
/// ```
///
/// `target_texel_ratio` is texels of imagery demanded per screen pixel — the WP1 LOD
/// harness's own metric (`texels / screen_px`, an *area* ratio), whose natural target
/// is `1.0` (one texel per pixel: neither blurry nor wasteful). **Higher means more
/// texels demanded per pixel, i.e. sharper, more-subdivided tiles** — `target_texel_ratio`
/// is a multiplier on `lod_factor`, not a divisor: turning the knob up asks for more
/// resolution, and more resolution means refining out to a *larger* distance, i.e. a
/// *larger* `lod_factor`. (An earlier version of this function and its doc comment
/// disagreed with itself on this — dividing by `target_texel_ratio` while documenting
/// "higher = sharper" one sentence after "higher = fewer texels per pixel". Division
/// also had the wrong degree: `target_texel_ratio` is an *area* ratio (texels² over
/// px²) but `lod_factor` scales a *linear* distance, so the conversion is a square
/// root, not the first power. Both were invisible at the shipped default, because
/// `sqrt(1.0) == 1.0 == 1.0/1.0`.)
///
/// # The calibration is exact in rationals, not merely to float precision
///
/// At the reference configuration — `texture_size = 512`, `viewport_height = 1080`,
/// `target_texel_ratio = 1`, and `fovy` at the engine's default `focal_length = 28` mm
/// on a `sensor_height = 24` mm sensor — this evaluates to **exactly `2`**:
///
/// `fovy/2 = atan(sensor_height / (2·focal_length)) = atan(24/56) = atan(3/7)`, and
/// `tan(atan(x)) ≡ x`, so `tan(fovy/2) = 3/7` *exactly* and `2·tan(fovy/2) = 6/7`.
/// `sqrt(target_texel_ratio) = sqrt(1) = 1`, so it drops out and the rational
/// arithmetic is identical to the pre-fix version. With `LOD_CALIBRATION_CONSTANT = 256/315`:
///
/// | step                              | exact value              |
/// |------------------------------------|--------------------------|
/// | `LOD_CALIBRATION_CONSTANT / 512`  | `(256/315)/512 = 1/630`  |
/// | `· 1080`                          | `1080/630 = 12/7`        |
/// | `· sqrt(1)`                       | `12/7` (unchanged)       |
/// | `2·tan(fovy/2)`                   | `6/7`                    |
/// | `(12/7) / (6/7)`                  | **`2`**                  |
///
/// or as one fraction: `(256 · 1080 · 7) / (315 · 512 · 6) = 1935360/967680 = 2/1`.
/// So this remains a provable no-op at the default config, not an approximate one; it
/// reproduces the old constant bit-for-bit in f32 (verified), and `apply_lod`'s `dist`
/// is untouched, so no tile can change level.
///
/// **This exactness is a property of `focal_length = 28` / `sensor_height = 24`
/// specifically.** `atan` of a rational is generally *not* rational — the identity
/// `tan(atan(x)) = x` is what sidesteps that here, and it only helps because the
/// engine defines `fovy` *as* an `atan` of the rational `24/56`. If the default
/// `focal_length` ever changes, `2·tan(fovy/2)` will generally no longer be a clean
/// rational, [`LOD_CALIBRATION_CONSTANT`] will need re-deriving against the new default,
/// and the "exactly `2.0`, bit-identical" claim breaks. Re-run the WP1 LOD harness if so.
///
/// # Away from the calibration point
///
/// The calibration above only pins `target_texel_ratio = 1.0` at one viewport/focal
/// length. `test_lod_factor_scales_with_sqrt_target_not_inverse_linear` and
/// `test_lod_harness_aggregate_ratio_scales_with_target` (in
/// `src/testing/lod/test_lod_sweep.rs`) check the `sqrt` relationship — both in the
/// isolated function and end-to-end through the real quadtree and the WP1 harness's
/// own `texels/screen_px` metric — at `target_texel_ratio = 4.0`, where a `/target`
/// bug or a linear-in-`target` bug would both disagree with the measured result but
/// agree with it at `target = 1.0`.
///
/// Deliberately *not* frozen out of this: 3a (measuring `dist` to the nearest point of
/// the node's OBB rather than to its centre) was specified alongside this in WP3 and
/// was measured to be incompatible with a no-op — it is deferred to WP4. See the
/// "Refuted, moved to WP4" note in `docs/pre-terrain-plan.md`.
pub fn lod_factor_for(
    target_texel_ratio: f32,
    texture_size_px: f32,
    viewport_height_px: f32,
    fovy_rad: f32,
) -> f32 {
    (LOD_CALIBRATION_CONSTANT / texture_size_px)
        * viewport_height_px
        * target_texel_ratio.sqrt()
        / (2.0 * (fovy_rad * 0.5).tan())
}

/// Outward unit normal of the ellipsoid at `p`, in f64 — the normalised gradient of
/// the implicit form (1.1). Exact whether or not `p` is on the surface.
fn ellipsoid_normal(p: DVec3) -> DVec3 {
    const INV_A2: f64 = 1.0 / (EARTH_RADIUS_A_F64 * EARTH_RADIUS_A_F64);
    const INV_B2: f64 = 1.0 / (EARTH_RADIUS_B_F64 * EARTH_RADIUS_B_F64);
    DVec3::new(p.x * INV_A2, p.y * INV_B2, p.z * INV_A2).normalize()
}

fn surface_point(lon_deg: f64, lat_deg: f64) -> DVec3 {
    let p = lon_lat_to_ecef_f64(lon_deg, lat_deg);
    DVec3::new(p[0], p[1], p[2])
}

/// The tangent frame used to orient a patch's bounding box (§6.2).
///
/// `east` is built **analytically from the centre longitude**, not as `Y × normal`.
/// The old cross-product form had two problems: near a pole it divided by
/// `cos φ' → 0`, and exactly at a pole it fell back to `+X`, which is *not*
/// orthogonal to the normal — the resulting oblique "coordinates" make the
/// reconstructed box fail to contain its own samples. That branch never fired
/// (no tile centre is exactly at ±90°) but it was a loaded gun. Here:
///
/// * `‖east‖ = 1` for every `λ_c`, poles included — no degeneracy, no branch;
/// * `east·up = 0` **identically**, since `up ∝ (k cos λ_c, ·, −k sin λ_c)`;
/// * `north = up × east` is automatically unit.
fn tangent_frame(center_lon_deg: f64, up: DVec3) -> (DVec3, DVec3) {
    let (sin_lon, cos_lon) = center_lon_deg.to_radians().sin_cos();
    let east = DVec3::new(-sin_lon, 0.0, -cos_lon);
    let north = up.cross(east).normalize();
    (east, north)
}

/// Fits an oriented box to a lon/lat patch, in f64.
///
/// Returns `(surface_centre, bounding_radius, obb)`, where `bounding_radius` is
/// the greatest distance from the patch centre to a sampled point (the renderer's
/// per-tile bounding radius) and `obb` is the box in the [`tangent_frame`] at that
/// centre.
fn fit_obb(b: &TileBounds, steps: u32) -> (DVec3, f32, OrientedBoundingBox) {
    let center_lon = b.center_lon();
    let center_lat = b.center_lat();
    let surface_center = surface_point(center_lon, center_lat);
    let up = ellipsoid_normal(surface_center);
    let (east, north) = tangent_frame(center_lon, up);

    let mut min_ext = DVec3::splat(f64::INFINITY);
    let mut max_ext = DVec3::splat(f64::NEG_INFINITY);
    let mut max_dist_sq = 0.0_f64;

    for i in 0..=steps {
        let u = i as f64 / steps as f64;
        let lon = b.lon_min + u * (b.lon_max - b.lon_min);
        for j in 0..=steps {
            let v = j as f64 / steps as f64;
            let lat = b.lat_min + v * (b.lat_max - b.lat_min);
            let rel = surface_point(lon, lat) - surface_center;
            max_dist_sq = max_dist_sq.max(rel.length_squared());
            let local = DVec3::new(rel.dot(east), rel.dot(north), rel.dot(up));
            min_ext = min_ext.min(local);
            max_ext = max_ext.max(local);
        }
    }

    let offset = (max_ext + min_ext) * 0.5;
    let extents = (max_ext - min_ext) * 0.5;
    let obb_center = surface_center + east * offset.x + north * offset.y + up * offset.z;

    let to_f32 = |v: DVec3| Vec3::new(v.x as f32, v.y as f32, v.z as f32);
    let obb = OrientedBoundingBox::new(
        obb_center,
        [
            to_f32(east * extents.x),
            to_f32(north * extents.y),
            to_f32(up * extents.z),
        ],
    );

    (surface_center, max_dist_sq.sqrt() as f32, obb)
}

/// The `[u0,u1] × [v0,v1]` sub-rectangle of a tile, in degrees.
///
/// `v` is parameterised in **Mercator y**, matching the mesh, so consecutive
/// sub-rectangles share an edge exactly and their union is the whole tile. The pole
/// stretch is reapplied to the sub-rectangle that actually touches the pole row.
fn sub_bounds(id: &TileId, b: &TileBounds, u0: f64, u1: f64, v0: f64, v1: f64) -> TileBounds {
    let mut lat_max = web_mercator_y_to_lat_f64(id.y as f64 + v0, id.z);
    let mut lat_min = web_mercator_y_to_lat_f64(id.y as f64 + v1, id.z);
    if id.y == 0 && v0 == 0.0 {
        lat_max = 90.0;
    }
    if id.y == (1_u32 << id.z) - 1 && v1 == 1.0 {
        lat_min = -90.0;
    }
    TileBounds {
        lon_min: b.lon_min + u0 * (b.lon_max - b.lon_min),
        lon_max: b.lon_min + u1 * (b.lon_max - b.lon_min),
        lat_min,
        lat_max,
    }
}

/// A tile's patch cut into a `k × k` grid of sub-patches, each with its own box.
///
/// Two things are stored per sub-patch, and both are needed:
///
/// * an **oriented box** bounding it, for the frustum test;
/// * the **spherical rectangle** itself, for the exact limb test (§3.4) — as the
///   `k+1` longitude and `k+1` latitude breakpoints, shared along each row and
///   column rather than stored per sub-patch. That is `32·(k+1)` bytes instead of
///   `64·k²`: at `k = 8`, 288 B rather than 4 kB.
///
/// The breakpoints are the same ones [`sub_bounds`] produces — longitude linear in
/// the tile's own λ span, latitude taken in **Mercator y** so consecutive sub-patches
/// share an edge exactly and the union of the `k²` sub-patches is the whole drawn
/// patch, pole stretch included. That union property is what makes
/// [`SubGrid::has_surviving_sub_patch`] sound.
pub struct SubGrid {
    k: u32,
    /// `k²` boxes, indexed `ui · k + vi` — `ui` along λ, `vi` along **Mercator y**,
    /// so `vi = 0` is the sub-patch row at the *north* edge of the tile.
    obbs: Vec<OrientedBoundingBox>,
    /// `(sin λ, cos λ)` at the `k+1` longitude breakpoints, increasing in λ.
    lon: Vec<(f64, f64)>,
    /// `(sin φ, cos φ)` at the `k+1` latitude breakpoints, increasing in φ —
    /// **index 0 is the south edge**, the same polarity as [`TilePatch`]'s
    /// `sin_lat`/`cos_lat` and the same as `lon` above, so every span handed to
    /// [`super::horizon::lat_span_max`] is `[low, high]` with no reversal at the
    /// call site.
    ///
    /// Note that this runs *against* `vi`, which follows Mercator y and therefore
    /// counts southwards; [`SubGrid::sub_patch_is_occluded`] converts once, in one
    /// named place.
    lat: Vec<(f64, f64)>,
}

impl SubGrid {
    /// Builds the grid, or `None` when `k < 2` (a 1×1 grid is the node's own OBB).
    fn build(id: &TileId, b: &TileBounds, k: u32) -> Option<Self> {
        if k < 2 {
            return None;
        }
        let n = k as usize;
        let mut obbs = Vec::with_capacity(n * n);
        for ui in 0..k {
            for vi in 0..k {
                let sb = sub_bounds(
                    id,
                    b,
                    ui as f64 / k as f64,
                    (ui + 1) as f64 / k as f64,
                    vi as f64 / k as f64,
                    (vi + 1) as f64 / k as f64,
                );
                // `steps = 4`, deliberately, and **not** `obb_grid_steps(id.z)`.
                // This is not an oversight to be tidied away: `steps` decides which
                // points of the sub-rectangle are sampled, so any other value gives
                // different extents for every sub-box in the tree and moves the
                // false-positive figures globally. A sub-patch is small enough that
                // §5.3's 3×3 argument holds comfortably at every zoom, which is why
                // the node-level taper does not apply here. Changing this number is
                // a recalibration, not a cleanup.
                obbs.push(fit_obb(&sb, 4).2);
            }
        }

        // The breakpoints, from the very same expressions `sub_bounds` uses, so a
        // sub-patch's rectangle and its box are fitted to identical numbers.
        //
        // `i` walks the breakpoints north → south (it follows Mercator y, like
        // `vi`), but `lat` is stored south → north so that it shares its polarity
        // with `lon` and with `TilePatch`. The values are therefore written at
        // `n - i` rather than pushed. This is *only* a permutation of where each
        // number lands: `phi` is still produced by the same expression at the same
        // `t`, bit for bit. **Do not "simplify" it by re-deriving the breakpoint
        // from the other end** — `1.0 - i as f64 / k as f64` is not bit-identical
        // to `i as f64 / k as f64` for k = 12, 6, 3, and one ulp here travels
        // through `web_mercator_y_to_lat_f64` into `sin_cos` and flips borderline
        // sub-patches.
        let mut lon = Vec::with_capacity(n + 1);
        let mut lat = vec![(0.0_f64, 0.0_f64); n + 1];
        for i in 0..=k {
            let t = i as f64 / k as f64;
            lon.push(
                (b.lon_min + t * (b.lon_max - b.lon_min))
                    .to_radians()
                    .sin_cos(),
            );

            let mut phi = web_mercator_y_to_lat_f64(id.y as f64 + t, id.z);
            if i == 0 && id.y == 0 {
                phi = 90.0;
            }
            if i == k && id.y == (1_u32 << id.z) - 1 {
                phi = -90.0;
            }
            lat[n - i as usize] = phi.to_radians().sin_cos();
        }

        Some(SubGrid { k, obbs, lon, lat })
    }

    fn heap_bytes(&self) -> usize {
        self.obbs.capacity() * std::mem::size_of::<OrientedBoundingBox>()
            + (self.lon.capacity() + self.lat.capacity()) * std::mem::size_of::<(f64, f64)>()
    }

    /// Does **any** sub-patch of this tile survive its own proof of invisibility?
    ///
    /// Not "is a sub-patch visible" — no stage here can establish that. A sub-patch
    /// is discarded only when it is provably invisible on its own: either its
    /// spherical rectangle is entirely behind the limb (exact, §3.4) or its box is
    /// separated from the frustum (exact, [`Frustum::intersects_obb`]). A survivor
    /// is merely one that no proof reached, and `true` here means exactly that.
    ///
    /// Discarding *every* sub-patch does prove the tile invisible, because the
    /// sub-patches' union is the whole drawn patch — so any drawable point lies in
    /// some sub-patch, and that sub-patch is invisible. Keeping the tile as soon as
    /// one survives is the conservative direction (I-6). ∎
    ///
    /// # Two passes, because the third frustum stage is 7× the first
    ///
    /// Pass 1 asks only the four planes, which answer `Outside` or `Inside`
    /// outright for every sub-patch that is not on the frustum boundary. An
    /// `Inside` sub-patch settles the node immediately; a tile with no straddling
    /// sub-patch at all is settled too. Pass 2 — the exact separating-axis set —
    /// therefore runs only for a node that has sub-patches on the frustum boundary
    /// and none strictly within it, which is the thin band where the answer was
    /// ever in doubt.
    ///
    /// The limb test comes first in both passes: it is 25 f64 flops against the
    /// four-plane test's ~92, and its λ half is hoisted out of the inner loop,
    /// since every sub-patch in column `ui` has the same λ span.
    fn has_surviving_sub_patch(&self, ctx: &CullContext) -> bool {
        let k = self.k as usize;
        let mut any_straddling = false;

        for ui in 0..k {
            let a_star = self.column_a_star(ctx, ui);
            for vi in 0..k {
                if self.sub_patch_is_occluded(ctx, a_star, vi) {
                    continue;
                }
                let sub_patch_obb = &self.obbs[ui * k + vi];
                let d = ctx.frustum.relative(sub_patch_obb.center);
                match ctx.frustum.classify_box(
                    d,
                    &sub_patch_obb.half_axes,
                    sub_patch_obb.half_axis_l1,
                ) {
                    PlaneVerdict::Inside => return true,
                    PlaneVerdict::Straddling => any_straddling = true,
                    PlaneVerdict::Outside => {}
                }
            }
        }
        if !any_straddling {
            return false;
        }

        for ui in 0..k {
            let a_star = self.column_a_star(ctx, ui);
            for vi in 0..k {
                if self.sub_patch_is_occluded(ctx, a_star, vi) {
                    continue;
                }
                if ctx.frustum.intersects_obb(&self.obbs[ui * k + vi]) {
                    return true;
                }
            }
        }
        false
    }

    /// `A*` for sub-column `ui`, shared by every sub-patch in it.
    #[inline]
    fn column_a_star(&self, ctx: &CullContext, ui: usize) -> f64 {
        super::horizon::lon_span_max(
            &ctx.horizon,
            &[self.lon[ui].0, self.lon[ui + 1].0],
            &[self.lon[ui].1, self.lon[ui + 1].1],
        )
    }

    /// Is sub-patch row `vi` of a column with this `A*` entirely behind the limb?
    ///
    /// The single place where the two directions meet: `vi` counts south along
    /// Mercator y (`vi = 0` is the north row, matching `obbs`), while `lat` is
    /// stored increasing in φ. `lo = k − 1 − vi` is that conversion, and it leaves
    /// the span itself in the `[low, high]` order every `*_span_max` expects.
    #[inline]
    fn sub_patch_is_occluded(&self, ctx: &CullContext, a_star: f64, vi: usize) -> bool {
        let lo = self.k as usize - 1 - vi;
        let s = super::horizon::lat_span_max(
            &ctx.horizon,
            a_star,
            &[self.lat[lo].0, self.lat[lo + 1].0],
            &[self.lat[lo].1, self.lat[lo + 1].1],
        );
        super::horizon::span_is_occluded(&ctx.horizon, s)
    }
}

/// What one stage proved about a node.
///
/// The asymmetry is the point. `Cull` is a **proof of invisibility** and `Keep` a
/// **proof of visibility** — or rather of everything that follows being unable to
/// prove invisibility, which is the same thing operationally: `Keep` skips every
/// later stage, so it may only be returned where a later stage could not have
/// culled soundly. `Undecided` passes the node on and is always safe (I-6).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StageVerdict {
    /// Provably invisible. Stop; the node and its subtree go.
    Cull,
    /// Settled in favour of keeping. Stop; **every later stage is skipped**.
    Keep,
    /// No proof either way. Hand the node to the next stage.
    Undecided,
}

/// One culling stage, as a **name** rather than as a closure or a piece of state.
///
/// Field-less and one byte: every per-frame quantity a stage needs lives in
/// [`CullContext`] (the way `horizon` does), and every per-node quantity lives on
/// the node. A stage only says *which* test to run, so the stage list is a
/// constant that outlives the frame instead of something rebuilt per frame.
///
/// That is why this is not `Stage::Horizon(HorizonCamera)`: carrying frame data in
/// the variant would force the list to be rebuilt every frame, and the list is
/// exactly the thing that should not change.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Stage {
    /// The exact limb test on the whole patch ([`TilePatch::is_occluded`]).
    Horizon,
    /// The frustum against the node's own box.
    ///
    /// This stage **absorbs the grid / no-grid dichotomy internally**, and that is
    /// deliberate: which of the two forms runs is a property of the *node*
    /// (`sub_grid.is_some()`), not of the configured mode, so the stage list stays
    /// the same for every node in a frame. A design that swapped stages in and out
    /// per node would make the list frame- and node-dependent for no gain.
    ///
    /// * with a grid — the four planes only ([`Frustum::classify_box`]), because an
    ///   `Inside` verdict already settles the node and a `Straddling` one is what
    ///   [`Stage::SubPatchGrid`] exists to resolve;
    /// * without one — the full separating-axis set ([`Frustum::intersects_obb`]),
    ///   since nothing downstream will look at this node again.
    NodeFrustum,
    /// Both exact tests again, per sub-patch of the `k × k` grid
    /// ([`SubGrid::has_surviving_sub_patch`]).
    ///
    /// `Undecided` when the node has no grid. Behind [`Stage::NodeFrustum`] that
    /// case is unreachable — a grid-less node is always settled there — but the
    /// stage is still total, because a pipeline may list it alone.
    SubPatchGrid,
    /// Outright atmospheric-fog cull — WP5 of `docs/pre-terrain-plan.md`. `Cull`
    /// when [`cesium_fog`] of the node's distance to the eye reaches `1.0`;
    /// `Undecided` otherwise (including whenever `ctx.fog_density == 0.0`, i.e. no
    /// fog set, or the camera is above `FogConfig::max_height_m`). Never `Keep`:
    /// fog only ever *removes* geometry that other stages already proved visible,
    /// it is never itself proof that something is visible.
    ///
    /// **Not geometrically sound, and not in [`CullPipeline::DEFAULT`].** See the
    /// warning on [`super::fog`]'s module doc comment before touching this stage's
    /// placement in any pipeline.
    Fog,
}

impl Stage {
    /// Runs this stage. `&self` on the node, never `&mut`: see
    /// [`QuadtreeNode::update`] for why that signature is load-bearing.
    #[inline]
    fn run(self, node: &QuadtreeNode, ctx: &CullContext) -> StageVerdict {
        match self {
            Stage::Horizon => {
                if node.patch.is_occluded(&ctx.horizon) {
                    StageVerdict::Cull
                } else {
                    StageVerdict::Undecided
                }
            }
            Stage::NodeFrustum => match &node.sub_grid {
                Some(_) => {
                    // The f64 subtraction is invariant I-2.
                    let delta = ctx.frustum.relative(node.obb.center);
                    match ctx.frustum.classify_box(
                        delta,
                        &node.obb.half_axes,
                        node.obb.half_axis_l1,
                    ) {
                        PlaneVerdict::Outside => StageVerdict::Cull,
                        PlaneVerdict::Inside => StageVerdict::Keep,
                        PlaneVerdict::Straddling => StageVerdict::Undecided,
                    }
                }
                None => {
                    if ctx.frustum.intersects_obb(&node.obb) {
                        StageVerdict::Keep
                    } else {
                        StageVerdict::Cull
                    }
                }
            },
            Stage::SubPatchGrid => match &node.sub_grid {
                Some(grid) => {
                    if grid.has_surviving_sub_patch(ctx) {
                        StageVerdict::Keep
                    } else {
                        StageVerdict::Cull
                    }
                }
                None => StageVerdict::Undecided,
            },
            Stage::Fog => {
                if ctx.fog_density <= 0.0 {
                    return StageVerdict::Undecided;
                }
                // The tile's own nearest-point distance, in metres — same choice
                // `QuadtreeNode::apply_lod`'s fog relaxation makes, and independent
                // of `LodDistanceMode` (a separate, orthogonal WP4/C experiment):
                // fog concealment is a property of the tile's own geometry, not of
                // which LOD distance metric happens to be active.
                let dist_m = node.obb.distance_to_point(ctx.frustum.eye)
                    * MEGAMETERS_TO_METERS;
                if cesium_fog(dist_m, ctx.fog_density) >= 1.0 {
                    StageVerdict::Cull
                } else {
                    StageVerdict::Undecided
                }
            }
        }
    }
}

/// The most stages a pipeline can hold — one of each [`Stage`].
pub const MAX_STAGES: usize = 4;

/// An ordered, switchable list of culling stages. `Copy`, 4 bytes, no allocation.
///
/// Lives on [`QuadtreeManager`], where it survives frames and changes only when a
/// mode does, and is copied by value into the per-frame [`CullContext`] so that
/// stays `Copy` too. It is emphatically **not** stored per node: the quadtree holds
/// tens of thousands of nodes and none of them has an opinion about which stages
/// run.
///
/// # Dispatch is an enum and a `match`, on purpose
///
/// Not `dyn Trait`: inlining dies at the stage boundary, and
/// [`Frustum::classify_box`] earns its cost only because `delta` stays in
/// registers. Not static generics either: every mode combination would get its own
/// monomorphisation, and the top of the call would still need an enum to choose
/// between them.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CullPipeline {
    stages: [Stage; MAX_STAGES],
    len: u8,
}

impl CullPipeline {
    /// The shipped pipeline: horizon, then the node's box, then the sub-patch grid.
    ///
    /// # Equivalence with the hand-written cascade it replaced
    ///
    /// For `DEFAULT`, [`CullPipeline::keeps`] reduces — by unfolding the loop over
    /// three known stages and applying the final rule — to exactly
    ///
    /// ```text
    /// with a grid:     is_occluded ? false : match classify_box {
    ///                                            Outside    => false,
    ///                                            Inside     => true,
    ///                                            Straddling => has_surviving_sub_patch,
    ///                                        }
    /// without a grid:  is_occluded ? false : intersects_obb
    /// ```
    ///
    /// Read off the three stages in order:
    ///
    /// 1. [`Stage::Horizon`] returns `Cull` — i.e. `keeps = false` — exactly when
    ///    `patch.is_occluded`, and `Undecided` otherwise, so the rest of the
    ///    cascade is reached under exactly the old `!is_occluded` condition.
    /// 2. [`Stage::NodeFrustum`] with a grid maps `Outside → Cull → false` and
    ///    `Inside → Keep → true`, which are the old arms verbatim, and leaves only
    ///    `Straddling` open. Without a grid it maps `intersects_obb` to
    ///    `Keep → true` / `Cull → false`, again the old arm verbatim, and leaves
    ///    nothing open.
    /// 3. [`Stage::SubPatchGrid`] is therefore reached only in the `Straddling`
    ///    case, where it returns `Keep → true` / `Cull → false` according to
    ///    `has_surviving_sub_patch` — the old `Straddling` arm.
    ///
    /// The final rule (`true`) is never reached under `DEFAULT`: stage 2 leaves the
    /// node open only when it has a grid, and stage 3 always decides when it does.
    ///
    /// Same calls, same order, same arguments, and nothing is computed that the old
    /// code did not compute — with one exception in the other direction: `delta` is
    /// no longer computed on the grid-less path, where the old code computed it and
    /// then never read it (`intersects_obb` derives its own). Dead arithmetic on
    /// f32/f64 has no observable effect, so the visible set is bit-identical.
    pub const DEFAULT: CullPipeline =
        CullPipeline::of(&[Stage::Horizon, Stage::NodeFrustum, Stage::SubPatchGrid]);

    /// `DEFAULT` plus [`Stage::Fog`] — WP5 of `docs/pre-terrain-plan.md`. **This is
    /// what `wgpu_state.rs` actually runs in production**; `DEFAULT` alone is not.
    ///
    /// # Do not use this in the culling harness. Ever.
    ///
    /// Fog culling is not geometrically sound — it deliberately discards tiles that
    /// are genuinely visible, so `Stage::Fog` makes false negatives non-zero *by
    /// design*. Every FN = 0 guarantee in `docs/culling-math.md`, and every sweep in
    /// `src/testing/culling/`, is proved against `CullPipeline::DEFAULT` — put this
    /// constant in place of it (in the harness, in `all_pipelines()`, in a bench) and
    /// every one of those sweeps goes red, correctly, because the thing they check
    /// (nothing visible is ever culled) is no longer true and was never supposed to
    /// be while measuring this pipeline. That is not a bug to fix; it is the reason
    /// `Stage::Fog` exists as an *addition* on top of `DEFAULT` rather than a change
    /// to it. See [`super::fog`]'s module doc comment for the full story.
    pub const DEFAULT_WITH_FOG: CullPipeline =
        CullPipeline::of(&[Stage::Horizon, Stage::NodeFrustum, Stage::SubPatchGrid, Stage::Fog]);

    /// Builds a pipeline from a stage list. Panics above [`MAX_STAGES`] stages —
    /// `const`, so a bad constant fails to compile rather than at run time.
    pub const fn of(stages: &[Stage]) -> CullPipeline {
        assert!(stages.len() <= MAX_STAGES, "too many culling stages");
        let mut out = [Stage::Horizon; MAX_STAGES];
        let mut i = 0;
        while i < stages.len() {
            out[i] = stages[i];
            i += 1;
        }
        CullPipeline {
            stages: out,
            len: stages.len() as u8,
        }
    }

    /// The stages, in execution order.
    pub fn stages(&self) -> &[Stage] {
        &self.stages[..self.len as usize]
    }

    /// Does this node survive the pipeline?
    ///
    /// # The final rule, and the property it buys
    ///
    /// A cascade that runs out of stages **keeps**. That single choice is what makes
    /// the stage list safe to shorten: dropping a stage removes proofs of
    /// invisibility and adds none, so for any pipeline `P` and any prefix `P'` of
    /// `P`, `kept(P) ⊆ kept(P')`. Soundness therefore never rests on a stage being
    /// *present* — only on each stage that *is* present being correct. Invariant
    /// I-7 is exactly this property one level up, and
    /// `test_stage_prefix_only_grows_the_kept_set` checks it.
    ///
    /// Turn the final rule to `false` and the property inverts: omitting a stage
    /// would start losing tiles, and every configuration but the full one would be
    /// unsound.
    ///
    /// # Why the loop is written over `0..MAX_STAGES` and not over the slice
    ///
    /// Measured, not stylistic. `for &stage in &self.stages[..self.len]` gives the
    /// optimiser a run-time trip count, so the three-way `match` in [`Stage::run`]
    /// stays a real branch and `QuadtreeManager::update` costs **7.3 µs** on the
    /// bench poses. A constant trip count with an early `break` unrolls into three
    /// specialised copies — one per slot, each with its stage known — and costs
    /// **6.9 µs**, against 6.7 µs for the hand-written cascade this replaced. Tidying
    /// it back into a slice loop buys nothing and costs 6 % of the frame's culling.
    #[inline]
    fn keeps(&self, node: &QuadtreeNode, ctx: &CullContext) -> bool {
        let len = self.len as usize;
        for i in 0..MAX_STAGES {
            if i == len {
                break;
            }
            match self.stages[i].run(node, ctx) {
                StageVerdict::Cull => return false,
                StageVerdict::Keep => return true,
                StageVerdict::Undecided => {}
            }
        }
        true
    }
}

impl Default for CullPipeline {
    fn default() -> Self {
        CullPipeline::DEFAULT
    }
}

/// Everything the per-node tests need, built once per frame.
///
/// A bag of frame data, and the stages' only source of it: a [`Stage`] holds no
/// state of its own, it indexes into this. Anything a new stage needs precomputed
/// per frame belongs here, next to `horizon`.
/// Which distance `QuadtreeNode::apply_lod` measures the camera against —
/// WP4/C (`docs/pre-terrain-plan.md`), measurement only.
///
/// `Centre` is the only mode production code ever selects: [`QuadtreeManager::new`]
/// defaults to it and nothing in `wgpu_state.rs` sets anything else, so this enum
/// existing changes no shipped behaviour. `Box` exists purely so the WP4/C harness
/// comparison can measure 3a (box-distance, refuted as a no-op in WP3 — see the
/// callout there) against an equal tile budget rather than an equal
/// `target_texel_ratio`. Whether to adopt `Box` for real is WP4/D's decision, gated
/// on the product trade C's report lays out — this switch does not make that
/// decision, it only makes the comparison measurable.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum LodDistanceMode {
    /// `(self.center - eye).length()` — today's only production behaviour, and the
    /// value `subdivide_dist`/`collapse_dist` have always been calibrated against.
    #[default]
    Centre,
    /// Distance to the nearest point of the node's (stretched) `obb` —
    /// [`OrientedBoundingBox::distance_to_point`], committed unused in `37d5f6f`
    /// for exactly this. Can only ever be `<=` the centre distance, so it can only
    /// trigger *more* subdivision, never less (WP3's refutation measured +70% tiles
    /// at equal `target_texel_ratio`, which is why WP4/C compares at equal tile
    /// budget instead).
    Box,
}

#[derive(Clone, Copy, Debug)]
pub struct CullContext {
    pub frustum: Frustum,
    pub horizon: HorizonCamera,
    /// Copied by value from [`QuadtreeManager::pipeline`] — 4 bytes, so the context
    /// stays `Copy` and the stage list is not rebuilt per frame.
    pub pipeline: CullPipeline,
    /// WP4/C measurement switch — see [`LodDistanceMode`]. Defaults to `Centre`,
    /// production's only value.
    pub lod_distance_mode: LodDistanceMode,
    /// This frame's atmospheric fog density (WP5) — `0.0` (the default) means "no
    /// fog effect", which is both the harness's only value and what a camera above
    /// `FogConfig::max_height_m` computes. Read by [`Stage::Fog`] (only ever
    /// present in [`CullPipeline::DEFAULT_WITH_FOG`]) and by
    /// `QuadtreeNode::apply_lod`'s relaxation. See [`super::fog`]'s module doc
    /// comment.
    pub fog_density: f32,
}

impl CullContext {
    /// The shipped configuration, [`CullPipeline::DEFAULT`].
    pub fn new(frustum: &Frustum) -> Self {
        Self::with_pipeline(frustum, CullPipeline::DEFAULT)
    }

    pub fn with_pipeline(frustum: &Frustum, pipeline: CullPipeline) -> Self {
        Self {
            frustum: *frustum,
            horizon: HorizonCamera::new(frustum.eye),
            pipeline,
            lod_distance_mode: LodDistanceMode::default(),
            fog_density: 0.0,
        }
    }

    /// WP4/C only — see [`LodDistanceMode`]. Not called anywhere in production.
    pub fn with_lod_distance_mode(mut self, mode: LodDistanceMode) -> Self {
        self.lod_distance_mode = mode;
        self
    }

    /// WP5 only — see [`super::fog::FogConfig`] and this struct's `fog_density` field.
    pub fn with_fog_density(mut self, fog_density: f32) -> Self {
        self.fog_density = fog_density;
        self
    }
}

pub struct QuadtreeNode {
    pub id: TileId,
    /// Patch centre on the ellipsoid, **f64** (invariant I-2).
    pub center: DVec3,
    /// Greatest distance from [`QuadtreeNode::center`] to a sampled patch point —
    /// the renderer's per-tile bounding sphere, fitted on the **drawn** (stretched)
    /// rectangle by [`fit_obb`].
    pub bounding_radius: f32,
    /// The same measurement taken on the ***un*-stretched** rectangle
    /// ([`tile_bounds_unstretched`]), and the input to `subdivide_dist` below.
    ///
    /// A geometry, not an LOD tuning knob: the knob is
    /// `TileEngineConfig::target_texel_ratio`, `lod_factor` (from [`lod_factor_for`])
    /// is what it derives, and the threshold is `subdivide_dist`. None of those is
    /// this number. See [`QuadtreeNode::new`] for why the un-stretched
    /// rectangle is the deliberate choice.
    pub unstretched_radius: f32,
    pub obb: OrientedBoundingBox,
    /// The `k × k` sub-patch grid, when `k ≥ 2` (see [`sub_boxes_per_axis`]).
    pub sub_grid: Option<Box<SubGrid>>,
    /// The eight trig constants of this tile's rectangle, for the horizon test.
    pub patch: TilePatch,
    pub visible: bool,
    pub children: Option<Box<[QuadtreeNode; 4]>>,
}

impl QuadtreeNode {
    pub fn new(id: TileId) -> Self {
        // I-5: the single source of tile bounds, shared with `TileMesh::generate`.
        let bounds = tile_bounds(&id);
        let (center, bounding_radius, obb) = fit_obb(&bounds, obb_grid_steps(id.z));

        // Deliberately measured on the ***un*-stretched** rectangle: a polar row's
        // true ground extent, not its pull to ±90°. Kept as-is (§8.4) — it makes
        // polar caps subdivide later, which is an accepted FP source and never an
        // FN one, because subdividing late only keeps a coarser tile that still
        // covers the ground. Nothing about it is an LOD parameter; it is the size
        // of a geometric object, and the threshold derived from it is
        // `subdivide_dist`.
        let raw = tile_bounds_unstretched(&id);
        let (_, unstretched_radius, _) = fit_obb(&raw, 2);

        let sub_grid = SubGrid::build(&id, &bounds, sub_boxes_per_axis(id.z)).map(Box::new);

        QuadtreeNode {
            id,
            center,
            bounding_radius,
            unstretched_radius,
            obb,
            sub_grid,
            patch: TilePatch::new(&bounds),
            visible: false,
            children: None,
        }
    }

    /// Heap bytes this node hangs off itself (not counting children).
    pub fn sub_grid_heap_bytes(&self) -> usize {
        self.sub_grid
            .as_ref()
            .map(|g| std::mem::size_of::<SubGrid>() + g.heap_bytes())
            .unwrap_or(0)
    }

    pub fn subdivide(&mut self) {
        let z = self.id.z + 1;
        let x = self.id.x * 2;
        let y = self.id.y * 2;

        self.children = Some(Box::new([
            QuadtreeNode::new(TileId { z, x, y }),        // Top-Left
            QuadtreeNode::new(TileId { z, x: x + 1, y }), // Top-Right
            QuadtreeNode::new(TileId { z, x, y: y + 1 }), // Bottom-Left
            QuadtreeNode::new(TileId {
                z,
                x: x + 1,
                y: y + 1,
            }), // Bottom-Right
        ]));
    }

    /// Visibility, then LOD — and the split between them is structural.
    ///
    /// Every visibility test lives in [`CullPipeline`] behind a `&self` node
    /// borrow; everything that *changes* the tree lives in
    /// [`QuadtreeNode::apply_lod`] behind `&mut self`. A stage therefore **cannot**
    /// touch `children`, which is the invariant I-7 hazard in person: `children =
    /// None` deletes a subtree, so a test that could do that would be a test that
    /// could silently lose tiles. Here the borrow checker forbids it.
    pub fn update(&mut self, ctx: &CullContext, lod_factor: f32) {
        if !ctx.pipeline.keeps(self, ctx) {
            self.visible = false;
            self.children = None;
            return;
        }

        self.visible = true;
        self.apply_lod(ctx, lod_factor);
    }

    /// LOD, hysteresis and recursion, for a node that survived culling.
    ///
    /// Not a culling stage and deliberately not reachable as one: it takes
    /// `&mut self`.
    fn apply_lod(&mut self, ctx: &CullContext, lod_factor: f32) {
        // LOD distance, also from an f64 subtraction (§8.2): free, since the frame
        // is camera-relative anyway. `Centre` (the default, and production's only
        // value) is exactly the pre-WP4/C expression; `Box` is WP4/C's measurement
        // switch — see `LodDistanceMode`. `OrientedBoundingBox::distance_to_point`
        // keeps the same f64-subtraction discipline internally.
        let dist = match ctx.lod_distance_mode {
            LodDistanceMode::Centre => (self.center - ctx.frustum.eye).length() as f32,
            LodDistanceMode::Box => self.obb.distance_to_point(ctx.frustum.eye),
        };

        // Fog relaxation — WP5. Cesium subtracts `fog(dist, density) * fog.sse`
        // from its screen-space-error term, so a partly-fogged tile's error sits
        // closer to (or under) the refine threshold and it refines less. This
        // engine has no error term to subtract from — it refines while `dist <
        // subdivide_dist` — so the derived equivalent shrinks `subdivide_dist`
        // itself by the same fraction the tile is fogged:
        //
        //   subdivide_dist' = subdivide_dist * (1 - fog(dist, density))
        //
        // At `fog = 0` (no fog, or this node outside it) the threshold is
        // unchanged. At `fog -> 1` — the boundary `Stage::Fog` culls the node
        // outright at, in `CullPipeline::DEFAULT_WITH_FOG` — the threshold shrinks
        // to 0, so a heavily-fogged node stops accepting further refinement in the
        // frames just before it disappears rather than staying maximally refined
        // right up to the cull. `fog.sse` is deliberately **not** used here: it is
        // a pixel-space screen-space-error constant, and this formula has no error
        // term in those units to scale — see `FogConfig::sse`'s doc comment for
        // where it is reserved instead. Distance is the node's own nearest-point
        // distance (`obb.distance_to_point`), matching `Stage::Fog`'s choice and
        // independent of `LodDistanceMode` — fog concealment is a property of the
        // tile's own geometry, not of which experimental LOD distance metric is
        // active.
        let fog_relaxation = if ctx.fog_density > 0.0 {
            let fog_dist_m = self.obb.distance_to_point(ctx.frustum.eye) * MEGAMETERS_TO_METERS;
            1.0 - cesium_fog(fog_dist_m, ctx.fog_density)
        } else {
            1.0
        };

        // Hysteresis logic: Subdivide at 1.0x, but don't collapse until 1.2x.
        // A 20% band prevents LOD oscillation when the camera straddles the
        // subdivision threshold. The old 1.05x band (~50 m at z=19) was too
        // narrow and caused rapid APPEAR/DISAPPEAR flicker on high-detail tiles.
        // `fog_relaxation` is applied before the band is derived, not after, so
        // the 20% band stays a constant *fraction* of the (possibly fog-shrunk)
        // threshold rather than a fixed absolute margin that would narrow, and
        // therefore oscillate more easily, as fog thickens.
        let is_subdivided = self.children.is_some();
        let subdivide_dist = self.unstretched_radius * lod_factor * fog_relaxation;
        let collapse_dist = subdivide_dist * 1.20;

        let should_be_subdivided = if is_subdivided {
            dist < collapse_dist
        } else {
            dist < subdivide_dist
        };

        // Subdivide condition
        if should_be_subdivided && self.id.z < MAX_ZOOM {
            if self.children.is_none() {
                self.subdivide();
            }
            self.reorder_children_near_to_far(ctx.frustum.eye);
            if let Some(children) = &mut self.children {
                for child in children.iter_mut() {
                    child.update(ctx, lod_factor);
                }
            }
        } else {
            self.children = None;
        }
    }

    /// Reorders `self.children` in place so the quadrant nearest `eye` (measured
    /// in the tile's own east/north tangent frame) lands at index 0 and the
    /// diagonally-opposite quadrant lands at index 3 — a camera-relative,
    /// near-to-far ordering, mirroring Cesium's `visitVisibleChildrenNearToFar`
    /// (WP2b, `docs/pre-terrain-plan.md`).
    ///
    /// # Why this reorders by quadrant *identity*, not by array position
    ///
    /// `apply_lod` calls this on **every** update tick a node stays subdivided,
    /// not only the tick `subdivide()` runs — so `self.children` is not
    /// reliably in creation order `[TL, TR, BL, BR]` by the time this method is
    /// called; it may already hold whatever order a previous tick's call left
    /// it in. A first version of this method assumed a known starting order
    /// and reordered by swapping fixed *positions* (e.g. "swap slots 0 and 2").
    /// That is wrong under repeated calls: each of the four cases is a
    /// self-inverse permutation (a single transposition, or two disjoint
    /// ones), so applying the *same* case twice in a row — which is exactly
    /// what happens when the camera doesn't move between ticks — silently
    /// undoes it, and applying it an even number of times across ticks (four,
    /// here: [`UPDATE_ITERATIONS`] in the test harness) returns the array to
    /// its original creation order with **no visible reordering at all**. This
    /// was caught by comparing an order-sensitive traversal dump before and
    /// after the change and finding it byte-identical despite the reorder
    /// firing thousands of times — see the WP2b report for the reproduction.
    ///
    /// The fix: identify each child's quadrant from its own [`TileId`] (a
    /// child's `x`/`y` parity relative to `self.id * 2` is exactly its
    /// quadrant, regardless of which array slot it currently sits in), then
    /// move quadrants into their target slots by a selection pass — for each
    /// output slot, find the *remaining* position holding the quadrant that
    /// belongs there and swap it in. This is idempotent by construction (a
    /// call that finds every quadrant already in place performs zero swaps)
    /// and correct from any starting arrangement, not just a fresh one. It is
    /// still exactly the four cases below, just expressed as "which quadrant
    /// goes in which slot" instead of "which positions to swap" — not a
    /// generic sort.
    ///
    /// [`subdivide`](Self::subdivide) always creates children in creation
    /// order `[TL, TR, BL, BR]` (indices 0..3, Web-Mercator tile-index terms:
    /// `y` increases southward, `x` eastward); every downstream traversal
    /// (`collect_visible_tiles`, `collect_renderable_tiles`) walks
    /// `children.iter()` with no reordering of its own, so whatever order the
    /// slots hold at the end of the frame's `apply_lod` calls is the order
    /// tiles are collected and drawn in — this is the entire mechanism, no
    /// traversal code needs to change.
    ///
    /// The two middle slots (1 and 2) end up holding the two edge-adjacent
    /// quadrants in an unspecified relative order — intentional: this is a
    /// front-to-back *hint* for early-Z and draw order, not a requirement to
    /// fully sort all four children.
    fn reorder_children_near_to_far(&mut self, eye: DVec3) {
        let bounds = tile_bounds(&self.id);
        let up = ellipsoid_normal(self.center);
        let (east_axis, north_axis) = tangent_frame(bounds.center_lon(), up);
        let to_eye = eye - self.center; // f64 subtraction (I-2).
        let east = east_axis.dot(to_eye) > 0.0;
        let north = north_axis.dot(to_eye) > 0.0;

        // TopLeft=0 (west,north), TopRight=1 (east,north), BottomLeft=2
        // (west,south), BottomRight=3 (east,south) — matches subdivide()'s
        // creation order exactly.
        let near_idx = match (east, north) {
            (false, true) => 0,  // TL
            (true, true) => 1,   // TR
            (false, false) => 2, // BL
            (true, false) => 3,  // BR
        };

        // Quadrant identity wanted at each output slot, by near_idx. Derived
        // from the same 4 cases as the position-swap table this replaces:
        // near_idx 0 -> [TL,TR,BL,BR], 1 -> [TR,TL,BR,BL], 2 -> [BL,BR,TL,TR],
        // 3 -> [BR,TR,BL,TL].
        const SLOT_QUADRANT: [[usize; 4]; 4] = [
            [0, 1, 2, 3],
            [1, 0, 3, 2],
            [2, 3, 0, 1],
            [3, 1, 2, 0],
        ];
        let want = SLOT_QUADRANT[near_idx];

        let base_x = self.id.x * 2;
        let base_y = self.id.y * 2;
        let quadrant_of = |id: TileId| match (id.x - base_x, id.y - base_y) {
            (0, 0) => 0, // TL
            (1, 0) => 1, // TR
            (0, 1) => 2, // BL
            (1, 1) => 3, // BR
            _ => unreachable!("child id is not one of self's 4 quadrants"),
        };

        if let Some(children) = &mut self.children {
            for slot in 0..4 {
                if quadrant_of(children[slot].id) == want[slot] {
                    continue;
                }
                let found = (slot + 1..4)
                    .find(|&i| quadrant_of(children[i].id) == want[slot])
                    .expect("all 4 quadrants are present exactly once");
                children.swap(slot, found);
            }
        }
    }

    pub fn center_f32(&self) -> Vec3 {
        Vec3::new(
            self.center.x as f32,
            self.center.y as f32,
            self.center.z as f32,
        )
    }

    pub fn collect_visible_tiles(&self, active_tiles: &mut Vec<(TileId, Vec3, f32)>) {
        if !self.visible {
            return;
        }
        if let Some(children) = &self.children {
            for child in children.iter() {
                child.collect_visible_tiles(active_tiles);
            }
        } else {
            active_tiles.push((self.id, self.center_f32(), self.bounding_radius));
        }
    }

    pub fn collect_renderable_tiles<F: FnMut(&TileId) -> bool>(
        &self,
        active_tiles: &mut Vec<(TileId, Vec3, f32)>,
        is_ready: &mut F,
    ) -> bool {
        if !self.visible {
            return true;
        }

        if let Some(children) = &self.children {
            let mut children_ready = true;
            let mut child_tiles = Vec::new();
            for child in children.iter() {
                if !child.collect_renderable_tiles(&mut child_tiles, is_ready) {
                    children_ready = false;
                    break;
                }
            }

            if children_ready {
                active_tiles.extend(child_tiles);
                return true;
            }
        }

        active_tiles.push((self.id, self.center_f32(), self.bounding_radius));
        is_ready(&self.id)
    }
}

#[cfg(test)]
mod reorder_children_tests {
    use super::*;

    /// A z=5 tile away from the poles, the equator and the prime meridian, so
    /// none of the tangent-frame axes are degenerate.
    fn make_node() -> QuadtreeNode {
        QuadtreeNode::new(TileId { z: 5, x: 10, y: 10 })
    }

    /// For a synthetic eye placed in each of the 4 quadrants relative to the
    /// node's own tangent frame, the near quadrant must land at index 0 and the
    /// diagonally-opposite quadrant at index 3 — the two positions the case
    /// table in [`QuadtreeNode::reorder_children_near_to_far`] fully determines.
    /// The two middle slots are asserted only as a *set*, since their relative
    /// order is deliberately unspecified.
    #[test]
    fn near_and_far_quadrants_land_at_0_and_3() {
        // (east, north, expected index at position 0)
        let cases = [
            (false, true, 0usize),  // TL near
            (true, true, 1usize),   // TR near
            (false, false, 2usize), // BL near
            (true, false, 3usize),  // BR near
        ];

        for (east, north, near_idx) in cases {
            let mut node = make_node();
            node.subdivide();
            let ids_before: Vec<TileId> =
                node.children.as_ref().unwrap().iter().map(|c| c.id).collect();

            let bounds = tile_bounds(&node.id);
            let up = ellipsoid_normal(node.center);
            let (east_axis, north_axis) = tangent_frame(bounds.center_lon(), up);
            let sx = if east { 1.0 } else { -1.0 };
            let sy = if north { 1.0 } else { -1.0 };
            // Far enough that the sign of the dot product is unambiguous; the
            // exact magnitude is irrelevant, only the sign matters.
            let eye = node.center + (east_axis * sx + north_axis * sy + up) * 1.0e7;

            node.reorder_children_near_to_far(eye);
            let ids_after: Vec<TileId> =
                node.children.as_ref().unwrap().iter().map(|c| c.id).collect();

            let far_idx = 3 - near_idx;
            assert_eq!(
                ids_after[0], ids_before[near_idx],
                "east={east} north={north}: near quadrant should land at index 0"
            );
            assert_eq!(
                ids_after[3], ids_before[far_idx],
                "east={east} north={north}: far (diagonally opposite) quadrant \
                 should land at index 3"
            );

            let mid_before: std::collections::HashSet<TileId> = [near_idx, far_idx]
                .iter()
                .fold(
                    (0..4).collect::<std::collections::HashSet<usize>>(),
                    |mut set, i| {
                        set.remove(i);
                        set
                    },
                )
                .into_iter()
                .map(|i| ids_before[i])
                .collect();
            let mid_after: std::collections::HashSet<TileId> =
                [ids_after[1], ids_after[2]].into_iter().collect();
            assert_eq!(
                mid_after, mid_before,
                "east={east} north={north}: middle two slots should hold the \
                 two edge-adjacent quadrants (order unspecified)"
            );
        }
    }

    /// Regression guard for the bug this method's doc comment describes:
    /// `apply_lod` calls `reorder_children_near_to_far` on every update tick a
    /// node stays subdivided, including ticks where `self.children` already
    /// holds a previous call's result — not only the tick right after
    /// `subdivide()`. A first implementation reordered by swapping fixed array
    /// *positions*, which is a self-inverse permutation per case; called twice
    /// in a row with the same `eye` (the common case: nothing moved between
    /// ticks) it silently reverted to the pre-reorder order, and over an even
    /// number of ticks it reverted every time, producing **zero** visible
    /// reordering despite firing on every tile in the tree. This test calls
    /// the method repeatedly with a fixed `eye` and asserts the array stops
    /// changing after the first call.
    #[test]
    fn reorder_is_idempotent_under_repeated_calls_with_same_eye() {
        for (east, north) in [(false, true), (true, true), (false, false), (true, false)] {
            let mut node = make_node();
            node.subdivide();

            let bounds = tile_bounds(&node.id);
            let up = ellipsoid_normal(node.center);
            let (east_axis, north_axis) = tangent_frame(bounds.center_lon(), up);
            let sx = if east { 1.0 } else { -1.0 };
            let sy = if north { 1.0 } else { -1.0 };
            let eye = node.center + (east_axis * sx + north_axis * sy + up) * 1.0e7;

            node.reorder_children_near_to_far(eye);
            let after_first: Vec<TileId> =
                node.children.as_ref().unwrap().iter().map(|c| c.id).collect();

            // Same tick-over-tick call the real update loop makes when the
            // camera hasn't moved. UPDATE_ITERATIONS is 4 in the harness, so
            // simulate a few repeats, not just one.
            for _ in 0..4 {
                node.reorder_children_near_to_far(eye);
                let after_repeat: Vec<TileId> =
                    node.children.as_ref().unwrap().iter().map(|c| c.id).collect();
                assert_eq!(
                    after_repeat, after_first,
                    "east={east} north={north}: reordering with an unchanged eye \
                     must be a no-op after the first call, not an oscillation"
                );
            }
        }
    }

    /// A second regression guard, closer to the real bug's shape: the method
    /// must reach the *same* canonical order regardless of what order the
    /// array happened to be in beforehand, since in production it is never
    /// guaranteed to still be in `subdivide()`'s creation order by the time it
    /// runs.
    #[test]
    fn reorder_reaches_same_result_from_any_starting_order() {
        let bounds_of = |node: &QuadtreeNode| tile_bounds(&node.id);
        let eye_for = |node: &QuadtreeNode, east: bool, north: bool| {
            let up = ellipsoid_normal(node.center);
            let (east_axis, north_axis) = tangent_frame(bounds_of(node).center_lon(), up);
            let sx = if east { 1.0 } else { -1.0 };
            let sy = if north { 1.0 } else { -1.0 };
            node.center + (east_axis * sx + north_axis * sy + up) * 1.0e7
        };

        // Reference: reorder from a fresh subdivide().
        let mut reference = make_node();
        reference.subdivide();
        let eye = eye_for(&reference, true, false); // BR near, arbitrary choice
        reference.reorder_children_near_to_far(eye);
        let expected: Vec<TileId> =
            reference.children.as_ref().unwrap().iter().map(|c| c.id).collect();

        // Same node, but scrambled into every other starting permutation of
        // the 4 children before reordering with the same eye.
        let mut base = make_node();
        base.subdivide();
        let original: Vec<TileId> =
            base.children.as_ref().unwrap().iter().map(|c| c.id).collect();

        let mut permutations: Vec<[usize; 4]> = Vec::new();
        for a in 0..4 {
            for b in 0..4 {
                if b == a {
                    continue;
                }
                for c in 0..4 {
                    if c == a || c == b {
                        continue;
                    }
                    let d = (0..4).find(|i| *i != a && *i != b && *i != c).unwrap();
                    permutations.push([a, b, c, d]);
                }
            }
        }
        assert_eq!(permutations.len(), 24);

        for perm in permutations {
            let mut node = make_node();
            node.subdivide();
            // Force this starting order by reordering position-by-position
            // via swaps against the known original ids.
            if let Some(children) = &mut node.children {
                let ids: Vec<TileId> = perm.iter().map(|&i| original[i]).collect();
                for target in 0..4 {
                    let cur = children.iter().position(|c| c.id == ids[target]).unwrap();
                    children.swap(target, cur);
                }
            }
            let scrambled: Vec<TileId> =
                node.children.as_ref().unwrap().iter().map(|c| c.id).collect();
            assert_eq!(scrambled, perm.iter().map(|&i| original[i]).collect::<Vec<_>>());

            node.reorder_children_near_to_far(eye);
            let got: Vec<TileId> = node.children.as_ref().unwrap().iter().map(|c| c.id).collect();
            assert_eq!(
                got, expected,
                "starting permutation {perm:?} did not converge to the same \
                 canonical order"
            );
        }
    }
}

pub struct QuadtreeManager {
    pub roots: [QuadtreeNode; 4],
    pub lod_factor: f32, // Multiplier for subdivision distance check
    /// Which culling stages run. Lives here — not on a node and not in the
    /// per-frame context — because it survives frames and changes only when a mode
    /// does; the context takes a copy.
    pub pipeline: CullPipeline,
    /// WP4/C measurement switch — see [`LodDistanceMode`]. Defaults to `Centre`;
    /// `wgpu_state.rs` never sets this, so production behaviour is unchanged by its
    /// existence.
    pub lod_distance_mode: LodDistanceMode,
    /// This frame's fog density — WP5. `0.0` (the default) is a true no-op: every
    /// caller that never sets this field gets exactly pre-WP5 behaviour, in both
    /// [`Stage::Fog`] (undecided, never culls) and `QuadtreeNode::apply_lod`'s
    /// relaxation (multiplies by `1.0`). `wgpu_state.rs` is the only production
    /// caller that sets it, recomputed fresh every frame from camera altitude — see
    /// [`super::fog::fog_density_for`].
    pub fog_density: f32,
}

impl Default for QuadtreeManager {
    fn default() -> Self {
        Self::new()
    }
}

impl QuadtreeManager {
    pub fn new() -> Self {
        Self {
            roots: [
                QuadtreeNode::new(TileId { z: 1, x: 0, y: 0 }), // NW
                QuadtreeNode::new(TileId { z: 1, x: 1, y: 0 }), // NE
                QuadtreeNode::new(TileId { z: 1, x: 0, y: 1 }), // SW
                QuadtreeNode::new(TileId { z: 1, x: 1, y: 1 }), // SE
            ],
            lod_factor: 2.0, // Default LOD tuning parameter
            pipeline: CullPipeline::DEFAULT,
            lod_distance_mode: LodDistanceMode::default(),
            fog_density: 0.0,
        }
    }

    /// The camera position is `frustum.eye`: the frustum *is* the camera-relative
    /// frame, so there is no second position argument to get out of step with it.
    pub fn update(&mut self, frustum: &Frustum) {
        let ctx = CullContext::with_pipeline(frustum, self.pipeline)
            .with_lod_distance_mode(self.lod_distance_mode)
            .with_fog_density(self.fog_density);
        for root in self.roots.iter_mut() {
            root.update(&ctx, self.lod_factor);
        }
    }

    pub fn get_visible_tiles(&self) -> Vec<(TileId, Vec3, f32)> {
        let mut active_tiles = Vec::new();
        for root in self.roots.iter() {
            root.collect_visible_tiles(&mut active_tiles);
        }
        active_tiles
    }

    pub fn get_renderable_tiles<F: FnMut(&TileId) -> bool>(
        &self,
        mut is_ready: F,
    ) -> Vec<(TileId, Vec3, f32)> {
        let mut active_tiles = Vec::new();
        for root in self.roots.iter() {
            root.collect_renderable_tiles(&mut active_tiles, &mut is_ready);
        }
        active_tiles
    }
}
