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
use super::surface::{Ellipsoid, SurfaceModel};
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
    (LOD_CALIBRATION_CONSTANT / texture_size_px) * viewport_height_px * target_texel_ratio.sqrt()
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

/// A patch sample at altitude `alt_mm` **megametres** above the ellipsoid, along the
/// surface normal there.
///
/// `alt_mm == 0.0` returns [`surface_point`] itself — bit-for-bit, not "to within
/// rounding" — which is what makes the flat path's `fit_obb` below identical to the
/// one that existed before the surface model was a type parameter. Megametres, not
/// the metres `lon_lat_alt_to_ecef_f64` takes: see [`super::surface`]'s units note.
#[inline]
fn patch_point(lon_deg: f64, lat_deg: f64, alt_mm: f64) -> DVec3 {
    let p = surface_point(lon_deg, lat_deg);
    if alt_mm == 0.0 {
        return p;
    }
    p + ellipsoid_normal(p) * alt_mm
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
///
/// # The altitude span
///
/// The grid is swept at each end of [`SurfaceModel::obb_altitude_span`]. When the
/// two ends coincide — which is the whole of [`Ellipsoid`], whose span is
/// `(0.0, 0.0)` — each grid point is sampled **once**, at altitude 0, via
/// [`patch_point`]'s exact short-circuit. The flat path therefore does exactly the
/// work, in exactly the order, that it did before this parameter existed.
///
/// `surface_center` stays on the ellipsoid whatever the span is: it is the origin
/// the mesh's f32 vertex offsets are taken against (I-2), not a bound. Moving it
/// with relief is a Phase C/D question, not a Phase A one.
fn fit_obb<S: SurfaceModel>(
    b: &TileBounds,
    steps: u32,
    extra: &S::NodeExtra,
) -> (DVec3, f32, OrientedBoundingBox) {
    let (alt_min, alt_max) = S::obb_altitude_span(extra);
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
            let mut accumulate = |alt: f64| {
                let rel = patch_point(lon, lat, alt) - surface_center;
                max_dist_sq = max_dist_sq.max(rel.length_squared());
                let local = DVec3::new(rel.dot(east), rel.dot(north), rel.dot(up));
                min_ext = min_ext.min(local);
                max_ext = max_ext.max(local);
            };
            accumulate(alt_min);
            if alt_max != alt_min {
                accumulate(alt_max);
            }
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

/// [`fit_obb`] at altitude 0 whatever the surface model is.
///
/// One caller — `unstretched_radius`, whose job is to measure the *ground* extent of
/// a tile for the LOD threshold, not the extent of the geometry drawn over it. See
/// the comment at that call site for why Phase D1 keeps it flat.
fn fit_obb_flat(b: &TileBounds, steps: u32) -> (DVec3, f32, OrientedBoundingBox) {
    fit_obb::<Ellipsoid>(b, steps, &())
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
///
/// One cell of the grid: its box, and whatever the surface model carries per patch.
///
/// The payload rides **inside** the existing `obbs` vector rather than in a second
/// one, and that is deliberate rather than tidy: `S::PatchExtra` is `()` for
/// [`Ellipsoid`], so `SubPatch<Ellipsoid>` is layout-identical to a bare
/// [`OrientedBoundingBox`], `SubGrid<Ellipsoid>` keeps its exact size, and
/// [`SubGrid::heap_bytes`] — which `culling::bench_update` reports as bytes per node —
/// does not move by a single byte. A second `Vec<S::PatchExtra>` would have cost the
/// flat path 24 B per gridded node for a vector that can never hold anything.
struct SubPatch<S: SurfaceModel> {
    obb: OrientedBoundingBox,
    extra: S::PatchExtra,
}

/// The surface-model parameter selects which [`fit_obb`] the sub-boxes are built by
/// (Phase D1 makes that sample a height interval) and what each sub-patch carries for
/// the limb test (Phase D2: a scaled-space bounding sphere). Both are `()`-sized for
/// [`Ellipsoid`], so the struct's size and layout are unchanged.
pub struct SubGrid<S: SurfaceModel = Ellipsoid> {
    k: u32,
    /// `k²` cells, indexed `ui · k + vi` — `ui` along λ, `vi` along **Mercator y**,
    /// so `vi = 0` is the sub-patch row at the *north* edge of the tile.
    obbs: Vec<SubPatch<S>>,
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

impl<S: SurfaceModel> SubGrid<S> {
    /// Builds the grid, or `None` when `k < 2` (a 1×1 grid is the node's own OBB).
    fn build(id: &TileId, b: &TileBounds, k: u32, extra: &S::NodeExtra) -> Option<Self> {
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
                let obb = fit_obb::<S>(&sb, 4, extra).2;
                let extra = S::patch_extra(&obb);
                obbs.push(SubPatch { obb, extra });
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
        self.obbs.capacity() * std::mem::size_of::<SubPatch<S>>()
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
                if self.sub_patch_is_occluded(ctx, a_star, ui, vi) {
                    continue;
                }
                let sub_patch_obb = &self.obbs[ui * k + vi].obb;
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
                if self.sub_patch_is_occluded(ctx, a_star, ui, vi) {
                    continue;
                }
                if ctx.frustum.intersects_obb(&self.obbs[ui * k + vi].obb) {
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

    /// Is sub-patch `(ui, vi)` of a column with this `A*` entirely behind the limb?
    ///
    /// The single place where the two directions meet: `vi` counts south along
    /// Mercator y (`vi = 0` is the north row, matching `obbs`), while `lat` is
    /// stored increasing in φ. `lo = k − 1 − vi` is that conversion, and it leaves
    /// the span itself in the `[low, high]` order every `*_span_max` expects.
    ///
    /// **Phase D2.** This used to call `span_is_occluded` directly; it now dispatches
    /// to the surface model, for the same reason the node-level test does. Phase C
    /// deliberately left it alone (`Heightfield`'s node-level test was still the flat
    /// one, so a split here would have been half a change); with D2 the sub-patch test
    /// is the same unsound rectangle collapse, one level finer, and by I-7 a false
    /// negative here deletes the same subtree.
    #[inline]
    fn sub_patch_is_occluded(&self, ctx: &CullContext, a_star: f64, ui: usize, vi: usize) -> bool {
        let lo = self.k as usize - 1 - vi;
        S::sub_patch_is_occluded(
            &ctx.horizon,
            a_star,
            &[self.lat[lo].0, self.lat[lo + 1].0],
            &[self.lat[lo].1, self.lat[lo + 1].1],
            &self.obbs[ui * self.k as usize + vi].extra,
        )
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
    /// **D3** — the tile is behind a *mountain*, not behind the planet
    /// ([`super::terrain_occlusion`], `docs/terrain-plan.md` §3.3). `Cull` when the
    /// circumsphere of the node's D1 box lies entirely below the guaranteed ridge this
    /// frame's occlusion march found in front of it; `Undecided` otherwise, including
    /// whenever the march is inactive (terrain off, camera above the altitude gate, or
    /// no `CullContext::terrain` at all — which is always, on the flat arm). Never
    /// `Keep`: like the limb and frustum stages it proves invisibility and nothing else.
    ///
    /// # Sound, unlike [`Stage::Fog`] — and that is why it sits in a default pipeline
    ///
    /// The two are adjacent in this `enum` and their contracts are opposite, so this is
    /// worth stating where both are in view. `Fog` **deliberately discards geometry that
    /// is genuinely visible**; that is what atmospheric fog culling is, it makes false
    /// negatives non-zero by design, and it is fenced out of [`CullPipeline::DEFAULT`]
    /// for exactly that reason. `TerrainOcclusion` discards only what it has proved
    /// invisible. It therefore belongs in [`CullPipeline::TERRAIN_DEFAULT`] — the
    /// pipeline the terrain arm of [`super::any::AnyQuadtree`] runs *and* the one the
    /// terrain harness measures — and is held to FN = 0 by
    /// `testing::terrain::test_terrain_occlusion::d3_never_hides_a_visible_vertex`
    /// rather than being kept away from the thing that would notice.
    ///
    /// # Why it is placed second, right after [`Stage::Horizon`]
    ///
    /// Not a preference — the only two slots where a stage that returns `Undecided` can
    /// still run. [`Stage::NodeFrustum`] answers `Keep` outright for every node without
    /// a sub-grid, and [`Stage::SubPatchGrid`] answers `Keep` or `Cull` for every node
    /// with one, so a stage appended *after* those never executes (this is the same
    /// structural fact [`CullPipeline::DEFAULT`]'s doc comment records as "the final rule
    /// is never reached under `DEFAULT`"). Of the two live slots, the limb test is both
    /// cheaper and more selective, so it keeps the first.
    TerrainOcclusion,
}

impl Stage {
    /// Runs this stage. `&self` on the node, never `&mut`: see
    /// [`QuadtreeNode::update`] for why that signature is load-bearing.
    #[inline]
    fn run<S: SurfaceModel>(self, node: &QuadtreeNode<S>, ctx: &CullContext) -> StageVerdict {
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
                let dist_m = node.obb.distance_to_point(ctx.frustum.eye) * MEGAMETERS_TO_METERS;
                if cesium_fog(dist_m, ctx.fog_density) >= 1.0 {
                    StageVerdict::Cull
                } else {
                    StageVerdict::Undecided
                }
            }
            Stage::TerrainOcclusion => {
                let Some(horizon) = ctx.terrain else {
                    return StageVerdict::Undecided;
                };
                // The node's own D1 box, which by I-1' contains every drawable point of
                // the tile, skirts included. The **box**, not `bounding_radius` and not
                // a sphere around it: a tile's box is a flat slab tangent to the globe,
                // and collapsing it to a sphere claims the tile could be overhead. See
                // `TerrainHorizon::occludes` for the measured difference (+11.8° against
                // −1.1° on the same z12 tile).
                if horizon.occludes(&node.obb, &tile_bounds(&node.id)) {
                    StageVerdict::Cull
                } else {
                    StageVerdict::Undecided
                }
            }
        }
    }
}

/// The most stages a pipeline can hold — one of each [`Stage`].
///
/// # Raised from 4 to 5 by D3, and what that cost the flat path
///
/// [`CullPipeline::keeps`] loops over `0..MAX_STAGES` with a `break` at `len`
/// deliberately, so the trip count is a constant and the three-way `match` in
/// [`Stage::run`] unrolls into one specialised copy per slot (measured: 6.9 µs against
/// 7.3 µs for a slice loop — see that function's doc comment). Raising the constant adds
/// a **fifth** such copy, which `CullPipeline::DEFAULT` (three stages) breaks out of
/// before reaching. The cost is therefore code size, not frame time, and `bench_update`
/// confirms it: mean `QuadtreeManager::update` over the 204 bench poses is unchanged
/// inside run-to-run variance, and `size_of::<QuadtreeNode<Ellipsoid>>()` cannot move
/// because a pipeline is not stored per node. `CullPipeline` itself grows from 4 B to
/// 6 B; it lives on [`QuadtreeManager`] and is copied by value into [`CullContext`]
/// once per frame.
pub const MAX_STAGES: usize = 5;

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
    pub const DEFAULT_WITH_FOG: CullPipeline = CullPipeline::of(&[
        Stage::Horizon,
        Stage::NodeFrustum,
        Stage::SubPatchGrid,
        Stage::Fog,
    ]);

    /// **The terrain arm's default** — `DEFAULT` with [`Stage::TerrainOcclusion`]
    /// inserted second, right behind the limb test. D3 of `docs/terrain-plan.md` §7.
    ///
    /// # This one *is* sound, and is measured as such
    ///
    /// The opposite of [`Self::DEFAULT_WITH_FOG`] in every respect that matters. Fog
    /// removes geometry that is genuinely visible, so it may never appear in a pipeline
    /// the culling harness measures. Terrain occlusion removes only geometry it has
    /// proved invisible, so it belongs in the default the terrain engine runs *and* in
    /// the one the terrain harness measures, and
    /// `testing::terrain::test_terrain_occlusion` holds it to FN = 0 against the drawn
    /// mesh. Putting it here and then measuring something else would defeat the point.
    ///
    /// `CullPipeline::DEFAULT` is untouched and stays what the flat globe runs and what
    /// every sweep in `src/testing/culling/` is proved against — a new `Stage` variant
    /// changes nothing about a pipeline that does not list it.
    ///
    /// # Second, not last
    ///
    /// See [`Stage::TerrainOcclusion`]: `NodeFrustum` and `SubPatchGrid` between them
    /// settle *every* node outright, so a fourth stage appended after them is
    /// unreachable. The two live slots are first and second, and the limb test — 25 f64
    /// flops and roughly half the globe — earns the first.
    pub const TERRAIN_DEFAULT: CullPipeline = CullPipeline::of(&[
        Stage::Horizon,
        Stage::TerrainOcclusion,
        Stage::NodeFrustum,
        Stage::SubPatchGrid,
    ]);

    /// [`Self::TERRAIN_DEFAULT`] plus [`Stage::Fog`] — what `wgpu_state.rs` runs on the
    /// terrain arm, exactly as [`Self::DEFAULT_WITH_FOG`] is what it runs on the flat
    /// one. **Not for the harness**, for fog's reason and fog's reason only.
    pub const TERRAIN_DEFAULT_WITH_FOG: CullPipeline = CullPipeline::of(&[
        Stage::Horizon,
        Stage::TerrainOcclusion,
        Stage::NodeFrustum,
        Stage::SubPatchGrid,
        Stage::Fog,
    ]);

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
    fn keeps<S: SurfaceModel>(&self, node: &QuadtreeNode<S>, ctx: &CullContext) -> bool {
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
pub struct CullContext<'a> {
    pub frustum: Frustum,
    pub horizon: HorizonCamera,
    /// This frame's terrain occlusion march — **D3**, and the only borrowed thing in
    /// this struct.
    ///
    /// A shared reference rather than a value on purpose. The march is a
    /// [`AZIMUTH_SECTORS`](super::terrain_occlusion::AZIMUTH_SECTORS) ×
    /// [`RANGE_RINGS`](super::terrain_occlusion::RANGE_RINGS) grid, ~6 kB, and this
    /// struct is `Copy` and is rebuilt every frame on **both** arms — storing it by
    /// value would charge the flat path a memcpy for a structure it can never fill.
    /// `Option<&_>` is one niche-optimised word and is `None` on the flat arm, where
    /// `QuadtreeManager::refresh_terrain_horizon` is never called.
    ///
    /// This is the one place the `NodeExtraSource` argument (a trait on the outside,
    /// because the height cache is `&mut` and lives on `TileSystem`) does not apply: the
    /// march is a finished, immutable, purely geometric structure by the time culling
    /// starts, so there is nothing to keep out of a per-node read.
    pub terrain: Option<&'a super::terrain_occlusion::TerrainHorizon>,
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
    /// Maximum zoom level to refine down to. Defaults to [`MAX_ZOOM`].
    pub max_zoom: u8,
}

impl<'a> CullContext<'a> {
    /// The shipped configuration, [`CullPipeline::DEFAULT`].
    pub fn new(frustum: &Frustum) -> Self {
        Self::with_pipeline(frustum, CullPipeline::DEFAULT)
    }

    pub fn with_pipeline(frustum: &Frustum, pipeline: CullPipeline) -> Self {
        Self {
            frustum: *frustum,
            horizon: HorizonCamera::new(frustum.eye),
            terrain: None,
            pipeline,
            lod_distance_mode: LodDistanceMode::default(),
            fog_density: 0.0,
            max_zoom: MAX_ZOOM,
        }
    }

    /// D3 only — this frame's occlusion march. `None` (the default) is what every
    /// existing caller and the whole flat path get, and it makes
    /// [`Stage::TerrainOcclusion`] a single null check.
    pub fn with_terrain_horizon(
        mut self,
        horizon: Option<&'a super::terrain_occlusion::TerrainHorizon>,
    ) -> Self {
        self.terrain = horizon;
        self
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

    /// Sets the maximum zoom level for quadtree subdivision.
    pub fn with_max_zoom(mut self, max_zoom: u8) -> Self {
        self.max_zoom = max_zoom;
        self
    }
}

/// One node of the tile quadtree.
///
/// # The surface-model parameter
///
/// `S` defaults to [`Ellipsoid`], whose [`SurfaceModel::NodeExtra`] and
/// [`SurfaceModel::PatchExtra`] are both `()`. Both payload fields are therefore
/// zero-sized and `QuadtreeNode` (i.e. `QuadtreeNode<Ellipsoid>`) is still exactly
/// 192 B — three cache lines — with `patch` still 64. See [`super::surface`].
pub struct QuadtreeNode<S: SurfaceModel = Ellipsoid> {
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
    pub sub_grid: Option<Box<SubGrid<S>>>,
    /// The eight trig constants of this tile's rectangle, for the horizon test.
    pub patch: TilePatch<S>,
    /// Whatever the surface model needs per node — nothing, in flat mode.
    pub extra: S::NodeExtra,
    pub visible: bool,
    pub children: Option<Box<[QuadtreeNode<S>; 4]>>,
}

impl QuadtreeNode<Ellipsoid> {
    /// The flat globe's node. See [`QuadtreeNode::for_surface`] for the generic
    /// form; this concrete wrapper is what keeps `QuadtreeNode::new(id)` resolving
    /// at call sites that never mention a surface model.
    pub fn new(id: TileId) -> Self {
        Self::for_surface(id)
    }
}

impl<S: SurfaceModel> QuadtreeNode<S> {
    pub fn for_surface(id: TileId) -> Self {
        Self::for_surface_with(id, <S::NodeExtra as Default>::default())
    }

    /// A node whose surface payload is known at construction — Phase D1.
    ///
    /// Everything derived from that payload (the two boxes, the bounding radii, the
    /// sub-grid and the patch) is fitted here, once. The flat path reaches this
    /// through [`Self::for_surface`] with `extra = ()` and compiles to the loop it
    /// always had.
    pub fn for_surface_with(id: TileId, extra: S::NodeExtra) -> Self {
        // I-5: the single source of tile bounds, shared with `TileMesh::generate`.
        let bounds = tile_bounds(&id);
        let (center, bounding_radius, obb) = fit_obb::<S>(&bounds, obb_grid_steps(id.z), &extra);

        // Deliberately measured on the ***un*-stretched** rectangle: a polar row's
        // true ground extent, not its pull to ±90°. Kept as-is (§8.4) — it makes
        // polar caps subdivide later, which is an accepted FP source and never an
        // FN one, because subdividing late only keeps a coarser tile that still
        // covers the ground. Nothing about it is an LOD parameter; it is the size
        // of a geometric object, and the threshold derived from it is
        // `subdivide_dist`.
        //
        // **Phase D1 deliberately keeps this on the zero-altitude span** — see
        // [`fit_obb_flat`]. Relief does grow the tile's true extent, and feeding that
        // growth in here would grow `subdivide_dist` with it and refine terrain mode
        // deeper than flat mode at the same camera distance. That is a real and
        // probably desirable effect, but it is an *LOD* change, it is `apply_lod`'s
        // dispatch site in `docs/terrain-plan.md` §1's table, and that site is Phase
        // E1 (`max(imagery_dist, terrain_dist)`, from the tile's measured deviation
        // from its parent). D1 is a culling change; smuggling an LOD change in with
        // it would make the tile-count delta in the D1 captures unreadable.
        let raw = tile_bounds_unstretched(&id);
        let (_, unstretched_radius, _) = fit_obb_flat(&raw, 2);

        let sub_grid =
            SubGrid::<S>::build(&id, &bounds, sub_boxes_per_axis(id.z), &extra).map(Box::new);

        QuadtreeNode {
            id,
            center,
            bounding_radius,
            unstretched_radius,
            patch: TilePatch::<S>::for_surface(&bounds, S::patch_extra(&obb)),
            obb,
            sub_grid,
            extra,
            visible: false,
            children: None,
        }
    }

    /// Re-fits everything derived from the surface payload, in place — Phase D1.
    ///
    /// A node is created long before its height tile lands, so its interval is a
    /// conservative inheritance ([`SurfaceModel::child_extra`]) until real data
    /// arrives and tightens it. This is how the tightening is applied without
    /// dropping the subtree: the children keep their own intervals and their own
    /// LOD state, and are re-derived by the same traversal that calls this.
    ///
    /// Returns immediately when the payload has not changed, which for
    /// [`Ellipsoid`] is *always* — `()` equals `()`, the comparison folds to a
    /// constant and the whole body is dead code the flat path never reaches.
    pub fn set_extra(&mut self, extra: S::NodeExtra) {
        if extra == self.extra {
            return;
        }
        let bounds = tile_bounds(&self.id);
        let (center, bounding_radius, obb) =
            fit_obb::<S>(&bounds, obb_grid_steps(self.id.z), &extra);
        // `unstretched_radius` is not re-fitted: it does not depend on `extra` at all
        // (see [`Self::for_surface_with`]), so there is nothing here for a payload
        // change to move.
        self.center = center;
        self.bounding_radius = bounding_radius;
        self.patch = TilePatch::<S>::for_surface(&bounds, S::patch_extra(&obb));
        self.obb = obb;
        self.sub_grid =
            SubGrid::<S>::build(&self.id, &bounds, sub_boxes_per_axis(self.id.z), &extra)
                .map(Box::new);
        self.extra = extra;
    }

    /// Heap bytes this node hangs off itself (not counting children).
    pub fn sub_grid_heap_bytes(&self) -> usize {
        self.sub_grid
            .as_ref()
            .map(|g| std::mem::size_of::<SubGrid<S>>() + g.heap_bytes())
            .unwrap_or(0)
    }

    /// Creates the four children, each inheriting this node's surface payload through
    /// [`SurfaceModel::child_extra`].
    ///
    /// Inheritance is the **only** source a new child has: the quadtree runs before
    /// anything has been fetched for a tile it has just decided to look at, so a
    /// child's own height data cannot exist yet by construction. `child_extra` is
    /// therefore where Phase D1's soundness lives, and why it widens rather than
    /// copies — see its doc comment. Real data replaces the inherited interval later,
    /// through [`Self::set_extra`].
    ///
    /// For [`Ellipsoid`] `child_extra` returns `()` and this is the function it always
    /// was.
    pub fn subdivide(&mut self) {
        let z = self.id.z + 1;
        let x = self.id.x * 2;
        let y = self.id.y * 2;

        let child =
            |id: TileId| QuadtreeNode::<S>::for_surface_with(id, S::child_extra(&self.extra, &id));

        self.children = Some(Box::new([
            child(TileId { z, x, y }),        // Top-Left
            child(TileId { z, x: x + 1, y }), // Top-Right
            child(TileId { z, x, y: y + 1 }), // Bottom-Left
            child(TileId {
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
        if should_be_subdivided && self.id.z < ctx.max_zoom {
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
        const SLOT_QUADRANT: [[usize; 4]; 4] =
            [[0, 1, 2, 3], [1, 0, 3, 2], [2, 3, 0, 1], [3, 1, 2, 0]];
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

/// Where a node's surface payload comes from — Phase D1's feed, and the whole of it.
///
/// # Why this is a trait and not a field on [`CullContext`]
///
/// The quadtree has no access to the height cache and must not acquire one: the cache
/// is `&mut` (it promotes in an LRU), it lives on `TileSystem`, and putting a borrow
/// of it into [`CullContext`] would give that `Copy`, per-frame, per-node-read struct
/// a lifetime parameter for the benefit of one surface model. A trait implemented on
/// the *outside* keeps the dependency pointing the right way: `globe::terrain` knows
/// about the quadtree, the quadtree knows nothing about terrain.
///
/// # Why it is a separate pass and not part of `update`
///
/// The payload is needed at node *construction*, deep inside `apply_lod`'s recursion,
/// and threading a source through `update → apply_lod → subdivide` would put an extra
/// argument on the hottest path in the culler for a model that is usually off. Instead
/// a new child inherits ([`SurfaceModel::child_extra`], sound but loose) and this pass
/// — run once per frame, before `update`, over the tree the previous frame left —
/// tightens every node whose data has since arrived. A node is therefore loose for at
/// most the frame it was born in, which costs false positives and never a false
/// negative.
///
/// Nothing implements this for [`Ellipsoid`] and nothing needs to: the flat path never
/// calls [`QuadtreeManager::refresh_extras`].
pub trait NodeExtraSource<S: SurfaceModel> {
    /// The payload for `id`, or `None` when there is nothing better than inheritance.
    ///
    /// `&self`: the refresh walk visits every node in the tree every frame and must
    /// not reorder an LRU by looking.
    fn extra_for(&self, id: &TileId) -> Option<S::NodeExtra>;
}

/// One node of [`QuadtreeManager::refresh_extras`]'s top-down walk.
///
/// Top-down because inheritance is: a node with no data of its own takes its parent's
/// *current* interval, which this pass may itself have just tightened.
fn refresh_node<S: SurfaceModel, X: NodeExtraSource<S>>(
    node: &mut QuadtreeNode<S>,
    parent: Option<S::NodeExtra>,
    src: &X,
) {
    let want = src.extra_for(&node.id).unwrap_or_else(|| match parent {
        Some(p) => S::child_extra(&p, &node.id),
        // A root. Nothing above it to inherit from, so it falls back to the model's
        // own worst case — for `Heightfield`, the whole range the Earth occupies.
        None => <S::NodeExtra as Default>::default(),
    });
    node.set_extra(want);

    let mine = node.extra;
    if let Some(children) = &mut node.children {
        for child in children.iter_mut() {
            refresh_node(child, Some(mine), src);
        }
    }
}

/// One node of [`QuadtreeManager::refresh_terrain_horizon`]'s occluder walk — **D3**.
///
/// Three outcomes, and the middle one is what keeps the walk bounded:
///
/// * `Skip` — the node's near edge is already beyond the march's range, so neither it nor
///   anything under it can touch a cell. This is the prune that stops the walk being
///   proportional to the whole tree.
/// * `Descend` with children — the node is angularly larger than the cells it sits on, so
///   its children have tighter (higher) floors over the same ground. A node whose
///   children were culled or never created falls through to `Stamp`, which is correct
///   rather than a fallback: it is then a leaf of the partition.
/// * `Stamp` — record this node's floor in every cell its footprint can reach.
///
/// The union of the stamped nodes is the whole globe, which is the property
/// [`TerrainHorizon::stamp`](super::terrain_occlusion::TerrainHorizon::stamp) relies on:
/// a cell's floor is the minimum over everything stamped into it, so a cell that is only
/// partly covered would keep a floor that is too high, and a floor that is too high is
/// the one error that loses geometry.
///
/// A node's **circumsphere** is what gets stamped — the box's, not `bounding_radius`'s,
/// for the same reason [`Stage::TerrainOcclusion`] queries with it: it is the sphere that
/// provably contains the node's ground. Over-covering a cell can only pull its floor
/// down, which is the conservative direction.
fn stamp_occluders<S: SurfaceModel>(
    node: &QuadtreeNode<S>,
    horizon: &mut super::terrain_occlusion::TerrainHorizon,
) {
    use super::terrain_occlusion::OccluderStep;

    let bounds = tile_bounds(&node.id);
    match horizon.classify(&bounds) {
        OccluderStep::Skip => {}
        OccluderStep::Descend => match &node.children {
            Some(children) => {
                for c in children.iter() {
                    stamp_occluders(c, horizon);
                }
            }
            None => stamp_node(node, &bounds, horizon),
        },
        OccluderStep::Stamp => stamp_node(node, &bounds, horizon),
    }
}

/// Stamps one node's `4 × 4` occluder grid, sub-cell by sub-cell — **D3**.
///
/// # Why each sub-cell is placed by the box's own axes
///
/// The node's box is built in the [`tangent_frame`] at its centre: `half_axes[0]` runs
/// east along the tile's `u`, `half_axes[1]` north against its Mercator `v`, and
/// `half_axes[2]` is the relief. A sub-cell's centre is therefore the box centre plus a
/// linear step along the first two, which costs six multiplies instead of a
/// `lon_lat_to_ecef` per sub-cell — sixteen of those per node, on every node in range,
/// every frame.
///
/// The linear step is not exact: the real sub-rectangle centres bow toward the eye by the
/// patch's sagitta, `≈ r²/2R`, which is 1.7 m on a z12 tile and a few hundred metres on a
/// z8 one. That error is added straight back onto the sub-cell's radius, because the
/// direction that matters here is **outward**: a stamp that covers more cells than it
/// should can only pull those cells' floors *down* (they take the minimum over everything
/// stamped into them, the true contributors included), while a stamp that covers fewer
/// leaves a cell holding a floor that is too high — which is a false negative, and by I-7
/// a subtree.
fn stamp_node<S: SurfaceModel>(
    node: &QuadtreeNode<S>,
    bounds: &TileBounds,
    horizon: &mut super::terrain_occlusion::TerrainHorizon,
) {
    use crate::globe::terrain::heightfield::OCCLUDER_GRID;

    let floors = S::occluder_floor(&node.extra);
    let h = &node.obb.half_axes;
    let east = DVec3::new(h[0].x as f64, h[0].y as f64, h[0].z as f64);
    let north = DVec3::new(h[1].x as f64, h[1].y as f64, h[1].z as f64);

    let n = OCCLUDER_GRID as f64;

    // The `OCCLUDER_GRID + 1` row boundaries, hoisted — §7c's first optimisation.
    //
    // `sub_bounds` derives two of these per sub-cell, and consecutive rows share one, so
    // the loop below used to call `web_mercator_y_to_lat_f64` **32 times per node** for 5
    // distinct values. It is an `atan` of a `sinh` and it was the largest single line in
    // the march: hoisting it took the walk from 174 µs to 115 µs on the `alps_inn_valley`
    // tree. The expressions and the `t` values are `sub_bounds`' own, so every rectangle
    // built below is bit-identical to what it returned.
    //
    // Two more of these calls per row were **dead** — a `lat0`/`lat1` pair computed and
    // then discarded through a `let _ = (lat0, lat1);`, left behind when this loop stopped
    // deriving the rectangle by hand and started calling `sub_bounds`. That is 8 of the 40
    // gone for nothing at all.
    let mut lat = [0.0f64; OCCLUDER_GRID + 1];
    let last_row = (1_u32 << node.id.z) - 1;
    for (j, slot) in lat.iter_mut().enumerate() {
        *slot = if j == 0 && node.id.y == 0 {
            90.0
        } else if j == OCCLUDER_GRID && node.id.y == last_row {
            -90.0
        } else {
            web_mercator_y_to_lat_f64(node.id.y as f64 + j as f64 / n, node.id.z)
        };
    }

    for j in 0..OCCLUDER_GRID {
        // `v` counts south, `half_axes[1]` points north.
        let ty = 1.0 - 2.0 * (j as f64 + 0.5) / n;
        for i in 0..OCCLUDER_GRID {
            let tx = 2.0 * (i as f64 + 0.5) / n - 1.0;
            let centre = node.obb.center + east * tx + north * ty;
            // The sub-cell's own ground rectangle, on the same `sub_bounds`
            // parameterisation the mesh and the sub-patch grid use — not a scaled copy of
            // the node's box, whose circumradius stops describing a ground footprint at
            // all once a tile spans degrees.
            let sb = TileBounds {
                lon_min: bounds.lon_min + (i as f64 / n) * (bounds.lon_max - bounds.lon_min),
                lon_max: bounds.lon_min
                    + ((i as f64 + 1.0) / n) * (bounds.lon_max - bounds.lon_min),
                lat_min: lat[j + 1],
                lat_max: lat[j],
            };
            let (gamma, gr, near) = horizon.extent_of_pub(&sb);
            horizon.stamp(
                centre,
                gamma,
                gr,
                near,
                floors[j * OCCLUDER_GRID + i] as f64,
            );
        }
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
            let ids_before: Vec<TileId> = node
                .children
                .as_ref()
                .unwrap()
                .iter()
                .map(|c| c.id)
                .collect();

            let bounds = tile_bounds(&node.id);
            let up = ellipsoid_normal(node.center);
            let (east_axis, north_axis) = tangent_frame(bounds.center_lon(), up);
            let sx = if east { 1.0 } else { -1.0 };
            let sy = if north { 1.0 } else { -1.0 };
            // Far enough that the sign of the dot product is unambiguous; the
            // exact magnitude is irrelevant, only the sign matters.
            let eye = node.center + (east_axis * sx + north_axis * sy + up) * 1.0e7;

            node.reorder_children_near_to_far(eye);
            let ids_after: Vec<TileId> = node
                .children
                .as_ref()
                .unwrap()
                .iter()
                .map(|c| c.id)
                .collect();

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
            let after_first: Vec<TileId> = node
                .children
                .as_ref()
                .unwrap()
                .iter()
                .map(|c| c.id)
                .collect();

            // Same tick-over-tick call the real update loop makes when the
            // camera hasn't moved. UPDATE_ITERATIONS is 4 in the harness, so
            // simulate a few repeats, not just one.
            for _ in 0..4 {
                node.reorder_children_near_to_far(eye);
                let after_repeat: Vec<TileId> = node
                    .children
                    .as_ref()
                    .unwrap()
                    .iter()
                    .map(|c| c.id)
                    .collect();
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
        let expected: Vec<TileId> = reference
            .children
            .as_ref()
            .unwrap()
            .iter()
            .map(|c| c.id)
            .collect();

        // Same node, but scrambled into every other starting permutation of
        // the 4 children before reordering with the same eye.
        let mut base = make_node();
        base.subdivide();
        let original: Vec<TileId> = base
            .children
            .as_ref()
            .unwrap()
            .iter()
            .map(|c| c.id)
            .collect();

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
            let scrambled: Vec<TileId> = node
                .children
                .as_ref()
                .unwrap()
                .iter()
                .map(|c| c.id)
                .collect();
            assert_eq!(
                scrambled,
                perm.iter().map(|&i| original[i]).collect::<Vec<_>>()
            );

            node.reorder_children_near_to_far(eye);
            let got: Vec<TileId> = node
                .children
                .as_ref()
                .unwrap()
                .iter()
                .map(|c| c.id)
                .collect();
            assert_eq!(
                got, expected,
                "starting permutation {perm:?} did not converge to the same \
                 canonical order"
            );
        }
    }

    #[test]
    fn subdivision_caps_at_max_zoom() {
        let mut qt = QuadtreeManager::new();
        qt.max_zoom = 3;
        // Place camera right on top of root 0
        let frustum = Frustum::planes_only([DVec3::ZERO; 4], qt.roots[0].center);
        qt.lod_factor = 1000.0; // Force maximum subdivision
        qt.update(&frustum);
        let visible = qt.get_visible_tiles();
        assert!(!visible.is_empty());
        for (id, _, _) in visible {
            assert!(id.z <= 3, "Tile zoom {} exceeded max_zoom 3", id.z);
        }
    }
}

/// The four root nodes and the per-frame knobs the traversal reads.
///
/// `S` defaults to [`Ellipsoid`] — see [`QuadtreeNode`] and [`super::surface`].
pub struct QuadtreeManager<S: SurfaceModel = Ellipsoid> {
    pub roots: [QuadtreeNode<S>; 4],
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
    /// Maximum zoom level to refine down to. Defaults to [`MAX_ZOOM`].
    pub max_zoom: u8,
    /// **D3** — this frame's occlusion march, or `None` when there is none.
    ///
    /// `None` on the flat arm always: [`Self::refresh_terrain_horizon`] is the only thing
    /// that sets it and nothing calls it for [`Ellipsoid`], whose
    /// [`SurfaceModel::occluder_floor`] is `-∞` anyway. One `Option<Box<_>>` word per
    /// *manager* (there are four in the process, not four per node), read once per frame
    /// when the context is built.
    terrain_horizon: Option<Box<super::terrain_occlusion::TerrainHorizon>>,
}

impl Default for QuadtreeManager<Ellipsoid> {
    fn default() -> Self {
        Self::new()
    }
}

impl QuadtreeManager<Ellipsoid> {
    /// The flat globe's quadtree. See [`QuadtreeManager::for_surface`] for the
    /// generic form; this concrete wrapper is what keeps `QuadtreeManager::new()`
    /// resolving at call sites that never mention a surface model.
    pub fn new() -> Self {
        Self::for_surface()
    }
}

impl<S: SurfaceModel> QuadtreeManager<S> {
    pub fn for_surface() -> Self {
        Self {
            roots: [
                QuadtreeNode::<S>::for_surface(TileId { z: 1, x: 0, y: 0 }), // NW
                QuadtreeNode::<S>::for_surface(TileId { z: 1, x: 1, y: 0 }), // NE
                QuadtreeNode::<S>::for_surface(TileId { z: 1, x: 0, y: 1 }), // SW
                QuadtreeNode::<S>::for_surface(TileId { z: 1, x: 1, y: 1 }), // SE
            ],
            lod_factor: 2.0, // Default LOD tuning parameter
            pipeline: CullPipeline::DEFAULT,
            lod_distance_mode: LodDistanceMode::default(),
            fog_density: 0.0,
            max_zoom: MAX_ZOOM,
            terrain_horizon: None,
        }
    }

    /// Re-derives every node's surface payload from `src` — Phase D1.
    ///
    /// Call once per frame, **before** [`Self::update`]. See [`NodeExtraSource`] for
    /// why this is a separate pass rather than an argument threaded through the
    /// traversal, and [`QuadtreeNode::set_extra`] for what a changed payload costs
    /// (a box, a radius and a `k × k` sub-grid — paid once, the frame the tile's
    /// heights land).
    ///
    /// Never called on the flat path.
    pub fn refresh_extras<X: NodeExtraSource<S>>(&mut self, src: &X) {
        for root in self.roots.iter_mut() {
            refresh_node(root, None, src);
        }
    }

    /// Rebuilds this frame's terrain occlusion march from the tree as it stands —
    /// **D3**, `docs/terrain-plan.md` §3.3.
    ///
    /// Call after [`Self::refresh_extras`] and before [`Self::update`], with the camera
    /// this frame will cull against. Never called for [`Ellipsoid`].
    ///
    /// # Why the occluders are the tree's own nodes
    ///
    /// Because the tree is the only place a *sound lower bound on the terrain surface*
    /// already exists. Every node carries one ([`SurfaceModel::occluder_floor`], which
    /// for `Heightfield` is D1's `HeightBounds::floor`), it is derived from B3's min/max
    /// mip with the cell range rounded outward, and where no data has arrived it is the
    /// parent's widened by the measured margin — loose, and loose downward, which
    /// occludes less rather than more. Querying the height cache again here would have
    /// meant deriving and measuring a *second* margin for the same quantity, in the
    /// other direction, with nothing to gain.
    ///
    /// # Why it is a partition and why that matters
    ///
    /// The walk descends from the roots and stops at every node it does not descend into
    /// — real leaves, nodes small enough that descending would not refine any cell they
    /// touch, and nodes the previous frame culled (which keep their payload and simply
    /// have no children). The stopping set therefore **covers the globe exactly once**,
    /// so every cell within range receives at least one stamp and no cell is left with
    /// an unearned floor. A cell that somehow received none stays at `+∞` and
    /// [`TerrainHorizon::finish`](super::terrain_occlusion::TerrainHorizon::finish)
    /// reads that as "nothing guaranteed", never as "guaranteed high".
    ///
    /// # The tree is one frame stale, and that is fine
    ///
    /// The occluders come from the tree the *previous* frame left behind, exactly as
    /// `refresh_extras`' inheritance does. A node's floor is a statement about the ground
    /// under a fixed footprint; it does not go stale when the camera moves. What can be
    /// stale is the tree's *depth* — a region refined to z15 last frame may want z16 this
    /// frame — and a coarser node has a *lower* floor, so staleness occludes less.
    pub fn refresh_terrain_horizon(
        &mut self,
        eye: DVec3,
        cam_alt: f64,
        cfg: &super::terrain_occlusion::TerrainOcclusionConfig,
    ) {
        let mut horizon = super::terrain_occlusion::TerrainHorizon::begin(eye, cam_alt, cfg);
        if horizon.is_active() {
            for root in self.roots.iter() {
                stamp_occluders(root, &mut horizon);
            }
            horizon.finish();
        }
        self.terrain_horizon = Some(Box::new(horizon));
    }

    /// Forgets this frame's march. The next [`Self::update`] runs D1+D2 only.
    pub fn clear_terrain_horizon(&mut self) {
        self.terrain_horizon = None;
    }

    /// This frame's march, if one was built — a read-only window for tests and debug
    /// readouts. `None` on the flat arm always.
    pub fn terrain_horizon(&self) -> Option<&super::terrain_occlusion::TerrainHorizon> {
        self.terrain_horizon.as_deref()
    }

    /// The camera position is `frustum.eye`: the frustum *is* the camera-relative
    /// frame, so there is no second position argument to get out of step with it.
    pub fn update(&mut self, frustum: &Frustum) {
        let ctx = CullContext::with_pipeline(frustum, self.pipeline)
            .with_lod_distance_mode(self.lod_distance_mode)
            .with_fog_density(self.fog_density)
            .with_terrain_horizon(self.terrain_horizon.as_deref())
            .with_max_zoom(self.max_zoom);
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
