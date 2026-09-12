#![allow(clippy::type_complexity)]
//! The tile quadtree and its per-node visibility decision.
//!
//! Derivation: `docs/culling-math.md` §9 is the consolidated algorithm this file
//! implements. Per node, in order:
//!
//! 1. **Horizon** — exact, f64, ~25 flops ([`super::horizon`]). Cheapest *and* most
//!    selective: roughly half the globe is below the limb at any time, and the whole
//!    back hemisphere falls to one comparison at the coarsest level.
//! 2. **Frustum** — the four side planes, camera-relative, ~92 flops
//!    ([`super::bounding_volume`]).
//! 3. LOD / hysteresis, unchanged.
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
//!   `q·c − 1` are the same expression up to a positive factor (Theorem 3.4).
//! * The near and far planes, and the `vh_mag_sq > −0.1` sub-surface band.
//! * The 8×8 sub-OBB grid for `5 ≤ z ≤ 16` — 64 boxes per node bounding a patch
//!   whose sagitta is 1.5 cm at z = 16 (§7).
//!
//! # Invariant I-7 — soundness at every level
//!
//! [`QuadtreeNode::update`] sets `children = None` on a cull and
//! `collect_visible_tiles` emits leaves only, so a wrong cull at *any* ancestor
//! deletes an entire subtree. Every test above is exact (§3) or provably
//! conservative (§2.8), at every level, so soundness follows by induction over the
//! tree. Do not add a test here that is only "sound at leaf granularity".

use glam::{DVec3, Vec3};

use super::bounding_volume::{Frustum, OrientedBoundingBox};
use super::horizon::{HorizonCamera, TilePatch};
use super::tile_id::{
    tile_bounds, tile_bounds_unstretched, web_mercator_y_to_lat_f64, TileBounds, TileId,
    MAX_ZOOM,
};
use crate::globe::geometry::{lon_lat_to_ecef_f64, EARTH_RADIUS_A_F64, EARTH_RADIUS_B_F64};

/// Screen-space budget behind the sub-box count (7.4).
///
/// A leaf sits at `D ≈ 2·a·θ_max` under the distance LOD, so its box overhangs its
/// own patch by `sagitta/D ≈ θ_max/4` radians at the eye. Allowing 5 % of screen
/// height (≈ 54 px of 1080, generous for *bounding*-volume slop) gives
/// `θ_max/4 ≤ 0.04`, i.e. `θ* = 0.16 rad`. This is the one free parameter in the
/// derivation; `θ* = 0.30` and `θ* = 0.16` both give `k = 1` for `z ≥ 5`, so the
/// choice only moves z ≤ 4, where the boxes are built once and never again.
const SUBDIVISION_BUDGET_RAD: f64 = 0.16;

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

/// Sub-boxes per axis, from (7.5): `k(z) = ceil(θ_max(z) / θ*)`.
///
/// With `θ_max(z) ≈ √2·π/2^z` at the equator (7.1) this is 14, 7, 4, 2 for z = 1..4
/// and **1 from z = 5 up**. `k = 1` means the node's own OBB is already a tight
/// enough proxy and no sub-boxes are stored at all.
fn sub_boxes_per_axis(z: u8) -> u32 {
    let theta_max = std::f64::consts::SQRT_2 * std::f64::consts::PI / (1u64 << z) as f64;
    (theta_max / SUBDIVISION_BUDGET_RAD).ceil().max(1.0) as u32
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
/// Returns `(surface_centre, radius, obb)`, where `radius` is the greatest distance
/// from the patch centre to a sampled point (the renderer's per-tile bounding
/// radius) and `obb` is the box in the [`tangent_frame`] at that centre.
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

/// Everything the per-node tests need, built once per frame.
#[derive(Clone, Copy, Debug)]
pub struct CullContext {
    pub frustum: Frustum,
    pub horizon: HorizonCamera,
}

impl CullContext {
    pub fn new(frustum: &Frustum) -> Self {
        Self {
            frustum: *frustum,
            horizon: HorizonCamera::new(frustum.eye),
        }
    }
}

pub struct QuadtreeNode {
    pub id: TileId,
    /// Patch centre on the ellipsoid, **f64** (invariant I-2).
    pub center: DVec3,
    pub radius: f32,
    pub lod_radius: f32,
    pub obb: OrientedBoundingBox,
    /// Sub-boxes, only for `z ≤ 4` (see [`sub_boxes_per_axis`]). `None` everywhere
    /// else, which is the overwhelming majority of the tree.
    pub sub_obbs: Option<Box<Vec<OrientedBoundingBox>>>,
    /// The eight trig constants of this tile's rectangle, for the horizon test.
    pub patch: TilePatch,
    pub visible: bool,
    pub children: Option<Box<[QuadtreeNode; 4]>>,
}

impl QuadtreeNode {
    pub fn new(id: TileId) -> Self {
        // I-5: the single source of tile bounds, shared with `TileMesh::generate`.
        let bounds = tile_bounds(&id);
        let (center, radius, obb) = fit_obb(&bounds, obb_grid_steps(id.z));

        // The LOD radius is deliberately measured on the *un*-stretched rectangle:
        // a polar row's true ground extent, not its pull to ±90°. Kept as-is (§8.4);
        // it makes polar caps subdivide later, which is an FP source, not an FN one.
        let raw = tile_bounds_unstretched(&id);
        let (_, lod_radius, _) = fit_obb(&raw, 2);

        let k = sub_boxes_per_axis(id.z);
        let sub_obbs = if k >= 2 {
            let mut boxes = Vec::with_capacity((k * k) as usize);
            for ui in 0..k {
                for vi in 0..k {
                    let sb = sub_bounds(
                        &id,
                        &bounds,
                        ui as f64 / k as f64,
                        (ui + 1) as f64 / k as f64,
                        vi as f64 / k as f64,
                        (vi + 1) as f64 / k as f64,
                    );
                    boxes.push(fit_obb(&sb, 4).2);
                }
            }
            Some(Box::new(boxes))
        } else {
            None
        };

        QuadtreeNode {
            id,
            center,
            radius,
            lod_radius,
            obb,
            sub_obbs,
            patch: TilePatch::new(&bounds),
            visible: false,
            children: None,
        }
    }

    /// Heap bytes this node hangs off itself (not counting children).
    pub fn sub_obb_heap_bytes(&self) -> usize {
        self.sub_obbs
            .as_ref()
            .map(|v| {
                std::mem::size_of::<Vec<OrientedBoundingBox>>()
                    + v.capacity() * std::mem::size_of::<OrientedBoundingBox>()
            })
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

    pub fn update(&mut self, ctx: &CullContext, lod_factor: f32) {
        // ── 1. Horizon. Exact, f64, and the strongest filter. ────────────────
        if self.patch.is_occluded(&ctx.horizon) {
            self.visible = false;
            self.children = None;
            return;
        }

        // ── 2. Camera-relative offset. The f64 subtraction is invariant I-2. ──
        let delta = ctx.frustum.relative(self.obb.center);

        // ── 3. Frustum: four side planes. ────────────────────────────────────
        if ctx
            .frustum
            .separated_from_box(delta, &self.obb.half_axes, self.obb.half_axis_l1)
        {
            self.visible = false;
            self.children = None;
            return;
        }

        // Coarse nodes (z ≤ 4) only: retest against the sub-boxes, whose union
        // covers the patch far more tightly than the single OBB. Keeping the node
        // if *any* sub-box survives is sound; rejecting only when all of them are
        // separated is the conservative direction.
        if let Some(boxes) = &self.sub_obbs {
            let any = boxes.iter().any(|b| {
                !ctx.frustum.separated_from_box(
                    ctx.frustum.relative(b.center),
                    &b.half_axes,
                    b.half_axis_l1,
                )
            });
            if !any {
                self.visible = false;
                self.children = None;
                return;
            }
        }

        self.visible = true;

        // LOD distance, also from an f64 subtraction (§8.2): free, since the frame
        // is camera-relative anyway.
        let dist = (self.center - ctx.frustum.eye).length() as f32;

        // Hysteresis logic: Subdivide at 1.0x, but don't collapse until 1.2x.
        // A 20% band prevents LOD oscillation when the camera straddles the
        // subdivision threshold. The old 1.05x band (~50 m at z=19) was too
        // narrow and caused rapid APPEAR/DISAPPEAR flicker on high-detail tiles.
        let is_subdivided = self.children.is_some();
        let subdivide_dist = self.lod_radius * lod_factor;
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
            if let Some(children) = &mut self.children {
                for child in children.iter_mut() {
                    child.update(ctx, lod_factor);
                }
            }
        } else {
            self.children = None;
        }
    }

    pub fn center_f32(&self) -> Vec3 {
        Vec3::new(self.center.x as f32, self.center.y as f32, self.center.z as f32)
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
            active_tiles.push((self.id, self.center_f32(), self.radius));
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

        active_tiles.push((self.id, self.center_f32(), self.radius));
        is_ready(&self.id)
    }
}

pub struct QuadtreeManager {
    pub roots: [QuadtreeNode; 4],
    pub lod_factor: f32, // Multiplier for subdivision distance check
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
        }
    }

    /// The camera position is `frustum.eye`: the frustum *is* the camera-relative
    /// frame, so there is no second position argument to get out of step with it.
    pub fn update(&mut self, frustum: &Frustum) {
        let ctx = CullContext::new(frustum);
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
