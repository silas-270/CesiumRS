use crate::globe::quadtree::TileId;
use crate::globe::terrain::{HeightPatch, HeightTileManager, PatchStatus};
use crate::globe::tiles::config::TileEngineConfig;
use crate::globe::tiles::mesh_worker::{MeshBuild, MeshWorkerPool};
use crate::globe::tiles::texture_manager::TileTextureManager;
use crate::globe::tiles::tile_cache::TileState;
use crate::globe::tiles::tile_fetcher::TilePriority;
use glam::Vec3;

pub struct RenderData<'a> {
    pub mesh_id: TileId,
    pub texture_id: TileId,
    pub bind_group: &'a wgpu::BindGroup,
    pub uv_scale_offset: [f32; 4],
}

/// **Phase E2** (`docs/terrain-plan.md` §8) — how many meshes may be rebuilt in one
/// frame, at most.
///
/// The cap exists because the rebuild path has no natural back-pressure of its own:
/// [`MeshWorkerPool`] is a rayon pool behind a `sync_channel(512)` and will accept
/// every request a frame can produce, so without a ceiling one frame that invalidates
/// a hundred meshes costs a hundred [`HeightPatch::sample`]s on the update thread and
/// two hundred buffer creations on the render thread, in that frame.
///
/// **Where the four comes from.** Measured by
/// `testing::terrain::test_mesh_lifetime::e2_what_one_rebuild_costs`, in the style
/// `culling::bench_update` uses — warm up, then 200 iterations, mean per call, never a
/// single-shot clock:
///
/// | `mesh_segments` | `HeightPatch::sample` | `generate_on::<Heightfield>` | 4 × sample | of a 16.6 ms frame |
/// |--:|--:|--:|--:|--:|
/// | **16** (shipped) | **7.8 µs** | 16.1 µs | **31.3 µs** | **0.19 %** |
/// | 32 | 31.0 µs | 51.5 µs | 123.9 µs | 0.75 % |
/// | 64 | 82.4 µs | 190.2 µs | 329.8 µs | 1.99 % |
///
/// Only the first column lands on the frame — `HeightPatch::sample` runs on the update
/// thread by construction, so the patch can cross into a rayon worker without a borrow
/// (see [`crate::globe::quadtree::surface::SurfaceModel::BuildCtx`]); the second column
/// is off it.
///
/// **So the arithmetic is not what binds, and saying so is the honest reading of the
/// table.** A full budget costs 0.19 % of a frame at the shipped density and 2 % at the
/// density §6 C4 measured as the outer option, and a budget of forty would still fit.
/// What sets the number is the *visual* argument: a rebuild is the ground under one
/// tile changing shape, and geometry is less forgiving than texture — a texture swap is
/// a blur, a mesh swap is the ground moving. Four tiles rippling over successive frames
/// reads as the surface sharpening; forty in one frame reads as a jolt. Four is also
/// enough that the worst burst this engine can produce — 54 stale meshes over six
/// levels, `a_five_level_descent_cannot_rebuild_more_than_the_budget_in_one_frame` —
/// drains in 14 frames, under a quarter of a second.
///
/// It is a ceiling, not a rate: the ordinary frame has nothing stale in it at all
/// (see [`fresher_height_source`] for why staleness is rare), and the budget is only
/// reached in the burst case a failed-then-retried fetch produces.
pub const MESH_REBUILD_BUDGET_PER_FRAME: usize = 4;

/// The height source `id`'s mesh *should* have been built from, if that is strictly
/// better than the one it *was* built from — otherwise `None`.
///
/// This is E2's staleness test, and it is deliberately the same predicate pair the
/// mesh builder itself runs: [`HeightTileManager::status_of`] for "is a build allowed
/// at all" and [`HeightTileManager::resolve_source`] for "which tile would answer".
/// Asking a different question here than [`HeightPatch::sample`] asks would let a
/// rebuild be scheduled that then produces the identical mesh, forever.
///
/// # No downgrade — the geometry half of `display_state`'s rule
///
/// A rebuild is only offered when the available source is **strictly deeper** than the
/// recorded one. The two ways it could be shallower are an LRU eviction of the deep
/// tile and a fetch that has since expired out of the negative cache; in both, the
/// mesh already on the card is the better of the two, and swapping it for a coarser
/// one would be the ground visibly flattening. `display_state` refuses exactly this
/// for textures (rule 4, "never downgraded back to a parent fallback"), and geometry
/// is the less forgiving of the two: a texture downgrade is a blur, a mesh downgrade
/// is a hillside dropping.
///
/// The rule is also what makes the loop terminate. Every accepted rebuild strictly
/// raises `height_source.z`, which is bounded by
/// [`TerrainConfig::max_level`](crate::globe::tiles::config::TerrainConfig::max_level),
/// so a tile can be rebuilt at most that many times before no further rebuild can be
/// offered. There is no oscillation to damp, which is why E2 needs no grace period of
/// the kind `display_state`'s 200 ms serves: that timer exists to absorb a set that
/// flips back and forth, and this one cannot flip back.
///
/// # How rare this is, and why that is a Phase C result rather than an E2 gap
///
/// Phase C made `status_of` answer `Ready` only once `source_tile_for(id)` has
/// arrived **or failed**. So the ordinary mesh is built from the deepest tile the
/// source will ever serve for it, and nothing can improve on it — a camera descending
/// five levels creates *new* nodes with no mesh, which is the `missing_meshes` path,
/// not this one. What is left is the case Phase C explicitly deferred to here: a tile
/// whose own height fetch **failed**, so the mesh was built from an ancestor, and
/// whose retry — the negative cache expires after
/// `TileEngineConfig::negative_cache_duration` — later succeeds. That, plus switching
/// terrain on at runtime, at which point every resident mesh carries `None`.
pub fn fresher_height_source(
    heights: &HeightTileManager,
    id: TileId,
    built_from: Option<TileId>,
) -> Option<TileId> {
    // Same gate as `HeightPatch::sample`: while the tile's own source is in flight no
    // mesh may be built from an ancestor, so no *re*build may be either.
    if heights.status_of(id) != PatchStatus::Ready {
        return None;
    }
    let available = heights.resolve_source(id)?;
    match built_from {
        // Strictly deeper only. Equal is the steady state; shallower is a downgrade.
        Some(had) if available.z <= had.z => None,
        _ => Some(available),
    }
}

/// The at most `budget` meshes worth rebuilding this frame, nearest to the camera
/// first.
///
/// Free function rather than a method so it can be measured and tested against a bare
/// [`HeightTileManager`], with no GPU device in the picture —
/// `testing::terrain::test_mesh_lifetime` drives the whole policy through it.
///
/// **Nearest first**, and the ordering is the policy's second half. When a burst of
/// retries lands, the tiles that matter are the ones filling the screen: a stale mesh
/// 200 km out is a silhouette a few pixels tall, while the one under the aircraft is
/// the ground it is about to touch. Ties break on `(z, x, y)` so the choice is
/// deterministic frame to frame and a headless capture is reproducible.
pub fn select_mesh_rebuilds(
    heights: &HeightTileManager,
    camera_pos: Vec3,
    drawn: &[(TileId, Vec3, Option<TileId>)],
    budget: usize,
) -> Vec<TileId> {
    if budget == 0 {
        return Vec::new();
    }
    let mut stale: Vec<(f32, TileId)> = drawn
        .iter()
        .filter(|(id, _, built_from)| fresher_height_source(heights, *id, *built_from).is_some())
        .map(|(id, center, _)| ((*center - camera_pos).length_squared(), *id))
        .collect();

    let order = |a: &(f32, TileId), b: &(f32, TileId)| {
        a.0.total_cmp(&b.0)
            .then_with(|| (a.1.z, a.1.x, a.1.y).cmp(&(b.1.z, b.1.x, b.1.y)))
    };
    stale.sort_unstable_by(order);
    stale.truncate(budget);
    stale.into_iter().map(|(_, id)| id).collect()
}

pub struct TileSystem {
    pub config: TileEngineConfig,
    pub texture_manager: TileTextureManager,
    /// Height tiles — `Some` **only** while `config.terrain.enabled`
    /// (`docs/terrain-plan.md` §5). `None` is the flat path, and on it nothing in this
    /// file does any extra work at all: no cache, no fetcher, no request.
    ///
    /// Phase C's mesh builder is its first real consumer — see the `missing_meshes`
    /// loop in [`Self::update`]. No *cull* reads it yet; that is Phase D.
    pub height_manager: Option<HeightTileManager>,
    pub mesh_worker: MeshWorkerPool,
    last_camera_pos: Option<Vec3>,
    /// The levels the globe is being **drawn** at, per tile — the feed
    /// [`Self::ground_height_at`] needs and the only thing in this file that knows
    /// anything about the renderer's cut of the quadtree.
    ///
    /// Empty, and never written, while terrain is off. See [`Self::set_drawn_meshes`].
    drawn: DrawnMeshes,
    /// Every imagery / height tile this frame asked for, whether or not the request
    /// was new. Anything still queued in a fetcher and **not** in here is a tile the
    /// camera has moved away from; see [`Self::cancel_unwanted_fetches`]. Kept across
    /// frames only to reuse the allocation.
    wanted_textures: std::collections::HashSet<TileId>,
    wanted_heights: std::collections::HashSet<TileId>,
    /// Positions whose ground height collision needs this frame; see
    /// [`Self::want_ground_at`].
    ground_points: Vec<glam::DVec3>,
}

/// Which tile's mesh covers a point, as of the last frame that drew one.
///
/// # Why this exists at all
///
/// [`HeightTileManager::peek_mesh_height_at_lon_lat`] is exact only if it is asked at
/// the level the mesh under the point was built at, and the height cache cannot know
/// that: it is a property of the quadtree's cut, which lives two layers up in
/// `wgpu_state`. This is the whole of the coupling — a set of ids in, a level out — and
/// it is deliberately the *renderable* set rather than the visible one, because a tile
/// whose own mesh has not arrived is drawn with its parent's and the parent's grid is
/// what the camera is standing on.
///
/// # One frame behind, and that is the correct phase
///
/// `TileSystem::ground_height_at` is called at the top of `update_logic`, before this
/// frame's quadtree runs. The meshes on the card at that moment are the previous
/// frame's, which is exactly what this holds.
#[derive(Default)]
pub struct DrawnMeshes {
    ids: std::collections::HashSet<TileId>,
    /// `(min z, max z)` over `ids`, so a query walks only the levels that exist.
    /// `None` when nothing is drawn.
    z_range: Option<(u8, u8)>,
}

impl DrawnMeshes {
    pub fn replace(&mut self, ids: impl Iterator<Item = TileId>) {
        self.ids.clear();
        let mut range: Option<(u8, u8)> = None;
        for id in ids {
            range = Some(match range {
                Some((lo, hi)) => (lo.min(id.z), hi.max(id.z)),
                None => (id.z, id.z),
            });
            self.ids.insert(id);
        }
        self.z_range = range;
    }

    /// The level of the deepest drawn tile containing `(lon, lat)`.
    ///
    /// Deepest-first, because the renderable set is a quadtree *cut* only up to the
    /// fallback rule: a parent drawn in place of a missing child sits in the set
    /// alongside its other children, and the child is the one being drawn where it
    /// exists.
    pub fn level_at(&self, lon_deg: f64, lat_deg: f64) -> Option<u8> {
        let (min_z, max_z) = self.z_range?;
        for z in (min_z..=max_z).rev() {
            let (id, _, _) = HeightTileManager::tile_uv_at_lon_lat(lon_deg, lat_deg, z);
            if self.ids.contains(&id) {
                return Some(z);
            }
        }
        None
    }
}

impl TileSystem {
    pub fn new(device: &wgpu::Device, queue: &wgpu::Queue, config: TileEngineConfig) -> Self {
        Self {
            texture_manager: TileTextureManager::new(device, queue, &config),
            height_manager: Self::build_height_manager(&config),
            mesh_worker: MeshWorkerPool::new(),
            config,
            last_camera_pos: None,
            drawn: DrawnMeshes::default(),
            wanted_textures: Default::default(),
            wanted_heights: Default::default(),
            ground_points: Vec::new(),
        }
    }

    /// A [`HeightTileManager`] when terrain is on, `None` when it is off.
    ///
    /// Also the re-entry point for a runtime toggle (debug panel, `ViewerCommand`):
    /// flipping `config.terrain.enabled` and reassigning `height_manager` from this is
    /// the whole switch, and turning terrain back off drops the cache and the fetcher's
    /// runtime with it.
    pub fn build_height_manager(config: &TileEngineConfig) -> Option<HeightTileManager> {
        config
            .terrain
            .enabled
            .then(|| HeightTileManager::new(config))
    }

    pub fn update(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        camera_pos: Vec3,
        visible_tiles: &[(TileId, Vec3, f32)],
        missing_meshes: &[TileId],
    ) {
        self.wanted_textures.clear();
        self.wanted_heights.clear();
        self.texture_manager.cache.begin_frame();
        if let Some(h) = self.height_manager.as_mut() {
            h.cache.begin_frame();
        }

        // Handle prefetching based on camera velocity
        if self.config.enable_prefetch {
            if let Some(last_pos) = self.last_camera_pos {
                let velocity = camera_pos - last_pos;
                if velocity.length_squared() > 1e-6 {
                    let norm_vel = velocity.normalize();

                    for (id, center, _) in visible_tiles {
                        if id.z < 4 {
                            continue;
                        } // Prevent root-level prefetch flooding

                        let to_tile = (*center - camera_pos).normalize_or_zero();
                        if to_tile.dot(norm_vel) > 0.5 {
                            let neighbors = vec![
                                TileId {
                                    z: id.z,
                                    x: id.x.saturating_add(1),
                                    y: id.y,
                                },
                                TileId {
                                    z: id.z,
                                    x: id.x.saturating_sub(1),
                                    y: id.y,
                                },
                                TileId {
                                    z: id.z,
                                    x: id.x,
                                    y: id.y.saturating_add(1),
                                },
                                TileId {
                                    z: id.z,
                                    x: id.x,
                                    y: id.y.saturating_sub(1),
                                },
                            ];

                            let max_x_y = (1 << id.z) - 1;
                            for n in neighbors {
                                if n.x <= max_x_y && n.y <= max_x_y {
                                    self.wanted_textures.insert(n);
                                    if self.texture_manager.cache.peek_state(&n).is_none() {
                                        self.texture_manager.request_tile(n, TilePriority::Low);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        self.last_camera_pos = Some(camera_pos);

        // Height tiles first, so a mesh that needs them can find them: the mesh loop
        // below *defers* a tile whose heights have not arrived rather than baking sea
        // level into it (`docs/terrain-plan.md` §5 B2), and a deferred tile is only
        // ever un-deferred by a request having gone out.
        // Heights are needed to *build* a mesh, so only tiles without one ask for them
        // (`missing_meshes` already holds visible, fallback, ancestor and rebuild
        // candidates). Asking for every visible tile, built or not, made the working set
        // near the ground several times the cache's capacity. The ground under the camera
        // and the aircraft is asked for too, so collision keeps its detailed data.
        if let Some(heights) = self.height_manager.as_mut() {
            for id in missing_meshes {
                Self::request_height_chain(heights, &mut self.wanted_heights, *id);
            }
            for pos in self.ground_points.drain(..) {
                let (lon, lat) = crate::globe::geometry::ecef_to_lon_lat_f64(pos);
                let (id, _, _) = HeightTileManager::tile_uv_at_lon_lat(lon, lat, heights.max_level());
                Self::request_height_chain(heights, &mut self.wanted_heights, id);
            }
            heights.update();
        }

        for id in missing_meshes {
            // Skip tiles whose mesh build is already in flight — `request_mesh`
            // deduplicates on `is_requested`, but `HeightPatch::sample` (361 bilinear
            // LRU lookups at mesh_segments=16) ran *before* that guard every frame,
            // burning hundreds of ms per burst for tiles that didn't need re-sampling.
            if self.mesh_worker.is_requested(id) {
                continue;
            }

            let segments = self.config.mesh_segments;
            let build = match self.height_manager.as_mut() {
                None => MeshBuild::Flat,
                Some(heights) => {
                    match HeightPatch::sample(
                        heights,
                        *id,
                        segments,
                        self.config.terrain.exaggeration,
                    ) {
                        Ok(patch) => MeshBuild::Terrain(Box::new(patch)),
                        // Not yet: skip this tile entirely and retry next frame. The
                        // alternative — a flat mesh now — is the failure §5 B2 exists
                        // to prevent, because nothing would later mark it stale.
                        Err(PatchStatus::Pending) => continue,
                        // The whole ancestor chain failed. No data is coming, so the
                        // flat mesh is the honest answer, and it is the *same* flat
                        // mesh the engine builds with terrain off.
                        Err(_) => MeshBuild::Flat,
                    }
                }
            };
            self.mesh_worker.request_mesh(*id, segments, build);
        }

        let mut ancestors_seen = std::collections::HashSet::new();
        for (id, _, _) in visible_tiles {
            self.wanted_textures.insert(*id);
            if self.texture_manager.cache.peek_state(id).is_none() {
                self.texture_manager.request_tile(*id, TilePriority::High);
            }

            // Every ancestor, not just the parent: whatever this tile falls back to while
            // its own texture is missing — and whatever the renderer falls back to when a
            // subtree is incomplete — has to be resident, or the fallback is grey. Present
            // ones are promoted so the LRU never evicts the chain a visible tile stands on.
            let mut a = id.parent();
            while let Some(p) = a {
                if !ancestors_seen.insert(p) {
                    break;
                }
                self.wanted_textures.insert(p);
                if self.texture_manager.cache.get_state(&p).is_none() {
                    self.texture_manager.request_tile(p, TilePriority::Low);
                }
                a = p.parent();
            }
        }

        self.cancel_unwanted_fetches();
        self.texture_manager.update(device, queue);
    }

    /// Drops queued fetches for tiles this frame no longer asked for.
    ///
    /// Without it a fast pan or zoom leaves every tile it swept past in the fetch
    /// queues. They still cost a connection slot, a PNG decode, a height decode and a
    /// GPU upload each, they sit ahead of what is now on screen, and while they are
    /// `Fetching` they hold cache slots. The ground under the camera then waits
    /// seconds for data the camera no longer needs — the "terrain catches up slowly
    /// after a fast move" symptom. Requests already on the wire are kept.
    ///
    /// Cheap: one lock per fetcher, and an early return when the queue is empty, which
    /// is every frame of a camera at rest once its tiles have arrived.
    fn cancel_unwanted_fetches(&mut self) {
        let tex = self.texture_manager.cancel_unwanted(&self.wanted_textures);
        let hgt = match self.height_manager.as_mut() {
            Some(h) => h.cancel_unwanted(&self.wanted_heights),
            None => 0,
        };
        if tex + hgt > 0 {
            log::debug!("[FETCH CANCEL] imagery={tex} height={hgt}");
        }
    }

    /// Queues `id`'s height tile and its immediate parent fallback, if they are not
    /// known, and records both as wanted this frame.
    fn request_height_chain(
        heights: &mut HeightTileManager,
        wanted: &mut std::collections::HashSet<TileId>,
        id: TileId,
    ) {
        let src = heights.source_tile_for(id);
        wanted.insert(src);
        if heights.cache.get_state(&src).is_none() {
            heights.request_tile(src, TilePriority::High);
        }

        if let Some(p) = src.parent() {
            wanted.insert(p);
            if heights.cache.get_state(&p).is_none() {
                heights.request_tile(p, TilePriority::Low);
            }
        }
    }

    pub fn compute_fallback_uv(child: TileId, parent: TileId) -> [f32; 4] {
        let mut scale_x = 1.0;
        let mut scale_y = 1.0;
        let mut offset_x = 0.0;
        let mut offset_y = 0.0;
        let mut curr = child;

        while let Some(p) = curr.parent() {
            let is_right = !curr.x.is_multiple_of(2);
            let is_bottom = !curr.y.is_multiple_of(2);

            scale_x *= 0.5;
            scale_y *= 0.5;
            offset_x = offset_x * 0.5 + if is_right { 0.5 } else { 0.0 };
            offset_y = offset_y * 0.5 + if is_bottom { 0.5 } else { 0.0 };

            if p == parent {
                break;
            }
            curr = p;
        }

        [scale_x, scale_y, offset_x, offset_y]
    }

    /// Non-mutating version: checks what texture would be shown for `id` without
    /// promoting anything in the LRU cache. Used by the display-state updater
    /// so that readiness checks don't silently evict parent fallback textures.
    pub fn peek_render_data(&self, id: TileId) -> Option<(TileId, [f32; 4])> {
        if let Some(TileState::Ready(_)) = self.texture_manager.cache.peek_state(&id) {
            return Some((id, [1.0, 1.0, 0.0, 0.0]));
        }

        let mut current_id = id;
        while let Some(parent_id) = current_id.parent() {
            if let Some(TileState::Ready(_)) = self.texture_manager.cache.peek_state(&parent_id) {
                let uv = Self::compute_fallback_uv(id, parent_id);
                return Some((parent_id, uv));
            }
            current_id = parent_id;
        }

        None
    }

    /// Mutable version used at draw time — promotes accessed textures in the LRU
    /// so that currently-rendered tiles are never evicted mid-frame.
    pub fn get_render_data(&mut self, id: TileId) -> Option<RenderData<'_>> {
        let target = if let Some(TileState::Ready(_)) = self.texture_manager.cache.get_state(&id) {
            Some((id, [1.0, 1.0, 0.0, 0.0]))
        } else {
            let mut current_id = id;
            let mut found = None;
            while let Some(parent_id) = current_id.parent() {
                if let Some(TileState::Ready(_)) = self.texture_manager.cache.get_state(&parent_id) {
                    found = Some((parent_id, Self::compute_fallback_uv(id, parent_id)));
                    break;
                }
                current_id = parent_id;
            }
            found
        };

        if let Some((tex_id, uv_scale_offset)) = target {
            if let Some(TileState::Ready((_, bg))) = self.texture_manager.cache.get_state(&tex_id) {
                return Some(RenderData {
                    mesh_id: id,
                    texture_id: tex_id,
                    bind_group: bg,
                    uv_scale_offset,
                });
            }
        }

        // Return the static fallback color bind group as a last-resort fallback
        Some(RenderData {
            mesh_id: id,
            texture_id: id,
            bind_group: &self.texture_manager.fallback_bind_group,
            uv_scale_offset: [1.0, 1.0, 0.0, 0.0],
        })
    }

    /// Height at `(u, v)` of `id` in **megametres**, or `None` when terrain is off or
    /// no ancestor's data has arrived. Phase B's single query entry point; Phase C's
    /// mesh builder is its first real caller.
    pub fn height_at(&mut self, id: TileId, u: f64, v: f64) -> Option<f64> {
        self.height_manager.as_mut()?.height_at(id, u, v)
    }

    /// Height of the **drawn** surface in megametres under an ECEF position, or `None`
    /// when terrain is off or no height data covering it has arrived.
    ///
    /// Phase E3's entry point for the parts of the engine that used to assume the
    /// surface was the ellipsoid — camera clearance, the collision floor, label
    /// placement. Unlike [`Self::height_at`] it takes `&self`, promotes nothing and
    /// enqueues nothing, so it is safe to call once per frame from the render path.
    ///
    /// **Exaggeration is applied here**, and only here on this path:
    /// `HeightTileManager::peek_height_at_lon_lat` hands back the raw DEM value, and
    /// `HeightPatch::sample` multiplies the same factor into the vertices the renderer
    /// draws. Leaving it off would put the camera's idea of the ground a factor
    /// `exaggeration` away from the ground it can see.
    ///
    /// `None` is a deliberate third state, not a zero. With terrain off the answer is
    /// always `None` and every caller falls back to the ellipsoid arithmetic it used
    /// before — that is what makes the flat path the *same* code rather than a code path
    /// that happens to add zero.
    pub fn ground_height_at(&self, pos: glam::DVec3) -> Option<f64> {
        // View-independent: the deepest resident height data, bilinear. The drawn mesh is
        // the wrong reference for collision — the points collision tests (under the
        // camera, under the aircraft) are often off-screen, where the only drawn tile is
        // a culled z1-z5 one whose grid spans hundreds of km; at Frankfurt that read
        // 215-325 m instead of 100 m, and changed as the view rotated.
        let h = self.height_manager.as_ref()?;
        let (lon, lat) = crate::globe::geometry::ecef_to_lon_lat_f64(pos);
        let raw = h.peek_height_at_lon_lat(lon, lat)?;
        Some(raw * self.config.terrain.exaggeration as f64)
    }

    /// Keeps the detailed height data under `pos` loaded (requested next `update`).
    pub fn want_ground_at(&mut self, pos: glam::DVec3) {
        if self.height_manager.is_some() {
            self.ground_points.push(pos);
        }
    }

    /// Height of the surface as currently **drawn** under `pos` — the triangle net at the
    /// level it is drawn at. Right for things placed on the visible surface (labels);
    /// wrong for collision, see [`Self::ground_height_at`].
    pub fn drawn_ground_height_at(&self, pos: glam::DVec3) -> Option<f64> {
        let h = self.height_manager.as_ref()?;
        let (lon, lat) = crate::globe::geometry::ecef_to_lon_lat_f64(pos);
        let raw = match self.drawn.level_at(lon, lat) {
            // The drawn surface: the triangle net, at the level it is drawn at.
            Some(z) => h.peek_mesh_height_at_lon_lat(lon, lat, z, self.config.mesh_segments)?,
            // Nothing is drawn there — behind the globe, outside the frustum, or the
            // very first frame. There is no drawn surface to agree with, so the
            // bilinear field is the best statement available and this is exactly the
            // pre-existing query.
            None => h.peek_height_at_lon_lat(lon, lat)?,
        };
        Some(raw * self.config.terrain.exaggeration as f64)
    }

    /// The tiles whose meshes the renderer is drawing, for [`Self::ground_height_at`].
    ///
    /// Called once per frame from `wgpu_state` with the **renderable** set — the one
    /// that already has the parent-mesh fallback applied — and a no-op on the flat path,
    /// where `ground_height_at` returns `None` before it looks at anything.
    pub fn set_drawn_meshes<'a>(&mut self, drawn: impl Iterator<Item = &'a TileId>) {
        if self.height_manager.is_none() {
            // Terrain has been switched off. Drop whatever the terrain arm left behind so
            // that switching it back on cannot answer one frame from a set that is a
            // session old, and then do nothing at all on every frame after this one.
            if self.drawn.z_range.is_some() {
                self.drawn.replace(std::iter::empty());
            }
            return;
        }
        self.drawn.replace(drawn.copied());
    }

    /// **Phase E2** — the meshes among `drawn` that a better height tile has outdated,
    /// at most [`MESH_REBUILD_BUDGET_PER_FRAME`] of them, nearest first.
    ///
    /// `drawn` is `(tile, its mesh's world centre, the height source that mesh was
    /// built from)`, which is what the renderer's mesh cache can answer and this module
    /// cannot — hence the argument rather than a lookup.
    ///
    /// **With terrain off this returns an empty `Vec` before looking at anything.**
    /// There is no `HeightTileManager`, so there is no height source, so no mesh can
    /// ever be out of date: every mesh on the flat path is `MeshBuild::Flat` and stays
    /// correct forever. That is the same shape as [`Self::ground_height_at`]'s leading
    /// `?` — one `Option` test, not a branch inside a loop.
    ///
    /// Tiles whose rebuild is **already on a worker** are dropped before the budget is
    /// applied, because a stale mesh stays stale in the cache until its replacement
    /// lands — several frames later — and would otherwise be re-picked every frame and
    /// block the tiles behind it. See [`MeshWorkerPool::is_requested`] for the
    /// measurement that found this.
    pub fn select_mesh_rebuilds(
        &self,
        camera_pos: Vec3,
        drawn: &[(TileId, Vec3, Option<TileId>)],
    ) -> Vec<TileId> {
        let Some(heights) = self.height_manager.as_ref() else {
            return Vec::new();
        };
        let pending: Vec<(TileId, Vec3, Option<TileId>)> = drawn
            .iter()
            .filter(|(id, _, _)| !self.mesh_worker.is_requested(id))
            .copied()
            .collect();
        select_mesh_rebuilds(heights, camera_pos, &pending, MESH_REBUILD_BUDGET_PER_FRAME)
    }

    /// Whether this system has any terrain at all — `false` whenever
    /// `TerrainConfig::enabled` is off, in which case [`Self::ground_height_at`] can
    /// only ever answer `None`.
    ///
    /// Lets a caller skip building a `&dyn` for a query that cannot succeed, which is
    /// what the label pass does to keep the flat path free of a vtable it has no use
    /// for.
    pub fn has_terrain(&self) -> bool {
        self.height_manager.is_some()
    }

    pub fn is_loading_complete(&self) -> bool {
        self.texture_manager.is_loading_complete()
            && self.mesh_worker.is_loading_complete()
            && self
                .height_manager
                .as_ref()
                .is_none_or(|h| h.is_loading_complete())
    }
}

/// Phase E3.4: the label pass asks the tile system where the ground is.
///
/// The whole implementation is [`TileSystem::ground_height_at`] with the argument
/// widened to f64 and the answer narrowed to f32 — labels are placed in the f32 world
/// frame, where a megametre-scale position has ~0.4 m of resolution and a height
/// carried in f64 would be thrown away by the addition anyway.
impl crate::label::GroundHeights for TileSystem {
    fn ground_height_above_ellipsoid(&self, pos: Vec3) -> Option<f32> {
        self.drawn_ground_height_at(glam::DVec3::new(pos.x as f64, pos.y as f64, pos.z as f64))
            .map(|h| h as f32)
    }
}
