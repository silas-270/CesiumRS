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
}

impl TileSystem {
    pub fn new(device: &wgpu::Device, queue: &wgpu::Queue, config: TileEngineConfig) -> Self {
        Self {
            texture_manager: TileTextureManager::new(device, queue, &config),
            height_manager: Self::build_height_manager(&config),
            mesh_worker: MeshWorkerPool::new(),
            config,
            last_camera_pos: None,
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
                                    self.texture_manager.request_tile(n, TilePriority::Low);
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
        if let Some(heights) = self.height_manager.as_mut() {
            for (id, _, _) in visible_tiles {
                Self::request_height_chain(heights, *id);
            }
            // Fallback parent meshes are drawn too, and they are not always in
            // `visible_tiles`. Without this they would defer forever.
            for id in missing_meshes {
                Self::request_height_chain(heights, *id);
            }
            heights.update();
        }

        for id in missing_meshes {
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

        for (id, _, _) in visible_tiles {
            self.texture_manager.request_tile(*id, TilePriority::High);

            // Proactively fetch missing parent textures at low priority so they
            // are available as fallbacks before the own texture arrives.
            let mut curr = *id;
            while let Some(p) = curr.parent() {
                if self.texture_manager.cache.get_state(&p).is_none() {
                    self.texture_manager.request_tile(p, TilePriority::Low);
                }
                curr = p;
            }
        }

        self.texture_manager.update(device, queue);
    }

    /// Queues `id`'s height tile and its whole ancestor chain, if they are not known.
    ///
    /// The tile itself goes in at `High`: a missing texture is a blur, a missing height
    /// tile is the wrong shape. The ancestor chain follows at `Low` for the same reason
    /// imagery prefetches it — and more so here, because past z15 the ancestor is not a
    /// fallback, it is the only data that will ever exist (`docs/terrain-plan.md` §2).
    fn request_height_chain(heights: &mut HeightTileManager, id: TileId) {
        heights.request_tile(id, TilePriority::High);

        let mut curr = heights.source_tile_for(id);
        while let Some(p) = curr.parent() {
            if heights.cache.get_state(&p).is_none() {
                heights.request_tile(p, TilePriority::Low);
            }
            curr = p;
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
        if let Some(TileState::Ready(_)) = self.texture_manager.cache.get_state(&id) {
            let bg = match self.texture_manager.cache.get_state(&id).unwrap() {
                TileState::Ready((_, bg)) => bg,
                _ => unreachable!(),
            };
            return Some(RenderData {
                mesh_id: id,
                texture_id: id,
                bind_group: bg,
                uv_scale_offset: [1.0, 1.0, 0.0, 0.0],
            });
        }

        let mut current_id = id;
        let mut found_parent = None;

        while let Some(parent_id) = current_id.parent() {
            if let Some(TileState::Ready(_)) = self.texture_manager.cache.get_state(&parent_id) {
                found_parent = Some((parent_id, Self::compute_fallback_uv(id, parent_id)));
                break;
            }
            current_id = parent_id;
        }

        if let Some((parent_id, uv_scale_offset)) = found_parent {
            let bg = match self.texture_manager.cache.get_state(&parent_id).unwrap() {
                TileState::Ready((_, bg)) => bg,
                _ => unreachable!(),
            };
            return Some(RenderData {
                mesh_id: id,
                texture_id: parent_id,
                bind_group: bg,
                uv_scale_offset,
            });
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
        let h = self.height_manager.as_ref()?;
        let (lon, lat) = crate::globe::geometry::ecef_to_lon_lat_f64(pos);
        Some(h.peek_height_at_lon_lat(lon, lat)? * self.config.terrain.exaggeration as f64)
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
        self.ground_height_at(glam::DVec3::new(pos.x as f64, pos.y as f64, pos.z as f64))
            .map(|h| h as f32)
    }
}
