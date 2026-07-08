use crate::globe::quadtree::TileId;
use crate::globe::tiles::config::TileEngineConfig;
use crate::globe::tiles::mesh_worker::MeshWorkerPool;
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
    pub mesh_worker: MeshWorkerPool,
    last_camera_pos: Option<Vec3>,
}

impl TileSystem {
    pub fn new(device: &wgpu::Device, queue: &wgpu::Queue, config: TileEngineConfig) -> Self {
        Self {
            texture_manager: TileTextureManager::new(device, queue, &config),
            mesh_worker: MeshWorkerPool::new(),
            config,
            last_camera_pos: None,
        }
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
                    if id.z < 4 { continue; } // Prevent root-level prefetch flooding

                    let to_tile = (*center - camera_pos).normalize_or_zero();
                    if to_tile.dot(norm_vel) > 0.5 {
                        let mut neighbors = Vec::new();
                        neighbors.push(TileId { z: id.z, x: id.x.saturating_add(1), y: id.y });
                        neighbors.push(TileId { z: id.z, x: id.x.saturating_sub(1), y: id.y });
                        neighbors.push(TileId { z: id.z, x: id.x, y: id.y.saturating_add(1) });
                        neighbors.push(TileId { z: id.z, x: id.x, y: id.y.saturating_sub(1) });
                        
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

        for id in missing_meshes {
            self.mesh_worker.request_mesh(*id, 16);
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

    pub fn compute_fallback_uv(child: TileId, parent: TileId) -> [f32; 4] {
        let mut scale_x = 1.0;
        let mut scale_y = 1.0;
        let mut offset_x = 0.0;
        let mut offset_y = 0.0;
        let mut curr = child;

        while let Some(p) = curr.parent() {
            let is_right = curr.x % 2 != 0;
            let is_bottom = curr.y % 2 != 0;

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

    pub fn is_loading_complete(&self) -> bool {
        self.texture_manager.is_loading_complete() && self.mesh_worker.is_loading_complete()
    }
}
