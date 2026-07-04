use crate::engine::globe::quadtree::TileId;
use crate::engine::globe::io::config::TileEngineConfig;
use crate::engine::globe::io::mesh_worker::MeshWorkerPool;
use crate::engine::globe::io::texture_manager::TileTextureManager;
use crate::engine::globe::io::tile_cache::TileState;
use crate::engine::globe::io::tile_fetcher::TilePriority;
use glam::Vec3;

pub struct RenderData<'a> {
    pub mesh_id: TileId,
    pub texture_id: TileId,
    pub bind_group: &'a wgpu::BindGroup,
    pub uv_scale_offset: [f32; 4],
}

pub struct TileOrchestrator {
    pub config: TileEngineConfig,
    pub texture_manager: TileTextureManager,
    pub mesh_worker: MeshWorkerPool,
    last_camera_pos: Option<Vec3>,
}

impl TileOrchestrator {
    pub fn new(device: &wgpu::Device, config: TileEngineConfig) -> Self {
        Self {
            texture_manager: TileTextureManager::new(device, &config),
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

                    // Very naive prefetch: if a tile is "in front" of the movement, guess its neighbors.
                    // A better approach is to translate the velocity into lon/lat delta and request those tiles.
                    // For now, let's just use the radius and center to estimate if we are moving towards it.
                    let to_tile = (*center - camera_pos).normalize_or_zero();
                    if to_tile.dot(norm_vel) > 0.5 {
                        // Prefetch neighbors in X and Y
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

        // Request visible tiles with High priority
        for (id, _, _) in visible_tiles {
            self.texture_manager.request_tile(*id, TilePriority::High);
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

        None
    }

    pub fn is_loading_complete(&self) -> bool {
        self.texture_manager.is_loading_complete() && self.mesh_worker.is_loading_complete()
    }
}
