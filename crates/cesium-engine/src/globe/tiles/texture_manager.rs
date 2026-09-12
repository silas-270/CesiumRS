use crate::globe::quadtree::TileId;
use crate::globe::tiles::config::TileEngineConfig;
use crate::globe::tiles::tile_cache::TileCacheManager;
use crate::globe::tiles::tile_fetcher::{TileFetcher, TileImage, TilePriority};
use tokio::sync::mpsc;

pub struct TileTextureManager {
    pub cache: TileCacheManager<(wgpu::Texture, wgpu::BindGroup)>,
    rx: mpsc::UnboundedReceiver<(TileId, Result<TileImage, String>)>,
    pub fetcher: TileFetcher,
    pub bind_group_layout: wgpu::BindGroupLayout,
    sampler: wgpu::Sampler,
    pub fallback_bind_group: wgpu::BindGroup,
    /// Memory ceiling for decoded tile textures; the entry count is derived
    /// from it once a tile's real size is known. See
    /// [`TileEngineConfig::tile_cache_budget_bytes`].
    budget_bytes: usize,
    /// Upper bound on entries regardless of how small tiles turn out to be.
    max_entries: std::num::NonZeroUsize,
    /// Byte size of the last decoded tile. Tiles from one imagery style are
    /// uniform, so this settles after the first one; it's re-checked per tile
    /// only so a style whose tiles differ in size still converges.
    bytes_per_tile: Option<usize>,
}

impl TileTextureManager {
    pub fn new(device: &wgpu::Device, queue: &wgpu::Queue, config: &TileEngineConfig) -> Self {
        let (tx, rx) = mpsc::unbounded_channel();

        let bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("Tile Texture Bind Group Layout"),
            entries: &[
                wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Texture {
                        sample_type: wgpu::TextureSampleType::Float { filterable: true },
                        view_dimension: wgpu::TextureViewDimension::D2,
                        multisampled: false,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 1,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
                    count: None,
                },
            ],
        });

        let sampler = device.create_sampler(&wgpu::SamplerDescriptor {
            label: Some("Tile Texture Sampler"),
            address_mode_u: wgpu::AddressMode::ClampToEdge,
            address_mode_v: wgpu::AddressMode::ClampToEdge,
            address_mode_w: wgpu::AddressMode::ClampToEdge,
            mag_filter: wgpu::FilterMode::Linear,
            min_filter: wgpu::FilterMode::Linear,
            mipmap_filter: wgpu::FilterMode::Linear,
            ..Default::default()
        });

        // Create fallback 1x1 texture using config.base_color
        let fallback_size = wgpu::Extent3d {
            width: 1,
            height: 1,
            depth_or_array_layers: 1,
        };
        let fallback_texture = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("Fallback Tile Texture"),
            size: fallback_size,
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Rgba8UnormSrgb,
            usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
            view_formats: &[],
        });
        queue.write_texture(
            wgpu::ImageCopyTexture {
                texture: &fallback_texture,
                mip_level: 0,
                origin: wgpu::Origin3d::ZERO,
                aspect: wgpu::TextureAspect::All,
            },
            &config.base_color,
            wgpu::ImageDataLayout {
                offset: 0,
                bytes_per_row: Some(4),
                rows_per_image: Some(1),
            },
            fallback_size,
        );
        let fallback_view = fallback_texture.create_view(&wgpu::TextureViewDescriptor::default());
        let fallback_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            layout: &bind_group_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: wgpu::BindingResource::TextureView(&fallback_view),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: wgpu::BindingResource::Sampler(&sampler),
                },
            ],
            label: Some("Fallback Tile Bind Group"),
        });

        let fetcher = TileFetcher::new(tx, config.base_imagery_url.clone(), config.offline_mode);
        let cache = TileCacheManager::new(config.max_cache_size, config.negative_cache_duration);

        Self {
            cache,
            rx,
            fetcher,
            bind_group_layout,
            sampler,
            fallback_bind_group,
            budget_bytes: config.tile_cache_budget_bytes,
            max_entries: config.max_cache_size,
            bytes_per_tile: None,
        }
    }

    pub fn request_tile(&mut self, id: TileId, priority: TilePriority) {
        if self.cache.get_state(&id).is_some() {
            return;
        }

        self.cache.mark_fetching(id);
        self.fetcher.request_tile(id, priority);
    }

    pub fn update(&mut self, device: &wgpu::Device, queue: &wgpu::Queue) {
        while let Ok((id, result)) = self.rx.try_recv() {
            self.process_tile_result(device, queue, id, result);
        }
    }

    fn process_tile_result(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        id: TileId,
        result: Result<TileImage, String>,
    ) {
        // Check if we still care about this tile (it hasn't been evicted from LRU)
        let is_still_needed = matches!(
            self.cache.get_state(&id),
            Some(crate::globe::tiles::tile_cache::TileState::Fetching)
        );

        if !is_still_needed {
            return; // Drop the result, we don't need it anymore
        }

        match result {
            Ok((width, height, rgba)) => {
                let size = wgpu::Extent3d {
                    width,
                    height,
                    depth_or_array_layers: 1,
                };

                let texture = device.create_texture(&wgpu::TextureDescriptor {
                    label: Some(&format!("Tile Texture {:?}", id)),
                    size,
                    mip_level_count: 1,
                    sample_count: 1,
                    dimension: wgpu::TextureDimension::D2,
                    format: wgpu::TextureFormat::Rgba8UnormSrgb,
                    usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
                    view_formats: &[],
                });

                queue.write_texture(
                    wgpu::ImageCopyTexture {
                        texture: &texture,
                        mip_level: 0,
                        origin: wgpu::Origin3d::ZERO,
                        aspect: wgpu::TextureAspect::All,
                    },
                    &rgba,
                    wgpu::ImageDataLayout {
                        offset: 0,
                        bytes_per_row: Some(4 * width),
                        rows_per_image: Some(height),
                    },
                    size,
                );

                let view = texture.create_view(&wgpu::TextureViewDescriptor::default());

                let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
                    layout: &self.bind_group_layout,
                    entries: &[
                        wgpu::BindGroupEntry {
                            binding: 0,
                            resource: wgpu::BindingResource::TextureView(&view),
                        },
                        wgpu::BindGroupEntry {
                            binding: 1,
                            resource: wgpu::BindingResource::Sampler(&self.sampler),
                        },
                    ],
                    label: Some(&format!("Tile Bind Group {:?}", id)),
                });

                self.cache.mark_ready(id, (texture, bind_group));
                self.apply_budget(width as usize * height as usize * 4);
            }
            Err(e) => {
                log::error!(
                    "Failed to fetch tile z:{} x:{} y:{}: {}",
                    id.z,
                    id.x,
                    id.y,
                    e
                );
                self.cache.mark_failed(id);
            }
        }
    }

    pub async fn fetch_and_upload_all(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        visible_tiles: &[(TileId, glam::Vec3, f32)],
    ) {
        loop {
            // Check if all requested tiles are resolved
            let mut ready_count = 0;
            for (id, _, _) in visible_tiles {
                let is_fetching = matches!(
                    self.cache.get_state(id),
                    Some(crate::globe::tiles::tile_cache::TileState::Fetching)
                );
                if !is_fetching {
                    ready_count += 1;
                }
            }
            if ready_count == visible_tiles.len() {
                break;
            }

            // Await the next tile from the background thread
            if let Some((id, result)) = self.rx.recv().await {
                self.process_tile_result(device, queue, id, result);
            } else {
                // Sender dropped, break to avoid infinite loop
                break;
            }
        }

        // Just in case there are any lingering fast-resolved messages in the queue
        self.update(device, queue);
    }

    /// Re-derives the entry count from the byte budget now that a tile's real
    /// decoded size is known, and shrinks the cache if it was sized for
    /// smaller tiles. A no-op while the size is unchanged, which is every tile
    /// after the first of a given imagery style.
    fn apply_budget(&mut self, bytes_per_tile: usize) {
        if self.bytes_per_tile == Some(bytes_per_tile) || bytes_per_tile == 0 {
            return;
        }
        self.bytes_per_tile = Some(bytes_per_tile);
        let capacity = crate::globe::tiles::config::tile_cache_entries_for(
            self.budget_bytes,
            bytes_per_tile,
            self.max_entries,
        );
        log::info!(
            "Tile cache: {}KiB per tile, {}MB budget -> {} entries (cap {})",
            bytes_per_tile / 1024,
            self.budget_bytes / (1024 * 1024),
            capacity.get(),
            self.max_entries.get()
        );
        self.cache.resize(capacity);
    }

    /// Overrides the entry count directly, ignoring the byte budget until the
    /// next tile size change re-derives it.
    pub fn resize(&mut self, new_capacity: std::num::NonZeroUsize) {
        self.cache.resize(new_capacity);
    }

    pub fn is_loading_complete(&self) -> bool {
        !self.cache.has_fetching()
    }

    pub fn clear(&mut self) {
        self.cache.clear();
        while self.rx.try_recv().is_ok() {}
    }
}
