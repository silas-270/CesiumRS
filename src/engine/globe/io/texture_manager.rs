use crate::engine::globe::quadtree::TileId;
use std::sync::mpsc::{self, Receiver};
use crate::engine::globe::io::tile_cache::TileCacheManager;
use crate::engine::globe::io::config::TileEngineConfig;
use crate::engine::globe::io::tile_fetcher::{TilePriority, TileFetcher};

pub struct TileTextureManager {
    pub cache: TileCacheManager<(wgpu::Texture, wgpu::BindGroup)>,
    rx: Receiver<(TileId, Result<Vec<u8>, String>)>,
    pub fetcher: TileFetcher,
    pub bind_group_layout: wgpu::BindGroupLayout,
    sampler: wgpu::Sampler,
}

impl TileTextureManager {
    pub fn new(device: &wgpu::Device, config: &TileEngineConfig) -> Self {
        let (tx, rx) = mpsc::channel();

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

        let fetcher = TileFetcher::new(tx, config.base_imagery_url.clone(), config.offline_mode);
        let cache = TileCacheManager::new(config.max_cache_size, config.negative_cache_duration);

        Self {
            cache,
            rx,
            fetcher,
            bind_group_layout,
            sampler,
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
            // Check if we still care about this tile (it hasn't been evicted from LRU)
            let is_still_needed = match self.cache.get_state(&id) {
                Some(crate::engine::globe::io::tile_cache::TileState::Fetching) => true,
                _ => false,
            };

            if !is_still_needed {
                continue; // Drop the result, we don't need it anymore
            }

            match result {
                Ok(rgba) => {
                    let size = wgpu::Extent3d {
                        width: 256,
                        height: 256,
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
                            bytes_per_row: Some(4 * 256),
                            rows_per_image: Some(256),
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
    }

    pub fn resize(&mut self, new_capacity: std::num::NonZeroUsize) {
        self.cache.resize(new_capacity);
    }

    pub fn is_loading_complete(&self) -> bool {
        !self.cache.has_fetching()
    }

    pub fn clear(&mut self) {
        self.cache.clear();
        while let Ok(_) = self.rx.try_recv() {}
    }
}
