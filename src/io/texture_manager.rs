use crate::globe::quadtree::TileId;
use std::collections::{HashMap, HashSet};
use std::sync::mpsc::{self, Receiver, Sender};
use std::thread;

pub struct TileTextureManager {
    cache: HashMap<TileId, (wgpu::Texture, wgpu::BindGroup)>,
    requesting: HashSet<TileId>,
    rx: Receiver<(TileId, Result<Vec<u8>, String>)>,
    tx: Sender<(TileId, Result<Vec<u8>, String>)>,
    pub bind_group_layout: wgpu::BindGroupLayout,
    sampler: wgpu::Sampler,
}

impl TileTextureManager {
    pub fn new(device: &wgpu::Device) -> Self {
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

        Self {
            cache: HashMap::new(),
            requesting: HashSet::new(),
            rx,
            tx,
            bind_group_layout,
            sampler,
        }
    }

    pub fn request_tile(&mut self, id: TileId, _device: &wgpu::Device, _queue: &wgpu::Queue) {
        if self.cache.contains_key(&id) || self.requesting.contains(&id) {
            return;
        }

        self.requesting.insert(id);
        let tx = self.tx.clone();

        thread::spawn(move || {
            let url = format!(
                "https://tile.openstreetmap.org/{}/{}/{}.png",
                id.z, id.x, id.y
            );

            let client = reqwest::blocking::Client::builder()
                .user_agent("CesiumRS/0.1.0")
                .build();

            let res = match client.and_then(|c| c.get(&url).send()) {
                Ok(response) => {
                    if response.status().is_success() {
                        match response.bytes() {
                            Ok(bytes) => match image::load_from_memory(&bytes) {
                                Ok(img) => Ok(img.to_rgba8().into_raw()),
                                Err(e) => Err(format!("Image decode error: {}", e)),
                            },
                            Err(e) => Err(format!("Failed to read bytes: {}", e)),
                        }
                    } else {
                        Err(format!("HTTP error: {}", response.status()))
                    }
                }
                Err(e) => Err(format!("Request failed: {}", e)),
            };

            let _ = tx.send((id, res));
        });
    }

    pub fn update(&mut self, device: &wgpu::Device, queue: &wgpu::Queue) {
        while let Ok((id, result)) = self.rx.try_recv() {
            self.requesting.remove(&id);

            match result {
                Ok(rgba) => {
                    log::info!(
                        "Successfully downloaded and decoded tile z:{} x:{} y:{}",
                        id.z,
                        id.x,
                        id.y
                    );

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

                    self.cache.insert(id, (texture, bind_group));
                }
                Err(e) => {
                    log::error!(
                        "Failed to fetch tile z:{} x:{} y:{}: {}",
                        id.z,
                        id.x,
                        id.y,
                        e
                    );
                }
            }
        }
    }

    pub fn cleanup_cache(&mut self, visible_tiles: &[TileId]) {
        let visible_set: HashSet<TileId> = visible_tiles.iter().copied().collect();
        self.cache.retain(|id, _| visible_set.contains(id));
    }

    pub fn get_texture(&self, id: TileId) -> Option<&(wgpu::Texture, wgpu::BindGroup)> {
        self.cache.get(&id)
    }

    pub fn is_loading_complete(&self) -> bool {
        self.requesting.is_empty()
    }
}
