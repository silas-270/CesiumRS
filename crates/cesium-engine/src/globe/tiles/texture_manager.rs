use crate::globe::quadtree::TileId;
use crate::globe::tiles::config::{TileEngineConfig, TileSourceMode, DEFAULT_IMAGERY_TEXTURE_SIZE_PX};
use crate::globe::tiles::tile_cache::TileCacheManager;
use crate::globe::tiles::tile_fetcher::{TileFetcher, TileImage, TilePriority};
use tokio::sync::mpsc;

/// Maximum number of imagery textures uploaded to the GPU in a single frame.
///
/// When the camera zooms out rapidly, large numbers of tile fetches can complete
/// simultaneously and pile up in the channel. Without a cap, the unbounded drain
/// in [`TileTextureManager::update`] uploads all of them in one call, which can
/// exhaust VRAM on integrated GPUs (causing `SurfaceError::OutOfMemory`). The
/// remaining backlog drains naturally over subsequent frames at this budget per frame,
/// which is imperceptible at 60 fps (worst-case a full burst of 200 tiles clears in
/// ~7 frames, under 120 ms). Analogous to `MESH_REBUILD_BUDGET_PER_FRAME` in
/// `globe/tiles/system.rs`.
pub const TEXTURE_UPLOAD_BUDGET_PER_FRAME: usize = 30;

/// Wall-clock ceiling on one frame's texture uploads, on top of the count cap above.
///
/// The count alone does not bound the frame: an upload is a `create_texture`, a 256 kB
/// to 1 MB `write_texture` copy and a bind group, and thirty of them measured 4–8 ms of
/// the update thread in `testing::rendering::terrain_rapid_pan` — nearly all of
/// `stream=` during a fast pan once height decoding had moved off-thread. At least one
/// tile is always uploaded, so the backlog always drains.
pub const TEXTURE_UPLOAD_TIME_BUDGET: std::time::Duration = std::time::Duration::from_millis(2);

/// Tracks the real decoded texel width of the current imagery style, live rather
/// than the frozen [`DEFAULT_IMAGERY_TEXTURE_SIZE_PX`] this replaces — deliberately a small, GPU-free struct (no
/// `wgpu::Device`/`Queue` in its API) so the tracking logic itself is unit-testable
/// without a GPU, unlike the rest of [`TileTextureManager`].
///
/// Assumes square tiles: every imagery style this engine serves (`standard_imagery_url()`
/// at 512x512, `satellite_imagery_url()` at 256x256) is, and `lod_factor_for` takes a
/// single scalar `texture_size_px`, not separate width/height.
#[derive(Clone, Copy, Debug, Default)]
pub struct ObservedTextureSize {
    last_seen_px: Option<f32>,
}

impl ObservedTextureSize {
    /// Records a freshly decoded tile's size. Called on every decode, not only the
    /// first, so a style whose tiles differ in size still converges — matching
    /// [`TileTextureManager::apply_budget`]'s own "re-checked per tile" rule for the
    /// byte budget, which this mirrors.
    pub fn record(&mut self, width: u32, height: u32) {
        debug_assert_eq!(
            width, height,
            "imagery tiles are assumed square; got {width}x{height}"
        );
        self.last_seen_px = Some(width as f32);
    }

    /// The live size once a tile has decoded, or [`DEFAULT_IMAGERY_TEXTURE_SIZE_PX`]
    /// before that — the same bootstrapping gap `DEFAULT_IMAGERY_TEXTURE_SIZE_PX`'s
    /// own doc comment already describes ("the texture manager only learns the real
    /// one after the first tile of a style decodes").
    pub fn current_px(&self) -> f32 {
        self.last_seen_px.unwrap_or(DEFAULT_IMAGERY_TEXTURE_SIZE_PX)
    }
}

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
    /// Live texel width of the current imagery style. See
    /// [`ObservedTextureSize`].
    texture_size: ObservedTextureSize,
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

        let fetcher = TileFetcher::new_with_source(
            tx,
            config.base_imagery_url.clone(),
            config.offline_mode,
            "Imagery",
            config.tile_source_mode.clone(),
        );
        let cache = TileCacheManager::new(config.max_cache_size, config.negative_cache_duration).labeled("tex");

        Self {
            cache,
            rx,
            fetcher,
            bind_group_layout,
            sampler,
            fallback_bind_group,
            // The slice of the tile budget left after terrain's declared share, which is
            // `tile_cache_budget_bytes` verbatim while terrain is off — see
            // `TileEngineConfig::imagery_cache_budget_bytes`.
            budget_bytes: config.imagery_cache_budget_bytes(),
            max_entries: config.max_cache_size,
            bytes_per_tile: None,
            texture_size: ObservedTextureSize::default(),
        }
    }

    /// The live imagery texel width — [`crate::globe::quadtree::lod_factor_for`]'s
    /// `texture_size_px` input, fed fresh every frame from `wgpu_state::update_logic`
    /// instead of the frozen `DEFAULT_IMAGERY_TEXTURE_SIZE_PX`. See
    /// [`ObservedTextureSize`] for the bootstrapping behaviour before any tile of the
    /// current style has decoded.
    pub fn current_texture_size_px(&self) -> f32 {
        self.texture_size.current_px()
    }

    pub fn request_tile(&mut self, id: TileId, priority: TilePriority) {
        if self.cache.peek_state(&id).is_some() {
            return;
        }

        self.cache.mark_fetching(id);
        self.fetcher.request_tile(id, priority);
    }

    /// This frame's imagery wish list — see [`TileFetcher::sync`]. Newly queued tiles
    /// are marked `Fetching`; dropped ones are forgotten so they can be asked for again.
    pub fn sync_requests(&mut self, wanted: &[(TileId, TilePriority, f32)]) {
        let (added, dropped) = self.fetcher.sync(wanted);
        for id in added {
            self.cache.mark_fetching(id);
        }
        for id in &dropped {
            self.cache.forget_fetching(id);
        }
    }

    pub fn update(&mut self, device: &wgpu::Device, queue: &wgpu::Queue) {
        // Cap GPU uploads per frame to prevent VRAM exhaustion on burst completions.
        // When the camera zooms out rapidly, hundreds of tiles can complete simultaneously;
        // draining them all in one frame caused a `SurfaceError::OutOfMemory` crash on
        // integrated GPUs. The remainder drains naturally over subsequent frames.
        let start = std::time::Instant::now();
        for i in 0..TEXTURE_UPLOAD_BUDGET_PER_FRAME {
            if i > 0 && start.elapsed() >= TEXTURE_UPLOAD_TIME_BUDGET {
                break;
            }
            match self.rx.try_recv() {
                Ok((id, result)) => self.process_tile_result(device, queue, id, result),
                Err(_) => break,
            }
        }
    }

    fn process_tile_result(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        id: TileId,
        result: Result<TileImage, String>,
    ) {
        if let Some(crate::globe::tiles::tile_cache::TileState::Ready(_)) = self.cache.peek_state(&id) {
            return; // Already ready
        }

        match result {
            Ok((width, height, rgba)) => {
                self.texture_size.record(width, height);
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

                log::debug!(
                    "[TEXTURE UPLOAD] id=z{}/x{}/y{} dims={}x{} size={}B",
                    id.z, id.x, id.y, width, height, width * height * 4
                );

                self.cache.mark_ready(id, (texture, bind_group));
                self.apply_budget(width as usize * height as usize * 4);
            }
            Err(e) => {
                log::warn!(
                    "[TEXTURE FAILED] z{}/x{}/y{}: {}",
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

    /// Changes the base imagery URL, resetting the cache and fetcher while keeping
    /// the GPU bind_group_layout, sampler, and fallback_bind_group intact and compatible
    /// with the compiled render pipelines.
    pub fn set_base_url(&mut self, url: String, offline_mode: bool) {
        self.set_base_url_with_source(url, offline_mode, TileSourceMode::HttpNetwork);
    }

    /// Like [`Self::set_base_url`] but also lets the caller switch [`TileSourceMode`].
    /// Used when `MapStyle::Offline` is activated at runtime via `ViewerHandle::map_set_style`.
    pub fn set_base_url_with_source(&mut self, url: String, offline_mode: bool, source_mode: TileSourceMode) {
        self.clear();
        self.bytes_per_tile = None;
        self.texture_size = ObservedTextureSize::default();
        let (tx, rx) = tokio::sync::mpsc::unbounded_channel();
        self.rx = rx;
        self.fetcher = TileFetcher::new_with_source(tx, url, offline_mode, "Imagery", source_mode);
    }


    /// Updates the imagery cache byte budget (e.g. when terrain is toggled) and
    /// resizes the cache accordingly if the decoded tile size is known.
    pub fn set_budget_bytes(&mut self, budget_bytes: usize) {
        self.budget_bytes = budget_bytes;
        if let Some(bpt) = self.bytes_per_tile {
            let capacity = crate::globe::tiles::config::tile_cache_entries_for(
                self.budget_bytes,
                bpt,
                self.max_entries,
            );
            self.cache.resize(capacity);
        }
    }
}
