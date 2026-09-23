use crate::globe::quadtree::TileId;
use crate::globe::tiles::config::TileSourceMode;
use crate::globe::tiles::vector::SvgTileRenderer;
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashSet};
use std::sync::{Arc, Mutex};
use tokio::runtime::Runtime;
use tokio::sync::{mpsc, Notify};


/// A decoded tile image: `(width, height, RGBA8 pixels)`. Dimensions travel
/// with the pixels because imagery sources differ in tile size — the Carto
/// basemap serves 512x512 (`@2x`) while Esri satellite serves 256x256.
pub type TileImage = (u32, u32, Vec<u8>);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TilePriority {
    High, // e.g., visible tile
    Low,  // e.g., prefetch tile
}

impl Ord for TilePriority {
    fn cmp(&self, other: &Self) -> Ordering {
        match (self, other) {
            (TilePriority::High, TilePriority::High) => Ordering::Equal,
            (TilePriority::Low, TilePriority::Low) => Ordering::Equal,
            (TilePriority::High, TilePriority::Low) => Ordering::Greater,
            (TilePriority::Low, TilePriority::High) => Ordering::Less,
        }
    }
}

impl PartialOrd for TilePriority {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// A queued request, ordered by class and then **importance** — the tile's size over
/// its distance, i.e. roughly how much of the screen it covers (Cesium orders its load
/// queue the same way). `gen` identifies the entry that is current for its tile: a
/// re-ranked tile gets a new entry and the old one is skipped when popped.
#[derive(Debug)]
struct PrioritizedRequest {
    priority: TilePriority,
    importance: f32,
    gen: u64,
    id: TileId,
}

impl PartialEq for PrioritizedRequest {
    fn eq(&self, other: &Self) -> bool {
        self.gen == other.gen
    }
}

impl Eq for PrioritizedRequest {}

impl PartialOrd for PrioritizedRequest {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for PrioritizedRequest {
    fn cmp(&self, other: &Self) -> Ordering {
        self.priority
            .cmp(&other.priority)
            .then_with(|| self.importance.total_cmp(&other.importance))
            .then_with(|| other.gen.cmp(&self.gen))
    }
}

/// A tile waiting for a connection slot.
struct Queued {
    gen: u64,
    priority: TilePriority,
    importance: f32,
    /// When a [`TileFetcher::sync`] last asked for it.
    wanted_at: std::time::Instant,
}

/// How long a queued tile may go unrequested before [`TileFetcher::sync`] drops it. A
/// short grace, so a tile flickering across a LOD boundary keeps its place. Wall time,
/// not frames: at a few hundred frames a second a frame count is gone in a blink.
const CANCEL_GRACE: std::time::Duration = std::time::Duration::from_millis(500);

#[derive(Default)]
struct RequestQueue {
    heap: BinaryHeap<PrioritizedRequest>,
    queued: std::collections::HashMap<TileId, Queued>,
    in_flight: HashSet<TileId>,
    next_gen: u64,
}

impl RequestQueue {
    fn push(&mut self, id: TileId, priority: TilePriority, importance: f32) {
        let gen = self.next_gen;
        self.next_gen += 1;
        self.heap.push(PrioritizedRequest { priority, importance, gen, id });
        let wanted_at = std::time::Instant::now();
        self.queued.insert(id, Queued { gen, priority, importance, wanted_at });
    }

    /// The most important current request, stale entries skipped.
    fn pop(&mut self) -> Option<PrioritizedRequest> {
        while let Some(req) = self.heap.pop() {
            if self.queued.get(&req.id).is_some_and(|q| q.gen == req.gen) {
                self.queued.remove(&req.id);
                self.in_flight.insert(req.id);
                return Some(req);
            }
        }
        None
    }

    /// Drops superseded heap entries once they outnumber the live ones.
    fn compact(&mut self) {
        if self.heap.len() > 4 * self.queued.len() + 256 {
            self.heap = self
                .queued
                .iter()
                .map(|(id, q)| PrioritizedRequest {
                    priority: q.priority,
                    importance: q.importance,
                    gen: q.gen,
                    id: *id,
                })
                .collect();
        }
    }
}

pub struct TileFetcher {
    runtime: Option<Runtime>,
    queue: Arc<Mutex<RequestQueue>>,
    notify: Arc<Notify>,
    pub label: &'static str,
}

impl Drop for TileFetcher {
    fn drop(&mut self) {
        if let Some(rt) = self.runtime.take() {
            rt.shutdown_background();
        }
    }
}

impl TileFetcher {
    pub fn new(
        tx: tokio::sync::mpsc::UnboundedSender<(TileId, Result<TileImage, String>)>,
        base_url: String,
        offline_mode: bool,
        label: &'static str,
    ) -> Self {
        Self::new_with_source(tx, base_url, offline_mode, label, TileSourceMode::default())
    }

    /// Full constructor that accepts a [`TileSourceMode`].
    ///
    /// [`Self::new`] is a convenience wrapper kept for the many existing call-sites that
    /// use `base_url + offline_mode` — those use [`TileSourceMode::HttpNetwork`] implicitly.
    pub fn new_with_source(
        tx: tokio::sync::mpsc::UnboundedSender<(TileId, Result<TileImage, String>)>,
        base_url: String,
        offline_mode: bool,
        label: &'static str,
        source_mode: TileSourceMode,
    ) -> Self {
        let runtime = tokio::runtime::Builder::new_multi_thread()
            .enable_all()
            .build()
            .expect("Failed to build tokio runtime");

        let client = reqwest::Client::builder()
            .user_agent("CesiumRS/0.1.0")
            .timeout(std::time::Duration::from_secs(5))
            .build()
            .expect("Failed to build reqwest client");

        let queue = Arc::new(Mutex::new(RequestQueue::default()));
        let notify = Arc::new(Notify::new());

        let worker_queue = queue.clone();
        let worker_notify = notify.clone();
        let worker_tx = tx.clone();

        // Extract the SVG renderer if we're in vector mode (cheap Arc clone).
        let svg_renderer: Option<SvgTileRenderer> = match source_mode {
            TileSourceMode::SvgVector(r) => Some((*r).clone()),
            TileSourceMode::HttpNetwork => None,
        };

        runtime.spawn(async move {
            Self::worker_loop(
                client,
                worker_queue,
                worker_notify,
                worker_tx,
                base_url,
                offline_mode,
                label,
                svg_renderer,
            )
            .await;
        });

        Self {
            runtime: Some(runtime),
            queue,
            notify,
            label,
        }
    }

    /// Queues one tile at the lowest importance of its class. For callers outside the
    /// per-frame [`Self::sync`] (tests, one-off loads); the engine uses `sync`.
    pub fn request_tile(&self, id: TileId, priority: TilePriority) {
        let mut q = self.queue.lock().unwrap();
        if q.in_flight.contains(&id) || q.queued.contains_key(&id) {
            return;
        }
        log::debug!("[FETCH REQ] kind={} id=z{}/x{}/y{} prio={:?}", self.label, id.z, id.x, id.y, priority);
        q.push(id, priority, 0.0);
        drop(q);
        self.notify.notify_one();
    }

    /// One frame's complete wish list: `(tile, class, importance)`.
    ///
    /// New tiles are queued, queued ones re-ranked, and queued tiles missing from the
    /// list for [`CANCEL_GRACE`] are dropped (requests already on the wire
    /// are left to finish). Returns `(queued now, dropped)` so the caller can mark and
    /// forget the matching cache placeholders.
    ///
    /// Replaces the newest-first queue, which starved: a moving camera requests new near
    /// tiles every frame, so anything requested earlier — the coarse ancestors every
    /// fallback depends on included — never reached the front. In one 7 s collision
    /// run the z0 height tile was requested at 0.02 s and had not arrived at the end.
    pub fn sync(&self, wanted: &[(TileId, TilePriority, f32)]) -> (Vec<TileId>, Vec<TileId>) {
        let mut q = self.queue.lock().unwrap();
        let now = std::time::Instant::now();
        let mut added = Vec::new();
        for &(id, priority, importance) in wanted {
            if q.in_flight.contains(&id) {
                continue;
            }
            match q.queued.get_mut(&id) {
                Some(entry) => {
                    entry.wanted_at = now;
                    let changed = entry.priority != priority
                        || (entry.importance - importance).abs() > 0.1 * entry.importance.abs().max(1e-6);
                    if changed {
                        q.push(id, priority, importance);
                    }
                }
                None => {
                    q.push(id, priority, importance);
                    added.push(id);
                }
            }
        }
        let dropped: Vec<TileId> = q
            .queued
            .iter()
            .filter(|(_, e)| now.duration_since(e.wanted_at) > CANCEL_GRACE)
            .map(|(id, _)| *id)
            .collect();
        for id in &dropped {
            q.queued.remove(id);
        }
        q.compact();
        drop(q);
        if !added.is_empty() {
            self.notify.notify_one();
        }
        (added, dropped)
    }

    /// Requests waiting for a connection slot (not counting those in flight).
    pub fn queued_len(&self) -> usize {
        self.queue.lock().unwrap().queued.len()
    }

    pub fn is_loading_complete(&self) -> bool {
        let q = self.queue.lock().unwrap();
        q.queued.is_empty() && q.in_flight.is_empty()
    }

    async fn worker_loop(
        client: reqwest::Client,
        queue: Arc<Mutex<RequestQueue>>,
        notify: Arc<Notify>,
        tx: mpsc::UnboundedSender<(TileId, Result<TileImage, String>)>,
        base_url: String,
        offline_mode: bool,
        label: &'static str,
        svg_renderer: Option<SvgTileRenderer>,
    ) {
        let semaphore = Arc::new(tokio::sync::Semaphore::new(16));

        loop {
            // Slot first, then pop. The other order picks the next request while all
            // slots are busy and then holds it through the wait, so the tile that goes
            // out is whatever was newest *then* — possibly seconds stale by the time
            // a slot frees, and immune to `sync` dropping it the whole time.
            let permit = semaphore.clone().acquire_owned().await.unwrap();
            let request = {
                let mut q = queue.lock().unwrap();
                q.pop()
            };

            if let Some(req) = request {
                let client_clone = client.clone();
                let tx_clone = tx.clone();
                let queue_clone = queue.clone();
                let id = req.id;

                log::debug!("[FETCH POP] kind={} id=z{}/x{}/y{} prio={:?}", label, id.z, id.x, id.y, req.priority);

                let url_clone = base_url.clone();
                let svg_clone = svg_renderer.clone();
                tokio::spawn(async move {
                    let res = if let Some(svg) = svg_clone {
                        // SVG vector mode: rasterize tile on a blocking thread so we don't
                        // stall the tokio reactor.  ~1–4 ms per tile on a desktop core.
                        tokio::task::spawn_blocking(move || svg.render_tile(id))
                            .await
                            .unwrap_or_else(|e| Err(format!("SVG rasterize task panicked: {e}")))
                    } else if offline_mode {
                        // Legacy offline stub: solid-white tiles for headless tests.
                        Ok((256, 256, vec![255; 256 * 256 * 4]))
                    } else {
                        Self::fetch_and_decode(client_clone, id, url_clone, label).await
                    };
                    let _ = tx_clone.send((id, res));
                    {
                        let mut q = queue_clone.lock().unwrap();
                        q.in_flight.remove(&id);
                    }
                    drop(permit);
                });
            } else {
                drop(permit);
                notify.notified().await;
            }
        }
    }

    async fn fetch_and_decode(
        client: reqwest::Client,
        id: TileId,
        base_url: String,
        label: &'static str,
    ) -> Result<TileImage, String> {
        let start_time = std::time::Instant::now();
        let url = base_url
            .replace("{z}", &id.z.to_string())
            .replace("{x}", &id.x.to_string())
            .replace("{y}", &id.y.to_string());

        log::debug!("[FETCH HTTP_START] kind={} id=z{}/x{}/y{} url={}", label, id.z, id.x, id.y, url);

        let response = client
            .get(&url)
            .send()
            .await
            .map_err(|e| {
                let elapsed = start_time.elapsed().as_secs_f64() * 1000.0;
                log::warn!("[FETCH ERR_NET] kind={} id=z{}/x{}/y{} elapsed={:.1}ms err={}", label, id.z, id.x, id.y, elapsed, e);
                format!("Request failed: {}", e)
            })?;

        let status = response.status();
        let http_time = start_time.elapsed().as_secs_f64() * 1000.0;

        if !status.is_success() {
            let elapsed = start_time.elapsed().as_secs_f64() * 1000.0;
            log::warn!("[FETCH ERR_HTTP] kind={} id=z{}/x{}/y{} status={} elapsed={:.1}ms", label, id.z, id.x, id.y, status, elapsed);
            return Err(format!("HTTP error: {}", status));
        }

        let bytes = response
            .bytes()
            .await
            .map_err(|e| {
                let elapsed = start_time.elapsed().as_secs_f64() * 1000.0;
                log::warn!("[FETCH ERR_READ] kind={} id=z{}/x{}/y{} elapsed={:.1}ms err={}", label, id.z, id.x, id.y, elapsed, e);
                format!("Failed to read bytes: {}", e)
            })?;

        let bytes_len = bytes.len();
        let decode_start = std::time::Instant::now();

        let result = tokio::task::spawn_blocking(move || {
            image::load_from_memory(&bytes)
                .map(|img| {
                    let rgba = img.to_rgba8();
                    (rgba.width(), rgba.height(), rgba.into_raw())
                })
                .map_err(|e| format!("Image decode error: {}", e))
        })
        .await
        .map_err(|e| {
            let elapsed = start_time.elapsed().as_secs_f64() * 1000.0;
            log::error!("[FETCH ERR_PANIC] kind={} id=z{}/x{}/y{} elapsed={:.1}ms err={}", label, id.z, id.x, id.y, elapsed, e);
            format!("Task panic: {}", e)
        })?;

        let decode_time = decode_start.elapsed().as_secs_f64() * 1000.0;
        let total_time = start_time.elapsed().as_secs_f64() * 1000.0;

        match &result {
            Ok((w, h, _)) => {
                log::debug!(
                    "[FETCH OK] kind={} id=z{}/x{}/y{} bytes={} net={:.1}ms decode={:.1}ms total={:.1}ms dims={}x{}",
                    label, id.z, id.x, id.y, bytes_len, http_time, decode_time, total_time, w, h
                );
            }
            Err(e) => {
                log::warn!(
                    "[FETCH ERR_DECODE] kind={} id=z{}/x{}/y{} elapsed={:.1}ms err={}",
                    label, id.z, id.x, id.y, total_time, e
                );
            }
        }

        result
    }
}
