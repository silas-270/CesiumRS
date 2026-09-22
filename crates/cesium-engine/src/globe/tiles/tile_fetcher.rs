use crate::globe::quadtree::TileId;
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

/// A queued request. `seq` grows with every request, so within a priority the heap
/// pops the **newest** first: after a fast pan the tiles just requested are the ones
/// on screen, and the ones requested a second ago are usually behind the camera.
#[derive(Debug)]
struct PrioritizedRequest {
    priority: TilePriority,
    seq: u64,
    id: TileId,
}

impl PartialEq for PrioritizedRequest {
    fn eq(&self, other: &Self) -> bool {
        self.priority == other.priority && self.seq == other.seq
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
            .then_with(|| self.seq.cmp(&other.seq))
    }
}

/// Requests waiting for a connection slot, plus every id queued **or** in flight — the
/// set `request_tile` deduplicates against.
#[derive(Default)]
struct RequestQueue {
    heap: BinaryHeap<PrioritizedRequest>,
    known: HashSet<TileId>,
    next_seq: u64,
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

        runtime.spawn(async move {
            Self::worker_loop(
                client,
                worker_queue,
                worker_notify,
                worker_tx,
                base_url,
                offline_mode,
                label,
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

    pub fn request_tile(&self, id: TileId, priority: TilePriority) {
        let mut q = self.queue.lock().unwrap();
        if q.known.insert(id) {
            log::debug!("[FETCH REQ] kind={} id=z{}/x{}/y{} prio={:?}", self.label, id.z, id.x, id.y, priority);
            let seq = q.next_seq;
            q.next_seq += 1;
            q.heap.push(PrioritizedRequest { priority, seq, id });
            self.notify.notify_one();
        }
    }

    /// Drops every **queued** request whose id `keep` rejects and returns those ids.
    /// Requests already on the wire are left alone — their bytes are half-paid for.
    ///
    /// This is what stops a fast camera sweep from leaving hundreds of requests for
    /// tiles it has already flown past at the head of the line, each costing a
    /// connection slot, a decode and an upload before the tiles now on screen get one.
    /// The caller owns the matching `Fetching` cache entries and must forget them, or
    /// they would never be requested again.
    pub fn cancel_queued(&self, mut keep: impl FnMut(&TileId) -> bool) -> Vec<TileId> {
        let mut q = self.queue.lock().unwrap();
        if q.heap.is_empty() {
            return Vec::new();
        }
        let mut cancelled = Vec::new();
        q.heap.retain(|r| {
            let k = keep(&r.id);
            if !k {
                cancelled.push(r.id);
            }
            k
        });
        for id in &cancelled {
            q.known.remove(id);
        }
        cancelled
    }

    /// Requests waiting for a connection slot (not counting those in flight).
    pub fn queued_len(&self) -> usize {
        self.queue.lock().unwrap().heap.len()
    }

    pub fn is_loading_complete(&self) -> bool {
        self.queue.lock().unwrap().known.is_empty()
    }

    async fn worker_loop(
        client: reqwest::Client,
        queue: Arc<Mutex<RequestQueue>>,
        notify: Arc<Notify>,
        tx: mpsc::UnboundedSender<(TileId, Result<TileImage, String>)>,
        base_url: String,
        offline_mode: bool,
        label: &'static str,
    ) {
        let semaphore = Arc::new(tokio::sync::Semaphore::new(16));

        loop {
            // Slot first, then pop. The other order picks the next request while all
            // slots are busy and then holds it through the wait, so the tile that goes
            // out is whatever was newest *then* — possibly seconds stale by the time
            // a slot frees, and immune to `cancel_queued` the whole time.
            let permit = semaphore.clone().acquire_owned().await.unwrap();
            let request = {
                let mut q = queue.lock().unwrap();
                q.heap.pop()
            };

            if let Some(req) = request {
                let client_clone = client.clone();
                let tx_clone = tx.clone();
                let queue_clone = queue.clone();
                let id = req.id;

                log::debug!("[FETCH POP] kind={} id=z{}/x{}/y{} prio={:?}", label, id.z, id.x, id.y, req.priority);

                let url_clone = base_url.clone();
                tokio::spawn(async move {
                    let res = if offline_mode {
                        Ok((256, 256, vec![255; 256 * 256 * 4]))
                    } else {
                        Self::fetch_and_decode(client_clone, id, url_clone, label).await
                    };
                    let _ = tx_clone.send((id, res));
                    {
                        let mut q = queue_clone.lock().unwrap();
                        q.known.remove(&id);
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
