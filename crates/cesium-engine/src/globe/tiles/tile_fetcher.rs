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

#[derive(Debug)]
struct PrioritizedRequest {
    priority: TilePriority,
    id: TileId,
}

impl PartialEq for PrioritizedRequest {
    fn eq(&self, other: &Self) -> bool {
        self.priority == other.priority && self.id == other.id
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
        self.priority.cmp(&other.priority)
    }
}

pub struct TileFetcher {
    runtime: Option<Runtime>,
    queue: Arc<Mutex<(BinaryHeap<PrioritizedRequest>, HashSet<TileId>)>>,
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

        let queue = Arc::new(Mutex::new((BinaryHeap::new(), HashSet::new())));
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
        if !q.1.contains(&id) {
            log::info!("[FETCH REQ] kind={} id=z{}/x{}/y{} prio={:?}", self.label, id.z, id.x, id.y, priority);
            q.1.insert(id);
            q.0.push(PrioritizedRequest { priority, id });
            self.notify.notify_one();
        }
    }

    pub fn is_loading_complete(&self) -> bool {
        self.queue.lock().unwrap().1.is_empty()
    }

    async fn worker_loop(
        client: reqwest::Client,
        queue: Arc<Mutex<(BinaryHeap<PrioritizedRequest>, HashSet<TileId>)>>,
        notify: Arc<Notify>,
        tx: mpsc::UnboundedSender<(TileId, Result<TileImage, String>)>,
        base_url: String,
        offline_mode: bool,
        label: &'static str,
    ) {
        let semaphore = Arc::new(tokio::sync::Semaphore::new(16));

        loop {
            // Get the next request or wait
            let request = {
                let mut q = queue.lock().unwrap();
                q.0.pop()
            };

            if let Some(req) = request {
                let permit = semaphore.clone().acquire_owned().await.unwrap();
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
                        q.1.remove(&id);
                    }
                    drop(permit);
                });
            } else {
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
                log::info!(
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
