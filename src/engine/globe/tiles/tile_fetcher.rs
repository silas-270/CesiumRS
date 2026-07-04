use crate::engine::globe::quadtree::TileId;
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashSet};
use std::sync::{mpsc, Arc, Mutex};
use tokio::runtime::Runtime;
use tokio::sync::Notify;

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
    _runtime: Runtime,
    queue: Arc<Mutex<(BinaryHeap<PrioritizedRequest>, HashSet<TileId>)>>,
    notify: Arc<Notify>,
}

impl TileFetcher {
    pub fn new(tx: mpsc::Sender<(TileId, Result<Vec<u8>, String>)>, base_url: String, offline_mode: bool) -> Self {
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
            Self::worker_loop(client, worker_queue, worker_notify, worker_tx, base_url, offline_mode).await;
        });

        Self {
            _runtime: runtime,
            queue,
            notify,
        }
    }

    pub fn request_tile(&self, id: TileId, priority: TilePriority) {
        let mut q = self.queue.lock().unwrap();
        if !q.1.contains(&id) {
            q.1.insert(id);
            q.0.push(PrioritizedRequest { priority, id });
            self.notify.notify_one();
        }
    }

    pub fn is_loading_complete(&self) -> bool {
        self.queue.lock().unwrap().0.is_empty()
    }

    async fn worker_loop(
        client: reqwest::Client,
        queue: Arc<Mutex<(BinaryHeap<PrioritizedRequest>, HashSet<TileId>)>>,
        notify: Arc<Notify>,
        tx: mpsc::Sender<(TileId, Result<Vec<u8>, String>)>,
        base_url: String,
        offline_mode: bool,
    ) {
        let semaphore = Arc::new(tokio::sync::Semaphore::new(8));

        loop {
            // Get the next request or wait
            let request = {
                let mut q = queue.lock().unwrap();
                let req = q.0.pop();
                if let Some(r) = &req {
                    q.1.remove(&r.id);
                }
                req
            };

            if let Some(req) = request {
                let permit = semaphore.clone().acquire_owned().await.unwrap();
                let client_clone = client.clone();
                let tx_clone = tx.clone();
                let id = req.id;

                let url_clone = base_url.clone();
                tokio::spawn(async move {
                    let res = if offline_mode {
                        Ok(vec![255; 256 * 256 * 4])
                    } else {
                        Self::fetch_and_decode(client_clone, id, url_clone).await
                    };
                    let _ = tx_clone.send((id, res));
                    drop(permit);
                });
            } else {
                notify.notified().await;
            }
        }
    }

    async fn fetch_and_decode(client: reqwest::Client, id: TileId, base_url: String) -> Result<Vec<u8>, String> {
        let url = base_url
            .replace("{z}", &id.z.to_string())
            .replace("{x}", &id.x.to_string())
            .replace("{y}", &id.y.to_string());

        let response = client
            .get(&url)
            .send()
            .await
            .map_err(|e| format!("Request failed: {}", e))?;

        if !response.status().is_success() {
            return Err(format!("HTTP error: {}", response.status()));
        }

        let bytes = response
            .bytes()
            .await
            .map_err(|e| format!("Failed to read bytes: {}", e))?;

        let result = tokio::task::spawn_blocking(move || {
            image::load_from_memory(&bytes)
                .map(|img| img.to_rgba8().into_raw())
                .map_err(|e| format!("Image decode error: {}", e))
        })
        .await
        .map_err(|e| format!("Task panic: {}", e))?;

        result
    }
}
