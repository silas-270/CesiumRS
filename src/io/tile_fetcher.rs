use crate::globe::quadtree::TileId;
use std::cmp::Ordering;
use std::collections::BinaryHeap;
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
    queue: Arc<Mutex<BinaryHeap<PrioritizedRequest>>>,
    notify: Arc<Notify>,
}

impl TileFetcher {
    pub fn new(tx: mpsc::Sender<(TileId, Result<Vec<u8>, String>)>) -> Self {
        let runtime = tokio::runtime::Builder::new_multi_thread()
            .enable_all()
            .build()
            .expect("Failed to build tokio runtime");

        let client = reqwest::Client::builder()
            .user_agent("CesiumRS/0.1.0")
            .build()
            .expect("Failed to build reqwest client");

        let queue = Arc::new(Mutex::new(BinaryHeap::new()));
        let notify = Arc::new(Notify::new());

        let worker_queue = queue.clone();
        let worker_notify = notify.clone();
        let worker_tx = tx.clone();

        runtime.spawn(async move {
            Self::worker_loop(client, worker_queue, worker_notify, worker_tx).await;
        });

        Self {
            _runtime: runtime,
            queue,
            notify,
        }
    }

    pub fn request_tile(&self, id: TileId, priority: TilePriority) {
        let mut q = self.queue.lock().unwrap();
        q.push(PrioritizedRequest { priority, id });
        self.notify.notify_one();
    }

    pub fn is_loading_complete(&self) -> bool {
        self.queue.lock().unwrap().is_empty()
    }

    async fn worker_loop(
        client: reqwest::Client,
        queue: Arc<Mutex<BinaryHeap<PrioritizedRequest>>>,
        notify: Arc<Notify>,
        tx: mpsc::Sender<(TileId, Result<Vec<u8>, String>)>,
    ) {
        let semaphore = Arc::new(tokio::sync::Semaphore::new(8));

        loop {
            // Get the next request or wait
            let request = {
                let mut q = queue.lock().unwrap();
                q.pop()
            };

            if let Some(req) = request {
                let permit = semaphore.clone().acquire_owned().await.unwrap();
                let client_clone = client.clone();
                let tx_clone = tx.clone();
                let id = req.id;

                tokio::spawn(async move {
                    let res = Self::fetch_and_decode(client_clone, id).await;
                    let _ = tx_clone.send((id, res));
                    drop(permit);
                });
            } else {
                notify.notified().await;
            }
        }
    }

    async fn fetch_and_decode(client: reqwest::Client, id: TileId) -> Result<Vec<u8>, String> {
        let url = format!("https://tile.openstreetmap.org/{}/{}/{}.png", id.z, id.x, id.y);

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
