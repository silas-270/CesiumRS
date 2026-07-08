use std::collections::HashSet;
use std::sync::mpsc;

use crate::globe::geometry::TileMesh;
use crate::globe::quadtree::TileId;

pub struct MeshWorkerPool {
    sender: mpsc::SyncSender<(TileId, TileMesh)>,
    receiver: mpsc::Receiver<(TileId, TileMesh)>,
    requested: HashSet<TileId>,
}

impl Default for MeshWorkerPool {
    fn default() -> Self {
        Self::new()
    }
}

impl MeshWorkerPool {
    pub fn new() -> Self {
        // Use a bounded sync channel. If the channel fills up, spawn_blocking will block
        // which is fine since it's on a rayon worker thread.
        let (sender, receiver) = mpsc::sync_channel(512);
        Self {
            sender,
            receiver,
            requested: HashSet::new(),
        }
    }

    pub fn request_mesh(&mut self, id: TileId, segments: u32) {
        if self.requested.contains(&id) {
            return;
        }

        self.requested.insert(id);
        let sender = self.sender.clone();

        // Use rayon for CPU-bound work — no async runtime needed.
        rayon::spawn(move || {
            let mesh = TileMesh::generate(&id, segments);
            let _ = sender.send((id, mesh));
        });
    }

    pub fn process_results(&mut self) -> Vec<(TileId, TileMesh)> {
        let mut results = Vec::new();
        while let Ok((id, mesh)) = self.receiver.try_recv() {
            self.requested.remove(&id);
            results.push((id, mesh));
        }
        results
    }

    pub fn is_loading_complete(&self) -> bool {
        self.requested.is_empty()
    }

    pub fn clear(&mut self) {
        self.requested.clear();
        while self.receiver.try_recv().is_ok() {}
    }
}
