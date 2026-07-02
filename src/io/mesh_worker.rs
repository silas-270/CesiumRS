use std::collections::HashSet;
use std::sync::mpsc;

use crate::globe::geometry::TileMesh;
use crate::globe::quadtree::TileId;

pub struct MeshWorkerPool {
    sender: mpsc::Sender<(TileId, TileMesh)>,
    receiver: mpsc::Receiver<(TileId, TileMesh)>,
    requested: HashSet<TileId>,
}

impl MeshWorkerPool {
    pub fn new() -> Self {
        let (sender, receiver) = mpsc::channel();
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

        tokio::task::spawn_blocking(move || {
            let mesh = TileMesh::generate(&id, segments);
            // Ignore the error if the receiver has been dropped
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
}
