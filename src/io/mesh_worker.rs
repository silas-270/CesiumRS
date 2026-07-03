use std::collections::HashSet;
use std::sync::{mpsc, Arc};
use tokio::runtime::Runtime;

use crate::globe::geometry::TileMesh;
use crate::globe::quadtree::TileId;
use crate::io::providers::TerrainProvider;

pub struct MeshWorkerPool {
    _runtime: Runtime,
    sender: mpsc::Sender<(TileId, Result<TileMesh, String>)>,
    receiver: mpsc::Receiver<(TileId, Result<TileMesh, String>)>,
    requested: HashSet<TileId>,
    provider: Arc<dyn TerrainProvider>,
}

impl MeshWorkerPool {
    pub fn new(provider: Arc<dyn TerrainProvider>) -> Self {
        let (sender, receiver) = mpsc::channel();
        let runtime = tokio::runtime::Builder::new_multi_thread()
            .enable_all()
            .build()
            .expect("Failed to build tokio runtime");

        Self {
            _runtime: runtime,
            sender,
            receiver,
            requested: HashSet::new(),
            provider,
        }
    }

    pub fn set_provider(&mut self, provider: Arc<dyn TerrainProvider>) {
        self.provider = provider;
        // Optionally clear requested set or cancel old tasks? For now just assign.
    }

    pub fn request_mesh(&mut self, id: TileId) {
        if self.requested.contains(&id) {
            return;
        }

        self.requested.insert(id);
        let sender = self.sender.clone();
        let provider = self.provider.clone();

        self._runtime.spawn(async move {
            let res = provider.request_tile_geometry(&id).await;
            let _ = sender.send((id, res));
        });
    }

    pub fn process_results(&mut self) -> Vec<(TileId, TileMesh)> {
        let mut results = Vec::new();
        while let Ok((id, res)) = self.receiver.try_recv() {
            self.requested.remove(&id);
            match res {
                Ok(mesh) => results.push((id, mesh)),
                Err(e) => log::error!("Failed to fetch mesh for {:?}: {}", id, e),
            }
        }
        results
    }
}
