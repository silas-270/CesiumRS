use std::collections::HashSet;
use std::sync::mpsc;

use crate::globe::geometry::TileMesh;
use crate::globe::quadtree::surface::Ellipsoid;
use crate::globe::quadtree::TileId;
use crate::globe::terrain::heightfield::{HeightPatch, Heightfield};

/// Which surface model a queued mesh is built on, with its build input already in hand.
///
/// This is where `docs/terrain-plan.md` §1's "monomorphise, don't branch" meets the
/// fact that the switch *is* a runtime config flag. The branch happens **once per
/// mesh**, here, on the update thread — not once per vertex and never inside the
/// quadtree's per-node loop, which is the thing constraint 1 protects. Each arm then
/// calls a separately monomorphised `generate_on`, so the flat arm compiles to exactly
/// what it compiled to before terrain existed.
///
/// The height patch is sampled by the caller, not by the worker: see
/// [`crate::globe::quadtree::surface::SurfaceModel::BuildCtx`] for why that matters to
/// Phase E2.
pub enum MeshBuild {
    /// The flat globe. Zero-sized input, `generate_on::<Ellipsoid>`.
    Flat,
    /// Relief, from an already-sampled patch. Boxed because it is ~3 kB at
    /// `mesh_segments = 16` and this enum is moved into a channel.
    Terrain(Box<HeightPatch>),
}

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

    pub fn request_mesh(&mut self, id: TileId, segments: u32, build: MeshBuild) {
        if self.requested.contains(&id) {
            return;
        }

        self.requested.insert(id);
        let sender = self.sender.clone();

        // Use rayon for CPU-bound work — no async runtime needed.
        rayon::spawn(move || {
            let mesh = match build {
                MeshBuild::Flat => TileMesh::generate_on::<Ellipsoid>(&id, segments, &()),
                MeshBuild::Terrain(patch) => {
                    TileMesh::generate_on::<Heightfield>(&id, segments, &patch)
                }
            };
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
