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
        let mode_str = match &build {
            MeshBuild::Flat => "Flat",
            MeshBuild::Terrain(_) => "Terrain",
        };
        log::debug!("[MESH REQ] id=z{}/x{}/y{} mode={}", id.z, id.x, id.y, mode_str);

        // Use rayon for CPU-bound work — no async runtime needed.
        rayon::spawn(move || {
            let start = std::time::Instant::now();
            let mesh = match build {
                MeshBuild::Flat => TileMesh::generate_on::<Ellipsoid>(&id, segments, &()),
                MeshBuild::Terrain(patch) => {
                    TileMesh::generate_on::<Heightfield>(&id, segments, &patch)
                }
            };
            log::debug!(
                "[MESH BUILT] id=z{}/x{}/y{} verts={} indices={} elapsed={:.2}ms",
                id.z, id.x, id.y, mesh.vertices.len(), mesh.indices.len(), start.elapsed().as_secs_f64() * 1000.0
            );
            let _ = sender.send((id, mesh));
        });
    }

    pub fn process_results(&mut self) -> Vec<(TileId, TileMesh)> {
        self.process_results_up_to(usize::MAX)
    }

    /// [`Self::process_results`], taking at most `max` finished meshes. The rest stay in
    /// the channel — and stay `requested`, so they are not built twice — for next frame.
    pub fn process_results_up_to(&mut self, max: usize) -> Vec<(TileId, TileMesh)> {
        let mut results = Vec::new();
        while results.len() < max {
            let Ok((id, mesh)) = self.receiver.try_recv() else {
                break;
            };
            self.requested.remove(&id);
            results.push((id, mesh));
        }
        results
    }

    /// Whether a build for `id` is already queued or running.
    ///
    /// [`Self::request_mesh`] deduplicates on this already, so a repeat request is
    /// harmless — but **Phase E2**'s rebuild budget is a budget of *slots*, and a slot
    /// spent re-offering a tile that is already on a worker is a slot no other stale
    /// tile gets. Measured before it was exposed: the staged burst in
    /// `rendering::terrain_e2_capture` queued 90 "rebuilds" to finish 74 tiles, because
    /// a tile stays stale in the cache until its new mesh lands and was therefore
    /// re-picked on every intervening frame, halving the effective budget and letting
    /// the nearest tiles block the ones behind them.
    pub fn is_requested(&self, id: &TileId) -> bool {
        self.requested.contains(id)
    }

    pub fn is_loading_complete(&self) -> bool {
        self.requested.is_empty()
    }

    pub fn clear(&mut self) {
        self.requested.clear();
        while self.receiver.try_recv().is_ok() {}
    }
}
