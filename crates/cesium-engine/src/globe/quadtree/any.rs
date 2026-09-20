//! Choosing a surface model at **run time** without paying for it per node.
//!
//! `docs/terrain-plan.md` §1 makes the surface model a *type* parameter precisely so
//! that no per-node branch exists: `QuadtreeNode<Ellipsoid>` is 192 B and its traversal
//! compiles to the code it compiled to before terrain existed. But
//! `TerrainConfig::enabled` is a run-time flag, and a program cannot hold a value whose
//! type depends on one. Something has to bridge that, and the question is only *where*.
//!
//! # One match per frame at the manager boundary, not one per node
//!
//! This enum is the bridge, and it is deliberately as high up as it can be: it wraps
//! whole [`QuadtreeManager`]s, so the `match` runs **once per call into the quadtree**
//! — five times a frame, against tens of thousands of nodes visited inside them. Each
//! arm then calls a fully monomorphised traversal, so the flat arm is byte-for-byte the
//! flat engine and the terrain arm never makes the flat one wonder whether it is
//! terrain.
//!
//! The alternative — a `terrain_enabled` field read inside the per-node loop — is the
//! design §1 rejects, for two reasons that both still apply: it costs a branch in the
//! hottest code in the engine, and every field terrain needs would have to live on
//! `QuadtreeNode` whether or not terrain is on, which is what
//! `test_horizon_hot_structs_have_not_grown` exists to prevent.
//!
//! §1 calls this the "fallback if the generics get ugly" and Phase A found the generics
//! did **not** get ugly — the parameter reached `QuadtreeManager` cleanly. That is
//! exactly what makes this enum cheap: because `S` reaches the top, the switch can sit
//! at the top too, and the two instantiations meet nowhere else.
//!
//! # Cost of the second instantiation
//!
//! Two monomorphisations of the traversal instead of one — more instructions in the
//! binary, none of them on the other's path. That is the price, it is paid in code size
//! rather than in frame time, and it buys `QuadtreeNode<Ellipsoid>` staying exactly the
//! struct it was.

use glam::Vec3;

use super::bounding_volume::Frustum;
use super::quadtree::{CullPipeline, QuadtreeManager};
use super::surface::Ellipsoid;
use super::tile_id::TileId;
use crate::globe::terrain::{HeightBoundsSource, HeightTileManager, Heightfield};

/// A quadtree over whichever surface model this run of the engine selected.
///
/// Construct with [`AnyQuadtree::for_terrain`] and then treat it as a
/// [`QuadtreeManager`]: every method below forwards to the active arm and exists only
/// because the two arms are different types.
pub enum AnyQuadtree {
    /// `TerrainConfig::enabled == false` — the globe this engine has always drawn.
    Flat(QuadtreeManager<Ellipsoid>),
    /// `TerrainConfig::enabled == true` — bounding volumes fitted over each node's
    /// height interval (D1) and the limb test on its scaled-space bounding sphere (D2).
    Terrain(QuadtreeManager<Heightfield>),
}

impl AnyQuadtree {
    /// The tree for a given terrain setting.
    ///
    /// Not a toggle: switching surface model means rebuilding the tree from its roots,
    /// because every node's boxes are fitted by the model. Terrain is selected at
    /// `WgpuState::new` and stays selected, which is also all
    /// `TileSystem::build_height_manager` supports today.
    pub fn for_terrain(terrain: bool) -> Self {
        if terrain {
            Self::Terrain(QuadtreeManager::<Heightfield>::for_surface())
        } else {
            Self::Flat(QuadtreeManager::<Ellipsoid>::new())
        }
    }

    /// Which culling stages run. Set once, at construction.
    pub fn set_pipeline(&mut self, pipeline: CullPipeline) {
        match self {
            Self::Flat(q) => q.pipeline = pipeline,
            Self::Terrain(q) => q.pipeline = pipeline,
        }
    }

    /// The three per-frame knobs `wgpu_state` recomputes every frame, set together
    /// because they are always set together.
    pub fn set_frame_params(&mut self, lod_factor: f32, max_zoom: u8, fog_density: f32) {
        match self {
            Self::Flat(q) => {
                q.lod_factor = lod_factor;
                q.max_zoom = max_zoom;
                q.fog_density = fog_density;
            }
            Self::Terrain(q) => {
                q.lod_factor = lod_factor;
                q.max_zoom = max_zoom;
                q.fog_density = fog_density;
            }
        }
    }

    /// Phase D1's per-frame bounds refresh. A no-op on the flat arm — and not merely
    /// cheap there, but *absent*: `Ellipsoid` has no `NodeExtraSource` implementation
    /// at all, so there is nothing for this to call.
    ///
    /// Call before [`Self::update`], with the height cache as it stands this frame.
    pub fn refresh_height_bounds(
        &mut self,
        heights: Option<&HeightTileManager>,
        segments: u32,
        exaggeration: f32,
    ) {
        if let (Self::Terrain(q), Some(heights)) = (self, heights) {
            q.refresh_extras(&HeightBoundsSource {
                heights,
                segments,
                exaggeration,
            });
        }
    }

    /// This frame's fog density, as [`Self::set_frame_params`] last set it. Read by
    /// the fog capture harness, which reports it alongside the picture.
    pub fn fog_density(&self) -> f32 {
        match self {
            Self::Flat(q) => q.fog_density,
            Self::Terrain(q) => q.fog_density,
        }
    }

    pub fn update(&mut self, frustum: &Frustum) {
        match self {
            Self::Flat(q) => q.update(frustum),
            Self::Terrain(q) => q.update(frustum),
        }
    }

    pub fn get_visible_tiles(&self) -> Vec<(TileId, Vec3, f32)> {
        match self {
            Self::Flat(q) => q.get_visible_tiles(),
            Self::Terrain(q) => q.get_visible_tiles(),
        }
    }

    pub fn get_renderable_tiles<F: FnMut(&TileId) -> bool>(
        &self,
        is_ready: F,
    ) -> Vec<(TileId, Vec3, f32)> {
        match self {
            Self::Flat(q) => q.get_renderable_tiles(is_ready),
            Self::Terrain(q) => q.get_renderable_tiles(is_ready),
        }
    }
}
