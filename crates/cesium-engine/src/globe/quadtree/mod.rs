#![allow(clippy::module_inception)]
pub mod bounding_volume;
pub mod fog;
pub mod horizon;
pub mod quadtree;
pub mod slab;
pub mod surface;
pub mod tile_id;

pub use bounding_volume::{Frustum, OrientedBoundingBox, PlaneVerdict};
pub use fog::{cesium_fog, fog_density_for, FogConfig, MEGAMETERS_TO_METERS};
pub use horizon::{point_is_occluded, transform_to_scaled_space, HorizonCamera, TilePatch};
pub use quadtree::{
    lod_factor_for, CullContext, CullPipeline, LodDistanceMode, QuadtreeManager, QuadtreeNode,
    Stage, StageVerdict, MAX_STAGES,
};
pub use surface::{Ellipsoid, SurfaceModel, VertexSample};
pub use tile_id::{
    tile_bounds, tile_bounds_unstretched, web_mercator_y_to_lat_f64, TileBounds, TileId, MAX_ZOOM,
};
