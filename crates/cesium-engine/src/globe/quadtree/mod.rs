#![allow(clippy::module_inception)]
pub mod any;
pub mod bounding_volume;
pub mod fog;
pub mod horizon;
pub mod quadtree;
pub mod slab;
pub mod surface;
pub mod terrain_occlusion;
pub mod tile_id;

pub use any::AnyQuadtree;
pub use bounding_volume::{Frustum, OrientedBoundingBox, PlaneVerdict};
pub use fog::{cesium_fog, fog_density_for, FogConfig, MEGAMETERS_TO_METERS};
pub use horizon::{
    point_is_occluded, sphere_is_occluded, transform_to_scaled_space, HorizonCamera, ScaledSphere,
    TilePatch,
};
pub use quadtree::{
    lod_factor_for, terrain_lod_factor_for, CullContext, CullPipeline, LodDistanceMode,
    NodeExtraSource, QuadtreeManager, QuadtreeNode, Stage, StageVerdict, TerrainFogPolicy,
    MAX_STAGES,
};
pub use surface::{Ellipsoid, SurfaceModel, VertexSample};
pub use terrain_occlusion::{
    OccluderStep, TerrainHorizon, TerrainOcclusionConfig, AZIMUTH_SECTORS, RANGE_RINGS,
};
pub use tile_id::{
    tile_bounds, tile_bounds_unstretched, web_mercator_y_to_lat_f64, TileBounds, TileId, MAX_ZOOM,
};
