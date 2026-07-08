# Public Interfaces Overview

This file documents all pub items across the codebase and briefly explains why they are public.

| File | Type | Name | Reason |
| :--- | :--- | :--- | :--- |
| `src/lib.rs` | `mod` | `engine` | Library entry point |
| `src/lib.rs` | `mod` | `viewer` | Library entry point |
| `src/lib.rs` | `mod` | `flight` | Library entry point |
| `src/lib.rs` | `mod` | `testing` | Library entry point |
| `src/lib.rs` | `use` | `viewer` | Library entry point |
| `src/lib.rs` | `fn` | `run` | Library entry point |
| `src/viewer.rs` | `struct` | `GlobeOptions` | Public viewer API |
| `src/viewer.rs` | `struct` | `ViewerOptions` | Public viewer API |
| `src/viewer.rs` | `struct` | `Viewer` | Public viewer API |
| `src/viewer.rs` | `fn` | `new` | Constructor for module struct |
| `src/viewer.rs` | `fn` | `run` | Public viewer API |
| `src/engine/mod.rs` | `mod` | `core` | Used across internal modules |
| `src/engine/mod.rs` | `mod` | `render` | Used across internal modules |
| `src/engine/mod.rs` | `mod` | `globe` | Used across internal modules |
| `src/engine/mod.rs` | `mod` | `camera` | Used across internal modules |
| `src/engine/mod.rs` | `mod` | `time` | Used across internal modules |
| `src/engine/mod.rs` | `mod` | `math` | Used across internal modules |
| `src/engine/mod.rs` | `mod` | `property` | Used across internal modules |
| `src/engine/mod.rs` | `mod` | `entity` | Used across internal modules |
| `src/engine/camera/camera.rs` | `enum` | `CameraMode` | Camera viewing math |
| `src/engine/camera/camera.rs` | `struct` | `Camera` | Camera viewing math |
| `src/engine/camera/camera.rs` | `fn` | `new` | Constructor for module struct |
| `src/engine/camera/camera.rs` | `fn` | `set_eye` | Camera viewing math |
| `src/engine/camera/camera.rs` | `fn` | `global_transform_f64` | Camera viewing math |
| `src/engine/camera/camera.rs` | `fn` | `global_transform` | Camera viewing math |
| `src/engine/camera/camera.rs` | `fn` | `set_anchor` | Camera viewing math |
| `src/engine/camera/camera.rs` | `fn` | `set_local_transform` | Camera viewing math |
| `src/engine/camera/camera.rs` | `fn` | `set_distance_clamp` | Camera viewing math |
| `src/engine/camera/camera.rs` | `fn` | `orbit_anchor` | Camera viewing math |
| `src/engine/camera/camera.rs` | `fn` | `rotate_local` | Camera viewing math |
| `src/engine/camera/camera.rs` | `fn` | `translate_local` | Camera viewing math |
| `src/engine/camera/camera.rs` | `fn` | `pitch` | Camera viewing math |
| `src/engine/camera/camera.rs` | `fn` | `zoom` | Camera viewing math |
| `src/engine/camera/camera.rs` | `fn` | `orbit_mouse` | Camera viewing math |
| `src/engine/camera/camera.rs` | `fn` | `look_around` | Camera viewing math |
| `src/engine/camera/camera.rs` | `fn` | `get_view_matrix` | Camera viewing math |
| `src/engine/camera/camera.rs` | `fn` | `altitude` | Camera viewing math |
| `src/engine/camera/camera.rs` | `fn` | `get_projection_matrix` | Camera viewing math |
| `src/engine/camera/camera.rs` | `fn` | `get_projection_matrix_f64` | Camera viewing math |
| `src/engine/camera/camera.rs` | `fn` | `get_view_matrix_f64` | Camera viewing math |
| `src/engine/camera/camera.rs` | `fn` | `calculate_frustum_planes` | Camera viewing math |
| `src/engine/camera/camera.rs` | `fn` | `screen_to_world_ray` | Camera viewing math |
| `src/engine/camera/camera.rs` | `fn` | `intersect_ellipsoid` | Camera viewing math |
| `src/engine/camera/camera.rs` | `fn` | `begin_drag` | Camera viewing math |
| `src/engine/camera/camera.rs` | `fn` | `drag` | Camera viewing math |
| `src/engine/camera/camera.rs` | `fn` | `end_drag` | Camera viewing math |
| `src/engine/camera/camera.rs` | `struct` | `GodCamera` | Camera viewing math |
| `src/engine/camera/camera.rs` | `fn` | `new` | Constructor for module struct |
| `src/engine/camera/camera.rs` | `fn` | `update` | Camera viewing math |
| `src/engine/camera/camera.rs` | `fn` | `process_mouse` | Camera viewing math |
| `src/engine/camera/camera.rs` | `fn` | `get_view_matrix` | Camera viewing math |
| `src/engine/camera/camera.rs` | `fn` | `get_projection_matrix` | Camera viewing math |
| `src/engine/camera/camera.rs` | `fn` | `global_transform_f64` | Camera viewing math |
| `src/engine/camera/camera.rs` | `fn` | `calculate_frustum_planes` | Camera viewing math |
| `src/engine/camera/mod.rs` | `mod` | `camera` | Camera viewing math |
| `src/engine/camera/mod.rs` | `use` | `camera` | Camera viewing math |
| `src/engine/core/app.rs` | `struct` | `App` | Engine core application |
| `src/engine/core/app.rs` | `fn` | `new` | Constructor for module struct |
| `src/engine/core/app.rs` | `fn` | `wgpu_state_mut` | Engine core application |
| `src/engine/core/app.rs` | `fn` | `window` | Engine core application |
| `src/engine/core/extension.rs` | `trait` | `GlobeExtension` | Engine core application |
| `src/engine/core/mod.rs` | `mod` | `app` | Engine core application |
| `src/engine/core/mod.rs` | `mod` | `extension` | Engine core application |
| `src/engine/entity/mod.rs` | `struct` | `Entity` | Used across internal modules |
| `src/engine/entity/mod.rs` | `fn` | `new` | Constructor for module struct |
| `src/engine/entity/mod.rs` | `struct` | `EntityCollection` | Used across internal modules |
| `src/engine/entity/mod.rs` | `fn` | `new` | Constructor for module struct |
| `src/engine/entity/mod.rs` | `fn` | `add` | Used across internal modules |
| `src/engine/entity/mod.rs` | `fn` | `get` | Used across internal modules |
| `src/engine/entity/mod.rs` | `fn` | `get_mut` | Used across internal modules |
| `src/engine/entity/mod.rs` | `fn` | `iter` | Used across internal modules |
| `src/engine/globe/geometry.rs` | `const` | `EARTH_RADIUS_A_F32` | Globe geometry processing |
| `src/engine/globe/geometry.rs` | `const` | `EARTH_RADIUS_B_F32` | Globe geometry processing |
| `src/engine/globe/geometry.rs` | `const` | `EARTH_RADIUS_A_F64` | Globe geometry processing |
| `src/engine/globe/geometry.rs` | `const` | `EARTH_RADIUS_B_F64` | Globe geometry processing |
| `src/engine/globe/geometry.rs` | `struct` | `Vertex` | Globe geometry processing |
| `src/engine/globe/geometry.rs` | `fn` | `desc` | Globe geometry processing |
| `src/engine/globe/geometry.rs` | `fn` | `lon_lat_to_ecef_f64` | Globe geometry processing |
| `src/engine/globe/geometry.rs` | `fn` | `lon_lat_alt_to_ecef_f64` | Globe geometry processing |
| `src/engine/globe/geometry.rs` | `struct` | `TileMesh` | Globe geometry processing |
| `src/engine/globe/geometry.rs` | `fn` | `generate` | Globe geometry processing |
| `src/engine/globe/mod.rs` | `mod` | `geometry` | Globe geometry processing |
| `src/engine/globe/mod.rs` | `mod` | `quadtree` | Globe geometry processing |
| `src/engine/globe/mod.rs` | `mod` | `terrain_parser` | Globe geometry processing |
| `src/engine/globe/mod.rs` | `mod` | `tiles` | Globe geometry processing |
| `src/engine/globe/quadtree.rs` | `fn` | `web_mercator_y_to_lat` | Globe geometry processing |
| `src/engine/globe/quadtree.rs` | `struct` | `TileId` | Globe geometry processing |
| `src/engine/globe/quadtree.rs` | `fn` | `parent` | Globe geometry processing |
| `src/engine/globe/quadtree.rs` | `struct` | `QuadtreeNode` | Globe geometry processing |
| `src/engine/globe/quadtree.rs` | `struct` | `OrientedBoundingBox` | Globe geometry processing |
| `src/engine/globe/quadtree.rs` | `struct` | `Frustum` | Globe geometry processing |
| `src/engine/globe/quadtree.rs` | `fn` | `from_planes` | Globe geometry processing |
| `src/engine/globe/quadtree.rs` | `fn` | `contains_point` | Globe geometry processing |
| `src/engine/globe/quadtree.rs` | `fn` | `intersects_obb` | Globe geometry processing |
| `src/engine/globe/quadtree.rs` | `fn` | `new` | Constructor for module struct |
| `src/engine/globe/quadtree.rs` | `fn` | `subdivide` | Globe geometry processing |
| `src/engine/globe/quadtree.rs` | `fn` | `update` | Globe geometry processing |
| `src/engine/globe/quadtree.rs` | `fn` | `collect_visible_tiles` | Globe geometry processing |
| `src/engine/globe/quadtree.rs` | `fn` | `collect_renderable_tiles` | Globe geometry processing |
| `src/engine/globe/quadtree.rs` | `struct` | `QuadtreeManager` | Globe geometry processing |
| `src/engine/globe/quadtree.rs` | `fn` | `new` | Constructor for module struct |
| `src/engine/globe/quadtree.rs` | `fn` | `update` | Globe geometry processing |
| `src/engine/globe/quadtree.rs` | `fn` | `get_visible_tiles` | Globe geometry processing |
| `src/engine/globe/quadtree.rs` | `fn` | `get_renderable_tiles` | Globe geometry processing |
| `src/engine/globe/terrain_parser.rs` | `struct` | `QuantizedMeshHeader` | Globe geometry processing |
| `src/engine/globe/terrain_parser.rs` | `struct` | `QuantizedMeshTile` | Globe geometry processing |
| `src/engine/globe/terrain_parser.rs` | `struct` | `QuantizedVertices` | Globe geometry processing |
| `src/engine/globe/terrain_parser.rs` | `struct` | `EdgeIndices` | Globe geometry processing |
| `src/engine/globe/terrain_parser.rs` | `enum` | `ParseError` | Globe geometry processing |
| `src/engine/globe/terrain_parser.rs` | `fn` | `parse_quantized_mesh` | Globe geometry processing |
| `src/engine/globe/tiles/config.rs` | `struct` | `TileEngineConfig` | Globe geometry processing |
| `src/engine/globe/tiles/mesh_worker.rs` | `struct` | `MeshWorkerPool` | Globe geometry processing |
| `src/engine/globe/tiles/mesh_worker.rs` | `fn` | `new` | Constructor for module struct |
| `src/engine/globe/tiles/mesh_worker.rs` | `fn` | `request_mesh` | Globe geometry processing |
| `src/engine/globe/tiles/mesh_worker.rs` | `fn` | `process_results` | Globe geometry processing |
| `src/engine/globe/tiles/mesh_worker.rs` | `fn` | `is_loading_complete` | Globe geometry processing |
| `src/engine/globe/tiles/mesh_worker.rs` | `fn` | `clear` | Globe geometry processing |
| `src/engine/globe/tiles/mod.rs` | `mod` | `config` | Globe geometry processing |
| `src/engine/globe/tiles/mod.rs` | `mod` | `mesh_worker` | Globe geometry processing |
| `src/engine/globe/tiles/mod.rs` | `mod` | `system` | Globe geometry processing |
| `src/engine/globe/tiles/mod.rs` | `mod` | `texture_manager` | Globe geometry processing |
| `src/engine/globe/tiles/mod.rs` | `mod` | `tile_cache` | Globe geometry processing |
| `src/engine/globe/tiles/mod.rs` | `mod` | `tile_fetcher` | Globe geometry processing |
| `src/engine/globe/tiles/system.rs` | `struct` | `RenderData` | Globe geometry processing |
| `src/engine/globe/tiles/system.rs` | `struct` | `TileSystem` | Globe geometry processing |
| `src/engine/globe/tiles/system.rs` | `fn` | `new` | Constructor for module struct |
| `src/engine/globe/tiles/system.rs` | `fn` | `update` | Globe geometry processing |
| `src/engine/globe/tiles/system.rs` | `fn` | `compute_fallback_uv` | Globe geometry processing |
| `src/engine/globe/tiles/system.rs` | `fn` | `peek_render_data` | Globe geometry processing |
| `src/engine/globe/tiles/system.rs` | `fn` | `get_render_data` | Globe geometry processing |
| `src/engine/globe/tiles/system.rs` | `fn` | `is_loading_complete` | Globe geometry processing |
| `src/engine/globe/tiles/texture_manager.rs` | `struct` | `TileTextureManager` | Globe geometry processing |
| `src/engine/globe/tiles/texture_manager.rs` | `fn` | `new` | Constructor for module struct |
| `src/engine/globe/tiles/texture_manager.rs` | `fn` | `request_tile` | Globe geometry processing |
| `src/engine/globe/tiles/texture_manager.rs` | `fn` | `update` | Globe geometry processing |
| `src/engine/globe/tiles/texture_manager.rs` | `fn` | `resize` | Globe geometry processing |
| `src/engine/globe/tiles/texture_manager.rs` | `fn` | `is_loading_complete` | Globe geometry processing |
| `src/engine/globe/tiles/texture_manager.rs` | `fn` | `clear` | Globe geometry processing |
| `src/engine/globe/tiles/tile_cache.rs` | `enum` | `TileState` | Globe geometry processing |
| `src/engine/globe/tiles/tile_cache.rs` | `struct` | `TileCacheManager` | Globe geometry processing |
| `src/engine/globe/tiles/tile_cache.rs` | `fn` | `new` | Constructor for module struct |
| `src/engine/globe/tiles/tile_cache.rs` | `fn` | `get_state` | Globe geometry processing |
| `src/engine/globe/tiles/tile_cache.rs` | `fn` | `peek_state` | Globe geometry processing |
| `src/engine/globe/tiles/tile_cache.rs` | `fn` | `mark_fetching` | Globe geometry processing |
| `src/engine/globe/tiles/tile_cache.rs` | `fn` | `mark_ready` | Globe geometry processing |
| `src/engine/globe/tiles/tile_cache.rs` | `fn` | `mark_failed` | Globe geometry processing |
| `src/engine/globe/tiles/tile_cache.rs` | `fn` | `resize` | Globe geometry processing |
| `src/engine/globe/tiles/tile_cache.rs` | `fn` | `has_fetching` | Globe geometry processing |
| `src/engine/globe/tiles/tile_cache.rs` | `fn` | `clear` | Globe geometry processing |
| `src/engine/globe/tiles/tile_fetcher.rs` | `enum` | `TilePriority` | Globe geometry processing |
| `src/engine/globe/tiles/tile_fetcher.rs` | `struct` | `TileFetcher` | Globe geometry processing |
| `src/engine/globe/tiles/tile_fetcher.rs` | `fn` | `new` | Constructor for module struct |
| `src/engine/globe/tiles/tile_fetcher.rs` | `fn` | `request_tile` | Globe geometry processing |
| `src/engine/globe/tiles/tile_fetcher.rs` | `fn` | `is_loading_complete` | Globe geometry processing |
| `src/engine/math/interpolation.rs` | `fn` | `linear_dvec3` | Shared mathematical utility |
| `src/engine/math/interpolation.rs` | `fn` | `hermite_dvec3` | Shared mathematical utility |
| `src/engine/math/interpolation.rs` | `fn` | `catmull_rom_dvec3` | Shared mathematical utility |
| `src/engine/math/interpolation.rs` | `fn` | `linear_f64` | Shared mathematical utility |
| `src/engine/math/interpolation.rs` | `fn` | `hermite_f64` | Shared mathematical utility |
| `src/engine/math/interpolation.rs` | `fn` | `catmull_rom_f64` | Shared mathematical utility |
| `src/engine/math/mod.rs` | `mod` | `interpolation` | Shared mathematical utility |
| `src/engine/math/mod.rs` | `mod` | `transform` | Shared mathematical utility |
| `src/engine/math/mod.rs` | `mod` | `trajectory` | Shared mathematical utility |
| `src/engine/math/trajectory.rs` | `struct` | `TransformState` | Shared mathematical utility |
| `src/engine/math/trajectory.rs` | `struct` | `TrajectoryEvaluator` | Shared mathematical utility |
| `src/engine/math/trajectory.rs` | `fn` | `new` | Constructor for module struct |
| `src/engine/math/trajectory.rs` | `fn` | `evaluate_raw` | Shared mathematical utility |
| `src/engine/math/trajectory.rs` | `fn` | `evaluate` | Shared mathematical utility |
| `src/engine/math/transform.rs` | `fn` | `surface_normal_ecef` | Shared mathematical utility |
| `src/engine/math/transform.rs` | `fn` | `enu_matrix_at_ecef` | Shared mathematical utility |
| `src/engine/math/transform.rs` | `fn` | `velocity_to_orientation` | Shared mathematical utility |
| `src/engine/property/mod.rs` | `mod` | `sampled` | Shared property abstraction |
| `src/engine/property/mod.rs` | `trait` | `Property` | Shared property abstraction |
| `src/engine/property/mod.rs` | `struct` | `ConstantProperty` | Shared property abstraction |
| `src/engine/property/mod.rs` | `fn` | `new` | Constructor for module struct |
| `src/engine/property/sampled.rs` | `enum` | `InterpolationAlgorithm` | Shared property abstraction |
| `src/engine/property/sampled.rs` | `struct` | `SampledPositionProperty` | Shared property abstraction |
| `src/engine/property/sampled.rs` | `fn` | `new` | Constructor for module struct |
| `src/engine/property/sampled.rs` | `fn` | `with_algorithm` | Shared property abstraction |
| `src/engine/property/sampled.rs` | `fn` | `add_sample` | Shared property abstraction |
| `src/engine/property/sampled.rs` | `fn` | `start_time` | Shared property abstraction |
| `src/engine/property/sampled.rs` | `fn` | `stop_time` | Shared property abstraction |
| `src/engine/property/sampled.rs` | `fn` | `samples` | Shared property abstraction |
| `src/engine/property/sampled.rs` | `struct` | `SampledScalarProperty` | Shared property abstraction |
| `src/engine/property/sampled.rs` | `fn` | `new` | Constructor for module struct |
| `src/engine/property/sampled.rs` | `fn` | `with_algorithm` | Shared property abstraction |
| `src/engine/property/sampled.rs` | `fn` | `add_sample` | Shared property abstraction |
| `src/engine/render/capture.rs` | `fn` | `capture_pixels` | Engine rendering abstraction |
| `src/engine/render/capture.rs` | `fn` | `capture_screenshot` | Engine rendering abstraction |
| `src/engine/render/debug_geometry.rs` | `struct` | `DebugVertex` | Engine rendering abstraction |
| `src/engine/render/debug_geometry.rs` | `fn` | `desc` | Engine rendering abstraction |
| `src/engine/render/debug_geometry.rs` | `fn` | `get_frustum_corners` | Engine rendering abstraction |
| `src/engine/render/debug_geometry.rs` | `fn` | `append_crosshair_lines` | Engine rendering abstraction |
| `src/engine/render/debug_geometry.rs` | `fn` | `append_frustum_lines` | Engine rendering abstraction |
| `src/engine/render/mod.rs` | `mod` | `wgpu_state` | Engine rendering abstraction |
| `src/engine/render/mod.rs` | `mod` | `debug_geometry` | Engine rendering abstraction |
| `src/engine/render/mod.rs` | `mod` | `pipelines` | Engine rendering abstraction |
| `src/engine/render/mod.rs` | `mod` | `capture` | Engine rendering abstraction |
| `src/engine/render/mod.rs` | `mod` | `polyline` | Engine rendering abstraction |
| `src/engine/render/mod.rs` | `mod` | `model` | Engine rendering abstraction |
| `src/engine/render/pipelines.rs` | `fn` | `create_pipelines` | Engine rendering abstraction |
| `src/engine/render/pipelines.rs` | `fn` | `create_sky_pipeline` | Engine rendering abstraction |
| `src/engine/render/wgpu_state.rs` | `struct` | `TileBuffers` | Engine rendering abstraction |
| `src/engine/render/wgpu_state.rs` | `struct` | `TilePushConstants` | Engine rendering abstraction |
| `src/engine/render/wgpu_state.rs` | `struct` | `TileDisplayEntry` | Engine rendering abstraction |
| `src/engine/render/wgpu_state.rs` | `struct` | `WgpuState` | Engine rendering abstraction |
| `src/engine/render/wgpu_state.rs` | `fn` | `resize` | Engine rendering abstraction |
| `src/engine/render/wgpu_state.rs` | `fn` | `get_fetch_stats` | Engine rendering abstraction |
| `src/engine/render/wgpu_state.rs` | `fn` | `resize_tile_cache` | Engine rendering abstraction |
| `src/engine/render/wgpu_state.rs` | `fn` | `update_tile_cache` | Engine rendering abstraction |
| `src/engine/render/wgpu_state.rs` | `fn` | `capture_pixels` | Engine rendering abstraction |
| `src/engine/render/wgpu_state.rs` | `fn` | `render` | Engine rendering abstraction |
| `src/engine/render/model/pipeline.rs` | `struct` | `ModelVertex` | Engine rendering abstraction |
| `src/engine/render/model/pipeline.rs` | `fn` | `desc` | Engine rendering abstraction |
| `src/engine/render/model/pipeline.rs` | `struct` | `ModelPushConstants` | Engine rendering abstraction |
| `src/engine/render/model/pipeline.rs` | `struct` | `ModelRenderer` | Engine rendering abstraction |
| `src/engine/render/model/pipeline.rs` | `fn` | `new` | Constructor for module struct |
| `src/engine/render/model/pipeline.rs` | `fn` | `draw` | Rendering command submission |
| `src/engine/render/polyline/builder.rs` | `struct` | `PolylineVertex` | Engine rendering abstraction |
| `src/engine/render/polyline/builder.rs` | `fn` | `desc` | Engine rendering abstraction |
| `src/engine/render/polyline/builder.rs` | `struct` | `AdaptiveSubdivisionBuilder` | Engine rendering abstraction |
| `src/engine/render/polyline/builder.rs` | `fn` | `new` | Constructor for module struct |
| `src/engine/render/polyline/builder.rs` | `fn` | `build` | Builder pattern execution |
| `src/engine/render/polyline/bvh.rs` | `struct` | `PolylineNode` | Engine rendering abstraction |
| `src/engine/render/polyline/bvh.rs` | `struct` | `PolylineBVH` | Engine rendering abstraction |
| `src/engine/render/polyline/bvh.rs` | `fn` | `build` | Builder pattern execution |
| `src/engine/render/polyline/bvh.rs` | `fn` | `collect_visible_segments` | Engine rendering abstraction |
| `src/engine/render/polyline/bvh.rs` | `fn` | `generate_vertices` | Engine rendering abstraction |
| `src/engine/render/polyline/mod.rs` | `mod` | `builder` | Engine rendering abstraction |
| `src/engine/render/polyline/mod.rs` | `mod` | `pipeline` | Engine rendering abstraction |
| `src/engine/render/polyline/mod.rs` | `mod` | `bvh` | Engine rendering abstraction |
| `src/engine/render/polyline/pipeline.rs` | `struct` | `PolylinePushConstants` | Engine rendering abstraction |
| `src/engine/render/polyline/pipeline.rs` | `struct` | `PolylineConfig` | Engine rendering abstraction |
| `src/engine/render/polyline/pipeline.rs` | `struct` | `PolylineRenderer` | Engine rendering abstraction |
| `src/engine/render/polyline/pipeline.rs` | `fn` | `new` | Constructor for module struct |
| `src/engine/render/polyline/pipeline.rs` | `fn` | `update_geometry` | Engine rendering abstraction |
| `src/engine/render/polyline/pipeline.rs` | `fn` | `draw` | Rendering command submission |
| `src/engine/time/mod.rs` | `struct` | `SimulationTime` | Used across internal modules |
| `src/engine/time/mod.rs` | `fn` | `new` | Constructor for module struct |
| `src/engine/time/mod.rs` | `struct` | `Clock` | Used across internal modules |
| `src/engine/time/mod.rs` | `fn` | `new` | Constructor for module struct |
| `src/engine/time/mod.rs` | `fn` | `tick` | Used across internal modules |
| `src/flight/app.rs` | `fn` | `load_flight_data` | Flight tracking module |
| `src/flight/app.rs` | `struct` | `FlightEntity` | Flight tracking module |
| `src/flight/app.rs` | `struct` | `FlightTrackerApp` | Flight tracking module |
| `src/flight/app.rs` | `fn` | `new` | Constructor for module struct |
| `src/flight/app.rs` | `fn` | `get_plane_state_at_time_delta` | Flight tracking module |
| `src/flight/app.rs` | `fn` | `get_plane_state_at` | Flight tracking module |
| `src/flight/app.rs` | `fn` | `get_sun_intensity_at` | Flight tracking module |
| `src/flight/app.rs` | `fn` | `add_flight_path` | Flight tracking module |
| `src/flight/mod.rs` | `mod` | `app` | Flight tracking module |
| `src/flight/mod.rs` | `mod` | `modes` | Flight tracking module |
| `src/flight/modes/cockpit.rs` | `fn` | `update_cockpit_mode` | Flight tracking module |
| `src/flight/modes/free.rs` | `fn` | `update_free_mode` | Flight tracking module |
| `src/flight/modes/mod.rs` | `mod` | `free` | Flight tracking module |
| `src/flight/modes/mod.rs` | `mod` | `tracking` | Flight tracking module |
| `src/flight/modes/mod.rs` | `mod` | `cockpit` | Flight tracking module |
| `src/flight/modes/tracking.rs` | `fn` | `update_tracking_mode` | Flight tracking module |
| `src/testing/mod.rs` | `mod` | `simulator` | Internal testing utility |
| `src/testing/mod.rs` | `mod` | `test_app` | Internal testing utility |
| `src/testing/mod.rs` | `mod` | `stress_app` | Internal testing utility |
| `src/testing/mod.rs` | `mod` | `regression_app` | Internal testing utility |
| `src/testing/mod.rs` | `mod` | `test_flicker_tracking` | Internal testing utility |
| `src/testing/mod.rs` | `mod` | `test_tile_monitor` | Internal testing utility |
| `src/testing/mod.rs` | `struct` | `VerifyConfig` | Internal testing utility |
| `src/testing/mod.rs` | `mod` | `test_tracking_camera` | Internal testing utility |
| `src/testing/regression_app.rs` | `struct` | `TestState` | Internal testing utility |
| `src/testing/regression_app.rs` | `struct` | `RegressionApp` | Internal testing utility |
| `src/testing/regression_app.rs` | `fn` | `new` | Constructor for module struct |
| `src/testing/simulator.rs` | `enum` | `SimulatedAction` | Internal testing utility |
| `src/testing/simulator.rs` | `struct` | `Simulator` | Internal testing utility |
| `src/testing/simulator.rs` | `fn` | `parse` | Internal testing utility |
| `src/testing/simulator.rs` | `fn` | `pump_events` | Internal testing utility |
| `src/testing/stress_app.rs` | `struct` | `StressApp` | Internal testing utility |
| `src/testing/stress_app.rs` | `fn` | `new` | Constructor for module struct |
| `src/testing/test_app.rs` | `struct` | `TestApp` | Internal testing utility |
| `src/testing/test_app.rs` | `fn` | `new` | Constructor for module struct |
| `src/testing/test_culling_false_negatives.rs` | `fn` | `run_test` | Internal testing utility |
| `src/testing/test_flicker_tracking.rs` | `struct` | `FlickerTrackingApp` | Internal testing utility |
| `src/testing/test_flicker_tracking.rs` | `fn` | `new` | Constructor for module struct |
| `src/testing/test_tile_monitor.rs` | `struct` | `TileMonitorApp` | Internal testing utility |
| `src/testing/test_tile_monitor.rs` | `fn` | `new` | Constructor for module struct |
| `crates/cesium-engine/src/label/mod.rs` | `struct` | `LabelManager` | Manager for loading and culling globe place labels |
| `crates/cesium-engine/src/label/mod.rs` | `fn` | `new` | Constructor for LabelManager |
| `crates/cesium-engine/src/label/mod.rs` | `fn` | `update` | Runs the spatial and depth-culling pipeline |
| `crates/cesium-engine/src/label/mod.rs` | `pub field` | `enabled` | Toggle switch to enable/disable label culling and rendering |
| `crates/cesium-engine/src/label/mod.rs` | `pub field` | `size_scale` | Scaling factor for label font sizes |
| `crates/cesium-engine/src/label/mod.rs` | `pub field` | `max_importance_rank` | Filter threshold for place prominence (0 to 15) |
| `crates/cesium-engine/src/label/mod.rs` | `pub field` | `show_anchor_dots` | Toggle to render white screen anchor dots |
| `crates/cesium-engine/src/label/mod.rs` | `struct` | `VisibleLabel` | Struct holding a single culled place ready for rendering |


