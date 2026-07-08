# CesiumRS

CesiumRS is a high-performance, 3D globe rendering engine written in Rust. It utilizes `wgpu` for cross-platform graphics, rendering a full WGS84 ellipsoid based on the Web Mercator projection.

This project implements a unified architecture designed to act as a robust back-end engine for GIS applications and flight trackers.

## Features

- **WGS84 Ellipsoid Rendering**: Geographically accurate rendering of the Earth, accounting for equatorial bulging and precise coordinate transformations.
- **High-Performance Tile System**: Quadtree-based tile streaming, speculative prefetching, and strict caching to maintain 60FPS at high zoom levels.
- **Flight Tracking Module (`cesium-flight`)**: Includes advanced 6-DOF camera tracking modes, flight path interpolation via Catmull-Rom splines, and polyline BVH rendering for massive flight routes.
- **Unified Public API**: Provides a thread-safe, clean, non-blocking interface (`CesiumViewer` and `ViewerHandle`) ideal for FFI / JNI integration (e.g. Kotlin/Android).

## Architecture

CesiumRS uses a Cargo Workspace divided into strict functional crates:

- `cesium_rs` (Root Crate): The front-end API boundary, test runners, and diagnostic harnesses.
- `cesium-engine`: The standalone core rendering engine, handling all `wgpu` state, math, terrain, and quadtree rendering.
- `cesium-flight`: The domain-specific plugin that implements flight data loading, camera tracking modes (Free, Tracking, Cockpit), and high-performance path rendering.

## Quickstart

```rust
use cesium_rs::{CesiumViewer, CameraMode};

fn main() {
    // 1. Create a flight tracker application plugin
    let (flight_app, flight_handle) = cesium_flight::tracker::FlightTrackerApp::with_handle();

    // 2. Build the CesiumViewer engine
    let viewer = CesiumViewer::builder()
        .tile_cache_size(2048)
        .max_screen_space_error(2.0)
        .enable_prefetch(true)
        .with_extension(Box::new(flight_app))
        .build();

    // 3. Obtain a thread-safe handle for runtime commands
    let cam = viewer.handle();

    // 4. Drive the engine from any thread
    std::thread::spawn(move || {
        flight_handle.load_flight("my_flight", include_str!("flight.json").to_string());
        flight_handle.play();
        
        // ECEF or Lon/Lat/Alt
        cam.camera_set_position(8.68, 50.11, 0.5); // Frankfurt, Germany
    });

    // 5. Take over the main thread (blocks forever)
    viewer.run();
}
```

## Testing & Diagnostics

The project features a suite of visual debugging harnesses. These test tools are strictly separated from the engine source code.

Run specific diagnostic modes using the included CLI:

```bash
# General viewer
cargo run --release

# Regression Test (Sweeps across latitudes/longitudes)
cargo run --release -- --regression

# Stress Test (High velocity, aggressive cache clearing)
cargo run --release -- --stress

# Tile Monitor (Diagnostic view for tile fetching)
cargo run --release -- --monitor
```

## Naming & Style Conventions
The codebase strictly adheres to standard Rust naming conventions (`snake_case` variables, `UpperCamelCase` types, `SCREAMING_SNAKE_CASE` constants). Ensure `cargo clippy` and `cargo fmt` are run before committing.
