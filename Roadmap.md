# CesiumRS - Flight Tracker Engine Roadmap

## 🎯 End Goal
A highly performant, custom 3D flight tracker engine built in Rust. It ingests an external array of coordinates and timestamps, smoothly interpolates the route in 3D space, and animates a 3D airplane model along that path. It operates as the core rendering backend, exposing a clean interface to a frontend UI (e.g., Kotlin on Android).

**Key Features to Deliver:**
- **Smooth Flight Paths:** 3D Spline interpolation for seamless movement and banking.
- **Dynamic Camera:** Three modes (Free, Tracking, Cockpit).
- **Environment:** Lightweight, GPU-friendly atmospheric effects (Skydome, sunset glow, Fresnel rim).
- **Cross-Platform:** Runs flawlessly on Desktop and Android using WebGPU/wgpu.

---

## 🛤️ Development Steps (Sorted for Logic & Fun)

### Step 1: The Mathematic Foundation (Path & Interpolation)
*The logical start. We need the data structure and the math before we can render anything new.*
- [X] Create a `flight` module to ingest the external JSON/Array of `[Latitude, Longitude, Altitude, Timestamp]`.
- [X] Implement a Spline Interpolation algorithm (e.g., Catmull-Rom) to generate smooth intermediate coordinates.
- [X] Calculate the derivatives: Find the forward vector (for Pitch/Yaw) and the change in direction (for Roll/Banking in curves).

### Step 2: Drawing the Route (Visualizing the Math)
*Seeing the math directly on the globe is highly rewarding and helps debugging.*
- [X] Create a new WebGPU render pipeline for lines.
- [X] Render the interpolated route as a 3D line (or tube) hovering slightly above the globe.
- [X] Ensure the line correctly follows the curvature of the WGS84 ellipsoid.

### Step 3: Enter the Airplane (glTF & Transforms)
*Bringing life to the application.*
- [X] Integrate a basic 3D model loader (e.g., using the `gltf` crate).
- [X] Create a mesh rendering pipeline to draw the static airplane model.
- [X] Apply the transformation matrix in each frame: Update the airplane's ECEF position and apply the quaternion rotations for Pitch, Yaw, and Roll based on the Step 1 data.
- [X] Connect this to the main `App` update loop (advancing simulation time).

### Step 4: The Camera Director (Modes)
*Now that the plane is flying, we need to observe it correctly.*
- [X] Refactor the camera system to support a `CameraController` trait.
- [X] **Free Mode:** Refine the existing earth-centered god camera.
- [X] **Tracking Mode:** Set the camera pivot to the plane's ECEF position. User input controls orbit and zoom relative to the plane.
- [X] **Cockpit Mode:** Lock camera position and view direction strictly to the airplane's transform. Disable user interaction.

### Step 5: The "Wow" Factor (Environment & Atmosphere)
*The most fun part: Making it look premium with almost zero GPU cost.*
- [ ] **Skydome / Skybox:** Render a background cube/dome with a dynamic gradient (or star texture) based on the camera view vector.
- [ ] **Atmospheric Halo (Fresnel):** Add a glowing rim around the earth in the terrain fragment shader using the dot product of the view vector and surface normal.
- [ ] **Sunset / Terrain Tinting:** Pass a `sun_direction` uniform to the terrain shader. Tint the tiles dynamically (e.g., orange/purple near the terminator line, darker on the night side) using simple Lambertian lighting calculations.

### Step 6: Polish & API Binding
*Bridging the gap to the final app.*
- [ ] Expose an API for the Android/Kotlin layer (e.g., `set_route()`, `set_time()`, `set_camera_mode()`).
- [ ] Finalize the FFI (Foreign Function Interface) bindings.
- [ ] Performance profiling and memory leak checks on mobile hardware.
