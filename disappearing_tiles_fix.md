# Fixing Disappearing Tiles / Camera Clipping Issue

This document explains the root causes of the disappearing tiles (which looked like the camera going through the Earth's surface) and how they were fixed. This information can be used to troubleshoot and resolve similar issues in other 3D globe or map rendering projects.

## The Problem

When zooming in closely to the surface of the Earth, the camera appeared to go through the terrain, causing tiles to disappear. This was caused by two interacting bugs in the camera's mathematics and projection logic:

1. **Ellipsoid vs Sphere Mismatch (Collision/Bounds):** The terrain geometry was correctly generated as a WGS84 ellipsoid (which bulges at the equator and is flattened at the poles). However, the camera's collision and altitude logic incorrectly assumed a perfectly spherical Earth.
2. **Near-Plane Clipping (Visual Disappearance):** Even when the camera was kept above the surface, the projection matrix's near clipping plane was set too aggressively. When the camera got too close, the near plane would clip the ground out of existence, creating the visual illusion that the camera was inside or had passed through the Earth.

---

## The Solution (with Code References)

To fix this, three main functions within the `src/math/camera.rs` file had to be updated.

### 1. Fixing the Altitude Calculation
**Location:** `Camera::altitude()` in `src/math/camera.rs`

**Issue:** The altitude was previously calculated by simply subtracting the equatorial radius (`6.378137`) from the camera's distance to the center of the Earth (`pos.length()`). This meant that near the poles, where the Earth's surface is closer to the center (`6.356752`), the camera thought it was much higher above the ground than it actually was.

**Fix:** The function was updated to calculate the exact distance to the ellipsoid surface along the camera's positional vector.
```rust
// The exact ellipsoid radii
let a = 6.378137_f32;
let b = 6.3567523142_f32;
let inv_a2 = 1.0 / (a * a);
let inv_b2 = 1.0 / (b * b);

// Intersect the ray from the origin with the ellipsoid to find the exact surface radius at this angle
let t = 1.0 / (dir.x * dir.x * inv_a2 + dir.y * dir.y * inv_b2 + dir.z * dir.z * inv_a2).sqrt();

// True altitude is the difference between the camera's distance to the center and the surface's distance to the center
pos.length() - t
```

### 2. Fixing the Collision Bounds
**Location:** `Camera::enforce_bounds()` in `src/math/camera.rs`

**Issue:** The camera was restricted from moving closer to the center of the Earth than a fixed global minimum distance (`6.378137 + 0.000002`). This caused the camera to hit an invisible spherical wall high above the poles, while perfectly touching the surface at the equator.

**Fix:** Similar to the altitude fix, the minimum allowed distance was changed to be dynamic. The camera now calculates the ellipsoid radius (`t`) at its current angle and uses `t + 0.000002` as the absolute minimum distance it can approach. This ensures the camera stops smoothly just above the surface everywhere on the planet.

### 3. Fixing the Near-Plane Clipping (The "Disappearing" Fix)
**Location:** `Camera::get_projection_matrix()` in `src/math/camera.rs`

**Issue:** The projection matrix dynamically adjusts its near clipping plane (`znear`) based on altitude to maintain Z-buffer precision. However, it was clamped to a minimum of `0.0001` units. In a coordinate system where 1.0 unit = 1000 km, `0.0001` units is 100 meters. When the user zoomed in closer than 100 meters, the terrain was clipped away by the near plane.

**Fix:** The minimum clamp for `znear` was significantly reduced down to `0.0000001` (0.1 meters).
```rust
// Old code:
// let alt = self.altitude().max(0.0001);
// let znear = (alt * 0.1).clamp(0.0001, 10.0);

// Fixed code:
let alt = self.altitude().max(0.000002);
let znear = (alt * 0.1).clamp(0.0000001, 10.0);
```
This change is what ultimately stops the tiles from vanishing when zooming in tightly, while the altitude and bounds fixes ensure the camera correctly stops before mathematically crossing the surface.
