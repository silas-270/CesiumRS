# Handoff — sky / sunset update (state as of 2026-09-24)

Read this first. Also read AGENTS.md,
especially **performance-budget-1-percent** and **build-with-cargo-remote** (never compile locally; use tar-over-ssh to lxhalle).

## Goal
The most realistic sunset/twilight sky possible, at the best performance.
**HARD RULE:** every rendering change may cost at most 1% performance compared with before. Cheat with LUTs, gradients or precomputation. No per-pixel or per-vertex ray marching.

## Where we left off
- Committed `b0be140`: a physically based sky in `render/atmosphere.wgsl` (Rayleigh + Mie + ozone, Earth's shadow, Belt of Venus).
  - It ray-marches 16 steps per sky pixel, which violates the 1% rule.
- **Uncommitted rework in the working tree.** An agent stopped mid-work, so it may be half-done:
  - New `crates/cesium-engine/src/render/sky_lut/` (`mod.rs`, `sky_lut.wgsl`): the atmosphere baked into a 128x98 Rgba16Float LUT, keyed on sun elevation and camera height.
    - It is re-rendered only when sin(sun elevation) changes by more than 1e-3 or the camera height by more than 3%.
    - Each re-render is spread over 4 frame bands.
    - The sky and globe sample the LUT instead of ray-marching.
  - Also modified: `atmosphere.wgsl`, `sky.wgsl`, `sky_pipeline/mod.rs`, `globe_pipeline/{shader.wgsl,pipeline.rs}`, `render/mod.rs`, `camera_uniform.rs`, `wgpu_state.rs` (includes `scene_timestamps` GPU timing), `docs/lighting.md`, `src/testing/rendering/mod.rs`.
  - New `src/testing/rendering/sky_perf.rs`: GPU timestamp harness.
    - Run with `cargo test --lib sky_perf -- --ignored --nocapture`.
    - Environment variables `SKY_PERF_FRAMES`, `SKY_PERF_BATCHES`, `SKY_PERF_SCENES` (dark_noon, dark_sunset, sat_noon, sat_sunset, sat_sunset_moving).
  - **`crates/cesium-engine/src/core/app.rs` contains the USER's own uncommitted edit** (the `Ordering` import and `_event_loop.exit()`). Never commit or revert it. Stage only your own files by path.
- **User report:** when last run, the sky showed heavy bugs: frame drops in specific situations, lag, and no smooth animation. The cause is not known yet.
- Open visual issues from the earlier screenshots:
  - The sun is too weak. It should be a bright disc with a clear glow, reddened and flattened near the horizon.
  - At −6° the anti-solar sky is already night-dark with stars. The end of civil twilight should still be clearly lit.
  - The noon horizon is paler and hazier than before (a matter of taste).
- Earlier perf numbers (`~/CesiumRS_handoff/perf/`) are very noisy because of thermal effects. Don't rely on them.

## Useful files outside the repo (`~/CesiumRS_handoff/`)
- `atmo.py`, `anti.py`: numpy copy of the sky model.
- `perf/`: `run_ab.sh`, `analyze.py`, raw results.
- `build_perf.sh`, `run.sh`, `fetch.sh`: build on lxhalle, run, copy back. The paths may point to an old scratchpad; adjust them.
- `compare_*.png`, `lut4/`: earlier captures.

## Resolution & Current State (Committed in `19feaa9`)
All reported issues have been resolved, verified visually via headless test mode, and committed:

1. **Architecture & Performance (Single-Pass Continuous LUT)**:
   - Removed the 4-band temporal slicing, height thresholds, and sun epsilon hysteresis from `crates/cesium-engine/src/render/sky_lut/mod.rs`.
   - The sky LUT (128x98 texels) is now re-rendered in a single pass each frame (`self.sky_lut.update(encoder)`).
   - This eliminates rolling-shutter temporal band tearing across the sky and removes frame-stepping/lag during camera movement and flight.

2. **Twilight & Atmosphere Scattering Fixes**:
   - Softened Earth's shadow penumbra in `crates/cesium-engine/src/render/atmosphere.wgsl` (`smoothstep(ATMO_R - 15.0, ATMO_R + 15.0, r_low)` instead of a 1 km razor edge).
   - Fixed denominator explosion in `atmo_ms_source` when `mu -> -1` (anti-solar direction), preventing anti-solar sky from blacking out prematurely.
   - Added `sky_lut_sky_uv` to clamp `theta <= theta_horizon`, preventing sky dome pixels in `sky.wgsl` from sampling ground-intercepted rows (which caused the massive purple dome artifact in twilight).
   - Tuned aerosol scattering (`ATMO_BETA_M_SCA = 2.8e-3`) and multiple scattering floor (`ATMO_MS_FLOOR = 0.08`) to eliminate pale, milky noon horizon haze.
   - Fixed star extinction curve in `sky.wgsl` so that civil twilight (-6°) remains star-free and naturally lit.

3. **Solar Disc & Aureole**:
   - Tuned sun disc with white-hot core, warm limb darkening, and atmospheric refraction lift/flattening.
   - Removed artificial wide procedural glow dome and applied a smoothstep window (`glow_window = smoothstep(0.97, 0.985, cos_sun)`), blending seamlessly into the sky LUT's continuous Mie forward scattering and completely eliminating the 14° circular boundary artifact.

4. **Visual Verification**:
   - All elevations (+10°, +3°, 0°, -3°, -6°, -10°) across ground, air, zenith, and anti-solar angles verified via `src/testing/rendering/sunset_capture.rs`.
