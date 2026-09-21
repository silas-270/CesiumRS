#[cfg(test)]
mod tests {
    use cesium_engine::camera::camera::CameraMode;
    use cesium_engine::globe::tiles::config::{TileEngineConfig, SATELLITE_IMAGERY_URL};
    use cesium_engine::render::wgpu_state::WgpuState;
    use std::time::Instant;

    fn run_orbit_benchmark(mode_name: &'static str, terrain_enabled: bool, base_url: &'static str) {
        let handle = std::thread::spawn(move || {
            pollster::block_on(async {
                let mut flight_app = Box::new(cesium_flight::tracker::FlightTrackerApp::new(
                    std::sync::Arc::new(std::sync::Mutex::new(0.05)), // 5% progress on STR-FRA (near climb)
                ));
                flight_app.add_flight_path(
                    "STR-FRA",
                    9.2219, 48.6899, // STR
                    8.5706, 50.0333, // FRA
                    1_800_000,
                    false,
                    Vec::new(),
                );
                flight_app.view_mode = CameraMode::Tracking;
                flight_app.last_view_mode = CameraMode::Free;
                flight_app.reset_viewport = true;

                let mut config = TileEngineConfig::default();
                config.base_imagery_url = base_url.to_string();
                config.terrain.enabled = terrain_enabled;

                let mut state = WgpuState::new(
                    None,
                    Some(winit::dpi::PhysicalSize::new(1280, 720)),
                    config,
                    Some(flight_app),
                )
                .await;

                println!("=== Benchmark: {} (terrain={}) ===", mode_name, terrain_enabled);
                for _ in 0..10 {
                    #[cfg(feature = "debug_panel")]
                    let res = state.render(None, false, |_, _| {});
                    #[cfg(not(feature = "debug_panel"))]
                    let res = state.render(None, false);
                    assert!(res.is_ok(), "Warmup render failed: {:?}", res);
                    std::thread::sleep(std::time::Duration::from_millis(50));
                }

                let mut frame_times: Vec<f64> = Vec::with_capacity(120);
                let mut peak_frame_idx = 0;
                let mut peak_frame_time = 0.0;
                let mut peak_breakdown = (0.0, 0.0, 0.0, 0.0, 0.0);

                for frame in 0..120 {
                    let start = Instant::now();

                    // Continuous orbit around the aircraft
                    state.camera.orbit_anchor(glam::Quat::from_axis_angle(glam::Vec3::Y, 0.05));

                    #[cfg(feature = "debug_panel")]
                    let res = state.render(None, false, |_, _| {});
                    #[cfg(not(feature = "debug_panel"))]
                    let res = state.render(None, false);
                    assert!(res.is_ok(), "Render failed: {:?}", res);

                    let dt = start.elapsed().as_secs_f64() * 1000.0;
                    frame_times.push(dt);

                    if dt > peak_frame_time {
                        peak_frame_time = dt;
                        peak_frame_idx = frame;
                        peak_breakdown = (
                            state.last_timings.update_logic_us / 1000.0,
                            state.last_subsystem_timings.quadtree_us / 1000.0,
                            state.last_subsystem_timings.terrain_horizon_us / 1000.0,
                            state.last_subsystem_timings.tile_streaming_us / 1000.0,
                            state.last_subsystem_timings.terrain_draw_us / 1000.0,
                        );
                    }

                    std::thread::sleep(std::time::Duration::from_millis(16));
                }

                let mut sorted = frame_times.clone();
                sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());

                let mean: f64 = frame_times.iter().sum::<f64>() / frame_times.len() as f64;
                let p50 = sorted[sorted.len() * 50 / 100];
                let p90 = sorted[sorted.len() * 90 / 100];
                let p95 = sorted[sorted.len() * 95 / 100];
                let p99 = sorted[sorted.len() * 99 / 100];
                let min = sorted[0];
                let max = sorted[sorted.len() - 1];

                println!("--- [{}] 120-frame Orbit Latency Distribution ---", mode_name);
                println!("  Min Frame Time:  {:.2} ms ({:.1} FPS)", min, 1000.0 / min);
                println!("  Avg Frame Time:  {:.2} ms ({:.1} FPS)", mean, 1000.0 / mean);
                println!("  p50 (Median):    {:.2} ms ({:.1} FPS)", p50, 1000.0 / p50);
                println!("  p90:             {:.2} ms ({:.1} FPS)", p90, 1000.0 / p90);
                println!("  p95:             {:.2} ms ({:.1} FPS)", p95, 1000.0 / p95);
                println!("  p99:             {:.2} ms ({:.1} FPS)", p99, 1000.0 / p99);
                println!("  PEAK Max Time:   {:.2} ms ({:.1} FPS) at frame {:02}", max, 1000.0 / max, peak_frame_idx);
                println!(
                    "    Peak Breakdown: update_logic={:.2}ms (quadtree={:.2}ms, march={:.2}ms, streaming={:.2}ms) | draw={:.2}ms",
                    peak_breakdown.0, peak_breakdown.1, peak_breakdown.2, peak_breakdown.3, peak_breakdown.4
                );
            });
        });
        handle.join().unwrap();
    }

    #[test]
    fn test_tracking_flight_orbit_terrain_3d_render() {
        run_orbit_benchmark("Satellite + Terrain 3D", true, SATELLITE_IMAGERY_URL);
    }

    #[test]
    fn test_tracking_flight_orbit_standard_2d_render() {
        run_orbit_benchmark(
            "Standard Carto 2D",
            false,
            cesium_engine::globe::tiles::config::STANDARD_IMAGERY_URL,
        );
    }

    #[test]
    fn test_tracking_mouse_orbit_circling_plane_str_fra() {
        let handle = std::thread::spawn(move || {
            pollster::block_on(async {
                let progress_arc = std::sync::Arc::new(std::sync::Mutex::new(0.0)); // Ground at STR
                let mut flight_app = Box::new(cesium_flight::tracker::FlightTrackerApp::new(
                    progress_arc.clone(),
                ));
                flight_app.add_flight_path(
                    "STR-FRA",
                    9.2219, 48.6899, // STR
                    8.5706, 50.0333, // FRA
                    1_800_000,
                    false,
                    Vec::new(),
                );
                flight_app.view_mode = CameraMode::Tracking;
                flight_app.last_view_mode = CameraMode::Free;
                flight_app.reset_viewport = true;

                let mut config = TileEngineConfig::default();
                config.base_imagery_url = SATELLITE_IMAGERY_URL.to_string();
                config.terrain.enabled = true;

                let mut state = WgpuState::new(
                    None,
                    Some(winit::dpi::PhysicalSize::new(1280, 720)),
                    config,
                    Some(flight_app),
                )
                .await;

                // Warm up
                for _ in 0..5 {
                    #[cfg(feature = "debug_panel")]
                    let _ = state.render(None, false, |_, _| {});
                    #[cfg(not(feature = "debug_panel"))]
                    let _ = state.render(None, false);
                }

                let mut prev_pos = state.camera.local_pos;

                // 1. Circle around plane horizontally 360 degrees
                for step in 0..36 {
                    state.camera.orbit_mouse(20.0, 0.0);

                    #[cfg(feature = "debug_panel")]
                    let res = state.render(None, false, |_, _| {});
                    #[cfg(not(feature = "debug_panel"))]
                    let res = state.render(None, false);
                    assert!(res.is_ok(), "Render failed during horizontal orbit: {:?}", res);

                    let cur_pos = state.camera.local_pos;
                    assert!(
                        (cur_pos - prev_pos).length() > 1e-6,
                        "Camera locked! Step {}: pos remained {:?}",
                        step, cur_pos
                    );
                    assert!(!cur_pos.x.is_nan() && !cur_pos.y.is_nan() && !cur_pos.z.is_nan());
                    prev_pos = cur_pos;
                }

                // 2. Pitch up (looking down at plane)
                for _ in 0..10 {
                    state.camera.orbit_mouse(0.0, 15.0);
                    #[cfg(feature = "debug_panel")]
                    let _ = state.render(None, false, |_, _| {});
                    #[cfg(not(feature = "debug_panel"))]
                    let _ = state.render(None, false);
                }
                assert!(state.camera.local_pos.y > 0.0, "Camera should be above plane when pitched up");

                // 3. Pitch down towards ground: should not lock or go below terrain clearance
                for _ in 0..30 {
                    state.camera.orbit_mouse(0.0, -20.0);
                    #[cfg(feature = "debug_panel")]
                    let _ = state.render(None, false, |_, _| {});
                    #[cfg(not(feature = "debug_panel"))]
                    let _ = state.render(None, false);
                }

                // Camera should still be able to orbit horizontally even when down near ground!
                let ground_pos = state.camera.local_pos;
                state.camera.orbit_mouse(25.0, 0.0);
                assert!(
                    (state.camera.local_pos - ground_pos).length() > 1e-6,
                    "Camera must not lock when near ground!"
                );

                // 4. Capture screenshot of the aircraft orbiting at STR on runway
                let capture_file = "tracking_orbit_str_fra.png";
                #[cfg(feature = "debug_panel")]
                let cap_res = state.render(Some(capture_file), false, |_, _| {});
                #[cfg(not(feature = "debug_panel"))]
                let cap_res = state.render(Some(capture_file), false);
                assert!(cap_res.is_ok(), "Capture render failed: {:?}", cap_res);
                assert!(std::path::Path::new(capture_file).exists(), "Capture file not found!");
                let _ = std::fs::remove_file(capture_file);
            });
        });
        handle.join().unwrap();
    }

    #[test]
    fn test_tracking_runway_ground_collision_and_style_switch() {
        let handle = std::thread::spawn(move || {
            pollster::block_on(async {
                let mut flight_app = Box::new(cesium_flight::tracker::FlightTrackerApp::new(
                    std::sync::Arc::new(std::sync::Mutex::new(0.0)), // 0.0% progress (sitting directly on STR runway)
                ));
                flight_app.add_flight_path(
                    "STR-FRA",
                    9.2219, 48.6899, // STR
                    8.5706, 50.0333, // FRA
                    1_800_000,
                    false,
                    Vec::new(),
                );
                flight_app.view_mode = CameraMode::Tracking;
                flight_app.last_view_mode = CameraMode::Free;
                flight_app.reset_viewport = true;

                let mut config = TileEngineConfig::default();
                config.base_imagery_url = cesium_engine::globe::tiles::config::STANDARD_IMAGERY_URL.to_string();
                config.terrain.enabled = false;

                let mut state = WgpuState::new(
                    None,
                    Some(winit::dpi::PhysicalSize::new(1280, 720)),
                    config,
                    Some(flight_app),
                )
                .await;

                // Initial render frames
                for _ in 0..5 {
                    #[cfg(feature = "debug_panel")]
                    let res = state.render(None, false, |_, _| {});
                    #[cfg(not(feature = "debug_panel"))]
                    let res = state.render(None, false);
                    assert!(res.is_ok());
                }

                // 1. Switch to Satellite + Terrain while tracking plane on runway
                state.set_base_imagery_url(cesium_engine::globe::tiles::config::SATELLITE_IMAGERY_URL.to_string());
                state.set_terrain_enabled(true);

                // Render multiple frames after switch — must not crash or freeze
                for _ in 0..10 {
                    #[cfg(feature = "debug_panel")]
                    let res = state.render(None, false, |_, _| {});
                    #[cfg(not(feature = "debug_panel"))]
                    let res = state.render(None, false);
                    assert!(res.is_ok(), "Post-switch render failed: {:?}", res);
                }

                // 2. Full 360-degree orbit around the aircraft on the runway with terrain ON
                let mut prev_pos = state.camera.local_pos;
                for step in 0..24 {
                    state.camera.orbit_mouse(30.0, 0.0);
                    #[cfg(feature = "debug_panel")]
                    let res = state.render(None, false, |_, _| {});
                    #[cfg(not(feature = "debug_panel"))]
                    let res = state.render(None, false);
                    assert!(res.is_ok(), "Orbit render failed at step {}: {:?}", step, res);

                    let cur_pos = state.camera.local_pos;
                    assert!(
                        (cur_pos - prev_pos).length() > 1e-6,
                        "Camera locked on runway at step {}: pos {:?}",
                        step, cur_pos
                    );
                    prev_pos = cur_pos;
                }

                // 3. Pitch downwards directly into the runway ground
                for _ in 0..20 {
                    state.camera.orbit_mouse(0.0, -30.0);
                    #[cfg(feature = "debug_panel")]
                    let res = state.render(None, false, |_, _| {});
                    #[cfg(not(feature = "debug_panel"))]
                    let res = state.render(None, false);
                    assert!(res.is_ok(), "Ground dip render failed: {:?}", res);
                }

                // Camera must remain strictly above ground
                let (cam_pos_dvec, _) = state.camera.global_transform_f64();
                let cam_dist = cam_pos_dvec.length();
                let cam_dir = cam_pos_dvec.normalize();
                let inv_a2 = 1.0 / (6.378137 * 6.378137);
                let inv_b2 = 1.0 / (6.3567523142 * 6.3567523142);
                let surface_radius = 1.0 / (cam_dir.x * cam_dir.x * inv_a2 + cam_dir.y * cam_dir.y * inv_b2 + cam_dir.z * cam_dir.z * inv_a2).sqrt();
                assert!(cam_dist >= surface_radius, "Camera must not penetrate below ellipsoid/ground! cam_dist={} vs surf={}", cam_dist, surface_radius);

                // 4. Capture screenshot of the aircraft on the runway with 3D terrain
                let capture_file = "runway_terrain_orbit.png";
                #[cfg(feature = "debug_panel")]
                let cap_res = state.render(Some(capture_file), false, |_, _| {});
                #[cfg(not(feature = "debug_panel"))]
                let cap_res = state.render(Some(capture_file), false);
                assert!(cap_res.is_ok(), "Runway capture render failed: {:?}", cap_res);
                assert!(std::path::Path::new(capture_file).exists(), "Runway capture file not found!");
            });
        });
        handle.join().unwrap();
    }
}
