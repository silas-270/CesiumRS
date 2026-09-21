#[cfg(test)]
mod tests {
    use cesium_engine::camera::camera::CameraMode;
    use cesium_engine::globe::tiles::config::{TileEngineConfig, SATELLITE_IMAGERY_URL};
    use cesium_engine::render::wgpu_state::WgpuState;
    use std::time::Instant;

    #[test]
    fn test_tracking_flight_orbit_terrain_render() {
        let handle = std::thread::spawn(|| {
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
                config.base_imagery_url = SATELLITE_IMAGERY_URL.to_string();
                config.terrain.enabled = true;

                let mut state = WgpuState::new(
                    None,
                    Some(winit::dpi::PhysicalSize::new(1280, 720)),
                    config,
                    Some(flight_app),
                )
                .await;

                println!("--- Warming up state with actual renders ---");
                for frame in 0..10 {
                    #[cfg(feature = "debug_panel")]
                    let res = state.render(None, false, |_, _| {});
                    #[cfg(not(feature = "debug_panel"))]
                    let res = state.render(None, false);
                    assert!(res.is_ok(), "Render failed: {:?}", res);
                    std::thread::sleep(std::time::Duration::from_millis(100));
                }

                println!("--- Simulating camera orbit around plane with full renders ---");
                let mut total_time_ms = 0.0;
                for frame in 0..60 {
                    let start = Instant::now();
                    
                    // Orbit camera around plane
                    state.camera.orbit_anchor(glam::Quat::from_axis_angle(glam::Vec3::Y, 0.05));

                    #[cfg(feature = "debug_panel")]
                    let res = state.render(None, false, |_, _| {});
                    #[cfg(not(feature = "debug_panel"))]
                    let res = state.render(None, false);
                    assert!(res.is_ok(), "Render failed: {:?}", res);
                    
                    let dt = start.elapsed().as_secs_f64() * 1000.0;
                    total_time_ms += dt;

                    let (requested, missing) = state.get_fetch_stats();
                    println!(
                        "Frame {:02}: render_total={:.2}ms | update_logic={:.2}ms quadtree={:.2}ms march={:.2}ms streaming={:.2}ms draw={:.2}ms | req={} miss={} dstate={} | alt_agl={:.2}m",
                        frame,
                        dt,
                        state.last_timings.update_logic_us / 1000.0,
                        state.last_subsystem_timings.quadtree_us / 1000.0,
                        state.last_subsystem_timings.terrain_horizon_us / 1000.0,
                        state.last_subsystem_timings.tile_streaming_us / 1000.0,
                        state.last_subsystem_timings.terrain_draw_us / 1000.0,
                        requested,
                        missing,
                        state.display_state.len(),
                        state.camera.altitude_agl() * 1_000_000.0,
                    );
                    std::thread::sleep(std::time::Duration::from_millis(16));
                }
                println!("Average frame render time: {:.2}ms ({:.1} FPS)", total_time_ms / 60.0, 1000.0 / (total_time_ms / 60.0));
            });
        });
        handle.join().unwrap();
    }
}
