//! GPU frame-time harness for the sky and globe shading: renders the real flight view
//! (tracking at FRA) offscreen at 1920x1080 and reads two GPU timestamps bracketing the
//! scene of every frame (`WgpuState::scene_timestamps`). Built to compare two commits
//! within 1%: run the same binary pair alternately, several times, and compare medians.
//!
//! Run with
//! `cargo test --lib sky_perf -- --ignored --nocapture`
//! Optional: `SKY_PERF_FRAMES` (default 400 per batch), `SKY_PERF_BATCHES` (default 3),
//! `SKY_PERF_SCENES` (comma list of dark_noon,dark_sunset,sat_noon,sat_sunset,
//! sat_sunset_moving).
//!
//! Prints one `RESULT` line per batch: scene, GPU median / p10 / p90 of the scene's GPU
//! time in microseconds, and the batch's wall time per frame (frames are queued back to
//! back, so that is the throughput: whichever of CPU and GPU is slower).

use cesium_engine::camera::camera::CameraMode;
use cesium_engine::globe::tiles::config::{TerrainConfig, TileEngineConfig, SATELLITE_IMAGERY_URL};
use cesium_engine::render::wgpu_state::WgpuState;

fn config(satellite: bool) -> TileEngineConfig {
    if satellite {
        TileEngineConfig {
            base_imagery_url: SATELLITE_IMAGERY_URL.to_string(),
            terrain: TerrainConfig { enabled: true, ..TerrainConfig::default() },
            ..TileEngineConfig::default()
        }
    } else {
        TileEngineConfig::default()
    }
}

fn percentile(sorted: &[f64], p: f64) -> f64 {
    let i = ((sorted.len() - 1) as f64 * p).round() as usize;
    sorted[i]
}

/// `moving`: sweep the flight's progress back and forth (a triangle wave, 100 frames each
/// way) at ten times the pace of the real 30-minute flight at 60 fps, so the sun and the
/// camera height move every frame and the sky LUT keeps re-rendering, as in a climb — the
/// static scenes render it only once.
async fn run_scene(name: &str, satellite: bool, progress: f64, moving: bool, frames: usize, batches: usize) {
    let shared_progress = std::sync::Arc::new(std::sync::Mutex::new(progress));
    let mut flight_app = Box::new(cesium_flight::tracker::FlightTrackerApp::new(shared_progress.clone()));
    flight_app.add_flight_path("flight_FRA_STR", 8.5706, 50.0333, 9.2219, 48.6899, 1_800_000, false, Vec::new());
    flight_app.view_mode = CameraMode::Tracking;
    flight_app.last_view_mode = CameraMode::Free;
    flight_app.reset_viewport = true;

    let (w, h) = (1920u32, 1080u32);
    let mut state = WgpuState::new(
        None,
        Some(winit::dpi::PhysicalSize::new(w, h)),
        config(satellite),
        Some(flight_app),
    )
    .await;
    let wanted = wgpu::Features::TIMESTAMP_QUERY | wgpu::Features::TIMESTAMP_QUERY_INSIDE_ENCODERS;
    assert!(
        state.device.features().contains(wanted),
        "adapter has no timestamp queries; set CESIUM_GPU_TIMING=1"
    );
    let query = state.device.create_query_set(&wgpu::QuerySetDescriptor {
        label: Some("sky_perf"),
        ty: wgpu::QueryType::Timestamp,
        count: 2,
    });
    // One 256-byte slot per frame (the resolve alignment), read back once per batch, so
    // frames are queued back to back and the GPU stays at its top clock: waiting on every
    // frame let it drop clocks in between and made the numbers swing by 30%.
    let slot = wgpu::QUERY_RESOLVE_BUFFER_ALIGNMENT;
    let resolve = state.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("sky_perf resolve"),
        size: slot * frames as u64,
        usage: wgpu::BufferUsages::QUERY_RESOLVE | wgpu::BufferUsages::COPY_SRC,
        mapped_at_creation: false,
    });
    let readback = state.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("sky_perf readback"),
        size: slot * frames as u64,
        usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    let period_ns = state.queue.get_timestamp_period() as f64;

    // Stream until the view is settled, as the 4K screenshot does.
    let aspect = w as f32 / h as f32;
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(90);
    let mut quiet = 0;
    let mut warm = 0;
    while std::time::Instant::now() < deadline {
        let vp = state.camera.get_projection_matrix(aspect) * state.camera.get_view_matrix();
        let visible = state.update_logic(aspect, vp);
        state
            .tile_system
            .texture_manager
            .fetch_and_upload_all(&state.device, &state.queue, &visible)
            .await;
        #[cfg(feature = "debug_panel")]
        let _ = state.render(None, false, |_, _| {});
        #[cfg(not(feature = "debug_panel"))]
        let _ = state.render(None, false);
        warm += 1;
        let settled = state.last_missing_tiles_count == 0 && state.tile_system.is_loading_complete();
        quiet = if settled { quiet + 1 } else { 0 };
        if quiet >= 30 && warm >= 60 {
            break;
        }
        std::thread::sleep(std::time::Duration::from_millis(20));
    }
    state.scene_timestamps = Some(query);

    // Spin the GPU up to its top clock before the first batch.
    let spin = std::time::Instant::now();
    while spin.elapsed() < std::time::Duration::from_secs(3) {
        #[cfg(feature = "debug_panel")]
        let _ = state.render(None, false, |_, _| {});
        #[cfg(not(feature = "debug_panel"))]
        let _ = state.render(None, false);
    }
    state.device.poll(wgpu::Maintain::Wait);

    for batch in 0..batches {
        let mut gpu = Vec::with_capacity(frames);
        let t_batch = std::time::Instant::now();
        for i in 0..frames {
            if moving {
                const STEP: f64 = 10.0 / (1800.0 * 60.0);
                let tri = (i % 200) as f64;
                let tri = if tri < 100.0 { tri } else { 200.0 - tri };
                *shared_progress.lock().unwrap() = progress + STEP * tri;
            }
            #[cfg(feature = "debug_panel")]
            let _ = state.render(None, false, |_, _| {});
            #[cfg(not(feature = "debug_panel"))]
            let _ = state.render(None, false);
            let mut enc = state.device.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: None });
            enc.resolve_query_set(state.scene_timestamps.as_ref().unwrap(), 0..2, &resolve, slot * i as u64);
            state.queue.submit(Some(enc.finish()));
        }
        let mut enc = state.device.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: None });
        enc.copy_buffer_to_buffer(&resolve, 0, &readback, 0, slot * frames as u64);
        state.queue.submit(Some(enc.finish()));
        readback.slice(..).map_async(wgpu::MapMode::Read, |_| {});
        state.device.poll(wgpu::Maintain::Wait);
        let frame_wall_us = t_batch.elapsed().as_secs_f64() * 1e6 / frames as f64;
        {
            let data = readback.slice(..).get_mapped_range();
            for i in 0..frames {
                let o = (slot * i as u64) as usize;
                let t0 = u64::from_le_bytes(data[o..o + 8].try_into().unwrap());
                let t1 = u64::from_le_bytes(data[o + 8..o + 16].try_into().unwrap());
                gpu.push(t1.wrapping_sub(t0) as f64 * period_ns / 1000.0);
            }
        }
        readback.unmap();
        gpu.sort_by(|a, b| a.partial_cmp(b).unwrap());
        println!(
            "RESULT {name} batch={batch} gpu_med_us={:.1} gpu_p10_us={:.1} gpu_p90_us={:.1} wall_per_frame_us={:.1} tiles_missing={}",
            percentile(&gpu, 0.5),
            percentile(&gpu, 0.1),
            percentile(&gpu, 0.9),
            frame_wall_us,
            state.last_missing_tiles_count,
        );
    }
}

#[test]
#[ignore = "GPU benchmark; run explicitly"]
fn sky_perf() {
    std::env::set_var("CESIUM_GPU_TIMING", "1");
    let frames: usize = std::env::var("SKY_PERF_FRAMES").ok().and_then(|v| v.parse().ok()).unwrap_or(400);
    let batches: usize = std::env::var("SKY_PERF_BATCHES").ok().and_then(|v| v.parse().ok()).unwrap_or(3);
    let wanted = std::env::var("SKY_PERF_SCENES").unwrap_or_default();
    for (name, satellite, progress, moving) in [
        ("dark_noon", false, 0.02, false),
        ("dark_sunset", false, 0.17, false),
        ("sat_noon", true, 0.02, false),
        ("sat_sunset", true, 0.17, false),
        ("sat_sunset_moving", true, 0.17, true),
    ] {
        if !wanted.is_empty() && !wanted.split(',').any(|s| s.trim() == name) {
            continue;
        }
        pollster::block_on(run_scene(name, satellite, progress, moving, frames, batches));
    }
}
