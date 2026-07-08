use cesium_engine::core::app::App;
use crate::testing::VerifyConfig;
use winit::application::ApplicationHandler;
use winit::event::WindowEvent;
use winit::event_loop::ActiveEventLoop;
use winit::window::WindowId;
use sha2::{Sha256, Digest};

#[derive(Clone, Debug)]
pub struct TestState {
    pub lat: f64,
    pub lon: f64,
    pub alt: f64,
    pub _pitch: f64,
    pub _yaw: f64,
}

pub struct RegressionApp<'a> {
    pub inner: App<'a>,
    pub config: VerifyConfig,
    pub states: Vec<TestState>,
    pub current_state_index: usize,
    pub frames_stable: u32,
    pub setup_done: bool,
    pub hasher: Sha256,
}

impl<'a> RegressionApp<'a> {
    pub fn new(config: VerifyConfig) -> Self {
        let mut states = Vec::new();
        // Generate deterministic grid (smaller for speed, but ~200-400 states is plenty for 100% determinism coverage)
        let latitudes = [-80.0, -40.0, 0.0, 40.0, 80.0];
        let longitudes = [-180.0, -90.0, 0.0, 90.0, 180.0];
        let altitudes = [0.1, 1.0, 10.0];
        let pitch_yaws = [(0.0, 0.0), (-0.5, 0.5), (-1.5, 3.14)];

        for &lat in &latitudes {
            for &lon in &longitudes {
                for &alt in &altitudes {
                    for &(pitch, yaw) in &pitch_yaws {
                        states.push(TestState { lat, lon, alt, _pitch: pitch, _yaw: yaw });
                    }
                }
            }
        }

        let mut app_config = cesium_engine::globe::tiles::config::TileEngineConfig::default();
        app_config.offline_mode = true;
        app_config.mesh_cache_size = std::num::NonZeroUsize::new(10000).unwrap();
        app_config.max_cache_size = std::num::NonZeroUsize::new(10000).unwrap();
        app_config.enable_prefetch = false;
        
        Self {
            inner: App::new(app_config, None),
            config,
            states,
            current_state_index: 0,
            frames_stable: 0,
            setup_done: false,
            hasher: Sha256::new(),
        }
    }

    fn apply_state(&mut self) {
        if let Some(state) = self.inner.wgpu_state_mut() {
            let ts = &self.states[self.current_state_index];
            
            // Convert Lat/Lon/Alt to ECEF (very rough conversion for testing camera placement)
            let rad_lat = ts.lat.to_radians();
            let rad_lon = ts.lon.to_radians();
            let r = 6.3781 + ts.alt; // Earth radius ~6.378 + altitude
            let x = r * rad_lat.cos() * rad_lon.cos();
            let y = r * rad_lat.cos() * rad_lon.sin();
            let z = r * rad_lat.sin();
            
            state.camera.set_eye(glam::Vec3::new(x as f32, y as f32, z as f32), glam::Vec3::ZERO);
            // We ignore pitch/yaw here for simplicity, or we could set local_ori
        }
    }
}

impl<'a> ApplicationHandler for RegressionApp<'a> {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        self.inner.resumed(event_loop);
        if !self.setup_done {
            self.apply_state();
            self.setup_done = true;
        }
    }

    fn window_event(&mut self, event_loop: &ActiveEventLoop, window_id: WindowId, event: WindowEvent) {
        if let WindowEvent::RedrawRequested = event {
            // Run all states instantly for pure logic determinism
            while self.current_state_index < self.states.len() {
                self.apply_state();
                
                if let Some(state) = self.inner.wgpu_state_mut() {
                    let aspect_ratio = 800.0 / 600.0;
                    let _main_view_proj = state.camera.get_projection_matrix(aspect_ratio) * state.camera.get_view_matrix();
                    let frustum_planes = state.camera.calculate_frustum_planes(aspect_ratio);
                    
                    let (global_pos_dvec, _) = state.camera.global_transform_f64();
                    let global_pos_f32 = glam::Vec3::new(global_pos_dvec.x as f32, global_pos_dvec.y as f32, global_pos_dvec.z as f32);
                    state.quadtree_manager.update(global_pos_f32, frustum_planes);
                    
                    let visible_tiles = state.quadtree_manager.get_visible_tiles();
                    
                    let mut sorted_tiles = visible_tiles.clone();
                    sorted_tiles.sort_by(|a, b| a.0.z.cmp(&b.0.z).then(a.0.x.cmp(&b.0.x)).then(a.0.y.cmp(&b.0.y)));
                    
                    let state_str = format!("Camera: {:?}, Tiles: {:?}", state.camera.global_transform(), sorted_tiles);
                    self.hasher.update(state_str.as_bytes());
                }
                
                self.current_state_index += 1;
                println!("Finished state {}/{}", self.current_state_index, self.states.len());
            }
            
            let result = self.hasher.clone().finalize();
            let hash_str: String = result.iter().map(|b| format!("{:02x}", b)).collect();
            println!("REGRESSION_HASH: {}", hash_str);
            event_loop.exit();
        } else {
            self.inner.window_event(event_loop, window_id, event);
        }
    }

    fn about_to_wait(&mut self, event_loop: &ActiveEventLoop) {
        self.inner.about_to_wait(event_loop);
    }
}
