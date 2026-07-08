pub mod camera;
pub mod culling;
pub mod flight;
pub mod harness;
pub mod misc;
pub mod rendering;
pub mod terrain;
pub mod tiles;

#[derive(Clone, Debug, Default)]
pub struct VerifyConfig {
    pub enabled: bool,
    pub stress: bool,
    pub regression: bool,
    pub flicker: bool,
    pub monitor: bool,
    pub stress_mode: String,
    pub prefetch: bool,
    pub cache_size: usize,
    pub cam_x: f64,
    pub cam_y: f64,
    pub cam_z: f64,
    pub out_path: String,
    pub actions: Option<String>,
}
