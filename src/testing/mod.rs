pub mod simulator;
pub mod test_app;

#[derive(Clone, Debug, Default)]
pub struct VerifyConfig {
    pub enabled: bool,
    pub cam_x: f64,
    pub cam_y: f64,
    pub cam_z: f64,
    pub out_path: String,
    pub actions: Option<String>,
}
