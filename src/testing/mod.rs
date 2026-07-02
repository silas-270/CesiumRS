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

#[cfg(test)]
mod test_glam;
#[cfg(test)]
mod test_winit;
#[cfg(test)]
mod test_high_alt;
#[cfg(test)]
mod test_20_tiles;
#[cfg(test)]
mod test_obb_debug;
#[cfg(test)]
mod test_point;
#[cfg(test)]
mod test_all_altitudes;
#[cfg(test)]
mod test_drag_zoom;
