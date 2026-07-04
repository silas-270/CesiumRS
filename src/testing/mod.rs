pub mod simulator;
pub mod test_app;
pub mod stress_app;

#[derive(Clone, Debug, Default)]
pub struct VerifyConfig {
    pub enabled: bool,
    pub stress: bool,
    pub stress_mode: String,
    pub prefetch: bool,
    pub cache_size: usize,
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
#[cfg(test)]
mod test_tile_cache;
#[cfg(test)]
mod test_tile_fetcher;
#[cfg(test)]
mod test_mesh_worker;
#[cfg(test)]
mod orchestrator_tests;
#[cfg(test)]
mod test_orchestrator_stress;

#[cfg(test)]
mod test_terrain_parser;
#[cfg(test)]
mod test_frustum_coverage;
#[cfg(test)]
mod test_parametric_sweeps;
#[cfg(test)]
mod test_z_sweep;