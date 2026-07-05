#![cfg(not(target_os = "android"))]

use cesium_rs::run;
use cesium_rs::testing::VerifyConfig;
use clap::Parser;

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Cli {
    #[arg(long)]
    pub verify: bool,

    #[arg(long)]
    pub stress: bool,

    #[arg(long)]
    pub regression: bool,

    #[arg(long, default_value_t = String::from("flight"))]
    pub stress_mode: String,

    #[arg(long)]
    pub prefetch: bool,

    #[arg(long, default_value_t = 512)]
    pub cache_size: usize,

    #[arg(long, default_value_t = 0.0)]
    pub cam_x: f64,

    #[arg(long, default_value_t = 0.0)]
    pub cam_y: f64,

    #[arg(long, default_value_t = 20.0)]
    pub cam_z: f64,

    #[arg(long, default_value_t = String::from("verification.png"))]
    pub out: String,

    #[arg(long)]
    pub actions: Option<String>,
}

fn main() {
    cfg_if::cfg_if! {
        if #[cfg(not(target_os = "android"))] {
            env_logger::Builder::from_default_env()
                .filter_level(log::LevelFilter::Warn)
                .filter_module("cesium_rs", log::LevelFilter::Info)
                .init();
        }
    }

    let cli = Cli::parse();
    let config = if cli.verify || cli.stress || cli.regression {
        Some(VerifyConfig {
            enabled: cli.verify,
            stress: cli.stress,
            regression: cli.regression,
            stress_mode: cli.stress_mode,
            prefetch: cli.prefetch,
            cache_size: cli.cache_size,
            cam_x: cli.cam_x,
            cam_y: cli.cam_y,
            cam_z: cli.cam_z,
            out_path: cli.out,
            actions: cli.actions,
        })
    } else {
        None
    };

    if let Some(cfg) = config {
        run(Some(cfg));
    } else {
        use cesium_rs::{Viewer, ViewerOptions, GlobeOptions};

        let viewer = Viewer::new(ViewerOptions {
            globe: GlobeOptions {
                tile_cache_size: 2048,
                enable_prefetch: true,
                maximum_screen_space_error: 2.0,
            },
            ..Default::default()
        });
        
        viewer.run();
    }
}
