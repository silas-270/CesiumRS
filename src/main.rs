use cesium_rs::run;
use cesium_rs::testing::VerifyConfig;
use clap::Parser;

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Cli {
    #[arg(long)]
    pub verify: bool,

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
    let config = if cli.verify {
        Some(VerifyConfig {
            enabled: cli.verify,
            cam_x: cli.cam_x,
            cam_y: cli.cam_y,
            cam_z: cli.cam_z,
            out_path: cli.out,
            actions: cli.actions,
        })
    } else {
        None
    };

    run(config);
}
