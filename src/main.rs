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

    #[arg(long)]
    pub flicker: bool,

    #[arg(long)]
    pub monitor: bool,

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

    #[arg(long)]
    pub profile: bool,

    #[arg(long)]
    pub benchmark: bool,

    #[arg(long)]
    pub routes_test: bool,
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
    
    if cli.routes_test {
        let routes = vec![
            cesium_rs::headless::api::HeadlessRoute {
                start: cesium_rs::headless::api::LatLon { lat: 25.2532, lon: 55.3657 },   // DXB
                end: cesium_rs::headless::api::LatLon { lat: 40.6413, lon: -73.7781 },    // JFK
            },
            cesium_rs::headless::api::HeadlessRoute {
                start: cesium_rs::headless::api::LatLon { lat: 25.2532, lon: 55.3657 },   // DXB
                end: cesium_rs::headless::api::LatLon { lat: 51.4700, lon: -0.4543 },     // LHR
            },
            cesium_rs::headless::api::HeadlessRoute {
                start: cesium_rs::headless::api::LatLon { lat: 25.2532, lon: 55.3657 },   // DXB
                end: cesium_rs::headless::api::LatLon { lat: -33.9399, lon: 151.1753 },   // SYD
            },
            cesium_rs::headless::api::HeadlessRoute {
                start: cesium_rs::headless::api::LatLon { lat: 25.2532, lon: 55.3657 },   // DXB
                end: cesium_rs::headless::api::LatLon { lat: 35.7720, lon: 140.3929 },    // NRT
            }
        ];
        
        let path = std::ffi::CString::new("routes_test.png").unwrap();
        
        cesium_rs::headless::api::render_routes_headless(
            800,
            600,
            routes.as_ptr(),
            routes.len(),
            path.as_ptr(),
        );
        return;
    }
    let config = if cli.verify || cli.stress || cli.regression || cli.flicker || cli.monitor || cli.profile || cli.benchmark {
        Some(VerifyConfig {
            enabled: cli.verify,
            stress: cli.stress,
            regression: cli.regression,
            flicker: cli.flicker,
            monitor: cli.monitor,
            profile: cli.profile,
            benchmark: cli.benchmark,
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
        let (flight_app, flight_handle) = cesium_flight::tracker::FlightTrackerApp::with_handle();

        // Load a flight path before starting
        flight_handle.load_flight(
            "flight_FRA_STR", 
            8.5706, 50.0333, // FRA
            9.2219, 48.6899, // STR
            1_800_000,       // 30 mins
            Some(249.0),     // FRA Runway 25C heading
            Some(73.0)       // STR Runway 07 heading
        );

        let viewer = cesium_rs::CesiumViewer::builder()
            .tile_cache_size(2048)
            .enable_prefetch(true)
            .max_screen_space_error(2.0)
            .with_extension(Box::new(flight_app))
            .build();

        // Obtain a handle before run() takes ownership
        let _cam = viewer.handle();

        viewer.run(); // Blocks — takes over the main thread
    }
}
