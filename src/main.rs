#![cfg(not(target_os = "android"))]

#[cfg(feature = "testing")]
mod inner {
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

        /// Render hub routes with custom camera parameters
        #[arg(long)]
        pub render_hub: bool,

        #[arg(long, default_value_t = 1080)]
        pub hub_width: u32,

        #[arg(long, default_value_t = 1670)]
        pub hub_height: u32,

        #[arg(long, default_value_t = 16.36)]
        pub hub_distance: f32,

        #[arg(long, default_value_t = -15.0, allow_hyphen_values = true)]
        pub hub_tilt: f32,

        #[arg(long, default_value_t = 0.0, allow_hyphen_values = true)]
        pub hub_pan: f32,

        #[arg(long, default_value = "horizon")]
        pub hub_camera: String,

        #[arg(long, default_value_t = 1.5)]
        pub hub_alt: f32,

        #[arg(long, default_value_t = 12.0, allow_hyphen_values = true)]
        pub hub_back: f32,

        #[arg(long, default_value_t = 25.0, allow_hyphen_values = true)]
        pub hub_pitch: f32,

        #[arg(long, default_value_t = 0.0, allow_hyphen_values = true)]
        pub hub_heading: f32,

        #[arg(long, default_value_t = 0.0, allow_hyphen_values = true)]
        pub hub_roll: f32,

        #[arg(long, default_value = "hub_render.png")]
        pub hub_out: String,

        #[arg(long)]
        pub hub_routes_file: Option<String>,

        /// Headless capture of the cockpit and tracking views for visual verification.
        #[arg(long)]
        pub cockpit: bool,

        /// Headless capture of the cockpit view at the Samsung S23's landscape and
        /// portrait resolutions, for iterating on the interior without an APK build.
        #[arg(long)]
        pub cockpit_s23: bool,

        /// Headless capture of routes in Free camera mode at laptop native resolution.
        #[arg(long)]
        pub free_routes: bool,

        /// Initial flight route to load on startup: preset name (e.g. 'FRA-STR', 'LHR-NRT', 'JFK-LHR')
        /// or coordinates 'lat1,lon1,lat2,lon2'.
        #[arg(long, default_value = "FRA-STR")]
        pub route: String,
    }

    pub fn main() {
        cfg_if::cfg_if! {
            if #[cfg(not(target_os = "android"))] {
                env_logger::Builder::from_default_env()
                    .filter_level(log::LevelFilter::Warn)
                    .filter_module("cesium_rs", log::LevelFilter::Info)
                    .init();
            }
        }

        let cli = Cli::parse();
        
        if cli.render_hub {
            let mut routes = Vec::new();
            if let Some(ref file_path) = cli.hub_routes_file {
                if let Ok(content) = std::fs::read_to_string(file_path) {
                    for line in content.lines() {
                        let parts: Vec<&str> = line.trim().split(',').collect();
                        if parts.len() == 4 {
                            if let (Ok(lat1), Ok(lon1), Ok(lat2), Ok(lon2)) = (
                                parts[0].trim().parse::<f64>(),
                                parts[1].trim().parse::<f64>(),
                                parts[2].trim().parse::<f64>(),
                                parts[3].trim().parse::<f64>(),
                            ) {
                                routes.push(cesium_rs::headless::api::HeadlessRoute {
                                    start: cesium_rs::headless::api::LatLon { lat: lat1, lon: lon1 },
                                    end: cesium_rs::headless::api::LatLon { lat: lat2, lon: lon2 },
                                });
                            }
                        }
                    }
                }
            }

            if routes.is_empty() {
                // Default to GRZ (Graz) routes matching actual app
                let grz = cesium_rs::headless::api::LatLon { lat: 46.9911, lon: 15.4396 };
                let dests = [
                    (50.0267, 8.5584),   // FRA
                    (48.3538, 11.7861),  // MUC
                    (48.1103, 16.5697),  // VIE
                    (47.4581, 8.5481),   // ZRH
                    (53.6304, 9.9882),   // HAM
                    (52.3617, 13.5023),  // BER
                    (51.1487, -0.1857),  // LGW
                    (51.2895, 6.7668),   // DUS
                    (36.8987, 30.8005),  // AYT
                    (27.1768, 33.7967),  // HRG
                    (36.7945, 27.0912),  // KGS
                    (39.5517, 2.7388),   // PMI
                ];
                for (lat, lon) in dests {
                    routes.push(cesium_rs::headless::api::HeadlessRoute {
                        start: grz,
                        end: cesium_rs::headless::api::LatLon { lat, lon },
                    });
                }
            }

            let path = std::ffi::CString::new(cli.hub_out.as_str()).unwrap();
            if cli.hub_camera == "horizon" {
                cesium_rs::headless::api::render_routes_headless_horizon(
                    cli.hub_width,
                    cli.hub_height,
                    routes.as_ptr(),
                    routes.len(),
                    path.as_ptr(),
                    cli.hub_alt,
                    cli.hub_back,
                    cli.hub_pitch,
                    cli.hub_heading,
                    cli.hub_roll,
                );
            } else {
                cesium_rs::headless::api::render_routes_headless_custom(
                    cli.hub_width,
                    cli.hub_height,
                    routes.as_ptr(),
                    routes.len(),
                    path.as_ptr(),
                    cli.hub_distance,
                    cli.hub_tilt,
                    cli.hub_pan,
                );
            }
            return;
        }

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
        let config = if cli.verify || cli.stress || cli.regression || cli.flicker || cli.monitor || cli.profile || cli.benchmark || cli.cockpit || cli.cockpit_s23 || cli.free_routes {
            Some(VerifyConfig {
                enabled: cli.verify,
                stress: cli.stress,
                regression: cli.regression,
                flicker: cli.flicker,
                monitor: cli.monitor,
                profile: cli.profile,
                benchmark: cli.benchmark,
                cockpit: cli.cockpit,
                cockpit_s23: cli.cockpit_s23,
                free_routes: cli.free_routes,
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

            let route_def = cesium_flight::preset::parse_route(&cli.route)
                .unwrap_or_else(|e| {
                    eprintln!("Warning: Failed to parse route '{}': {}. Falling back to FRA-STR.", cli.route, e);
                    cesium_flight::preset::parse_route("FRA-STR").unwrap()
                });

            flight_handle.load_route_def(&route_def);

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
}

#[cfg(feature = "testing")]
fn main() {
    inner::main();
}

#[cfg(not(feature = "testing"))]
fn main() {
    cfg_if::cfg_if! {
        if #[cfg(not(target_os = "android"))] {
            env_logger::Builder::from_default_env()
                .filter_level(log::LevelFilter::Warn)
                .filter_module("cesium_rs", log::LevelFilter::Info)
                .init();
        }
    }

    let (flight_app, flight_handle) = cesium_flight::tracker::FlightTrackerApp::with_handle();

    let default_route = cesium_flight::preset::parse_route("FRA-STR").unwrap();
    flight_handle.load_route_def(&default_route);

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
