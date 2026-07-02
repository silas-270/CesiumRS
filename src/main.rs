use cesium_rs::run;

fn main() {
    cfg_if::cfg_if! {
        if #[cfg(not(target_os = "android"))] {
            env_logger::Builder::from_default_env()
                .filter_level(log::LevelFilter::Warn)
                .filter_module("cesium_rs", log::LevelFilter::Info)
                .init();
        }
    }

    run();
}
