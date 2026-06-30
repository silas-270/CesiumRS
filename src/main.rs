use cesium_rs::run;

fn main() {
    cfg_if::cfg_if! {
        if #[cfg(not(target_os = "android"))] {
            env_logger::init();
        }
    }
    
    run();
}
