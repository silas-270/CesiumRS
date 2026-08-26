//! Runtime asset lookup for models too large to bake into the binary.
//!
//! Platforms that keep assets somewhere the filesystem cannot reach — Android, where they
//! live inside the APK — register a loader once at startup via [`set_loader`]. Everywhere
//! else the default disk lookup is enough and no registration is needed.

use std::path::PathBuf;
use std::sync::OnceLock;

type Loader = Box<dyn Fn(&str) -> Option<Vec<u8>> + Send + Sync>;

static LOADER: OnceLock<Loader> = OnceLock::new();

/// Installs the platform asset loader. Only the first call takes effect.
pub fn set_loader(f: impl Fn(&str) -> Option<Vec<u8>> + Send + Sync + 'static) {
    let _ = LOADER.set(Box::new(f));
}

/// Reads an asset by file name (no directory component).
///
/// Uses the registered loader when there is one, otherwise looks for `assets/<name>`
/// relative to the working directory and then to the executable.
pub fn load(name: &str) -> Option<Vec<u8>> {
    if let Some(loader) = LOADER.get() {
        return loader(name);
    }

    for dir in disk_search_paths() {
        let path = dir.join(name);
        match std::fs::read(&path) {
            Ok(bytes) => return Some(bytes),
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => continue,
            Err(e) => log::warn!("Failed to read asset {}: {}", path.display(), e),
        }
    }

    None
}

fn disk_search_paths() -> Vec<PathBuf> {
    let mut paths = vec![PathBuf::from("assets")];
    if let Ok(exe) = std::env::current_exe() {
        if let Some(dir) = exe.parent() {
            paths.push(dir.join("assets"));
        }
    }
    paths
}
