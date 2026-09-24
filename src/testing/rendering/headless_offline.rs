//! Headless renders (Hub, Onboarding, Account and destination globes) always use the
//! bundled vector map, so they must produce a globe without any network access.
//!
//! The config carries no HTTP imagery source and terrain is off, so the render can only
//! succeed from the embedded SVG. Checks that the PNG exists and actually contains the
//! globe (opaque pixels) around the transparent sky.

use crate::headless::api::render_routes_headless;
use std::ffi::CString;
use std::fs;
use std::path::Path;

#[test]
fn headless_render_draws_globe_from_offline_vector_map() {
    let out_path = "test_headless_offline_vector.png";
    let c_path = CString::new(out_path).unwrap();
    let routes = vec![];

    let ok = render_routes_headless(400, 300, routes.as_ptr(), routes.len(), c_path.as_ptr());
    assert!(ok, "headless render reported failure");
    assert!(Path::new(out_path).exists(), "headless render produced no image");

    let rgba = image::open(out_path).expect("open render").into_rgba8();
    let opaque = rgba.pixels().filter(|p| p[3] == 255).count();
    let _ = fs::remove_file(out_path);

    let total = (rgba.width() * rgba.height()) as usize;
    assert!(
        opaque * 10 > total,
        "globe missing from offline headless render: {opaque}/{total} opaque pixels"
    );
}
