use crate::headless::api::render_routes_headless;
use std::ffi::CString;
use std::path::Path;
use std::fs;

#[test]
fn test_transparent_headless_render() {
    let out_path = "test_headless_transparent.png";
    let c_path = CString::new(out_path).unwrap();

    // No routes needed, just check the empty background rendering behavior
    let routes = vec![];

    render_routes_headless(
        400, // width
        300, // height
        routes.as_ptr(),
        routes.len(),
        c_path.as_ptr(),
    );

    // Verify the file was created
    assert!(Path::new(out_path).exists(), "Headless render failed to produce an image file!");

    // Verify the image contains transparent pixels (alpha < 255)
    let img = image::open(out_path).expect("Failed to open generated image");
    let rgba = img.into_rgba8();

    let mut has_transparent = false;
    for pixel in rgba.pixels() {
        if pixel[3] < 255 {
            has_transparent = true;
            break;
        }
    }

    assert!(has_transparent, "Image does not have a transparent background! The sky or clear color might be incorrectly rendering opaquely.");

    // Cleanup
    let _ = fs::remove_file(out_path);
}
