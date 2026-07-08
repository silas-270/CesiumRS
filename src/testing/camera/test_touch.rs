use cesium_engine::camera::camera::Camera;
use cesium_engine::core::touch::TouchInterpreter;
use winit::event::{Touch, TouchPhase};

// Helper function to build a fake touch event
fn make_touch(id: u64, phase: TouchPhase, x: f64, y: f64) -> Touch {
    unsafe {
        Touch {
            device_id: std::mem::zeroed(),
            phase,
            location: winit::dpi::PhysicalPosition::new(x, y),
            force: None,
            id,
        }
    }
}

#[test]
fn test_touch_interpreter_single_finger_pan() {
    let mut camera = Camera::new(glam::Vec3::new(0.0, 0.0, 20.0), glam::Vec3::ZERO);
    let mut interpreter = TouchInterpreter::new();
    let screen_w = 800.0;
    let screen_h = 600.0;

    // 1. First finger touch down
    let touch_down = make_touch(1, TouchPhase::Started, 400.0, 300.0);
    let redrew = interpreter.handle_touch_event(&touch_down, &mut camera, screen_w, screen_h);
    assert!(!redrew, "Touch down shouldn't request redraw on its own");

    // 2. First finger drag
    let touch_move = make_touch(1, TouchPhase::Moved, 450.0, 300.0);
    let redrew2 = interpreter.handle_touch_event(&touch_move, &mut camera, screen_w, screen_h);
    assert!(redrew2, "Drag move should trigger camera movement and request redraw");

    // 3. First finger lift
    let touch_up = make_touch(1, TouchPhase::Ended, 450.0, 300.0);
    let _ = interpreter.handle_touch_event(&touch_up, &mut camera, screen_w, screen_h);
}

#[test]
fn test_touch_interpreter_pinch_to_zoom() {
    let mut camera = Camera::new(glam::Vec3::new(0.0, 0.0, 20.0), glam::Vec3::ZERO);
    let start_distance = camera.local_pos.length();

    let mut interpreter = TouchInterpreter::new();
    let screen_w = 800.0;
    let screen_h = 600.0;

    // Place two fingers down
    let f1_down = make_touch(1, TouchPhase::Started, 350.0, 300.0);
    interpreter.handle_touch_event(&f1_down, &mut camera, screen_w, screen_h);

    let f2_down = make_touch(2, TouchPhase::Started, 450.0, 300.0);
    interpreter.handle_touch_event(&f2_down, &mut camera, screen_w, screen_h);

    // Zoom in (spread fingers apart: from 100px dist to 150px dist)
    let f1_move = make_touch(1, TouchPhase::Moved, 325.0, 300.0);
    interpreter.handle_touch_event(&f1_move, &mut camera, screen_w, screen_h);

    let f2_move = make_touch(2, TouchPhase::Moved, 475.0, 300.0);
    let redrew = interpreter.handle_touch_event(&f2_move, &mut camera, screen_w, screen_h);

    assert!(redrew, "Pinch zoom should trigger redraw");
    let end_distance = camera.local_pos.length();
    assert!(end_distance < start_distance, "Pinch open should zoom in (decrease distance to target)");
}

#[test]
fn test_touch_interpreter_two_finger_tilt() {
    let mut camera = Camera::new(glam::Vec3::new(0.0, 0.0, 20.0), glam::Vec3::ZERO);
    let mut interpreter = TouchInterpreter::new();
    let screen_w = 800.0;
    let screen_h = 600.0;

    // Place two fingers down
    let f1_down = make_touch(1, TouchPhase::Started, 350.0, 300.0);
    interpreter.handle_touch_event(&f1_down, &mut camera, screen_w, screen_h);

    let f2_down = make_touch(2, TouchPhase::Started, 450.0, 300.0);
    interpreter.handle_touch_event(&f2_down, &mut camera, screen_w, screen_h);

    // Swipe both fingers downwards (tilt)
    let f1_move = make_touch(1, TouchPhase::Moved, 350.0, 350.0);
    interpreter.handle_touch_event(&f1_move, &mut camera, screen_w, screen_h);

    let f2_move = make_touch(2, TouchPhase::Moved, 450.0, 350.0);
    let redrew = interpreter.handle_touch_event(&f2_move, &mut camera, screen_w, screen_h);

    assert!(redrew, "Two-finger vertical swipe should trigger redraw");
}

#[test]
fn test_camera_inertia_decay() {
    let mut camera = Camera::new(glam::Vec3::new(0.0, 0.0, 20.0), glam::Vec3::ZERO);
    let mut interpreter = TouchInterpreter::new();
    let screen_w = 800.0;
    let screen_h = 600.0;

    // Simulate drag start
    let touch_down = make_touch(1, TouchPhase::Started, 400.0, 300.0);
    interpreter.handle_touch_event(&touch_down, &mut camera, screen_w, screen_h);

    // Simulate fast drag updates
    let touch_move1 = make_touch(1, TouchPhase::Moved, 450.0, 300.0);
    interpreter.handle_touch_event(&touch_move1, &mut camera, screen_w, screen_h);

    std::thread::sleep(std::time::Duration::from_millis(16));

    let touch_move2 = make_touch(1, TouchPhase::Moved, 500.0, 300.0);
    interpreter.handle_touch_event(&touch_move2, &mut camera, screen_w, screen_h);

    // End drag (lift finger) - should activate inertia
    let touch_up = make_touch(1, TouchPhase::Ended, 500.0, 300.0);
    interpreter.handle_touch_event(&touch_up, &mut camera, screen_w, screen_h);

    assert!(camera.inertia_active, "Inertia should be active after fast release");
    assert!(camera.inertia_velocity > 0.0, "Inertia velocity should be positive");

    let prev_pos = camera.local_pos;

    // Update inertia (simulate 1 frame of 16ms decay)
    let active = camera.update_inertia(0.016);
    assert!(active, "Inertia should still be active");
    
    let new_pos = camera.local_pos;
    assert_ne!(prev_pos, new_pos, "Camera position should have moved due to inertia");
}

