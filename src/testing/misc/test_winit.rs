#[test]
fn test_winit() {
    let device_id = winit::event::DeviceId::dummy();
    let event = winit::event::WindowEvent::MouseInput {
        device_id,
        state: winit::event::ElementState::Pressed,
        button: winit::event::MouseButton::Left,
    };
    println!("{:?}", event);
}
