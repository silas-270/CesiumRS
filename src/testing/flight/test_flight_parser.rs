use cesium_engine::property::Property;
use cesium_engine::time::SimulationTime;
use cesium_flight::telemetry::generate;

#[test]
fn test_flight_generation() {
    let pts = generate(8.5706, 50.0333, 9.2219, 48.6899, 1_800_000, None, None);
    assert!(!pts.is_empty());
    
    let start_pos = pts.first().unwrap();
    let end_pos = pts.last().unwrap();
    
    assert!((start_pos.latitude - 50.0333).abs() < 1e-4);
    assert!((start_pos.longitude - 8.5706).abs() < 1e-4);
    
    assert!((end_pos.latitude - 48.6899).abs() < 1e-4);
    assert!((end_pos.longitude - 9.2219).abs() < 1e-4);
}
