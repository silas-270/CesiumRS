use crate::engine::flight::load_flight_path;
use crate::engine::time::SimulationTime;
use crate::engine::property::Property;

#[test]
fn test_flight_parsing() {
    let prop = load_flight_path("flight_FRA_STR.json").expect("Failed to load flight JSON");

    // The first point is at 0ms.
    let start_pos = prop.evaluate(SimulationTime::new(0.0)).expect("No position at start");
    
    // The last point in the snippet the user provided is 91993ms, but the file may be longer.
    // Let's just evaluate at 10.0 seconds.
    let mid_pos = prop.evaluate(SimulationTime::new(10.0)).expect("No position at 10s");

    let distance = (start_pos - mid_pos).length();
    assert!(distance > 0.0);
}
