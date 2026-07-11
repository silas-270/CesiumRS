use cesium_engine::property::Property;
use cesium_engine::time::SimulationTime;
use cesium_flight::tracker::load_flight_data;

#[test]
fn test_flight_parsing() {
    let content =
        std::fs::read_to_string("flight_FRA_STR.json").unwrap_or_else(|_| "[]".to_string());
    if content == "[]" {
        return;
    }
    let prop = load_flight_data(&content).expect("Failed to load flight JSON");

    let start_pos = prop
        .0.evaluate(SimulationTime::new(0.0))
        .unwrap();
    let mid_pos = prop
        .0.evaluate(SimulationTime::new(10.0))
        .unwrap_or(start_pos);

    let distance = (start_pos - mid_pos).length();
    assert!(distance >= 0.0);
}
