use crate::engine::time::SimulationTime;

pub mod sampled;

/// A generic trait for properties that can change over time.
pub trait Property<T> {
    fn evaluate(&self, time: SimulationTime) -> Option<T>;
}

/// A property whose value never changes.
pub struct ConstantProperty<T> {
    value: T,
}

impl<T: Clone> ConstantProperty<T> {
    pub fn new(value: T) -> Self {
        Self { value }
    }
}

impl<T: Clone> Property<T> for ConstantProperty<T> {
    fn evaluate(&self, _time: SimulationTime) -> Option<T> {
        Some(self.value.clone())
    }
}
