pub mod sampled;

// Re-export the core trait and constant property so callers can use
// `engine::property::Property` without knowing where it lives.
pub use sampled::{ConstantProperty, Property};
