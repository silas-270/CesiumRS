//! Flight plan generation: two airports and a duration in, telemetry out.
//!
//! See `docs/flight-plan.md` in the repository root for the whole feature — what each
//! stage models, where the numbers come from, and which of them are real aviation
//! constraints rather than tuning.

pub mod aircraft;
pub mod airspace;
pub mod atmosphere;
pub mod generator;
pub mod geo;
pub mod lateral;
pub mod path;
pub mod runway;
pub mod schedule;
pub mod vertical;
pub mod wind;

pub use generator::{generate, FlightPlanConfig, FlightRequest, TelemetryPoint, WindModel};
pub use geo::LatLon;
