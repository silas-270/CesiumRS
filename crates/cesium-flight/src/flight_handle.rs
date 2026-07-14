use std::sync::mpsc;

#[derive(Clone, Debug)]
pub struct RunwayData {
    pub airport_id: i32,
    pub length_ft: f32,
    pub width_ft: f32,
    pub le_heading: f32,
    pub le_lat: f64,
    pub le_lon: f64,
    pub he_heading: f32,
    pub he_lat: f64,
    pub he_lon: f64,
}

/// Commands that can be sent to a `FlightTrackerApp` from another thread.
pub enum FlightCommand {
    /// Load a new flight path from runway coordinates.
    LoadFlight {
        id: String,
        departure_lon: f64,
        departure_lat: f64,
        arrival_lon: f64,
        arrival_lat: f64,
        total_duration_ms: u64,
        dep_heading_deg: Option<f64>,
        arr_heading_deg: Option<f64>,
        is_secondary: bool,
        runways: Vec<RunwayData>,
    },
    /// Set the playback progress (0.0 – 1.0) of the primary flight.
    SetProgress(f64),
    /// Set playback speed multiplier.
    SetSpeed(f64),
    /// Start playback.
    Play,
    /// Pause playback.
    Pause,
}

/// A cloneable, `Send + Sync` handle for sending commands to a `FlightTrackerApp`
/// that is running inside the engine loop.
#[derive(Clone)]
pub struct FlightHandle {
    tx: mpsc::SyncSender<FlightCommand>,
}

impl FlightHandle {
    pub(crate) fn new(tx: mpsc::SyncSender<FlightCommand>) -> Self {
        Self { tx }
    }

    /// Load a flight path by generating it from runway coordinates. Non-blocking.
    pub fn load_flight(
        &self, 
        id: impl Into<String>, 
        departure_lon: f64, 
        departure_lat: f64,
        arrival_lon: f64,
        arrival_lat: f64,
        total_duration_ms: u64,
        dep_heading_deg: Option<f64>,
        arr_heading_deg: Option<f64>,
        runways: Vec<RunwayData>,
    ) {
        let _ = self.tx.try_send(FlightCommand::LoadFlight {
            id: id.into(),
            departure_lon,
            departure_lat,
            arrival_lon,
            arrival_lat,
            total_duration_ms,
            dep_heading_deg,
            arr_heading_deg,
            is_secondary: false,
            runways,
        });
    }

    /// Load a secondary (reference) flight path. Non-blocking.
    pub fn load_secondary_flight(
        &self, 
        id: impl Into<String>, 
        departure_lon: f64, 
        departure_lat: f64,
        arrival_lon: f64,
        arrival_lat: f64,
        total_duration_ms: u64,
        dep_heading_deg: Option<f64>,
        arr_heading_deg: Option<f64>,
    ) {
        let _ = self.tx.try_send(FlightCommand::LoadFlight {
            id: id.into(),
            departure_lon,
            departure_lat,
            arrival_lon,
            arrival_lat,
            total_duration_ms,
            dep_heading_deg,
            arr_heading_deg,
            is_secondary: true,
            runways: Vec::new(),
        });
    }

    /// Set the flight playback progress (0.0 – 1.0). Non-blocking.
    pub fn set_progress(&self, progress: f64) {
        let _ = self.tx.try_send(FlightCommand::SetProgress(progress));
    }

    /// Set the playback speed multiplier. Non-blocking.
    pub fn set_speed(&self, speed: f64) {
        let _ = self.tx.try_send(FlightCommand::SetSpeed(speed));
    }

    /// Start playback. Non-blocking.
    pub fn play(&self) {
        let _ = self.tx.try_send(FlightCommand::Play);
    }

    /// Pause playback. Non-blocking.
    pub fn pause(&self) {
        let _ = self.tx.try_send(FlightCommand::Pause);
    }
}
