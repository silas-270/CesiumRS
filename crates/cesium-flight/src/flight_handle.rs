use std::sync::mpsc;

/// Commands that can be sent to a `FlightTrackerApp` from another thread.
pub enum FlightCommand {
    /// Load a new flight path from a JSON string.
    LoadFlight { id: String, json: String, is_secondary: bool },
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

    /// Load a flight from a JSON string. Non-blocking.
    pub fn load_flight(&self, id: impl Into<String>, json: impl Into<String>) {
        let _ = self.tx.try_send(FlightCommand::LoadFlight {
            id: id.into(),
            json: json.into(),
            is_secondary: false,
        });
    }

    /// Load a secondary (reference) flight path. Non-blocking.
    pub fn load_secondary_flight(&self, id: impl Into<String>, json: impl Into<String>) {
        let _ = self.tx.try_send(FlightCommand::LoadFlight {
            id: id.into(),
            json: json.into(),
            is_secondary: true,
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
