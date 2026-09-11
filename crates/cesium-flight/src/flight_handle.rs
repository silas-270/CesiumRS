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

/// How much of the route line is drawn.
///
/// Showing the whole route from the first second of a long-haul flight both gives the
/// route away and fills the screen with a line that is nowhere near the aircraft. The
/// windowed mode keeps only the part being flown, which is the part worth looking at.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum RouteLineMode {
    /// The whole route, departure to arrival.
    Full,
    /// Only the stretch around the aircraft, fading out at both ends.
    Window {
        /// Distance behind and ahead of the aircraft, in metres.
        behind_m: f64,
        ahead_m: f64,
    },
    /// No route line at all.
    Hidden,
}

impl Default for RouteLineMode {
    fn default() -> Self {
        Self::Full
    }
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
    /// Replace the planning options applied to every flight loaded afterwards.
    ///
    /// Sent separately from `LoadFlight` so that adding a planning option does not
    /// change the signature every caller of `load_flight` uses. Commands are handled in
    /// order, so setting this immediately before a load applies to that load.
    SetPlanConfig(crate::telemetry::FlightPlanConfig),
    /// Replace how much of the route line is drawn. Applies immediately, to every
    /// flight currently loaded.
    SetRouteLineMode(RouteLineMode),
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

    /// Load a flight path from a pre-defined or parsed route definition. Non-blocking.
    pub fn load_route_def(&self, route: &crate::preset::FlightRouteDef) {
        self.load_flight(
            &route.id,
            route.departure_lon,
            route.departure_lat,
            route.arrival_lon,
            route.arrival_lat,
            route.total_duration_ms,
            route.dep_heading_deg,
            route.arr_heading_deg,
            Vec::new(),
        );
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

    /// Set the planning options used by subsequent `load_flight` calls. Non-blocking.
    ///
    /// The main use is field elevation, which stays off until the globe renders
    /// terrain — see `FlightPlanConfig::terrain_elevation`.
    pub fn set_plan_config(&self, config: crate::telemetry::FlightPlanConfig) {
        let _ = self.tx.try_send(FlightCommand::SetPlanConfig(config));
    }

    /// Set how much of the route line is drawn. Non-blocking.
    ///
    /// Unlike `set_plan_config` this takes effect at once — it changes what is drawn,
    /// not how the next flight is planned.
    pub fn set_route_line_mode(&self, mode: RouteLineMode) {
        let _ = self.tx.try_send(FlightCommand::SetRouteLineMode(mode));
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
