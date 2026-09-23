use glam::DVec3;
use std::sync::mpsc;

use cesium_engine::camera::camera::CameraMode;
use cesium_engine::core::extension::GlobeExtension;
use cesium_engine::globe::geometry::lon_lat_alt_to_ecef_f64;
use cesium_engine::property::sampled::{InterpolationAlgorithm, SampledPositionProperty};
use cesium_engine::render::model_pipeline::pipeline::ModelRenderer;
use cesium_engine::render::polyline_pipeline::builder::AdaptiveSubdivisionBuilder;
use cesium_engine::render::polyline_pipeline::pipeline::{DrawParams, PolylineConfig, PolylineRenderer};
use cesium_engine::time::SimulationTime;

use crate::flight_handle::{FlightCommand, FlightHandle};

use crate::telemetry::{generate, FlightPlanConfig, FlightRequest, LatLon};
use crate::terrain_fit::{AircraftFit, GroundEnds, LineFit};

pub struct PendingFlight {
    pub id: String,
    pub departure_lon: f64,
    pub departure_lat: f64,
    pub arrival_lon: f64,
    pub arrival_lat: f64,
    pub total_duration_ms: u64,
    pub dep_heading_deg: Option<f64>,
    pub arr_heading_deg: Option<f64>,
    pub is_secondary: bool,
    pub runways: Vec<crate::flight_handle::RunwayData>,
    /// Captured from the app's current config when the load command is handled.
    pub config: FlightPlanConfig,
}

impl PendingFlight {
    fn to_request(&self) -> FlightRequest {
        FlightRequest {
            departure: LatLon::new(self.departure_lat, self.departure_lon),
            arrival: LatLon::new(self.arrival_lat, self.arrival_lon),
            target_duration_ms: self.total_duration_ms,
            dep_heading_deg: self.dep_heading_deg,
            arr_heading_deg: self.arr_heading_deg,
            runways: self.runways.clone(),
            config: self.config,
        }
    }
}

pub struct FlightEntity {
    pub id: String,
    pub renderer: PolylineRenderer,
    pub config: PolylineConfig,
    pub property: SampledPositionProperty,
    pub sun_intensity_property: cesium_engine::property::sampled::SampledScalarProperty,
    pub telemetry_points: Vec<crate::telemetry::generator::TelemetryPoint>,
    pub total_duration_ms: u64,
    pub reference_point: glam::DVec3,
    /// Progress and the distance along the route it corresponds to, in Megametres, taken
    /// from the control points the ribbon is drawn from.
    ///
    /// Progress is a fraction of the *flight time*, and a flight covers ground very
    /// unevenly against the clock, so this is what turns "fifty miles ahead of the
    /// aircraft" into something the shader can compare against. Sorted by progress.
    pub route_distances: Vec<(f32, f32)>,
    /// Where the flight meets the ground at either end; see [`crate::terrain_fit`].
    pub ends: GroundEnds,
    /// The route line's control points, fitted onto the terrain near the airports by the
    /// same rule as the aircraft. `renderer` draws [`LineFit::points`].
    pub line: LineFit,
    /// Runway corridors along departure and arrival runways for terrain flattening.
    pub runway_corridors: Vec<cesium_engine::globe::terrain::RunwayCorridor>,
}


impl FlightEntity {
    /// How far along the route the aircraft is at `progress`, in Megametres.
    fn distance_at(&self, progress: f64) -> f32 {
        let table = &self.route_distances;
        if table.is_empty() {
            return 0.0;
        }
        let p = (progress as f32).clamp(0.0, 1.0);
        match table.binary_search_by(|(q, _)| q.partial_cmp(&p).unwrap_or(std::cmp::Ordering::Equal)) {
            Ok(i) => table[i].1,
            Err(0) => table[0].1,
            Err(i) if i >= table.len() => table[table.len() - 1].1,
            Err(i) => {
                let (p0, d0) = table[i - 1];
                let (p1, d1) = table[i];
                if p1 <= p0 {
                    d0
                } else {
                    d0 + (d1 - d0) * (p - p0) / (p1 - p0)
                }
            }
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct FlightTelemetry {
    pub progress: f64,
    pub latitude: f64,
    pub longitude: f64,
    pub altitude: f64,
    pub velocity_m_s: f64,
    pub heading_rad: f64,
    pub pitch_rad: f64,
    pub roll_rad: f64,
}

/// Rebuilds the layout the engine uses for its camera uniform.
///
/// Structurally identical layouts are interchangeable in wgpu, so renderers created
/// outside `init()` — where the engine hands us its own layout — can use this instead.
fn camera_bind_group_layout(device: &wgpu::Device) -> wgpu::BindGroupLayout {
    device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        entries: &[wgpu::BindGroupLayoutEntry {
            binding: 0,
            visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT,
            ty: wgpu::BindingType::Buffer {
                ty: wgpu::BufferBindingType::Uniform,
                has_dynamic_offset: false,
                min_binding_size: None,
            },
            count: None,
        }],
        label: Some("camera_bind_group_layout"),
    })
}

fn build_runway_corridors(
    pending: &PendingFlight,
    points: &[crate::telemetry::generator::TelemetryPoint],
) -> Vec<cesium_engine::globe::terrain::RunwayCorridor> {
    use cesium_engine::globe::terrain::RunwayCorridor;
    use crate::telemetry::geo::{distance_m, initial_bearing, destination, LatLon};

    if points.is_empty() {
        return Vec::new();
    }

    let mut corridors = Vec::new();
    let on_ground = |p: &&crate::telemetry::generator::TelemetryPoint, field: f64| {
        (p.altitude - field).abs() <= crate::terrain_fit::ON_GROUND_TOLERANCE_M
    };

    // 1. Departure Runway Corridor
    let first = &points[0];
    let dep_on = points.iter().take_while(|p| on_ground(p, first.altitude)).count();
    let dep_heading_rad = if dep_on > 1 {
        let last_dep = &points[dep_on - 1];
        initial_bearing(
            LatLon::new(first.latitude, first.longitude),
            LatLon::new(last_dep.latitude, last_dep.longitude),
        )
    } else {
        first.heading_rad
    };

    let p0 = LatLon::new(first.latitude, first.longitude);
    let mut dep_runway_data: Option<crate::flight_handle::RunwayData> = None;
    let mut best_dist = f64::MAX;
    for r in &pending.runways {
        let d1 = distance_m(p0, LatLon::new(r.le_lat, r.le_lon));
        let d2 = distance_m(p0, LatLon::new(r.he_lat, r.he_lon));
        let d = d1.min(d2);
        if d < 10_000.0 && d < best_dist {
            best_dist = d;
            dep_runway_data = Some(r.clone());
        }
    }

    let (dep_start, dep_end, dep_width_m) = if let Some(r) = dep_runway_data {
        let d_le = distance_m(p0, LatLon::new(r.le_lat, r.le_lon));
        let d_he = distance_m(p0, LatLon::new(r.he_lat, r.he_lon));
        let width = if r.width_ft > 0.0 { r.width_ft as f64 * 0.3048 } else { 45.0 };
        if d_le <= d_he {
            (LatLon::new(r.le_lat, r.le_lon), LatLon::new(r.he_lat, r.he_lon), width)
        } else {
            (LatLon::new(r.he_lat, r.he_lon), LatLon::new(r.le_lat, r.le_lon), width)
        }
    } else {
        let width = 45.0;
        let end = destination(p0, dep_heading_rad, 3300.0);
        (p0, end, width)
    };

    let dep_known_elevs = crate::preset::lookup_runway_threshold_elevations(
        first.latitude,
        first.longitude,
        dep_heading_rad.to_degrees(),
    );
    let (dep_start_elev, dep_end_elev) = if let Some((h0, h1)) = dep_known_elevs {
        (Some(h0), Some(h1))
    } else {
        (Some(pending.config.dep_elevation_m), Some(pending.config.dep_elevation_m))
    };

    let dep_half_width = (dep_width_m * 0.5 + 5.0).max(25.0);
    corridors.push(RunwayCorridor::new(
        dep_start.lon_deg,
        dep_start.lat_deg,
        dep_end.lon_deg,
        dep_end.lat_deg,
        dep_start_elev,
        dep_end_elev,
        dep_half_width,
        35.0,
    ));

    // 2. Arrival Runway Corridor
    let last = &points[points.len() - 1];
    let arr_on = points.iter().rev().take_while(|p| on_ground(p, last.altitude)).count();
    let touchdown_idx = points.len().saturating_sub(arr_on);
    let touchdown_pt = &points[touchdown_idx];
    let p_arr = LatLon::new(last.latitude, last.longitude);
    let mut arr_runway_data: Option<crate::flight_handle::RunwayData> = None;
    let mut best_arr_dist = f64::MAX;
    for r in &pending.runways {
        let d1 = distance_m(p_arr, LatLon::new(r.le_lat, r.le_lon));
        let d2 = distance_m(p_arr, LatLon::new(r.he_lat, r.he_lon));
        let d = d1.min(d2);
        if d < 10_000.0 && d < best_arr_dist {
            best_arr_dist = d;
            arr_runway_data = Some(r.clone());
        }
    }

    let arr_heading_rad = if arr_on > 1 {
        initial_bearing(
            LatLon::new(touchdown_pt.latitude, touchdown_pt.longitude),
            LatLon::new(last.latitude, last.longitude),
        )
    } else {
        last.heading_rad
    };

    let (arr_start, arr_end, arr_width_m) = if let Some(r) = arr_runway_data {
        let pt_touchdown = LatLon::new(touchdown_pt.latitude, touchdown_pt.longitude);
        let d_le = distance_m(pt_touchdown, LatLon::new(r.le_lat, r.le_lon));
        let d_he = distance_m(pt_touchdown, LatLon::new(r.he_lat, r.he_lon));
        let width = if r.width_ft > 0.0 { r.width_ft as f64 * 0.3048 } else { 45.0 };
        if d_le <= d_he {
            (LatLon::new(r.le_lat, r.le_lon), LatLon::new(r.he_lat, r.he_lon), width)
        } else {
            (LatLon::new(r.he_lat, r.he_lon), LatLon::new(r.le_lat, r.le_lon), width)
        }
    } else {
        let width = 45.0;
        let pt_touchdown = LatLon::new(touchdown_pt.latitude, touchdown_pt.longitude);
        let start = destination(pt_touchdown, arr_heading_rad + std::f64::consts::PI, 400.0);
        let end = destination(start, arr_heading_rad, 3300.0);
        (start, end, width)
    };

    let arr_known_elevs = crate::preset::lookup_runway_threshold_elevations(
        last.latitude,
        last.longitude,
        arr_heading_rad.to_degrees(),
    );
    let (arr_start_elev, arr_end_elev) = if let Some((h0, h1)) = arr_known_elevs {
        (Some(h0), Some(h1))
    } else {
        (Some(pending.config.arr_elevation_m), Some(pending.config.arr_elevation_m))
    };

    let arr_half_width = (arr_width_m * 0.5 + 5.0).max(25.0);
    corridors.push(RunwayCorridor::new(
        arr_start.lon_deg,
        arr_start.lat_deg,
        arr_end.lon_deg,
        arr_end.lat_deg,
        arr_start_elev,
        arr_end_elev,
        arr_half_width,
        35.0,
    ));

    corridors
}

/// Plans `pending` and builds what is drawn for it. `None` for a plan with no points.
fn build_flight(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    config: &wgpu::SurfaceConfiguration,
    camera_bind_group_layout: &wgpu::BindGroupLayout,
    pending: &PendingFlight,
) -> Option<FlightEntity> {
    use cesium_engine::property::Property;

    let points = generate(&pending.to_request());
    let calculated_duration_ms = points.last().map(|p| p.time_offset_ms).unwrap_or(0);

    let mut property = SampledPositionProperty::new().with_algorithm(InterpolationAlgorithm::CatmullRom);
    let mut sun_intensity_property = cesium_engine::property::sampled::SampledScalarProperty::new().with_algorithm(InterpolationAlgorithm::CatmullRom);

    for pt in &points {
        let ecef_array = lon_lat_alt_to_ecef_f64(pt.longitude, pt.latitude, pt.altitude);
        let position = DVec3::from_array(ecef_array);
        let time = SimulationTime::new(pt.time_offset_ms as f64 / 1000.0);
        property.add_sample(time, position);
        sun_intensity_property.add_sample(time, pt.sun_intensity as f64);
    }

    let start_time = property.start_time()?;
    let reference_point = property.evaluate(start_time).unwrap_or(glam::DVec3::ZERO);
    let ends = GroundEnds::new(&points)?;
    let mut builder = AdaptiveSubdivisionBuilder::new(1e-7); // High precision tolerance
    builder.max_segment_lengths = ends.line_spacing();
    let control_points = builder.build(&property, reference_point);
    let route_distances: Vec<(f32, f32)> =
        control_points.iter().map(|cp| (cp.progress, cp.distance)).collect();

    println!("Flight path loaded: {} ({} control points)", pending.id, control_points.len());

    let line = LineFit::new(control_points, &ends, &points);
    let mut renderer = PolylineRenderer::new(device, config, camera_bind_group_layout);
    // Upload geometry once; the terrain fit rewrites only the parts it moves.
    renderer.update_geometry(device, queue, line.points());

    let mut poly_config = PolylineConfig {
        color_end: [0.9, 0.9, 0.9, 1.0],
        ..PolylineConfig::default()
    };

    if pending.is_secondary {
        poly_config.split_progress = 0.5;
    }

    let runway_corridors = build_runway_corridors(pending, &points);

    Some(FlightEntity {
        id: pending.id.clone(),
        renderer,
        config: poly_config,
        property,
        sun_intensity_property,
        telemetry_points: points,
        total_duration_ms: calculated_duration_ms,
        reference_point,
        route_distances,
        ends,
        line,
        runway_corridors,
    })
}


pub struct FlightTrackerApp {
    pub progress: std::sync::Arc<std::sync::Mutex<f64>>,
    pub pending_flights: Vec<PendingFlight>,
    pub flights: Vec<FlightEntity>,
    pub airplane_renderer: Option<ModelRenderer>,
    /// Interior drawn in cockpit mode. Loaded lazily the first time the mode is entered,
    /// since the asset is far larger than the exterior and most sessions never need it.
    pub cockpit_renderer: Option<ModelRenderer>,
    /// Set once the load has failed so a missing asset isn't retried every frame.
    cockpit_load_failed: bool,
    pub last_update_time: std::time::Instant,
    pub is_playing: bool,
    pub play_speed: f64,
    pub view_mode: CameraMode,
    pub last_view_mode: CameraMode,
    pub reset_viewport: bool,
    command_rx: Option<mpsc::Receiver<FlightCommand>>,
    pub current_telemetry: std::sync::Arc<std::sync::Mutex<Option<FlightTelemetry>>>,
    /// The live camera's mode/position/rotation, refreshed every frame so the Android side
    /// can snapshot it (e.g. to persist across a flight being backgrounded and resumed).
    pub current_camera_state: std::sync::Arc<
        std::sync::Mutex<Option<(cesium_engine::camera::camera::CameraMode, glam::Vec3, glam::Quat)>>,
    >,
    /// A camera position/rotation to apply the next time the view resets (mode switch or a
    /// freshly loaded flight), consumed once. Used to restore a previously saved perspective
    /// instead of the mode's own default framing; `None` leaves today's default-reset behaviour
    /// untouched.
    pub pending_camera_restore: std::sync::Arc<std::sync::Mutex<Option<(glam::Vec3, glam::Quat)>>>,
    /// Cached from `init()` so we can create PolylineRenderers on-demand in `update()`.
    cached_surface_config: Option<wgpu::SurfaceConfiguration>,
    /// Planning options applied to flights loaded from here on.
    pub plan_config: FlightPlanConfig,
    /// How much of the route line is drawn. Applies to every loaded flight at once.
    pub route_line_mode: crate::flight_handle::RouteLineMode,
    /// Window extents the debug panel last set, in nautical miles, kept so the numbers
    /// survive a trip through Full or Hidden. FocusFlight sets its own over the FFI and
    /// never sees these.
    #[cfg(feature = "debug_panel")]
    debug_route_window_nm: (f64, f64),
    /// Text input buffer for custom route coordinates or preset in UI
    pub custom_route_input: String,
    /// Status or error message for route loading in UI
    pub route_status_msg: Option<(String, bool)>,
    /// Where the aircraft meets the rendered terrain, refreshed every frame by
    /// `sample_ground`; see [`crate::terrain_fit`].
    terrain: AircraftFit,
    /// Active runway corridors for terrain flattening.
    pub cached_corridors: Vec<cesium_engine::globe::terrain::RunwayCorridor>,
}

/// Height of the exterior model's origin above the flight position when airborne, metres.
const AIRBORNE_MODEL_LIFT_M: f64 = 7.5;
/// Gap left between the gear and the ground, metres.
const GEAR_CLEARANCE_M: f64 = 0.2;

impl FlightTrackerApp {
    /// Constructs the app and a handle for sending commands to it from other threads.
    pub fn with_handle() -> (Self, FlightHandle) {
        let (tx, rx) = mpsc::sync_channel(64);
        let progress = std::sync::Arc::new(std::sync::Mutex::new(0.0_f64));
        let current_telemetry = std::sync::Arc::new(std::sync::Mutex::new(None));
        let app = Self {
            progress,
            pending_flights: Vec::new(),
            flights: Vec::new(),
            airplane_renderer: None,
            cockpit_renderer: None,
            cockpit_load_failed: false,
            last_update_time: std::time::Instant::now(),
            terrain: AircraftFit::default(),
            cached_corridors: Vec::new(),
            is_playing: false,
            play_speed: 0.1,
            view_mode: cesium_engine::camera::camera::CameraMode::Free,
            last_view_mode: cesium_engine::camera::camera::CameraMode::Free,
            reset_viewport: true,
            command_rx: Some(rx),
            current_telemetry,
            current_camera_state: std::sync::Arc::new(std::sync::Mutex::new(None)),
            pending_camera_restore: std::sync::Arc::new(std::sync::Mutex::new(None)),
            cached_surface_config: None,
            plan_config: FlightPlanConfig::default(),
            route_line_mode: crate::flight_handle::RouteLineMode::default(),
            #[cfg(feature = "debug_panel")]
            debug_route_window_nm: (40.0, 150.0),
            custom_route_input: String::new(),
            route_status_msg: None,
        };
        (app, FlightHandle::new(tx))
    }

    /// Legacy constructor kept for backwards compat with the test harness.
    pub fn new(progress: std::sync::Arc<std::sync::Mutex<f64>>) -> Self {
        Self {
            progress,
            pending_flights: Vec::new(),
            flights: Vec::new(),
            airplane_renderer: None,
            cockpit_renderer: None,
            cockpit_load_failed: false,
            last_update_time: std::time::Instant::now(),
            terrain: AircraftFit::default(),
            cached_corridors: Vec::new(),
            is_playing: false,
            play_speed: 0.1,
            view_mode: cesium_engine::camera::camera::CameraMode::Free,
            last_view_mode: cesium_engine::camera::camera::CameraMode::Free,
            reset_viewport: true,
            command_rx: None,
            current_telemetry: std::sync::Arc::new(std::sync::Mutex::new(None)),
            current_camera_state: std::sync::Arc::new(std::sync::Mutex::new(None)),
            pending_camera_restore: std::sync::Arc::new(std::sync::Mutex::new(None)),
            cached_surface_config: None,
            plan_config: FlightPlanConfig::default(),
            route_line_mode: crate::flight_handle::RouteLineMode::default(),
            #[cfg(feature = "debug_panel")]
            debug_route_window_nm: (40.0, 150.0),
            custom_route_input: String::new(),
            route_status_msg: None,
        }
    }

    pub fn get_plane_state_at_time_delta(
        &self,
        progress_val: f64,
        delta_seconds: f64,
    ) -> Option<cesium_engine::math::trajectory::TransformState> {
        if let Some(flight) = self.flights.first() {
            let start_t = flight
                .property
                .start_time()
                .map(|t| t.seconds)
                .unwrap_or(0.0);
            let stop_t = flight
                .property
                .stop_time()
                .map(|t| t.seconds)
                .unwrap_or(1.0);
            let current_time_seconds = start_t + progress_val * (stop_t - start_t);
            let time =
                cesium_engine::time::SimulationTime::new(current_time_seconds + delta_seconds);

            let evaluator =
                cesium_engine::math::trajectory::TrajectoryEvaluator::new(&flight.property, 30.0);
            evaluator.evaluate(time)
        } else {
            None
        }
    }

    pub fn get_plane_state_at(
        &self,
        progress_val: f64,
    ) -> Option<cesium_engine::math::trajectory::TransformState> {
        let mut state = self.get_plane_state_at_time_delta(progress_val, 0.0);

        if let Some(ref mut s) = state {
            if progress_val > 0.999 {
                // Plane has arrived. Derive rotation robustly by looking exactly 1 second in the past.
                if let Some(prev_state) = self.get_plane_state_at_time_delta(progress_val, -1.0) {
                    s.rotation = prev_state.rotation;
                }
            }
            s.position += s.position.normalize() * (self.terrain.offset_m * 1.0e-6);
        }

        state
    }

    /// Refreshes the aircraft's [`AircraftFit`] for the current progress from the engine's
    /// ground query.
    fn fit_to_terrain(&mut self, ground: &dyn Fn(DVec3) -> Option<f64>) {
        let p = *self.progress.lock().unwrap();
        let Some(flight) = self.flights.first() else {
            self.terrain = AircraftFit::default();
            return;
        };
        let t_s = p * flight.total_duration_ms as f64 / 1000.0;
        let ends = flight.ends;
        let (Some(raw), Some(altitude_m)) = (
            self.get_plane_state_at_time_delta(p, 0.0),
            crate::terrain_fit::altitude_at(&flight.telemetry_points, t_s),
        ) else {
            return;
        };
        self.terrain.update(&ends, p, t_s, altitude_m, raw.position, ground);
    }

    pub fn get_sun_intensity_at(&self, progress_val: f64) -> Option<f64> {
        if let Some(flight) = self.flights.first() {
            let start_t = flight
                .property
                .start_time()
                .map(|t| t.seconds)
                .unwrap_or(0.0);
            let stop_t = flight
                .property
                .stop_time()
                .map(|t| t.seconds)
                .unwrap_or(1.0);
            let current_time_seconds = start_t + progress_val * (stop_t - start_t);
            let time = cesium_engine::time::SimulationTime::new(current_time_seconds);

            use cesium_engine::property::Property;
            flight
                .sun_intensity_property
                .evaluate(time)
                .map(|v| v.clamp(0.0, 1.0))
        } else {
            None
        }
    }

    pub fn get_telemetry_at(&self, progress_val: f64) -> Option<FlightTelemetry> {
        if let Some(flight) = self.flights.first() {
            let target_time_ms = (progress_val * flight.total_duration_ms as f64) as u64;
            let pts = &flight.telemetry_points;
            if pts.is_empty() { return None; }
            
            let idx = pts.partition_point(|p| p.time_offset_ms < target_time_ms);
            
            if idx == 0 {
                let p = &pts[0];
                Some(FlightTelemetry {
                    progress: progress_val,
                    latitude: p.latitude,
                    longitude: p.longitude,
                    altitude: p.altitude,
                    velocity_m_s: p.velocity_m_s,
                    heading_rad: p.heading_rad,
                    pitch_rad: p.pitch_rad,
                    roll_rad: p.roll_rad,
                })
            } else if idx >= pts.len() {
                let p = &pts[pts.len() - 1];
                Some(FlightTelemetry {
                    progress: progress_val,
                    latitude: p.latitude,
                    longitude: p.longitude,
                    altitude: p.altitude,
                    velocity_m_s: p.velocity_m_s,
                    heading_rad: p.heading_rad,
                    pitch_rad: p.pitch_rad,
                    roll_rad: p.roll_rad,
                })
            } else {
                let p0 = &pts[idx - 1];
                let p1 = &pts[idx];
                let dt = (p1.time_offset_ms - p0.time_offset_ms) as f64;
                let t = if dt > 0.0 { (target_time_ms as f64 - p0.time_offset_ms as f64) / dt } else { 0.0 };
                
                let lerp = |a, b| a + (b - a) * t;
                
                let mut d_heading = p1.heading_rad - p0.heading_rad;
                if d_heading > std::f64::consts::PI { d_heading -= 2.0 * std::f64::consts::PI; }
                if d_heading < -std::f64::consts::PI { d_heading += 2.0 * std::f64::consts::PI; }
                let heading_rad = p0.heading_rad + d_heading * t;
                
                Some(FlightTelemetry {
                    progress: progress_val,
                    latitude: lerp(p0.latitude, p1.latitude),
                    longitude: lerp(p0.longitude, p1.longitude),
                    altitude: lerp(p0.altitude, p1.altitude),
                    velocity_m_s: lerp(p0.velocity_m_s, p1.velocity_m_s),
                    heading_rad,
                    pitch_rad: lerp(p0.pitch_rad, p1.pitch_rad),
                    roll_rad: lerp(p0.roll_rad, p1.roll_rad),
                })
            }
        } else {
            None
        }
    }

    pub fn add_flight_path(
        &mut self,
        id: &str,
        departure_lon: f64,
        departure_lat: f64,
        arrival_lon: f64,
        arrival_lat: f64,
        total_duration_ms: u64,
        is_secondary: bool,
        runways: Vec<crate::flight_handle::RunwayData>,
    ) {
        let mut runways = runways;
        if runways.is_empty() {
            runways = crate::preset::lookup_airport_runways(departure_lat, departure_lon);
            runways.extend(crate::preset::lookup_airport_runways(arrival_lat, arrival_lon));
        }
        let mut config = self.plan_config;
        if let Some(elev) = crate::preset::lookup_airport_elevation(departure_lat, departure_lon) {
            config.dep_elevation_m = elev;
        }
        if let Some(elev) = crate::preset::lookup_airport_elevation(arrival_lat, arrival_lon) {
            config.arr_elevation_m = elev;
        }
        self.pending_flights.push(PendingFlight {
            id: id.to_string(),
            departure_lon,
            departure_lat,
            arrival_lon,
            arrival_lat,
            total_duration_ms,
            dep_heading_deg: None,
            arr_heading_deg: None,
            is_secondary,
            runways,
            config,
        });
    }

    /// Load a route from a pre-defined or parsed `FlightRouteDef`.
    pub fn load_route(&mut self, route: crate::preset::FlightRouteDef) {
        let mut runways = crate::preset::lookup_airport_runways(route.departure_lat, route.departure_lon);
        runways.extend(crate::preset::lookup_airport_runways(route.arrival_lat, route.arrival_lon));
        let mut config = self.plan_config;
        if let Some(elev) = route
            .dep_elevation_m
            .or_else(|| crate::preset::lookup_airport_elevation(route.departure_lat, route.departure_lon))
        {
            config.dep_elevation_m = elev;
        }
        if let Some(elev) = route
            .arr_elevation_m
            .or_else(|| crate::preset::lookup_airport_elevation(route.arrival_lat, route.arrival_lon))
        {
            config.arr_elevation_m = elev;
        }
        self.pending_flights.push(PendingFlight {
            id: route.id,
            departure_lon: route.departure_lon,
            departure_lat: route.departure_lat,
            arrival_lon: route.arrival_lon,
            arrival_lat: route.arrival_lat,
            total_duration_ms: route.total_duration_ms,
            dep_heading_deg: route.dep_heading_deg,
            arr_heading_deg: route.arr_heading_deg,
            is_secondary: false,
            runways,
            config,
        });
    }


    /// Draws the cockpit interior around the camera, at true world scale.
    ///
    /// The model is placed so its eye reference point lands exactly where the camera sits.
    /// Deriving that from the plane state rather than assuming the camera is already there
    /// keeps the interior correctly positioned under the debug god-camera too.
    fn render_cockpit<'res>(
        &'res self,
        render_pass: &mut wgpu::RenderPass<'res>,
        camera_bind_group: &'res wgpu::BindGroup,
        viewport_size: [f32; 2],
        camera_pos_f64: [f64; 3],
        airplane_state: Option<cesium_engine::math::trajectory::TransformState>,
    ) {
        let cockpit = match &self.cockpit_renderer {
            Some(c) => c,
            None => return,
        };
        let state = match airplane_state {
            Some(s) => s,
            None => return,
        };

        // Offset from the aircraft origin to the model origin, rotated into world space.
        // Differencing in f64 before the cast keeps the ~1.3e-6 Mm result well conditioned.
        let origin_local = crate::cockpit_model::model_origin_offset_mm();
        let origin_world = state.position
            + state.rotation
                * DVec3::new(
                    origin_local.x as f64,
                    origin_local.y as f64,
                    origin_local.z as f64,
                );
        let relative_pos_f64 = origin_world - DVec3::from_slice(&camera_pos_f64);
        let relative_pos = glam::Vec3::new(
            relative_pos_f64.x as f32,
            relative_pos_f64.y as f32,
            relative_pos_f64.z as f32,
        );

        let rot_f32 = glam::Quat::from_xyzw(
            state.rotation.x as f32,
            state.rotation.y as f32,
            state.rotation.z as f32,
            state.rotation.w as f32,
        )
        .normalize();

        // No yaw correction: the cockpit GLB is already -Z forward, matching the plane frame.
        let model_matrix = glam::Mat4::from_translation(relative_pos)
            * glam::Mat4::from_quat(rot_f32)
            * glam::Mat4::from_scale(glam::Vec3::splat(crate::cockpit_model::MODEL_SCALE));

        // Dim the cockpit toward night so the interior structure sits in deep shadow
        // and the emissive display panels brightly illuminate the flight deck.
        let sun = self
            .get_sun_intensity_at(*self.progress.lock().unwrap())
            .unwrap_or(1.0)
            .clamp(0.0, 1.0) as f32;
        let ambient_override = 0.05 + 0.29 * sun.powf(1.5);

        use cesium_engine::render::model_pipeline::pipeline::ModelPushConstants;
        let push = ModelPushConstants {
            model_matrix_0: model_matrix.x_axis.to_array(),
            model_matrix_1: model_matrix.y_axis.to_array(),
            model_matrix_2: model_matrix.z_axis.to_array(),
            model_matrix_3: model_matrix.w_axis.to_array(),
            camera_pos: [
                camera_pos_f64[0] as f32,
                camera_pos_f64[1] as f32,
                camera_pos_f64[2] as f32,
                1.0,
            ],
            viewport_size,
            // True world scale, and no depth bias — nothing here needs lifting off terrain.
            min_pixel_size: 0.0,
            depth_bias: 0.0,
            ambient_override,
            specular_strength: 0.15,
            detail_strength: 0.13,
            // No rim light indoors. It is an edge term that stands for light wrapping
            // round a silhouette against open sky, and inside a flight deck there is no
            // sky behind the window posts — it just added a flat 0.10 to every surface at
            // a grazing angle to the eye, which is precisely the frames, and made them
            // glow. Measured: the posts carried +0.102 of it while the panel carried 0.
            rim_strength: 0.0,
            // An interior is lit by what comes in through the windows, not by a bare sun.
            diffuse_weight: 0.35,
        };

        cockpit.draw(render_pass, camera_bind_group, push);
    }
}

impl GlobeExtension for FlightTrackerApp {
    fn init(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        config: &wgpu::SurfaceConfiguration,
        camera_bind_group_layout: &wgpu::BindGroupLayout,
    ) {
        self.cached_surface_config = Some(config.clone());

        // Drain any commands that were sent before init
        if let Some(rx) = &self.command_rx {
            while let Ok(cmd) = rx.try_recv() {
                match cmd {
                    FlightCommand::LoadFlight {
                        id,
                        departure_lon,
                        departure_lat,
                        arrival_lon,
                        arrival_lat,
                        dep_heading_deg,
                        arr_heading_deg,
                        dep_elevation_m,
                        arr_elevation_m,
                        is_secondary,
                        runways,
                        total_duration_ms,
                    } => {
                        let mut config = self.plan_config;
                        if let Some(elev) = dep_elevation_m
                            .or_else(|| crate::preset::lookup_airport_elevation(departure_lat, departure_lon))
                        {
                            config.dep_elevation_m = elev;
                        }
                        if let Some(elev) = arr_elevation_m
                            .or_else(|| crate::preset::lookup_airport_elevation(arrival_lat, arrival_lon))
                        {
                            config.arr_elevation_m = elev;
                        }
                        if !is_secondary {
                            self.pending_flights.retain(|f| f.is_secondary);
                        }
                        self.pending_flights.push(PendingFlight {
                            id,
                            departure_lon,
                            departure_lat,
                            arrival_lon,
                            arrival_lat,
                            total_duration_ms,
                            dep_heading_deg,
                            arr_heading_deg,
                            is_secondary,
                            runways,
                            config,
                        });
                    }
                    FlightCommand::SetRouteLineMode(m) => {
                        self.route_line_mode = m;
                    }
                    FlightCommand::SetPlanConfig(c) => {
                        self.plan_config = c;
                    }
                    FlightCommand::SetProgress(p) => {
                        *self.progress.lock().unwrap() = p.clamp(0.0, 1.0);
                    }
                    FlightCommand::SetSpeed(s) => {
                        self.play_speed = s;
                    }
                    FlightCommand::Play => self.is_playing = true,
                    FlightCommand::Pause => self.is_playing = false,
                }
            }
        }

        // Load the exterior aircraft model (baked into the binary)
        match crate::aircraft_model::load(device, queue, config, camera_bind_group_layout) {
            Ok(renderer) => {
                println!("A350-1000 successfully loaded and renderer initialized!");
                self.airplane_renderer = Some(renderer);
            }
            Err(e) => eprintln!("Failed to initialize ModelRenderer: {:?}", e),
        }

        for pending in self.pending_flights.drain(..) {
            if let Some(flight) = build_flight(device, queue, config, camera_bind_group_layout, &pending) {
                self.flights.push(flight);
            }
        }
        self.cached_corridors = self.flights.iter().flat_map(|f| f.runway_corridors.clone()).collect();
    }

    fn sample_ground(&mut self, ground: &dyn Fn(DVec3) -> Option<f64>) {
        for flight in &mut self.flights {
            flight.line.sample(&flight.ends, ground);
        }
        self.fit_to_terrain(ground);
    }


    fn update(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        _camera_pos_dvec3: DVec3,
        _frustum: &[DVec3; 4],
        camera: &mut cesium_engine::camera::camera::Camera,
        aspect_ratio: f32,
    ) {
        // Drain commands submitted via FlightHandle
        if let Some(rx) = &self.command_rx {
            while let Ok(cmd) = rx.try_recv() {
                match cmd {
                    FlightCommand::LoadFlight {
                        id,
                        departure_lon,
                        departure_lat,
                        arrival_lon,
                        arrival_lat,
                        dep_heading_deg,
                        arr_heading_deg,
                        dep_elevation_m,
                        arr_elevation_m,
                        is_secondary,
                        runways,
                        total_duration_ms,
                    } => {
                        log::info!("Received {} runways from Android database!", runways.len());
                        for r in &runways {
                            log::info!("Runway {}: {}x{}ft, LE {}° ({},{}), HE {}° ({},{})", 
                                r.airport_id, r.length_ft, r.width_ft, 
                                r.le_heading, r.le_lat, r.le_lon, 
                                r.he_heading, r.he_lat, r.he_lon);
                        }
                        let mut config = self.plan_config;
                        if let Some(elev) = dep_elevation_m
                            .or_else(|| crate::preset::lookup_airport_elevation(departure_lat, departure_lon))
                        {
                            config.dep_elevation_m = elev;
                        }
                        if let Some(elev) = arr_elevation_m
                            .or_else(|| crate::preset::lookup_airport_elevation(arrival_lat, arrival_lon))
                        {
                            config.arr_elevation_m = elev;
                        }
                        if !is_secondary {
                            self.pending_flights.retain(|f| f.is_secondary);
                        }
                        self.pending_flights.push(PendingFlight {
                            id,
                            departure_lon,
                            departure_lat,
                            arrival_lon,
                            arrival_lat,
                            total_duration_ms,
                            dep_heading_deg,
                            arr_heading_deg,
                            is_secondary,
                            runways,
                            config,
                        });
                    }
                    FlightCommand::SetRouteLineMode(m) => {
                        self.route_line_mode = m;
                    }
                    FlightCommand::SetPlanConfig(c) => {
                        self.plan_config = c;
                    }
                    FlightCommand::SetProgress(p) => {
                        *self.progress.lock().unwrap() = p.clamp(0.0, 1.0);
                    }
                    FlightCommand::SetSpeed(s) => {
                        self.play_speed = s;
                    }
                    FlightCommand::Play => self.is_playing = true,
                    FlightCommand::Pause => self.is_playing = false,
                }
            }
        }

        // On-demand flight materialization
        if !self.pending_flights.is_empty() {
            if let Some(config) = &self.cached_surface_config {
                // Clear old flights so the previous flight path doesn't stay visible
                self.flights.clear();
                
                // Reset playback state for the new flight
                *self.progress.lock().unwrap() = 0.0;
                self.is_playing = false;
                
                // Reset camera back to Tracking default perspective
                self.reset_viewport = true;

                let camera_bind_group_layout = camera_bind_group_layout(device);

                for pending in self.pending_flights.drain(..) {
                    if let Some(flight) =
                        build_flight(device, queue, config, &camera_bind_group_layout, &pending)
                    {
                        self.flights.push(flight);
                    }
                }
                self.cached_corridors = self.flights.iter().flat_map(|f| f.runway_corridors.clone()).collect();
            }
        }


        // The parts of each route line the terrain fit moved in `sample_ground`.
        for flight in &mut self.flights {
            for range in flight.line.take_changes().into_iter().flatten() {
                flight
                    .renderer
                    .write_points(queue, range.start, &flight.line.points()[range]);
            }
        }

        let now = std::time::Instant::now();
        let dt = now.duration_since(self.last_update_time).as_secs_f64();
        self.last_update_time = now;

        if self.is_playing {
            let mut p = *self.progress.lock().unwrap();
            p += self.play_speed * dt;
            if p > 1.0 {
                p = 1.0;
                self.is_playing = false;
            } else if p < 0.0 {
                p = 0.0;
                self.is_playing = false;
            }
            *self.progress.lock().unwrap() = p;
        }

        let current_progress = *self.progress.lock().unwrap();

        {
            // Always-on cost regardless of camera mode — kept separate from the
            // per-mode spans below so it isn't mistaken for mode-specific overhead.
            let _span = cesium_engine::core::trace::ScopedTrace::new("cesium.update.telemetry");

            // Update the shared telemetry object
            if let Some(telemetry) = self.get_telemetry_at(current_progress) {
                if let Ok(mut lock) = self.current_telemetry.lock() {
                    *lock = Some(telemetry);
                }
            }

            if let Ok(mut lock) = self.current_camera_state.lock() {
                *lock = Some((camera.mode, camera.local_pos, camera.local_ori));
            }

            if let Some(intensity) = self.get_sun_intensity_at(current_progress) {
                camera.sun_intensity = intensity as f32;
            }
        }

        // Camera Mode two-way sync — must happen before the flight loop so that
        // mode_switched_or_reset is correct for the dirty-flag check.
        let mut mode_switched_or_reset = false;
        if self.view_mode != self.last_view_mode {
            camera.mode = self.view_mode;
            self.last_view_mode = self.view_mode;
            self.reset_viewport = true;
        } else if camera.mode != self.view_mode {
            self.view_mode = camera.mode;
            self.last_view_mode = camera.mode;
            self.reset_viewport = true;
        }
        if self.reset_viewport {
            mode_switched_or_reset = true;
            self.reset_viewport = false;
        }

        // Pull the interior in the first time cockpit mode is entered. This blocks for the
        // asset read and parse, so it costs one frame on entry and nothing afterwards.
        if self.view_mode == CameraMode::Cockpit
            && self.cockpit_renderer.is_none()
            && !self.cockpit_load_failed
        {
            // One-time cost on cockpit-mode entry (asset read/parse/GPU upload) —
            // spanned separately since it's expected to be a spike, not steady-state.
            let _span = cesium_engine::core::trace::ScopedTrace::new("cesium.update.cockpit_model_prep");
            if let Some(config) = self.cached_surface_config.clone() {
                let layout = camera_bind_group_layout(device);
                self.cockpit_renderer =
                    crate::cockpit_model::load(device, queue, &config, &layout);
                self.cockpit_load_failed = self.cockpit_renderer.is_none();
            }
        }

        if let Some(state) = self.get_plane_state_at(current_progress) {
            // A line a frame, so debug.
            if log::log_enabled!(log::Level::Debug) {
                let (lon, lat) = cesium_engine::globe::geometry::ecef_to_lon_lat_f64(state.position);
                log::debug!(
                    "[FLIGHT MOVE] prog={:.3}% lat={:.4}° lon={:.4}° mode={:?}",
                    current_progress * 100.0,
                    lat,
                    lon,
                    self.view_mode
                );
            }
            match self.view_mode {
                CameraMode::Tracking => {
                    let _span = cesium_engine::core::trace::ScopedTrace::new("cesium.update.camera_mode.tracking");
                    crate::camera_modes::tracking::update_tracking_mode(
                        camera,
                        &state,
                        mode_switched_or_reset,
                    );
                }
                CameraMode::Cockpit => {
                    let _span = cesium_engine::core::trace::ScopedTrace::new("cesium.update.camera_mode.cockpit");
                    crate::camera_modes::cockpit::update_cockpit_mode(
                        camera,
                        &state,
                        mode_switched_or_reset,
                    );
                }
                CameraMode::Free => {
                    let _span = cesium_engine::core::trace::ScopedTrace::new("cesium.update.camera_mode.free");
                    crate::camera_modes::free::update_free_mode(
                        camera,
                        &self.flights,
                        aspect_ratio,
                        mode_switched_or_reset,
                    );
                }
            }
        } else if self.view_mode == CameraMode::Free {
            let _span = cesium_engine::core::trace::ScopedTrace::new("cesium.update.camera_mode.free");
            // Free mode does not require an active plane state
            crate::camera_modes::free::update_free_mode(
                camera,
                &self.flights,
                aspect_ratio,
                mode_switched_or_reset,
            );
        }

        // Applied after the mode's own default framing above, so it overrides rather than
        // races it. Only present on the one frame right after a restore was requested; consumed
        // immediately so later resets during the same session fall back to each mode's default
        // again, same as today.
        if mode_switched_or_reset {
            if let Some((pos, ori)) = self.pending_camera_restore.lock().unwrap().take() {
                camera.set_local_transform(pos, ori);
            }
        }
    }

    fn render<'res>(
        &'res self,
        render_pass: &mut wgpu::RenderPass<'res>,
        camera_bind_group: &'res wgpu::BindGroup,
        viewport_size: [f32; 2],
        camera_pos_f64: [f64; 3],
    ) {
        let current_progress = *self.progress.lock().unwrap();
        let airplane_state = self.get_plane_state_at(current_progress);

        // In cockpit mode we draw the cockpit interior.
        if self.view_mode == CameraMode::Cockpit {
            let _span = cesium_engine::core::trace::ScopedTrace::new("cesium.render.cockpit_model");
            self.render_cockpit(
                render_pass,
                camera_bind_group,
                viewport_size,
                camera_pos_f64,
                airplane_state,
            );
        }

        let _span = cesium_engine::core::trace::ScopedTrace::new("cesium.render.entities");
        // Hiding the route skips the ribbon entirely rather than drawing it transparent,
        // so it costs nothing. The aircraft below is drawn either way — it is the route
        // line that is hidden, not the flight.
        let ribbons: &[FlightEntity] =
            if self.route_line_mode == crate::flight_handle::RouteLineMode::Hidden {
                &[]
            } else {
                &self.flights
            };
        for flight in ribbons {
            let mut config = flight.config.clone();
            config.physical_half_width = 1.49 / 1_000_000.0;
            config.split_progress = current_progress as f32;

            // The window is handed to the shader as two absolute distances along the
            // route rather than as a radius, because that is what the ribbon's own
            // vertices carry and it costs two floats instead of three — and two is all
            // the 128-byte push-constant block has left.
            match self.route_line_mode {
                crate::flight_handle::RouteLineMode::Window { behind_m, ahead_m } => {
                    const M_TO_MM: f64 = 1.0 / 1_000_000.0;
                    let here = flight.distance_at(current_progress) as f64;
                    // Deliberately not clamped to the route's own start: letting the
                    // window hang off the front keeps the ribbon solid at the departure
                    // airport instead of fading it out over the first few miles.
                    config.window_start = (here - behind_m * M_TO_MM) as f32;
                    config.window_end = (here + ahead_m * M_TO_MM) as f32;
                }
                _ => {
                    config.window_start = -1.0;
                    config.window_end = -1.0;
                }
            }

            // Compute airplane position relative to camera in f64, then cast to f32.
            let airplane_ecef: Option<glam::DVec3> = airplane_state.map(|s| s.position);
            let cam_pos = glam::DVec3::from_slice(&camera_pos_f64);
            config.airplane_pos = if let Some(ecef) = airplane_ecef {
                let rel = ecef - cam_pos;
                [rel.x as f32, rel.y as f32, rel.z as f32, 1.0_f32] // w=1 activates camera-relative split
            } else {
                [0.0, 0.0, 0.0, 0.0] // w=0 falls back to legacy progress comparison
            };

            config.airplane_forward = if let Some(state) = airplane_state {
                let cur_rot = state.rotation;
                let rot_f32 = glam::Quat::from_xyzw(
                    cur_rot.x as f32,
                    cur_rot.y as f32,
                    cur_rot.z as f32,
                    cur_rot.w as f32,
                )
                .normalize();
                let forward = rot_f32 * glam::Vec3::new(0.0, 0.0, -1.0);
                [forward.x, forward.y, forward.z, 0.0]
            } else {
                [0.0, 0.0, 0.0, 0.0]
            };

            let _cam_pos_dvec3 = glam::DVec3::from_slice(&camera_pos_f64);

            flight.renderer.draw(DrawParams {
                render_pass,
                camera_bind_group,
                viewport_size,
                camera_pos_f64,
                reference_point: [
                    flight.reference_point.x,
                    flight.reference_point.y,
                    flight.reference_point.z,
                ],
                config: &config,
            });
        }

        // Draw airplane exterior if not in cockpit mode
        if self.view_mode != CameraMode::Cockpit {
            if let Some(airplane) = &self.airplane_renderer {
            if let Some(state) = airplane_state {
                // The model's scale grows with camera distance (below), about an origin
                // above its gear, so the lift that keeps it out of the ground has to be
                // worked out at that scale. Airborne it sits 7.5 m up, clear of the
                // ribbon's own 5 m z-fighting offset in polyline.wgsl; on the ground the
                // gear rests on the terrain; and at any height its lowest point stays
                // above the terrain under it however large the zoom has made it.
                let up_dir = state.position.normalize();
                let camera_pos = glam::DVec3::from_slice(&camera_pos_f64);
                let model_scale_m = (((state.position + up_dir * AIRBORNE_MODEL_LIFT_M * 1.0e-6)
                    - camera_pos)
                    .length()
                    * 0.008325)
                    .clamp(33.5e-6, 1.0)
                    * 1.0e6;
                let gear_depth_m = -(airplane.min_y as f64) * model_scale_m;
                let w = self.terrain.weight;
                let lift_m = (AIRBORNE_MODEL_LIFT_M * (1.0 - w) + (gear_depth_m + GEAR_CLEARANCE_M) * w)
                    .max(gear_depth_m + GEAR_CLEARANCE_M - self.terrain.agl_m);
                let elevated_position = state.position + up_dir * (lift_m * 1.0e-6);
                let relative_pos_f64 = elevated_position - camera_pos;
                let relative_pos = glam::Vec3::new(
                    relative_pos_f64.x as f32,
                    relative_pos_f64.y as f32,
                    relative_pos_f64.z as f32,
                );
                let translation = glam::Mat4::from_translation(relative_pos);

                let cur_rot = state.rotation;
                let rot_f32 = glam::Quat::from_xyzw(
                    cur_rot.x as f32,
                    cur_rot.y as f32,
                    cur_rot.z as f32,
                    cur_rot.w as f32,
                )
                .normalize();
                let rotation = glam::Mat4::from_quat(rot_f32);

                // Dynamic scaling based on camera distance
                let distance = relative_pos.length(); // Distance in Megameters

                // Desired length of the airplane in Megameters
                let desired_length_mm = distance * 0.008325;

                let min_length_mm = 33.5 / 1_000_000.0; // 33.5 meters (half of A350 length)
                let max_length_m = 1000.0 * 1000.0; // 1000 km
                let max_length_mm = max_length_m / 1_000_000.0;

                let clamped_length_mm = desired_length_mm.clamp(min_length_mm, max_length_mm);

                // The mesh is normalised to a bounding radius of 1.0 local unit, so this
                // is a radius in Megametres rather than a length despite the names.
                let scale_factor = clamped_length_mm / 1.0;
                let scale = glam::Mat4::from_scale(glam::Vec3::splat(scale_factor));

                // Apply a constant yaw correction to align the model with standard axes
                let model_correction = glam::Mat4::from_euler(
                    glam::EulerRot::YXZ,
                    crate::aircraft_model::YAW_CORRECTION, // Yaw
                    0.0,                                   // Pitch
                    0.0,                                   // Roll
                );

                let model_matrix = translation * rotation * scale * model_correction;

                    let sun = self
                        .get_sun_intensity_at(current_progress)
                        .unwrap_or(1.0) as f32;
                    let ambient_override = 0.03 + 0.15 * sun;
                    let rim_strength = 0.08 + 0.17 * sun;

                    use cesium_engine::render::model_pipeline::pipeline::ModelPushConstants;
                    let push = ModelPushConstants {
                        model_matrix_0: model_matrix.x_axis.to_array(),
                        model_matrix_1: model_matrix.y_axis.to_array(),
                        model_matrix_2: model_matrix.z_axis.to_array(),
                        model_matrix_3: model_matrix.w_axis.to_array(),
                        camera_pos: [
                            camera_pos_f64[0] as f32,
                            camera_pos_f64[1] as f32,
                            camera_pos_f64[2] as f32,
                            1.0,
                        ],
                        viewport_size,
                        min_pixel_size: 100.0,
                        depth_bias: 0.0,
                        // Ambient floor and rim scale with daylight, keeping the aircraft
                        // dark in nighttime silhouettes while rich in sunset and daylight.
                        ambient_override,
                        specular_strength: 0.35,
                        detail_strength: 0.0,
                        rim_strength,
                        diffuse_weight: 1.0,
                    };

                airplane.draw(render_pass, camera_bind_group, push);
            }
        }
    }
}

    #[cfg(feature = "debug_panel")]
    fn render_ui(&mut self, _ctx: &egui::Context, ui: &mut egui::Ui) {
        ui.label("Flight Controls");
        let mut p = *self.progress.lock().unwrap() as f32;
        if ui
            .add(egui::Slider::new(&mut p, 0.0..=1.0).text("Flight Progress"))
            .changed()
        {
            *self.progress.lock().unwrap() = p as f64;
            self.is_playing = false; // Pause when manually dragged
        }

        ui.horizontal(|ui| {
            if ui
                .button(if self.is_playing { "Pause" } else { "Play" })
                .clicked()
            {
                self.is_playing = !self.is_playing;
            }
            ui.add(egui::Slider::new(&mut self.play_speed, -0.5..=0.5).text("Speed"));
        });

        ui.separator();
        ui.label("Camera Mode");
        ui.horizontal(|ui| {
            ui.radio_value(&mut self.view_mode, CameraMode::Free, "Free");
            ui.radio_value(&mut self.view_mode, CameraMode::Tracking, "Tracking");
            ui.radio_value(&mut self.view_mode, CameraMode::Cockpit, "Cockpit");

            if ui.button("Reset Viewport").clicked() {
                self.reset_viewport = true;
            }
        });

        ui.separator();
        ui.label("Load Route Preset");
        ui.horizontal_wrapped(|ui| {
            for preset in crate::preset::PRESETS {
                if ui.button(preset.id).on_hover_text(preset.label).clicked() {
                    self.load_route(preset.to_route_def());
                    self.route_status_msg = Some((format!("Loaded {}", preset.id), false));
                }
            }
        });

        ui.horizontal(|ui| {
            ui.label("Custom Route:");
            let response = ui.add(
                egui::TextEdit::singleline(&mut self.custom_route_input)
                    .hint_text("preset or lat1,lon1,lat2,lon2")
                    .desired_width(180.0),
            );
            let enter_pressed = response.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
            if ui.button("Load").clicked() || enter_pressed {
                match crate::preset::parse_route(&self.custom_route_input) {
                    Ok(route_def) => {
                        let id = route_def.id.clone();
                        self.load_route(route_def);
                        self.route_status_msg = Some((format!("Loaded {}", id), false));
                    }
                    Err(e) => {
                        self.route_status_msg = Some((e, true));
                    }
                }
            }
        });

        if let Some((msg, is_error)) = &self.route_status_msg {
            if *is_error {
                ui.colored_label(egui::Color32::RED, msg);
            } else {
                ui.colored_label(egui::Color32::GREEN, msg);
            }
        }

        ui.separator();
        ui.label("Route Line");
        {
            use crate::flight_handle::RouteLineMode;
            const NM: f64 = crate::telemetry::geo::NAUTICAL_MILE_M;
            let (behind_nm, ahead_nm) = self.debug_route_window_nm;
            let windowed = |b: f64, a: f64| RouteLineMode::Window {
                behind_m: b * NM,
                ahead_m: a * NM,
            };
            let mode = self.route_line_mode;
            ui.horizontal(|ui| {
                if ui.radio(matches!(mode, RouteLineMode::Full), "Full").clicked() {
                    self.route_line_mode = RouteLineMode::Full;
                }
                if ui
                    .radio(matches!(mode, RouteLineMode::Window { .. }), "Window")
                    .clicked()
                {
                    self.route_line_mode = windowed(behind_nm, ahead_nm);
                }
                if ui.radio(matches!(mode, RouteLineMode::Hidden), "Hidden").clicked() {
                    self.route_line_mode = RouteLineMode::Hidden;
                }
            });
            if matches!(self.route_line_mode, RouteLineMode::Window { .. }) {
                let mut changed = false;
                ui.horizontal(|ui| {
                    ui.label("behind");
                    changed |= ui
                        .add(
                            egui::DragValue::new(&mut self.debug_route_window_nm.0)
                                .speed(1.0)
                                .range(0.0..=5000.0),
                        )
                        .changed();
                    ui.label("ahead");
                    changed |= ui
                        .add(
                            egui::DragValue::new(&mut self.debug_route_window_nm.1)
                                .speed(1.0)
                                .range(0.0..=5000.0),
                        )
                        .changed();
                    ui.label("NM");
                });
                if changed {
                    let (b, a) = self.debug_route_window_nm;
                    self.route_line_mode = windowed(b, a);
                }
            }
        }
    }

    fn runway_corridors(&self) -> &[cesium_engine::globe::terrain::RunwayCorridor] {
        &self.cached_corridors
    }
}

