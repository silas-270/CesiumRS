use std::sync::Mutex;
use std::sync::atomic::Ordering;
use jni::{
    objects::JClass,
    sys::{jdouble, jint, jlong},
    JNIEnv,
};

use cesium_flight::flight_handle::{FlightHandle, RunwayData};
use crate::api::{CameraMode, MapStyle, ViewerHandle};

pub struct PendingFlightData {
    pub dep_lon: f64,
    pub dep_lat: f64,
    pub arr_lon: f64,
    pub arr_lat: f64,
    pub duration_ms: u64,
}

// Global state to bridge Kotlin and the android_main thread
/// Field elevations for the next flight, in metres.
///
/// Separate from `FLIGHT_DATA` so that supplying them stays optional: when nothing has
/// been set, flights are planned at sea level, which is what the globe currently
/// renders. See `FlightPlanConfig::terrain_elevation`.
pub static FIELD_ELEVATIONS: Mutex<Option<(f64, f64)>> = Mutex::new(None);

pub static FLIGHT_DATA: Mutex<Option<PendingFlightData>> = Mutex::new(None);
pub static RUNWAY_DATA: Mutex<Option<Vec<RunwayData>>> = Mutex::new(None);
pub static FLIGHT_HANDLE: Mutex<Option<FlightHandle>> = Mutex::new(None);
pub static VIEWER_HANDLE: Mutex<Option<ViewerHandle>> = Mutex::new(None);
pub static EVENT_LOOP_PROXY: Mutex<Option<winit::event_loop::EventLoopProxy<cesium_engine::core::app::EngineEvent>>> = Mutex::new(None);

pub static CURRENT_CAMERA_STATE: Mutex<
    Option<std::sync::Arc<std::sync::Mutex<Option<(cesium_engine::camera::camera::CameraMode, glam::Vec3, glam::Quat)>>>>,
> = Mutex::new(None);
pub static PENDING_CAMERA_RESTORE: Mutex<
    Option<std::sync::Arc<std::sync::Mutex<Option<(glam::Vec3, glam::Quat)>>>>,
> = Mutex::new(None);



#[no_mangle]
pub extern "system" fn Java_com_example_focusflight_engine_live_CesiumLiveJniBridge_nativeSetPendingFlight(
    mut _env: JNIEnv,
    _cls: JClass,
    dep_lon: jdouble,
    dep_lat: jdouble,
    arr_lon: jdouble,
    arr_lat: jdouble,
    duration_ms: jlong,
) {
    *FLIGHT_DATA.lock().unwrap() = Some(PendingFlightData {
        dep_lon,
        dep_lat,
        arr_lon,
        arr_lat,
        duration_ms: duration_ms as u64,
    });
}

#[no_mangle]
pub extern "system" fn Java_com_example_focusflight_engine_live_CesiumLiveJniBridge_nativeSetProgress(
    mut _env: JNIEnv,
    _cls: JClass,
    progress: jdouble,
) {
    if let Some(handle) = FLIGHT_HANDLE.lock().unwrap().as_ref() {
        handle.set_progress(progress);
    }
}

#[no_mangle]
pub extern "system" fn Java_com_example_focusflight_engine_live_CesiumLiveJniBridge_nativeSetCameraMode(
    mut _env: JNIEnv,
    _cls: JClass,
    mode: jint,
) {
    if let Some(handle) = VIEWER_HANDLE.lock().unwrap().as_ref() {
        let m = match mode {
            1 => CameraMode::Tracking,
            2 => CameraMode::Cockpit,
            _ => CameraMode::Free,
        };
        handle.camera_set_mode(m);
    }
}

#[no_mangle]
pub extern "system" fn Java_com_example_focusflight_engine_live_CesiumLiveJniBridge_nativeSetMapStyle(
    mut _env: JNIEnv,
    _cls: JClass,
    style: jint,
) {
    if let Some(handle) = VIEWER_HANDLE.lock().unwrap().as_ref() {
        let s = match style {
            1 => MapStyle::Satellite,
            _ => MapStyle::Standard,
        };
        handle.map_set_style(s);
    }
}

pub static CURRENT_TELEMETRY: Mutex<Option<std::sync::Arc<std::sync::Mutex<Option<cesium_flight::tracker::FlightTelemetry>>>>> = Mutex::new(None);

#[no_mangle]
pub extern "system" fn Java_com_example_focusflight_engine_live_CesiumLiveJniBridge_nativeGetTelemetry(
    env: JNIEnv,
    _cls: JClass,
) -> jni::sys::jdoubleArray {
    let telemetry_opt = if let Some(arc) = CURRENT_TELEMETRY.lock().unwrap().as_ref() {
        *arc.lock().unwrap()
    } else {
        None
    };
    
    // If we have no telemetry, return a zeroed array or null. We'll return an array of 8 zeros.
    let vals = if let Some(t) = telemetry_opt {
        [t.progress, t.latitude, t.longitude, t.altitude, t.velocity_m_s, t.heading_rad, t.pitch_rad, t.roll_rad]
    } else {
        [0.0; 8]
    };
    
    let array = env.new_double_array(8).unwrap();
    env.set_double_array_region(&array, 0, &vals).unwrap();
    array.into_raw()
}

/// Snapshot of the live camera's mode/position/rotation, for persisting across a flight being
/// backgrounded and resumed. `[mode (0=Free/1=Tracking/2=Cockpit), pos.x, pos.y, pos.z, ori.x,
/// ori.y, ori.z, ori.w]`, mirroring `nativeGetTelemetry`'s shape. All zeros if unavailable.
#[no_mangle]
pub extern "system" fn Java_com_example_focusflight_engine_live_CesiumLiveJniBridge_nativeGetCameraPose(
    env: JNIEnv,
    _cls: JClass,
) -> jni::sys::jdoubleArray {
    let state_opt = if let Some(arc) = CURRENT_CAMERA_STATE.lock().unwrap().as_ref() {
        *arc.lock().unwrap()
    } else {
        None
    };

    let vals = if let Some((mode, pos, ori)) = state_opt {
        let mode_val = match mode {
            cesium_engine::camera::camera::CameraMode::Tracking => 1.0,
            cesium_engine::camera::camera::CameraMode::Cockpit => 2.0,
            cesium_engine::camera::camera::CameraMode::Free => 0.0,
        };
        [
            mode_val,
            pos.x as f64,
            pos.y as f64,
            pos.z as f64,
            ori.x as f64,
            ori.y as f64,
            ori.z as f64,
            ori.w as f64,
        ]
    } else {
        [0.0; 8]
    };

    let array = env.new_double_array(8).unwrap();
    env.set_double_array_region(&array, 0, &vals).unwrap();
    array.into_raw()
}

/// Applies a previously saved camera position/rotation the next time the view resets (mode
/// switch or a freshly loaded flight). Call `nativeSetCameraMode` first so the mode itself is
/// already correct when this lands. A no-op for a brand-new flight, which simply never calls it.
#[no_mangle]
pub extern "system" fn Java_com_example_focusflight_engine_live_CesiumLiveJniBridge_nativeSetCameraPose(
    mut _env: JNIEnv,
    _cls: JClass,
    x: jdouble,
    y: jdouble,
    z: jdouble,
    qx: jdouble,
    qy: jdouble,
    qz: jdouble,
    qw: jdouble,
) {
    if let Some(arc) = PENDING_CAMERA_RESTORE.lock().unwrap().as_ref() {
        let pos = glam::Vec3::new(x as f32, y as f32, z as f32);
        let ori = glam::Quat::from_xyzw(qx as f32, qy as f32, qz as f32, qw as f32);
        *arc.lock().unwrap() = Some((pos, ori));
    }
}

#[no_mangle]
pub extern "system" fn Java_com_example_focusflight_engine_live_CesiumLiveJniBridge_nativeSetRenderingEnabled(
    mut _env: JNIEnv,
    _cls: JClass,
    enabled: jni::sys::jboolean,
) {
    cesium_engine::core::app::RENDERING_ENABLED.store(enabled != 0, Ordering::Relaxed);
}

#[no_mangle]
pub extern "system" fn Java_com_example_focusflight_engine_live_CesiumLiveJniBridge_nativeSetSuspended(
    mut _env: JNIEnv,
    _cls: JClass,
    suspended: jni::sys::jboolean,
) {
    let is_suspended = suspended != 0;
    
    // Send event to wake up the event loop and tell it to suspend or resume
    if let Some(proxy) = EVENT_LOOP_PROXY.lock().unwrap().as_ref() {
        let event = if is_suspended {
            cesium_engine::core::app::EngineEvent::Suspend
        } else {
            cesium_engine::core::app::EngineEvent::Resume
        };
        let _ = proxy.send_event(event);
    }
}

#[no_mangle]
pub extern "system" fn Java_com_example_focusflight_engine_live_CesiumLiveJniBridge_nativeDestroyEngine(
    mut _env: JNIEnv,
    _cls: JClass,
) {
    // Send Destroy so the winit event loop exits cleanly and WgpuState drops,
    // releasing all Vulkan resources. Only called when isChangingConfigurations is false.
    if let Some(proxy) = EVENT_LOOP_PROXY.lock().unwrap().take() {
        let _ = proxy.send_event(cesium_engine::core::app::EngineEvent::Destroy);
    }
}

#[no_mangle]
pub extern "system" fn Java_com_example_focusflight_engine_live_CesiumLiveJniBridge_nativeLoadPendingFlight(
    mut _env: JNIEnv,
    _cls: JClass,
) {
    let handle_lock = FLIGHT_HANDLE.lock().unwrap();
    if let Some(handle) = handle_lock.as_ref() {
        if let Some(data) = FLIGHT_DATA.lock().unwrap().take() {
            let runways = RUNWAY_DATA.lock().unwrap().take().unwrap_or_default();
            let mut config = cesium_flight::telemetry::FlightPlanConfig::default();
            if let Some((dep_m, arr_m)) = FIELD_ELEVATIONS.lock().unwrap().take() {
                config.terrain_elevation = true;
                config.dep_elevation_m = dep_m;
                config.arr_elevation_m = arr_m;
            }
            handle.set_plan_config(config);
            handle.load_flight(
                "primary",
                data.dep_lon,
                data.dep_lat,
                data.arr_lon,
                data.arr_lat,
                data.duration_ms,
                None,
                None,
                runways,
            );
        }
    }
}

/// Debug-only hook for performance testing (see tools/run_perf_scenario.sh):
/// tags a captured Perfetto trace with `scenario_id` and switches camera mode
/// for the steady-state scenarios. Only exported when built with
/// `--features perf_trace`; never present in a shipped release `.so`.
#[cfg(feature = "perf_trace")]
#[no_mangle]
pub extern "system" fn Java_com_example_focusflight_engine_live_CesiumLiveJniBridge_nativeRunPerfScenario(
    mut _env: JNIEnv,
    _cls: JClass,
    scenario_id: jint,
) {
    if let Some(handle) = VIEWER_HANDLE.lock().unwrap().as_ref() {
        handle.run_perf_scenario(scenario_id);
    }
}

/// Supplies the elevations of the two airports for the next flight, and by doing so
/// turns terrain-aware planning on for it.
///
/// Optional, and currently uncalled: the globe renders no terrain, so a flight starting
/// at a real field elevation would visibly float above a sea-level surface. The planner
/// handles elevation correctly either way — the takeoff roll lengthens in thin air, and
/// cruise levels are checked against the ground beneath them — so the Kotlin side only
/// needs to declare and call this once the globe has terrain to sit on.
#[no_mangle]
pub extern "system" fn Java_com_example_focusflight_engine_live_CesiumLiveJniBridge_nativeSetFieldElevations(
    mut _env: JNIEnv,
    _cls: JClass,
    dep_elevation_m: jdouble,
    arr_elevation_m: jdouble,
) {
    *FIELD_ELEVATIONS.lock().unwrap() = Some((dep_elevation_m, arr_elevation_m));
}

#[no_mangle]
pub extern "system" fn Java_com_example_focusflight_engine_live_CesiumLiveJniBridge_nativeSetRunways(
    mut env: JNIEnv,
    _cls: JClass,
    airport_ids: jni::objects::JIntArray,
    length_ft: jni::objects::JFloatArray,
    width_ft: jni::objects::JFloatArray,
    le_heading: jni::objects::JFloatArray,
    le_lat: jni::objects::JDoubleArray,
    le_lon: jni::objects::JDoubleArray,
    he_heading: jni::objects::JFloatArray,
    he_lat: jni::objects::JDoubleArray,
    he_lon: jni::objects::JDoubleArray,
) {
    let count = env.get_array_length(&airport_ids).unwrap_or(0) as usize;
    if count == 0 {
        *RUNWAY_DATA.lock().unwrap() = Some(vec![]);
        return;
    }
    
    let mut vec_airport_ids = vec![0i32; count];
    let mut vec_length_ft = vec![0f32; count];
    let mut vec_width_ft = vec![0f32; count];
    let mut vec_le_heading = vec![0f32; count];
    let mut vec_le_lat = vec![0f64; count];
    let mut vec_le_lon = vec![0f64; count];
    let mut vec_he_heading = vec![0f32; count];
    let mut vec_he_lat = vec![0f64; count];
    let mut vec_he_lon = vec![0f64; count];

    let _ = env.get_int_array_region(&airport_ids, 0, &mut vec_airport_ids);
    let _ = env.get_float_array_region(&length_ft, 0, &mut vec_length_ft);
    let _ = env.get_float_array_region(&width_ft, 0, &mut vec_width_ft);
    let _ = env.get_float_array_region(&le_heading, 0, &mut vec_le_heading);
    let _ = env.get_double_array_region(&le_lat, 0, &mut vec_le_lat);
    let _ = env.get_double_array_region(&le_lon, 0, &mut vec_le_lon);
    let _ = env.get_float_array_region(&he_heading, 0, &mut vec_he_heading);
    let _ = env.get_double_array_region(&he_lat, 0, &mut vec_he_lat);
    let _ = env.get_double_array_region(&he_lon, 0, &mut vec_he_lon);

    let mut runways = Vec::with_capacity(count);
    for i in 0..count {
        runways.push(RunwayData {
            airport_id: vec_airport_ids[i],
            length_ft: vec_length_ft[i],
            width_ft: vec_width_ft[i],
            le_heading: vec_le_heading[i],
            le_lat: vec_le_lat[i],
            le_lon: vec_le_lon[i],
            he_heading: vec_he_heading[i],
            he_lat: vec_he_lat[i],
            he_lon: vec_he_lon[i],
        });
    }
    
    *RUNWAY_DATA.lock().unwrap() = Some(runways);
}
