use std::sync::Mutex;
use jni::{
    objects::JClass,
    sys::{jdouble, jint, jlong},
    JNIEnv,
};

use cesium_flight::flight_handle::FlightHandle;
use crate::api::{CameraMode, ViewerHandle};

pub struct PendingFlightData {
    pub dep_lon: f64,
    pub dep_lat: f64,
    pub arr_lon: f64,
    pub arr_lat: f64,
    pub duration_ms: u64,
}

// Global state to bridge Kotlin and the android_main thread
pub static FLIGHT_DATA: Mutex<Option<PendingFlightData>> = Mutex::new(None);
pub static FLIGHT_HANDLE: Mutex<Option<FlightHandle>> = Mutex::new(None);
pub static VIEWER_HANDLE: Mutex<Option<ViewerHandle>> = Mutex::new(None);

#[no_mangle]
pub extern "system" fn Java_com_example_focusflight_engine_CesiumBridge_nativeSetPendingFlight(
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
pub extern "system" fn Java_com_example_focusflight_engine_CesiumBridge_nativeSetProgress(
    mut _env: JNIEnv,
    _cls: JClass,
    progress: jdouble,
) {
    if let Some(handle) = FLIGHT_HANDLE.lock().unwrap().as_ref() {
        handle.set_progress(progress);
    }
}

#[no_mangle]
pub extern "system" fn Java_com_example_focusflight_engine_CesiumBridge_nativeSetCameraMode(
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

pub static CURRENT_TELEMETRY: Mutex<Option<std::sync::Arc<std::sync::Mutex<Option<cesium_flight::tracker::FlightTelemetry>>>>> = Mutex::new(None);

#[no_mangle]
pub extern "system" fn Java_com_example_focusflight_engine_CesiumBridge_nativeGetTelemetry(
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
