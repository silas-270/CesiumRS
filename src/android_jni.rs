use std::sync::Mutex;
use std::sync::atomic::Ordering;
use jni::{
    objects::JClass,
    sys::{jdouble, jint, jlong},
    JNIEnv,
};

use cesium_flight::flight_handle::{FlightHandle, RunwayData};
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
pub static RUNWAY_DATA: Mutex<Option<Vec<RunwayData>>> = Mutex::new(None);
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

#[no_mangle]
pub extern "system" fn Java_com_example_focusflight_engine_CesiumBridge_nativeSetRenderingEnabled(
    mut _env: JNIEnv,
    _cls: JClass,
    enabled: jni::sys::jboolean,
) {
    cesium_engine::core::app::RENDERING_ENABLED.store(enabled != 0, Ordering::Relaxed);
}

#[no_mangle]
pub extern "system" fn Java_com_example_focusflight_engine_CesiumBridge_nativeLoadPendingFlight(
    mut _env: JNIEnv,
    _cls: JClass,
) {
    if let Some(data) = FLIGHT_DATA.lock().unwrap().take() {
        let runways = RUNWAY_DATA.lock().unwrap().take().unwrap_or_default();
        if let Some(handle) = FLIGHT_HANDLE.lock().unwrap().as_ref() {
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

#[no_mangle]
pub extern "system" fn Java_com_example_focusflight_engine_CesiumBridge_nativeSetRunways(
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
