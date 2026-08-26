use cesium_engine::camera::camera::Camera;
use cesium_engine::math::trajectory::TransformState;

/// Where the pilot's head sits in the aircraft's local frame, in Megametres.
///
/// The plane's local -Z points forward, so this is 44 m forward of the aircraft origin and
/// 17 m up. `cockpit_model` lines the interior up against this point, so the two must stay
/// in sync.
pub const CAMERA_LOCAL_MM: glam::Vec3 = glam::Vec3::new(0.0, 17.0 / 1_000_000.0, -44.0 / 1_000_000.0);

/// Downward tilt of the default cockpit view, in radians (8 degrees).
///
/// Looking straight down the boresight puts the instrument panel just under the bottom of
/// the frame, since the aircraft flies slightly nose-up. A small tilt frames the glareshield
/// and displays along the bottom edge while keeping the horizon comfortably in view.
const DEFAULT_PITCH_DOWN: f32 = 0.14;

/// Persistent aft-and-up nudge on top of [`CAMERA_LOCAL_MM`], aircraft-local frame
/// (+Y up, +Z aft). Applied only to the camera, not the interior model, so it pulls
/// the eye back from the seat rather than moving the whole cockpit with it. Starting
/// values — tune visually via `--cockpit-s23`.
const CAMERA_PULLBACK_MM: glam::Vec3 =
    glam::Vec3::new(0.0, 0.03 / 1_000_000.0, 0.18 / 1_000_000.0);

pub fn update_cockpit_mode(
    camera: &mut Camera,
    state: &TransformState,
    mode_switched_or_reset: bool,
) {
    // Cockpit is tied to the plane's exact rotation (banks and pitches with it).
    camera.set_anchor(state.position, state.rotation);

    // Re-applied every frame, not just on entry: the seat is bolted to the airframe, so
    // the interior can never drift out from around the camera.
    camera.local_pos = CAMERA_LOCAL_MM + CAMERA_PULLBACK_MM;

    if mode_switched_or_reset {
        // Along the nose, tilted slightly down so the panel is in frame.
        camera.local_ori = glam::Quat::from_axis_angle(glam::Vec3::X, -DEFAULT_PITCH_DOWN);
    }
}
