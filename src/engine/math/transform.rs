use glam::{DVec3, DQuat, DMat3};
use crate::engine::globe::geometry::{EARTH_RADIUS_A_F64, EARTH_RADIUS_B_F64};

const INV_A2_F64: f64 = 1.0 / (EARTH_RADIUS_A_F64 * EARTH_RADIUS_A_F64);
const INV_B2_F64: f64 = 1.0 / (EARTH_RADIUS_B_F64 * EARTH_RADIUS_B_F64);

/// Computes the surface normal vector for a given ECEF position on the WGS84 ellipsoid.
pub fn surface_normal_ecef(ecef: DVec3) -> DVec3 {
    let nx = ecef.x * INV_A2_F64;
    let ny = ecef.y * INV_B2_F64;
    let nz = ecef.z * INV_A2_F64;
    DVec3::new(nx, ny, nz).normalize()
}

/// Computes the East-North-Up (ENU) rotation matrix at a given ECEF position.
pub fn enu_matrix_at_ecef(ecef: DVec3) -> DMat3 {
    let up = surface_normal_ecef(ecef);
    
    // North pole is special case
    let mut east = DVec3::new(0.0, 1.0, 0.0).cross(up);
    if east.length_squared() < 1e-8 {
        east = DVec3::new(1.0, 0.0, 0.0);
    } else {
        east = east.normalize();
    }
    
    let north = up.cross(east).normalize();
    
    // Columns of the matrix are East, North, Up
    DMat3::from_cols(east, north, up)
}

/// Creates a quaternion orientation from a velocity vector at a given ECEF position.
/// Aligns the "forward" direction with the velocity, and keeps "up" aligned with the local surface normal (no banking).
pub fn velocity_to_orientation(ecef: DVec3, velocity: DVec3) -> DQuat {
    if velocity.length_squared() < 1e-8 {
        return DQuat::IDENTITY;
    }
    
    let forward = velocity.normalize();
    let global_up = surface_normal_ecef(ecef);
    
    let right = forward.cross(global_up);
    if right.length_squared() < 1e-8 {
        // Velocity is straight up or down
        let enu = enu_matrix_at_ecef(ecef);
        return DQuat::from_mat3(&enu);
    }
    
    let right = right.normalize();
    let local_up = right.cross(forward).normalize();
    
    // We want the object's local -Z to be forward, local +Y to be up, local +X to be right
    // This matches the standard Right-Handed GL/glTF coordinate system
    // Column 0: X axis (right)
    // Column 1: Y axis (up)
    // Column 2: Z axis (backward)
    let rot_mat = DMat3::from_cols(right, local_up, -forward);
    
    DQuat::from_mat3(&rot_mat)
}
