use glam::{DVec3, DQuat};

pub fn linear_dvec3(p0: DVec3, p1: DVec3, t: f64) -> DVec3 {
    p0.lerp(p1, t)
}

pub fn slerp_dquat(q0: DQuat, q1: DQuat, t: f64) -> DQuat {
    q0.slerp(q1, t)
}

pub fn hermite_dvec3(p0: DVec3, m0: DVec3, p1: DVec3, m1: DVec3, t: f64) -> DVec3 {
    let t2 = t * t;
    let t3 = t2 * t;

    let h00 = 2.0 * t3 - 3.0 * t2 + 1.0;
    let h10 = t3 - 2.0 * t2 + t;
    let h01 = -2.0 * t3 + 3.0 * t2;
    let h11 = t3 - t2;

    p0 * h00 + m0 * h10 + p1 * h01 + m1 * h11
}

pub fn catmull_rom_dvec3(p0: DVec3, p1: DVec3, p2: DVec3, p3: DVec3, t: f64) -> DVec3 {
    let m1 = (p2 - p0) * 0.5;
    let m2 = (p3 - p1) * 0.5;
    hermite_dvec3(p1, m1, p2, m2, t)
}
