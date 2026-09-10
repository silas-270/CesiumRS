use glam::DVec3;

pub fn linear_dvec3(p0: DVec3, p1: DVec3, t: f64) -> DVec3 {
    p0.lerp(p1, t)
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

/// Time-aware non-uniform Catmull-Rom interpolation for DVec3.
/// Accounts for varying time intervals between sample points, eliminating overshoots and kinks.
pub fn catmull_rom_timed_dvec3(
    p0: DVec3,
    p1: DVec3,
    p2: DVec3,
    p3: DVec3,
    dt01: f64,
    dt12: f64,
    dt23: f64,
    t: f64,
) -> DVec3 {
    let dt01 = dt01.max(1e-6);
    let dt12 = dt12.max(1e-6);
    let dt23 = dt23.max(1e-6);

    let secant01 = (p1 - p0) / dt01;
    let secant12 = (p2 - p1) / dt12;
    let secant23 = (p3 - p2) / dt23;

    let v1 = secant12 * (dt01 / (dt01 + dt12)) + secant01 * (dt12 / (dt01 + dt12));
    let v2 = secant23 * (dt12 / (dt12 + dt23)) + secant12 * (dt23 / (dt12 + dt23));

    let m1 = v1 * dt12;
    let m2 = v2 * dt12;

    hermite_dvec3(p1, m1, p2, m2, t)
}

pub fn linear_f64(p0: f64, p1: f64, t: f64) -> f64 {
    p0 + (p1 - p0) * t
}

pub fn hermite_f64(p0: f64, m0: f64, p1: f64, m1: f64, t: f64) -> f64 {
    let t2 = t * t;
    let t3 = t2 * t;

    let h00 = 2.0 * t3 - 3.0 * t2 + 1.0;
    let h10 = t3 - 2.0 * t2 + t;
    let h01 = -2.0 * t3 + 3.0 * t2;
    let h11 = t3 - t2;

    p0 * h00 + m0 * h10 + p1 * h01 + m1 * h11
}

pub fn catmull_rom_f64(p0: f64, p1: f64, p2: f64, p3: f64, t: f64) -> f64 {
    let m1 = (p2 - p0) * 0.5;
    let m2 = (p3 - p1) * 0.5;
    hermite_f64(p1, m1, p2, m2, t)
}

/// Time-aware non-uniform Catmull-Rom interpolation for f64 scalar values.
pub fn catmull_rom_timed_f64(
    p0: f64,
    p1: f64,
    p2: f64,
    p3: f64,
    dt01: f64,
    dt12: f64,
    dt23: f64,
    t: f64,
) -> f64 {
    let dt01 = dt01.max(1e-6);
    let dt12 = dt12.max(1e-6);
    let dt23 = dt23.max(1e-6);

    let secant01 = (p1 - p0) / dt01;
    let secant12 = (p2 - p1) / dt12;
    let secant23 = (p3 - p2) / dt23;

    let v1 = secant12 * (dt01 / (dt01 + dt12)) + secant01 * (dt12 / (dt01 + dt12));
    let v2 = secant23 * (dt12 / (dt12 + dt23)) + secant12 * (dt23 / (dt12 + dt23));

    let m1 = v1 * dt12;
    let m2 = v2 * dt12;

    hermite_f64(p1, m1, p2, m2, t)
}
