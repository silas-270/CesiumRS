use glam::Vec3;

#[derive(Clone, Copy, Debug)]
pub struct OrientedBoundingBox {
    pub center: Vec3,
    pub half_axes: [Vec3; 3],
}

pub struct Frustum {
    pub planes: [(Vec3, f32); 6],
}

impl Frustum {
    pub fn from_planes(planes: [(glam::DVec3, f64); 6]) -> Self {
        let mut f32_planes = [(Vec3::ZERO, 0.0); 6];
        for i in 0..6 {
            f32_planes[i] = (
                Vec3::new(
                    planes[i].0.x as f32,
                    planes[i].0.y as f32,
                    planes[i].0.z as f32,
                ),
                planes[i].1 as f32,
            );
        }
        Self { planes: f32_planes }
    }

    pub fn contains_point(&self, p: Vec3) -> bool {
        for (normal, distance) in &self.planes {
            if normal.dot(p) + *distance < 0.0 {
                return false;
            }
        }
        true
    }

    pub fn intersects_obb(&self, obb: &OrientedBoundingBox) -> bool {
        for (n, d) in &self.planes {
            let r = n.dot(obb.half_axes[0]).abs()
                + n.dot(obb.half_axes[1]).abs()
                + n.dot(obb.half_axes[2]).abs();
            if n.dot(obb.center) + d < -r {
                return false;
            }
        }
        true
    }
}

pub(super) fn get_tile_corner(lon_deg: f32, lat_deg: f32, alt: f32) -> Vec3 {
    let a = 6.378137_f32;
    let b = 6.356_752_4_f32;

    let phi = lat_deg.to_radians();
    let theta = lon_deg.to_radians();

    let x = a * phi.cos() * theta.cos();
    let y = b * phi.sin();
    let z = -a * phi.cos() * theta.sin();

    let pos = Vec3::new(x, y, z);
    if alt == 0.0 {
        pos
    } else {
        pos + pos.normalize() * alt
    }
}

pub(super) fn transform_to_scaled_space(p: Vec3) -> Vec3 {
    let a = 6.378137_f32;
    let b = 6.356_752_4_f32;
    Vec3::new(p.x / a, p.y / b, p.z / a)
}

pub(super) fn compute_horizon_culling_point(
    direction_to_point: Vec3,
    corners: &[Vec3],
) -> Option<Vec3> {
    if direction_to_point.length_squared() < 0.000001 {
        return None;
    }
    let scaled_dir = transform_to_scaled_space(direction_to_point).normalize();

    let mut max_magnitude = 0.0_f32;
    for &p in corners {
        let scaled_pos = transform_to_scaled_space(p);
        let mut mag_sq = scaled_pos.length_squared();
        let mut mag = mag_sq.sqrt();

        let dir = if mag > 0.000001 {
            scaled_pos / mag
        } else {
            Vec3::ZERO
        };

        // For the purpose of this computation, points below the ellipsoid are considered to be on it instead.
        mag_sq = mag_sq.max(1.0);
        mag = mag.max(1.0);

        let cos_alpha = dir.dot(scaled_dir);
        let cross = dir.cross(scaled_dir);
        let sin_alpha = cross.length();

        let cos_beta = 1.0 / mag;
        let sin_beta = (mag_sq - 1.0).max(0.0).sqrt() * cos_beta;

        let denom = cos_alpha * cos_beta - sin_alpha * sin_beta;
        if denom <= 0.0 {
            // all points should face the same direction, but this one doesn't
            return None;
        }

        let candidate = 1.0 / denom;
        max_magnitude = max_magnitude.max(candidate);
    }

    if max_magnitude <= 0.0 || max_magnitude.is_nan() || max_magnitude.is_infinite() {
        None
    } else {
        Some(scaled_dir * max_magnitude)
    }
}
