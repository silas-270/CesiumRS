use glam::{Mat4, Vec3, Vec4};

pub fn web_mercator_y_to_lat(y: f32, z: u8) -> f32 {
    let n = (1_u32 << z) as f32;
    let phi = (std::f32::consts::PI * (1.0 - 2.0 * y / n)).sinh().atan();
    phi.to_degrees()
}

const MAX_ZOOM: u8 = 20;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct TileId {
    pub z: u8,
    pub x: u32,
    pub y: u32,
}

impl TileId {
    pub fn parent(&self) -> Option<TileId> {
        if self.z == 0 {
            None
        } else {
            Some(TileId {
                z: self.z - 1,
                x: self.x / 2,
                y: self.y / 2,
            })
        }
    }
}

pub struct QuadtreeNode {
    pub id: TileId,
    pub center: Vec3,
    pub radius: f32,
    pub lod_radius: f32,
    pub obb: OrientedBoundingBox,
    pub surface_points: [Vec3; 9],
    pub horizon_culling_point: Option<Vec3>,
    pub visible: bool,
    pub children: Option<Box<[QuadtreeNode; 4]>>,
}

#[derive(Clone, Copy, Debug)]
pub struct OrientedBoundingBox {
    pub center: Vec3,
    pub half_axes: [Vec3; 3],
}

pub struct Frustum {
    pub planes: [(Vec3, f32); 6],
    pub view_proj: Mat4,
}

impl Frustum {
    pub fn from_matrix(m: Mat4) -> Self {
        let row0 = m.row(0);
        let row1 = m.row(1);
        let row2 = m.row(2);
        let row3 = m.row(3);

        let extract = |row: Vec4| -> (Vec3, f32) {
            let n = row.truncate();
            let len = n.length();
            if len > 0.000001 {
                (n / len, row.w / len)
            } else {
                (Vec3::ZERO, 0.0)
            }
        };

        Self {
            planes: [
                extract(row3 + row0), // Left
                extract(row3 - row0), // Right
                extract(row3 + row1), // Bottom
                extract(row3 - row1), // Top
                extract(row3 + row2), // Near (wgpu Z is 0 to 1)
                extract(row3 - row2), // Far
            ],
            view_proj: m,
        }
    }

    pub fn contains_point(&self, p: Vec3) -> bool {
        let p_clip = self.view_proj * p.extend(1.0);
        let w = p_clip.w;
        if w <= 0.0 {
            return false;
        }
        p_clip.x >= -w
            && p_clip.x <= w
            && p_clip.y >= -w
            && p_clip.y <= w
            && p_clip.z >= 0.0
            && p_clip.z <= w
    }

    pub fn intersects_surface_points(&self, points: &[Vec3; 9]) -> bool {
        let epsilon = 0.0001_f32; // Small 100-meter safety margin for floating-point stability
        for (n, d) in &self.planes {
            let mut all_outside = true;
            for p in points {
                if n.dot(*p) + d >= -epsilon {
                    all_outside = false;
                    break;
                }
            }
            if all_outside {
                return false;
            }
        }
        true
    }

    pub fn intersects_sphere(&self, center: Vec3, radius: f32) -> bool {
        for (n, d) in &self.planes {
            if n.dot(center) + d < -radius {
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

fn get_tile_corner(lon_deg: f32, lat_deg: f32, alt: f32) -> Vec3 {
    let a = 6.378137_f32;
    let b = 6.3567523142_f32;

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

fn transform_to_scaled_space(p: Vec3) -> Vec3 {
    let a = 6.378137_f32;
    let b = 6.3567523142_f32;
    Vec3::new(p.x / a, p.y / b, p.z / a)
}

fn compute_horizon_culling_point(direction_to_point: Vec3, corners: &[Vec3]) -> Option<Vec3> {
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

impl QuadtreeNode {
    pub fn new(id: TileId) -> Self {
        let (center, radius, lod_radius, surface_points, obb) = Self::compute_bounding_volume(&id);
        let geographic_corners = Self::compute_geographic_corners(&id);
        let horizon_culling_point =
            compute_horizon_culling_point(center.normalize(), &geographic_corners);

        QuadtreeNode {
            id,
            center,
            radius,
            lod_radius,
            obb,
            surface_points,
            horizon_culling_point,
            visible: false,
            children: None,
        }
    }

    fn compute_bounding_volume(id: &TileId) -> (Vec3, f32, f32, [Vec3; 9], OrientedBoundingBox) {
        let z_pow = (1_u32 << id.z) as f32;

        let lon_min = -180.0 + (id.x as f32) * 360.0 / z_pow;
        let lon_max = -180.0 + ((id.x + 1) as f32) * 360.0 / z_pow;
        let raw_lat_max = web_mercator_y_to_lat(id.y as f32, id.z);
        let raw_lat_min = web_mercator_y_to_lat((id.y + 1) as f32, id.z);

        let mut lat_max = raw_lat_max;
        let mut lat_min = raw_lat_min;

        if id.y == 0 {
            lat_max = 90.0;
        }
        if id.y == (1_u32 << id.z) - 1 {
            lat_min = -90.0;
        }

        let center_lon = (lon_min + lon_max) * 0.5;
        let center_lat = (lat_min + lat_max) * 0.5;

        let surface_center = get_tile_corner(center_lon, center_lat, 0.0);

        let mut points = [Vec3::ZERO; 9];
        let mut idx = 0;
        let lons = [lon_min, center_lon, lon_max];
        let lats = [lat_min, center_lat, lat_max];

        let mut max_dist_sq = 0.0_f32;

        for &lon in &lons {
            for &lat in &lats {
                let p = get_tile_corner(lon, lat, 0.0);
                points[idx] = p;
                idx += 1;

                let dist_sq = (p - surface_center).length_squared();
                max_dist_sq = max_dist_sq.max(dist_sq);
            }
        }

        let radius = max_dist_sq.sqrt();

        // Calculate an LOD radius without the 90-degree stretch
        let raw_center_lat = (raw_lat_min + raw_lat_max) * 0.5;
        let raw_surface_center = get_tile_corner(center_lon, raw_center_lat, 0.0);
        let mut raw_max_dist_sq = 0.0_f32;
        let raw_lats = [raw_lat_min, raw_center_lat, raw_lat_max];
        for &lon in &lons {
            for &lat in &raw_lats {
                let p = get_tile_corner(lon, lat, 0.0);
                let dist_sq = (p - raw_surface_center).length_squared();
                raw_max_dist_sq = raw_max_dist_sq.max(dist_sq);
            }
        }
        let lod_radius = raw_max_dist_sq.sqrt();

        // Compute OrientedBoundingBox using a dense sample grid
        let a2 = 6.378137_f32 * 6.378137_f32;
        let b2 = 6.3567523142_f32 * 6.3567523142_f32;
        let normal = Vec3::new(
            surface_center.x / a2,
            surface_center.y / b2,
            surface_center.z / a2,
        )
        .normalize();

        let mut east = Vec3::new(0.0, 1.0, 0.0).cross(normal).normalize_or_zero();
        if east.length_squared() < 0.1 {
            east = Vec3::new(1.0, 0.0, 0.0);
        }
        let north = normal.cross(east).normalize();

        let mut min_ext = Vec3::new(f32::MAX, f32::MAX, f32::MAX);
        let mut max_ext = Vec3::new(f32::MIN, f32::MIN, f32::MIN);

        // Sample a dense grid to accurately capture the curved surface shape.
        // For large tiles (z<=4), the curvature causes OBB bloat.
        let steps = if id.z < 5 { 8 } else { 2 };
        for i in 0..=steps {
            let u = i as f32 / steps as f32;
            let lon = lon_min + u * (lon_max - lon_min);
            for j in 0..=steps {
                let v = j as f32 / steps as f32;
                let lat = lat_min + v * (lat_max - lat_min);
                let p = get_tile_corner(lon, lat, 0.0);
                let rel = p - surface_center;
                let x = rel.dot(east);
                let y = rel.dot(north);
                let z = rel.dot(normal);
                min_ext = min_ext.min(Vec3::new(x, y, z));
                max_ext = max_ext.max(Vec3::new(x, y, z));
            }
        }

        let offset = (max_ext + min_ext) * 0.5;
        let obb_center = surface_center + east * offset.x + north * offset.y + normal * offset.z;
        let extents = (max_ext - min_ext) * 0.5;

        let obb = OrientedBoundingBox {
            center: obb_center,
            half_axes: [east * extents.x, north * extents.y, normal * extents.z],
        };

        (surface_center, radius, lod_radius, points, obb)
    }

    fn compute_geographic_corners(id: &TileId) -> [Vec3; 4] {
        let z_pow = (1_u32 << id.z) as f32;

        let lon_min = -180.0 + (id.x as f32) * 360.0 / z_pow;
        let lon_max = -180.0 + ((id.x + 1) as f32) * 360.0 / z_pow;
        let mut lat_max = web_mercator_y_to_lat(id.y as f32, id.z);
        let mut lat_min = web_mercator_y_to_lat((id.y + 1) as f32, id.z);

        if id.y == 0 {
            lat_max = 90.0;
        }
        if id.y == (1_u32 << id.z) - 1 {
            lat_min = -90.0;
        }

        [
            get_tile_corner(lon_min, lat_min, 0.0),
            get_tile_corner(lon_max, lat_min, 0.0),
            get_tile_corner(lon_max, lat_max, 0.0),
            get_tile_corner(lon_min, lat_max, 0.0),
        ]
    }

    pub fn subdivide(&mut self) {
        let z = self.id.z + 1;
        let x = self.id.x * 2;
        let y = self.id.y * 2;

        self.children = Some(Box::new([
            QuadtreeNode::new(TileId { z, x, y }),        // Top-Left
            QuadtreeNode::new(TileId { z, x: x + 1, y }), // Top-Right
            QuadtreeNode::new(TileId { z, x, y: y + 1 }), // Bottom-Left
            QuadtreeNode::new(TileId {
                z,
                x: x + 1,
                y: y + 1,
            }), // Bottom-Right
        ]));
    }

    pub fn update(&mut self, camera_pos: Vec3, lod_factor: f32, frustum: &Frustum) {
        if let Some(hcp) = self.horizon_culling_point {
            let a = 6.378137_f32;
            let b = 6.3567523142_f32;
            let cv = Vec3::new(camera_pos.x / a, camera_pos.y / b, camera_pos.z / a);
            let vh_mag_sq = cv.length_squared() - 1.0;

            // Only cull if the camera is strictly outside the ellipsoid
            if vh_mag_sq > 0.0 {
                let vt = hcp - cv;
                let vt_dot_vc = -vt.dot(cv);

                let is_occluded = vt_dot_vc > vh_mag_sq
                    && (vt_dot_vc * vt_dot_vc) / vt.length_squared() > vh_mag_sq;

                if is_occluded {
                    self.visible = false;
                    self.children = None;
                    return;
                }
            }
        }

        if !frustum.intersects_obb(&self.obb) {
            self.visible = false;
            self.children = None;
            return;
        }

        // Surface-point frustum test: check if any of the tile's surface
        // actually projects inside the camera's clip-space volume.
        // Skip this test when the camera is close to the tile — at close range
        // the frustum is too narrow for grid-based sampling to reliably hit it,
        // but the tile is obviously visible since the camera is on top of it.
        {
            let cam_to_tile = (self.center - camera_pos).length();
            if cam_to_tile > self.radius * 1.5 {
                let a2 = 6.378137_f32 * 6.378137_f32;
                let b2 = 6.3567523142_f32 * 6.3567523142_f32;

                let mut any_visible = false;
                for p in &self.surface_points {
                    if frustum.contains_point(*p) {
                        let normal = Vec3::new(p.x / a2, p.y / b2, p.z / a2).normalize();
                        if normal.dot(camera_pos - *p) > 0.0 {
                            any_visible = true;
                            break;
                        }
                    }
                }

                if !any_visible {
                    // Dense fallback: generate a denser grid from the tile's geographic bounds
                    let z_pow = (1_u32 << self.id.z) as f32;
                    let lon_min = -180.0 + (self.id.x as f32) * 360.0 / z_pow;
                    let lon_max = -180.0 + ((self.id.x + 1) as f32) * 360.0 / z_pow;
                    let mut lat_max = web_mercator_y_to_lat(self.id.y as f32, self.id.z);
                    let mut lat_min = web_mercator_y_to_lat((self.id.y + 1) as f32, self.id.z);
                    if self.id.y == 0 {
                        lat_max = 90.0;
                    }
                    if self.id.y == (1_u32 << self.id.z) - 1 {
                        lat_min = -90.0;
                    }

                    let mut found = false;
                    let steps = 3;
                    for i in 0..=steps {
                        if found {
                            break;
                        }
                        let u = i as f32 / steps as f32;
                        let lon = lon_min + u * (lon_max - lon_min);
                        for j in 0..=steps {
                            let v = j as f32 / steps as f32;
                            let lat = lat_min + v * (lat_max - lat_min);
                            let p = get_tile_corner(lon, lat, 0.0);
                            if frustum.contains_point(p) {
                                let normal = Vec3::new(p.x / a2, p.y / b2, p.z / a2).normalize();
                                if normal.dot(camera_pos - p) > 0.0 {
                                    found = true;
                                    break;
                                }
                            }
                        }
                    }
                    if !found {
                        self.visible = false;
                        self.children = None;
                        return;
                    }
                }
            }
        }

        self.visible = true;

        let dist = (self.center - camera_pos).length();

        // Hysteresis logic: Subdivide at 1.0x, but don't collapse until 1.05x
        let is_subdivided = self.children.is_some();
        let subdivide_dist = self.lod_radius * lod_factor;
        let collapse_dist = subdivide_dist * 1.05;

        let should_be_subdivided = if is_subdivided {
            dist < collapse_dist
        } else {
            dist < subdivide_dist
        };

        // Subdivide condition
        if should_be_subdivided && self.id.z < MAX_ZOOM {
            if self.children.is_none() {
                self.subdivide();
            }
            if let Some(children) = &mut self.children {
                for child in children.iter_mut() {
                    child.update(camera_pos, lod_factor, frustum);
                }
            }
        } else {
            self.children = None;
        }
    }

    pub fn collect_visible_tiles(&self, active_tiles: &mut Vec<(TileId, Vec3, f32)>) {
        if !self.visible {
            return;
        }
        if let Some(children) = &self.children {
            for child in children.iter() {
                child.collect_visible_tiles(active_tiles);
            }
        } else {
            active_tiles.push((self.id, self.center, self.radius));
        }
    }

    pub fn collect_renderable_tiles<F: Fn(&TileId) -> bool>(
        &self,
        active_tiles: &mut Vec<(TileId, Vec3, f32)>,
        is_ready: &F,
    ) -> bool {
        if !self.visible {
            return true;
        }

        if let Some(children) = &self.children {
            let mut children_ready = true;
            let mut child_tiles = Vec::new();
            for child in children.iter() {
                if !child.collect_renderable_tiles(&mut child_tiles, is_ready) {
                    children_ready = false;
                    break;
                }
            }

            if children_ready {
                active_tiles.extend(child_tiles);
                return true;
            }
        }

        active_tiles.push((self.id, self.center, self.radius));
        is_ready(&self.id)
    }
}

pub struct QuadtreeManager {
    pub roots: [QuadtreeNode; 4],
    pub lod_factor: f32, // Multiplier for subdivision distance check
}

impl QuadtreeManager {
    pub fn new() -> Self {
        Self {
            roots: [
                QuadtreeNode::new(TileId { z: 1, x: 0, y: 0 }), // NW
                QuadtreeNode::new(TileId { z: 1, x: 1, y: 0 }), // NE
                QuadtreeNode::new(TileId { z: 1, x: 0, y: 1 }), // SW
                QuadtreeNode::new(TileId { z: 1, x: 1, y: 1 }), // SE
            ],
            lod_factor: 2.0, // Default LOD tuning parameter
        }
    }

    pub fn update(&mut self, camera_global_pos: Vec3, view_proj: Mat4) {
        let frustum = Frustum::from_matrix(view_proj);
        for root in self.roots.iter_mut() {
            root.update(camera_global_pos, self.lod_factor, &frustum);
        }
    }

    pub fn get_visible_tiles(&self) -> Vec<(TileId, Vec3, f32)> {
        let mut active_tiles = Vec::new();
        for root in self.roots.iter() {
            root.collect_visible_tiles(&mut active_tiles);
        }
        active_tiles
    }

    pub fn get_renderable_tiles<F: Fn(&TileId) -> bool>(&self, is_ready: F) -> Vec<(TileId, Vec3, f32)> {
        let mut active_tiles = Vec::new();
        for root in self.roots.iter() {
            root.collect_renderable_tiles(&mut active_tiles, &is_ready);
        }
        active_tiles
    }
}
