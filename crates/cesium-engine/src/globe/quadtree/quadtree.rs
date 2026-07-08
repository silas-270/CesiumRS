use glam::Vec3;
use super::tile_id::{TileId, MAX_ZOOM, web_mercator_y_to_lat};
use super::bounding_volume::{OrientedBoundingBox, Frustum, compute_horizon_culling_point, get_tile_corner};

pub struct QuadtreeNode {
    pub id: TileId,
    pub center: Vec3,
    pub radius: f32,
    pub lod_radius: f32,
    pub obb: OrientedBoundingBox,
    pub tight_obbs: Option<Box<Vec<OrientedBoundingBox>>>,
    pub surface_points: [Vec3; 9],
    pub horizon_culling_point: Option<Vec3>,
    pub visible: bool,
    pub children: Option<Box<[QuadtreeNode; 4]>>,
}

impl QuadtreeNode {
    pub fn new(id: TileId) -> Self {
        let (center, radius, lod_radius, surface_points, obb, tight_obbs) = Self::compute_bounding_volume(&id);
        let geographic_corners = Self::compute_geographic_corners(&id);
        let horizon_culling_point =
            compute_horizon_culling_point(center.normalize(), &geographic_corners);

        QuadtreeNode {
            id,
            center,
            radius,
            lod_radius,
            obb,
            tight_obbs,
            surface_points,
            horizon_culling_point,
            visible: false,
            children: None,
        }
    }

    fn compute_sub_obb(id: &TileId, u_min: f32, u_max: f32, v_min: f32, v_max: f32) -> OrientedBoundingBox {
        let z_pow = (1_u32 << id.z) as f32;
        let base_lon = -180.0 + (id.x as f32) * 360.0 / z_pow;
        let lon_span = 360.0 / z_pow;
        
        let sub_lon_min = base_lon + u_min * lon_span;
        let sub_lon_max = base_lon + u_max * lon_span;
        
        let y_min = id.y as f32 + v_min;
        let y_max = id.y as f32 + v_max;
        
        let mut sub_lat_max = web_mercator_y_to_lat(y_min, id.z);
        let mut sub_lat_min = web_mercator_y_to_lat(y_max, id.z);
        
        if id.y == 0 && v_min == 0.0 {
            sub_lat_max = 90.0;
        }
        if id.y == (1_u32 << id.z) - 1 && v_max == 1.0 {
            sub_lat_min = -90.0;
        }

        let center_lon = (sub_lon_min + sub_lon_max) * 0.5;
        let center_lat = (sub_lat_min + sub_lat_max) * 0.5;
        let surface_center = get_tile_corner(center_lon, center_lat, 0.0);

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

        let steps = 4;
        for i in 0..=steps {
            let u = i as f32 / steps as f32;
            let lon = sub_lon_min + u * (sub_lon_max - sub_lon_min);
            for j in 0..=steps {
                let v = j as f32 / steps as f32;
                let lat = sub_lat_min + v * (sub_lat_max - sub_lat_min);
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

        OrientedBoundingBox {
            center: obb_center,
            half_axes: [east * extents.x, north * extents.y, normal * extents.z],
        }
    }

    fn compute_bounding_volume(id: &TileId) -> (Vec3, f32, f32, [Vec3; 9], OrientedBoundingBox, Option<Box<Vec<OrientedBoundingBox>>>) {
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

        let mut tight_obbs = None;
        if id.z <= 16 {
            let mut obbs = Vec::new();
            let subdivisions = 8;
            for u_idx in 0..subdivisions {
                for v_idx in 0..subdivisions {
                    let u_min = u_idx as f32 / subdivisions as f32;
                    let u_max = (u_idx + 1) as f32 / subdivisions as f32;
                    let v_min = v_idx as f32 / subdivisions as f32;
                    let v_max = (v_idx + 1) as f32 / subdivisions as f32;

                    obbs.push(Self::compute_sub_obb(id, u_min, u_max, v_min, v_max));
                }
            }
            tight_obbs = Some(Box::new(obbs));
        }

        (surface_center, radius, lod_radius, points, obb, tight_obbs)
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

            // Allow culling even if the camera is slightly below the ellipsoid surface (up to ~300km)
            // This ensures horizon culling works when the camera is exactly at altitude 0.
            if vh_mag_sq > -0.1 {
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

        if let Some(obbs) = &self.tight_obbs {
            let mut any_intersect = false;
            for obb in obbs.iter() {
                if frustum.intersects_obb(obb) {
                    let normal = obb.half_axes[2].normalize_or_zero();
                    let cam_to_center = camera_pos - obb.center;
                    
                    let max_extent = obb.half_axes[0].length().max(obb.half_axes[1].length());
                    // Only consider the sub-OBB visible if it's not strictly behind the horizon.
                    // The tangent plane at the center of the sub-OBB is a good approximation.
                    // We allow a margin of `max_extent` to account for the curvature of the sub-OBB.
                    if normal.dot(cam_to_center) > -max_extent {
                        any_intersect = true;
                        break;
                    }
                }
            }
            if !any_intersect {
                self.visible = false;
                self.children = None;
                return;
            }
        }

        self.visible = true;

        let dist = (self.center - camera_pos).length();

        // Hysteresis logic: Subdivide at 1.0x, but don't collapse until 1.2x.
        // A 20% band prevents LOD oscillation when the camera straddles the
        // subdivision threshold. The old 1.05├ù band (Ôëê50 m at z=19) was too
        // narrow and caused rapid APPEAR/DISAPPEAR flicker on high-detail tiles.
        let is_subdivided = self.children.is_some();
        let subdivide_dist = self.lod_radius * lod_factor;
        let collapse_dist = subdivide_dist * 1.20;

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

    pub fn collect_renderable_tiles<F: FnMut(&TileId) -> bool>(
        &self,
        active_tiles: &mut Vec<(TileId, Vec3, f32)>,
        is_ready: &mut F,
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

    pub fn update(&mut self, camera_global_pos: Vec3, frustum_planes: [(glam::DVec3, f64); 6]) {
        let frustum = Frustum::from_planes(frustum_planes);
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

    pub fn get_renderable_tiles<F: FnMut(&TileId) -> bool>(&self, mut is_ready: F) -> Vec<(TileId, Vec3, f32)> {
        let mut active_tiles = Vec::new();
        for root in self.roots.iter() {
            root.collect_renderable_tiles(&mut active_tiles, &mut is_ready);
        }
        active_tiles
    }
}
