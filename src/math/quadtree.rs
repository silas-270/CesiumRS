use glam::{Vec3, Mat4, Vec4};
use crate::math::geometry::lon_lat_to_ecef;

const MAX_ZOOM: u8 = 20;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct TileId {
    pub z: u8,
    pub x: u32,
    pub y: u32,
}

pub struct QuadtreeNode {
    pub id: TileId,
    pub center: Vec3,
    pub radius: f32,
    pub visible: bool,
    pub children: Option<Box<[QuadtreeNode; 4]>>,
}

pub struct Frustum {
    pub planes: [(Vec3, f32); 6],
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
                extract(row2),        // Near (wgpu Z is 0 to 1)
                extract(row3 - row2), // Far
            ]
        }
    }

    pub fn intersects_sphere(&self, center: Vec3, radius: f32) -> bool {
        for (n, d) in &self.planes {
            if n.dot(center) + d < -radius {
                return false;
            }
        }
        true
    }
}


impl QuadtreeNode {
    pub fn new(id: TileId) -> Self {
        let (center, radius) = Self::compute_bounding_volume(&id);
        Self {
            id,
            center,
            radius,
            visible: false,
            children: None,
        }
    }

    fn compute_bounding_volume(id: &TileId) -> (Vec3, f32) {
        let z_pow_x = (1_u32 << (id.z + 1)) as f32; // 2^(z+1) for longitude
        let z_pow_y = (1_u32 << id.z) as f32;       // 2^z for latitude

        // Longitude spans -180 to 180 over 2^(z+1) tiles
        let lon_min = -180.0 + (id.x as f32) * 360.0 / z_pow_x;
        let lon_max = -180.0 + ((id.x + 1) as f32) * 360.0 / z_pow_x;

        // Latitude spans 90 to -90 over 2^z tiles (Y=0 is North)
        let lat_max = 90.0 - (id.y as f32) * 180.0 / z_pow_y;
        let lat_min = 90.0 - ((id.y + 1) as f32) * 180.0 / z_pow_y;

        let center_lon = (lon_min + lon_max) * 0.5;
        let center_lat = (lat_min + lat_max) * 0.5;

        let center = lon_lat_to_ecef(center_lon, center_lat);
        
        // Use corner to calculate bounding radius
        let corner = lon_lat_to_ecef(lon_min, lat_max);
        let radius = (center - corner).length();

        (center, radius)
    }

    pub fn subdivide(&mut self) {
        let z = self.id.z + 1;
        let x = self.id.x * 2;
        let y = self.id.y * 2;

        self.children = Some(Box::new([
            QuadtreeNode::new(TileId { z, x, y }),         // Top-Left
            QuadtreeNode::new(TileId { z, x: x + 1, y }),     // Top-Right
            QuadtreeNode::new(TileId { z, x, y: y + 1 }),     // Bottom-Left
            QuadtreeNode::new(TileId { z, x: x + 1, y: y + 1 }), // Bottom-Right
        ]));
    }

    pub fn update(&mut self, camera_pos: Vec3, lod_factor: f32, frustum: &Frustum) {
        if !frustum.intersects_sphere(self.center, self.radius) {
            self.visible = false;
            self.children = None; // Drop children to save memory if culled
            return;
        }

        // Horizon Culling
        let camera_dist_sq = camera_pos.length_squared();
        let r_earth = 6.3567523_f32; // Conservative Earth polar radius
        let r_earth_sq = r_earth * r_earth;

        if camera_dist_sq > r_earth_sq {
            let v_to_c = self.center - camera_pos;
            let t_sq = v_to_c.length_squared();
            let t = t_sq.sqrt();
            let camera_dist = camera_dist_sq.sqrt();
            
            if t > self.radius {
                let alpha = (r_earth / camera_dist).asin();
                let beta = (self.radius / t).asin();
                
                let cos_theta = (-camera_pos).dot(v_to_c) / (camera_dist * t);
                let theta = cos_theta.clamp(-1.0, 1.0).acos();
                
                if theta + beta < alpha {
                    let d_h_sq = camera_dist_sq - r_earth_sq;
                    let d_h = d_h_sq.sqrt();
                    
                    let horizon_plane_dist = d_h * (d_h / camera_dist);
                    let node_front_dist = t * cos_theta - self.radius;
                    
                    if node_front_dist > horizon_plane_dist {
                        self.visible = false;
                        self.children = None;
                        return;
                    }
                }
            }
        }

        self.visible = true;

        let dist = (self.center - camera_pos).length();

        // Subdivide condition: closer than radius * lod_factor
        if dist < self.radius * lod_factor && self.id.z < MAX_ZOOM {
            if self.children.is_none() {
                self.subdivide();
            }
            if let Some(children) = &mut self.children {
                for child in children.iter_mut() {
                    child.update(camera_pos, lod_factor, frustum);
                }
            }
        } else {
            // Merge condition: too far away, drop children
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
}

pub struct QuadtreeManager {
    pub roots: [QuadtreeNode; 2],
    pub lod_factor: f32, // Multiplier for subdivision distance check
}

impl QuadtreeManager {
    pub fn new() -> Self {
        Self {
            roots: [
                QuadtreeNode::new(TileId { z: 0, x: 0, y: 0 }), // West Hemisphere
                QuadtreeNode::new(TileId { z: 0, x: 1, y: 0 }), // East Hemisphere
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
}
