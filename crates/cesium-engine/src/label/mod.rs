pub mod culling;

use bytemuck::{Pod, Zeroable};
use glam::{Vec3, Quat};
use crate::globe::quadtree::Frustum;

#[repr(C, align(4))]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
pub struct PackedLabel {
    pub ecef_pos: [f32; 3],
    pub ecef_normal: [f32; 3],
    pub name_offset: u32,
    pub name_len: u16,
    pub scale_rank: u8,
    pub label_rank: u8,
}

pub struct LabelDatabase {
    pub labels: Vec<PackedLabel>,
    pub string_table: &'static [u8],
    pub lod_offsets: [u32; 16],
}

impl LabelDatabase {
    pub fn load(bytes: &'static [u8]) -> Self {
        // 1. Verify Magic
        assert!(bytes.len() >= 76, "Label binary file is truncated");
        assert_eq!(&bytes[0..4], b"CLBL", "Invalid magic bytes in label binary");
        
        // 2. Parse header
        let version = u32::from_le_bytes(bytes[4..8].try_into().unwrap());
        assert_eq!(version, 1, "Unsupported label file version");
        
        let count = u32::from_le_bytes(bytes[8..12].try_into().unwrap()) as usize;
        
        let mut lod_offsets = [0u32; 16];
        for i in 0..16 {
            let start = 12 + i * 4;
            lod_offsets[i] = u32::from_le_bytes(bytes[start..start+4].try_into().unwrap());
        }
        
        // 3. Copy struct bytes into Vec to guarantee alignment
        let struct_bytes_start = 76;
        let struct_bytes_len = count * std::mem::size_of::<PackedLabel>();
        let struct_bytes_end = struct_bytes_start + struct_bytes_len;
        
        assert!(bytes.len() >= struct_bytes_end, "Label binary struct section is truncated");
        let struct_bytes = &bytes[struct_bytes_start..struct_bytes_end];
        
        let mut labels = vec![PackedLabel::zeroed(); count];
        let labels_byte_slice: &mut [u8] = bytemuck::cast_slice_mut(&mut labels);
        labels_byte_slice.copy_from_slice(struct_bytes);
        
        // 4. String table
        let string_table = &bytes[struct_bytes_end..];
        
        Self {
            labels,
            string_table,
            lod_offsets,
        }
    }

    /// O(1) filtering: returns all labels up to a given scale rank zoom level.
    pub fn get_labels_for_zoom(&self, zoom: usize) -> &[PackedLabel] {
        let max_idx = self.lod_offsets[zoom.min(15)] as usize;
        &self.labels[0..max_idx]
    }

    /// Resolves the name of the label from the string table
    pub fn get_name(&self, label: &PackedLabel) -> &'static str {
        let start = label.name_offset as usize;
        let end = start + label.name_len as usize;
        if end <= self.string_table.len() {
            let slice: &'static [u8] = &self.string_table[start..end];
            std::str::from_utf8(slice).unwrap_or("")
        } else {
            ""
        }
    }
}

pub struct VisibleLabel {
    pub name: &'static str,
    pub ecef_pos: Vec3,
    pub scale_rank: u8,
    pub label_rank: u8,
}

pub struct LabelManager {
    db: LabelDatabase,
    pub visible_labels: Vec<VisibleLabel>,
    last_update_pos: Vec3,
    last_update_ori: Quat,
    frame_accum: usize,
}

impl LabelManager {
    pub fn new() -> Self {
        // Load the compiled-in binary file
        let bytes = include_bytes!("populated_places.bin");
        let db = LabelDatabase::load(bytes);
        
        Self {
            db,
            visible_labels: Vec::new(),
            last_update_pos: Vec3::ZERO,
            last_update_ori: Quat::IDENTITY,
            frame_accum: 9999, // Force immediate update on first frame
        }
    }

    /// Updates the visible label cache based on camera position, orientation, and frustum planes.
    pub fn update(&mut self, camera_pos: Vec3, camera_ori: Quat, current_zoom: usize, frustum: &Frustum) {
        self.frame_accum += 1;
        
        let pos_dist = (camera_pos - self.last_update_pos).length_squared();
        let ori_diff = 1.0 - self.last_update_ori.dot(camera_ori).abs();
        
        // Skip updates if camera has not moved much and we are under the frame threshold (e.g. 6 frames = ~10Hz)
        if self.frame_accum < 6 && pos_dist < 0.0001 && ori_diff < 0.001 {
            return;
        }
        
        self.frame_accum = 0;
        self.last_update_pos = camera_pos;
        self.last_update_ori = camera_ori;
        
        self.visible_labels.clear();
        
        // Precompute camera unit-sphere scaling factors
        let a = 6.378137_f32;
        let b = 6.356_752_4_f32;
        let cv = Vec3::new(camera_pos.x / a, camera_pos.y / b, camera_pos.z / a);
        let vh_mag_sq = cv.length_squared() - 1.0;
        
        // Step 1: O(1) LOD selection based on zoom
        let candidate_labels = self.db.get_labels_for_zoom(current_zoom);
        
        // Step 2: Culling loop
        for label in candidate_labels {
            let label_pos = Vec3::new(label.ecef_pos[0], label.ecef_pos[1], label.ecef_pos[2]);
            
            // Check horizon culling first (Branchless math, rejects ~50% of labels quickly)
            if culling::is_behind_horizon(cv, vh_mag_sq, label_pos) {
                continue;
            }
            
            // Check frustum culling (Branchy loop, only run for labels in front of the horizon)
            if !culling::is_in_frustum(frustum, label_pos) {
                continue;
            }
            
            // If it passes both, resolve name and store it
            let name = self.db.get_name(label);
            self.visible_labels.push(VisibleLabel {
                name,
                ecef_pos: label_pos,
                scale_rank: label.scale_rank,
                label_rank: label.label_rank,
            });
        }
    }
}
