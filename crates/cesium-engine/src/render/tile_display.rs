use crate::globe::quadtree::TileId;
use std::time::Instant;

pub struct TileBuffers {
    pub vertex_buffer: wgpu::Buffer,
    pub index_buffer: wgpu::Buffer,
    pub num_indices: u32,
    pub center_f64: [f64; 3],
}

#[repr(C)]
#[derive(Debug, Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
pub struct TilePushConstants {
    pub relative_center: [f32; 4], // Padding to 16 bytes included
    pub uv_scale_offset: [f32; 4],
}

/// Stable per-tile texture assignment. Once set, this is only changed
/// under controlled conditions (sibling-complete upgrade, or timeout).
#[derive(Clone)]
pub struct TileDisplayEntry {
    /// Which texture tile is actually sampled (may be an ancestor for fallback).
    pub texture_id: TileId,
    /// UV scale/offset to sample the correct region of `texture_id`.
    pub uv_scale_offset: [f32; 4],
    /// When this tile first became visible (used for timeout-based sibling upgrade).
    pub first_seen: Instant,
    /// True if this tile is currently showing its own hi-res texture (not a fallback).
    pub showing_own_texture: bool,
    /// Fix 2: set to Some(now) on the first frame the tile leaves the visible set.
    /// The entry is only evicted from display_state once this has been Some for ≥200 ms,
    /// giving transient LOD oscillations time to resolve without a texture blink.
    pub absent_since: Option<Instant>,
}
