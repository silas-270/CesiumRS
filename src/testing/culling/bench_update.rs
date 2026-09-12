//! Timed A/B harness for `QuadtreeManager::update`.
//!
//! Not a correctness test — it prints wall-clock per update and an estimate of the
//! quadtree's resident bytes, so the culling rework can be quoted with real numbers
//! instead of a flop count. `#[ignore]`d because it is a measurement, not a gate.
//!
//! ```text
//! cargo test --release --lib culling::bench -- --ignored --test-threads=1 --nocapture
//! ```
//!
//! Single-threaded on purpose: `update` is called once per frame on the main
//! thread, so per-call latency is the number that matters, not throughput.

use std::time::Instant;

use cesium_engine::globe::quadtree::{QuadtreeManager, QuadtreeNode};

use super::cameras::{build_camera, ViewParams};
use super::cells;

/// Representative poses: the nadir ladder (every altitude decade, several
/// latitudes) plus the zoom-cliff ladder (deepest zoom 11 through 20).
fn bench_cells() -> Vec<ViewParams> {
    let mut v = cells::nadir_ladder();
    v.extend(cells::zoom_cliff_cells());
    v
}

/// Resident bytes of one node, including whatever it hangs off the heap.
fn node_bytes(node: &QuadtreeNode) -> usize {
    let mut total = std::mem::size_of::<QuadtreeNode>();
    total += node
        .tight_obbs
        .as_ref()
        .map(|v| {
            std::mem::size_of::<Vec<cesium_engine::globe::quadtree::OrientedBoundingBox>>()
                + v.capacity()
                    * std::mem::size_of::<cesium_engine::globe::quadtree::OrientedBoundingBox>()
        })
        .unwrap_or(0);
    if let Some(children) = &node.children {
        total += std::mem::size_of::<[QuadtreeNode; 4]>() - 4 * std::mem::size_of::<QuadtreeNode>();
        for c in children.iter() {
            total += node_bytes(c);
        }
    }
    total
}

fn node_count(node: &QuadtreeNode) -> usize {
    1 + node
        .children
        .as_ref()
        .map(|c| c.iter().map(node_count).sum::<usize>())
        .unwrap_or(0)
}

#[test]
#[ignore = "measurement, not a gate: prints QuadtreeManager::update latency and quadtree footprint"]
fn bench_quadtree_update() {
    let cells = bench_cells();
    println!("  {} camera poses", cells.len());

    let mut total_ns = 0u128;
    let mut total_updates = 0u64;
    let mut total_nodes = 0usize;
    let mut total_bytes = 0usize;
    let mut worst_us = 0.0_f64;
    let mut worst_ctx = String::new();

    for p in &cells {
        let cam = build_camera(p);
        let aspect = p.aspect();
        let planes = cam.calculate_frustum_planes(aspect as f32);
        let (cam_pos_d, _) = cam.global_transform_f64();
        let cam_pos = glam::Vec3::new(cam_pos_d.x as f32, cam_pos_d.y as f32, cam_pos_d.z as f32);

        let mut qt = QuadtreeManager::new();
        // Warm-up: build the tree and settle the LOD hysteresis band, so the timed
        // loop measures the steady-state per-frame cost rather than construction.
        for _ in 0..8 {
            qt.update(cam_pos, planes);
        }

        const ITERS: u32 = 200;
        let t0 = Instant::now();
        for _ in 0..ITERS {
            qt.update(cam_pos, planes);
        }
        let elapsed = t0.elapsed();

        let per_update_us = elapsed.as_secs_f64() * 1.0e6 / ITERS as f64;
        if per_update_us > worst_us {
            worst_us = per_update_us;
            worst_ctx = format!(
                "lat={:.1} alt={:.0}m pitch={:.0} mode={}",
                p.lat_deg,
                p.alt_m,
                p.pitch_deg,
                p.mode_name()
            );
        }

        total_ns += elapsed.as_nanos();
        total_updates += ITERS as u64;

        let nodes: usize = qt.roots.iter().map(node_count).sum();
        let bytes: usize = qt.roots.iter().map(node_bytes).sum();
        total_nodes += nodes;
        total_bytes += bytes;
    }

    let mean_us = total_ns as f64 / 1000.0 / total_updates as f64;
    println!("  mean  QuadtreeManager::update : {mean_us:.1} us");
    println!("  worst QuadtreeManager::update : {worst_us:.1} us  [{worst_ctx}]");
    println!(
        "  quadtree footprint            : {} nodes, {:.1} kB total, {:.0} B/node",
        total_nodes,
        total_bytes as f64 / 1024.0,
        total_bytes as f64 / total_nodes.max(1) as f64
    );
    println!(
        "  size_of::<QuadtreeNode>()     : {} B",
        std::mem::size_of::<QuadtreeNode>()
    );
}
