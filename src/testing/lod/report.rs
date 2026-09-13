//! Reporting for the LOD harness.
//!
//! Same shape as [`super::super::culling::report`]: a per-pose CSV and a per-tile
//! CSV into the temp dir, never the repo, plus a human-readable summary.

use std::collections::BTreeMap;
use std::io::Write;
use std::path::PathBuf;

use super::sweep::{PoseResult, TEXTURE_SIZE_PX};

/// Directory all LOD-harness artefacts are written to.
pub fn output_dir() -> PathBuf {
    let dir = std::env::temp_dir().join("cesium_lod_harness");
    let _ = std::fs::create_dir_all(&dir);
    dir
}

/// One row per pose: the cost side of the trade (`docs/pre-terrain-plan.md` WP1 —
/// "also record the cost side").
pub fn write_poses_csv(name: &str, results: &[PoseResult]) -> PathBuf {
    let path = output_dir().join(format!("{name}_poses.csv"));
    let mut file = match std::fs::File::create(&path) {
        Ok(f) => f,
        Err(e) => {
            eprintln!("lod harness: could not write {}: {e}", path.display());
            return path;
        }
    };
    let _ = writeln!(
        file,
        "sweep,mode,lat_deg,lon_deg,alt_m,pitch_deg,yaw_deg,roll_deg,width,height,\
         tile_count,texture_bytes,deepest_zoom,degenerate,offscreen_tiles,behind_eye_tiles"
    );
    for r in results {
        let p = &r.params;
        let offscreen = r.tiles.iter().filter(|t| !t.has_screen_area()).count();
        let behind_eye = r.tiles.iter().filter(|t| t.behind_eye).count();
        let _ = writeln!(
            file,
            "{},{},{:.6},{:.6},{:.3},{:.3},{:.3},{:.3},{},{},{},{},{},{},{},{}",
            p.sweep,
            p.mode_name(),
            p.lat_deg,
            p.lon_deg,
            p.alt_m,
            p.pitch_deg,
            p.yaw_deg,
            p.roll_deg,
            p.width,
            p.height,
            r.tile_count,
            r.texture_bytes,
            r.deepest_zoom,
            r.degenerate,
            offscreen,
            behind_eye,
        );
    }
    path
}

/// One row per visible tile. This is where the ratio distribution actually lives —
/// the poses CSV only has the cost side.
pub fn write_tiles_csv(name: &str, results: &[PoseResult]) -> PathBuf {
    let path = output_dir().join(format!("{name}_tiles.csv"));
    let mut file = match std::fs::File::create(&path) {
        Ok(f) => f,
        Err(e) => {
            eprintln!("lod harness: could not write {}: {e}", path.display());
            return path;
        }
    };
    let _ = writeln!(
        file,
        "sweep,mode,lat_deg,lon_deg,alt_m,pitch_deg,width,height,\
         tile_z,tile_x,tile_y,screen_px,texels,ratio,partly_offscreen,behind_eye"
    );
    for r in results {
        let p = &r.params;
        for t in &r.tiles {
            let _ = writeln!(
                file,
                "{},{},{:.6},{:.6},{:.3},{:.3},{},{},{},{},{},{:.3},{:.1},{:.6},{},{}",
                p.sweep,
                p.mode_name(),
                p.lat_deg,
                p.lon_deg,
                p.alt_m,
                p.pitch_deg,
                p.width,
                p.height,
                t.id.z,
                t.id.x,
                t.id.y,
                t.screen_px,
                t.texels,
                t.ratio,
                t.partly_offscreen,
                t.behind_eye,
            );
        }
    }
    path
}

/// One tile plus enough of its pose to find it again.
#[derive(Clone, Debug)]
pub struct WorstTile {
    pub sweep: &'static str,
    pub mode: &'static str,
    pub lat_deg: f64,
    pub lon_deg: f64,
    pub alt_m: f64,
    pub tile_z: u8,
    pub tile_x: u32,
    pub tile_y: u32,
    pub screen_px: f64,
    pub ratio: f64,
}

/// Count and the aggregate ratio for one zoom level.
///
/// `aggregate_ratio` is `sum(texels) / sum(screen_px)` over every tile at this
/// zoom, **not** a mean of per-tile ratios — the culling harness's rule applies
/// here too (`report::summarize`'s doc comment): sum raw quantities and divide
/// once, so a handful of sliver tiles at the frustum edge cannot swing the number
/// the way averaging per-tile rates would.
#[derive(Clone, Debug, Default)]
pub struct ZoomBand {
    pub tile_count: usize,
    pub sum_screen_px: f64,
    pub sum_texels: f64,
}

impl ZoomBand {
    pub fn aggregate_ratio(&self) -> f64 {
        if self.sum_screen_px > 0.0 {
            self.sum_texels / self.sum_screen_px
        } else {
            f64::INFINITY
        }
    }
}

/// Aggregate numbers for a whole sweep.
pub struct Summary {
    pub poses: usize,
    pub degenerate_poses: usize,
    pub total_tiles: usize,
    pub total_texture_bytes: u64,
    pub max_deepest_zoom: u8,
    pub offscreen_tiles: usize,
    pub behind_eye_tiles: usize,
    /// Every finite-ratio tile's ratio, pooled across every pose — the population
    /// [`mean`], [`median`] and [`percentile`] operate on. Pooled, not
    /// averaged-per-pose: a pose with 400 tiles and a pose with 4 contribute in
    /// proportion to their tile counts, not equally.
    ratios: Vec<f64>,
    pub aggregate_ratio: f64,
    pub worst_under_refined: Option<WorstTile>,
    pub worst_over_refined: Option<WorstTile>,
    pub by_zoom: BTreeMap<u8, ZoomBand>,
}

fn percentile(sorted: &[f64], p: f64) -> f64 {
    if sorted.is_empty() {
        return f64::NAN;
    }
    let rank = (p * (sorted.len() - 1) as f64).round() as usize;
    sorted[rank.min(sorted.len() - 1)]
}

impl Summary {
    pub fn mean(&self) -> f64 {
        if self.ratios.is_empty() {
            f64::NAN
        } else {
            self.ratios.iter().sum::<f64>() / self.ratios.len() as f64
        }
    }

    pub fn median(&self) -> f64 {
        percentile(&self.ratios, 0.5)
    }

    pub fn p5(&self) -> f64 {
        percentile(&self.ratios, 0.05)
    }

    pub fn p95(&self) -> f64 {
        percentile(&self.ratios, 0.95)
    }

    pub fn sampled_tiles(&self) -> usize {
        self.ratios.len()
    }
}

pub fn summarize(results: &[PoseResult]) -> Summary {
    let mut ratios = Vec::new();
    let mut sum_screen_px = 0.0_f64;
    let mut sum_texels = 0.0_f64;
    let mut offscreen_tiles = 0;
    let mut behind_eye_tiles = 0;
    let mut total_tiles = 0;
    let mut total_texture_bytes = 0_u64;
    let mut max_deepest_zoom = 0_u8;
    let mut degenerate_poses = 0;
    let mut by_zoom: BTreeMap<u8, ZoomBand> = BTreeMap::new();
    let mut worst_under_refined: Option<WorstTile> = None;
    let mut worst_over_refined: Option<WorstTile> = None;

    for r in results {
        if r.degenerate {
            degenerate_poses += 1;
        }
        total_tiles += r.tile_count;
        total_texture_bytes += r.texture_bytes;
        max_deepest_zoom = max_deepest_zoom.max(r.deepest_zoom);

        for t in &r.tiles {
            let band = by_zoom.entry(t.zoom).or_default();
            band.tile_count += 1;

            if !t.has_screen_area() {
                offscreen_tiles += 1;
                if t.behind_eye {
                    behind_eye_tiles += 1;
                }
                continue;
            }
            if t.behind_eye {
                // Partial coverage: some of the patch's samples were behind the
                // eye but at least one quad still projected. Counted, but its
                // ratio still enters the pools below — the area it did contribute
                // is real, just an under-count of the tile's true extent.
                behind_eye_tiles += 1;
            }

            ratios.push(t.ratio);
            sum_screen_px += t.screen_px;
            sum_texels += t.texels;
            band.sum_screen_px += t.screen_px;
            band.sum_texels += t.texels;

            let p = &r.params;
            let as_worst = |t: &super::sweep::TileMetric| WorstTile {
                sweep: p.sweep,
                mode: p.mode_name(),
                lat_deg: p.lat_deg,
                lon_deg: p.lon_deg,
                alt_m: p.alt_m,
                tile_z: t.id.z,
                tile_x: t.id.x,
                tile_y: t.id.y,
                screen_px: t.screen_px,
                ratio: t.ratio,
            };
            if worst_under_refined
                .as_ref()
                .map(|w| t.ratio < w.ratio)
                .unwrap_or(true)
            {
                worst_under_refined = Some(as_worst(t));
            }
            if worst_over_refined
                .as_ref()
                .map(|w| t.ratio > w.ratio)
                .unwrap_or(true)
            {
                worst_over_refined = Some(as_worst(t));
            }
        }
    }

    ratios.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let aggregate_ratio = if sum_screen_px > 0.0 {
        sum_texels / sum_screen_px
    } else {
        f64::INFINITY
    };

    Summary {
        poses: results.len(),
        degenerate_poses,
        total_tiles,
        total_texture_bytes,
        max_deepest_zoom,
        offscreen_tiles,
        behind_eye_tiles,
        ratios,
        aggregate_ratio,
        worst_under_refined,
        worst_over_refined,
        by_zoom,
    }
}

pub fn render_report(
    name: &str,
    poses_csv: &std::path::Path,
    tiles_csv: &std::path::Path,
    results: &[PoseResult],
) -> String {
    let s = summarize(results);
    let mut out = String::new();

    out.push_str(&format!("\n=== lod harness: {name} ===\n"));
    out.push_str(&format!("poses CSV: {}\n", poses_csv.display()));
    out.push_str(&format!("tiles CSV: {}\n", tiles_csv.display()));
    out.push_str(&format!(
        "poses: {}  |  degenerate (no visible tile): {}  |  texture_size: {}x{}\n",
        s.poses, s.degenerate_poses, TEXTURE_SIZE_PX, TEXTURE_SIZE_PX,
    ));
    out.push_str(&format!(
        "tiles: {}  |  sampled (finite ratio): {}  |  offscreen/clipped-away: {}  |  \
         touching behind-eye samples: {}  |  texture bytes: {:.1} MiB  |  deepest zoom: {}\n",
        s.total_tiles,
        s.sampled_tiles(),
        s.offscreen_tiles,
        s.behind_eye_tiles,
        s.total_texture_bytes as f64 / (1024.0 * 1024.0),
        s.max_deepest_zoom,
    ));
    out.push_str(&format!(
        "ratio (texels/screen_px, want ~1.0): aggregate {:.3}  |  mean {:.3}  |  \
         median {:.3}  |  p5 {:.3}  |  p95 {:.3}\n",
        s.aggregate_ratio,
        s.mean(),
        s.median(),
        s.p5(),
        s.p95(),
    ));

    if (s.mean() / s.median()).abs() > 5.0 {
        out.push_str(&format!(
            "NOTE: mean ({:.1}) is far from median ({:.3}) — the arithmetic mean of a \
             texels/screen_px ratio is dominated by near-zero-area sliver tiles (the \
             culling FP tolerance lets a handful of barely-on-screen tiles through), whose \
             ratio blows up as screen_px -> 0. `aggregate_ratio` and the median are the \
             representative summaries; `mean` and `p95` are reported because WP1 asks for \
             them, but read them next to `worst_over_refined` before trusting them.\n",
            s.mean(),
            s.median(),
        ));
    }

    if let Some(w) = &s.worst_under_refined {
        out.push_str(&format!(
            "worst under-refined (blurry, ratio << 1): z={:<2} x={:<6} y={:<6} ratio={:.4} \
             screen_px={:.1}  [{} mode={} lat={:.3} lon={:.3} alt={:.0}m]\n",
            w.tile_z, w.tile_x, w.tile_y, w.ratio, w.screen_px, w.sweep, w.mode, w.lat_deg, w.lon_deg, w.alt_m
        ));
    }
    if let Some(w) = &s.worst_over_refined {
        out.push_str(&format!(
            "worst over-refined (wasteful, ratio >> 1): z={:<2} x={:<6} y={:<6} ratio={:.4} \
             screen_px={:.4}  [{} mode={} lat={:.3} lon={:.3} alt={:.0}m]\n",
            w.tile_z, w.tile_x, w.tile_y, w.ratio, w.screen_px, w.sweep, w.mode, w.lat_deg, w.lon_deg, w.alt_m
        ));
    }

    out.push_str("\n-- per-zoom-level aggregate ratio --\n");
    for (z, band) in &s.by_zoom {
        out.push_str(&format!(
            "  z={:<3} tiles={:<7} aggregate_ratio={:.4}\n",
            z,
            band.tile_count,
            band.aggregate_ratio(),
        ));
    }

    out.push('\n');
    out
}

/// Writes both CSVs, prints the report, returns it so a test can put it in a panic
/// message too.
pub fn emit(name: &str, results: &[PoseResult]) -> String {
    let poses_csv = write_poses_csv(name, results);
    let tiles_csv = write_tiles_csv(name, results);
    let report = render_report(name, &poses_csv, &tiles_csv, results);
    println!("{report}");
    report
}
