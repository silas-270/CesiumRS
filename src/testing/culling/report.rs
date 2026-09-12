//! Reporting for the culling harness.
//!
//! Two outputs:
//!
//! * A per-cell CSV, written into `std::env::temp_dir()/cesium_culling_harness/`.
//!   **Never into the repository working directory** — several of the tests this
//!   harness replaces dropped `fuzz_results.csv` and friends into the repo root.
//!
//! * A human-readable summary printed on failure: the worst cells, and the false
//!   negatives clustered by latitude band, altitude decade and zoom level, so the
//!   next debugging session starts with a map instead of a wall of numbers.

use std::collections::BTreeMap;
use std::io::Write;
use std::path::PathBuf;

use super::sweep::{self, CellResult};

/// Directory all harness artefacts are written to.
pub fn output_dir() -> PathBuf {
    let dir = std::env::temp_dir().join("cesium_culling_harness");
    let _ = std::fs::create_dir_all(&dir);
    dir
}

/// Writes one CSV row per parameter cell. Returns the path for the report header.
pub fn write_csv(name: &str, results: &[CellResult]) -> PathBuf {
    let path = output_dir().join(format!("{name}.csv"));
    let mut file = match std::fs::File::create(&path) {
        Ok(f) => f,
        Err(e) => {
            eprintln!("culling harness: could not write {}: {e}", path.display());
            return path;
        }
    };

    let limb_cols: Vec<String> = sweep::LIMB_BUCKET_EDGES_DEG
        .iter()
        .map(|e| format!("fn_limb_le_{}", e.to_string().replace('.', "p")))
        .collect();

    let _ = writeln!(
        file,
        "sweep,mode,lat_deg,lon_deg,alt_m,pitch_deg,yaw_deg,roll_deg,width,height,\
         tiles,min_z,max_z,samples_considered,samples_visible,samples_marginal,\
         false_negatives,fn_rate,false_positive_tiles,fp_rate,marginal_tiles,{}",
        limb_cols.join(",")
    );

    for r in results {
        let p = &r.params;
        let limb: Vec<String> = r.fn_limb_buckets.iter().map(|v| v.to_string()).collect();
        let _ = writeln!(
            file,
            "{},{},{:.6},{:.6},{:.3},{:.3},{:.3},{:.3},{},{},{},{},{},{},{},{},{},{:.6},{},{:.6},{},{}",
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
            r.tiles,
            r.min_z,
            r.max_z,
            r.samples_considered,
            r.samples_visible,
            r.samples_marginal,
            r.false_negatives,
            r.fn_rate(),
            r.false_positive_tiles,
            r.fp_rate(),
            r.marginal_tiles,
            limb.join(","),
        );
    }

    path
}

/// Writes every retained false-negative record, one row each.
///
/// Per-cell aggregates say *how bad*; this file says *exactly where*, which is
/// what a fix session needs. Skipped entirely when a sweep is clean.
pub fn write_false_negative_csv(name: &str, results: &[CellResult]) -> Option<PathBuf> {
    if results.iter().all(|r| r.fn_records.is_empty()) {
        return None;
    }
    let path = output_dir().join(format!("{name}_false_negatives.csv"));
    let mut file = match std::fs::File::create(&path) {
        Ok(f) => f,
        Err(e) => {
            eprintln!("culling harness: could not write {}: {e}", path.display());
            return None;
        }
    };
    let _ = writeln!(
        file,
        "sweep,mode,cam_lat_deg,cam_lon_deg,cam_alt_m,pitch_deg,yaw_deg,roll_deg,width,height,\
         max_z,source,fn_lat_deg,fn_lon_deg,limb_deg,facing_cos,ndc_x,ndc_y"
    );
    for r in results {
        let p = &r.params;
        for f in &r.fn_records {
            let _ = writeln!(
                file,
                "{},{},{:.6},{:.6},{:.3},{:.3},{:.3},{:.3},{},{},{},{},{:.6},{:.6},{:.6},{:.9},{:.6},{:.6}",
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
                r.max_z,
                f.source,
                f.lat,
                f.lon,
                f.limb_deg(),
                f.facing_cos,
                f.ndc_x,
                f.ndc_y,
            );
        }
    }
    Some(path)
}

/// Aggregate numbers for a whole sweep.
pub struct Summary {
    pub cells: usize,
    pub cells_with_fn: usize,
    pub total_visible_samples: usize,
    pub total_false_negatives: usize,
    pub total_tiles: usize,
    pub total_false_positive_tiles: usize,
    pub worst_fn_rate: f64,
    pub worst_fp_rate: f64,
}

pub fn summarize(results: &[CellResult]) -> Summary {
    let mut s = Summary {
        cells: results.len(),
        cells_with_fn: 0,
        total_visible_samples: 0,
        total_false_negatives: 0,
        total_tiles: 0,
        total_false_positive_tiles: 0,
        worst_fn_rate: 0.0,
        worst_fp_rate: 0.0,
    };
    for r in results {
        if r.false_negatives > 0 {
            s.cells_with_fn += 1;
        }
        s.total_visible_samples += r.samples_visible;
        s.total_false_negatives += r.false_negatives;
        s.total_tiles += r.tiles;
        s.total_false_positive_tiles += r.false_positive_tiles;
        s.worst_fn_rate = s.worst_fn_rate.max(r.fn_rate());
        s.worst_fp_rate = s.worst_fp_rate.max(r.fp_rate());
    }
    s
}

/// The human-readable failure map. Returned as a string so tests can put it in the
/// panic message (where `cargo test` will actually show it) as well as stdout.
pub fn render_report(name: &str, csv: &std::path::Path, results: &[CellResult]) -> String {
    let s = summarize(results);
    let mut out = String::new();

    out.push_str(&format!("\n=== culling harness: {name} ===\n"));
    out.push_str(&format!("CSV: {}\n", csv.display()));
    out.push_str(&format!(
        "cells: {}  |  cells with FN: {}  |  visible samples: {}  |  FN: {} ({:.4}%)\n",
        s.cells,
        s.cells_with_fn,
        s.total_visible_samples,
        s.total_false_negatives,
        100.0 * pct(s.total_false_negatives, s.total_visible_samples),
    ));
    out.push_str(&format!(
        "tiles: {}  |  FP tiles: {} ({:.2}%)  |  worst cell FN rate: {:.4}%  |  worst cell FP rate: {:.2}%\n",
        s.total_tiles,
        s.total_false_positive_tiles,
        100.0 * pct(s.total_false_positive_tiles, s.total_tiles),
        100.0 * s.worst_fn_rate,
        100.0 * s.worst_fp_rate,
    ));

    // ── Worst cells by FN rate ───────────────────────────────────────────────
    let mut by_fn: Vec<&CellResult> = results.iter().filter(|r| r.false_negatives > 0).collect();
    by_fn.sort_by(|a, b| b.fn_rate().partial_cmp(&a.fn_rate()).unwrap());
    if !by_fn.is_empty() {
        out.push_str("\n-- worst cells by false-negative rate --\n");
        for r in by_fn.iter().take(12) {
            let p = &r.params;
            out.push_str(&format!(
                "  {:<26} mode={:<8} lat={:>8.3} lon={:>9.3} alt={:>12.1}m \
                 pitch={:>7.2} yaw={:>7.2} roll={:>7.2} {}x{}  \
                 FN={}/{} ({:.3}%) tiles={} z={}..{}\n",
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
                r.false_negatives,
                r.samples_visible,
                100.0 * r.fn_rate(),
                r.tiles,
                r.min_z,
                r.max_z,
            ));
            for ex in r.fn_records.iter().take(3) {
                out.push_str(&format!(
                    "        e.g. lat={:>8.4} lon={:>9.4} src={:<9} limb={:.4}deg ndc=({:+.3},{:+.3})\n",
                    ex.lat, ex.lon, ex.source, ex.limb_deg(), ex.ndc_x, ex.ndc_y
                ));
            }
        }
    }

    // ── Clusters ─────────────────────────────────────────────────────────────
    let mut by_lat: BTreeMap<i64, (usize, usize)> = BTreeMap::new();
    let mut by_alt: BTreeMap<i64, (usize, usize)> = BTreeMap::new();
    let mut by_zoom: BTreeMap<u8, (usize, usize)> = BTreeMap::new();
    for r in results {
        let lat_band = ((r.params.lat_deg / 15.0).floor() as i64) * 15;
        let alt_decade = if r.params.alt_m > 0.0 {
            r.params.alt_m.log10().floor() as i64
        } else {
            -99
        };
        let e = by_lat.entry(lat_band).or_insert((0, 0));
        e.0 += r.false_negatives;
        e.1 += r.samples_visible;
        let e = by_alt.entry(alt_decade).or_insert((0, 0));
        e.0 += r.false_negatives;
        e.1 += r.samples_visible;
        let e = by_zoom.entry(r.max_z).or_insert((0, 0));
        e.0 += r.false_negatives;
        e.1 += r.samples_visible;
    }

    if s.total_false_negatives > 0 {
        out.push_str("\n-- FN clusters by latitude band (15deg) --\n");
        for (band, (fneg, vis)) in &by_lat {
            if *fneg > 0 {
                out.push_str(&format!(
                    "  [{:>4}..{:>4})  FN={:>7}  of {:>9} visible ({:.3}%)\n",
                    band,
                    band + 15,
                    fneg,
                    vis,
                    100.0 * pct(*fneg, *vis)
                ));
            }
        }
        out.push_str("\n-- FN clusters by altitude decade (log10 metres) --\n");
        for (dec, (fneg, vis)) in &by_alt {
            if *fneg > 0 {
                out.push_str(&format!(
                    "  1e{:<3}m      FN={:>7}  of {:>9} visible ({:.3}%)\n",
                    dec,
                    fneg,
                    vis,
                    100.0 * pct(*fneg, *vis)
                ));
            }
        }
        out.push_str("\n-- FN clusters by distance inside the visible limb --\n");
        let mut limb = [0usize; sweep::LIMB_BUCKET_EDGES_DEG.len()];
        for r in results {
            for (i, v) in r.fn_limb_buckets.iter().enumerate() {
                limb[i] += v;
            }
        }
        for (i, edge) in sweep::LIMB_BUCKET_EDGES_DEG.iter().enumerate() {
            if limb[i] > 0 {
                out.push_str(&format!("  <= {edge:>5.2} deg  FN={:>7}\n", limb[i]));
            }
        }

        out.push_str("\n-- FN clusters by deepest zoom reached --\n");
        for (z, (fneg, vis)) in &by_zoom {
            if *fneg > 0 {
                out.push_str(&format!(
                    "  z={:<3}        FN={:>7}  of {:>9} visible ({:.3}%)\n",
                    z,
                    fneg,
                    vis,
                    100.0 * pct(*fneg, *vis)
                ));
            }
        }
    }

    out.push('\n');
    out
}

fn pct(num: usize, den: usize) -> f64 {
    if den == 0 {
        0.0
    } else {
        num as f64 / den as f64
    }
}

/// Convenience used by every sweep test: write both CSVs, print the report,
/// return it so the test can put it in the panic message too.
pub fn emit(name: &str, results: &[CellResult]) -> String {
    let csv = write_csv(name, results);
    let fn_csv = write_false_negative_csv(name, results);
    let mut report = render_report(name, &csv, results);
    if let Some(p) = fn_csv {
        report.push_str(&format!("per-false-negative CSV: {}\n", p.display()));
    }
    if results.iter().any(|r| r.fn_records_truncated) {
        report.push_str(&format!(
            "NOTE: at least one cell exceeded {} retained false-negative records; \
             the per-FN CSV is truncated (aggregate counts are not).\n",
            sweep::MAX_FN_RECORDS_PER_CELL
        ));
    }
    println!("{report}");
    report
}
