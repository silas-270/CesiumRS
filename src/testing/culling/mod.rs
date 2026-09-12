//! Visibility-culling correctness harness.
//!
//! This module is a **measuring instrument**, not a fix. It quantifies false
//! positives and false negatives in the engine's visibility culling across a wide
//! sweep of camera angles, positions, altitudes and zoom levels. Where the engine
//! is genuinely wrong, the harness records the number; it never works around it.
//!
//! Layout (one concern per file, per `AGENTS.md`):
//!
//! | file | concern |
//! |------|---------|
//! | [`geodesy`] | ellipsoid + Web-Mercator math, tile addressing |
//! | [`oracle`]  | the independent f64 "is this point visible" ground truth |
//! | [`sat`]     | exact convex-hull intersection ground truth (Layer 1) |
//! | [`cameras`] | deterministic camera construction + seeded RNG |
//! | [`cells`]   | parameter-cell generation (the sweep axes) |
//! | [`sweep`]   | the per-cell measurement run and the FP/FN definitions |
//! | [`report`]  | CSV (into the temp dir) and the human-readable failure map |
//!
//! Test cases live in `test_*.rs` and contain no measurement logic of their own.
//!
//! ## Running it
//!
//! ```text
//! # regression guards — all green, this is the gate
//! cargo test --release --lib culling:: -- --test-threads=1 --nocapture
//!
//! # the update-latency benchmark (measures, does not assert)
//! cargo test --release --lib culling::bench -- --ignored --nocapture
//! ```
//!
//! The harness originally shipped five `#[ignore]`d **defect probes** — tests that
//! were red by design, each naming a real culling defect with its measured number,
//! as a to-do list rather than a weakened threshold. All five were closed by the
//! rework in `docs/culling-math.md` and are now ordinary guards, running in the
//! default gate. That is why the gate went from 1.9 s to ~37 s: it now includes the
//! 128-cell limb band and the 100 000-cell fuzz sweep, 1.7 G oracle classifications
//! between them. They are the two most informative things here and they are not
//! optional. [`bench_update`] is the only `#[ignore]`d test left.
//!
//! Per-cell CSVs land in `$TMPDIR/cesium_culling_harness/`, never in the repo,
//! along with a `<sweep>_false_negatives.csv` holding one row per miss. On failure
//! each sweep prints a map: worst cells, plus false negatives clustered by
//! latitude band, altitude decade, limb angle and deepest zoom reached.
//!
//! ### Threading
//!
//! Every parallel section runs inside [`sweep::harness_pool`] — the harness's own
//! rayon pool, not the global one — so its total width stays bounded no matter how
//! many sweeps libtest decides to run at once. `--test-threads=1` is nevertheless
//! **recommended**: it gives each sweep the whole pool in turn, keeps the printed
//! per-sweep timings meaningful, and keeps the reports from interleaving. Set
//! `CESIUM_CULLING_THREADS` to override the pool width for A/B timing runs.
//!
//! ### Build profile
//!
//! `--release` matters more than core count here: this is dense f64 matrix and
//! trig work, and the repo defines no `[profile.test]` opt-level override, so a
//! plain `cargo test` builds it all at `opt-level = 0`. Measured on a 128-core
//! box:
//!
//! | run | debug | release |
//! |-----|-------|---------|
//! | the gate, before the rework (~40 M visible samples) | 4.6 s | 1.9 s |
//! | the gate, now (~1.8 G visible samples) | ~2 min 30 s | ~37 s |
//!
//! The heaviest probe — the 100 000-cell fuzz sweep, 1.08 billion oracle
//! classifications — takes 27 s of wall clock and 47 minutes of CPU in release,
//! i.e. it keeps ~59 of the 128 cores genuinely busy end to end. The full debug
//! probe run burns 229 minutes of CPU in 2.5 minutes of wall clock (~92x).
//!
//! `[profile.test] opt-level = 2` is now set in the workspace `Cargo.toml`, which
//! gives most of that ~8x without `--release`; `--release` is still recommended for
//! the heavy sweeps.
//!
//! ## Units
//!
//! Everything is in **megameters** (1 unit = 1000 km) and in the engine's Y-up,
//! negated-Z ECEF frame. Altitudes passed to the geodesy helpers are in metres,
//! matching `globe::geometry::lon_lat_alt_to_ecef_f64`.

#[cfg(test)]
pub mod cameras;
#[cfg(test)]
pub mod cells;
#[cfg(test)]
pub mod geodesy;
#[cfg(test)]
pub mod oracle;
#[cfg(test)]
pub mod report;
#[cfg(test)]
pub mod sat;
#[cfg(test)]
pub mod sweep;

#[cfg(test)]
pub mod bench_update;
#[cfg(test)]
pub mod test_analytic_planes;
#[cfg(test)]
pub mod test_globe_sweep;
#[cfg(test)]
pub mod test_label_culling;
