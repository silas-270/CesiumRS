#!/usr/bin/env python3
"""Turns a per-subsystem timings report (from benchmark.rs / perf_simulator.rs,
or a device-side dump in the same shape) into a ranked "which part of the frame
budget is this" summary, and optionally diffs two reports for regressions.

Primary input is always the `SubsystemTimings` JSON emitted by the Rust
harnesses — that alone answers "which subsystem, what % of the frame, is it a
spike" without needing any device-specific tooling. A captured Perfetto trace
and/or a memory-sample log are optional, best-effort inputs: this script tries
`trace_processor_shell`/parses the log if given, but a missing tool or file is
a warning, not a failure — see `tools/run_perf_scenario.sh` at the Focusflight
repo root for how these three inputs get produced from an on-device run.

Usage:
    python3 analyze_perf.py --subsystems-json benchmark_report.json
    python3 analyze_perf.py --subsystems-json perf_simulator_report.json --per-mode
    python3 analyze_perf.py --baseline old_report.json --current new_report.json
    python3 analyze_perf.py --subsystems-json report.json --trace cesium_perf.pftrace \\
        --mem-log mem_samples.log --out perf_report.json
"""
import argparse
import json
import shutil
import subprocess
import sys
from pathlib import Path

# Regression threshold: flag a subsystem whose % share of the frame OR whose
# p99 grew by more than this fraction between --baseline and --current.
DEFAULT_REGRESSION_THRESHOLD = 0.15


def load_report(path):
    with open(path) as f:
        return json.load(f)


def is_per_mode_report(report):
    """perf_simulator_report.json has one top-level key per camera mode
    ("Free"/"Cockpit"/"Tracking"), each holding its own `subsystems` block —
    distinct from benchmark_report.json's single flat report."""
    return all(isinstance(v, dict) and "subsystems" in v for v in report.values())


def subsystem_breakdown(report):
    """[(name, average_us, p90_us, p99_us, pct_of_frame)], ranked by % of frame,
    for one flat report (top-level `subsystems` dict plus the coarse buckets)."""
    subsystems = report.get("subsystems", {})
    # Frame budget = the two coarse buckets already measured around the
    # subsystem spans (update_logic_us + render_scene_us), so % shares are
    # comparable across reports even as new subsystem fields get added.
    frame_budget_us = report.get("average_update_logic_us", 0) + report.get(
        "average_render_scene_us", 0
    )
    rows = []
    for name, stats in subsystems.items():
        avg = stats.get("average_us", 0.0)
        pct = (avg / frame_budget_us * 100) if frame_budget_us > 0 else 0.0
        rows.append((name, avg, stats.get("p90_us", 0.0), stats.get("p99_us", 0.0), pct))
    rows.sort(key=lambda r: r[4], reverse=True)
    return rows, frame_budget_us


def print_breakdown(label, report):
    rows, frame_budget_us = subsystem_breakdown(report)
    print(f"\n=== {label} (frame budget ~{frame_budget_us:.0f}us) ===")
    print(f"{'subsystem':<28} {'% of frame':>10} {'avg us':>10} {'p90 us':>10} {'p99 us':>10}")
    for name, avg, p90, p99, pct in rows:
        print(f"{name:<28} {pct:>9.1f}% {avg:>10.1f} {p90:>10.1f} {p99:>10.1f}")


def detect_spikes(report, threshold_multiplier=3.0):
    """Flags any subsystem whose p99 exceeds `threshold_multiplier`x its own
    average — a simple stand-in for "trailing rolling median" when only
    aggregate avg/p90/p99 (not the raw per-frame series) is available."""
    spikes = []
    for name, stats in report.get("subsystems", {}).items():
        avg = stats.get("average_us", 0.0)
        p99 = stats.get("p99_us", 0.0)
        if avg > 0 and p99 > avg * threshold_multiplier:
            spikes.append((name, avg, p99, p99 / avg))
    spikes.sort(key=lambda s: s[3], reverse=True)
    return spikes


def diff_reports(baseline, current, threshold):
    """Regression check: any subsystem whose % of frame budget or p99 grew by
    more than `threshold` (fractional) between the two reports."""
    base_rows, _ = subsystem_breakdown(baseline)
    cur_rows, _ = subsystem_breakdown(current)
    base_by_name = {r[0]: r for r in base_rows}
    cur_by_name = {r[0]: r for r in cur_rows}

    regressions = []
    for name, (_, cur_avg, cur_p90, cur_p99, cur_pct) in cur_by_name.items():
        base = base_by_name.get(name)
        if base is None:
            continue
        _, base_avg, base_p90, base_p99, base_pct = base
        pct_growth = (cur_pct - base_pct) / base_pct if base_pct > 0 else 0.0
        p99_growth = (cur_p99 - base_p99) / base_p99 if base_p99 > 0 else 0.0
        if pct_growth > threshold or p99_growth > threshold:
            regressions.append(
                {
                    "subsystem": name,
                    "baseline_pct": base_pct,
                    "current_pct": cur_pct,
                    "pct_growth": pct_growth,
                    "baseline_p99_us": base_p99,
                    "current_p99_us": cur_p99,
                    "p99_growth": p99_growth,
                }
            )
    regressions.sort(key=lambda r: max(r["pct_growth"], r["p99_growth"]), reverse=True)
    return regressions


def parse_mem_log(path):
    """Parses the `dumpsys meminfo`-polling fallback log from
    tools/run_perf_scenario.sh (lines like `ts=<epoch> TOTAL <kb> TK` /
    `Native Heap <kb> TK`) into a steady-state average and slope (bytes/sec),
    the leak signal for the long-duration scenario. Best-effort: unparseable
    lines are skipped rather than failing the whole run."""
    samples = []  # (timestamp, total_kb)
    ts = None
    for line in Path(path).read_text().splitlines():
        line = line.strip()
        if line.startswith("ts="):
            try:
                ts = float(line.split("=", 1)[1])
            except ValueError:
                ts = None
            continue
        if "TOTAL" in line and ts is not None:
            parts = line.split()
            for i, tok in enumerate(parts):
                if tok == "TOTAL" and i + 1 < len(parts):
                    try:
                        samples.append((ts, int(parts[i + 1])))
                    except ValueError:
                        pass
                    break
    if len(samples) < 2:
        return None
    kb_values = [kb for _, kb in samples]
    avg_kb = sum(kb_values) / len(kb_values)
    t0, kb0 = samples[0]
    t1, kb1 = samples[-1]
    slope_kb_per_sec = (kb1 - kb0) / (t1 - t0) if t1 > t0 else 0.0
    return {
        "sample_count": len(samples),
        "average_total_kb": avg_kb,
        "slope_kb_per_sec": slope_kb_per_sec,
    }


def slice_trace_by_scenario(trace_path):
    """Best-effort: shells out to `trace_processor_shell` (from the Perfetto
    SDK) to list `cesium.scenario.*` instant markers, if the tool is on PATH.
    Returns None (with a warning printed) rather than failing when it isn't —
    the subsystem-JSON analysis above doesn't depend on this."""
    shell = shutil.which("trace_processor_shell")
    if shell is None:
        print(
            "warning: trace_processor_shell not found on PATH — skipping trace slicing "
            "(subsystem-JSON analysis above is unaffected). Install the Perfetto SDK's "
            "trace_processor_shell to enable this.",
            file=sys.stderr,
        )
        return None
    query = (
        "select name, ts from slice where name like 'cesium.scenario.%' order by ts;"
    )
    try:
        result = subprocess.run(
            [shell, "-q", "-", trace_path],
            input=query,
            capture_output=True,
            text=True,
            timeout=60,
        )
    except (OSError, subprocess.TimeoutExpired) as e:
        print(f"warning: trace_processor_shell failed: {e}", file=sys.stderr)
        return None
    if result.returncode != 0:
        print(f"warning: trace_processor_shell exited {result.returncode}: {result.stderr}", file=sys.stderr)
        return None
    return result.stdout


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--subsystems-json", help="benchmark_report.json or perf_simulator_report.json")
    parser.add_argument("--trace", help="Optional .pftrace to slice by cesium.scenario.* markers")
    parser.add_argument("--mem-log", help="Optional dumpsys-meminfo polling log for leak/growth detection")
    parser.add_argument("--out", help="Write the combined JSON report here")
    parser.add_argument("--baseline", help="Regression mode: older subsystems-json report")
    parser.add_argument("--current", help="Regression mode: newer subsystems-json report")
    parser.add_argument(
        "--regression-threshold",
        type=float,
        default=DEFAULT_REGRESSION_THRESHOLD,
        help=f"Fractional growth in %% of frame or p99 to flag as a regression (default {DEFAULT_REGRESSION_THRESHOLD})",
    )
    args = parser.parse_args()

    if args.baseline and args.current:
        baseline = load_report(args.baseline)
        current = load_report(args.current)
        regressions = diff_reports(baseline, current, args.regression_threshold)
        if not regressions:
            print("No regressions found.")
        else:
            print(f"Found {len(regressions)} regressed subsystem(s):")
            for r in regressions:
                print(
                    f"  {r['subsystem']}: {r['baseline_pct']:.1f}% -> {r['current_pct']:.1f}% of frame "
                    f"({r['pct_growth']*100:+.0f}%), p99 {r['baseline_p99_us']:.0f}us -> "
                    f"{r['current_p99_us']:.0f}us ({r['p99_growth']*100:+.0f}%)"
                )
        if args.out:
            with open(args.out, "w") as f:
                json.dump({"regressions": regressions}, f, indent=2)
        return

    if not (args.subsystems_json or args.trace or args.mem_log):
        parser.error("pass --subsystems-json, --trace, and/or --mem-log (or use --baseline/--current)")

    output = {}

    if not args.subsystems_json:
        print(
            "note: no --subsystems-json given — skipping the per-subsystem %% of frame budget "
            "breakdown (see tools/run_perf_scenario.sh's note on dumping it from a device build).",
            file=sys.stderr,
        )
    else:
        report = load_report(args.subsystems_json)
        output.update(_subsystem_output(report, Path(args.subsystems_json).stem))

    if args.mem_log:
        mem = parse_mem_log(args.mem_log)
        if mem is None:
            print(f"\nwarning: could not parse memory samples from {args.mem_log}", file=sys.stderr)
        else:
            print(
                f"\nMemory: avg TOTAL {mem['average_total_kb']:.0f} KB, "
                f"slope {mem['slope_kb_per_sec']:+.2f} KB/s over {mem['sample_count']} samples"
            )
            output["memory"] = mem

    if args.trace:
        trace_slices = slice_trace_by_scenario(args.trace)
        if trace_slices:
            print(f"\nScenario markers in {args.trace}:\n{trace_slices}")
            output["scenario_markers_raw"] = trace_slices

    if args.out:
        with open(args.out, "w") as f:
            json.dump(output, f, indent=2)
        print(f"\nReport written to {args.out}")
    return


def _subsystem_output(report, label):
    """Prints the breakdown for one flat report or, for a perf_simulator.rs-style
    per-mode report, one breakdown per mode. Returns the JSON-serializable dict."""
    if not is_per_mode_report(report):
        print_breakdown(label, report)
        spikes = detect_spikes(report)
        if spikes:
            print("\nSpikes (p99 > 3x avg): " + ", ".join(f"{n} ({m:.1f}x)" for n, _, _, m in spikes))
        rows, frame_budget_us = subsystem_breakdown(report)
        return {
            "frame_budget_us": frame_budget_us,
            "subsystems_by_pct": [
                {"name": n, "average_us": a, "p90_us": p90, "p99_us": p99, "pct_of_frame": pct}
                for n, a, p90, p99, pct in rows
            ],
            "spikes": [{"name": n, "average_us": a, "p99_us": p99, "ratio": r} for n, a, p99, r in spikes],
        }

    output = {}
    for mode_name, mode_report in report.items():
        print_breakdown(mode_name, mode_report)
        spikes = detect_spikes(mode_report)
        if spikes:
            print("  Spikes (p99 > 3x avg): " + ", ".join(f"{n} ({m:.1f}x)" for n, _, _, m in spikes))
        rows, frame_budget_us = subsystem_breakdown(mode_report)
        output[mode_name] = {
            "frame_budget_us": frame_budget_us,
            "subsystems_by_pct": [
                {"name": n, "average_us": a, "p90_us": p90, "p99_us": p99, "pct_of_frame": pct}
                for n, a, p90, p99, pct in rows
            ],
            "spikes": [{"name": n, "average_us": a, "p99_us": p99, "ratio": r} for n, a, p99, r in spikes],
        }
    return output


if __name__ == "__main__":
    main()
