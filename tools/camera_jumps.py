#!/usr/bin/env python3
"""Find discontinuities in camera_trace.csv.

A frame is a jump in a column when its change is much larger than the changes of
the frames around it (and above a small absolute floor). Usage:

    python3 tools/camera_jumps.py [camera_trace.csv] [--top N]
"""
import csv
import math
import statistics
import sys

# column -> (absolute floor for a jump, is an angle in degrees)
COLUMNS = {
    "pos_m": (0.5, False),
    "heading": (0.5, True),
    "pitch": (0.5, True),
    "roll": (0.5, True),
    "orbit_yaw": (0.5, True),
    "orbit_pitch": (0.5, True),
    "orbit_dist_m": (0.5, False),
    "agl_m": (1.0, False),
    "ground_cam_m": (1.0, False),
    "ground_anchor_m": (1.0, False),
}
RATIO = 5.0
WINDOW = 15


def wrap(d):
    return (d + 180.0) % 360.0 - 180.0


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    path = args[0] if args else "camera_trace.csv"
    top = 25
    if "--top" in sys.argv:
        top = int(sys.argv[sys.argv.index("--top") + 1])

    with open(path) as f:
        rows = list(csv.DictReader(f))
    if len(rows) < 3:
        print("trace too short")
        return
    for r in rows:
        for k, v in r.items():
            if k != "mode":
                try:
                    r[k] = float(v)
                except ValueError:
                    r[k] = float("nan")

    n = len(rows)
    deltas = {c: [0.0] * n for c in COLUMNS}
    for i in range(1, n):
        a, b = rows[i - 1], rows[i]
        deltas["pos_m"][i] = math.dist((a["x_m"], a["y_m"], a["z_m"]), (b["x_m"], b["y_m"], b["z_m"]))
        for c, (_, is_angle) in COLUMNS.items():
            if c == "pos_m":
                continue
            d = b[c] - a[c]
            if is_angle:
                d = wrap(d)
            deltas[c][i] = abs(d) if not math.isnan(d) else 0.0

    jumps = []  # (ratio, frame index, column)
    per_col = {c: 0 for c in COLUMNS}
    for c, (floor, _) in COLUMNS.items():
        d = deltas[c]
        for i in range(1, n):
            if rows[i]["mode"] != rows[i - 1]["mode"] or d[i] < floor:
                continue
            # agl/ground switch from "no data" to data when the first height tile lands.
            if c in ("agl_m", "ground_cam_m") and math.isnan(rows[i - 1]["ground_cam_m"]) != math.isnan(rows[i]["ground_cam_m"]):
                continue
            nb = [d[j] for j in range(max(1, i - WINDOW), min(n, i + WINDOW + 1)) if j != i]
            base = statistics.median(nb) if nb else 0.0
            ratio = d[i] / max(base, floor / RATIO)
            if ratio >= RATIO:
                jumps.append((ratio, i, c))
                per_col[c] += 1

    dur = (rows[-1]["t_ms"] - rows[0]["t_ms"]) / 1000.0
    clamped = sum(1 for r in rows if r["collision_move_m"] > 0.01)
    print(f"{path}: {n} frames, {dur:.1f} s, collision pass moved the camera in {clamped} frames")
    print("\njumps per column:")
    for c, k in per_col.items():
        print(f"  {c:<16} {k}")

    by_frame = {}
    for ratio, i, c in jumps:
        by_frame.setdefault(i, []).append((c, ratio))

    print(f"\nworst {top} jump frames (all columns that jumped in that frame):")
    ranked = sorted(by_frame.items(), key=lambda kv: -max(r for _, r in kv[1]))[:top]
    for i, cols in sorted(ranked):
        r, p = rows[i], rows[i - 1]
        desc = ", ".join(
            f"{c} {deltas[c][i]:.2f} (x{ratio:.0f})" for c, ratio in sorted(cols, key=lambda x: -x[1])
        )
        print(
            f"  frame {int(r['frame'])} t={r['t_ms'] / 1000:.2f}s {r['mode']} "
            f"agl {p['agl_m']:.1f}->{r['agl_m']:.1f} m  ground_cam {p['ground_cam_m']:.1f}->{r['ground_cam_m']:.1f}  "
            f"collision_move {r['collision_move_m']:.2f} m | {desc}"
        )


if __name__ == "__main__":
    main()
