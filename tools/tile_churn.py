#!/usr/bin/env python3
"""Analyse tile_trace.csv: refetches, downgrades, flicker and slow frames.

    python3 tools/tile_churn.py [tile_trace.csv] [--slow MS]
"""
import csv
import sys
from collections import Counter, defaultdict


def parent_chain(t):
    z, x, y = t
    while z > 0:
        z, x, y = z - 1, x // 2, y // 2
        yield (z, x, y)


def pct(v, q):
    v = sorted(v)
    return v[min(int(len(v) * q), len(v) - 1)] if v else 0.0


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    path = args[0] if args else "tile_trace.csv"
    slow_ms = float(sys.argv[sys.argv.index("--slow") + 1]) if "--slow" in sys.argv else 50.0

    rows = [r for r in csv.DictReader(open(path)) if None not in r.values()]
    lap = "start"
    laps = ["start"]
    stats = defaultdict(Counter)          # lap -> counter
    refetch_gaps = defaultdict(list)      # (kind, cause) -> gaps in s
    refetch_z = defaultdict(Counter)      # kind -> z histogram
    state = {}                            # (kind, id) -> dict
    frame_t = {}                          # frame -> t_ms
    frame_info = {}
    frame_lap = {}
    frame_events = defaultdict(Counter)
    draw_del_t = {}
    by_frame_draw = defaultdict(lambda: ([], []))  # frame -> (adds, dels)

    for r in rows:
        f = int(r["frame"])
        t = float(r["t_ms"]) / 1000.0
        kind, ev = r["kind"], r["event"]
        tid = (int(r["z"]), int(r["x"]), int(r["y"])) if r["z"] else None

        if kind == "mark":
            lap = r["info"]
            laps.append(lap)
            continue
        if kind == "frame":
            frame_t[f] = t
            frame_info[f] = r["info"]
            frame_lap[f] = lap
            continue
        frame_events[f][f"{kind}.{ev}"] += 1

        if kind == "draw":
            if ev == "ADD":
                by_frame_draw[f][0].append(tid)
                if tid in draw_del_t and t - draw_del_t[tid] < 2.0:
                    stats[lap]["draw flicker (re-added <2s after removal)"] += 1
            else:
                by_frame_draw[f][1].append(tid)
                draw_del_t[tid] = t
            continue

        s = state.setdefault((kind, tid), {"ready": False, "removed": None, "cancelled": None})
        if ev == "REQ":
            stats[lap][f"{kind} REQ"] += 1
            if s["ready"]:
                cause, when = s["removed"] if s["removed"] else ("rebuild/replace", t)
                stats[lap][f"{kind} REFETCH ({cause})"] += 1
                refetch_gaps[(kind, cause)].append(t - when)
                refetch_z[kind][tid[0]] += 1
            elif s["cancelled"] is not None:
                stats[lap][f"{kind} re-request after cancel"] += 1
            s["removed"] = None
        elif ev == "READY":
            s["ready"] = True
            s["removed"] = None
            stats[lap][f"{kind} READY"] += 1
        elif ev in ("EVICT", "EXPIRE"):
            if s["ready"]:
                s["removed"] = (ev + (":" + r["info"] if r["info"] else ""), t)
            stats[lap][f"{kind} {ev} {r['info']}".strip()] += 1
        elif ev == "CANCEL":
            s["cancelled"] = t
            stats[lap][f"{kind} CANCEL"] += 1
        elif ev == "FAIL":
            stats[lap][f"{kind} FAIL"] += 1

    # Downgrades: a drawn tile removed while one of its ancestors is added in the same frame.
    for f, (adds, dels) in by_frame_draw.items():
        added = set(adds)
        l = frame_lap.get(f, "start")
        for d in dels:
            if any(a in added for a in parent_chain(d)):
                stats[l]["draw DOWNGRADE (tile replaced by ancestor)"] += 1
        dels_set = set(dels)
        for a in adds:
            if any(p in dels_set for p in parent_chain(a)):
                stats[l]["draw upgrade (ancestor replaced by tile)"] += 1

    frames = sorted(frame_t)
    dts = {frames[i]: (frame_t[frames[i]] - frame_t[frames[i - 1]]) * 1000 for i in range(1, len(frames))}

    print(f"{path}: {len(rows)} events, {len(frames)} frames\n")
    for l in dict.fromkeys(laps):
        ds = [dt for f, dt in dts.items() if frame_lap.get(f) == l]
        if not ds and not stats[l]:
            continue
        print(f"== {l}: {len(ds)} frames, dt p50 {pct(ds, .5):.1f} ms  p99 {pct(ds, .99):.1f}  max {max(ds, default=0):.1f}  >{slow_ms:.0f}ms: {sum(d > slow_ms for d in ds)}")
        for k, v in sorted(stats[l].items()):
            print(f"   {k:<50} {v}")

    print("\nrefetches: cause -> count, gap between losing the tile and asking again (s): p50 / max")
    for (kind, cause), g in sorted(refetch_gaps.items()):
        print(f"   {kind:<5} {cause:<28} {len(g):5}   {pct(g, .5):6.1f} / {max(g):6.1f}")
    for kind, h in refetch_z.items():
        print(f"   {kind} refetches by z: " + " ".join(f"z{z}:{n}" for z, n in sorted(h.items())))

    worst = sorted(dts.items(), key=lambda kv: -kv[1])[:15]
    print(f"\nslowest frames (events recorded in that frame):")
    for f, dt in sorted(worst):
        ev = ", ".join(f"{k} {n}" for k, n in frame_events[f].most_common(6))
        print(f"   frame {f} [{frame_lap.get(f)}] dt {dt:7.1f} ms  {frame_info.get(f, '')} | {ev}")


if __name__ == "__main__":
    main()
