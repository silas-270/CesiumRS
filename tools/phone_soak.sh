#!/usr/bin/env bash
# Sustained-performance soak on the attached device.
#
#   tools/phone_soak.sh [duration_seconds] [interval_seconds]
#   tools/phone_soak.sh              # 10 minutes, sampling every 20s
#   tools/phone_soak.sh 3600 30      # 1 hour, sampling every 30s
#
# Samples an independent window each interval (gfxinfo is reset every time, so each row
# is that window alone, not a running average) and reports the trend. What this is
# looking for is degradation over time: frame times creeping up, memory climbing, the
# SoC throttling as it heats.
#
# REQUIRES the app to be in the foreground and actively rendering a flight — it checks
# this before committing to the full run, because an idle app reports zero frames and
# nonsense percentiles.
set -uo pipefail

PKG="com.example.focusflight"
DURATION_S="${1:-600}"
INTERVAL_S="${2:-20}"
OUT="$(dirname "$0")/soak_$(date +%H%M%S)"
mkdir -p "$OUT"
CSV="$OUT/samples.csv"

echo "t_s,frames,jank_pct,p50_ms,p90_ms,p99_ms,pss_kb,native_kb,temp_c,prime_mhz" > "$CSV"

adb logcat -c 2>/dev/null
adb shell dumpsys gfxinfo "$PKG" reset >/dev/null 2>&1

# Pre-flight: is the app actually rendering? An idle app yields frames=0 and garbage
# percentiles, and there is no point discovering that an hour later.
echo "Checking the app is rendering..."
adb shell dumpsys gfxinfo "$PKG" reset >/dev/null 2>&1
sleep 5
PROBE=$(adb shell dumpsys gfxinfo "$PKG" 2>/dev/null | tr -d '\r' \
        | sed -n 's/^Total frames rendered: \([0-9]*\).*/\1/p' | head -1)
PROBE=${PROBE:-0}
if [ "$PROBE" -lt 10 ]; then
  echo
  echo "ABORT: only $PROBE frames in 5s — the app is not rendering."
  echo "Open Focusflight, start a flight, put it in the view you want measured,"
  echo "leave it in the foreground with the screen on, then re-run this."
  exit 1
fi
echo "OK — $PROBE frames in 5s (~$((PROBE / 5)) fps). Starting."
echo

START=$SECONDS
echo "Soaking for ${DURATION_S}s, sampling every ${INTERVAL_S}s..."
printf "%6s %8s %8s %7s %7s %7s %9s %9s %7s %8s\n" \
  "t(s)" "frames" "jank%" "p50ms" "p90ms" "p99ms" "pss(MB)" "native(MB)" "temp(C)" "prime(MHz)"

while [ $((SECONDS - START)) -lt "$DURATION_S" ]; do
  sleep "$INTERVAL_S"
  T=$((SECONDS - START))

  G=$(adb shell dumpsys gfxinfo "$PKG" 2>/dev/null | tr -d '\r')
  adb shell dumpsys gfxinfo "$PKG" reset >/dev/null 2>&1

  FRAMES=$(sed -n 's/^Total frames rendered: \([0-9]*\).*/\1/p' <<<"$G" | head -1)
  JANK=$(sed -n 's/^Janky frames: [0-9]* (\([0-9.]*\)%).*/\1/p' <<<"$G" | head -1)
  P50=$(sed -n 's/^50th percentile: \([0-9]*\)ms.*/\1/p' <<<"$G" | head -1)
  P90=$(sed -n 's/^90th percentile: \([0-9]*\)ms.*/\1/p' <<<"$G" | head -1)
  P99=$(sed -n 's/^99th percentile: \([0-9]*\)ms.*/\1/p' <<<"$G" | head -1)

  M=$(adb shell dumpsys meminfo "$PKG" 2>/dev/null | tr -d '\r')
  PSS=$(sed -n 's/^ *TOTAL PSS: *\([0-9]*\).*/\1/p' <<<"$M" | head -1)
  [ -z "${PSS:-}" ] && PSS=$(awk '/^ *TOTAL +[0-9]/{print $2; exit}' <<<"$M")
  NATIVE=$(awk '/Native Heap/{print $3; exit}' <<<"$M")

  TEMP=$(adb shell dumpsys battery 2>/dev/null | tr -d '\r' | sed -n 's/^ *temperature: \([0-9-]*\).*/\1/p' | head -1)
  PRIME=$(adb shell 'cat /sys/devices/system/cpu/cpu7/cpufreq/scaling_cur_freq 2>/dev/null' | tr -d '\r')

  FRAMES=${FRAMES:-0}; JANK=${JANK:-0}; P50=${P50:-0}; P90=${P90:-0}; P99=${P99:-0}
  PSS=${PSS:-0}; NATIVE=${NATIVE:-0}; TEMP=${TEMP:-0}; PRIME=${PRIME:-0}

  TEMP_C=$(awk "BEGIN{printf \"%.1f\", $TEMP/10}")
  PRIME_MHZ=$(awk "BEGIN{printf \"%.0f\", $PRIME/1000}")
  PSS_MB=$(awk "BEGIN{printf \"%.0f\", $PSS/1024}")
  NAT_MB=$(awk "BEGIN{printf \"%.0f\", $NATIVE/1024}")

  echo "$T,$FRAMES,$JANK,$P50,$P90,$P99,$PSS,$NATIVE,$TEMP_C,$PRIME_MHZ" >> "$CSV"
  printf "%6s %8s %8s %7s %7s %7s %9s %9s %7s %8s\n" \
    "$T" "$FRAMES" "$JANK" "$P50" "$P90" "$P99" "$PSS_MB" "$NAT_MB" "$TEMP_C" "$PRIME_MHZ"
done

adb logcat -d > "$OUT/logcat.txt" 2>/dev/null

echo
echo "════════════════════ SUMMARY ════════════════════"
python3 - "$CSV" <<'PY'
import csv, sys, statistics as st
rows = list(csv.DictReader(open(sys.argv[1])))
rows = [r for r in rows if int(r["frames"]) > 0]
if len(rows) < 4:
    print(f"Only {len(rows)} usable samples — was the app rendering?")
    sys.exit()
n = max(1, len(rows) // 3)
first, last = rows[:n], rows[-n:]
def avg(rs, k): return sum(float(r[k]) for r in rs) / len(rs)

print(f"{len(rows)} samples over {rows[-1]['t_s']}s\n")
print(f"{'metric':<16}{'first third':>14}{'last third':>14}{'change':>12}")
for key, label, unit in [
    ("p50_ms",  "frame p50",  " ms"),
    ("p90_ms",  "frame p90",  " ms"),
    ("p99_ms",  "frame p99",  " ms"),
    ("jank_pct","jank",       " %"),
    ("pss_kb",  "total PSS",  " MB"),
    ("native_kb","native heap"," MB"),
    ("temp_c",  "battery temp"," C"),
    ("prime_mhz","prime clock"," MHz"),
]:
    a, b = avg(first, key), avg(last, key)
    if unit == " MB": a, b = a/1024, b/1024
    delta = b - a
    pct = (delta / a * 100) if a else 0
    print(f"{label:<16}{a:>11.1f}{unit}{b:>11.1f}{unit}{pct:>+10.1f}%")

window = float(rows[1]["t_s"]) - float(rows[0]["t_s"]) if len(rows) > 1 else 20.0
fps = [float(r["frames"]) / window for r in rows]
print(f"\nframes/s        first {st.mean(fps[:n]):.1f}   last {st.mean(fps[-n:]):.1f}   overall {st.mean(fps):.1f}")
PY
echo "═════════════════════════════════════════════════"
echo
echo "Crashes / panics:"
grep -aiE "panic|FATAL EXCEPTION|UnsatisfiedLink|OutOfMemory" "$OUT/logcat.txt" | tail -5 || true
echo "Artifacts: $OUT"
