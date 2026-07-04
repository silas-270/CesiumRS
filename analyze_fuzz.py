import csv
from collections import defaultdict
import math

results_path = r"C:\Users\kamme\Desktop\CesiumRS\fuzz_results.csv"

cases = []
total_fp = 0
total_fn = 0
total_active = 0
total_rendered = 0

with open(results_path, 'r', encoding='utf-8') as f:
    reader = csv.DictReader(f)
    for row in reader:
        rendered = int(row['rendered'])
        if rendered > 0:
            cases.append({
                'id': int(row['case_id']),
                'lat': float(row['lat']),
                'lon': float(row['lon']),
                'alt': float(row['alt']),
                'pitch': float(row['pitch']),
                'yaw': float(row['yaw']),
                'roll': float(row['roll']),
                'rendered': rendered,
                'hit': int(row['hit']),
                'fp': int(row['false_positives']),
                'fn': int(row['false_negatives'])
            })
            total_active += 1
            total_fp += int(row['false_positives'])
            total_fn += int(row['false_negatives'])
            total_rendered += rendered

print(f"--- 10,000 TEST ANALYSIS SUMMARY ---")
print(f"Total Active Cases (Earth Visible): {total_active}")
print(f"Total False Negatives (Missing Tiles): {total_fn}")
print(f"Total False Positives (Ghost Tiles): {total_fp}")
print(f"Average False Positives per Active Case: {total_fp / total_active:.2f}")

cases.sort(key=lambda c: c['fp'], reverse=True)

worst_10 = cases[:10]
print("\n--- TOP 10 WORST FALSE POSITIVE CONFIGURATIONS ---")
for c in worst_10:
    print(f"ID {c['id']}: FP {c['fp']} (Rendered: {c['rendered']}, Hit: {c['hit']}) | Pos: lat={c['lat']:.1f}, lon={c['lon']:.1f}, alt={c['alt']:.2f} | Rot: pitch={c['pitch']:.1f}, yaw={c['yaw']:.1f}, roll={c['roll']:.1f}")

# Group by altitude ranges to see where it happens most
alt_bins = defaultdict(list)
for c in cases:
    bin_key = math.floor(c['alt']) # 0-1, 1-2, etc.
    alt_bins[bin_key].append(c['fp'])

print("\n--- FALSE POSITIVES BY ALTITUDE ---")
for alt in sorted(alt_bins.keys()):
    avg = sum(alt_bins[alt]) / len(alt_bins[alt])
    print(f"Alt {alt} to {alt+1}: Avg {avg:.2f} FPs (Max: {max(alt_bins[alt])}, Samples: {len(alt_bins[alt])})")

# Group by extreme pitch/roll
print("\n--- FALSE POSITIVES BY EXTREME ROLL ---")
roll_bins = {'Flat (0-30 deg)': [], 'Moderate (30-60 deg)': [], 'Extreme (60-90+ deg)': []}
for c in cases:
    r = abs(c['roll'])
    if r > 90: r = 180 - r # Normalizing roll absolute magnitude relative to horizon
    if r < 30: roll_bins['Flat (0-30 deg)'].append(c['fp'])
    elif r < 60: roll_bins['Moderate (30-60 deg)'].append(c['fp'])
    else: roll_bins['Extreme (60-90+ deg)'].append(c['fp'])

for k, v in roll_bins.items():
    if v:
        print(f"{k}: Avg {sum(v)/len(v):.2f} FPs (Max: {max(v)})")

