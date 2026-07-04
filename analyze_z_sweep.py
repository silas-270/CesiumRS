import csv

results_path = r"C:\Users\kamme\Desktop\CesiumRS\z_sweep_results_fine.csv"

z_sweep = []

with open(results_path, 'r', encoding='utf-8') as f:
    reader = csv.DictReader(f)
    for row in reader:
        data = {
            'z': float(row['z']),
            'rendered': int(row['rendered']),
            'hit': int(row['hit']),
            'fp': int(row['false_positives']),
            'fn': int(row['false_negatives'])
        }
        z_sweep.append(data)

print("--- Z-AXIS SWEEP (Pos=0,0,Z) ---")
# Print a sample of points near 8.2 specifically
for d in z_sweep:
    if 8.0 < d['z'] < 8.5:
        print(f"Z {d['z']:.3f}: {d['fp']} FPs (Rendered: {d['rendered']}, Hit: {d['hit']}, FNs: {d['fn']})")

print("\n--- SAMPLE OF ALL Z ---")
for i in range(0, len(z_sweep), max(1, len(z_sweep)//10)):
    d = z_sweep[i]
    print(f"Z {d['z']:.3f}: {d['fp']} FPs (Rendered: {d['rendered']})")
