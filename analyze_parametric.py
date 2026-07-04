import csv

results_path = r"C:\Users\kamme\Desktop\CesiumRS\parametric_results.csv"

altitude_sweep = []
pitch_sweep = []
roll_sweep = []
combined_sweep = []

with open(results_path, 'r', encoding='utf-8') as f:
    reader = csv.DictReader(f)
    for row in reader:
        sweep = row['sweep_type']
        data = {
            'param': float(row['param_value']),
            'fp': int(row['false_positives']),
            'rendered': int(row['rendered'])
        }
        if sweep == 'Altitude':
            altitude_sweep.append(data)
        elif sweep == 'Pitch':
            pitch_sweep.append(data)
        elif sweep == 'Roll':
            roll_sweep.append(data)
        elif sweep == 'Combined_Pitch_Roll':
            combined_sweep.append({
                'pitch': float(row['pitch']),
                'roll': float(row['roll']),
                'fp': int(row['false_positives'])
            })

print("--- ALTITUDE SWEEP (Pitch=-90, straight down) ---")
# Print a sample of 10 points along the curve
for i in range(0, len(altitude_sweep), max(1, len(altitude_sweep)//10)):
    d = altitude_sweep[i]
    print(f"Alt {d['param']:.5f}: {d['fp']} FPs (Rendered: {d['rendered']})")

print("\n--- PITCH SWEEP (Alt=0.1, -90 to 90) ---")
for i in range(0, len(pitch_sweep), max(1, len(pitch_sweep)//10)):
    d = pitch_sweep[i]
    print(f"Pitch {d['param']:.1f}: {d['fp']} FPs (Rendered: {d['rendered']})")

print("\n--- ROLL SWEEP (Alt=0.1, Pitch=-45, -180 to 180) ---")
for i in range(0, len(roll_sweep), max(1, len(roll_sweep)//10)):
    d = roll_sweep[i]
    print(f"Roll {d['param']:.1f}: {d['fp']} FPs (Rendered: {d['rendered']})")

print("\n--- COMBINED SWEEP HIGHEST FALSE POSITIVES ---")
combined_sweep.sort(key=lambda x: x['fp'], reverse=True)
for i in range(5):
    if i < len(combined_sweep):
        d = combined_sweep[i]
        print(f"Pitch {d['pitch']:.1f}, Roll {d['roll']:.1f}: {d['fp']} FPs")
