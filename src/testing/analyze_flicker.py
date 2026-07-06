import csv

def analyze_metrics():
    drops = []
    prev_renderable = 0
    prev_visible = 0
    prev_missing = 0
    with open('flicker_metrics.csv', 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            frame = int(row['Frame'])
            if frame < 100: 
                # Skip first 100 frames to avoid initial load noise
                prev_renderable = int(row['RenderableTiles'])
                prev_visible = int(row['VisibleTiles'])
                prev_missing = int(row['MissingCount'])
                continue

            visible = int(row['VisibleTiles'])
            renderable = int(row['RenderableTiles'])
            missing = int(row['MissingCount'])
            progress = float(row['Progress'])

            # Look for cases where renderable is less than visible AFTER initial load
            if renderable < visible:
                drops.append(f"Frame {frame} (Prog {progress:.4f}): Renderable {renderable} < Visible {visible} (Missing: {missing})")
            
            # Look for missing count spikes
            if missing > 0 and prev_missing == 0:
                drops.append(f"Frame {frame} (Prog {progress:.4f}): Missing count spiked to {missing}")

            # Look for 1-frame glitches in visible or renderable
            if prev_renderable - renderable > 3:
                drops.append(f"Frame {frame} (Prog {progress:.4f}): Renderable dropped by {prev_renderable - renderable}")
            
            if prev_visible - visible > 3:
                drops.append(f"Frame {frame} (Prog {progress:.4f}): Visible dropped by {prev_visible - visible}")

            prev_renderable = renderable
            prev_visible = visible
            prev_missing = missing

    if len(drops) == 0:
        print("No drops found!")
    else:
        print(f"Found {len(drops)} potential drops:")
        for d in drops[:50]: # Print first 50 to avoid spam
            print(d)
        if len(drops) > 50:
            print(f"... and {len(drops) - 50} more")

if __name__ == '__main__':
    analyze_metrics()
