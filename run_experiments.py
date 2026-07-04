import os
import subprocess
import pandas as pd
import matplotlib.pyplot as plt

def run_test(mode, prefetch, cache_size, label):
    cmd = ["cargo", "run", "--", "--stress", "--stress-mode", mode, "--cache-size", str(cache_size)]
    if prefetch:
        cmd.append("--prefetch")
    print(f"Running {label}...")
    subprocess.run(cmd, check=True)
    
    # It generates stress_results_{mode}.csv
    filename = f"stress_results_{mode}.csv"
    new_filename = f"stress_results_{label.replace(' ', '_').replace('/', '_')}.csv"
    os.replace(filename, new_filename)
    
    df = pd.read_csv(new_filename)
    df['missing_percent'] = (df['missing_tiles'] / df['requested_tiles']) * 100
    df['missing_percent'] = df['missing_percent'].fillna(0)
    df['missing_percent_smooth'] = df['missing_percent'].rolling(window=10, min_periods=1).mean()
    return df

experiments = [
    ("poi", True, 2048, "POI Mode - High Quality"),
    ("poi", False, 256, "POI Mode - High Speed"),
    ("flight", True, 2048, "Flight Mode - High Quality"),
    ("flight", False, 256, "Flight Mode - High Speed"),
]

fig, axes = plt.subplots(2, 2, figsize=(16, 10))
axes = axes.flatten()

for i, (mode, prefetch, cache, label) in enumerate(experiments):
    df = run_test(mode, prefetch, cache, label)
    ax1 = axes[i]
    color = 'tab:red'
    ax1.set_xlabel('Time (Frames)')
    ax1.set_ylabel('Fetch Failure Rate (%)', color=color)
    ax1.plot(df['frame'], df['missing_percent_smooth'], color=color, linewidth=2, label='Miss Rate (%)')
    ax1.tick_params(axis='y', labelcolor=color)
    ax1.set_ylim(0, 105)

    ax2 = ax1.twinx()  
    color = 'tab:blue'
    ax2.set_ylabel('Camera Speed Multiplier', color=color)  
    ax2.plot(df['frame'], df['speed_multiplier'], color=color, linestyle='--', linewidth=2, label='Speed')
    ax2.tick_params(axis='y', labelcolor=color)
    
    ax1.set_title(f"{label}\n(Prefetch: {prefetch}, Cache: {cache})")

fig.tight_layout()
plt.savefig('experiment_results.png')
print("Saved experiment_results.png")
