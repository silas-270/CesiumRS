import pandas as pd
import matplotlib.pyplot as plt

# Load data
df = pd.read_csv('stress_results.csv')

# Calculate missing percentage
df['missing_percent'] = (df['missing_tiles'] / df['requested_tiles']) * 100
df['missing_percent'] = df['missing_percent'].fillna(0) # in case requested_tiles is 0

# Smooth the missing percent to make the line cleaner
df['missing_percent_smooth'] = df['missing_percent'].rolling(window=10, min_periods=1).mean()

# Create plot with 2 y-axes
fig, ax1 = plt.subplots(figsize=(12, 7))

color = 'tab:red'
ax1.set_xlabel('Time (Frames)')
ax1.set_ylabel('Fetch Failure Rate (%)', color=color)
ax1.plot(df['frame'], df['missing_percent_smooth'], color=color, linewidth=2, label='Miss Rate (%)')
ax1.tick_params(axis='y', labelcolor=color)
ax1.set_ylim(0, 105)

ax2 = ax1.twinx()  
color = 'tab:blue'
ax2.set_ylabel('Camera Speed Multiplier', color=color)  
ax2.plot(df['frame'], df['speed_multiplier'], color=color, linestyle='--', linewidth=2, label='Speed Multiplier')
ax2.tick_params(axis='y', labelcolor=color)

plt.title('Tile Fetch Failure Rate Under Increasing Camera Stress')
fig.tight_layout()  
plt.savefig('stress_plot.png')
print("Plot saved as stress_plot.png")
