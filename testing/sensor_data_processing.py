import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os

# Get the directory where this script is located
script_dir = os.path.dirname(os.path.abspath(__file__))

version = 10
graphs_dir = os.path.join(script_dir, 'graphs')

# Build the full path to the CSV files
keyence_file_path = os.path.join(script_dir, f"keyence_output{10}.csv")
arduino_file_path = os.path.join(script_dir, f"arduino_output{10}.txt")

# Import Keyence CSV file
df_keyence = pd.read_csv(keyence_file_path, header=None)

# Find the column with the most non-null values
column_lengths = df_keyence.count()
longest_column_index = column_lengths.idxmax()

# Create a new DataFrame with only the longest column
df_keyence = pd.DataFrame({
    'displacement_mm': df_keyence[longest_column_index]
})

# Remove any NaN values
df_keyence = df_keyence.dropna()

# Create time_s column that starts at 0 and increments by 0.0002
df_keyence.insert(0, 'time_s', [i * 0.0002 for i in range(len(df_keyence))])

# Find global max and min for Keyence
keyence_max_idx = df_keyence['displacement_mm'].idxmax()
keyence_min_idx = df_keyence['displacement_mm'].idxmin()

print(f"\nKeyence - Global Maximum:")
print(f"Index: {keyence_max_idx}")
print(f"Time: {df_keyence.loc[keyence_max_idx, 'time_s']:.4f} s")
print(f"Displacement: {df_keyence.loc[keyence_max_idx, 'displacement_mm']:.4f} mm")

print(f"\nKeyence - Global Minimum:")
print(f"Index: {keyence_min_idx}")
print(f"Time: {df_keyence.loc[keyence_min_idx, 'time_s']:.4f} s")
print(f"Displacement: {df_keyence.loc[keyence_min_idx, 'displacement_mm']:.4f} mm")

# Determine which occurs first and second (max or min) for Keyence
keyence_first_extreme_idx = min(keyence_max_idx, keyence_min_idx)
keyence_second_extreme_idx = max(keyence_max_idx, keyence_min_idx)
keyence_first_extreme_type = "Max" if keyence_first_extreme_idx == keyence_max_idx else "Min"
keyence_second_extreme_type = "Max" if keyence_second_extreme_idx == keyence_max_idx else "Min"

print(f"\nKeyence - First extreme point is {keyence_first_extreme_type} at index {keyence_first_extreme_idx}")
print(f"Keyence - Second extreme point is {keyence_second_extreme_type} at index {keyence_second_extreme_idx}")

# Function to determine which extreme is the true turning point
def select_turning_point(df, first_idx, second_idx, first_type):
    """
    Determines which extreme point (first or second) represents the turning point.
    The turning point is where the signal changes direction and begins the main response.
    
    Strategy: Check which extreme has more data after it (indicating it's the start of the main response)
    and analyze the trend after each extreme.
    """
    data_after_first = len(df) - first_idx
    data_after_second = len(df) - second_idx
    
    # Calculate the slope/trend after each extreme (looking at next 10% of remaining data)
    window_first = max(10, int(0.1 * data_after_first))
    window_second = max(10, int(0.1 * data_after_second))
    
    # Get data windows after each extreme
    first_window = df.loc[first_idx:min(first_idx + window_first, len(df)-1), 'displacement_mm']
    second_window = df.loc[second_idx:min(second_idx + window_second, len(df)-1), 'displacement_mm']
    
    # Calculate if there's a consistent trend after each point
    first_has_trend = len(first_window) > 1 and abs(first_window.iloc[-1] - first_window.iloc[0]) > 0.1
    second_has_trend = len(second_window) > 1 and abs(second_window.iloc[-1] - second_window.iloc[0]) > 0.1
    
    print(f"\n  Data points after first extreme: {data_after_first}")
    print(f"  Data points after second extreme: {data_after_second}")
    print(f"  First extreme has trend: {first_has_trend}")
    print(f"  Second extreme has trend: {second_has_trend}")
    
    # Decision logic:
    # 1. If second extreme has very little data after it (<5% of total), use first
    # 2. If second extreme has significant data and shows a trend, use second
    # 3. Otherwise, use the one with more data after it
    
    total_points = len(df)
    
    if data_after_second < 0.05 * total_points:
        print(f"  -> Selected FIRST extreme (second has insufficient data)")
        return first_idx, first_type
    elif data_after_second > 0.2 * total_points and second_has_trend:
        print(f"  -> Selected SECOND extreme (has significant data and trend)")
        return second_idx, "Max" if first_type == "Min" else "Min"
    elif data_after_first > data_after_second:
        print(f"  -> Selected FIRST extreme (has more data)")
        return first_idx, first_type
    else:
        print(f"  -> Selected SECOND extreme (has more data)")
        return second_idx, "Max" if first_type == "Min" else "Min"

# Select the turning point for Keyence
print("\n=== Keyence Turning Point Selection ===")
keyence_turning_idx, keyence_turning_type = select_turning_point(
    df_keyence, keyence_first_extreme_idx, keyence_second_extreme_idx, keyence_first_extreme_type
)

# Create dataframe starting from turning point
df_keyence_after_extreme = df_keyence.loc[keyence_turning_idx:].copy()
df_keyence_after_extreme.reset_index(drop=True, inplace=True)

# Shift time to start at 0
df_keyence_after_extreme['time_s'] = df_keyence_after_extreme['time_s'] - df_keyence_after_extreme['time_s'].iloc[0]

# Shift displacement to start at 0
df_keyence_after_extreme['displacement_mm'] = df_keyence_after_extreme['displacement_mm'] - df_keyence_after_extreme['displacement_mm'].iloc[0]

print(f"Keyence DataFrame after turning point:")
print(f"Number of rows: {len(df_keyence_after_extreme)}")

# Import Arduino TXT file with ", " delimiter and specify engine to avoid warning
df_arduino = pd.read_csv(arduino_file_path, header=None, delimiter=', ', names=['time_ms', 'displacement_mm'], engine='python')

# Strip whitespace first
df_arduino['time_ms'] = df_arduino['time_ms'].astype(str).str.strip()
df_arduino['displacement_mm'] = df_arduino['displacement_mm'].astype(str).str.strip()

# Convert to numeric, treating '-' as NaN
df_arduino['time_ms'] = pd.to_numeric(df_arduino['time_ms'], errors='coerce')
df_arduino['displacement_mm'] = pd.to_numeric(df_arduino['displacement_mm'], errors='coerce')/1000

# Remove rows where displacement_mm is NaN (was just '-')
df_arduino = df_arduino.dropna(subset=['displacement_mm'])

# Convert bits to mm (with scaling)
df_arduino['displacement_mm'] = df_arduino['displacement_mm'] * 0.48828125 * 0.9335003416

# Create time_s column by calculating cumulative time differences in seconds
df_arduino['time_s'] = (df_arduino['time_ms'] - df_arduino['time_ms'].iloc[0]) / 1000.0

# Find global max and min for Arduino
arduino_max_idx = df_arduino['displacement_mm'].idxmax()
arduino_min_idx = df_arduino['displacement_mm'].idxmin()

print(f"\n\nArduino - Global Maximum:")
print(f"Index: {arduino_max_idx}")
print(f"Time: {df_arduino.loc[arduino_max_idx, 'time_s']:.4f} s")
print(f"Displacement: {df_arduino.loc[arduino_max_idx, 'displacement_mm']:.4f} mm")

print(f"\nArduino - Global Minimum:")
print(f"Index: {arduino_min_idx}")
print(f"Time: {df_arduino.loc[arduino_min_idx, 'time_s']:.4f} s")
print(f"Displacement: {df_arduino.loc[arduino_min_idx, 'displacement_mm']:.4f} mm")

# Determine which occurs first and second (max or min) for Arduino
arduino_first_extreme_idx = min(arduino_max_idx, arduino_min_idx)
arduino_second_extreme_idx = max(arduino_max_idx, arduino_min_idx)
arduino_first_extreme_type = "Max" if arduino_first_extreme_idx == arduino_max_idx else "Min"
arduino_second_extreme_type = "Max" if arduino_second_extreme_idx == arduino_max_idx else "Min"

print(f"\nArduino - First extreme point is {arduino_first_extreme_type} at index {arduino_first_extreme_idx}")
print(f"Arduino - Second extreme point is {arduino_second_extreme_type} at index {arduino_second_extreme_idx}")

# Select the turning point for Arduino
print("\n=== Arduino Turning Point Selection ===")
arduino_turning_idx, arduino_turning_type = select_turning_point(
    df_arduino, arduino_first_extreme_idx, arduino_second_extreme_idx, arduino_first_extreme_type
)

# Create dataframe starting from turning point
df_arduino_after_extreme = df_arduino.loc[arduino_turning_idx:].copy()
df_arduino_after_extreme.reset_index(drop=True, inplace=True)

# Shift time to start at 0
df_arduino_after_extreme['time_s'] = df_arduino_after_extreme['time_s'] - df_arduino_after_extreme['time_s'].iloc[0]

# Shift displacement to start at 0
df_arduino_after_extreme['displacement_mm'] = df_arduino_after_extreme['displacement_mm'] - df_arduino_after_extreme['displacement_mm'].iloc[0]
df_arduino_after_extreme['displacement_mm'] = df_arduino_after_extreme['displacement_mm'] * (-1)

# Rename columns to distinguish between sensors
df_keyence_after_extreme_renamed = df_keyence_after_extreme.rename(columns={
    'time_s': 'keyence_time_s',
    'displacement_mm': 'keyence_displacement_mm'
})

df_arduino_after_extreme_renamed = df_arduino_after_extreme.rename(columns={
    'time_s': 'arduino_time_s',
    'displacement_mm': 'arduino_displacement_mm'
})

# Combine both dataframes side by side
df_combined = pd.concat([df_keyence_after_extreme_renamed, df_arduino_after_extreme_renamed], axis=1)

# Export to CSV
output_file_path = os.path.join(script_dir, f"processed_data{version}.csv")
df_combined.to_csv(output_file_path, index=False)
print(f"\nData exported to: {output_file_path}")

# Create the plot for Keyence data with max and min marked
plt.figure(figsize=(10, 6))
plt.plot(df_keyence['time_s'], df_keyence['displacement_mm'], linewidth=0.8, label='Full Data')
plt.scatter(df_keyence.loc[keyence_max_idx, 'time_s'], 
           df_keyence.loc[keyence_max_idx, 'displacement_mm'],
           color='red', s=100, zorder=5, label=f'Max ({df_keyence.loc[keyence_max_idx, "displacement_mm"]:.4f} mm)')
plt.scatter(df_keyence.loc[keyence_min_idx, 'time_s'], 
           df_keyence.loc[keyence_min_idx, 'displacement_mm'],
           color='blue', s=100, zorder=5, label=f'Min ({df_keyence.loc[keyence_min_idx, "displacement_mm"]:.4f} mm)')
plt.axvline(x=df_keyence.loc[keyence_turning_idx, 'time_s'], 
            color='green', linestyle='--', linewidth=2, 
            label=f'Turning Point ({keyence_turning_type})')
plt.xlabel('Time (s)')
plt.ylabel('Displacement (mm)')
plt.title('Keyence Sensor - Global Extremes and Turning Point')
plt.legend()
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig(os.path.join(graphs_dir, f'keyence_extremes{version}.png'), dpi=300, bbox_inches='tight')

# Create the plot for Arduino data with max and min marked
plt.figure(figsize=(10, 6))
plt.plot(df_arduino['time_s'], df_arduino['displacement_mm'], linewidth=0.8, color='orange', label='Full Data')
plt.scatter(df_arduino.loc[arduino_max_idx, 'time_s'], 
           df_arduino.loc[arduino_max_idx, 'displacement_mm'],
           color='red', s=100, zorder=5, label=f'Max ({df_arduino.loc[arduino_max_idx, "displacement_mm"]:.4f} mm)')
plt.scatter(df_arduino.loc[arduino_min_idx, 'time_s'], 
           df_arduino.loc[arduino_min_idx, 'displacement_mm'],
           color='blue', s=100, zorder=5, label=f'Min ({df_arduino.loc[arduino_min_idx, "displacement_mm"]:.4f} mm)')
plt.axvline(x=df_arduino.loc[arduino_turning_idx, 'time_s'], 
            color='green', linestyle='--', linewidth=2, 
            label=f'Turning Point ({arduino_turning_type})')
plt.xlabel('Time (s)')
plt.ylabel('Displacement (mm)')
plt.title('Arduino Sensor - Global Extremes and Turning Point')
plt.legend()
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig(os.path.join(graphs_dir, f'arduino_extremes{version}.png'), dpi=300, bbox_inches='tight')

# Create combined plot with both sensors after extreme point
plt.figure(figsize=(12, 7))
plt.plot(df_keyence_after_extreme['time_s'], df_keyence_after_extreme['displacement_mm'], 
         linewidth=1.5, label='Keyence Sensor', alpha=0.8)
plt.plot(df_arduino_after_extreme['time_s'], df_arduino_after_extreme['displacement_mm'], 
         linewidth=1.5, label='Arduino Sensor', alpha=0.8)
plt.xlabel('Time (s)', fontsize=12)
plt.ylabel('Displacement (mm)', fontsize=12)
plt.title('Keyence and Arduino Sensors - Data After Turning Point', fontsize=14, fontweight='bold')
plt.legend(fontsize=11)
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig(os.path.join(graphs_dir, f'combined_sensors{version}.png'), dpi=300, bbox_inches='tight')

# Print some summary statistics
print("\n\n=== Summary Statistics ===")
print(f"\nKeyence - Turning point: {keyence_turning_type} at time {df_keyence.loc[keyence_turning_idx, 'time_s']:.4f} s")
print(f"Keyence - Time range: 0 to {df_keyence_after_extreme['time_s'].iloc[-1]:.4f} s")
print(f"Keyence - Displacement range: {df_keyence_after_extreme['displacement_mm'].min():.4f} to {df_keyence_after_extreme['displacement_mm'].max():.4f} mm")
print(f"\nArduino - Turning point: {arduino_turning_type} at time {df_arduino.loc[arduino_turning_idx, 'time_s']:.4f} s")
print(f"Arduino - Time range: 0 to {df_arduino_after_extreme['time_s'].iloc[-1]:.4f} s")
print(f"Arduino - Displacement range: {df_arduino_after_extreme['displacement_mm'].min():.4f} to {df_arduino_after_extreme['displacement_mm'].max():.4f} mm")
print(f"\nGraphs saved to: {graphs_dir}")

plt.show()
