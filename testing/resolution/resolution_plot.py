import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
from scipy.signal import argrelextrema
from scipy import signal

#### CONTROLS ####
version = 37

def plot_raw_sensor_data(df_merged, version):
    """
    Plot Keyence, Magnetic (filtered), and Posic Encoder data against time on the same graph.
    
    Parameters:
    -----------
    df_merged : pandas.DataFrame
        The merged dataframe containing all sensor data
    version : int
        Version number for the plot title and filename
    """
    
    fig, ax = plt.subplots(1, 1, figsize=(16, 6))
    
    # Plot Keyence displacement
    # valid_keyence = df_merged['keyence_mm'].notna()
    # ax.scatter(
    #     df_merged.loc[valid_keyence, 'time_s'],
    #     df_merged.loc[valid_keyence, 'keyence_mm'],
    #     linewidth=1.5,
    #     color='blue',
    #     label='Keyence',
    #     alpha=0.8
    # )
    valid_keyence = df_merged['keyence_mm'].notna()
    ax.plot(
        df_merged.loc[valid_keyence, 'time_s'],
        df_merged.loc[valid_keyence, 'keyence_mm'],
        linewidth=1.5,
        color='blue',
        label='Keyence',
        alpha=0.8
    )
    
    # Plot Magnetic filtered
    # valid_magnetic = df_merged['magnetic_raw_mm'].notna()
    # ax.scatter(
    #     df_merged.loc[valid_magnetic, 'time_s'],
    #     df_merged.loc[valid_magnetic, 'magnetic_raw_mm'],
    #     linewidth=1.5,
    #     color='red',
    #     label='Magnetic',
    #     alpha=0.8
    # )
    valid_magnetic = df_merged['magnetic_raw_mm'].notna()
    ax.plot(
        df_merged.loc[valid_magnetic, 'time_s'],
        df_merged.loc[valid_magnetic, 'magnetic_raw_mm'],
        linewidth=1.5,
        color='red',
        label='Magnetic',
        alpha=0.8
    )

    # # Plot Magnetic filtered
    # valid_magnetic = df_merged['magnetic_raw_filtered'].notna()
    # ax.plot(
    #     df_merged.loc[valid_magnetic, 'time_s'],
    #     df_merged.loc[valid_magnetic, 'magnetic_raw_filtered'],
    #     linewidth=1.5,
    #     color='red',
    #     label='Magnetic (filtered)',
    #     alpha=0.8
    # )
    
    # # Plot Posic Encoder filtered
    # valid_posic = df_merged['posic_encoder_mm'].notna()
    # ax.plot(
    #     df_merged.loc[valid_posic, 'time_s'],
    #     df_merged.loc[valid_posic, 'posic_encoder_mm'],
    #     linewidth=1.5,
    #     color='green',
    #     label='Posic Encoder',
    #     alpha=0.8
    # )

    # # Plot Posic Encoder filtered
    # valid_posic = df_merged['posic_encoder_filtered'].notna()
    # ax.plot(
    #     df_merged.loc[valid_posic, 'time_s'],
    #     df_merged.loc[valid_posic, 'posic_encoder_filtered'],
    #     linewidth=1.5,
    #     color='green',
    #     label='Posic Encoder (LP Filter)',
    #     alpha=0.8
    # )
    
    ax.set_xlabel('Time (s)', fontsize=12)
    ax.set_ylabel('Displacement (mm)', fontsize=12)
    ax.set_title(f'Raw Sensor Data vs Time (Version {version})', fontsize=14)
    ax.legend(fontsize=10, loc='best')
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    output_path = os.path.join(graphs_dir, f'raw_sensor_data_v{version}.png')
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"\nRaw sensor data plot saved to: {output_path}")

def split_by_time_gap(df_merged, gap_threshold_s=1.5):
    """
    Splits df_merged into segments where the time difference between consecutive rows
    exceeds gap_threshold_s, then extracts the last keyence_mm value from each segment.
    
    Parameters:
    -----------
    df_merged : pandas.DataFrame
        The merged dataframe containing all sensor data
    gap_threshold_s : float
        Time gap threshold in seconds to define a new segment (default: 100)
    
    Returns:
    --------
    segments : list of pd.DataFrame
        List of dataframes, one per segment
    last_keyence_points : pd.DataFrame
        Dataframe containing the last keyence_mm point from each segment
    """
    
    # Calculate time differences between consecutive rows
    time_diffs = df_merged['time_s'].diff()
    
    # Find indices where the gap exceeds the threshold
    split_indices = time_diffs[time_diffs > gap_threshold_s].index.tolist()
    
    # Build segment boundaries
    boundaries = [0] + split_indices + [len(df_merged)]
    
    segments = []
    last_keyence_points = []
    
    for i in range(len(boundaries) - 1):
        start = boundaries[i]
        end = boundaries[i + 1]
        segment = df_merged.iloc[start:end].copy()
        segments.append(segment)
        
        # Get the last row where keyence_mm is not NaN
        valid_keyence = segment['keyence_mm'].dropna()
        if not valid_keyence.empty:
            last_idx = valid_keyence.index[-1]
            last_keyence_points.append(segment.loc[last_idx])
    
    last_keyence_df = pd.DataFrame(last_keyence_points).reset_index(drop=True)
    
    print(f"Found {len(segments)} segments with gap threshold of {gap_threshold_s}s")
    print(f"Last keyence points:\n{last_keyence_df[['time_s', 'keyence_mm']]}")
    
    return segments, last_keyence_df

def plot_mirror_error(last_keyence_df):
    """
    Plots error by subtracting keyence_mm values from opposite ends of the dataframe.
    (row N - row 0, row N-1 - row 1, etc.)
    X-axis shows the magnetic_raw_mm value of the outer row, rounded to nearest tenth.
    
    Parameters:
    -----------
    last_keyence_df : pd.DataFrame
        Dataframe of last keyence points per segment (output of split_by_time_gap)
    """
    
    n = len(last_keyence_df)
    mid = n // 2
    
    errors = []
    x_labels = []
    x_positions = []
    
    for i in range(mid):
        outer_idx = n - 1 - i  # row 18, 17, 16...
        inner_idx = i           # row 0, 1, 2...
        
        outer_row = last_keyence_df.iloc[outer_idx]
        inner_row = last_keyence_df.iloc[inner_idx]
        
        error = outer_row['keyence_mm'] - inner_row['keyence_mm']
        x_val = round(outer_row['magnetic_raw_mm'], 1)
        
        errors.append(error)
        x_labels.append(str(x_val))
        x_positions.append(i)
    
    fig, ax = plt.subplots(figsize=(12, 5))
    
    ax.bar(x_positions, errors, color='steelblue', edgecolor='black', alpha=0.8)
    ax.axhline(0, color='black', linewidth=0.8, linestyle='--')
    
    ax.set_xticks(x_positions)
    ax.set_xticklabels(x_labels, rotation=45, ha='right')
    ax.set_xlabel('Magnetic Raw Position (mm)', fontsize=12)
    ax.set_ylabel('Keyence Error (mm)', fontsize=12)
    ax.set_title('Repeatability Error: Outer vs Inner Keyence Readings', fontsize=14)
    ax.grid(True, axis='y', alpha=0.3)
    
    plt.tight_layout()
    
    output_path = os.path.join(graphs_dir, f'mirror_error_v{version}.png')
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"Mirror error plot saved to: {output_path}")
    
    # plt.show()
    
    return errors, x_labels

# Get the directory where this script is located
script_dir = os.path.dirname(os.path.abspath(__file__))

graphs_dir = os.path.join(script_dir, 'graphs')

# Build the full path to the CSV files
arduino_file_path = os.path.join(script_dir, f"arduino_output{version}.txt")
keyence_file_path = os.path.join(script_dir, f"keyence_output{version}.csv")

# Import Arduino TXT file with 4 columns: time, magnetic_raw, magnetic_filtered, posic_encoder
df_arduino = pd.read_csv(
    arduino_file_path, 
    header=None, 
    delimiter=', ', 
    names=['time_ms', 'magnetic_raw_mm', 'magnetic_filtered_mm', 'posic_encoder_mm'], 
    engine='python'
)

# Strip whitespace from all columns
for col in df_arduino.columns:
    df_arduino[col] = df_arduino[col].astype(str).str.strip()

# Convert all columns to numeric, treating '-' as NaN
for col in df_arduino.columns:
    df_arduino[col] = pd.to_numeric(df_arduino[col], errors='coerce')

# # Remove rows where any measurement column is NaN
# df_arduino = df_arduino.dropna(subset=['magnetic_raw_mm', 'magnetic_filtered_mm', 'posic_encoder_mm'])

print(f"Arduino dataframe length {len(df_arduino)}")

# Import Keyence CSV file
df_keyence = pd.read_csv(keyence_file_path, header=None)

# Find the column with the most non-null values (since Keyence outputs multiple columns, only the long one has the position data)
column_lengths = df_keyence.count()
longest_column_index = column_lengths.idxmax()

# Create a new DataFrame with only the longest column
df_keyence = pd.DataFrame({
    'keyence_mm': df_keyence[longest_column_index]
})

# Merge both keyence and arduino dataframe (keyence will usually have less data points)
df_merged = pd.concat([df_arduino, df_keyence], axis=1)

# Create time_s column by calculating cumulative time differences in seconds
df_merged['time_s'] = (df_merged['time_ms'] - df_merged['time_ms'].iloc[0]) / 1000.0
# Zero the displacement values
df_merged['keyence_mm'] = (df_merged['keyence_mm'] - df_merged['keyence_mm'].iloc[0]) * -1
df_merged['posic_encoder_mm'] = df_merged['posic_encoder_mm'] - df_merged['posic_encoder_mm'].iloc[0]
df_merged['magnetic_raw'] = df_merged['magnetic_raw_mm']
df_merged['magnetic_raw_mm'] = df_merged['magnetic_raw_mm'] - df_merged['magnetic_raw_mm'].iloc[0]
# df_merged['magnetic_raw_mm'] = df_merged['magnetic_raw_mm'] * -1 * 0.00048828125 * 0.9459754389
df_merged['magnetic_raw_mm'] = df_merged['magnetic_raw_mm'] * -1 * 0.00048828125

# Plot raw sensor data against time
plot_raw_sensor_data(df_merged, version)

segments, last_keyence_df = split_by_time_gap(df_merged, gap_threshold_s=1.5)
errors, x_labels = plot_mirror_error(last_keyence_df)

# print(last_keyence_df)

plt.show()
