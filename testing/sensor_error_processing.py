import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
from scipy import signal

# Get the directory where this script is located
script_dir = os.path.dirname(os.path.abspath(__file__))

version = 10
graphs_dir = os.path.join(script_dir, 'graphs')

# Build the full path to the processed CSV file
processed_file_path = os.path.join(script_dir, f'processed_data{version}.csv')

# Read the processed data CSV
df_combined = pd.read_csv(processed_file_path)

print("Combined DataFrame loaded:")
print(f"Shape: {df_combined.shape}")
print(f"\nColumn names: {df_combined.columns.tolist()}")

# Extract Keyence dataframe
df_keyence = df_combined[['keyence_time_s', 'keyence_displacement_mm']].copy()
df_keyence = df_keyence.rename(columns={
    'keyence_time_s': 'time_s',
    'keyence_displacement_mm': 'displacement_mm'
})
# Remove any rows with NaN values
df_keyence = df_keyence.dropna()

print(f"\n\nKeyence DataFrame:")
print(f"Shape: {df_keyence.shape}")
print(f"Time range: {df_keyence['time_s'].min():.4f} to {df_keyence['time_s'].max():.4f} s")

# Extract Arduino dataframe
df_arduino = df_combined[['arduino_time_s', 'arduino_displacement_mm']].copy()
df_arduino = df_arduino.rename(columns={
    'arduino_time_s': 'time_s',
    'arduino_displacement_mm': 'displacement_mm'
})
# Remove any rows with NaN values
df_arduino = df_arduino.dropna()

print(f"\n\nArduino DataFrame:")
print(f"Shape: {df_arduino.shape}")
print(f"Time range: {df_arduino['time_s'].min():.4f} to {df_arduino['time_s'].max():.4f} s")

# Find shared time values (inner join on time_s)
df_merged = pd.merge(df_keyence, df_arduino, on='time_s', how='inner', suffixes=('_keyence', '_arduino'))

print(f"\n\nMerged DataFrame (only shared time_s values):")
print(f"Shape: {df_merged.shape}")
print(f"Number of shared time points: {len(df_merged)}")
print(f"\nFirst few rows of merged data:")
print(df_merged.head())
print(f"\nLast few rows of merged data:")
print(df_merged.tail())

# Calculate error (Keyence - Arduino)
df_merged['error'] = df_merged['displacement_mm_keyence'] - df_merged['displacement_mm_arduino']

print(f"\n\nError Statistics:")
print(f"Mean error: {df_merged['error'].mean():.4f} mm")
print(f"Std deviation: {df_merged['error'].std():.4f} mm")
print(f"Max error: {df_merged['error'].max():.4f} mm")
print(f"Min error: {df_merged['error'].min():.4f} mm")

# Create first figure with two subplots (Time domain)
fig1, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))
fig1.canvas.manager.set_window_title('Time Domain Analysis')

# Plot 1: Keyence and Arduino sensors with dual y-axis
color_keyence = 'tab:blue'
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Keyence Displacement (mm)', color=color_keyence)
line1 = ax1.plot(df_merged['time_s'], df_merged['displacement_mm_keyence'], 
                 color=color_keyence, alpha=0.7, linewidth=1.5, label='Keyence')
ax1.tick_params(axis='y', labelcolor=color_keyence)
ax1.grid(True, alpha=0.3)

# Create second y-axis for Arduino
ax1_twin = ax1.twinx()
color_arduino = 'tab:orange'
ax1_twin.set_ylabel('Arduino Displacement (mm)', color=color_arduino)
line2 = ax1_twin.plot(df_merged['time_s'], df_merged['displacement_mm_arduino'], 
                      color=color_arduino, alpha=0.7, linewidth=1.5, label='Arduino')
ax1_twin.tick_params(axis='y', labelcolor=color_arduino)

# Add title and legend
ax1.set_title('Keyence and Arduino Sensor Measurements vs Time (Dual Y-Axis)')
lines = line1 + line2
labels = [l.get_label() for l in lines]
ax1.legend(lines, labels, loc='upper left')

# Plot 2: Error
ax2.scatter(df_merged['time_s'], df_merged['error'], 
           s=10, alpha=0.6, label='Error', color='tab:green')
ax2.axhline(y=0, color='r', linestyle='--', linewidth=1.5, label='Zero Error')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Error (Keyence - Arduino) (mm)')
ax2.set_title('Error Between Keyence and Arduino Sensors vs Time')
ax2.legend()
ax2.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig(os.path.join(graphs_dir, f'error_sensor{version}.png'), dpi=300, bbox_inches='tight')

# Create second figure for FFT Analysis
fig2, ax3 = plt.subplots(figsize=(12, 7))
fig2.canvas.manager.set_window_title('FFT Analysis')

# Calculate sampling rate
time_diff = np.diff(df_merged['time_s'])
avg_dt = np.mean(time_diff)
sampling_rate = 1.0 / avg_dt
n = len(df_merged)
print(f"\n\nFFT Analysis:")
print(time_diff[:10])
print(f"Average sampling interval: {avg_dt:.6f} s")
print(f"Average sampling rate: {sampling_rate:.2f} Hz")

# FFT for Error
fhat = np.fft.fft(df_merged['error'], n)
psd = fhat * np.conj(fhat) / n
freq = (1 / (avg_dt * n)) * np.arange(n)
L = np.arange(1, np.floor(n/2), dtype='int')  # Fixed: get first half of spectrum, excluding DC

# Plot on the existing ax3
ax3.plot(freq[L], psd[L], color='c', linewidth=1.5)
ax3.set_xlabel('Frequency (Hz)')
ax3.set_ylabel('Power Spectral Density')
ax3.set_title('FFT Analysis of Error Signal')
ax3.grid(True, alpha=0.3)
ax3.set_xlim([0, 1/(2*avg_dt)])  # Nyquist frequency

plt.tight_layout()
plt.savefig(os.path.join(graphs_dir, f'fft_sensor{version}.png'), dpi=300, bbox_inches='tight')

# Show both figures
plt.show()
