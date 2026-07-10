#!/usr/bin/env python3
"""
CSV Data Visualizer for Magnetometer Data
Reads recorded CSV data and creates visualizations.
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import argparse
import os
from pathlib import Path

def load_csv_data(csv_file):
    """Load magnetometer data from CSV file."""
    try:
        # Read CSV file
        df = pd.read_csv(csv_file)

        # Expected columns: Bx1,By1,Bz1,Bx2,By2,Bz2,...,Bx16,By16,Bz16,x,y,z,mx,my,mz
        expected_cols = 48 + 6  # 48 magnetic + 6 pose

        if len(df.columns) != expected_cols:
            print(f"Warning: Expected {expected_cols} columns, got {len(df.columns)}")

        print(f"Loaded {len(df)} data points from {csv_file}")
        print(f"Columns: {list(df.columns)}")

        return df

    except Exception as e:
        print(f"Error loading CSV file: {e}")
        return None

def plot_all_sensors_z(df, save_path=None):
    """Plot Z-axis data for all 16 sensors."""
    fig, axes = plt.subplots(4, 4, figsize=(16, 12))
    axes = axes.flatten()
    fig.suptitle('Magnetic Field Z-Axis for All 16 Sensors (Recorded Data)', fontsize=14)

    for i in range(16):
        bz_col = f'Bz{i+1}'
        if bz_col in df.columns:
            axes[i].plot(df.index, df[bz_col], 'b-', linewidth=1.5, label=f'Bz{i+1}')
            axes[i].set_title(f'Sensor {i+1}')
            axes[i].set_ylabel('Bz (units)')
            axes[i].grid(True, alpha=0.3)
            axes[i].legend()

            # Auto-scale y-axis
            if len(df) > 0:
                y_min, y_max = df[bz_col].min(), df[bz_col].max()
                margin = (y_max - y_min) * 0.1 if y_max != y_min else abs(y_max) * 0.1
                axes[i].set_ylim(y_min - margin, y_max + margin)

    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"Saved plot to {save_path}")

    plt.show()

def plot_single_sensor_all_axes(df, sensor_num, save_path=None):
    """Plot Bx, By, Bz for a single sensor."""
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 8))
    fig.suptitle(f'Magnetic Field Components - Sensor {sensor_num} (Recorded Data)', fontsize=14)

    bx_col = f'Bx{sensor_num}'
    by_col = f'By{sensor_num}'
    bz_col = f'Bz{sensor_num}'

    if bx_col in df.columns:
        ax1.plot(df.index, df[bx_col], 'r-', linewidth=2, label='Bx')
        ax1.set_ylabel('Bx (units)')
        ax1.grid(True, alpha=0.3)
        ax1.legend()
        ax1.set_title('X-Axis')

    if by_col in df.columns:
        ax2.plot(df.index, df[by_col], 'g-', linewidth=2, label='By')
        ax2.set_ylabel('By (units)')
        ax2.grid(True, alpha=0.3)
        ax2.legend()
        ax2.set_title('Y-Axis')

    if bz_col in df.columns:
        ax3.plot(df.index, df[bz_col], 'b-', linewidth=2, label='Bz')
        ax3.set_ylabel('Bz (units)')
        ax3.set_xlabel('Time (samples)')
        ax3.grid(True, alpha=0.3)
        ax3.legend()
        ax3.set_title('Z-Axis')

    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"Saved plot to {save_path}")

    plt.show()

def plot_pose_data(df, save_path=None):
    """Plot position and orientation data."""
    fig, ((ax1, ax2, ax3), (ax4, ax5, ax6)) = plt.subplots(2, 3, figsize=(15, 8))
    fig.suptitle('Position and Orientation Data (Recorded)', fontsize=14)

    # Position data
    if 'x' in df.columns:
        ax1.plot(df.index, df['x'], 'r-', linewidth=2)
        ax1.set_ylabel('X Position')
        ax1.set_title('Position X')
        ax1.grid(True, alpha=0.3)

    if 'y' in df.columns:
        ax2.plot(df.index, df['y'], 'g-', linewidth=2)
        ax2.set_ylabel('Y Position')
        ax2.set_title('Position Y')
        ax2.grid(True, alpha=0.3)

    if 'z' in df.columns:
        ax3.plot(df.index, df['z'], 'b-', linewidth=2)
        ax3.set_ylabel('Z Position')
        ax3.set_title('Position Z')
        ax3.grid(True, alpha=0.3)

    # Orientation data
    if 'mx' in df.columns:
        ax4.plot(df.index, df['mx'], 'r-', linewidth=2)
        ax4.set_ylabel('MX Orientation')
        ax4.set_xlabel('Time (samples)')
        ax4.set_title('Orientation MX')
        ax4.grid(True, alpha=0.3)

    if 'my' in df.columns:
        ax5.plot(df.index, df['my'], 'g-', linewidth=2)
        ax5.set_ylabel('MY Orientation')
        ax5.set_xlabel('Time (samples)')
        ax5.set_title('Orientation MY')
        ax5.grid(True, alpha=0.3)

    if 'mz' in df.columns:
        ax6.plot(df.index, df['mz'], 'b-', linewidth=2)
        ax6.set_ylabel('MZ Orientation')
        ax6.set_xlabel('Time (samples)')
        ax6.set_title('Orientation MZ')
        ax6.grid(True, alpha=0.3)

    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"Saved plot to {save_path}")

    plt.show()

def plot_magnetic_field_heatmap(df, save_path=None):
    """Create a heatmap of magnetic field strength across all sensors."""
    # Calculate magnetic field magnitude for each sensor at each time point
    magnitudes = []
    sensor_names = []

    for i in range(16):
        bx_col = f'Bx{i+1}'
        by_col = f'By{i+1}'
        bz_col = f'Bz{i+1}'

        if all(col in df.columns for col in [bx_col, by_col, bz_col]):
            # Calculate magnitude: sqrt(Bx^2 + By^2 + Bz^2)
            magnitude = np.sqrt(df[bx_col]**2 + df[by_col]**2 + df[bz_col]**2)
            magnitudes.append(magnitude)
            sensor_names.append(f'Sensor {i+1}')

    if magnitudes:
        magnitudes = np.array(magnitudes)

        fig, ax = plt.subplots(figsize=(12, 8))
        im = ax.imshow(magnitudes, aspect='auto', cmap='viridis', interpolation='nearest')

        ax.set_title('Magnetic Field Magnitude Heatmap (All Sensors)', fontsize=14)
        ax.set_xlabel('Time (samples)')
        ax.set_ylabel('Sensor')

        # Set y-axis ticks to show sensor numbers
        ax.set_yticks(range(len(sensor_names)))
        ax.set_yticklabels(sensor_names)

        # Add colorbar
        cbar = plt.colorbar(im, ax=ax)
        cbar.set_label('Magnetic Field Magnitude (units)')

        plt.tight_layout()

        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"Saved heatmap to {save_path}")

        plt.show()

def analyze_data(df):
    """Provide basic statistical analysis of the data."""
    print("\n" + "="*60)
    print("DATA ANALYSIS SUMMARY")
    print("="*60)

    print(f"Total data points: {len(df)}")
    print(".2f")

    # Analyze magnetic field data
    print("\nMAGNETIC FIELD STATISTICS:")
    mag_cols = [col for col in df.columns if col.startswith('B')]
    if mag_cols:
        print(f"Magnetic field columns: {len(mag_cols)}")

        # Group by sensor and axis
        for sensor in range(1, 17):
            sensor_cols = [f'Bx{sensor}', f'By{sensor}', f'Bz{sensor}']
            if all(col in df.columns for col in sensor_cols):
                print(f"\nSensor {sensor}:")
                for axis in ['x', 'y', ' ']:
                    col = f'B{axis}{sensor}'
                    if col in df.columns:
                        mean_val = df[col].mean()
                        std_val = df[col].std()
                        min_val = df[col].min()
                        max_val = df[col].max()
                        print(f"  B{axis}: mean={mean_val:.6f}, std={std_val:.6f}, range=[{min_val:.6f}, {max_val:.6f}]")

    # Analyze pose data
    pose_cols = ['x', 'y', 'z', 'mx', 'my', 'mz']
    print(f"\nPOSE DATA STATISTICS:")
    for col in pose_cols:
        if col in df.columns:
            mean_val = df[col].mean()
            std_val = df[col].std()
            min_val = df[col].min()
            max_val = df[col].max()
            print(f"  {col}: mean={mean_val:.6f}, std={std_val:.6f}, range=[{min_val:.6f}, {max_val:.6f}]")

def main():
    parser = argparse.ArgumentParser(description='Magnetometer CSV Data Visualizer')
    parser.add_argument('csv_file', help='Path to CSV file containing magnetometer data')
    parser.add_argument('--sensor', '-s', type=int, default=1, choices=range(1, 17),
                       help='Sensor number to plot individually (1-16, default: 1)')
    parser.add_argument('--plot-type', '-p', choices=['all_z', 'single_sensor', 'pose', 'heatmap', 'all'],
                       default='all', help='Type of plot to generate')
    parser.add_argument('--save', action='store_true',
                       help='Save plots to files instead of displaying')
    parser.add_argument('--output-dir', '-o', default='plots',
                       help='Output directory for saved plots (default: plots)')

    args = parser.parse_args()

    # Check if CSV file exists
    if not os.path.exists(args.csv_file):
        print(f"Error: CSV file '{args.csv_file}' not found")
        return

    # Load data
    df = load_csv_data(args.csv_file)
    if df is None:
        return

    # Analyze data
    analyze_data(df)

    # Create output directory if saving
    if args.save:
        output_dir = Path(args.output_dir)
        output_dir.mkdir(exist_ok=True)

    # Generate plots based on type
    if args.plot_type == 'all_z' or args.plot_type == 'all':
        save_path = output_dir / f"{Path(args.csv_file).stem}_all_sensors_z.png" if args.save else None
        plot_all_sensors_z(df, save_path)

    if args.plot_type == 'single_sensor' or args.plot_type == 'all':
        save_path = output_dir / f"{Path(args.csv_file).stem}_sensor_{args.sensor}_all_axes.png" if args.save else None
        plot_single_sensor_all_axes(df, args.sensor, save_path)

    if args.plot_type == 'pose' or args.plot_type == 'all':
        save_path = output_dir / f"{Path(args.csv_file).stem}_pose_data.png" if args.save else None
        plot_pose_data(df, save_path)

    if args.plot_type == 'heatmap' or args.plot_type == 'all':
        save_path = output_dir / f"{Path(args.csv_file).stem}_heatmap.png" if args.save else None
        plot_magnetic_field_heatmap(df, save_path)

if __name__ == "__main__":
    main()