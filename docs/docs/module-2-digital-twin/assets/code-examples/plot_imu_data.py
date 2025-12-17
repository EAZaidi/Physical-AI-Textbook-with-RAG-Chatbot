#!/usr/bin/env python3
"""
Plot IMU data from CSV file
Usage: python3 plot_imu_data.py [input_file.csv]
"""

import sys
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np


def plot_imu_data(csv_file='imu_data.csv', output_file='03-imu-plot.png'):
    """
    Plot IMU acceleration and angular velocity data

    Args:
        csv_file: Input CSV file path
        output_file: Output PNG file path
    """
    # Read CSV
    print(f'Reading {csv_file}...')
    data = pd.read_csv(csv_file)

    print(f'Loaded {len(data)} samples')
    print(f'Time range: {data["timestamp"].min():.2f}s to {data["timestamp"].max():.2f}s')

    # Create figure with 2 subplots
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))

    # Plot linear acceleration
    ax1.plot(data['timestamp'], data['ax'], label='ax', linewidth=0.8, alpha=0.7)
    ax1.plot(data['timestamp'], data['ay'], label='ay', linewidth=0.8, alpha=0.7)
    ax1.plot(data['timestamp'], data['az'], label='az', linewidth=0.8, alpha=0.7)
    ax1.set_ylabel('Linear Acceleration (m/s²)', fontsize=11)
    ax1.set_title('IMU Data Visualization', fontsize=13, fontweight='bold')
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.3)
    ax1.set_xlim(data['timestamp'].min(), data['timestamp'].max())

    # Add statistics to acceleration plot
    az_mean = data['az'].mean()
    az_std = data['az'].std()
    ax1.axhline(az_mean, color='green', linestyle='--', linewidth=1, alpha=0.5,
                label=f'az mean: {az_mean:.2f} m/s²')
    ax1.text(0.02, 0.98, f'az: μ={az_mean:.3f} m/s², σ={az_std:.3f} m/s²',
             transform=ax1.transAxes, verticalalignment='top',
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5),
             fontsize=9)

    # Plot angular velocity
    ax2.plot(data['timestamp'], data['gx'], label='gx', linewidth=0.8, alpha=0.7)
    ax2.plot(data['timestamp'], data['gy'], label='gy', linewidth=0.8, alpha=0.7)
    ax2.plot(data['timestamp'], data['gz'], label='gz', linewidth=0.8, alpha=0.7)
    ax2.set_xlabel('Time (s)', fontsize=11)
    ax2.set_ylabel('Angular Velocity (rad/s)', fontsize=11)
    ax2.legend(loc='upper right')
    ax2.grid(True, alpha=0.3)
    ax2.set_xlim(data['timestamp'].min(), data['timestamp'].max())

    # Add statistics to angular velocity plot
    gx_std = data['gx'].std()
    gy_std = data['gy'].std()
    gz_std = data['gz'].std()
    ax2.text(0.02, 0.98,
             f'Noise: σx={gx_std:.4f}, σy={gy_std:.4f}, σz={gz_std:.4f} rad/s',
             transform=ax2.transAxes, verticalalignment='top',
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5),
             fontsize=9)

    plt.tight_layout()

    # Save figure
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f'Plot saved to {output_file}')

    # Display statistics
    print('\nStatistics:')
    print(f'  Acceleration (m/s²):')
    print(f'    ax: μ={data["ax"].mean():.3f}, σ={data["ax"].std():.3f}')
    print(f'    ay: μ={data["ay"].mean():.3f}, σ={data["ay"].std():.3f}')
    print(f'    az: μ={data["az"].mean():.3f}, σ={data["az"].std():.3f} (gravity ~9.81)')
    print(f'  Angular velocity (rad/s):')
    print(f'    gx: μ={data["gx"].mean():.4f}, σ={data["gx"].std():.4f}')
    print(f'    gy: μ={data["gy"].mean():.4f}, σ={data["gy"].std():.4f}')
    print(f'    gz: μ={data["gz"].mean():.4f}, σ={data["gz"].std():.4f}')


def main():
    csv_file = sys.argv[1] if len(sys.argv) > 1 else 'imu_data.csv'
    output_file = sys.argv[2] if len(sys.argv) > 2 else '03-imu-plot.png'

    try:
        plot_imu_data(csv_file, output_file)
    except FileNotFoundError:
        print(f'Error: File not found: {csv_file}')
        print('Run export_imu_csv.py first to generate IMU data')
        sys.exit(1)
    except Exception as e:
        print(f'Error: {e}')
        sys.exit(1)


if __name__ == '__main__':
    main()
