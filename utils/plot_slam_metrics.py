"""
Plot SLAM performance metrics from CSV file.
"""

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from pathlib import Path


def plot_slam_metrics(csv_file='slam_metrics.csv'):
    """
    Create comprehensive plots of SLAM performance metrics.

    Parameters:
    -----------
    csv_file : str
        Name of the CSV file in logs/ directory
    """
    # Load data
    script_dir = Path(__file__).parent.parent
    filepath = script_dir / 'logs' / csv_file

    if not filepath.exists():
        print(f"Error: Metrics file not found at {filepath}")
        return

    df = pd.read_csv(filepath)

    # Find lap completion times (when lap count increases)
    lap_times = []
    if 'Laps Completed' in df.columns:
        lap_changes = df[df['Laps Completed'].diff() > 0]
        lap_times = lap_changes['Time (s)'].tolist()

    # Find loop closure times
    loop_closure_times = []
    if 'Loop Closures' in df.columns:
        closure_changes = df[df['Loop Closures'].diff() > 0]
        loop_closure_times = closure_changes['Time (s)'].tolist()

    # Helper function to add lap markers
    def add_lap_markers(ax):
        for lap_time in lap_times:
            ax.axvline(x=lap_time, color='green', linestyle='--', alpha=0.5, linewidth=1.5)
        for closure_time in loop_closure_times:
            ax.axvline(x=closure_time, color='orange', linestyle=':', alpha=0.7, linewidth=2)

    # Create figure with subplots
    fig, axes = plt.subplots(3, 2, figsize=(15, 12))
    title = f'EKF-SLAM Performance Metrics'
    if len(lap_times) > 0:
        title += f' ({len(lap_times)} laps, {len(loop_closure_times)} loop closures)'
    fig.suptitle(title, fontsize=16, fontweight='bold')

    # 1. Position Error over time
    ax = axes[0, 0]
    ax.plot(df['Time (s)'], df['Position Error (m)'], 'b-', linewidth=2, label='Position Error')
    ax.fill_between(df['Time (s)'], 0, df['Position Error (m)'], alpha=0.3)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Error (m)')
    ax.set_title('Position Error (Euclidean Distance)')
    ax.grid(True, alpha=0.3)
    add_lap_markers(ax)  # Add lap markers
    ax.legend()

    # Add statistics
    mean_error = df['Position Error (m)'].mean()
    max_error = df['Position Error (m)'].max()
    final_error = df['Position Error (m)'].iloc[-1]
    ax.text(0.02, 0.98, f'Mean: {mean_error:.2f}m\nMax: {max_error:.2f}m\nFinal: {final_error:.2f}m',
            transform=ax.transAxes, verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

    # 2. X and Y Errors
    ax = axes[0, 1]
    ax.plot(df['Time (s)'], df['X Error (m)'], 'r-', linewidth=2, label='X Error', alpha=0.7)
    ax.plot(df['Time (s)'], df['Y Error (m)'], 'g-', linewidth=2, label='Y Error', alpha=0.7)
    ax.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Error (m)')
    ax.set_title('X and Y Position Errors')
    ax.grid(True, alpha=0.3)
    add_lap_markers(ax)  # Add lap markers
    ax.legend()

    # 3. Heading Error
    ax = axes[1, 0]
    ax.plot(df['Time (s)'], df['Theta Error (deg)'], 'purple', linewidth=2)
    ax.fill_between(df['Time (s)'], 0, df['Theta Error (deg)'], alpha=0.3, color='purple')
    ax.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Error (degrees)')
    ax.set_title('Heading Error')
    ax.grid(True, alpha=0.3)
    add_lap_markers(ax)  # Add lap markers

    mean_theta_error = df['Theta Error (deg)'].abs().mean()
    max_theta_error = df['Theta Error (deg)'].abs().max()
    ax.text(0.02, 0.98, f'Mean: {mean_theta_error:.2f}°\nMax: {max_theta_error:.2f}°',
            transform=ax.transAxes, verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

    # 4. Uncertainty over time
    ax = axes[1, 1]
    ax.plot(df['Time (s)'], df['Uncertainty X (m)'], 'r--', linewidth=2, label='σ_x', alpha=0.7)
    ax.plot(df['Time (s)'], df['Uncertainty Y (m)'], 'g--', linewidth=2, label='σ_y', alpha=0.7)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Uncertainty (m)')
    ax.set_title('Position Uncertainty (1-sigma)')
    ax.grid(True, alpha=0.3)
    add_lap_markers(ax)  # Add lap markers
    ax.legend()

    # 5. Landmarks
    ax = axes[2, 0]
    ax.plot(df['Time (s)'], df['Landmarks Mapped'], 'b-', linewidth=2, label='Mapped')
    ax.plot(df['Time (s)'], df['Landmarks Detected'], 'orange', linewidth=1.5, alpha=0.7, label='Detected')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Count')
    ax.set_title('Landmark Statistics')
    ax.grid(True, alpha=0.3)
    add_lap_markers(ax)  # Add lap markers
    ax.legend()

    # 6. Error vs Uncertainty (Consistency Check)
    ax = axes[2, 1]
    ax.plot(df['Time (s)'], df['Position Error (m)'], 'b-', linewidth=2, label='Position Error')

    # Plot 1-sigma, 2-sigma, 3-sigma bounds
    avg_uncertainty = (df['Uncertainty X (m)'] + df['Uncertainty Y (m)']) / 2
    ax.plot(df['Time (s)'], avg_uncertainty, 'g--', linewidth=1.5, label='1σ bound', alpha=0.7)
    ax.plot(df['Time (s)'], 2 * avg_uncertainty, 'y--', linewidth=1.5, label='2σ bound', alpha=0.7)
    ax.plot(df['Time (s)'], 3 * avg_uncertainty, 'r--', linewidth=1.5, label='3σ bound', alpha=0.7)

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Error / Uncertainty (m)')
    ax.set_title('Filter Consistency (Error vs Uncertainty)')
    ax.grid(True, alpha=0.3)
    add_lap_markers(ax)  # Add lap markers
    ax.legend()

    # Calculate consistency percentage (error within 2-sigma)
    within_2sigma = (df['Position Error (m)'] <= 2 * avg_uncertainty).sum()
    consistency = within_2sigma / len(df) * 100
    ax.text(0.02, 0.98, f'Within 2σ: {consistency:.1f}%',
            transform=ax.transAxes, verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='lightgreen' if consistency > 90 else 'lightyellow', alpha=0.5))

    plt.tight_layout()

    # Save figure
    output_path = script_dir / 'logs' / 'slam_metrics_plot.png'
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"\nPlot saved to: {output_path}")

    # Show plot
    plt.show()

    # Print summary statistics
    print("\n" + "="*60)
    print("SLAM PERFORMANCE SUMMARY")
    print("="*60)
    print(f"Duration: {df['Time (s)'].iloc[-1]:.1f}s")
    print(f"\nPosition Error:")
    print(f"  Mean:  {df['Position Error (m)'].mean():.3f}m")
    print(f"  Max:   {df['Position Error (m)'].max():.3f}m")
    print(f"  Final: {df['Position Error (m)'].iloc[-1]:.3f}m")
    print(f"\nHeading Error:")
    print(f"  Mean:  {df['Theta Error (deg)'].abs().mean():.2f}°")
    print(f"  Max:   {df['Theta Error (deg)'].abs().max():.2f}°")
    print(f"  Final: {abs(df['Theta Error (deg)'].iloc[-1]):.2f}°")
    print(f"\nLandmarks:")
    print(f"  Total Mapped: {df['Landmarks Mapped'].iloc[-1]}")
    print(f"  Avg Detected: {df['Landmarks Detected'].mean():.1f}")
    print(f"\nFilter Consistency:")
    print(f"  Within 2σ: {consistency:.1f}%")
    print("="*60)


if __name__ == "__main__":
    import sys

    # Allow specifying CSV file as command line argument
    if len(sys.argv) > 1:
        csv_file = sys.argv[1]
    else:
        csv_file = 'slam_metrics.csv'

    plot_slam_metrics(csv_file)
