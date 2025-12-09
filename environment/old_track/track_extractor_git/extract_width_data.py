"""
Quick script to extract track width data from TUMFTM database.
No visualization - just data extraction and saving.
"""

import numpy as np
import requests
from io import StringIO
from pathlib import Path


def extract_monza_with_width():
    """Extract Monza track data including width information."""

    print("=" * 60)
    print("Extracting Monza track with width data...")
    print("=" * 60)

    # Download data
    url = "https://raw.githubusercontent.com/TUMFTM/racetrack-database/master/tracks/Monza.csv"

    try:
        print(f"\n1. Downloading from TUMFTM database...")
        resp = requests.get(url, timeout=10)
        resp.raise_for_status()
        csv_text = resp.text

        # CSV format: x_m, y_m, w_tr_right_m, w_tr_left_m
        data = np.loadtxt(StringIO(csv_text), delimiter=",", skiprows=1)

        print(f"   ✓ Downloaded {len(data)} track points")

    except Exception as e:
        print(f"   ✗ Failed: {e}")
        return False

    # Extract data
    x_m = data[:, 0]
    y_m = data[:, 1]
    width_right = data[:, 2]
    width_left = data[:, 3]

    print(f"\n2. Processing track data...")
    print(f"   Track width range:")
    print(f"   - Left:  {width_left.min():.1f}m to {width_left.max():.1f}m")
    print(f"   - Right: {width_right.min():.1f}m to {width_right.max():.1f}m")
    print(f"   - Total: {(width_left + width_right).min():.1f}m to {(width_left + width_right).max():.1f}m")

    # Process coordinates (same as original)
    x = x_m - x_m.mean()
    y = y_m - y_m.min()

    # Rotate -7 degrees
    angle_deg = -7.0
    theta = np.deg2rad(angle_deg)
    xr = x * np.cos(theta) - y * np.sin(theta)
    yr = x * np.sin(theta) + y * np.cos(theta)

    # Normalize to [0, 1]
    xr -= xr.min()
    yr -= yr.min()
    scale = max(np.ptp(xr), np.ptp(yr))
    xr /= scale
    yr /= scale

    # Create output arrays
    track_2d = np.column_stack([xr, yr])
    track_3d = np.column_stack([xr, yr, np.zeros_like(xr)])
    track_width = np.column_stack([width_left, width_right])

    print(f"   ✓ Normalized coordinates to [0, 1] range")

    # Save files
    print(f"\n3. Saving files to ../track_npy/...")
    # Use absolute path relative to this script
    script_dir = Path(__file__).parent
    output_dir = script_dir.parent / 'track_npy'
    output_dir.mkdir(parents=True, exist_ok=True)
    print(f"   Output directory: {output_dir.absolute()}")

    # Save 2D (overwrite existing)
    np.save(output_dir / 'monza_track_2d.npy', track_2d)
    print(f"   ✓ monza_track_2d.npy ({track_2d.shape})")

    # Save 3D (overwrite existing)
    np.save(output_dir / 'monza_track_3d.npy', track_3d)
    print(f"   ✓ monza_track_3d.npy ({track_3d.shape})")

    # Save width data (NEW)
    np.save(output_dir / 'monza_track_width.npy', track_width)
    print(f"   ✓ monza_track_width.npy ({track_width.shape}) [NEW]")

    # Save complete metadata (NEW)
    metadata = {
        'centerline_2d': track_2d,
        'centerline_3d': track_3d,
        'width_left': width_left,
        'width_right': width_right,
        'track_name': 'monza',
        'rotation_deg': angle_deg,
        'n_points': len(track_2d),
        'source': 'TUMFTM racetrack-database',
        'url': url
    }
    np.savez(output_dir / 'monza_metadata.npz', **metadata)
    print(f"   ✓ monza_metadata.npz [NEW]")

    print(f"\n" + "=" * 60)
    print("✓ Extraction complete!")
    print("=" * 60)

    return True


if __name__ == "__main__":
    extract_monza_with_width()
