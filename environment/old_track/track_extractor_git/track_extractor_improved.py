"""
Improved track extraction from TUMFTM racetrack database.
Downloads accurate GPS/surveyed racing line data with variable track width.
"""

import numpy as np
import matplotlib.pyplot as plt
import requests
from io import StringIO
from pathlib import Path


class TrackExtractor:
    """Extract and process racing track data from TUMFTM database."""

    AVAILABLE_TRACKS = {
        'monza': 'https://raw.githubusercontent.com/TUMFTM/racetrack-database/master/tracks/Monza.csv',
        'spa': 'https://raw.githubusercontent.com/TUMFTM/racetrack-database/master/tracks/Spa.csv',
        'silverstone': 'https://raw.githubusercontent.com/TUMFTM/racetrack-database/master/tracks/Silverstone.csv',
        'barcelona': 'https://raw.githubusercontent.com/TUMFTM/racetrack-database/master/tracks/Barcelona.csv',
    }

    def __init__(self, track_name='monza', rotation_deg=0.0):
        """
        Initialize track extractor.

        Parameters:
        -----------
        track_name : str
            Name of track to extract (monza, spa, silverstone, barcelona)
        rotation_deg : float
            Rotation angle in degrees for visual alignment
        """
        self.track_name = track_name.lower()
        self.rotation_deg = rotation_deg
        self.raw_data = None
        self.centerline = None
        self.track_width_left = None
        self.track_width_right = None

    def download_track_data(self):
        """Download track data from GitHub repository."""
        if self.track_name not in self.AVAILABLE_TRACKS:
            raise ValueError(
                f"Track '{self.track_name}' not available. "
                f"Choose from: {list(self.AVAILABLE_TRACKS.keys())}"
            )

        url = self.AVAILABLE_TRACKS[self.track_name]

        try:
            print(f"Downloading {self.track_name} track data from TUMFTM database...")
            resp = requests.get(url, timeout=10)
            resp.raise_for_status()
            csv_text = resp.text

            # CSV format: x_m, y_m, w_tr_right_m, w_tr_left_m
            self.raw_data = np.loadtxt(
                StringIO(csv_text),
                delimiter=",",
                skiprows=1
            )

            print(f"✓ Downloaded {len(self.raw_data)} track points")
            return True

        except requests.RequestException as e:
            print(f"✗ Failed to download track data: {e}")
            return False

    def process_track_data(self, normalize=True):
        """
        Process raw track data with normalization and rotation.

        Parameters:
        -----------
        normalize : bool
            Whether to normalize coordinates to [0, 1] range
        """
        if self.raw_data is None:
            raise ValueError("No track data loaded. Call download_track_data() first.")

        x_m = self.raw_data[:, 0]
        y_m = self.raw_data[:, 1]
        self.track_width_right = self.raw_data[:, 2]
        self.track_width_left = self.raw_data[:, 3]

        # Center the track
        x = x_m - x_m.mean()
        y = y_m - y_m.min()

        # Apply rotation if specified
        if self.rotation_deg != 0.0:
            theta = np.deg2rad(self.rotation_deg)
            xr = x * np.cos(theta) - y * np.sin(theta)
            yr = x * np.sin(theta) + y * np.cos(theta)
            x, y = xr, yr

        # Normalize to [0, 1] box while preserving aspect ratio
        if normalize:
            x -= x.min()
            y -= y.min()
            scale = max(np.ptp(x), np.ptp(y))
            x /= scale
            y /= scale

        self.centerline = np.column_stack([x, y])

        print(f"✓ Processed track data")
        print(f"  Points: {len(self.centerline)}")
        print(f"  Bounds: x=[{x.min():.3f}, {x.max():.3f}], y=[{y.min():.3f}, {y.max():.3f}]")
        print(f"  Width range: {self.track_width_left.min():.1f}m - {self.track_width_left.max():.1f}m (left)")
        print(f"               {self.track_width_right.min():.1f}m - {self.track_width_right.max():.1f}m (right)")

    def compute_boundaries(self):
        """Compute inner and outer track boundaries using variable width."""
        if self.centerline is None:
            raise ValueError("No processed track data. Call process_track_data() first.")

        inner_boundary = []
        outer_boundary = []

        n_points = len(self.centerline)

        for i in range(n_points):
            p = self.centerline[i]

            # Calculate tangent vector
            if i < n_points - 1:
                next_p = self.centerline[i + 1]
                tangent = next_p - p
            else:
                prev_p = self.centerline[i - 1]
                tangent = p - prev_p

            # Normalize tangent
            tangent_len = np.linalg.norm(tangent)
            if tangent_len > 0:
                tangent = tangent / tangent_len
            else:
                tangent = np.array([1, 0])

            # Perpendicular vector (normal)
            perpendicular = np.array([-tangent[1], tangent[0]])

            # Use actual track width from data
            width_left = self.track_width_left[i]
            width_right = self.track_width_right[i]

            # Calculate boundary points
            inner = p - perpendicular * width_right
            outer = p + perpendicular * width_left

            inner_boundary.append(inner)
            outer_boundary.append(outer)

        return np.array(inner_boundary), np.array(outer_boundary)

    def visualize(self, show_width_variation=True):
        """Visualize the extracted track with optional width variation."""
        if self.centerline is None:
            raise ValueError("No processed track data. Call process_track_data() first.")

        fig, axes = plt.subplots(1, 2 if show_width_variation else 1,
                                 figsize=(15, 6) if show_width_variation else (10, 6))

        if not show_width_variation:
            axes = [axes]

        # Plot 1: Track outline
        ax = axes[0]

        # Compute boundaries for visualization
        inner, outer = self.compute_boundaries()

        # Plot boundaries
        ax.fill(outer[:, 0], outer[:, 1], color='#2d5016', alpha=0.3, label='Run-off area')
        ax.fill(inner[:, 0], inner[:, 1], color='#7fc97f', alpha=0.5)

        # Plot track surface
        ax.plot(self.centerline[:, 0], self.centerline[:, 1],
                linewidth=8, color='#f89f1b', solid_capstyle='round',
                label='Track edge')
        ax.plot(self.centerline[:, 0], self.centerline[:, 1],
                linewidth=4, color='#222222', solid_capstyle='round',
                label='Racing line')

        # Mark start/finish
        ax.plot(self.centerline[0, 0], self.centerline[0, 1],
                'r*', markersize=15, label='Start/Finish')

        ax.set_aspect('equal', 'box')
        ax.axis('off')
        ax.set_title(f'{self.track_name.title()} - Racing Line', fontsize=14, pad=12)
        ax.legend(loc='upper right')

        # Plot 2: Width variation
        if show_width_variation:
            ax2 = axes[1]

            # Calculate distance along track
            diff = np.diff(self.centerline, axis=0)
            segment_lengths = np.sqrt((diff**2).sum(axis=1))
            distance = np.concatenate([[0], np.cumsum(segment_lengths)])

            total_width = self.track_width_left + self.track_width_right

            ax2.plot(distance, total_width, 'b-', linewidth=2, label='Total width')
            ax2.fill_between(distance, total_width, alpha=0.3)
            ax2.axhline(total_width.mean(), color='r', linestyle='--',
                       label=f'Average: {total_width.mean():.1f}m')

            ax2.set_xlabel('Distance along track (normalized)', fontsize=11)
            ax2.set_ylabel('Track width (m)', fontsize=11)
            ax2.set_title('Track Width Variation', fontsize=14, pad=12)
            ax2.grid(True, alpha=0.3)
            ax2.legend()

        plt.tight_layout()
        plt.show()

    def save(self, output_dir='../track_npy'):
        """
        Save processed track data to numpy files.

        Parameters:
        -----------
        output_dir : str
            Directory to save output files
        """
        if self.centerline is None:
            raise ValueError("No processed track data. Call process_track_data() first.")

        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)

        # Save 2D centerline
        track_2d_path = output_path / f'{self.track_name}_track_2d.npy'
        np.save(track_2d_path, self.centerline)
        print(f"✓ Saved 2D track: {track_2d_path}")

        # Save 3D centerline (z=0)
        track_3d = np.column_stack([self.centerline, np.zeros(len(self.centerline))])
        track_3d_path = output_path / f'{self.track_name}_track_3d.npy'
        np.save(track_3d_path, track_3d)
        print(f"✓ Saved 3D track: {track_3d_path}")

        # Save track width data
        width_data = np.column_stack([self.track_width_left, self.track_width_right])
        width_path = output_path / f'{self.track_name}_track_width.npy'
        np.save(width_path, width_data)
        print(f"✓ Saved track width: {width_path}")

        # Save complete metadata
        metadata = {
            'centerline_2d': self.centerline,
            'centerline_3d': track_3d,
            'width_left': self.track_width_left,
            'width_right': self.track_width_right,
            'track_name': self.track_name,
            'rotation_deg': self.rotation_deg,
            'n_points': len(self.centerline)
        }
        metadata_path = output_path / f'{self.track_name}_metadata.npz'
        np.savez(metadata_path, **metadata)
        print(f"✓ Saved metadata: {metadata_path}")

        return {
            'track_2d': track_2d_path,
            'track_3d': track_3d_path,
            'width': width_path,
            'metadata': metadata_path
        }


def extract_track(track_name='monza', rotation_deg=-7.0, visualize=True, save_output=True):
    """
    Convenience function to extract and process a track.

    Parameters:
    -----------
    track_name : str
        Name of track (monza, spa, silverstone, barcelona)
    rotation_deg : float
        Rotation angle for visual alignment
    visualize : bool
        Whether to show visualization
    save_output : bool
        Whether to save output files

    Returns:
    --------
    TrackExtractor instance
    """
    extractor = TrackExtractor(track_name=track_name, rotation_deg=rotation_deg)

    if not extractor.download_track_data():
        return None

    extractor.process_track_data()

    if visualize:
        extractor.visualize(show_width_variation=True)

    if save_output:
        extractor.save()

    return extractor


if __name__ == "__main__":
    # Extract Monza with slight rotation for better visual alignment
    print("=" * 60)
    print("TRACK EXTRACTOR - TUMFTM Racetrack Database")
    print("=" * 60)

    extractor = extract_track(
        track_name='monza',
        rotation_deg=-7.0,
        visualize=True,
        save_output=True
    )

    if extractor:
        print("\n" + "=" * 60)
        print("✓ Extraction complete!")
        print("=" * 60)

        # Show available tracks
        print("\nAvailable tracks for extraction:")
        for track in TrackExtractor.AVAILABLE_TRACKS.keys():
            print(f"  - {track}")
