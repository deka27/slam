"""
Clean Monza race track environment using extracted coordinates.
Uses real Monza track layout for SLAM simulation.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon


class MonzaTrack:
    def __init__(self, coords_file='environment/track_2d/monza_coords.npy'):
        # Track parameters
        self.track_width = 12.0  # meters (scale appropriately)

        # Load extracted centerline coordinates
        self.track_centerline = self._load_centerline(coords_file)

        # Scale to real-world dimensions (Monza is ~5.8km)
        self.track_centerline = self._scale_to_real_size(self.track_centerline)

        # Create inner and outer boundaries
        self.inner_boundary = []
        self.outer_boundary = []
        self._compute_boundaries()

    def _load_centerline(self, coords_file):
        """Load extracted centerline coordinates"""
        coords = np.load(coords_file)
        return coords

    def _scale_to_real_size(self, coords):
        """Scale normalized coordinates to real-world meters"""
        # Monza is approximately 5793 meters long
        # Scale the track so the total path length matches

        # Calculate current path length
        diff = np.diff(coords, axis=0)
        lengths = np.sqrt((diff**2).sum(axis=1))
        current_length = lengths.sum()

        # Scale factor to match Monza's real length
        scale_factor = 5793 / current_length

        # Apply scaling
        coords_scaled = coords * scale_factor

        return coords_scaled

    def _compute_boundaries(self):
        """Compute inner and outer track boundaries"""
        if len(self.track_centerline) < 2:
            return

        n_points = len(self.track_centerline)

        for i in range(n_points):
            p = self.track_centerline[i]

            # Calculate tangent direction
            if i < n_points - 1:
                next_p = self.track_centerline[i + 1]
                tangent = next_p - p
            else:
                prev_p = self.track_centerline[i - 1]
                tangent = p - prev_p

            # Normalize tangent
            tangent_len = np.linalg.norm(tangent)
            if tangent_len > 0:
                tangent = tangent / tangent_len
            else:
                tangent = np.array([1, 0])

            # Perpendicular (normal) vector
            normal = np.array([-tangent[1], tangent[0]])

            # Inner and outer points
            inner = p - normal * (self.track_width / 2)
            outer = p + normal * (self.track_width / 2)

            self.inner_boundary.append(inner)
            self.outer_boundary.append(outer)

        self.inner_boundary = np.array(self.inner_boundary)
        self.outer_boundary = np.array(self.outer_boundary)

    def visualize(self):
        """Visualize the track"""
        fig, ax = plt.subplots(figsize=(12, 8))

        # Fill track area
        if len(self.inner_boundary) > 0 and len(self.outer_boundary) > 0:
            track_polygon = np.vstack([
                self.outer_boundary,
                self.inner_boundary[::-1]
            ])
            track_patch = Polygon(track_polygon, facecolor='lightgray',
                                alpha=0.5, edgecolor='none')
            ax.add_patch(track_patch)

        # Plot track boundaries
        ax.plot(self.inner_boundary[:, 0], self.inner_boundary[:, 1],
                'r-', linewidth=3, label='Track boundaries')
        ax.plot(self.outer_boundary[:, 0], self.outer_boundary[:, 1],
                'r-', linewidth=3)

        # Plot centerline
        ax.plot(self.track_centerline[:, 0], self.track_centerline[:, 1],
                'k--', linewidth=1, alpha=0.5, label='Centerline')

        # Mark start/finish
        if len(self.track_centerline) > 0:
            ax.plot(self.track_centerline[0, 0], self.track_centerline[0, 1],
                   'go', markersize=12, label='Start/Finish', zorder=5)

        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.legend()
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title('Monza Track - Clean (Real Layout)')

        plt.tight_layout()
        plt.show()

    def get_track_boundaries(self):
        """Return track boundaries for SLAM"""
        return {
            'centerline': self.track_centerline,
            'inner_boundary': self.inner_boundary,
            'outer_boundary': self.outer_boundary,
            'track_width': self.track_width
        }


if __name__ == "__main__":
    track = MonzaTrack()
    print(f"Track loaded: {len(track.track_centerline)} centerline points")
    print(f"Track width: {track.track_width}m")
    track.visualize()
