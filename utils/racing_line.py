"""
Racing line generator for optimal path planning on race tracks.
Uses curvature-based optimization to create smooth, fast racing lines.
"""

import numpy as np
from scipy.interpolate import splprep, splev
from scipy.ndimage import gaussian_filter1d


class RacingLineGenerator:
    """
    Generate optimal racing line from track centerline.

    Racing line principles:
    - Enter corners wide (outside)
    - Hit apex (inside)
    - Exit wide (outside)
    - Minimize curvature for maximum speed
    """

    def __init__(self, centerline, track_width_left, track_width_right):
        """
        Initialize racing line generator.

        Parameters:
        -----------
        centerline : np.array
            Track centerline coordinates (N, 2)
        track_width_left : np.array
            Left track width at each point (N,)
        track_width_right : np.array
            Right track width at each point (N,)
        """
        self.centerline = centerline
        self.track_width_left = track_width_left
        self.track_width_right = track_width_right
        self.racing_line = None

    def generate(self, aggression=0.7, smoothing=5):
        """
        Generate racing line.

        Parameters:
        -----------
        aggression : float
            How aggressively to take corners (0=centerline, 1=maximum)
            0.7 means use 70% of available track width
        smoothing : int
            Smoothing factor for the racing line

        Returns:
        --------
        racing_line : np.array
            Optimized racing line coordinates (N, 2)
        """
        # Step 1: Calculate curvature along centerline
        curvature = self._calculate_curvature(self.centerline)

        # Step 2: Calculate optimal offset based on curvature
        # Positive offset = move right, negative = move left
        offset = self._calculate_racing_offset(curvature, aggression)

        # Step 3: Apply offset to centerline
        racing_line = self._apply_offset(self.centerline, offset)

        # Step 4: Smooth the racing line
        racing_line = self._smooth_path(racing_line, smoothing)

        # Step 5: Ensure racing line stays within track bounds
        racing_line = self._clip_to_track_bounds(racing_line)

        self.racing_line = racing_line
        return racing_line

    def _calculate_curvature(self, path):
        """
        Calculate curvature at each point along the path.

        Curvature κ = |x'y'' - y'x''| / (x'² + y'²)^(3/2)

        Parameters:
        -----------
        path : np.array
            Path coordinates (N, 2)

        Returns:
        --------
        curvature : np.array
            Signed curvature at each point (N,)
        """
        # Compute first derivatives (velocity)
        dx = np.gradient(path[:, 0])
        dy = np.gradient(path[:, 1])

        # Compute second derivatives (acceleration)
        ddx = np.gradient(dx)
        ddy = np.gradient(dy)

        # Curvature formula
        numerator = dx * ddy - dy * ddx
        denominator = (dx**2 + dy**2)**(3/2)

        # Avoid division by zero
        denominator = np.where(denominator == 0, 1e-10, denominator)

        curvature = numerator / denominator

        # Smooth curvature to reduce noise
        curvature = gaussian_filter1d(curvature, sigma=3, mode='wrap')

        return curvature

    def _calculate_racing_offset(self, curvature, aggression):
        """
        Calculate optimal lateral offset based on curvature.

        Racing line logic:
        - Left turn (positive curvature): offset right (positive) on entry/exit, left (negative) at apex
        - Right turn (negative curvature): offset left (negative) on entry/exit, right (positive) at apex

        Parameters:
        -----------
        curvature : np.array
            Curvature at each point
        aggression : float
            How much to use available track width (0-1)

        Returns:
        --------
        offset : np.array
            Lateral offset from centerline
        """
        # Normalize curvature to understand corner severity
        max_curvature = np.abs(curvature).max()
        if max_curvature == 0:
            return np.zeros_like(curvature)

        normalized_curvature = curvature / max_curvature

        # Calculate available track width at each point
        # Use the minimum of left/right to be conservative
        available_width = np.minimum(self.track_width_left, self.track_width_right)

        # Racing line strategy:
        # - High curvature sections (corners): go wide (opposite direction)
        # - This creates gentler effective curvature
        # - Use more aggressive offset for better curvature reduction

        # Identify corner regions (high curvature)
        curvature_magnitude = np.abs(normalized_curvature)

        # For corners, offset strongly in opposite direction to reduce curvature
        # For straights, stay on centerline
        offset = -normalized_curvature * available_width * aggression

        # Apply extra smoothing for very gentle transitions
        offset = gaussian_filter1d(offset, sigma=15, mode='wrap')

        # Second pass smoothing for even gentler curves
        offset = gaussian_filter1d(offset, sigma=5, mode='wrap')

        return offset

    def _apply_offset(self, path, offset):
        """
        Apply lateral offset to path.

        Parameters:
        -----------
        path : np.array
            Original path (N, 2)
        offset : np.array
            Lateral offset at each point (N,)

        Returns:
        --------
        offset_path : np.array
            Path with offset applied (N, 2)
        """
        offset_path = []

        for i in range(len(path)):
            # Calculate tangent direction
            if i < len(path) - 1:
                tangent = path[i + 1] - path[i]
            else:
                tangent = path[i] - path[i - 1]

            # Normalize tangent
            tangent_len = np.linalg.norm(tangent)
            if tangent_len > 0:
                tangent = tangent / tangent_len
            else:
                tangent = np.array([1, 0])

            # Perpendicular vector (normal)
            # Rotate tangent 90° counter-clockwise
            normal = np.array([-tangent[1], tangent[0]])

            # Apply offset
            offset_point = path[i] + normal * offset[i]
            offset_path.append(offset_point)

        return np.array(offset_path)

    def _smooth_path(self, path, smoothing):
        """
        Smooth path using spline interpolation.

        Parameters:
        -----------
        path : np.array
            Path to smooth (N, 2)
        smoothing : float
            Smoothing factor

        Returns:
        --------
        smoothed_path : np.array
            Smoothed path (N, 2)
        """
        if smoothing == 0:
            return path

        # Close the path for spline fitting
        closed_path = np.vstack([path, path[0]])

        try:
            # Fit spline
            tck, u = splprep([closed_path[:, 0], closed_path[:, 1]],
                            s=smoothing * len(path),
                            per=True)  # Periodic for closed loop

            # Evaluate spline at same number of points
            u_new = np.linspace(0, 1, len(path))
            x_new, y_new = splev(u_new, tck)

            smoothed_path = np.column_stack([x_new, y_new])

        except Exception as e:
            print(f"Warning: Spline smoothing failed ({e}), using original path")
            smoothed_path = path

        return smoothed_path

    def _clip_to_track_bounds(self, path):
        """
        Ensure racing line stays within track boundaries.

        Parameters:
        -----------
        path : np.array
            Racing line path (N, 2)

        Returns:
        --------
        clipped_path : np.array
            Path clipped to track bounds (N, 2)
        """
        clipped_path = []

        for i in range(len(path)):
            # Get current point
            point = path[i]

            # Calculate distance from centerline
            center = self.centerline[i]
            diff = point - center

            # Calculate normal direction
            if i < len(self.centerline) - 1:
                tangent = self.centerline[i + 1] - self.centerline[i]
            else:
                tangent = self.centerline[i] - self.centerline[i - 1]

            tangent_len = np.linalg.norm(tangent)
            if tangent_len > 0:
                tangent = tangent / tangent_len
                normal = np.array([-tangent[1], tangent[0]])

                # Project offset onto normal direction
                offset_dist = np.dot(diff, normal)

                # Clip to available width (with safety margin)
                safety_margin = 0.8  # Use 80% of available width
                max_left = self.track_width_left[i] * safety_margin
                max_right = self.track_width_right[i] * safety_margin

                # Clip offset
                offset_dist = np.clip(offset_dist, -max_right, max_left)

                # Apply clipped offset
                clipped_point = center + normal * offset_dist
            else:
                clipped_point = point

            clipped_path.append(clipped_point)

        return np.array(clipped_path)

    def visualize_comparison(self):
        """
        Create visualization comparing centerline and racing line.
        (For debugging/analysis)
        """
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(1, 2, figsize=(15, 6))

        # Plot 1: Both lines
        ax = axes[0]
        ax.plot(self.centerline[:, 0], self.centerline[:, 1],
                'orange', linewidth=2, label='Centerline', alpha=0.7)
        ax.plot(self.racing_line[:, 0], self.racing_line[:, 1],
                'blue', linewidth=2, label='Racing Line')
        ax.set_aspect('equal')
        ax.legend()
        ax.set_title('Track Lines')
        ax.grid(True, alpha=0.3)

        # Plot 2: Curvature comparison
        ax = axes[1]
        curvature_center = self._calculate_curvature(self.centerline)
        curvature_racing = self._calculate_curvature(self.racing_line)

        distance = np.arange(len(curvature_center))
        ax.plot(distance, np.abs(curvature_center),
                'orange', label='Centerline curvature', alpha=0.7)
        ax.plot(distance, np.abs(curvature_racing),
                'blue', label='Racing line curvature')
        ax.set_xlabel('Distance along track')
        ax.set_ylabel('Absolute curvature')
        ax.set_title('Curvature Comparison (Lower = Faster)')
        ax.legend()
        ax.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.show()


def generate_racing_line(centerline, track_width_left, track_width_right,
                        aggression=0.7, smoothing=5):
    """
    Convenience function to generate racing line.

    Parameters:
    -----------
    centerline : np.array
        Track centerline (N, 2)
    track_width_left : np.array
        Left track width (N,)
    track_width_right : np.array
        Right track width (N,)
    aggression : float
        How aggressively to use track width (0-1)
    smoothing : int
        Smoothing factor

    Returns:
    --------
    racing_line : np.array
        Optimized racing line (N, 2)
    """
    generator = RacingLineGenerator(centerline, track_width_left, track_width_right)
    return generator.generate(aggression, smoothing)
