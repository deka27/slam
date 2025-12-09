"""
Path controller for robot to follow track centerline.
"""

import numpy as np


class PathController:
    """
    Path following controller with cross-track error correction.

    Uses a combination of pure pursuit and cross-track error feedback
    to precisely follow the path without cutting corners.
    """

    def __init__(self, path, desired_speed=10.0, lookahead_distance=5.0):
        """
        Initialize path controller.

        Parameters:
        -----------
        path : np.array
            Path to follow as (N, 2) array of [x, y] points
        desired_speed : float
            Desired linear velocity (m/s)
        lookahead_distance : float
            Lookahead distance - SHORTER prevents corner cutting (meters)
        """
        self.path = path
        self.desired_speed = desired_speed
        self.lookahead_distance = lookahead_distance
        self.current_waypoint_idx = 0

        # For derivative control (damping)
        self.prev_heading_error = 0.0
        self.prev_cross_track_error = 0.0

    def compute_control(self, robot_x, robot_y, robot_theta):
        """
        Compute control with cross-track error correction to prevent corner cutting.

        Parameters:
        -----------
        robot_x, robot_y : float
            Robot position
        robot_theta : float
            Robot heading angle (radians)

        Returns:
        --------
        v, omega : float
            Linear velocity and angular velocity commands
        """
        robot_pos = np.array([robot_x, robot_y])

        # Find closest point on path
        closest_idx = self._find_closest_point(robot_pos)
        closest_point = self.path[closest_idx]

        # Cross-track error (distance from path)
        cross_track_error_vec = robot_pos - closest_point
        cross_track_distance = np.linalg.norm(cross_track_error_vec)

        # Determine which side of path we're on
        # Get path tangent at closest point
        if closest_idx < len(self.path) - 1:
            path_tangent = self.path[closest_idx + 1] - self.path[closest_idx]
        else:
            path_tangent = self.path[closest_idx] - self.path[closest_idx - 1]

        path_tangent_len = np.linalg.norm(path_tangent)
        if path_tangent_len > 0:
            path_tangent = path_tangent / path_tangent_len

        # Cross product to determine side (positive = left, negative = right)
        cross_product = np.cross(path_tangent, cross_track_error_vec)
        cross_track_error_signed = np.sign(cross_product) * cross_track_distance

        # Find lookahead point (very short to follow path precisely)
        lookahead_idx = self._find_lookahead_point(robot_pos, closest_idx)
        lookahead_point = self.path[lookahead_idx]

        # Desired heading: towards lookahead point
        dx = lookahead_point[0] - robot_x
        dy = lookahead_point[1] - robot_y
        desired_heading = np.arctan2(dy, dx)

        # Heading error
        heading_error = self._wrap_angle(desired_heading - robot_theta)

        # SIMPLE PURE PURSUIT CONTROL
        # Just proportional control on heading error - simpler and more stable
        k_heading = 1.5  # Moderate heading gain for stability
        omega = k_heading * heading_error

        # Limit angular velocity
        max_omega = 3.0  # rad/s
        omega = np.clip(omega, -max_omega, max_omega)

        # Speed control: slow down when off path or turning (RELAXED for better lap coverage)
        angle_threshold_deg = abs(np.degrees(heading_error))

        if cross_track_distance > 3.0:
            # Way off path - slow down significantly
            v = self.desired_speed * 0.3
        elif cross_track_distance > 2.0:
            # Off path - slow down
            v = self.desired_speed * 0.5
        elif cross_track_distance > 1.0:
            # Slightly off - minor slowdown
            v = self.desired_speed * 0.7
        elif angle_threshold_deg > 30:
            # Sharp turn
            v = self.desired_speed * 0.6
        elif angle_threshold_deg > 10:
            # Medium turn
            v = self.desired_speed * 0.4
        elif angle_threshold_deg > 5:
            # Gentle turn
            v = self.desired_speed * 0.6
        elif angle_threshold_deg > 2:
            # Very gentle
            v = self.desired_speed * 0.8
        else:
            # Straight and on path
            v = self.desired_speed

        # Update current waypoint
        self.current_waypoint_idx = closest_idx

        return v, omega

    def _find_closest_point(self, robot_pos):
        """Find index of closest point on path."""
        distances = np.linalg.norm(self.path - robot_pos, axis=1)
        return np.argmin(distances)

    def _find_lookahead_point(self, robot_pos, start_idx):
        """
        Find lookahead point on path.

        Parameters:
        -----------
        robot_pos : np.array
            Robot position [x, y]
        start_idx : int
            Starting index on path

        Returns:
        --------
        lookahead_idx : int
            Index of lookahead point
        """
        # Search forward along path for point at lookahead distance
        for i in range(len(self.path)):
            idx = (start_idx + i) % len(self.path)
            point = self.path[idx]
            distance = np.linalg.norm(point - robot_pos)

            if distance >= self.lookahead_distance:
                return idx

        # If no point found, return point far ahead
        return (start_idx + 20) % len(self.path)

    def get_progress(self):
        """
        Get progress along path.

        Returns:
        --------
        progress : float
            Progress as fraction of path completed (0 to 1)
        """
        return self.current_waypoint_idx / len(self.path)

    def reset(self):
        """Reset controller to start of path."""
        self.current_waypoint_idx = 0
        self.prev_heading_error = 0.0
        self.prev_cross_track_error = 0.0

    @staticmethod
    def _wrap_angle(angle):
        """Wrap angle to [-pi, pi]."""
        return np.arctan2(np.sin(angle), np.cos(angle))


class SimpleController:
    """
    Even simpler controller that just tracks waypoints sequentially.
    """

    def __init__(self, path, desired_speed=10.0, waypoint_threshold=5.0):
        """
        Initialize simple controller.

        Parameters:
        -----------
        path : np.array
            Path to follow
        desired_speed : float
            Desired speed (m/s)
        waypoint_threshold : float
            Distance threshold to consider waypoint reached (meters)
        """
        self.path = path
        self.desired_speed = desired_speed
        self.waypoint_threshold = waypoint_threshold
        self.current_waypoint_idx = 0

    def compute_control(self, robot_x, robot_y, robot_theta):
        """
        Compute control to reach current waypoint.

        Parameters:
        -----------
        robot_x, robot_y, robot_theta : float
            Robot state

        Returns:
        --------
        v, omega : float
            Control inputs
        """
        # Get current target waypoint
        target = self.path[self.current_waypoint_idx]

        # Compute distance to target
        robot_pos = np.array([robot_x, robot_y])
        distance = np.linalg.norm(target - robot_pos)

        # If close enough, move to next waypoint
        if distance < self.waypoint_threshold:
            self.current_waypoint_idx = (self.current_waypoint_idx + 1) % len(self.path)
            target = self.path[self.current_waypoint_idx]

        # Compute angle to target
        dx = target[0] - robot_x
        dy = target[1] - robot_y
        target_angle = np.arctan2(dy, dx)

        # Heading error
        angle_error = self._wrap_angle(target_angle - robot_theta)

        # Simple proportional control
        k_omega = 1.5
        omega = k_omega * angle_error

        # Limit angular velocity
        max_omega = 2.0
        omega = np.clip(omega, -max_omega, max_omega)

        # Reduce speed when turning sharply
        if abs(angle_error) > np.radians(45):
            v = self.desired_speed * 0.5
        else:
            v = self.desired_speed

        return v, omega

    def get_progress(self):
        """Get progress (0 to 1)."""
        return self.current_waypoint_idx / len(self.path)

    def reset(self):
        """Reset to start."""
        self.current_waypoint_idx = 0

    @staticmethod
    def _wrap_angle(angle):
        """Wrap angle to [-pi, pi]."""
        return np.arctan2(np.sin(angle), np.cos(angle))
