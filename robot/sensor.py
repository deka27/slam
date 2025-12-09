"""
Range-bearing sensor for landmark detection.
Used for EKF-SLAM.
"""

import numpy as np


class RangeBearingSensor:
    """
    Range-bearing sensor that detects landmarks.

    Measurements:
    - Range: distance to landmark (meters)
    - Bearing: angle from robot heading to landmark (radians)
    """

    def __init__(self, max_range=50.0, field_of_view=np.pi,
                 range_noise_std=0.1, bearing_noise_std=0.05):
        """
        Initialize range-bearing sensor.

        Parameters:
        -----------
        max_range : float
            Maximum detection range (meters)
        field_of_view : float
            Field of view angle (radians), centered on robot heading
        range_noise_std : float
            Standard deviation of range measurement noise (meters)
        bearing_noise_std : float
            Standard deviation of bearing measurement noise (radians)
        """
        self.max_range = max_range
        self.field_of_view = field_of_view
        self.range_noise_std = range_noise_std
        self.bearing_noise_std = bearing_noise_std

        # Measurement noise covariance matrix
        self.Q = np.array([
            [range_noise_std**2, 0],
            [0, bearing_noise_std**2]
        ])

    def measure(self, robot_x, robot_y, robot_theta, landmarks, add_noise=True):
        """
        Measure range and bearing to visible landmarks.

        Parameters:
        -----------
        robot_x, robot_y : float
            Robot position
        robot_theta : float
            Robot heading (radians)
        landmarks : list of dict
            List of landmarks, each with 'id' and 'position' [x, y]
        add_noise : bool
            Whether to add measurement noise

        Returns:
        --------
        measurements : list of dict
            List of measurements, each containing:
            - 'landmark_id': landmark identifier
            - 'range': measured distance (meters)
            - 'bearing': measured angle (radians, relative to robot heading)
            - 'position': landmark position [x, y] (for visualization)
        """
        measurements = []

        robot_pos = np.array([robot_x, robot_y])

        for landmark in landmarks:
            landmark_id = landmark['id']
            landmark_pos = np.array(landmark['position'])

            # Calculate true range and bearing
            diff = landmark_pos - robot_pos
            true_range = np.linalg.norm(diff)

            # Check if within max range
            if true_range > self.max_range:
                continue

            # Calculate bearing (angle from robot heading to landmark)
            angle_to_landmark = np.arctan2(diff[1], diff[0])
            true_bearing = self._wrap_angle(angle_to_landmark - robot_theta)

            # Check if within field of view
            if abs(true_bearing) > self.field_of_view / 2:
                continue

            # Add measurement noise if requested
            if add_noise:
                measured_range = true_range + np.random.normal(0, self.range_noise_std)
                measured_bearing = true_bearing + np.random.normal(0, self.bearing_noise_std)
            else:
                measured_range = true_range
                measured_bearing = true_bearing

            measurements.append({
                'landmark_id': landmark_id,
                'range': measured_range,
                'bearing': measured_bearing,
                'position': landmark_pos.tolist()
            })

        return measurements

    def measurement_model(self, robot_x, robot_y, robot_theta, landmark_x, landmark_y):
        """
        Expected measurement for a landmark given robot pose.
        Used for EKF update step.

        Parameters:
        -----------
        robot_x, robot_y, robot_theta : float
            Robot pose
        landmark_x, landmark_y : float
            Landmark position

        Returns:
        --------
        h : np.array
            Expected measurement [range, bearing]
        """
        dx = landmark_x - robot_x
        dy = landmark_y - robot_y

        expected_range = np.sqrt(dx**2 + dy**2)
        expected_bearing = self._wrap_angle(np.arctan2(dy, dx) - robot_theta)

        return np.array([expected_range, expected_bearing])

    def measurement_jacobian(self, robot_x, robot_y, robot_theta, landmark_x, landmark_y):
        """
        Jacobian of measurement model with respect to robot pose.
        Used for EKF update step.

        Parameters:
        -----------
        robot_x, robot_y, robot_theta : float
            Robot pose
        landmark_x, landmark_y : float
            Landmark position

        Returns:
        --------
        H : np.array (2, 3)
            Measurement Jacobian matrix
        """
        dx = landmark_x - robot_x
        dy = landmark_y - robot_y
        q = dx**2 + dy**2
        sqrt_q = np.sqrt(q)

        # Jacobian of [range, bearing] with respect to [x, y, theta]
        H = np.array([
            [-dx/sqrt_q, -dy/sqrt_q, 0],
            [dy/q, -dx/q, -1]
        ])

        return H

    @staticmethod
    def _wrap_angle(angle):
        """Wrap angle to [-pi, pi]."""
        return np.arctan2(np.sin(angle), np.cos(angle))
