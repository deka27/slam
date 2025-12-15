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
                 range_noise_std=0.1, bearing_noise_std=0.05, noise_type='gaussian'):
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
        noise_type : str
            Type of noise distribution: 'gaussian', 'uniform', 'heavy_tailed',
            'laplacian', 'bimodal', 'asymmetric'
        """
        self.max_range = max_range
        self.field_of_view = field_of_view
        self.range_noise_std = range_noise_std
        self.bearing_noise_std = bearing_noise_std
        self.noise_type = noise_type

        # Measurement noise covariance matrix (always Gaussian for EKF)
        self.Q = np.array([
            [range_noise_std**2, 0],
            [0, bearing_noise_std**2]
        ])

    def _generate_noise(self, std):
        """
        Generate noise sample based on configured noise type.

        Parameters:
        -----------
        std : float
            Standard deviation (or scale parameter)

        Returns:
        --------
        noise : float
            Noise sample
        """
        if self.noise_type == 'gaussian':
            return np.random.normal(0, std)

        elif self.noise_type == 'uniform':
            # Uniform distribution with same variance as Gaussian
            # For uniform(-a, a), variance = a^2/3, so a = std*sqrt(3)
            a = std * np.sqrt(3)
            return np.random.uniform(-a, a)

        elif self.noise_type == 'heavy_tailed':
            # Student's t-distribution with df=3 (heavier tails)
            # Scale to match std
            return np.random.standard_t(df=3) * std / np.sqrt(3)

        elif self.noise_type == 'laplacian':
            # Laplace distribution (sharper peak, heavier tails)
            # For Laplace(0, b), std = b*sqrt(2), so b = std/sqrt(2)
            b = std / np.sqrt(2)
            return np.random.laplace(0, b)

        elif self.noise_type == 'bimodal':
            # Mixture: 80% N(0, std) + 20% N(0, 5*std)
            if np.random.rand() < 0.8:
                return np.random.normal(0, std)
            else:
                return np.random.normal(0, 5 * std)

        elif self.noise_type == 'asymmetric':
            # Exponential distribution (always positive) - mean shifted to 0
            # For Exp(lambda), std = 1/lambda
            scale = std
            return np.random.exponential(scale) - scale  # Shift to zero mean

        else:
            # Default to Gaussian
            return np.random.normal(0, std)

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
                measured_range = true_range + self._generate_noise(self.range_noise_std)
                measured_bearing = true_bearing + self._generate_noise(self.bearing_noise_std)
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
