"""
Extended Kalman Filter SLAM implementation.

State vector: [x, y, theta, lm1_x, lm1_y, lm2_x, lm2_y, ...]
- Robot pose: [x, y, theta]
- Landmarks: [lm_i_x, lm_i_y] for each landmark i
"""

import numpy as np


class EKF_SLAM:
    """
    Extended Kalman Filter for Simultaneous Localization and Mapping.

    Maintains:
    - State estimate (robot pose + landmark positions)
    - Covariance matrix (uncertainty)
    - Landmark associations
    """

    def __init__(self, initial_pose, motion_noise, measurement_noise, validation_gate=3.0):
        """
        Initialize EKF-SLAM.

        Parameters:
        -----------
        initial_pose : np.array (3,)
            Initial robot pose [x, y, theta]
        motion_noise : np.array (3, 3)
            Motion model noise covariance
        measurement_noise : np.array (2, 2)
            Measurement noise covariance
        validation_gate : float, optional
            Mahalanobis distance threshold for outlier rejection (default: 3.0 sigma)
        """
        # State vector: [x, y, theta, landmarks...]
        self.state = initial_pose.copy()

        # Covariance matrix: starts with robot pose uncertainty
        # Initial uncertainty: moderate to allow filter to adapt
        self.covariance = np.diag([0.5, 0.5, 0.1])  # 0.5m in x,y, 0.1 rad in theta

        # Noise matrices
        self.R = motion_noise  # Process noise
        self.Q = measurement_noise  # Measurement noise

        # Validation gate for outlier rejection
        self.validation_gate = validation_gate

        # Landmark bookkeeping
        self.landmark_ids = []  # List of landmark IDs in state
        self.num_landmarks = 0

        # Statistics
        self.measurements_accepted = 0
        self.measurements_rejected = 0

        # Consistency metrics
        self.latest_nis_values = []  # Store NIS for each measurement update
        self.latest_nees = 0.0  # Store latest NEES value

        # Loop closure detection
        self.pose_history = []  # Store poses for loop closure detection
        self.pose_history_interval = 30  # Store pose every N updates
        self.update_counter = 0
        self.loop_closures_detected = 0
        self.last_closure_update = -999  # Track last closure to prevent spam
        self.closure_cooldown = 150  # Minimum updates between closures (~2.5s at 60Hz)

    @property
    def robot_pose(self):
        """Get current robot pose estimate."""
        return self.state[:3]

    @property
    def robot_covariance(self):
        """Get robot pose covariance."""
        return self.covariance[:3, :3]

    def get_landmark_position(self, landmark_id):
        """
        Get estimated position of a landmark.

        Parameters:
        -----------
        landmark_id : int
            Landmark identifier

        Returns:
        --------
        position : np.array (2,) or None
            Landmark position [x, y], or None if not in map
        """
        if landmark_id not in self.landmark_ids:
            return None

        idx = self.landmark_ids.index(landmark_id)
        lm_start = 3 + idx * 2
        return self.state[lm_start:lm_start + 2]

    def predict(self, v, omega, dt):
        """
        Prediction step: propagate state using motion model.

        Parameters:
        -----------
        v : float
            Linear velocity (m/s)
        omega : float
            Angular velocity (rad/s)
        dt : float
            Time step (seconds)
        """
        # Extract robot pose
        x, y, theta = self.state[:3]

        # Motion model: unicycle kinematics
        x_new = x + v * dt * np.cos(theta)
        y_new = y + v * dt * np.sin(theta)
        theta_new = theta + omega * dt
        theta_new = self._wrap_angle(theta_new)

        # Update robot pose in state
        self.state[:3] = [x_new, y_new, theta_new]

        # Jacobian of motion model with respect to robot pose
        G_t = np.eye(len(self.state))
        G_t[0, 2] = -v * dt * np.sin(theta)
        G_t[1, 2] = v * dt * np.cos(theta)

        # Propagate covariance
        # Only robot pose is affected by motion noise
        R_full = np.zeros((len(self.state), len(self.state)))
        R_full[:3, :3] = self.R

        self.covariance = G_t @ self.covariance @ G_t.T + R_full

        # Ensure symmetry
        self.covariance = (self.covariance + self.covariance.T) / 2

    def update(self, measurements, sensor, ground_truth_pose=None):
        """
        Update step: correct state using landmark measurements.

        Parameters:
        -----------
        measurements : list of dict
            Sensor measurements, each with 'landmark_id', 'range', 'bearing'
        sensor : RangeBearingSensor
            Sensor object for computing measurement model
        ground_truth_pose : np.array (3,), optional
            Ground truth robot pose [x, y, theta] for landmark initialization.
            If provided, new landmarks are initialized using this pose instead
            of the estimated pose to prevent feedback loops.
        """
        # Clear NIS values for this update batch
        self.latest_nis_values = []

        for measurement in measurements:
            landmark_id = measurement['landmark_id']
            z = np.array([measurement['range'], measurement['bearing']])

            if landmark_id not in self.landmark_ids:
                # New landmark: initialize in map
                self._initialize_landmark(landmark_id, measurement, sensor, ground_truth_pose)
            else:
                # Known landmark: update using EKF
                self._update_landmark(landmark_id, z, sensor)

    def _initialize_landmark(self, landmark_id, measurement, sensor, ground_truth_pose=None):
        """
        Initialize a new landmark in the map.

        Parameters:
        -----------
        landmark_id : int
            Landmark identifier
        measurement : dict
            Measurement containing 'range' and 'bearing'
        sensor : RangeBearingSensor
            Sensor object
        ground_truth_pose : np.array (3,), optional
            Ground truth robot pose [x, y, theta] for initialization.
            If provided, use this instead of estimated pose to prevent
            initialization errors from corrupting the map.
        """
        # Extract robot pose (use ground truth if provided)
        if ground_truth_pose is not None:
            x, y, theta = ground_truth_pose
        else:
            x, y, theta = self.state[:3]

        # Compute landmark position from measurement
        r = measurement['range']
        phi = measurement['bearing']

        lm_x = x + r * np.cos(theta + phi)
        lm_y = y + r * np.sin(theta + phi)

        # Add landmark to state
        self.state = np.append(self.state, [lm_x, lm_y])
        self.landmark_ids.append(landmark_id)
        self.num_landmarks += 1

        # Expand covariance matrix
        # Initialize landmark with high uncertainty
        n = len(self.state)
        new_cov = np.zeros((n, n))
        new_cov[:-2, :-2] = self.covariance
        new_cov[-2, -2] = 10.0  # High initial uncertainty in x
        new_cov[-1, -1] = 10.0  # High initial uncertainty in y
        self.covariance = new_cov

    def _update_landmark(self, landmark_id, z, sensor):
        """
        Update state using measurement of known landmark.

        Parameters:
        -----------
        landmark_id : int
            Landmark identifier
        z : np.array (2,)
            Measurement [range, bearing]
        sensor : RangeBearingSensor
            Sensor object
        """
        # Get landmark index in state
        idx = self.landmark_ids.index(landmark_id)
        lm_start = 3 + idx * 2

        # Extract robot pose and landmark position
        x, y, theta = self.state[:3]
        lm_x, lm_y = self.state[lm_start:lm_start + 2]

        # Expected measurement
        z_hat = sensor.measurement_model(x, y, theta, lm_x, lm_y)

        # Innovation (measurement residual)
        innovation = z - z_hat
        innovation[1] = self._wrap_angle(innovation[1])  # Wrap bearing

        # Measurement Jacobian
        H = self._measurement_jacobian(x, y, theta, lm_x, lm_y, lm_start)

        # Innovation covariance
        S = H @ self.covariance @ H.T + self.Q

        # Ensure S is symmetric (numerical stability)
        S = (S + S.T) / 2

        # ========== VALIDATION GATE: Reject outliers ==========
        # Compute Mahalanobis distance (how many std devs away is the measurement?)
        # NIS (Normalized Innovation Squared) = innovation^T * S^-1 * innovation
        # Use solve instead of inv for numerical stability
        try:
            S_inv_innovation = np.linalg.solve(S, innovation)
            nis = float(innovation.T @ S_inv_innovation)
            # Ensure NIS is non-negative (numerical errors can make it slightly negative)
            nis = max(0.0, nis)
            mahalanobis = np.sqrt(nis)
        except np.linalg.LinAlgError:
            # S is singular, reject measurement
            self.measurements_rejected += 1
            return

        # Reject measurement if it's too far from expected (likely wrong association or outlier)
        if mahalanobis > self.validation_gate:
            self.measurements_rejected += 1
            # Optionally log the rejection for debugging
            # print(f"[SLAM] Rejected measurement for landmark {landmark_id}: "
            #       f"Mahalanobis distance = {mahalanobis:.2f} > {self.validation_gate}")
            return  # Skip this measurement

        self.measurements_accepted += 1

        # Store NIS value for accepted measurements (for consistency checking)
        self.latest_nis_values.append(float(nis))
        # =======================================================

        # Kalman gain
        # K = P @ H.T @ inv(S)
        # More stable: solve S @ x = H.T for each column, then multiply by P
        # Or equivalently: K = P @ H.T @ inv(S) = (inv(S) @ H @ P).T = solve(S, H @ P).T
        # But to avoid dimension issues, use: K = P @ H.T @ inv(S) directly with pinv for stability
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            # Use pseudoinverse if S is singular
            S_inv = np.linalg.pinv(S)
        K = self.covariance @ H.T @ S_inv

        # Update state
        self.state = self.state + K @ innovation
        self.state[2] = self._wrap_angle(self.state[2])  # Wrap robot theta

        # Update covariance (Joseph form for numerical stability)
        I = np.eye(len(self.state))
        IKH = I - K @ H
        self.covariance = IKH @ self.covariance @ IKH.T + K @ self.Q @ K.T

        # Ensure symmetry and positive definiteness
        self.covariance = (self.covariance + self.covariance.T) / 2
        # Add small epsilon to diagonal for numerical stability
        self.covariance += np.eye(len(self.state)) * 1e-6

    def _measurement_jacobian(self, x, y, theta, lm_x, lm_y, lm_idx):
        """
        Compute Jacobian of measurement model.

        Parameters:
        -----------
        x, y, theta : float
            Robot pose
        lm_x, lm_y : float
            Landmark position
        lm_idx : int
            Starting index of landmark in state vector

        Returns:
        --------
        H : np.array (2, n)
            Measurement Jacobian
        """
        n = len(self.state)
        H = np.zeros((2, n))

        # Compute Jacobian components
        dx = lm_x - x
        dy = lm_y - y
        q = dx**2 + dy**2
        sqrt_q = np.sqrt(q)

        # Jacobian with respect to robot pose
        H[0, 0] = -dx / sqrt_q
        H[0, 1] = -dy / sqrt_q
        H[0, 2] = 0

        H[1, 0] = dy / q
        H[1, 1] = -dx / q
        H[1, 2] = -1

        # Jacobian with respect to landmark position
        H[0, lm_idx] = dx / sqrt_q
        H[0, lm_idx + 1] = dy / sqrt_q

        H[1, lm_idx] = -dy / q
        H[1, lm_idx + 1] = dx / q

        return H

    def get_all_landmarks(self):
        """
        Get all landmarks in the map.

        Returns:
        --------
        landmarks : list of dict
            Each landmark has 'id', 'position', and 'covariance'
        """
        landmarks = []

        for i, lm_id in enumerate(self.landmark_ids):
            lm_start = 3 + i * 2
            position = self.state[lm_start:lm_start + 2]
            cov = self.covariance[lm_start:lm_start + 2, lm_start:lm_start + 2]

            landmarks.append({
                'id': lm_id,
                'position': position,
                'covariance': cov
            })

        return landmarks

    def get_rejection_stats(self):
        """
        Get measurement acceptance/rejection statistics.

        Returns:
        --------
        stats : dict
            Statistics including accepted, rejected, and rejection rate
        """
        total = self.measurements_accepted + self.measurements_rejected
        rejection_rate = (self.measurements_rejected / total * 100) if total > 0 else 0

        return {
            'accepted': self.measurements_accepted,
            'rejected': self.measurements_rejected,
            'total': total,
            'rejection_rate': rejection_rate
        }

    def detect_loop_closure(self, current_landmarks, distance_threshold=5.0, landmark_threshold=3):
        """
        Detect if robot has returned to a previously visited location.

        Parameters:
        -----------
        current_landmarks : list
            Currently visible landmark IDs
        distance_threshold : float
            Maximum distance to consider a loop closure (meters)
        landmark_threshold : int
            Minimum number of common landmarks to confirm closure

        Returns:
        --------
        closure_detected : bool
            True if loop closure detected
        closure_pose : np.array or None
            The historical pose that matches (if closure detected)
        """
        # Cooldown check: prevent detecting closures too frequently
        if self.update_counter - self.last_closure_update < self.closure_cooldown:
            return False, None

        current_pose = self.robot_pose[:2]  # Only x, y for distance check

        # Check against pose history
        for hist_pose, hist_landmarks in self.pose_history:
            # Calculate distance to historical pose
            dist = np.linalg.norm(current_pose - hist_pose[:2])

            if dist < distance_threshold:
                # Check landmark overlap
                common_landmarks = set(current_landmarks) & set(hist_landmarks)

                if len(common_landmarks) >= landmark_threshold:
                    # Loop closure detected!
                    self.last_closure_update = self.update_counter
                    return True, hist_pose

        return False, None

    def apply_loop_closure_correction(self, detected_pose):
        """
        Apply a simple loop closure correction.

        This is a basic correction that adjusts the pose estimate
        towards the detected historical pose.

        Parameters:
        -----------
        detected_pose : np.array
            The historical pose that was detected as a closure
        """
        current_pose = self.robot_pose

        # Calculate correction (weighted average)
        # Use uncertainty to weight the correction
        uncertainty = np.sqrt(np.trace(self.robot_covariance[:2, :2]))
        correction_weight = min(0.3, uncertainty / 2.0)  # Stronger correction if more uncertain

        # Apply correction to pose
        pose_correction = detected_pose - current_pose
        pose_correction[2] = self._wrap_angle(pose_correction[2])

        # Update state with weighted correction
        self.state[:3] += correction_weight * pose_correction

        # Reduce uncertainty after loop closure
        self.covariance[:3, :3] *= (1.0 - correction_weight * 0.5)

        self.loop_closures_detected += 1

    def store_pose_history(self, current_landmarks):
        """
        Store current pose for future loop closure detection.

        Parameters:
        -----------
        current_landmarks : list
            Currently visible landmark IDs
        """
        self.update_counter += 1

        if self.update_counter % self.pose_history_interval == 0:
            pose = self.robot_pose.copy()
            self.pose_history.append((pose, current_landmarks.copy()))

            # Limit history size to prevent memory issues
            if len(self.pose_history) > 200:
                self.pose_history.pop(0)

    def compute_nees(self, ground_truth_pose):
        """
        Compute NEES (Normalized Estimation Error Squared) for filter consistency.

        NEES measures whether the estimation error is consistent with the
        estimated covariance. It should be chi-squared distributed with
        degrees of freedom = dimension of state (3 for robot pose).

        Expected value ≈ 3 for a consistent filter.
        95% confidence interval for chi-squared(3): [0.35, 7.81]

        Parameters:
        -----------
        ground_truth_pose : np.array (3,)
            True robot pose [x, y, theta]

        Returns:
        --------
        nees : float
            NEES value
        """
        # Estimation error
        error = self.robot_pose - ground_truth_pose
        error[2] = self._wrap_angle(error[2])  # Wrap angle error

        # NEES = error^T * P^(-1) * error
        # Only compute for robot pose (not landmarks)
        P_robot = self.robot_covariance

        try:
            nees = error.T @ np.linalg.inv(P_robot) @ error
            self.latest_nees = float(nees)
        except np.linalg.LinAlgError:
            # Covariance is singular, return large value
            self.latest_nees = 999.0

        return self.latest_nees

    def get_average_nis(self):
        """
        Get average NIS from latest measurement updates.

        NIS should be chi-squared distributed with degrees of freedom = 2
        (for range-bearing measurements).

        Expected value ≈ 2 for a consistent filter.
        95% confidence interval for chi-squared(2): [0.05, 5.99]

        Returns:
        --------
        avg_nis : float
            Average NIS value, or 0 if no measurements
        """
        if len(self.latest_nis_values) == 0:
            return 0.0
        return float(np.mean(self.latest_nis_values))

    @staticmethod
    def _wrap_angle(angle):
        """Wrap angle to [-pi, pi]."""
        return np.arctan2(np.sin(angle), np.cos(angle))
