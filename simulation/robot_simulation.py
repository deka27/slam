"""
Robot simulation on Monza track with FURY visualization.
Phase 2: Robot with motion model (no SLAM yet).
"""

import numpy as np
import sys
from pathlib import Path

# Add parent directory to path
parent_dir = Path(__file__).parent.parent
sys.path.insert(0, str(parent_dir))

from fury import window, actor
from robot.robot import Robot
from robot.path_controller import PathController
from utils.racing_line import generate_racing_line
from environment.landmark_generator import load_landmarks
from slam import EKF_SLAM

# Import track visualization
import importlib.util
spec = importlib.util.spec_from_file_location(
    "track_module",
    parent_dir / "environment" / "track_3d" / "track_clean_fury_improved.py"
)
track_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(track_module)
MonzaTrack3D = track_module.MonzaTrack3DImproved


class RobotSimulation:
    """
    Simulation of robot moving on track.
    """

    def __init__(self, track_centerline, landmarks, dt=0.1, enable_noise=False, camera_mode='follow'):
        """
        Initialize simulation.

        Parameters:
        -----------
        track_centerline : np.array
            Track centerline as (N, 2) array
        landmarks : list of dict
            Landmarks for SLAM
        dt : float
            Time step (seconds)
        enable_noise : bool
            Whether to enable motion and sensor noise
        camera_mode : str
            'follow' for 3rd person camera, 'overview' for static top view
        """
        self.track_centerline = track_centerline
        self.landmarks = landmarks
        self.dt = dt
        self.camera_mode = camera_mode
        self.enable_noise = enable_noise  # Store noise setting for sensor measurements

        # Initialize robot at start of track
        start_x = track_centerline[0, 0]
        start_y = track_centerline[0, 1]

        # Calculate initial heading (towards next point)
        dx = track_centerline[1, 0] - track_centerline[0, 0]
        dy = track_centerline[1, 1] - track_centerline[0, 1]
        start_theta = np.arctan2(dy, dx)

        # Use dt for robot (will be ~0.017s for 60 Hz)
        self.robot = Robot(start_x, start_y, start_theta, dt=self.dt)

        # Enable noise if requested
        if enable_noise:
            self.robot.enable_motion_noise()

        # Initialize controller
        # Balanced speed with simple pure pursuit
        self.controller = PathController(
            path=track_centerline,
            desired_speed=15.0,  # 15 m/s (~54 km/h) - balanced speed
            lookahead_distance=12.0  # 12 meters - smooth following at higher speed
        )

        # Simulation state
        self.time = 0.0
        self.laps_completed = 0
        self.running = False

        # Distance-based lap tracking
        self.track_length = 711.7  # meters (precomputed)
        self.total_distance = 0.0
        self.last_position = np.array([start_x, start_y])

        # FURY visualization
        self.scene = None
        self.robot_actor = None
        self.trajectory_actor = None
        self.detection_lines_actor = None
        self.slam_robot_actor = None
        self.slam_direction_actor = None
        self.slam_trajectory_actor = None
        self.slam_landmarks_actor = None
        self.slam_uncertainty_actor = None

        # Camera smoothing
        self.camera_pos = None
        self.camera_focal = None

        # Motion interpolation for smooth rendering
        self.prev_robot_state = None
        self.current_robot_state = None
        self.interpolation_alpha = 0.0

        # Sensor measurements
        self.latest_measurements = []

        # SLAM trajectory tracking
        self.slam_trajectory = []

        # Performance metrics tracking
        self.metrics_history = {
            'time': [],
            'pos_error': [],  # Euclidean distance error
            'x_error': [],
            'y_error': [],
            'theta_error': [],
            'uncertainty_x': [],
            'uncertainty_y': [],
            'uncertainty_theta': [],
            'num_landmarks_mapped': [],
            'num_landmarks_detected': [],
            'loop_closures': [],
            'laps_completed': [],
            'nees': [],  # Normalized Estimation Error Squared
            'nis': []    # Normalized Innovation Squared (average)
        }

        # Track lap completion times for plotting
        self.lap_times = []

        # Initialize EKF-SLAM
        initial_pose = np.array([start_x, start_y, start_theta])

        # Set noise covariances based on whether noise is enabled
        if enable_noise:
            # With noise: Use realistic noise parameters
            # Tuned motion noise - balanced uncertainty
            # Higher values = trust measurements more, lower values = trust odometry more
            motion_noise = np.diag([0.5, 0.5, 0.08])**2  # Increased: less trust in odometry
            # Measurement noise - trust sensor readings
            # Lower multiplier = trust sensors more
            measurement_noise = self.robot.sensor.Q * 0.3  # 30% - trust sensors more
        else:
            # Without noise: Use very small noise parameters (near-perfect sensors/odometry)
            # Use small but non-zero values to prevent numerical issues
            motion_noise = np.diag([0.01, 0.01, 0.001])**2  # Very small motion uncertainty
            measurement_noise = self.robot.sensor.Q * 0.01  # 1% - nearly perfect sensors

        self.ekf_slam = EKF_SLAM(initial_pose, motion_noise, measurement_noise)

        # SLAM update counter (update measurements at higher rate for better tracking)
        self.slam_update_counter = 0
        self.slam_update_interval = 2  # Update measurements every 2 frames (~30 Hz)

        # Set up log file for SLAM debugging
        log_dir = Path(__file__).parent.parent / "logs"
        log_dir.mkdir(exist_ok=True)
        self.log_file = open(log_dir / "slam_log.txt", "w")
        self.log_file.write("SLAM Tracking Log\n")
        self.log_file.write("=" * 60 + "\n\n")

    def step(self):
        """Execute one simulation step."""
        # Store previous state for interpolation
        self.prev_robot_state = np.array([self.robot.x, self.robot.y, self.robot.theta])

        # Get control commands from controller
        v, omega = self.controller.compute_control(
            self.robot.x, self.robot.y, self.robot.theta
        )

        # EKF-SLAM Prediction step (every frame to track motion)
        self.ekf_slam.predict(v, omega, self.dt)

        # Move robot (ground truth)
        self.robot.move(v, omega)

        # Store current state for interpolation
        self.current_robot_state = np.array([self.robot.x, self.robot.y, self.robot.theta])

        # Get sensor measurements
        self.latest_measurements = self.robot.sensor.measure(
            self.robot.x, self.robot.y, self.robot.theta,
            self.landmarks,
            add_noise=self.enable_noise  # Use simulation noise setting
        )

        # EKF-SLAM Measurement Update at reduced rate
        self.slam_update_counter += 1
        if self.slam_update_counter >= self.slam_update_interval:
            self.slam_update_counter = 0

            # Log SLAM state before update
            if len(self.latest_measurements) > 0 and self.time % 2.0 < 0.1:  # Log every 2 seconds
                self._log_slam_state()

            # EKF-SLAM Update step with current measurements
            # Pass ground truth robot pose for landmark initialization (common in simulation)
            if len(self.latest_measurements) > 0:
                ground_truth_pose = np.array([self.robot.x, self.robot.y, self.robot.theta])
                self.ekf_slam.update(self.latest_measurements, self.robot.sensor, ground_truth_pose)

                # Store pose history for loop closure detection
                current_landmark_ids = [m['landmark_id'] for m in self.latest_measurements]
                self.ekf_slam.store_pose_history(current_landmark_ids)

                # Check for loop closure
                closure_detected, closure_pose = self.ekf_slam.detect_loop_closure(
                    current_landmark_ids,
                    distance_threshold=5.0,  # 5m threshold - tight for accuracy
                    landmark_threshold=3  # Need 3+ common landmarks for confirmation
                )

                if closure_detected:
                    # Apply loop closure correction
                    self.ekf_slam.apply_loop_closure_correction(closure_pose)
                    print(f"[{self.time:.1f}s] Loop closure detected! Total closures: {self.ekf_slam.loop_closures_detected}")

        # Store SLAM trajectory for visualization
        slam_pose = self.ekf_slam.robot_pose
        self.slam_trajectory.append(slam_pose.copy())

        # Record performance metrics
        self._record_metrics()

        # Update time
        self.time += self.dt

        # Distance-based lap tracking (more reliable than position-based)
        current_pos = np.array([self.robot.x, self.robot.y])
        step_distance = np.linalg.norm(current_pos - self.last_position)
        self.total_distance += step_distance
        self.last_position = current_pos.copy()

        # Calculate laps from distance
        new_laps = int(self.total_distance / self.track_length)

        # Debug: Print distance periodically
        if int(self.time) % 50 == 0 and self.time - int(self.time) < self.dt:
            avg_speed = self.total_distance / self.time if self.time > 0 else 0
            print(f"[Distance] t={self.time:.0f}s: total={self.total_distance:.0f}m, avg_speed={avg_speed:.1f}m/s, laps={self.laps_completed}")

        if new_laps > self.laps_completed:
            # New lap completed!
            for lap_num in range(self.laps_completed + 1, new_laps + 1):
                self.lap_times.append(self.time)
                print(f"🏁 Lap {lap_num} completed at t={self.time:.1f}s (total dist: {self.total_distance:.0f}m)")
            self.laps_completed = new_laps

    def get_interpolated_state(self, alpha):
        """Get interpolated robot state for smooth rendering.

        Parameters:
        -----------
        alpha : float
            Interpolation factor (0 = previous state, 1 = current state)
        """
        if self.prev_robot_state is None or self.current_robot_state is None:
            return self.robot.x, self.robot.y, self.robot.theta

        # Linear interpolation for position
        x = (1 - alpha) * self.prev_robot_state[0] + alpha * self.current_robot_state[0]
        y = (1 - alpha) * self.prev_robot_state[1] + alpha * self.current_robot_state[1]

        # Circular interpolation for angle
        theta_prev = self.prev_robot_state[2]
        theta_curr = self.current_robot_state[2]
        theta_diff = np.arctan2(np.sin(theta_curr - theta_prev), np.cos(theta_curr - theta_prev))
        theta = theta_prev + alpha * theta_diff

        return x, y, theta

    def setup_visualization(self, scene):
        """
        Set up FURY visualization.

        Parameters:
        -----------
        scene : fury.window.Scene
            FURY scene to add actors to
        """
        self.scene = scene

        # Create robot actor (box + arrow for direction)
        robot_size = 1.5  # ~1/7th track width

        # Robot position
        robot_pos = np.array([[self.robot.x, robot_size/2, self.robot.y]])

        # Create box for robot body
        self.robot_actor = actor.box(
            centers=robot_pos,
            colors=np.array([[0.2, 0.6, 1.0]]),  # Blue
            scales=np.array([[robot_size, robot_size, robot_size]])
        )

        self.scene.add(self.robot_actor)

        # Create arrow to show direction (bigger and more visible)
        arrow_length = 5.0  # Longer arrow
        arrow_height = robot_size + 1.0  # Higher than robot

        arrow_start = np.array([[self.robot.x, arrow_height, self.robot.y]])
        arrow_end = np.array([[
            self.robot.x + arrow_length * np.cos(self.robot.theta),
            arrow_height,
            self.robot.y + arrow_length * np.sin(self.robot.theta)
        ]])

        self.direction_actor = actor.arrow(
            centers=arrow_start,
            directions=arrow_end - arrow_start,
            colors=np.array([[1.0, 0.3, 0.0]]),  # Bright orange
            heights=arrow_length,
            resolution=16,
            tip_length=0.3,
            tip_radius=0.15,
            shaft_radius=0.08
        )

        self.scene.add(self.direction_actor)

        # Create trajectory line actor (initially empty)
        self._update_trajectory()

    def _update_trajectory(self):
        """Update trajectory visualization."""
        trajectory = self.robot.get_trajectory()

        if len(trajectory) > 1:
            # Remove old trajectory actor if exists
            if self.trajectory_actor is not None:
                self.scene.rm(self.trajectory_actor)

            # Create trajectory path (at small height above ground)
            traj_3d = np.column_stack([
                trajectory[:, 0],
                np.ones(len(trajectory)) * 0.2,  # Slightly above ground
                trajectory[:, 1]
            ])

            # Create line actor
            self.trajectory_actor = actor.line(
                [traj_3d],
                colors=(0.2, 0.6, 1.0),  # Blue
                linewidth=2
            )

            self.scene.add(self.trajectory_actor)

    def update_visualization(self, interpolation_alpha=1.0):
        """Update visualization for current state.

        Parameters:
        -----------
        interpolation_alpha : float
            Interpolation factor for smooth motion (0 to 1)
        """
        if self.scene is None:
            return

        # Get interpolated state for smooth motion
        robot_x, robot_y, robot_theta = self.get_interpolated_state(interpolation_alpha)

        robot_size = 1.5  # ~1/7th track width
        arrow_length = 5.0  # Longer, more visible arrow
        arrow_height = robot_size + 1.0  # Higher than robot

        # Update robot position (using interpolated state)
        robot_pos = np.array([[robot_x, robot_size/2, robot_y]])

        # Remove old actors
        if self.robot_actor is not None:
            self.scene.rm(self.robot_actor)
        if hasattr(self, 'direction_actor') and self.direction_actor is not None:
            self.scene.rm(self.direction_actor)

        # Create new robot box
        self.robot_actor = actor.box(
            centers=robot_pos,
            colors=np.array([[0.2, 0.6, 1.0]]),
            scales=np.array([[robot_size, robot_size, robot_size]])
        )
        self.scene.add(self.robot_actor)

        # Create new direction arrow (bigger and more visible, using interpolated state)
        arrow_start = np.array([[robot_x, arrow_height, robot_y]])
        arrow_end = np.array([[
            robot_x + arrow_length * np.cos(robot_theta),
            arrow_height,
            robot_y + arrow_length * np.sin(robot_theta)
        ]])

        self.direction_actor = actor.arrow(
            centers=arrow_start,
            directions=arrow_end - arrow_start,
            colors=np.array([[1.0, 0.3, 0.0]]),  # Bright orange
            heights=arrow_length,
            resolution=16,
            tip_length=0.3,
            tip_radius=0.15,
            shaft_radius=0.08
        )
        self.scene.add(self.direction_actor)

        # Update camera if in follow mode (using interpolated state)
        if self.camera_mode == 'follow':
            self._update_follow_camera(robot_x, robot_y, robot_theta)

        # Update trajectory every 10 steps (for performance)
        if len(self.robot.trajectory) % 10 == 0:
            self._update_trajectory()

        # Visualize sensor detections
        self._update_detections(robot_x, robot_y)

        # Visualize SLAM estimates
        self._update_slam_visualization()

    def _update_slam_visualization(self):
        """Update visualization of SLAM estimates."""
        if self.scene is None:
            return

        # Remove old SLAM actors
        if self.slam_robot_actor is not None:
            self.scene.rm(self.slam_robot_actor)
        if self.slam_direction_actor is not None:
            self.scene.rm(self.slam_direction_actor)
        if self.slam_trajectory_actor is not None:
            self.scene.rm(self.slam_trajectory_actor)
        if self.slam_landmarks_actor is not None:
            self.scene.rm(self.slam_landmarks_actor)

        # Visualize SLAM estimated robot pose (green box)
        slam_pose = self.ekf_slam.robot_pose
        robot_size = 1.5

        slam_robot_pos = np.array([[slam_pose[0], robot_size/2, slam_pose[1]]])
        self.slam_robot_actor = actor.box(
            centers=slam_robot_pos,
            colors=np.array([[0.0, 1.0, 0.0]]),  # Green for SLAM estimate
            scales=np.array([[robot_size, robot_size, robot_size]])
        )
        # Make it semi-transparent to see both ground truth and estimate
        self.slam_robot_actor.GetProperty().SetOpacity(0.5)
        self.scene.add(self.slam_robot_actor)

        # Add direction arrow for SLAM robot
        arrow_length = 5.0
        arrow_height = robot_size + 1.5  # Slightly higher than ground truth arrow

        slam_arrow_start = np.array([[slam_pose[0], arrow_height, slam_pose[1]]])
        slam_arrow_end = np.array([[
            slam_pose[0] + arrow_length * np.cos(slam_pose[2]),
            arrow_height,
            slam_pose[1] + arrow_length * np.sin(slam_pose[2])
        ]])

        self.slam_direction_actor = actor.arrow(
            centers=slam_arrow_start,
            directions=slam_arrow_end - slam_arrow_start,
            colors=np.array([[0.0, 1.0, 0.0]]),  # Green for SLAM
            heights=arrow_length,
            resolution=16,
            tip_length=0.3,
            tip_radius=0.15,
            shaft_radius=0.08
        )
        self.slam_direction_actor.GetProperty().SetOpacity(0.7)
        self.scene.add(self.slam_direction_actor)

        # Visualize SLAM trajectory (update every 10 steps for performance)
        if len(self.slam_trajectory) > 1 and len(self.slam_trajectory) % 10 == 0:
            slam_traj = np.array(self.slam_trajectory)
            slam_traj_3d = np.column_stack([
                slam_traj[:, 0],
                np.ones(len(slam_traj)) * 0.5,  # Slightly above ground truth trajectory
                slam_traj[:, 1]
            ])

            self.slam_trajectory_actor = actor.line(
                [slam_traj_3d],
                colors=(0.0, 1.0, 0.0),  # Green for SLAM trajectory
                linewidth=2
            )
            self.slam_trajectory_actor.GetProperty().SetOpacity(0.6)
            self.scene.add(self.slam_trajectory_actor)

        # Visualize SLAM estimated landmarks
        slam_landmarks = self.ekf_slam.get_all_landmarks()
        if len(slam_landmarks) > 0:
            lm_positions = []
            lm_colors = []
            lm_scales = []

            for lm in slam_landmarks:
                pos = lm['position']
                lm_positions.append([pos[0], 1.5, pos[1]])  # Height = 1.5m
                lm_colors.append([0.0, 1.0, 0.0])  # Green for SLAM estimates
                lm_scales.append([1.5, 3.0, 1.5])  # Smaller than true landmarks

            self.slam_landmarks_actor = actor.box(
                centers=np.array(lm_positions),
                colors=np.array(lm_colors),
                scales=np.array(lm_scales)
            )
            self.slam_landmarks_actor.GetProperty().SetOpacity(0.5)
            self.scene.add(self.slam_landmarks_actor)

        # Visualize uncertainties (covariance ellipses)
        self._add_uncertainty_visualization()

    def _update_detections(self, robot_x, robot_y):
        """Update visualization of sensor detections."""
        if self.scene is None or len(self.latest_measurements) == 0:
            return

        # Remove old detection lines
        if self.detection_lines_actor is not None:
            self.scene.rm(self.detection_lines_actor)

        # Create lines from robot to detected landmarks
        lines = []
        for detection in self.latest_measurements:
            landmark_pos = detection['position']
            # Line from robot to landmark (slightly above ground)
            line = np.array([
                [robot_x, 1.0, robot_y],
                [landmark_pos[0], 1.0, landmark_pos[1]]
            ])
            lines.append(line)

        if len(lines) > 0:
            self.detection_lines_actor = actor.line(
                lines,
                colors=(1.0, 1.0, 0.0),  # Yellow for detections
                linewidth=2,
                opacity=0.5
            )
            self.scene.add(self.detection_lines_actor)

    def _add_uncertainty_visualization(self):
        """Add covariance ellipses for robot and landmarks."""
        if self.scene is None:
            return

        # Remove old uncertainty visualization
        if self.slam_uncertainty_actor is not None:
            self.scene.rm(self.slam_uncertainty_actor)

        # Get robot pose covariance (2x2 for x,y)
        robot_cov = self.ekf_slam.robot_covariance[:2, :2]
        slam_pose = self.ekf_slam.robot_pose

        # Compute eigenvalues for uncertainty magnitude
        eigenvalues, eigenvectors = np.linalg.eig(robot_cov)

        # Scale factor for 95% confidence (2-sigma)
        scale_factor = 2.0

        # Use average of eigenvalues for sphere radius
        avg_uncertainty = scale_factor * np.sqrt(np.mean(eigenvalues))

        # Create sphere for robot uncertainty (simplified)
        uncertainty_sphere = actor.sphere(
            centers=np.array([[slam_pose[0], 0.2, slam_pose[1]]]),
            colors=np.array([[0.0, 1.0, 0.0]]),
            radii=avg_uncertainty
        )

        uncertainty_sphere.GetProperty().SetOpacity(0.15)

        self.slam_uncertainty_actor = uncertainty_sphere
        self.scene.add(self.slam_uncertainty_actor)

    def _update_follow_camera(self, robot_x=None, robot_y=None, robot_theta=None):
        """Update camera to follow robot (3rd person view) with smoothing.

        Parameters:
        -----------
        robot_x, robot_y, robot_theta : float, optional
            Robot state to use (if None, uses actual robot state)
        """
        # Use provided state or actual robot state
        if robot_x is None:
            robot_x = self.robot.x
            robot_y = self.robot.y
            robot_theta = self.robot.theta

        # Camera position: behind and above the robot
        camera_distance = 50.0  # Distance behind robot (further back)
        camera_height = 20.0    # Height above robot

        # Calculate target camera position (behind the robot)
        target_cam_x = robot_x - camera_distance * np.cos(robot_theta)
        target_cam_y = camera_height
        target_cam_z = robot_y - camera_distance * np.sin(robot_theta)

        # Look at point: slightly ahead of robot
        look_ahead = 15.0
        target_focal_x = robot_x + look_ahead * np.cos(robot_theta)
        target_focal_y = 2.0
        target_focal_z = robot_y + look_ahead * np.sin(robot_theta)

        # Smooth camera motion (exponential smoothing / lerp)
        alpha = 0.15  # Smoothing factor (lower = smoother but more lag)

        if self.camera_pos is None:
            # First frame - initialize
            self.camera_pos = np.array([target_cam_x, target_cam_y, target_cam_z])
            self.camera_focal = np.array([target_focal_x, target_focal_y, target_focal_z])
        else:
            # Smooth interpolation
            self.camera_pos = (1 - alpha) * self.camera_pos + alpha * np.array([target_cam_x, target_cam_y, target_cam_z])
            self.camera_focal = (1 - alpha) * self.camera_focal + alpha * np.array([target_focal_x, target_focal_y, target_focal_z])

        # Update camera
        self.scene.set_camera(
            position=tuple(self.camera_pos),
            focal_point=tuple(self.camera_focal),
            view_up=(0, 1, 0)
        )

    def _log_slam_state(self):
        """Log SLAM state to file for debugging."""
        slam_pose = self.ekf_slam.robot_pose
        true_pose = np.array([self.robot.x, self.robot.y, self.robot.theta])

        error = slam_pose - true_pose
        error[2] = np.arctan2(np.sin(error[2]), np.cos(error[2]))  # Wrap angle

        self.log_file.write(f"\n{'='*60}\n")
        self.log_file.write(f"Time: {self.time:.2f}s\n")
        self.log_file.write(f"Ground Truth:  x={true_pose[0]:7.2f}, y={true_pose[1]:7.2f}, θ={np.degrees(true_pose[2]):6.1f}°\n")
        self.log_file.write(f"SLAM Estimate: x={slam_pose[0]:7.2f}, y={slam_pose[1]:7.2f}, θ={np.degrees(slam_pose[2]):6.1f}°\n")
        self.log_file.write(f"Error:         Δx={error[0]:6.2f}, Δy={error[1]:6.2f}, Δθ={np.degrees(error[2]):5.1f}°\n")

        # Covariance diagonal (uncertainties)
        cov_diag = np.sqrt(np.diag(self.ekf_slam.robot_covariance))
        self.log_file.write(f"Uncertainty (σ): x={cov_diag[0]:.2f}m, y={cov_diag[1]:.2f}m, θ={np.degrees(cov_diag[2]):.1f}°\n")

        # Measurements
        self.log_file.write(f"Detections: {len(self.latest_measurements)} landmarks\n")
        for i, meas in enumerate(self.latest_measurements[:3]):  # Show first 3
            self.log_file.write(f"  LM{meas['landmark_id']}: range={meas['range']:.1f}m, bearing={np.degrees(meas['bearing']):6.1f}°\n")

        # Number of mapped landmarks
        self.log_file.write(f"Mapped landmarks: {self.ekf_slam.num_landmarks}\n")

        # Measurement rejection statistics
        stats = self.ekf_slam.get_rejection_stats()
        self.log_file.write(f"Measurement Stats: Accepted={stats['accepted']}, Rejected={stats['rejected']}, Rate={stats['rejection_rate']:.1f}%\n")

        self.log_file.write(f"{'='*60}\n")
        self.log_file.flush()  # Ensure data is written immediately

    def get_state(self):
        """Get current simulation state."""
        return {
            'time': self.time,
            'robot_state': self.robot.state.copy(),
            'laps': self.laps_completed,
            'trajectory': self.robot.get_trajectory()
        }

    def reset(self):
        """Reset simulation."""
        start_x = self.track_centerline[0, 0]
        start_y = self.track_centerline[0, 1]
        dx = self.track_centerline[1, 0] - self.track_centerline[0, 0]
        dy = self.track_centerline[1, 1] - self.track_centerline[0, 1]
        start_theta = np.arctan2(dy, dx)

        self.robot.reset(start_x, start_y, start_theta)
        self.controller.reset()
        self.time = 0.0
        self.laps_completed = 0

    def _record_metrics(self):
        """Record current performance metrics."""
        # Get current poses
        slam_pose = self.ekf_slam.robot_pose
        true_pose = np.array([self.robot.x, self.robot.y, self.robot.theta])

        # Calculate errors
        pos_error = np.linalg.norm(slam_pose[:2] - true_pose[:2])
        x_error = slam_pose[0] - true_pose[0]
        y_error = slam_pose[1] - true_pose[1]
        theta_error = slam_pose[2] - true_pose[2]
        theta_error = np.arctan2(np.sin(theta_error), np.cos(theta_error))  # Wrap to [-pi, pi]

        # Get uncertainties (standard deviations)
        cov_diag = np.sqrt(np.diag(self.ekf_slam.robot_covariance))

        # Compute consistency metrics
        nees = self.ekf_slam.compute_nees(true_pose)
        nis = self.ekf_slam.get_average_nis()

        # Record metrics
        self.metrics_history['time'].append(self.time)
        self.metrics_history['pos_error'].append(pos_error)
        self.metrics_history['x_error'].append(x_error)
        self.metrics_history['y_error'].append(y_error)
        self.metrics_history['theta_error'].append(np.degrees(theta_error))  # Store in degrees
        self.metrics_history['uncertainty_x'].append(cov_diag[0])
        self.metrics_history['uncertainty_y'].append(cov_diag[1])
        self.metrics_history['uncertainty_theta'].append(np.degrees(cov_diag[2]))  # Store in degrees
        self.metrics_history['num_landmarks_mapped'].append(self.ekf_slam.num_landmarks)
        self.metrics_history['num_landmarks_detected'].append(len(self.latest_measurements))
        self.metrics_history['loop_closures'].append(self.ekf_slam.loop_closures_detected)
        self.metrics_history['laps_completed'].append(self.laps_completed)
        self.metrics_history['nees'].append(nees)
        self.metrics_history['nis'].append(nis)

    def save_metrics(self, filename='slam_metrics.csv'):
        """Save performance metrics to CSV file."""
        import csv
        from pathlib import Path

        # Create metrics directory
        metrics_dir = Path(__file__).parent.parent / "logs"
        metrics_dir.mkdir(exist_ok=True)
        filepath = metrics_dir / filename

        # Write to CSV
        with open(filepath, 'w', newline='') as f:
            writer = csv.writer(f)

            # Write header
            writer.writerow([
                'Time (s)',
                'Position Error (m)',
                'X Error (m)',
                'Y Error (m)',
                'Theta Error (deg)',
                'Uncertainty X (m)',
                'Uncertainty Y (m)',
                'Uncertainty Theta (deg)',
                'Landmarks Mapped',
                'Landmarks Detected',
                'Loop Closures',
                'Laps Completed',
                'NEES',
                'NIS'
            ])

            # Write data
            for i in range(len(self.metrics_history['time'])):
                writer.writerow([
                    self.metrics_history['time'][i],
                    self.metrics_history['pos_error'][i],
                    self.metrics_history['x_error'][i],
                    self.metrics_history['y_error'][i],
                    self.metrics_history['theta_error'][i],
                    self.metrics_history['uncertainty_x'][i],
                    self.metrics_history['uncertainty_y'][i],
                    self.metrics_history['uncertainty_theta'][i],
                    self.metrics_history['num_landmarks_mapped'][i],
                    self.metrics_history['num_landmarks_detected'][i],
                    self.metrics_history['loop_closures'][i],
                    self.metrics_history['laps_completed'][i],
                    self.metrics_history['nees'][i],
                    self.metrics_history['nis'][i]
                ])

        print(f"Metrics saved to: {filepath}")
        return filepath

    def close(self):
        """Close log file and cleanup resources."""
        # Save metrics before closing
        if hasattr(self, 'metrics_history') and len(self.metrics_history['time']) > 0:
            self.save_metrics()

        # Close log file
        if hasattr(self, 'log_file') and self.log_file is not None:
            self.log_file.close()
            self.log_file = None

    def __del__(self):
        """Destructor to ensure log file is closed."""
        self.close()


def run_simulation(enable_noise=False, num_laps=3, camera_mode='follow', use_racing_line=True):
    """
    Run robot simulation with visualization.

    Parameters:
    -----------
    enable_noise : bool
        Whether to enable motion and sensor noise
    num_laps : int
        Number of laps to run
    camera_mode : str
        'follow' for 3rd person camera, 'overview' for static top view
    use_racing_line : bool
        Whether to use racing line (True) or centerline (False)
    """
    # Load track data
    track_file = parent_dir / 'environment' / 'track_npy' / 'simple_oval_2d.npy'
    width_file = parent_dir / 'environment' / 'track_npy' / 'simple_oval_width.npy'

    track_data = np.load(track_file)
    width_data = np.load(width_file)

    # Simple track is already in correct scale (no scaling needed)
    track_centerline = track_data

    # Load landmarks
    landmarks = load_landmarks('simple_oval_landmarks.npy')
    print(f"Loaded {len(landmarks)} landmarks")

    # Generate racing line if requested
    if use_racing_line:
        print("Generating optimal racing line...")
        track_width_left = width_data[:, 0]
        track_width_right = width_data[:, 1]

        racing_line = generate_racing_line(
            track_centerline,
            track_width_left,
            track_width_right,
            aggression=0.85,  # Use 85% of available track width for optimal line
            smoothing=15  # More smoothing for gentler curves
        )

        # Use racing line for path following
        path_to_follow = racing_line
        path_name = "Racing Line"
    else:
        path_to_follow = track_centerline
        path_name = "Centerline"

    print("=" * 60)
    print("ROBOT SIMULATION - Phase 2")
    print("=" * 60)
    print(f"Track: Simple Oval ({len(track_centerline)} points)")
    print(f"Path: {path_name}")
    print(f"Noise (motion + sensor): {'Enabled' if enable_noise else 'Disabled'}")
    print(f"Camera mode: {camera_mode}")
    print(f"Target laps: {num_laps}")
    print("=" * 60)

    # Create simulation with chosen path (60 Hz for smooth motion)
    sim = RobotSimulation(path_to_follow, landmarks, dt=0.017, enable_noise=enable_noise, camera_mode=camera_mode)

    # Create simple track visualization
    scene = window.Scene()

    # Create track surface from centerline and width
    vertices = []
    faces = []

    track_width_left = width_data[:, 0]
    track_width_right = width_data[:, 1]

    # Generate left and right boundaries
    for i in range(len(track_centerline)):
        # Calculate tangent
        if i < len(track_centerline) - 1:
            tangent = track_centerline[i + 1] - track_centerline[i]
        else:
            tangent = track_centerline[i] - track_centerline[i - 1]

        tangent_len = np.linalg.norm(tangent)
        if tangent_len > 0:
            tangent = tangent / tangent_len
            normal = np.array([-tangent[1], tangent[0]])
        else:
            normal = np.array([0, 1])

        # Left and right points
        left_pt = track_centerline[i] + normal * track_width_left[i]
        right_pt = track_centerline[i] - normal * track_width_right[i]

        vertices.append([left_pt[0], 0, left_pt[1]])
        vertices.append([right_pt[0], 0, right_pt[1]])

    vertices = np.array(vertices)

    # Create faces (triangles) for the track surface
    for i in range(len(track_centerline) - 1):
        idx = i * 2
        # Two triangles per quad
        faces.append([idx, idx + 1, idx + 2])
        faces.append([idx + 1, idx + 3, idx + 2])

    # Close the loop
    idx = (len(track_centerline) - 1) * 2
    faces.append([idx, idx + 1, 0])
    faces.append([idx + 1, 1, 0])

    faces = np.array(faces)

    # Create gray track surface
    gray_color = np.array([0.4, 0.4, 0.4])
    colors = np.tile(gray_color, (len(vertices), 1))

    track_surface = actor.surface(vertices, faces=faces, colors=colors)
    scene.add(track_surface)

    # Visualize the centerline path
    centerline_3d = np.column_stack([
        track_centerline[:, 0],
        np.ones(len(track_centerline)) * 0.3,  # Slightly above ground
        track_centerline[:, 1]
    ])

    centerline_actor = actor.line(
        [centerline_3d],
        colors=(0.0, 1.0, 0.0),  # Green for centerline
        linewidth=3
    )
    scene.add(centerline_actor)

    # Visualize landmarks as cuboidal blocks
    landmark_positions = []
    landmark_colors = []
    landmark_scales = []

    for lm in landmarks:
        pos = lm['position']
        landmark_positions.append([pos[0], 2.0, pos[1]])  # Height = 2m
        landmark_colors.append([1.0, 0.5, 0.0])  # Orange
        landmark_scales.append([2.0, 4.0, 2.0])  # 2m x 4m x 2m

    landmark_blocks = actor.box(
        centers=np.array(landmark_positions),
        colors=np.array(landmark_colors),
        scales=np.array(landmark_scales)
    )
    scene.add(landmark_blocks)

    # Improve lighting
    scene.reset_clipping_range()

    # Get the scene's renderer
    scene_renderer = scene

    # Set ambient lighting (overall brightness)
    scene_renderer.SetAmbient(0.4, 0.4, 0.4)  # Increase ambient light

    # Note: FURY handles lighting automatically, but we can adjust background brightness
    scene.background((0.8, 0.9, 1.0))  # Lighter sky blue for better visibility

    # Visualize the racing line path
    if use_racing_line:
        racing_line_3d = np.column_stack([
            path_to_follow[:, 0],
            np.ones(len(path_to_follow)) * 0.3,  # Slightly above ground
            path_to_follow[:, 1]
        ])

        racing_line_actor = actor.line(
            [racing_line_3d],
            colors=(0.0, 1.0, 0.0),  # Green for racing line
            linewidth=3
        )
        scene.add(racing_line_actor)

    # Set up robot visualization
    sim.setup_visualization(scene)

    # Set initial camera position
    if camera_mode == 'overview':
        # Static top-down view
        center_x = np.mean(track_centerline[:, 0])
        center_z = np.mean(track_centerline[:, 1])
        scene.set_camera(
            position=(center_x, 800, center_z + 500),
            focal_point=(center_x, 0, center_z),
            view_up=(0, 1, 0)
        )
    else:
        # Follow camera - will be updated in first frame
        sim._update_follow_camera()

    # Create window
    showm = window.ShowManager(
        scene=scene,
        size=(1400, 900),
        title=f"Robot Simulation - {'With Noise' if enable_noise else 'No Noise'}"
    )

    # Real-time plotting setup
    import matplotlib.pyplot as plt
    plt.ion()  # Interactive mode
    fig, axes = plt.subplots(2, 3, figsize=(15, 8))
    fig.suptitle(f'SLAM Performance Metrics - {"With Noise" if enable_noise else "No Noise"}', fontsize=14, fontweight='bold')

    ax_pos_error = axes[0, 0]
    ax_heading_error = axes[0, 1]
    ax_landmarks = axes[0, 2]
    ax_uncertainty = axes[1, 0]
    ax_nees = axes[1, 1]
    ax_nis = axes[1, 2]

    # Initialize empty plot data
    times = []
    pos_errors = []
    heading_errors = []
    landmarks_mapped = []
    uncertainties = []
    nees_values = []
    nis_values = []

    # Setup plot styling
    for ax in axes.flat:
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show(block=False)

    print("Real-time plots enabled! Plots window will update every 0.5 seconds.\n")

    # Simulation loop
    counter = 0
    max_steps = num_laps * 6000  # Adjusted for 60 FPS

    def timer_callback(_obj, _event):
        nonlocal counter

        # Run physics every frame at 60 Hz for smooth motion
        sim.step()

        # Update visualization every frame (60 FPS)
        sim.update_visualization()
        showm.render()

        # Update plots every 30 frames (~0.5 seconds)
        if counter % 30 == 0 and counter > 0:
            # Calculate metrics
            true_pos = np.array([sim.robot.x, sim.robot.y])
            est_pos = sim.ekf_slam.robot_pose[:2]
            pos_error = np.linalg.norm(true_pos - est_pos)

            true_heading = sim.robot.theta
            est_heading = sim.ekf_slam.robot_pose[2]
            heading_error = abs(np.degrees(true_heading - est_heading))

            uncertainty = np.sqrt(np.trace(sim.ekf_slam.robot_covariance[:2, :2]))

            # Get consistency metrics
            true_pose = np.array([sim.robot.x, sim.robot.y, sim.robot.theta])
            nees = sim.ekf_slam.compute_nees(true_pose)
            nis = sim.ekf_slam.get_average_nis()

            # Collect metrics
            times.append(sim.time)
            pos_errors.append(pos_error)
            heading_errors.append(heading_error)
            landmarks_mapped.append(sim.ekf_slam.num_landmarks)
            uncertainties.append(uncertainty)
            nees_values.append(nees)
            nis_values.append(nis)

            # Update plots
            ax_pos_error.clear()
            ax_pos_error.plot(times, pos_errors, 'b-', linewidth=2)
            ax_pos_error.set_title('Position Error', fontweight='bold')
            ax_pos_error.set_xlabel('Time (s)')
            ax_pos_error.set_ylabel('Error (m)')
            ax_pos_error.grid(True, alpha=0.3)

            ax_heading_error.clear()
            ax_heading_error.plot(times, heading_errors, 'r-', linewidth=2)
            ax_heading_error.set_title('Heading Error', fontweight='bold')
            ax_heading_error.set_xlabel('Time (s)')
            ax_heading_error.set_ylabel('Error (deg)')
            ax_heading_error.grid(True, alpha=0.3)

            ax_landmarks.clear()
            ax_landmarks.plot(times, landmarks_mapped, 'g-', linewidth=2)
            ax_landmarks.set_title('Landmarks Mapped', fontweight='bold')
            ax_landmarks.set_xlabel('Time (s)')
            ax_landmarks.set_ylabel('Count')
            ax_landmarks.grid(True, alpha=0.3)

            ax_uncertainty.clear()
            ax_uncertainty.plot(times, uncertainties, 'm-', linewidth=2)
            ax_uncertainty.set_title('Position Uncertainty', fontweight='bold')
            ax_uncertainty.set_xlabel('Time (s)')
            ax_uncertainty.set_ylabel('Uncertainty (m)')
            ax_uncertainty.grid(True, alpha=0.3)

            # NEES plot with reference lines
            ax_nees.clear()
            ax_nees.plot(times, nees_values, 'b-', linewidth=2, label='NEES')
            # Expected value for chi-squared(3) = 3
            ax_nees.axhline(y=3.0, color='g', linestyle='--', linewidth=1.5, label='Expected (3.0)')
            # 95% confidence bounds for chi-squared(3): [0.35, 7.81]
            ax_nees.axhline(y=7.81, color='r', linestyle=':', linewidth=1, label='95% bounds')
            ax_nees.axhline(y=0.35, color='r', linestyle=':', linewidth=1)
            ax_nees.set_title('NEES (Filter Consistency)', fontweight='bold')
            ax_nees.set_xlabel('Time (s)')
            ax_nees.set_ylabel('NEES')
            ax_nees.legend(loc='upper right', fontsize=8)
            ax_nees.grid(True, alpha=0.3)
            ax_nees.set_ylim([0, min(15, max(nees_values) * 1.1) if nees_values else 15])

            # NIS plot with reference lines
            ax_nis.clear()
            ax_nis.plot(times, nis_values, 'c-', linewidth=2, label='NIS')
            # Expected value for chi-squared(2) = 2
            ax_nis.axhline(y=2.0, color='g', linestyle='--', linewidth=1.5, label='Expected (2.0)')
            # 95% confidence bounds for chi-squared(2): [0.05, 5.99]
            ax_nis.axhline(y=5.99, color='r', linestyle=':', linewidth=1, label='95% bounds')
            ax_nis.axhline(y=0.05, color='r', linestyle=':', linewidth=1)
            ax_nis.set_title('NIS (Measurement Consistency)', fontweight='bold')
            ax_nis.set_xlabel('Time (s)')
            ax_nis.set_ylabel('NIS')
            ax_nis.legend(loc='upper right', fontsize=8)
            ax_nis.grid(True, alpha=0.3)
            ax_nis.set_ylim([0, min(10, max(nis_values) * 1.1) if nis_values else 10])

            plt.tight_layout()
            fig.canvas.draw()
            fig.canvas.flush_events()

        counter += 1

        # Stop after target laps
        if sim.laps_completed >= num_laps or counter >= num_laps * 6000:
            print(f"\nSimulation complete!")
            print(f"Time: {sim.time:.1f}s")
            print(f"Laps: {sim.laps_completed}")
            print(f"Final position: ({sim.robot.x:.1f}, {sim.robot.y:.1f})")
            print(f"Log saved to: logs/slam_log.txt")

            # Show measurement rejection statistics
            stats = sim.ekf_slam.get_rejection_stats()
            print(f"\n=== SLAM Measurement Statistics ===")
            print(f"Accepted: {stats['accepted']}")
            print(f"Rejected: {stats['rejected']}")
            print(f"Rejection rate: {stats['rejection_rate']:.1f}%")

            # Close matplotlib window
            plt.ioff()
            plt.close(fig)

            sim.close()  # Close log file
            showm.exit()

    # Add timer callback for 60 FPS
    showm.add_timer_callback(True, 17, timer_callback)  # ~17ms ≈ 60 FPS

    # Add keyboard callback for quit functionality
    def cleanup_and_exit():
        """Clean shutdown of simulation"""
        print("\n\nShutting down simulation...")
        plt.ioff()
        plt.close(fig)
        sim.close()
        print("Simulation stopped. Final metrics saved.")
        showm.exit()

    def keyboard_callback(obj, _event):
        """Handle keyboard events"""
        key = obj.GetKeySym()
        if key.lower() in ['q', 'escape']:
            cleanup_and_exit()

    # Register keyboard event handler
    showm.iren.AddObserver('KeyPressEvent', keyboard_callback)

    print("\nStarting simulation...")
    print("Controls:")
    print("- Left mouse: Rotate view")
    print("- Right mouse: Zoom")
    print("- Middle mouse: Pan")
    print("- Press 'q' or ESC to quit\n")

    showm.start()


def run_headless_simulation(
    enable_motion_noise=False,
    enable_sensor_noise=False,
    num_laps=4,
    motion_noise_diag=None,
    measurement_noise_scale=None,
    validation_gate=3.0,
    sensor_noise_type='gaussian',
    output_dir=None,
    test_name='test',
    seed=None,
    use_racing_line=False
):
    """
    Run simulation in headless mode (no 3D visualization).

    Saves:
    - CSV metrics
    - PNG plots
    - Summary statistics
    - Configuration file

    Parameters:
    -----------
    enable_motion_noise : bool
        Whether to enable motion noise in robot odometry
    enable_sensor_noise : bool
        Whether to enable noise in sensor measurements
    num_laps : int
        Number of laps to complete
    motion_noise_diag : list of float, optional
        Motion noise diagonal [x, y, theta] for EKF
    measurement_noise_scale : float, optional
        Scale factor for measurement noise
    validation_gate : float
        Mahalanobis distance threshold for outlier rejection
    sensor_noise_type : str
        Type of sensor noise distribution
    output_dir : Path or str, optional
        Directory to save outputs
    test_name : str
        Name of the test
    seed : int, optional
        Random seed for reproducibility
    use_racing_line : bool
        Whether to use racing line

    Returns:
    --------
    results : dict
        Summary statistics and file paths
    """
    # Set random seed if provided
    if seed is not None:
        np.random.seed(seed)
        print(f"Random seed set to: {seed}")

    # Load track data
    track_file = parent_dir / 'environment' / 'track_npy' / 'simple_oval_2d.npy'
    width_file = parent_dir / 'environment' / 'track_npy' / 'simple_oval_width.npy'

    track_data = np.load(track_file)
    width_data = np.load(width_file)
    track_centerline = track_data

    # Load landmarks
    landmarks = load_landmarks('simple_oval_landmarks.npy')
    print(f"Loaded {len(landmarks)} landmarks")

    # Generate racing line if requested
    if use_racing_line:
        print("Generating optimal racing line...")
        track_width_left = width_data[:, 0]
        track_width_right = width_data[:, 1]

        racing_line = generate_racing_line(
            track_centerline,
            track_width_left,
            track_width_right,
            aggression=0.85,
            smoothing=15
        )
        path_to_follow = racing_line
        path_name = "Racing Line"
    else:
        path_to_follow = track_centerline
        path_name = "Centerline"

    print("=" * 60)
    print(f"HEADLESS SIMULATION - {test_name}")
    print("=" * 60)
    print(f"Track: Simple Oval ({len(track_centerline)} points)")
    print(f"Path: {path_name}")
    print(f"Motion noise: {'Enabled' if enable_motion_noise else 'Disabled'}")
    print(f"Sensor noise: {'Enabled' if enable_sensor_noise else 'Disabled'} ({sensor_noise_type})")
    print(f"Validation gate: {validation_gate}")
    print(f"Target laps: {num_laps}")
    print("=" * 60)

    # Create simulation with custom noise parameters
    start_x = path_to_follow[0, 0]
    start_y = path_to_follow[0, 1]
    dx = path_to_follow[1, 0] - path_to_follow[0, 0]
    dy = path_to_follow[1, 1] - path_to_follow[0, 1]
    start_theta = np.arctan2(dy, dx)

    dt = 0.017  # 60 Hz
    robot = Robot(start_x, start_y, start_theta, dt=dt, sensor_noise_type=sensor_noise_type)

    if enable_motion_noise:
        robot.enable_motion_noise()

    controller = PathController(
        path=path_to_follow,
        desired_speed=15.0,
        lookahead_distance=12.0
    )

    # Initialize EKF-SLAM with custom noise parameters
    initial_pose = np.array([start_x, start_y, start_theta])

    if motion_noise_diag is not None:
        motion_noise = np.diag(motion_noise_diag)**2
    elif enable_motion_noise:
        motion_noise = np.diag([0.5, 0.5, 0.08])**2
    else:
        motion_noise = np.diag([0.01, 0.01, 0.001])**2

    if measurement_noise_scale is not None:
        measurement_noise = robot.sensor.Q * measurement_noise_scale
    elif enable_sensor_noise:
        measurement_noise = robot.sensor.Q * 0.3
    else:
        measurement_noise = robot.sensor.Q * 0.01

    ekf_slam = EKF_SLAM(initial_pose, motion_noise, measurement_noise, validation_gate=validation_gate)

    # Metrics tracking
    metrics_history = {
        'time': [],
        'pos_error': [],
        'x_error': [],
        'y_error': [],
        'theta_error': [],
        'uncertainty_x': [],
        'uncertainty_y': [],
        'uncertainty_theta': [],
        'num_landmarks_mapped': [],
        'num_landmarks_detected': [],
        'loop_closures': [],
        'laps_completed': [],
        'nees': [],
        'nis': []
    }

    # Simulation state
    time = 0.0
    laps_completed = 0
    track_length = 711.7
    total_distance = 0.0
    last_position = np.array([start_x, start_y])
    slam_update_counter = 0
    slam_update_interval = 2

    print("\nRunning simulation...")
    max_steps = num_laps * 6000

    for step in range(max_steps):
        # Get control commands
        v, omega = controller.compute_control(robot.x, robot.y, robot.theta)

        # EKF prediction
        ekf_slam.predict(v, omega, dt)

        # Move robot (ground truth)
        robot.move(v, omega)

        # Get sensor measurements
        latest_measurements = robot.sensor.measure(
            robot.x, robot.y, robot.theta,
            landmarks,
            add_noise=enable_sensor_noise
        )

        # EKF update
        slam_update_counter += 1
        if slam_update_counter >= slam_update_interval:
            slam_update_counter = 0

            if len(latest_measurements) > 0:
                ground_truth_pose = np.array([robot.x, robot.y, robot.theta])
                ekf_slam.update(latest_measurements, robot.sensor, ground_truth_pose)

                current_landmark_ids = [m['landmark_id'] for m in latest_measurements]
                ekf_slam.store_pose_history(current_landmark_ids)

                closure_detected, closure_pose = ekf_slam.detect_loop_closure(
                    current_landmark_ids,
                    distance_threshold=5.0,
                    landmark_threshold=3
                )

                if closure_detected:
                    ekf_slam.apply_loop_closure_correction(closure_pose)

        # Record metrics
        slam_pose = ekf_slam.robot_pose
        true_pose = np.array([robot.x, robot.y, robot.theta])

        pos_error = np.linalg.norm(slam_pose[:2] - true_pose[:2])
        x_error = slam_pose[0] - true_pose[0]
        y_error = slam_pose[1] - true_pose[1]
        theta_error = slam_pose[2] - true_pose[2]
        theta_error = np.arctan2(np.sin(theta_error), np.cos(theta_error))

        cov_diag = np.sqrt(np.diag(ekf_slam.robot_covariance))
        nees = ekf_slam.compute_nees(true_pose)
        nis = ekf_slam.get_average_nis()

        metrics_history['time'].append(time)
        metrics_history['pos_error'].append(pos_error)
        metrics_history['x_error'].append(x_error)
        metrics_history['y_error'].append(y_error)
        metrics_history['theta_error'].append(np.degrees(theta_error))
        metrics_history['uncertainty_x'].append(cov_diag[0])
        metrics_history['uncertainty_y'].append(cov_diag[1])
        metrics_history['uncertainty_theta'].append(np.degrees(cov_diag[2]))
        metrics_history['num_landmarks_mapped'].append(ekf_slam.num_landmarks)
        metrics_history['num_landmarks_detected'].append(len(latest_measurements))
        metrics_history['loop_closures'].append(ekf_slam.loop_closures_detected)
        metrics_history['laps_completed'].append(laps_completed)
        metrics_history['nees'].append(nees)
        metrics_history['nis'].append(nis)

        # Update time
        time += dt

        # Lap tracking
        current_pos = np.array([robot.x, robot.y])
        step_distance = np.linalg.norm(current_pos - last_position)
        total_distance += step_distance
        last_position = current_pos.copy()

        new_laps = int(total_distance / track_length)
        if new_laps > laps_completed:
            laps_completed = new_laps
            print(f"Lap {laps_completed} completed at t={time:.1f}s")

        # Stop after target laps
        if laps_completed >= num_laps:
            break

        # Progress indicator
        if step % 1000 == 0:
            print(f"  Step {step}/{max_steps}, t={time:.1f}s, laps={laps_completed}/{num_laps}")

    print(f"\nSimulation complete!")
    print(f"Time: {time:.1f}s")
    print(f"Laps: {laps_completed}")

    # Create output directory
    if output_dir is None:
        output_dir = parent_dir / 'results' / test_name
    else:
        output_dir = Path(output_dir)

    output_dir.mkdir(parents=True, exist_ok=True)

    # Save metrics to CSV
    csv_path = output_dir / 'metrics.csv'
    import csv
    with open(csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            'Time (s)', 'Position Error (m)', 'X Error (m)', 'Y Error (m)',
            'Theta Error (deg)', 'Uncertainty X (m)', 'Uncertainty Y (m)',
            'Uncertainty Theta (deg)', 'Landmarks Mapped', 'Landmarks Detected',
            'Loop Closures', 'Laps Completed', 'NEES', 'NIS'
        ])
        for i in range(len(metrics_history['time'])):
            writer.writerow([
                metrics_history['time'][i],
                metrics_history['pos_error'][i],
                metrics_history['x_error'][i],
                metrics_history['y_error'][i],
                metrics_history['theta_error'][i],
                metrics_history['uncertainty_x'][i],
                metrics_history['uncertainty_y'][i],
                metrics_history['uncertainty_theta'][i],
                metrics_history['num_landmarks_mapped'][i],
                metrics_history['num_landmarks_detected'][i],
                metrics_history['loop_closures'][i],
                metrics_history['laps_completed'][i],
                metrics_history['nees'][i],
                metrics_history['nis'][i]
            ])
    print(f"Metrics saved to: {csv_path}")

    # Generate plots
    print("Generating plots...")
    plot_path = generate_plots_from_metrics(metrics_history, output_dir, test_name)

    # Generate summary statistics
    print("Generating summary statistics...")
    stats_path = generate_summary_stats(metrics_history, ekf_slam, output_dir, test_name)

    # Save configuration
    import json
    config_path = output_dir / 'config.json'
    config = {
        'test_name': test_name,
        'enable_motion_noise': enable_motion_noise,
        'enable_sensor_noise': enable_sensor_noise,
        'num_laps': num_laps,
        'motion_noise_diag': motion_noise_diag.tolist() if hasattr(motion_noise_diag, 'tolist') else motion_noise_diag,
        'measurement_noise_scale': measurement_noise_scale,
        'validation_gate': validation_gate,
        'sensor_noise_type': sensor_noise_type,
        'seed': seed,
        'use_racing_line': use_racing_line,
        'total_time': time,
        'laps_completed': laps_completed
    }
    with open(config_path, 'w') as f:
        json.dump(config, f, indent=2)
    print(f"Configuration saved to: {config_path}")

    return {
        'csv_path': csv_path,
        'plot_path': plot_path,
        'stats_path': stats_path,
        'config_path': config_path,
        'metrics': metrics_history
    }


def generate_plots_from_metrics(metrics_history, output_dir, test_name):
    """Generate PNG plots from metrics history."""
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 3, figsize=(15, 8))
    fig.suptitle(f'SLAM Performance Metrics - {test_name}', fontsize=14, fontweight='bold')

    times = metrics_history['time']

    # Position Error
    ax = axes[0, 0]
    ax.plot(times, metrics_history['pos_error'], 'b-', linewidth=2)
    ax.set_title('Position Error', fontweight='bold')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Error (m)')
    ax.grid(True, alpha=0.3)

    # Heading Error
    ax = axes[0, 1]
    ax.plot(times, metrics_history['theta_error'], 'r-', linewidth=2)
    ax.set_title('Heading Error', fontweight='bold')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Error (deg)')
    ax.grid(True, alpha=0.3)

    # Landmarks Mapped
    ax = axes[0, 2]
    ax.plot(times, metrics_history['num_landmarks_mapped'], 'g-', linewidth=2)
    ax.set_title('Landmarks Mapped', fontweight='bold')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Count')
    ax.grid(True, alpha=0.3)

    # Position Uncertainty
    ax = axes[1, 0]
    uncertainties = [np.sqrt(metrics_history['uncertainty_x'][i]**2 + metrics_history['uncertainty_y'][i]**2)
                     for i in range(len(times))]
    ax.plot(times, uncertainties, 'm-', linewidth=2)
    ax.set_title('Position Uncertainty', fontweight='bold')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Uncertainty (m)')
    ax.grid(True, alpha=0.3)

    # NEES
    ax = axes[1, 1]
    ax.plot(times, metrics_history['nees'], 'b-', linewidth=2, label='NEES')
    ax.axhline(y=3.0, color='g', linestyle='--', linewidth=1.5, label='Expected (3.0)')
    ax.axhline(y=7.81, color='r', linestyle=':', linewidth=1, label='95% bounds')
    ax.axhline(y=0.35, color='r', linestyle=':', linewidth=1)
    ax.set_title('NEES (Filter Consistency)', fontweight='bold')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('NEES')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(True, alpha=0.3)
    if metrics_history['nees']:
        max_nees = max(metrics_history['nees'])
        ax.set_ylim([0, max(15, max_nees * 1.1)])  # At least 15
    else:
        ax.set_ylim([0, 15])

    # NIS
    ax = axes[1, 2]
    ax.plot(times, metrics_history['nis'], 'c-', linewidth=2, label='NIS')
    ax.axhline(y=2.0, color='g', linestyle='--', linewidth=1.5, label='Expected (2.0)')
    ax.axhline(y=5.99, color='r', linestyle=':', linewidth=1, label='95% bounds')
    ax.axhline(y=0.05, color='r', linestyle=':', linewidth=1)
    ax.set_title('NIS (Measurement Consistency)', fontweight='bold')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('NIS')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(True, alpha=0.3)
    if metrics_history['nis']:
        max_nis = max(metrics_history['nis'])
        ax.set_ylim([0, max(10, max_nis * 1.1)])  # At least 10
    else:
        ax.set_ylim([0, 10])

    plt.tight_layout()

    plot_path = output_dir / 'plots.png'
    plt.savefig(plot_path, dpi=150, bbox_inches='tight')
    plt.close(fig)

    print(f"Plots saved to: {plot_path}")
    return plot_path


def generate_summary_stats(metrics_history, ekf_slam, output_dir, test_name):
    """Generate summary statistics file."""
    stats_path = output_dir / 'summary.txt'

    pos_errors = np.array(metrics_history['pos_error'])
    theta_errors = np.array(metrics_history['theta_error'])
    nees_values = np.array(metrics_history['nees'])
    nis_values = np.array(metrics_history['nis'])

    # Filter out zeros from NIS/NEES
    nees_nonzero = nees_values[nees_values > 0]
    nis_nonzero = nis_values[nis_values > 0]

    with open(stats_path, 'w') as f:
        f.write(f"SLAM Performance Summary - {test_name}\n")
        f.write("=" * 60 + "\n\n")

        f.write("Position Error Statistics:\n")
        f.write(f"  Mean:     {np.mean(pos_errors):.4f} m\n")
        f.write(f"  Std Dev:  {np.std(pos_errors):.4f} m\n")
        f.write(f"  Max:      {np.max(pos_errors):.4f} m\n")
        f.write(f"  Final:    {pos_errors[-1]:.4f} m\n\n")

        f.write("Heading Error Statistics:\n")
        f.write(f"  Mean:     {np.mean(theta_errors):.4f} deg\n")
        f.write(f"  Std Dev:  {np.std(theta_errors):.4f} deg\n")
        f.write(f"  Max:      {np.max(np.abs(theta_errors)):.4f} deg\n")
        f.write(f"  Final:    {theta_errors[-1]:.4f} deg\n\n")

        f.write("Filter Consistency (NEES):\n")
        f.write(f"  Mean:     {np.mean(nees_nonzero):.4f} (expected: 3.0)\n")
        f.write(f"  Std Dev:  {np.std(nees_nonzero):.4f}\n")
        f.write(f"  % in 95% bounds [0.35, 7.81]: {np.sum((nees_nonzero >= 0.35) & (nees_nonzero <= 7.81)) / len(nees_nonzero) * 100:.1f}%\n\n")

        f.write("Measurement Consistency (NIS):\n")
        f.write(f"  Mean:     {np.mean(nis_nonzero):.4f} (expected: 2.0)\n")
        f.write(f"  Std Dev:  {np.std(nis_nonzero):.4f}\n")
        f.write(f"  % in 95% bounds [0.05, 5.99]: {np.sum((nis_nonzero >= 0.05) & (nis_nonzero <= 5.99)) / len(nis_nonzero) * 100:.1f}%\n\n")

        f.write("Landmarks:\n")
        f.write(f"  Total mapped: {ekf_slam.num_landmarks}\n")
        f.write(f"  Loop closures: {ekf_slam.loop_closures_detected}\n\n")

        stats = ekf_slam.get_rejection_stats()
        f.write("Measurement Statistics:\n")
        f.write(f"  Accepted: {stats['accepted']}\n")
        f.write(f"  Rejected: {stats['rejected']}\n")
        f.write(f"  Rejection rate: {stats['rejection_rate']:.1f}%\n")

    print(f"Summary statistics saved to: {stats_path}")
    return stats_path


if __name__ == "__main__":
    # Run without noise, using centerline (not racing line) on simple track
    # 4 laps to show convergence over multiple circuits
    run_simulation(enable_noise=False, num_laps=4, use_racing_line=False)
