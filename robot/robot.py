"""
Robot class with unicycle motion model for SLAM simulation.
"""

import numpy as np
from robot.sensor import RangeBearingSensor


class Robot:
    """
    Robot with unicycle motion model.

    State: [x, y, theta]
    - x, y: position in meters
    - theta: heading angle in radians

    Control inputs: [v, omega]
    - v: linear velocity (m/s)
    - omega: angular velocity (rad/s)
    """

    def __init__(self, x=0.0, y=0.0, theta=0.0, dt=0.1, sensor_noise_type='gaussian'):
        """
        Initialize robot.

        Parameters:
        -----------
        x, y : float
            Initial position (meters)
        theta : float
            Initial heading angle (radians)
        dt : float
            Time step for simulation (seconds)
        sensor_noise_type : str
            Type of sensor noise distribution
        """
        self.state = np.array([x, y, theta])
        self.dt = dt

        # Motion noise parameters (will be set later)
        self.motion_noise_enabled = False
        self.alpha = np.array([0.1, 0.01, 0.01, 0.1])  # Motion noise coefficients

        # Range-bearing sensor for landmark detection
        self.sensor = RangeBearingSensor(
            max_range=80.0,  # Increased range to see more landmarks
            field_of_view=np.pi,  # 180 degrees
            range_noise_std=0.1,
            bearing_noise_std=0.05,
            noise_type=sensor_noise_type
        )

        # History for visualization
        self.trajectory = [self.state.copy()]

    @property
    def x(self):
        """Current x position"""
        return self.state[0]

    @property
    def y(self):
        """Current y position"""
        return self.state[1]

    @property
    def theta(self):
        """Current heading angle"""
        return self.state[2]

    def motion_model(self, v, omega):
        """
        Unicycle motion model (deterministic).

        x_{k+1} = x_k + v·Δt·cos(θ_k)
        y_{k+1} = y_k + v·Δt·sin(θ_k)
        θ_{k+1} = θ_k + ω·Δt

        Parameters:
        -----------
        v : float
            Linear velocity (m/s)
        omega : float
            Angular velocity (rad/s)

        Returns:
        --------
        new_state : np.array
            New state [x, y, theta]
        """
        x, y, theta = self.state

        # Update state using unicycle model
        x_new = x + v * self.dt * np.cos(theta)
        y_new = y + v * self.dt * np.sin(theta)
        theta_new = theta + omega * self.dt

        # Wrap angle to [-pi, pi]
        theta_new = self._wrap_angle(theta_new)

        return np.array([x_new, y_new, theta_new])

    def move(self, v, omega):
        """
        Move robot with given control inputs.

        Parameters:
        -----------
        v : float
            Linear velocity (m/s)
        omega : float
            Angular velocity (rad/s)
        """
        # Add noise to control inputs if enabled
        if self.motion_noise_enabled:
            v_noisy, omega_noisy = self._add_motion_noise(v, omega)
        else:
            v_noisy, omega_noisy = v, omega

        # Apply motion model
        self.state = self.motion_model(v_noisy, omega_noisy)

        # Store trajectory
        self.trajectory.append(self.state.copy())

    def _add_motion_noise(self, v, omega):
        """
        Add noise to control inputs.

        Noise model from Probabilistic Robotics (Thrun et al.):
        σ_v² = α₁·v² + α₂·ω²
        σ_ω² = α₃·v² + α₄·ω²

        Parameters:
        -----------
        v : float
            Commanded linear velocity
        omega : float
            Commanded angular velocity

        Returns:
        --------
        v_noisy, omega_noisy : float
            Noisy control inputs
        """
        # Noise standard deviations
        sigma_v = np.sqrt(self.alpha[0] * v**2 + self.alpha[1] * omega**2)
        sigma_omega = np.sqrt(self.alpha[2] * v**2 + self.alpha[3] * omega**2)

        # Add Gaussian noise
        v_noisy = v + np.random.normal(0, sigma_v)
        omega_noisy = omega + np.random.normal(0, sigma_omega)

        return v_noisy, omega_noisy

    def enable_motion_noise(self, alpha=None):
        """
        Enable motion noise.

        Parameters:
        -----------
        alpha : np.array, optional
            Motion noise coefficients [α₁, α₂, α₃, α₄]
        """
        self.motion_noise_enabled = True
        if alpha is not None:
            self.alpha = np.array(alpha)
        print(f"Motion noise enabled with α = {self.alpha}")

    def disable_motion_noise(self):
        """Disable motion noise."""
        self.motion_noise_enabled = False
        print("Motion noise disabled")

    def get_trajectory(self):
        """
        Get robot trajectory.

        Returns:
        --------
        trajectory : np.array
            Trajectory as (N, 3) array
        """
        return np.array(self.trajectory)

    def reset(self, x=0.0, y=0.0, theta=0.0):
        """Reset robot to initial state."""
        self.state = np.array([x, y, theta])
        self.trajectory = [self.state.copy()]

    @staticmethod
    def _wrap_angle(angle):
        """
        Wrap angle to [-pi, pi].

        Parameters:
        -----------
        angle : float
            Angle in radians

        Returns:
        --------
        wrapped : float
            Angle wrapped to [-pi, pi]
        """
        return np.arctan2(np.sin(angle), np.cos(angle))

    def compute_motion_jacobian(self, v, omega):
        """
        Compute motion model Jacobian F.

        F = [1    0    -v·Δt·sin(θ)]
            [0    1     v·Δt·cos(θ)]
            [0    0     1           ]

        Parameters:
        -----------
        v : float
            Linear velocity
        omega : float
            Angular velocity

        Returns:
        --------
        F : np.array
            3×3 Jacobian matrix
        """
        theta = self.theta
        dt = self.dt

        F = np.array([
            [1, 0, -v * dt * np.sin(theta)],
            [0, 1,  v * dt * np.cos(theta)],
            [0, 0,  1]
        ])

        return F

    def __repr__(self):
        """String representation of robot state."""
        return f"Robot(x={self.x:.2f}, y={self.y:.2f}, θ={np.degrees(self.theta):.1f}°)"
