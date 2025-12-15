"""
Test configurations for SLAM simulation experiments.

Each configuration specifies noise parameters, validation gates,
and other settings for systematic testing of the EKF-SLAM system.
"""

import numpy as np

# Global settings
SEED = 2  # Random seed for reproducibility
NUM_LAPS = 4  # Number of laps for each test
USE_RACING_LINE = False  # Use centerline by default

# Test configurations dictionary
TEST_CONFIGS = {
    # ========== Core Gaussian Noise Tests ==========

    'baseline_perfect': {
        'description': 'Perfect sensors and odometry - theoretical best case',
        'enable_motion_noise': False,
        'enable_sensor_noise': False,
        'motion_noise_diag': [0.01, 0.01, 0.001],
        'measurement_noise_scale': 0.01,
        'validation_gate': 3.0,
        'sensor_noise_type': 'gaussian',
        'seed': SEED,
        'num_laps': NUM_LAPS,
        'use_racing_line': USE_RACING_LINE
    },

    'sensor_noise_only': {
        'description': 'High sensor noise with near-perfect odometry',
        'enable_motion_noise': False,  # Perfect odometry
        'enable_sensor_noise': True,   # Add sensor noise
        'motion_noise_diag': [0.01, 0.01, 0.001],  # Nearly perfect odometry in filter
        'measurement_noise_scale': 0.9,  # 90% sensor noise
        'validation_gate': 3.0,
        'sensor_noise_type': 'gaussian',
        'seed': SEED,
        'num_laps': NUM_LAPS,
        'use_racing_line': USE_RACING_LINE
    },

    'moderate_noise': {
        'description': 'Moderate motion uncertainty, high sensor noise',
        'enable_motion_noise': True,
        'enable_sensor_noise': True,
        'motion_noise_diag': [0.25, 0.25, 0.04],
        'measurement_noise_scale': 0.9,
        'validation_gate': 3.0,
        'sensor_noise_type': 'gaussian',
        'seed': SEED,
        'num_laps': NUM_LAPS,
        'use_racing_line': USE_RACING_LINE
    },

    'high_noise': {
        'description': 'High motion uncertainty, high sensor noise - challenging scenario',
        'enable_motion_noise': True,
        'enable_sensor_noise': True,
        'motion_noise_diag': [0.5, 0.5, 0.08],
        'measurement_noise_scale': 0.9,
        'validation_gate': 3.0,
        'sensor_noise_type': 'gaussian',
        'seed': SEED,
        'num_laps': NUM_LAPS,
        'use_racing_line': USE_RACING_LINE
    },

    # ========== Filter Tuning Tests ==========

    'optimistic_filter': {
        'description': 'Filter underestimates noise - expects inconsistent NEES/NIS',
        'enable_motion_noise': True,
        'enable_sensor_noise': True,
        'motion_noise_diag': [0.1, 0.1, 0.02],  # Filter thinks noise is low
        'measurement_noise_scale': 0.3,
        'validation_gate': 3.0,
        'sensor_noise_type': 'gaussian',
        'seed': SEED,
        'num_laps': NUM_LAPS,
        'use_racing_line': USE_RACING_LINE
    },

    'pessimistic_filter': {
        'description': 'Filter overestimates noise - expects low NEES/NIS',
        'enable_motion_noise': True,
        'enable_sensor_noise': True,
        'motion_noise_diag': [1.0, 1.0, 0.15],  # Filter thinks noise is high
        'measurement_noise_scale': 2.0,
        'validation_gate': 3.0,
        'sensor_noise_type': 'gaussian',
        'seed': SEED,
        'num_laps': NUM_LAPS,
        'use_racing_line': USE_RACING_LINE
    },

    # ========== Validation Gate Tests ==========

    'tight_validation_gate': {
        'description': 'Stricter outlier rejection (2-sigma)',
        'enable_motion_noise': True,
        'enable_sensor_noise': True,
        'motion_noise_diag': [0.25, 0.25, 0.04],
        'measurement_noise_scale': 0.9,
        'validation_gate': 2.0,  # Stricter than default 3.0
        'sensor_noise_type': 'gaussian',
        'seed': SEED,
        'num_laps': NUM_LAPS,
        'use_racing_line': USE_RACING_LINE
    },

    'loose_validation_gate': {
        'description': 'More permissive outlier rejection (5-sigma)',
        'enable_motion_noise': True,
        'enable_sensor_noise': True,
        'motion_noise_diag': [0.25, 0.25, 0.04],
        'measurement_noise_scale': 0.9,
        'validation_gate': 5.0,  # More permissive
        'sensor_noise_type': 'gaussian',
        'seed': SEED,
        'num_laps': NUM_LAPS,
        'use_racing_line': USE_RACING_LINE
    },

    # ========== Non-Gaussian Noise Tests ==========

    'uniform_noise': {
        'description': 'Uniform noise distribution - bounded noise',
        'enable_motion_noise': True,
        'enable_sensor_noise': True,
        'motion_noise_diag': [0.25, 0.25, 0.04],
        'measurement_noise_scale': 0.9,
        'validation_gate': 3.0,
        'sensor_noise_type': 'uniform',  # Non-Gaussian
        'seed': SEED,
        'num_laps': NUM_LAPS,
        'use_racing_line': USE_RACING_LINE
    },

    'heavy_tailed_noise': {
        'description': 'Student-t noise (df=3) - occasional large outliers',
        'enable_motion_noise': True,
        'enable_sensor_noise': True,
        'motion_noise_diag': [0.25, 0.25, 0.04],
        'measurement_noise_scale': 0.9,
        'validation_gate': 3.0,
        'sensor_noise_type': 'heavy_tailed',  # Non-Gaussian
        'seed': SEED,
        'num_laps': NUM_LAPS,
        'use_racing_line': USE_RACING_LINE
    },

    'laplacian_noise': {
        'description': 'Laplacian noise - sharper peak, heavier tails',
        'enable_motion_noise': True,
        'enable_sensor_noise': True,
        'motion_noise_diag': [0.25, 0.25, 0.04],
        'measurement_noise_scale': 0.9,
        'validation_gate': 3.0,
        'sensor_noise_type': 'laplacian',  # Non-Gaussian
        'seed': SEED,
        'num_laps': NUM_LAPS,
        'use_racing_line': USE_RACING_LINE
    },

    'bimodal_noise': {
        'description': 'Bimodal noise - simulates sensor glitches',
        'enable_motion_noise': True,
        'enable_sensor_noise': True,
        'motion_noise_diag': [0.25, 0.25, 0.04],
        'measurement_noise_scale': 0.9,
        'validation_gate': 3.0,
        'sensor_noise_type': 'bimodal',  # Non-Gaussian
        'seed': SEED,
        'num_laps': NUM_LAPS,
        'use_racing_line': USE_RACING_LINE
    },

    'asymmetric_noise': {
        'description': 'Asymmetric (exponential) noise - tests bias handling',
        'enable_motion_noise': True,
        'enable_sensor_noise': True,
        'motion_noise_diag': [0.25, 0.25, 0.04],
        'measurement_noise_scale': 0.9,
        'validation_gate': 3.0,
        'sensor_noise_type': 'asymmetric',  # Non-Gaussian
        'seed': SEED,
        'num_laps': NUM_LAPS,
        'use_racing_line': USE_RACING_LINE
    }
}


def get_test_config(test_name):
    """
    Get configuration for a specific test.

    Parameters:
    -----------
    test_name : str
        Name of the test

    Returns:
    --------
    config : dict
        Test configuration
    """
    if test_name not in TEST_CONFIGS:
        raise ValueError(f"Unknown test: {test_name}. Available tests: {list(TEST_CONFIGS.keys())}")

    return TEST_CONFIGS[test_name].copy()


def get_all_test_names():
    """Get list of all available test names."""
    return list(TEST_CONFIGS.keys())


def get_gaussian_tests():
    """Get only Gaussian noise tests."""
    return [
        'baseline_perfect',
        'sensor_noise_only',
        'moderate_noise',
        'high_noise',
        'optimistic_filter',
        'pessimistic_filter',
        'tight_validation_gate',
        'loose_validation_gate'
    ]


def get_non_gaussian_tests():
    """Get only non-Gaussian noise tests."""
    return [
        'uniform_noise',
        'heavy_tailed_noise',
        'laplacian_noise',
        'bimodal_noise',
        'asymmetric_noise'
    ]


if __name__ == "__main__":
    # Print all available tests
    print("Available Test Configurations:")
    print("=" * 60)

    print("\n### Gaussian Noise Tests ###")
    for test_name in get_gaussian_tests():
        config = TEST_CONFIGS[test_name]
        print(f"\n{test_name}:")
        print(f"  {config['description']}")
        print(f"  Motion noise: {config['motion_noise_diag']}")
        print(f"  Sensor noise scale: {config['measurement_noise_scale']}")
        print(f"  Validation gate: {config['validation_gate']}")

    print("\n\n### Non-Gaussian Noise Tests ###")
    for test_name in get_non_gaussian_tests():
        config = TEST_CONFIGS[test_name]
        print(f"\n{test_name}:")
        print(f"  {config['description']}")
        print(f"  Noise type: {config['sensor_noise_type']}")
        print(f"  Motion noise: {config['motion_noise_diag']}")
        print(f"  Sensor noise scale: {config['measurement_noise_scale']}")
