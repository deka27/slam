"""
SLAM simulation test suite.

This package contains test configurations and runners for
systematic evaluation of the EKF-SLAM implementation.
"""

from tests.test_configs import (
    TEST_CONFIGS,
    get_test_config,
    get_all_test_names,
    get_gaussian_tests,
    get_non_gaussian_tests
)

__all__ = [
    'TEST_CONFIGS',
    'get_test_config',
    'get_all_test_names',
    'get_gaussian_tests',
    'get_non_gaussian_tests'
]
