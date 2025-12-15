"""
Test runner for SLAM simulation experiments.

Runs multiple test configurations and organizes results.
"""

import sys
from pathlib import Path
import time
from datetime import datetime

# Add parent directory to path
parent_dir = Path(__file__).parent.parent
sys.path.insert(0, str(parent_dir))

from simulation.robot_simulation import run_headless_simulation
from tests.test_configs import (
    TEST_CONFIGS,
    get_test_config,
    get_all_test_names,
    get_gaussian_tests,
    get_non_gaussian_tests
)


def run_single_test(test_name, output_base_dir=None):
    """
    Run a single test configuration.

    Parameters:
    -----------
    test_name : str
        Name of the test to run
    output_base_dir : Path or str, optional
        Base directory for all results

    Returns:
    --------
    results : dict
        Test results including paths and metrics
    """
    print("\n" + "=" * 70)
    print(f"RUNNING TEST: {test_name}")
    print("=" * 70)

    # Get test configuration
    config = get_test_config(test_name)

    # Set up output directory (inside tests folder)
    if output_base_dir is None:
        output_base_dir = Path(__file__).parent / 'results'
    else:
        output_base_dir = Path(output_base_dir)

    output_dir = output_base_dir / test_name

    # Run simulation
    start_time = time.time()

    try:
        results = run_headless_simulation(
            enable_motion_noise=config['enable_motion_noise'],
            enable_sensor_noise=config['enable_sensor_noise'],
            num_laps=config['num_laps'],
            motion_noise_diag=config['motion_noise_diag'],
            measurement_noise_scale=config['measurement_noise_scale'],
            validation_gate=config['validation_gate'],
            sensor_noise_type=config['sensor_noise_type'],
            output_dir=output_dir,
            test_name=test_name,
            seed=config['seed'],
            use_racing_line=config['use_racing_line']
        )

        elapsed_time = time.time() - start_time
        results['elapsed_time'] = elapsed_time
        results['status'] = 'success'

        print(f"\n✓ Test completed successfully in {elapsed_time:.1f}s")
        return results

    except Exception as e:
        elapsed_time = time.time() - start_time
        print(f"\n✗ Test failed after {elapsed_time:.1f}s")
        print(f"Error: {str(e)}")

        return {
            'status': 'failed',
            'error': str(e),
            'elapsed_time': elapsed_time
        }


def run_all_tests(test_subset=None, output_base_dir=None):
    """
    Run all tests or a subset of tests.

    Parameters:
    -----------
    test_subset : list of str, optional
        List of test names to run. If None, runs all tests.
    output_base_dir : Path or str, optional
        Base directory for all results

    Returns:
    --------
    summary : dict
        Summary of all test results
    """
    if test_subset is None:
        test_names = get_all_test_names()
    else:
        test_names = test_subset

    print("\n" + "=" * 70)
    print("SLAM SIMULATION TEST SUITE")
    print("=" * 70)
    print(f"Running {len(test_names)} tests")
    print(f"Tests: {', '.join(test_names)}")
    print("=" * 70)

    # Set up results directory (inside tests folder)
    if output_base_dir is None:
        output_base_dir = Path(__file__).parent / 'results'
    else:
        output_base_dir = Path(output_base_dir)

    output_base_dir.mkdir(parents=True, exist_ok=True)

    # Run all tests
    all_results = {}
    total_start_time = time.time()

    for i, test_name in enumerate(test_names, 1):
        print(f"\n[{i}/{len(test_names)}] {test_name}")
        results = run_single_test(test_name, output_base_dir)
        all_results[test_name] = results

    total_elapsed = time.time() - total_start_time

    # Generate overall summary
    print("\n" + "=" * 70)
    print("TEST SUITE SUMMARY")
    print("=" * 70)

    successful_tests = [name for name, res in all_results.items() if res['status'] == 'success']
    failed_tests = [name for name, res in all_results.items() if res['status'] == 'failed']

    print(f"\nTotal tests run: {len(test_names)}")
    print(f"Successful: {len(successful_tests)}")
    print(f"Failed: {len(failed_tests)}")
    print(f"Total time: {total_elapsed:.1f}s ({total_elapsed/60:.1f} minutes)")

    if failed_tests:
        print("\nFailed tests:")
        for test_name in failed_tests:
            error = all_results[test_name].get('error', 'Unknown error')
            print(f"  - {test_name}: {error}")

    # Save summary to file
    summary_file = output_base_dir / 'test_suite_summary.txt'
    with open(summary_file, 'w') as f:
        f.write("SLAM Test Suite Summary\n")
        f.write("=" * 60 + "\n")
        f.write(f"Run date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(f"Total tests: {len(test_names)}\n")
        f.write(f"Successful: {len(successful_tests)}\n")
        f.write(f"Failed: {len(failed_tests)}\n")
        f.write(f"Total time: {total_elapsed:.1f}s\n\n")

        f.write("Test Results:\n")
        f.write("-" * 60 + "\n")
        for test_name in test_names:
            res = all_results[test_name]
            status = res['status']
            elapsed = res.get('elapsed_time', 0)
            f.write(f"{test_name}: {status} ({elapsed:.1f}s)\n")

            if status == 'failed':
                f.write(f"  Error: {res.get('error', 'Unknown')}\n")

        if successful_tests:
            f.write("\n\nSuccessful Tests:\n")
            f.write("-" * 60 + "\n")
            for test_name in successful_tests:
                res = all_results[test_name]
                f.write(f"\n{test_name}:\n")
                f.write(f"  Results directory: {output_base_dir / test_name}\n")
                f.write(f"  Elapsed time: {res['elapsed_time']:.1f}s\n")

    print(f"\nSummary saved to: {summary_file}")
    print("=" * 70)

    return {
        'all_results': all_results,
        'summary_file': summary_file,
        'successful': successful_tests,
        'failed': failed_tests,
        'total_time': total_elapsed
    }


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description='Run SLAM simulation tests')
    parser.add_argument(
        '--test',
        type=str,
        help='Run a specific test by name'
    )
    parser.add_argument(
        '--gaussian-only',
        action='store_true',
        help='Run only Gaussian noise tests'
    )
    parser.add_argument(
        '--non-gaussian-only',
        action='store_true',
        help='Run only non-Gaussian noise tests'
    )
    parser.add_argument(
        '--output-dir',
        type=str,
        default=None,
        help='Base directory for results (default: tests/results)'
    )
    parser.add_argument(
        '--list',
        action='store_true',
        help='List all available tests and exit'
    )

    args = parser.parse_args()

    # List tests if requested
    if args.list:
        print("\nAvailable Tests:")
        print("=" * 60)
        print("\nGaussian Noise Tests:")
        for test in get_gaussian_tests():
            print(f"  - {test}")
        print("\nNon-Gaussian Noise Tests:")
        for test in get_non_gaussian_tests():
            print(f"  - {test}")
        sys.exit(0)

    # Determine which tests to run
    if args.test:
        # Run single test
        test_names = [args.test]
    elif args.gaussian_only:
        # Run only Gaussian tests
        test_names = get_gaussian_tests()
    elif args.non_gaussian_only:
        # Run only non-Gaussian tests
        test_names = get_non_gaussian_tests()
    else:
        # Run all tests
        test_names = None

    # Run tests
    if args.test:
        # Single test mode
        results = run_single_test(args.test, args.output_dir)
        if results['status'] == 'failed':
            sys.exit(1)
    else:
        # Multi-test mode
        summary = run_all_tests(test_names, args.output_dir)
        if summary['failed']:
            sys.exit(1)
