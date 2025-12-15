# SLAM Simulation Test Suite

Comprehensive testing framework for evaluating EKF-SLAM performance under various noise conditions.

## Quick Start

### Run All Tests
```bash
cd /home/maile/code/slam/tests
python run_simulation_tests.py
```

### Run a Single Test
```bash
python run_simulation_tests.py --test baseline_perfect
```

### Run Only Gaussian Tests
```bash
python run_simulation_tests.py --gaussian-only
```

### Run Only Non-Gaussian Tests
```bash
python run_simulation_tests.py --non-gaussian-only
```

### List Available Tests
```bash
python run_simulation_tests.py --list
```

## Test Categories

### Gaussian Noise Tests (8 tests)
1. **baseline_perfect** - Perfect sensors, no noise
2. **sensor_noise_only** - High sensor noise, perfect odometry
3. **moderate_noise** - Moderate motion + sensor noise
4. **high_noise** - High motion + sensor noise
5. **optimistic_filter** - Filter underestimates noise (tests inconsistency)
6. **pessimistic_filter** - Filter overestimates noise
7. **tight_validation_gate** - 2-sigma outlier rejection
8. **loose_validation_gate** - 5-sigma outlier rejection

### Non-Gaussian Noise Tests (5 tests)
1. **uniform_noise** - Bounded uniform distribution
2. **heavy_tailed_noise** - Student-t distribution (occasional large outliers)
3. **laplacian_noise** - Sharper peak, heavier tails than Gaussian
4. **bimodal_noise** - Mixture distribution (simulates sensor glitches)
5. **asymmetric_noise** - Exponential distribution (tests bias)

## Results Structure

Each test creates a folder in `tests/results/` with:
```
tests/
└── results/
    ├── test_name/
    │   ├── metrics.csv          # Time-series data (pos error, NEES, NIS, etc.)
    │   ├── plots.png            # 6-panel performance visualization
    │   ├── summary.txt          # Statistical summary
    │   └── config.json          # Test configuration
    └── test_suite_summary.txt   # Overall test run summary
```

## Output Files

### metrics.csv
Time-series data with columns:
- Time, Position Error, X/Y/Theta Errors
- Uncertainties, Landmarks Mapped/Detected
- Loop Closures, Laps Completed
- **NEES** (Normalized Estimation Error Squared)
- **NIS** (Normalized Innovation Squared)

### plots.png
Six subplots:
1. Position Error vs Time
2. Heading Error vs Time
3. Landmarks Mapped vs Time
4. Position Uncertainty vs Time
5. **NEES** with 95% confidence bounds [0.35, 7.81]
6. **NIS** with 95% confidence bounds [0.05, 5.99]

### summary.txt
Statistical summary:
- Position/Heading error statistics (mean, std, max, final)
- **NEES consistency** (% within 95% bounds)
- **NIS consistency** (% within 95% bounds)
- Landmarks mapped, loop closures
- Measurement acceptance/rejection rates

## Test Parameters

All tests use:
- **Random seed**: 2 (reproducible results)
- **Laps**: 4 (approximately 2.8 km total distance)
- **Path**: Centerline (not racing line)
- **Track**: Simple oval (712m perimeter, 58 landmarks)

### Noise Parameters by Test

| Test | Motion Noise (x, y, θ) | Sensor Noise Scale | Gate |
|------|------------------------|-------------------|------|
| baseline_perfect | (0.01, 0.01, 0.001) | 0.01 | 3.0 |
| sensor_noise_only | (0.01, 0.01, 0.001) | 0.90 | 3.0 |
| moderate_noise | (0.25, 0.25, 0.04) | 0.90 | 3.0 |
| high_noise | (0.50, 0.50, 0.08) | 0.90 | 3.0 |
| optimistic_filter | (0.10, 0.10, 0.02) | 0.30 | 3.0 |
| pessimistic_filter | (1.00, 1.00, 0.15) | 2.00 | 3.0 |
| tight_validation_gate | (0.25, 0.25, 0.04) | 0.90 | 2.0 |
| loose_validation_gate | (0.25, 0.25, 0.04) | 0.90 | 5.0 |

Non-Gaussian tests use moderate_noise parameters with different sensor noise distributions.

## Understanding NEES and NIS

### NEES (Normalized Estimation Error Squared)
- Measures **filter consistency** for state estimation
- Chi-squared distributed with 3 DOF (robot pose: x, y, θ)
- **Expected value**: 3.0
- **95% bounds**: [0.35, 7.81]
- Values consistently outside bounds indicate filter tuning issues

### NIS (Normalized Innovation Squared)
- Measures **measurement consistency**
- Chi-squared distributed with 2 DOF (range-bearing measurements)
- **Expected value**: 2.0
- **95% bounds**: [0.05, 5.99]
- High values suggest underestimated measurement noise
- Low values suggest overestimated measurement noise

## Expected Results

### Gaussian Tests
- **baseline_perfect**: Very low errors, NEES/NIS near expected values
- **sensor_noise_only**: Moderate errors, good NEES/NIS consistency
- **moderate/high_noise**: Higher errors, NEES/NIS should stay in bounds
- **optimistic_filter**: NEES/NIS **high** (filter too confident)
- **pessimistic_filter**: NEES/NIS **low** (filter too cautious)

### Non-Gaussian Tests
- **uniform**: Minor degradation, NEES/NIS slightly off
- **heavy_tailed**: Validation gate rejects outliers, may still work
- **laplacian**: Moderate degradation
- **bimodal**: Significant degradation, inconsistent filter
- **asymmetric**: Biased estimates, poor performance

## Customizing Tests

Edit `test_configs.py` to add new tests:

```python
TEST_CONFIGS['my_custom_test'] = {
    'description': 'My custom test',
    'enable_motion_noise': True,
    'enable_sensor_noise': True,
    'motion_noise_diag': [0.3, 0.3, 0.05],
    'measurement_noise_scale': 0.7,
    'validation_gate': 3.0,
    'sensor_noise_type': 'gaussian',
    'seed': 2,
    'num_laps': 4,
    'use_racing_line': False
}
```

Then run:
```bash
python run_simulation_tests.py --test my_custom_test
```

## Performance Notes

- Headless mode (no 3D visualization) for faster execution
- Results are automatically saved
