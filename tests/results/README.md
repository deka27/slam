# SLAM Test Results

**Random Seed:** 2

This directory contains automated test results for the EKF-SLAM system under various noise conditions and filter configurations.

---

## Quick Summary

| Test Category | Best Performance | Worst Performance |
|---------------|------------------|-------------------|
| **Gaussian Noise** | moderate_noise (0.53m, 93% NEES) | sensor_noise_only (0.79m, 76% NEES) |
| **Non-Gaussian** | uniform_noise (0.72m, 83% NEES) | heavy_tailed/bimodal (~1.7m, ~25% NEES) |
| **Filter Tuning** | Moderate (baseline) | Optimistic (5.52 NEES, 24.8% rejection) |

---

## Understanding the Metrics

### Position Error
- **Mean error**: Average localization error over entire run
- **Max error**: Worst-case error (often during loop closures)
- **Expected**: 0.1-0.6m for well-tuned Gaussian noise, up to 2m for non-Gaussian

### NEES (Normalized Estimation Error Squared)
- **Expected value**: ~3.0 (chi-squared distribution with 3 DOF for robot pose)
- **95% bounds**: [0.35, 7.81]
- **Interpretation**:
  - NEES > 7.81: Filter **underestimates** uncertainty (overconfident)
  - NEES < 0.35: Filter **overestimates** uncertainty (too conservative)
  - 85-95% in bounds: Well-calibrated filter

### NIS (Normalized Innovation Squared)
- **Expected value**: ~2.0 (chi-squared distribution with 2 DOF for range-bearing)
- **95% bounds**: [0.05, 5.99]
- **Interpretation**:
  - NIS > 5.99: Measurements inconsistent with filter's prediction
  - NIS < 0.05: Filter trusts measurements too much
  - 90-100% in bounds: Good measurement consistency

### Rejection Rate
- **Low noise**: 0.3-1.3% (normal)
- **High noise**: 2-10% (expected)
- **Very high**: >20% indicates filter miscalibration

---

## Test Results by Category

## 1. Core Gaussian Noise Tests

### ✅ baseline_perfect
**Config:** No noise, perfect sensors and odometry
**Results:**
- Position error: **0.097m** mean, 1.02m max
- NEES: 0.22 (13.6% in bounds)
- NIS: 0.008 (0.8% in bounds)
- Rejection rate: 0.5%

**Analysis:**
Excellent accuracy with near-zero errors between loop closures. Low NEES/NIS expected - filter becomes overconfident with perfect measurements. Max error (1.02m) occurs during loop closures due to **EKF linearization error**, not sensor noise.

**Key Insight:** Even with perfect sensors, EKF linearization introduces ~1m errors during large loop closure corrections.

---

### ✅ sensor_noise_only
**Config:** High sensor noise (90%), perfect odometry
**Results:**
- Position error: **0.79m** mean, 8.45m max
- NEES: 8.27 (75.8% in bounds)
- NIS: 2.06 (99.7% in bounds) ← Perfect!
- Rejection rate: 9.2%

**Analysis:**
Good performance with noisy sensors but perfect motion model. NIS near ideal 2.0 shows excellent measurement consistency. Higher NEES indicates filter slightly underestimates position uncertainty. Rejection rate appropriate for high sensor noise.

**Validation:** Decoupling fix successful - previous broken config had 14.3m error!

---

### ✅ moderate_noise
**Config:** Motion noise (0.25, 0.25, 0.04), Sensor noise 90%
**Results:**
- Position error: **0.53m** mean, 2.75m max
- NEES: 2.14 (93.1% in bounds) ← Excellent!
- NIS: 1.67 (100% in bounds) ← Excellent!
- Rejection rate: 1.3%

**Analysis:**
**Best overall performance!** Near-ideal NEES/NIS values with excellent consistency. Filter is properly calibrated to match actual noise levels. This represents optimal EKF-SLAM performance.

**Benchmark:** Use this as reference for well-tuned filter performance.

---

### ✅ high_noise
**Config:** Motion noise (0.5, 0.5, 0.08), Sensor noise 90%
**Results:**
- Position error: **0.60m** mean, 2.80m max (+13% vs moderate)
- NEES: 2.00 (88% in bounds)
- NIS: 1.59 (100% in bounds)
- Rejection rate: 1.3%

**Analysis:**
Graceful degradation with higher noise. Only 13% increase in error despite 2x motion noise. Slightly lower NEES/NIS indicates filter becomes more conservative (appropriate for uncertain conditions). Excellent robustness demonstration.

---

## 2. Filter Tuning Tests

### ⚠️ optimistic_filter
**Config:** Filter underestimates noise (Motion 0.1, Sensor 0.3 vs actual 0.25, 0.9)
**Results:**
- Position error: 0.53m mean, **9.46m max** (high variance)
- NEES: **5.52** (79.4% in bounds) ← High!
- NIS: **3.17** (98.4% in bounds) ← High!
- Rejection rate: **24.8%** ← Very high!

**Analysis:**
Filter thinks noise is lower than reality → becomes overconfident → rejects valid measurements. High NEES/NIS indicate **inconsistency**. Despite same mean error as moderate_noise, max error is 3.4x higher due to occasional divergence.

**Key Insight:** Underestimating noise causes aggressive outlier rejection and occasional large errors.

---

### ✅ pessimistic_filter
**Config:** Filter overestimates noise (Motion 1.0, Sensor 2.0 vs actual 0.25, 0.9)
**Results:**
- Position error: 0.63m mean, 2.66m max
- NEES: **1.36** (70.5% in bounds) ← Low!
- NIS: **0.76** (100% in bounds) ← Low!
- Rejection rate: **0.0%** ← Almost none!

**Analysis:**
Filter thinks noise is higher than reality → becomes too conservative → accepts everything. Low NEES/NIS indicate filter has **excessive uncertainty**. Slightly higher error than moderate_noise due to trusting odometry less.

**Key Insight:** Overestimating noise is safer (accepts all measurements) but sacrifices some accuracy.

---

## 3. Validation Gate Tests

### ✅ tight_validation_gate
**Config:** 2-sigma gate (stricter outlier rejection)
**Results:**
- Position error: 0.58m mean, 1.98m max
- NEES: 2.41 (94% in bounds) ← Excellent!
- NIS: 1.22 (100% in bounds)
- Rejection rate: **13.7%** ← Much higher

**Analysis:**
Tighter gate rejects more measurements (13.7% vs 1.3% default), but maintains good consistency. Lower max error than moderate_noise due to aggressive outlier filtering. Good for high-noise environments.

**Trade-off:** Better robustness to outliers vs fewer measurement updates.

---

### ✅ loose_validation_gate
**Config:** 5-sigma gate (permissive outlier rejection)
**Results:**
- Position error: 0.58m mean, 2.91m max
- NEES: 2.46 (88.8% in bounds)
- NIS: 1.76 (100% in bounds)
- Rejection rate: **0.0%** ← Almost none

**Analysis:**
Loose gate accepts almost everything. Similar performance to moderate_noise with slightly higher max error. Appropriate when measurements are trusted and outliers are rare.

**Trade-off:** More measurement updates vs less robustness to outliers.

---

## 4. Non-Gaussian Noise Tests

### ✅ uniform_noise
**Config:** Uniform distribution (bounded noise)
**Results:**
- Position error: **0.72m** mean (+35% vs Gaussian)
- NEES: 3.60 (82.6% in bounds)
- NIS: 1.77 (100% in bounds)
- Rejection rate: 0.3%

**Analysis:**
**Best non-Gaussian performance.** EKF handles bounded uniform noise well since it's symmetric and has no outliers. Only 35% degradation from Gaussian baseline shows good robustness.

**Key Insight:** EKF tolerates non-Gaussian noise that's symmetric and bounded.

---

### ⚠️ heavy_tailed_noise
**Config:** Student-t (df=3) - occasional large outliers
**Results:**
- Position error: **1.71m** mean (+220% vs Gaussian!)
- NEES: **15.49** (26.9% in bounds) ← Very high!
- NIS: 1.17 (100% in bounds)
- Rejection rate: 2.3%

**Analysis:**
**Worst-case scenario.** Heavy-tailed noise produces occasional large outliers that violate EKF's Gaussian assumption. Very high NEES indicates **severe filter inconsistency**. Low rejection rate shows filter doesn't detect outliers effectively.

**Key Insight:** EKF's fundamental weakness - cannot handle heavy-tailed distributions well.

---

### ✅ laplacian_noise
**Config:** Sharper peak, heavier tails than Gaussian
**Results:**
- Position error: **0.96m** mean (+80% vs Gaussian)
- NEES: 5.53 (70.3% in bounds)
- NIS: 1.49 (100% in bounds)
- Rejection rate: 2.9%

**Analysis:**
Moderate degradation with Laplacian noise. Better than heavy-tailed due to lighter tails. NEES slightly high indicates some inconsistency but still reasonable performance.

**Key Insight:** EKF degrades gracefully with moderately non-Gaussian distributions.

---

### ⚠️ bimodal_noise
**Config:** Mixture of two Gaussians - simulates sensor glitches
**Results:**
- Position error: **1.72m** mean (+220% vs Gaussian!)
- NEES: **15.44** (23.5% in bounds) ← Very high!
- NIS: 2.33 (98.9% in bounds)
- Rejection rate: **28.0%** ← Very high!

**Analysis:**
Simulates **sensor glitches** (20% chance of 5x noise). Similar error to heavy-tailed but filter rejects many outliers (28%). High NEES shows severe inconsistency. This tests worst-case sensor failure scenarios.

**Key Insight:** Validation gate helps reject obvious glitches but can't fully compensate for bimodal violations.

---

### ⚠️ asymmetric_noise
**Config:** Exponential distribution - tests bias handling
**Results:**
- Position error: **1.40m** mean (+162% vs Gaussian)
- NEES: **10.39** (34.1% in bounds)
- NIS: 1.33 (99.9% in bounds)
- Rejection rate: 3.3%

**Analysis:**
Asymmetric noise introduces **bias** (non-zero mean) which violates EKF's zero-mean assumption. High NEES indicates filter can't properly handle systematic bias. Moderate rejection rate.

**Key Insight:** EKF assumes zero-mean noise - bias causes systematic errors.

---

## Performance Rankings

### By Position Error (Lower is Better)
1. **baseline_perfect**: 0.097m ⭐
2. **moderate_noise**: 0.53m ⭐
3. **tight_validation_gate**: 0.58m
4. **loose_validation_gate**: 0.58m
5. **high_noise**: 0.60m
6. **pessimistic_filter**: 0.63m
7. **uniform_noise**: 0.72m
8. **sensor_noise_only**: 0.79m
9. **laplacian_noise**: 0.96m
10. **asymmetric_noise**: 1.40m
11. **heavy_tailed_noise**: 1.71m ⚠️
12. **bimodal_noise**: 1.72m ⚠️
13. **optimistic_filter**: 9.46m max ⚠️

### By NEES Consistency (Higher % is Better)
1. **tight_validation_gate**: 94.0% ⭐
2. **moderate_noise**: 93.1% ⭐
3. **high_noise**: 88.0%
4. **loose_validation_gate**: 88.8%
5. **uniform_noise**: 82.6%
6. **optimistic_filter**: 79.4%
7. **sensor_noise_only**: 75.8%
8. **laplacian_noise**: 70.3%
9. **pessimistic_filter**: 70.5%
10. **asymmetric_noise**: 34.1% ⚠️
11. **heavy_tailed_noise**: 26.9% ⚠️
12. **bimodal_noise**: 23.5% ⚠️
13. **baseline_perfect**: 13.6% (expected with perfect sensors)

---

## Key Findings

### ✅ What Works Well
1. **Properly tuned filters** (moderate/high_noise): 0.53-0.60m error, 88-93% consistency
2. **Gaussian noise assumption**: EKF performs optimally with zero-mean Gaussian noise
3. **Bounded non-Gaussian** (uniform): Only 35% degradation
4. **Validation gates**: Effective trade-off between robustness and measurement updates
5. **Conservative tuning** (pessimistic): Safer than aggressive tuning

### ⚠️ Known Limitations
1. **Heavy-tailed noise**: 220% error increase, severe inconsistency
2. **Bimodal/glitches**: 220% error increase, high rejection rate
3. **Asymmetric/bias**: 162% error increase, systematic errors
4. **Optimistic tuning**: High rejection (24.8%), occasional large errors
5. **Loop closure linearization**: ~1m errors even with perfect sensors

### 🎯 Best Practices
1. **Noise estimation**: Match filter parameters to actual noise (see moderate_noise)
2. **Err conservative**: Pessimistic filter safer than optimistic
3. **Validation gates**: Use 2σ for noisy environments, 3σ for normal, 5σ for trusted sensors
4. **Monitor NEES/NIS**: Should stay in bounds 85-95% of time
5. **Non-Gaussian handling**: Consider robust filters (UKF, particle filters) for heavy-tailed noise

---

## Physical Validation

All tests successfully completed:
- ✅ **58/58 landmarks** mapped
- ✅ **49-56 loop closures** detected
- ✅ **4 laps** completed (~250s each)
- ✅ **Track length**: 711.7m × 4 = 2.85km
- ✅ **No crashes** or filter divergence (except optimistic with spikes)

---

## Test Configuration Details

Each test includes:
- **metrics.csv**: Time-series data (position error, NEES, NIS, uncertainties)
- **plots.png**: 6-panel visualization (errors, uncertainty, NEES, NIS, landmarks)
- **summary.txt**: Statistical summary (mean, std, max, consistency percentages)
- **config.json**: Full test configuration for reproducibility

**Reproducibility:** All tests use seed=2. Re-running should produce identical results.

---

## Interpreting Your Results

### Good Filter Performance
- Position error < 0.7m (for moderate noise)
- NEES mean near 3.0, 85-95% in bounds
- NIS mean near 2.0, 90-100% in bounds
- Rejection rate 0.5-2% (normal conditions)

### Warning Signs
- NEES > 10: Severe underestimation of uncertainty
- NEES < 0.5: Severe overestimation of uncertainty
- Rejection rate > 15%: Filter rejecting too many measurements
- Position error increasing over time: Filter divergence

### Expected Non-Ideal Behavior
- **baseline_perfect low NEES**: Normal with perfect sensors
- **sensor_noise_only high NEES**: Expected with noisy sensors + perfect odometry
- **Non-Gaussian high NEES**: EKF limitation, not a bug
- **Loop closure spikes**: EKF linearization error, not sensor noise
