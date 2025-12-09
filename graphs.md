  ---
  1. Position Error (Top-Left, Blue Line)

  What it measures:
  The Euclidean distance between where the robot actually is (ground truth) vs where the EKF thinks it is (estimate).

  Position Error = √[(true_x - estimated_x)² + (true_y - estimated_y)²]

  Why it matters:
  This is the main performance metric for SLAM. Lower is better!

  What to expect:
  - Start (0-50s): High error (~1-2 meters) - robot is still learning
  - Middle (50-200s): Error decreases as robot sees landmarks repeatedly
  - End: Should converge to ~0.3-0.5 meters (excellent performance)

  Good values: < 0.5m
  **Acceptable:** 0.5-1.0m
  **Poor:** > 1.0m

  ---
  2. Heading Error (Top-Right, Red Line)

  What it measures:
  The angular difference between the robot's true heading (which way it's actually facing) vs its estimated heading
  (which way EKF thinks it's facing).

  Heading Error = |true_θ - estimated_θ| (in degrees)

  Why it matters:
  If the robot thinks it's facing North but is actually facing East, it will misplace all landmarks! Heading accuracy
  is crucial for correct mapping.

  What to expect:
  - Lots of noise/spikes - heading changes constantly as robot turns
  - Should stay mostly between 0-5 degrees
  - Occasional spikes to 10-15 degrees during sharp turns

  Good values: < 2°
  **Acceptable:** 2-5°
  **Poor:** > 10° sustained

  ---
  3. Landmarks Mapped (Bottom-Left, Green Line)

  What it measures:
  The total number of unique landmarks that the robot has discovered and added to its map.

  Why it matters:
  Shows how much of the environment the robot has explored. More landmarks = more reference points = better
  localization.

  What to expect:
  - Rapid increase (0-50s): From 0 to 58 landmarks as robot does first lap
  - Plateau: Stays at 58 once all landmarks are discovered
  - Should be a smooth increasing curve, then flat

  For your project:
  - Total landmarks available: 58
  - Expected final count: 58 (100% mapping success)

  ---
  4. Position Uncertainty (Bottom-Right, Magenta Line)

  What it measures:
  How confident the robot is about its position estimate. This is NOT the actual error - it's what the robot thinks
  its error might be.

  Calculated from the covariance matrix:
  Uncertainty = √(trace(P[0:2, 0:2]))
  where P is the robot's position covariance.

  Why it matters:
  This tells you if the robot is realistic about its knowledge:
  - Low uncertainty + Low actual error = ✅ Robot is accurate AND confident
  - Low uncertainty + High actual error = ❌ Robot is overconfident (dangerous!)
  - High uncertainty + Low actual error = ⚠️ Robot is underconfident (inefficient)

  What to expect:
  - Start (~1.0m): Robot is unsure about its position
  - Decreases over time: As robot sees landmarks, confidence increases
  - Converges to ~0.4-0.5m: Steady-state uncertainty
  - Should match actual position error! (This is "filter consistency")

  Good filter: Uncertainty ≈ Actual Position Error
  Overconfident filter: Uncertainty < Actual Error (bad!)
  **Underconfident filter:** Uncertainty > Actual Error (inefficient)

  ---
  🎯 What Good Performance Looks Like:

  First Lap (0-70 seconds):

  - Position Error: High and noisy (1-2m) - robot is exploring
  - Heading Error: Variable (0-10°)
  - Landmarks Mapped: Rapidly increasing (0 → 58)
  - Uncertainty: Decreasing (1.0m → 0.5m)

  Subsequent Laps (70-280 seconds):

  - Position Error: Converging down (→ 0.3-0.5m)
  - Heading Error: Stable and low (< 5°)
  - Landmarks Mapped: Flat at 58
  - Uncertainty: Stable (~0.4-0.5m)

  Key Insight - Filter Consistency:

  If you see:
  Position Error ≈ Position Uncertainty
  Your filter is well-calibrated! This is what you want to report.

  Your results with noise:
  - Position Error: ~0.39m (mean)
  - Uncertainty: ~0.4-0.5m
  - 95.9% of the time, error is within 2σ bounds ← This is excellent!

  ---
  📈 How to Interpret Your Plots:

  Good signs:
  - ✅ Position error decreases over time
  - ✅ Heading error stays mostly under 5°
  - ✅ All landmarks discovered (green line reaches 58)
  - ✅ Uncertainty decreases then stabilizes
  - ✅ Blue and magenta lines track each other (error ≈ uncertainty)

  Warning signs:
  - ❌ Position error increases over time (filter diverging)
  - ❌ Landmarks not all discovered (green line < 58)
  - ❌ Uncertainty keeps growing (not learning from measurements)
  - ❌ Uncertainty << Error (overconfident filter)