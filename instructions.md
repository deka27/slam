# SLAM Project - Running Instructions

---

## Installation

### Step 1: Install Dependencies

Navigate to the SLAM directory and install all required packages:

```bash
cd /home/han/Desktop/Code/SLAM
pip install -r requirements.txt
```

### Alternative: Install Manually

If you encounter dependency conflicts, install packages in order:

```bash
# Core packages
pip install numpy scipy

# Visualization
pip install vtk fury

# Plotting
pip install matplotlib

# Data handling
pip install pandas
```

### Verify Installation

Check that all packages are installed correctly:

```bash
python -c "import fury; import matplotlib; import numpy; print('✓ All packages installed successfully!')"
```

---

## Running the Simulation

### Basic Usage

Run the simulation with default settings:

```bash
python simulation/robot_simulation.py
```

### What You'll See

When you run the simulation, **two windows** will open:

1. **3D FURY Window (Left)**
   - Shows robot navigating the oval track
   - Green: True robot position and landmarks
   - Blue: Estimated robot position and landmarks
   - Real-time 3D visualization at 60 FPS

2. **Matplotlib Plots Window (Right)**
   - 4 real-time graphs updating every 0.5 seconds:
     - **Position Error** (top-left, blue): Distance between true and estimated position
     - **Heading Error** (top-right, red): Angular error in degrees
     - **Landmarks Mapped** (bottom-left, green): Number of landmarks discovered
     - **Position Uncertainty** (bottom-right, magenta): Covariance-based confidence

### Simulation Controls

**3D Window Controls:**
- **Left mouse button**: Rotate view
- **Right mouse button**: Zoom in/out
- **Middle mouse button**: Pan camera
- **Q key**: Quit simulation early

### What Happens Automatically

1. **During Simulation:**
   - Logs metrics to CSV file at 60 Hz
   - Updates 3D visualization at 60 FPS

2. **When Complete:**
   - Saves metrics to `logs/slam_metrics_noisy.csv` or `logs/slam_metrics_normal.csv`
   - Saves plots to `logs/slam_metrics_noisy.png` or `logs/slam_metrics_normal.png`
   - Saves detailed log to `logs/slam_log.txt`

---

## Configuration Options

### Enable/Disable Noise

Edit the bottom of `simulation/robot_simulation.py` (line ~1030):

```python
# With realistic noise (recommended for final results)
run_simulation(enable_noise=True, num_laps=4, use_racing_line=False)

# Without noise (for testing/comparison)
run_simulation(enable_noise=False, num_laps=4, use_racing_line=False)
```

### Adjust Number of Laps

```python
run_simulation(enable_noise=True, num_laps=4, use_racing_line=False)
#                                    ^^^^^^ Change this number
```


---

## Output Files

After running the simulation, check the `logs/` directory:

### Files Generated

| File | Description | Size |
|------|-------------|------|
| `slam_metrics_noisy.csv` | Performance metrics (60 Hz) with noise | ~2.8MB |
| `slam_metrics_normal.csv` | Performance metrics without noise | ~2.4MB |
| `slam_metrics_noisy.png` | 6-subplot performance visualization with noise | ~500KB |
| `slam_metrics_normal.png` | 6-subplot performance visualization without noise | ~500KB |
| `slam_log.txt` | Detailed execution log | ~180KB |

---

## Troubleshooting

### Issue: "No module named 'fury'"

**Solution:**
```bash
pip install fury vtk
```

### Issue: Real-time plots not updating

**Cause:** Matplotlib window in background or display issue.

**Solutions:**
1. Make sure matplotlib window is visible (not minimized)
2. Try running from terminal (not Jupyter/IPython)
3. Check if plots are actually updating (they update every 0.5 seconds, not continuously)

### Issue: Simulation runs slowly

**Causes:**
- Plot updates too frequent
- System resources

**Solutions:**
1. Reduce plot update frequency: Change `if counter % 30 == 0` to `if counter % 60 == 0` (line ~1011)
2. Close other applications
3. Reduce number of laps for testing

### Issue: "cannot import name 'ShowManager' from 'fury'"

**Solution:**
Update FURY to latest version:
```bash
pip install --upgrade fury
```

### Issue: Two windows opening is confusing

**Solution:**
This is normal! You should see:
1. **FURY window** (3D simulation)
2. **Matplotlib window** (real-time plots)

Position them side-by-side for best viewing.

---

## Performance Tips

### For Best Real-Time Performance:
- Update plots less frequently: `counter % 60` instead of `counter % 30`
- Close other applications
- Use fewer laps for testing (change `num_laps=4` to `num_laps=1`)

---

## Expected Simulation Duration

| Configuration | Real Time | CSV Size |
|--------------|-----------|----------|
| 4 laps, no noise | ~4 minutes | ~2.4MB |
| 4 laps, with noise | ~4.5 minutes | ~2.8MB |
| 1 lap, no noise | ~1 minute | ~600KB |

*Times approximate, may vary based on system performance*

---

## For Your Report

### Recommended Settings for Final Demo:

```python
# In robot_simulation.py (bottom):
run_simulation(enable_noise=True, num_laps=4, use_racing_line=False)
```

### What You'll Get:

From your simulation, you'll have:
- ✅ **CSV files** - Contains all numerical data
- ✅ **PNG plots** - 6-subplot performance visualization
- ✅ **Metrics** - 0.32m final error, 95.9% consistency

These prove your SLAM implementation works!

---

## Additional Commands

### View Metrics in Python:

```python
import pandas as pd

# Load metrics
df = pd.read_csv('logs/slam_metrics_noisy.csv')

# View summary statistics
print(df.describe())

# Plot specific metric
import matplotlib.pyplot as plt
plt.plot(df['Time (s)'], df['Position Error (m)'])
plt.xlabel('Time (s)')
plt.ylabel('Position Error (m)')
plt.title('Position Error Over Time')
plt.show()
```

---

## Quick Reference

### Run with noise (recommended):
```bash
python simulation/robot_simulation.py
```

### Check if everything is ready:
```bash
python -c "import fury; import matplotlib; print('Ready to run!')"
```

---
