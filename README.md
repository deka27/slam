# SLAM Project - Running Instructions

---

## Installation

### Step 1: Install Dependencies

Navigate to the SLAM directory and install all required packages:

```bash
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

After running the simulation, check the `logs/` directory.