# SLAM Simulation Visual Guide

This document explains all the visual elements (lines, colors, shapes) in the SLAM simulation.

## Overview

The simulation displays two parallel views of the world:
1. **Ground Truth** (what's actually happening) - shown in <span style="color: #3399FF;">**blue**</span> and <span style="color: #FF8000;">**orange**</span>
2. **SLAM Estimates** (what the robot thinks is happening) - shown in <span style="color: #00FF00;">**green**</span>

The goal is to see these converge over time as SLAM improves its estimates.

---

## Ground Truth Elements (Reality)

### Track Surface - <span style="color: #666666;">**Gray**</span> `[0.4, 0.4, 0.4]`
- The racing track itself
- Gray surface rendered at ground level (height = 0)
- Code: `robot_simulation.py:881`

### Centerline - <span style="color: #00FF00;">**Green**</span> `(0.0, 1.0, 0.0)`
- Center of the track
- <span style="color: #00FF00;">Green</span> line at height 0.3m above ground
- Shows the geometric center of the track
- Code: `robot_simulation.py:896`

### True Landmarks - <span style="color: #FF8000;">**Orange**</span> Boxes `[1.0, 0.5, 0.0]`
- Actual landmark positions (58 total around track)
- Size: 2m x 4m x 2m (width x height x depth)
- Height: 2.0m above ground
- These are the "ground truth" landmark locations
- Code: `robot_simulation.py:909`

### True Robot - <span style="color: #3399FF;">**Blue**</span> Box `[0.2, 0.6, 1.0]`
- Actual robot position
- Size: 1.5m x 1.5m x 1.5m cube
- Shows where the robot really is
- Code: `robot_simulation.py:291, 379`

### Direction Arrow - <span style="color: #FF4D00;">**Bright Orange**</span> `[1.0, 0.3, 0.0]`
- Shows which way robot is actually facing
- Length: 5.0 meters
- Height: 2.5m above ground (above robot)
- Points in direction of robot's heading angle (x)
- Code: `robot_simulation.py:311, 395`

### Robot Trajectory - <span style="color: #3399FF;">**Blue**</span> Line `(0.2, 0.6, 1.0)`
- Path the robot has actually traveled
- Height: 0.2m above ground
- Updated every 10 simulation steps for performance
- Shows ground truth path over time
- Code: `robot_simulation.py:343`

---

## SLAM Estimate Elements (What Robot Thinks)

### SLAM Robot - <span style="color: #00FF00;">**Green**</span> Box (Semi-transparent) `[0.0, 1.0, 0.0]`
- Where SLAM thinks the robot is
- Size: 1.5m x 1.5m x 1.5m cube (same as true robot)
- Opacity: 0.5 (50% transparent to see through to <span style="color: #3399FF;">blue</span> robot)
- When it overlaps <span style="color: #3399FF;">blue</span> robot = good estimate!
- Code: `robot_simulation.py:440`

### SLAM Direction - <span style="color: #00FF00;">**Green**</span> Arrow `[0.0, 1.0, 0.0]`
- Which way SLAM thinks robot is facing
- Length: 5.0 meters
- Height: 3.0m above ground (slightly higher than <span style="color: #FF4D00;">orange</span> arrow)
- Opacity: 0.7 (70% transparent)
- Code: `robot_simulation.py:461`

### SLAM Trajectory - <span style="color: #00FF00;">**Green**</span> Line (Semi-transparent) `(0.0, 1.0, 0.0)`
- Path SLAM thinks the robot has traveled
- Height: 0.5m above ground (higher than <span style="color: #3399FF;">blue</span> trajectory)
- Opacity: 0.6 (60% transparent)
- Updated every 10 steps for performance
- When it matches <span style="color: #3399FF;">blue</span> line = SLAM is tracking well!
- Code: `robot_simulation.py:482`

### SLAM Landmarks - <span style="color: #00FF00;">**Green**</span> Boxes (Semi-transparent) `[0.0, 1.0, 0.0]`
- Where SLAM thinks landmarks are located
- Size: 1.5m x 3.0m x 1.5m (slightly smaller than true landmarks)
- Height: 1.5m above ground (lower than true landmarks at 2.0m)
- Opacity: 0.5 (50% transparent)
- New landmarks appear as robot discovers them
- When they overlap <span style="color: #FF8000;">orange</span> landmarks = good map!
- Code: `robot_simulation.py:498`

### Uncertainty Sphere - <span style="color: #00FF00;">**Green**</span> Bubble (Very Transparent) `[0.0, 1.0, 0.0]`
- Shows how uncertain SLAM is about robot position
- Radius: 2x (95% confidence interval)
- Opacity: 0.15 (15% transparent - very faint)
- Height: 0.2m above ground
- Larger sphere = more uncertain
- Shrinks over time as SLAM gets more measurements
- Code: `robot_simulation.py:566`

---

## Sensor Data Elements

### Detection Lines - <span style="color: #FFFF00;">**Yellow**</span> Lines (Semi-transparent) `(1.0, 1.0, 0.0)`
- Lines from robot to landmarks it can currently see
- Height: 1.0m above ground
- Opacity: 0.5 (50% transparent)
- Shows active sensor measurements in real-time
- More <span style="color: #FFFF00;">yellow</span> lines = robot can see more landmarks = better SLAM updates
- No <span style="color: #FFFF00;">yellow</span> lines = robot temporarily "blind" (no landmarks in range/FOV)
- Code: `robot_simulation.py:535`

---

## Visual Interpretation Guide

### When SLAM is Working Well:
- <span style="color: #3399FF;">**Blue**</span> robot and <span style="color: #00FF00;">**green**</span> robot overlap (accurate position estimate)
- <span style="color: #FF8000;">Orange</span> landmarks and <span style="color: #00FF00;">green</span> landmarks overlap (accurate map)
- <span style="color: #3399FF;">Blue</span> trajectory and green trajectory follow same path (good tracking)
- <span style="color: #00FF00;">Green</span> uncertainty sphere shrinks over time (increasing confidence)
- <span style="color: #FFFF00;">Yellow</span> lines connecting to multiple landmarks (good sensor coverage)

### When SLAM Has Errors:
X Gap between <span style="color: #3399FF;">blue</span> and <span style="color: #00FF00;">green</span> robots (position error)
X <span style="color: #FF8000;">Orange</span> and <span style="color: #00FF00;">green</span> landmarks don't align (mapping error)
X Trajectories diverge (accumulated drift)
X Large <span style="color: #00FF00;">green</span> bubble (high uncertainty)
X Few or no <span style="color: #FFFF00;">yellow</span> lines (poor sensor coverage)

### During Loop Closure:
-  Sudden "snap" - green elements realign with blue/orange
-  Trajectories correct accumulated drift
-  Console prints: "Loop closure detected!"
-  Errors reduce dramatically
-  Loop closure counter increments

---

## Height Layout (Vertical Spacing)

To prevent visual clutter, elements are placed at different heights:

| Element | Height Above Ground | Purpose |
|---------|-------------------|---------|
| Track Surface | 0.0m | Base level |
| True Robot Trajectory (blue) | 0.2m | Just above ground |
| Uncertainty Sphere | 0.2m | Around SLAM robot |
| Centerline/Racing Line | 0.3m | Track reference |
| SLAM Robot Trajectory (green) | 0.5m | Above blue trajectory |
| Detection Lines | 1.0m | Mid-height visibility |
| SLAM Landmarks (green) | 1.5m | Slightly lower |
| True Landmarks (orange) | 2.0m | Higher for visibility |
| Direction Arrows | 2.5-3.0m | Above everything else |

---

## Transparency Levels

Different transparency levels help see overlapping elements:

| Element | Opacity | Why |
|---------|---------|-----|
| Uncertainty Sphere | 15% | Very subtle, doesn't obscure |
| SLAM Robot | 50% | See both true and estimated positions |
| SLAM Landmarks | 50% | Compare with true landmarks |
| Detection Lines | 50% | Show connections without clutter |
| SLAM Trajectory | 60% | See both paths clearly |
| SLAM Direction Arrow | 70% | More visible than robot |

---

## Size Differences

| Element | Size (W x H x D) | Notes |
|---------|------------------|-------|
| True Landmarks | 2.0m x 4.0m x 2.0m | Taller boxes |
| SLAM Landmarks | 1.5m x 3.0m x 1.5m | Slightly smaller |
| Robots (both) | 1.5m x 1.5m x 1.5m | Same size |
| Direction Arrows | 5.0m length | Long and visible |

---

## What to Watch During Simulation

### Start (0-10 seconds):
- Green and blue robots may be far apart (initial uncertainty)
- Big green uncertainty bubble (very uncertain)
- Few <span style="color: #00FF00;">green</span> landmarks (haven't mapped many yet)
- <span style="color: #FFFF00;">Yellow</span> lines connecting to nearby orange landmarks

### Middle Phase (10-60 seconds):
- Green robot moves closer to blue robot
- Uncertainty bubble shrinks
- More <span style="color: #00FF00;">green</span> landmarks appear (building the map)
- Green and orange landmarks align better

### After First Loop Closure (~50-70 seconds):
- Sudden "snap" - trajectories realign
- Errors reduce dramatically
- Console: "Loop closure detected! Total closures: 1"
- Uncertainty may briefly increase, then decrease

### Steady State (after multiple laps):
- Green and blue nearly perfectly aligned
- All 58 landmarks mapped (58 green boxes)
- Small uncertainty sphere
- Smooth trajectory tracking

---

## Color Scheme Summary

**Color Code:**
- <span style="color: #3399FF;">**Blue**</span> = Ground Truth (Robot)
- <span style="color: #FF8000;">**Orange**</span> = Ground Truth (Landmarks & Direction)
- <span style="color: #00FF00;">**Green**</span> = SLAM Estimates (Everything)
- <span style="color: #FFFF00;">**Yellow**</span> = Active Sensor Measurements
- <span style="color: #666666;">**Gray**</span> = Track Surface
- <span style="color: #CCE6FF;">**Light Blue**</span> = Background Sky

**Mental Model:**
"Is the green stuff aligned with the blue/orange stuff?"
- YES = SLAM is working well!
- NO = SLAM has errors

---

## Technical Details

### Update Frequencies:
- Robot motion: 60 Hz (every frame)
- SLAM prediction: 60 Hz (every frame)
- SLAM measurement update: 30 Hz (every 2 frames)
- Trajectory visualization: Every 10 frames
- Metrics plots: Every 30 frames (~0.5 seconds)

### Sensor Parameters:
- Max range: 80 meters
- Field of view: 180x (x radians)
- Range noise: x = 0.1 meters
- Bearing noise: x = 0.05 radians (~2.86x)

### Motion Parameters:
- Desired speed: 15 m/s (~54 km/h)
- Lookahead distance: 12 meters
- Time step: 0.017 seconds (~60 Hz)

---

## Troubleshooting Visual Issues

### Can't see SLAM robot (green):
- It may be perfectly aligned with blue robot (good!)
- Try rotating view to see transparency

### Can't see uncertainty sphere:
- It may be very small (SLAM is confident)
- Opacity is only 15% - look carefully
- Early in simulation it should be visible

### Trajectories look choppy:
- Normal - updated every 10 frames for performance
- Not a bug, just optimization

### <span style="color: #FFFF00;">Yellow</span> lines flickering:
- Normal - shows real-time sensor measurements
- Landmarks enter/exit field of view constantly

### Green landmarks appearing gradually:
- Expected behavior!
- SLAM initializes landmarks on first detection
- Should reach 58 total after ~1 lap

---

## References

- Main simulation code: `simulation/robot_simulation.py`
- Robot model: `robot/robot.py`
- Sensor model: `robot/sensor.py`
- EKF-SLAM algorithm: `slam/ekf_slam.py`
- Visualization library: FURY (https://fury.gl)
