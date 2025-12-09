import numpy as np
import matplotlib.pyplot as plt
import requests
from io import StringIO

# ------------------------------------------------------------
# 1) Download accurate Monza centerline CSV
# ------------------------------------------------------------
url = "https://raw.githubusercontent.com/TUMFTM/racetrack-database/master/tracks/Monza.csv"

resp = requests.get(url)
resp.raise_for_status()
csv_text = resp.text

# CSV is comma-separated: x_m, y_m, w_tr_right_m, w_tr_left_m
data = np.loadtxt(StringIO(csv_text), delimiter=",", skiprows=1)

x_m = data[:, 0]
y_m = data[:, 1]

# ------------------------------------------------------------
# 2) Normalise / rotate for nice plotting
# ------------------------------------------------------------

# Center roughly
x = x_m - x_m.mean()
y = y_m - y_m.min()

# Optional: rotate a bit so it looks like your reference image
angle_deg = -7.0
theta = np.deg2rad(angle_deg)

xr = x * np.cos(theta) - y * np.sin(theta)
yr = x * np.sin(theta) + y * np.cos(theta)

# Scale to [0, 1] box
xr -= xr.min()
yr -= yr.min()
scale = max(np.ptp(xr), np.ptp(yr))
xr /= scale
yr /= scale

# 2D track coordinates
monza_track = np.column_stack([xr, yr])  # shape (N, 2)

# Optional 3D version with z = 0 for each point
monza_track_3d = np.column_stack([xr, yr, np.zeros_like(xr)])  # shape (N, 3)

# ------------------------------------------------------------
# 3) Plot with Matplotlib
# ------------------------------------------------------------
fig, ax = plt.subplots(figsize=(10, 5))

# Orange outline
ax.plot(
    monza_track[:, 0],
    monza_track[:, 1],
    linewidth=10,
    color="#f89f1b",
    solid_capstyle="round",
)

# Dark asphalt
ax.plot(
    monza_track[:, 0],
    monza_track[:, 1],
    linewidth=6,
    color="#222222",
    solid_capstyle="round",
)

ax.set_aspect("equal", "box")
ax.axis("off")
ax.set_title("Autodromo Nazionale Monza – accurate track", pad=12)

plt.tight_layout()
plt.show()

# ------------------------------------------------------------
# 4) Save to .npy for later 3D use
# ------------------------------------------------------------
np.save("monza_track_2d.npy", monza_track)
np.save("monza_track_3d.npy", monza_track_3d)

print("Saved monza_track_2d.npy (N x 2) and monza_track_3d.npy (N x 3)")
