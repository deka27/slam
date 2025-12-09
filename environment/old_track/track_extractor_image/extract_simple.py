import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
from skimage import measure


def extract_track_simple(path):
    """
    Simple extraction: just trace the white outline directly.
    No skeletonization, just contour detection.
    """
    # Load image
    img = Image.open(path).convert("L")
    data = np.array(img)

    # Threshold to binary
    binary = data > 127

    # Find contours
    contours = measure.find_contours(binary, 0.5)

    # Take the longest contour (should be the track outline)
    track_contour = max(contours, key=len)

    print(f"Found contour with {len(track_contour)} points")

    # Convert from image coords to normalized [0, 1] x [0, 1]
    # Swap because contour gives (row, col) = (y, x)
    coords = np.column_stack([track_contour[:, 1], track_contour[:, 0]])

    # Normalize while preserving aspect ratio
    xs = coords[:, 0]
    ys = coords[:, 1]

    # Center coordinates
    xs = xs - xs.min()
    ys = ys - ys.min()

    # Scale by the SAME factor (use the larger dimension)
    max_dim = max(xs.max(), ys.max())
    xs = xs / max_dim
    ys = 1.0 - (ys / max_dim)  # Flip y axis

    coords_norm = np.column_stack([xs, ys])

    # Downsample to reasonable number of points
    step = max(1, len(coords_norm) // 500)  # ~500 points
    coords_norm = coords_norm[::step]

    # Close the loop
    coords_norm = np.vstack([coords_norm, coords_norm[0]])

    return coords_norm


# Extract
print("Extracting track...")
monza = extract_track_simple("environment/Large.png")
print(f"Final track: {len(monza)} points")

# Save coordinates
np.save("environment/monza_coords.npy", monza)
print("Saved coordinates to environment/monza_coords.npy")

# Plot
fig, ax = plt.subplots(figsize=(10, 6))

# Orange outline
ax.plot(
    monza[:, 0],
    monza[:, 1],
    linewidth=18,
    color="#f89f1b",
    solid_capstyle="round",
)

# Dark asphalt
ax.plot(
    monza[:, 0],
    monza[:, 1],
    linewidth=12,
    color="#333333",
    solid_capstyle="round",
)

ax.set_aspect("equal", "box")
ax.axis("off")
plt.tight_layout()
plt.savefig("environment/monza_centerline.png", dpi=150, bbox_inches="tight")
print("Saved to environment/monza_centerline.png")
plt.show()
