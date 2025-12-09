"""
Generate simple geometric tracks for testing SLAM.
"""

import numpy as np
import matplotlib.pyplot as plt


def generate_oval_track(length=200, width=100, num_points=200):
    """
    Generate simple oval track (rectangle with rounded ends).

    Parameters:
    -----------
    length : float
        Length of straight sections (meters)
    width : float
        Width of track (meters)
    num_points : int
        Number of points to generate

    Returns:
    --------
    centerline : np.array
        Track centerline (num_points, 2)
    """
    # Oval is two straights connected by two semicircles
    straight_length = length
    radius = width / 2

    # Total track length
    semicircle_length = np.pi * radius
    total_length = 2 * straight_length + 2 * semicircle_length

    # Generate points along track
    centerline = []

    for i in range(num_points):
        # Parameter along track [0, 1]
        s = i / num_points * total_length

        if s < straight_length:
            # First straight (bottom)
            x = s
            y = 0
        elif s < straight_length + semicircle_length:
            # First semicircle (right)
            angle = (s - straight_length) / semicircle_length * np.pi
            x = straight_length + radius * np.sin(angle)
            y = radius * (1 - np.cos(angle))
        elif s < 2 * straight_length + semicircle_length:
            # Second straight (top)
            x = 2 * straight_length + semicircle_length - s
            y = 2 * radius
        else:
            # Second semicircle (left)
            angle = (s - 2 * straight_length - semicircle_length) / semicircle_length * np.pi
            x = -radius * np.sin(angle)
            y = radius * (1 + np.cos(angle))

        centerline.append([x, y])

    centerline = np.array(centerline)

    # Center the track
    center = centerline.mean(axis=0)
    centerline -= center

    return centerline


def generate_circle_track(radius=100, num_points=200):
    """
    Generate simple circular track.

    Parameters:
    -----------
    radius : float
        Radius of circle (meters)
    num_points : int
        Number of points

    Returns:
    --------
    centerline : np.array
        Track centerline (num_points, 2)
    """
    angles = np.linspace(0, 2 * np.pi, num_points, endpoint=False)
    x = radius * np.cos(angles)
    y = radius * np.sin(angles)

    centerline = np.column_stack([x, y])
    return centerline


def generate_figure8_track(radius=80, num_points=200):
    """
    Generate figure-8 track.

    Parameters:
    -----------
    radius : float
        Radius of each loop (meters)
    num_points : int
        Number of points

    Returns:
    --------
    centerline : np.array
        Track centerline (num_points, 2)
    """
    t = np.linspace(0, 2 * np.pi, num_points, endpoint=False)

    # Parametric figure-8 (lemniscate)
    scale = radius
    x = scale * np.sin(t)
    y = scale * np.sin(t) * np.cos(t)

    centerline = np.column_stack([x, y])
    return centerline


def save_simple_track(track_type='oval', filename='simple_track.npy'):
    """
    Generate and save a simple track.

    Parameters:
    -----------
    track_type : str
        'oval', 'circle', or 'figure8'
    filename : str
        Output filename
    """
    if track_type == 'oval':
        centerline = generate_oval_track(length=200, width=100, num_points=300)
        track_width_left = np.ones(len(centerline)) * 6.0  # 6m on each side
        track_width_right = np.ones(len(centerline)) * 6.0
    elif track_type == 'circle':
        centerline = generate_circle_track(radius=100, num_points=300)
        track_width_left = np.ones(len(centerline)) * 6.0
        track_width_right = np.ones(len(centerline)) * 6.0
    elif track_type == 'figure8':
        centerline = generate_figure8_track(radius=80, num_points=300)
        track_width_left = np.ones(len(centerline)) * 6.0
        track_width_right = np.ones(len(centerline)) * 6.0
    else:
        raise ValueError(f"Unknown track type: {track_type}")

    # Save files
    base_name = filename.replace('.npy', '')

    # Create track_npy directory if it doesn't exist
    import os
    from pathlib import Path

    # Get script directory and construct path to track_npy
    script_dir = Path(__file__).parent
    track_dir = script_dir / 'track_npy'
    track_dir.mkdir(exist_ok=True)

    np.save(track_dir / f'{base_name}_2d.npy', centerline)
    np.save(track_dir / f'{base_name}_width.npy',
            np.column_stack([track_width_left, track_width_right]))

    print(f"Saved {track_type} track:")
    print(f"  - {track_dir / f'{base_name}_2d.npy'}")
    print(f"  - {track_dir / f'{base_name}_width.npy'}")
    print(f"  Points: {len(centerline)}")
    print(f"  Track width: 12m (6m each side)")

    # Visualize
    plt.figure(figsize=(10, 8))

    # Plot centerline
    plt.plot(centerline[:, 0], centerline[:, 1], 'b-', linewidth=2, label='Centerline')

    # Plot boundaries
    for i in range(len(centerline)):
        if i < len(centerline) - 1:
            tangent = centerline[i + 1] - centerline[i]
        else:
            tangent = centerline[i] - centerline[i - 1]

        tangent_len = np.linalg.norm(tangent)
        if tangent_len > 0:
            tangent = tangent / tangent_len
            normal = np.array([-tangent[1], tangent[0]])

            left = centerline[i] + normal * track_width_left[i]
            right = centerline[i] - normal * track_width_right[i]

            if i % 20 == 0:  # Plot every 20th point
                plt.plot([left[0], right[0]], [left[1], right[1]],
                        'r-', alpha=0.3, linewidth=0.5)

    plt.axis('equal')
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.title(f'{track_type.capitalize()} Track - Simple & Easy to Follow')
    plt.xlabel('X (meters)')
    plt.ylabel('Y (meters)')
    plt.tight_layout()
    plt.savefig(track_dir / f'{base_name}_preview.png', dpi=150)
    plt.show()

    return centerline


if __name__ == "__main__":
    print("=" * 60)
    print("SIMPLE TRACK GENERATOR")
    print("=" * 60)

    # Generate oval track (easiest to follow)
    save_simple_track(track_type='oval', filename='simple_oval')

    print("\nTrack generated! Much easier to follow than Monza.")
    print("\nSimulation will automatically use this track with updated paths.")
