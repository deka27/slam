"""
Generate landmarks for SLAM on simple tracks.
"""

import numpy as np


def generate_landmarks_oval(track_centerline, track_width, num_landmarks=40, extra_curve_landmarks=True):
    """
    Generate landmarks around an oval track.

    Places landmarks outside the track boundaries, with extra density in curved sections.

    Parameters:
    -----------
    track_centerline : np.array
        Track centerline (N, 2)
    track_width : float
        Track width (meters)
    num_landmarks : int
        Base number of landmarks to generate
    extra_curve_landmarks : bool
        If True, add extra landmarks in curved sections

    Returns:
    --------
    landmarks : list of dict
        Each landmark has 'id' and 'position' [x, y]
    """
    landmarks = []

    # Calculate curvature at each point to identify curves
    curvatures = []
    for i in range(len(track_centerline)):
        if i == 0 or i == len(track_centerline) - 1:
            curvatures.append(0)
            continue

        # Approximate curvature using three points
        p1 = track_centerline[i - 1]
        p2 = track_centerline[i]
        p3 = track_centerline[i + 1]

        # Vectors
        v1 = p2 - p1
        v2 = p3 - p2

        # Angle change approximation
        angle_change = np.arctan2(v2[1], v2[0]) - np.arctan2(v1[1], v1[0])
        angle_change = np.arctan2(np.sin(angle_change), np.cos(angle_change))  # Wrap to [-pi, pi]

        curvatures.append(abs(angle_change))

    curvatures = np.array(curvatures)

    # Place base landmarks evenly spaced
    base_indices = np.linspace(0, len(track_centerline) - 1, num_landmarks, dtype=int)

    # Add extra landmarks in high curvature areas
    if extra_curve_landmarks:
        # Find high curvature regions (top 30%)
        curve_threshold = np.percentile(curvatures, 70)
        curve_indices = np.where(curvatures > curve_threshold)[0]

        # Sample extra landmarks from curved regions
        if len(curve_indices) > 0:
            extra_count = num_landmarks // 2  # Add 50% more landmarks in curves
            extra_indices = np.random.choice(curve_indices, size=min(extra_count, len(curve_indices)), replace=False)
            all_indices = np.concatenate([base_indices, extra_indices])
            all_indices = np.unique(all_indices)  # Remove duplicates
        else:
            all_indices = base_indices
    else:
        all_indices = base_indices

    landmark_id = 0
    for idx in sorted(all_indices):
        # Get point on centerline
        center_point = track_centerline[idx]

        # Calculate normal direction
        if idx < len(track_centerline) - 1:
            tangent = track_centerline[idx + 1] - track_centerline[idx]
        else:
            tangent = track_centerline[idx] - track_centerline[idx - 1]

        tangent_len = np.linalg.norm(tangent)
        if tangent_len > 0:
            tangent = tangent / tangent_len
            normal = np.array([-tangent[1], tangent[0]])
        else:
            normal = np.array([0, 1])

        # Alternate between left and right side
        side = 1 if landmark_id % 2 == 0 else -1

        # Place landmark outside track boundary
        offset = track_width * 0.6 + 3.0  # Outside track + 3m margin
        landmark_pos = center_point + normal * side * offset

        landmarks.append({
            'id': landmark_id,
            'position': landmark_pos.tolist()
        })

        landmark_id += 1

    return landmarks


def save_landmarks(landmarks, filename='simple_oval_landmarks.npy'):
    """
    Save landmarks to file.

    Parameters:
    -----------
    landmarks : list of dict
        Landmarks to save
    filename : str
        Output filename
    """
    from pathlib import Path

    # Get script directory
    script_dir = Path(__file__).parent
    output_path = script_dir / 'track_npy' / filename

    # Convert to numpy format: (N, 3) with [id, x, y]
    landmark_array = np.array([
        [lm['id'], lm['position'][0], lm['position'][1]]
        for lm in landmarks
    ])

    np.save(output_path, landmark_array)
    print(f"Saved {len(landmarks)} landmarks to {output_path}")

    return landmark_array


def load_landmarks(filename='simple_oval_landmarks.npy'):
    """
    Load landmarks from file.

    Parameters:
    -----------
    filename : str
        Input filename

    Returns:
    --------
    landmarks : list of dict
        Loaded landmarks
    """
    from pathlib import Path

    script_dir = Path(__file__).parent
    input_path = script_dir / 'track_npy' / filename

    landmark_array = np.load(input_path)

    landmarks = [
        {'id': int(row[0]), 'position': [row[1], row[2]]}
        for row in landmark_array
    ]

    return landmarks


if __name__ == "__main__":
    from pathlib import Path

    print("=" * 60)
    print("LANDMARK GENERATOR")
    print("=" * 60)

    # Load simple oval track
    track_dir = Path(__file__).parent / 'track_npy'
    centerline = np.load(track_dir / 'simple_oval_2d.npy')
    width_data = np.load(track_dir / 'simple_oval_width.npy')

    # Track width is 6m on each side = 12m total
    track_width = 12.0

    # Generate landmarks with higher density
    # Base 40 landmarks + extra 20 in curves = ~60 total
    landmarks = generate_landmarks_oval(
        centerline,
        track_width,
        num_landmarks=40,
        extra_curve_landmarks=True
    )

    # Save landmarks
    save_landmarks(landmarks, 'simple_oval_landmarks.npy')

    print(f"\nGenerated {len(landmarks)} landmarks around the track")
    print(f"  - Base landmarks: 40")
    print(f"  - Extra curve landmarks: {len(landmarks) - 40}")
    print("Landmarks placed outside track boundaries for visibility")
    print("Higher density in curved sections for better SLAM observability")
