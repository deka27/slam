"""
Improved Monza race track in 3D using FURY with variable track width.
Uses real TUMFTM database with accurate GPS/surveyed data.
"""

import numpy as np
from fury import window, actor


class MonzaTrack3DImproved:
    def __init__(self,
                 coords_file='environment/track_npy/monza_track_2d.npy',
                 width_file='environment/track_npy/monza_track_width.npy',
                 use_variable_width=True):
        """
        Initialize Monza track with optional variable width.

        Parameters:
        -----------
        coords_file : str
            Path to centerline coordinates file
        width_file : str
            Path to track width data file
        use_variable_width : bool
            Whether to use variable track width from data
        """
        self.scene = window.Scene()
        self.scene.background((0.7, 0.85, 1.0))  # Sky blue background

        # Default track parameters
        self.default_track_width = 12.0  # meters
        self.wall_height = 2.0   # meters
        self.use_variable_width = use_variable_width

        # Load extracted centerline
        self.track_centerline = self._load_and_scale_centerline(coords_file)

        # Load track width if available
        self.track_width_data = self._load_track_width(width_file)

        # Build track
        self._build_track()
        self._add_ground()

    def _load_and_scale_centerline(self, coords_file):
        """Load and scale centerline coordinates"""
        coords = np.load(coords_file)

        # Scale to real-world size (Monza ~5.8km)
        diff = np.diff(coords, axis=0)
        lengths = np.sqrt((diff**2).sum(axis=1))
        current_length = lengths.sum()
        scale_factor = 5793 / current_length

        coords_scaled = coords * scale_factor

        return coords_scaled

    def _load_track_width(self, width_file):
        """Load track width data (left, right in meters)"""
        try:
            width_data = np.load(width_file)
            print(f"✓ Loaded variable track width data: {len(width_data)} points")
            return width_data
        except FileNotFoundError:
            print(f"⚠ Width file not found, using constant width: {self.default_track_width}m")
            return None

    def _get_track_width(self, index):
        """Get track width at given index"""
        if self.use_variable_width and self.track_width_data is not None:
            # width_data is [width_left, width_right]
            width_left = self.track_width_data[index, 0]
            width_right = self.track_width_data[index, 1]
            total_width = width_left + width_right
            return total_width, width_left, width_right
        else:
            # Use constant width
            half_width = self.default_track_width / 2
            return self.default_track_width, half_width, half_width

    def _build_track(self):
        """Build track boundaries using smooth tubes with variable width"""
        # Build inner and outer boundary paths
        inner_path = []
        outer_path = []

        for i in range(len(self.track_centerline)):
            p = self.track_centerline[i]

            # Calculate perpendicular
            if i < len(self.track_centerline) - 1:
                next_p = self.track_centerline[i + 1]
                tangent = next_p - p
            else:
                prev_p = self.track_centerline[i - 1]
                tangent = p - prev_p

            tangent_len = np.linalg.norm(tangent)
            if tangent_len > 0:
                tangent = tangent / tangent_len
            else:
                tangent = np.array([1, 0])

            perpendicular = np.array([-tangent[1], tangent[0]])

            # Get width at this point
            _, width_left, width_right = self._get_track_width(i)

            # Calculate inner and outer points at wall height
            inner = np.array([
                p[0] - perpendicular[0] * width_right,
                self.wall_height / 2,
                p[1] - perpendicular[1] * width_right
            ])
            outer = np.array([
                p[0] + perpendicular[0] * width_left,
                self.wall_height / 2,
                p[1] + perpendicular[1] * width_left
            ])

            inner_path.append(inner)
            outer_path.append(outer)

        inner_path = np.array(inner_path)
        outer_path = np.array(outer_path)

        # Create smooth tube walls using streamtube
        inner_wall = actor.streamtube(
            [inner_path],
            colors=(0.9, 0.1, 0.1),
            linewidth=0.8
        )
        outer_wall = actor.streamtube(
            [outer_path],
            colors=(0.9, 0.1, 0.1),
            linewidth=0.8
        )

        self.scene.add(inner_wall)
        self.scene.add(outer_wall)

        # Create track surface
        self._create_smooth_track_surface()

    def _create_smooth_track_surface(self):
        """Create flat drivable track surface with variable width"""
        vertices = []
        faces = []

        n_points = len(self.track_centerline)

        # Create vertices for left and right edges
        for i in range(n_points):
            p = self.track_centerline[i]

            # Calculate perpendicular
            if i < n_points - 1:
                next_p = self.track_centerline[i + 1]
                tangent = next_p - p
            else:
                prev_p = self.track_centerline[i - 1]
                tangent = p - prev_p

            tangent_len = np.linalg.norm(tangent)
            if tangent_len > 0:
                tangent = tangent / tangent_len
            else:
                tangent = np.array([1, 0])

            perpendicular = np.array([-tangent[1], tangent[0]])

            # Get width at this point
            _, width_left, width_right = self._get_track_width(i)

            # Left and right edge vertices at ground level
            left = np.array([
                p[0] + perpendicular[0] * width_left,
                0.05,
                p[1] + perpendicular[1] * width_left
            ])
            right = np.array([
                p[0] - perpendicular[0] * width_right,
                0.05,
                p[1] - perpendicular[1] * width_right
            ])

            vertices.append(left)
            vertices.append(right)

        # Create triangular faces (including closing the loop)
        for i in range(n_points):
            # Two triangles per segment
            v0 = i * 2      # left current
            v1 = i * 2 + 1  # right current

            # Next point (wraps around to close the loop)
            next_i = (i + 1) % n_points
            v2 = next_i * 2      # left next
            v3 = next_i * 2 + 1  # right next

            # First triangle
            faces.append([v0, v1, v2])
            # Second triangle
            faces.append([v1, v3, v2])

        vertices = np.array(vertices)
        faces = np.array(faces)

        # Create color array (one color per vertex)
        gray_color = np.array([0.2, 0.2, 0.2])
        colors = np.tile(gray_color, (len(vertices), 1))

        # Create surface actor
        track_surface = actor.surface(
            vertices,
            faces=faces,
            colors=colors  # Dark gray asphalt
        )
        self.scene.add(track_surface)

    def _add_ground(self):
        """Add ground plane"""
        # Calculate track bounds
        if len(self.track_centerline) > 0:
            min_x = np.min(self.track_centerline[:, 0]) - 50
            max_x = np.max(self.track_centerline[:, 0]) + 50
            min_z = np.min(self.track_centerline[:, 1]) - 50
            max_z = np.max(self.track_centerline[:, 1]) + 50

            center_x = (min_x + max_x) / 2
            center_z = (min_z + max_z) / 2
            size_x = max_x - min_x
            size_z = max_z - min_z
        else:
            center_x, center_z = 0, 0
            size_x, size_z = 6000, 6000

        # Create ground plane (grass)
        ground = actor.box(
            centers=np.array([[center_x, -1, center_z]]),
            colors=np.array([[0.2, 0.6, 0.2]]),  # Green grass
            scales=np.array([[size_x, 0.1, size_z]])
        )
        self.scene.add(ground)

        # Add start/finish line (aligned with track direction)
        if len(self.track_centerline) > 1:
            self._add_start_line()

    def _add_start_line(self):
        """Add start/finish line aligned with track direction"""
        start_point = self.track_centerline[0]
        next_point = self.track_centerline[1]

        # Calculate track direction at start
        tangent = next_point - start_point
        tangent_len = np.linalg.norm(tangent)
        if tangent_len > 0:
            tangent = tangent / tangent_len
        else:
            return  # Skip if invalid

        # Perpendicular to track (across the width)
        perpendicular = np.array([-tangent[1], tangent[0]])

        # Get width at start
        _, width_left, width_right = self._get_track_width(0)

        # Create a thin rectangular stripe across the track
        line_thickness = 1.0  # 1 meter thick stripe along track direction
        height = 0.06  # Just above track surface

        # Four corners of the start line
        vertices = np.array([
            # Back left
            [start_point[0] + perpendicular[0] * width_left - tangent[0] * line_thickness/2,
             height,
             start_point[1] + perpendicular[1] * width_left - tangent[1] * line_thickness/2],
            # Back right
            [start_point[0] - perpendicular[0] * width_right - tangent[0] * line_thickness/2,
             height,
             start_point[1] - perpendicular[1] * width_right - tangent[1] * line_thickness/2],
            # Front left
            [start_point[0] + perpendicular[0] * width_left + tangent[0] * line_thickness/2,
             height,
             start_point[1] + perpendicular[1] * width_left + tangent[1] * line_thickness/2],
            # Front right
            [start_point[0] - perpendicular[0] * width_right + tangent[0] * line_thickness/2,
             height,
             start_point[1] - perpendicular[1] * width_right + tangent[1] * line_thickness/2],
        ])

        # Two triangles to form the rectangle
        faces = np.array([
            [0, 1, 2],
            [1, 3, 2]
        ])

        # White color for all vertices
        colors = np.ones((4, 3))  # All white

        # Create surface
        start_line = actor.surface(vertices, faces=faces, colors=colors)
        self.scene.add(start_line)

    def get_scene(self):
        """Return the FURY scene"""
        return self.scene

    def get_track_boundaries(self):
        """Return track boundaries for SLAM with variable width"""
        inner_boundary = []
        outer_boundary = []

        for i in range(len(self.track_centerline)):
            p = self.track_centerline[i]

            if i < len(self.track_centerline) - 1:
                next_p = self.track_centerline[i + 1]
                tangent = next_p - p
            else:
                prev_p = self.track_centerline[i - 1]
                tangent = p - prev_p

            tangent_len = np.linalg.norm(tangent)
            if tangent_len > 0:
                tangent = tangent / tangent_len
            else:
                tangent = np.array([1, 0])

            normal = np.array([-tangent[1], tangent[0]])

            # Get width at this point
            _, width_left, width_right = self._get_track_width(i)

            inner = p - normal * width_right
            outer = p + normal * width_left

            inner_boundary.append(inner)
            outer_boundary.append(outer)

        return {
            'centerline': self.track_centerline,
            'inner_boundary': np.array(inner_boundary),
            'outer_boundary': np.array(outer_boundary),
            'variable_width': self.use_variable_width,
            'width_data': self.track_width_data
        }


def show_environment(use_variable_width=True):
    """Display the improved track environment"""
    track = MonzaTrack3DImproved(use_variable_width=use_variable_width)
    scene = track.get_scene()

    # Set up camera to view the entire track
    if len(track.track_centerline) > 0:
        center_x = np.mean(track.track_centerline[:, 0])
        center_z = np.mean(track.track_centerline[:, 1])

        scene.set_camera(position=(center_x, 800, center_z + 500),
                         focal_point=(center_x, 0, center_z),
                         view_up=(0, 1, 0))

    # Create window and show
    title = "Monza Track 3D - Improved (Variable Width)" if use_variable_width else \
            "Monza Track 3D - Improved (Constant Width)"

    showm = window.ShowManager(scene=scene, size=(1400, 900), title=title)

    print("\nControls:")
    print("- Left mouse: Rotate")
    print("- Right mouse: Zoom")
    print("- Middle mouse: Pan")
    print(f"\nTrack width: {'Variable (from TUMFTM data)' if use_variable_width else f'Constant ({track.default_track_width}m)'}")

    showm.start()


if __name__ == "__main__":
    # Show with variable width (requires width data file)
    show_environment(use_variable_width=True)
