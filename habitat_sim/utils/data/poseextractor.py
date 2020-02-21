import collections
import copy

import matplotlib.pyplot as plt
import numpy as np

import habitat_sim
from habitat_sim.utils.common import quat_from_two_vectors


class PoseExtractor(object):
    r"""Class that takes in a topdown view and pathfinder and determines a list of reasonable camera poses

    :property topdown_view: 2D array representing topdown view of scene
    :property pathfinder: the pathfinder from the Simulator object
    :property pixels_per_meter: resolution of the topdown view (explained in ImageExtractor)
    :property gridpoints: list of positions for the camera
    :property dist: distance between each camera position
    """

    def __init__(self, topdown_view, pathfinder, pixels_per_meter=0.1):
        self.topdown_view = topdown_view
        self.pathfinder = pathfinder
        self.pixels_per_meter = pixels_per_meter
        self.gridpoints = None

        # Determine the physical spacing between each camera position
        x, z = self.topdown_view.shape
        self.dist = (
            min(x, z) // 10
        )  # This produces lots of simular images for small scenes. Perhaps a smarter solution exists

    def extract_poses(self, labels):
        r"""Uses the topdown map to define positions in the scene from which to generate images. Returns
        a list of poses, where each pose is (position, rotation, class label) for the camera. Currently
        class label only supports 'unnavigable points', meaning the user cannot yet specify something
        like 'chair' to obtain images of.

        :property labels: The labels to take images of (currently only supports unnavigable points)
        """
        height, width = self.topdown_view.shape
        n_gridpoints_width, n_gridpoints_height = (
            width // self.dist - 1,
            height // self.dist - 1,
        )
        self.gridpoints = []
        for h in range(n_gridpoints_height):
            for w in range(n_gridpoints_width):
                point = (self.dist + h * self.dist, self.dist + w * self.dist)
                if self.valid_point(*point):
                    self.gridpoints.append(point)

        # Find the closest point of the target class to each gridpoint
        poses = []
        self.cpis = []
        for point in self.gridpoints:
            closest_point_of_interest, label = self._bfs(point, labels)
            if closest_point_of_interest is None:
                continue

            poses.append((point, closest_point_of_interest, label))
            self.cpis.append(closest_point_of_interest)

        # Convert from topdown map coordinate system to that of the pathfinder
        startw, starty, starth = self._get_pathfinder_reference_point()
        for i, pose in enumerate(poses):
            pos, cpi, label = pose
            r1, c1 = pos
            r2, c2 = cpi
            new_pos = np.array(
                [
                    startw + c1 * self.pixels_per_meter,
                    starty,
                    starth + r1 * self.pixels_per_meter,
                ]
            )
            new_cpi = np.array(
                [
                    startw + c2 * self.pixels_per_meter,
                    starty,
                    starth + r2 * self.pixels_per_meter,
                ]
            )
            cam_normal = new_cpi - new_pos
            new_rot = self._compute_quat(cam_normal)
            poses[i] = (new_pos, new_rot, label)

        return poses

    def valid_point(self, row, col):
        r"""Whether a point is navigable

        :property row: row in the topdown view
        :property col: col in the topdown view
        """
        return self.topdown_view[row][col] == 1.0

    def is_point_of_interest(self, point, labels):
        r"""Whether one of the class labels exists at the specified point.

        :property point: the point to consider
        :property labels: The labels to take images of (currently only supports unnavigable points)
        """
        r, c = point
        is_interesting = False
        if self.topdown_view[r][c] in labels:
            is_interesting = True

        return is_interesting, self.topdown_view[r][c]

    def _show_topdown_view(self, cmap="seismic_r", show_valid_points=False):
        if show_valid_points:
            topdown_view_copy = copy.copy(self.topdown_view)
            for p in self.gridpoints:
                r, c = p
                topdown_view_copy[r][c] = 0.5

            for cpi in self.cpis:
                r, c = cpi
                topdown_view_copy[r][c] = 0.65

            plt.imshow(topdown_view_copy, cmap=cmap)
        else:
            plt.imshow(self.topdown_view, cmap=cmap)

        plt.show()

    def _get_pathfinder_reference_point(self):
        bound1, bound2 = self.pathfinder.get_bounds()
        startw = min(bound1[0], bound2[0])
        starth = min(bound1[2], bound2[2])
        starty = self.pathfinder.get_random_navigable_point()[
            1
        ]  # Can't think of a better way to get a valid y-axis value
        return (startw, starty, starth)  # width, y, height

    def _compute_quat(self, cam_normal):
        """Rotations start from -z axis"""
        return quat_from_two_vectors(habitat_sim.geo.FRONT, cam_normal)

    def _bfs(self, point, labels):
        step = 3  # making this larger really speeds up BFS

        def get_neighbors(p):
            r, c = p
            return [
                (r - step, c - step),
                (r - step, c),
                (r - step, c + step),
                (r, c - step),
                (r, c + step),
                (r + step, c - step),
                (r + step, c),
                (r + step, c + step),
            ]

        point_row, point_col = point
        bounding_box = [
            point_row - 2 * self.dist,
            point_row + 2 * self.dist,
            point_col - 2 * self.dist,
            point_col + 2 * self.dist,
        ]
        in_bounds = (
            lambda row, col: bounding_box[0] <= row <= bounding_box[1]
            and bounding_box[2] <= col <= bounding_box[3]
        )
        is_valid = lambda row, col: 0 <= row < len(
            self.topdown_view
        ) and 0 <= col < len(self.topdown_view[0])
        visited = (
            set()
        )  # Can use the topdown view as visited set to save space at cost of time to reset it for each bfs
        q = collections.deque([(point, 0)])
        while q:
            cur, layer = q.popleft()
            if not in_bounds(*cur):  # No point of interest found within bounding box
                return None, None

            visited.add(cur)
            is_point_of_interest, label = self.is_point_of_interest(cur, labels)
            if is_point_of_interest:
                if layer > self.dist / 2:
                    return cur, label
                else:
                    return None, None

            for n in get_neighbors(cur):
                if n not in visited and is_valid(*n):
                    q.append((n, layer + step))

        return None, None
