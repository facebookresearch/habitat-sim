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
    :property res: resolution of the topdown view (explained in HabitatDataset)
    :property gridpoints: list of positions for the camera
    :property dist: distance between each camera position
    """

    def __init__(self, topdown_views, sim, res=0.1):
        self.tdv_fp_ref_triples = topdown_views
        self.sim = sim
        self.res = res

    def extract_poses(self, labels):
        poses = []
        for tdv, fp, ref_point in self.tdv_fp_ref_triples:
            view = tdv.topdown_view
            # Determine the physical spacing between each camera position
            x, z = view.shape
            dist = min(x, z) // 10
            poses.extend(
                self.extract_poses_single_scene(labels, view, fp, dist, ref_point)
            )

        return np.array(poses)

    def extract_poses_single_scene(self, labels, view, fp, dist, ref_point):
        height, width = view.shape
        n_gridpoints_width, n_gridpoints_height = (
            width // dist - 1,
            height // dist - 1,
        )
        gridpoints = []
        for h in range(n_gridpoints_height):
            for w in range(n_gridpoints_width):
                point = (dist + h * dist, dist + w * dist)
                if self.valid_point(*point, view):
                    gridpoints.append(point)

        # Find the closest point of the target class to each gridpoint
        poses = []
        cpis = []
        for point in gridpoints:
            closest_point_of_interest, label = self._bfs(point, labels, view, dist)
            if closest_point_of_interest is None:
                continue

            poses.append((point, closest_point_of_interest, label, fp))
            cpis.append(closest_point_of_interest)

        # Convert from topdown map coordinate system to that of the pathfinder
        startw, starty, starth = ref_point
        for i, pose in enumerate(poses):
            pos, cpi, label, filepath = pose
            r1, c1 = pos
            r2, c2 = cpi
            new_pos = np.array([startw + c1 * self.res, starty, starth + r1 * self.res])
            new_cpi = np.array([startw + c2 * self.res, starty, starth + r2 * self.res])
            cam_normal = new_cpi - new_pos
            new_rot = self._compute_quat(cam_normal)
            poses[i] = (new_pos, new_rot, label, filepath)

        return poses

    def valid_point(self, row, col, view):
        return view[row][col] == 1.0

    def is_point_of_interest(self, point, labels, view):
        r, c = point
        is_interesting = False
        if view[r][c] in labels:
            is_interesting = True

        return is_interesting, view[r][c]

    def _show_topdown_view(self, cmap="seismic_r"):
        fig = plt.figure(figsize=(12.0, 12.0))
        columns = 4
        rows = math.ceil(len(self.tdv_fp_ref_triples) / columns)
        for i in range(1, columns * rows + 1):
            if i > len(self.tdv_fp_ref_triples):
                break

            img = self.tdv_fp_ref_triples[i - 1][0].topdown_view
            fig.add_subplot(rows, columns, i)
            plt.xticks([])
            plt.yticks([])
            plt.imshow(img, cmap=cmap)

        plt.show()

    def _compute_quat(self, cam_normal):
        """Rotations start from -z axis"""
        return quat_from_two_vectors(np.array([0, 0, -1]), cam_normal)

    def _bfs(self, point, labels, view, dist):
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
            point_row - 2 * dist,
            point_row + 2 * dist,
            point_col - 2 * dist,
            point_col + 2 * dist,
        ]
        in_bounds = (
            lambda row, col: bounding_box[0] <= row <= bounding_box[1]
            and bounding_box[2] <= col <= bounding_box[3]
        )
        is_valid = lambda row, col: 0 <= row < len(view) and 0 <= col < len(view[0])
        visited = (
            set()
        )  # Can use the topdown view as visited set to save space at cost of time to reset it for each bfs
        q = collections.deque([(point, 0)])
        while q:
            cur, layer = q.popleft()
            if not in_bounds(*cur):  # No point of interest found within bounding box
                return None, None

            visited.add(cur)
            is_point_of_interest, label = self.is_point_of_interest(cur, labels, view)
            if is_point_of_interest:
                if layer > dist / 2:
                    return cur, label
                else:
                    return None, None

            for n in get_neighbors(cur):
                if n not in visited and is_valid(*n):
                    q.append((n, layer + step))

        return None, None
