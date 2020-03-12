import collections
import copy
import math

import matplotlib.pyplot as plt
import numpy as np

import habitat_sim
from habitat_sim.utils.common import quat_from_two_vectors


class PoseExtractor:
    r"""Class that takes in a topdown view and pathfinder and determines a list of reasonable camera poses

    :property tdv_fp_ref_triples: List of tuples containing (TopdownView Object, scene_filepath, reference point)
        information for each scene. Each scene requires:
            TopdownView: To extract poses
            scene_filepath: The file path to the mesh file. Necessary for scene switches.
            reference point: A reference point from the coordinate system of the scene. Necessary for specifying poses
                in the scene's coordinate system.

    :property sim: Simulator object used for pose extraction
    :property pixels_per_meter: Resolution of topdown map. 0.1 means each pixel in the topdown map
        represents 0.1 x 0.1 meters in the coordinate system of the pathfinder
    """

    def __init__(self, topdown_views, sim, pixels_per_meter=0.1):
        self.tdv_fp_ref_triples = topdown_views
        self.sim = sim
        self.pixels_per_meter = pixels_per_meter

    def extract_poses(self, labels, extraction_method):
        r"""Returns a numpy array of camera poses. If extraction method is "closest", this method will
        use bfs to find the closest point of interest to each camera position. If the extraction method
        is "panorama", it will extract a full panorama of camera poses for each camera position
        """
        poses = []
        for tdv, fp, ref_point in self.tdv_fp_ref_triples:
            view = tdv.topdown_view
            # Determine the physical spacing between each camera position
            x, z = view.shape
            dist = min(x, z) // 10
            poses.extend(
                self._extract_poses_single_scene(
                    labels, view, fp, dist, ref_point, extraction_method
                )
            )

        return np.array(poses)

    def _extract_poses_single_scene(
        self, labels, view, fp, dist, ref_point, extraction_method
    ):
        height, width = view.shape
        n_gridpoints_width, n_gridpoints_height = (
            width // dist - 1,
            height // dist - 1,
        )
        gridpoints = []
        for h in range(n_gridpoints_height):
            for w in range(n_gridpoints_width):
                point = (dist + h * dist, dist + w * dist)
                if self._valid_point(*point, view):
                    gridpoints.append(point)

        # Find the closest point of the target class to each gridpoint
        poses = []
        cpis = []
        for point in gridpoints:
            if extraction_method == "closest":
                closest_point_of_interest, label = self._bfs(point, labels, view, dist)
                if closest_point_of_interest is None:
                    continue

                poses.append((point, closest_point_of_interest, label, fp))
                cpis.append(closest_point_of_interest)
            elif extraction_method == "panorama":
                cpi_label_pairs = self._panorama_extraction(point, view, dist)
                poses.extend(
                    [(point, cpi, label, fp) for cpi, label in cpi_label_pairs]
                )
                cpis.extend([cpi for cpi, _ in cpi_label_pairs])

        # Convert from topdown map coordinate system to that of the pathfinder
        startw, starty, starth = ref_point
        for i, pose in enumerate(poses):
            pos, cpi, label, filepath = pose
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
            poses[i] = (new_pos, new_rot, label, filepath)

        return poses

    def _valid_point(self, row, col, view):
        return view[row][col] == 1.0

    def _is_point_of_interest(self, point, labels, view):
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
        return quat_from_two_vectors(habitat_sim.geo.FRONT, cam_normal)

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
            is_point_of_interest, label = self._is_point_of_interest(cur, labels, view)
            if is_point_of_interest:
                if layer > dist / 2:
                    return cur, label
                else:
                    return None, None

            for n in get_neighbors(cur):
                if n not in visited and is_valid(*n):
                    q.append((n, layer + step))

        return None, None

    def _panorama_extraction(self, point, view, dist):
        in_bounds_of_tdv = lambda row, col: 0 <= row < len(view) and 0 <= col < len(
            view[0]
        )
        cpi_label_pairs = []
        r, c = point
        neighbor_dist = dist // 2
        neighbors = [
            (r - neighbor_dist, c - neighbor_dist),
            (r - neighbor_dist, c),
            (r - neighbor_dist, c + neighbor_dist),
            (r, c - neighbor_dist),
            (r, c + neighbor_dist),
            (r + neighbor_dist, c - neighbor_dist),
            # (r + step, c), # Exclude the pose that is in the opposite direction of habitat_sim.geo.FRONT, causes the quaternion computation to mess up
            (r + neighbor_dist, c + neighbor_dist),
        ]

        for n in neighbors:
            # Only add the neighbor point if it is navigable. This prevents camera poses that
            # are just really close-up photos of some object
            if in_bounds_of_tdv(*n) and self._valid_point(*n, view):
                cpi_label_pairs.append((n, 0.0))

        return cpi_label_pairs
