# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import collections
from typing import List, Tuple, Union

import attr
import numpy as np
from numpy import bool_, float32, float64, ndarray
from quaternion import quaternion

import habitat_sim
from habitat_sim import registry as registry
from habitat_sim.utils.common import quat_from_two_vectors


@attr.s(auto_attribs=True, init=False, slots=True)
class TopdownView:
    topdown_view: habitat_sim.nav.PathFinder

    def __init__(self, sim, height, meters_per_pixel=0.1):
        self.topdown_view = sim.pathfinder.get_topdown_view(
            meters_per_pixel, height
        ).astype(np.float64)


class PoseExtractor:
    r"""Abstract class that represents a method for extracting camera poses given a list of topdown view(s).

    :property tdv_fp_ref_triples: List of tuples containing (TopdownView Object, scene_filepath, reference point)
        information for each scene. Each scene requires:
            TopdownView: To extract poses
            scene_filepath: The file path to the mesh file. Necessary for scene switches.
            reference point: A reference point from the coordinate system of the scene. Necessary for specifying poses in the scene's coordinate system.

    :property meters_per_pixel: Resolution of topdown map. 0.1 means each pixel in the topdown map
        represents 0.1 x 0.1 meters in the coordinate system of the scene that the map represents.
    :property labels: List of class labels to extract images of.
    """

    def __init__(
        self,
        topdown_views: List[Tuple[TopdownView, str, Tuple[float32, float32, float32]]],
        meters_per_pixel: float = 0.1,
    ) -> None:
        self.tdv_fp_ref_triples = topdown_views
        self.meters_per_pixel = meters_per_pixel
        self.labels = [0.0]

    def extract_all_poses(self) -> np.ndarray:
        r"""Returns a numpy array of camera poses. For each scene, this method extends the list of poses according to the extraction rule defined in extract_poses."""
        poses = []
        for tdv, fp, ref_point in self.tdv_fp_ref_triples:
            view = (
                tdv.topdown_view
            )  # 2D numpy array representing the topdown view of the scene
            _poses = self._convert_to_scene_coordinate_system(
                self.extract_poses(view, fp), ref_point
            )
            poses.extend(_poses)

        return np.array(poses)

    def extract_poses(
        self, view: np.ndarray, fp: str
    ) -> List[Tuple[Tuple[int, int], Tuple[int, int], str]]:
        r"""Extracts poses according to a programatic rule.

        :property view: 2D numpy array representing the topdown view of the scene.
        :property fp: filepath to the scene (necessary to return to the ImageExtractor).
        """
        raise NotImplementedError

    def _valid_point(self, row: int, col: int, view: ndarray) -> bool_:
        return view[row][col] == 1.0

    def _is_point_of_interest(
        self, point: Tuple[int, int], labels: List[float], view: ndarray
    ) -> Tuple[bool, float64]:
        r, c = point
        is_interesting = False
        if view[r][c] in labels:
            is_interesting = True

        return is_interesting, view[r][c]

    def _compute_quat(self, cam_normal: ndarray) -> quaternion:
        """Rotations start from -z axis"""
        return quat_from_two_vectors(habitat_sim.geo.FRONT, cam_normal)

    def _convert_to_scene_coordinate_system(
        self,
        poses: List[Tuple[Tuple[int, int], Tuple[int, int], str]],
        ref_point: Tuple[float32, float32, float32],
    ) -> List[Tuple[Tuple[int, int], quaternion, str]]:
        # Convert from topdown map coordinate system to that of the scene
        startw, starty, starth = ref_point
        for i, pose in enumerate(poses):
            pos, cpi, filepath = pose
            r1, c1 = pos
            r2, c2 = cpi
            new_pos = np.array(
                [
                    startw + c1 * self.meters_per_pixel,
                    starty,
                    starth + r1 * self.meters_per_pixel,
                ]
            )
            new_cpi = np.array(
                [
                    startw + c2 * self.meters_per_pixel,
                    starty,
                    starth + r2 * self.meters_per_pixel,
                ]
            )
            cam_normal = new_cpi - new_pos
            new_rot = self._compute_quat(cam_normal)
            new_pos_t: Tuple[int, int] = tuple(new_pos)  # type: ignore[assignment]
            poses[i] = (new_pos_t, new_rot, filepath)

        return poses


@registry.register_pose_extractor(name="closest_point_extractor")
class ClosestPointExtractor(PoseExtractor):
    def __init__(
        self,
        topdown_views: List[Tuple[TopdownView, str, Tuple[float32, float32, float32]]],
        meters_per_pixel: float = 0.1,
    ) -> None:
        super().__init__(topdown_views, meters_per_pixel)

    def extract_poses(
        self, view: ndarray, fp: str
    ) -> List[Tuple[Tuple[int, int], Tuple[int, int], str]]:
        # Determine the physical spacing between each camera position
        height, width = view.shape
        dist = min(height, width) // 10  # We can modify this to be user-defined later

        # Create a grid of camera positions
        n_gridpoints_width, n_gridpoints_height = (
            width // dist - 1,
            height // dist - 1,
        )

        # Exclude camera positions at invalid positions
        gridpoints = []
        for h in range(n_gridpoints_height):
            for w in range(n_gridpoints_width):
                point = (dist + h * dist, dist + w * dist)
                if self._valid_point(*point, view):
                    gridpoints.append(point)

        # Find the closest point of the target class to each gridpoint
        poses = []
        for point in gridpoints:
            closest_point_of_interest, label = self._bfs(point, self.labels, view, dist)
            if closest_point_of_interest is None:
                continue

            poses.append((point, closest_point_of_interest, fp))

        # Returns poses in the coordinate system of the topdown view
        return poses

    def _bfs(
        self, point: Tuple[int, int], labels: List[float], view: ndarray, dist: int
    ) -> Union[Tuple[Tuple[int, int], float64], Tuple[None, None]]:
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
        visited = set()
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


@registry.register_pose_extractor(name="panorama_extractor")
class PanoramaExtractor(PoseExtractor):
    def __init__(
        self,
        topdown_views: List[Tuple[TopdownView, str, Tuple[float32, float32, float32]]],
        meters_per_pixel: float = 0.1,
    ) -> None:
        super().__init__(topdown_views, meters_per_pixel)

    def extract_poses(
        self, view: ndarray, fp: str
    ) -> List[Tuple[Tuple[int, int], Tuple[int, int], str]]:
        # Determine the physical spacing between each camera position
        height, width = view.shape
        dist = min(height, width) // 10  # We can modify this to be user-defined later

        # Create a grid of camera positions
        n_gridpoints_width, n_gridpoints_height = (
            width // dist - 1,
            height // dist - 1,
        )

        # Exclude camera positions at invalid positions
        gridpoints = []
        for h in range(n_gridpoints_height):
            for w in range(n_gridpoints_width):
                point = (dist + h * dist, dist + w * dist)
                if self._valid_point(*point, view):
                    gridpoints.append(point)

        # Find the closest point of the target class to each gridpoint
        poses = []
        for point in gridpoints:
            point_label_pairs = self._panorama_extraction(point, view, dist)
            poses.extend([(point, point_, fp) for point_, label in point_label_pairs])

        # Returns poses in the coordinate system of the topdown view
        return poses

    def _panorama_extraction(
        self, point: Tuple[int, int], view: ndarray, dist: int
    ) -> List[Tuple[Tuple[int, int], float]]:
        in_bounds_of_topdown_view = lambda row, col: 0 <= row < len(
            view
        ) and 0 <= col < len(view[0])
        point_label_pairs = []
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
            if in_bounds_of_topdown_view(*n) and self._valid_point(*n, view):
                point_label_pairs.append((n, 0.0))

        return point_label_pairs
