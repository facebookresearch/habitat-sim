import collections
import math
import os.path as osp
import pdb

import matplotlib.pyplot as plt
import numpy as np

import habitat_sim
import habitat_sim.bindings as hsim
from examples.settings import make_cfg
from habitat_sim.agent import AgentState
from habitat_sim.utils.common import quat_from_angle_axis


class DataExtractor(object):
    r"""Main class that extracts data by creating a simulator and generating a topdown map from which to
    iteratively generate image data
    """

    def __init__(self, scene_filepath, label):
        self.scene_filepath = scene_filepath
        self.cfg = self._config_sim(self.scene_filepath)
        self.sim = habitat_sim.Simulator(self.cfg)

        self.res = 0.1
        self.label = label
        self.topdown_view = self.get_topdown_view(self.sim.pathfinder, res=self.res)
        self.pose_extractor = PoseExtractor(
            self.topdown_view, self.sim.pathfinder, self.res
        )
        self.poses = self.pose_extractor.extract_poses(
            dist=20, label=self.label
        )  # list of poses

    def get_topdown_view(self, pathfinder, res=0.1):
        topdown_view = np.array(pathfinder.get_topdown_view(res)).astype(np.float64)
        return topdown_view

    def has_next(self):
        return len(self.poses) > 0

    def next(self):
        if not self.poses:
            return None

        pos, rot = self.poses.pop()

        # Set agent state to the specified pose and return the observed rgb image
        new_state = AgentState()
        new_state.position = pos
        new_state.rotation = rot
        self.sim.agents[0].set_state(new_state)
        obs = self.sim.get_sensor_observations()
        return obs["color_sensor"]

    def _config_sim(self, scene_filepath):
        sim_settings = {
            "width": 512,  # Spatial resolution of the observations
            "height": 512,
            "scene": scene_filepath,  # Scene path
            "default_agent": 0,
            "sensor_height": 1.5,  # Height of sensors in meters
            "color_sensor": True,  # RGB sensor
            "semantic_sensor": True,  # Semantic sensor
            "depth_sensor": True,  # Depth sensor
            "silent": True,
        }

        return make_cfg(sim_settings)


class PoseExtractor(object):
    def __init__(self, topdown_view, pathfinder, res=0.1):
        self.topdown_view = topdown_view
        self.pathfinder = pathfinder
        self.res = res
        self.reference_point = self._get_pathfinder_reference_point()

    def extract_poses(self, dist, label):
        height, width = self.topdown_view.shape
        n_gridpoints_width, n_gridpoints_height = width // dist - 1, height // dist - 1
        gridpoints = []
        for h in range(n_gridpoints_height):
            for w in range(n_gridpoints_width):
                point = (dist + h * dist, dist + w * dist)
                if self.valid_point(*point):
                    gridpoints.append(point)

        # Find the closest point of the target class to each gridpoint
        poses = []
        for point in [gridpoints[4], gridpoints[5], gridpoints[51]]:
            closest_point_of_interest = self._bfs(point, label)
            r, c = closest_point_of_interest
            poses.append((point, closest_point_of_interest))

        # Convert the coordinates in our reference frame to that of the pathfinder
        startw, startz, starth = self.reference_point
        for i, pose in enumerate(poses):
            pos, cpi = pose
            r1, c1 = pos
            r2, c2 = cpi
            print(f"Pos: {pos}, cpi: {cpi}")
            new_pos = np.array([startw + c1 * self.res, startz, starth + r1 * self.res])
            new_cpi = np.array([startw + c2 * self.res, startz, starth + r2 * self.res])
            cam_normal = new_cpi - new_pos
            new_rot = self._compute_quat(new_pos, cam_normal)
            print(f"Rot: {new_rot}")
            poses[i] = (new_pos, new_rot)

        return poses

    def valid_point(self, row, col):
        return self.topdown_view[row][col]

    def is_point_of_interest(self, point, label):
        return self.topdown_view[point[0], point[1]] == label

    def _get_pathfinder_reference_point(self):
        bound1, bound2 = self.pathfinder.get_bounds()
        startw = min(bound1[0], bound2[0])
        starth = min(bound1[2], bound2[2])
        startz = self.pathfinder.get_random_navigable_point()[
            1
        ]  # Can't think of a better way to get a valid z-axis value
        return (startw, startz, starth)  # width, z, height

    def _compute_quat(self, position, cam_normal):
        def norm(v):
            return math.sqrt(sum([abs(a) ** 2 for a in v]))

        theta = (
            -180
            * np.arccos(
                np.dot(position, cam_normal) / (norm(position) * norm(cam_normal))
            )
            / np.pi
        )

        print(position)
        print(cam_normal)
        print(f"angle: {theta}")
        axis = np.array([0, 1, 0])
        return quat_from_angle_axis(theta, axis)

    def _bfs(self, point, label):
        def get_neighbors(p):
            r, c = p
            step = 2
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

        is_valid = lambda row, col: 0 <= row < len(
            self.topdown_view
        ) and 0 <= col < len(self.topdown_view[0])
        visited = (
            set()
        )  # Can use the topdown view as visited set to save space at cost of time to reset it for each bfs
        q = collections.deque([point])
        while q:
            cur = q.popleft()
            visited.add(cur)
            if self.is_point_of_interest(
                cur, label
            ):  # Looking for False labels right now. Change this param later
                return cur

            for n in get_neighbors(cur):
                if n not in visited and is_valid(*n):
                    q.append(n)

        raise Exception("No closest point of target class found in BFS")
