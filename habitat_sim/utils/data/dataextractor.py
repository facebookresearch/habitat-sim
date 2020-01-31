import collections
import math

import matplotlib.pyplot as plt
import numpy as np
import torch
from torch.utils.data import Dataset

import habitat_sim
import habitat_sim.bindings as hsim
from examples.settings import make_cfg
from habitat_sim.agent import AgentState
from habitat_sim.utils.common import quat_from_two_vectors


class HabitatDataset(Dataset):
    r"""Main class that extracts data by creating a simulator and generating a topdown map from which to
    iteratively generate image data
    """

    def __init__(self, scene_filepath, labels, img_size=(512, 512)):
        self.scene_filepath = scene_filepath
        self.labels = set(labels)
        self.img_size = img_size
        self.cfg = self._config_sim(self.scene_filepath)
        self.sim = habitat_sim.Simulator(self.cfg)
        self.res = 0.1
        self.tdv = TopdownView(self.sim, self.res)
        self.topdown_view = self.tdv.topdown_view

        self.pose_extractor = PoseExtractor(
            self.topdown_view, self.sim.pathfinder, self.res
        )
        self.poses = self.pose_extractor.extract_poses(
            dist=20, labels=self.labels
        )  # list of poses

        self.label_map = {0.0: "unnavigable", 1.0: "navigable"}

    def __len__(self):
        return len(self.poses)

    def __getitem__(self, idx):
        if isinstance(idx, slice):
            start, stop, step = idx.start, idx.stop, idx.step
            if start is None:
                start = 0
            if stop is None:
                stop = len(self.poses)
            if step is None:
                step = 1

            return [
                self.__getitem__(i)
                for i in range(start, stop, step)
                if i < len(self.poses)
            ]

        pos, rot, label = self.poses[idx]
        new_state = AgentState()
        new_state.position = pos
        new_state.rotation = rot
        self.sim.agents[0].set_state(new_state)
        obs = self.sim.get_sensor_observations()
        sample = {"img": obs["color_sensor"], "label": self.label_map[label]}

        return sample

    def _config_sim(self, scene_filepath):
        sim_settings = {
            "width": self.img_size[1],  # Spatial resolution of the observations
            "height": self.img_size[0],
            "scene": scene_filepath,  # Scene path
            "default_agent": 0,
            "sensor_height": 1.5,  # Height of sensors in meters
            "color_sensor": True,  # RGB sensor
            "semantic_sensor": True,  # Semantic sensor
            "depth_sensor": True,  # Depth sensor
            "silent": True,
        }

        return make_cfg(sim_settings)


class TopdownView(object):
    def __init__(self, sim, res=0.1):
        self.topdown_view = np.array(sim.pathfinder.get_topdown_view(res)).astype(
            np.float64
        )


class PoseExtractor(object):
    def __init__(self, topdown_view, pathfinder, res=0.1):
        self.topdown_view = topdown_view
        self.pathfinder = pathfinder
        self.res = res
        self.reference_point = self._get_pathfinder_reference_point()

    def extract_poses(self, dist, labels):
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
        for point in gridpoints[3:8]:
            closest_point_of_interest, label = self._bfs(point, labels)
            poses.append((point, closest_point_of_interest, label))

        # Convert the coordinates in our reference frame to that of the pathfinder
        startw, startz, starth = self.reference_point
        for i, pose in enumerate(poses):
            pos, cpi, label = pose
            r1, c1 = pos
            r2, c2 = cpi
            new_pos = np.array([startw + c1 * self.res, startz, starth + r1 * self.res])
            new_cpi = np.array([startw + c2 * self.res, startz, starth + r2 * self.res])
            cam_normal = new_cpi - new_pos
            new_rot = self._compute_quat(cam_normal)
            poses[i] = (new_pos, new_rot, label)

        return poses

    def valid_point(self, row, col):
        return self.topdown_view[row][col]

    def is_point_of_interest(self, point, labels):
        r, c = point
        is_interesting = False
        if self.topdown_view[r][c] in labels:
            is_interesting = True

        return is_interesting, self.topdown_view[r][c]

    def _get_pathfinder_reference_point(self):
        bound1, bound2 = self.pathfinder.get_bounds()
        startw = min(bound1[0], bound2[0])
        starth = min(bound1[2], bound2[2])
        startz = self.pathfinder.get_random_navigable_point()[
            1
        ]  # Can't think of a better way to get a valid z-axis value
        return (startw, startz, starth)  # width, z, height

    def _compute_quat(self, cam_normal):
        """Rotations start from -z axis"""
        return quat_from_two_vectors(np.array([0, 0, -1]), cam_normal)

    def _bfs(self, point, labels):
        def get_neighbors(p):
            r, c = p
            step = 3
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
            is_point_of_interest, label = self.is_point_of_interest(cur, labels)
            if is_point_of_interest:
                return cur, label

            for n in get_neighbors(cur):
                if n not in visited and is_valid(*n):
                    q.append(n)

        raise Exception("No closest point of target class found in BFS")
