import collections
import copy
import math
import os

import matplotlib.pyplot as plt
import numpy as np

import habitat_sim
import habitat_sim.bindings as hsim
from habitat_sim.agent import AgentState
from habitat_sim.utils.common import quat_from_two_vectors


def make_cfg(settings):
    sim_cfg = hsim.SimulatorConfiguration()
    sim_cfg.enable_physics = False
    sim_cfg.gpu_device_id = 0
    sim_cfg.scene.id = settings["scene"]

    # define default sensor parameters (see src/esp/Sensor/Sensor.h)
    sensors = {
        "color_sensor": {  # active if sim_settings["color_sensor"]
            "sensor_type": hsim.SensorType.COLOR,
            "resolution": [settings["height"], settings["width"]],
            "position": [0.0, settings["sensor_height"], 0.0],
        },
        "depth_sensor": {  # active if sim_settings["depth_sensor"]
            "sensor_type": hsim.SensorType.DEPTH,
            "resolution": [settings["height"], settings["width"]],
            "position": [0.0, settings["sensor_height"], 0.0],
        },
        "semantic_sensor": {  # active if sim_settings["semantic_sensor"]
            "sensor_type": hsim.SensorType.SEMANTIC,
            "resolution": [settings["height"], settings["width"]],
            "position": [0.0, settings["sensor_height"], 0.0],
        },
    }

    # create sensor specifications
    sensor_specs = []
    for sensor_uuid, sensor_params in sensors.items():
        if settings[sensor_uuid]:
            sensor_spec = hsim.SensorSpec()
            sensor_spec.uuid = sensor_uuid
            sensor_spec.sensor_type = sensor_params["sensor_type"]
            sensor_spec.resolution = sensor_params["resolution"]
            sensor_spec.position = sensor_params["position"]
            sensor_spec.gpu2gpu_transfer = False
            sensor_specs.append(sensor_spec)

    # create agent specifications
    agent_cfg = habitat_sim.agent.AgentConfiguration()
    agent_cfg.sensor_specifications = sensor_specs

    return habitat_sim.Configuration(sim_cfg, [agent_cfg])


class ImageExtractor:
    r"""Main class that extracts data by creating a simulator and generating a topdown map from which to
    iteratively generate image data.

    :property scene_filepath: The location of the .glb file given to the simulator
    :property labels: class labels of things to tather images of
    :property cfg: configuration for simulator of type SimulatorConfiguration
    :property sim: Simulator object
    :property pixels_per_meter: Resolution of topdown map. 0.1 means each pixel in the topdown map
        represents 0.1 x 0.1 meters in the coordinate system of the pathfinder
    :property tdv: TopdownView object
    :property topdown_view: The actual 2D array representing the topdown view
    :property pose_extractor: PoseExtractor object
    :property poses: list of camera poses gathered from pose_extractor
    :property label_map: maps lable numbers on the topdown map to their name
    :property out_name_to_sensor_name: maps name of output to the sensor same corresponding to that output
    :property output: list of output names that the user wants e.g. ['rgba', 'depth']
    """

    def __init__(
        self,
        scene_filepath,
        labels=[0.0],
        img_size=(512, 512),
        output=["rgba"],
        sim=None,
    ):
        self.scene_filepath = scene_filepath
        self.labels = set(labels)
        self.cfg = self._config_sim(self.scene_filepath, img_size)

        if sim is None:
            sim = habitat_sim.Simulator(self.cfg)
        else:
            # If a sim is provided we have to make a new cfg
            self.cfg = self._config_sim(sim.config.sim_cfg.scene.id, img_size)
            sim.reconfigure(self.cfg)

        self.sim = sim
        self.pixels_per_meter = 0.1
        ref_point = self._get_pathfinder_reference_point(self.sim.pathfinder)
        self.tdv = TopdownView(self.sim, ref_point[1], self.pixels_per_meter)
        self.topdown_view = self.tdv.topdown_view

        self.pose_extractor = PoseExtractor(
            self.topdown_view, self.sim.pathfinder, self.pixels_per_meter
        )
        self.poses = self.pose_extractor.extract_poses(
            labels=self.labels
        )  # list of poses
        self.label_map = {0.0: "unnavigable", 1.0: "navigable"}

        # Configure the output each data sample
        self.out_name_to_sensor_name = {
            "rgba": "color_sensor",
            "depth": "depth_sensor",
            "semantic": "semamtic_sensor",
        }
        self.output = output

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
        sample = {
            out_name: obs[self.out_name_to_sensor_name[out_name]]
            for out_name in self.output
        }
        sample["label"] = self.label_map[label]

        return sample

    def _config_sim(self, scene_filepath, img_size):
        sim_settings = {
            "width": img_size[1],  # Spatial resolution of the observations
            "height": img_size[0],
            "scene": scene_filepath,  # Scene path
            "default_agent": 0,
            "sensor_height": 1.5,  # Height of sensors in meters
            "color_sensor": True,  # RGBA sensor
            "semantic_sensor": True,  # Semantic sensor
            "depth_sensor": True,  # Depth sensor
            "silent": True,
        }

        return make_cfg(sim_settings)

    def _get_pathfinder_reference_point(self, pf):
        bound1, bound2 = pf.get_bounds()
        startw = min(bound1[0], bound2[0])
        starth = min(bound1[2], bound2[2])
        starty = pf.get_random_navigable_point()[
            1
        ]  # Can't think of a better way to get a valid y-axis value
        return (startw, starty, starth)  # width, y, height


class TopdownView(object):
    def __init__(self, sim, height, pixels_per_meter=0.1):
        self.topdown_view = np.array(
            sim.pathfinder.get_topdown_view(pixels_per_meter, height)
        ).astype(np.float64)


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
        return quat_from_two_vectors(np.array([0, 0, -1]), cam_normal)

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
