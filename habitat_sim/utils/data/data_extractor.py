import math
import os

import matplotlib.pyplot as plt
import numpy as np

import habitat_sim
import habitat_sim.bindings as hsim
from habitat_sim.agent import AgentState
from habitat_sim.utils.data.pose_extractor import PoseExtractor


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
            "semantic": "semantic_sensor",
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

    def close(self):
        r"""Deletes the instance of the simulator. Necessary for instatiating a different ImageExtractor.
        """
        if self.sim is not None:
            self.sim.close()
            del self.sim
            self.sim = None

    def _config_sim(self, scene_filepath, img_size):
        settings = {
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
        self.topdown_view = sim.pathfinder.get_topdown_view(
            pixels_per_meter, height
        ).astype(np.float64)
