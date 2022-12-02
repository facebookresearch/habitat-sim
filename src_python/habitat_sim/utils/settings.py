# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

from typing import Any, Dict, Optional

import habitat_sim
import habitat_sim.agent

# [default_sim_settings]
default_sim_settings: Dict[str, Any] = {
    "scene_dataset_config_file": "default",
    "scene": "NONE",
    "width": 640,
    "height": 480,
    "default_agent": 0,
    "seed": 1,
    "physics_config_file": "data/default.physics_config.json",
    "sensors": {
        # Examples:
        # "color_sensor": {},
        # "depth_sensor": {},
        # If left empty, a default sensor called "color_sensor" will be created.
        # Any unassigned sensor settings are populated with the default values below
    },
}
# [/default_sim_settings]

default_sensor_settings: Dict[str, Any] = {
    "hfov": 90,
    "position": [0, 1.5, 0],
    "orientation": [0, 0, 0],
    "sensor_type": habitat_sim.sensor.SensorType.COLOR,
    "sensor_subtype": habitat_sim.sensor.SensorSubType.PINHOLE,
    # can be an int agent ID or "None".
    # TODO: If the value is "None", then make it a global sensor attached to root scene node
    "agent_id": default_sim_settings["default_agent"],
}


# TODO: possibly remove, useful for debugging and testing
def print_settings(settings: Dict[str, Any], nest_level: Optional[int] = 0):
    for k, v in settings.items():
        if isinstance(v, Dict):
            print("   " * nest_level + f"{k}:")
            print_settings(v, nest_level + 1)
        else:
            print("   " * nest_level + f"{k}: {v}")


def update_sensor_settings(settings: Dict[str, Any], uuid: str, **kw_args) -> None:
    if "sensors" not in settings:
        settings["sensors"] = {}
    if uuid not in settings["sensors"]:
        # if there is no Dict of sensor settings for a sensor with this uuid, create a new
        # Dict of sensor settings with this uuid and initiate all sensor setting fields
        # to their default values
        settings["sensors"][uuid] = default_sensor_settings.copy()

    # update all Dict fields in the given sensor settings with the new values
    for k in kw_args:
        settings["sensors"][uuid][k] = kw_args[k]


# build SimulatorConfiguration
def make_cfg(settings: Dict[str, Any]):
    r"""Isolates the boilerplate code to create a habitat_sim.Configuration from a settings dictionary.

    :param settings: A dict with pre-defined keys, each a basic simulator initialization parameter.
    Allows configuration of dataset and scene, visual sensor parameters, and basic agent parameters.
    Optionally creates up to one of each of a variety of aligned visual sensors under Agent 0.
    The output can be passed directly into habitat_sim.simulator.Simulator constructor or reconfigure
    to initialize a Simulator instance.
    """
    sim_cfg = habitat_sim.SimulatorConfiguration()

    # define scene and gpu device parameters
    if "scene_dataset_config_file" in settings:
        sim_cfg.scene_dataset_config_file = settings["scene_dataset_config_file"]

    if "enable_physics" in settings:
        sim_cfg.enable_physics = settings["enable_physics"]

    if "physics_config_file" in settings:
        sim_cfg.physics_config_file = settings["physics_config_file"]

    if "scene_light_setup" in settings:
        sim_cfg.scene_light_setup = settings["scene_light_setup"]

    sim_cfg.frustum_culling = settings.get("frustum_culling", False)

    sim_cfg.gpu_device_id = 0

    if not hasattr(sim_cfg, "scene_id"):
        raise RuntimeError(
            "Error: Please upgrade habitat-sim. SimulatorConfig API version mismatch"
        )
    sim_cfg.scene_id = settings["scene"]

    # define default sensor parameters (see src/esp/Sensor/Sensor.h)
    sensor_specs = []

    def create_camera_spec(**kw_args):
        camera_sensor_spec = habitat_sim.CameraSensorSpec()
        for k in kw_args:
            setattr(camera_sensor_spec, k, kw_args[k])
        return camera_sensor_spec

    # TODO Figure out how to implement copying of specs
    def create_fisheye_spec(**kw_args):
        fisheye_sensor_spec = habitat_sim.FisheyeSensorDoubleSphereSpec()
        fisheye_sensor_spec.sensor_model_type = (
            habitat_sim.FisheyeSensorModelType.DOUBLE_SPHERE
        )

        # The default value (alpha, xi) is set to match the lens "GoPro" found in Table 3 of this paper:
        # Vladyslav Usenko, Nikolaus Demmel and Daniel Cremers: The Double Sphere
        # Camera Model, The International Conference on 3D Vision (3DV), 2018
        # You can find the intrinsic parameters for the other lenses in the same table as well.
        fisheye_sensor_spec.xi = -0.27
        fisheye_sensor_spec.alpha = 0.57
        fisheye_sensor_spec.focal_length = [364.84, 364.86]

        # The default principal_point_offset is the middle of the image
        fisheye_sensor_spec.principal_point_offset = None

        # default: fisheye_sensor_spec.principal_point_offset = [i/2 for i in fisheye_sensor_spec.resolution]
        for k in kw_args:
            setattr(fisheye_sensor_spec, k, kw_args[k])
        return fisheye_sensor_spec

    def create_equirect_spec(**kw_args):
        equirect_sensor_spec = habitat_sim.EquirectangularSensorSpec()
        for k in kw_args:
            setattr(equirect_sensor_spec, k, kw_args[k])
        return equirect_sensor_spec

    # if user has not specified any sensors of their own,
    # use default sensor with uuid of "color_sensor"
    if len(settings.get("sensors")) == 0:
        settings["sensors"]["color_sensor"] = {}

    for uuid, sensor_settings in settings["sensors"].items():

        # update all non assigned sensor setting fields to their default values
        sensor_settings = {**default_sensor_settings, **sensor_settings}

        channels = (
            4
            if sensor_settings["sensor_type"] is habitat_sim.sensor.SensorType.COLOR
            else 1
        )

        if (  # fisheye_rgba_sensor, fisheye_depth_sensor, fisheye_semantic_sensor
            "fisheye" in uuid
        ):
            fisheye_spec = create_fisheye_spec(
                uuid=uuid,
                position=sensor_settings["position"],
                orientation=sensor_settings["orientation"],
                resolution=[settings["height"], settings["width"]],
                sensor_type=sensor_settings["sensor_type"],
                channels=channels,
            )
            sensor_specs.append(fisheye_spec)
        elif (  # equirect_rgba_sensor, equirect_depth_sensor, equirect_semantic_sensor
            "equirect" in uuid
        ):
            equirect_spec = create_equirect_spec(
                uuid=uuid,
                position=sensor_settings["position"],
                orientation=sensor_settings["orientation"],
                resolution=[settings["height"], settings["width"]],
                sensor_type=sensor_settings["sensor_type"],
                channels=channels,
            )
            sensor_specs.append(equirect_spec)
        else:  # color_sensor, depth_sensor, semantic_sensor, ortho_rgba_sensor, ortho_depth_sensor, ortho_semantic_sensor
            camera_spec = create_camera_spec(
                uuid=uuid,
                hfov=sensor_settings["hfov"],
                position=sensor_settings["position"],
                orientation=sensor_settings["orientation"],
                resolution=[settings["height"], settings["width"]],
                sensor_type=sensor_settings["sensor_type"],
                sensor_subtype=sensor_settings["sensor_subtype"],
                channels=channels,
            )
            sensor_specs.append(camera_spec)

    # create agent specifications
    agent_cfg = habitat_sim.agent.AgentConfiguration()
    agent_cfg.sensor_specifications = sensor_specs
    agent_cfg.action_space = {
        "move_forward": habitat_sim.agent.ActionSpec(
            "move_forward", habitat_sim.agent.ActuationSpec(amount=0.25)
        ),
        "turn_left": habitat_sim.agent.ActionSpec(
            "turn_left", habitat_sim.agent.ActuationSpec(amount=10.0)
        ),
        "turn_right": habitat_sim.agent.ActionSpec(
            "turn_right", habitat_sim.agent.ActuationSpec(amount=10.0)
        ),
    }

    return habitat_sim.Configuration(sim_cfg, [agent_cfg])
