# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import habitat_sim
import habitat_sim.agent

default_sim_settings = {
    # settings shared by example.py and benchmark.py
    "max_frames": 1000,
    "width": 640,
    "height": 480,
    "default_agent": 0,
    "sensor_height": 1.5,
    "color_sensor": True,  # RGB sensor (default: ON)
    "semantic_sensor": False,  # semantic sensor (default: OFF)
    "depth_sensor": False,  # depth sensor (default: OFF)
    "ortho_sensor": False,  # Orthographic RGB sensor (default: OFF)
    "seed": 1,
    "silent": False,  # do not print log info (default: OFF)
    # settings exclusive to example.py
    "save_png": False,  # save the pngs to disk (default: OFF)
    "print_semantic_scene": False,
    "print_semantic_mask_stats": False,
    "compute_shortest_path": False,
    "compute_action_shortest_path": False,
    "scene": "data/scene_datasets/habitat-test-scenes/skokloster-castle.glb",
    "test_scene_data_url": "http://dl.fbaipublicfiles.com/habitat/habitat-test-scenes.zip",
    "goal_position": [5.047, 0.199, 11.145],
    "enable_physics": False,
    "enable_gfx_replay_save": False,
    "physics_config_file": "./data/default.physics_config.json",
    "num_objects": 10,
    "test_object_index": 0,
    "frustum_culling": True,
}

# build SimulatorConfiguration
def make_cfg(settings):
    sim_cfg = habitat_sim.SimulatorConfiguration()
    if "frustum_culling" in settings:
        sim_cfg.frustum_culling = settings["frustum_culling"]
    else:
        sim_cfg.frustum_culling = False
    if "enable_physics" in settings:
        sim_cfg.enable_physics = settings["enable_physics"]
    if "physics_config_file" in settings:
        sim_cfg.physics_config_file = settings["physics_config_file"]
    if not settings["silent"]:
        print("sim_cfg.physics_config_file = " + sim_cfg.physics_config_file)
    if "scene_light_setup" in settings:
        sim_cfg.scene_light_setup = settings["scene_light_setup"]
    sim_cfg.gpu_device_id = 0
    if not hasattr(sim_cfg, "scene_id"):
        raise RuntimeError(
            "Error: Please upgrade habitat-sim. SimulatorConfig API version mismatch"
        )
    sim_cfg.scene_id = settings["scene"]

    # define default sensor parameters (see src/esp/Sensor/Sensor.h)
    sensors = {
        "color_sensor": {  # active if sim_settings["color_sensor"]
            "sensor_type": habitat_sim.SensorType.COLOR,
            "resolution": [settings["height"], settings["width"]],
            "position": [0.0, settings["sensor_height"], 0.0],
            "sensor_subtype": habitat_sim.SensorSubType.PINHOLE,
        },
        "depth_sensor": {  # active if sim_settings["depth_sensor"]
            "sensor_type": habitat_sim.SensorType.DEPTH,
            "resolution": [settings["height"], settings["width"]],
            "position": [0.0, settings["sensor_height"], 0.0],
            "sensor_subtype": habitat_sim.SensorSubType.PINHOLE,
        },
        "semantic_sensor": {  # active if sim_settings["semantic_sensor"]
            "sensor_type": habitat_sim.SensorType.SEMANTIC,
            "resolution": [settings["height"], settings["width"]],
            "position": [0.0, settings["sensor_height"], 0.0],
            "sensor_subtype": habitat_sim.SensorSubType.PINHOLE,
        },
        "ortho_sensor": {  # active if sim_settings["ortho_sensor"]
            "sensor_type": habitat_sim.SensorType.COLOR,
            "resolution": [settings["height"], settings["width"]],
            "position": [0.0, settings["sensor_height"], 0.0],
            "sensor_subtype": habitat_sim.SensorSubType.ORTHOGRAPHIC,
        },
    }

    # create sensor specifications
    sensor_specs = []
    for sensor_uuid, sensor_params in sensors.items():
        if settings[sensor_uuid]:
            sensor_spec = habitat_sim.SensorSpec()
            sensor_spec.uuid = sensor_uuid
            sensor_spec.sensor_type = sensor_params["sensor_type"]
            sensor_spec.sensor_subtype = sensor_params["sensor_subtype"]
            sensor_spec.resolution = sensor_params["resolution"]
            sensor_spec.position = sensor_params["position"]
            sensor_spec.gpu2gpu_transfer = False
            if not settings["silent"]:
                print("==== Initialized Sensor Spec: =====")
                print("Sensor uuid: ", sensor_spec.uuid)
                print("Sensor type: ", sensor_spec.sensor_type)
                print("Sensor position: ", sensor_spec.position)
                print("===================================")

            sensor_specs.append(sensor_spec)

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

    # override action space to no-op to test physics
    if sim_cfg.enable_physics:
        agent_cfg.action_space = {
            "move_forward": habitat_sim.agent.ActionSpec(
                "move_forward", habitat_sim.agent.ActuationSpec(amount=0.0)
            )
        }

    return habitat_sim.Configuration(sim_cfg, [agent_cfg])
