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
    "hfov": 90,
    "color_sensor": True,  # RGB sensor (default: ON)
    "semantic_sensor": False,  # semantic sensor (default: OFF)
    "depth_sensor": False,  # depth sensor (default: OFF)
    "ortho_rgba_sensor": False,  # Orthographic RGB sensor (default: OFF)
    "ortho_depth_sensor": False,  # Orthographic depth sensor (default: OFF)
    "ortho_semantic_sensor": False,  # Orthographic semantic sensor (default: OFF)
    "fisheye_rgba_sensor": False,
    "fisheye_depth_sensor": False,
    "fisheye_semantic_sensor": False,
    "equirect_rgba_sensor": False,
    "equirect_depth_sensor": False,
    "equirect_semantic_sensor": False,
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
    sensor_specs = []

    def create_camera_spec(**kw_args):
        camera_sensor_spec = habitat_sim.CameraSensorSpec()
        camera_sensor_spec.sensor_type = habitat_sim.SensorType.COLOR
        camera_sensor_spec.resolution = [settings["height"], settings["width"]]
        camera_sensor_spec.position = [0, settings["sensor_height"], 0]
        for k in kw_args:
            setattr(camera_sensor_spec, k, kw_args[k])
        return camera_sensor_spec

    if settings["color_sensor"]:
        color_sensor_spec = create_camera_spec(
            uuid="color_sensor",
            hfov=settings["hfov"],
            sensor_type=habitat_sim.SensorType.COLOR,
            sensor_subtype=habitat_sim.SensorSubType.PINHOLE,
        )
        sensor_specs.append(color_sensor_spec)

    if settings["depth_sensor"]:
        depth_sensor_spec = create_camera_spec(
            uuid="depth_sensor",
            hfov=settings["hfov"],
            sensor_type=habitat_sim.SensorType.DEPTH,
            channels=1,
            sensor_subtype=habitat_sim.SensorSubType.PINHOLE,
        )
        sensor_specs.append(depth_sensor_spec)

    if settings["semantic_sensor"]:
        semantic_sensor_spec = create_camera_spec(
            uuid="semantic_sensor",
            hfov=settings["hfov"],
            sensor_type=habitat_sim.SensorType.SEMANTIC,
            channels=1,
            sensor_subtype=habitat_sim.SensorSubType.PINHOLE,
        )
        sensor_specs.append(semantic_sensor_spec)

    if settings["ortho_rgba_sensor"]:
        ortho_rgba_sensor_spec = create_camera_spec(
            uuid="ortho_rgba_sensor",
            sensor_type=habitat_sim.SensorType.COLOR,
            sensor_subtype=habitat_sim.SensorSubType.ORTHOGRAPHIC,
        )
        sensor_specs.append(ortho_rgba_sensor_spec)

    if settings["ortho_depth_sensor"]:
        ortho_depth_sensor_spec = create_camera_spec(
            uuid="ortho_depth_sensor",
            sensor_type=habitat_sim.SensorType.DEPTH,
            channels=1,
            sensor_subtype=habitat_sim.SensorSubType.ORTHOGRAPHIC,
        )
        sensor_specs.append(ortho_depth_sensor_spec)

    if settings["ortho_semantic_sensor"]:
        ortho_semantic_sensor_spec = create_camera_spec(
            uuid="ortho_semantic_sensor",
            sensor_type=habitat_sim.SensorType.SEMANTIC,
            channels=1,
            sensor_subtype=habitat_sim.SensorSubType.ORTHOGRAPHIC,
        )
        sensor_specs.append(ortho_semantic_sensor_spec)

    # TODO Figure out how to implement copying of specs
    def create_fisheye_spec(**kw_args):
        fisheye_sensor_spec = habitat_sim.FisheyeSensorDoubleSphereSpec()
        fisheye_sensor_spec.uuid = "fisheye_sensor"
        fisheye_sensor_spec.sensor_type = habitat_sim.SensorType.COLOR
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

        fisheye_sensor_spec.resolution = [settings["height"], settings["width"]]
        # The default principal_point_offset is the middle of the image
        fisheye_sensor_spec.principal_point_offset = None
        # default: fisheye_sensor_spec.principal_point_offset = [i/2 for i in fisheye_sensor_spec.resolution]
        fisheye_sensor_spec.position = [0, settings["sensor_height"], 0]
        for k in kw_args:
            setattr(fisheye_sensor_spec, k, kw_args[k])
        return fisheye_sensor_spec

    if settings["fisheye_rgba_sensor"]:
        fisheye_rgba_sensor_spec = create_fisheye_spec(uuid="fisheye_rgba_sensor")
        sensor_specs.append(fisheye_rgba_sensor_spec)
    if settings["fisheye_depth_sensor"]:
        fisheye_depth_sensor_spec = create_fisheye_spec(
            uuid="fisheye_depth_sensor",
            sensor_type=habitat_sim.SensorType.DEPTH,
            channels=1,
        )
        sensor_specs.append(fisheye_depth_sensor_spec)
    if settings["fisheye_semantic_sensor"]:
        fisheye_semantic_sensor_spec = create_fisheye_spec(
            uuid="fisheye_semantic_sensor",
            sensor_type=habitat_sim.SensorType.SEMANTIC,
            channels=1,
        )
        sensor_specs.append(fisheye_semantic_sensor_spec)

    def create_equirect_spec(**kw_args):
        equirect_sensor_spec = habitat_sim.EquirectangularSensorSpec()
        equirect_sensor_spec.uuid = "equirect_rgba_sensor"
        equirect_sensor_spec.sensor_type = habitat_sim.SensorType.COLOR
        equirect_sensor_spec.resolution = [settings["height"], settings["width"]]
        equirect_sensor_spec.position = [0, settings["sensor_height"], 0]
        for k in kw_args:
            setattr(equirect_sensor_spec, k, kw_args[k])
        return equirect_sensor_spec

    if settings["equirect_rgba_sensor"]:
        equirect_rgba_sensor_spec = create_equirect_spec(uuid="equirect_rgba_sensor")
        sensor_specs.append(equirect_rgba_sensor_spec)

    if settings["equirect_depth_sensor"]:
        equirect_depth_sensor_spec = create_equirect_spec(
            uuid="equirect_depth_sensor",
            sensor_type=habitat_sim.SensorType.DEPTH,
            channels=1,
        )
        sensor_specs.append(equirect_depth_sensor_spec)

    if settings["equirect_semantic_sensor"]:
        equirect_semantic_sensor_spec = create_equirect_spec(
            uuid="equirect_semantic_sensor",
            sensor_type=habitat_sim.SensorType.SEMANTIC,
            channels=1,
        )
        sensor_specs.append(equirect_semantic_sensor_spec)

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
