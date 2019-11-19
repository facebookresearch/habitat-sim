# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import argparse
import glob
import os
import random
import subprocess

import magnum as mn
import numpy as np
from PIL import Image

import habitat_sim
import habitat_sim.utils.common as ut

# --------------------------
# Utilities
# --------------------------


def save_observation(observations, frame, prefix=""):
    if "color" in observations:
        rgb_img = Image.fromarray(observations["color"], mode="RGBA")
        rgb_img.save("quadrotor_prototype_output/" + prefix + "rgba.%05d.png" % frame)
    if "depth" in observations:
        depth_img = Image.fromarray(
            (observations["depth"] / 10 * 255).astype(np.uint8), mode="L"
        )
        depth_img.save(
            "quadrotor_prototype_output/" + prefix + "depth.%05d.png" % frame
        )


def get_random_direction():
    # spherical rejection sample for random unit vector
    rand_dir = np.random.rand(3) * 2.0 - np.array([1.0, 1.0, 1.0])
    while np.linalg.norm(rand_dir) > 1:
        rand_dir = np.random.rand(3) * 2.0 - np.array([1.0, 1.0, 1.0])
    return ut.np_normalized(rand_dir)


class SceneCollisionStepFilter:
    def __init__(self, sim, collision_proxy_id):
        self.sim = sim
        self.collision_proxy_id = collision_proxy_id

    def scene_collision_step_filter(self, start_pos, end_pos):
        a = 0
        # TODO: sync object with


if __name__ == "__main__":

    settings = {"image_height": 512, "image_width": 512, "random_seed": 0}

    # pick a scene file to load
    scene_file = "data/scene_datasets/habitat-test-scenes/skokloster-castle.glb"
    # scene_file = "NONE"

    # NOTE: unix based file management
    os.system("mkdir quadrotor_prototype_output")

    # --------------------------
    # Simulator Setup
    # --------------------------

    # create a SimulatorConfiguration object
    sim_cfg = habitat_sim.SimulatorConfiguration()

    sim_cfg.scene.id = scene_file

    # configure the simulator to intialize a PhysicsManager
    sim_cfg.enable_physics = True

    # configure the simulator to load a specific simulator configuration file (define paths you your object models here)
    sim_cfg.physics_config_file = "data/default.phys_scene_config.json"

    # pick this based on your device
    relative_camera_position = [0.0, 0, 0.0]
    # Note: all sensors must have the same resolution
    sensors = {
        "color": {
            "sensor_type": habitat_sim.SensorType.COLOR,
            "resolution": [settings["image_height"], settings["image_width"]],
            "position": relative_camera_position,
        },
        "depth": {
            "sensor_type": habitat_sim.SensorType.DEPTH,
            "resolution": [settings["image_height"], settings["image_width"]],
            "position": relative_camera_position,
        },
    }

    sensor_specs = []
    for sensor_uuid, sensor_params in sensors.items():
        sensor_spec = habitat_sim.SensorSpec()
        sensor_spec.uuid = sensor_uuid
        sensor_spec.sensor_type = sensor_params["sensor_type"]
        sensor_spec.resolution = sensor_params["resolution"]
        sensor_spec.position = sensor_params["position"]

        sensor_specs.append(sensor_spec)

    # setup the agent/camera
    agent_cfg = habitat_sim.agent.AgentConfiguration()
    agent_cfg.sensor_specifications = sensor_specs

    # setup action space
    agent_cfg.action_space = {
        "move_forward": habitat_sim.agent.ActionSpec(
            "move_forward", habitat_sim.agent.ActuationSpec(amount=0.25)
        ),
        "move_up": habitat_sim.agent.ActionSpec(
            "move_up", habitat_sim.agent.ActuationSpec(amount=0.2)
        ),
        "move_down": habitat_sim.agent.ActionSpec(
            "move_down", habitat_sim.agent.ActuationSpec(amount=0.2)
        ),
        "turn_left": habitat_sim.agent.ActionSpec(
            "turn_left", habitat_sim.agent.ActuationSpec(amount=10.0)
        ),
        "turn_right": habitat_sim.agent.ActionSpec(
            "turn_right", habitat_sim.agent.ActuationSpec(amount=10.0)
        ),
    }

    # generate the full simulator configuration
    combined_cfg = habitat_sim.Configuration(sim_cfg, [agent_cfg])

    # initialize the simulator
    sim = habitat_sim.Simulator(combined_cfg)

    # set a new step filter using a synced KINEMATIC object
    sim.agents[0].controls.move_filter_fn = sim._scene_collision_step_filter

    # set random seed
    random.seed(settings["random_seed"])
    sim.seed(settings["random_seed"])

    # initialize the agent state
    # sim.initialize_agent(0)

    # ----------------------------------
    # Quadrotor Prototype Functionality
    # ----------------------------------

    static_3rd_person_camera_position = np.array([-0.569043, 2.04804, 12.6156])

    # assuming the first entry in the object list corresponds to the quadrotor model
    quadrotor_id = sim.add_object(0)
    sim._collision_proxy_id = quadrotor_id

    # the object will not be affected by forces such as gravity and will remain static until explicitly moved
    sim.set_object_motion_type(habitat_sim.physics.MotionType.KINEMATIC, quadrotor_id)

    # place the object/agent in the air
    sim.set_translation(np.array([-0.569043, 2.04804, 13.6156]), quadrotor_id)
    sim.agents[0].scene_node.translation = sim.get_translation(quadrotor_id)

    # do steps
    num_frames = 301
    next_progress_report = 0
    for frame in range(num_frames):

        if frame >= next_progress_report:
            print("Starting step " + str(frame) + " / " + str(num_frames))
            next_progress_report += num_frames / 10.0

        # sample and do action
        action_names = list(combined_cfg.agents[0].action_space.keys())
        action = random.choice(action_names)

        # count on the filter function to sync the proxy object state with the agent state
        collided = sim._default_agent.act(action)
        if collided:
            print("step " + str(frame) + " collided.")

        # get/save 1st person images
        agent_observations = sim.get_sensor_observations()
        save_observation(agent_observations, frame, "agent_")

        # move camera to 3rd person view
        sim.get_agent(0).scene_node.translation = np.array(
            static_3rd_person_camera_position
        )
        sim.get_agent(0).scene_node.rotation = ut.quat_to_magnum(
            ut.quat_look_at(
                sim.get_translation(quadrotor_id), static_3rd_person_camera_position
            )
        )

        agent_observations = sim.get_sensor_observations()
        save_observation(agent_observations, frame, "3rdperson_")

        # reset the agent for the next step
        sim.get_agent(0).scene_node.translation = sim.get_translation(quadrotor_id)
        sim.get_agent(0).scene_node.rotation = sim.get_rotation(quadrotor_id)

    # make a video from the frames
    fps = 30
    os.system(
        "ffmpeg -y -r "
        + str(fps)
        + " -f image2 -i quadrotor_prototype_output/agent_rgba.%05d.png -f mp4 -q:v 0 -vcodec mpeg4 -r "
        + str(fps)
        + " quadrotor_prototype_output/agent.mp4"
    )
    os.system(
        "ffmpeg -y -r "
        + str(fps)
        + " -f image2 -i quadrotor_prototype_output/3rdperson_rgba.%05d.png -f mp4 -q:v 0 -vcodec mpeg4 -r "
        + str(fps)
        + " quadrotor_prototype_output/3rdperson.mp4"
    )
    os.system("open quadrotor_prototype_output/agent.mp4")
    os.system("open quadrotor_prototype_output/3rdperson.mp4")
