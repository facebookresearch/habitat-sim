# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import argparse
import math
import os
import random

import numpy as np
from PIL import Image
from settings import default_sim_settings

import habitat_sim
import habitat_sim.agent
import habitat_sim.bindings as hsim
from habitat_sim.physics import MotionType


def save_depth_observation(obs, total_frames):
    depth_obs = obs["depth_sensor"]
    depth_img = Image.fromarray((depth_obs / 10 * 255).astype(np.uint8), mode="L")
    depth_img.save("clutter_nav_output/test.depth.%05d.png" % total_frames)


def save_color_observation(obs, total_frames):
    color_obs = obs["color_sensor"]
    color_img = Image.fromarray(color_obs, mode="RGBA")
    color_img.save("clutter_nav_output/test.rgba.%05d.png" % total_frames)


def gen_clutter(sim, num_obj):
    # first cleanup existing objects
    active_objects = sim.get_existing_object_ids()
    for id in active_objects:
        sim.remove_object(id)

    # then add new objects
    for i in range(num_obj):
        obj_template_id = random.randint(0, sim.get_physics_object_library_size())
        add_and_place_object_from_navmesh(obj_template_id, sim)


def add_and_place_object_from_navmesh(obj_template_id, sim):
    obj_id = sim.add_object(obj_template_id)
    sim.set_object_motion_type(MotionType.KINEMATIC, obj_id)
    print("motion type: " + str(sim.get_object_motion_type(obj_id)))
    bb = sim.get_object_local_bb(obj_id)
    # print(bb)
    # print(bb.size())

    tries = 0
    placed_well = False
    while not placed_well and tries < 99:
        rad = 0
        position = np.zeros(3)
        while (rad < bb.size()[0] / 2.0 or rad < bb.size()[2] / 2.0) and tries < 99:
            position = sim.pathfinder.get_random_navigable_point()
            rad = sim.pathfinder.distance_to_closest_obstacle(position)
            # print("New Position: " + str(position) + ", rad: "+ str(rad) + " vs. " + str(bb.size()))
            tries += 1

        offset = np.array([0.0, bb.size()[1] / 2.0, 0.0])

        sim.set_translation(position + offset, obj_id)

        R = habitat_sim.utils.quat_from_angle_axis(
            random.random() * math.pi * 2, np.array([0.0, 1.0, 0.0])
        )
        sim.set_rotation(habitat_sim.utils.common.quat_to_magnum(R), obj_id)

        placed_well = not sim.contact_test(obj_id)
        # print("placed_well: " + str(placed_well))

    if not placed_well:
        sim.remove_object(obj_id)
        # print("not placed well: removed")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--scene",
        type=str,
        default="data/scene_datasets/habitat-test-scenes/van-gogh-room.glb",
    )
    parser.add_argument("--save_png", action="store_true")
    parser.add_argument("--render_video", action="store_true")
    parser.add_argument("--max_frames", type=int, default=100)
    args = parser.parse_args()

    os.system("rm -r clutter_nav_output")
    os.system("mkdir clutter_nav_output")

    # grab an esay format for storing sim settings
    settings = default_sim_settings.copy()
    settings["save_png"] = args.save_png
    settings["seed"] = 5
    settings["max_frames"] = args.max_frames
    settings["sensor_height"] = 0.6  # locobot camera height

    sim_cfg = hsim.SimulatorConfiguration()
    sim_cfg.enable_physics = True

    # sim_cfg.scene.id = "/Users/alexclegg/downloads/matterpak_Va2ovXYc6it/empty_room.glb"
    # sim_cfg.scene.id = "data/scene_datasets/habitat-test-scenes/skokloster-castle.glb"
    # sim_cfg.scene.id = "/private/home/akadian/sim2real/sim2real/data/scene_datasets/coda/empty_room.glb"
    sim_cfg.scene.id = args.scene

    sim_cfg.gpu_device_id = 0

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
    agent_cfg.action_space = {
        "move_forward": habitat_sim.agent.ActionSpec(
            "move_forward", habitat_sim.agent.ActuationSpec(amount=0.25)
        ),
        "turn_right": habitat_sim.agent.ActionSpec(
            "turn_right", habitat_sim.agent.ActuationSpec(amount=10.0)
        ),
    }

    # construct the simulator
    _cfg = habitat_sim.Configuration(sim_cfg, [agent_cfg])
    sim = habitat_sim.Simulator(_cfg)

    if not sim.pathfinder.is_loaded:
        sim.recompute_navmesh(agent_radius=agent_cfg.radius)

    random.seed(settings["seed"])
    sim.seed(settings["seed"])

    total_frames = 0
    for episode in range(2):
        # construct the scene
        print("generating clutter")
        gen_clutter(sim, 12)
        print("clutter generation successful")

        # rebuild the navmesh
        print("recompute the navmesh")
        sim.recompute_navmesh(agent_radius=agent_cfg.radius)
        print("recompute successful")
        # import pdb; pdb.set_trace()

        # place the agent
        agent = sim.initialize_agent(settings["default_agent"])

        # start_state = agent.get_state()
        # start_state.position = sim.pathfinder.get_random_navigable_point()
        # agent.set_state(start_state)

        # force starting position on first floor (try 100 samples)
        num_start_tries = 0
        # start_state.position = sim.pathfinder.get_random_navigable_point()
        while agent.get_state().position[1] > 0.5 and num_start_tries < 100:
            # start_state.position = sim.pathfinder.get_random_navigable_point()
            sim.initialize_agent(settings["default_agent"])
            num_start_tries += 1
        # agent.set_state(start_state)

        action_names = list(_cfg.agents[settings["default_agent"]].action_space.keys())

        # ------------------------
        # do some timesteps
        # ------------------------
        episode_frames = 0

        # really just using the utilities here
        milestone = 0
        while episode_frames < settings["max_frames"]:
            # if episode_frames > milestone / 10 * settings["max_frames"]:
            print(
                "computing frame "
                + str(episode_frames)
                + " of "
                + str(settings["max_frames"])
            )
            milestone += 1

            action = random.choice(action_names)
            observations = sim.step(action)
            if settings["save_png"]:
                if settings["color_sensor"]:
                    save_color_observation(observations, total_frames)
                if settings["depth_sensor"]:
                    save_depth_observation(observations, total_frames)

            state = sim.last_state()
            # print(state.position)
            # print(state.sensor_states["color_sensor"])

            total_frames += 1
            episode_frames += 1

    # create a video with ffmpeg
    if args.render_video:
        os.system(
            "ffmpeg -r 20 -f image2 -s 1920x1080 -i clutter_nav_output/test.rgba.%05d.png -crf 15 -pix_fmt yuv420p clutter_nav_output/test.mp4"
        )
        os.system("open clutter_nav_output/test.mp4")

    print("clutter_nav successful")


if __name__ == "__main__":
    main()
