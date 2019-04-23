# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.


import os
import random
import sys
import time

import numpy as np
from enum import Enum
from PIL import Image

import habitat_sim
import habitat_sim.agent
import habitat_sim.bindings as hsim

from habitat_sim.utils import d3_40_colors_rgb
from settings import default_sim_settings, make_cfg


class DemoRunnerType(Enum):
    BENCHMARK = 1
    EXAMPLE = 2


class DemoRunner:
    def __init__(self, sim_settings, simulator_demo_type):
        if simulator_demo_type == DemoRunnerType.EXAMPLE:
            self.set_sim_settings(sim_settings)

    def set_sim_settings(self, sim_settings):
        self._sim_settings = sim_settings.copy()

    def save_color_observation(self, obs, total_frames):
        color_obs = obs["color_sensor"]
        color_img = Image.fromarray(color_obs, mode="RGBA")
        color_img.save("test.rgba.%05d.png" % total_frames)

    def save_semantic_observation(self, obs, total_frames):
        semantic_obs = obs["semantic_sensor"]
        semantic_img = Image.new("P", (semantic_obs.shape[1], semantic_obs.shape[0]))
        semantic_img.putpalette(d3_40_colors_rgb.flatten())
        semantic_img.putdata((semantic_obs.flatten() % 40).astype(np.uint8))
        semantic_img.save("test.sem.%05d.png" % total_frames)

    def save_depth_observation(self, obs, total_frames):
        if self._sim_settings["depth_sensor"]:
            depth_obs = obs["depth_sensor"]
            depth_img = Image.fromarray(
                (depth_obs / 10 * 255).astype(np.uint8), mode="L"
            )
            depth_img.save("test.depth.%05d.png" % total_frames)

    def output_semantic_mask_stats(self, obs, total_frames):
        semantic_obs = obs["semantic_sensor"]
        counts = np.bincount(semantic_obs.flatten())
        total_count = np.sum(counts)
        print(f"Pixel statistics for frame {total_frames}")
        for object_i, count in enumerate(counts):
            sem_obj = self._sim.semantic_scene.objects[object_i]
            cat = sem_obj.category.name()
            pixel_ratio = count / total_count
            if pixel_ratio > 0.01:
                print(f"obj_id:{sem_obj.id},category:{cat},pixel_ratio:{pixel_ratio}")

    def init_agent_state(self, agent_id):
        # initialize the agent at a random start state
        agent = self._sim.initialize_agent(agent_id)
        start_state = agent.get_state()

        # force starting position on first floor (try 100 samples)
        num_start_tries = 0
        while start_state.position[1] > 0.5 and num_start_tries < 100:
            start_state.position = self._sim.pathfinder.get_random_navigable_point()
            num_start_tries += 1
        agent.set_state(start_state)

        if not self._sim_settings["silent"]:
            print(
                "start_state.position\t",
                start_state.position,
                "start_state.rotation\t",
                start_state.rotation,
            )

        return start_state

    def compute_shortest_path(self, start_pos, end_pos):
        self._shortest_path.requested_start = start_pos
        self._shortest_path.requested_end = end_pos
        self._sim.pathfinder.find_path(self._shortest_path)
        print("shortest_path.geodesic_distance", self._shortest_path.geodesic_distance)

    def do_time_steps(self):
        total_frames = 0
        start_time = time.time()
        action_names = list(
            self._cfg.agents[self._sim_settings["default_agent"]].action_space.keys()
        )

        while total_frames < self._sim_settings["max_frames"]:
            action = random.choice(action_names)
            if not self._sim_settings["silent"]:
                print("action", action)
            observations = self._sim.step(action)

            if self._sim_settings["save_png"]:
                if self._sim_settings["color_sensor"]:
                    self.save_color_observation(observations, total_frames)
                if self._sim_settings["depth_sensor"]:
                    self.save_depth_observation(observations, total_frames)
                if self._sim_settings["semantic_sensor"]:
                    self.save_semantic_observation(observations, total_frames)

            state = self._sim.last_state()

            if not self._sim_settings["silent"]:
                print("position\t", state.position, "\t", "rotation\t", state.rotation)

            if self._sim_settings["compute_shortest_path"]:
                self.compute_shortest_path(
                    state.position, self._sim_settings["goal_position"]
                )

            if self._sim_settings["compute_action_shortest_path"]:
                self._action_shortest_path.requested_start.position = state.position
                self._action_shortest_path.requested_start.rotation = state.rotation
                self._action_pathfinder.find_path(self._action_shortest_path)
                print(
                    "len(action_shortest_path.actions)",
                    len(self._action_shortest_path.actions),
                )

            if (
                self._sim_settings["semantic_sensor"]
                and self._sim_settings["print_semantic_mask_stats"]
            ):
                self.output_semantic_mask_stats(observations, total_frames)

            total_frames += 1

        end_time = time.time()
        perf = {}
        perf["total_time"] = end_time - start_time
        perf["fps"] = total_frames / perf["total_time"]

        return perf

    def print_semantic_scene(self):
        if self._sim_settings["print_semantic_scene"]:
            scene = self._sim.semantic_scene
            print(f"House center:{scene.aabb.center} dims:{scene.aabb.sizes}")
            for level in scene.levels:
                print(
                    f"Level id:{level.id}, center:{level.aabb.center},"
                    f" dims:{level.aabb.sizes}"
                )
                for region in level.regions:
                    print(
                        f"Region id:{region.id}, category:{region.category.name()},"
                        f" center:{region.aabb.center}, dims:{region.aabb.sizes}"
                    )
                    for obj in region.objects:
                        print(
                            f"Object id:{obj.id}, category:{obj.category.name()},"
                            f" center:{obj.aabb.center}, dims:{obj.aabb.sizes}"
                        )
            input("Press Enter to continue...")

    def init_common(self):
        self._cfg = make_cfg(self._sim_settings)
        self._sim = habitat_sim.Simulator(self._cfg)

        random.seed(self._sim_settings["seed"])
        self._sim.seed(self._sim_settings["seed"])

        # initialize the agent at a random start state
        start_state = self.init_agent_state(self._sim_settings["default_agent"])

        return start_state

    def benchmark(self, settings):
        self.set_sim_settings(settings)
        self.init_common()

        perf = self.do_time_steps()

        self._sim.close()
        del self._sim

        return perf

    def example(self):
        start_state = self.init_common()

        # initialize and compute shortest path to goal
        self._shortest_path = hsim.ShortestPath()
        self.compute_shortest_path(
            start_state.position, self._sim_settings["goal_position"]
        )

        # set the goal headings, and compute action shortest path
        if self._sim_settings["compute_action_shortest_path"]:
            agent_id = self._sim_settings["default_agent"]
            goal_headings = self._sim_settings["goal_headings"]
            self._action_pathfinder = self._sim.make_action_pathfinder(agent_id)

            self._action_shortest_path = hsim.MultiGoalActionSpaceShortestPath()
            self._action_shortest_path.requested_start.position = start_state.position
            self._action_shortest_path.requested_start.rotation = start_state.rotation

            # explicitly reset the start position
            self._shortest_path.requested_start = start_state.position

            # initialize the requested ends when computing the action shortest path
            next_goal_idx = 0
            while next_goal_idx < len(goal_headings):
                sampled_pos = self._sim.pathfinder.get_random_navigable_point()
                self._shortest_path.requested_end = sampled_pos
                if (
                    self._sim.pathfinder.find_path(self._shortest_path)
                    and self._shortest_path.geodesic_distance < 5.0
                    and self._shortest_path.geodesic_distance > 2.5
                ):
                    self._action_shortest_path.requested_ends.append(
                        hsim.ActionSpacePathLocation(
                            sampled_pos, goal_headings[next_goal_idx]
                        )
                    )
                    next_goal_idx += 1

            self._shortest_path.requested_end = self._sim_settings["goal_position"]
            self._sim.pathfinder.find_path(self._shortest_path)

            self._action_pathfinder.find_path(self._action_shortest_path)
            print(
                "len(action_shortest_path.actions)",
                len(self._action_shortest_path.actions),
            )

        # print semantic scene
        self.print_semantic_scene()

        perf = self.do_time_steps()
        self._sim.close()
        del self._sim

        return perf
