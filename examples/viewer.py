# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import ctypes
import math
import os
import random
import sys
import time
from enum import Enum
from typing import Any, Callable, Dict, List, Optional, Tuple

flags = sys.getdlopenflags()
sys.setdlopenflags(flags | ctypes.RTLD_GLOBAL)

import magnum as mn
import numpy as np
from magnum.platform.glfw import Application

import habitat_sim
from examples.settings import default_sim_settings, make_cfg
from habitat_sim import physics
from habitat_sim.logging import LoggingContext, logger
from habitat_sim.utils.common import quat_from_angle_axis


class SampleVolume:
    def __init__(
        self,
        volume: mn.Range3D,
        translation: mn.Vector3,
        rotation: mn.Quaternion,
        name: str,
    ) -> None:
        self.volume = volume
        self.translation = translation
        self.rotation = rotation
        self.name = name
        self.scalar_volume = volume.size_x() * volume.size_y() * volume.size_z()

    def sample_uniform_local(self) -> mn.Vector3:
        """
        Sample a uniform random point within Receptacle in local space.
        """
        return mn.Vector3(np.random.uniform(self.volume.min, self.volume.max))

    def sample_uniform_global(self) -> mn.Vector3:
        """
        Sample a uniform random point in the local Receptacle volume and then transform it into global space.
        """
        local_sample = self.sample_uniform_local()
        global_sample = self.rotation.transform_vector(local_sample)
        global_sample = self.translation + global_sample
        return global_sample

    def contains_point(self, point) -> bool:
        """
        Returns whether or not a point is contained in the volume
        """
        local_point = self.rotation.inverted().transform_vector(
            point - self.translation
        )
        return self.volume.contains(local_point)

    def debug_draw(self, sim: habitat_sim.Simulator, color: mn.Color4 = None) -> None:
        # draw the volume with a debug line render box
        transform_matrix = mn.Matrix4.from_(self.rotation.to_matrix(), self.translation)
        debug_line_renderer = sim.get_debug_line_render()
        debug_line_renderer.push_transform(transform_matrix)
        if color is None:
            color = mn.Color4.magenta()
        debug_line_renderer.draw_box(self.volume.min, self.volume.max, color)
        debug_line_renderer.pop_transform()
        # debug_line_renderer.draw_circle(self.translation, 0.25, color)


def match_volumes(
    sample_volumes: Dict[str, Dict[str, SampleVolume]]
) -> Dict[str, List[str]]:
    # match single agent spawns to a list of target spawns
    agents = sample_volumes["agent_spawns"]
    targets = sample_volumes["target_volumes"]
    matches = {}
    for agent in agents:
        matches[agent] = []
    for target in targets:
        obj = str(target)
        first_underscore = obj.find("_") + 1
        agent = obj[: obj.find("_", first_underscore)]
        matches[agent].append(target)
    return matches


def get_region_volumes(
    sample_volumes: Dict[str, Dict[str, SampleVolume]]
) -> Dict[str, List[str]]:
    # split spawns in to agent and target sets by room tag
    region_splits = {}
    for volume_type in sample_volumes.keys():
        for volume_name, volume in sample_volumes[volume_type].items():
            region_tag = volume_name.split("_")[0]
            # print(f"tag = {volume_name} - region = {region_tag}")
            if region_tag not in region_splits:
                region_splits[region_tag] = {
                    "agent_spawns": {},
                    "target_volumes": {},
                }
            region_splits[region_tag][volume_type][volume_name] = volume
    print(region_splits)
    return region_splits


def read_sample_volume_metadata(metadata_file) -> Dict[str, Dict[str, SampleVolume]]:
    # read the target and agent sampling metadata from json back into lists of SampleVolumes
    import json

    with open(metadata_file, "r") as infile:
        metadata = json.load(infile)
        sample_volumes = {"agent_spawns": {}, "target_volumes": {}}
        for key in sample_volumes:
            for item in metadata[key]:
                sample_volumes[key][item["name"]] = SampleVolume(
                    volume=mn.Range3D.from_center(
                        mn.Vector3(), mn.Vector3(item["size"]) * 0.5
                    ),
                    translation=mn.Vector3(item["translation"]),
                    rotation=mn.Quaternion(
                        mn.Vector3(item["rotation"][1:]), item["rotation"][0]
                    ),
                    name=item["name"],
                )

        print(sample_volumes)
        return sample_volumes


def add_sphere_object(
    sim: habitat_sim.Simulator, radius: float, pos: mn.Vector3
) -> habitat_sim.physics.ManagedRigidObject:
    """
    Add a sphere to the world at a global position.
    Returns the new object.
    """
    obj_attr_mgr = sim.get_object_template_manager()
    sphere_template = obj_attr_mgr.get_template_by_handle(
        obj_attr_mgr.get_template_handles("icosphereSolid")[0]
    )
    sphere_template.scale = mn.Vector3(radius)
    obj_attr_mgr.register_template(sphere_template, "my_sphere")
    new_object = sim.get_rigid_object_manager().add_object_by_template_handle(
        "my_sphere"
    )
    new_object.motion_type = habitat_sim.physics.MotionType.DYNAMIC
    new_object.translation = pos
    return new_object


def get_volume_weighted_sample(sample_volumes: Dict[str, SampleVolume]) -> SampleVolume:
    # pick a random SampleVolume weighted by volume
    volume_cache = []
    total_volume = 0
    # print("Volume totals: ")
    for v in sample_volumes.values():
        total_volume += v.scalar_volume
        volume_cache.append(total_volume)
        # print(f"    {total_volume} - {v.name}")
    rand_value = random.random() * total_volume
    # print(f" rand = {rand_value}")
    for ix, volume in enumerate(volume_cache):
        if volume >= rand_value:
            # print(f"  - result = {list(sample_volumes.values())[ix].name}")
            return list(sample_volumes.values())[ix]
    raise AssertionError


def get_random_target_sample(
    sim: habitat_sim.Simulator,
    sample_volumes: Dict[str, SampleVolume],
    radius=0.05,
    max_attempts=1000,
) -> Tuple[mn.Vector3, str]:
    # get a random spherical sample point with specified radius from a random SampleVolume which is NOT intersecting the scene geometry

    # NOTE: naive uniform sample
    # sample_volume = list(sample_volumes.values())[
    #    random.randint(0, len(sample_volumes) - 1)
    # ]

    sample_volume = get_volume_weighted_sample(sample_volumes)
    sample_sphere = add_sphere_object(
        sim, radius, sample_volume.sample_uniform_global()
    )
    # rejection sampling
    attempts = 0
    while sample_sphere.contact_test() and attempts < max_attempts:
        sample_sphere.translation = sample_volume.sample_uniform_global()
        attempts += 1
    if attempts == max_attempts:
        logger.warn(
            f"Failed to sample a point in '{sample_volume.name}' in {max_attempts} tries."
        )
        return None
    sample_position = sample_sphere.translation
    sim.get_rigid_object_manager().remove_object_by_id(sample_sphere.object_id)
    return (sample_position, sample_volume.name)


def set_agent_state(
    sim: habitat_sim.Simulator,
    point: mn.Vector3,
    face_target: Optional[mn.Vector3] = None,
):
    # set agent position
    agent = sim.get_agent(0)
    sampled_state = habitat_sim.agent.agent.AgentState()
    sampled_state.position = sim.pathfinder.snap_point(point)

    if face_target is not None:
        # orient the agent toward the point
        agent_to_obj = face_target - sampled_state.position
        agent_local_forward = np.array([0, 0, -1.0])
        flat_to_obj = np.array([agent_to_obj[0], 0.0, agent_to_obj[2]])
        # unit y normal plane for rotation
        det = (
            flat_to_obj[0] * agent_local_forward[2]
            - agent_local_forward[0] * flat_to_obj[2]
        )
        turn_angle = math.atan2(det, np.dot(agent_local_forward, flat_to_obj))
        sampled_state.rotation = quat_from_angle_axis(turn_angle, np.array([0, 1.0, 0]))
    else:
        # set a random agent orientation
        sampled_state.rotation = quat_from_angle_axis(
            sim.random.uniform_float(0, 2.0 * np.pi), np.array([0, 1, 0])
        )

    # set the state to the agent
    agent.set_state(sampled_state, is_initial=False)


def get_random_agent_sample_from_volume(
    sim: habitat_sim.Simulator,
    sample_volumes: Dict[str, SampleVolume],
    max_attempts=1000,
) -> mn.Vector3:
    # get a random agent spawn location from a random SampleVolume
    sample_volume = get_volume_weighted_sample(sample_volumes)

    attempts = 0
    snap_point = None
    found_sample = False
    while not found_sample and attempts < max_attempts:
        snap_point = sim.pathfinder.snap_point(sample_volume.sample_uniform_global())
        if not np.isnan(snap_point).any() and sample_volume.contains_point(snap_point):
            found_sample = True
            break
        attempts += 1
    if attempts == max_attempts:
        logger.warn(
            f"Failed to sample agent point in '{sample_volume.name}' in {max_attempts} tries."
        )
        return None
    return snap_point


def draw_debug_sphere(
    sim: habitat_sim.Simulator, radius: float, position: mn.Vector3, color: mn.Color4
):
    debug_line_render = sim.get_debug_line_render()
    rings = 5
    for axis in [mn.Vector3(1, 0, 0), mn.Vector3(0, 0, 1)]:
        for ring in range(rings):
            matrix = mn.Matrix4.rotation(mn.Rad(math.pi * ring / rings), axis)
            matrix.translation = position
            debug_line_render.push_transform(matrix)
            debug_line_render.draw_circle(mn.Vector3(), radius, color)
            debug_line_render.pop_transform()


# YAML config writing utils
def initialize_data():
    data = {}
    data["PATHS"] = {
        "WEBAPP_FOLDER": "task/server_files/habitat_vr_app",
        "WEBAPP_DATA": "{WEBAPP_FOLDER}/data",
        "WEBAPP_CONFIG": "{WEBAPP_DATA}/config",
        "TASK_CONFIG_PATH": "{WEBAPP_CONFIG}/taskData.json",
        "INSTANCE_CONFIG_PATH": "{WEBAPP_CONFIG}/instanceData.json",
        "DATA_CSV_PATH": "{WEBAPP_CONFIG}/data.csv",
        "OUTPUT_FOLDER": "~/floorplanner-pilot-dataset",
    }

    data["TASK_1"] = {
        "METADATA": {
            "NAME": "object_reaching",
            "DESCRIPTION": "Reach for the specified object",
        },
    }
    return data


def write_yaml(yaml_data, data_path="volume_data.tyaml"):
    import yaml

    with open(data_path, "w") as outfile:
        yaml.safe_dump(
            yaml_data,
            outfile,
            default_flow_style=False,
            sort_keys=False,
            allow_unicode=False,
        )
    print(f"Wrote volume sample dataset to {data_path}")


def write_dataset_to_yaml(dataset, data_path="volume_data.tyaml"):
    yaml_data = initialize_data()
    for index, datapoint in enumerate(dataset):
        yaml_data["TASK_1"][f"INSTANCE_{index}"] = {
            "COMMENT": f"{datapoint['object_label']}_{datapoint['start_region']}",
            "SUBJECT": 1,
            "DATASET_NAME": "floorplanner_chunks",
            "DATASET_PATH": "data/floorplanner_chunks/floorplanner.scene_dataset_config.json",
            "SCENE": f"102815835-{datapoint['room']}",
            "OBJECT_LABEL": datapoint["object_label"],
            "OBJECT_BBOX": [1.0, 2.0, 3.0],
            "START_POSITION": list(datapoint["start_position"]),
            "GOAL_POSITION": list(datapoint["goal_position"]),
            "START_REGION": datapoint["start_region"],
        }
    write_yaml(yaml_data, data_path)


def generate_dataset_per_room(sim, region_volumes):
    # generate a full dataset of point and agent starting locations
    num_samples_per_region = 100
    included_regions = [
        "kitchen1",
        "kitchen2",
        "dining1",
    ]
    dataset = []
    for region in included_regions:
        # scrape all numbers from the region tag:
        room_name = "".join(i for i in region if not i.isdigit())
        for _sample in range(num_samples_per_region):
            # generate an agent location
            agent_start = get_random_agent_sample_from_volume(
                sim, region_volumes[region]["agent_spawns"]
            )
            # generate goal
            target_point, target_name = get_random_target_sample(
                sim, region_volumes[region]["target_volumes"]
            )
            # cache the results
            dataset.append(
                {
                    "room": room_name,
                    "object_label": target_name,
                    "goal_position": target_point,
                    "start_position": agent_start,
                    "start_region": region,
                }
            )
    return dataset


class HabitatSimInteractiveViewer(Application):
    def __init__(self, sim_settings: Dict[str, Any]) -> None:
        configuration = self.Configuration()
        configuration.title = "Habitat Sim Interactive Viewer"
        Application.__init__(self, configuration)
        self.sim_settings: Dict[str:Any] = sim_settings
        self.fps: float = 60.0
        # draw Bullet debug line visualizations (e.g. collision meshes)
        self.debug_bullet_draw = False
        # draw active contact point debug line visualizations
        self.contact_debug_draw = False
        # cache most recently loaded URDF file for quick-reload
        self.cached_urdf = ""

        # set proper viewport size
        self.viewport_size: mn.Vector2i = mn.gl.default_framebuffer.viewport.size()
        self.sim_settings["width"] = self.viewport_size[0]
        self.sim_settings["height"] = self.viewport_size[1]

        # set up our movement map
        key = Application.KeyEvent.Key
        self.pressed = {
            key.UP: False,
            key.DOWN: False,
            key.LEFT: False,
            key.RIGHT: False,
            key.A: False,
            key.D: False,
            key.S: False,
            key.W: False,
            key.X: False,
            key.Z: False,
        }

        # set up our movement key bindings map
        key = Application.KeyEvent.Key
        self.key_to_action = {
            key.UP: "look_up",
            key.DOWN: "look_down",
            key.LEFT: "turn_left",
            key.RIGHT: "turn_right",
            key.A: "move_left",
            key.D: "move_right",
            key.S: "move_backward",
            key.W: "move_forward",
            key.X: "move_down",
            key.Z: "move_up",
        }

        # Cycle mouse utilities
        self.mouse_interaction = MouseMode.LOOK
        self.mouse_grabber: Optional[MouseGrabber] = None
        self.previous_mouse_point = None

        # toggle physics simulation on/off
        self.simulating = True

        # toggle a single simulation step at the next opportunity if not
        # simulating continuously.
        self.simulate_single_step = False

        # configure our simulator
        self.cfg: Optional[habitat_sim.simulator.Configuration] = None
        self.sim: Optional[habitat_sim.simulator.Simulator] = None
        self.reconfigure_sim()

        # compute NavMesh if not already loaded by the scene.
        if (
            not self.sim.pathfinder.is_loaded
            and self.cfg.sim_cfg.scene_id.lower() != "none"
        ):
            self.navmesh_config_and_recompute()

        self.time_since_last_simulation = 0.0
        LoggingContext.reinitialize_from_env()
        logger.setLevel("INFO")
        self.print_help_text()

        # construct the sample volumes
        self.sample_volumes = read_sample_volume_metadata(
            "/home/alexclegg/Documents/VR_mocap_resources/102815835.json"
        )
        # self.agent_target_matches = match_volumes(self.sample_volumes)
        self.region_volumes = get_region_volumes(self.sample_volumes)
        self.active_region = list(self.region_volumes.keys())[0]
        self.sample_timer = {
            "time_since_last_sample": 0.0,
            "sample_frequency": 1.0,  # seconds
        }
        self.recent_sample = get_random_target_sample(
            self.sim, self.sample_volumes["target_volumes"]
        )[0]
        self.sample_dataset = []
        self.draw_dataset_sample = None

    def draw_contact_debug(self):
        """
        This method is called to render a debug line overlay displaying active contact points and normals.
        Yellow lines show the contact distance along the normal and red lines show the contact normal at a fixed length.
        """
        yellow = mn.Color4.yellow()
        red = mn.Color4.red()
        cps = self.sim.get_physics_contact_points()
        self.sim.get_debug_line_render().set_line_width(1.5)
        camera_position = self.render_camera.render_camera.node.absolute_translation
        # only showing active contacts
        active_contacts = (x for x in cps if x.is_active)
        for cp in active_contacts:
            # red shows the contact distance
            self.sim.get_debug_line_render().draw_transformed_line(
                cp.position_on_b_in_ws,
                cp.position_on_b_in_ws
                + cp.contact_normal_on_b_in_ws * -cp.contact_distance,
                red,
            )
            # yellow shows the contact normal at a fixed length for visualization
            self.sim.get_debug_line_render().draw_transformed_line(
                cp.position_on_b_in_ws,
                # + cp.contact_normal_on_b_in_ws * cp.contact_distance,
                cp.position_on_b_in_ws + cp.contact_normal_on_b_in_ws * 0.1,
                yellow,
            )
            self.sim.get_debug_line_render().draw_circle(
                translation=cp.position_on_b_in_ws,
                radius=0.005,
                color=yellow,
                normal=camera_position - cp.position_on_b_in_ws,
            )

    def debug_draw(self):
        """
        Additional draw commands to be called during draw_event.
        """
        if self.debug_bullet_draw:
            render_cam = self.render_camera.render_camera
            proj_mat = render_cam.projection_matrix.__matmul__(render_cam.camera_matrix)
            self.sim.physics_debug_draw(proj_mat)
        if self.contact_debug_draw:
            self.draw_contact_debug()

        # draw the sample volumes
        for agent_volume in self.sample_volumes["agent_spawns"].values():
            # agent boxes in green
            agent_volume.debug_draw(sim=self.sim, color=mn.Color4.green())
        for target_volume in self.sample_volumes["target_volumes"].values():
            # target boxes in magenta
            target_volume.debug_draw(sim=self.sim, color=mn.Color4.magenta())
        # draw the sample
        if self.recent_sample is not None:
            draw_debug_sphere(self.sim, 0.05, self.recent_sample, mn.Color4.red())
            # draw a line from in front of the user to the sample
            camera_transform = (
                self.render_camera.render_camera.node.absolute_transformation()
            )
            self.sim.get_debug_line_render().draw_transformed_line(
                self.recent_sample,
                camera_transform.transform_point(mn.Vector3(0, 0, -0.5)),
                mn.Color4.red(),
            )

        # draw the sample dataset:
        camera_position = self.render_camera.render_camera.node.absolute_translation
        for data_point in self.sample_dataset:
            self.sim.get_debug_line_render().draw_circle(
                translation=data_point["goal_position"],
                radius=0.05,
                color=mn.Color4.red(),
                normal=camera_position - data_point["goal_position"],
            )
            self.sim.get_debug_line_render().draw_circle(
                translation=data_point["start_position"],
                radius=0.1,
                color=mn.Color4.blue(),
                normal=mn.Vector3(0, 1, 0),
            )
        if self.draw_dataset_sample is not None:
            self.sim.get_debug_line_render().draw_transformed_line(
                self.sample_dataset[self.draw_dataset_sample]["start_position"],
                self.sample_dataset[self.draw_dataset_sample]["goal_position"],
                mn.Color4.yellow(),
            )

    def draw_event(
        self,
        simulation_call: Optional[Callable] = None,
        global_call: Optional[Callable] = None,
        active_agent_id_and_sensor_name: Tuple[int, str] = (0, "color_sensor"),
    ) -> None:
        """
        Calls continuously to re-render frames and swap the two frame buffers
        at a fixed rate.
        """
        agent_acts_per_sec = self.fps

        mn.gl.default_framebuffer.clear(
            mn.gl.FramebufferClear.COLOR | mn.gl.FramebufferClear.DEPTH
        )

        # Agent actions should occur at a fixed rate per second
        self.time_since_last_simulation += Timer.prev_frame_duration
        num_agent_actions: int = self.time_since_last_simulation * agent_acts_per_sec
        self.move_and_look(int(num_agent_actions))

        # resampling logic for demo purposes
        # self.sample_timer["time_since_last_sample"] += Timer.prev_frame_duration
        # if (
        #    self.sample_timer["time_since_last_sample"]
        #    > self.sample_timer["sample_frequency"]
        # ):
        #    self.recent_sample = get_random_target_sample(
        #        self.sim, self.sample_volumes["target_volumes"]
        #    )[0]
        #    self.sample_timer["time_since_last_sample"] = math.fmod(
        #        self.time_since_last_simulation, self.sample_timer["sample_frequency"]
        #    )

        # Occasionally a frame will pass quicker than 1/60 seconds
        if self.time_since_last_simulation >= 1.0 / self.fps:
            if self.simulating or self.simulate_single_step:
                # step physics at a fixed rate
                # In the interest of frame rate, only a single step is taken,
                # even if time_since_last_simulation is quite large
                self.sim.step_world(1.0 / self.fps)
                self.simulate_single_step = False
                if simulation_call is not None:
                    simulation_call()
            if global_call is not None:
                global_call()

            # reset time_since_last_simulation, accounting for potential overflow
            self.time_since_last_simulation = math.fmod(
                self.time_since_last_simulation, 1.0 / self.fps
            )

        keys = active_agent_id_and_sensor_name

        self.sim._Simulator__sensors[keys[0]][keys[1]].draw_observation()
        agent = self.sim.get_agent(keys[0])
        self.render_camera = agent.scene_node.node_sensor_suite.get(keys[1])
        self.debug_draw()
        self.render_camera.render_target.blit_rgba_to_default()
        mn.gl.default_framebuffer.bind()

        self.swap_buffers()
        Timer.next_frame()
        self.redraw()

    def default_agent_config(self) -> habitat_sim.agent.AgentConfiguration:
        """
        Set up our own agent and agent controls
        """
        make_action_spec = habitat_sim.agent.ActionSpec
        make_actuation_spec = habitat_sim.agent.ActuationSpec
        MOVE, LOOK = 0.07, 1.5

        # all of our possible actions' names
        action_list = [
            "move_left",
            "turn_left",
            "move_right",
            "turn_right",
            "move_backward",
            "look_up",
            "move_forward",
            "look_down",
            "move_down",
            "move_up",
        ]

        action_space: Dict[str, habitat_sim.agent.ActionSpec] = {}

        # build our action space map
        for action in action_list:
            actuation_spec_amt = MOVE if "move" in action else LOOK
            action_spec = make_action_spec(
                action, make_actuation_spec(actuation_spec_amt)
            )
            action_space[action] = action_spec

        sensor_spec: List[habitat_sim.sensor.SensorSpec] = self.cfg.agents[
            self.agent_id
        ].sensor_specifications

        agent_config = habitat_sim.agent.AgentConfiguration(
            height=1.5,
            radius=0.1,
            sensor_specifications=sensor_spec,
            action_space=action_space,
            body_type="cylinder",
        )
        return agent_config

    def reconfigure_sim(self) -> None:
        """
        Utilizes the current `self.sim_settings` to configure and set up a new
        `habitat_sim.Simulator`, and then either starts a simulation instance, or replaces
        the current simulator instance, reloading the most recently loaded scene
        """
        # configure our sim_settings but then set the agent to our default
        self.cfg = make_cfg(self.sim_settings)
        self.agent_id: int = self.sim_settings["default_agent"]
        self.cfg.agents[self.agent_id] = self.default_agent_config()

        if self.sim_settings["stage_requires_lighting"]:
            logger.info("Setting synthetic lighting override for stage.")
            self.cfg.sim_cfg.override_scene_light_defaults = True
            self.cfg.sim_cfg.scene_light_setup = habitat_sim.gfx.DEFAULT_LIGHTING_KEY

        if self.sim is None:
            self.sim = habitat_sim.Simulator(self.cfg)

        else:  # edge case
            if self.sim.config.sim_cfg.scene_id == self.cfg.sim_cfg.scene_id:
                # we need to force a reset, so change the internal config scene name
                self.sim.config.sim_cfg.scene_id = "NONE"
            self.sim.reconfigure(self.cfg)
        # post reconfigure
        self.active_scene_graph = self.sim.get_active_scene_graph()
        self.default_agent = self.sim.get_agent(self.agent_id)
        self.agent_body_node = self.default_agent.scene_node
        self.render_camera = self.agent_body_node.node_sensor_suite.get("color_sensor")
        # set sim_settings scene name as actual loaded scene
        self.sim_settings["scene"] = self.sim.curr_scene_name

        Timer.start()
        self.step = -1

    def move_and_look(self, repetitions: int) -> None:
        """
        This method is called continuously with `self.draw_event` to monitor
        any changes in the movement keys map `Dict[KeyEvent.key, Bool]`.
        When a key in the map is set to `True` the corresponding action is taken.
        """
        # avoids unecessary updates to grabber's object position
        if repetitions == 0:
            return

        key = Application.KeyEvent.Key
        agent = self.sim.agents[self.agent_id]
        press: Dict[key.key, bool] = self.pressed
        act: Dict[key.key, str] = self.key_to_action

        action_queue: List[str] = [act[k] for k, v in press.items() if v]

        for _ in range(int(repetitions)):
            [agent.act(x) for x in action_queue]

        # update the grabber transform when our agent is moved
        if self.mouse_grabber is not None:
            # update location of grabbed object
            self.update_grab_position(self.previous_mouse_point)

    def invert_gravity(self) -> None:
        """
        Sets the gravity vector to the negative of it's previous value. This is
        a good method for testing simulation functionality.
        """
        gravity: mn.Vector3 = self.sim.get_gravity() * -1
        self.sim.set_gravity(gravity)

    def key_press_event(self, event: Application.KeyEvent) -> None:
        """
        Handles `Application.KeyEvent` on a key press by performing the corresponding functions.
        If the key pressed is part of the movement keys map `Dict[KeyEvent.key, Bool]`, then the
        key will be set to False for the next `self.move_and_look()` to update the current actions.
        """
        key = event.key
        pressed = Application.KeyEvent.Key
        mod = Application.InputEvent.Modifier

        shift_pressed = bool(event.modifiers & mod.SHIFT)
        alt_pressed = bool(event.modifiers & mod.ALT)
        # warning: ctrl doesn't always pass through with other key-presses

        if key == pressed.ESC:
            event.accepted = True
            self.exit_event(Application.ExitEvent)
            return

        elif key == pressed.H:
            self.print_help_text()

        elif key == pressed.TAB:
            # NOTE: (+ALT) - reconfigure without cycling scenes
            if not alt_pressed:
                # cycle the active scene from the set available in MetadataMediator
                inc = -1 if shift_pressed else 1
                scene_ids = self.sim.metadata_mediator.get_scene_handles()
                cur_scene_index = 0
                if self.sim_settings["scene"] not in scene_ids:
                    matching_scenes = [
                        (ix, x)
                        for ix, x in enumerate(scene_ids)
                        if self.sim_settings["scene"] in x
                    ]
                    if not matching_scenes:
                        logger.warning(
                            f"The current scene, '{self.sim_settings['scene']}', is not in the list, starting cycle at index 0."
                        )
                    else:
                        cur_scene_index = matching_scenes[0][0]
                else:
                    cur_scene_index = scene_ids.index(self.sim_settings["scene"])

                next_scene_index = min(
                    max(cur_scene_index + inc, 0), len(scene_ids) - 1
                )
                self.sim_settings["scene"] = scene_ids[next_scene_index]
            self.reconfigure_sim()
            logger.info(
                f"Reconfigured simulator for scene: {self.sim_settings['scene']}"
            )

        elif key == pressed.SPACE:
            if not self.sim.config.sim_cfg.enable_physics:
                logger.warn("Warning: physics was not enabled during setup")
            else:
                self.simulating = not self.simulating
                logger.info(f"Command: physics simulating set to {self.simulating}")

        elif key == pressed.PERIOD:
            if self.simulating:
                logger.warn("Warning: physic simulation already running")
            else:
                self.simulate_single_step = True
                logger.info("Command: physics step taken")

        elif key == pressed.COMMA:
            self.debug_bullet_draw = not self.debug_bullet_draw
            logger.info(f"Command: toggle Bullet debug draw: {self.debug_bullet_draw}")

        elif key == pressed.C:
            if shift_pressed:
                self.contact_debug_draw = not self.contact_debug_draw
                logger.info(
                    f"Command: toggle contact debug draw: {self.contact_debug_draw}"
                )
            else:
                # perform a discrete collision detection pass and enable contact debug drawing to visualize the results
                logger.info(
                    "Command: perform discrete collision detection and visualize active contacts."
                )
                self.sim.perform_discrete_collision_detection()
                self.contact_debug_draw = True
                # TODO: add a nice log message with concise contact pair naming.

        elif key == pressed.T:
            # load URDF
            fixed_base = alt_pressed
            urdf_file_path = ""
            if shift_pressed and self.cached_urdf:
                urdf_file_path = self.cached_urdf
            else:
                urdf_file_path = input("Load URDF: provide a URDF filepath:").strip()

            if not urdf_file_path:
                logger.warn("Load URDF: no input provided. Aborting.")
            elif not urdf_file_path.endswith((".URDF", ".urdf")):
                logger.warn("Load URDF: input is not a URDF. Aborting.")
            elif os.path.exists(urdf_file_path):
                self.cached_urdf = urdf_file_path
                aom = self.sim.get_articulated_object_manager()
                ao = aom.add_articulated_object_from_urdf(
                    urdf_file_path, fixed_base, 1.0, 1.0, True
                )
                ao.translation = self.agent_body_node.transformation.transform_point(
                    [0.0, 1.0, -1.5]
                )
            else:
                logger.warn("Load URDF: input file not found. Aborting.")

        elif key == pressed.M:
            self.cycle_mouse_mode()
            logger.info(f"Command: mouse mode set to {self.mouse_interaction}")

        elif key == pressed.V:
            self.invert_gravity()
            logger.info("Command: gravity inverted")

        elif key == pressed.N:
            # (default) - toggle navmesh visualization
            # NOTE: (+ALT) - re-sample the agent position on the NavMesh
            # NOTE: (+SHIFT) - re-compute the NavMesh
            if alt_pressed:
                logger.info("Command: resample agent state from navmesh")
                if self.sim.pathfinder.is_loaded:
                    new_agent_state = habitat_sim.AgentState()
                    new_agent_state.position = (
                        self.sim.pathfinder.get_random_navigable_point()
                    )
                    new_agent_state.rotation = quat_from_angle_axis(
                        self.sim.random.uniform_float(0, 2.0 * np.pi),
                        np.array([0, 1, 0]),
                    )
                    self.default_agent.set_state(new_agent_state)
                else:
                    logger.warning(
                        "NavMesh is not initialized. Cannot sample new agent state."
                    )
            elif shift_pressed:
                logger.info("Command: recompute navmesh")
                self.navmesh_config_and_recompute()
            else:
                if self.sim.pathfinder.is_loaded:
                    self.sim.navmesh_visualization = not self.sim.navmesh_visualization
                    logger.info("Command: toggle navmesh")
                else:
                    logger.warn("Warning: recompute navmesh first")

        elif key == pressed.Q:

            if alt_pressed:
                # sample a new region
                self.active_region = list(self.region_volumes.keys())[
                    random.randint(0, len(self.region_volumes) - 1)
                ]
                print(f"Active sampling region = {self.active_region}")
            if shift_pressed == alt_pressed:
                # both or neither
                target_volumes = self.sample_volumes["target_volumes"]
                target_volumes = self.region_volumes[self.active_region][
                    "target_volumes"
                ]
                # generate a new sample target point
                self.recent_sample = get_random_target_sample(self.sim, target_volumes)[
                    0
                ]
            if shift_pressed:
                # generate a new agent state
                agent_sample_volumes = self.sample_volumes["agent_spawns"]
                agent_sample_volumes = self.region_volumes[self.active_region][
                    "agent_spawns"
                ]
                set_agent_state(
                    self.sim,
                    get_random_agent_sample_from_volume(self.sim, agent_sample_volumes),
                    face_target=self.recent_sample,
                )

        elif key == pressed.E:
            if shift_pressed and alt_pressed:
                write_dataset_to_yaml(self.sample_dataset)
            elif shift_pressed:
                self.draw_dataset_sample = (self.draw_dataset_sample + 1) % len(
                    self.sample_dataset
                )
            elif alt_pressed:
                self.draw_dataset_sample = (self.draw_dataset_sample - 1) % len(
                    self.sample_dataset
                )
                if self.draw_dataset_sample < 0:
                    self.draw_dataset_sample = len(self.sample_dataset) - 1
            else:
                self.recent_sample = None
                self.sample_dataset = generate_dataset_per_room(
                    self.sim, self.region_volumes
                )
                self.draw_dataset_sample = 0

        # update map of moving/looking keys which are currently pressed
        if key in self.pressed:
            self.pressed[key] = True
        event.accepted = True
        self.redraw()

    def key_release_event(self, event: Application.KeyEvent) -> None:
        """
        Handles `Application.KeyEvent` on a key release. When a key is released, if it
        is part of the movement keys map `Dict[KeyEvent.key, Bool]`, then the key will
        be set to False for the next `self.move_and_look()` to update the current actions.
        """
        key = event.key

        # update map of moving/looking keys which are currently pressed
        if key in self.pressed:
            self.pressed[key] = False
        event.accepted = True
        self.redraw()

    def mouse_move_event(self, event: Application.MouseMoveEvent) -> None:
        """
        Handles `Application.MouseMoveEvent`. When in LOOK mode, enables the left
        mouse button to steer the agent's facing direction. When in GRAB mode,
        continues to update the grabber's object positiion with our agents position.
        """
        button = Application.MouseMoveEvent.Buttons
        # if interactive mode -> LOOK MODE
        if event.buttons == button.LEFT and self.mouse_interaction == MouseMode.LOOK:
            agent = self.sim.agents[self.agent_id]
            delta = self.get_mouse_position(event.relative_position) / 2
            action = habitat_sim.agent.ObjectControls()
            act_spec = habitat_sim.agent.ActuationSpec

            # left/right on agent scene node
            action(agent.scene_node, "turn_right", act_spec(delta.x))

            # up/down on cameras' scene nodes
            action = habitat_sim.agent.ObjectControls()
            sensors = list(self.agent_body_node.subtree_sensors.values())
            [action(s.object, "look_down", act_spec(delta.y), False) for s in sensors]

        # if interactive mode is TRUE -> GRAB MODE
        elif self.mouse_interaction == MouseMode.GRAB and self.mouse_grabber:
            # update location of grabbed object
            self.update_grab_position(self.get_mouse_position(event.position))

        self.previous_mouse_point = self.get_mouse_position(event.position)
        self.redraw()
        event.accepted = True

    def mouse_press_event(self, event: Application.MouseEvent) -> None:
        """
        Handles `Application.MouseEvent`. When in GRAB mode, click on
        objects to drag their position. (right-click for fixed constraints)
        """
        button = Application.MouseEvent.Button
        physics_enabled = self.sim.get_physics_simulation_library()

        # if interactive mode is True -> GRAB MODE
        if self.mouse_interaction == MouseMode.GRAB and physics_enabled:
            render_camera = self.render_camera.render_camera
            ray = render_camera.unproject(self.get_mouse_position(event.position))
            raycast_results = self.sim.cast_ray(ray=ray)

            if raycast_results.has_hits():
                hit_object, ao_link = -1, -1
                hit_info = raycast_results.hits[0]

                if hit_info.object_id >= 0:
                    # we hit an non-staged collision object
                    ro_mngr = self.sim.get_rigid_object_manager()
                    ao_mngr = self.sim.get_articulated_object_manager()
                    ao = ao_mngr.get_object_by_id(hit_info.object_id)
                    ro = ro_mngr.get_object_by_id(hit_info.object_id)

                    if ro:
                        # if grabbed an object
                        hit_object = hit_info.object_id
                        object_pivot = ro.transformation.inverted().transform_point(
                            hit_info.point
                        )
                        object_frame = ro.rotation.inverted()
                    elif ao:
                        # if grabbed the base link
                        hit_object = hit_info.object_id
                        object_pivot = ao.transformation.inverted().transform_point(
                            hit_info.point
                        )
                        object_frame = ao.rotation.inverted()
                    else:
                        for ao_handle in ao_mngr.get_objects_by_handle_substring():
                            ao = ao_mngr.get_object_by_handle(ao_handle)
                            link_to_obj_ids = ao.link_object_ids

                            if hit_info.object_id in link_to_obj_ids:
                                # if we got a link
                                ao_link = link_to_obj_ids[hit_info.object_id]
                                object_pivot = (
                                    ao.get_link_scene_node(ao_link)
                                    .transformation.inverted()
                                    .transform_point(hit_info.point)
                                )
                                object_frame = ao.get_link_scene_node(
                                    ao_link
                                ).rotation.inverted()
                                hit_object = ao.object_id
                                break
                    # done checking for AO

                    if hit_object >= 0:
                        node = self.agent_body_node
                        constraint_settings = physics.RigidConstraintSettings()

                        constraint_settings.object_id_a = hit_object
                        constraint_settings.link_id_a = ao_link
                        constraint_settings.pivot_a = object_pivot
                        constraint_settings.frame_a = (
                            object_frame.to_matrix() @ node.rotation.to_matrix()
                        )
                        constraint_settings.frame_b = node.rotation.to_matrix()
                        constraint_settings.pivot_b = hit_info.point

                        # by default use a point 2 point constraint
                        if event.button == button.RIGHT:
                            constraint_settings.constraint_type = (
                                physics.RigidConstraintType.Fixed
                            )

                        grip_depth = (
                            hit_info.point - render_camera.node.absolute_translation
                        ).length()

                        self.mouse_grabber = MouseGrabber(
                            constraint_settings,
                            grip_depth,
                            self.sim,
                        )
                    else:
                        logger.warn("Oops, couldn't find the hit object. That's odd.")
                # end if didn't hit the scene
            # end has raycast hit
        # end has physics enabled

        self.previous_mouse_point = self.get_mouse_position(event.position)
        self.redraw()
        event.accepted = True

    def mouse_scroll_event(self, event: Application.MouseScrollEvent) -> None:
        """
        Handles `Application.MouseScrollEvent`. When in LOOK mode, enables camera
        zooming (fine-grained zoom using shift) When in GRAB mode, adjusts the depth
        of the grabber's object. (larger depth change rate using shift)
        """
        scroll_mod_val = (
            event.offset.y
            if abs(event.offset.y) > abs(event.offset.x)
            else event.offset.x
        )
        if not scroll_mod_val:
            return

        # use shift to scale action response
        shift_pressed = bool(event.modifiers & Application.InputEvent.Modifier.SHIFT)
        alt_pressed = bool(event.modifiers & Application.InputEvent.Modifier.ALT)
        ctrl_pressed = bool(event.modifiers & Application.InputEvent.Modifier.CTRL)

        # if interactive mode is False -> LOOK MODE
        if self.mouse_interaction == MouseMode.LOOK:
            # use shift for fine-grained zooming
            mod_val = 1.01 if shift_pressed else 1.1
            mod = mod_val if scroll_mod_val > 0 else 1.0 / mod_val
            cam = self.render_camera
            cam.zoom(mod)
            self.redraw()

        elif self.mouse_interaction == MouseMode.GRAB and self.mouse_grabber:
            # adjust the depth
            mod_val = 0.1 if shift_pressed else 0.01
            scroll_delta = scroll_mod_val * mod_val
            if alt_pressed or ctrl_pressed:
                # rotate the object's local constraint frame
                agent_t = self.agent_body_node.transformation_matrix()
                # ALT - yaw
                rotation_axis = agent_t.transform_vector(mn.Vector3(0, 1, 0))
                if alt_pressed and ctrl_pressed:
                    # ALT+CTRL - roll
                    rotation_axis = agent_t.transform_vector(mn.Vector3(0, 0, -1))
                elif ctrl_pressed:
                    # CTRL - pitch
                    rotation_axis = agent_t.transform_vector(mn.Vector3(1, 0, 0))
                self.mouse_grabber.rotate_local_frame_by_global_angle_axis(
                    rotation_axis, mn.Rad(scroll_delta)
                )
            else:
                # update location of grabbed object
                self.mouse_grabber.grip_depth += scroll_delta
                self.update_grab_position(self.get_mouse_position(event.position))
        self.redraw()
        event.accepted = True

    def mouse_release_event(self, event: Application.MouseEvent) -> None:
        """
        Release any existing constraints.
        """
        del self.mouse_grabber
        self.mouse_grabber = None
        event.accepted = True

    def update_grab_position(self, point: mn.Vector2i) -> None:
        """
        Accepts a point derived from a mouse click event and updates the
        transform of the mouse grabber.
        """
        # check mouse grabber
        if not self.mouse_grabber:
            return

        render_camera = self.render_camera.render_camera
        ray = render_camera.unproject(point)

        rotation: mn.Matrix3x3 = self.agent_body_node.rotation.to_matrix()
        translation: mn.Vector3 = (
            render_camera.node.absolute_translation
            + ray.direction * self.mouse_grabber.grip_depth
        )
        self.mouse_grabber.update_transform(mn.Matrix4.from_(rotation, translation))

    def get_mouse_position(self, mouse_event_position: mn.Vector2i) -> mn.Vector2i:
        """
        This function will get a screen-space mouse position appropriately
        scaled based on framebuffer size and window size.  Generally these would be
        the same value, but on certain HiDPI displays (Retina displays) they may be
        different.
        """
        scaling = mn.Vector2i(self.framebuffer_size) / mn.Vector2i(self.window_size)
        return mouse_event_position * scaling

    def cycle_mouse_mode(self) -> None:
        """
        This method defines how to cycle through the mouse mode.
        """
        if self.mouse_interaction == MouseMode.LOOK:
            self.mouse_interaction = MouseMode.GRAB
        elif self.mouse_interaction == MouseMode.GRAB:
            self.mouse_interaction = MouseMode.LOOK

    def navmesh_config_and_recompute(self) -> None:
        """
        This method is setup to be overridden in for setting config accessibility
        in inherited classes.
        """
        self.navmesh_settings = habitat_sim.NavMeshSettings()
        self.navmesh_settings.set_defaults()
        self.navmesh_settings.agent_height = self.cfg.agents[self.agent_id].height
        self.navmesh_settings.agent_radius = self.cfg.agents[self.agent_id].radius
        self.navmesh_settings.regionMinSize = 1

        self.sim.recompute_navmesh(
            self.sim.pathfinder,
            self.navmesh_settings,
            include_static_objects=True,
        )

    def exit_event(self, event: Application.ExitEvent):
        """
        Overrides exit_event to properly close the Simulator before exiting the
        application.
        """
        self.sim.close(destroy=True)
        event.accepted = True
        exit(0)

    def print_help_text(self) -> None:
        """
        Print the Key Command help text.
        """
        logger.info(
            """
=====================================================
Welcome to the Habitat-sim Python Viewer application!
=====================================================
Mouse Functions ('m' to toggle mode):
----------------
In LOOK mode (default):
    LEFT:
        Click and drag to rotate the agent and look up/down.
    WHEEL:
        Modify orthographic camera zoom/perspective camera FOV (+SHIFT for fine grained control)

In GRAB mode (with 'enable-physics'):
    LEFT:
        Click and drag to pickup and move an object with a point-to-point constraint (e.g. ball joint).
    RIGHT:
        Click and drag to pickup and move an object with a fixed frame constraint.
    WHEEL (with picked object):
        default - Pull gripped object closer or push it away.
        (+ALT) rotate object fixed constraint frame (yaw)
        (+CTRL) rotate object fixed constraint frame (pitch)
        (+ALT+CTRL) rotate object fixed constraint frame (roll)
        (+SHIFT) amplify scroll magnitude


Key Commands:
-------------
    esc:        Exit the application.
    'h':        Display this help message.
    'm':        Cycle mouse interaction modes.

    Agent Controls:
    'wasd':     Move the agent's body forward/backward and left/right.
    'zx':       Move the agent's body up/down.
    arrow keys: Turn the agent's body left/right and camera look up/down.

    Utilities:
    'r':        Reset the simulator with the most recently loaded scene.
    'n':        Show/hide NavMesh wireframe.
                (+SHIFT) Recompute NavMesh with default settings.
                (+ALT) Re-sample the agent(camera)'s position and orientation from the NavMesh.
    ',':        Render a Bullet collision shape debug wireframe overlay (white=active, green=sleeping, blue=wants sleeping, red=can't sleep).
    'c':        Run a discrete collision detection pass and render a debug wireframe overlay showing active contact points and normals (yellow=fixed length normals, red=collision distances).
                (+SHIFT) Toggle the contact point debug render overlay on/off.

    Object Interactions:
    SPACE:      Toggle physics simulation on/off.
    '.':        Take a single simulation step if not simulating continuously.
    'v':        (physics) Invert gravity.
    't':        Load URDF from filepath
                (+SHIFT) quick re-load the previously specified URDF
                (+ALT) load the URDF with fixed base
=====================================================
"""
        )


class MouseMode(Enum):
    LOOK = 0
    GRAB = 1
    MOTION = 2


class MouseGrabber:
    """
    Create a MouseGrabber from RigidConstraintSettings to manipulate objects.
    """

    def __init__(
        self,
        settings: physics.RigidConstraintSettings,
        grip_depth: float,
        sim: habitat_sim.simulator.Simulator,
    ) -> None:
        self.settings = settings
        self.simulator = sim

        # defines distance of the grip point from the camera for pivot updates
        self.grip_depth = grip_depth
        self.constraint_id = sim.create_rigid_constraint(settings)

    def __del__(self):
        self.remove_constraint()

    def remove_constraint(self) -> None:
        """
        Remove a rigid constraint by id.
        """
        self.simulator.remove_rigid_constraint(self.constraint_id)

    def updatePivot(self, pos: mn.Vector3) -> None:
        self.settings.pivot_b = pos
        self.simulator.update_rigid_constraint(self.constraint_id, self.settings)

    def update_frame(self, frame: mn.Matrix3x3) -> None:
        self.settings.frame_b = frame
        self.simulator.update_rigid_constraint(self.constraint_id, self.settings)

    def update_transform(self, transform: mn.Matrix4) -> None:
        self.settings.frame_b = transform.rotation()
        self.settings.pivot_b = transform.translation
        self.simulator.update_rigid_constraint(self.constraint_id, self.settings)

    def rotate_local_frame_by_global_angle_axis(
        self, axis: mn.Vector3, angle: mn.Rad
    ) -> None:
        """rotate the object's local constraint frame with a global angle axis input."""
        object_transform = mn.Matrix4()
        rom = self.simulator.get_rigid_object_manager()
        aom = self.simulator.get_articulated_object_manager()
        if rom.get_library_has_id(self.settings.object_id_a):
            object_transform = rom.get_object_by_id(
                self.settings.object_id_a
            ).transformation
        else:
            # must be an ao
            object_transform = (
                aom.get_object_by_id(self.settings.object_id_a)
                .get_link_scene_node(self.settings.link_id_a)
                .transformation
            )
        local_axis = object_transform.inverted().transform_vector(axis)
        R = mn.Matrix4.rotation(angle, local_axis.normalized())
        self.settings.frame_a = R.rotation().__matmul__(self.settings.frame_a)
        self.simulator.update_rigid_constraint(self.constraint_id, self.settings)


class Timer:
    """
    Timer class used to keep track of time between buffer swaps
    and guide the display frame rate.
    """

    start_time = 0.0
    prev_frame_time = 0.0
    prev_frame_duration = 0.0
    running = False

    @staticmethod
    def start() -> None:
        """
        Starts timer and resets previous frame time to the start time
        """
        Timer.running = True
        Timer.start_time = time.time()
        Timer.prev_frame_time = Timer.start_time
        Timer.prev_frame_duration = 0.0

    @staticmethod
    def stop() -> None:
        """
        Stops timer and erases any previous time data, reseting the timer
        """
        Timer.running = False
        Timer.start_time = 0.0
        Timer.prev_frame_time = 0.0
        Timer.prev_frame_duration = 0.0

    @staticmethod
    def next_frame() -> None:
        """
        Records previous frame duration and updates the previous frame timestamp
        to the current time. If the timer is not currently running, perform nothing.
        """
        if not Timer.running:
            return
        Timer.prev_frame_duration = time.time() - Timer.prev_frame_time
        Timer.prev_frame_time = time.time()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()

    # optional arguments
    parser.add_argument(
        "--scene",
        default="NONE",
        type=str,
        help='scene/stage file to load (default: "NONE")',
    )
    parser.add_argument(
        "--dataset",
        default="default",
        type=str,
        metavar="DATASET",
        help="dataset configuration file to use (default: default)",
    )
    parser.add_argument(
        "--disable_physics",
        action="store_true",
        help="disable physics simulation (default: False)",
    )
    parser.add_argument(
        "--stage_requires_lighting",
        action="store_true",
        help="Override configured lighting to use synthetic lighting for the stage.",
    )

    args = parser.parse_args()

    # Setting up sim_settings
    sim_settings: Dict[str, Any] = default_sim_settings
    sim_settings["scene"] = args.scene
    sim_settings["scene_dataset_config_file"] = args.dataset
    sim_settings["enable_physics"] = not args.disable_physics
    sim_settings["stage_requires_lighting"] = args.stage_requires_lighting

    HabitatSimInteractiveViewer(sim_settings).exec()
