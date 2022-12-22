# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import ctypes
import json
import math
import os
import sys
import time
from typing import Any, Callable, Dict, List, Optional, Tuple, Union

flags = sys.getdlopenflags()
sys.setdlopenflags(flags | ctypes.RTLD_GLOBAL)

import git
import magnum as mn
import numpy as np
from colorama import init
from magnum.platform.glfw import Application
from matplotlib import pyplot as plt
from PIL import Image
from qa_scene_settings import default_sim_settings, make_cfg
from qa_scene_utils import (
    ANSICodes,
    Timer,
    create_unique_filename,
    print_dataset_info,
    print_if_logging,
    section_divider_str,
)

import habitat_sim
from habitat_sim.utils import common as utils
from habitat_sim.utils import viz_utils as vut
from habitat_sim.utils.common import d3_40_colors_rgb

# clean up types with TypeVars
NavmeshMetrics = Dict[str, Union[int, float]]

# get the output directory and data path
repo = git.Repo(".", search_parent_directories=True)
dir_path = repo.working_tree_dir
data_path = os.path.join(dir_path, "data")
output_directory = "./tools/qa_scenes/qa_scenes_output/"  # @param {type:"string"}
output_path = os.path.join(dir_path, output_directory)
if not os.path.exists(output_path):
    os.mkdir(output_path)

silent: bool = False

# NOTE: change this to config file to test
qa_config_filename = "apt_1"

config_directory = "./tools/qa_scenes/configs/"
qa_config_extension = ".qa_scene_config.json"
qa_config_filepath = os.path.join(
    config_directory, qa_config_filename + qa_config_extension
)


class QASceneProcessingViewer(Application):
    def __init__(self, sim_settings: Dict[str, Any]) -> None:

        # Construct magnum.platform.glfw.Application
        configuration = self.Configuration()
        configuration.title = "QA Scene Processing Viewer"
        Application.__init__(self, configuration)

        # set proper viewport size
        self.sim_settings: Dict[str, Any] = sim_settings
        self.viewport_size: mn.Vector2i = mn.gl.default_framebuffer.viewport.size()
        self.sim_settings["width"] = self.viewport_size[0]
        self.sim_settings["height"] = self.viewport_size[1]

        # x_range: mn.Vector2i = mn.Vector2i(0, sim_settings["width"])
        # y_range: mn.Vector2i = mn.Vector2i(0, sim_settings["height"])
        # viewport_range: mn.Range2Di = mn.Range2Di(x_range, y_range)
        # self.viewport_size: mn.Vector2i = viewport_range.size()
        # self.framebuffer = mn.gl.Framebuffer(viewport_range)

        # variables that track sim time and render time
        self.total_frame_count: int = 0  # TODO debugging, remove

        self.fps: float = 60.0
        self.average_fps: float = self.fps
        self.prev_frame_duration: float = 0.0
        self.frame_duration_sum: float = 0.0

        self.physics_step_duration: float = 1.0 / self.fps
        self.prev_sim_duration: float = 0.0
        self.sim_duration_sum: float = 0.0
        self.avg_sim_duration: float = 0.0
        self.sim_steps_tracked: int = 0

        self.render_frames_to_track: int = 30
        self.prev_render_duration: float = 0.0
        self.render_duration_sum: float = 0.0
        self.avg_render_duration: float = 0.0
        self.render_frames_tracked: int = 0
        self.time_since_last_simulation = 0.0

        # toggle physics simulation on/off
        self.simulating = False

        # toggle a single simulation step at the next opportunity if not
        # simulating continuously.
        self.simulate_single_step = False

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
            key.Q: False,
            key.E: False,
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
            key.Q: "move_down",
            key.E: "move_up",
        }

        # Configure and construct simulator
        self.cfg: Optional[habitat_sim.Configuration] = None
        self.sim: Optional[habitat_sim.Simulator] = None
        self.reconfigure_sim(self.sim_settings)

    def reconfigure_sim(
        self,
        sim_settings: Dict[str, Any],
    ) -> None:

        self.sim_settings = sim_settings

        self.cfg = make_cfg_mm(self.sim_settings)
        self.agent_id: int = self.sim_settings["default_agent"]
        self.cfg.agents[self.agent_id] = self.default_agent_config()
        self.mm = self.cfg.metadata_mediator

        if self.sim_settings["stage_requires_lighting"]:
            print("Setting synthetic lighting override for stage.")
            self.cfg.sim_cfg.override_scene_light_defaults = True
            self.cfg.sim_cfg.scene_light_setup = habitat_sim.gfx.DEFAULT_LIGHTING_KEY

        if self.sim is None:
            self.sim = habitat_sim.Simulator(self.cfg)
        else:
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

        # # Reset agent transform
        # self.agent_body_node.translation = mn.Vector3(
        #     np.zeros(3)
        # )
        # self.agent_body_node.rotation = utils.quat_from_angle_axis(
        #     theta=0.0, axis=np.array([0, 1, 0])
        # )

        Timer.start()
        self.step = -1

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

        # self.framebuffer.clear(
        mn.gl.default_framebuffer.clear(
            mn.gl.FramebufferClear.COLOR | mn.gl.FramebufferClear.DEPTH
        )

        # Agent actions should occur at a fixed rate per second
        agent_acts_per_sec = self.fps
        self.time_since_last_simulation += Timer.prev_frame_duration
        num_agent_actions: int = self.time_since_last_simulation * agent_acts_per_sec
        self.move_and_look(int(num_agent_actions))

        self.prev_frame_duration = Timer.prev_frame_duration
        self.time_since_last_simulation += Timer.prev_frame_duration

        # Occasionally a frame will pass quicker than 1 / fps seconds
        if self.time_since_last_simulation >= self.physics_step_duration:
            if self.simulating or self.simulate_single_step:
                # step physics at a fixed rate
                # In the interest of frame rate, only a single step is taken,
                # even if time_since_last_simulation is quite large
                sim_start_time = time.time()
                self.sim.step_world(self.physics_step_duration)
                sim_end_time = time.time()
                self.prev_sim_duration = sim_end_time - sim_start_time
                self.sim_steps_tracked += 1

                self.simulate_single_step = False
                if simulation_call is not None:
                    simulation_call()
            else:
                self.sim_steps_tracked = 0  # TODO debugging, remove
                self.prev_sim_duration = 0.0
                self.sim_duration_sum = 0.0
                self.avg_sim_duration = 0.0

            if global_call is not None:
                global_call()

            # reset time_since_last_simulation, accounting for potential overflow
            self.time_since_last_simulation = math.fmod(
                self.time_since_last_simulation, self.physics_step_duration
            )

        # Get agent id, agent, and sensor uuid
        keys = active_agent_id_and_sensor_name
        agent_id = keys[0]
        agent = self.sim.get_agent(agent_id)
        self.sensor_uuid = keys[1]

        # gather and render sensor observation while timing it
        render_start_time = time.time()
        # observations = self.sim.get_sensor_observations(agent_id)
        self.sim.get_sensor_observations(agent_id)
        render_end_time = time.time()
        self.prev_render_duration = render_end_time - render_start_time

        # TODO write a good comment here, not sure what "blit" is
        self.render_camera = agent.scene_node.node_sensor_suite.get(self.sensor_uuid)
        self.render_camera.render_target.blit_rgba_to_default()
        mn.gl.default_framebuffer.bind()
        # self.framebuffer.bind()

        self.swap_buffers()
        Timer.next_frame()
        self.redraw()

    def key_press_event(self, event: Application.KeyEvent) -> None:
        """
        Handles `Application.KeyEvent` on a key press by performing the corresponding functions.
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

        # TODO make sure this works
        elif key == pressed.TAB:
            # NOTE: (+ALT) - reconfigure without cycling scenes
            if not alt_pressed:
                # cycle the active scene from the set available in MetadataMediator
                inc = -1 if shift_pressed else 1
                scene_ids = self.mm.get_scene_handles()
                cur_scene_index = 0
                if self.sim_settings["scene"] not in scene_ids:
                    matching_scenes = [
                        (ix, x)
                        for ix, x in enumerate(scene_ids)
                        if self.sim_settings["scene"] in x
                    ]
                    if not matching_scenes:
                        print(
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
            print(f"Reconfigured simulator for scene: {self.sim_settings['scene']}")

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
        ...

    def mouse_press_event(self, event: Application.MouseEvent) -> None:
        ...

    def mouse_scroll_event(self, event: Application.MouseScrollEvent) -> None:
        ...

    def mouse_release_event(self, event: Application.MouseEvent) -> None:
        ...

    def exit_event(self, event: Application.ExitEvent) -> None:
        """
        Overrides exit_event to properly close the Simulator before exiting the
        application.
        """
        self.sim.close(destroy=True)
        event.accepted = True
        exit(0)


# Change to do something like this maybe: https://stackoverflow.com/a/41432704
def display_sample(
    rgb_obs,
    semantic_obs: Optional[np.array] = None,
    depth_obs: Optional[np.array] = None,
    output_file=None,
):

    print_if_logging(silent, f"output file = {output_file}")
    rgb_img = Image.fromarray(rgb_obs, mode="RGBA")

    arr = [rgb_img]
    titles = ["rgb"]
    if semantic_obs is not None:
        semantic_img = Image.new("P", (semantic_obs.shape[1], semantic_obs.shape[0]))
        semantic_img.putpalette(d3_40_colors_rgb.flatten())
        semantic_img.putdata((semantic_obs.flatten() % 40).astype(np.uint8))
        semantic_img = semantic_img.convert("RGBA")
        arr.append(semantic_img)
        titles.append("semantic")

    if depth_obs is not None:
        depth_img = Image.fromarray((depth_obs / 10 * 255).astype(np.uint8), mode="L")
        arr.append(depth_img)
        titles.append("depth")

    plt.figure(figsize=(12, 8))
    for i, data in enumerate(arr):
        ax = plt.subplot(1, 3, i + 1)
        ax.axis("off")
        ax.set_title(titles[i])
        plt.imshow(data)

    if output_file is not None:
        print_if_logging(silent, "saving")
        plt.savefig(fname=output_file)
    # else:
    #    plt.show(block=False)
    ...


def pil_save_obs(output_file, rgb=None, semantic=None, depth=None):
    images = []
    if rgb is not None:
        images.append(vut.observation_to_image(rgb, "color"))
    if semantic is not None:
        images.append(vut.observation_to_image(semantic, "semantic"))
    if depth is not None:
        images.append(vut.observation_to_image(depth, "depth"))

    if len(images) == 0:
        print_if_logging(silent, "No images, aborting.")
        return

    concat_image_width = 0
    image_starts = []
    image_height = 0
    for image in images:
        image_starts.append(concat_image_width)
        concat_image_width += image.width
        image_height = max(image_height, image.height)

    contact_image = Image.new("RGB", (concat_image_width, image_height))

    for im_ix, image in enumerate(images):
        contact_image.paste(image, (image_starts[im_ix], 0))

    contact_image.save(output_file)


def make_cfg_mm(settings) -> habitat_sim.Configuration:
    """
    Create a Configuration with an attached MetadataMediator for shared dataset access and re-use
    """
    config = make_cfg(settings)

    # create and attach a MetadataMediator
    mm = habitat_sim.metadata.MetadataMediator(config.sim_cfg)

    return habitat_sim.Configuration(config.sim_cfg, config.agents, mm)


######################################################
# navmesh metrics code
######################################################


def collect_navmesh_metrics(sim) -> NavmeshMetrics:
    nav_metrics = {}
    if sim.pathfinder.is_loaded:
        nav_metrics["num_islands"] = sim.pathfinder.num_islands
        nav_metrics["island_areas"] = []
        largest_island_area = 0
        second_largest_island_area = 0
        for island_id in range(sim.pathfinder.num_islands):
            nav_metrics["island_areas"].append(sim.pathfinder.island_area(island_id))
            if nav_metrics["island_areas"][island_id] > largest_island_area:
                second_largest_island_area = largest_island_area
                largest_island_area = nav_metrics["island_areas"][island_id]
            elif nav_metrics["island_areas"][island_id] > second_largest_island_area:
                second_largest_island_area = nav_metrics["island_areas"][island_id]
        nav_metrics["island_ratio"] = (
            largest_island_area - second_largest_island_area
        ) / largest_island_area
        nav_metrics["total_area"] = sim.pathfinder.island_area(-1)
        nav_metrics["largest_island_coverage"] = (
            largest_island_area / nav_metrics["total_area"]
        )
    return nav_metrics


def aggregate_navmesh_metrics(
    all_scenes_navmesh_metrics: Dict[str, NavmeshMetrics], filename
) -> None:
    import csv

    # save a csv of navmesh metrics
    with open(filename, "w") as f:
        writer = csv.writer(f, quoting=csv.QUOTE_ALL)
        writer.writerow(["Scene", "num_islands", "area", "ratio", "coverage", "areas"])
        for scene_hash, metrics in all_scenes_navmesh_metrics.items():
            row_data = [
                scene_hash,
                metrics["num_islands"],
                metrics["total_area"],
                metrics["island_ratio"],
                metrics["largest_island_coverage"],
            ]
            row_data.extend(metrics["island_areas"])
            writer.writerow(row_data)


def save_navmesh_data(sim):
    os.makedirs("navmeshes/", exist_ok=True)
    if sim.pathfinder.is_loaded:
        for island in range(sim.pathfinder.num_islands):
            vert_data = sim.pathfinder.build_navmesh_vertices(island)
            index_data = sim.pathfinder.build_navmesh_vertex_indices(island)
            export_navmesh_data_to_obj(
                filename=f"navmeshes/{island}.obj",
                vertex_data=vert_data,
                index_data=index_data,
            )
    else:
        print_if_logging(silent, "Cannot save navmesh data, no pathfinder loaded")


def export_navmesh_data_to_obj(filename, vertex_data, index_data):
    with open(filename, "w") as f:
        file_data = ""
        for vert in vertex_data:
            file_data += (
                "v " + str(vert[0]) + " " + str(vert[1]) + " " + str(vert[2]) + "\n"
            )
        assert len(index_data) % 3 == 0, "must be triangles"
        for ix in range(int(len(index_data) / 3)):
            # NOTE: obj starts indexing at 1
            file_data += (
                "f "
                + str(index_data[ix * 3] + 1)
                + " "
                + str(index_data[ix * 3 + 1] + 1)
                + " "
                + str(index_data[ix * 3 + 2] + 1)
                + "\n"
            )
        f.write(file_data)


######################################################
# end navmesh metrics code
######################################################


def profile_scene(cfg):
    """
    Profile a scene's performance for rendering, collisions, physics, etc...
    """

    profile_info = {
        "avg_render_time": 0,  # averge time taken to render a frame
        "physics_realtime_ratio": 0,  # simulation : realtime ratio
    }

    with habitat_sim.Simulator(cfg) as sim:
        # profile rendering
        agent = sim.initialize_agent(sim_settings["default_agent"])
        agent_state = habitat_sim.AgentState()
        render_time_total = 0
        orientation_samples = 4
        for _orientation in range(orientation_samples):
            agent_state.rotation = utils.quat_from_angle_axis(
                theta=_orientation * (math.pi / 2.0), axis=np.array([0, 1, 0])
            )
            agent.set_state(agent_state)
            start_time = time.time()
            sim.get_sensor_observations()
            render_time_total += time.time() - start_time

        profile_info["avg_render_time"] = render_time_total / orientation_samples

        # TODO: profile physics
        # sim_horizon = 10  # seconds of simulation to sample over

        # TODO: run collision grid test
        ...


def collision_grid_test(sim):
    # TODO: refactor this
    # run a collision detection grid
    cell_size = 0.5

    obj_templates_mgr = sim.get_object_template_manager()
    rigid_obj_mgr = sim.get_rigid_object_manager()

    scene_bb = sim.get_active_scene_graph().get_root_node().cumulative_bb

    cube_handle = obj_templates_mgr.get_template_handles("cubeSolid")[0]
    cube_template_cpy = obj_templates_mgr.get_template_by_handle(cube_handle)
    cube_template_cpy.scale = np.ones(3) * cell_size

    obj_templates_mgr.register_template(cube_template_cpy, "my_scaled_cube")
    obj = rigid_obj_mgr.add_object_by_template_handle("my_scaled_cube")

    current_cell = scene_bb.min
    while current_cell.x < scene_bb.max.x:

        current_cell.y = scene_bb.min.y
        while current_cell.y < scene_bb.max.y:

            current_cell.z = scene_bb.min.z
            while current_cell.z < scene_bb.max.z:

                # TODO: the check
                obj.translation = current_cell
                print_if_logging(silent, obj.translation)
                sim.perform_discrete_collision_detection()
                current_cell.z += cell_size

            current_cell.y += cell_size

        current_cell.x += cell_size


def process_stages(
    mm: habitat_sim.metadata.MetadataMediator,
    sim_settings: Dict[str, Any],
    generate_navmesh=False,
) -> Dict[str, NavmeshMetrics]:
    # process each stage
    text_format = ANSICodes.BROWN.value
    print_if_logging(silent, text_format + "STAGES")
    stage_handles = mm.stage_template_manager.get_templates_by_handle_substring()
    # optionally manually cull some problem scenes
    # stage_handles = [x for x in stage_handles if "106366104_174226320" not in x]
    # stage_handles = [x for x in stage_handles if x.split("/")[-1].split(".")[0] in modified_stage_handles]

    failure_log = []
    all_scenes_navmesh_metrics = {}
    for stage_handle in stage_handles:

        # print stage handle
        text_format = ANSICodes.BROWN.value
        print_if_logging(silent, text_format + section_divider_str)
        print_if_logging(silent, text_format + f"-{stage_handle}\n")
        cfg.sim_cfg.scene_id = stage_handle

        if stage_handle == "NONE":
            continue

        # attempt to construct simulator and process stage
        try:
            with habitat_sim.Simulator(cfg) as sim:
                stage_filename = stage_handle.split("/")[-1]
                text_format = ANSICodes.PURPLE.value
                print_if_logging(
                    silent, text_format + f"\n  -stage filename: {stage_filename}\n"
                )

                # generate and save navmesh
                if generate_navmesh:
                    stage_directory = stage_handle[: -len(stage_filename)]
                    print_if_logging(
                        silent, text_format + f"  -stage directory: {stage_directory}\n"
                    )
                    # scene_dataset_directory = mm.active_dataset[:-len(mm.active_dataset.split("/")[-1])]
                    # stage_directory = os.path.join(scene_dataset_directory, stage_directory)

                    # create navmesh settings and compute navmesh
                    navmesh_filename = (
                        stage_filename[: -len(stage_filename.split(".")[-1])]
                        + "navmesh"
                    )
                    print_if_logging(
                        silent,
                        text_format + f"  -navmesh filename: {navmesh_filename}\n",
                    )
                    navmesh_settings = habitat_sim.NavMeshSettings()
                    navmesh_settings.set_defaults()
                    sim.recompute_navmesh(sim.pathfinder, navmesh_settings)

                    # save navmesh
                    if os.path.exists(stage_directory):
                        sim.pathfinder.save_nav_mesh(stage_directory + navmesh_filename)
                    else:
                        failure_log.append(
                            (
                                stage_handle,
                                f"No target directory for navmesh: {stage_directory}",
                            )
                        )

                # init agent and get its sensor observations from 4 different poses
                agent = sim.initialize_agent(sim_settings["default_agent"])
                agent_state = habitat_sim.AgentState()
                for _orientation in range(4):
                    agent_state.rotation = utils.quat_from_angle_axis(
                        theta=_orientation * (math.pi / 2.0), axis=np.array([0, 1, 0])
                    )
                    agent.set_state(agent_state)

                    # extract the images
                    observations = sim.get_sensor_observations()
                    rgb = (
                        observations["color_sensor"]
                        if "color_sensor" in observations
                        else None
                    )
                    semantic = (
                        observations["semantic_sensor"]
                        if "semantic_sensor" in observations
                        else None
                    )
                    depth = (
                        observations["depth_sensor"]
                        if "depth_sensor" in observations
                        else None
                    )
                    pil_save_obs(
                        output_file=output_path
                        + stage_handle.split("/")[-1]
                        + str(_orientation)
                        + ".png",
                        rgb=rgb,
                        semantic=semantic,
                        depth=depth,
                    )

                # process navmesh
                all_scenes_navmesh_metrics[stage_filename] = collect_navmesh_metrics(
                    sim
                )
                save_navmesh_data(sim)

        except Exception as e:
            # store any exceptions raised when constructing simulator
            failure_log.append((stage_handle, e))

    # print failure log
    text_format = ANSICodes.GREEN.value
    print_if_logging(silent, text_format + f"\nFailure log = {failure_log}\n")

    # print number of stages we attempted to process
    print_if_logging(
        silent, text_format + f"Tried {len(stage_handles)-1} stages.\n"
    )  # manually decrement the "NONE" scene
    print_if_logging(silent, text_format + section_divider_str)

    return all_scenes_navmesh_metrics


def process_scenes(
    mm: habitat_sim.metadata.MetadataMediator,
    sim_settings: Dict[str, Any],
) -> None:
    # get scene handles
    text_format = ANSICodes.BRIGHT_MAGENTA.value
    print_if_logging(silent, text_format + "SCENES")
    scene_handles: List[str] = mm.get_scene_handles()

    # determine indices of scenes to process
    start_index = 0
    end_index = len(scene_handles)

    # start index provided is valid given the number of scenes
    if (
        "start_scene_index" in sim_settings
        and isinstance(sim_settings["start_scene_index"], int)
        and sim_settings["start_scene_index"] >= 0
        and sim_settings["start_scene_index"] <= len(scene_handles)
    ):
        start_index = sim_settings["start_scene_index"]

    # end index provided is valid given the number of scenes
    if (
        "end_scene_index" in sim_settings
        and isinstance(sim_settings["end_scene_index"], int)
        and sim_settings["end_scene_index"] > sim_settings["start_scene_index"]
        and sim_settings["end_scene_index"] <= len(scene_handles)
    ):
        # end index is exclusive
        end_index = sim_settings["end_scene_index"] + 1

    # process specified scenes
    failure_log = []
    # all_scenes_navmesh_metrics = {}
    for i in range(start_index, end_index):
        scene_handle = scene_handles[i]

        # print scene handle
        text_format = ANSICodes.BRIGHT_MAGENTA.value
        print_if_logging(silent, text_format + section_divider_str)
        print_if_logging(silent, text_format + f"-{scene_handle}\n")
        cfg.sim_cfg.scene_id = scene_handle

        # attempt to construct simulator and process scene
        try:
            with habitat_sim.Simulator(cfg) as sim:
                text_format = ANSICodes.PURPLE.value
                scene_filename = scene_handle.split("/")[-1]
                print_if_logging(
                    silent, text_format + f"\n  -scene filename: {scene_filename}\n"
                )
                scene_directory = scene_handle[: -len(scene_filename)]
                print_if_logging(
                    silent, text_format + f"  -scene directory: {scene_directory}\n"
                )
                sim.close()

        except Exception as e:
            # store any exceptions raised when constructing simulator
            failure_log.append((scene_handle, e))

    # print failure log
    text_format = ANSICodes.GREEN.value
    print_if_logging(silent, text_format + f"\nFailure log = {failure_log}\n")


def iteratively_test_all_scenes(
    cfg: habitat_sim.Configuration, sim_settings: Dict[str, Any], generate_navmesh=False
) -> None:

    mm = cfg.metadata_mediator

    print_dataset_info(silent, mm)

    process_scenes(mm, sim_settings)

    all_scenes_navmesh_metrics: Dict[str, NavmeshMetrics] = process_stages(
        mm, sim_settings, generate_navmesh
    )

    # create cvs detailing navmesh metrics
    filename = create_unique_filename(
        output_path, ".csv", sim_settings["output_file_prefix"], "navmesh_metrics"
    )
    aggregate_navmesh_metrics(all_scenes_navmesh_metrics, filename=filename)

    # print that we are done processing
    text_format = ANSICodes.BRIGHT_RED.value
    print_if_logging(silent, text_format + "PROCESSING COMPLETE")
    print_if_logging(silent, text_format + section_divider_str)


def parse_config_json_file(
    sim_settings: Dict[str, Any],
    config_json,
) -> Dict[str, Any]:
    """
    Update possibly nested sim_settings dictionary. Modifies sim_settings in place.
    """
    for key, value in config_json.items():
        if isinstance(value, Dict) and value:
            nested_settings = parse_config_json_file(sim_settings.get(key, {}), value)
            sim_settings[key] = nested_settings
        else:
            sim_settings[key] = config_json[key]

    return sim_settings


if __name__ == "__main__":
    import argparse

    # parse command line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config_file_path",
        dest="config_file_path",
        type=str,
        help=f'config file to load (default: "{qa_config_filepath}")',
    )
    parser.set_defaults(config_file_path=os.path.join(dir_path, qa_config_filepath))
    args, _ = parser.parse_known_args()

    # Populate sim_settings with data from qa_scene_config.json file
    sim_settings: Dict[str, Any] = default_sim_settings.copy()
    with open(os.path.join(dir_path, args.config_file_path)) as config_json:
        parse_config_json_file(sim_settings, json.load(config_json))

    sim_settings["scene_dataset_config_file"] = os.path.join(
        data_path, sim_settings["scene_dataset_config_file"]
    )

    # setup colored console print statement logic
    init(autoreset=True)

    # create sim config and process scenes/stages
    text_format = ANSICodes.BRIGHT_RED.value
    print_if_logging(silent, text_format + "\nBEGIN PROCESSING")
    print_if_logging(silent, text_format + section_divider_str)

    if "run_viewer" in sim_settings and sim_settings["run_viewer"]:
        # create viewer app
        QASceneProcessingViewer(sim_settings).exec()
    else:
        # make simulator configuration and process all scenes without viewing them in app
        cfg = make_cfg_mm(sim_settings)
        iteratively_test_all_scenes(cfg, sim_settings, generate_navmesh=True)
