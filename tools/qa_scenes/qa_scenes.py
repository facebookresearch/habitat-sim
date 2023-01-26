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
from qa_scene_utils import (  # print_dataset_info,
    ANSICodes,
    Timer,
    create_unique_filename,
    print_if_logging,
    section_divider_str,
)

import habitat_sim
from habitat_sim.agent import Agent, AgentState

# from habitat_sim.physics import MotionType
# from habitat_sim.simulator import ObservationDict
from habitat_sim.utils import common as utils
from habitat_sim.utils import viz_utils as vut
from habitat_sim.utils.common import d3_40_colors_rgb, quat_from_angle_axis

# clean up types with TypeVars
NavmeshMetrics = Dict[str, Union[int, float]]

# get the data path
repo = git.Repo(".", search_parent_directories=True)
dir_path = repo.working_tree_dir
data_path = os.path.join(dir_path, "data")

# get the output directory
output_directory = "./tools/qa_scenes/qa_scenes_output/"  # @param {type:"string"}
output_path = os.path.join(dir_path, output_directory)
if not os.path.exists(output_path):
    os.mkdir(output_path)

# if there are no "scene_instance.json" files in this dataset, create default scenes
# in this folder using the stages in the dataset instead.
default_scene_dir = os.path.join(data_path, "default_qa_scenes")
if not os.path.exists(default_scene_dir):
    os.mkdir(default_scene_dir)

silent: bool = False
MAX_TEST_TIME = sys.float_info.max

# NOTE: change this to config file name to test
qa_config_filename = "default"
# qa_config_filename = "simple_room"
# qa_config_filename = "mp3d_example"
# qa_config_filename = "ycb"
# qa_config_filename = "replica_cad"

config_directory = "./tools/qa_scenes/configs/"
qa_config_filepath = os.path.join(
    config_directory, qa_config_filename + ".qa_scene_config.json"
)


class QASceneProcessingViewer(Application):
    """
    For testing, or just visualization
    """

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

        self.fps: float = sim_settings["fps"]
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
        self.simulating = True

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

        # setup process to iterate an object through positions in a 3D grid and
        # check its collisions with the other objects in the scene
        self.running_collision_test = False
        # TODO: remove
        self.frame_count = 0

    def reconfigure_sim(
        self,
        sim_settings: Dict[str, Any],
    ) -> None:

        self.sim_settings = sim_settings
        self.sim_settings["color_sensor"] = True
        self.sim_settings["depth_sensor"] = False
        self.sim_settings["semantic_sensor"] = False

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

        # get default agent and its scene node
        self.default_agent = self.sim.get_agent(self.agent_id)
        # self.agent_scene_node = self.default_agent.scene_node

        # get agent position from config file
        agent_pos = mn.Vector3(self.sim_settings["agent_pos"])

        # get agent rotation from config file (angle, [axis])
        r = self.sim_settings["agent_rot"]
        agent_rot = quat_from_angle_axis(r[0], np.array(r[1:4]))

        # set agent transform
        self.default_agent.set_state(AgentState(agent_pos, agent_rot))

        # # get the sensor.CameraSensor object
        # self.camera_sensor = self.agent_scene_node.node_sensor_suite.get("color_sensor")

        # # place camera looking down from above to see whole scene
        # self.place_scene_topdown_camera()

        # set sim_settings scene name as actual loaded scene
        self.sim_settings["scene"] = self.sim.curr_scene_name

        Timer.start()
        self.step = -1

    def place_scene_topdown_camera(self) -> None:
        """
        Place the camera in the scene center looking down.
        """
        scene_bb = self.sim.get_active_scene_graph().get_root_node().cumulative_bb
        look_down = mn.Quaternion.rotation(mn.Deg(-90), mn.Vector3.x_axis())
        max_dim = max(scene_bb.size_x(), scene_bb.size_z())
        cam_pos = scene_bb.center()
        cam_pos[1] += 0.52 * max_dim + scene_bb.size_y() / 2.0
        self.default_agent.scene_node.translation = cam_pos
        self.default_agent.scene_node.rotation = look_down

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
        agent_acts_per_sec = self.fps

        # self.framebuffer.clear(
        mn.gl.default_framebuffer.clear(
            mn.gl.FramebufferClear.COLOR | mn.gl.FramebufferClear.DEPTH
        )

        # Agent actions should occur at a fixed rate per second
        self.time_since_last_simulation += Timer.prev_frame_duration
        num_agent_actions: int = self.time_since_last_simulation * agent_acts_per_sec
        self.move_and_look(int(num_agent_actions))

        # run collision test
        if self.running_collision_test and self.frame_count % 30 == 0:
            self.run_discrete_collision_test()

        self.frame_count += 1

        # Occasionally a frame will pass quicker than 1 / fps seconds
        if self.time_since_last_simulation >= self.physics_step_duration:
            if self.simulating or self.simulate_single_step:
                # step physics at a fixed rate
                # In the interest of frame rate, only a single step is taken,
                # even if time_since_last_simulation is quite large
                if not self.running_collision_test:
                    self.sim.step_world(self.physics_step_duration)
                self.simulate_single_step = False
                if simulation_call is not None:
                    simulation_call()
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

        # observations: Dict[str, Any] = self.sim.get_sensor_observations(agent_id)
        self.sim.get_sensor_observations(agent_id)

        # get the sensor.CameraSensor object
        self.camera_sensor = agent.scene_node.node_sensor_suite.get(self.sensor_uuid)

        # TODO write a good comment here, not sure what "blit" is
        self.camera_sensor.render_target.blit_rgba_to_default()
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
            self.reconfigure_sim(self.sim_settings)
            print(f"Reconfigured simulator for scene: {self.sim_settings['scene']}")

        elif key == pressed.C and not self.running_collision_test:
            self.running_collision_test = True
            self.setup_collision_test()
            self.contact_debug_draw = True

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

    def exit_event(self, event: Application.ExitEvent) -> None:
        """
        Overrides exit_event to properly close the Simulator before exiting the
        application.
        """
        self.sim.close(destroy=True)
        event.accepted = True
        exit(0)

    def setup_collision_test(self):
        self.cell_size = 1.0
        scale_factor = 0.5 * self.cell_size

        obj_templates_mgr = self.sim.get_object_template_manager()
        rigid_obj_mgr = self.sim.get_rigid_object_manager()

        self.scene_bb = self.sim.get_active_scene_graph().get_root_node().cumulative_bb

        cube_handle = obj_templates_mgr.get_template_handles("cubeSolid")[0]
        cube_template_cpy = obj_templates_mgr.get_template_by_handle(cube_handle)
        cube_template_cpy.scale = np.ones(3) * scale_factor

        obj_templates_mgr.register_template(cube_template_cpy, "my_scaled_cube")
        self.scaled_cube = rigid_obj_mgr.add_object_by_template_handle("my_scaled_cube")

        self.grid_pos = self.scene_bb.min

        # self.scaled_cube.awake = True
        self.scaled_cube.motion_type = habitat_sim.physics.MotionType.DYNAMIC
        # self.sim.config.sim_cfg.enable_physics = True
        # self.sim.perform_discrete_collision_detection()
        ...

    def run_discrete_collision_test(self):
        self.scaled_cube.translation = self.grid_pos
        self.sim.perform_discrete_collision_detection()

        # update z coordinate
        self.grid_pos.z += self.cell_size

        # if z coordinate exceeds max
        if self.grid_pos.z >= self.scene_bb.max.z:
            # reset z coordinate and update y coordinate
            self.grid_pos.z = self.scene_bb.min.z
            self.grid_pos.y += self.cell_size

            # if y coordinate exceeds max
            if self.grid_pos.y >= self.scene_bb.max.y:
                # reset y coordinate and update x coordinate
                self.grid_pos.y = self.scene_bb.min.y
                self.grid_pos.x += self.cell_size

                # if x coordinate exceeds max
                if self.grid_pos.x >= self.scene_bb.max.x:
                    # we are done running collision test
                    self.running_collision_test = False


# Change to do something like this maybe: https://stackoverflow.com/a/41432704
def display_sample(
    rgb_obs: np.array,
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


def pil_save_obs(
    output_file,
    rgb: np.ndarray,
    semantic: Optional[np.array] = None,
    depth: Optional[np.array] = None,
) -> None:
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


######################################################
# navmesh metrics code
######################################################


def process_navmesh(
    sim: habitat_sim.Simulator,
    scene_filename: str,
    scene_handle: str,
    all_scenes_navmesh_metrics: Dict[str, NavmeshMetrics],
    failure_log: List[Tuple[str, Any]],
):
    # get and print scene directory
    scene_directory = scene_handle[: -len(scene_filename)]
    print_if_logging(silent, text_format + f"  -scene directory: {scene_directory}\n")

    # get and print navmesh filename
    navmesh_filename = scene_filename[: -len(scene_filename.split(".")[-1])] + "navmesh"
    print_if_logging(
        silent,
        text_format + f"  -navmesh filename: {navmesh_filename}\n",
    )

    # create navmesh settings and compute navmesh
    navmesh_settings = habitat_sim.NavMeshSettings()
    navmesh_settings.set_defaults()
    sim.recompute_navmesh(sim.pathfinder, navmesh_settings)

    # save navmesh
    if os.path.exists(scene_directory):
        sim.pathfinder.save_nav_mesh(scene_directory + navmesh_filename)

        # process navmesh
        all_scenes_navmesh_metrics[scene_filename] = collect_navmesh_metrics(sim)
        save_navmesh_data(sim)
    else:
        # TODO: possibly remove this, we have two failure logs for navmeshes
        failure_log.append(
            (
                scene_handle,
                f"No target directory for navmesh: {scene_directory}",
            )
        )


def collect_navmesh_metrics(sim: habitat_sim.Simulator) -> NavmeshMetrics:
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


def save_navmesh_data(sim: habitat_sim.Simulator):
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


######################################################
# scene profiling code
######################################################


def place_scene_topdown_camera(sim: habitat_sim.Simulator, agent: Agent) -> None:
    """
    Place the camera in the scene center looking down.
    """
    scene_bb = sim.get_active_scene_graph().get_root_node().cumulative_bb
    look_down = mn.Quaternion.rotation(mn.Deg(-90), mn.Vector3.x_axis())
    max_dim = max(scene_bb.size_x(), scene_bb.size_z())
    cam_pos = scene_bb.center()
    cam_pos[1] += 0.52 * max_dim + scene_bb.size_y() / 2.0
    agent.scene_node.translation = cam_pos
    agent.scene_node.rotation = look_down


def render_sensor_observations(
    sim: habitat_sim.Simulator,
    scene_filename: str,
    min_max_avg_render_times: Dict[str, List[float]],
    detailed_info_json_dict: Dict[str, Any],
    failure_log: List[Tuple[str, Any]],
) -> None:
    """ """
    text_format = ANSICodes.PURPLE.value
    print_if_logging(
        silent, text_format + f"\nRendering sensor obs for scene: {scene_filename}"
    )

    # init agent
    agent = sim.initialize_agent(sim_settings["default_agent"])
    agent_state = habitat_sim.AgentState()

    # need sensor observation for each cardinal direction and from above
    cardinal_directions: int = 4
    num_poses: int = cardinal_directions + 1
    rgb: List[np.ndarray] = [None] * num_poses
    semantic: List[np.ndarray] = [None] * num_poses
    depth: List[np.ndarray] = [None] * num_poses

    # take sensor observation for each cardinal direction
    render_time_total: float = 0.0
    min_render_time: float = MAX_TEST_TIME
    max_render_time: float = 0.0
    render_times_for_pose: List[float] = [0.0] * num_poses

    try:
        for pose_num in range(num_poses):
            image_filename_suffix: str = ""
            if pose_num == num_poses - 1:
                # take sensor observation from above with all objects visible
                place_scene_topdown_camera(sim, agent)
                image_filename_suffix = "__from_above"
            else:
                # take sensor observation from horizontal direction
                agent_state.rotation = utils.quat_from_angle_axis(
                    theta=pose_num * (math.pi / 2.0), axis=np.array([0, 1, 0])
                )
                agent.set_state(agent_state)
                angle_deg: int = pose_num * 90
                image_filename_suffix = f"__{angle_deg}_deg"

            start_time: float = time.time()
            observations = sim.get_sensor_observations()
            render_times_for_pose[pose_num]: float = time.time() - start_time

            if render_times_for_pose[pose_num] < min_render_time:
                min_render_time = render_times_for_pose[pose_num]
            if render_times_for_pose[pose_num] > max_render_time:
                max_render_time = render_times_for_pose[pose_num]

            render_time_total += render_times_for_pose[pose_num]

            if "color_sensor" in observations:
                rgb[pose_num] = observations["color_sensor"]
            if "semantic_sensor" in observations:
                semantic[pose_num] = observations["semantic_sensor"]
            if "depth_sensor" in observations:
                depth[pose_num] = observations["depth_sensor"]

            pil_save_obs(
                output_file=output_path
                + scene_filename.split(".")[0]
                + image_filename_suffix
                + ".png",
                rgb=rgb[pose_num],
                semantic=semantic[pose_num],
                depth=depth[pose_num],
            )

        avg_render_time: float = render_time_total / num_poses

        # save min, max, and avg render times for this scene
        min_max_avg_render_times[scene_filename] = [
            min_render_time,
            max_render_time,
            avg_render_time,
        ]

    except Exception as e:
        msg = f"error in render_sensor_observations(...) for scene: {scene_filename}"
        failure_log.append((msg, e))

        # save min, max, and avg render times for this scene
        min_max_avg_render_times[scene_filename] = [
            MAX_TEST_TIME,
            MAX_TEST_TIME,
            MAX_TEST_TIME,
        ]

    # construct json entry for exhaustive render test info and append it to list
    # variables to store exhaustive collision test info for json
    render_test_json_info: List[Dict[str, Any]] = []
    render_test_constants: Dict[str, Any] = {
        "render_test_max_time": sim_settings["render_test_max_time"],
    }
    render_test_json_info.append(render_test_constants)

    json_entry: Dict[str, Any] = sim_settings["render_test_json_entry"].copy()
    json_entry["cardinal_dir_render_times"] = [
        render_times_for_pose[0],
        render_times_for_pose[1],
        render_times_for_pose[2],
        render_times_for_pose[3],
    ]
    json_entry["overhead_render_time"] = render_times_for_pose[4]

    json_entry["min_render_time"] = min_max_avg_render_times[scene_filename][0]
    json_entry["max_render_time"] = min_max_avg_render_times[scene_filename][1]
    json_entry["avg_render_time"] = min_max_avg_render_times[scene_filename][2]

    json_entry["exceeds_time_threshold"] = bool(
        min_max_avg_render_times[scene_filename][1]
        > sim_settings["render_test_max_time"]
    )
    render_test_json_info.append(json_entry)

    # add this render test info to our comprehensive json dict
    detailed_info_json_dict[scene_filename].update(
        {"render_test": render_test_json_info}
    )


def collision_grid_test(
    sim: habitat_sim.Simulator,
    scene_filename: str,
    min_max_avg_collision_times: Dict[str, List[float]],
    detailed_info_json_dict: Dict[str, Any],
    failure_log: List[Tuple[str, Any]],
):
    """ """
    text_format = ANSICodes.PURPLE.value
    print_if_logging(
        silent, text_format + f"\nRunning collision test for scene: {scene_filename}"
    )

    # create scaled cube to iterate through scene and test its collisions
    cell_size = sim_settings["collision_test_cell_size"]

    # scales from center of cube, so we scale halfway in both positive and
    # negative directions for each dimension
    scale_factor = 0.5 * cell_size

    obj_templates_mgr = sim.get_object_template_manager()
    rigid_obj_mgr = sim.get_rigid_object_manager()

    scene_bb = sim.get_active_scene_graph().get_root_node().cumulative_bb

    cube_handle = obj_templates_mgr.get_template_handles("cubeSolid")[0]
    cube_template_cpy = obj_templates_mgr.get_template_by_handle(cube_handle)
    cube_template_cpy.scale = np.ones(3) * scale_factor

    obj_templates_mgr.register_template(cube_template_cpy, "my_scaled_cube")
    scaled_cube = rigid_obj_mgr.add_object_by_template_handle("my_scaled_cube")

    grid_pos = scene_bb.min

    # variables to store exhaustive collision test info for json
    collision_test_json_info: List[Dict[str, Any]] = []
    collision_test_overall_data: Dict[str, Any] = {
        "--": "Overall Collision Test Info",
        "collision_test_cell_size": sim_settings["collision_test_cell_size"],
        "max_time_threshold": sim_settings["collision_test_max_time_threshold"],
    }
    collision_test_json_info.append(collision_test_overall_data)
    test_num: int = 1
    ijk_indices = [0, 0, 0]

    # iterate cube through a 3D grid of positions, checking its collisions at
    # each one.
    min_collision_time: float = MAX_TEST_TIME
    max_collision_time: float = 0.0
    total_collision_time: float = 0.0
    avg_collision_time: float = MAX_TEST_TIME

    running_collision_test = True
    while running_collision_test:
        scaled_cube.translation = grid_pos
        try:
            start_time = time.time()
            sim.perform_discrete_collision_detection()
            collision_test_time = time.time() - start_time
            total_collision_time += collision_test_time

            # determine if this is the min or max collision time so far
            if collision_test_time < min_collision_time:
                min_collision_time = collision_test_time
            if collision_test_time > max_collision_time:
                max_collision_time = collision_test_time

            # construct json entry for exhaustive collision test info and append it to list
            entry: Dict[str, Any] = sim_settings["collision_test_json_entry"].copy()
            entry["collision_test_num"] = test_num
            entry["ijk_indices"] = [ijk_indices[0], ijk_indices[1], ijk_indices[2]]
            entry["pos"] = [grid_pos.x, grid_pos.y, grid_pos.z]
            entry["test_time"] = collision_test_time
            entry["exceeds_time_threshold"] = bool(
                collision_test_time > sim_settings["collision_test_max_time_threshold"]
            )
            collision_test_json_info.append(entry)

        except Exception as e:
            # store any exceptions raised when constructing simulator
            msg = f"failure in collision_grid_test(...) for scene: {scene_filename}\nat cube position: {grid_pos}"
            failure_log.append((msg, e))

        # TODO: the following code would probably be clearer as a nested for-loop.
        # That way we don't have to manually track the i,j, and k indices

        # update z coordinate
        grid_pos.z += cell_size
        ijk_indices[2] += 1

        # if z coordinate exceeds max
        if grid_pos.z >= scene_bb.max.z:
            # reset z coordinate and update y coordinate
            grid_pos.z = scene_bb.min.z
            ijk_indices[2] = 0

            grid_pos.y += cell_size
            ijk_indices[1] += 1

            # if y coordinate exceeds max
            if grid_pos.y >= scene_bb.max.y:
                # reset y coordinate and update x coordinate
                grid_pos.y = scene_bb.min.y
                ijk_indices[1] = 0

                grid_pos.x += cell_size
                ijk_indices[0] += 1

                # if x coordinate exceeds max
                if grid_pos.x >= scene_bb.max.x:
                    # we are done running collision test
                    running_collision_test = False

        test_num += 1

    # save collision time summary to save in csv later
    avg_collision_time = total_collision_time / test_num
    min_max_avg_collision_times[scene_filename] = [
        min_collision_time,
        max_collision_time,
        avg_collision_time,
    ]

    # add this collision test info to our comprehensive json dict
    collision_test_overall_data["min_collision_time"] = min_collision_time
    collision_test_overall_data["max_collision_time"] = max_collision_time
    collision_test_overall_data["avg_collision_time"] = avg_collision_time
    detailed_info_json_dict[scene_filename].update(
        {"collision_test": collision_test_json_info}
    )


def asset_sleep_test(
    sim: habitat_sim.Simulator,
    scene_filename: str,
    min_max_avg_asleep_times: Dict[str, List[float]],
    detailed_info_json_dict: Dict[str, Any],
    failure_log: List[Tuple[str, Any]],
) -> None:
    """
    step world for how many seconds specified in the config file and record
    how long it takes for each rigid object to fall asleep, if at all.
    """
    # get object attributes manager, rigid object manager, and get asset filenames
    obj_templates_mgr = sim.get_object_template_manager()
    obj_templates_mgr.load_configs(sim_settings["scene_dataset_config_file"])
    rigid_obj_mgr = sim.get_rigid_object_manager()

    if rigid_obj_mgr.get_num_objects() == 0:
        failure_log.append(("No dataset objects in scene", scene_filename))
        return

    rigid_obj_handles: List[str] = rigid_obj_mgr.get_object_handles("")

    # store times it takes for each object to fall asleep, as well as the min,
    # max, and avg times
    all_asleep_times: Dict[str, float] = {}
    min_asleep_time: float = MAX_TEST_TIME
    max_asleep_time: float = 0.0
    avg_asleep_time: float = MAX_TEST_TIME

    # variables to store exhaustive sleep test info for json
    asleep_test_json_info: List[Dict[str, Any]] = []
    asleep_test_overall_data: Dict[str, Any] = {
        "--": "Overall Sleep Test Summary",
        "asset_sleep_test_duration_seconds": sim_settings[
            "asset_sleep_test_duration_seconds"
        ],
    }
    asleep_test_json_info.append(asleep_test_overall_data)

    # Step physics at given fps for given number of seconds and test if the
    # dataset objects are asleep
    physics_step_dur: float = 1.0 / sim_settings["sleep_test_steps_per_sec"]
    curr_sim_time: float = 0.0
    test_duration: float = sim_settings["asset_sleep_test_duration_seconds"]

    total_obj_asleep: int = 0
    while curr_sim_time <= test_duration:
        for handle in rigid_obj_handles:
            rigid_obj = rigid_obj_mgr.get_object_by_handle(handle)

            # object is asleep, record the current time
            if not rigid_obj.awake and all_asleep_times.get(handle) is None:
                all_asleep_times[handle] = curr_sim_time
                if curr_sim_time < min_asleep_time:
                    min_asleep_time = curr_sim_time
                if curr_sim_time > max_asleep_time:
                    max_asleep_time = curr_sim_time

                total_obj_asleep += 1
                if total_obj_asleep == len(rigid_obj_handles):
                    break

        sim.step_world(physics_step_dur)
        curr_sim_time += physics_step_dur

    # after stepping physics for the given number of seconds, check if any
    # objects are still awake and store the results in a comprehensive json
    any_obj_awake: bool = False
    asleep_time_total: float = 0.0
    for handle in rigid_obj_handles:
        rigid_obj = rigid_obj_mgr.get_object_by_handle(handle)
        if rigid_obj.awake:
            any_obj_awake = True
            max_asleep_time = MAX_TEST_TIME
            all_asleep_times[handle] = MAX_TEST_TIME
        else:
            asleep_time_total += all_asleep_times[handle]

        # construct json entry for exhaustive collision test info and append it to list
        entry: Dict[str, Any] = sim_settings["asleep_test_json_entry"].copy()
        entry["asset_handle"] = handle
        entry["test_time"] = all_asleep_times[handle]
        entry["successfully_sleeps"] = not rigid_obj.awake
        asleep_test_json_info.append(entry)

    if not any_obj_awake:
        avg_asleep_time = asleep_time_total / len(rigid_obj_handles)

    min_max_avg_asleep_times[scene_filename] = [
        min_asleep_time,
        max_asleep_time,
        avg_asleep_time,
    ]

    # update asleep test info summary (min, max, and avg sleep times) and add this
    # sleep test info to our comprehensive json dict
    asleep_test_overall_data["min_asleep_time"] = min_asleep_time
    asleep_test_overall_data["max_asleep_time"] = max_asleep_time
    asleep_test_overall_data["avg_asleep_time"] = avg_asleep_time
    detailed_info_json_dict[scene_filename].update(
        {"asleep_test": asleep_test_json_info}
    )


def save_test_times_csv(
    scene_handles: List[str],
    start_index: int,
    end_index: int,
    min_max_avg_render_times: Dict[str, List[float]],
    min_max_avg_collision_times: Dict[str, List[float]],
    min_max_avg_asleep_times: Dict[str, List[float]],
    filename: str,
) -> None:
    import csv

    # save a csv of navmesh metrics
    with open(filename, "w") as f:
        writer = csv.writer(f, quoting=csv.QUOTE_ALL)
        writer.writerow(
            [
                "Scene",
                "Render Times ->",
                "min",
                "max",
                "avg",
                "Collision Times ->",
                "min",
                "max",
                "avg",
                "Sleep Times ->",
                "min",
                "max",
                "avg",
            ]
        )
        for i in range(start_index, end_index):
            scene_filename = scene_handles[i].split("/")[-1]

            row_data = [scene_filename, ""]

            if sim_settings["render_sensor_obs"]:
                row_data.append(min_max_avg_render_times[scene_filename][0])
                row_data.append(min_max_avg_render_times[scene_filename][1])
                row_data.append(min_max_avg_render_times[scene_filename][2])
            else:
                row_data.append("")
                row_data.append("")
                row_data.append("")
            row_data.append("")

            if sim_settings["run_collision_test"]:
                row_data.append(min_max_avg_collision_times[scene_filename][0])
                row_data.append(min_max_avg_collision_times[scene_filename][1])
                row_data.append(min_max_avg_collision_times[scene_filename][2])
            else:
                row_data.append("")
                row_data.append("")
                row_data.append("")
            row_data.append("")

            if sim_settings["run_asset_sleep_test"]:
                row_data.append(min_max_avg_asleep_times[scene_filename][0])
                row_data.append(min_max_avg_asleep_times[scene_filename][1])
                row_data.append(min_max_avg_asleep_times[scene_filename][2])
            else:
                row_data.append("")
                row_data.append("")
                row_data.append("")

            writer.writerow(row_data)


######################################################
# end scene profiling code
######################################################


def process_scene(
    cfg: habitat_sim.Configuration,
    scene_handle: str,
    all_scenes_navmesh_metrics: Dict[str, NavmeshMetrics],
    min_max_avg_render_times: Dict[str, List[float]],
    min_max_avg_collision_times: Dict[str, List[float]],
    min_max_avg_asleep_times: Dict[str, List[float]],
    detailed_info_json_dict: Dict[str, Any],
    failure_log: List[Tuple[str, Any]],
):
    cfg.sim_cfg.scene_id = scene_handle

    # print scene handle
    text_format = ANSICodes.BRIGHT_MAGENTA.value
    print_if_logging(silent, text_format + section_divider_str)
    print_if_logging(silent, text_format + f"-{scene_handle}\n")

    try:
        with habitat_sim.Simulator(cfg) as sim:

            text_format = ANSICodes.BRIGHT_RED.value
            scene_filename = scene_handle.split("/")[-1]
            print_if_logging(
                silent, text_format + f"\n\t---processing scene: {scene_filename}\n"
            )
            detailed_info_json_dict[scene_filename] = {}

            # generate and save navmesh
            if sim_settings["generate_navmesh"]:
                process_navmesh(
                    sim,
                    scene_filename,
                    scene_handle,
                    all_scenes_navmesh_metrics,
                    failure_log,
                )

            # Get sensor observations from 5 different poses.
            # Record all rendering times, as well as min, max, and avg rendering time.
            # Save the observations in an image and the times in a json/csv
            if sim_settings["render_sensor_obs"]:
                render_sensor_observations(
                    sim,
                    scene_filename,
                    min_max_avg_render_times,
                    detailed_info_json_dict,
                    failure_log,
                )

            # run collision grid test, save all test times, as well as min, max, and
            # avgerage times in a json/csv
            if sim_settings["run_collision_test"]:
                collision_grid_test(
                    sim,
                    scene_filename,
                    min_max_avg_collision_times,
                    detailed_info_json_dict,
                    failure_log,
                )

            # run physics for a set number of seconds (default to 10 seconds) to see
            # how long the assets take to go to sleep
            if sim_settings["run_asset_sleep_test"]:
                asset_sleep_test(
                    sim,
                    scene_filename,
                    min_max_avg_asleep_times,
                    detailed_info_json_dict,
                    failure_log,
                )

            sim.close()

    except Exception as e:
        # store any exceptions raised when processing scene
        msg = f"error with scene {scene_handle} in process_scene(...) function"
        failure_log.append((msg, e))


def process_requested_scenes(
    cfg: habitat_sim.Configuration, sim_settings: Dict[str, Any]
) -> None:

    mm = cfg.metadata_mediator

    # get scene handles
    scene_handles: List[str] = mm.get_scene_handles()

    # make stages into simple scenes with just the stage
    stage_handles: List[
        str
    ] = mm.stage_template_manager.get_templates_by_handle_substring()

    # TODO: if no scenes, make default scenes using the stages
    if len(scene_handles) == 0:
        scene_handles = construct_default_scenes(stage_handles)
        # no default scenes to process (there were no stages)
        if len(scene_handles) == 0:
            return

    # determine indices of scenes to process
    start_index = 0
    end_index = len(scene_handles)

    # determine if start index provided in config is valid
    if (
        isinstance(sim_settings["start_scene_index"], int)
        and sim_settings["start_scene_index"] >= 0
        and sim_settings["start_scene_index"] <= len(scene_handles)
    ):
        start_index = sim_settings["start_scene_index"]

    # determine if end index provided in config is valid
    if (
        isinstance(sim_settings["end_scene_index"], int)
        and sim_settings["end_scene_index"] >= sim_settings["start_scene_index"]
        and sim_settings["end_scene_index"] <= len(scene_handles)
    ):
        # end index is exclusive
        end_index = sim_settings["end_scene_index"] + 1

    text_format = ANSICodes.BRIGHT_MAGENTA.value
    print_if_logging(silent, text_format + "SCENES")

    # process specified scenes
    all_scenes_navmesh_metrics: Dict[str, NavmeshMetrics] = {}
    min_max_avg_render_times: Dict[str, List[float]] = {}
    min_max_avg_collision_times: Dict[str, List[float]] = {}
    min_max_avg_asleep_times: Dict[str, List[float]] = {}
    detailed_info_json_dict: Dict[str, Any] = {}
    failure_log: List[Tuple[str, Any]] = []
    for i in range(start_index, end_index):
        process_scene(
            cfg,
            scene_handles[i],
            all_scenes_navmesh_metrics,
            min_max_avg_render_times,
            min_max_avg_collision_times,
            min_max_avg_asleep_times,
            detailed_info_json_dict,
            failure_log,
        )

    # create cvs detailing scene navmesh metrics
    navmesh_cvs_filename = create_unique_filename(
        dir_path=output_path,
        filename_prefix=sim_settings["output_file_prefix"],
        filename_suffix="scene_navmesh_metrics",
        extension=".csv",
    )
    aggregate_navmesh_metrics(all_scenes_navmesh_metrics, filename=navmesh_cvs_filename)

    # TODO: create cvs detailing rendering, collision, and asset sleep test metric
    # summaries. More comprehensive metrics will be stored in a json
    test_times_cvs_filename = create_unique_filename(
        dir_path=output_path,
        filename_prefix=sim_settings["output_file_prefix"],
        filename_suffix="test_time_metrics",
        extension=".csv",
    )
    save_test_times_csv(
        scene_handles,
        start_index,
        end_index,
        min_max_avg_render_times,
        min_max_avg_collision_times,
        min_max_avg_asleep_times,
        test_times_cvs_filename,
    )

    # save json file with comprehensive test data for each scene
    json_filename = create_unique_filename(
        dir_path=output_path,
        filename_prefix=sim_settings["output_file_prefix"],
        filename_suffix="detailed_info",
        extension=".json",
    )
    with open(json_filename, "w") as outfile:
        json.dump(detailed_info_json_dict, fp=outfile, indent=2)

    # print failure log
    text_format = ANSICodes.GREEN.value
    print_if_logging(silent, text_format + "\nOverall failure log:")
    for error in failure_log:
        print_if_logging(silent, text_format + f"{error}")
    print_if_logging(silent, text_format + section_divider_str)

    # print number of scenes we attempted to process
    print_if_logging(silent, text_format + f"\nTried {end_index - start_index} scenes.")
    print_if_logging(silent, text_format + section_divider_str)


def construct_default_scenes(stage_handles: List[str]) -> List[str]:

    text_format = ANSICodes.BRIGHT_RED.value
    print_if_logging(
        silent,
        text_format
        + "No scenes available; constructing defaults using available stages.\n",
    )

    non_null_stage_handles: List[str] = []
    for stage_handle in stage_handles:
        if stage_handle != "NONE":
            non_null_stage_handles.append(stage_handle)

    if len(non_null_stage_handles) == 0:
        print_if_logging(silent, text_format + "No stages available either.\n")

    return []


def parse_config_json_file(
    sim_settings: Dict[str, Any],
    config_json,
) -> Dict[str, Any]:
    """
    Update possibly nested sim_settings dictionary from config file.
    Modifies sim_settings in place.
    """
    for key, value in config_json.items():
        if isinstance(value, Dict) and value:
            nested_settings = parse_config_json_file(sim_settings.get(key, {}), value)
            sim_settings[key] = nested_settings
        else:
            sim_settings[key] = config_json[key]

    return sim_settings


def make_cfg_mm(sim_settings: Dict[str, Any]) -> habitat_sim.Configuration:
    """
    Create a Configuration with an attached MetadataMediator for shared dataset access and re-use
    """
    config = make_cfg(sim_settings)

    # create and attach a MetadataMediator
    mm = habitat_sim.metadata.MetadataMediator(config.sim_cfg)

    return habitat_sim.Configuration(config.sim_cfg, config.agents, mm)


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

    # begin processing
    text_format = ANSICodes.BRIGHT_RED.value
    print_if_logging(silent, text_format + "\nBEGIN PROCESSING")
    print_if_logging(silent, text_format + section_divider_str)

    if sim_settings["run_viewer"]:
        # create viewer app
        QASceneProcessingViewer(sim_settings).exec()
    else:
        # make simulator configuration and process all scenes without viewing them in app
        cfg = make_cfg_mm(sim_settings)
        process_requested_scenes(cfg, sim_settings)

    # done processing
    text_format = ANSICodes.BRIGHT_RED.value
    print_if_logging(silent, text_format + "PROCESSING COMPLETE")
    print_if_logging(silent, text_format + section_divider_str)
