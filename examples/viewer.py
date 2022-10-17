# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import ctypes
import datetime
import math
import os
import string
import sys
import time
from enum import Enum
from typing import Any, Callable, Dict, List, Optional, Tuple

import git
import psutil

flags = sys.getdlopenflags()
sys.setdlopenflags(flags | ctypes.RTLD_GLOBAL)

import magnum as mn
import numpy as np
from magnum import shaders, text
from magnum.platform.glfw import Application
from viewer_settings import default_sim_settings, make_cfg

import habitat_sim
from habitat_sim import physics
from habitat_sim.logging import LoggingContext, logger
from habitat_sim.utils import viz_utils as vut
from habitat_sim.utils.common import quat_from_angle_axis

repo = git.Repo(".", search_parent_directories=True)
dir_path = repo.working_tree_dir
# fmt: off
output_directory = "examples/video_output/"  # @param {type:"string"}
# fmt: on
output_path = os.path.join(dir_path, output_directory)
if not os.path.exists(output_path):
    os.mkdir(output_path)


class MemoryUnitConverter:
    """
    class to convert computer memory value units, e.g.
    1,000,000 bytes to 1 megabyte
    """

    BYTES = 0
    KILOBYTES = 1
    MEGABYTES = 2
    GIGABYTES = 3
    TERABYTES = 4

    UNIT_STRS = ["bytes", "KB", "MB", "GB", "TB"]

    UNIT_CONVERSIONS = [1, 1 << 10, 1 << 20, 1 << 30, 1 << 40]


class HabitatSimInteractiveViewer(Application):

    # Default transforms of agent and dataset object as static variables
    # to use when resetting the agent and dataset object transforms
    DEFAULT_AGENT_POSITION = np.array([0.25, 0.34, 0.73])  # in front of table
    DEFAULT_AGENT_ROTATION = mn.Quaternion.identity_init()
    DEFAULT_OBJ_POSITION = np.array([0.25, 1.75, -0.23])  # above table
    DEFAULT_OBJ_ROTATION = mn.Quaternion.identity_init()

    # it is usually 1.5, but 1.0 is a little closer to table
    DEFAULT_SENSOR_HEIGHT = 1.0

    # How fast displayed dataset object spins when in Kinematic mode
    REVOLUTION_DURATION_SEC = 4.0
    ROTATION_DEGREES_PER_SEC = 360.0 / REVOLUTION_DURATION_SEC

    # Default (r,g,b) components of bounding box color during debug draw
    BOUNDING_BOX_RGB = mn.Vector3(1.0, 0.8, 1.0)

    # We don't always have an NVIDIA GPU, you can enable this to log
    # GPU memory usage if you have one
    USING_NVIDIA_GPU = True

    # the maximum number of chars displayable in the app window
    # using the magnum text module. These chars are used to
    # display the CPU/GPU usage data
    MAX_DISPLAY_TEXT_CHARS = 256

    # how much to displace window text relative to the center of the
    # app window (e.g if you want the display text in the top left of
    # the app window, you will displace the text
    # window width * -TEXT_DELTA_FROM_CENTER in the x axis and
    # widnow height * TEXT_DELTA_FROM_CENTER in the y axis, as the text
    # position defaults to the middle of the app window)
    TEXT_DELTA_FROM_CENTER = 0.49

    # font size of the magnum in-window display text that displays
    # CPU and GPU usage info
    DISPLAY_FONT_SIZE = 16.0

    def __init__(self, sim_settings: Dict[str, Any]) -> None:

        configuration = self.Configuration()
        configuration.title = "Habitat Sim Interactive Viewer"
        Application.__init__(self, configuration)
        self.sim_settings: Dict[str:Any] = sim_settings
        self.fps: float = 60.0
        self.physics_step_duration: float = 1.0 / self.fps

        # draw Bullet debug line visualizations (e.g. collision meshes)
        self.debug_bullet_draw = False
        # draw active contact point debug line visualizations
        self.contact_debug_draw = False
        # draw bounding box of currently displayed rigid body object from dataset
        self.bounding_box_debug_draw = False
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

        # Load a TrueTypeFont plugin and open the font file
        self.display_font = text.FontManager().load_and_instantiate("TrueTypeFont")
        relative_path_to_font = "../data/fonts/ProggyClean.ttf"
        self.display_font.open_file(
            os.path.join(os.path.dirname(__file__), relative_path_to_font),
            13,
        )

        # Glyphs we need to render text to screen
        self.glyph_cache = text.GlyphCache(mn.Vector2i(256))
        self.display_font.fill_glyph_cache(
            self.glyph_cache,
            string.ascii_lowercase
            + string.ascii_uppercase
            + string.digits
            + ":-_+,./! %µ",
        )

        # magnum text object that displays CPU/GPU usage data in the app window
        self.window_text = text.Renderer2D(
            self.display_font,
            self.glyph_cache,
            HabitatSimInteractiveViewer.DISPLAY_FONT_SIZE,
            text.Alignment.TOP_LEFT,
        )
        self.window_text.reserve(HabitatSimInteractiveViewer.MAX_DISPLAY_TEXT_CHARS)

        # text object transform in window space is Projection matrix times Translation Matrix
        # put text in top left of window
        self.window_text_transform = mn.Matrix3.projection(
            mn.Vector2(self.viewport_size)
        ) @ mn.Matrix3.translation(
            mn.Vector2(
                self.viewport_size[0]
                * -HabitatSimInteractiveViewer.TEXT_DELTA_FROM_CENTER,
                self.viewport_size[1]
                * HabitatSimInteractiveViewer.TEXT_DELTA_FROM_CENTER,
            )
        )
        self.shader = shaders.VectorGL2D()

        # make magnum text background transparent
        mn.gl.Renderer.enable(mn.gl.Renderer.Feature.BLENDING)
        mn.gl.Renderer.set_blend_function(
            mn.gl.Renderer.BlendFunction.ONE,
            mn.gl.Renderer.BlendFunction.ONE_MINUS_SOURCE_ALPHA,
        )
        mn.gl.Renderer.set_blend_equation(
            mn.gl.Renderer.BlendEquation.ADD, mn.gl.Renderer.BlendEquation.ADD
        )

        # variables that track sim time, render time, and CPU/GPU usage
        self.total_frame_count: int = 0  # TODO debugging, remove

        self.render_frames_to_track: int = 30

        self.prev_frame_duration: float = 0.0
        self.frame_duration_sum: float = 0.0
        self.average_fps: float = self.fps

        self.prev_sim_duration: float = 0.0
        self.sim_duration_sum: float = 0.0
        self.avg_sim_duration: float = 0.0
        self.sim_steps_tracked: int = 0

        self.prev_render_duration: float = 0.0
        self.render_duration_sum: float = 0.0
        self.avg_render_duration: float = 0.0
        self.render_frames_tracked: int = 0

        self.ram_memory_used_bytes: int = 0
        self.obj_ram_memory_used: Dict[str, int] = {}

        self.decimal_points_round = 4

        # Cycle mouse utilities
        self.mouse_interaction = MouseMode.LOOK
        self.mouse_grabber: Optional[MouseGrabber] = None
        self.previous_mouse_point = None

        # toggle physics simulation on/off
        self.simulating = False

        # toggle a single simulation step at the next opportunity if not
        # simulating continuously.
        self.simulate_single_step = False

        # Object rotates in kinematic mode.
        # -self.curr_angle_rotated_degrees: defines how much it has rotated
        #   from [0, 360) degrees.
        # -self.object_rotation_axis: defines around which axis it is rotating.
        self.curr_angle_rotated_degrees = 0.0
        self.object_rotation_axis = ObjectRotationAxis.Y

        # -self.video_frames: list to store video frames as observations
        # -self.recording: If we are currently storing frames in a List to be
        #   written to a file when done.
        # -self.saving_video: if the previous recording is still being written
        #   to a file. We don't want to record new frames in that case.
        self.video_frames = []
        self.recording = False
        self.saving_video = False

        # TODO testing
        self.idle_start_time: float = 0
        self.idle_end_time: float = 0
        self.obj_awake: bool = False
        self.rot_index: int = 0

        # configure our simulator
        self.cfg: Optional[habitat_sim.simulator.Configuration] = None
        self.sim: Optional[habitat_sim.simulator.Simulator] = None
        self.reconfigure_sim()

        # Get object attribute manager for test objects from dataset,
        # load the dataset, store all the object template handles in a list,
        self.object_attributes_manager = self.sim.get_object_template_manager()
        self.object_attributes_manager.load_configs(
            self.sim_settings["scene_dataset_config_file"]
        )
        self.object_template_handles = (
            self.object_attributes_manager.get_file_template_handles("")
        )
        print_in_color(
            f"number of ojects in dataset: {len(self.object_template_handles)}",
            PrintColors.BLUE,
        )

        # -self.object_template_handle_index: stores current object's index in the
        #   object dataset.
        # -self.curr_object: stores the current ManagedBulletRigidObject from dataset
        self.object_template_handle_index = 0
        self.curr_object = None

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

    def draw_bounding_boxes_debug(self):
        """
        Draw the bounding box of the current object. The corners of the bounding
        box are ordered like this:
        [
            bounding_box.back_bottom_left,
            bounding_box.back_bottom_right,
            bounding_box.back_top_right,
            bounding_box.back_top_left,
            bounding_box.front_top_left,
            bounding_box.front_top_right,
            bounding_box.front_bottom_right,
            bounding_box.front_bottom_left,
        ]
        """
        rgb = HabitatSimInteractiveViewer.BOUNDING_BOX_RGB
        line_color = mn.Color4.from_xyz(rgb)
        bb_corners: List[mn.Vector3] = self.get_bounding_box_corners(self.curr_object)
        num_corners = len(bb_corners)
        self.sim.get_debug_line_render().set_line_width(0.01)
        obj_transform = self.curr_object.transformation

        # only need to iterate over first 4 corners to draw whole thing
        for i in range(int(num_corners / 2)):
            # back of box
            back_corner_local_pos = bb_corners[i]
            back_corner_world_pos = obj_transform.transform_point(back_corner_local_pos)
            next_back_index = (i + 1) % 4
            next_back_corner_local_pos = bb_corners[next_back_index]
            next_back_corner_world_pos = obj_transform.transform_point(
                next_back_corner_local_pos
            )
            self.sim.get_debug_line_render().draw_transformed_line(
                back_corner_world_pos,
                next_back_corner_world_pos,
                line_color,
            )
            # side edge that this corner is a part of
            front_counterpart_index = num_corners - i - 1
            front_counterpart_local_pos = bb_corners[front_counterpart_index]
            front_counterpart_world_pos = obj_transform.transform_point(
                front_counterpart_local_pos
            )
            self.sim.get_debug_line_render().draw_transformed_line(
                back_corner_world_pos,
                front_counterpart_world_pos,
                line_color,
            )
            # front of box
            next_front_index = (front_counterpart_index - 4 - 1) % 4 + 4
            next_front_corner_local_pos = bb_corners[next_front_index]
            next_front_corner_world_pos = obj_transform.transform_point(
                next_front_corner_local_pos
            )
            self.sim.get_debug_line_render().draw_transformed_line(
                front_counterpart_world_pos,
                next_front_corner_world_pos,
                line_color,
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
        if self.bounding_box_debug_draw:
            self.draw_bounding_boxes_debug()

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
        self.prev_frame_duration = Timer.prev_frame_duration
        self.time_since_last_simulation += Timer.prev_frame_duration
        num_agent_actions: int = self.time_since_last_simulation * agent_acts_per_sec
        self.move_and_look(int(num_agent_actions))

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

        # # If in Kinematic movement mode and there is a ManagedBulletRigidObject
        # # that is displayed from the dataset, rotate it
        # if self.curr_object is not None and not self.simulating:
        #     self.rotate_displayed_object(self.curr_object)

        # TODO testing awake functionality
        if (
            self.curr_object is not None
            and self.obj_awake
            and not self.curr_object.awake
        ):
            self.idle_end_time = time.time()
            duration = round(
                self.idle_end_time - self.idle_start_time, self.decimal_points_round
            )
            dur_str: str = "{:,}".format(duration)
            self.obj_awake = False
            print_in_color(
                f"time until idle - {self.curr_object.handle}\n{dur_str} sec",
                PrintColors.CYAN,
            )

        # Get agent id, agent, and sensor uuid
        keys = active_agent_id_and_sensor_name
        agent_id = keys[0]
        agent = self.sim.get_agent(agent_id)
        self.sensor_uuid = keys[1]

        # gather and render sensor observation while timing it
        render_start_time = time.time()
        observations = self.sim.get_sensor_observations(agent_id)
        render_end_time = time.time()
        self.prev_render_duration = render_end_time - render_start_time

        # TODO write a good comment here, not sure what "blit" is
        self.render_camera = agent.scene_node.node_sensor_suite.get(self.sensor_uuid)
        self.debug_draw()
        self.render_camera.render_target.blit_rgba_to_default()
        mn.gl.default_framebuffer.bind()

        # TODO draw CPU/GPU usage data and other info to the app window
        self.draw_text(self.render_camera.specification())

        # if we are recording and no recording is currently being saved to file,
        # store the sensor observation as a frame in a list of observations
        if self.recording and not self.saving_video:
            self.video_frames.append(observations)

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

        # Reset agent transform
        self.agent_body_node.translation = mn.Vector3(
            HabitatSimInteractiveViewer.DEFAULT_AGENT_POSITION
        )
        self.agent_body_node.rotation = (
            HabitatSimInteractiveViewer.DEFAULT_AGENT_ROTATION
        )

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
                logger.warning("Warning: physics was not enabled during setup")
            else:
                if self.simulating and self.curr_object is not None:
                    # if setting to kinematic, reset object transform,
                    # velocity, and angular velocity
                    obj = self.curr_object
                    obj.translation = HabitatSimInteractiveViewer.DEFAULT_OBJ_POSITION
                    rotations: List[Tuple] = self.sim_settings["sim_test_rotations"]
                    angle_axis: Tuple = (
                        mn.Rad(mn.Deg(rotations[self.rot_index][0])),
                        mn.Vector3(rotations[self.rot_index][1]),
                    )
                    obj.rotation = mn.Quaternion.rotation(angle_axis[0], angle_axis[1])
                    obj.linear_velocity = mn.Vector3(0.0)
                    obj.angular_velocity = mn.Vector3(0.0)

                    # reset rotation angle for displaying object when kinematic
                    self.curr_angle_rotated_degrees = 0.0
                    self.object_rotation_axis = ObjectRotationAxis.Y

                self.simulating = not self.simulating
                logger.info(f"Command: physics simulating set to {self.simulating}\n")

        elif key == pressed.PERIOD:
            if self.simulating:
                logger.warning("Warning: physic simulation already running")
            else:
                self.simulate_single_step = True
                logger.info("Command: physics step taken")

        elif key == pressed.SEMICOLON:
            """
            Prints memory usage for CPU and GPU (although it assumes you have an
            NVIDIA GPU as of now, so you have to set
            HabitatSimInteractiveViewer.USING_NVIDIA_GPU to True if you want to print
            GPU memory usage)
            """
            self.print_memory_usage()

        elif key == pressed.COMMA:
            self.debug_bullet_draw = not self.debug_bullet_draw
            logger.info(f"Command: toggle Bullet debug draw: {self.debug_bullet_draw}")

        elif key == pressed.SLASH:
            logger.info("Command: drop object from multiple angles to test physics")

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
                ...

        elif key == pressed.K:
            """
            Toggle the drawing of the current object's bounding box
            (if the object is not None)
            """

            self.bounding_box_debug_draw = not self.bounding_box_debug_draw

            if self.curr_object is not None:
                # if object exists
                obj_name = self.curr_object.handle.replace("_:0000", "")
                if self.bounding_box_debug_draw:
                    # if turned on bb drawing
                    print_in_color(
                        f"Draw bounding box for object: {obj_name}\n",
                        PrintColors.MAGENTA,
                        logging=True,
                    )
                else:
                    # if turned off bb drawing
                    print_in_color(
                        f"Don't draw bounding box for object: {obj_name}\n",
                        PrintColors.MAGENTA,
                        logging=True,
                    )
            else:
                # if NULL object
                print_in_color(
                    "Command: can't draw bounding box of object: None\n",
                    PrintColors.RED,
                    logging=True,
                )
                self.bounding_box_debug_draw = False

        elif key == pressed.T:
            # load URDF
            fixed_base = alt_pressed
            urdf_file_path = ""
            if shift_pressed and self.cached_urdf:
                urdf_file_path = self.cached_urdf
            else:
                urdf_file_path = input("Load URDF: provide a URDF filepath:").strip()

            if not urdf_file_path:
                logger.warning("Load URDF: no input provided. Aborting.")
            elif not urdf_file_path.endswith((".URDF", ".urdf")):
                logger.warning("Load URDF: input is not a URDF. Aborting.")
            elif os.path.exists(urdf_file_path):
                self.cached_urdf = urdf_file_path
                articulated_object_manager = self.sim.get_articulated_object_manager()
                articulated_object = (
                    articulated_object_manager.add_articulated_object_from_urdf(
                        urdf_file_path, fixed_base, 1.0, 1.0, True
                    )
                )
                articulated_object.translation = (
                    self.agent_body_node.transformation.transform_point(
                        [0.0, 1.0, -1.5]
                    )
                )
            else:
                logger.warning("Load URDF: input file not found. Aborting.")

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
                    logger.warning("Warning: recompute navmesh first")

        elif key == pressed.I:
            # decrement template handle index and add corresponding
            # ManagedBulletRigidObject to rigid object manager from dataset
            self.cycle_dataset_object(-1)

        elif key == pressed.P:
            # increment template handle index and add corresponding
            # ManagedBulletRigidObject to rigid object manager from dataset
            self.cycle_dataset_object(1)

        elif key == pressed.O:
            if self.curr_object:
                # snap current ManagedBulletRigidObject to the nearest surface in
                # the direction of gravity with physics on to make sure it doesn't
                # topple over
                self.simulating = True
                logger.info(
                    "Command: snapping dataset object to table and switching to Dynamic motion\n"
                )
                self.snap_down_object(self.curr_object)
                self.idle_start_time = time.time()
                self.obj_awake = True
            else:
                logger.info("Can't snap NULL object to table\n")

        elif key == pressed.R:
            if self.curr_object:
                rotations: List[Tuple] = self.sim_settings["sim_test_rotations"]
                self.rot_index = (self.rot_index + 1) % len(rotations)
                angle_axis: Tuple = (
                    mn.Rad(mn.Deg(rotations[self.rot_index][0])),
                    mn.Vector3(rotations[self.rot_index][1]),
                )
                self.curr_object.rotation = mn.Quaternion.rotation(
                    angle_axis[0], angle_axis[1]
                )

        elif key == pressed.L:
            # press L to start recording, then L again to stop it

            if not self.recording and not self.saving_video:
                # if we are not recording and not writing prev recording to file,
                # we can start a new recording
                self.recording = True
                print_in_color("* " * 39, PrintColors.RED)
                print_in_color("Command: start recording\n", PrintColors.RED)
            elif not self.recording and self.saving_video:
                # if we are not recording but still writing prev recording to file,
                # wait until the video file is written before recording again
                print_in_color("-" * 78, PrintColors.RED)
                print_in_color(
                    "Command: can't record, still saving previous recording\n",
                    PrintColors.RED,
                )
                print_in_color("-" * 78 + "\n", PrintColors.RED)
            elif self.recording and not self.saving_video:
                # if we are recording but not writing prev recording to file, we need
                # to stop recording and save the recorded frames to a video file
                self.recording = False
                self.saving_video = True

                self.save_video_file()

        elif key == pressed.ONE:
            # Apply impulse to dataset object if it exists and it is Dynamic motion mode
            if self.curr_object and self.simulating:
                print_in_color(
                    "\nCommand: applying impulse to object.\n",
                    PrintColors.YELLOW,
                    logging=True,
                )
                self.curr_object.apply_impulse(
                    mn.Vector3(0, 1, 0), mn.Vector3(0, 0, -0.1)
                )
            elif self.curr_object is None:
                print_in_color(
                    "\nCommand: can't apply impulse, no object exists.\n",
                    PrintColors.YELLOW,
                    logging=True,
                )
            elif not self.simulating:
                print_in_color(
                    "\nCommand: can't apply impulse, turn on Dynamic motion mode.\n",
                    PrintColors.YELLOW,
                    logging=True,
                )

        elif key == pressed.TWO:
            # Apply force to dataset object if it exists and it is Dynamic motion mode
            if self.curr_object and self.simulating:
                print_in_color(
                    "\nCommand: applying force to object.\n",
                    PrintColors.YELLOW,
                    logging=True,
                )
                self.curr_object.apply_force(
                    mn.Vector3(0, 40, 0), mn.Vector3(0, 0, -0.1)
                )
            elif self.curr_object is None:
                print_in_color(
                    "\nCommand: can't apply force, no object exists.\n",
                    PrintColors.YELLOW,
                    logging=True,
                )
            elif not self.simulating:
                print_in_color(
                    "\nCommand: can't apply force, turn on Dynamic motion mode.\n",
                    PrintColors.YELLOW,
                    logging=True,
                )

        elif key == pressed.THREE:
            # Apply impulse torque to dataset object if it exists and it is Dynamic motion mode
            if self.curr_object and self.simulating:
                print_in_color(
                    "\nCommand: applying impulse torque to object.\n",
                    PrintColors.YELLOW,
                    logging=True,
                )
                self.curr_object.apply_impulse_torque(mn.Vector3(0, 0.1, 0))
            elif self.curr_object is None:
                print_in_color(
                    "\nCommand: can't apply impulse torque, no object exists.\n",
                    PrintColors.YELLOW,
                    logging=True,
                )
            elif not self.simulating:
                print_in_color(
                    "\nCommand: can't apply impulse torque, turn on Dynamic motion mode.\n",
                    PrintColors.YELLOW,
                    logging=True,
                )

        elif key == pressed.FOUR:
            # Apply torque to dataset object if it exists and it is Dynamic motion mode
            if self.curr_object and self.simulating:
                print_in_color(
                    "\nCommand: applying torque to object.\n",
                    PrintColors.YELLOW,
                    logging=True,
                )
                self.curr_object.apply_torque(mn.Vector3(0, 10, 0))
            elif self.curr_object is None:
                print_in_color(
                    "\nCommand: can't apply torque, no object exists.\n",
                    PrintColors.YELLOW,
                    logging=True,
                )
            elif not self.simulating:
                print_in_color(
                    "\nCommand: can't apply torque, turn on Dynamic motion mode.\n",
                    PrintColors.YELLOW,
                    logging=True,
                )

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
                    rigid_object_manager = self.sim.get_rigid_object_manager()
                    articulated_object_manager = (
                        self.sim.get_articulated_object_manager()
                    )
                    articulated_object = articulated_object_manager.get_object_by_id(
                        hit_info.object_id
                    )
                    rigid_object = rigid_object_manager.get_object_by_id(
                        hit_info.object_id
                    )

                    if rigid_object:
                        # if grabbed an object
                        hit_object = hit_info.object_id
                        object_pivot = (
                            rigid_object.transformation.inverted().transform_point(
                                hit_info.point
                            )
                        )
                        object_frame = rigid_object.rotation.inverted()
                    elif articulated_object:
                        # if grabbed the base link
                        hit_object = hit_info.object_id
                        object_pivot = articulated_object.transformation.inverted().transform_point(
                            hit_info.point
                        )
                        object_frame = articulated_object.rotation.inverted()
                    else:
                        for (
                            ao_handle
                        ) in (
                            articulated_object_manager.get_objects_by_handle_substring()
                        ):
                            articulated_object = (
                                articulated_object_manager.get_object_by_handle(
                                    ao_handle
                                )
                            )
                            link_to_obj_ids = articulated_object.link_object_ids

                            if hit_info.object_id in link_to_obj_ids:
                                # if we got a link
                                ao_link = link_to_obj_ids[hit_info.object_id]
                                object_pivot = (
                                    articulated_object.get_link_scene_node(ao_link)
                                    .transformation.inverted()
                                    .transform_point(hit_info.point)
                                )
                                object_frame = articulated_object.get_link_scene_node(
                                    ao_link
                                ).rotation.inverted()
                                hit_object = articulated_object.object_id
                                break
                    # done checking for articulated_object

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
                        logger.warning(
                            "Oops, couldn't find the hit object. That's odd."
                        )
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

        self.sim.recompute_navmesh(
            self.sim.pathfinder,
            self.navmesh_settings,
            include_static_objects=True,
        )

    def cycle_dataset_object(self, index_delta: int = 1) -> None:
        keep_simulating = self.simulating
        if index_delta < 0:
            # instantiate previous dataset object
            self.object_template_handle_index -= 1
            if self.object_template_handle_index < 0:
                self.object_template_handle_index = (
                    len(self.object_template_handles) - 1
                )
        elif index_delta > 0:
            # instantiate next dataset object
            self.object_template_handle_index += 1
            if self.object_template_handle_index >= len(self.object_template_handles):
                self.object_template_handle_index = 0
        else:
            return
        self.add_new_object_from_dataset(
            self.object_template_handle_index,
            HabitatSimInteractiveViewer.DEFAULT_OBJ_POSITION,
        )
        if keep_simulating:
            # make sure physics stays on, but place object upright in center of table
            self.simulating = True
            self.snap_down_object(self.curr_object)

    def add_new_object_from_dataset(self, index, position=DEFAULT_OBJ_POSITION) -> None:
        """
        Add to scene the ManagedBulletRigidObject at given template handle index
        from dataset at the provided position.
        """

        # make sure there are any ManagedBulletRigidObjects from a dataset
        if len(self.object_template_handles) == 0:
            print_in_color(
                "\nCommand: no objects in dataset to add to rigid object manager",
                PrintColors.BLUE,
                logging=True,
            )
            return

        # get rigid object manager and clear it
        rigid_object_manager = self.sim.get_rigid_object_manager()
        rigid_object_manager.remove_all_objects()

        # get ManagedBulletRigidObject template handle at given index from dataset
        object_template_handle = self.object_template_handles[index]

        # Configure the initial object orientation via local Euler angle (degrees):
        rotation_x_degrees = 0
        rotation_y_degrees = 0
        rotation_z_degrees = 0

        # compose the rotations
        rotation_x_quaternion = mn.Quaternion.rotation(
            mn.Deg(rotation_x_degrees), mn.Vector3(1.0, 0, 0)
        )
        rotation_y_quaternion = mn.Quaternion.rotation(
            mn.Deg(rotation_y_degrees), mn.Vector3(0, 1.0, 0)
        )
        rotation_z_quaternion = mn.Quaternion.rotation(
            mn.Deg(rotation_z_degrees), mn.Vector3(0, 0, 1.0)
        )
        rotation_quaternion = (
            rotation_z_quaternion * rotation_y_quaternion * rotation_x_quaternion
        )

        start_ram_used = psutil.virtual_memory()[3]
        # Add object instantiated by desired template to scene using template handle
        self.curr_object = rigid_object_manager.add_object_by_template_handle(
            object_template_handle
        )
        end_ram_used = psutil.virtual_memory()[3]
        ram_memory_used_bytes = end_ram_used - start_ram_used

        # Agent local coordinate system is Y up and -Z forward.
        # Move object above table surface, then turn on Kinematic mode
        # to easily reposition object in center of table
        self.curr_object.translation = position
        self.curr_object.rotation = rotation_quaternion
        self.simulating = False
        self.curr_angle_rotated_degrees = 0.0

        # reset axis that the new object rotates around when being displayed
        self.object_rotation_axis = ObjectRotationAxis.Y

        # for some reason they all end in "_:0000" so remove that substring before print
        self.obj_name = self.curr_object.handle.replace("_:0000", "")

        # store RAM memory used for this object if not already stored
        if self.obj_ram_memory_used.get(self.obj_name) == None:
            self.obj_ram_memory_used[self.obj_name] = ram_memory_used_bytes

        # print out object name and its index into the list of the objects
        # in dataset.
        print_in_color(
            f'\nCommand: placing object "{self.obj_name}" from template handle index: {index}\n',
            PrintColors.BLUE,
            logging=True,
        )

    def snap_down_object(
        self,
        obj: habitat_sim.physics.ManagedRigidObject,
        support_obj_ids: Optional[List[int]] = None,
    ) -> bool:
        """
        Attempt to project an object in the gravity direction onto the surface below it.
        :param sim: The Simulator instance.
        :param obj: The RigidObject instance.
        :param support_obj_ids: A list of object ids designated as valid support surfaces for object placement. Contact with other objects is a criteria for placement rejection. If none provided, default support surface is the stage/ground mesh (-1).
        :param vdb: Optionally provide a DebugVisualizer (vdb) to render debug images of each object's computed snap position before collision culling.
        Reject invalid placements by checking for penetration with other existing objects.
        Returns boolean success.
        If placement is successful, the object state is updated to the snapped location.
        If placement is rejected, object position is not modified and False is returned.
        To use this utility, generate an initial placement for any object above any of the designated support surfaces and call this function to attempt to snap it onto the nearest surface in the gravity direction.
        """
        cached_position = obj.translation

        if support_obj_ids is None:
            # set default support surface to stage/ground mesh
            support_obj_ids = [-1]

        bounding_box_ray_prescreen_results = self.bounding_box_ray_prescreen(
            obj, support_obj_ids
        )

        if bounding_box_ray_prescreen_results["surface_snap_point"] is None:
            # no support under this object, return failure
            return False

        # finish up
        if bounding_box_ray_prescreen_results["surface_snap_point"] is not None:
            # accept the final location if a valid location exists
            obj.translation = bounding_box_ray_prescreen_results["surface_snap_point"]
            self.sim.perform_discrete_collision_detection()
            cps = self.sim.get_physics_contact_points()
            for cp in cps:
                if (
                    cp.object_id_a == obj.object_id or cp.object_id_b == obj.object_id
                ) and (
                    (cp.contact_distance < -0.01)
                    or not (
                        cp.object_id_a in support_obj_ids
                        or cp.object_id_b in support_obj_ids
                    )
                ):
                    obj.translation = cached_position
                    # print(f" Failure: contact in final position w/ distance = {cp.contact_distance}.")
                    # print(f" Failure: contact in final position with non support object {cp.object_id_a} or {cp.object_id_b}.")
                    return False
            return True
        else:
            # no valid position found, reset and return failure
            obj.translation = cached_position
            return False

    def rotate_displayed_object(
        self, obj, degrees_per_sec=ROTATION_DEGREES_PER_SEC
    ) -> None:
        """
        When ManagedBulletRigidObject "obj" from dataset is in Kinematic mode, it is
        displayed above the table in simple_room.glb and rotates so that the agent can
        see all sides of it. This function rotates the object from the past frame to
        this frame
        """

        # How much to rotate current ManagedBulletRigidObject this frame
        delta_rotation_degrees = degrees_per_sec * Timer.prev_frame_duration

        # check for a full 360 degree revolution
        # and clamp to 360 degrees if we have passed it
        reset_curr_rotation_angle = False
        if self.curr_angle_rotated_degrees + delta_rotation_degrees >= 360.0:
            reset_curr_rotation_angle = True
            delta_rotation_degrees = 360.0 - self.curr_angle_rotated_degrees

        if self.object_rotation_axis == ObjectRotationAxis.Y:
            # rotate about object's local y axis (up vector)
            y_rotation_in_degrees = mn.Deg(delta_rotation_degrees)
            y_rotation_in_radians = mn.Rad(y_rotation_in_degrees)
            obj.rotate_y_local(y_rotation_in_radians)
        else:
            # rotate about object's local x axis (horizontal vector)
            x_rotation_in_degrees = mn.Deg(delta_rotation_degrees)
            x_rotation_in_radians = mn.Rad(x_rotation_in_degrees)
            obj.rotate_x_local(x_rotation_in_radians)

        # update current rotation angle
        self.curr_angle_rotated_degrees += delta_rotation_degrees

        # reset current rotation angle if it passed 360 degrees
        if reset_curr_rotation_angle:
            self.curr_angle_rotated_degrees = 0.0

            # change ManagedBulletRigidObject's axis of rotation
            if self.object_rotation_axis == ObjectRotationAxis.Y:
                self.object_rotation_axis = ObjectRotationAxis.X
            else:
                self.object_rotation_axis = ObjectRotationAxis.Y

    def save_video_file(self) -> None:
        """
        write each sensor observation for "color_sensor" in self.video_frames to video file
        """
        # Current date and time so we can make unique video file names for each recording
        date_and_time = datetime.datetime.now()

        # year-month-day
        date = date_and_time.strftime("%Y-%m-%d")

        # hour:min:sec - capital H is military time, %I is standard time
        # (0-12 hour time format)
        time = date_and_time.strftime("%H:%M:%S")

        # construct file path and write consecutive frames to new video file
        file_path = f"{output_path}viewer_recording__date_{date}__time_{time}.mp4"
        print_in_color("-" * 78, PrintColors.RED)
        print_in_color(
            f"Command: End recording, saving frames to the video file below \n{file_path}",
            PrintColors.RED,
        )
        print_in_color("-" * 78 + "\n", PrintColors.RED)

        vut.make_video(
            observations=self.video_frames,
            primary_obs=self.sensor_uuid,
            primary_obs_type="color",
            video_file=file_path,
            fps=self.fps,
            open_vid=False,
        )

        print_in_color(
            "Recording is saved, you can record something else now", PrintColors.RED
        )
        print_in_color("* " * 39 + "\n", PrintColors.RED)

        self.saving_video = False
        self.video_frames.clear()

    def get_bounding_box_corners(
        self,
        obj: habitat_sim.physics.ManagedRigidObject,
    ) -> List[mn.Vector3]:
        """
        Return a list of object bounding box corners in object local space.
        """
        bounding_box = obj.root_scene_node.cumulative_bb
        return [
            bounding_box.back_bottom_left,
            bounding_box.back_bottom_right,
            bounding_box.back_top_right,
            bounding_box.back_top_left,
            bounding_box.front_top_left,
            bounding_box.front_top_right,
            bounding_box.front_bottom_right,
            bounding_box.front_bottom_left,
        ]

    def bounding_box_ray_prescreen(
        self,
        obj: habitat_sim.physics.ManagedRigidObject,
        support_obj_ids: Optional[List[int]] = None,
        check_all_corners: bool = False,
    ) -> Dict[str, Any]:
        """
        Pre-screen a potential placement by casting rays in the gravity direction from the object center of mass (and optionally each corner of its bounding box) checking for interferring objects below.
        :param sim: The Simulator instance.
        :param obj: The RigidObject instance.
        :param support_obj_ids: A list of object ids designated as valid support surfaces for object placement. Contact with other objects is a criteria for placement rejection.
        :param check_all_corners: Optionally cast rays from all bounding box corners instead of only casting a ray from the center of mass.
        """
        if support_obj_ids is None:
            # set default support surface to stage/ground mesh
            support_obj_ids = [-1]
        lowest_key_point: mn.Vector3 = None
        lowest_key_point_height = None
        highest_support_impact: mn.Vector3 = None
        highest_support_impact_height = None
        highest_support_impact_with_stage = False
        raycast_results = []
        gravity_dir = self.sim.get_gravity().normalized()
        object_local_to_global = obj.transformation
        bounding_box_corners = self.get_bounding_box_corners(obj)
        key_points = [mn.Vector3(0)] + bounding_box_corners  # [COM, c0, c1 ...]
        support_impacts: Dict[int, mn.Vector3] = {}  # indexed by keypoints
        for ix, key_point in enumerate(key_points):
            world_point = object_local_to_global.transform_point(key_point)
            # NOTE: instead of explicit Y coordinate, we project onto any gravity vector
            world_point_height = world_point.projected_onto_normalized(
                -gravity_dir
            ).length()
            if lowest_key_point is None or lowest_key_point_height > world_point_height:
                lowest_key_point = world_point
                lowest_key_point_height = world_point_height
            # cast a ray in gravity direction
            if ix == 0 or check_all_corners:
                ray = habitat_sim.geo.Ray(world_point, gravity_dir)
                raycast_results.append(self.sim.cast_ray(ray))
                # classify any obstructions before hitting the support surface
                for hit in raycast_results[-1].hits:
                    if hit.object_id == obj.object_id:
                        continue
                    elif hit.object_id in support_obj_ids:
                        hit_point = ray.origin + ray.direction * hit.ray_distance
                        support_impacts[ix] = hit_point
                        support_impact_height = mn.math.dot(hit_point, -gravity_dir)

                        if (
                            highest_support_impact is None
                            or highest_support_impact_height < support_impact_height
                        ):
                            highest_support_impact = hit_point
                            highest_support_impact_height = support_impact_height
                            highest_support_impact_with_stage = hit.object_id == -1

                    # terminates at the first non-self ray hit
                    break
        # compute the relative base height of the object from its lowest bounding_box corner and COM
        base_rel_height = (
            lowest_key_point_height
            - obj.translation.projected_onto_normalized(-gravity_dir).length()
        )

        # account for the affects of stage mesh margin
        margin_offset = (
            0
            if not highest_support_impact_with_stage
            else self.sim.get_stage_initialization_template().margin
        )

        surface_snap_point = (
            None
            if 0 not in support_impacts
            else support_impacts[0] + gravity_dir * (base_rel_height - margin_offset)
        )
        # return list of obstructed and grounded rays, relative base height, distance to first surface impact, and ray results details
        return {
            "base_rel_height": base_rel_height,
            "surface_snap_point": surface_snap_point,
            "raycast_results": raycast_results,
        }

    def print_memory_usage(self) -> None:
        """
        Print CPU and GPU memory usage
        """
        print_in_color(
            """
==================================================
Memory Usage
==================================================
            """,
            PrintColors.LIGHT_GREEN,
            logging=True,
        )
        self.print_cpu_usage()

    def print_cpu_usage(self) -> None:
        cpu_percent = psutil.cpu_percent()
        cpu_stats = psutil.cpu_stats()
        cpu_freq = psutil.cpu_freq()
        print_in_color(
            f"""CPU Usage
----------------------------------
CPU Memory
    CPU memory usage: {cpu_percent:.2f}%
CPU Stats
    Context switches: {cpu_stats.ctx_switches:,}
    Interrupts: {cpu_stats.interrupts:,}
    Software interrupts: {cpu_stats.soft_interrupts:,}
    System calls: {cpu_stats.syscalls:,}
CPU Frequency
    Current frequency: {cpu_freq.current:,.2f} MHz
    Min frequency: {cpu_freq.min:,} MHz
    Max frequency: {cpu_freq.max:,} MHz
----------------------------------
            """,
            PrintColors.CYAN,
        )

    def exit_event(self, event: Application.ExitEvent) -> None:
        """
        Overrides exit_event to properly close the Simulator before exiting the
        application.
        """
        self.sim.close(destroy=True)
        event.accepted = True
        exit(0)

    def calc_time_stats(self) -> None:
        self.render_frames_tracked += 1
        self.total_frame_count += 1  # TODO for debugging, remove

        self.frame_duration_sum += self.prev_frame_duration
        self.prev_frame_duration = 0.0

        self.render_duration_sum += self.prev_render_duration
        self.prev_render_duration = 0.0

        self.sim_duration_sum += self.prev_sim_duration
        self.prev_sim_duration = 0.0

        if self.render_frames_tracked % self.render_frames_to_track == 0:
            # # TODO debug logging, remove
            # print(f"total frame count:  {self.total_frame_count}")
            # print(f"render frame count: {self.render_frames_tracked}")
            # print(f"sim step count:     {self.sim_steps_tracked}\n")

            # calculate average frame rate
            frame_duration_avg = self.frame_duration_sum / self.render_frames_to_track
            self.average_fps = round(
                1.0 / frame_duration_avg, self.decimal_points_round
            )
            self.frame_duration_sum = 0.0

            # calculate average render time
            self.avg_render_duration = (
                self.render_duration_sum / self.render_frames_to_track
            )
            self.avg_render_duration = round(
                self.avg_render_duration / self.physics_step_duration,
                self.decimal_points_round,
            )
            self.render_duration_sum = 0.0
            self.render_frames_tracked = 0

            # calculate average simulation time if simulating and we have any tracked
            # sim steps. We don't always step physics on each draw_event() call
            if self.simulating and self.sim_steps_tracked != 0:
                self.avg_sim_duration = self.sim_duration_sum / self.sim_steps_tracked
                self.avg_sim_duration = round(
                    self.avg_sim_duration / self.physics_step_duration,
                    self.decimal_points_round,
                )
            self.sim_duration_sum = 0.0
            self.sim_steps_tracked = 0

    def get_ram_usage_string(
        self, unit_type: int = MemoryUnitConverter.MEGABYTES
    ) -> str:
        if self.curr_object is None:
            return "None"
        unit_conversion: int = MemoryUnitConverter.UNIT_CONVERSIONS[unit_type]
        unit_str: str = MemoryUnitConverter.UNIT_STRS[unit_type]
        ram_memory_used_bytes = self.obj_ram_memory_used.get(self.obj_name)
        ram_memory_used = round(
            ram_memory_used_bytes / unit_conversion, self.decimal_points_round
        )
        return f"{ram_memory_used} {unit_str}"

    def draw_text(self, sensor_spec) -> None:
        self.calc_time_stats()
        ram_usage_string = self.get_ram_usage_string()

        self.shader.bind_vector_texture(self.glyph_cache.texture)
        self.shader.transformation_projection_matrix = self.window_text_transform
        self.shader.color = [1.0, 1.0, 1.0]
        self.window_text.render(
            f"""
avg fps: {self.average_fps}
avg sim time ratio: {self.avg_sim_duration if self.simulating else "N/A"}
avg render time ratio: {self.avg_render_duration}
sensor type: {str(sensor_spec.sensor_type.name).lower()}
sensor subtype: {str(sensor_spec.sensor_subtype.name).lower()}
curr obj: {self.obj_name if self.curr_object is not None else "None"}
obj RAM usage: {ram_usage_string}
{str(self.mouse_interaction).lower()}
            """
        )
        self.shader.draw(self.window_text.mesh)

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
    RIGHT: Rotate current object from dataset displayed (if any) if in kinematic mode

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
    ';'         Print to terminal the CPU and GPU memory usage for this process.
    ',':        Render a Bullet collision shape debug wireframe overlay (white=active, green=sleeping, blue=wants sleeping, red=can't sleep).
    'c':        Run a discrete collision detection pass and render a debug wireframe overlay showing active contact points and normals (yellow=fixed length normals, red=collision distances).
                (+SHIFT) Toggle the contact point debug render overlay on/off.
    'k':        Draw bounding boxes

    Object Interactions:
    SPACE:      Toggle physics simulation on/off.
    '.':        Take a single simulation step if not simulating continuously.
    'v':        (physics) Invert gravity.
    'i':        Go backward through dataset of objects and generate current object above table.
    'p':        Go forward through dataset of objects and generate current object above table.
    'o':        Turn on physics and snap current object onto the surface below it.
    'l':        Press 'L' to start recording, then 'L' again to stop recording
    't':        Load URDF from filepath
                (+SHIFT) quick re-load the previously specified URDF
                (+ALT) load the URDF with fixed base
    '1':        Apply impulse to current object.
    '2':        Apply force to current object.
    '3':        Apply impulse torque to current object.
    '4':        Apply torque to current object.
=====================================================
"""
        )


class MouseMode(Enum):
    LOOK = 0
    GRAB = 1
    MOTION = 2


class ObjectRotationAxis(Enum):
    Y = 0
    X = 1
    Z = 2


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
        rigid_object_manager = self.simulator.get_rigid_object_manager()
        articulated_object_manager = self.simulator.get_articulated_object_manager()
        if rigid_object_manager.get_library_has_id(self.settings.object_id_a):
            object_transform = rigid_object_manager.get_object_by_id(
                self.settings.object_id_a
            ).transformation
        else:
            # must be an articulated_object
            object_transform = (
                articulated_object_manager.get_object_by_id(self.settings.object_id_a)
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


class PrintColors:
    """
    Console printing ANSI color codes
    """

    HEADER = "\033[95m"
    WHITE = "\u001b[37m"
    RED = "\033[1;31m"
    GREEN = "\033[92m"
    BLUE = "\033[94m"
    CYAN = "\033[96m"
    MAGENTA = "\u001b[35m"
    YELLOW = "\u001b[33m"
    BROWN = "\033[0;33m"
    LIGHT_RED = "\033[1;31m"
    LIGHT_GREEN = "\033[1;32m"
    LIGHT_BLUE = "\033[1;34m"
    LIGHT_PURPLE = "\033[1;35m"
    LIGHT_CYAN = "\033[1;36m"
    LIGHT_WHITE = "\033[1;37m"
    LIGHT_GRAY = "\033[0;37m"
    TEST = "\u001a[35m"
    WARNING = "\033[93m"
    FAIL = "\033[91m"
    ENDC = "\033[0m"
    BOLD = "\033[1m"
    UNDERLINE = "\033[4m"


def print_in_color(print_string="", color=PrintColors.WHITE, logging=False) -> None:
    """
    Allows us to print to console in different colors
    """
    if logging:
        logger.info(color + print_string + PrintColors.ENDC)
    else:
        print(color + print_string + PrintColors.ENDC)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()

    # optional arguments
    parser.add_argument(
        "--scene",
        default="./data/test_assets/scenes/simple_room.glb",
        type=str,
        help='scene/stage file to load (default: "./data/test_assets/scenes/simple_room.glb")',
    )
    parser.add_argument(
        "--dataset",
        default="./data/objects/ycb/ycb.scene_dataset_config.json",
        type=str,
        metavar="DATASET",
        help='dataset configuration file to use (default: "./data/objects/ycb/ycb.scene_dataset_config.json")',
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
    sim_settings["sensor_height"] = HabitatSimInteractiveViewer.DEFAULT_SENSOR_HEIGHT
    sim_settings["stage_requires_lighting"] = args.stage_requires_lighting

    # start the application
    HabitatSimInteractiveViewer(sim_settings).exec()
