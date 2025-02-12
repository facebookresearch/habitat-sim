#!/usr/bin/env python3

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import ctypes
import math
import os
import string
import sys
import time
from enum import Enum
from typing import Any, Callable, Dict, List, Optional, Tuple, Union

flags = sys.getdlopenflags()
sys.setdlopenflags(flags | ctypes.RTLD_GLOBAL)

import magnum as mn
import numpy as np
from magnum import shaders, text
from magnum.platform.glfw import Application

import habitat_sim
from habitat_sim import ReplayRenderer, ReplayRendererConfiguration
from habitat_sim.logging import LoggingContext, logger
from habitat_sim.utils.classes import MarkerSetsEditor, ObjectEditor, SemanticDisplay
from habitat_sim.utils.common import quat_from_angle_axis
from habitat_sim.utils.namespace import hsim_physics
from habitat_sim.utils.settings import default_sim_settings, make_cfg

# file holding all URDF filenames
URDF_FILES = os.path.join(
    os.path.dirname(os.path.realpath(__file__)), "urdfFileNames.txt"
)
# file holding hashes of objects that have no links
NOLINK_URDF_FILES = os.path.join(
    os.path.dirname(os.path.realpath(__file__)), "urdfsWithNoLinks.txt"
)


class HabitatSimInteractiveViewer(Application):
    # the maximum number of chars displayable in the app window
    # using the magnum text module. These chars are used to
    # display the CPU/GPU usage data
    MAX_DISPLAY_TEXT_CHARS = 512

    # how much to displace window text relative to the center of the
    # app window (e.g if you want the display text in the top left of
    # the app window, you will displace the text
    # window width * -TEXT_DELTA_FROM_CENTER in the x axis and
    # window height * TEXT_DELTA_FROM_CENTER in the y axis, as the text
    # position defaults to the middle of the app window)
    TEXT_DELTA_FROM_CENTER = 0.49

    # font size of the magnum in-window display text that displays
    # CPU and GPU usage info
    DISPLAY_FONT_SIZE = 16.0

    def __init__(self, sim_settings: Dict[str, Any]) -> None:
        self.sim_settings: Dict[str:Any] = sim_settings

        self.enable_batch_renderer: bool = self.sim_settings["enable_batch_renderer"]
        self.num_env: int = (
            self.sim_settings["num_environments"] if self.enable_batch_renderer else 1
        )

        # Compute environment camera resolution based on the number of environments to render in the window.
        window_size: mn.Vector2 = (
            self.sim_settings["window_width"],
            self.sim_settings["window_height"],
        )

        configuration = self.Configuration()
        configuration.title = "Habitat Sim Interactive Viewer"
        configuration.size = window_size
        Application.__init__(self, configuration)
        self.fps: float = 60.0

        # Compute environment camera resolution based on the number of environments to render in the window.
        grid_size: mn.Vector2i = ReplayRenderer.environment_grid_size(self.num_env)
        camera_resolution: mn.Vector2 = mn.Vector2(self.framebuffer_size) / mn.Vector2(
            grid_size
        )
        self.sim_settings["width"] = camera_resolution[0]
        self.sim_settings["height"] = camera_resolution[1]

        # draw Bullet debug line visualizations (e.g. collision meshes)
        self.debug_bullet_draw = False
        # draw active contact point debug line visualizations
        self.contact_debug_draw = False

        # cache most recently loaded URDF file for quick-reload
        self.cached_urdf = ""

        # set up our movement map
        key = Application.Key
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

        # Glyphs we need to render everything
        self.glyph_cache = text.GlyphCacheGL(
            mn.PixelFormat.R8_UNORM, mn.Vector2i(256), mn.Vector2i(1)
        )
        self.display_font.fill_glyph_cache(
            self.glyph_cache,
            string.ascii_lowercase
            + string.ascii_uppercase
            + string.digits
            + ":-_+,.! %µ",
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
            self.framebuffer_size
        ) @ mn.Matrix3.translation(
            mn.Vector2(self.framebuffer_size)
            * mn.Vector2(
                -HabitatSimInteractiveViewer.TEXT_DELTA_FROM_CENTER,
                HabitatSimInteractiveViewer.TEXT_DELTA_FROM_CENTER,
            )
        )
        self.shader = shaders.VectorGL2D()

        # make magnum text background transparent
        mn.gl.Renderer.enable(mn.gl.Renderer.Feature.BLENDING)
        mn.gl.Renderer.set_blend_function(
            mn.gl.Renderer.BlendFunction.ONE,
            mn.gl.Renderer.BlendFunction.ONE_MINUS_SOURCE_ALPHA,
        )
        # Set blend function
        mn.gl.Renderer.set_blend_equation(
            mn.gl.Renderer.BlendEquation.ADD, mn.gl.Renderer.BlendEquation.ADD
        )

        # variables that track app data and CPU/GPU usage
        self.num_frames_to_track = 60

        # Cycle mouse utilities
        self.mouse_interaction = MouseMode.LOOK
        self.previous_mouse_point = None

        # toggle physics simulation on/off
        self.simulating = True

        # toggle a single simulation step at the next opportunity if not
        # simulating continuously.
        self.simulate_single_step = False

        # configure our simulator
        self.cfg: Optional[habitat_sim.simulator.Configuration] = None
        self.sim: Optional[habitat_sim.simulator.Simulator] = None
        self.tiled_sims: list[habitat_sim.simulator.Simulator] = None
        self.replay_renderer_cfg: Optional[ReplayRendererConfiguration] = None
        self.replay_renderer: Optional[ReplayRenderer] = None

        self.last_hit_details = None

        self.navmesh_dirty = False

        # mouse raycast visualization
        self.mouse_cast_results = None
        self.mouse_cast_has_hits = False

        self.ao_link_map = None

        self.agent_start_location = mn.Vector3(-5.7, 0.0, -4.0)
        self.ao_place_location = mn.Vector3(-7.7, 1.0, -4.0)

        # Load simulatioon scene
        self.reconfigure_sim()

        # load file holding urdf filenames needing handles
        print(
            f"URDF hashes file name : {URDF_FILES} | No-link URDFS file name : {NOLINK_URDF_FILES}"
        )
        # Build a List of URDF hash names loaded from disk, where each entry
        # is a dictionary of hash, status, and notes (if present).
        # As URDFs are completed their status is changed from "unfinished" to "done"
        self.urdf_hash_names_list = self.load_urdf_filenames()
        # Start with first idx in self.urdf_hash_names_list
        self.urdf_edit_hash_idx = self._get_next_hash_idx(
            start_idx=0, forward=True, status="unfinished"
        )

        # load markersets for every object and ao into a cache
        task_names_set = {"faucets", "handles"}
        self.markersets_util = MarkerSetsEditor(self.sim, task_names_set)
        self.markersets_util.set_current_taskname("handles")

        # Editing for object selection
        self.obj_editor = ObjectEditor(self.sim)
        # Set default editing to rotation
        self.obj_editor.set_edit_mode_rotate()
        # Force save of urdf hash to NOLINK_URDF_FILES file
        self.force_urdf_notes_save = False

        # Load first object to place markers on
        self.load_urdf_obj()

        # Semantics
        self.dbg_semantics = SemanticDisplay(self.sim)

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

    def _get_next_hash_idx(self, start_idx: int, forward: bool, status: str):
        if forward:
            iter_range = range(start_idx, len(self.urdf_hash_names_list))
        else:
            iter_range = range(start_idx, -1, -1)

        for i in iter_range:
            if self.urdf_hash_names_list[i]["status"] == status:
                return i
        print(
            f"No {status} hashes left to be found {('forward of' if forward else 'backward from')} starting idx {start_idx}."
        )
        return -1

    def _set_hash_list_status(self, hash_idx: int, is_finished: bool):
        pass

    def load_urdf_filenames(self):
        # list of dicts holding hash, status, notes
        urdf_hash_names_list: List[Dict[str:str]] = []
        # File names of all URDFs
        with open(URDF_FILES, "r") as f:
            for line in f.readlines():
                vals = line.split(",", maxsplit=2)
                finished = "done" if vals[1].strip().lower() == "true" else "unfinished"
                new_dict = {"hash": vals[0].strip(), "status": finished}
                if len(vals) > 2:
                    new_dict["notes"] = vals[2].strip()
                else:
                    new_dict["notes"] = ""
                urdf_hash_names_list.append(new_dict)

        return urdf_hash_names_list

    def update_nolink_file(self, save_no_markers: bool):
        # remove urdf hash from NOLINK_URDF_FILES if it has links, add it if it does not
        urdf_hash: str = self.urdf_hash_names_list[self.urdf_edit_hash_idx]["hash"]
        # preserve all text in file after comma
        urdf_nolink_hash_names: Dict[str, str] = {}
        with open(NOLINK_URDF_FILES, "r") as f:
            for line in f.readlines():
                if len(line.strip()) == 0:
                    continue
                vals = line.split(",", maxsplit=1)
                # hash is idx0; notes is idx1
                urdf_nolink_hash_names[vals[0]] = vals[1].strip()
        if save_no_markers:
            # it has no markers or we are forcing a save, so add it to record if it isn't already there
            if urdf_hash not in urdf_nolink_hash_names:
                # add empty string
                urdf_nolink_hash_names[urdf_hash] = ""
        else:
            # if it has markers now, remove it from record
            urdf_nolink_hash_names.pop(urdf_hash, None)
        # save no-link status results
        with open(NOLINK_URDF_FILES, "w") as f:
            for file_hash, notes in urdf_nolink_hash_names.items():
                f.write(f"{file_hash}, {notes}\n")

    def save_urdf_filesnames(self):
        # save current state of URDF files
        with open(URDF_FILES, "w") as f:
            for urdf_entry in self.urdf_hash_names_list:
                notes = "" if len(urdf_entry) > 2 else f", {urdf_entry['notes']}"
                status = urdf_entry["status"].strip().lower() == "done"
                f.write(f"{urdf_entry['hash']}, {status}{notes}\n")

    def _delete_sel_obj_update_nolink_file(self, sel_obj_hash: str):
        sel_obj = self.obj_editor.get_target_sel_obj()
        if sel_obj is None:
            sel_obj = hsim_physics.get_obj_from_handle(sel_obj_hash)

        save_no_markers = (
            self.force_urdf_notes_save
            or sel_obj.marker_sets.num_tasksets == 0
            or (not sel_obj.marker_sets.has_taskset("handles"))
        )
        print(
            f"Object {sel_obj.handle} has {sel_obj.marker_sets.num_tasksets} tasksets and Force save set to {self.force_urdf_notes_save} == Save as no marker urdf? {save_no_markers}"
        )
        # remove currently selected objects
        removed_obj_handles = self.obj_editor.remove_sel_objects()
        # should only have 1 handle
        for handle in removed_obj_handles:
            print(f"Removed {handle}")
        # finalize removal
        self.obj_editor.remove_all_objs()
        # update record of object hashes with/without markers with current file's state
        self.update_nolink_file(save_no_markers=save_no_markers)

    def load_urdf_obj(self):
        # Next object to be edited
        sel_obj_hash = self.urdf_hash_names_list[self.urdf_edit_hash_idx]["hash"]
        print(f"URDF hash we want : `{sel_obj_hash}`")
        # Load object into scene
        _, self.navmesh_dirty = self.obj_editor.load_from_substring(
            navmesh_dirty=self.navmesh_dirty,
            obj_substring=sel_obj_hash,
            build_loc=self.ao_place_location,
        )
        self.ao_link_map = hsim_physics.get_ao_link_id_map(self.sim)
        self.markersets_util.update_markersets()
        self.markersets_util.set_current_taskname("handles")

    def cycle_through_urdfs(self, shift_pressed: bool) -> None:
        # current object hash
        old_sel_obj_hash = self.urdf_hash_names_list[self.urdf_edit_hash_idx]["hash"]
        # Determine the status we are looking for when we search for the next desired index
        if shift_pressed:
            status = "done"
            start_idx = self.urdf_edit_hash_idx
        else:
            status = "unfinished"
            start_idx = self.urdf_edit_hash_idx
            # Moving forward - set current to finished
            self.urdf_hash_names_list[self.urdf_edit_hash_idx]["status"] = "done"

        # Get the idx of the next object we want to edit
        # Either the idx of the next record that is unfinished, or the most recent previous record that is done
        next_idx = self._get_next_hash_idx(
            start_idx=start_idx, forward=not shift_pressed, status=status
        )

        # If we don't have a valid next index then handle edge case
        if next_idx == -1:
            if not shift_pressed:
                # save current status
                self.save_urdf_filesnames()
                # moving forward - done!
                print(
                    f"Finished going through all {len(self.urdf_hash_names_list)} loaded urdf files. Exiting."
                )
                self.exit_event(Application.ExitEvent)
            else:
                # moving backward, at the start of all the objects so nowhere to go
                print(f"No objects previous to current object {old_sel_obj_hash}.")
                return
        # set edited state in urdf file list appropriately if moving backward, set previous to unfinished, leave current unfinished
        if shift_pressed:
            # Moving backward - set previous to unfinished, leave current unchanged
            self.urdf_hash_names_list[next_idx]["status"] = "unfinished"

        # remove the current selected object and update the no_link file
        self._delete_sel_obj_update_nolink_file(sel_obj_hash=old_sel_obj_hash)

        # Update the current edit hash idx
        self.urdf_edit_hash_idx = next_idx
        # save current status
        self.save_urdf_filesnames()
        print(f"URDF hash we just finished : `{old_sel_obj_hash}`")
        # load next urdf object
        self.load_urdf_obj()
        # reset force save to False for each object
        self.force_urdf_notes_save = False

    def draw_contact_debug(self, debug_line_render: Any):
        """
        This method is called to render a debug line overlay displaying active contact points and normals.
        Yellow lines show the contact distance along the normal and red lines show the contact normal at a fixed length.
        """
        yellow = mn.Color4.yellow()
        red = mn.Color4.red()
        cps = self.sim.get_physics_contact_points()
        debug_line_render.set_line_width(1.5)
        camera_position = self.render_camera.render_camera.node.absolute_translation
        # only showing active contacts
        active_contacts = (x for x in cps if x.is_active)
        for cp in active_contacts:
            # red shows the contact distance
            debug_line_render.draw_transformed_line(
                cp.position_on_b_in_ws,
                cp.position_on_b_in_ws
                + cp.contact_normal_on_b_in_ws * -cp.contact_distance,
                red,
            )
            # yellow shows the contact normal at a fixed length for visualization
            debug_line_render.draw_transformed_line(
                cp.position_on_b_in_ws,
                # + cp.contact_normal_on_b_in_ws * cp.contact_distance,
                cp.position_on_b_in_ws + cp.contact_normal_on_b_in_ws * 0.1,
                yellow,
            )
            debug_line_render.draw_circle(
                translation=cp.position_on_b_in_ws,
                radius=0.005,
                color=yellow,
                normal=camera_position - cp.position_on_b_in_ws,
            )

    def debug_draw(self):
        """
        Additional draw commands to be called during draw_event.
        """

        debug_line_render = self.sim.get_debug_line_render()
        if self.debug_bullet_draw:
            render_cam = self.render_camera.render_camera
            proj_mat = render_cam.projection_matrix.__matmul__(render_cam.camera_matrix)
            self.sim.physics_debug_draw(proj_mat)

        if self.contact_debug_draw:
            self.draw_contact_debug(debug_line_render)
        # draw semantic information
        self.dbg_semantics.draw_region_debug(debug_line_render=debug_line_render)
        # draw markersets information
        if self.markersets_util.marker_sets_per_obj is not None:
            self.markersets_util.draw_marker_sets_debug(
                debug_line_render,
                self.render_camera.render_camera.node.absolute_translation,
            )

        self.obj_editor.draw_selected_objects(debug_line_render)
        # mouse raycast circle
        # This is confusing with the marker placement
        # if self.mouse_cast_has_hits:
        #     debug_line_render.draw_circle(
        #         translation=self.mouse_cast_results.hits[0].point,
        #         radius=0.005,
        #         color=mn.Color4(mn.Vector3(1.0), 1.0),
        #         normal=self.mouse_cast_results.hits[0].normal,
        #     )

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

        # Occasionally a frame will pass quicker than 1/60 seconds
        if self.time_since_last_simulation >= 1.0 / self.fps:
            if self.simulating or self.simulate_single_step:
                self.sim.step_world(1.0 / self.fps)
                self.simulate_single_step = False
                if simulation_call is not None:
                    simulation_call()
            if global_call is not None:
                global_call()
            if self.navmesh_dirty:
                self.navmesh_config_and_recompute()
                self.navmesh_dirty = False

            # reset time_since_last_simulation, accounting for potential overflow
            self.time_since_last_simulation = math.fmod(
                self.time_since_last_simulation, 1.0 / self.fps
            )

        keys = active_agent_id_and_sensor_name

        if self.enable_batch_renderer:
            self.render_batch()
        else:
            self.sim._Simulator__sensors[keys[0]][keys[1]].draw_observation()
            agent = self.sim.get_agent(keys[0])
            self.render_camera = agent.scene_node.node_sensor_suite.get(keys[1])
            self.debug_draw()
            self.render_camera.render_target.blit_rgba_to_default()

        # draw CPU/GPU usage data and other info to the app window
        mn.gl.default_framebuffer.bind()
        self.draw_text(self.render_camera.specification())

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

        if self.enable_batch_renderer:
            self.cfg.enable_batch_renderer = True
            self.cfg.sim_cfg.create_renderer = False
            self.cfg.sim_cfg.enable_gfx_replay_save = True

        if self.sim_settings["use_default_lighting"]:
            logger.info("Setting default lighting override for scene.")
            self.cfg.sim_cfg.override_scene_light_defaults = True
            self.cfg.sim_cfg.scene_light_setup = habitat_sim.gfx.DEFAULT_LIGHTING_KEY

        if self.sim is None:
            self.tiled_sims = []
            for _i in range(self.num_env):
                self.tiled_sims.append(habitat_sim.Simulator(self.cfg))
            self.sim = self.tiled_sims[0]
        else:  # edge case
            for i in range(self.num_env):
                if (
                    self.tiled_sims[i].config.sim_cfg.scene_id
                    == self.cfg.sim_cfg.scene_id
                ):
                    # we need to force a reset, so change the internal config scene name
                    self.tiled_sims[i].config.sim_cfg.scene_id = "NONE"
                self.tiled_sims[i].reconfigure(self.cfg)

        # post reconfigure
        self.default_agent = self.sim.get_agent(self.agent_id)

        new_agent_state = habitat_sim.AgentState()
        new_agent_state.position = self.agent_start_location
        new_agent_state.rotation = quat_from_angle_axis(
            0.5 * np.pi,
            np.array([0, 1, 0]),
        )
        self.default_agent.set_state(new_agent_state)

        self.render_camera = self.default_agent.scene_node.node_sensor_suite.get(
            "color_sensor"
        )

        # set sim_settings scene name as actual loaded scene
        self.sim_settings["scene"] = self.sim.curr_scene_name

        # Initialize replay renderer
        if self.enable_batch_renderer and self.replay_renderer is None:
            self.replay_renderer_cfg = ReplayRendererConfiguration()
            self.replay_renderer_cfg.num_environments = self.num_env
            self.replay_renderer_cfg.standalone = (
                False  # Context is owned by the GLFW window
            )
            self.replay_renderer_cfg.sensor_specifications = self.cfg.agents[
                self.agent_id
            ].sensor_specifications
            self.replay_renderer_cfg.gpu_device_id = self.cfg.sim_cfg.gpu_device_id
            self.replay_renderer_cfg.force_separate_semantic_scene_graph = False
            self.replay_renderer_cfg.leave_context_with_background_renderer = False
            self.replay_renderer = ReplayRenderer.create_batch_replay_renderer(
                self.replay_renderer_cfg
            )
            # Pre-load composite files
            if sim_settings["composite_files"] is not None:
                for composite_file in sim_settings["composite_files"]:
                    self.replay_renderer.preload_file(composite_file)

        self.ao_link_map = hsim_physics.get_ao_link_id_map(self.sim)
        # check that clearing joint positions on save won't corrupt the content
        for ao in (
            self.sim.get_articulated_object_manager()
            .get_objects_by_handle_substring()
            .values()
        ):
            for joint_val in ao.joint_positions:
                assert (
                    joint_val == 0
                ), "If this fails, there are non-zero joint positions in the scene_instance or default pose. Export with 'i' will clear these."

        Timer.start()
        self.step = -1

    def render_batch(self):
        """
        This method updates the replay manager with the current state of environments and renders them.
        """
        for i in range(self.num_env):
            # Apply keyframe
            keyframe = self.tiled_sims[i].gfx_replay_manager.extract_keyframe()
            self.replay_renderer.set_environment_keyframe(i, keyframe)
            # Copy sensor transforms
            sensor_suite = self.tiled_sims[i]._sensors
            for sensor_uuid, sensor in sensor_suite.items():
                transform = sensor._sensor_object.node.absolute_transformation()
                self.replay_renderer.set_sensor_transform(i, sensor_uuid, transform)
            # Render
            self.replay_renderer.render(mn.gl.default_framebuffer)

    def move_and_look(self, repetitions: int) -> None:
        """
        This method is called continuously with `self.draw_event` to monitor
        any changes in the movement keys map `Dict[KeyEvent.key, Bool]`.
        When a key in the map is set to `True` the corresponding action is taken.
        """
        if repetitions == 0:
            return

        agent = self.sim.agents[self.agent_id]
        press: Dict[Application.Key.key, bool] = self.pressed
        act: Dict[Application.Key.key, str] = self.key_to_action

        action_queue: List[str] = [act[k] for k, v in press.items() if v]

        for _ in range(int(repetitions)):
            [agent.act(x) for x in action_queue]

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
        pressed = Application.Key
        mod = Application.Modifier

        shift_pressed = bool(event.modifiers & mod.SHIFT)
        alt_pressed = bool(event.modifiers & mod.ALT)
        # warning: ctrl doesn't always pass through with other key-presses

        if key == pressed.ESC:
            event.accepted = True
            self.exit_event(Application.ExitEvent)
            return
        elif key == pressed.TAB:
            self.cycle_through_urdfs(shift_pressed=shift_pressed)

        elif key == pressed.SPACE:
            if not self.sim.config.sim_cfg.enable_physics:
                logger.warn("Warning: physics was not enabled during setup")
            else:
                self.simulating = not self.simulating
                logger.info(f"Command: physics simulating set to {self.simulating}")

        elif key == pressed.PERIOD:
            if self.simulating:
                logger.warn("Warning: physics simulation already running")
            else:
                self.simulate_single_step = True
                logger.info("Command: physics step taken")

        elif key == pressed.COMMA:
            self.debug_bullet_draw = not self.debug_bullet_draw
            logger.info(f"Command: toggle Bullet debug draw: {self.debug_bullet_draw}")

        elif key == pressed.B:
            # Save all markersets that have been changed
            self.markersets_util.save_all_dirty_markersets()

        elif key == pressed.C:
            self.contact_debug_draw = not self.contact_debug_draw
            log_str = f"Command: toggle contact debug draw: {self.contact_debug_draw}"
            if self.contact_debug_draw:
                # perform a discrete collision detection pass and enable contact debug drawing to visualize the results
                # TODO: add a nice log message with concise contact pair naming.
                log_str = f"{log_str}: performing discrete collision detection and visualize active contacts."
                self.sim.perform_discrete_collision_detection()
            logger.info(log_str)
        elif key == pressed.Q:
            # rotate selected object(s) to left
            self.navmesh_dirty = self.obj_editor.edit_left(self.navmesh_dirty)
        elif key == pressed.E:
            # rotate selected object(s) right
            self.navmesh_dirty = self.obj_editor.edit_right(self.navmesh_dirty)
        elif key == pressed.R:
            # cycle through rotation amount
            self.obj_editor.change_edit_vals(toggle=shift_pressed)
        elif key == pressed.F:
            self.force_urdf_notes_save = not self.force_urdf_notes_save
            print(
                f"Force save of hash to URDF notes file set to {self.force_urdf_notes_save}"
            )
        elif key == pressed.G:
            # If shift pressed then open, otherwise close
            # If alt pressed then selected, otherwise all
            self.obj_editor.set_ao_joint_states(
                do_open=shift_pressed, selected=alt_pressed
            )
            if not shift_pressed:
                # if closing then redo navmesh
                self.navmesh_config_and_recompute()
        elif key == pressed.H:
            self.print_help_text()
        elif key == pressed.K:
            # Cyle through semantics display
            info_str = self.dbg_semantics.cycle_semantic_region_draw()
            logger.info(info_str)

        elif key == pressed.M:
            self.cycle_mouse_mode()
            logger.info(f"Command: mouse mode set to {self.mouse_interaction}")

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

        elif key == pressed.V:
            # self.invert_gravity()
            # logger.info("Command: gravity inverted")
            # Duplicate all the selected objects and place them in the scene
            # or inject a new object by queried handle substring in front of
            # the agent if no objects selected

            new_obj_list, self.navmesh_dirty = self.obj_editor.build_objects(
                self.navmesh_dirty,
                build_loc=self.ao_place_location,
            )
            if len(new_obj_list) == 0:
                print("Failed to add any new objects.")
            else:
                print(f"Finished adding {len(new_obj_list)} object(s).")
                self.ao_link_map = hsim_physics.get_ao_link_id_map(self.sim)
                self.markersets_util.update_markersets()

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

    def calc_mouse_cast_results(self, screen_location: mn.Vector3) -> None:
        render_camera = self.render_camera.render_camera
        ray = render_camera.unproject(self.get_mouse_position(screen_location))
        mouse_cast_results = self.sim.cast_ray(ray=ray)
        self.mouse_cast_has_hits = (
            mouse_cast_results is not None and mouse_cast_results.has_hits()
        )
        self.mouse_cast_results = mouse_cast_results

    def is_left_mse_btn(
        self, event: Union[Application.PointerEvent, Application.PointerMoveEvent]
    ) -> bool:
        """
        Returns whether the left mouse button is pressed
        """
        if isinstance(event, Application.PointerEvent):
            return event.pointer == Application.Pointer.MOUSE_LEFT
        elif isinstance(event, Application.PointerMoveEvent):
            return event.pointers & Application.Pointer.MOUSE_LEFT
        else:
            return False

    def is_right_mse_btn(
        self, event: Union[Application.PointerEvent, Application.PointerMoveEvent]
    ) -> bool:
        """
        Returns whether the right mouse button is pressed
        """
        if isinstance(event, Application.PointerEvent):
            return event.pointer == Application.Pointer.MOUSE_RIGHT
        elif isinstance(event, Application.PointerMoveEvent):
            return event.pointers & Application.Pointer.MOUSE_RIGHT
        else:
            return False

    def pointer_move_event(self, event: Application.PointerMoveEvent) -> None:
        """
        Handles `Application.PointerMoveEvent`. When in LOOK mode, enables the left
        mouse button to steer the agent's facing direction.
        """

        # if interactive mode -> LOOK MODE
        if self.is_left_mse_btn(event) and self.mouse_interaction == MouseMode.LOOK:
            agent = self.sim.agents[self.agent_id]
            delta = self.get_mouse_position(event.relative_position) / 2
            action = habitat_sim.agent.ObjectControls()
            act_spec = habitat_sim.agent.ActuationSpec

            # left/right on agent scene node
            action(agent.scene_node, "turn_right", act_spec(delta.x))

            # up/down on cameras' scene nodes
            action = habitat_sim.agent.ObjectControls()
            sensors = list(self.default_agent.scene_node.subtree_sensors.values())
            [action(s.object, "look_down", act_spec(delta.y), False) for s in sensors]

        self.previous_mouse_point = self.get_mouse_position(event.position)
        self.redraw()
        event.accepted = True

    def pointer_press_event(self, event: Application.PointerEvent) -> None:
        """
        Handles `Application.PointerEvent`. When in MARKER mode :
        LEFT CLICK : places a marker at mouse position on targeted object if not the stage
        RIGHT CLICK : removes the closest marker to mouse position on targeted object
        """
        physics_enabled = self.sim.get_physics_simulation_library()
        is_left_mse_btn = self.is_left_mse_btn(event)
        is_right_mse_btn = self.is_right_mse_btn(event)
        mod = Application.Modifier
        shift_pressed = bool(event.modifiers & mod.SHIFT)
        # alt_pressed = bool(event.modifiers & mod.ALT)
        self.calc_mouse_cast_results(event.position)

        if physics_enabled and self.mouse_cast_has_hits:
            # If look enabled
            if self.mouse_interaction == MouseMode.LOOK:
                mouse_cast_results = self.mouse_cast_results
                if is_right_mse_btn:
                    # Find object being clicked
                    obj_found = False
                    obj = None
                    # find first non-stage object
                    hit_idx = 0
                    while hit_idx < len(mouse_cast_results.hits) and not obj_found:
                        self.last_hit_details = mouse_cast_results.hits[hit_idx]
                        hit_obj_id = mouse_cast_results.hits[hit_idx].object_id
                        obj = hsim_physics.get_obj_from_id(self.sim, hit_obj_id)
                        if obj is None:
                            hit_idx += 1
                        else:
                            obj_found = True
                    if obj_found:
                        print(
                            f"Object: {obj.handle} is {'Articlated' if obj.is_articulated else 'Rigid'} Object at {obj.translation}"
                        )
                    else:
                        print("This is the stage.")

                    if not shift_pressed:
                        # clear all selected objects and set to found obj
                        self.obj_editor.set_sel_obj(obj)
                    elif obj_found:
                        # add or remove object from selected objects, depending on whether it is already selected or not
                        self.obj_editor.toggle_sel_obj(obj)
            # else if marker enabled
            elif self.mouse_interaction == MouseMode.MARKER:
                # hit_info = self.mouse_cast_results.hits[0]
                sel_obj = self.markersets_util.place_marker_at_hit_location(
                    self.mouse_cast_results.hits[0],
                    self.ao_link_map,
                    is_left_mse_btn,
                )
                # clear all selected objects and set to found obj
                self.obj_editor.set_sel_obj(sel_obj)

        self.previous_mouse_point = self.get_mouse_position(event.position)
        self.redraw()
        event.accepted = True

    def scroll_event(self, event: Application.ScrollEvent) -> None:
        """
        Handles `Application.ScrollEvent`. When in LOOK mode, enables camera
        zooming (fine-grained zoom using shift) When in MARKER mode, wheel cycles through available taskset names
        """
        scroll_mod_val = (
            event.offset.y
            if abs(event.offset.y) > abs(event.offset.x)
            else event.offset.x
        )
        if not scroll_mod_val:
            return

        # use shift to scale action response
        mod = Application.Modifier
        shift_pressed = bool(event.modifiers & mod.SHIFT)
        # alt_pressed = bool(event.modifiers & mod.ALT)
        # ctrl_pressed = bool(event.modifiers & mod.CTRL)

        # if interactive mode is False -> LOOK MODE
        if self.mouse_interaction == MouseMode.LOOK:
            # use shift for fine-grained zooming
            mod_val = 1.01 if shift_pressed else 1.1
            mod = mod_val if scroll_mod_val > 0 else 1.0 / mod_val
            cam = self.render_camera
            cam.zoom(mod)
            self.redraw()

        elif self.mouse_interaction == MouseMode.MARKER:
            self.markersets_util.cycle_current_taskname(scroll_mod_val > 0)
        self.redraw()
        event.accepted = True

    def pointer_release_event(self, event: Application.PointerEvent) -> None:
        """
        Release any existing constraints.
        """
        event.accepted = True

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
            self.mouse_interaction = MouseMode.MARKER
        elif self.mouse_interaction == MouseMode.MARKER:
            self.mouse_interaction = MouseMode.LOOK

    def navmesh_config_and_recompute(self) -> None:
        """
        This method is setup to be overridden in for setting config accessibility
        in inherited classes.
        """
        if self.cfg.sim_cfg.scene_id.lower() == "none":
            return
        self.navmesh_settings = habitat_sim.NavMeshSettings()
        self.navmesh_settings.set_defaults()
        self.navmesh_settings.agent_height = self.cfg.agents[self.agent_id].height
        self.navmesh_settings.agent_radius = self.cfg.agents[self.agent_id].radius
        self.navmesh_settings.include_static_objects = True
        self.sim.recompute_navmesh(
            self.sim.pathfinder,
            self.navmesh_settings,
        )

    def exit_event(self, event: Application.ExitEvent):
        """
        Overrides exit_event to properly close the Simulator before exiting the
        application.
        """
        for i in range(self.num_env):
            self.tiled_sims[i].close(destroy=True)
            event.accepted = True
        exit(0)

    def draw_text(self, sensor_spec):
        # make magnum text background transparent for text
        mn.gl.Renderer.enable(mn.gl.Renderer.Feature.BLENDING)
        mn.gl.Renderer.set_blend_function(
            mn.gl.Renderer.BlendFunction.ONE,
            mn.gl.Renderer.BlendFunction.ONE_MINUS_SOURCE_ALPHA,
        )

        self.shader.bind_vector_texture(self.glyph_cache.texture)
        self.shader.transformation_projection_matrix = self.window_text_transform
        self.shader.color = [1.0, 1.0, 1.0]

        sensor_type_string = str(sensor_spec.sensor_type.name)
        sensor_subtype_string = str(sensor_spec.sensor_subtype.name)
        if self.mouse_interaction == MouseMode.LOOK:
            mouse_mode_string = "LOOK"
        elif self.mouse_interaction == MouseMode.MARKER:
            mouse_mode_string = "MARKER"
        edit_string = self.obj_editor.edit_disp_str()
        self.window_text.render(
            f"""
{self.fps} FPS
Sensor Type: {sensor_type_string}
Sensor Subtype: {sensor_subtype_string}
{edit_string}
Selected MarkerSets TaskSet name : {self.markersets_util.get_current_taskname()}
Mouse Interaction Mode: {mouse_mode_string}
FORCE SAVE URDF HASH IN NOTES FILE : {self.force_urdf_notes_save}
            """
        )
        self.shader.draw(self.window_text.mesh)

        # Disable blending for text
        mn.gl.Renderer.disable(mn.gl.Renderer.Feature.BLENDING)

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

In MARKER mode :
    LEFT CLICK : Add a marker to the target object at the mouse location, if not the stage
    RIGHT CLICK : Remove the closest marker to the mouse location on the target object

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
    'g' :       Modify AO link states :
                    (+SHIFT) : Open Selected AO
                    (-SHIFT) : Close Selected AO
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
    MARKER = 2


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
        Starts timer and resets previous frame time to the start time.
        """
        Timer.running = True
        Timer.start_time = time.time()
        Timer.prev_frame_time = Timer.start_time
        Timer.prev_frame_duration = 0.0

    @staticmethod
    def stop() -> None:
        """
        Stops timer and erases any previous time data, resetting the timer.
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
        default="./data/test_assets/scenes/simple_room.glb",
        type=str,
        help='scene/stage file to load (default: "./data/test_assets/scenes/simple_room.glb")',
    )
    parser.add_argument(
        "--dataset",
        default="default",
        type=str,
        metavar="DATASET",
        help='dataset configuration file to use (default: "default")',
    )
    parser.add_argument(
        "--disable-physics",
        action="store_true",
        help="disable physics simulation (default: False)",
    )
    parser.add_argument(
        "--use-default-lighting",
        action="store_true",
        help="Override configured lighting to use default lighting for the stage.",
    )
    parser.add_argument(
        "--hbao",
        action="store_true",
        help="Enable horizon-based ambient occlusion, which provides soft shadows in corners and crevices.",
    )
    parser.add_argument(
        "--enable-batch-renderer",
        action="store_true",
        help="Enable batch rendering mode. The number of concurrent environments is specified with the num-environments parameter.",
    )
    parser.add_argument(
        "--num-environments",
        default=1,
        type=int,
        help="Number of concurrent environments to batch render. Note that only the first environment simulates physics and can be controlled.",
    )
    parser.add_argument(
        "--composite-files",
        type=str,
        nargs="*",
        help="Composite files that the batch renderer will use in-place of simulation assets to improve memory usage and performance. If none is specified, the original scene files will be loaded from disk.",
    )
    parser.add_argument(
        "--width",
        default=800,
        type=int,
        help="Horizontal resolution of the window.",
    )
    parser.add_argument(
        "--height",
        default=600,
        type=int,
        help="Vertical resolution of the window.",
    )

    args = parser.parse_args()

    if args.num_environments < 1:
        parser.error("num-environments must be a positive non-zero integer.")
    if args.width < 1:
        parser.error("width must be a positive non-zero integer.")
    if args.height < 1:
        parser.error("height must be a positive non-zero integer.")

    # Setting up sim_settings
    sim_settings: Dict[str, Any] = default_sim_settings
    sim_settings["scene"] = args.scene
    sim_settings["scene_dataset_config_file"] = args.dataset
    sim_settings["enable_physics"] = not args.disable_physics
    sim_settings["use_default_lighting"] = args.use_default_lighting
    sim_settings["enable_batch_renderer"] = args.enable_batch_renderer
    sim_settings["num_environments"] = args.num_environments
    sim_settings["composite_files"] = args.composite_files
    sim_settings["window_width"] = args.width
    sim_settings["window_height"] = args.height
    sim_settings["default_agent_navmesh"] = False
    sim_settings["enable_hbao"] = args.hbao

    # start the application
    HabitatSimInteractiveViewer(sim_settings).exec()
