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
from typing import Any, Callable, Dict, List, Optional, Tuple, Union

flags = sys.getdlopenflags()
sys.setdlopenflags(flags | ctypes.RTLD_GLOBAL)

import magnum as mn
from magnum import shaders, text
from magnum.platform.glfw import Application

import habitat_sim
from habitat_sim import ReplayRenderer, ReplayRendererConfiguration
from habitat_sim.logging import LoggingContext, logger
from habitat_sim.utils.classes import ObjectEditor, SemanticDisplay
from habitat_sim.utils.namespace import hsim_physics
from habitat_sim.utils.settings import default_sim_settings, make_cfg

# This class is dependent on hab-lab
from habitat_sim.utils.sim_utils import SpotAgent


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
            key.Q: False,
            key.E: False,
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
        # whether to render window text or not
        self.do_draw_text = True

        # make magnum text background transparent
        mn.gl.Renderer.enable(mn.gl.Renderer.Feature.BLENDING)
        mn.gl.Renderer.set_blend_function(
            mn.gl.Renderer.BlendFunction.ONE,
            mn.gl.Renderer.BlendFunction.ONE_MINUS_SOURCE_ALPHA,
        )
        mn.gl.Renderer.set_blend_equation(
            mn.gl.Renderer.BlendEquation.ADD, mn.gl.Renderer.BlendEquation.ADD
        )

        # variables that track app data and CPU/GPU usage
        self.num_frames_to_track = 60

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
        self.removed_clutter: Dict[str, str] = {}

        self.navmesh_dirty = False
        self.removed_objects_debug_frames = []

        # mouse raycast visualization
        self.mouse_cast_results = None
        self.mouse_cast_has_hits = False

        self.reconfigure_sim()

        # Editing
        self.obj_editor = ObjectEditor(self.sim)

        # Semantics
        self.dbg_semantics = SemanticDisplay(self.sim)

        # create spot right after reconfigure
        self.spot_agent = SpotAgent(self.sim)
        # set for spot's radius
        self.cfg.agents[self.agent_id].radius = 0.3

        # compute NavMesh if not already loaded by the scene.
        if self.cfg.sim_cfg.scene_id.lower() != "none":
            self.navmesh_config_and_recompute()

        self.spot_agent.place_on_navmesh()

        self.time_since_last_simulation = 0.0
        LoggingContext.reinitialize_from_env()
        logger.setLevel("INFO")
        self.print_help_text()

    def draw_removed_objects_debug_frames(self):
        """
        Draw debug frames for all the recently removed objects.
        """
        for trans, aabb in self.removed_objects_debug_frames:
            dblr = self.sim.get_debug_line_render()
            dblr.push_transform(trans)
            dblr.draw_box(aabb.min, aabb.max, mn.Color4.red())
            dblr.pop_transform()

    def remove_outdoor_objects(self):
        """
        Check all object instance and remove those which are marked outdoors.
        """
        self.removed_objects_debug_frames = []
        rom = self.sim.get_rigid_object_manager()
        for obj in rom.get_objects_by_handle_substring().values():
            if self.obj_is_outdoor(obj):
                self.removed_objects_debug_frames.append(
                    (obj.transformation, obj.root_scene_node.cumulative_bb)
                )
                rom.remove_object_by_id(obj.object_id)

    def obj_is_outdoor(self, obj):
        """
        Check if an object is outdoors or not by raycasting upwards.
        """
        up = mn.Vector3(0, 1.0, 0)
        ray_results = self.sim.cast_ray(habitat_sim.geo.Ray(obj.translation, up))
        if ray_results.has_hits():
            for hit in ray_results.hits:
                if hit.object_id == obj.object_id:
                    continue
                return False

        # no hits, so outdoors
        return True

    def draw_contact_debug(self, debug_line_render: Any):
        """
        This method is called to render a debug line overlay displaying active contact points and normals.
        Red lines show the contact distance along the normal and yellow lines show the contact normal at a fixed length.
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
        render_cam = self.render_camera.render_camera
        if self.debug_bullet_draw:
            proj_mat = render_cam.projection_matrix.__matmul__(render_cam.camera_matrix)
            self.sim.physics_debug_draw(proj_mat)
        if self.contact_debug_draw:
            self.draw_contact_debug(debug_line_render)
        # draw semantic information
        self.dbg_semantics.draw_region_debug(debug_line_render=debug_line_render)

        if self.last_hit_details is not None:
            debug_line_render.draw_circle(
                translation=self.last_hit_details.point,
                radius=0.02,
                normal=self.last_hit_details.normal,
                color=mn.Color4.yellow(),
                num_segments=12,
            )
        # draw object-related visualizations
        self.obj_editor.draw_obj_vis(
            camera_trans=render_cam.node.absolute_translation,
            debug_line_render=debug_line_render,
        )
        # mouse raycast circle
        if self.mouse_cast_has_hits:
            debug_line_render.draw_circle(
                translation=self.mouse_cast_results.hits[0].point,
                radius=0.005,
                color=mn.Color4(mn.Vector3(1.0), 1.0),
                normal=self.mouse_cast_results.hits[0].normal,
            )

        self.draw_removed_objects_debug_frames()

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
        # move agent camera based on input relative to spot
        self.spot_agent.set_agent_camera_transform(self.default_agent.scene_node)

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
        if self.do_draw_text:
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

        # #resave scene instance
        # self.sim.save_current_scene_config(overwrite=True)
        # sys. exit()

        # post reconfigure
        self.default_agent = self.sim.get_agent(self.agent_id)
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
        # avoids unnecessary updates to grabber's object position
        if repetitions == 0:
            return

        key = Application.Key
        press: Dict[Application.Key.key, bool] = self.pressed
        # Set the spot up to move
        self.spot_agent.move_spot(
            move_fwd=press[key.W],
            move_back=press[key.S],
            move_up=press[key.Z],
            move_down=press[key.X],
            slide_left=press[key.Q],
            slide_right=press[key.E],
            turn_left=press[key.A],
            turn_right=press[key.D],
        )

    def save_scene(self, event: Application.KeyEvent, exit_scene: bool):
        """
        Save current scene. Exit if shift is pressed
        """

        # Save spot's state and remove it
        self.spot_agent.cache_transform_and_remove()

        # Save scene
        self.obj_editor.save_current_scene()

        # save clutter
        if len(self.removed_clutter) > 0:
            with open("removed_clutter.txt", "a") as f:
                for obj_name in self.removed_clutter:
                    f.write(obj_name + "\n")
            # clear clutter
            self.removed_clutter: Dict[str, str] = {}
        # whether to exit scene
        if exit_scene:
            event.accepted = True
            self.exit_event(Application.ExitEvent)
            return
        # Restore spot at previous location
        self.spot_agent.restore_at_previous_loc()

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
            # If shift_pressed then exit without save
            if shift_pressed:
                event.accepted = True
                self.exit_event(Application.ExitEvent)
                return

            # Otherwise, save scene if it has been edited before exiting
            self.save_scene(event, exit_scene=True)
            return
        elif key == pressed.ZERO:
            # reset agent camera location
            self.spot_agent.init_spot_cam()

        elif key == pressed.ONE:
            # Toggle spot's clipping/restriction to navmesh
            self.spot_agent.toggle_clip()

        elif key == pressed.TWO:
            # Match target object's x dim
            self.navmesh_dirty = self.obj_editor.match_x_dim(self.navmesh_dirty)

        elif key == pressed.THREE:
            # Match target object's y dim
            self.navmesh_dirty = self.obj_editor.match_y_dim(self.navmesh_dirty)

        elif key == pressed.FOUR:
            # Match target object's z dim
            self.navmesh_dirty = self.obj_editor.match_z_dim(self.navmesh_dirty)

        elif key == pressed.FIVE:
            # Match target object's orientation
            self.navmesh_dirty = self.obj_editor.match_orientation(self.navmesh_dirty)

        elif key == pressed.SIX:
            # Select all items matching selected item. Shift to include all currently selected items
            self.obj_editor.select_all_matching_objects(only_matches=not shift_pressed)

        elif key == pressed.H:
            # Print help text to console
            self.print_help_text()

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

        elif key == pressed.LEFT:
            # Move or rotate selected object(s) left
            self.navmesh_dirty = self.obj_editor.edit_left(self.navmesh_dirty)

        elif key == pressed.RIGHT:
            # Move or rotate selected object(s) right
            self.navmesh_dirty = self.obj_editor.edit_right(self.navmesh_dirty)

        elif key == pressed.UP:
            # Move or rotate selected object(s) up
            self.navmesh_dirty = self.obj_editor.edit_up(
                self.navmesh_dirty, toggle=alt_pressed
            )

        elif key == pressed.DOWN:
            # Move or rotate selected object(s) down
            self.navmesh_dirty = self.obj_editor.edit_down(
                self.navmesh_dirty, toggle=alt_pressed
            )

        elif key == pressed.BACKSPACE or key == pressed.Y:
            # 'Remove' all selected objects by moving them out of view.
            # Removal only becomes permanent when scene is saved
            # If shift pressed, undo removals
            if shift_pressed:
                restored_obj_handles = self.obj_editor.restore_removed_objects()
                if key == pressed.Y:
                    for handle in restored_obj_handles:
                        obj_name = handle.split("/")[-1].split("_:")[0]
                        self.removed_clutter.pop(obj_name, None)
                # select restored objects
                self.obj_editor.sel_obj_list(restored_obj_handles)
            else:
                removed_obj_handles = self.obj_editor.remove_sel_objects()
                if key == pressed.Y:
                    for handle in removed_obj_handles:
                        # Mark removed clutter
                        obj_name = handle.split("/")[-1].split("_:")[0]
                        self.removed_clutter[obj_name] = ""
            self.navmesh_config_and_recompute()

        elif key == pressed.B:
            # Cycle through available edit amount values
            self.obj_editor.change_edit_vals(toggle=shift_pressed)

        elif key == pressed.C:
            # Display contacts
            self.contact_debug_draw = not self.contact_debug_draw
            log_str = f"Command: toggle contact debug draw: {self.contact_debug_draw}"
            if self.contact_debug_draw:
                # perform a discrete collision detection pass and enable contact debug drawing to visualize the results
                # TODO: add a nice log message with concise contact pair naming.
                log_str = f"{log_str}: performing discrete collision detection and visualize active contacts."
                self.sim.perform_discrete_collision_detection()
            logger.info(log_str)
        elif key == pressed.F:
            # find and remove duplicates
            self.obj_editor.handle_duplicate_objects(
                find_objs=(not shift_pressed),
                remove_dupes=shift_pressed,
                trans_eps=0.01,
            )
        elif key == pressed.G:
            # cycle through edit modes
            self.obj_editor.change_edit_mode(toggle=shift_pressed)

        elif key == pressed.I:
            # Save scene, exiting if shift has been pressed
            self.save_scene(event=event, exit_scene=shift_pressed)

        elif key == pressed.J:
            # If shift pressed then open, otherwise close
            # If alt pressed then selected, otherwise all
            self.obj_editor.set_ao_joint_states(
                do_open=shift_pressed, selected=alt_pressed
            )
            if not shift_pressed:
                # if closing then redo navmesh
                self.navmesh_config_and_recompute()

        elif key == pressed.K:
            # Cycle through semantics display
            info_str = self.dbg_semantics.cycle_semantic_region_draw()
            logger.info(info_str)
        elif key == pressed.L:
            # Cycle through types of objects to draw highlight box around - aos, rigids, both, none
            self.obj_editor.change_draw_box_types(toggle=shift_pressed)

        elif key == pressed.N:
            # (default) - toggle navmesh visualization
            # NOTE: (+ALT) - re-sample the agent position on the NavMesh
            # NOTE: (+SHIFT) - re-compute the NavMesh
            if alt_pressed:
                logger.info("Command: resample agent state from navmesh")
                self.spot_agent.place_on_navmesh()
            elif shift_pressed:
                logger.info("Command: recompute navmesh")
                self.navmesh_config_and_recompute()
            else:
                if self.sim.pathfinder.is_loaded:
                    self.sim.navmesh_visualization = not self.sim.navmesh_visualization
                    logger.info("Command: toggle navmesh")
                else:
                    logger.warning("Warning: recompute navmesh first")
        elif key == pressed.P:
            # Toggle whether showing performance data on screen or not
            self.do_draw_text = not self.do_draw_text

        elif key == pressed.T:
            self.remove_outdoor_objects()

        elif key == pressed.U:
            # Undo all edits on selected objects 1 step, or redo undone, if shift
            if shift_pressed:
                self.obj_editor.redo_sel_edits()
            else:
                self.obj_editor.undo_sel_edits()

        elif key == pressed.V:
            # Duplicate all the selected objects and place them in the scene
            # or inject a new object by queried handle substring in front of
            # the agent if no objects selected

            # Use shift to play object at most recent hit location, if it exists
            if shift_pressed and self.last_hit_details is not None:
                build_loc = self.last_hit_details.point
            else:
                build_loc = self.spot_agent.get_point_in_front(
                    disp_in_front=[1.5, 0.0, 0.0]
                )

            new_obj_list, self.navmesh_dirty = self.obj_editor.build_objects(
                self.navmesh_dirty,
                build_loc=build_loc,
            )
            if len(new_obj_list) == 0:
                print("Failed to add any new objects.")
            else:
                print(f"Finished adding {len(new_obj_list)} object(s).")

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
        mouse button to steer the agent's facing direction. When in GRAB mode,
        continues to update the grabber's object position with our agents position.
        """

        # if interactive mode -> LOOK MODE
        if self.is_left_mse_btn(event):
            shift_pressed = bool(event.modifiers & Application.Modifier.SHIFT)
            alt_pressed = bool(event.modifiers & Application.Modifier.ALT)
            self.spot_agent.mod_spot_cam(
                mse_rel_pos=event.relative_position,
                shift_pressed=shift_pressed,
                alt_pressed=alt_pressed,
            )

        self.previous_mouse_point = self.get_mouse_position(event.position)
        self.redraw()
        event.accepted = True

    def pointer_press_event(self, event: Application.PointerEvent) -> None:
        """
        Handles `Application.PointerEvent`. When in GRAB mode, click on
        objects to drag their position. (right-click for fixed constraints)
        """
        physics_enabled = self.sim.get_physics_simulation_library()
        # is_left_mse_btn = self.is_left_mse_btn(event)
        is_right_mse_btn = self.is_right_mse_btn(event)
        mod = Application.Modifier
        shift_pressed = bool(event.modifiers & mod.SHIFT)
        # alt_pressed = bool(event.modifiers & mod.ALT)
        self.calc_mouse_cast_results(event.position)

        # select an object with RIGHT-click
        if physics_enabled and self.mouse_cast_has_hits:
            mouse_cast_hit_results = self.mouse_cast_results.hits
            if is_right_mse_btn:
                # Find object being clicked
                obj_found = False
                obj = None
                # find first non-stage object
                hit_idx = 0
                while hit_idx < len(mouse_cast_hit_results) and not obj_found:
                    self.last_hit_details = mouse_cast_hit_results[hit_idx]
                    hit_obj_id = mouse_cast_hit_results[hit_idx].object_id
                    obj = hsim_physics.get_obj_from_id(self.sim, hit_obj_id)
                    if obj is None:
                        hit_idx += 1
                    else:
                        obj_found = True
                if obj_found:
                    print(
                        f"Object: {obj.handle} is {'Articulated' if obj.is_articulated else 'Rigid'} Object at {obj.translation}"
                    )
                else:
                    print("This is the stage.")

                if not shift_pressed:
                    # clear all selected objects and set to found obj
                    self.obj_editor.set_sel_obj(obj)
                elif obj_found:
                    # add or remove object from selected objects, depending on whether it is already selected or not
                    self.obj_editor.toggle_sel_obj(obj)

        self.previous_mouse_point = self.get_mouse_position(event.position)
        self.redraw()
        event.accepted = True

    def scroll_event(self, event: Application.ScrollEvent) -> None:
        """
        Handles `Application.ScrollEvent`. When in LOOK mode, enables camera
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
        mod = Application.Modifier
        shift_pressed = bool(event.modifiers & mod.SHIFT)
        alt_pressed = bool(event.modifiers & mod.ALT)
        # ctrl_pressed = bool(event.modifiers & mod.CTRL)

        # LOOK MODE
        # use shift for fine-grained zooming
        self.spot_agent.mod_spot_cam(
            scroll_mod_val=scroll_mod_val,
            shift_pressed=shift_pressed,
            alt_pressed=alt_pressed,
        )

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

    def navmesh_config_and_recompute(self) -> None:
        """
        This method is setup to be overridden in for setting config accessibility
        in inherited classes.
        """
        self.navmesh_settings = habitat_sim.NavMeshSettings()
        self.navmesh_settings.set_defaults()
        self.navmesh_settings.agent_height = self.cfg.agents[self.agent_id].height
        self.navmesh_settings.agent_radius = self.cfg.agents[self.agent_id].radius
        self.navmesh_settings.include_static_objects = True

        # first cache AO motion types and set to STATIC for navmesh
        ao_motion_types = []
        for ao in (
            self.sim.get_articulated_object_manager()
            .get_objects_by_handle_substring()
            .values()
        ):
            # ignore the robot
            if "hab_spot" not in ao.handle:
                ao_motion_types.append((ao, ao.motion_type))
                ao.motion_type = habitat_sim.physics.MotionType.STATIC

        self.sim.recompute_navmesh(self.sim.pathfinder, self.navmesh_settings)

        # reset AO motion types from cache
        for ao, ao_orig_motion_type in ao_motion_types:
            ao.motion_type = ao_orig_motion_type

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
        edit_string = self.obj_editor.edit_disp_str()
        self.window_text.render(
            f"""
{self.fps} FPS
Scene ID : {os.path.split(self.cfg.sim_cfg.scene_id)[1].split('.scene_instance')[0]}
Sensor Type: {sensor_type_string}
Sensor Subtype: {sensor_subtype_string}
{edit_string}
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
Welcome to the Habitat-sim Python Spot Viewer application!
=====================================================
Mouse Functions
----------------
    LEFT:
        Click and drag to rotate the view around Spot.
    RIGHT:
        Select an object(s) to modify. If multiple objects selected all will be modified equally
        (+SHIFT) add/remove object from selected set. Most recently selected object (with yellow box) will be target object.
    WHEEL:
        Zoom in and out on Spot view.
        (+ALT): Raise/Lower the camera's target above Spot.


Key Commands:
-------------
    esc:        Exit the application.
    'h':        Display this help message.
    'p':        Toggle the display of on-screen data

  Spot/Camera Controls:
    'wasd':     Move Spot's body forward/backward and rotate left/right.
    'qe':       Move Spot's body in strafe left/right.

    '0':        Reset the camera around Spot (after raising/lowering)
    '1' :       Disengage/re-engage the navmesh constraints (no-clip toggle). When toggling back on,
                before collision/navmesh is re-engaged, the closest point to the navmesh is searched
                for. If found, spot is snapped to it, but if not found, spot will stay in no-clip
                mode and a message will display.

  Scene Object Modification UI:
    'g' : Change Edit mode to either Move or Rotate the selected object(s)
    'b' (+ SHIFT) : Increment (Decrement) the current edit amounts.
    Editing :
    - With 1 or more objects selected:
      - When Move Object Edit mode is selected :
        - LEFT/RIGHT arrow keys: move the selected object(s) along global X axis.
        - UP/DOWN arrow keys: move the selected object(s) along global Z axis.
            (+ALT): move the selected object(s) up/down (global Y axis)
      - When Rotate Object Edit mode is selected :
        - LEFT/RIGHT arrow keys: rotate the selected object(s) around global Y axis.
        - UP/DOWN arrow keys: rotate the selected object(s) around global Z axis.
            (+ALT): rotate the selected object(s) around global X axis.
      - BACKSPACE: delete the selected object(s)
            'y': delete the selected object and record it as clutter.
      - Matching target selected object(s) (rendered with yellow box) specified dimension :
        - '2': all selected objects match selected 'target''s x value
        - '3': all selected objects match selected 'target''s y value
        - '4': all selected objects match selected 'target''s z value
        - '5': all selected objects match selected 'target''s orientation
      'u': Undo Edit
          - With an object selected: Undo a single modification step for the selected object(s)
              (+SHIFT) : redo modification to the selected object(s)

    '6': Select all objects that match the type of the current target/highlit (yellow box) object

    'i': Save the current, modified, scene_instance file. Also save removed_clutter.txt containing object names of all removed clutter objects.
         - With Shift : also close the viewer.

    'j': Modify AO link states :
         (+SHIFT) : Open Selected/All AOs
         (-SHIFT) : Close Selected/All AOs
         (+ALT) : Modify Selected AOs
         (-ALT) : Modify All AOs

    'l' : Toggle types of objects to display boxes around : None, AOs, Rigids, Both


    'v':
     - With object(s) selected : Duplicate the selected object(s)
     - With no object selected : Load an object by providing a uniquely identifying substring of the object's name


  Utilities:
    'r':        Reset the simulator with the most recently loaded scene.
    ',':        Render a Bullet collision shape debug wireframe overlay (white=active, green=sleeping, blue=wants sleeping, red=can't sleep).
    'c':        Toggle the contact point debug render overlay on/off. If toggled to true,
                then run a discrete collision detection pass and render a debug wireframe overlay
                showing active contact points and normals (yellow=fixed length normals, red=collision distances).
    'f':        Manage potential duplicate objects
         (+SHIFT) : Find and display potentially duplciate objects
         (+ALT) : Remove objects found as potential duplicates
    'k'         Toggle Semantic visualization bounds (currently only Semantic Region annotations)
    'n':        Show/hide NavMesh wireframe.
                (+SHIFT) Recompute NavMesh with Spot settings (already done).
                (+ALT) Re-sample Spot's position from the NavMesh.


  Simulation:
    SPACE:      Toggle physics simulation on/off.
    '.':        Take a single simulation step if not simulating continuously.
=====================================================
"""
        )


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
        default="./data/objects/ycb/ycb.scene_dataset_config.json",
        type=str,
        metavar="DATASET",
        help='dataset configuration file to use (default: "./data/objects/ycb/ycb.scene_dataset_config.json")',
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
        default=1080,
        type=int,
        help="Horizontal resolution of the window.",
    )
    parser.add_argument(
        "--height",
        default=720,
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
    sim_settings["sensor_height"] = 0
    sim_settings["enable_hbao"] = args.hbao

    # start the application
    HabitatSimInteractiveViewer(sim_settings).exec()
