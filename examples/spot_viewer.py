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
from typing import Any, Callable, Dict, List, Optional, Tuple

flags = sys.getdlopenflags()
sys.setdlopenflags(flags | ctypes.RTLD_GLOBAL)

import habitat.articulated_agents.robots.spot_robot as spot_robot
import habitat.sims.habitat_simulator.sim_utilities as sutils
import magnum as mn
import numpy as np
from habitat.datasets.rearrange.navmesh_utils import get_largest_island_index
from magnum import shaders, text
from magnum.platform.glfw import Application
from omegaconf import DictConfig

import habitat_sim
from habitat_sim import ReplayRenderer, ReplayRendererConfiguration
from habitat_sim.logging import LoggingContext, logger
from habitat_sim.utils.settings import default_sim_settings, make_cfg

SPOT_DIR = "data/robots/hab_spot_arm/urdf/hab_spot_arm.urdf"
if not os.path.isfile(SPOT_DIR):
    # support other layout
    SPOT_DIR = "data/scene_datasets/robots/hab_spot_arm/urdf/hab_spot_arm.urdf"


# Describe edit type
class EditMode(Enum):
    MOVE = 0
    ROTATE = 1
    NUM_VALS = 2


EDIT_MODE_NAMES = ["Move object", "Rotate object"]


# Describe edit distance values
class DistanceMode(Enum):
    TINY = 0
    VERY_SMALL = 1
    SMALL = 2
    MEDIUM = 3
    LARGE = 4
    HUGE = 5
    NUM_VALS = 6


# distance values in m
DISTANCE_MODE_VALS = [0.001, 0.01, 0.02, 0.05, 0.1, 0.5]
# angle value multipliers (in degrees) - multiplied by conversion
ROTATION_MULT_VALS = [1.0, 10.0, 30.0, 45.0, 60.0, 90.0]
# 1 radian
BASE_EDIT_ROT_AMT = math.pi / 180.0


class ExtractedBaseVelNonCylinderAction:
    def __init__(self, sim, spot):
        self._sim = sim
        self.spot = spot
        self.base_vel_ctrl = habitat_sim.physics.VelocityControl()
        self.base_vel_ctrl.controlling_lin_vel = True
        self.base_vel_ctrl.lin_vel_is_local = True
        self.base_vel_ctrl.controlling_ang_vel = True
        self.base_vel_ctrl.ang_vel_is_local = True
        self._allow_dyn_slide = True
        self._allow_back = True
        self._longitudinal_lin_speed = 10.0
        self._lateral_lin_speed = 10.0
        self._ang_speed = 10.0
        self._navmesh_offset = [[0.0, 0.0], [0.25, 0.0], [-0.25, 0.0]]
        self._enable_lateral_move = True
        self._collision_threshold = 1e-5

    def collision_check(self, trans, target_trans, target_rigid_state, compute_sliding):
        """
        trans: the transformation of the current location of the robot
        target_trans: the transformation of the target location of the robot given the center original Navmesh
        target_rigid_state: the target state of the robot given the center original Navmesh
        compute_sliding: if we want to compute sliding or not
        """
        # Get the offset positions
        num_check_cylinder = len(self._navmesh_offset)
        nav_pos_3d = [np.array([xz[0], 0.0, xz[1]]) for xz in self._navmesh_offset]
        cur_pos = [trans.transform_point(xyz) for xyz in nav_pos_3d]
        goal_pos = [target_trans.transform_point(xyz) for xyz in nav_pos_3d]

        # For step filter of offset positions
        end_pos = []
        for i in range(num_check_cylinder):
            pos = self._sim.step_filter(cur_pos[i], goal_pos[i])
            # Sanitize the height
            pos[1] = 0.0
            cur_pos[i][1] = 0.0
            goal_pos[i][1] = 0.0
            end_pos.append(pos)

        # Planar move distance clamped by NavMesh
        move = []
        for i in range(num_check_cylinder):
            move.append((end_pos[i] - goal_pos[i]).length())

        # For detection of linear or angualr velocities
        # There is a collision if the difference between the clamped NavMesh position and target position is too great for any point.
        diff = len([v for v in move if v > self._collision_threshold])

        if diff > 0:
            # Wrap the move direction if we use sliding
            # Find the largest diff moving direction, which means that there is a collision in that cylinder
            if compute_sliding:
                max_idx = np.argmax(move)
                move_vec = end_pos[max_idx] - cur_pos[max_idx]
                new_end_pos = trans.translation + move_vec
                return True, mn.Matrix4.from_(
                    target_rigid_state.rotation.to_matrix(), new_end_pos
                )
            return True, trans
        else:
            return False, target_trans

    def update_base(self, if_rotation):
        """
        Update the base of the robot
        if_rotation: if the robot is rotating or not
        """
        # Get the control frequency
        ctrl_freq = 60
        # Get the current transformation
        trans = self.spot.sim_obj.transformation
        # Get the current rigid state
        rigid_state = habitat_sim.RigidState(
            mn.Quaternion.from_matrix(trans.rotation()), trans.translation
        )
        # Integrate to get target rigid state
        target_rigid_state = self.base_vel_ctrl.integrate_transform(
            1 / ctrl_freq, rigid_state
        )
        # Get the traget transformation based on the target rigid state
        target_trans = mn.Matrix4.from_(
            target_rigid_state.rotation.to_matrix(),
            target_rigid_state.translation,
        )
        # We do sliding only if we allow the robot to do sliding and current
        # robot is not rotating
        compute_sliding = self._allow_dyn_slide and not if_rotation
        # Check if there is a collision
        did_coll, new_target_trans = self.collision_check(
            trans, target_trans, target_rigid_state, compute_sliding
        )
        # Update the base
        self.spot.sim_obj.transformation = new_target_trans

        if self.spot._base_type == "leg":
            # Fix the leg joints
            self.spot.leg_joint_pos = self.spot.params.leg_init_params

    def step(self, forward, lateral, angular):
        """
        provide forward, lateral, and angular velocities as [-1,1].
        """
        longitudinal_lin_vel = forward
        lateral_lin_vel = lateral
        ang_vel = angular
        longitudinal_lin_vel = (
            np.clip(longitudinal_lin_vel, -1, 1) * self._longitudinal_lin_speed
        )
        lateral_lin_vel = np.clip(lateral_lin_vel, -1, 1) * self._lateral_lin_speed
        ang_vel = np.clip(ang_vel, -1, 1) * self._ang_speed
        if not self._allow_back:
            longitudinal_lin_vel = np.maximum(longitudinal_lin_vel, 0)

        self.base_vel_ctrl.linear_velocity = mn.Vector3(
            longitudinal_lin_vel, 0, -lateral_lin_vel
        )
        self.base_vel_ctrl.angular_velocity = mn.Vector3(0, ang_vel, 0)

        if longitudinal_lin_vel != 0.0 or lateral_lin_vel != 0.0 or ang_vel != 0.0:
            self.update_base(ang_vel != 0.0)


def recompute_ao_bbs(ao: habitat_sim.physics.ManagedArticulatedObject) -> None:
    """
    Recomputes the link SceneNode bounding boxes for all ao links.
    NOTE: Gets around an observed loading bug. Call before trying to peek an AO.
    """
    for link_ix in range(-1, ao.num_links):
        link_node = ao.get_link_scene_node(link_ix)
        link_node.compute_cumulative_bb()


class HabitatSimInteractiveViewer(Application):
    # the maximum number of chars displayable in the app window
    # using the magnum text module. These chars are used to
    # display the CPU/GPU usage data
    MAX_DISPLAY_TEXT_CHARS = 256

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
        self.glyph_cache = text.GlyphCache(mn.Vector2i(256))
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
        mn.gl.Renderer.set_blend_equation(
            mn.gl.Renderer.BlendEquation.ADD, mn.gl.Renderer.BlendEquation.ADD
        )

        # variables that track app data and CPU/GPU usage
        self.num_frames_to_track = 60

        # Editing
        # Edit mode
        self.curr_edit_mode = EditMode.MOVE
        # Edit distance/amount
        self.curr_edit_multiplier = DistanceMode.VERY_SMALL

        # Initialize base edit changes
        self.set_edit_vals()

        self.previous_mouse_point = None

        # toggle physics simulation on/off
        self.simulating = True

        # toggle a single simulation step at the next opportunity if not
        # simulating continuously.
        self.simulate_single_step = False

        self.spot = None
        self.spot_action = None
        self.spot_forward = 0
        self.spot_lateral = 0
        self.spot_angular = 0
        self.camera_distance = 2.0
        self.camera_angles = mn.Vector2()

        # object selection and manipulation interface
        self.selected_object = None
        self.selected_object_orig_transform = mn.Matrix4().identity_init()
        self.last_hit_details = None
        # cache modified states of any objects moved by the interface.
        self.modified_objects_buffer: Dict[
            habitat_sim.physics.ManagedRigidObject, mn.Matrix4
        ] = {}
        self.removed_clutter = []

        self.navmesh_dirty = False
        self.removed_objects_debug_frames = []

        # configure our simulator
        self.cfg: Optional[habitat_sim.simulator.Configuration] = None
        self.sim: Optional[habitat_sim.simulator.Simulator] = None
        self.tiled_sims: list[habitat_sim.simulator.Simulator] = None
        self.replay_renderer_cfg: Optional[ReplayRendererConfiguration] = None
        self.replay_renderer: Optional[ReplayRenderer] = None
        self.reconfigure_sim()

        # compute NavMesh if not already loaded by the scene.
        if self.cfg.sim_cfg.scene_id.lower() != "none":
            self.navmesh_config_and_recompute()

        self.place_spot()

        self.time_since_last_simulation = 0.0
        LoggingContext.reinitialize_from_env()
        logger.setLevel("INFO")
        self.print_help_text()

    def set_edit_vals(self):
        # Set current scene object edit values for translation and rotation
        # 1 cm * multiplier
        self.edit_translation_dist = DISTANCE_MODE_VALS[self.curr_edit_multiplier.value]
        # 1 radian * multiplier
        self.edit_rotation_amt = (
            BASE_EDIT_ROT_AMT * ROTATION_MULT_VALS[self.curr_edit_multiplier.value]
        )

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

    def place_spot(self):
        if self.sim.pathfinder.is_loaded:
            largest_island_ix = get_largest_island_index(
                pathfinder=self.sim.pathfinder,
                sim=self.sim,
                allow_outdoor=False,
            )
            print(f"Largest indoor island index = {largest_island_ix}")
            valid_spot_point = None
            max_attempts = 1000
            attempt = 0
            while valid_spot_point is None and attempt < max_attempts:
                spot_point = self.sim.pathfinder.get_random_navigable_point(
                    island_index=largest_island_ix
                )
                if self.sim.pathfinder.distance_to_closest_obstacle(spot_point) >= 0.25:
                    valid_spot_point = spot_point
                attempt += 1
            if valid_spot_point is not None:
                self.spot.base_pos = valid_spot_point

    def clear_furniture_joint_states(self):
        """
        Clear all furniture object joint states.
        """
        for ao in (
            self.sim.get_articulated_object_manager()
            .get_objects_by_handle_substring()
            .values()
        ):
            # ignore the robot
            if "hab_spot" not in ao.handle:
                j_pos = ao.joint_positions
                ao.joint_positions = [0.0 for _ in range(len(j_pos))]
                j_vel = ao.joint_velocities
                ao.joint_velocities = [0.0 for _ in range(len(j_vel))]

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
        if self.last_hit_details is not None:
            self.sim.get_debug_line_render().draw_circle(
                translation=self.last_hit_details.point,
                radius=0.02,
                normal=self.last_hit_details.normal,
                color=mn.Color4.yellow(),
                num_segments=12,
            )
        if self.selected_object is not None:
            aabb = None
            if isinstance(
                self.selected_object, habitat_sim.physics.ManagedBulletRigidObject
            ):
                aabb = self.selected_object.collision_shape_aabb
            else:
                aabb = sutils.get_ao_root_bb(self.selected_object)
            dblr = self.sim.get_debug_line_render()
            dblr.push_transform(self.selected_object.transformation)
            dblr.draw_box(aabb.min, aabb.max, mn.Color4.magenta())
            dblr.pop_transform()

            ot = self.selected_object.translation
            # draw global coordinate axis
            dblr.draw_transformed_line(
                ot - mn.Vector3.x_axis(), ot + mn.Vector3.x_axis(), mn.Color4.red()
            )
            dblr.draw_transformed_line(
                ot - mn.Vector3.y_axis(), ot + mn.Vector3.y_axis(), mn.Color4.green()
            )
            dblr.draw_transformed_line(
                ot - mn.Vector3.z_axis(), ot + mn.Vector3.z_axis(), mn.Color4.blue()
            )
            dblr.draw_circle(
                ot + mn.Vector3.x_axis() * 0.95,
                radius=0.05,
                color=mn.Color4.red(),
                normal=mn.Vector3.x_axis(),
            )
            dblr.draw_circle(
                ot + mn.Vector3.y_axis() * 0.95,
                radius=0.05,
                color=mn.Color4.green(),
                normal=mn.Vector3.y_axis(),
            )
            dblr.draw_circle(
                ot + mn.Vector3.z_axis() * 0.95,
                radius=0.05,
                color=mn.Color4.blue(),
                normal=mn.Vector3.z_axis(),
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

        keys = active_agent_id_and_sensor_name

        # set agent position relative to spot
        x_rot = mn.Quaternion.rotation(
            mn.Rad(self.camera_angles[0]), mn.Vector3(1, 0, 0)
        )
        y_rot = mn.Quaternion.rotation(
            mn.Rad(self.camera_angles[1]), mn.Vector3(0, 1, 0)
        )
        local_camera_vec = mn.Vector3(0, 0, 1)
        local_camera_position = y_rot.transform_vector(
            x_rot.transform_vector(local_camera_vec * self.camera_distance)
        )
        camera_position = local_camera_position + self.spot.base_pos
        self.default_agent.scene_node.transformation = mn.Matrix4.look_at(
            camera_position,
            self.spot.base_pos,
            mn.Vector3(0, 1, 0),
        )

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

        self.init_spot()

        Timer.start()
        self.step = -1

    def init_spot(self):
        # add the robot to the world via the wrapper
        robot_path = SPOT_DIR
        agent_config = DictConfig({"articulated_agent_urdf": robot_path})
        self.spot = spot_robot.SpotRobot(agent_config, self.sim, fixed_base=True)
        self.spot.reconfigure()
        self.spot.update()
        self.spot_action = ExtractedBaseVelNonCylinderAction(self.sim, self.spot)

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

        key = Application.KeyEvent.Key
        press: Dict[Application.KeyEvent.Key.key, bool] = self.pressed

        inc = 0.02
        min_val = 0.1

        if press[key.W] and not press[key.S]:
            self.spot_forward = max(min_val, self.spot_forward + inc)
        elif press[key.S] and not press[key.W]:
            self.spot_forward = min(-min_val, self.spot_forward - inc)
        else:
            self.spot_forward /= 2.0
            if abs(self.spot_forward) < min_val:
                self.spot_forward = 0

        if press[key.Q] and not press[key.E]:
            self.spot_lateral = max(min_val, self.spot_lateral + inc)
        elif press[key.E] and not press[key.Q]:
            self.spot_lateral = min(-min_val, self.spot_lateral - inc)
        else:
            self.spot_lateral /= 2.0
            if abs(self.spot_lateral) < min_val:
                self.spot_lateral = 0

        if press[key.A] and not press[key.D]:
            self.spot_angular = max(min_val, self.spot_angular + inc)
        elif press[key.D] and not press[key.A]:
            self.spot_angular = min(-min_val, self.spot_angular - inc)
        else:
            self.spot_angular /= 2.0
            if abs(self.spot_angular) < min_val:
                self.spot_angular = 0

        self.spot_action.step(
            forward=self.spot_forward,
            lateral=self.spot_lateral,
            angular=self.spot_angular,
        )

    def invert_gravity(self) -> None:
        """
        Sets the gravity vector to the negative of it's previous value. This is
        a good method for testing simulation functionality.
        """
        gravity: mn.Vector3 = self.sim.get_gravity() * -1
        self.sim.set_gravity(gravity)

    def move_selected_object(
        self,
        translation: Optional[mn.Vector3] = None,
        rotation: Optional[mn.Quaternion] = None,
    ):
        """
        Move the selected object with a given modification and save the resulting state to the buffer.
        """
        modify_buffer = translation is not None or rotation is not None
        if self.selected_object is not None and modify_buffer:
            orig_mt = self.selected_object.motion_type
            self.selected_object.motion_type = habitat_sim.physics.MotionType.KINEMATIC
            if translation is not None:
                self.selected_object.translation = (
                    self.selected_object.translation + translation
                )
            if rotation is not None:
                self.selected_object.rotation = rotation * self.selected_object.rotation
            self.selected_object.motion_type = orig_mt
            self.navmesh_dirty = True
            self.modified_objects_buffer[
                self.selected_object
            ] = self.selected_object.transformation

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
            pass

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
            # if movement mode
            if self.curr_edit_mode == EditMode.MOVE:
                self.move_selected_object(
                    translation=mn.Vector3.x_axis() * self.edit_translation_dist
                )
            # if rotation mode : rotate around y axis
            else:
                self.move_selected_object(
                    rotation=mn.Quaternion.rotation(
                        mn.Rad(self.edit_rotation_amt), mn.Vector3.y_axis()
                    )
                )
        elif key == pressed.RIGHT:
            # if movement mode
            if self.curr_edit_mode == EditMode.MOVE:
                self.move_selected_object(
                    translation=-mn.Vector3.x_axis() * self.edit_translation_dist
                )
            # if rotation mode : rotate around y axis
            else:
                self.move_selected_object(
                    rotation=mn.Quaternion.rotation(
                        -mn.Rad(self.edit_rotation_amt), mn.Vector3.y_axis()
                    )
                )
        elif key == pressed.UP:
            # if movement mode
            if self.curr_edit_mode == EditMode.MOVE:
                if alt_pressed:
                    self.move_selected_object(
                        translation=mn.Vector3.y_axis() * self.edit_translation_dist
                    )
                else:
                    self.move_selected_object(
                        translation=mn.Vector3.z_axis() * self.edit_translation_dist
                    )
            # if rotation mode : rotate around x or z axis
            else:
                if alt_pressed:
                    # rotate around x axis
                    self.move_selected_object(
                        rotation=mn.Quaternion.rotation(
                            mn.Rad(self.edit_rotation_amt), mn.Vector3.x_axis()
                        )
                    )
                else:
                    # rotate around z axis
                    self.move_selected_object(
                        rotation=mn.Quaternion.rotation(
                            mn.Rad(self.edit_rotation_amt), mn.Vector3.z_axis()
                        )
                    )

        elif key == pressed.DOWN:
            # if movement mode
            if self.curr_edit_mode == EditMode.MOVE:
                if alt_pressed:
                    self.move_selected_object(
                        translation=-mn.Vector3.y_axis() * self.edit_translation_dist
                    )
                else:
                    self.move_selected_object(
                        translation=-mn.Vector3.z_axis() * self.edit_translation_dist
                    )
            # if rotation mode : rotate around x or z axis
            else:
                if alt_pressed:
                    # rotate around x axis
                    self.move_selected_object(
                        rotation=mn.Quaternion.rotation(
                            -mn.Rad(self.edit_rotation_amt), mn.Vector3.x_axis()
                        )
                    )
                else:
                    # rotate around z axis
                    self.move_selected_object(
                        rotation=mn.Quaternion.rotation(
                            -mn.Rad(self.edit_rotation_amt), mn.Vector3.z_axis()
                        )
                    )

        elif key == pressed.BACKSPACE or key == pressed.C:
            if self.selected_object is not None:
                if key == pressed.C:
                    obj_name = self.selected_object.handle.split("/")[-1].split("_:")[0]
                    self.removed_clutter.append(obj_name)
                print(f"Removed {self.selected_object.handle}")
                if isinstance(
                    self.selected_object, habitat_sim.physics.ManagedBulletRigidObject
                ):
                    self.sim.get_rigid_object_manager().remove_object_by_handle(
                        self.selected_object.handle
                    )
                else:
                    self.sim.get_articulated_object_manager().remove_object_by_handle(
                        self.selected_object.handle
                    )
                self.selected_object = None
                self.navmesh_config_and_recompute()
        elif key == pressed.B:
            # cycle through edit dist/amount multiplier
            mod_val = -1 if shift_pressed else 1
            self.curr_edit_multiplier = DistanceMode(
                (
                    self.curr_edit_multiplier.value
                    + DistanceMode.NUM_VALS.value
                    + mod_val
                )
                % DistanceMode.NUM_VALS.value
            )
            # update the edit values
            self.set_edit_vals()

        elif key == pressed.G:
            # toggle edit mode
            mod_val = -1 if shift_pressed else 1
            self.curr_edit_mode = EditMode(
                (self.curr_edit_mode.value + EditMode.NUM_VALS.value + mod_val)
                % EditMode.NUM_VALS.value
            )

        elif key == pressed.I:
            # dump the modified object states buffer to JSON.
            # print(f"Writing modified_objects_buffer to 'scene_mod_buffer.json': {self.modified_objects_buffer}")
            # with open("scene_mod_buffer.json", "w") as f:
            #    f.write(json.dumps(self.modified_objects_buffer, indent=2))
            aom = self.sim.get_articulated_object_manager()
            spot_loc = self.spot.sim_obj.rigid_state
            aom.remove_object_by_handle(self.spot.sim_obj.handle)

            # clear furniture joint positions before saving
            self.clear_furniture_joint_states()

            self.sim.save_current_scene_config(overwrite=True)
            print("Saved modified scene instance JSON to original location.")
            # de-duplicate and save clutter list
            self.removed_clutter = list(dict.fromkeys(self.removed_clutter))
            with open("removed_clutter.txt", "a") as f:
                for obj_name in self.removed_clutter:
                    f.write(obj_name + "\n")
            # only exit if shift pressed
            if shift_pressed:
                event.accepted = True
                self.exit_event(Application.ExitEvent)
                return
            # rebuild spot
            self.init_spot()
            # put em back
            self.spot.sim_obj.rigid_state = spot_loc

        elif key == pressed.J:
            if shift_pressed and isinstance(
                self.selected_object, habitat_sim.physics.ManagedArticulatedObject
            ):
                # open the selected receptacle
                for link_ix in self.selected_object.get_link_ids():
                    if self.selected_object.get_link_joint_type(link_ix) in [
                        habitat_sim.physics.JointType.Prismatic,
                        habitat_sim.physics.JointType.Revolute,
                    ]:
                        sutils.open_link(self.selected_object, link_ix)
            else:
                self.clear_furniture_joint_states()
                self.navmesh_config_and_recompute()

        elif key == pressed.N:
            # (default) - toggle navmesh visualization
            # NOTE: (+ALT) - re-sample the agent position on the NavMesh
            # NOTE: (+SHIFT) - re-compute the NavMesh
            if alt_pressed:
                logger.info("Command: resample agent state from navmesh")
                self.place_spot()
            elif shift_pressed:
                logger.info("Command: recompute navmesh")
                self.navmesh_config_and_recompute()
            else:
                if self.sim.pathfinder.is_loaded:
                    self.sim.navmesh_visualization = not self.sim.navmesh_visualization
                    logger.info("Command: toggle navmesh")
                else:
                    logger.warn("Warning: recompute navmesh first")

        elif key == pressed.T:
            self.remove_outdoor_objects()
            pass

        elif key == pressed.U:
            # if an object is selected, restore its last transformation state - UNDO of edits since last selected
            print("Undo selected")
            if self.selected_object is not None:
                print(
                    f"Sel Obj : {self.selected_object.handle} : Current object transformation : \n{self.selected_object.transformation}\n Being replaced by saved transformation : \n{self.selected_object.transformation}"
                )
                orig_mt = self.selected_object.motion_type
                self.selected_object.motion_type = (
                    habitat_sim.physics.MotionType.KINEMATIC
                )
                self.selected_object.transformation = (
                    self.selected_object_orig_transform
                )
                self.selected_object.motion_type = orig_mt

        elif key == pressed.V:
            # inject a new AO by handle substring in front of the agent

            # get user input
            ao_substring = input(
                "Load ArticulatedObject. Enter an AO handle substring, first match will be added:"
            ).strip()

            aotm = self.sim.metadata_mediator.ao_template_manager
            aom = self.sim.get_articulated_object_manager()
            ao_handles = aotm.get_template_handles(ao_substring)
            if len(ao_handles) == 0:
                print(f"No AO found matching substring: '{ao_substring}'")
                return
            elif len(ao_handles) > 1:
                print(f"Multiple AOs found matching substring: '{ao_substring}'.")
            matching_ao_handle = ao_handles[0]
            print(f"Adding AO: '{matching_ao_handle}'")
            aot = aotm.get_template_by_handle(matching_ao_handle)
            aot.base_type = "FIXED"
            aotm.register_template(aot)
            ao = aom.add_articulated_object_by_template_handle(matching_ao_handle)
            if ao is not None:
                recompute_ao_bbs(ao)
                in_front_of_spot = self.spot.base_transformation.transform_point(
                    [1.5, 0.0, 0.0]
                )
                ao.translation = in_front_of_spot
            else:
                print("Failed to load AO.")

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
        continues to update the grabber's object position with our agents position.
        """
        button = Application.MouseMoveEvent.Buttons
        # if interactive mode -> LOOK MODE
        if event.buttons == button.LEFT:
            self.camera_angles[0] -= float(event.relative_position[1]) * 0.01
            self.camera_angles[1] -= float(event.relative_position[0]) * 0.01
            self.camera_angles[0] = max(-1.55, min(0.5, self.camera_angles[0]))
            self.camera_angles[1] = math.fmod(self.camera_angles[1], math.pi * 2)

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
        mod = Application.InputEvent.Modifier
        shift_pressed = bool(event.modifiers & mod.SHIFT)

        # select an object with Shift+RIGHT-click
        if physics_enabled and event.button == button.RIGHT and shift_pressed:
            self.selected_object = None
            render_camera = self.render_camera.render_camera
            ray = render_camera.unproject(self.get_mouse_position(event.position))
            mouse_cast_results = self.sim.cast_ray(ray=ray)
            if mouse_cast_results.has_hits():
                # find first non-stage object
                hit_idx = 0
                obj_found = False
                while hit_idx < len(mouse_cast_results.hits) and not obj_found:
                    self.last_hit_details = mouse_cast_results.hits[hit_idx]
                    hit_obj_id = mouse_cast_results.hits[hit_idx].object_id
                    self.selected_object = sutils.get_obj_from_id(self.sim, hit_obj_id)
                    if self.selected_object is None:
                        hit_idx += 1
                    else:
                        obj_found = True
                if obj_found:
                    print(
                        f"Object: {self.selected_object.handle} is {type(self.selected_object)}"
                    )
                else:
                    print("This is the stage.")
            # record current selected object's transformation, to restore if undo is pressed
            if self.selected_object is not None:
                self.selected_object_orig_transform = (
                    self.selected_object.transformation
                )

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
        # alt_pressed = bool(event.modifiers & Application.InputEvent.Modifier.ALT)
        # ctrl_pressed = bool(event.modifiers & Application.InputEvent.Modifier.CTRL)

        # LOOK MODE
        # use shift for fine-grained zooming
        mod_val = 0.3 if shift_pressed else 0.15
        scroll_delta = scroll_mod_val * mod_val
        self.camera_distance -= scroll_delta

        self.redraw()
        event.accepted = True

    def mouse_release_event(self, event: Application.MouseEvent) -> None:
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
        self.navmesh_settings.agent_radius = 0.3
        self.navmesh_settings.include_static_objects = True

        # first cache AO motion types and set to STATIC for navmesh
        ao_motion_types = {}
        for ao in (
            self.sim.get_articulated_object_manager()
            .get_objects_by_handle_substring()
            .values()
        ):
            # ignore the robot
            if "hab_spot" not in ao.handle:
                ao_motion_types[ao.handle] = ao.motion_type
                ao.motion_type = habitat_sim.physics.MotionType.STATIC

        self.sim.recompute_navmesh(self.sim.pathfinder, self.navmesh_settings)

        # reset AO motion types from cache
        for ao in (
            self.sim.get_articulated_object_manager()
            .get_objects_by_handle_substring()
            .values()
        ):
            # ignore the robot
            if ao.handle in ao_motion_types:
                ao.motion_type = ao_motion_types[ao.handle]

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
        edit_mode_string = EDIT_MODE_NAMES[self.curr_edit_mode.value]

        dist_mode_substr = (
            f"Translation: {self.edit_translation_dist}m"
            if self.curr_edit_mode == EditMode.MOVE
            else f"Rotation:{ROTATION_MULT_VALS[self.curr_edit_multiplier.value]} deg "
        )
        edit_distance_mode_string = f"{dist_mode_substr}"
        self.window_text.render(
            f"""
{self.fps} FPS
Scene ID : {os.path.split(self.cfg.sim_cfg.scene_id)[1].split('.scene_instance')[0]}
Sensor Type: {sensor_type_string}
Sensor Subtype: {sensor_subtype_string}
Edit Mode: {edit_mode_string}
Edit Value: {edit_distance_mode_string}
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
In LOOK mode (default):
    LEFT:
        Click and drag to rotate the view around Spot.
    WHEEL:
        Zoom in and out on Spot view.


Key Commands:
-------------
    esc:        Exit the application.
    'h':        Display this help message.

    Spot Controls:
    'wasd':     Move Spot's body forward/backward and rotate left/right.
    'qe':       Move Spot's body in strafe left/right.

    Scene Object Modification UI:
    'SHIFT+right-click': Select an object to modify.
    'g' : Change Edit mode to either Move or Rotate the selected object
    'b' (+ SHIFT) : Increment (Decrement) the current edit amounts.
        - With an object selected:
            When Move Object mode is selected :
            - LEFT/RIGHT arrow keys: move the object along global X axis.
            - UP/DOWN arrow keys: move the object along global Z axis.
                (+ALT): move the object up/down (global Y axis)
            When Rotate Object mode is selected :
            - LEFT/RIGHT arrow keys: rotate the object around global Y axis.
            - UP/DOWN arrow keys: rotate the object around global Z axis.
                (+ALT): rotate the object around global X axis.
            - BACKSPACE: delete the selected object
            - 'c': delete the selected object and record it as clutter.
    'i': save the current, modified, scene_instance file. Also save removed_clutter.txt containing object names of all removed clutter objects.
         - With Shift : also close the viewer.

    Utilities:
    'r':        Reset the simulator with the most recently loaded scene.
    'n':        Show/hide NavMesh wireframe.
                (+SHIFT) Recompute NavMesh with Spot settings (already done).
                (+ALT) Re-sample Spot's position from the NavMesh.
    ',':        Render a Bullet collision shape debug wireframe overlay (white=active, green=sleeping, blue=wants sleeping, red=can't sleep).

    Object Interactions:
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
