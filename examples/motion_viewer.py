# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import ctypes
import random
import sys
import time
from typing import Any, Callable, Dict, Optional

flags = sys.getdlopenflags()
sys.setdlopenflags(flags | ctypes.RTLD_GLOBAL)

import magnum as mn
from magnum.platform.glfw import Application
from viewer import HabitatSimInteractiveViewer, MouseMode

import habitat_sim
import habitat_sim.physics as phy
from examples.fairmotion_interface import FairmotionInterface
from examples.settings import default_sim_settings
from habitat_sim.logging import logger


class FairmotionSimInteractiveViewer(HabitatSimInteractiveViewer):
    def __init__(
        self, sim_settings: Dict[str, Any], fm_settings: Dict[str, Any]
    ) -> None:
        super().__init__(sim_settings)

        # fairmotion init
        self.fm_demo = FairmotionInterface(
            self.sim,
            amass_path=fm_settings["amass_path"],
            urdf_path=fm_settings["urdf_path"],
            bm_path=fm_settings["bm_path"],
            metadata_file=fm_settings["metadata_file"],
        )

        # cache argument values for reconfigure
        self.fm_settings = fm_settings

        # configuring MOTION display objects
        # selection sphere icon
        obj_tmp_mgr = self.sim.get_object_template_manager()
        self.sphere_template_id = obj_tmp_mgr.load_configs(
            "../habitat-sim/data/test_assets/objects/sphere"
        )[0]
        sphere_template = obj_tmp_mgr.get_template_by_id(self.sphere_template_id)
        sphere_template.scale = [0.30, 0.30, 0.30]
        obj_tmp_mgr.register_template(sphere_template)

        # selection origin box
        self.box_template_id = obj_tmp_mgr.load_configs(
            "../habitat-sim/data/test_assets/objects/nested_box"
        )[0]
        box_template = obj_tmp_mgr.get_template_by_id(self.box_template_id)
        box_template.scale = [0.15, 0.025, 2.5]
        obj_tmp_mgr.register_template(box_template)

        # motion mode attributes
        self.selected_mocap_char: Optional[FairmotionInterface] = None
        self.select_sphere_obj_id: int = -1
        self.select_box_obj_id: int = -1

        # shortest path attributes
        self.spline_path_traj_obj_id = -1

        self.navmesh_config_and_recompute()

    def debug_draw(self):
        """
        Additional draw commands to be called during draw_event.
        """

        def draw_frame():
            red = mn.Color4(1.0, 0.0, 0.0, 1.0)
            green = mn.Color4(0.0, 1.0, 0.0, 1.0)
            blue = mn.Color4(0.0, 0.0, 1.0, 1.0)
            # yellow = mn.Color4(1.0, 1.0, 0.0, 1.0)

            # x axis
            self.sim.get_debug_line_render().draw_transformed_line(
                mn.Vector3(), mn.Vector3(1.0, 0.0, 0.0), red
            )
            # y axis
            self.sim.get_debug_line_render().draw_transformed_line(
                mn.Vector3(), mn.Vector3(0.0, 1.0, 0.0), green
            )
            # z axis
            self.sim.get_debug_line_render().draw_transformed_line(
                mn.Vector3(), mn.Vector3(0.0, 0.0, 1.0), blue
            )

        """
        if self.fm_demo is not None and self.fm_demo.model is not None:
            # print(f"model 2 root = {self.fm_demo.model2.transformation}")
            character_pos = self.fm_demo.model.translation
            coordinate_transform = mn.Matrix4.rotation(
                mn.Rad(mn.math.pi), mn.Vector3(0.0, 1.0, 0.0)
            )
            look_at_T = mn.Matrix4.look_at(
                mn.Vector3(), character_pos, mn.Vector3(0.0, 1.0, 0.0)
            )
            final_transform = look_at_T.__matmul__(coordinate_transform)
            # self.fm_demo.model2.transformation = final_transform
            self.sim.get_debug_line_render().push_transform(final_transform)
            draw_frame()
            self.sim.get_debug_line_render().pop_transform()
        """

    def draw_event(self, simulation_call: Optional[Callable] = None) -> None:
        """
        Calls continuously to re-render frames and swap the two frame buffers
        at a fixed rate. Use `simulation_call` to perform method calls during
        a simulation step.
        """

        def play_motion() -> None:
            if self.fm_demo.motion is not None:
                self.fm_demo.next_pose()
                self.fm_demo.next_pose()
                self.fm_demo.update_pathfollower_sequential()

        super().draw_event(simulation_call=play_motion)

    def key_press_event(self, event: Application.KeyEvent) -> None:
        """
        Handles `Application.KeyEvent` on a key press by performing the corresponding functions.
        If the key pressed is part of the movement keys map `Dict[KeyEvent.key, Bool]`, then the
        key will be set to False for the next `self.move_and_look()` to update the current actions.
        """

        key = event.key
        pressed = Application.KeyEvent.Key
        mod = Application.InputEvent.Modifier

        if key == pressed.F:
            if event.modifiers == mod.SHIFT:
                self.remove_selector_obj()
                self.fm_demo.hide_model()
                logger.info("Command: hide model")
            else:
                logger.info("Command: load model")
                self.fm_demo.load_model()

        elif key == pressed.J:
            if event.modifiers == mod.ALT:
                self.simulating = False
                self.fm_demo.update_pathfollower_sequential(
                    path_time=random.uniform(0.0, 10.0)
                )
            elif not self.sim.pathfinder.is_loaded:
                logger.warn("Warning: pathfinder not initialized, recompute navmesh")
            else:
                logger.info("Command: shortest path between two random points")
                path = self.find_short_path_from_two_points()
                self.fm_demo.setup_pathfollower(path)

        elif key == pressed.K:
            # Toggle Key Frames
            self.fm_demo.cycle_model_previews()

        elif key == pressed.SLASH:
            # Toggle reverse direction of motion
            self.fm_demo.is_reversed = not self.fm_demo.is_reversed

        elif key == pressed.M:
            # cycle through mouse modes
            if self.mouse_interaction == MouseMode.MOTION:
                self.remove_selector_obj()
            self.cycle_mouse_mode()
            logger.info(f"Command: mouse mode set to {self.mouse_interaction}")
            event.accepted = True
            self.redraw()
            return

        elif key == pressed.R:
            self.remove_selector_obj()
            super().reconfigure_sim()
            # reset character to default state
            self.fm_demo = FairmotionInterface(self.sim, metadata_file="default")
            event.accepted = True
            self.redraw()
            return

        elif key == pressed.SPACE:
            if not self.sim.config.sim_cfg.enable_physics:
                logger.warn("Warning: physics was not enabled during setup")
            else:
                self.simulating = not self.simulating
                logger.info(f"Command: physics simulating set to {self.simulating}")
            if self.simulating:
                self.remove_selector_obj()
            event.accepted = True
            self.redraw()
            return

        elif key == pressed.P:
            if event.modifiers == mod.CTRL:
                logger.info(f"Last file loaded: {self.fm_demo.last_metadata_file}")
            elif event.modifiers == mod.SHIFT:
                if self.fm_demo.last_metadata_file is None:
                    logger.warn("Warning: No previous file loaded.")
                else:
                    self.fm_demo.save_metadata(self.fm_demo.last_metadata_file)
            else:
                # ask for user input
                fn = input(
                    "Enter filename/filepath to save to (no input will generate a filename)(type 'esc' to abort save):"
                )
                if fn == "esc":
                    logger.info("No File Saved.")
                # if none, generate name and save
                # else, use name to save
                else:
                    self.fm_demo.save_metadata(fn)

        elif key == pressed.L:
            if event.modifiers == mod.CTRL:
                logger.info(f"Last file loaded: {self.fm_demo.last_metadata_file}")
            elif event.modifiers == mod.SHIFT:
                if self.fm_demo.last_metadata_file is None:
                    logger.warn("Warning: No previous file loaded.")
                else:
                    self.fm_demo.fetch_metadata(self.fm_demo.last_metadata_file)
            else:
                fn = input(
                    "Enter filename/filepath to load metadata file (enter nothing to abort):"
                )
                if fn in ["", None]:
                    logger.info("No File Loaded.")
                else:
                    self.fm_demo.fetch_metadata(fn)

        elif key == pressed.PERIOD:
            if self.simulating:
                logger.warn("Warning: physic simulation already running")
            else:
                self.simulate_single_step = True
                logger.info("Command: physics step taken")
                self.remove_selector_obj()
            event.accepted = True
            self.redraw()
            return

        # Testing!
        elif key == pressed.EQUAL:
            self.fm_demo.update_pathfollower(step_size=2)

        super().key_press_event(event)

    def mouse_press_event(self, event: Application.MouseEvent) -> None:
        """
        Handles `Application.MouseEvent`. When in GRAB mode, click on
        objects to drag their position. (right-click for fixed constraints).
        When in MOTION mode select Fairmotion characters with left-click,
        place them in a new location with right-click.
        """
        button = Application.MouseEvent.Button
        physics_enabled = self.sim.get_physics_simulation_library()

        # if interactive mode is True -> MOTION MODE
        if self.mouse_interaction == MouseMode.MOTION and physics_enabled:
            render_camera = self.render_camera.render_camera
            ray = render_camera.unproject(self.get_mouse_position(event.position))
            raycast_results = self.sim.cast_ray(ray=ray)

            if raycast_results.has_hits():
                hit_info = raycast_results.hits[0]

                if event.button == button.LEFT:
                    if self.fm_demo.belongs_to(hit_info.object_id):
                        if not self.fm_demo.model:
                            self.fm_demo.load_model()
                        self.simulating = False
                        self.create_selector_obj(self.fm_demo)
                    else:
                        self.remove_selector_obj()

                elif event.button == button.RIGHT and self.selected_mocap_char:
                    point = hit_info.point
                    self.fm_demo.set_transform_offsets(translate_offset=point)
                    self.create_selector_obj(self.fm_demo)
            # end has raycast hit

        super().mouse_press_event(event)

    def mouse_scroll_event(self, event: Application.MouseScrollEvent) -> None:
        """
        Handles `Application.MouseScrollEvent`. When in LOOK mode, enables camera
        zooming (fine-grained zoom using shift). When in GRAB mode, adjusts the depth
        of the grabber's object. (larger depth change rate using shift). When in MOTION
        mode, rotate them about the floor-normal axis with the scroll wheel. (fine-grained
        rotate using shift).
        """
        if self.mouse_interaction == MouseMode.MOTION and self.selected_mocap_char:
            physics_enabled = self.sim.get_physics_simulation_library()

            scroll_mod_val = (
                event.offset.y
                if abs(event.offset.y) > abs(event.offset.x)
                else event.offset.x
            )

            if not scroll_mod_val:
                return

            # use shift to scale action response
            shift_pressed = event.modifiers == Application.InputEvent.Modifier.SHIFT

            if (
                self.mouse_interaction == MouseMode.MOTION
                and physics_enabled
                and self.selected_mocap_char
            ):
                delta = mn.Quaternion.rotation(
                    mn.Deg(scroll_mod_val * (1 if shift_pressed else 20)),
                    mn.Vector3.z_axis(),
                )
                self.fm_demo.set_transform_offsets(
                    rotate_offset=self.fm_demo.rotation_offset * delta
                )
            self.create_selector_obj(self.fm_demo)

        super().mouse_scroll_event(event)

    def cycle_mouse_mode(self):
        """
        Cycles through mouse modes that belong to the MouseMode emun.
        """
        self.mouse_interaction = MouseMode(
            (self.mouse_interaction.value + 1) % len(MouseMode)
        )

    def create_selector_obj(self, mocap_char: FairmotionInterface):
        """
        Creates the selection icon above the given fairmotion character.
        """
        self.remove_selector_obj()

        # selection sphere icon
        obj = mocap_char.rgd_obj_mgr.add_object_by_template_id(self.sphere_template_id)
        obj.collidable = False
        obj.motion_type = phy.MotionType.KINEMATIC
        obj.translation = mocap_char.model.translation + mn.Vector3(0, 1.10, 0)
        self.select_sphere_obj_id = obj.object_id

        # selection origin box
        obj = mocap_char.rgd_obj_mgr.add_object_by_template_id(self.box_template_id)
        obj.collidable = False
        obj.motion_type = phy.MotionType.KINEMATIC
        obj.translation = mocap_char.translation_offset + mn.Vector3(0, 0.8, 0)
        obj.rotation = mocap_char.rotation_offset
        self.select_box_obj_id = obj.object_id

        self.selected_mocap_char = mocap_char

    def remove_selector_obj(self):
        """
        Removes the selection icon from the sim to indicate de-selection.
        """
        manager = self.sim.get_rigid_object_manager()

        # selection sphere icon
        if self.select_sphere_obj_id != -1:
            manager.remove_object_by_id(self.select_sphere_obj_id)
            self.select_sphere_obj_id = -1

        # selection origin box
        if self.select_box_obj_id != -1:
            manager.remove_object_by_id(self.select_box_obj_id)
            self.select_box_obj_id = -1

        self.selected_mocap_char = None

    def find_short_path_from_two_points(
        self, sample1=None, sample2=None
    ) -> habitat_sim.ShortestPath():
        """
        Finds two random points on the NavMesh, calculates a shortest path between
        the two, and creates a trajectory object to visualize the path.
        """
        if self.spline_path_traj_obj_id >= 0:
            self.sim.get_rigid_object_manager().remove_object_by_id(
                self.spline_path_traj_obj_id
            )
        self.path_traj_obj_id = -1

        found_path = False
        while not found_path:
            sample1 = None
            sample2 = None
            while any([sample1 is None, sample2 is None]):
                sample1 = sample1 or self.sim.pathfinder.get_random_navigable_point()
                sample2 = sample2 or self.sim.pathfinder.get_random_navigable_point()

                # constraint points to be on first floor
                if sample1[1] != sample2[1] or sample1[1] > 2:
                    logger.warn(
                        "Warning: points are out of acceptable area, replacing with randoms"
                    )
                    sample1, sample2 = None, None

            path = habitat_sim.ShortestPath()
            path.requested_start = sample1
            path.requested_end = sample2
            found_path = self.sim.pathfinder.find_path(path)
            self.path_points = path.points

        spline_points = habitat_sim.geo.build_catmull_rom_spline(path.points, 10, 0.5)
        path.points = spline_points

        colors_spline = [mn.Color3.blue(), mn.Color3.green()]

        self.spline_path_traj_obj_id = self.sim.add_gradient_trajectory_object(
            traj_vis_name=f"spline_{time.strftime('%Y-%m-%d_%H-%M-%S')}",
            colors=colors_spline,
            points=spline_points,
            radius=0.01,
        )

        return path

    def navmesh_config_and_recompute(self) -> None:
        """
        Overwrite the NavMesh function to compute more restricted bounds for character.
        """
        art_obj_mgr, art_cache = self.sim.get_articulated_object_manager(), {}
        rgd_obj_mgr, rgd_cache = self.sim.get_rigid_object_manager(), {}

        # Setting all articulated objects to static
        for obj_handle in art_obj_mgr.get_object_handles():
            # save original motion type
            art_cache[obj_handle] = art_obj_mgr.get_object_by_handle(
                obj_handle
            ).motion_type

            # setting object motion_type to static
            art_obj_mgr.get_object_by_handle(
                obj_handle
            ).motion_type = phy.MotionType.STATIC

        # Setting all rigid objects to static
        for obj_handle in rgd_obj_mgr.get_object_handles():
            # save original motion type
            rgd_cache[obj_handle] = rgd_obj_mgr.get_object_by_handle(
                obj_handle
            ).motion_type

            # setting object motion_type to static
            rgd_obj_mgr.get_object_by_handle(
                obj_handle
            ).motion_type = phy.MotionType.STATIC

        # compute NavMesh to be wary of Scene Objects
        self.navmesh_settings = habitat_sim.NavMeshSettings()
        self.navmesh_settings.set_defaults()
        self.navmesh_settings.agent_radius = 0.4
        self.sim.recompute_navmesh(
            self.sim.pathfinder,
            self.navmesh_settings,
            include_static_objects=True,
        )

        # Set all articulated objects back to original motion_type
        for obj_handle in art_obj_mgr.get_object_handles():
            art_obj_mgr.get_object_by_handle(obj_handle).motion_type = art_cache[
                obj_handle
            ]

        # Set all rigid objects back to original motion_type
        for obj_handle in rgd_obj_mgr.get_object_handles():
            rgd_obj_mgr.get_object_by_handle(obj_handle).motion_type = rgd_cache[
                obj_handle
            ]

    def print_help_text(self) -> None:
        """
        Print the Key Command help text.
        """
        logger.info(
            """
=========================================================
Welcome to the Habitat-sim Fairmotion Viewer application!
=========================================================
Mouse Functions ('m' to toggle mode):
----------------
In LOOK mode (default):
    LEFT:
        Click and drag to rotate the agent and look up/down.
    WHEEL:
        Modify orthographic camera zoom/perspective camera FOV
        (+ SHIFT): for fine-grained control

In GRAB mode (with 'enable-physics'):
    LEFT:
        Click and drag to pickup and move an object with a point-to-point constraint (e.g. ball joint).
    RIGHT:
        Click and drag to pickup and move an object with a fixed frame constraint.
    WHEEL (with picked object):
        Pull gripped object closer or push it away.

In MOTION mode (with 'enable-physics'):
    LEFT:
        Click a Fairmotion character to set it as selected or clcik anywhere else to deselect.
    RIGHT (With selected Fairmotion character):
        Click anywhere on the scene to translate a selected Fairmotion character to the clicked location.
    WHEEL (with selected Fairmotion character):
        Rotate the orientation of a selected Fairmotion character along an axis normal to the floor of the scene.
        (+ SHIFT): for fine-grained control

Key Commands:
-------------
    esc:        Exit the application.
    'h':        Display this help message.
    'm':        Cycle through mouse mode.

    Agent Controls:
    'wasd':     Move the agent's body forward/backward and left/right.
    'zx':       Move the agent's body up/down.
    arrow keys: Turn the agent's body left/right and camera look up/down.

    Utilities:
    'r':        Reset the simulator with the most recently loaded scene and default fairmotion character.

    Object Interactions:
    SPACE:      Toggle physics simulation on/off.
    '.':        Take a single simulation step if not simulating continuously.
    'v':        (physics) Invert gravity.
    'n':        Show/hide NavMesh wireframe.
                (+SHIFT) Recompute NavMesh with default settings.

    Fairmotion Interface:
    'f':        Load model with current motion data.
                [shft] Hide model.
    'j':        Load model to follow a path between two randomly chosen points.
    'k':        Toggle key frame preview of loaded motion.
    '/':        Set motion to play in reverse.
    'l':        Fetch and load data from a file give by the user's input.
                (+ SHIFT) Auto load current character data from last file fetched.
                (+ CTRL) Print the name of the last file fetched.
    'p':        Save current characterdata to a file give by the user's input.
                (+ SHIFT) Auto save current character data to last file fetched.
                (+ CTRL) Print the name of the last file fetched.
=========================================================
"""
        )


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
        "--amass_path",
        type=str,
        help="amass motion file path to load motion from (default: None)",
    )
    parser.add_argument(
        "--urdf_path",
        type=str,
        help="urdf file path to load model from (default: None)",
    )
    parser.add_argument(
        "--bm_path",
        type=str,
        help="npz type file path to load motion model from (default: None)",
    )
    parser.add_argument(
        "--metadata_file",
        type=str,
        help="JSON metadata file that should be used to load the scene and character (default: None)",
    )

    args = parser.parse_args()

    # Setting up sim_settings
    sim_settings: Dict[str, Any] = default_sim_settings
    sim_settings["scene"] = args.scene
    sim_settings["scene_dataset_config_file"] = args.dataset
    sim_settings["enable_physics"] = not args.disable_physics

    fm_settings: Dict[str, Any] = {}
    fm_settings["amass_path"] = args.amass_path
    fm_settings["urdf_path"] = args.urdf_path
    fm_settings["bm_path"] = args.bm_path
    fm_settings["metadata_file"] = args.metadata_file

    FairmotionSimInteractiveViewer(sim_settings, fm_settings).exec()
