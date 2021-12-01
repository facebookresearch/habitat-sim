# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import json
import math
import os
import random
import time
from typing import Any, Dict, List, Optional, Tuple

import magnum as mn
from fairmotion.core import motion
from fairmotion.data import amass
from fairmotion.ops import conversions
from fairmotion_interface_utils import (
    ActionOrder,
    Activity,
    Motions,
    MType,
    PathData,
    Preview,
    Timer,
)

import habitat_sim
import habitat_sim.physics as phy
from habitat_sim.logging import LoggingContext, logger

#### Constants ###
ROOT, LAST = 0, -1
METADATA_DEFAULT_WHEN_MISSING_FILE = {
    "urdf_path": "data/test_assets/urdf/amass_male.urdf",
    "amass_path": "data/fairmotion/amass_test_data/CMU/CMU/13/13_08_poses.npz",
    "bm_path": "data/fairmotion/amass_test_data/smplh/male/model.npz",
    "rotation": mn.Quaternion(),
    "translation": mn.Vector3([2.5, 0.07, 0.7]),
}
METADATA_DIR = "data/fairmotion/"


class FairmotionInterface:
    def __init__(
        self,
        sim,
        fps,
        urdf_path=None,
        amass_path=None,
        bm_path=None,
        metadata_file=None,
    ) -> None:
        # general interface attrs
        LoggingContext.reinitialize_from_env()
        self.sim: Optional[habitat_sim.simulator.Simulator] = sim
        self.draw_fps: float = fps
        self.art_obj_mgr = self.sim.get_articulated_object_manager()
        self.rgd_obj_mgr = self.sim.get_rigid_object_manager()
        self.motion: Optional[motion.Motion] = None
        self.user_metadata = {}
        self.last_metadata_file: Optional[str] = None
        self.motion_stepper = 0
        self.rotation_offset: Optional[mn.Quaternion] = None
        self.translation_offset: Optional[mn.Vector3] = None
        self.is_reversed = False
        self.activity: Activity = Activity.NONE

        # key frame attrs
        self.key_frames = None
        self.key_frame_models = []
        self.preview_mode: Preview = Preview.OFF
        self.traj_ids: List[int] = []

        # path follower attrs
        self.model: Optional[phy.ManagedArticulatedObject] = None
        self.path_data: Optional[PathData] = None

        # sequence attrs
        self.order_queue: List[ActionOrder] = []
        self.staging_queue: List[Tuple] = []  # Rename to self.sequence_buffer
        self.last_seq_location: mn.Vector3 = mn.Vector3(2.0, 0.0, 3.0)
        self.incomplete_order: List[Any] = []

        self.setup_default_metadata()

        if metadata_file:
            try:
                self.fetch_metadata(metadata_file)
                self.set_data()
            except FileNotFoundError:
                logger.error(f"No file with path `{metadata_file}`, creating new file.")
                self.set_data(
                    urdf_path=urdf_path,
                    amass_path=amass_path,
                    bm_path=bm_path,
                )
                self.save_metadata(metadata_file)
                self.last_metadata_file = metadata_file

        else:
            # This sets the instance defaults with init(args)
            self.set_data(urdf_path=urdf_path, amass_path=amass_path, bm_path=bm_path)
            self.save_metadata("default")

        # positional offsets
        self.set_transform_offsets(
            rotate_offset=self.user_metadata["rotation"],
            translate_offset=self.user_metadata["translation"],
        )
        self.load_motion()

    def setup_default_metadata(self) -> None:
        """
        Called by init(), this method forces the default metadata into the metadata library.
        """
        # if folder doesn't exist, then create it.
        if not os.path.isdir(METADATA_DIR):
            os.makedirs(METADATA_DIR)

        # if default file isnt in metadata directory, create it from hardcoded metadata
        if not os.path.exists(METADATA_DIR + "default.json"):
            self.user_metadata = METADATA_DEFAULT_WHEN_MISSING_FILE
            self.save_metadata("default")

        # if default file exists, take it from there
        self.fetch_metadata("default")
        self.last_metadata_file: Optional[str] = None

    def set_data(
        self,
        urdf_path: Optional[str] = None,
        amass_path: Optional[str] = None,
        bm_path: Optional[str] = None,
        rotation: Optional[mn.Quaternion] = None,
        translation: Optional[mn.Vector3] = None,
    ) -> None:
        """
        A method that will take attributes of the model data sets as arguments, filling in
        empty args with the default set of data, and adds the data to the meta_data library.
        """
        data = self.user_metadata
        data["urdf_path"] = urdf_path or data["urdf_path"]
        data["amass_path"] = amass_path or data["amass_path"]
        data["bm_path"] = bm_path or data["bm_path"]
        data["rotation"] = rotation or data["rotation"]
        data["translation"] = translation or data["translation"]

    def metadata_parser(self, metadata_dict: Dict[str, Any], to_file: bool):
        """
        Convert metadata dict to and from a form that is json serializable
        """
        data = metadata_dict.copy()

        if to_file:
            for k, v in data.items():
                if isinstance(v, mn.Quaternion):
                    q_list = [list(v.vector)]
                    q_list.append(v.scalar)
                    data[k] = q_list
                elif isinstance(v, mn.Vector3):
                    data[k] = list(v)

        elif not to_file:
            for k, v in data.items():
                if k == "rotation":
                    data[k] = mn.Quaternion(v)
                elif k == "translation":
                    data[k] = mn.Vector3(v)
        return data

    def save_metadata(self, file: str):
        """
        Saves the current metadata to a json file in given file path
        """
        meta_access = [METADATA_DIR + file + ".json", METADATA_DIR + file, file]

        if file:
            # sanitizing file path
            for filename in meta_access:
                if os.path.exists(filename):
                    file = filename
                    break

                # we are no longer overwriting a file
                elif "/" not in file:
                    # file is not a file path, we need to aim it at our directory
                    if ".json" not in file:
                        # add file type
                        file = file + ".json"
                    file = METADATA_DIR + file
        else:
            # generate filename from timestamp
            file = METADATA_DIR + "user_" + time.strftime("%Y-%m-%d_%H-%M-%S")

        logger.info(f"Saving data to file: {file}")

        # updating user_metadata to reflect characters position
        if self.rotation_offset and self.translation_offset:
            data = self.user_metadata
            data["rotation"] = self.rotation_offset
            data["translation"] = self.translation_offset

        data = self.metadata_parser(self.user_metadata, to_file=True)

        with open(file, "w") as f:
            json.dump(data, f)
        logger.info(f"Saved: {file}")

    def fetch_metadata(self, file):
        """
        Fetch metadata from a json file in given file name and sets metadata
        """
        meta_access = [METADATA_DIR + file + ".json", METADATA_DIR + file, file]

        # sanitizing file path
        for filename in meta_access:
            if os.path.exists(filename):
                file = filename
                break

        with open(file, "r") as f:
            data = json.load(f)

        # set data to what was fetched
        self.user_metadata = self.metadata_parser(data, to_file=False)
        logger.info(f"Fetched: {file}")

        self.last_metadata_file = file

        # updating user_metadata to reflect characters position
        if self.rotation_offset and self.translation_offset:
            data = self.user_metadata
            self.rotation_offset = data["rotation"]
            self.translation_offset = data["translation"]

        self.hide_model()
        self.load_motion()

    def set_transform_offsets(
        self,
        rotate_offset: Optional[mn.Quaternion] = None,
        translate_offset: Optional[mn.Vector3] = None,
    ) -> None:
        """
        This method updates the offset of the model with the positional data passed to it.
        Use this for changing the location and orientation of the model.
        """
        self.rotation_offset = rotate_offset or self.rotation_offset
        self.translation_offset = translate_offset or self.translation_offset

        if self.traj_ids:
            # removes trajectory
            for t_id in self.traj_ids:
                self.sim.get_rigid_object_manager().remove_object_by_id(t_id)
                self.traj_ids = []
        self.next_pose(repeat=True)
        self.setup_key_frames()
        self.build_trajectory_vis()

    def load_motion(self) -> None:
        """
        Loads the motion currently set by metadata.
        """
        # loading text because the setup pauses here during motion load
        logger.info("Loading...")
        data = self.user_metadata
        self.motion = amass.load(file=data["amass_path"], bm_path=data["bm_path"])
        self.set_transform_offsets(
            rotate_offset=data["rotation"], translate_offset=data["translation"]
        )
        logger.info("Done Loading.")

    def load_model(self) -> None:
        """
        Loads the model currently set by metadata.
        """
        # loading text because the setup pauses here during motion load
        logger.info("Loading...")
        self.hide_model()
        self.activity = Activity.MOTION_FOLLOW

        # keeps the model up to date with current data target
        data = self.user_metadata

        # add an ArticulatedObject to the world with a fixed base
        self.model = self.art_obj_mgr.add_articulated_object_from_urdf(
            filepath=data["urdf_path"], fixed_base=True
        )
        assert self.model.is_alive

        # change motion_type to KINEMATIC
        self.model.motion_type = phy.MotionType.KINEMATIC

        self.model.translation = self.translation_offset
        self.next_pose(repeat=True)
        logger.info("Done Loading.")

    def hide_model(self) -> None:
        """
        Removes model from scene.
        """
        if self.model:
            self.art_obj_mgr.remove_object_by_handle(self.model.handle)
            self.model = None

    # currently the next_pose method is simply called twice in simulating a frame
    def next_pose(self, repeat=False, step_size=None) -> None:
        """
        Set the model state from the next frame in the motion trajectory. `repeat` is
        set to `True` when the user would like to repeat the last frame.
        """
        # precondition
        if not all([self.model, self.motion, self.activity == Activity.MOTION_FOLLOW]):
            return
        print(self.motion_stepper)
        # tracks is_reversed and changes the direction of the motion accordingly.
        def sign(i):
            return -1 * i if self.is_reversed else i

        step_size = step_size or int(self.motion.fps / self.draw_fps)

        # repeat
        if not repeat:
            # iterate the frame counter
            self.motion_stepper = (
                self.motion_stepper + sign(step_size)
            ) % self.motion.num_frames()

        (
            new_pose,
            new_root_translate,
            new_root_rotation,
        ) = self.convert_CMUamass_single_pose(
            self.motion.poses[abs(self.motion_stepper)], self.model
        )

        self.model.joint_positions = new_pose
        self.model.rotation = new_root_rotation
        self.model.translation = new_root_translate

    def convert_CMUamass_single_pose(
        self, pose, model, raw=False
    ) -> Tuple[List[float], mn.Vector3, mn.Quaternion]:
        """
        This conversion is specific to the datasets from CMU
        """
        new_pose = []

        # Root joint
        root_T = pose.get_transform(ROOT, local=False)

        final_rotation_correction = mn.Quaternion()

        if not raw:
            final_rotation_correction = (
                self.global_correction_quat(mn.Vector3.z_axis(), mn.Vector3.x_axis())
                * self.rotation_offset
            )

        root_rotation = final_rotation_correction * mn.Quaternion.from_matrix(
            mn.Matrix3x3(root_T[0:3, 0:3])
        )
        root_translation = (
            self.translation_offset
            + final_rotation_correction.transform_vector(root_T[0:3, 3])
        )

        Q, _ = conversions.T2Qp(root_T)

        # Other joints
        for model_link_id in model.get_link_ids():
            joint_type = model.get_link_joint_type(model_link_id)
            joint_name = model.get_link_name(model_link_id)
            pose_joint_index = pose.skel.index_joint[joint_name]

            # When the target joint do not have dof, we simply ignore it
            if joint_type == phy.JointType.Fixed:
                continue

            # When there is no matching between the given pose and the simulated character,
            # the character just tries to hold its initial pose
            if pose_joint_index is None:
                raise KeyError(
                    "Error: pose data does not have a transform for that joint name"
                )
            elif joint_type not in [phy.JointType.Spherical]:
                raise NotImplementedError(
                    f"Error: {joint_type} is not a supported joint type"
                )
            else:
                T = pose.get_transform(pose_joint_index, local=True)
                if joint_type == phy.JointType.Spherical:
                    Q, _ = conversions.T2Qp(T)

            new_pose += list(Q)

        return new_pose, root_translation, root_rotation

    def setup_key_frames(self, num_key_frames: int = 10) -> None:
        """
        Prepares key frame data from the currently loaded motion data.
        """
        key_frames = [self.motion.poses[0]]

        total_dist = 0.0
        last_key_frame = self.motion.poses[0].get_transform(ROOT, local=False)[0:3, 3]

        # get total distance traveled by motion
        for pose in self.motion.poses:
            delta = pose.get_transform(ROOT, local=False)[0:3, 3] - last_key_frame
            total_dist += mn.Vector3(delta).length()
            last_key_frame = pose.get_transform(ROOT, local=False)[0:3, 3]

        # how much distance should occur between each key frame for the allotted frame count
        threshold = total_dist / num_key_frames
        last_key_frame = self.motion.poses[0].get_transform(ROOT, local=False)[0:3, 3]

        for pose in self.motion.poses:
            delta = pose.get_transform(ROOT, local=False)[0:3, 3] - last_key_frame
            if mn.Vector3(delta).length() >= threshold:
                key_frames.append(pose)
                last_key_frame = pose.get_transform(ROOT, local=False)[0:3, 3]

        self.key_frames = key_frames

    def toggle_key_frames(self) -> None:
        """
        Toggle the display of the key frames preview
        """
        if not self.motion:
            logger.error("Error: a motion is not loaded")
            return

        if (
            self.preview_mode in [Preview.KEYFRAMES, Preview.ALL]
            and not self.key_frame_models
        ):
            data = self.user_metadata

            for k in self.key_frames:

                self.key_frame_models.append(
                    self.art_obj_mgr.add_articulated_object_from_urdf(
                        filepath=data["urdf_path"], fixed_base=True
                    )
                )

                (
                    new_pose,
                    new_root_translate,
                    new_root_rotation,
                ) = self.convert_CMUamass_single_pose(k, self.key_frame_models[LAST])

                self.key_frame_models[LAST].motion_type = phy.MotionType.KINEMATIC
                self.key_frame_models[LAST].joint_positions = new_pose
                self.key_frame_models[LAST].rotation = new_root_rotation
                self.key_frame_models[LAST].translation = new_root_translate

        elif self.key_frame_models:
            for m in self.key_frame_models:
                # removes key frames
                self.art_obj_mgr.remove_object_by_handle(m.handle)
            self.key_frame_models = []

    def build_trajectory_vis(self) -> None:
        """
        Build and display the trajectory visuals for the key frames
        """
        if not self.motion:
            logger.error("Error: a motion is not loaded")
            return

        points_to_preview = []

        if len(self.key_frames) >= 2:
            traj_radius = 0.02
            traj_offset = self.translation_offset

            joint_names = [["rankle", "lankle"], "upperneck", "root"]

            final_rotation_correction = (
                self.global_correction_quat(mn.Vector3.z_axis(), mn.Vector3.x_axis())
                * self.rotation_offset
            )

            def define_preview_points(joint_names: List[str]) -> List[mn.Vector3]:
                """
                Pass in a list containing names of joints as strings and/or lists containing
                multiple names of joints, where the lists will result in an interpolated
                trajectory object being built from the mid point of the listed names and the
                single joint names will produce there own trajectory object. The total amount
                of trajectory objects that are produced is the length of the outer list.
                Example: joint_names = [["rankle", "lankle"], "upperneck"]

                Returns a list of lists of points to build the trajectory objects.
                """
                traj_objects = []
                joint_list = list(self.key_frames[0].skel.index_joint.keys())
                for joints in joint_names:
                    if not isinstance(joints, list):
                        joints = [joints]

                    # check if the joint names in this list are valid joints in the model
                    if not all(x in joint_list for x in joints):
                        logger.error(f"Error: Invalid joint names passed -> {joints}")
                        continue

                    midpoints = []
                    for i in self.key_frames:
                        mp = mn.Vector3.zero_init()
                        for j in joints:
                            mp += mn.Vector3(
                                i.get_transform(i.skel.index_joint[j], local=False)[
                                    0:3, 3
                                ]
                            )

                        mp /= len(joints)

                        midpoints.append(
                            traj_offset + final_rotation_correction.transform_vector(mp)
                        )
                    traj_objects.append(midpoints)
                return traj_objects

            points_to_preview = define_preview_points(joint_names)

        colors = [mn.Color3.red(), mn.Color3.yellow(), mn.Color3.green()]

        if self.preview_mode in [Preview.TRAJECTORY, Preview.ALL]:
            if not self.traj_ids:
                for i, p in enumerate(points_to_preview):
                    self.traj_ids.append(
                        self.sim.add_gradient_trajectory_object(
                            traj_vis_name=f"{joint_names[i]}{int(time.time() * 1000)}",
                            colors=colors,
                            points=p,
                            num_segments=3,
                            radius=traj_radius,
                            smooth=True,
                            num_interpolations=10,
                        )
                    )

        elif self.traj_ids:
            # removes trajectory
            for t_id in self.traj_ids:
                self.sim.get_rigid_object_manager().remove_object_by_id(t_id)
                self.traj_ids = []

    def cycle_model_previews(self) -> None:
        """
        Cycles through the Preview mode enum to display model/motion previews.
        """
        self.preview_mode = Preview((self.preview_mode.value + 1) % len(Preview))
        self.toggle_key_frames()
        self.build_trajectory_vis()

    def belongs_to(self, obj_id: int) -> bool:
        """
        Accepts an object id and returns True if the obj_id belongs to an object
        owned by this Fairmotion character.
        """
        # checking our model links
        if self.model and obj_id in self.model.link_object_ids:
            return True

        # checking our model
        if self.model and obj_id is self.model.object_id:
            return True

        # checking all key frame models
        if any(
            obj_id in ko_ids
            for ko_ids in (i.link_object_ids.keys() for i in self.key_frame_models)
        ):
            return True

        # checking all key frame models
        if any(obj_id == to for to in self.traj_ids):
            return True

        return False

    def push_action_order(self) -> None:
        """
        Place an action order at the end of the order queue to be consumed by process_action_order().
        """
        pf = self.sim.pathfinder
        queue = self.order_queue
        test_locations = [  # Locations in apt0 scene
            [[2.63, 0.0, 0.13], None],  # Front of Stairs
            [[-1.15, 0.0, -3.48], None],  # End of Hall
            [[-1.35, 0.0, 1.56], None],  # Kitchen Mat
            [[2.74, 0.0, 7.24], mn.Vector3.z_axis()],  # Wash TV
            [[2.30, 0.0, 4.616], None],  # Living Room Rug
            [[-1.09, 0.0, 3.00], None],  # Front of Fridge
            [[2.79, 0.0, 2.52], None],  # Corridor
        ]

        def snap_to_NM(point) -> mn.Vector3:
            pf = self.sim.pathfinder

            # drop location below NavMesh to guarantee bottom floor snap
            point = mn.Vector3(point) + mn.Vector3(0.0, -2.0, 0.0)

            # snap point to NavMesh
            point = pf.snap_point(point)

            return point

        action_orders_look_up = {
            "clean tv": ActionOrder(
                Motions.wash_window, snap_to_NM([2.74, 0.0, 7.00]), mn.Vector3.z_axis()
            ),
            "drink kitchen": ActionOrder(
                Motions.drink_beverage, snap_to_NM([-1.35, 0.0, 1.56]), None
            ),
            "sweep corridor": ActionOrder(
                Motions.sweep_floor, snap_to_NM([2.79, 0.0, 2.52]), None
            ),
            "reach fridge": ActionOrder(
                Motions.reach_for,
                snap_to_NM([-1.09, 0.0, 3.00]),
                -(mn.Vector3.x_axis()),
            ),
            "exercise hall": ActionOrder(
                Motions.jumping_jacks,
                snap_to_NM([-1.15, 0.0, -3.48]),
                mn.Vector3.x_axis(),
            ),
        }

        location = random.choice(test_locations)

        # drop location below NavMesh to guarantee bottom floor snap
        location[0] = mn.Vector3(location[0]) + mn.Vector3(0.0, -2.0, 0.0)

        # snap point to NavMesh
        location[0] = pf.snap_point(location[0])

        # push order queue
        # queue.append(ActionOrder(Motions.walk_to_walk, location[0], facing=location[1]))
        queue.append(action_orders_look_up["clean tv"])
        queue.append(action_orders_look_up["exercise hall"])
        queue.append(action_orders_look_up["reach fridge"])
        queue.append(action_orders_look_up["drink kitchen"])

    def process_action_order(self) -> None:
        """
        Pop an action order of the order queue and handle it as necessary.
        """
        # precondition
        if not all([self.model]):
            return

        Timer.start()

        # percent of draw window to use for processing
        THRESHOLD = 0.50

        # check for incomplete_order
        if self.incomplete_order:
            # continue where we left off
            action_order = self.incomplete_order[0]

            if action_order.motion_data.type == MType.TRANSITIVE:
                # handle continuing MType.TRANSITIVE
                path_ = self.incomplete_order[1]
                motion_ = action_order.motion_data

            elif action_order.motion_data.type == MType.SCENIC:
                # handle continuing MType.SCENIC
                time_ = self.incomplete_order[1]
                motion_ = action_order.motion_data

            self.incomplete_order = []

        else:
            # pop ActionOrder
            if self.order_queue:
                action_order = self.order_queue.pop(0)
            else:
                return

            if action_order.motion_data.type == MType.TRANSITIVE:
                # flush zero length paths
                if self.last_seq_location and (
                    (
                        mn.Vector3(self.last_seq_location)
                        - mn.Vector3(action_order.location)
                    ).length()
                    < 0.25
                ):
                    return

                # utility variables
                path_ = self.find_short_path_to_goal(
                    action_order.location, self.last_seq_location
                )
                motion_ = action_order.motion_data

            elif action_order.motion_data.type == MType.SCENIC:
                if (
                    action_order.location is not None
                    and self.last_seq_location
                    and (
                        (
                            mn.Vector3(self.last_seq_location)
                            - mn.Vector3(action_order.location)
                        ).length()
                        > 0.15
                    )
                ):
                    # Scenic Action Order AO(facing, Motions.(scenic))
                    scenic_AO = ActionOrder(
                        action_order.motion_data, facing=action_order.facing
                    )
                    self.order_queue.insert(0, scenic_AO)

                    # Transitive Action Order AO(location, facing, Motions.(scenic).transitive_motion_)
                    print(action_order.motion_data.transitive_motion_.num_of_frames)
                    transit_AO = ActionOrder(
                        action_order.motion_data.transitive_motion_,
                        facing=action_order.facing,
                        location=action_order.location,
                    )
                    self.order_queue.insert(0, transit_AO)
                    print("added two AOs via Scenic Split")
                    return

                motion_ = action_order.motion_data
                time_ = 0.0

        ################# CODE FOR TRANSITIVE MOTION #################

        ### CODE FOR TRANSITIVE MOTION ### [maybe turn into a local function]
        if action_order.motion_data.type == MType.TRANSITIVE:
            # track whether action is finised being processed
            finished_processing = False

            while Timer.check() < (THRESHOLD / self.draw_fps):
                # either take argument value for path_time or continue with cycle
                path_.time = path_.time + (1.0 / self.draw_fps)

                mocap_cycles_past = math.floor(path_.time / motion_.time_length)
                mocap_time_curr = math.fmod(path_.time, motion_.time_length)
                mocap_frame = int(mocap_time_curr * motion_.fps)

                # find distance progressed along shortest path
                path_displaced = (
                    mocap_cycles_past * motion_.map_of_total_displacement[LAST]
                    + motion_.map_of_total_displacement[mocap_frame]
                )
                if path_displaced > path_.length:
                    # go to next action
                    path_displaced = 0.0
                    finished_processing = True
                    break

                new_pose, _, _ = self.convert_CMUamass_single_pose(
                    motion_.poses[mocap_frame], self.model, raw=True
                )

                global_neutral_correction = self.global_correction_quat(
                    mn.Vector3.z_axis(), motion_.direction_forward
                )
                # character's root node position on the line and the forward direction of the path
                char_pos, forward_V = self.point_at_path_t(path_.points, path_displaced)
                look_at_path_T = mn.Matrix4.look_at(
                    char_pos, char_pos + forward_V.normalized(), mn.Vector3.y_axis()
                )

                full_transform = motion_.poses[mocap_frame].get_transform(
                    ROOT, local=True
                )
                full_transform = mn.Matrix4(full_transform)
                full_transform.translation -= motion_.center_of_root_drift
                full_transform = (
                    mn.Matrix4.from_(
                        global_neutral_correction.to_matrix(), mn.Vector3()
                    )
                    @ full_transform
                )

                # while transform is facing -Z, remove forward displacement
                full_transform.translation *= mn.Vector3.x_axis() + mn.Vector3.y_axis()
                full_transform = look_at_path_T @ full_transform

                ## Pose Manipulation Section ##
                # Tail-end Pose Interpolation #
                # margin_p = 0.06  # %
                margin_d = 0.15  # length

                # interpolate first margin dist with standing pose
                if path_displaced < margin_d:
                    # T is a float (0.0, 1.0) representing progress end margin
                    t = 1 - ((path_displaced) / (margin_d))

                    # get pose of interpolating pose B
                    inter_pose, _, _ = self.convert_CMUamass_single_pose(
                        Motions.standing_pose, self.model, raw=True
                    )
                    new_pose = self.interpolate_pose(new_pose, inter_pose, t)

                # interpolate last margin dist with standing pose
                if path_displaced > (path_.length - margin_d):
                    # T is a float (0.0, 1.0) representing progress end margin
                    t = 1 - ((path_.length - path_displaced) / (margin_d))

                    # get pose of interpolating pose B
                    inter_pose, _, _ = self.convert_CMUamass_single_pose(
                        Motions.standing_pose, self.model, raw=True
                    )
                    new_pose = self.interpolate_pose(new_pose, inter_pose, t)
                """
                    [WIP]
                    # interpolate last margin% with facing
                    if action_order.facing is not None and path_displaced > ((1 - margin_p) * path_.length):
                        # T is a float (0.0, 1.0) representing progress end margin
                        t = (path_displaced - ((1 - margin_p) * path_.length)) / (margin_p * path_.length)

                        # remove y component and normalize
                        action_order.facing[1], forward_V[1] = 0.0, 0.0
                        action_order.facing = action_order.facing.normalized()
                        forward_V = forward_V.normalized()

                        angle = mn.math.angle(action_order.facing, forward_V)
                        rotate = mn.Matrix4.rotation_y(mn.Rad(angle.__mul__(t)))

                        full_transform = rotate @ full_transform

                        print("Implement facing interpolation now", mn.Rad(angle.__mul__(t)))
                """

                # push staging queue
                self.staging_queue.append([new_pose, full_transform])

            # Processing Snapshot #
            if finished_processing:
                print("finished processing transitive")

                self.incomplete_order = []
                self.last_seq_location = action_order.location
                print("ln827 self.last_loc", action_order.location)
                # self.last_seq_facing =
            else:
                # [ActionOrder, PathData]
                self.incomplete_order.append(action_order)
                self.incomplete_order.append(path_)

        ################# CODE FOR SCENIC MOTION #################

        ### CODE FOR SCENIC MOTION ### [maybe turn into a local function]
        elif action_order.motion_data.type == MType.SCENIC:
            # cache last location
            final_char_location = self.last_seq_location
            finished_processing = False

            while Timer.check() < (THRESHOLD / self.draw_fps):

                if time_ > action_order.motion_data.time_length:
                    finished_processing = True
                    break

                # get time that motion is at
                time_ = time_ + (1.0 / self.draw_fps)

                mocap_cycles_past = math.floor(time_ / motion_.time_length)
                mocap_time_curr = math.fmod(time_, motion_.time_length)
                mocap_frame = int(mocap_time_curr * motion_.fps)

                # center position to origin minus Z component
                pos_offset = motion_.start_translation * (
                    mn.Vector3(1.0, 1.0, 1.0) - motion_.direction_up
                )

                new_pose, _, _ = self.convert_CMUamass_single_pose(
                    motion_.poses[mocap_frame], self.model, raw=True
                )
                global_correction = self.global_correction_quat(
                    motion_.direction_up, motion_.direction_forward
                )

                # 0. get root transform
                print("mocap_frame", mocap_frame, "# of Frames ", motion_.num_of_frames)
                transform_ = mn.Matrix4(
                    motion_.poses[mocap_frame].get_transform(ROOT, local=True)
                )

                # 1. apply pos offset
                transform_.translation -= pos_offset

                # 2. apply global correction to get face -z up y
                transform_ = (
                    mn.Matrix4.from_(global_correction.to_matrix(), mn.Vector3())
                    @ transform_
                )

                # 3. apply look_at transformation
                last_loc = self.last_seq_location
                motion_facing = action_order.facing or mn.Vector3.z_axis()  # temp

                look_at_Transform = mn.Matrix4.look_at(
                    last_loc, last_loc + motion_facing, mn.Vector3.y_axis()
                )
                transform_ = look_at_Transform @ transform_
                final_char_location = transform_.translation

                # push staging queue
                self.staging_queue.append([new_pose, transform_])

            # Processing Snapshot #
            if finished_processing:
                self.incomplete_order = []
                self.last_seq_location = final_char_location
                # self.last_seq_facing =
            else:
                # [ActionOrder, float(time_past)]
                self.incomplete_order.append(action_order)
                self.incomplete_order.append(time_)

    def pop_staging_queue_and_pose(self) -> None:
        """
        Take a step of model data from the staging queue and load it into our model to
        progress on the sequence of Action Orders. Call this on the draw_event to play scheduled
        motion at simulation speed.
        """
        # precondition
        if not all(
            [self.model, self.staging_queue, self.activity == Activity.SEQUENCE]
        ):
            return

        # pop staging queue
        pose_data = self.staging_queue.pop(0)

        # pose_data = [joint_positions, transformation]
        self.model.joint_positions = pose_data[0]
        self.model.transformation = pose_data[1]

    def set_activity_to_SEQ(self):
        """[TEMPORARY]"""
        self.activity = Activity.SEQUENCE

    # def interpolate_poses(poseA, poseB, t) -> List[mn.Quaternion]:
    def interpolate_pose(self, poseA, poseB, t) -> List[mn.Quaternion]:
        """
        Pass in two motion pose structs and return them interpolated.
        """

        link_ids = self.model.get_link_ids()
        pos_i_range = [self.model.get_link_num_joint_pos(i) for i in link_ids]
        pos_i_offs = [self.model.get_link_joint_pos_offset(i) for i in link_ids]

        poseC = []

        for i in link_ids:
            if pos_i_range[i] != 4:
                continue
            Q_a = poseA[pos_i_offs[i] : pos_i_offs[i] + pos_i_range[i] - 1]
            Q_b = poseB[pos_i_offs[i] : pos_i_offs[i] + pos_i_range[i] - 1]

            Q_a = [Q_a, poseA[pos_i_offs[i] + pos_i_range[i] - 1]]
            Q_b = [Q_b, poseB[pos_i_offs[i] + pos_i_range[i] - 1]]

            Q_a = mn.Quaternion(Q_a)
            Q_b = mn.Quaternion(Q_b)

            Q_c = mn.math.slerp(Q_a, Q_b, t)

            poseC.append(Q_c.vector.x)
            poseC.append(Q_c.vector.y)
            poseC.append(Q_c.vector.z)
            poseC.append(Q_c.scalar)

        return poseC

    def setup_pathfollower(self, path: habitat_sim.ShortestPath()):
        """
        Prepare the pathfollowing character and any data needed to execute the update function.
        """
        self.hide_model()
        self.activity = Activity.PATH_FOLLOW_SEQ

        # Load a model to follow path
        self.model = self.art_obj_mgr.add_articulated_object_from_urdf(
            filepath=self.user_metadata["urdf_path"], fixed_base=True
        )
        self.model.motion_type = phy.MotionType.KINEMATIC

        self.state_motion = Motions.run_to_run

        # initiate path data
        self.path_data = PathData(path.points)

        # First update with step_size 0 to start character
        self.update_pathfollower_sequential(step_size=0)

    def update_pathfollower_sequential(
        self, step_size: int = 1, path_time: Optional[float] = None
    ) -> None:
        """
        When this method is called, the path follower model is updated to the next frame
        of the path following sequence along with the corresponding next frame of the motion
        cycle that was loaded in with `setup_pathfollower()`.
        """
        # precondition
        if not all(
            [self.model, self.path_data, self.activity == Activity.PATH_FOLLOW_SEQ]
        ):
            return

        path_ = self.path_data
        motion_ = self.state_motion

        # either take argument value for path_time or continue with cycle
        path_.time = path_time or (path_.time + (step_size / self.draw_fps))

        mocap_cycles_past = math.floor(path_.time / motion_.time_length)
        mocap_time_curr = math.fmod(path_.time, motion_.time_length)
        mocap_frame = int(mocap_time_curr * motion_.fps)

        # handle wrapping or edgecase for dath displacement passing goal
        # find distance progressed along shortest path
        path_displacement = (
            mocap_cycles_past * motion_.map_of_total_displacement[LAST]
            + motion_.map_of_total_displacement[mocap_frame]
        )

        print(path_.time, path_displacement, mocap_frame)

        if path_displacement > path_.length:
            path_.time = 0.0
            self.path_displacement = 0.0

        new_pose, _, _ = self.convert_CMUamass_single_pose(
            motion_.poses[mocap_frame], self.model, raw=True
        )

        global_neutral_correction = self.global_correction_quat(
            mn.Vector3.z_axis(), motion_.direction_forward
        )
        # character's root node position on the line and the forward direction of the path
        char_pos, forward_V = self.point_at_path_t(path_.points, path_displacement)
        look_at_path_T = mn.Matrix4.look_at(
            char_pos, char_pos + forward_V.normalized(), mn.Vector3.y_axis()
        )

        full_transform = motion_.poses[mocap_frame].get_transform(ROOT, local=True)
        full_transform = mn.Matrix4(full_transform)
        full_transform.translation -= motion_.center_of_root_drift
        full_transform = (
            mn.Matrix4.from_(global_neutral_correction.to_matrix(), mn.Vector3())
            @ full_transform
        )

        # while transform is facing -Z, remove forward displacement
        full_transform.translation *= mn.Vector3.x_axis() + mn.Vector3.y_axis()
        full_transform = look_at_path_T @ full_transform

        # apply joint angles
        self.model.joint_positions = new_pose
        self.model.transformation = full_transform

    def point_at_path_t(
        self, path_points: List[mn.Vector3], t: float
    ) -> Tuple[mn.Vector3, mn.Vector3]:
        """
        Function that give a point on the path at time t.
        """
        covered: float = 0.0
        i: int = 0
        j: int = 0

        while i < len(path_points):

            progress: float = (mn.Vector3(path_points[i] - path_points[j])).length()
            if t >= progress + covered:
                covered += progress
                j = i
                i += 1
            else:
                # we are finally pointing to the correct
                break

        # wrap just in case t is greater than total path length
        if i == len(path_points):
            return self.point_at_path_t(path_points, math.fmod(t, covered))

        offset = mn.Vector3(0.0, 0.9, 0.0)
        direction = (mn.Vector3(path_points[i] - path_points[j])).normalized()
        point = mn.Vector3(path_points[j] + ((t - covered) * direction)) + offset
        return point, direction

    def global_correction_quat(
        self, up_v: mn.Vector3, forward_v: mn.Vector3
    ) -> mn.Quaternion:
        """
        Given the upward direction and the forward direction of a local space frame, this methd produces
        the correction quaternion to convert the frame to global space (+Y up, -Z forward).
        """
        angle1 = mn.math.angle(up_v.normalized(), mn.Vector3.y_axis())
        axis1 = mn.math.cross(up_v.normalized(), mn.Vector3.y_axis())
        rotation1 = mn.Quaternion.rotation(angle1, axis1)

        forward_v = rotation1.transform_vector(forward_v)
        forward_v = forward_v * (mn.Vector3(1.0, 1.0, 1.0) - mn.Vector3.y_axis())
        angle2 = mn.math.angle(forward_v.normalized(), -1 * mn.Vector3.z_axis())
        axis2 = mn.Vector3.y_axis()
        rotation2 = mn.Quaternion.rotation(angle2, axis2)

        return rotation2 * rotation1

    # change return to PathData for typing, but currently it causes an error in Pylance``
    def find_short_path_to_goal(self, goal_point, start_point=None) -> PathData:
        """
        Finds two random points on the NavMesh, calculates a shortest path between
        the two, and creates a trajectory object to visualize the path.
        """
        pf = self.sim.pathfinder
        goal_point = pf.snap_point(goal_point)

        found_path = False
        while not found_path:

            while start_point is None:
                start_point = pf.get_random_navigable_point()
                start_point = pf.snap_point(start_point)

                # constraint points to be on first floor
                if start_point[1] != goal_point[1]:
                    logger.warn(
                        "Warning: start point is out of acceptable area, replacing randomly"
                    )

                    start_point = None

            start_point = pf.snap_point(start_point)

            path = habitat_sim.ShortestPath()
            path.requested_start = start_point
            path.requested_end = goal_point
            found_path = pf.find_path(path)
            start_point = None

        spline_points = habitat_sim.geo.build_catmull_rom_spline(path.points, 10, 0.75)

        return PathData(spline_points)
