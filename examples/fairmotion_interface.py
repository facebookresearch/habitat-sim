# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import math
import os
from enum import Enum
from typing import Any, Dict, List, Tuple

import magnum as mn
from fairmotion.core import motion
from fairmotion.data import amass
from fairmotion.ops import conversions

import habitat_sim
from habitat_sim.logging import LoggingContext, logger
from habitat_sim.physics import JointType

#### Constants
ROOT = 0
FILE_SUFFIX = "_fm_data"
METADATA_DEFAULT = {
    "default": {
        "urdf_path": "../habitat-sim/data/test_assets/urdf/amass_male.urdf",
        "amass_path": "../fairmotion/amass_test_data/CMU/CMU/06/06_12_poses.npz",
        "bm_path": "../fairmotion/amass_test_data/smplh/male/model.npz",
        "rotation": mn.Quaternion.rotation(mn.Deg(-90), mn.Vector3.x_axis())
        * mn.Quaternion.rotation(mn.Deg(90), mn.Vector3.z_axis()),
        "translation": mn.Vector3([2.5, 0.0, 0.7]),
    }
}
METADATA_DIR = "../habitat-sim/data/fairmotion/"


class FairmotionInterface:
    def __init__(
        self,
        sim,
        metadata_name=None,
        urdf_path=None,
        amass_path=None,
        bm_path=None,
        metadata_dir=METADATA_DIR,
    ) -> None:
        LoggingContext.reinitialize_from_env()
        self.sim = sim
        self.art_obj_mgr = self.sim.get_articulated_object_manager()
        self.model: habitat_sim.physics.ManagedArticulatedObject = None
        self.motion: motion.Motion = None
        self.metadata = {}
        self.metadata_dir = metadata_dir or METADATA_DIR
        self.motion_stepper = 0
        self.setup_default_metadata()

        self.key_frames = None
        self.key_frame_models = []
        self.preview_mode = Preview.OFF
        self.traj_ids: List[int] = []
        self.is_reversed = False

        # loading from file if given
        # if file doesn't exist, make a new set of data with
        # metadata_name as name
        if metadata_name is not None:
            try:
                self.fetch_metadata(metadata_name)
                self.set_data(name=metadata_name)
            except Exception:
                Exception(f"No file with name {metadata_name}, creating new file.")
                self.set_data(
                    name=metadata_name,
                    urdf_path=urdf_path,
                    amass_path=amass_path,
                    bm_path=bm_path,
                )
                self.save_metadata(metadata_name)
        else:
            # This sets the instance defaults with init(args)
            self.set_data(urdf_path=urdf_path, amass_path=amass_path, bm_path=bm_path)

        # positional offsets
        self.rotation_offset: mn.Quaternion = self.metadata["default"]["rotation"]
        self.translation_offset: mn.Vector3 = self.metadata["default"]["translation"]

        self.load_motion()

    def setup_default_metadata(self) -> None:
        """
        Called by init(), this method forces the default metadata into the metadata library.
        """
        # If folder doesn't exist, then create it.
        if not os.path.isdir(self.metadata_dir):
            os.makedirs(self.metadata_dir)

        self.metadata["default"] = METADATA_DEFAULT["default"]
        self.save_metadata("default")
        self.fetch_metadata("default")

    def set_data(
        self,
        name="default",
        urdf_path=None,
        amass_path=None,
        bm_path=None,
        rotation=None,
        translation=None,
    ) -> None:
        """
        A method that will take attributes of the model data sets as arguments, filling in
        empty args with the default set of data, and adds the data to the meta_data library.
        """

        data = None
        # if name is default, set data for "default"
        # else if name is in metadata, set data values to args or default values
        if name in self.metadata:
            data = self.metadata[name]

        # else if name not in metadata, build data from name, args, default values, and add to lib
        else:
            self.metadata[name] = {}
            data = self.metadata[name]

        data["urdf_path"] = urdf_path or self.metadata["default"]["urdf_path"]
        data["amass_path"] = amass_path or self.metadata["default"]["amass_path"]
        data["bm_path"] = bm_path or self.metadata["default"]["bm_path"]
        data["rotation"] = rotation or self.metadata["default"]["rotation"]
        data["translation"] = translation or self.metadata["default"]["translation"]

    def metadata_parser(self, metadata_dict: Dict[str, Any], to_file: bool):
        """
        Convert metadata dict to and from a form that is json serializable
        """
        data = metadata_dict.copy()

        if to_file:
            for k, v in data.items():
                if type(v) == mn.Quaternion:
                    q_list = [list(v.vector)]
                    q_list.append(v.scalar)
                    data[k] = q_list
                elif type(v) == mn.Vector3:
                    data[k] = list(v)

        elif not to_file:
            for k, v in data.items():
                if k == "rotation":
                    data[k] = mn.Quaternion(v)
                elif k == "translation":
                    data[k] = mn.Vector3(v)
        return data

    def save_metadata(self, name: str):
        """
        Saves the current metadata to a txt file in given file path
        """
        import json

        filepath = f"{self.metadata_dir}{name + FILE_SUFFIX}.json"
        data = {name: self.metadata_parser(self.metadata[name], to_file=True)}

        with open(filepath, "w") as file:
            json.dump(data, file)

    def fetch_metadata(self, name):
        """
        Fetch metadata from a json file in given file name and sets current metadata
        """
        import json

        if FILE_SUFFIX in name and len(name) > len(FILE_SUFFIX):
            # if name give is full name of file, truncate FILE_SUFFIX
            filename = name
            name = name[: -len(FILE_SUFFIX)]
        else:
            # including FILE_SUFFIX if not included in name
            filename = name + FILE_SUFFIX

        filepath = f"{self.metadata_dir}{filename}.json"

        try:
            with open(filepath, "r") as file:
                data = json.load(file)
        except Exception:
            raise Exception("Error: File does not appear to exist.")

        self.metadata[name] = self.metadata_parser(data[name], to_file=False)

    def set_transform_offsets(
        self, rotate_offset: mn.Quaternion = None, translate_offset: mn.Vector3 = None
    ) -> None:
        """
        This method updates the offset of the model with the positional data passed to it.
        Use this for changing the locaton and orientation of the model.
        """
        self.rotation_offset = rotate_offset or self.rotation_offset
        self.translation_offset = translate_offset or self.translation_offset
        self.next_pose(repeat=True)

    def load_motion(self) -> None:
        """
        Loads the motion currently set by metadata.
        """
        data = self.metadata["default"]
        self.motion = amass.load(file=data["amass_path"], bm_path=data["bm_path"])
        self.setup_key_frames()
        self.build_trajectory_vis()

    def load_model(self) -> None:
        """
        Loads the model currently set my metadata.
        """
        self.hide_model()
        data = self.metadata["default"]

        # add an ArticulatedObject to the world with a fixed base
        self.model = self.art_obj_mgr.add_articulated_object_from_urdf(
            filepath=data["urdf_path"], fixed_base=True
        )
        assert self.model.is_alive

        # change motion_type to KINEMATIC
        self.model.motion_type = habitat_sim.physics.MotionType.KINEMATIC

        # translate Human to appear infront of staircase in apt_0
        self.model.translation = self.translation_offset
        self.next_pose()
        self.motion_stepper -= 1

    def hide_model(self) -> None:
        """
        Removes model from scene.
        """
        if self.model:
            self.art_obj_mgr.remove_object_by_handle(self.model.handle)
            self.model = None

    # currently the next_pose method is simply called twice in simulating a frame
    def next_pose(self, repeat=False) -> None:
        """
        Use this method to step to next frame in draw event
        """

        # This function tracks is_reversed and changes the direction of
        # the motion accordingly.
        def sign(i):
            return -1 * i if self.is_reversed else i

        # repeat last frame: used mostly for position state change
        if repeat:
            self.motion_stepper = (
                self.motion_stepper - sign(1)
            ) % self.motion.num_frames()

        if not self.model or not self.motion:
            return

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

        # iterate the frame counter
        self.motion_stepper = (self.motion_stepper + sign(1)) % self.motion.num_frames()

    def convert_CMUamass_single_pose(
        self, pose, model
    ) -> Tuple[List[float], mn.Vector3, mn.Quaternion]:
        """
        This conversion is specific to the datasets from CMU
        """
        new_pose = []

        # Root joint
        root_T = pose.get_transform(ROOT, local=False)

        # adding offsets to root transformation
        # rotation
        root_rotation = self.rotation_offset * mn.Quaternion.from_matrix(
            mn.Matrix3x3(root_T[0:3, 0:3])
        )

        # translation
        root_translation = (
            self.translation_offset
            + self.rotation_offset.transform_vector(root_T[0:3, 3])
        )

        Q, _ = conversions.T2Qp(root_T)

        # Other joints
        for model_link_id in model.get_link_ids():
            joint_type = model.get_link_joint_type(model_link_id)
            joint_name = model.get_link_name(model_link_id)
            pose_joint_index = pose.skel.index_joint[joint_name]

            # When the target joint do not have dof, we simply ignore it
            if joint_type == JointType.Fixed:
                continue

            # When there is no matching between the given pose and the simulated character,
            # the character just tries to hold its initial pose
            if pose_joint_index is None:
                raise NotImplementedError(
                    "Error: pose data does not have a transform for that joint name"
                )
            elif joint_type not in [JointType.Spherical]:
                raise NotImplementedError(
                    f"Error: {joint_type} is not a supported joint type"
                )
            else:
                T = pose.get_transform(pose_joint_index, local=True)
                if joint_type == JointType.Spherical:
                    Q, _ = conversions.T2Qp(T)

            new_pose += list(Q)
        return new_pose, root_translation, root_rotation

    def setup_key_frames(self, num_key_frames: int = 10) -> None:
        """
        Prepares key frame data from the currently loaded motion data.
        """
        key_frames = [self.motion.poses[0]]

        # euclidean distance
        def distance(translation: mn.Vector3) -> float:
            summ = 0
            for n in translation:
                summ += n * n
            return math.sqrt(summ)

        total_dist = 0.0
        last_key_frame = self.motion.poses[0].get_transform(ROOT, local=False)[0:3, 3]

        # get total distance traveled by motion
        for pose in self.motion.poses:
            delta = pose.get_transform(ROOT, local=False)[0:3, 3] - last_key_frame
            total_dist += distance(delta)
            last_key_frame = pose.get_transform(ROOT, local=False)[0:3, 3]

        # how much distance should occur between each key frame for the allotted frame count
        threshold = total_dist / num_key_frames
        last_key_frame = self.motion.poses[0].get_transform(ROOT, local=False)[0:3, 3]

        for pose in self.motion.poses:
            delta = pose.get_transform(ROOT, local=False)[0:3, 3] - last_key_frame
            if distance(delta) >= threshold:
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
            data = self.metadata["default"]

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
                ) = self.convert_CMUamass_single_pose(k, self.key_frame_models[-1])

                self.key_frame_models[
                    -1
                ].motion_type = habitat_sim.physics.MotionType.KINEMATIC
                self.key_frame_models[-1].joint_positions = new_pose
                self.key_frame_models[-1].rotation = new_root_rotation
                self.key_frame_models[-1].translation = new_root_translate

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
            traj_offset = self.translation_offset + mn.Vector3(0, 0.1, 0)

            def define_preview_points(joint_names: List[str]) -> List[mn.Vector3]:
                """
                Pass in a list containing names of joints as strings, where the joints will
                result in an interpolated trajectory object being built from the mid point of
                the listed names. This would have more functionality if multiple trajectory
                objects could be added at once but currently only one can me added at a time.
                Example: joint_names = ["rankle", "lankle"]
                         joint_names = ["upperneck"]

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
                            traj_offset + self.rotation_offset.transform_vector(mp)
                        )
                    traj_objects.append(midpoints)
                return traj_objects

            points_to_preview = define_preview_points(["lankle", "rankle"])

        # TODO: This function is not working. It is supposed to produce a gradient
        #       from RED to YELLOW to GREEN but it is producing a black solely
        colors = [
            mn.Color3(255.0, 0.0, 0.0),
            mn.Color3(255.0, 255.0, 0.0),
            mn.Color3(0.0, 255.0, 0.0),
        ]

        if self.preview_mode in [Preview.TRAJECTORY, Preview.ALL]:
            if not self.traj_ids:
                for p in points_to_preview:
                    self.traj_ids.append(
                        self.sim.add_gradient_trajectory_object(
                            traj_vis_name="key_frame_traj",
                            colors=colors,
                            points=p,
                            num_segments=3,
                            radius=traj_radius,
                            smooth=False,
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


class Preview(Enum):
    OFF = 0
    KEYFRAMES = 1
    TRAJECTORY = 2
    ALL = 3
