# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import math
from typing import Any, Dict

from fairmotion.core import motion
from fairmotion.data import amass
from fairmotion.ops import conversions
from magnum import Color3, Deg, Matrix3x3, Quaternion, Vector3

import habitat_sim
from habitat_sim.physics import JointType

#### Constants
ROOT = 0
FILE_SUFFIX = "_fm_data"


class FairmotionInterface:
    def __init__(
        self, viewer, metadata_name=None, urdf_path=None, amass_path=None, bm_path=None
    ) -> None:

        self.viewer = viewer
        self.art_obj_mgr = self.viewer.sim.get_articulated_object_manager()
        self.model: habitat_sim.physics.ManagedArticulatedObject = None
        self.motion: motion.Motion = None
        self.metadata = {}
        self.motion_stepper = 0
        self.setup_default_metadata()

        self.key_frames = None
        self.key_frame_models = []
        self.show_key_frames = False
        self.traj_id: int = None
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
        self.rotation_offset: Quaternion = self.metadata["default"]["rotation"]
        self.translation_offset: Vector3 = self.metadata["default"]["translation"]

        self.load_motion()

    def setup_default_metadata(self):
        # I am including an outer key "name" to leave this open to multiple save files
        METADATA_DEFAULT = {
            "default": {
                "urdf_path": "../habitat-sim/data/test_assets/urdf/amass_male.urdf",
                "amass_path": "../fairmotion/amass_test_data/CMU/CMU/06/06_12_poses.npz",
                "bm_path": "../fairmotion/amass_test_data/smplh/male/model.npz",
                "rotation": Quaternion.rotation(Deg(-90), Vector3.x_axis())
                * Quaternion.rotation(Deg(90), Vector3.z_axis()),
                "translation": Vector3([2.5, 0.0, 0.7]),
            }
        }

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
    ):

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
                if type(v) == Quaternion:
                    q_list = [list(v.vector)]
                    q_list.append(v.scalar)
                    # Tuple[Tuple[float, float, float], float]
                    data[k] = q_list

                elif type(v) == Vector3:
                    data[k] = list(v)

        elif not to_file:
            for k, v in data.items():
                if k == "rotation":
                    # Tuple[Tuple[float, float, float], float]
                    data[k] = Quaternion(v)
                elif k == "translation":
                    data[k] = Vector3(v)
        return data

    def save_metadata(self, name: str):
        """
        Saves the current metadata to a txt file in given file path
        """
        import json

        filepath = f"../habitat-sim/examples/fairmotion_data/{name + FILE_SUFFIX}.json"
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
            print(f"name of metadata after slice => {name}")
        else:
            # including FILE_SUFFIX if not included in name
            filename = name + FILE_SUFFIX

        filepath = f"../habitat-sim/examples/fairmotion_data/{filename}.json"

        try:
            with open(filepath, "r") as file:
                data = json.load(file)
        except Exception:
            raise Exception("Error: File does not appear to exist.")

        self.metadata[name] = self.metadata_parser(data[name], to_file=False)

    def set_transform_offsets(
        self, rotate_offset: Quaternion = None, translate_offset: Vector3 = None
    ):
        self.rotation_offset = rotate_offset or self.rotation_offset
        self.translation_offset = translate_offset or self.translation_offset
        self.next_pose(repeat=True)

    def load_motion(self):
        data = self.metadata["default"]
        self.motion = amass.load(file=data["amass_path"], bm_path=data["bm_path"])

        self.setup_key_frames()

    def load_model(self):
        self.hide_model()
        data = self.metadata["default"]

        # add an ArticulatedObject to the world with a fixed base
        self.model = self.art_obj_mgr.add_articulated_object_from_urdf(
            filepath=data["urdf_path"], fixed_base=True
        )
        assert self.model.is_alive

        print(f"model = {((self.model.root_scene_node.translation))}")
        self.model.root_scene_node.translate(Vector3([5, 5, 5]))
        print(f"model = {((self.model.root_scene_node.translation))}")
        # change motion_type to KINEMATIC
        self.model.motion_type = habitat_sim.physics.MotionType.KINEMATIC

        # TODO: Make this instead place the model infront of you
        # translate Human to appear infront of staircase in apt_0
        self.model.translation = self.translation_offset
        self.next_pose()
        self.motion_stepper -= 1

    def hide_model(self):
        if self.model:
            self.art_obj_mgr.remove_object_by_handle(self.model.handle)

    # currently the next_pose method is simply called twice in simulating a frame
    def next_pose(self, repeat=False):
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

    def convert_CMUamass_single_pose(self, pose, model):
        """
        This conversion is specific to the datasets from CMU
        """
        new_pose = []

        # Root joint
        root_T = pose.get_transform(ROOT, local=False)

        ### adding offsets to root transformation
        # rotation
        root_rotation = self.rotation_offset * Quaternion.from_matrix(
            Matrix3x3(root_T[0:3, 0:3])
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

    def setup_key_frames(self, num_key_frames=10):

        key_frames = [self.motion.poses[0]]

        # euclidean distance
        def distance(translation: Vector3):
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

    def toggle_key_frames(self):
        """
        Toggle the display of the key frames preview
        """
        self.show_key_frames = not self.show_key_frames
        if self.show_key_frames:
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

            self.buildTrajectoryVis()

        else:
            for m in self.key_frame_models:
                # removes key frames
                self.art_obj_mgr.remove_object_by_handle(m.handle)

            # removes trajectory
            self.viewer.sim.get_rigid_object_manager().remove_object_by_id(self.traj_id)

    def buildTrajectoryVis(self):
        """
        Build and display the trajectory visuals for the key frames
        """
        if len(self.key_frames) >= 2:
            traj_radius = 0.02
            traj_offset = self.translation_offset + Vector3(0, 0, 0)

            root_points = [
                Vector3(x.get_transform(ROOT, local=False)[0:3, 3])
                for x in self.key_frames
            ]
            print(f"root_points[0] = {root_points[0]}")
            root_points = [
                traj_offset + self.rotation_offset.transform_vector(x)
                for x in root_points
            ]

            # TODO: This function is not working. It is supposed to produce a gradient
            #       from RED to YELLOW to GREEN but it is producing a black solely
            colors = [
                Color3(255.0, 0.0, 0.0),
                Color3(255.0, 255.0, 0.0),
                Color3(0.0, 255.0, 0.0),
            ]
            self.traj_id = self.viewer.sim.add_gradient_trajectory_object(
                traj_vis_name="key_frame_traj",
                colors=colors,
                points=root_points,
                num_segments=3,
                radius=traj_radius,
                smooth=False,
                num_interpolations=10,
            )
