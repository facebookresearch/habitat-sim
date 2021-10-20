from fairmotion.core import motion
from fairmotion.data import amass
from fairmotion.ops import conversions
from magnum import Deg, Matrix3x3, Quaternion, Vector3

import habitat_sim
from habitat_sim.physics import JointType

#### Constants
ROOT = 0

# I am including an outer key "name" to leave this open to multiple save files
DEFAULT_METADATA = {
    "default": {
        "urdf_path": "../habitat-sim/data/test_assets/urdf/amass_male.urdf",
        "amass_path": "../fairmotion/amass_test_data/CMU/CMU/06/06_12_poses.npz",
        "bm_path": "../fairmotion/amass_test_data/smplh/male/model.npz",
    }
}


class FairmotionInterface:
    def __init__(
        self, viewer, metadata=None, urdf_path=None, amass_path=None, bm_path=None
    ) -> None:

        self.viewer = viewer
        self.art_obj_mgr = self.viewer.sim.get_articulated_object_manager()
        self.model: habitat_sim.physics.ManagedArticulatedObject = None
        self.motion: motion.Motion = None
        self.metadata = {}
        self.motion_stepper = 0

        # positional offsets
        self.rotation_offset: Quaternion = Quaternion.rotation(
            Deg(-90), Vector3.x_axis()
        ) * Quaternion.rotation(
            Deg(90), Vector3.z_axis()
        )  # TEMPORARY HARDCODED
        self.translation_offset: Vector3 = Vector3(
            [2.5, 0.0, 0.7]
        )  # TEMPORARY HARDCODED

        self.metadata = DEFAULT_METADATA

        if metadata is not None:
            self.metadata = metadata
        else:
            self.set_data(urdf_path=urdf_path, amass_path=amass_path, bm_path=bm_path)

    def set_data(self, name="default", urdf_path=None, amass_path=None, bm_path=None):
        data = self.metadata[name]
        data["urdf_path"] = urdf_path or self.metadata["default"]["urdf_path"]
        data["amass_path"] = amass_path or self.metadata["default"]["amass_path"]
        data["bm_path"] = bm_path or self.metadata["default"]["bm_path"]

    def save_metadata():
        """
        Saves the current metadata to a txt file in given file path
        """

    def fetch_metadata():
        """
        Fetch metadata from a txt file in given file path and sets current metadata
        """

    def load_motion(self):
        data = self.metadata["default"]
        self.motion = amass.load(file=data["amass_path"], bm_path=data["bm_path"])

    def load_model(self):
        data = self.metadata["default"]

        # add an ArticulatedObject to the world with a fixed base
        self.model = self.art_obj_mgr.add_articulated_object_from_urdf(
            filepath=data["urdf_path"], fixed_base=True
        )
        assert self.model.is_alive

        # change motion_type to KINEMATIC
        self.model.motion_type = habitat_sim.physics.MotionType.KINEMATIC

        # TODO: Make this instead place the model infront of you
        # translate Human to appear infront of staircase in apt_0
        self.model.translate(self.translation_offset)

        # TEMPORARY FOR TESTING
        self.viewer.agent_body_node.translate(
            Vector3([2.44567, 0.119373, 3.42486])
            - self.viewer.agent_body_node.translation
        )  # TEMPORARY HARDCODED

    # currently the next_pose method is simply called twice in simulating a frame
    def next_pose(self):
        """
        Use this method to step to next frame in draw event
        """
        if not self.model or not self.motion:
            return

        (
            new_pose,
            new_root_translate,
            new_root_rotation,
        ) = self.convert_CMUamass_single_pose(self.motion.poses[self.motion_stepper])
        self.model.joint_positions = new_pose
        self.model.rotation = new_root_rotation
        self.model.translation = new_root_translate

        # iterate the frame counter
        self.motion_stepper += 1

        if self.motion_stepper >= self.motion.num_frames():
            self.motion_stepper = 0

    def convert_CMUamass_single_pose(self, pose):
        """
        This conversion is specific to the datasets from CMU
        """
        new_pose = []

        # Root joint
        root_T = pose.get_transform(ROOT, local=False)

        ### adding offsets to root transformation
        # rotation
        root_rotation = self.rotation_offset * Quaternion.from_matrix(
            Matrix3x3(root_T[0:3, 0:3])  # TEMPORARY HARDCODED
        )

        # translation
        root_translation = (  # TEMPORARY HARDCODED
            self.translation_offset
            + self.rotation_offset.transform_vector(root_T[0:3, 3])
        )  # correct

        Q, _ = conversions.T2Qp(root_T)

        # Other joints
        for model_link_id in self.model.get_link_ids():
            joint_type = self.model.get_link_joint_type(model_link_id)
            joint_name = self.model.get_link_name(model_link_id)
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
