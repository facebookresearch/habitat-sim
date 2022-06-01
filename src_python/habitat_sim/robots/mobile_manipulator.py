from collections import defaultdict
from typing import Dict, List, Optional, Set, Tuple

import attr
import magnum as mn
import numpy as np

from habitat_sim.physics import JointMotorSettings
from habitat_sim.robots.robot_interface import RobotInterface
from habitat_sim.simulator import Simulator
from habitat_sim.utils.common import orthonormalize_rotation_shear


@attr.s(auto_attribs=True, slots=True)
class RobotCameraParams:
    """Data to configure a camera placement on the robot.
    :property attached_link_id: Which link ID this camera is attached to, -1
        for the base link.
    :property cam_offset_pos: The 3D position of the camera relative to the
        transformation of the attached link.
    :property cam_look_at_pos: The 3D of where the camera should face relative
        to the transformation of the attached link.
    :property relative_transform: An added local transform for the camera.
    """

    attached_link_id: int
    cam_offset_pos: mn.Vector3
    cam_look_at_pos: mn.Vector3
    relative_transform: mn.Matrix4 = mn.Matrix4.identity_init()


# TODO: refactor this class to support spherical joints: multiple dofs per link and #dofs != #positions
@attr.s(auto_attribs=True, slots=True)
class MobileManipulatorParams:
    """Data to configure a mobile manipulator.
    :property arm_joints: The joint ids of the arm joints.
    :property gripper_joints: The habitat sim joint ids of any grippers.
    :property wheel_joints: The joint ids of the wheels. If the wheels are not controlled, then this should be None

    :property arm_init_params: The starting joint angles of the arm. If None,
        resets to 0.
    :property gripper_init_params: The starting joint positions of the gripper. If None,
        resets to 0.

    :property ee_offset: The 3D offset from the end-effector link to the true
        end-effector position.
    :property ee_link: The Habitat Sim link ID of the end-effector.
    :property ee_constraint: A (2, N) shaped array specifying the upper and
        lower limits for each end-effector joint where N is the arm DOF.

    :property cameras: The cameras and where they should go. The key is the
        prefix to match in the sensor names. For example, a key of `"robot_head"`
        will match sensors `"robot_head_rgb"` and `"robot_head_depth"`

    :property gripper_closed_state: All gripper joints must achieve this
        state for the gripper to be considered closed.
    :property gripper_open_state: All gripper joints must achieve this
        state for the gripper to be considered open.
    :property gripper_state_eps: Error margin for detecting whether gripper is closed.

    :property arm_mtr_pos_gain: The position gain of the arm motor.
    :property arm_mtr_vel_gain: The velocity gain of the arm motor.
    :property arm_mtr_max_impulse: The maximum impulse of the arm motor.

    :property wheel_mtr_pos_gain: The position gain of the wheeled motor (if
        there are wheels).
    :property wheel_mtr_vel_gain: The velocity gain of the wheel motor (if
        there are wheels).
    :property wheel_mtr_max_impulse: The maximum impulse of the wheel motor (if
        there are wheels).
    :property base_offset: The offset of the root transform from the center ground point for navmesh kinematic control.
    """

    arm_joints: List[int]
    gripper_joints: List[int]
    wheel_joints: Optional[List[int]]

    arm_init_params: Optional[np.ndarray]
    gripper_init_params: Optional[np.ndarray]

    ee_offset: mn.Vector3
    ee_link: int
    ee_constraint: np.ndarray

    cameras: Dict[str, RobotCameraParams]

    gripper_closed_state: np.ndarray
    gripper_open_state: np.ndarray
    gripper_state_eps: float

    arm_mtr_pos_gain: float
    arm_mtr_vel_gain: float
    arm_mtr_max_impulse: float

    wheel_mtr_pos_gain: float
    wheel_mtr_vel_gain: float
    wheel_mtr_max_impulse: float

    base_offset: mn.Vector3
    base_link_names: Set[str]


class MobileManipulator(RobotInterface):
    """Robot with a controllable base and arm."""

    def __init__(
        self,
        params: MobileManipulatorParams,
        urdf_path: str,
        sim: Simulator,
        limit_robo_joints: bool = True,
        fixed_base: bool = True,
    ):
        r"""Constructor

        :param limit_robo_joints: If true, joint limits of robot are always
            enforced.
        """
        super().__init__()
        self.urdf_path = urdf_path
        self.params = params
        self._fix_joint_values: Optional[np.ndarray] = None

        self._sim = sim
        self._limit_robo_joints = limit_robo_joints
        self._fixed_base = fixed_base
        self.sim_obj = None

        self._cameras = defaultdict(list)
        for camera_prefix in self.params.cameras:
            for sensor_name in self._sim._sensors:
                if sensor_name.startswith(camera_prefix):
                    self._cameras[camera_prefix].append(sensor_name)

        # NOTE: the follow members cache static info for improved efficiency over querying the API
        # maps joint ids to motor settings for convenience
        self.joint_motors: Dict[int, Tuple[int, JointMotorSettings]] = {}
        # maps joint ids to position index
        self.joint_pos_indices: Dict[int, int] = {}
        # maps joint ids to velocity index
        self.joint_dof_indices: Dict[int, int] = {}
        self.joint_limits: Tuple[np.ndarray, np.ndarray] = None

        # defaults for optional params
        if self.params.gripper_init_params is None:
            self.params.gripper_init_params = np.zeros(
                len(self.params.gripper_joints), dtype=np.float32
            )
        if self.params.arm_init_params is None:
            self.params.arm_init_params = np.zeros(
                len(self.params.arm_joints), dtype=np.float32
            )

    def reconfigure(self) -> None:
        """Instantiates the robot the scene. Loads the URDF, sets initial state of parameters, joints, motors, etc..."""
        ao_mgr = self._sim.get_articulated_object_manager()
        self.sim_obj = ao_mgr.add_articulated_object_from_urdf(
            self.urdf_path, fixed_base=self._fixed_base
        )
        if self._limit_robo_joints:
            # automatic joint limit clamping after each call to sim.step_physics()
            self.sim_obj.auto_clamp_joint_limits = True
        for link_id in self.sim_obj.get_link_ids():
            self.joint_pos_indices[link_id] = self.sim_obj.get_link_joint_pos_offset(
                link_id
            )
            self.joint_dof_indices[link_id] = self.sim_obj.get_link_dof_offset(link_id)
        self.joint_limits = self.sim_obj.joint_position_limits

        # remove any default damping motors
        for motor_id in self.sim_obj.existing_joint_motor_ids:
            self.sim_obj.remove_joint_motor(motor_id)
        # re-generate all joint motors with arm gains.

        jms = JointMotorSettings()
        self.sim_obj.create_all_motors(jms)
        self._update_motor_settings_cache()

        if self.params.arm_joints is not None:
            jms = JointMotorSettings(
                0,  # position_target
                self.params.arm_mtr_pos_gain,  # position_gain
                0,  # velocity_target
                self.params.arm_mtr_vel_gain,  # velocity_gain
                self.params.arm_mtr_max_impulse,  # max_impulse
            )
            for i in self.params.arm_joints:
                self.sim_obj.update_joint_motor(self.joint_motors[i][0], jms)
        self._update_motor_settings_cache()

        if self.params.gripper_joints is not None:
            jms = JointMotorSettings(
                0,  # position_target
                self.params.arm_mtr_pos_gain,  # position_gain
                0,  # velocity_target
                self.params.arm_mtr_vel_gain,  # velocity_gain
                self.params.arm_mtr_max_impulse,  # max_impulse
            )
            for i in self.params.gripper_joints:
                self.sim_obj.update_joint_motor(self.joint_motors[i][0], jms)
        self._update_motor_settings_cache()

        # set correct gains for wheels
        if self.params.wheel_joints is not None:
            jms = JointMotorSettings(
                0,  # position_target
                self.params.wheel_mtr_pos_gain,  # position_gain
                0,  # velocity_target
                self.params.wheel_mtr_vel_gain,  # velocity_gain
                self.params.wheel_mtr_max_impulse,  # max_impulse
            )
            # pylint: disable=not-an-iterable
            for i in self.params.wheel_joints:
                self.sim_obj.update_joint_motor(self.joint_motors[i][0], jms)

        # set initial states and targets
        self.arm_joint_pos = self.params.arm_init_params
        self.gripper_joint_pos = self.params.gripper_init_params

        self._update_motor_settings_cache()

    def update(self) -> None:
        """Updates the camera transformations and performs necessary checks on
        joint limits and sleep states.
        """
        agent_node = self._sim._default_agent.scene_node
        inv_T = agent_node.transformation.inverted()

        for cam_prefix, sensor_names in self._cameras.items():
            for sensor_name in sensor_names:
                sens_obj = self._sim._sensors[sensor_name]._sensor_object
                cam_info = self.params.cameras[cam_prefix]

                if cam_info.attached_link_id == -1:
                    link_trans = self.sim_obj.transformation
                else:
                    link_trans = self.sim_obj.get_link_scene_node(
                        self.params.ee_link
                    ).transformation

                cam_transform = mn.Matrix4.look_at(
                    cam_info.cam_offset_pos,
                    cam_info.cam_look_at_pos,
                    mn.Vector3(0, 1, 0),
                )
                cam_transform = link_trans @ cam_transform @ cam_info.relative_transform
                cam_transform = inv_T @ cam_transform

                sens_obj.node.transformation = orthonormalize_rotation_shear(
                    cam_transform
                )
        if self._fix_joint_values is not None:
            self.arm_joint_pos = self._fix_joint_values

        self.sim_obj.awake = True

    def reset(self) -> None:
        """Reset the joints on the existing robot.
        NOTE: only arm and gripper joint motors (not gains) are reset by default, derived class should handle any other changes."""

        # reset the initial joint positions
        self.sim_obj.clear_joint_states()

        self.arm_joint_pos = self.params.arm_init_params
        self._fix_joint_values = None
        self.gripper_joint_pos = self.params.gripper_init_params

        self._update_motor_settings_cache()
        self.update()

    #############################################
    # ARM RELATED
    #############################################
    @property
    def arm_joint_limits(self) -> Tuple[np.ndarray, np.ndarray]:
        """Get the arm joint limits in radians"""

        # deref self vars to cut access in half
        joint_pos_indices = self.joint_pos_indices
        lower_joints_limits, upper_joint_limits = self.joint_limits
        arm_joints = self.params.arm_joints
        arm_pos_indices = [joint_pos_indices[x] for x in arm_joints]
        lower_lims = np.array(
            [lower_joints_limits[i] for i in arm_pos_indices], dtype=np.float32
        )
        upper_lims = np.array(
            [upper_joint_limits[i] for i in arm_pos_indices], dtype=np.float32
        )
        return lower_lims, upper_lims

    @property
    def ee_link_id(self) -> int:
        """Gets the Habitat Sim link id of the end-effector."""
        return self.params.ee_link

    @property
    def ee_local_offset(self) -> mn.Vector3:
        """Gets the relative offset of the end-effector center from the
        end-effector link.
        """
        return self.params.ee_offset

    def calculate_ee_forward_kinematics(self, joint_state: np.ndarray) -> np.ndarray:
        """Gets the end-effector position for the given joint state."""
        self.sim_obj.joint_positions = joint_state
        return self.ee_transform.translation

    def calculate_ee_inverse_kinematics(
        self, ee_target_position: np.ndarray
    ) -> np.ndarray:
        """Gets the joint states necessary to achieve the desired end-effector
        configuration.
        """
        raise NotImplementedError("Currently no implementation for generic IK.")

    @property
    def ee_transform(self) -> mn.Matrix4:
        """Gets the transformation of the end-effector location. This is offset
        from the end-effector link location.
        """
        ef_link_transform = self.sim_obj.get_link_scene_node(
            self.params.ee_link
        ).transformation
        ef_link_transform.translation = ef_link_transform.transform_point(
            self.ee_local_offset
        )
        return ef_link_transform

    @property
    def gripper_joint_pos(self) -> np.ndarray:
        """Get the current gripper joint positions."""

        # deref self vars to cut access in half
        joint_pos_indices = self.joint_pos_indices
        gripper_joints = self.params.gripper_joints
        sim_obj_joint_pos = self.sim_obj.joint_positions

        gripper_pos_indices = (joint_pos_indices[x] for x in gripper_joints)
        return np.array(
            [sim_obj_joint_pos[i] for i in gripper_pos_indices],
            dtype=np.float32,
        )

    @gripper_joint_pos.setter
    def gripper_joint_pos(self, ctrl: List[float]):
        """Kinematically sets the gripper joints and sets the motors to target."""
        joint_positions = self.sim_obj.joint_positions
        for i, jidx in enumerate(self.params.gripper_joints):
            self._set_motor_pos(jidx, ctrl[i])
            joint_positions[self.joint_pos_indices[jidx]] = ctrl[i]
        self.sim_obj.joint_positions = joint_positions

    def set_gripper_target_state(self, gripper_state: float) -> None:
        """Set the gripper motors to a desired symmetric state of the gripper [0,1] -> [open, closed]"""
        for i, jidx in enumerate(self.params.gripper_joints):
            delta = (
                self.params.gripper_closed_state[i] - self.params.gripper_open_state[i]
            )
            target = self.params.gripper_open_state[i] + delta * gripper_state
            self._set_motor_pos(jidx, target)

    def close_gripper(self) -> None:
        """Set gripper to the close state"""
        self.set_gripper_target_state(1)

    def open_gripper(self) -> None:
        """Set gripper to the open state"""
        self.set_gripper_target_state(0)

    @property
    def is_gripper_open(self) -> bool:
        """True if all gripper joints are within eps of the open state."""
        return (
            np.amax(
                np.abs(
                    self.gripper_joint_pos - np.array(self.params.gripper_open_state)
                )
            )
            < self.params.gripper_state_eps
        )

    @property
    def is_gripper_closed(self) -> bool:
        """True if all gripper joints are within eps of the closed state."""
        return (
            np.amax(
                np.abs(
                    self.gripper_joint_pos - np.array(self.params.gripper_closed_state)
                )
            )
            < self.params.gripper_state_eps
        )

    @property
    def arm_joint_pos(self) -> np.ndarray:
        """Get the current arm joint positions."""

        # deref self vars to cut access in half
        joint_pos_indices = self.joint_pos_indices
        arm_joints = self.params.arm_joints
        sim_obj_joint_pos = self.sim_obj.joint_positions

        arm_pos_indices = (joint_pos_indices[x] for x in arm_joints)
        return np.array(
            [sim_obj_joint_pos[i] for i in arm_pos_indices], dtype=np.float32
        )

    @arm_joint_pos.setter
    def arm_joint_pos(self, ctrl: List[float]):
        """Kinematically sets the arm joints and sets the motors to target."""
        self._validate_arm_ctrl_input(ctrl)

        joint_positions = self.sim_obj.joint_positions

        for i, jidx in enumerate(self.params.arm_joints):
            self._set_motor_pos(jidx, ctrl[i])
            joint_positions[self.joint_pos_indices[jidx]] = ctrl[i]
        self.sim_obj.joint_positions = joint_positions

    def _validate_arm_ctrl_input(self, ctrl: List[float]):
        """
        Raises an exception if the control input is NaN or does not match the
        joint dimensions.
        """
        if len(ctrl) != len(self.params.arm_joints):
            raise ValueError("Control dimension does not match joint dimension")
        if np.any(np.isnan(ctrl)):
            raise ValueError("Control is NaN")

    def set_fixed_arm_joint_pos(self, fix_arm_joint_pos):
        """
        Will fix the arm to a desired position at every internal timestep. Can
        be used for kinematic arm control.
        """
        self._validate_arm_ctrl_input(fix_arm_joint_pos)
        self._fix_joint_values = fix_arm_joint_pos
        self.arm_joint_pos = fix_arm_joint_pos

    @property
    def arm_velocity(self) -> np.ndarray:
        """Get the velocity of the arm joints."""

        # deref self vars to cut access in half
        joint_dof_indices = self.joint_dof_indices
        arm_joints = self.params.arm_joints
        sim_obj_joint_vel = self.sim_obj.joint_velocities

        arm_dof_indices = (joint_dof_indices[x] for x in arm_joints)
        return np.array(
            [sim_obj_joint_vel[i] for i in arm_dof_indices],
            dtype=np.float32,
        )

    @property
    def arm_motor_pos(self) -> np.ndarray:
        """Get the current target of the arm joints motors."""
        motor_targets = np.zeros(len(self.params.arm_init_params))
        for i, jidx in enumerate(self.params.arm_joints):
            motor_targets[i] = self._get_motor_pos(jidx)
        return motor_targets

    @arm_motor_pos.setter
    def arm_motor_pos(self, ctrl: List[float]) -> None:
        """Set the desired target of the arm joint motors."""
        self._validate_arm_ctrl_input(ctrl)

        for i, jidx in enumerate(self.params.arm_joints):
            self._set_motor_pos(jidx, ctrl[i])

    def clip_ee_to_workspace(self, pos: np.ndarray) -> np.ndarray:
        """Clips a 3D end-effector position within region the robot can reach."""
        return np.clip(
            pos, self.params.ee_constraint[:, 0], self.params.ee_constraint[:, 1]
        )

    #############################################
    # WHEEL RELATED
    #############################################

    # TODO: add some functions for easy wheel control

    #############################################
    # BASE RELATED
    #############################################

    @property
    def base_pos(self):
        """Get the robot base ground position via configured local offset from origin."""
        return self.sim_obj.translation + self.sim_obj.transformation.transform_vector(
            self.params.base_offset
        )

    @base_pos.setter
    def base_pos(self, position: mn.Vector3):
        """Set the robot base to a desired ground position (e.g. NavMesh point) via configured local offset from origin."""
        if len(position) != 3:
            raise ValueError("Base position needs to be three dimensions")
        self.sim_obj.translation = (
            position
            - self.sim_obj.transformation.transform_vector(self.params.base_offset)
        )

    @property
    def base_rot(self) -> float:
        return self.sim_obj.rotation.angle()

    @base_rot.setter
    def base_rot(self, rotation_y_rad: float):
        self.sim_obj.rotation = mn.Quaternion.rotation(
            mn.Rad(rotation_y_rad), mn.Vector3(0, 1, 0)
        )

    @property
    def base_transformation(self):
        return self.sim_obj.transformation

    def is_base_link(self, link_id: int) -> bool:
        return self.sim_obj.get_link_name(link_id) in self.params.base_link_names

    #############################################
    # HIDDEN
    #############################################

    def _validate_joint_idx(self, joint):
        if joint not in self.joint_motors:
            raise ValueError(
                f"Requested joint {joint} not in joint motors with indices (keys {self.joint_motors.keys()}) and {self.joint_motors}"
            )

    def _set_motor_pos(self, joint, ctrl):
        self._validate_joint_idx(joint)
        self.joint_motors[joint][1].position_target = ctrl
        self.sim_obj.update_joint_motor(
            self.joint_motors[joint][0], self.joint_motors[joint][1]
        )

    def _get_motor_pos(self, joint):
        self._validate_joint_idx(joint)
        return self.joint_motors[joint][1].position_target

    def _set_joint_pos(self, joint_idx, angle):
        # NOTE: This is pretty inefficient and should not be used iteratively
        set_pos = self.sim_obj.joint_positions
        set_pos[self.joint_pos_indices[joint_idx]] = angle
        self.sim_obj.joint_positions = set_pos

    def _interpolate_arm_control(
        self, targs, idxs, seconds, ctrl_freq, get_observations=False
    ):
        curs = np.array([self._get_motor_pos(i) for i in idxs])
        diff = targs - curs
        T = int(seconds * ctrl_freq)
        delta = diff / T

        observations = []
        for i in range(T):
            joint_positions = self.sim_obj.joint_positions
            for j, jidx in enumerate(idxs):
                self._set_motor_pos(jidx, delta[j] * (i + 1) + curs[j])
                joint_positions[self.joint_pos_indices[jidx]] = (
                    delta[j] * (i + 1) + curs[j]
                )
            self.sim_obj.joint_positions = joint_positions
            self._sim.step_world(1 / ctrl_freq)
            if get_observations:
                observations.append(self._sim.get_sensor_observations())
        return observations

    def _update_motor_settings_cache(self):
        """Updates the JointMotorSettings cache for cheaper future updates"""
        self.joint_motors = {}
        for motor_id, joint_id in self.sim_obj.existing_joint_motor_ids.items():
            self.joint_motors[joint_id] = (
                motor_id,
                self.sim_obj.get_joint_motor_settings(motor_id),
            )
