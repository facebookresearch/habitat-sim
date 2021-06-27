from typing import Dict, List, Optional, Tuple

import attr
import magnum as mn
import numpy as np

from habitat_sim.physics import JointMotorSettings
from habitat_sim.robots.robot_interface import RobotInterface
from habitat_sim.simulator import Simulator


@attr.s(auto_attribs=True, slots=True)
class MobileManipulatorParams:
    """Data to configure a mobile manipulator.
    :property arm_joints: The joint ids of the arm joints.
    :property wheel_joints: The joint ids of the wheels. If the wheels are not controlled, then this should be None

    :property arm_init_params: The starting joint angles of the arm. If None,
        nothing is set.

    :property ee_offset: The 3D offset from the end-effector link to the true
        end-effector position.
    :property ee_link: The Habitat Sim link ID of the end-effector.
    :property ee_constraint: A (2, N) shaped array specifying the upper and
        lower limits for each end-effector joint where N is the arm DOF.

    :property arm_cam_offset_pos: The 3D offset of the arm camera from the
        end-effector position.
    :property head_cam_offset_pos: The 3D offset of the head camera from the
        base of the robot.
    :property head_cam_look_pos: The 3D offset of where the head should face,
        relative to the head camera.

    :property gripper_joints: The habitat sim joint ids of any grippers.
    :property gripper_closed_state: All gripper joints must achieve this
        value for the gripper to be considered closed.
    :property gripper_open_state: All gripper joints must achieve this
        value for the gripper to be considered open.
    :property gripper_closed_eps: Error margin for detecting whether gripper is closed.

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
    :property ctrl_freq: The number of control actions per second.
    """

    arm_joints: List[int]
    wheel_joints: Optional[List[int]]

    arm_init_params: Optional[List[float]]

    ee_offset: mn.Vector3
    ee_link: int
    ee_constraint: np.ndarray

    arm_cam_offset_pos: mn.Vector3
    head_cam_offset_pos: mn.Vector3
    head_cam_look_pos: mn.Vector3

    gripper_joints: List[int]
    gripper_closed_state: float
    gripper_open_state: float
    gripper_closed_eps: float

    arm_mtr_pos_gain: float
    arm_mtr_vel_gain: float
    arm_mtr_max_impulse: float

    wheel_mtr_pos_gain: float
    wheel_mtr_vel_gain: float
    wheel_mtr_max_impulse: float

    base_offset: mn.Vector3

    ctrl_freq: int


class MobileManipulator(RobotInterface):
    """Robot with a controllable base and arm."""

    def __init__(
        self,
        params: MobileManipulatorParams,
        urdf_path: str,
        sim: Simulator,
        limit_robo_joints: bool = True,
    ):
        """Constructor
        :param limit_robo_joints: If true, joint limits of robot are always
            enforced.
        """
        super().__init__()
        self.urdf_path = urdf_path
        self.params = params

        self._gripper_state = 0.0
        self._sim = sim
        self._limit_robo_joints = limit_robo_joints

        self._arm_sensor_names = [
            s for s in self._sim._sensors if s.startswith("robot_arm_")
        ]
        self._head_sensor_names = [
            s for s in self._sim._sensors if s.startswith("robot_head_")
        ]

        # NOTE: the follow members cache static info for improved efficiency over querying the API
        # maps joint ids to motor settings for convenience
        self.joint_motors: Dict[int, Tuple[int, JointMotorSettings]] = {}
        # maps joint ids to position index
        self.joint_pos_indices: Dict[int, int] = {}
        # maps joint ids to velocity index
        self.joint_dof_indices: Dict[int, int] = {}
        self.joint_limits: Tuple[np.ndarray, np.ndarray] = None

    def reconfigure(self) -> None:
        """Instantiates the robot the scene. Loads the URDF, sets initial state of parameters, joints, motors, etc..."""
        ao_mgr = self._sim.get_articulated_object_manager()
        self._robot = ao_mgr.add_articulated_object_from_urdf(self.urdf_path)
        for link_id in self._robot.get_link_ids():
            self.joint_pos_indices[link_id] = self._robot.get_link_joint_pos_offset(
                link_id
            )
            self.joint_dof_indices[link_id] = self._robot.get_link_dof_offset(link_id)
        self.joint_limits = self._robot.joint_position_limits

    def update(self) -> None:
        """Updates the camera transformations and performs necessary checks on
        joint limits and sleep states.
        """
        agent_node = self._sim._default_agent.scene_node
        inv_T = agent_node.transformation.inverted()

        # Update the cameras
        for sensor_name in self._head_sensor_names:
            sens_obj = self._sim._sensors[sensor_name]._sensor_object
            head_T = self._get_head_cam_transform()
            sens_obj.node.transformation = inv_T @ head_T

        for sensor_name in self._arm_sensor_names:
            sens_obj = self._sim._sensors[sensor_name]._sensor_object
            arm_T = self._get_arm_cam_transform()
            sens_obj.node.transformation = inv_T @ arm_T

        # Guard against out of limit joints
        # TODO: should auto clamping be enabled instead? How often should we clamp?
        if self._limit_robo_joints:
            self._robot.clamp_joint_limits()

        self._robot.awake = True

    def reset(self) -> None:
        """Reset the joints on the existing robot.
        NOTE: only arm and wheel joint motors are reset by default, derived class should handle any other motors."""

        # Init the fetch starting joint positions.
        if self.params.arm_init_params is not None:
            self.arm_pos = self.params.arm_init_params

        # TODO: should all arm joints have the same gain/impulse settings?
        jms = JointMotorSettings(
            0,  # position_target
            self.params.arm_mtr_pos_gain,  # position_gain
            0,  # velocity_target
            self.params.arm_mtr_vel_gain,  # velocity_gain
            self.params.arm_mtr_max_impulse,  # max_impulse
        )
        # set initial pose for arm motors
        for ix, joint_id in enumerate(self.params.arm_joints):
            jms.position_target = self.params.arm_init_params[ix]
            self._robot.update_joint_motor(self.joint_motors[joint_id][0], jms)

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
                self._robot.update_joint_motor(self.joint_motors[i][0], jms)

        # update motor settings dict
        self.joint_motors = {}
        for motor_id, joint_id in self._robot.existing_joint_motor_ids.items():
            self.joint_motors[joint_id] = (
                motor_id,
                self._robot.get_joint_motor_settings(motor_id),
            )

    #############################################
    # ARM RELATED
    #############################################
    @property
    def arm_joint_limits(self) -> Tuple[np.ndarray, np.ndarray]:
        """Get the arm joint limits in radians"""
        arm_pos_indices = list(
            map(lambda x: self.joint_pos_indices[x], self.params.arm_joints)
        )

        lower_lims = [self.joint_limits[0][i] for i in arm_pos_indices]
        upper_lims = [self.joint_limits[1][i] for i in arm_pos_indices]
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
        self._robot.joint_positions = joint_state
        return self.get_ee_transform().translation

    def calculate_ee_inverse_kinematics(
        self, ee_target_position: np.ndarray
    ) -> np.ndarray:
        """Gets the joint states necessary to achieve the desired end-effector
        configuration.
        """
        raise NotImplementedError("Currently no implementation for generic IK.")

    def get_ee_transform(self) -> mn.Matrix4:
        """Gets the transformation of the end-effector location. This is offset
        from the end-effector link location.
        """
        ef_link_transform = self._robot.get_link_scene_node(
            self.params.ee_link
        ).transformation
        ef_link_transform.translation = ef_link_transform.transform_point(
            self.ee_local_offset
        )
        return ef_link_transform

    @property
    def gripper_state(self) -> float:
        return self._gripper_state

    @gripper_state.setter
    def gripper_state(self, gripper_state: float) -> None:
        """Set the desired state of the gripper"""
        # TODO: should this apply the change to motors or joints?
        self._gripper_state = gripper_state

    def close_gripper(self) -> None:
        """Set gripper to the close state"""
        self.gripper_state = self.params.gripper_closed_state

    def open_gripper(self) -> None:
        """Set gripper to the open state"""
        self.gripper_state = self.params.gripper_open_state

    @property
    def is_gripper_open(self) -> bool:
        # Give some threshold for the open state.
        grip_dist = np.abs(self._gripper_state - self.params.gripper_open_state)
        return grip_dist < self.params.gripper_closed_eps

    @property
    def arm_joint_pos(self) -> np.ndarray:
        """Get the current arm joint positions in radians."""
        arm_pos_indices = list(
            map(lambda x: self.joint_pos_indices[x], self.params.arm_joints)
        )
        return [self._robot.joint_positions[i] for i in arm_pos_indices]

    @arm_joint_pos.setter
    def arm_joint_pos(self, ctrl: List[float]):
        """Kinematically sets the arm joints and sets the motors to target."""
        joint_positions = self._robot.joint_positions
        for i, jidx in enumerate(self.params.arm_joints):
            self._set_mtr_pos(jidx, ctrl[i])
            joint_positions[self.joint_pos_indices[jidx]] = ctrl[i]
        self._robot.joint_positions = joint_positions

    def get_arm_velocity(self) -> np.ndarray:
        """Get the velocity of the arm joints."""
        arm_dof_indices = list(
            map(lambda x: self.joint_dof_indices[x], self.params.arm_joints)
        )
        return [self._robot.joint_velocities[i] for i in arm_dof_indices]

    def set_arm_motor_pos(self, ctrl: List[float]) -> None:
        """Set the desired target of the arm joints for the controller to
        achieve.
        """
        for i, jidx in enumerate(self.params.arm_joints):
            self._set_mtr_pos(jidx, ctrl[i])

    def retract_arm(self) -> None:
        """Moves the arm to a pre-specified retracted state out of the way of
        the head camera.
        """
        raise NotImplementedError("Robot does not implement retract arm")

    def ready_arm(self) -> None:
        """Moves the arm back to the resting position."""
        raise NotImplementedError("Robot does not implement ready arm")

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

    def set_base_pos(self, position):
        """Set the robot base to a desired ground position (e.g. NavMesh point)."""
        self._robot.translation = position + self.params.base_offset

    #############################################
    # HIDDEN
    #############################################
    def _get_arm_cam_transform(self):
        """Helper function to get the transformation of where the arm camera
        should be placed.
        """
        ee_trans = self._robot.get_link_scene_node(self.params.ee_link).transformation
        offset_trans = mn.Matrix4.translation(self.params.arm_cam_offset_pos)
        rot_trans = mn.Matrix4.rotation_y(mn.Deg(-90))
        spin_trans = mn.Matrix4.rotation_z(mn.Deg(90))
        arm_T = ee_trans @ offset_trans @ rot_trans @ spin_trans
        return arm_T

    def _get_head_cam_transform(self):
        """Helper function to get the transformation of where the head camera
        should be placed.
        """
        look_at = self._robot.transformation.transform_point(
            self.params.head_cam_look_pos
        )
        cam_pos = self._robot.transformation.transform_point(
            self.params.head_cam_offset_pos
        )
        return mn.Matrix4.look_at(cam_pos, look_at, mn.Vector3(0, -1, 0))

    def _set_mtr_pos(self, joint, ctrl):
        self.joint_motors[joint][1].position_target = ctrl
        self._robot.update_joint_motor(
            self.joint_motors[joint][0], self.joint_motors[joint][1]
        )

    def _get_mtr_pos(self, joint):
        return self.joint_motors[joint][1].position_target

    def _set_joint_pos(self, joint_idx, angle):
        # NOTE: This is pretty inefficient and should not be used iteratively
        set_pos = self._robot.joint_positions
        set_pos[self.joint_pos_indices[joint_idx]] = angle
        self._robot.joint_positions = set_pos

    def _interpolate_arm_control(self, targs, idxs, seconds, get_observations=False):
        curs = np.array([self._get_mtr_pos(i) for i in idxs])
        diff = targs - curs
        T = int(seconds * self.params.ctrl_freq)
        delta = diff / T

        observations = []
        for i in range(T):
            joint_positions = self._robot.joint_positions
            for j, jidx in enumerate(idxs):
                self._set_mtr_pos(jidx, delta[j] * (i + 1) + curs[j])
                joint_positions[self.joint_pos_indices[jidx]] = (
                    delta[j] * (i + 1) + curs[j]
                )
            self._robot.joint_positions = joint_positions
            self._sim.step_world(1 / self.params.ctrl_freq)
            if get_observations:
                observations.append(self._sim.get_sensor_observations())
        return observations
