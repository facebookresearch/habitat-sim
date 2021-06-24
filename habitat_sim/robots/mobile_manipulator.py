from typing import List, Optional, Tuple

import attr
import magnum as mn
import numpy as np

import habitat_sim

# from habitat.core.simulator import Simulator
from habitat_sim.robots.robot_interface import RobotInterface


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
    :property ee_constraint: A (2,N) shaped array specifying the upper and
        lower limits for each joint where N is the arm DOF.

    :property arm_cam_offset_pos: The 3D offset of the arm camera from the
        end-effector position.
    :property head_cam_offset_pos: The 3D offset of the head camera from the
        base of the robot.
    :property head_cam_look_pos: The 3D offset of where the head should face,
        relative to the head camera.

    :property gripper_joints: The habitat sim joint ids of any grippers.
    :property gripper_closed_state: All gripper joints need to achieve this
        value for the gripper to be considered closed.
    :property gripper_open_state: See gripper_closed_state.

    :property arm_mtr_pos_gain: The position gain of the arm motor.
    :property arm_mtr_vel_gain: The velocity gain of the arm motor.
    :property arm_mtr_max_impulse: The maximum impulse of the arm motor.

    :property wheel_mtr_pos_gain: The position gain of the wheeled motor (if
        there are wheels).
    :property wheel_mtr_vel_gain: The velocity gain of the wheel motor (if
        there are wheels).
    :property wheel_mtr_max_impulse: The maximum impulse of the wheel motor (if
        there are wheels).
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

    arm_mtr_pos_gain: float
    arm_mtr_vel_gain: float
    arm_mtr_max_impulse: float

    wheel_mtr_pos_gain: float
    wheel_mtr_vel_gain: float
    wheel_mtr_max_impulse: float

    ctrl_freq: int


class MobileManipulator(RobotInterface):
    """Robot with a controllable base and arm."""

    def __init__(
        self,
        params: MobileManipulatorParams,
        urdf_path: str,
        sim: habitat_sim.simulator.Simulator,
        limit_robo_joints: bool = True,
    ):
        """Constructor
        :param limit_robo_joints: If true, joint limits of robot are always
            enforced.
        """
        self.urdf_path = urdf_path
        self.params = params

        self._gripper_state = 0.0
        self._robot: habitat_sim.physics.ManagedBulletArticulatedObject = None
        self._sim = sim
        self._model_rot = mn.Matrix4.rotation(mn.Rad(-1.56), mn.Vector3(1.0, 0, 0))
        self._limit_robo_joints = limit_robo_joints

        self._arm_sensor_names = [
            s for s in self._sim._sensors if s.startswith("robot_arm_")
        ]
        self._head_sensor_names = [
            s for s in self._sim._sensors if s.startswith("robot_head_")
        ]

    def get_robot_sim_id(self) -> int:
        """Gets the underlying simulator ID of the robot."""
        return self._robot.object_id

    def reconfigure(self) -> None:
        """Adds the articulated robot object to the scene."""
        ao_mgr = self._sim.get_articulated_object_manager()
        self._robot = ao_mgr.add_articulated_object_from_urdf(self.urdf_path)

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
        """Reset the joints on the existing robot."""
        jms = habitat_sim.physics.JointMotorSettings(
            0,  # position_target
            self.params.arm_mtr_pos_gain,  # position_gain
            0,  # velocity_target
            self.params.arm_mtr_vel_gain,  # velocity_gain
            self.params.arm_mtr_max_impulse,  # max_impulse
        )
        for i in self.params.arm_joints:
            self._robot.update_joint_motor(i, jms)

        # Init the fetch starting joint positions.
        if self.params.arm_init_params is not None:
            self.set_arm_pos(self.params.arm_init_params)

        if self.params.wheel_joints is not None:
            jms = habitat_sim.physics.JointMotorSettings(
                0,  # position_target
                self.params.wheel_mtr_pos_gain,  # position_gain
                0,  # velocity_target
                self.params.wheel_mtr_vel_gain,  # velocity_gain
                self.params.wheel_mtr_max_impulse,  # max_impulse
            )
            # pylint: disable=not-an-iterable
            for i in self.params.wheel_joints:
                self._robot.update_joint_motor(i, jms)

    #############################################
    # ARM RELATED
    #############################################
    def get_arm_joint_lims(self) -> Tuple[np.ndarray, np.ndarray]:
        """Get the arm joint limits in radians"""
        lower_lims, upper_lims = self._robot.joint_position_limits
        lower_lims = lower_lims[self.params.arm_joints]
        upper_lims = upper_lims[self.params.arm_joints]
        return lower_lims, upper_lims

    def get_ee_link_id(self) -> int:
        """Gets the Habitat Sim link ID of the end-effector."""
        return self.params.ee_link

    def get_ee_local_offset(self) -> mn.Vector3:
        """Gets the relative offset of the end-effector center from the
        end-effector link.
        """
        return self.params.ee_offset

    def calculate_ee_fk(self, js: np.ndarray) -> np.ndarray:
        """Gets the end-effector position for the given joint state."""
        raise NotImplementedError("Currently no implementation for generic FK.")

    def calculate_ee_ik(self, ee_targ: np.ndarray) -> np.ndarray:
        """Gets the joint states necessary to achieve the desired end-effector
        configuration.
        """
        raise NotImplementedError("Currently no implementation for generic KK.")

    def get_end_effector_transform(self) -> mn.Matrix4:
        """Gets the transformation of the end-effector location. This is offset
        from the end-effector link location.
        """
        return (
            self._robot.get_link_scene_node(self.params.ee_link)
            .transformation()
            .transform_point(self.get_ee_local_offset())
        )

    def set_gripper_state(self, gripper_state: float) -> None:
        """Set the desired state of the gripper"""
        self._gripper_state = gripper_state

    def close_gripper(self) -> None:
        """Set gripper to the close state"""
        self.set_gripper_state(self.params.gripper_closed_state)

    def open_gripper(self) -> None:
        """Set gripper to the open state"""
        self.set_gripper_state(self.params.gripper_open_state)

    def is_gripper_open(self) -> bool:
        # Give some threshold for the open state.
        grip_dist = np.abs(self._gripper_state - self.params.gripper_open_state)
        return grip_dist < 0.001

    def set_arm_pos(self, ctrl: List[float]):
        """Kinematically sets the arm joints and sets the motors to target."""
        # TODO: pretty inefficent way to do this, better to modify the full pose iteratively and set once
        for i, jidx in enumerate(self.params.arm_joints):
            self._set_mtr_pos(jidx, ctrl[i])
            self._set_joint_pos(jidx, ctrl[i])

    def get_arm_pos(self) -> np.ndarray:
        """Get the current arm joint positions in radians."""
        js = self._robot.joint_positions
        return np.array(js)[self.params.arm_joints]

    def get_arm_vel(self) -> np.ndarray:
        """Get the velocity of the arm joints."""
        js_vel = self._robot.joint_velocities
        return np.array(js_vel)[self.params.arm_joints]

    def set_arm_mtr_pos(self, ctrl: List[float]) -> None:
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
    # HIDDEN
    #############################################
    def _get_arm_cam_transform(self):
        """Helper function to get the transformation of where the arm camera
        should be placed.
        """
        ee_trans = self._robot.get_link_scene_node(self.params.ee_link).transformation()
        offset_trans = mn.Matrix4.translation(self.params.arm_cam_offset_pos)
        rot_trans = mn.Matrix4.rotation_y(mn.Deg(-90))
        spin_trans = mn.Matrix4.rotation_z(mn.Deg(90))
        arm_T = ee_trans @ offset_trans @ rot_trans @ spin_trans
        return arm_T

    def _get_head_cam_transform(self):
        """Helper function to get the transformation of where the head camera
        should be placed.
        """
        look_at = self._robot.transformation().transform_point(
            self.params.head_cam_look_pos
        )
        cam_pos = self._robot.transformation().transform_point(
            self.params.head_cam_offset_pos
        )
        return mn.Matrix4.look_at(cam_pos, look_at, mn.Vector3(0, -1, 0))

    def _set_mtr_pos(self, joint, ctrl):
        jms = self._robot.get_joint_motor_settings(joint)
        jms.position_target = ctrl
        self._robot.update_joint_motor(joint, jms)

    def _get_mtr_pos(self, joint):
        jms = self._robot.get_joint_motor_settings(joint)
        return jms.position_target

    def _set_joint_pos(self, joint_idx, angle):
        # TODO: This is pretty inefficient and should not be used iteratively
        set_pos = self._robot.joint_positions
        set_pos[joint_idx] = angle
        self._robot.joint_positions = set_pos

    def _interpolate_arm_control(self, targs, idxs, seconds):
        curs = np.array([self._get_mtr_pos(i) for i in idxs])
        diff = targs - curs
        T = int(seconds * self.params.ctrl_freq)
        delta = diff / T

        for i in range(T):
            for j, jidx in enumerate(idxs):
                self._set_mtr_pos(jidx, delta[j] * (i + 1) + curs[j])
                self._set_joint_pos(jidx, delta[j] * (i + 1) + curs[j])
            self._sim.step_world(1 / self.params.ctrl_freq)
