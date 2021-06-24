from typing import List, Optional, Tuple

import attr
import magnum as mn
import numpy as np

# from habitat.core.simulator import Simulator
from robot_interface import RobotInterface

import habitat_sim


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
        sim: habitat_sim.Simulator,
        limit_robo_joints: bool = True,
    ):
        """Constructor
        :param limit_robo_joints: If true, joint limits of robot are always
            enforced.
        """
        self.urdf_path = urdf_path
        self.params = params

        self._gripper_state = [0.0]
        self._robot_id = None
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
        return self._robot_id

    def reset(self) -> None:
        """Adds the articulated robot object to the scene."""
        self._robot_id = self._sim.add_articulated_object_from_urdf(
            self.urdf_path, True
        )
        if self._robot_id == -1:
            raise ValueError("Could not load " + self.urdf_path)

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
        if self._limit_robo_joints:
            upper_lims = self._sim.get_articulated_object_position_limits(
                self._robot_id, True
            )
            lower_lims = self._sim.get_articulated_object_position_limits(
                self._robot_id, False
            )
            robot_joint_pos = self._sim.get_articulated_object_positions(self._robot_id)
            new_robot_joint_pos = np.clip(robot_joint_pos, lower_lims, upper_lims)
            if (new_robot_joint_pos != robot_joint_pos).any():
                self._sim.set_articulated_object_positions(
                    self._robot_id, new_robot_joint_pos
                )

        self._sim.set_articulated_object_sleep(self._robot_id, False)

    def reconfigure(self) -> None:
        """Reset the joints on the existing robot."""
        jms = habitat_sim.physics.JointMotorSettings(
            0,  # position_target
            self.params.arm_mtr_pos_gain,  # position_gain
            0,  # velocity_target
            self.params.arm_mtr_vel_gain,  # velocity_gain
            self.params.arm_mtr_max_impulse,  # max_impulse
        )
        for i in self.params.arm_joints:
            self._sim.update_joint_motor(self._robot_id, i, jms)

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
                self._sim.update_joint_motor(self._robot_id, i, jms)

    #############################################
    # ARM RELATED
    #############################################
    def get_arm_joint_lims(self) -> Tuple[np.ndarray, np.ndarray]:
        """Get the arm joint limits in radians"""
        upper_lims = self._sim.get_articulated_object_position_limits(
            self._robot_id, True
        )
        lower_lims = self._sim.get_articulated_object_position_limits(
            self._robot_id, False
        )
        lower_lims, upper_lims = np.array(lower_lims), np.array(upper_lims)
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
        link_rigid_state = self._sim.get_articulated_link_rigid_state(
            self._robot_id, self.params.ee_link
        )
        # Move the end effector up a bit so it is in the middle of the gripper
        mat = mn.Matrix4.from_(
            link_rigid_state.rotation.to_matrix(), link_rigid_state.translation
        )
        mat = mat @ mn.Matrix4.translation(self.get_ee_local_offset())
        return mat

    def set_gripper_state(self, gripper_state: List[float]) -> None:
        """Set the desired state of the gripper"""
        self._gripper_state = gripper_state

    def close_gripper(self) -> None:
        """Set gripper to the close state"""
        self.set_gripper_state([self.params.gripper_closed_state])

    def open_gripper(self) -> None:
        """Set gripper to the open state"""
        self.set_gripper_state([self.params.gripper_open_state])

    def is_gripper_open(self) -> bool:
        # Give some threshold for the open state, but give some threshold.
        grip_dist = np.abs(self._gripper_state[0] - self.params.gripper_open_state)
        return grip_dist < 0.001

    def set_arm_pos(self, ctrl: List[float]):
        """Kinematically sets the arm joints and sets the motors to target."""
        for i, jidx in enumerate(self.params.arm_joints):
            self._set_mtr_pos(jidx, ctrl[i])
            self._set_joint_pos(jidx, ctrl[i])

    def get_arm_pos(self) -> np.ndarray:
        """Get the current arm joint positions in radians."""
        js = self._sim.get_articulated_object_positions(self._robot_id)
        return np.array(js)[self.params.arm_joints]

    def get_arm_vel(self) -> np.ndarray:
        """Get the velocity of the arm joints."""
        js_vel = self._sim.get_articulated_object_velocities(self._robot_id)
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
    # BASE RELATED
    #############################################
    def set_base_transform(self, T: mn.Matrix4):
        """Sets the transform of the robot base."""
        self._sim.set_articulated_object_root_state(self._robot_id, T)

    def set_base_position_2d(self, new_pos: np.ndarray):
        """Sets the robot base to a 2D location.
        :param new_pos: 2D coordinates of where the robot will be placed. The height
          will be same as current position.
        """
        base_transform = self._sim.get_articulated_object_root_state(self._robot_id)
        pos = base_transform.translation
        base_transform.translation = mn.Vector3(new_pos[0], pos[1], new_pos[1])
        self._sim.set_articulated_object_root_state(self._robot_id, base_transform)

    def set_base_position_3d(self, new_pos: np.ndarray):
        """Sets the robot base to a 3D location
        :param new_pos: 3D coordinates of where the robot will be placed.
        """
        base_transform = self._sim.get_articulated_object_root_state(self._robot_id)
        pos = base_transform.translation
        base_transform.translation = mn.Vector3(new_pos[0], pos[1], new_pos[1])
        self._sim.set_articulated_object_root_state(self._robot_id, base_transform)

    def set_base_rotation(self, rot_rad: float):
        """Set the rotation of the robot along the y-axis. The position will
        remain the same.
        """
        cur_trans = self._sim.get_articulated_object_root_state(self._robot_id)
        pos = cur_trans.translation

        add_rot_mat = mn.Matrix4.rotation(mn.Rad(rot_rad), mn.Vector3(0.0, 0, 1))
        new_trans = self._model_rot @ add_rot_mat
        new_trans.translation = pos
        self._sim.set_articulated_object_root_state(self._robot_id, new_trans)

    def get_base_transform(self) -> mn.Matrix4:
        """Get the base transformation"""
        return self._sim.get_articulated_object_root_state(self._robot_id)

    #############################################
    # HIDDEN
    #############################################
    def _get_arm_cam_transform(self):
        """Helper function to get the transformation of where the arm camera
        should be placed.
        """
        link_rigid_state = self._sim.get_articulated_link_rigid_state(
            self._robot_id, self.params.ee_link
        )
        ee_trans = mn.Matrix4.from_(
            link_rigid_state.rotation.to_matrix(), link_rigid_state.translation
        )

        offset_trans = mn.Matrix4.translation(self.params.arm_cam_offset_pos)
        rot_trans = mn.Matrix4.rotation_y(mn.Deg(-90))
        spin_trans = mn.Matrix4.rotation_z(mn.Deg(90))
        arm_T = ee_trans @ offset_trans @ rot_trans @ spin_trans
        return arm_T

    def _get_head_cam_transform(self):
        """Helper function to get the transformation of where the head camera
        should be placed.
        """
        robot_state = self._sim.get_articulated_object_root_state(self._robot_id)

        look_at = robot_state.transform_point(self.params.head_cam_look_pos)
        cam_pos = robot_state.transform_point(self.params.head_cam_offset_pos)

        return mn.Matrix4.look_at(cam_pos, look_at, mn.Vector3(0, -1, 0))

    def _set_mtr_pos(self, joint, ctrl):
        jms = self._sim.get_joint_motor_settings(self._robot_id, joint)
        jms.position_target = ctrl
        self._sim.update_joint_motor(self._robot_id, joint, jms)

    def _get_mtr_pos(self, joint):
        jms = self._sim.get_joint_motor_settings(self._robot_id, joint)
        return jms.position_target

    def _set_joint_pos(self, joint_idx, angle):
        set_pos = np.array(self._sim.get_articulated_object_positions(self._robot_id))
        set_pos[joint_idx] = angle
        self._sim.set_articulated_object_positions(self._robot_id, set_pos)

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
