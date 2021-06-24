import magnum as mn
import numpy as np

from habitat_sim.robots.mobile_manipulator import (
    MobileManipulator,
    MobileManipulatorParams,
)


class FetchRobot(MobileManipulator):
    def __init__(self, urdf_path, sim, limit_robo_joints=True):
        fetch_params = MobileManipulatorParams(
            # TODO: these joint indices are wrong for new Fetch
            arm_joints=list(range(4, 11)),
            # TODO: set wheel joints
            wheel_joints=None,
            arm_init_params=[
                -0.45,
                -1.08,
                0.1,
                0.935,
                -0.001,
                1.573,
                0.005,
                0.00,
                0.00,
            ],
            ee_offset=mn.Vector3(0.08, 0, 0),
            ee_link=20,
            ee_constraint=np.array([[0.4, 1.2], [-0.7, 0.7], [0.25, 1.5]]),
            arm_cam_offset_pos=mn.Vector3(0, 0.0, 0.1),
            head_cam_offset_pos=mn.Vector3(0.17, 0.0, 1.2),
            head_cam_look_pos=mn.Vector3(1, 0.0, 0.75),
            gripper_joints=[11, 12],
            gripper_closed_state=0.04,
            gripper_open_state=0.0,
            arm_mtr_pos_gain=0.3,
            arm_mtr_vel_gain=0.3,
            arm_mtr_max_impulse=10.0,
            wheel_mtr_pos_gain=0.0,
            wheel_mtr_vel_gain=1.3,
            wheel_mtr_max_impulse=10.0,
            ctrl_freq=30,
        )
        super().__init__(fetch_params, urdf_path, sim, limit_robo_joints)
        self.back_joint_id = 0
        self.head_rot_jid = 3
        self.head_tilt_jid = 2

    def update(self):
        super().update()
        # Fix the head.
        self._set_joint_pos(self.head_rot_jid, np.pi / 2)
        self._set_joint_pos(self.head_tilt_jid, 0)
        # Fix the back
        fix_back_val = 0.15
        self._set_joint_pos(self.back_joint_id, fix_back_val)
        self._set_mtr_pos(self.back_joint_id, fix_back_val)
        for grip_idx in self.params.gripper_joints:
            self._set_mtr_pos(grip_idx, self._gripper_state)
            self._set_joint_pos(grip_idx, self._gripper_state)

    #############################################
    # ARM RELATED
    #############################################

    def retract_arm(self):
        # TODO: not full arm state here
        self._interpolate_arm_control(
            [1.2299035787582397, 2.345386505126953],
            self.params.arm_joints,
            -1,
        )
        # self._interpolate_arm_control(
        #     [1.2299035787582397, 2.345386505126953],
        #     [self.arm_start + 1, self.arm_start + 3],
        #     -1,
        # )

    def ready_arm(self):
        # TODO: not full arm state here
        self._interpolate_arm_control([-0.45, 0.1], self.params.arm_joints, -1)
