import magnum as mn
import numpy as np
import pybullet as p

from habitat_sim.robots.mobile_manipulator import (
    MobileManipulator,
    MobileManipulatorParams,
)


class IkHelper:
    def __init__(self, urdf_path: str):
        self._arm_len = 7
        self.urdf_path = urdf_path

    def setup_sim(self):
        self.pc_id = p.connect(p.DIRECT)

        self.robo_id = p.loadURDF(
            self.urdf_path,
            basePosition=[0, 0, 0],
            useFixedBase=True,
            flags=p.URDF_USE_INERTIA_FROM_FILE,
            physicsClientId=self.pc_id,
        )

        p.setGravity(0, 0, -9.81, physicsClientId=self.pc_id)
        JOINT_DAMPING = 0.5
        self.pb_link_idx = 7

        for link_idx in range(15):
            p.changeDynamics(
                self.robo_id,
                link_idx,
                linearDamping=0.0,
                angularDamping=0.0,
                jointDamping=JOINT_DAMPING,
                physicsClientId=self.pc_id,
            )
            p.changeDynamics(
                self.robo_id, link_idx, maxJointVelocity=200, physicsClientId=self.pc_id
            )

    def set_arm_state(self, joint_pos, joint_vel=None):
        if joint_vel is None:
            joint_vel = np.zeros((len(joint_pos),))
        for i in range(7):
            p.resetJointState(
                self.robo_id, i, joint_pos[i], joint_vel[i], physicsClientId=self.pc_id
            )

    def calc_ik(self, targ_ee):
        """
        targ_ee is in ROBOT COORDINATE FRAME NOT IN EE COORDINATE FRAME
        """
        js = p.calculateInverseKinematics(
            self.robo_id, self.pb_link_idx, targ_ee, physicsClientId=self.pc_id
        )
        return js[: self._arm_len]


class FetchRobot(MobileManipulator):
    def __init__(self, urdf_path, sim, limit_robo_joints=True):
        fetch_params = MobileManipulatorParams(
            arm_joints=list(range(4, 11)),
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

        self._ik = IkHelper(self.urdf_path)
        self._ik.setup_sim()

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
    def calculate_ee_fk(self, js):
        self._ik.set_arm_state(js, np.zeros(js.shape))
        ls = p.getLinkState(
            self._ik.robo_id,
            self._ik.pb_link_idx,
            computeForwardKinematics=1,
            physicsClientId=self._ik.pc_id,
        )
        world_ee = ls[4]
        return np.array(world_ee)

    def calculate_ee_ik(self, ee_targ):
        joint_pos = self.get_arm_pos()
        joint_vel = self.get_arm_vel()

        self._ik.set_arm_state(joint_pos, joint_vel)

        des_joint_pos = self._ik.calc_ik(ee_targ)
        des_joint_pos = list(des_joint_pos)
        return np.array(des_joint_pos)

    def get_arm_joint_lims(self):
        lower = []
        upper = []
        for joint_i in range(self._ik._arm_len):
            ret = p.getJointInfo(
                self._ik.robo_id, joint_i, physicsClientId=self._ik.pc_id
            )
            lower.append(ret[8])
            if ret[9] == -1:
                upper.append(2 * np.pi)
            else:
                upper.append(ret[9])
        return np.array(lower), np.array(upper)

    def retract_arm(self):
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
        self._interpolate_arm_control([-0.45, 0.1], self.params.arm_joints, -1)
