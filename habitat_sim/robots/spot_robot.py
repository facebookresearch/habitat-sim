import magnum as mn
import numpy as np

from habitat_sim.robots.mobile_manipulator import (
    MobileManipulator,
    MobileManipulatorParams,
    RobotCameraParams,
)


class SpotRobot(MobileManipulator):
    def _get_spot_params(self):
        return MobileManipulatorParams(
            arm_joints=list(range(0, 7)),
            gripper_joints=[7],
            leg_joints=list(range(8, 20)),
            # NOTE: default to retracted arm. Zero vector is full extension.
            arm_init_params=[0.0, -3.14, 0.0, 3.0, 0.0, 0.0, 0.0],
            # NOTE: default closed
            gripper_init_params=[0.00],
            # NOTE: default to rough standing pose with balance
            leg_init_params=[
                0.0,
                0.7,
                -1.5,
                0.0,
                0.7,
                -1.5,
                0.0,
                0.7,
                -1.5,
                0.0,
                0.7,
                -1.5,
            ],
            ee_offset=mn.Vector3(0.08, 0, 0),
            ee_link=6,
            # TODO: figure this one out if necessary
            ee_constraint=np.array([[0.4, 1.2], [-0.7, 0.7], [0.25, 1.5]]),
            # TODO: these need to be adjusted. Copied from Fetch currently.
            cameras={
                "robot_arm": RobotCameraParams(
                    cam_offset_pos=mn.Vector3(0, 0.0, 0.1),
                    cam_look_at_pos=mn.Vector3(0.1, 0.0, 0.0),
                    attached_link_id=6,
                    relative_transform=mn.Matrix4.rotation_y(mn.Deg(-90))
                    @ mn.Matrix4.rotation_z(mn.Deg(90)),
                ),
                "robot_head": RobotCameraParams(
                    cam_offset_pos=mn.Vector3(0.17, 1.2, 0.0),
                    cam_look_at_pos=mn.Vector3(0.75, 1.0, 0.0),
                    attached_link_id=-1,
                ),
                "robot_third": RobotCameraParams(
                    cam_offset_pos=mn.Vector3(-0.5, 1.7, -0.5),
                    cam_look_at_pos=mn.Vector3(1, 0.0, 0.75),
                    attached_link_id=-1,
                ),
            },
            gripper_closed_state=[0.0],
            gripper_open_state=[-1.56],
            gripper_state_eps=0.01,
            arm_mtr_pos_gain=0.3,
            arm_mtr_vel_gain=0.3,
            arm_mtr_max_impulse=10.0,
            # TODO: leg motor defaults for dynamic stability
            leg_mtr_pos_gain=2.0,
            leg_mtr_vel_gain=1.3,
            leg_mtr_max_impulse=100.0,
            # NOTE: empirically set from default NavMesh error and initial leg pose.
            base_offset=mn.Vector3(0, -0.35, 0),
            base_link_names={
                "base",
            },
        )

    def __init__(self, urdf_path, sim, limit_robo_joints=True, fixed_base=True):
        super().__init__(
            self._get_spot_params(), urdf_path, sim, limit_robo_joints, fixed_base
        )

    # TODO: ?
    # @property
    # def base_transformation(self):
    #    add_rot = mn.Matrix4.rotation(mn.Rad(-np.pi / 2), mn.Vector3(1.0, 0, 0))
    #    return self.sim_obj.transformation @ add_rot
