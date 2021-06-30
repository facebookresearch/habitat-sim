from abc import ABC, abstractmethod

from habitat_sim.physics import ManagedBulletArticulatedObject


class RobotInterface(ABC):
    """Generic robot interface defines standard API functions."""

    def __init__(self):
        """Initializes this wrapper, but does not instantiate the robot."""
        # the Habitat ArticulatedObject API access wrapper
        self.sim_obj: ManagedBulletArticulatedObject = None

    def get_robot_sim_id(self) -> int:
        """Get the unique id for referencing the robot."""
        return self.sim_obj.object_id

    @abstractmethod
    def update(self):
        """Updates any properties or internal systems for the robot such as camera transformations, joint limits, and sleep states."""

    @abstractmethod
    def reset(self):
        """Reset the joint and motor states of an existing robot."""

    @abstractmethod
    def reconfigure(self):
        """Instantiates the robot the scene. Loads the URDF, sets initial state of parameters, joints, motors, etc..."""

    def get_link_and_joint_names(self) -> str:
        """Get a string listing all robot link and joint names for debugging purposes."""
        link_joint_names = ""
        # print relevant joint/link info for debugging
        for link_id in self.sim_obj.get_link_ids():
            link_joint_names += f"{link_id} = {self.sim_obj.get_link_name(link_id)} | {self.sim_obj.get_link_joint_name(link_id)} :: type = {self.sim_obj.get_link_joint_type(link_id)} \n"
        return link_joint_names
