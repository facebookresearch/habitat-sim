from abc import ABC, abstractmethod

from habitat_sim.physics import ManagedBulletArticulatedObject


class RobotInterface(ABC):
    """Generic robot interface defines standard API functions."""

    def __init__(self):
        """Initializes this wrapper, but does not instantiate the robot."""
        self._robot: ManagedBulletArticulatedObject = None

    def get_robot_sim_id(self) -> int:
        """Get the unique id for referencing the robot."""
        return self._robot.object_id

    @abstractmethod
    def update(self):
        """Updates any properties or internal systems for the robot such as camera transformations, joint limits, and sleep states."""

    @abstractmethod
    def reset(self):
        """Reset the joint and motor states of an existing robot."""

    @abstractmethod
    def reconfigure(self):
        """Instantiates the robot the scene. Loads the URDF, sets initial state of parameters, joints, motors, etc..."""
