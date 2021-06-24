from abc import ABC, abstractmethod


class RobotInterface(ABC):
    def __init__(self):
        self.robot = None

    @abstractmethod
    def get_robot_sim_id(self) -> int:
        pass

    @abstractmethod
    def update(self):
        pass

    @abstractmethod
    def reset(self):
        pass

    @abstractmethod
    def reconfigure(self):
        pass
