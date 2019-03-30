import attr
import numpy as np
from .controls import ObjectControls, ActuationSpec
from habitat_sim import utils
from habitat_sim.sensors import SensorSuite
import habitat_sim.bindings as hsim
import habitat_sim.errors


from typing import Dict, Any, List

__all__ = ["ActionSpec", "SixDOFPose", "AgentState", "AgentConfig", "Agent"]


BodyActions = {
    "move_right",
    "move_left",
    "move_forward",
    "move_backward",
    "turn_left",
    "turn_right",
}


def _default_action_space():
    return dict(
        move_forward=ActionSpec("move_forward", ActuationSpec(amount=0.25)),
        turn_left=ActionSpec("turn_left", ActuationSpec(amount=10.0)),
        turn_right=ActionSpec("turn_right", ActuationSpec(amount=10.0)),
    )


@attr.s(auto_attribs=True, slots=True)
class ActionSpec(object):
    name: str
    actuation: ActuationSpec


@attr.s(auto_attribs=True, slots=True)
class SixDOFPose(object):
    position: np.array = np.zeros(3)
    rotation: np.quaternion = np.quaternion(1, 0, 0, 0)


@attr.s(auto_attribs=True, slots=True)
class AgentState(object):
    position: np.array = np.zeros(3)
    rotation: np.quaternion = np.quaternion(1, 0, 0, 0)
    velocity: np.array = np.zeros(3)
    angular_velocity: np.array = np.zeros(3)
    force: np.array = np.zeros(3)
    torque: np.array = np.zeros(3)
    sensor_states: Dict[str, SixDOFPose] = attr.Factory(dict)


@attr.s(auto_attribs=True, slots=True)
class AgentConfig(object):
    height: float = 1.5
    radius: float = 0.1
    mass: float = 32.0
    linear_acceleration: float = 20.0
    angular_acceleration: float = 4 * np.pi
    linear_friction: float = 0.5
    angular_friction: float = 1.0
    coefficient_of_restitution: float = 0.0
    sensor_specifications: List[hsim.SensorSpec] = attr.Factory(
        lambda: [hsim.SensorSpec()]
    )
    action_space: Dict[Any, ActionSpec] = attr.Factory(_default_action_space)
    body_type: str = "cylinder"


@attr.s(auto_attribs=True)
class Agent(object):
    agent_config: AgentConfig = attr.Factory(AgentConfig)
    sensors: SensorSuite = attr.Factory(SensorSuite)
    controls: ObjectControls = attr.Factory(ObjectControls)
    body: hsim.AttachedObject = attr.Factory(hsim.AttachedObject)

    def __attrs_post_init__(self):
        self.body.object_type = hsim.AttachedObjectType.AGENT
        self.reconfigure(self.agent_config)

    def reconfigure(self, agent_config: AgentConfig):
        self.sensors.clear()

        self.agent_config = agent_config
        for spec in self.agent_config.sensor_specifications:
            self.sensors.add(hsim.PinholeCamera(spec))

    def attach(self, scene_node: hsim.SceneNode):
        self.body.attach(scene_node)
        for _, v in self.sensors.items():
            if not v.is_valid:
                v.attach(self.scene_node.create_child())

    def dettach(self):
        self.body.dettach()
        for _, v in self.sensors.items():
            v.dettach()

    def act(self, action_name):
        habitat_sim.errors.assert_obj_valid(self.body)
        assert (
            action_name in self.agent_config.action_space
        ), f"No action {action_name} in action space"
        action = self.agent_config.action_space[action_name]

        if action.name in BodyActions:
            self.controls.action(
                self.scene_node, action.name, action.actuation, apply_filter=True
            )
        else:
            for _, v in self.sensors.items():
                habitat_sim.errors.assert_obj_valid(v)
                self.controls.action(
                    v.get_scene_node(),
                    action.name,
                    action.actuation,
                    apply_filter=False,
                )

    def get_state(self):
        habitat_sim.errors.assert_obj_valid(self.body)
        state = AgentState(
            self.body.get_absolute_position(),
            utils.quat_from_coeffs(self.body.get_rotation()),
        )

        for k, v in self.sensors.items():
            habitat_sim.errors.assert_obj_valid(v)
            state.sensor_states[k] = SixDOFPose(
                v.get_absolute_position(),
                state.rotation * utils.quat_from_coeffs(v.get_rotation()),
            )

        return state

    def set_state(self, state: AgentState, reset_sensors: bool = True):
        habitat_sim.errors.assert_obj_valid(self.body)

        self.body.reset_transformation()

        self.body.translate(state.position)
        self.body.set_rotation(utils.quat_to_coeffs(state.rotation))

        if reset_sensors:
            for _, v in self.sensors.items():
                v.set_transformation_from_spec()

        for k, v in state.sensor_states.items():
            assert k in self.sensors
            s = self.sensors[k]

            s.reset_transformation()
            s.translate(
                utils.quat_rotate_vector(
                    state.rotation.inverse(), v.position - state.position
                )
            )
            s.set_rotation(utils.quat_to_coeffs(state.rotation.inverse() * v.rotation))

    @property
    def scene_node(self):
        habitat_sim.errors.assert_obj_valid(self.body)
        return self.body.get_scene_node()

    @property
    def state(self):
        return self.get_state()

    @state.setter
    def state(self, new_state):
        self.set_state(new_state, reset_sensors=True)
