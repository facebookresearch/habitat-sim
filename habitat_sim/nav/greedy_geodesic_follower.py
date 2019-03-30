import habitat_sim.agent
from habitat_sim import utils
from habitat_sim import errors
import habitat_sim.bindings as hsim
import numpy as np
import attr
from typing import List, Dict, Any, Optional


@attr.s
class GreedyGeodesicFollower(object):
    pathfinder: hsim.PathFinder = attr.ib()
    agent: habitat_sim.agent.Agent = attr.ib()
    goal_radius: Optional[float] = attr.ib(default=None)
    action_mapping: Dict[int, Any] = attr.ib(init=False, factory=dict)
    impl: hsim.GreedyGeodesicFollowerImpl = attr.ib(init=False, default=None)
    forward_spec: habitat_sim.agent.ActuationSpec = attr.ib(init=False, default=None)
    left_spec: habitat_sim.agent.ActuationSpec = attr.ib(init=False, default=None)
    right_spec: habitat_sim.agent.ActuationSpec = attr.ib(init=False, default=None)

    def __attrs_post_init__(self):
        self.action_mapping[-1] = None

        key, spec = self._find_action("move_forward")
        self.forward_spec = spec
        self.action_mapping[0] = key

        key, spec = self._find_action("turn_left")
        self.left_spec = spec
        self.action_mapping[1] = key

        key, spec = self._find_action("turn_right")
        self.right_spec = spec
        self.action_mapping[2] = key

        if self.goal_radius is None:
            self.goal_radius = 0.75 * self.forward_spec.amount

        self.impl = hsim.GreedyGeodesicFollowerImpl(
            self.pathfinder,
            self._move_forward,
            self._turn_left,
            self._turn_right,
            self.goal_radius,
            self.forward_spec.amount,
            np.deg2rad(self.left_spec.amount),
        )

    def _find_action(self, name):
        candidates = list(
            filter(
                lambda v: v[1].name == name,
                self.agent.agent_config.action_space.items(),
            )
        )

        assert (
            len(candidates) == 1
        ), f"Could not find an action spec corresponding to {name}"

        return candidates[0][0], candidates[0][1].actuation

    def _move_forward(self, obj: hsim.SceneNode):
        self.agent.controls(obj, "move_forward", self.forward_spec, True)

    def _turn_left(self, obj: hsim.SceneNode):
        self.agent.controls(obj, "turn_left", self.left_spec, True)

    def _turn_right(self, obj: hsim.SceneNode):
        self.agent.controls(obj, "turn_right", self.right_spec, True)

    def next_action_along(self, goal_pos: np.array) -> Any:
        state = self.agent.state
        next_act = self.impl.next_action_along(
            state.position, utils.quat_to_coeffs(state.rotation), goal_pos
        )

        if next_act == -2:
            raise errors.GreedyFollowerError()
        else:
            return self.action_mapping[next_act]

    def find_path(self, goal_pos: np.array) -> List[Any]:
        state = self.agent.state
        path = self.impl.find_path(
            state.position, utils.quat_to_coeffs(state.rotation), goal_pos
        )

        if len(path) == 0:
            raise errors.GreedyFollowerError()

        path = list(map(lambda v: self.action_mapping[v], path))

        return path
