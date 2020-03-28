from typing import Any, Dict, List, Optional

import attr
import numpy as np

from habitat_sim import agent, errors, scene
from habitat_sim.nav import GreedyFollowerCodes, GreedyGeodesicFollowerImpl, PathFinder
from habitat_sim.utils.common import quat_to_coeffs


@attr.s(auto_attribs=True)
class GreedyGeodesicFollower(object):
    r"""Greedily fits actions to follow the geodesic shortest path

    :property pathfinder: Instance of the pathfinder that has the correct
        navmesh already loaded
    :property agent: Agent to fit actions for. This agent's current
        configuration is used to specify the actions. The fitted actions will
        also correspond to keys in the agents action_space. :py:`None` is used
        to signify that the goal location has been reached
    :property goal_radius: Specifies how close the agent must get to the goal
        in order for it to be considered reached.  If :py:`None`, :py:`0.75`
        times the agents step size is used.
    """

    pathfinder: PathFinder
    agent: agent.Agent
    goal_radius: Optional[float] = attr.ib(default=None)
    action_mapping: Dict[GreedyFollowerCodes, Any] = attr.ib(
        init=False, factory=dict, repr=False
    )
    impl: GreedyGeodesicFollowerImpl = attr.ib(init=False, default=None, repr=False)
    forward_spec: agent.ActuationSpec = attr.ib(init=False, default=None, repr=False)
    left_spec: agent.ActuationSpec = attr.ib(init=False, default=None, repr=False)
    right_spec: agent.ActuationSpec = attr.ib(init=False, default=None, repr=False)

    def __attrs_post_init__(self):
        self.action_mapping[GreedyFollowerCodes.STOP] = None

        key, spec = self._find_action("move_forward")
        self.forward_spec = spec
        self.action_mapping[GreedyFollowerCodes.FORWARD] = key

        key, spec = self._find_action("turn_left")
        self.left_spec = spec
        self.action_mapping[GreedyFollowerCodes.LEFT] = key

        key, spec = self._find_action("turn_right")
        self.right_spec = spec
        self.action_mapping[GreedyFollowerCodes.RIGHT] = key

        if self.goal_radius is None:
            self.goal_radius = 0.75 * self.forward_spec.amount

        self.impl = GreedyGeodesicFollowerImpl(
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

    def _move_forward(self, obj: scene.SceneNode):
        self.agent.controls(obj, "move_forward", self.forward_spec, True)

    def _turn_left(self, obj: scene.SceneNode):
        self.agent.controls(obj, "turn_left", self.left_spec, True)

    def _turn_right(self, obj: scene.SceneNode):
        self.agent.controls(obj, "turn_right", self.right_spec, True)

    def next_action_along(self, goal_pos: np.ndarray) -> Any:
        r"""Find the next action to greedily follow the geodesic shortest path
        from the agent's current position to get to the goal

        :param goal_pos: The position of the goal
        :return: The action to take
        """
        state = self.agent.state
        next_act = self.impl.next_action_along(
            state.position, quat_to_coeffs(state.rotation), goal_pos
        )

        if next_act == GreedyFollowerCodes.ERROR:
            raise errors.GreedyFollowerError()
        else:
            return self.action_mapping[next_act]

    def find_path(self, goal_pos: np.ndarray) -> List[Any]:
        r"""Finds the sequence actions that greedily follow the geodesic
        shortest path from the agent's current position to get to the goal

        :param goal_pos: The position of the goal
        :return: The list of actions to take. Ends with :py:`None`.

        This is roughly equivilent to just calling `next_action_along()` until
        it returns :py:`None`, but is faster.
        """
        state = self.agent.state
        path = self.impl.find_path(
            state.position, quat_to_coeffs(state.rotation), goal_pos
        )

        if len(path) == 0:
            raise errors.GreedyFollowerError()

        path = list(map(lambda v: self.action_mapping[v], path))

        return path
