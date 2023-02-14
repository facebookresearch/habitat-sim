# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

from typing import Any, Dict, List, Optional, Tuple

import attr
import numpy as np

from habitat_sim import errors, scene
from habitat_sim.agent.agent import Agent
from habitat_sim.agent.controls.controls import ActuationSpec
from habitat_sim.nav import GreedyFollowerCodes, GreedyGeodesicFollowerImpl, PathFinder
from habitat_sim.utils.common import quat_to_magnum


@attr.s(auto_attribs=True, init=False)
class GreedyGeodesicFollower:
    r"""Planner that greedily fits actions to follow the geodesic shortest path.

    The planner plans on perfect actions (assumes actuation noise is unbiased) and thus
    requires the ``move_forward``, ``turn_left``, and ``turn_right`` actions to be
    present in the agents action space.  If you would like to use different actions
    (i.e noisy actions), you can override the action key ommited for a given action.

    Planner code heavily inspired by
    https://github.com/s-gupta/map-plan-baseline

    """

    pathfinder: PathFinder
    agent: Agent
    goal_radius: Optional[float]
    action_mapping: Dict[GreedyFollowerCodes, Any]
    impl: GreedyGeodesicFollowerImpl
    forward_spec: ActuationSpec
    left_spec: ActuationSpec
    right_spec: ActuationSpec
    last_goal: Optional[np.ndarray]

    def __init__(
        self,
        pathfinder: PathFinder,
        agent: Agent,
        goal_radius: Optional[float] = None,
        *,
        stop_key: Optional[Any] = None,
        forward_key: Optional[Any] = None,
        left_key: Optional[Any] = None,
        right_key: Optional[Any] = None,
        fix_thrashing: bool = True,
        thrashing_threshold: int = 16,
    ) -> None:
        r"""Constructor

        :param pathfinder: Instance of the pathfinder that has the correct
            navmesh already loaded
        :param agent: Agent to fit actions for. This agent's current
            configuration is used to specify the actions. The fitted actions will
            also correspond to keys in the agents action_space. :py:`None` is used
            to signify that the goal location has been reached
        :param goal_radius: Specifies how close the agent must get to the goal
            in order for it to be considered reached.  If :py:`None`, :py:`0.75`
            times the agents step size is used.
        :param stop_key: The action key to emit when the agent should stop.
                            Default :py:`None`
        :param forward_key: The action key to emit when the agent should
                               take the move_forward action
                               Default: The key of the action that calls
                               the move_forward actuation spec
        :param left_key: The action key to emit when the agent should
                               take the turn_left action
                               Default: The key of the action that calls
                               the turn_left actuation spec
        :param right_key: The action key to emit when the agent should
                               take the turn_right action
                               Default: The key of the action that calls
                               the turn_right actuation spec
        :param fix_thrashing: Whether or not to attempt to fix thrashing
        :param thrashing_threshold: The number of actions in a left -> right -> left -> ..
                                       sequence needed to be considered thrashing
        """

        self.pathfinder = pathfinder
        self.agent = agent
        self.last_goal = None

        self.action_mapping = {}
        self.action_mapping[GreedyFollowerCodes.STOP] = stop_key

        key, spec = self._find_action("move_forward")
        self.forward_spec = spec
        self.action_mapping[GreedyFollowerCodes.FORWARD] = (
            key if forward_key is None else forward_key
        )

        key, spec = self._find_action("turn_left")
        self.left_spec = spec
        self.action_mapping[GreedyFollowerCodes.LEFT] = (
            key if left_key is None else left_key
        )

        key, spec = self._find_action("turn_right")
        self.right_spec = spec
        self.action_mapping[GreedyFollowerCodes.RIGHT] = (
            key if right_key is None else right_key
        )

        self.goal_radius = (
            0.75 * self.forward_spec.amount if goal_radius is None else goal_radius
        )

        self.impl = GreedyGeodesicFollowerImpl(
            self.pathfinder,
            self._move_forward,
            self._turn_left,
            self._turn_right,
            self.goal_radius,
            self.forward_spec.amount,
            np.deg2rad(self.left_spec.amount),
            fix_thrashing,
            thrashing_threshold,
        )

    def _find_action(self, name: str) -> Tuple[str, ActuationSpec]:
        candidates = list(
            filter(
                lambda kv: kv[1].name == name,
                self.agent.agent_config.action_space.items(),
            )
        )

        assert (
            len(candidates) == 1
        ), f"Could not find an action spec corresponding to {name}"

        return candidates[0][0], candidates[0][1].actuation

    def _move_forward(self, obj: scene.SceneNode) -> bool:
        return self.agent.controls(obj, "move_forward", self.forward_spec, True)

    def _turn_left(self, obj: scene.SceneNode) -> bool:
        return self.agent.controls(obj, "turn_left", self.left_spec, True)

    def _turn_right(self, obj: scene.SceneNode) -> bool:
        return self.agent.controls(obj, "turn_right", self.right_spec, True)

    def next_action_along(self, goal_pos: np.ndarray) -> Any:
        r"""Find the next action to greedily follow the geodesic shortest path
        from the agent's current position to get to the goal

        :param goal_pos: The position of the goal
        :return: The action to take
        """
        if self.last_goal is None or not np.allclose(goal_pos, self.last_goal):
            self.reset()
            self.last_goal = goal_pos

        state = self.agent.state
        next_act = self.impl.next_action_along(
            quat_to_magnum(state.rotation), state.position, goal_pos
        )

        if next_act == GreedyFollowerCodes.ERROR:
            raise errors.GreedyFollowerError()
        return self.action_mapping[next_act]

    def find_path(self, goal_pos: np.ndarray) -> List[Any]:
        r"""Finds the sequence actions that greedily follow the geodesic
        shortest path from the agent's current position to get to the goal

        :param goal_pos: The position of the goal
        :return: The list of actions to take. Ends with :py:`None`.

        This is roughly equivilent to just calling `next_action_along()` until
        it returns :py:`None`, but is faster.

        .. note-warning::

            Do not use this method if the agent has actuation noise.
            Instead, use :ref:`next_action_along` to find the action
            to take in a given state, then take that action, and repeat!
        """
        self.reset()

        state = self.agent.state
        path = self.impl.find_path(
            quat_to_magnum(state.rotation), state.position, goal_pos
        )

        if len(path) == 0:
            raise errors.GreedyFollowerError()

        path = [self.action_mapping[v] for v in path]

        return path

    def reset(self) -> None:
        self.impl.reset()
        self.last_goal = None
