# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import glob
from os import path as osp

import numpy as np
import pytest
import tqdm

import habitat_sim

NUM_TESTS = 100
TURN_DEGREE = 30.0
ACCEPTABLE_SPLS = {
    ("try_step", False): 0.97,
    ("try_step_no_sliding", False): 0.925,
    ("try_step", True): 0.82,
    ("try_step_no_sliding", True): 0.60,
}


base_dir = osp.abspath(osp.join(osp.dirname(__file__), ".."))

test_navmeshes = [
    osp.join(base_dir, "data/scene_datasets/mp3d/17DRP5sb8fy/17DRP5sb8fy.navmesh"),
    osp.join(
        base_dir, "data/scene_datasets/habitat-test-scenes/skokloster-castle.navmesh"
    ),
    osp.join(base_dir, "data/scene_datasets/habitat-test-scenes/van-gogh-room.navmesh"),
]


test_all = False
gibson_base = osp.join(base_dir, "data/scene_datasets/gibson")
if test_all and osp.exists(gibson_base):
    test_navmeshes += glob.glob(f"{gibson_base}/*.navmesh")

mp3d_base = osp.join(base_dir, "data/scene_datasets/mp3d")
if test_all and osp.exists(mp3d_base):
    test_navmeshes += glob.glob(f"{mp3d_base}/*/*.navmesh")

mp3d_example_base = osp.join(base_dir, "data/scene_datasets/mp3d_example")
if test_all and osp.exists(mp3d_example_base):
    test_navmeshes += glob.glob(f"{mp3d_example_base}/*/*.navmesh")


@pytest.fixture(scope="module")
def pbar():
    if test_all:
        return tqdm.tqdm(total=len(test_navmeshes) * NUM_TESTS)
    else:
        return None


num_fails = 0.0
num_tested = 0
total_spl = 0.0


@pytest.mark.parametrize("test_navmesh", test_navmeshes)
@pytest.mark.parametrize("move_filter_fn", ["try_step", "try_step_no_sliding"])
@pytest.mark.parametrize("action_noise", [False, True])
def test_greedy_follower(test_navmesh, move_filter_fn, action_noise, pbar):
    global num_fails
    global num_tested
    global total_spl

    if not osp.exists(test_navmesh):
        pytest.skip(f"{test_navmesh} not found")

    pathfinder = habitat_sim.PathFinder()
    pathfinder.load_nav_mesh(test_navmesh)
    assert pathfinder.is_loaded
    pathfinder.seed(0)
    np.random.seed(seed=0)

    scene_graph = habitat_sim.SceneGraph()
    agent = habitat_sim.Agent(scene_graph.get_root_node().create_child())
    agent.controls.move_filter_fn = getattr(pathfinder, move_filter_fn)

    agent.agent_config.action_space["turn_left"].actuation.amount = TURN_DEGREE
    agent.agent_config.action_space["turn_right"].actuation.amount = TURN_DEGREE

    if action_noise:
        # "_" prefix the perfect actions so that we can use noisy actions instead
        agent.agent_config.action_space = {
            "_" + k: v for k, v in agent.agent_config.action_space.items()
        }

        agent.agent_config.action_space.update(
            **dict(
                move_forward=habitat_sim.ActionSpec(
                    "pyrobot_noisy_move_forward",
                    habitat_sim.PyRobotNoisyActuationSpec(amount=0.25),
                ),
                turn_left=habitat_sim.ActionSpec(
                    "pyrobot_noisy_turn_left",
                    habitat_sim.PyRobotNoisyActuationSpec(amount=TURN_DEGREE),
                ),
                turn_right=habitat_sim.ActionSpec(
                    "pyrobot_noisy_turn_right",
                    habitat_sim.PyRobotNoisyActuationSpec(amount=TURN_DEGREE),
                ),
            )
        )

    follower = habitat_sim.GreedyGeodesicFollower(
        pathfinder,
        agent,
        forward_key="move_forward",
        left_key="turn_left",
        right_key="turn_right",
    )

    test_spl = 0.0
    for _ in range(NUM_TESTS):
        follower.reset()

        state = habitat_sim.AgentState()
        while True:
            state.position = pathfinder.get_random_navigable_point()
            goal_pos = pathfinder.get_random_navigable_point()
            path = habitat_sim.ShortestPath()
            path.requested_start = state.position
            path.requested_end = goal_pos

            if pathfinder.find_path(path) and path.geodesic_distance > 2.0:
                break

        agent.state = state
        failed = False
        gt_geo = path.geodesic_distance
        agent_distance = 0.0
        last_xyz = state.position
        num_acts = 0

        # If there is not action noise, then we can use find_path to get all the actions
        if not action_noise:
            try:
                action_list = follower.find_path(goal_pos)
            except habitat_sim.errors.GreedyFollowerError:
                action_list = [None]

        while True:
            # If there is action noise, we need to plan a single action, actually take it, and repeat
            if action_noise:
                try:
                    next_action = follower.next_action_along(goal_pos)
                except habitat_sim.errors.GreedyFollowerError:
                    break
            else:
                next_action = action_list[0]
                action_list = action_list[1:]

            if next_action is None:
                break

            agent.act(next_action)

            agent_distance += float(np.linalg.norm(last_xyz - agent.state.position))
            last_xyz = agent.state.position

            num_acts += 1
            if num_acts > 1e4:
                break

        end_state = agent.state

        path.requested_start = end_state.position
        pathfinder.find_path(path)

        failed = path.geodesic_distance > follower.forward_spec.amount
        spl = float(not failed) * gt_geo / max(gt_geo, agent_distance)
        test_spl += spl

        if test_all:
            num_fails += float(failed)
            num_tested += 1
            total_spl += spl
            pbar.set_postfix(
                num_fails=num_fails,
                failure_rate=num_fails / num_tested,
                spl=total_spl / num_tested,
            )
            pbar.update()

    if not test_all:
        assert test_spl / NUM_TESTS >= ACCEPTABLE_SPLS[(move_filter_fn, action_noise)]
