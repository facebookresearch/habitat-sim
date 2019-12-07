import glob
import os
import os.path as osp

import numpy as np
import pytest
import tqdm

import habitat_sim

base_dir = osp.abspath(osp.join(osp.dirname(__file__), ".."))

test_navmeshes = [
    osp.join(base_dir, "data/scene_datasets/mp3d/17DRP5sb8fy/17DRP5sb8fy.navmesh"),
    osp.join(
        base_dir, "data/scene_datasets/habitat-test-scenes/skokloster-castle.navmesh"
    ),
    osp.join(base_dir, "data/scene_datasets/habitat-test-scenes/van-gogh-room.navmesh"),
]


test_all = False
gibson_base = osp.join(base_dir, "data/scene_datasets/gibson_train_val")
if test_all and osp.exists(gibson_base):
    test_navmeshes += glob.glob(f"{gibson_base}/*.navmesh")

mp3d_base = osp.join(base_dir, "data/scene_datasets/mp3d")
if test_all and osp.exists(gibson_base):
    test_navmeshes += glob.glob(f"{mp3d_base}/*/*.navmesh")


@pytest.fixture(scope="session")
def scene_graph():
    return habitat_sim.SceneGraph()


@pytest.fixture(scope="module")
def pbar():
    if test_all:
        return tqdm.tqdm(total=len(test_navmeshes))
    else:
        return None


num_fails = 0


@pytest.mark.parametrize("test_navmesh", test_navmeshes)
def test_greedy_follower(test_navmesh, scene_graph, pbar):
    global num_fails
    if not osp.exists(test_navmesh):
        pytest.skip(f"{test_navmesh} not found")

    pathfinder = habitat_sim.PathFinder()
    pathfinder.load_nav_mesh(test_navmesh)
    assert pathfinder.is_loaded

    agent = habitat_sim.Agent(scene_graph.get_root_node().create_child())
    agent.controls.move_filter_fn = pathfinder.try_step
    follower = habitat_sim.GreedyGeodesicFollower(pathfinder, agent)

    num_tests = 50

    for _ in range(num_tests):
        state = agent.state
        while True:
            state.position = pathfinder.get_random_navigable_point()
            goal_pos = pathfinder.get_random_navigable_point()
            path = habitat_sim.ShortestPath()
            path.requested_start = state.position
            path.requested_end = goal_pos

            if pathfinder.find_path(path) and path.geodesic_distance > 2.0:
                break

        try:
            agent.state = state
            path = follower.find_path(goal_pos)
            for i, action in enumerate(path):
                if action is not None:
                    agent.act(action)
                else:
                    assert i == len(path) - 1

            end_state = agent.state
            assert (
                np.linalg.norm(end_state.position - goal_pos)
                <= follower.forward_spec.amount
            ), "Didn't make it"
        except Exception as e:
            if test_all:
                num_fails += 1
                pbar.set_postfix(num_fails=num_fails)
            else:
                raise e

        try:
            agent.state = state
            for _ in range(int(1e4)):
                action = follower.next_action_along(goal_pos)
                if action is None:
                    break
                agent.act(action)

            state = agent.state
            assert (
                np.linalg.norm(state.position - goal_pos)
                <= follower.forward_spec.amount
            ), "Didn't make it"
        except Exception as e:
            if test_all:
                num_fails += 1
                pbar.set_postfix(num_fails=num_fails)
            else:
                raise e

    if test_all:
        pbar.update()
