import habitat_sim
import habitat_sim.bindings as hsim
import os.path as osp
import pytest
import numpy as np
import glob
import tqdm
import os

base_dir = osp.abspath(osp.join(osp.dirname(__file__), ".."))

test_navmeshes = [
    osp.join(base_dir, "tests/17DRP5sb8fy/17DRP5sb8fy.navmesh"),
    osp.join(
        base_dir, "data/scene_datasets/habitat-test-scenes/skokloster-castle.navmesh"
    ),
    osp.join(base_dir, "data/scene_datasets/habitat-test-scenes/van-gogh-room.navmesh"),
]

gibson_base = osp.join(base_dir, "data/scene_datasets/gibson_train_val")
if osp.exists(gibson_base):
    test_navmeshes += glob.glob(f"{gibson_base}/*.navmesh")

mp3d_base = osp.join(base_dir, "data/scene_datasets/mp3d")
if osp.exists(gibson_base):
    test_navmeshes += glob.glob(f"{mp3d_base}/*/*.navmesh")


@pytest.fixture(scope="session")
def scene_graph():
    return hsim.SceneGraph()


@pytest.fixture(scope="module")
def pbar1():
    return tqdm.tqdm(total=len(test_navmeshes))


@pytest.fixture(scope="module")
def pbar2():
    return tqdm.tqdm(total=len(test_navmeshes))


@pytest.mark.parametrize("test_navmesh", test_navmeshes)
def test_greedy_follower(test_navmesh, scene_graph, pbar1, pbar2):
    pbar1.update()
    if not osp.exists(test_navmesh):
        pytest.skip(f"{test_navmesh} not found")

    pathfinder = hsim.PathFinder()
    pathfinder.load_nav_mesh(test_navmesh)
    assert pathfinder.is_loaded

    agent = habitat_sim.Agent()
    agent.attach(scene_graph.get_root_node().create_child())
    agent.controls.move_filter_fn = pathfinder.try_step
    follower = habitat_sim.GreedyGeodesicFollower(pathfinder, agent)

    num_tests = 500
    max_failure_rate = 0.01
    num_fails = 0

    for _ in tqdm.trange(500, leave=False):
        state = agent.state
        while True:
            state.position = pathfinder.get_random_navigable_point()
            goal_pos = pathfinder.get_random_navigable_point()
            path = hsim.ShortestPath()
            path.requested_start = state.position
            path.requested_end = goal_pos

            if pathfinder.find_path(path) and path.geodesic_distance > 2.0:
                break

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

        agent.state = state
        for _ in range(int(1e4)):
            action = follower.next_action_along(goal_pos)
            if action is None:
                break
            agent.act(action)

        state = agent.state
        assert (
            np.linalg.norm(state.position - goal_pos) <= follower.forward_spec.amount
        ), "Didn't make it"

    pbar2.update()
