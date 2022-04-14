import math
from os import path as osp

import numpy as np
import pytest

import examples.settings
import habitat_sim

EPS = 1e-5

base_dir = osp.abspath(osp.join(osp.dirname(__file__), ".."))

test_scenes = [
    osp.join(base_dir, "data/scene_datasets/mp3d_example/17DRP5sb8fy/17DRP5sb8fy.glb"),
    osp.join(base_dir, "data/scene_datasets/habitat-test-scenes/skokloster-castle.glb"),
    osp.join(base_dir, "data/scene_datasets/habitat-test-scenes/van-gogh-room.glb"),
]


def get_shortest_path(sim, samples):
    path_results = []

    for sample in samples:
        path = habitat_sim.ShortestPath()
        path.requested_start = sample[0]
        path.requested_end = sample[1]
        found_path = sim.pathfinder.find_path(path)
        path_results.append((found_path, path.geodesic_distance, path.points))

    return path_results


@pytest.mark.parametrize("test_scene", test_scenes)
def test_sample_near(test_scene):
    if not osp.exists(test_scene):
        pytest.skip(f"{test_scene} not found")

    cfg_settings = examples.settings.default_sim_settings.copy()
    cfg_settings["scene"] = test_scene
    hab_cfg = examples.settings.make_cfg(cfg_settings)

    with habitat_sim.Simulator(hab_cfg) as sim:
        distance = 5.0
        num_samples = 100
        for _ in range(num_samples):
            point = sim.pathfinder.get_random_navigable_point()
            new_point = sim.pathfinder.get_random_navigable_point_near(
                point, distance, max_tries=100
            )
            assert sim.pathfinder.is_navigable(
                new_point
            ), f"{new_point} is not navigable. Derived from {point}"
            assert (
                np.linalg.norm(point - new_point) <= distance
            ), f"Point is not near enough: {point}, {new_point}"


@pytest.mark.parametrize("test_scene", test_scenes)
def test_recompute_navmesh(test_scene):
    if not osp.exists(test_scene):
        pytest.skip(f"{test_scene} not found")

    cfg_settings = examples.settings.default_sim_settings.copy()
    cfg_settings["scene"] = test_scene
    hab_cfg = examples.settings.make_cfg(cfg_settings)

    with habitat_sim.Simulator(hab_cfg) as sim:
        sim.navmesh_visualization = True
        assert sim.navmesh_visualization
        sim.navmesh_visualization = False
        assert not sim.navmesh_visualization

        # generate random point pairs
        num_samples = 100
        samples = []
        for _ in range(num_samples):
            samples.append(
                (
                    sim.pathfinder.get_random_navigable_point(),
                    sim.pathfinder.get_random_navigable_point(),
                )
            )

        # compute shortest paths between these points on the loaded navmesh
        loaded_navmesh_path_results = get_shortest_path(sim, samples)
        assert len(sim.pathfinder.build_navmesh_vertices()) > 0
        assert len(sim.pathfinder.build_navmesh_vertex_indices()) > 0
        hab_cfg = examples.settings.make_cfg(cfg_settings)
        agent_config = hab_cfg.agents[hab_cfg.sim_cfg.default_agent_id]
        agent_config.radius *= 2.0
        sim.reconfigure(hab_cfg)
        # compute shortest paths between these points on the loaded navmesh with twice radius
        loaded_navmesh_2rad_path_results = get_shortest_path(sim, samples)

        navmesh_settings = habitat_sim.NavMeshSettings()
        navmesh_settings.set_defaults()
        assert sim.recompute_navmesh(sim.pathfinder, navmesh_settings)
        assert sim.pathfinder.is_loaded
        assert len(sim.pathfinder.build_navmesh_vertices()) > 0
        assert len(sim.pathfinder.build_navmesh_vertex_indices()) > 0

        recomputed_navmesh_results = get_shortest_path(sim, samples)

        navmesh_settings.agent_radius *= 2.0
        assert sim.recompute_navmesh(sim.pathfinder, navmesh_settings)
        assert sim.pathfinder.is_loaded  # this may not always be viable...

        recomputed_2rad_navmesh_results = get_shortest_path(sim, samples)

        some_diff = False
        for i in range(num_samples):
            assert loaded_navmesh_path_results[i][0] == recomputed_navmesh_results[i][0]
            assert (
                loaded_navmesh_2rad_path_results[i][0]
                == recomputed_2rad_navmesh_results[i][0]
            )
            if loaded_navmesh_path_results[i][0]:
                assert (
                    abs(
                        loaded_navmesh_path_results[i][1]
                        - recomputed_navmesh_results[i][1]
                    )
                    < EPS
                )

            if recomputed_2rad_navmesh_results[i][0]:
                assert (
                    abs(
                        loaded_navmesh_2rad_path_results[i][1]
                        - recomputed_2rad_navmesh_results[i][1]
                    )
                    < EPS
                )

            if (
                loaded_navmesh_path_results[i][0]
                != recomputed_2rad_navmesh_results[i][0]
                or loaded_navmesh_path_results[i][1]
                - recomputed_2rad_navmesh_results[i][1]
                > EPS
            ):
                some_diff = True

        assert some_diff


@pytest.mark.parametrize("test_scene", test_scenes)
def test_navmesh_area(test_scene):
    if not osp.exists(test_scene):
        pytest.skip(f"{test_scene} not found")

    cfg_settings = examples.settings.default_sim_settings.copy()
    cfg_settings["scene"] = test_scene
    hab_cfg = examples.settings.make_cfg(cfg_settings)
    with habitat_sim.Simulator(hab_cfg) as sim:
        # get the initial navmesh area. This test assumes default navmesh assets.
        loadedNavMeshArea = sim.pathfinder.navigable_area
        if test_scene.endswith("skokloster-castle.glb"):
            assert math.isclose(loadedNavMeshArea, 226.65673828125)
        elif test_scene.endswith("van-gogh-room.glb"):
            assert math.isclose(loadedNavMeshArea, 9.17772102355957)

        navmesh_settings = habitat_sim.NavMeshSettings()
        navmesh_settings.set_defaults()
        assert sim.recompute_navmesh(sim.pathfinder, navmesh_settings)
        assert sim.pathfinder.is_loaded

        # get the re-computed navmesh area. This test assumes NavMeshSettings default values.
        recomputedNavMeshArea1 = sim.pathfinder.navigable_area
        if test_scene.endswith("skokloster-castle.glb"):
            assert math.isclose(recomputedNavMeshArea1, 565.1781616210938)
        elif test_scene.endswith("van-gogh-room.glb"):
            assert math.isclose(recomputedNavMeshArea1, 9.17772102355957)


@pytest.mark.parametrize("agent_radius_mul", [0.5, 1.0, 2.0])
def test_save_navmesh_settings(agent_radius_mul, tmpdir):
    test_scene = osp.join(
        base_dir, "data/scene_datasets/habitat-test-scenes/skokloster-castle.glb"
    )
    if not osp.exists(test_scene):
        pytest.skip(f"{test_scene} not found")

    cfg_settings = examples.settings.default_sim_settings.copy()
    cfg_settings["scene"] = test_scene
    hab_cfg = examples.settings.make_cfg(cfg_settings)
    agent_config = hab_cfg.agents[hab_cfg.sim_cfg.default_agent_id]
    agent_config.radius *= agent_radius_mul

    with habitat_sim.Simulator(hab_cfg) as sim:
        assert sim.pathfinder.is_loaded
        sim.pathfinder.save_nav_mesh(osp.join(tmpdir, "out.navmesh"))
        pathfinder = habitat_sim.PathFinder()
        assert pathfinder.nav_mesh_settings is None
        pathfinder.load_nav_mesh(osp.join(tmpdir, "out.navmesh"))
        assert pathfinder.is_loaded
        assert sim.pathfinder.nav_mesh_settings == pathfinder.nav_mesh_settings
