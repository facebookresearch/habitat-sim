# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import math
from os import path as osp
from typing import Any, Dict

import numpy as np
import pytest

import habitat_sim
import habitat_sim.utils.settings

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

    cfg_settings = habitat_sim.utils.settings.default_sim_settings.copy()
    cfg_settings["scene"] = test_scene
    hab_cfg = habitat_sim.utils.settings.make_cfg(cfg_settings)

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

    cfg_settings = habitat_sim.utils.settings.default_sim_settings.copy()
    cfg_settings["scene"] = test_scene
    hab_cfg = habitat_sim.utils.settings.make_cfg(cfg_settings)

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
        cfg_settings["agent_radius"] = 0.2
        hab_cfg = habitat_sim.utils.settings.make_cfg(cfg_settings)
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

    cfg_settings = habitat_sim.utils.settings.default_sim_settings.copy()
    cfg_settings["scene"] = test_scene
    hab_cfg = habitat_sim.utils.settings.make_cfg(cfg_settings)
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
            assert math.isclose(recomputedNavMeshArea1, 565.177978515625)
        elif test_scene.endswith("van-gogh-room.glb"):
            assert math.isclose(recomputedNavMeshArea1, 9.17772102355957)


@pytest.mark.parametrize("agent_radius_mul", [0.5, 1.0, 2.0])
def test_save_navmesh_settings(agent_radius_mul, tmpdir):
    test_scene = osp.join(
        base_dir, "data/scene_datasets/habitat-test-scenes/skokloster-castle.glb"
    )
    if not osp.exists(test_scene):
        pytest.skip(f"{test_scene} not found")

    cfg_settings = habitat_sim.utils.settings.default_sim_settings.copy()
    cfg_settings["scene"] = test_scene
    hab_cfg = habitat_sim.utils.settings.make_cfg(cfg_settings)
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


# cached test results for assertions
cached_islandtest_scene_results: Dict[str, Dict[str, Any]] = {
    "17DRP5sb8fy": {
        "island_cache": {
            0: {"area": 1.1425001621246338, "radius": 1.6779788732528687},
            1: {"area": 50.89643096923828, "radius": 8.03792953491211},
        },
        "num_islands": 2,
    },
    "skokloster-castle": {
        "island_cache": {0: {"area": 226.65673828125, "radius": 12.550561904907227}},
        "num_islands": 1,
    },
    "van-gogh-room": {
        "island_cache": {
            0: {"area": 7.305000305175781, "radius": 3.304431200027466},
            1: {"area": 1.8727209568023682, "radius": 1.4128597974777222},
        },
        "num_islands": 2,
    },
}


@pytest.mark.parametrize("test_scene", test_scenes)
def test_navmesh_islands(test_scene):
    if not osp.exists(test_scene):
        pytest.skip(f"{test_scene} not found")

    test_scene_name = test_scene.split("/")[-1].split(".")[0]
    cached_test_results = cached_islandtest_scene_results[test_scene_name]

    cfg_settings = habitat_sim.utils.settings.default_sim_settings.copy()
    cfg_settings["scene"] = test_scene
    hab_cfg = habitat_sim.utils.settings.make_cfg(cfg_settings)

    with habitat_sim.Simulator(hab_cfg) as sim:
        # print(dir(sim.pathfinder))
        # print(f"sim.pathfinder.num_islands = {sim.pathfinder.num_islands}")
        num_islands = sim.pathfinder.num_islands
        # cached_test_results["num_islands"] = num_islands
        assert num_islands == cached_test_results["num_islands"]
        island_mesh_data: Dict[Any, Any] = {
            "total_verts": 0,
            "total_indicies": 0,
        }
        for island_index in range(num_islands):
            print(f"Island {island_index}")
            island_area = sim.pathfinder.island_area(island_index)
            island_radius = sim.pathfinder.island_radius(island_index)
            # cached_test_results["island_cache"][island_index] = {}
            # cached_test_results["island_cache"][island_index]["area"] = island_area
            assert (
                island_area == cached_test_results["island_cache"][island_index]["area"]
            )
            # cached_test_results["island_cache"][island_index]["radius"] = island_radius
            assert (
                island_radius
                == cached_test_results["island_cache"][island_index]["radius"]
            )

            # test getting island submeshes getNavMeshData
            island_mesh_verts = sim.pathfinder.build_navmesh_vertices(island_index)
            island_mesh_indices = sim.pathfinder.build_navmesh_vertex_indices(
                island_index
            )
            island_mesh_data[island_index] = (island_mesh_verts, island_mesh_indices)
            assert len(island_mesh_verts) > 0
            assert len(island_mesh_indices) > 0
            island_mesh_data["total_verts"] += len(island_mesh_verts)
            island_mesh_data["total_indicies"] += len(island_mesh_indices)

            # sanity check that no indices are higher than number of verts
            for ix in island_mesh_indices:
                assert ix < len(island_mesh_verts)

            # sanity check that no vertices exist on multiple islands
            for prev_island in range(island_index):
                for vert in island_mesh_verts:
                    for other_vert in island_mesh_data[prev_island][0]:
                        assert not np.allclose(
                            vert, other_vert
                        ), "Vertex is duplicated in multiple islands."

            # test random sample on islands
            rand_island_point = sim.pathfinder.get_random_navigable_point(
                island_index=island_index
            )
            rand_island_point_sphere = sim.pathfinder.get_random_navigable_point_near(
                circle_center=rand_island_point, radius=0.5, island_index=island_index
            )
            assert np.linalg.norm(rand_island_point - rand_island_point_sphere) < 0.5
            assert sim.pathfinder.is_navigable(rand_island_point)
            assert sim.pathfinder.is_navigable(rand_island_point_sphere)

            # test get island
            assert sim.pathfinder.get_island(rand_island_point) == island_index
            assert sim.pathfinder.get_island(rand_island_point_sphere) == island_index

            # test snap point on islands
            snapped_island_point = sim.pathfinder.snap_point(
                rand_island_point, island_index
            )
            assert np.allclose(rand_island_point, snapped_island_point)
            snapped_island_point_sphere = sim.pathfinder.snap_point(
                rand_island_point_sphere, island_index
            )
            assert np.allclose(rand_island_point_sphere, snapped_island_point_sphere)

        # mesh data for the full scene
        navmesh_verts = sim.pathfinder.build_navmesh_vertices()
        navmesh_indices = sim.pathfinder.build_navmesh_vertex_indices()
        assert len(navmesh_verts) > 0
        assert len(navmesh_indices) > 0

        # check total mesh data against island meshes
        assert island_mesh_data["total_verts"] == len(navmesh_verts)
        assert island_mesh_data["total_indicies"] == len(navmesh_indices)


@pytest.mark.parametrize("test_scene", test_scenes)
def test_topdown_map(test_scene):
    # optionally generate human-readable map images
    generate_test_map_images = False

    if not osp.exists(test_scene):
        pytest.skip(f"{test_scene} not found")

    test_scene_name = test_scene.split("/")[-1].split(".")[0]

    cfg_settings = habitat_sim.utils.settings.default_sim_settings.copy()
    cfg_settings["scene"] = test_scene
    hab_cfg = habitat_sim.utils.settings.make_cfg(cfg_settings)

    with habitat_sim.Simulator(hab_cfg) as sim:
        # use the lowest navmesh vert as the slice height
        navmesh_verts = sim.pathfinder.build_navmesh_vertices(-1)
        height = min(x[1] for x in navmesh_verts)

        binary_top_down_map = sim.pathfinder.get_topdown_view(0.1, height)
        island_top_down_map = sim.pathfinder.get_topdown_island_view(0.1, height)

        # cache new ground truth map caches
        filename = osp.join(
            base_dir, "data/test_assets/" + test_scene_name + "_topdown_binary.npy"
        )
        # np.save(filename, binary_top_down_map)
        binary_topdown_map_cached = np.load(filename)

        filename = osp.join(
            base_dir, "data/test_assets/" + test_scene_name + "_topdown_islands.npy"
        )
        # np.save(filename, island_top_down_map)
        islands_topdown_map_cached = np.load(filename)

        # check the computed data against the cache
        assert np.array_equal(binary_topdown_map_cached, binary_top_down_map)
        assert np.array_equal(islands_topdown_map_cached, island_top_down_map)

        # example/test of simple image generation from island maps
        from habitat_sim.utils.viz_utils import get_island_colored_map

        island_colored_map_image = get_island_colored_map(island_top_down_map)

        if generate_test_map_images:
            island_colored_map_image.save(filename + ".png")
            # island_colored_map_image.show()
