# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

"""
Unified tests for the nav/ subsystem: PathFinder, NavMeshSettings,
ShortestPath, MultiGoalShortestPath, HitRecord, GreedyGeodesicFollower,
topdown views, island queries, obstacle queries, and navmesh recomputation.

The GreedyGeodesicFollower SPL benchmark lives in test_greedy_follower.py
and the hypothesis-based snap_point fuzz test lives in test_snap_point.py.
"""

import json
import math
from os import path as osp
from typing import Any, Dict

import numpy as np
import pytest

# Need to import quaternion library here despite it not being used or else importing
# habitat_sim below will cause an invalid free() when audio is enabled in sim compilation
import quaternion  # noqa: F401

import habitat_sim
import habitat_sim.errors
import habitat_sim.utils.settings

EPS = 1e-5

base_dir = osp.abspath(osp.join(osp.dirname(__file__), ".."))

_test_scene = osp.join(
    base_dir, "data/scene_datasets/habitat-test-scenes/skokloster-castle.glb"
)
_test_navmesh = osp.join(
    base_dir, "data/scene_datasets/habitat-test-scenes/skokloster-castle.navmesh"
)

test_scenes = [
    osp.join(base_dir, "data/scene_datasets/mp3d_example/17DRP5sb8fy/17DRP5sb8fy.glb"),
    osp.join(base_dir, "data/scene_datasets/habitat-test-scenes/skokloster-castle.glb"),
    osp.join(base_dir, "data/scene_datasets/habitat-test-scenes/van-gogh-room.glb"),
]


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


@pytest.fixture(scope="module")
def pathfinder():
    """Load a PathFinder with the skokloster-castle navmesh."""
    if not osp.exists(_test_navmesh):
        pytest.skip(f"{_test_navmesh} not found")
    pf = habitat_sim.PathFinder()
    pf.load_nav_mesh(_test_navmesh)
    assert pf.is_loaded
    pf.seed(42)
    return pf


@pytest.fixture(scope="module")
def sim():
    """Create a Simulator with the skokloster-castle scene."""
    if not osp.exists(_test_scene):
        pytest.skip(f"{_test_scene} not found")
    cfg_settings = habitat_sim.utils.settings.default_sim_settings.copy()
    cfg_settings["scene"] = _test_scene
    hab_cfg = habitat_sim.utils.settings.make_cfg(cfg_settings)
    simulator = habitat_sim.Simulator(hab_cfg)
    yield simulator
    simulator.close()


def _get_shortest_path(sim, samples):
    """Helper: compute shortest paths for a list of (start, end) pairs."""
    results = []
    for start, end in samples:
        path = habitat_sim.ShortestPath()
        path.requested_start = start
        path.requested_end = end
        found = sim.pathfinder.find_path(path)
        results.append((found, path.geodesic_distance, path.points))
    return results


# ===================================================================
# PathFinder — construction and loading
# ===================================================================


def test_pathfinder_not_loaded():
    """A freshly constructed PathFinder should report is_loaded=False."""
    pf = habitat_sim.PathFinder()
    assert not pf.is_loaded
    assert pf.nav_mesh_settings is None


def test_pathfinder_save_load_round_trip(pathfinder, tmp_path):
    """A saved navmesh can be re-loaded with identical settings and area."""
    save_path = str(tmp_path / "round_trip.navmesh")
    pathfinder.save_nav_mesh(save_path)

    pf2 = habitat_sim.PathFinder()
    pf2.load_nav_mesh(save_path)
    assert pf2.is_loaded
    assert pathfinder.nav_mesh_settings == pf2.nav_mesh_settings
    assert abs(pathfinder.navigable_area - pf2.navigable_area) < 1e-3


@pytest.mark.parametrize("agent_radius_mul", [0.5, 1.0, 2.0])
def test_save_navmesh_settings(agent_radius_mul, tmp_path):
    """Saved navmesh files retain NavMeshSettings for varied agent radii."""
    if not osp.exists(_test_scene):
        pytest.skip(f"{_test_scene} not found")

    cfg_settings = habitat_sim.utils.settings.default_sim_settings.copy()
    cfg_settings["scene"] = _test_scene
    hab_cfg = habitat_sim.utils.settings.make_cfg(cfg_settings)
    agent_config = hab_cfg.agents[hab_cfg.sim_cfg.default_agent_id]
    agent_config.radius *= agent_radius_mul

    with habitat_sim.Simulator(hab_cfg) as _sim:
        assert _sim.pathfinder.is_loaded
        navmesh_path = str(tmp_path / "out.navmesh")
        _sim.pathfinder.save_nav_mesh(navmesh_path)

        pf = habitat_sim.PathFinder()
        assert pf.nav_mesh_settings is None
        pf.load_nav_mesh(navmesh_path)
        assert pf.is_loaded
        assert _sim.pathfinder.nav_mesh_settings == pf.nav_mesh_settings


# ===================================================================
# PathFinder — bounds
# ===================================================================


def test_pathfinder_get_bounds(pathfinder):
    """get_bounds returns (min, max) 3D vectors where max >= min."""
    bounds = pathfinder.get_bounds()
    assert len(bounds) == 2
    lo, hi = bounds
    assert len(lo) == 3 and len(hi) == 3
    for i in range(3):
        assert hi[i] >= lo[i], f"Axis {i}: max ({hi[i]}) < min ({lo[i]})"


# ===================================================================
# PathFinder — random navigable point sampling
# ===================================================================


@pytest.mark.parametrize("test_scene", test_scenes)
def test_get_random_navigable_point_near(test_scene):
    """Points from get_random_navigable_point_near are navigable and within
    the requested radius."""
    if not osp.exists(test_scene):
        pytest.skip(f"{test_scene} not found")

    cfg_settings = habitat_sim.utils.settings.default_sim_settings.copy()
    cfg_settings["scene"] = test_scene
    hab_cfg = habitat_sim.utils.settings.make_cfg(cfg_settings)

    with habitat_sim.Simulator(hab_cfg) as _sim:
        distance = 5.0
        for _ in range(100):
            point = _sim.pathfinder.get_random_navigable_point()
            new_point = _sim.pathfinder.get_random_navigable_point_near(
                point, distance, max_tries=100
            )
            assert _sim.pathfinder.is_navigable(
                new_point
            ), f"{new_point} is not navigable. Derived from {point}"
            assert (
                np.linalg.norm(point - new_point) <= distance
            ), f"Point not near enough: {point}, {new_point}"


# ===================================================================
# PathFinder — is_navigable
# ===================================================================


def test_is_navigable_with_custom_y_delta(pathfinder):
    """is_navigable respects max_y_delta: on-navmesh points pass, elevated
    points fail."""
    pt = pathfinder.get_random_navigable_point()
    assert pathfinder.is_navigable(pt)
    assert pathfinder.is_navigable(pt, max_y_delta=0.5)
    lifted = np.array(pt) + np.array([0.0, 10.0, 0.0])
    assert not pathfinder.is_navigable(lifted, max_y_delta=0.5)


# ===================================================================
# PathFinder — obstacle queries / HitRecord
# ===================================================================


def test_distance_to_closest_obstacle(pathfinder):
    """Returns a finite non-negative value for navigable points."""
    pt = pathfinder.get_random_navigable_point()
    dist = pathfinder.distance_to_closest_obstacle(pt, max_search_radius=5.0)
    assert dist >= 0.0
    assert math.isfinite(dist)


def test_distance_to_closest_obstacle_default_radius(pathfinder):
    """Works without an explicit max_search_radius."""
    pt = pathfinder.get_random_navigable_point()
    dist = pathfinder.distance_to_closest_obstacle(pt)
    assert dist >= 0.0


def test_closest_obstacle_surface_point(pathfinder):
    """Returns a HitRecord with valid position, normal, and distance."""
    pt = pathfinder.get_random_navigable_point()
    hit = pathfinder.closest_obstacle_surface_point(pt, max_search_radius=5.0)
    assert isinstance(hit, habitat_sim.HitRecord)
    assert len(hit.hit_pos) == 3
    assert len(hit.hit_normal) == 3
    assert math.isfinite(hit.hit_dist)


def test_hit_record_default_construction():
    """HitRecord can be default-constructed and its fields accessed."""
    hr = habitat_sim.HitRecord()
    _ = hr.hit_pos
    _ = hr.hit_normal
    _ = hr.hit_dist


# ===================================================================
# PathFinder — try_step / try_step_no_sliding
# ===================================================================


def test_try_step(pathfinder):
    """try_step returns a point on or near the navmesh."""
    start = pathfinder.get_random_navigable_point()
    end = start + np.array([0.1, 0.0, 0.1])
    result = pathfinder.try_step(start, end)
    assert len(result) == 3
    snapped = pathfinder.snap_point(result)
    assert np.allclose(result, snapped, atol=0.1)


def test_try_step_no_sliding(pathfinder):
    """try_step_no_sliding returns a 3D point."""
    start = pathfinder.get_random_navigable_point()
    end = start + np.array([0.1, 0.0, 0.1])
    result = pathfinder.try_step_no_sliding(start, end)
    assert len(result) == 3


def test_try_step_into_obstacle(pathfinder):
    """try_step clamps movement when the target is far beyond the navmesh."""
    start = pathfinder.get_random_navigable_point()
    end = start + np.array([100.0, 0.0, 100.0])
    result = pathfinder.try_step(start, end)
    assert not np.allclose(result, end, atol=1.0)


# ===================================================================
# ShortestPath
# ===================================================================


def test_shortest_path_fields():
    """ShortestPath fields are readable, writable, and have sane defaults."""
    sp = habitat_sim.ShortestPath()
    sp.requested_start = np.array([0.0, 0.0, 0.0])
    sp.requested_end = np.array([1.0, 0.0, 1.0])
    assert np.allclose(sp.requested_start, [0, 0, 0])
    assert np.allclose(sp.requested_end, [1, 0, 1])
    assert len(sp.points) == 0


def test_shortest_path_no_path(pathfinder):
    """find_path returns False and sets geodesic_distance=inf when no path
    exists."""
    sp = habitat_sim.ShortestPath()
    sp.requested_start = np.array([0.0, 0.0, 0.0])
    sp.requested_end = np.array([99999.0, 99999.0, 99999.0])
    found = pathfinder.find_path(sp)
    assert not found
    assert sp.geodesic_distance == float("inf")


# ===================================================================
# MultiGoalShortestPath
# ===================================================================


def test_multi_goal_shortest_path(pathfinder):
    """Finds the geodesically closest of several goals."""
    start = pathfinder.get_random_navigable_point()
    goals = [pathfinder.get_random_navigable_point() for _ in range(5)]

    mgsp = habitat_sim.MultiGoalShortestPath()
    mgsp.requested_start = start
    mgsp.requested_ends = goals

    found = pathfinder.find_path(mgsp)
    if found:
        assert mgsp.geodesic_distance < float("inf")
        assert len(mgsp.points) > 0
        assert 0 <= mgsp.closest_end_point_index < len(goals)
    else:
        assert mgsp.geodesic_distance == float("inf")


def test_multi_goal_shortest_path_single_goal(pathfinder):
    """With one goal, MultiGoalShortestPath matches single-goal
    ShortestPath."""
    start = pathfinder.get_random_navigable_point()
    goal = pathfinder.get_random_navigable_point()

    mgsp = habitat_sim.MultiGoalShortestPath()
    mgsp.requested_start = start
    mgsp.requested_ends = [goal]
    found_mg = pathfinder.find_path(mgsp)

    sp = habitat_sim.ShortestPath()
    sp.requested_start = start
    sp.requested_end = goal
    found_sg = pathfinder.find_path(sp)

    assert found_mg == found_sg
    if found_mg:
        assert abs(mgsp.geodesic_distance - sp.geodesic_distance) < 1e-5


# ===================================================================
# PathFinder — island queries
# ===================================================================

# Cached ground-truth for island assertions.
# scene_name -> {"num_islands": int, "island_cache": {idx: {"area", "radius"}}}
cached_island_results: Dict[str, Dict[str, Any]] = {
    "17DRP5sb8fy": {
        "island_cache": {
            0: {"area": 1.1425001621246338, "radius": 1.6779788732528687},
            1: {"area": 50.89643096923828, "radius": 8.037928581237793},
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
    """Validates island count, per-island area/radius, mesh data integrity,
    per-island random sampling, get_island, and snap_point."""
    if not osp.exists(test_scene):
        pytest.skip(f"{test_scene} not found")

    scene_name = test_scene.split("/")[-1].split(".")[0]
    expected = cached_island_results[scene_name]

    cfg_settings = habitat_sim.utils.settings.default_sim_settings.copy()
    cfg_settings["scene"] = test_scene
    hab_cfg = habitat_sim.utils.settings.make_cfg(cfg_settings)

    with habitat_sim.Simulator(hab_cfg) as _sim:
        num_islands = _sim.pathfinder.num_islands
        assert num_islands == expected["num_islands"]

        island_mesh_data: Dict[Any, Any] = {"total_verts": 0, "total_indices": 0}
        for island_index in range(num_islands):
            island_area = _sim.pathfinder.island_area(island_index)
            island_radius = _sim.pathfinder.island_radius(island_index)
            assert (
                island_area == expected["island_cache"][island_index]["area"]
            ), f"Island {island_index}: area mismatch"
            assert (
                island_radius == expected["island_cache"][island_index]["radius"]
            ), f"Island {island_index}: radius mismatch"

            # Per-island mesh data
            island_verts = _sim.pathfinder.build_navmesh_vertices(island_index)
            island_indices = _sim.pathfinder.build_navmesh_vertex_indices(island_index)
            island_mesh_data[island_index] = (island_verts, island_indices)
            assert len(island_verts) > 0
            assert len(island_indices) > 0
            island_mesh_data["total_verts"] += len(island_verts)
            island_mesh_data["total_indices"] += len(island_indices)

            for ix in island_indices:
                assert ix < len(island_verts), "Index out of vertex bounds"

            # No vertex should appear on a previous island
            for prev in range(island_index):
                for v in island_verts:
                    for ov in island_mesh_data[prev][0]:
                        assert not np.allclose(
                            v, ov
                        ), "Vertex duplicated across islands"

            # Per-island random sampling
            rand_pt = _sim.pathfinder.get_random_navigable_point(
                island_index=island_index
            )
            rand_pt_near = _sim.pathfinder.get_random_navigable_point_near(
                circle_center=rand_pt, radius=0.5, island_index=island_index
            )
            assert np.linalg.norm(rand_pt - rand_pt_near) < 0.5
            assert _sim.pathfinder.is_navigable(rand_pt)
            assert _sim.pathfinder.is_navigable(rand_pt_near)
            assert _sim.pathfinder.get_island(rand_pt) == island_index
            assert _sim.pathfinder.get_island(rand_pt_near) == island_index

            snapped = _sim.pathfinder.snap_point(rand_pt, island_index)
            assert np.allclose(rand_pt, snapped)
            snapped_near = _sim.pathfinder.snap_point(rand_pt_near, island_index)
            assert np.allclose(rand_pt_near, snapped_near)

        # Full-scene mesh totals should match sum of per-island meshes
        all_verts = _sim.pathfinder.build_navmesh_vertices()
        all_indices = _sim.pathfinder.build_navmesh_vertex_indices()
        assert len(all_verts) > 0
        assert len(all_indices) > 0
        assert island_mesh_data["total_verts"] == len(all_verts)
        assert island_mesh_data["total_indices"] == len(all_indices)


def test_island_radius_by_point(pathfinder):
    """island_radius(point) matches island_radius(get_island(point))."""
    pt = pathfinder.get_random_navigable_point()
    radius_by_pt = pathfinder.island_radius(pt)
    assert radius_by_pt > 0.0

    island_idx = pathfinder.get_island(pt)
    radius_by_idx = pathfinder.island_radius(island_idx)
    assert abs(radius_by_pt - radius_by_idx) < 1e-5


# ===================================================================
# Navigable area
# ===================================================================


@pytest.mark.parametrize("test_scene", test_scenes)
def test_navmesh_area(test_scene):
    """Loaded and recomputed navmesh areas match expected ground-truth."""
    if not osp.exists(test_scene):
        pytest.skip(f"{test_scene} not found")

    cfg_settings = habitat_sim.utils.settings.default_sim_settings.copy()
    cfg_settings["scene"] = test_scene
    hab_cfg = habitat_sim.utils.settings.make_cfg(cfg_settings)
    with habitat_sim.Simulator(hab_cfg) as _sim:
        loaded_area = _sim.pathfinder.navigable_area
        if test_scene.endswith("skokloster-castle.glb"):
            assert math.isclose(loaded_area, 226.65673828125)
        elif test_scene.endswith("van-gogh-room.glb"):
            assert math.isclose(loaded_area, 9.17772102355957)

        navmesh_settings = habitat_sim.NavMeshSettings()
        navmesh_settings.set_defaults()
        assert _sim.recompute_navmesh(_sim.pathfinder, navmesh_settings)
        assert _sim.pathfinder.is_loaded

        recomputed_area = _sim.pathfinder.navigable_area
        if test_scene.endswith("skokloster-castle.glb"):
            assert math.isclose(recomputed_area, 565.177978515625)
        elif test_scene.endswith("van-gogh-room.glb"):
            assert math.isclose(recomputed_area, 9.17772102355957)


# ===================================================================
# Navmesh recomputation
# ===================================================================


@pytest.mark.parametrize("test_scene", test_scenes)
def test_recompute_navmesh(test_scene):
    """Recomputing with same NavMeshSettings gives identical shortest-path
    results; doubling agent_radius produces at least some differences."""
    if not osp.exists(test_scene):
        pytest.skip(f"{test_scene} not found")

    cfg_settings = habitat_sim.utils.settings.default_sim_settings.copy()
    cfg_settings["scene"] = test_scene
    hab_cfg = habitat_sim.utils.settings.make_cfg(cfg_settings)

    with habitat_sim.Simulator(hab_cfg) as _sim:
        num_samples = 100
        samples = [
            (
                _sim.pathfinder.get_random_navigable_point(),
                _sim.pathfinder.get_random_navigable_point(),
            )
            for _ in range(num_samples)
        ]

        loaded_results = _get_shortest_path(_sim, samples)
        assert len(_sim.pathfinder.build_navmesh_vertices()) > 0
        assert len(_sim.pathfinder.build_navmesh_vertex_indices()) > 0

        # Recompute with default settings — should be identical
        navmesh_settings = habitat_sim.NavMeshSettings()
        navmesh_settings.set_defaults()
        assert _sim.recompute_navmesh(_sim.pathfinder, navmesh_settings)
        assert _sim.pathfinder.is_loaded
        assert len(_sim.pathfinder.build_navmesh_vertices()) > 0
        assert len(_sim.pathfinder.build_navmesh_vertex_indices()) > 0

        recomputed_results = _get_shortest_path(_sim, samples)

        for i in range(num_samples):
            assert loaded_results[i][0] == recomputed_results[i][0]
            if loaded_results[i][0]:
                assert abs(loaded_results[i][1] - recomputed_results[i][1]) < EPS

        # Recompute with doubled agent_radius — should differ somewhere
        navmesh_settings.agent_radius *= 2.0
        assert _sim.recompute_navmesh(_sim.pathfinder, navmesh_settings)

        doubled_results = _get_shortest_path(_sim, samples)
        some_diff = any(
            loaded_results[i][0] != doubled_results[i][0]
            or loaded_results[i][1] - doubled_results[i][1] > EPS
            for i in range(num_samples)
        )
        assert some_diff, "Doubling agent_radius should change at least one path"


# ===================================================================
# Navmesh visualization toggle (extracted from old test_recompute_navmesh)
# ===================================================================


@pytest.mark.gfxtest
@pytest.mark.parametrize("test_scene", test_scenes)
def test_navmesh_visualization_toggle(test_scene):
    """navmesh_visualization can be toggled on and off."""
    if not osp.exists(test_scene):
        pytest.skip(f"{test_scene} not found")

    cfg_settings = habitat_sim.utils.settings.default_sim_settings.copy()
    cfg_settings["scene"] = test_scene
    hab_cfg = habitat_sim.utils.settings.make_cfg(cfg_settings)

    with habitat_sim.Simulator(hab_cfg) as _sim:
        _sim.navmesh_visualization = True
        assert _sim.navmesh_visualization
        _sim.navmesh_visualization = False
        assert not _sim.navmesh_visualization


# ===================================================================
# Topdown map views
# ===================================================================


@pytest.mark.parametrize("test_scene", test_scenes)
def test_topdown_map(test_scene):
    """Topdown binary and island views match cached ground-truth arrays,
    and get_island_colored_map produces an image with the correct size."""
    if not osp.exists(test_scene):
        pytest.skip(f"{test_scene} not found")

    scene_name = test_scene.split("/")[-1].split(".")[0]

    cfg_settings = habitat_sim.utils.settings.default_sim_settings.copy()
    cfg_settings["scene"] = test_scene
    hab_cfg = habitat_sim.utils.settings.make_cfg(cfg_settings)

    with habitat_sim.Simulator(hab_cfg) as _sim:
        navmesh_verts = _sim.pathfinder.build_navmesh_vertices(-1)
        height = min(x[1] for x in navmesh_verts)

        binary_map = _sim.pathfinder.get_topdown_view(0.1, height)
        island_map = _sim.pathfinder.get_topdown_island_view(0.1, height)

        # Compare against cached ground-truth
        binary_cache_path = osp.join(
            base_dir, f"data/test_assets/{scene_name}_topdown_binary.npy"
        )
        island_cache_path = osp.join(
            base_dir, f"data/test_assets/{scene_name}_topdown_islands.npy"
        )
        if not osp.exists(binary_cache_path) or not osp.exists(island_cache_path):
            pytest.skip(f"Cached topdown maps not found for {scene_name}")

        assert np.array_equal(np.load(binary_cache_path), binary_map)
        assert np.array_equal(np.load(island_cache_path), island_map)

        # Verify that the island colored map utility runs without error
        from habitat_sim.utils.viz_utils import get_island_colored_map

        colored_image = get_island_colored_map(island_map)
        assert colored_image.size[0] > 0
        assert colored_image.size[1] > 0


# ===================================================================
# NavMeshSettings — field access, equality, JSON round-trip
# ===================================================================


def test_navmesh_settings_defaults():
    """NavMeshSettings.set_defaults populates sensible values."""
    settings = habitat_sim.NavMeshSettings()
    settings.set_defaults()
    assert settings.cell_size > 0
    assert settings.cell_height > 0
    assert settings.agent_height > 0
    assert settings.agent_radius > 0
    assert settings.agent_max_climb >= 0
    assert settings.agent_max_slope > 0
    assert settings.region_min_size >= 0
    assert settings.region_merge_size >= 0
    assert settings.verts_per_poly >= 3
    assert settings.filter_low_hanging_obstacles in (True, False)
    assert settings.filter_ledge_spans in (True, False)
    assert settings.filter_walkable_low_height_spans in (True, False)


def test_navmesh_settings_equality():
    """Two NavMeshSettings with the same defaults should be equal."""
    s1 = habitat_sim.NavMeshSettings()
    s1.set_defaults()
    s2 = habitat_sim.NavMeshSettings()
    s2.set_defaults()
    assert s1 == s2
    assert s1 == s2


def test_navmesh_settings_inequality():
    """NavMeshSettings with different values should be unequal."""
    s1 = habitat_sim.NavMeshSettings()
    s1.set_defaults()
    s2 = habitat_sim.NavMeshSettings()
    s2.set_defaults()
    s2.agent_radius = s1.agent_radius * 3.0
    assert s1 != s2
    assert s1 != s2


def test_navmesh_settings_field_modification():
    """Individual NavMeshSettings fields can be read and written."""
    settings = habitat_sim.NavMeshSettings()
    settings.set_defaults()

    settings.cell_size = 0.123
    assert abs(settings.cell_size - 0.123) < 1e-6

    settings.agent_height = 2.5
    assert abs(settings.agent_height - 2.5) < 1e-6

    settings.include_static_objects = True
    assert settings.include_static_objects is True


def test_navmesh_settings_json_round_trip(tmp_path):
    """NavMeshSettings can be written to JSON and read back identically."""
    settings = habitat_sim.NavMeshSettings()
    settings.set_defaults()
    settings.agent_radius = 0.42

    json_path = str(tmp_path / "navmesh_settings.json")
    settings.write_to_json(json_path)

    assert osp.exists(json_path)
    with open(json_path) as f:
        data = json.load(f)
    assert isinstance(data, dict)

    settings2 = habitat_sim.NavMeshSettings()
    settings2.read_from_json(json_path)
    assert settings == settings2


# ===================================================================
# GreedyGeodesicFollower — action key resolution
# ===================================================================


def test_greedy_follower_default_keys(sim):
    """Default action keys are resolved from the agent's action_space."""
    agent = sim.get_agent(0)
    follower = habitat_sim.GreedyGeodesicFollower(sim.pathfinder, agent)
    from habitat_sim.nav import GreedyFollowerCodes

    assert follower.action_mapping[GreedyFollowerCodes.FORWARD] == "move_forward"
    assert follower.action_mapping[GreedyFollowerCodes.LEFT] == "turn_left"
    assert follower.action_mapping[GreedyFollowerCodes.RIGHT] == "turn_right"
    assert follower.action_mapping[GreedyFollowerCodes.STOP] is None


def test_greedy_follower_custom_keys(sim):
    """Custom forward/left/right/stop keys override defaults."""
    agent = sim.get_agent(0)
    follower = habitat_sim.GreedyGeodesicFollower(
        sim.pathfinder,
        agent,
        stop_key="STOP",
        forward_key="FWD",
        left_key="LT",
        right_key="RT",
    )
    from habitat_sim.nav import GreedyFollowerCodes

    assert follower.action_mapping[GreedyFollowerCodes.FORWARD] == "FWD"
    assert follower.action_mapping[GreedyFollowerCodes.LEFT] == "LT"
    assert follower.action_mapping[GreedyFollowerCodes.RIGHT] == "RT"
    assert follower.action_mapping[GreedyFollowerCodes.STOP] == "STOP"


# ===================================================================
# GreedyGeodesicFollower — goal_radius
# ===================================================================


def test_greedy_follower_custom_goal_radius(sim):
    """A custom goal_radius overrides the default."""
    agent = sim.get_agent(0)
    follower = habitat_sim.GreedyGeodesicFollower(
        sim.pathfinder, agent, goal_radius=1.5
    )
    assert follower.goal_radius == 1.5


def test_greedy_follower_default_goal_radius(sim):
    """Default goal_radius is 0.75 * forward_spec.amount."""
    agent = sim.get_agent(0)
    follower = habitat_sim.GreedyGeodesicFollower(sim.pathfinder, agent)
    expected = 0.75 * follower.forward_spec.amount
    assert abs(follower.goal_radius - expected) < 1e-6


# ===================================================================
# GreedyGeodesicFollower — reset and goal caching
# ===================================================================


def test_greedy_follower_reset_clears_goal(sim):
    """reset() clears last_goal."""
    agent = sim.get_agent(0)
    follower = habitat_sim.GreedyGeodesicFollower(sim.pathfinder, agent)
    follower.last_goal = np.array([1.0, 0.0, 1.0])
    follower.reset()
    assert follower.last_goal is None


def test_greedy_follower_goal_caching(sim):
    """Repeated next_action_along with the same goal does not reset."""
    agent = sim.get_agent(0)
    follower = habitat_sim.GreedyGeodesicFollower(sim.pathfinder, agent)

    state = habitat_sim.AgentState()
    state.position = sim.pathfinder.get_random_navigable_point()
    agent.state = state

    goal = sim.pathfinder.get_random_navigable_point()
    path = habitat_sim.ShortestPath()
    path.requested_start = state.position
    path.requested_end = goal
    if not sim.pathfinder.find_path(path) or path.geodesic_distance < 2.0:
        pytest.skip("Could not find a suitable path for goal caching test")

    try:
        follower.next_action_along(goal)
    except habitat_sim.errors.GreedyFollowerError:
        pytest.skip("GreedyFollowerError on first action")

    assert follower.last_goal is not None
    assert np.allclose(follower.last_goal, goal)

    try:
        follower.next_action_along(goal)
    except habitat_sim.errors.GreedyFollowerError:
        pass
    assert follower.last_goal is not None
    assert np.allclose(follower.last_goal, goal)


def test_greedy_follower_new_goal_resets(sim):
    """next_action_along with a different goal updates last_goal."""
    agent = sim.get_agent(0)
    follower = habitat_sim.GreedyGeodesicFollower(sim.pathfinder, agent)

    state = habitat_sim.AgentState()
    state.position = sim.pathfinder.get_random_navigable_point()
    agent.state = state

    goal1 = sim.pathfinder.get_random_navigable_point()
    goal2 = sim.pathfinder.get_random_navigable_point()

    try:
        follower.next_action_along(goal1)
    except habitat_sim.errors.GreedyFollowerError:
        pass

    try:
        follower.next_action_along(goal2)
    except habitat_sim.errors.GreedyFollowerError:
        pass
    assert follower.last_goal is not None
    assert np.allclose(follower.last_goal, goal2)


# ===================================================================
# GreedyGeodesicFollower — find_path output format
# ===================================================================


def test_greedy_follower_find_path_ends_with_none(sim):
    """find_path returns a list of action keys ending with None (stop)."""
    agent = sim.get_agent(0)
    follower = habitat_sim.GreedyGeodesicFollower(sim.pathfinder, agent)

    for _ in range(20):
        state = habitat_sim.AgentState()
        state.position = sim.pathfinder.get_random_navigable_point()
        agent.state = state

        goal = sim.pathfinder.get_random_navigable_point()
        path = habitat_sim.ShortestPath()
        path.requested_start = state.position
        path.requested_end = goal

        if not sim.pathfinder.find_path(path) or path.geodesic_distance < 2.0:
            continue

        try:
            actions = follower.find_path(goal)
        except habitat_sim.errors.GreedyFollowerError:
            continue

        assert len(actions) > 0
        assert actions[-1] is None, "Last action should be None (stop key)"
        for a in actions[:-1]:
            assert a in ("move_forward", "turn_left", "turn_right")
        return

    pytest.skip("Could not find a suitable path after 20 attempts")
