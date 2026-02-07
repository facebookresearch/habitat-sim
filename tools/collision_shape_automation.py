# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import argparse
import csv
import ctypes
import math
import os
import random
import sys
import time
from typing import Any, Dict, List, Optional, Tuple

coacd_imported = False
try:
    import coacd
    import trimesh

    coacd_imported = True
except Exception:
    coacd_imported = False
    print("Failed to import coacd, is it installed? Linux only: 'pip install coacd'")

# not adding this causes some failures in mesh import
flags = sys.getdlopenflags()
sys.setdlopenflags(flags | ctypes.RTLD_GLOBAL)


# imports from Habitat-lab
# NOTE: requires PR 1108 branch: rearrange-gen-improvements (https://github.com/facebookresearch/habitat-lab/pull/1108)
import habitat.datasets.rearrange.samplers.receptacle as hab_receptacle
import habitat.sims.habitat_simulator.debug_visualizer as hab_debug_vis
import magnum as mn
import numpy as np
from habitat.sims.habitat_simulator.sim_utilities import snap_down

import habitat_sim
from habitat_sim.utils.settings import default_sim_settings, make_cfg

# object samples:
# chair - good approximation: 0a5e809804911e71de6a4ef89f2c8fef5b9291b3.glb
# shelves - bad approximation: d1d1e0cdaba797ee70882e63f66055675c3f1e7f.glb

# 71 equidistant points on a unit hemisphere generated from icosphere subdivision
# Sphere center is (0,0,0) and no points lie on x,z plane
# used for hemisphere raycasting from Receptacle points
icosphere_points_subdiv_3 = [
    mn.Vector3(-0.276388, 0.447220, -0.850649),
    mn.Vector3(-0.483971, 0.502302, -0.716565),
    mn.Vector3(-0.232822, 0.657519, -0.716563),
    mn.Vector3(0.723607, 0.447220, -0.525725),
    mn.Vector3(0.531941, 0.502302, -0.681712),
    mn.Vector3(0.609547, 0.657519, -0.442856),
    mn.Vector3(0.723607, 0.447220, 0.525725),
    mn.Vector3(0.812729, 0.502301, 0.295238),
    mn.Vector3(0.609547, 0.657519, 0.442856),
    mn.Vector3(-0.276388, 0.447220, 0.850649),
    mn.Vector3(-0.029639, 0.502302, 0.864184),
    mn.Vector3(-0.232822, 0.657519, 0.716563),
    mn.Vector3(-0.894426, 0.447216, 0.000000),
    mn.Vector3(-0.831051, 0.502299, 0.238853),
    mn.Vector3(-0.753442, 0.657515, 0.000000),
    mn.Vector3(-0.251147, 0.967949, 0.000000),
    mn.Vector3(-0.077607, 0.967950, 0.238853),
    mn.Vector3(0.000000, 1.000000, 0.000000),
    mn.Vector3(-0.525730, 0.850652, 0.000000),
    mn.Vector3(-0.361800, 0.894429, 0.262863),
    mn.Vector3(-0.638194, 0.723610, 0.262864),
    mn.Vector3(-0.162456, 0.850654, 0.499995),
    mn.Vector3(-0.447209, 0.723612, 0.525728),
    mn.Vector3(-0.688189, 0.525736, 0.499997),
    mn.Vector3(-0.483971, 0.502302, 0.716565),
    mn.Vector3(0.203181, 0.967950, 0.147618),
    mn.Vector3(0.138197, 0.894430, 0.425319),
    mn.Vector3(0.052790, 0.723612, 0.688185),
    mn.Vector3(0.425323, 0.850654, 0.309011),
    mn.Vector3(0.361804, 0.723612, 0.587778),
    mn.Vector3(0.262869, 0.525738, 0.809012),
    mn.Vector3(0.531941, 0.502302, 0.681712),
    mn.Vector3(0.203181, 0.967950, -0.147618),
    mn.Vector3(0.447210, 0.894429, 0.000000),
    mn.Vector3(0.670817, 0.723611, 0.162457),
    mn.Vector3(0.425323, 0.850654, -0.309011),
    mn.Vector3(0.670817, 0.723611, -0.162457),
    mn.Vector3(0.850648, 0.525736, 0.000000),
    mn.Vector3(0.812729, 0.502301, -0.295238),
    mn.Vector3(-0.077607, 0.967950, -0.238853),
    mn.Vector3(0.138197, 0.894430, -0.425319),
    mn.Vector3(0.361804, 0.723612, -0.587778),
    mn.Vector3(-0.162456, 0.850654, -0.499995),
    mn.Vector3(0.052790, 0.723612, -0.688185),
    mn.Vector3(0.262869, 0.525738, -0.809012),
    mn.Vector3(-0.029639, 0.502302, -0.864184),
    mn.Vector3(-0.361800, 0.894429, -0.262863),
    mn.Vector3(-0.447209, 0.723612, -0.525728),
    mn.Vector3(-0.638194, 0.723610, -0.262864),
    mn.Vector3(-0.688189, 0.525736, -0.499997),
    mn.Vector3(-0.831051, 0.502299, -0.238853),
    mn.Vector3(-0.956626, 0.251149, 0.147618),
    mn.Vector3(-0.861804, 0.276396, 0.425322),
    mn.Vector3(-0.670821, 0.276397, 0.688189),
    mn.Vector3(-0.436007, 0.251152, 0.864188),
    mn.Vector3(-0.155215, 0.251152, 0.955422),
    mn.Vector3(0.138199, 0.276397, 0.951055),
    mn.Vector3(0.447215, 0.276397, 0.850649),
    mn.Vector3(0.687159, 0.251152, 0.681715),
    mn.Vector3(0.860698, 0.251151, 0.442858),
    mn.Vector3(0.947213, 0.276396, 0.162458),
    mn.Vector3(0.947213, 0.276397, -0.162458),
    mn.Vector3(0.860698, 0.251151, -0.442858),
    mn.Vector3(0.687159, 0.251152, -0.681715),
    mn.Vector3(0.447216, 0.276397, -0.850648),
    mn.Vector3(0.138199, 0.276397, -0.951055),
    mn.Vector3(-0.155215, 0.251152, -0.955422),
    mn.Vector3(-0.436007, 0.251152, -0.864188),
    mn.Vector3(-0.670820, 0.276396, -0.688190),
    mn.Vector3(-0.861804, 0.276394, -0.425323),
    mn.Vector3(-0.956626, 0.251149, -0.147618),
]


def get_scaled_hemisphere_vectors(scale: float):
    """
    Scales the icosphere_points for use with raycasting applications.
    """
    return [v * scale for v in icosphere_points_subdiv_3]


class COACDParams:
    def __init__(
        self,
    ) -> None:
        # Parameter tuning tricks from https://github.com/SarahWeiii/CoACD:

        # The default parameters are fast versions. If you care less about running time but more about the number of components, try to increase searching depth, searching node, and searching iteration for better cutting strategies.
        self.threshold = 0.05  # adjust the threshold (0.01~1) to balance the level of detail and the number of decomposed components. A higher value gives coarser results, and a lower value gives finer-grained results. You can refer to Fig. 14 in our paper for more details.
        self.max_convex_hull = -1
        self.preprocess = True  # ensure input mesh is 2-manifold solid if you want to skip pre-process. Skipping manifold pre-processing can better preserve input details, but can crash or fail otherwise if input is not manifold.
        self.preprocess_resolution = 30  # controls the quality of manifold preprocessing. A larger value can make the preprocessed mesh closer to the original mesh but also lead to more triangles and longer runtime.
        self.mcts_nodes = 20
        self.mcts_iterations = 150
        self.mcts_max_depth = 3
        self.pca = False
        self.merge = True
        self.seed = 0

    def __str__(self) -> str:
        return f"COACDParams(threshold={self.threshold} | max_convex_hull={self.max_convex_hull} | preprocess={self.preprocess} | preprocess_resolution={self.preprocess_resolution} | mcts_nodes={self.mcts_nodes} | mcts_iterations={self.mcts_iterations} | mcts_max_depth={self.mcts_max_depth} | pca={self.pca} | merge={self.merge} | seed={self.seed})"


def print_dict_structure(input_dict: Dict[Any, Any], whitespace: str = "") -> None:
    """
    Quick structure investigation for dictionary.
    Prints dict key->type recursively with incremental whitespace formatting.
    """
    if whitespace == "":
        print("-----------------------------------")
        print("Print Dict Structure Results:")
    for key in input_dict:
        if isinstance(input_dict[key], Dict):
            print(whitespace + f"{key}:-")
            print_dict_structure(
                input_dict=input_dict[key], whitespace=whitespace + " "
            )
        else:
            print(whitespace + f"{key}: {type(input_dict[key])}")
    if whitespace == "":
        print("-----------------------------------")


# =======================================================================
# Range3D surface sampling utils


def compute_area_weights_for_range3d_faces(range3d: mn.Range3D):
    """
    Compute a set of area weights from a Range3D.
    """

    face_areas = [
        range3d.size_x() * range3d.size_y(),  # front/back
        range3d.size_x() * range3d.size_z(),  # top/bottom
        range3d.size_y() * range3d.size_z(),  # sides
    ]
    area_accumulator = []
    for ix in range(6):
        area_ix = ix % 3
        if ix == 0:
            area_accumulator.append(face_areas[area_ix])
        else:
            area_accumulator.append(face_areas[area_ix] + area_accumulator[-1])

    normalized_area_accumulator = [x / area_accumulator[-1] for x in area_accumulator]

    return normalized_area_accumulator


def get_range3d_sample_planes(range3d: mn.Range3D):
    """
    Get origin and basis vectors for each face's sample planes.
    """
    # For each face a starting point and two edge vectors (un-normalized)
    face_info: List[Tuple[mn.Vector3, mn.Vector3, mn.Vector3]] = [
        (
            range3d.front_bottom_left,
            mn.Vector3.x_axis(range3d.size_x()),
            mn.Vector3.y_axis(range3d.size_y()),
        ),  # front
        (
            range3d.back_top_left,
            mn.Vector3.x_axis(range3d.size_x()),
            mn.Vector3.z_axis(range3d.size_z()),
        ),  # top
        (
            range3d.back_bottom_left,
            mn.Vector3.y_axis(range3d.size_y()),
            mn.Vector3.z_axis(range3d.size_z()),
        ),  # left
        (
            range3d.back_bottom_left,
            mn.Vector3.x_axis(range3d.size_x()),
            mn.Vector3.y_axis(range3d.size_y()),
        ),  # back
        (
            range3d.back_bottom_left,
            mn.Vector3.x_axis(range3d.size_x()),
            mn.Vector3.z_axis(range3d.size_z()),
        ),  # bottom
        (
            range3d.back_bottom_right,
            mn.Vector3.y_axis(range3d.size_y()),
            mn.Vector3.z_axis(range3d.size_z()),
        ),  # right
    ]
    return face_info


def sample_jittered_points_from_range3d(range3d: mn.Range3D, num_points: int = 100):
    """
    Use jittered sampling to compute a more uniformly distributed set of random points.
    """
    normalized_area_accumulator = compute_area_weights_for_range3d_faces(range3d)
    normalized_areas = []
    for vix in range(len(normalized_area_accumulator)):
        if vix == 0:
            normalized_areas.append(normalized_area_accumulator[vix])
        else:
            normalized_areas.append(
                normalized_area_accumulator[vix] - normalized_area_accumulator[vix - 1]
            )

    # get number of points per face based on area
    # NOTE: rounded up, so may be slightly more points than requested.
    points_per_face = [max(1, math.ceil(x * num_points)) for x in normalized_areas]

    # get face plane basis
    face_info = get_range3d_sample_planes(range3d)

    # one internal list of each face of the box:
    samples = []
    for _ in range(6):
        samples.append([])

    real_total_points = 0
    # print("Sampling Stats: ")
    # for each face, jittered sample of total area:
    for face_ix, f in enumerate(face_info):
        # get ratio of width/height in local space to plan jittering
        aspect_ratio = f[1].length() / f[2].length()
        num_wide = max(1, int(math.sqrt(aspect_ratio * points_per_face[face_ix])))
        num_high = max(1, int((points_per_face[face_ix] + num_wide - 1) / num_wide))
        total_points = num_wide * num_high
        real_total_points += total_points
        # print(f"    f_{face_ix}: ")
        # print(f"        points_per_face = {points_per_face[face_ix]}")
        # print(f"        aspect_ratio = {aspect_ratio}")
        # print(f"        num_wide = {num_wide}")
        # print(f"        num_high = {num_high}")
        # print(f"        total_points = {total_points}")

        # get jittered cell sizes
        dx = f[1] / num_wide
        dy = f[2] / num_high
        for x in range(num_wide):
            for y in range(num_high):
                # get cell origin
                org = f[0] + x * dx + y * dy
                # point is randomly placed in the cell
                point = org + random.random() * dx + random.random() * dy
                samples[face_ix].append(point)
    # print(f"        real_total_points = {real_total_points}")

    return samples


def sample_points_from_range3d(
    range3d: mn.Range3D, num_points: int = 100
) -> List[List[mn.Vector3]]:
    """
    Sample 'num_points' from the surface of a box defeined by 'range3d'.
    """

    # -----------------------------------------
    # area weighted face sampling
    normalized_area_accumulator = compute_area_weights_for_range3d_faces(range3d)

    def sample_face() -> int:
        """
        Weighted sampling of a face from the area accumulator.
        """
        rand = random.random()
        for ix in range(6):
            if normalized_area_accumulator[ix] > rand:
                return ix
        raise (AssertionError, "Should not reach here.")

    # -----------------------------------------

    face_info = get_range3d_sample_planes(range3d)

    # one internal list of each face of the box:
    samples = []
    for _ in range(6):
        samples.append([])

    # sample points for random faces
    for _ in range(num_points):
        face_ix = sample_face()
        f = face_info[face_ix]
        point = f[0] + random.random() * f[1] + random.random() * f[2]
        samples[face_ix].append(point)

    return samples


# End - Range3D surface sampling utils
# =======================================================================


def sample_points_from_sphere(
    center: mn.Vector3,
    radius: float,
    num_points: int = 100,
) -> List[List[mn.Vector3]]:
    """
    Sample num_points from a sphere defined by center and radius.
    Return all points in two identical lists to indicate pairwise raycasting.
    :param center: sphere center position
    :param radius: sphere radius
    :param num_points: number of points to sample
    """
    samples = []

    # sample points
    while len(samples) < num_points:
        # rejection sample unit sphere from volume
        rand_point = np.random.random(3) * 2.0 - np.ones(1)
        vec_len = np.linalg.norm(rand_point)
        if vec_len < 1.0:
            # inside the sphere, so project to the surface
            samples.append(mn.Vector3(rand_point / vec_len))
        # else outside the sphere, so rejected

    # move from unit sphere to input sphere
    samples = [x * radius + center for x in samples]

    # collect into pairwise datastructure
    samples = [samples, samples]

    return samples


def receptacle_density_sample(
    sim: habitat_sim.simulator.Simulator,
    receptacle: hab_receptacle.TriangleMeshReceptacle,
    target_radius: float = 0.04,
    max_points: int = 100,
    min_points: int = 5,
    max_tries: int = 200,
):
    target_point_area = math.pi * target_radius**2
    expected_points = receptacle.total_area / target_point_area

    # if necessary, compute new target_radius to best cover the area
    if expected_points > max_points or expected_points < min_points:
        expected_points = max(min_points, min(max_points, expected_points))
        target_radius = math.sqrt(receptacle.total_area / (expected_points * math.pi))

    # print(f"receptacle_density_sample(`{receptacle.name}`): area={receptacle.total_area}, r={target_radius}, num_p={expected_points}")

    sampled_points = []
    num_tries = 0
    min_dist = target_radius * 2
    while len(sampled_points) < expected_points and num_tries < max_tries:
        sample_point = receptacle.sample_uniform_global(sim, sample_region_scale=1.0)
        success = True
        for existing_point in sampled_points:
            if (sample_point - existing_point).length() < min_dist:
                num_tries += 1
                success = False
                break
        if success:
            # print(f"        success {sample_point} in {num_tries} tries")

            # if no rejection, add the point
            sampled_points.append(sample_point)
            num_tries = 0

    # print(f"    found {len(sampled_points)}/{expected_points} points.")

    return sampled_points, target_radius


def run_pairwise_raycasts(
    points: List[List[mn.Vector3]],
    sim: habitat_sim.Simulator,
    min_dist: float = 0.05,
    discard_invalid_results: bool = True,
) -> List[habitat_sim.physics.RaycastResults]:
    """
    Raycast between each pair of points from different surfaces.
    :param min_dist: The minimum ray distance to allow. Cull all candidate pairs closer than this distance.
    :param discard_invalid_results: If true, discard ray hit distances > 1
    """
    ray_max_local_dist = 100.0  # default
    if discard_invalid_results:
        # disallow contacts outside of the bounding volume (shouldn't happen anyway...)
        ray_max_local_dist = 1.0
    all_raycast_results: List[habitat_sim.physics.RaycastResults] = []
    print("Rays detected with invalid hit distance: ")
    for fix0 in range(len(points)):
        for fix1 in range(len(points)):
            if fix0 != fix1:  # no pairs on the same face
                for p0 in points[fix0]:
                    for p1 in points[fix1]:
                        if (p0 - p1).length() > min_dist:
                            # this is a valid pair of points
                            ray = habitat_sim.geo.Ray(p0, p1 - p0)  # origin, direction
                            # raycast
                            all_raycast_results.append(
                                sim.cast_ray(ray=ray, max_distance=ray_max_local_dist)
                            )
                            # reverse direction as separate entry (because exiting a convex does not generate a hit record)
                            ray2 = habitat_sim.geo.Ray(p1, p0 - p1)  # origin, direction
                            # raycast
                            all_raycast_results.append(
                                sim.cast_ray(ray=ray2, max_distance=ray_max_local_dist)
                            )

                            # prints invalid rays if not discarded by discard_invalid_results==True
                            for ix in [-1, -2]:
                                if all_raycast_results[ix].has_hits() and (
                                    all_raycast_results[ix].hits[0].ray_distance > 1
                                    or all_raycast_results[ix].hits[0].ray_distance < 0
                                ):
                                    print(
                                        f"     distance={all_raycast_results[ix].hits[0].ray_distance}"
                                    )

    return all_raycast_results


def debug_draw_raycast_results(
    sim, ground_truth_results, proxy_results, subsample_number: int = 100, seed=0
) -> None:
    """
    Render debug lines for a subset of raycast results, randomly subsampled for efficiency.
    """
    random.seed(seed)
    red = mn.Color4.red()
    yellow = mn.Color4.yellow()
    blue = mn.Color4.blue()
    grey = mn.Color4(mn.Vector3(0.6), 1.0)
    for _ in range(subsample_number):
        result_ix = random.randint(0, len(ground_truth_results) - 1)
        ray = ground_truth_results[result_ix].ray
        gt_results = ground_truth_results[result_ix]
        pr_results = proxy_results[result_ix]

        if gt_results.has_hits() or pr_results.has_hits():
            # some logic for line colors
            first_hit_dist = 0
            # pairs of distances for overshooting the ground truth and undershooting the ground truth
            overshoot_dists = []
            undershoot_dists = []

            # draw first hits for gt and proxy
            if gt_results.has_hits():
                sim.get_debug_line_render().draw_circle(
                    translation=ray.origin
                    + ray.direction * gt_results.hits[0].ray_distance,
                    radius=0.005,
                    color=blue,
                    normal=gt_results.hits[0].normal,
                )
            if pr_results.has_hits():
                sim.get_debug_line_render().draw_circle(
                    translation=ray.origin
                    + ray.direction * pr_results.hits[0].ray_distance,
                    radius=0.005,
                    color=yellow,
                    normal=pr_results.hits[0].normal,
                )

            if not gt_results.has_hits():
                first_hit_dist = pr_results.hits[0].ray_distance
                overshoot_dists.append((first_hit_dist, 1.0))
            elif not pr_results.has_hits():
                first_hit_dist = gt_results.hits[0].ray_distance
                undershoot_dists.append((first_hit_dist, 1.0))
            else:
                # both have hits
                first_hit_dist = min(
                    gt_results.hits[0].ray_distance, pr_results.hits[0].ray_distance
                )

                # compute overshoots and undershoots for first hit:
                if gt_results.hits[0].ray_distance < pr_results.hits[0].ray_distance:
                    # undershoot
                    undershoot_dists.append(
                        (
                            gt_results.hits[0].ray_distance,
                            pr_results.hits[0].ray_distance,
                        )
                    )
                else:
                    # overshoot
                    overshoot_dists.append(
                        (
                            gt_results.hits[0].ray_distance,
                            pr_results.hits[0].ray_distance,
                        )
                    )

            # draw blue lines for overlapping distances
            sim.get_debug_line_render().draw_transformed_line(
                ray.origin, ray.origin + ray.direction * first_hit_dist, blue
            )

            # draw red lines for overshoots (proxy is outside the ground truth)
            for d0, d1 in overshoot_dists:
                sim.get_debug_line_render().draw_transformed_line(
                    ray.origin + ray.direction * d0,
                    ray.origin + ray.direction * d1,
                    red,
                )

            # draw yellow lines for undershoots (proxy is inside the ground truth)
            for d0, d1 in undershoot_dists:
                sim.get_debug_line_render().draw_transformed_line(
                    ray.origin + ray.direction * d0,
                    ray.origin + ray.direction * d1,
                    yellow,
                )

        else:
            # no hits, grey line
            sim.get_debug_line_render().draw_transformed_line(
                ray.origin, ray.origin + ray.direction, grey
            )


def get_raycast_results_cumulative_error_metric(
    ground_truth_results, proxy_results
) -> float:
    """
    Generates a scalar error metric from raycast results normalized to [0,1].

    absolute_error = sum(abs(gt_1st_hit_dist-pr_1st_hit_dist))

    To normalize error:
        0 corresponds to gt distances (absolute_error == 0)
        1 corresponds to max error. For each ray, max error is max(gt_1st_hit_dist, ray_length-gt_1st_hit_dist).
            max_error = sum(max(gt_1st_hit_dist, ray_length-gt_1st_hit_dist))
    normalized_error = error/max_error
    """
    assert len(ground_truth_results) == len(
        proxy_results
    ), "raycast results must be equivalent."

    max_error = 0
    absolute_error = 0
    for r_ix in range(len(ground_truth_results)):
        ray = ground_truth_results[r_ix].ray
        ray_len = ray.direction.length()
        local_max_error = ray_len
        gt_dist = ray_len
        if ground_truth_results[r_ix].has_hits():
            gt_dist = ground_truth_results[r_ix].hits[0].ray_distance * ray_len
            local_max_error = max(gt_dist, ray_len - gt_dist)
        max_error += local_max_error
        local_proxy_dist = ray_len
        if proxy_results[r_ix].has_hits():
            local_proxy_dist = proxy_results[r_ix].hits[0].ray_distance * ray_len
        local_absolute_error = abs(local_proxy_dist - gt_dist)
        absolute_error += local_absolute_error

    normalized_error = absolute_error / max_error
    return normalized_error


# ===================================================================
# CollisionProxyOptimizer class provides a stateful API for
# configurable evaluation and optimization of collision proxy shapes.
# ===================================================================


class CollisionProxyOptimizer:
    """
    Stateful control flow for using Habitat-sim to evaluate and optimize collision proxy shapes.
    """

    def __init__(
        self,
        sim_settings: Dict[str, Any],
        output_directory: Optional[str] = None,
        mm: Optional[habitat_sim.metadata.MetadataMediator] = None,
    ) -> None:
        # load the dataset into a persistent, shared MetadataMediator instance.
        self.mm = mm if mm is not None else habitat_sim.metadata.MetadataMediator()
        self.mm.active_dataset = sim_settings["scene_dataset_config_file"]
        self.sim_settings = sim_settings.copy()

        # path to the desired output directory for images/csv
        self.output_directory = output_directory
        if output_directory is not None:
            os.makedirs(self.output_directory, exist_ok=True)

        # if true, render and save debug images in self.output_directory
        self.generate_debug_images = False

        # option to use Receptacle annotations to compute an additional accuracy metric
        self.compute_receptacle_useability_metrics = True
        # add a vertical epsilon offset to the receptacle points for analysis. This is added directly to the sampled points.
        self.rec_point_vertical_offset = 0.041

        self.init_caches()

    def init_caches(self):
        """
        Re-initialize all internal data caches to prepare for re-use.
        """
        # cache of test points, rays, distances, etc... for use by active processes
        # NOTE: entries created by `setup_obj_gt` and cleaned by `clean_obj_gt` for memory efficiency.
        self.gt_data: Dict[str, Dict[str, Any]] = {}

        # cache global results to be written to csv.
        self.results: Dict[str, Dict[str, Any]] = {}

    def get_proxy_index(self, obj_handle: str) -> int:
        """
        Get the current proxy index for an object.
        """
        return self.gt_data[obj_handle]["proxy_index"]

    def increment_proxy_index(self, obj_handle: str) -> int:
        """
        Increment the current proxy index.
        Only do this after all processing for the current proxy is complete.
        """
        self.gt_data[obj_handle]["proxy_index"] += 1

    def get_proxy_shape_id(self, obj_handle: str) -> str:
        """
        Get a string representation of the current proxy shape.
        """
        return f"pr{self.get_proxy_index(obj_handle)}"

    def get_cfg_with_mm(
        self, scene: str = "NONE"
    ) -> habitat_sim.simulator.Configuration:
        """
        Get a Configuration object for initializing habitat_sim Simulator object with the correct dataset and MetadataMediator passed along.

        :param scene: The desired scene entry, defaulting to the empty NONE scene.
        """
        sim_settings = self.sim_settings.copy()
        sim_settings["scene_dataset_config_file"] = self.mm.active_dataset
        sim_settings["scene"] = scene
        cfg = make_cfg(sim_settings)
        cfg.metadata_mediator = self.mm
        return cfg

    def setup_obj_gt(
        self,
        obj_handle: str,
        sample_shape: str = "jittered_aabb",
        num_point_samples=100,
    ) -> None:
        """
        Prepare the ground truth and sample point sets for an object.
        """
        assert (
            obj_handle not in self.gt_data
        ), f"`{obj_handle}` already setup in gt_data: {self.gt_data.keys()}"

        # find object
        otm = self.mm.object_template_manager
        obj_template = otm.get_template_by_handle(obj_handle)
        assert obj_template is not None, f"Could not find object handle `{obj_handle}`"

        # create a stage template with the object's render mesh as a "ground truth" for metrics
        stm = self.mm.stage_template_manager
        stage_template_name = obj_handle + "_as_stage"
        new_stage_template = stm.create_new_template(handle=stage_template_name)
        new_stage_template.render_asset_handle = obj_template.render_asset_handle
        new_stage_template.orient_up = obj_template.orient_up
        new_stage_template.orient_front = obj_template.orient_front
        stm.register_template(
            template=new_stage_template, specified_handle=stage_template_name
        )

        # initialize the object's runtime data cache
        self.gt_data[obj_handle] = {
            "proxy_index": 0,  # used to recover and increment `shape_id` during optimization and evaluation
            "stage_template_name": stage_template_name,
            "receptacles": {},  # sub-cache for receptacle metric data and results
            "raycasts": {},  # subcache for shape raycasting metric data
            "shape_test_results": {
                "gt": {}
            },  # subcache for shape and physics metric results
        }

        # correct now for any COM automation
        obj_template.compute_COM_from_shape = False
        obj_template.com = mn.Vector3(0)
        otm.register_template(obj_template)

        if self.compute_receptacle_useability_metrics or self.generate_debug_images:
            # pre-process the ground truth object and receptacles
            rec_vertical_offset = mn.Vector3(0, self.rec_point_vertical_offset, 0)
            cfg = self.get_cfg_with_mm()
            with habitat_sim.Simulator(cfg) as sim:
                # load the gt object
                rom = sim.get_rigid_object_manager()
                obj = rom.add_object_by_template_handle(obj_handle)
                assert obj.is_alive, "Object was not added correctly."

                if self.compute_receptacle_useability_metrics:
                    # get receptacles defined for the object:
                    source_template_file = obj.creation_attributes.file_directory
                    user_attr = obj.user_attributes
                    obj_receptacles = hab_receptacle.parse_receptacles_from_user_config(
                        user_attr,
                        parent_object_handle=obj.handle,
                        parent_template_directory=source_template_file,
                    )

                    # sample test points on the receptacles
                    for receptacle in obj_receptacles:
                        if type(receptacle) == hab_receptacle.TriangleMeshReceptacle:
                            rec_test_points = []
                            t_radius = 0.01
                            # adaptive density sample:
                            rec_test_points, t_radius = receptacle_density_sample(
                                sim, receptacle
                            )
                            # add the vertical offset
                            rec_test_points = [
                                p + rec_vertical_offset for p in rec_test_points
                            ]

                            # random sample:
                            # for _ in range(num_point_samples):
                            #    rec_test_points.append(
                            #        receptacle.sample_uniform_global(
                            #            sim, sample_region_scale=1.0
                            #        )
                            #    )
                            self.gt_data[obj_handle]["receptacles"][receptacle.name] = {
                                "sample_points": rec_test_points,
                                "shape_id_results": {},
                            }
                            if self.generate_debug_images:
                                debug_lines = []
                                for face in range(
                                    int(len(receptacle.mesh_data.indices) / 3)
                                ):
                                    verts = receptacle.get_face_verts(f_ix=face)
                                    for edge in range(3):
                                        debug_lines.append(
                                            (
                                                [verts[edge], verts[(edge + 1) % 3]],
                                                mn.Color4.green(),
                                            )
                                        )
                                debug_circles = []
                                for p in rec_test_points:
                                    debug_circles.append(
                                        (
                                            (
                                                p,  # center
                                                t_radius,  # radius
                                                mn.Vector3(0, 1, 0),  # normal
                                                mn.Color4.red(),  # color
                                            )
                                        )
                                    )
                                if (
                                    self.generate_debug_images
                                    and self.output_directory is not None
                                ):
                                    # use DebugVisualizer to get 6-axis view of the object
                                    dvb = hab_debug_vis.DebugVisualizer(
                                        sim=sim,
                                        output_path=self.output_directory,
                                        default_sensor_uuid="color_sensor",
                                    )
                                    dvb.peek_rigid_object(
                                        obj,
                                        peek_all_axis=True,
                                        additional_savefile_prefix=f"{receptacle.name}_",
                                        debug_lines=debug_lines,
                                        debug_circles=debug_circles,
                                    )

                if self.generate_debug_images and self.output_directory is not None:
                    # use DebugVisualizer to get 6-axis view of the object
                    dvb = hab_debug_vis.DebugVisualizer(
                        sim=sim,
                        output_path=self.output_directory,
                        default_sensor_uuid="color_sensor",
                    )
                    dvb.peek_rigid_object(
                        obj, peek_all_axis=True, additional_savefile_prefix="gt_"
                    )

        # load a simulator instance with this object as the stage
        cfg = self.get_cfg_with_mm(scene=stage_template_name)
        with habitat_sim.Simulator(cfg) as sim:
            # get test points from bounding box info:
            scene_bb = sim.get_active_scene_graph().get_root_node().cumulative_bb
            inflated_scene_bb = scene_bb.scaled(mn.Vector3(1.25))
            inflated_scene_bb = mn.Range3D.from_center(
                scene_bb.center(), inflated_scene_bb.size() / 2.0
            )
            # NOTE: to save the referenced Range3D object, we need to deep or Magnum will destroy the underlying C++ objects.
            self.gt_data[obj_handle]["scene_bb"] = mn.Range3D(
                scene_bb.min, scene_bb.max
            )
            self.gt_data[obj_handle]["inflated_scene_bb"] = inflated_scene_bb
            test_points = None
            if sample_shape == "aabb":
                # bounding box sample
                test_points = sample_points_from_range3d(
                    range3d=inflated_scene_bb, num_points=num_point_samples
                )
            elif sample_shape == "jittered_aabb":
                # bounding box sample
                test_points = sample_jittered_points_from_range3d(
                    range3d=inflated_scene_bb, num_points=num_point_samples
                )
            elif sample_shape == "sphere":
                # bounding sphere sample
                half_diagonal = (scene_bb.max - scene_bb.min).length() / 2.0
                test_points = sample_points_from_sphere(
                    center=inflated_scene_bb.center(),
                    radius=half_diagonal,
                    num_points=num_point_samples,
                )
            else:
                raise NotImplementedError(
                    f"sample_shape == `{sample_shape}` is not implemented. Use `sphere` or `aabb`."
                )
            self.gt_data[obj_handle]["test_points"] = test_points

            # compute and cache "ground truth" raycast on object as stage
            gt_raycast_results = run_pairwise_raycasts(test_points, sim)
            self.gt_data[obj_handle]["raycasts"]["gt"] = gt_raycast_results

    def clean_obj_gt(self, obj_handle: str) -> None:
        """
        Cleans the global object cache to better manage process memory.
        Call this to clean-up after global data are written and detailed sample data are no longer necessary.
        """
        assert (
            obj_handle in self.gt_data
        ), f"`{obj_handle}` does not have any entry in gt_data: {self.gt_data.keys()}. Call to `setup_obj_gt(obj_handle)` required."
        self.gt_data.pop(obj_handle)

    def compute_baseline_metrics(self, obj_handle: str) -> None:
        """
        Computes 2 baselines for the evaluation metric and caches the results:
        1. No collision object
        2. AABB collision object
        """
        assert (
            obj_handle in self.gt_data
        ), f"`{obj_handle}` does not have any entry in gt_data: {self.gt_data.keys()}. Call to `setup_obj_gt(obj_handle)` required."

        # start with empty scene
        cfg = self.get_cfg_with_mm()
        with habitat_sim.Simulator(cfg) as sim:
            empty_raycast_results = run_pairwise_raycasts(
                self.gt_data[obj_handle]["test_points"], sim
            )
            self.gt_data[obj_handle]["raycasts"]["empty"] = empty_raycast_results

        cfg = self.get_cfg_with_mm()
        with habitat_sim.Simulator(cfg) as sim:
            # modify the template
            obj_template = sim.get_object_template_manager().get_template_by_handle(
                obj_handle
            )
            assert (
                obj_template is not None
            ), f"Could not find object handle `{obj_handle}`"
            # bounding box as collision object
            obj_template.bounding_box_collisions = True
            sim.get_object_template_manager().register_template(obj_template)

            # load the object
            sim.get_rigid_object_manager().add_object_by_template_handle(obj_handle)

            # run evaluation
            bb_raycast_results = run_pairwise_raycasts(
                self.gt_data[obj_handle]["test_points"], sim
            )
            self.gt_data[obj_handle]["raycasts"]["bb"] = bb_raycast_results

            # un-modify the template
            obj_template.bounding_box_collisions = False
            sim.get_object_template_manager().register_template(obj_template)

    def compute_proxy_metrics(self, obj_handle: str) -> None:
        """
        Computes the evaluation metric on the currently configred proxy shape and caches the results.
        """
        assert (
            obj_handle in self.gt_data
        ), f"`{obj_handle}` does not have any entry in gt_data: {self.gt_data.keys()}. Call to `setup_obj_gt(obj_handle)` required."

        # when evaluating multiple proxy shapes, need unique ids:
        pr_id = self.get_proxy_shape_id(obj_handle)

        # start with empty scene
        cfg = self.get_cfg_with_mm()
        with habitat_sim.Simulator(cfg) as sim:
            # modify the template to render collision object
            otm = self.mm.object_template_manager
            obj_template = otm.get_template_by_handle(obj_handle)
            render_asset = obj_template.render_asset_handle
            obj_template.render_asset_handle = obj_template.collision_asset_handle
            otm.register_template(obj_template)

            # load the object
            obj = sim.get_rigid_object_manager().add_object_by_template_handle(
                obj_handle
            )
            assert obj.is_alive, "Object was not added correctly."

            # check that collision shape bounding box is similar
            col_bb = obj.root_scene_node.cumulative_bb
            assert self.gt_data[obj_handle]["inflated_scene_bb"].contains(
                col_bb.min
            ) and self.gt_data[obj_handle]["inflated_scene_bb"].contains(
                col_bb.max
            ), f"Inflated bounding box does not contain the collision shape. (Object `{obj_handle}`)"

            if self.generate_debug_images and self.output_directory is not None:
                # use DebugVisualizer to get 6-axis view of the object
                dvb = hab_debug_vis.DebugVisualizer(
                    sim=sim,
                    output_path=self.output_directory,
                    default_sensor_uuid="color_sensor",
                )
                dvb.peek_rigid_object(
                    obj, peek_all_axis=True, additional_savefile_prefix=pr_id + "_"
                )

            # run evaluation
            pr_raycast_results = run_pairwise_raycasts(
                self.gt_data[obj_handle]["test_points"], sim
            )
            self.gt_data[obj_handle]["raycasts"][pr_id] = pr_raycast_results

            # undo template modification
            obj_template.render_asset_handle = render_asset
            otm.register_template(obj_template)

    def compute_receptacle_access_metrics(
        self, obj_handle: str, use_gt=False, acces_ratio_threshold: float = 0.1
    ):
        """
        Compute a heuristic for the accessibility of all Receptacles for an object.
        Uses raycasting from previously sampled receptacle locations to approximate how open a particular receptacle is.
        :param use_gt: Compute the metric for the ground truth shape instead of the currently active collision proxy (default)
        :param acces_ratio_threshold: The ratio of accessible:blocked rays necessary for a Receptacle point to be considered accessible
        """
        # algorithm:
        # For each receptacle, r:
        #  For each sample point, s:
        #    Generate `num_point_rays` directions, d (length bb diagnonal) and Ray(origin=s+d, direction=d)
        #    For each ray:
        #      If dist > 1, success, otherwise failure
        #
        # metrics:
        # - %rays
        # - %points w/ success% > eps(10%) #call these successful/accessible
        # - average % for points
        # ? how to get regions?
        # ? debug draw this metric?
        # ? how to diff b/t gt and pr?

        print(f"compute_receptacle_access_metrics - obj_handle = {obj_handle}")

        # start with empty scene or stage as scene:
        scene_name = "NONE"
        if use_gt:
            scene_name = self.gt_data[obj_handle]["stage_template_name"]
        cfg = self.get_cfg_with_mm(scene=scene_name)
        with habitat_sim.Simulator(cfg) as sim:
            obj_rec_data = self.gt_data[obj_handle]["receptacles"]
            shape_id = "gt"
            obj = None
            if not use_gt:
                # load the object
                obj = sim.get_rigid_object_manager().add_object_by_template_handle(
                    obj_handle
                )
                assert obj.is_alive, "Object was not added correctly."

                # when evaluating multiple proxy shapes, need unique ids:
                shape_id = self.get_proxy_shape_id(obj_handle)

            # gather hemisphere rays scaled to object's size
            # NOTE: because the receptacle points can be located anywhere in the bounding box, raycast radius must be bb diagonal length
            ray_sphere_radius = self.gt_data[obj_handle]["scene_bb"].size().length()
            assert ray_sphere_radius > 0, "otherwise we have an error"
            ray_sphere_points = get_scaled_hemisphere_vectors(ray_sphere_radius)

            # save a list of point accessibility scores for debugging and visualization
            receptacle_point_access_scores = {}
            dvb: Optional[hab_debug_vis.DebugVisualizer] = None
            if self.output_directory is not None:
                dvb = hab_debug_vis.DebugVisualizer(
                    sim=sim,
                    output_path=self.output_directory,
                    default_sensor_uuid="color_sensor",
                )

            # collect hemisphere raycast samples for all receptacle sample points
            for receptacle_name in obj_rec_data:
                sample_point_ray_results: List[
                    List[habitat_sim.physics.RaycastResults]
                ] = []
                sample_point_access_ratios: List[float] = []
                # access rate is percent of "accessible" points apssing the threshold
                receptacle_access_rate = 0
                # access score is average accessibility of points
                receptacle_access_score = 0
                sample_points = obj_rec_data[receptacle_name]["sample_points"]
                for sample_point in sample_points:
                    # NOTE: rays must originate outside the shape because origins inside a convex will not collide.
                    # move ray origins to new point location
                    hemi_rays = [
                        habitat_sim.geo.Ray(v + sample_point, -v)
                        for v in ray_sphere_points
                    ]
                    # rays are not unit length, so use local max_distance==1 ray length
                    ray_results = [
                        sim.cast_ray(ray=ray, max_distance=1.0) for ray in hemi_rays
                    ]
                    sample_point_ray_results.append(ray_results)

                    # compute per-point access metrics
                    blocked_rays = len([rr for rr in ray_results if rr.has_hits()])
                    sample_point_access_ratios.append(
                        (len(ray_results) - blocked_rays) / len(ray_results)
                    )
                    receptacle_access_score += sample_point_access_ratios[-1]
                    if sample_point_access_ratios[-1] > acces_ratio_threshold:
                        receptacle_access_rate += 1
                    receptacle_point_access_scores[
                        receptacle_name
                    ] = sample_point_access_ratios

                receptacle_access_score /= len(sample_points)
                receptacle_access_rate /= len(sample_points)

                if shape_id not in obj_rec_data[receptacle_name]["shape_id_results"]:
                    obj_rec_data[receptacle_name]["shape_id_results"][shape_id] = {}
                assert (
                    "access_results"
                    not in obj_rec_data[receptacle_name]["shape_id_results"][shape_id]
                ), f"Overwriting existing 'access_results' data for '{receptacle_name}'|'{shape_id}'."
                obj_rec_data[receptacle_name]["shape_id_results"][shape_id][
                    "access_results"
                ] = {
                    "receptacle_point_access_scores": receptacle_point_access_scores[
                        receptacle_name
                    ],
                    "sample_point_ray_results": sample_point_ray_results,
                    "receptacle_access_score": receptacle_access_score,
                    "receptacle_access_rate": receptacle_access_rate,
                }
                access_results = obj_rec_data[receptacle_name]["shape_id_results"][
                    shape_id
                ]["access_results"]

                print(f" receptacle_name = {receptacle_name}")
                print(f" receptacle_access_score = {receptacle_access_score}")
                print(f" receptacle_access_rate = {receptacle_access_rate}")

                if self.generate_debug_images and dvb is not None:
                    # generate receptacle access debug images
                    # 1a Show missed rays vs 1b hit rays
                    debug_lines = []
                    for ray_results in access_results["sample_point_ray_results"]:
                        for hit_record in ray_results:
                            if not hit_record.has_hits():
                                debug_lines.append(
                                    (
                                        [
                                            hit_record.ray.origin,
                                            hit_record.ray.origin
                                            + hit_record.ray.direction,
                                        ],
                                        mn.Color4.green(),
                                    )
                                )
                    if use_gt:
                        dvb.peek_scene(
                            peek_all_axis=True,
                            additional_savefile_prefix=f"gt_{receptacle_name}_access_rays_",
                            debug_lines=debug_lines,
                            debug_circles=None,
                        )
                    else:
                        dvb.peek_rigid_object(
                            obj,
                            peek_all_axis=True,
                            additional_savefile_prefix=f"{shape_id}_{receptacle_name}_access_rays_",
                            debug_lines=debug_lines,
                            debug_circles=None,
                        )

                    # 2 Show only rec points colored by "access" metric or percentage
                    debug_circles = []
                    color_r = mn.Color4.red().to_xyz()
                    color_g = mn.Color4.green().to_xyz()
                    delta = color_g - color_r
                    for point_access_ratio, point in zip(
                        receptacle_point_access_scores[receptacle_name],
                        obj_rec_data[receptacle_name]["sample_points"],
                    ):
                        point_color_xyz = color_r + delta * point_access_ratio
                        debug_circles.append(
                            (
                                point,
                                0.02,
                                mn.Vector3(0, 1, 0),
                                mn.Color4.from_xyz(point_color_xyz),
                            )
                        )
                    # use DebugVisualizer to get 6-axis view of the object
                    if use_gt:
                        dvb.peek_scene(
                            peek_all_axis=True,
                            additional_savefile_prefix=f"gt_{receptacle_name}_point_ratios_",
                            debug_lines=None,
                            debug_circles=debug_circles,
                        )
                    else:
                        dvb.peek_rigid_object(
                            obj,
                            peek_all_axis=True,
                            additional_savefile_prefix=f"{shape_id}_{receptacle_name}_point_ratios_",
                            debug_lines=None,
                            debug_circles=debug_circles,
                        )
                    # obj_rec_data[receptacle_name]["results"][shape_id]["sample_point_ray_results"]

    def construct_cylinder_object(
        self,
        mm: habitat_sim.metadata.MetadataMediator,
        cyl_radius: float = 0.04,
        cyl_height: float = 0.15,
    ):
        constructed_cyl_temp_name = "scaled_cyl_template"
        otm = mm.object_template_manager
        cyl_temp_handle = otm.get_synth_template_handles("cylinder")[0]
        cyl_temp = otm.get_template_by_handle(cyl_temp_handle)
        cyl_temp.scale = mn.Vector3(cyl_radius, cyl_height / 2.0, cyl_radius)
        otm.register_template(cyl_temp, constructed_cyl_temp_name)
        return constructed_cyl_temp_name

    def compute_receptacle_stability(
        self,
        obj_handle: str,
        use_gt: bool = False,
        cyl_radius: float = 0.04,
        cyl_height: float = 0.15,
        accepted_height_error: float = 0.1,
    ):
        """
        Try to place a dynamic cylinder on the receptacle points. Record snap error and physical stability.

        :param obj_handle: The object to evaluate.
        :param use_gt: Compute the metric for the ground truth shape instead of the currently active collision proxy (default)
        :param cyl_radius: Radius of the test cylinder object (default similar to food can)
        :param cyl_height: Height of the test cylinder object (default similar to food can)
        :param accepted_height_error: The acceptacle distance from receptacle to snapped point considered successful (meters)
        """

        constructed_cyl_obj_handle = self.construct_cylinder_object(
            self.mm, cyl_radius, cyl_height
        )

        assert (
            len(self.gt_data[obj_handle]["receptacles"].keys()) > 0
        ), "Object must have receptacle sampling metadata defined. See `setup_obj_gt`"

        # start with empty scene or stage as scene:
        scene_name = "NONE"
        if use_gt:
            scene_name = self.gt_data[obj_handle]["stage_template_name"]
        cfg = self.get_cfg_with_mm(scene=scene_name)
        with habitat_sim.Simulator(cfg) as sim:
            dvb: Optional[hab_debug_vis.DebugVisualizer] = None
            if self.generate_debug_images and self.output_directory is not None:
                dvb = hab_debug_vis.DebugVisualizer(
                    sim=sim,
                    output_path=self.output_directory,
                    default_sensor_uuid="color_sensor",
                )
            # load the object
            rom = sim.get_rigid_object_manager()
            obj = None
            support_obj_ids = [-1]
            shape_id = "gt"
            if not use_gt:
                obj = rom.add_object_by_template_handle(obj_handle)
                support_obj_ids = [obj.object_id]
                assert obj.is_alive, "Object was not added correctly."
                # need to make the object STATIC so it doesn't move
                obj.motion_type = habitat_sim.physics.MotionType.STATIC
                # when evaluating multiple proxy shapes, need unique ids:
                shape_id = self.get_proxy_shape_id(obj_handle)

            # add the test object
            cyl_test_obj = rom.add_object_by_template_handle(constructed_cyl_obj_handle)
            cyl_test_obj_com_height = cyl_test_obj.root_scene_node.cumulative_bb.max[1]
            assert cyl_test_obj.is_alive, "Test object was not added correctly."

            # we sample above the receptacle to account for margin, but we compare distance to the actual receptacle height
            receptacle_sample_height_correction = mn.Vector3(
                0, -self.rec_point_vertical_offset, 0
            )

            # evaluation the sample points for each receptacle
            rec_data = self.gt_data[obj_handle]["receptacles"]
            for rec_name in rec_data:
                sample_points = rec_data[rec_name]["sample_points"]

                failed_snap = 0
                failed_by_distance = 0
                failed_unstable = 0
                point_stabilities = []
                for sample_point in sample_points:
                    cyl_test_obj.translation = sample_point
                    cyl_test_obj.rotation = mn.Quaternion.identity_init()
                    # snap check
                    success = snap_down(
                        sim, cyl_test_obj, support_obj_ids=support_obj_ids, vdb=dvb
                    )
                    if success:
                        expected_height_error = abs(
                            (
                                cyl_test_obj.translation
                                - (sample_point + receptacle_sample_height_correction)
                            ).length()
                            - cyl_test_obj_com_height
                        )
                        if expected_height_error > accepted_height_error:
                            failed_by_distance += 1
                            point_stabilities.append(False)
                            continue

                        # physical stability analysis
                        snap_position = cyl_test_obj.translation
                        identity_q = mn.Quaternion.identity_init()
                        displacement_limit = 0.04  # meters
                        rotation_limit = mn.Rad(0.1)  # radians
                        max_sim_time = 3.0
                        dt = 0.5
                        start_time = sim.get_world_time()
                        object_is_stable = True
                        while sim.get_world_time() - start_time < max_sim_time:
                            sim.step_world(dt)
                            linear_displacement = (
                                cyl_test_obj.translation - snap_position
                            ).length()
                            # NOTE: negative quaternion represents the same rotation, but gets a different angle error so check both
                            angular_displacement = min(
                                mn.math.half_angle(cyl_test_obj.rotation, identity_q),
                                mn.math.half_angle(
                                    -1 * cyl_test_obj.rotation, identity_q
                                ),
                            )
                            if (
                                angular_displacement > rotation_limit
                                or linear_displacement > displacement_limit
                            ):
                                object_is_stable = False
                                break
                            if not cyl_test_obj.awake:
                                # the object has settled, no need to continue simulating
                                break
                        # NOTE: we assume that if the object has not moved past the threshold in 'max_sim_time', then it must be stabel enough
                        if not object_is_stable:
                            failed_unstable += 1
                            point_stabilities.append(False)
                        else:
                            point_stabilities.append(True)
                    else:
                        failed_snap += 1
                        point_stabilities.append(False)

                successful_points = (
                    len(sample_points)
                    - failed_snap
                    - failed_by_distance
                    - failed_unstable
                )
                success_ratio = successful_points / len(sample_points)
                print(
                    f"{shape_id}: receptacle '{rec_name}' success_ratio = {success_ratio}"
                )
                print(
                    f"     failed_snap = {failed_snap}|failed_by_distance = {failed_by_distance}|failed_unstable={failed_unstable}|total={len(sample_points)}"
                )
                # TODO: visualize this error

                # write results to cache
                if shape_id not in rec_data[rec_name]["shape_id_results"]:
                    rec_data[rec_name]["shape_id_results"][shape_id] = {}
                assert (
                    "stability_results"
                    not in rec_data[rec_name]["shape_id_results"][shape_id]
                ), f"Overwriting existing 'stability_results' data for '{rec_name}'|'{shape_id}'."
                rec_data[rec_name]["shape_id_results"][shape_id][
                    "stability_results"
                ] = {
                    "success_ratio": success_ratio,
                    "failed_snap": failed_snap,
                    "failed_by_distance": failed_by_distance,
                    "failed_unstable": failed_unstable,
                    "total": len(sample_points),
                    "point_stabilities": point_stabilities,
                }

    def setup_shape_test_results_cache(self, obj_handle: str, shape_id: str) -> None:
        """
        Ensure the 'shape_test_results' sub-cache is initialized for a 'shape_id'.
        """
        if shape_id not in self.gt_data[obj_handle]["shape_test_results"]:
            self.gt_data[obj_handle]["shape_test_results"][shape_id] = {
                "settle_report": {},
                "sphere_shake_report": {},
                "collision_grid_report": {},
            }

    def run_physics_settle_test(self, obj_handle):
        """
        Drops the object on a plane and waits for it to sleep.
        Provides a heuristic measure of dynamic stability. If the object jitters, bounces, or oscillates it won't sleep.
        """

        cfg = self.get_cfg_with_mm()
        with habitat_sim.Simulator(cfg) as sim:
            rom = sim.get_rigid_object_manager()
            obj = rom.add_object_by_template_handle(obj_handle)
            assert obj.is_alive, "Object was not added correctly."

            # when evaluating multiple proxy shapes, need unique ids:
            shape_id = self.get_proxy_shape_id(obj_handle)
            self.setup_shape_test_results_cache(obj_handle, shape_id)

            # add a plane
            otm = sim.get_object_template_manager()
            cube_plane_handle = "cubePlaneSolid"
            if not otm.get_library_has_handle(cube_plane_handle):
                cube_prim_handle = otm.get_template_handles("cubeSolid")[0]
                cube_template = otm.get_template_by_handle(cube_prim_handle)
                cube_template.scale = mn.Vector3(20, 0.05, 20)
                otm.register_template(cube_template, cube_plane_handle)
                assert otm.get_library_has_handle(cube_plane_handle)
            plane_obj = rom.add_object_by_template_handle(cube_plane_handle)
            assert plane_obj.is_alive, "Plane object was not added correctly."
            plane_obj.motion_type = habitat_sim.physics.MotionType.STATIC

            # use DebugVisualizer to get 6-axis view of the object
            dvb: Optional[hab_debug_vis.DebugVisualizer] = None
            if self.generate_debug_images and self.output_directory is not None:
                dvb = hab_debug_vis.DebugVisualizer(
                    sim=sim,
                    output_path=self.output_directory,
                    default_sensor_uuid="color_sensor",
                )
                dvb.peek_rigid_object(
                    obj,
                    peek_all_axis=True,
                    additional_savefile_prefix=f"plane_snap_{shape_id}_",
                )

            # snap the object to the plane
            obj_col_bb = obj.collision_shape_aabb
            obj.translation = mn.Vector3(0, obj_col_bb.max[1] - obj_col_bb.min[1], 0)
            success = snap_down(sim, obj, support_obj_ids=[plane_obj.object_id])

            if not success:
                print("Failed to snap object to plane...")
                self.gt_data[obj_handle]["shape_test_results"][shape_id][
                    "settle_report"
                ] = {
                    "success": False,
                    "realtime": "NA",
                    "max_time": "NA",
                    "settle_time": "NA",
                }
                return

            # simulate for settling
            max_sim_time = 5.0
            dt = 0.25
            real_start_time = time.time()
            object_is_stable = False
            start_time = sim.get_world_time()
            while sim.get_world_time() - start_time < max_sim_time:
                sim.step_world(dt)
                # dvb.peek_rigid_object(
                #    obj,
                #    peek_all_axis=True,
                #    additional_savefile_prefix=f"plane_snap_{sim.get_world_time() - start_time}_",
                # )

                if not obj.awake:
                    object_is_stable = True
                    # the object has settled, no need to continue simulating
                    break
            real_test_time = time.time() - real_start_time
            sim_settle_time = sim.get_world_time() - start_time
            print(f"Physics Settle Time Report: '{obj_handle}'")
            if object_is_stable:
                print(f"    Settled in {sim_settle_time} sim seconds.")
            else:
                print(f"    Failed to settle in {max_sim_time} sim seconds.")
            print(f"    Test completed in {real_test_time} seconds.")

            self.gt_data[obj_handle]["shape_test_results"][shape_id][
                "settle_report"
            ] = {
                "success": object_is_stable,
                "realtime": real_test_time,
                "max_time": max_sim_time,
                "settle_time": sim_settle_time,
            }

    def compute_grid_collision_times(self, obj_handle, subdivisions=0, use_gt=False):
        """
        Runs a collision test over a subdivided grid of box shapes within the object's AABB.
        Measures discrete collision check efficiency.

        "param subdivisions": number of recursive subdivisions to create the grid. E.g. 0 is the bb, 1 is 8 box of 1/2 bb size, etc...
        """

        scene_name = "NONE"
        if use_gt:
            scene_name = self.gt_data[obj_handle]["stage_template_name"]
        cfg = self.get_cfg_with_mm(scene=scene_name)
        with habitat_sim.Simulator(cfg) as sim:
            rom = sim.get_rigid_object_manager()
            shape_id = "gt"
            shape_bb = None
            if not use_gt:
                obj = rom.add_object_by_template_handle(obj_handle)
                assert obj.is_alive, "Object was not added correctly."
                # need to make the object STATIC so it doesn't move
                obj.motion_type = habitat_sim.physics.MotionType.STATIC
                # when evaluating multiple proxy shapes, need unique ids:
                shape_id = self.get_proxy_shape_id(obj_handle)
                shape_bb = obj.root_scene_node.cumulative_bb
            else:
                shape_bb = sim.get_active_scene_graph().get_root_node().cumulative_bb

            self.setup_shape_test_results_cache(obj_handle, shape_id)

            # add the collision box
            otm = sim.get_object_template_manager()
            cube_prim_handle = otm.get_template_handles("cubeSolid")[0]
            cube_template = otm.get_template_by_handle(cube_prim_handle)
            num_segments = 2**subdivisions
            subdivision_scale = 1.0 / (num_segments)
            cube_template.scale = shape_bb.size() * subdivision_scale
            # TODO: test this scale
            otm.register_template(cube_template, "cubeTestSolid")

            test_obj = rom.add_object_by_template_handle("cubeTestSolid")
            assert test_obj.is_alive, "Test box object was not added correctly."

            cell_scale = cube_template.scale
            # run the grid test
            test_start_time = time.time()
            max_col_time = 0
            for x in range(num_segments):
                for y in range(num_segments):
                    for z in range(num_segments):
                        box_center = (
                            shape_bb.min
                            + mn.Vector3.x_axis(cell_scale[0]) * x
                            + mn.Vector3.y_axis(cell_scale[1]) * y
                            + mn.Vector3.z_axis(cell_scale[2]) * z
                            + cell_scale / 2.0
                        )
                        test_obj.translation = box_center
                        col_start = time.time()
                        test_obj.contact_test()
                        col_time = time.time() - col_start
                        max_col_time = max(max_col_time, col_time)
            total_test_time = time.time() - test_start_time
            avg_test_time = total_test_time / (num_segments**3)

            print(
                f"Physics grid collision test report: {obj_handle}. {subdivisions} subdivisions."
            )
            print(
                f"    Test took {total_test_time} seconds for {num_segments**3} collision tests."
            )

            # TODO: test this

            self.gt_data[obj_handle]["shape_test_results"][shape_id][
                "collision_grid_report"
            ][subdivisions] = {
                "total_col_time": total_test_time,
                "avg_col_time": avg_test_time,
                "max_col_time": max_col_time,
            }

    def run_physics_sphere_shake_test(self, obj_handle):
        """
        Places the DYNAMIC object in a sphere with other primitives and varies gravity to mix the objects.
        Per-frame physics compute time serves as a metric for dynamic simulation efficiency.
        """

        # prepare a sphere stage
        sphere_radius = self.gt_data[obj_handle]["scene_bb"].size().length() * 1.5
        sphere_stage_handle = "sphereTestStage"
        stm = self.mm.stage_template_manager
        sphere_template = stm.create_new_template(sphere_stage_handle)
        sphere_template.render_asset_handle = "data/test_assets/objects/sphere.glb"
        sphere_template.scale = mn.Vector3(sphere_radius * 2.0)  # glb is radius 0.5
        stm.register_template(sphere_template, sphere_stage_handle)

        # prepare the test sphere object
        otm = self.mm.object_template_manager
        sphere_test_handle = "sphereTestCollisionObject"
        sphere_prim_handle = otm.get_template_handles("sphereSolid")[0]
        sphere_template = otm.get_template_by_handle(sphere_prim_handle)
        test_sphere_radius = sphere_radius / 100.0
        sphere_template.scale = mn.Vector3(test_sphere_radius)
        otm.register_template(sphere_template, sphere_test_handle)
        assert otm.get_library_has_handle(sphere_test_handle)

        shape_id = self.get_proxy_shape_id(obj_handle)
        self.setup_shape_test_results_cache(obj_handle, shape_id)

        cfg = self.get_cfg_with_mm(scene=sphere_stage_handle)
        with habitat_sim.Simulator(cfg) as sim:
            rom = sim.get_rigid_object_manager()
            obj = rom.add_object_by_template_handle(obj_handle)
            assert obj.is_alive, "Object was not added correctly."

            # fill the remaining space with small spheres
            num_spheres = 0
            while num_spheres < 100:
                sphere_obj = rom.add_object_by_template_handle(sphere_test_handle)
                assert sphere_obj.is_alive, "Object was not added correctly."
                num_tries = 0
                while num_tries < 50:
                    num_tries += 1
                    # sample point
                    new_point = mn.Vector3(np.random.random(3) * 2.0 - np.ones(1))
                    while new_point.length() >= 0.99:
                        new_point = mn.Vector3(np.random.random(3) * 2.0 - np.ones(1))
                    sphere_obj.translation = new_point
                    if not sphere_obj.contact_test():
                        num_spheres += 1
                        break
                if num_tries == 50:
                    # we hit our max, so end the search
                    rom.remove_object_by_handle(sphere_obj.handle)
                    break

            # run the simulation for timing
            gravity = sim.get_gravity()
            grav_rotation_rate = 0.5  # revolutions per second
            max_sim_time = 10.0
            dt = 0.25
            real_start_time = time.time()
            start_time = sim.get_world_time()
            while sim.get_world_time() - start_time < max_sim_time:
                sim.step_world(dt)
                # change gravity
                cur_time = sim.get_world_time() - start_time
                grav_revolutions = grav_rotation_rate * cur_time
                # rotate the gravity vector around the Z axis
                g_quat = mn.Quaternion.rotation(
                    mn.Rad(grav_revolutions * mn.math.pi * 2), mn.Vector3(0, 0, 1)
                )
                sim.set_gravity(g_quat.transform_vector(gravity))

            real_test_time = time.time() - real_start_time

            print(f"Physics 'sphere shake' report: {obj_handle}")
            print(
                f"    {num_spheres} spheres took {real_test_time} seconds for {max_sim_time} sim seconds."
            )

            self.gt_data[obj_handle]["shape_test_results"][shape_id][
                "sphere_shake_report"
            ] = {
                "realtime": real_test_time,
                "sim_time": max_sim_time,
                "num_spheres": num_spheres,
            }

    def compute_gt_errors(self, obj_handle: str) -> None:
        """
        Compute and cache all ground truth error metrics.
        Assumes `self.gt_data[obj_handle]["raycasts"]` keys are different raycast results to be compared.
          'gt' must exist.
        """

        assert (
            obj_handle in self.gt_data
        ), f"`{obj_handle}` does not have any entry in gt_data: {self.gt_data.keys()}. Call to `setup_obj_gt(obj_handle)` required."
        assert (
            len(self.gt_data[obj_handle]["raycasts"]) > 1
        ), "Only gt results acquired, no error to compute. Try `compute_proxy_metrics` or `compute_baseline_metrics`."
        assert (
            "gt" in self.gt_data[obj_handle]["raycasts"]
        ), "Must have a ground truth to compare against. Should be generated in `setup_obj_gt(obj_handle)`."

        for shape_id in self.gt_data[obj_handle]["raycasts"]:
            self.setup_shape_test_results_cache(obj_handle, shape_id)
            if (
                shape_id != "gt"
                and "normalized_raycast_error"
                not in self.gt_data[obj_handle]["shape_test_results"][shape_id]
            ):
                normalized_error = get_raycast_results_cumulative_error_metric(
                    self.gt_data[obj_handle]["raycasts"]["gt"],
                    self.gt_data[obj_handle]["raycasts"][shape_id],
                )
                self.gt_data[obj_handle]["shape_test_results"][shape_id][
                    "normalized_raycast_error"
                ] = normalized_error

    def get_obj_render_mesh_filepath(self, obj_template_handle: str):
        """
        Return the filepath of the render mesh for an object.
        """
        otm = self.mm.object_template_manager
        obj_template = otm.get_template_by_handle(obj_template_handle)
        assert obj_template is not None, "Object template is not registerd."
        return os.path.abspath(obj_template.render_asset_handle)

    def permute_param_variations(
        self, param_ranges: Dict[str, List[Any]]
    ) -> List[List[Any]]:
        """
        Generate a list of all permutations of the provided parameter ranges defined in a Dict.
        """
        permutations = [[]]

        # permute variations
        for attr, values in param_ranges.items():
            new_permutations = []
            for v in values:
                for permutation in permutations:
                    extended_permutation = [(attr, v)]
                    for setting in permutation:
                        extended_permutation.append(setting)
                    new_permutations.append(extended_permutation)
            permutations = new_permutations
        print(f"Parameter permutations = {len(permutations)}")
        for setting in permutations:
            print(f"    {setting}")

        return permutations

    def run_coacd_grid_search(
        self,
        obj_template_handle: str,
        param_range_override: Optional[Dict[str, List[Any]]] = None,
    ) -> None:
        """
        Run grid search on relevant COACD params for an object.
        """

        # Parameter tuning tricks from https://github.com/SarahWeiii/CoACD in definition of COACDParams.

        param_ranges = {
            "threshold": [0.04, 0.01],
        }

        if param_range_override is not None:
            param_ranges = param_range_override

        permutations = self.permute_param_variations(param_ranges)

        coacd_start_time = time.time()
        coacd_iteration_times = {}
        coacd_num_hulls = {}
        # evaluate COACD settings
        for setting in permutations:
            coacd_param = COACDParams()
            setting_string = ""
            for attr, val in setting:
                setattr(coacd_param, attr, val)
                setting_string += f" '{attr}'={val}"

                self.increment_proxy_index(obj_template_handle)
                shape_id = self.get_proxy_shape_id(obj_template_handle)

                coacd_iteration_time = time.time()
                output_file, num_hulls = self.run_coacd(
                    obj_template_handle, coacd_param
                )

                # setup the proxy
                otm = self.mm.object_template_manager
                obj_template = otm.get_template_by_handle(obj_template_handle)
                obj_template.collision_asset_handle = output_file
                otm.register_template(obj_template)

                if "coacd_settings" not in self.gt_data[obj_template_handle]:
                    self.gt_data[obj_template_handle]["coacd_settings"] = {}
                self.gt_data[obj_template_handle]["coacd_settings"][shape_id] = (
                    coacd_param,
                    setting_string,
                )
                # store the asset file for this shape_id
                if "coacd_output_files" not in self.gt_data[obj_template_handle]:
                    self.gt_data[obj_template_handle]["coacd_output_files"] = {}
                self.gt_data[obj_template_handle]["coacd_output_files"][
                    shape_id
                ] = output_file

                self.compute_proxy_metrics(obj_template_handle)
                # self.compute_grid_collision_times(obj_template_handle, subdivisions=1)
                # self.run_physics_settle_test(obj_template_handle)
                # self.run_physics_sphere_shake_test(obj_template_handle)
                if self.compute_receptacle_useability_metrics:
                    self.compute_receptacle_access_metrics(
                        obj_handle=obj_template_handle, use_gt=False
                    )
                    self.compute_receptacle_stability(
                        obj_handle=obj_template_handle, use_gt=False
                    )
                coacd_iteration_times[shape_id] = time.time() - coacd_iteration_time
                coacd_num_hulls[shape_id] = num_hulls

        print(f"Total CAOCD time = {time.time()-coacd_start_time}")
        print("    Iteration times = ")
        for shape_id, settings in self.gt_data[obj_template_handle][
            "coacd_settings"
        ].items():
            print(
                f"     {shape_id} - {settings[1]} - {coacd_iteration_times[shape_id]}"
            )

    def run_coacd(
        self,
        obj_template_handle: str,
        params: COACDParams,
        output_file: Optional[str] = None,
    ) -> str:
        """
        Run COACD on an object given a set of parameters producing a file.
        If output_file is not provided, defaults to "COACD_output/obj_name.glb" where obj_name is truncated handle (filename, no path or file ending).
        """
        assert (
            coacd_imported
        ), "coacd is not installed. Linux only: 'pip install coacd'."
        if output_file is None:
            obj_name = obj_template_handle.split(".object_config.json")[0].split("/")[
                -1
            ]
            output_file = (
                "COACD_output/"
                + obj_name
                + "_"
                + self.get_proxy_shape_id(obj_template_handle)
                + ".glb"
            )
            os.makedirs(os.path.dirname(output_file), exist_ok=True)
        input_filepath = self.get_obj_render_mesh_filepath(obj_template_handle)
        # TODO: this seems dirty, maybe refactor:
        tris = trimesh.load(input_filepath).triangles
        verts = []
        indices = []
        v_counter = 0
        for tri in tris:
            indices.append([v_counter, v_counter + 1, v_counter + 2])
            v_counter += 3
            for vert in tri:
                verts.append(vert)
        imesh = coacd.Mesh()
        imesh.vertices = verts
        imesh.indices = indices
        parts = coacd.run_coacd(
            imesh,
            threshold=params.threshold,
            max_convex_hull=params.max_convex_hull,
            preprocess=params.preprocess,
            preprocess_resolution=params.preprocess_resolution,
            mcts_nodes=params.mcts_nodes,
            mcts_iterations=params.mcts_iterations,
            mcts_max_depth=params.mcts_max_depth,
            pca=params.pca,
            merge=params.merge,
            seed=params.seed,
        )
        mesh_parts = [
            trimesh.Trimesh(np.array(p.vertices), np.array(p.indices).reshape((-1, 3)))
            for p in parts
        ]
        scene = trimesh.Scene()

        np.random.seed(0)
        for p in mesh_parts:
            p.visual.vertex_colors[:, :3] = (np.random.rand(3) * 255).astype(np.uint8)
            scene.add_geometry(p)
        scene.export(output_file)
        return output_file, len(parts)

    def compute_shape_score(self, obj_h: str, shape_id: str) -> float:
        """
        Compute the shape score for the given object and shape_id.
        Higher shape score is better performance on the metrics.
        """
        shape_score = 0

        # start with normalized error
        normalized_error = self.gt_data[obj_h]["shape_test_results"][shape_id][
            "normalized_raycast_error"
        ]
        shape_score -= normalized_error

        # sum up scores for al receptacles
        for _rec_name, rec_data in self.gt_data[obj_h]["receptacles"].items():
            sh_rec_dat = rec_data["shape_id_results"][shape_id]
            gt_rec_dat = rec_data["shape_id_results"]["gt"]
            gt_access = gt_rec_dat["access_results"]["receptacle_access_score"]
            gt_stability = gt_rec_dat["stability_results"]["success_ratio"]

            # filter out generally bad receptacles from the score
            if gt_access < 0.15 or gt_stability < 0.5:
                "this receptacle is not good anyway, so skip it"
                continue

            # penalize different acces than ground truth (more access than gt is also bad as it implies worse overall shape matching)
            rel_access_score = abs(
                gt_access - sh_rec_dat["access_results"]["receptacle_access_score"]
            )
            shape_score -= rel_access_score

            # penalize stability directly (more stability than ground truth is not a problem)
            stability_ratio = sh_rec_dat["stability_results"]["success_ratio"]
            shape_score += stability_ratio

        return shape_score

    def optimize_object_col_shape(
        self,
        obj_h: str,
        col_shape_dir: Optional[str] = None,
        method="coacd",
        param_range_override: Optional[Dict[str, List[Any]]] = None,
    ):
        """
        Run COACD optimization for a specific object.
        Identify the optimal collision shape and save the result as the new default.

        :return: Tuple(best_shape_id, best_shape_score, original_shape_score) if best_shape_id == "pr0", then optimization didn't change anything.
        """
        otm = self.mm.object_template_manager
        obj_temp = otm.get_template_by_handle(obj_h)
        cur_col_shape_path = os.path.abspath(obj_temp.collision_asset_handle)
        self.setup_obj_gt(obj_h)
        self.compute_proxy_metrics(obj_h)
        self.compute_receptacle_stability(obj_h, use_gt=True)
        self.compute_receptacle_stability(obj_h)
        self.compute_receptacle_access_metrics(obj_h, use_gt=True)
        self.compute_receptacle_access_metrics(obj_h, use_gt=False)
        if method == "coacd":
            self.run_coacd_grid_search(obj_h, param_range_override)
        self.compute_gt_errors(obj_h)

        # time to select the best version
        best_shape_id = "pr0"
        pr0_shape_score = self.compute_shape_score(obj_h, "pr0")
        settings_key = method + "_settings"
        best_shape_score = pr0_shape_score
        shape_scores = {}
        for shape_id in self.gt_data[obj_h][settings_key]:
            shape_score = self.compute_shape_score(obj_h, shape_id)
            shape_scores[shape_id] = shape_score
            # we only want significantly better shapes (10% or 0.1 score better threshold)
            if (
                shape_score > (best_shape_score + abs(best_shape_score) * 0.1)
                and shape_score - best_shape_score > 0.1
            ):
                best_shape_id = shape_id
                best_shape_score = shape_score

        print(self.gt_data[obj_h][settings_key])
        print(shape_scores)

        if best_shape_id != "pr0":
            # re-save the best version
            print(
                f"Best shape_id = {best_shape_id} with shape score {best_shape_score} better than 'pr0' with shape score {pr0_shape_score}."
            )
            # copy the collision asset into the dataset directory
            if method == "coacd":
                asset_file = self.gt_data[obj_h]["coacd_output_files"][best_shape_id]
                os.system(f"cp {asset_file} {cur_col_shape_path}")
        else:
            print(
                f"Best shape_id = {best_shape_id} with shape score {best_shape_score}."
            )

        best_shape_params = None
        if best_shape_id != "pr0":
            best_shape_params = self.gt_data[obj_h][settings_key][best_shape_id]

        # self.cache_global_results()
        self.clean_obj_gt(obj_h)
        # then save results to file
        # self.save_results_to_csv("cpo_out")
        return (best_shape_id, best_shape_score, pr0_shape_score, best_shape_params)

    def cache_global_results(self) -> None:
        """
        Cache the current global cumulative results.
        Do this after an object's computation is done (compute_gt_errors) before cleaning the gt data.
        """

        for obj_handle in self.gt_data:
            # populate the high-level sub-cache definitions
            if obj_handle not in self.results:
                self.results[obj_handle] = {
                    "shape_metrics": {},
                    "receptacle_metrics": {},
                }
            # populate the per-shape metric sub-cache
            for shape_id, shape_results in self.gt_data[obj_handle][
                "shape_test_results"
            ].items():
                if shape_id == "gt":
                    continue
                self.results[obj_handle]["shape_metrics"][shape_id] = {"col_grid": {}}
                sm = self.results[obj_handle]["shape_metrics"][shape_id]
                if "normalized_raycast_error" in shape_results:
                    sm["normalized_raycast_error"] = shape_results[
                        "normalized_raycast_error"
                    ]
                if len(shape_results["settle_report"]) > 0:
                    sm["settle_success"] = shape_results["settle_report"]["success"]
                    sm["settle_time"] = shape_results["settle_report"]["settle_time"]
                    sm["settle_max_step_time"] = shape_results["settle_report"][
                        "max_time"
                    ]
                    sm["settle_realtime"] = shape_results["settle_report"]["realtime"]
                if len(shape_results["sphere_shake_report"]) > 0:
                    sm["shake_simtime"] = shape_results["sphere_shake_report"][
                        "sim_time"
                    ]
                    sm["shake_realtime"] = shape_results["sphere_shake_report"][
                        "realtime"
                    ]
                    sm["shake_num_spheres"] = shape_results["sphere_shake_report"][
                        "num_spheres"
                    ]
                if len(shape_results["collision_grid_report"]) > 0:
                    for subdiv, col_subdiv_results in shape_results[
                        "collision_grid_report"
                    ].items():
                        sm["col_grid"][subdiv] = {
                            "total_time": col_subdiv_results["total_col_time"],
                            "avg_time": col_subdiv_results["avg_col_time"],
                            "max_time": col_subdiv_results["max_col_time"],
                        }
            # populate the receptacle metric sub-cache
            for rec_name, rec_data in self.gt_data[obj_handle]["receptacles"].items():
                self.results[obj_handle]["receptacle_metrics"][rec_name] = {}
                for shape_id, shape_data in rec_data["shape_id_results"].items():
                    self.results[obj_handle]["receptacle_metrics"][rec_name][
                        shape_id
                    ] = {}
                    rsm = self.results[obj_handle]["receptacle_metrics"][rec_name][
                        shape_id
                    ]
                    if "stability_results" in shape_data:
                        rsm["stability_success_ratio"] = shape_data[
                            "stability_results"
                        ]["success_ratio"]
                        rsm["failed_snap"] = shape_data["stability_results"][
                            "failed_snap"
                        ]
                        rsm["failed_by_distance"] = shape_data["stability_results"][
                            "failed_by_distance"
                        ]
                        rsm["failed_unstable"] = shape_data["stability_results"][
                            "failed_unstable"
                        ]
                        rsm["total"] = shape_data["stability_results"]["total"]
                    if "access_results" in shape_data:
                        rsm["receptacle_access_score"] = shape_data["access_results"][
                            "receptacle_access_score"
                        ]
                        rsm["receptacle_access_rate"] = shape_data["access_results"][
                            "receptacle_access_rate"
                        ]

    def save_results_to_csv(self, filename: str) -> None:
        """
        Save current global results to a csv file in the self.output_directory.
        """

        assert len(self.results) > 0, "There must be results to save."

        assert (
            self.output_directory is not None
        ), "Must have an output directory to save."

        import csv

        filepath = os.path.join(self.output_directory, filename)

        # first collect all active metrics to log
        active_subdivs = []
        active_shape_metrics = []
        for _obj_handle, obj_results in self.results.items():
            for _shape_id, shape_results in obj_results["shape_metrics"].items():
                for metric in shape_results:
                    if metric == "col_grid":
                        for subdiv in shape_results["col_grid"]:
                            if subdiv not in active_subdivs:
                                active_subdivs.append(subdiv)
                    else:
                        if metric not in active_shape_metrics:
                            active_shape_metrics.append(metric)
        active_subdivs = sorted(active_subdivs)

        # save shape metric csv
        with open(filepath + ".csv", "w") as f:
            writer = csv.writer(f, quoting=csv.QUOTE_ALL)
            # first collect all column names (metrics):
            existing_cols = ["object_handle|shape_id"]
            existing_cols.extend(active_shape_metrics)
            for subdiv in active_subdivs:
                existing_cols.append(f"col_grid_{subdiv}_total_time")
                existing_cols.append(f"col_grid_{subdiv}_avg_time")
                existing_cols.append(f"col_grid_{subdiv}_max_time")
            # write column names row
            writer.writerow(existing_cols)

            # write results rows
            for obj_handle, obj_results in self.results.items():
                for shape_id, shape_results in obj_results["shape_metrics"].items():
                    row_data = [obj_handle + "|" + shape_id]
                    for metric_key in active_shape_metrics:
                        if metric_key in shape_results:
                            row_data.append(shape_results[metric_key])
                        else:
                            row_data.append("")
                    for subdiv in active_subdivs:
                        if subdiv in shape_results["col_grid"]:
                            row_data.append(
                                shape_results["col_grid"][subdiv]["total_time"]
                            )
                            row_data.append(
                                shape_results["col_grid"][subdiv]["avg_time"]
                            )
                            row_data.append(
                                shape_results["col_grid"][subdiv]["max_time"]
                            )
                        else:
                            for _ in range(3):
                                row_data.append("")
                    writer.writerow(row_data)

        # collect active receptacle metrics
        active_rec_metrics = []
        for _obj_handle, obj_results in self.results.items():
            for _rec_name, rec_results in obj_results["receptacle_metrics"].items():
                for _shape_id, shape_results in rec_results.items():
                    for metric in shape_results:
                        if metric not in active_rec_metrics:
                            active_rec_metrics.append(metric)

        # export receptacle metrics to CSV
        if self.compute_receptacle_useability_metrics:
            rec_filepath = filepath + "_receptacle_metrics"
            with open(rec_filepath + ".csv", "w") as f:
                writer = csv.writer(f, quoting=csv.QUOTE_ALL)
                # first collect all column names:
                existing_cols = ["obj_handle|receptacle|shape_id"]
                existing_cols.extend(active_rec_metrics)

                # write column names row
                writer.writerow(existing_cols)

                # write results rows
                for obj_handle, obj_results in self.results.items():
                    for rec_name, rec_results in obj_results[
                        "receptacle_metrics"
                    ].items():
                        for shape_id, shape_results in rec_results.items():
                            row_data = [obj_handle + "|" + rec_name + "|" + shape_id]
                            for metric_key in active_rec_metrics:
                                if metric_key in shape_results:
                                    row_data.append(shape_results[metric_key])
                                else:
                                    row_data.append("")
                            # write row data
                            writer.writerow(row_data)

    def compute_and_save_results_for_objects(
        self, obj_handle_substrings: List[str], output_filename: str = "cpo_out"
    ) -> None:
        # first find all full object handles
        otm = self.mm.object_template_manager
        obj_handles = []
        for obj_h in obj_handle_substrings:
            # find the full handle
            matching_obj_handles = otm.get_file_template_handles(obj_h)
            assert (
                len(matching_obj_handles) == 1
            ), f"None or many matching handles to substring `{obj_h}`: {matching_obj_handles}"
            obj_handles.append(matching_obj_handles[0])

        print(f"Found handles: {obj_handles}.")
        print("Computing metrics:")
        # then compute metrics for all objects and cache
        for obix, obj_h in enumerate(obj_handles):
            print("-------------------------------")
            print(f"Computing metric for `{obj_h}`, {obix}|{len(obj_handles)}")
            print("-------------------------------")
            self.setup_obj_gt(obj_h)
            # self.compute_baseline_metrics(obj_h)
            self.compute_proxy_metrics(obj_h)

            # physics tests
            # self.run_physics_settle_test(obj_h)
            # self.run_physics_sphere_shake_test(obj_h)
            # self.compute_grid_collision_times(obj_h, subdivisions=0)
            # self.compute_grid_collision_times(obj_h, subdivisions=1)
            # self.compute_grid_collision_times(obj_h, subdivisions=2)

            # receptacle metrics:
            if self.compute_receptacle_useability_metrics:
                self.compute_receptacle_stability(obj_h, use_gt=True)
                self.compute_receptacle_stability(obj_h)
                print(" GT Receptacle Metrics:")
                self.compute_receptacle_access_metrics(obj_h, use_gt=True)
                print(" PR Receptacle Metrics:")
                self.compute_receptacle_access_metrics(obj_h, use_gt=False)
            self.compute_gt_errors(obj_h)
            print_dict_structure(self.gt_data)
            self.cache_global_results()
            print_dict_structure(self.results)
            self.clean_obj_gt(obj_h)

        # then save results to file
        self.save_results_to_csv(output_filename)


def object_has_receptacles(
    object_template_handle: str,
    otm: habitat_sim.attributes_managers.ObjectAttributesManager,
) -> bool:
    """
    Returns whether or not an object has a receptacle defined in its config file.
    """
    # this prefix will be present for any entry which defines a receptacle
    receptacle_prefix_string = "receptacle_"

    object_template = otm.get_template_by_handle(object_template_handle)
    assert (
        object_template is not None
    ), f"No template matching handle {object_template_handle}."

    user_cfg = object_template.get_user_config()

    return any(
        sub_config_key.startswith(receptacle_prefix_string)
        for sub_config_key in user_cfg.get_subconfig_keys()
    )


def get_objects_in_scene(
    dataset_path: str, scene_handle: str, mm: habitat_sim.metadata.MetadataMediator
) -> List[str]:
    """
    Load a scene and return a list of object template handles for all instantiated objects.
    """
    sim_settings = default_sim_settings.copy()
    sim_settings["scene_dataset_config_file"] = dataset_path
    sim_settings["scene"] = scene_handle
    sim_settings["default_agent_navmesh"] = False

    cfg = make_cfg(sim_settings)
    cfg.metadata_mediator = mm

    with habitat_sim.Simulator(cfg) as sim:
        scene_object_template_handles = []
        rom = sim.get_rigid_object_manager()
        live_objects = rom.get_objects_by_handle_substring()
        for _obj_handle, obj in live_objects.items():
            if obj.creation_attributes.handle not in scene_object_template_handles:
                scene_object_template_handles.append(obj.creation_attributes.handle)
        return scene_object_template_handles


def parse_object_orientations_from_metadata_csv(
    metadata_csv: str,
) -> Dict[str, Tuple[mn.Vector3, mn.Vector3]]:
    """
    Parse the 'up' and 'front' vectors of objects from a csv metadata file.

    :param metadata_csv: The absolute filepath of the metadata CSV.

    :return: A Dict mapping object ids to a Tuple of up, front vectors.
    """

    def str_to_vec(vec_str: str) -> mn.Vector3:
        """
        Convert a list of 3 comma separated strings into a Vector3.
        """
        elem_str = [float(x) for x in vec_str.split(",")]
        assert len(elem_str) == 3, f"string '{vec_str}' must be a 3 vec."
        return mn.Vector3(tuple(elem_str))

    orientations = {}

    with open(metadata_csv, newline="") as csvfile:
        reader = csv.reader(csvfile, delimiter=",")
        id_row_ix = -1
        up_row_ix = -1
        front_row_ix = -1
        for rix, data_row in enumerate(reader):
            if rix == 0:
                id_row_ix = data_row.index("id")
                up_row_ix = data_row.index("up")
                front_row_ix = data_row.index("front")
            else:
                up = data_row[up_row_ix]
                front = data_row[front_row_ix]
                if len(up) == 0 or len(front) == 0:
                    # both must be set or neither
                    assert len(up) == 0
                    assert len(front) == 0
                else:
                    orientations[data_row[id_row_ix]] = (
                        str_to_vec(up),
                        str_to_vec(front),
                    )

    return orientations


def correct_object_orientations(
    obj_handles: List[str],
    obj_orientations: Dict[str, Tuple[mn.Vector3, mn.Vector3]],
    mm: habitat_sim.metadata.MetadataMediator,
) -> None:
    """
    Correct the orientations for all object templates in 'obj_handles' as specified by 'obj_orientations'.

    :param obj_handles: A list of object template handles.
    :param obj_orientations: A dict mapping object names (abridged, not handles) to Tuple of (up,front) orientation vectors.
    """
    obj_handle_to_orientation = {}
    for obj_name in obj_orientations:
        for obj_handle in obj_handles:
            if obj_name in obj_handle:
                obj_handle_to_orientation[obj_handle] = obj_orientations[obj_name]
    print(f"obj_handle_to_orientation = {obj_handle_to_orientation}")
    for obj_handle, orientation in obj_handle_to_orientation.items():
        obj_template = mm.object_template_manager.get_template_by_handle(obj_handle)
        obj_template.orient_up = orientation[0]
        obj_template.orient_front = orientation[1]
        mm.object_template_manager.register_template(obj_template)


def write_failure_ids(
    failures: List[Tuple[int, str, str]], filename="failures_out.txt"
) -> None:
    """
    Write handles from failure tuples to file for use as exclusion or for follow-up investigation.
    """
    with open(filename, "w") as file:
        for f in failures:
            file.write(f[1])


def main():
    parser = argparse.ArgumentParser(
        description="Automate collision shape creation and validation."
    )
    parser.add_argument("--dataset", type=str, help="path to SceneDataset.")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument(
        "--scenes", type=str, nargs="+", help="one or more scenes to optimize."
    )
    group.add_argument(
        "--objects", type=str, nargs="+", help="one or more objects to optimize."
    )
    group.add_argument(
        "--all-rec-objects",
        action="store_true",
        help="Optimize all objects in the dataset with receptacles.",
    )
    group.add_argument(
        "--objects-file",
        type=str,
        help="optimize objects from a file containing object names separated by newline characters.",
    )
    parser.add_argument(
        "--start-ix",
        default=-1,
        type=int,
        help="If optimizing all assets, provide a start index.",
    )
    parser.add_argument(
        "--end-ix",
        default=-1,
        type=int,
        help="If optimizing all assets, provide an end index.",
    )
    parser.add_argument(
        "--parts-only",
        action="store_true",
        help="culls all objects without _part_ in the name.",
    )
    parser.add_argument(
        "--exclude",
        type=str,
        nargs="+",
        help="one or more objects to exclude from optimization (e.g. if it inspires a crash in COACD).",
    )
    parser.add_argument(
        "--exclude-files",
        type=str,
        nargs="+",
        help="provide one or more files with objects to exclude from optimization (NOTE: txt file with one id on each line, object names may include prefix 'fpModel.' which will be stripped.).",
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default="collision_shape_automation/",
        help="output directory for saved csv and images. Default = `./collision_shape_automation/`.",
    )
    parser.add_argument(
        "--debug-images",
        action="store_true",
        help="turns on debug image output.",
    )
    parser.add_argument(
        "--export-fp-model-ids",
        type=str,
        help="Intercept optimization to output a txt file with model ids for online model categorizer view.",
    )
    parser.add_argument(
        "--coacd-thresholds",
        type=float,
        nargs="+",
        help="one or more coacd thresholds [0-1] (lower is more detailed) to search. If not provided, default are [0.04, 0.01].",
    )
    args = parser.parse_args()

    if not args.all_rec_objects:
        assert (
            args.start_ix == -1
        ), "Can only provide start index for all objects optimization."
        assert (
            args.end_ix == -1
        ), "Can only provide end index for all objects optimization."

    param_range_overrides = None
    if args.coacd_thresholds:
        param_range_overrides = {
            "threshold": args.coacd_thresholds,
        }

    sim_settings = default_sim_settings.copy()
    sim_settings["scene_dataset_config_file"] = args.dataset
    # necessary for debug rendering
    sim_settings["sensor_height"] = 0
    sim_settings["width"] = 720
    sim_settings["height"] = 720
    sim_settings["clear_color"] = mn.Color4.magenta() * 0.5
    sim_settings["default_agent_navmesh"] = False

    # use the CollisionProxyOptimizer to compute metrics for multiple objects
    cpo = CollisionProxyOptimizer(sim_settings, output_directory=args.output_dir)
    cpo.generate_debug_images = args.debug_images
    otm = cpo.mm.object_template_manager

    excluded_object_strings = []
    if args.exclude:
        excluded_object_strings = args.exclude
    if args.exclude_files:
        for filepath in args.exclude_files:
            assert os.path.exists(filepath)
            with open(filepath, "r") as f:
                lines = [line.strip().split("fpModel.")[-1] for line in f.readlines()]
                excluded_object_strings.extend(lines)
    excluded_object_strings = list(dict.fromkeys(excluded_object_strings))

    # ----------------------------------------------------
    # specific object handle provided
    if args.objects or args.all_rec_objects or args.objects_file:
        assert (
            not args.export_fp_model_ids
        ), "Feature not available for objects, only for scenes."

        unique_objects = None

        if args.objects:
            # deduplicate the list
            unique_objects = list(dict.fromkeys(args.objects))
        elif args.objects_file:
            assert os.path.exists(args.objects_file)
            with open(args.objects_file, "r") as f:
                lines = [line.strip() for line in f.readlines()]
                unique_objects = list(dict.fromkeys(lines))
        elif args.all_rec_objects:
            objects_in_dataset = otm.get_file_template_handles()
            rec_obj_in_dataset = [
                objects_in_dataset[i]
                for i in range(len(objects_in_dataset))
                if object_has_receptacles(objects_in_dataset[i], otm)
            ]
            print(
                f"Number of objects in dataset with receptacles = {len(rec_obj_in_dataset)}"
            )
            unique_objects = rec_obj_in_dataset

        # validate the object handles
        object_handles = []
        for object_name in unique_objects:
            # get templates matches
            matching_templates = otm.get_file_template_handles(object_name)
            assert (
                len(matching_templates) > 0
            ), f"No matching templates in the dataset for '{object_name}'"
            assert (
                len(matching_templates) == 1
            ), f"More than one matching template in the dataset for '{object_name}': {matching_templates}"
            obj_h = matching_templates[0]

            # skip excluded objects
            exclude_object = False
            for ex_obj in excluded_object_strings:
                if ex_obj in obj_h:
                    print(f"Excluding object {object_name}.")
                    exclude_object = True
                    break
            if not exclude_object:
                object_handles.append(obj_h)

        if args.parts_only:
            object_handles = [obj_h for obj_h in object_handles if "_part_" in obj_h]
            print(f"part objects only = {object_handles}")

        # optimize the objects
        results = []
        failures = []
        start = args.start_ix if args.start_ix >= 0 else 0
        end = args.end_ix if args.end_ix >= 0 else len(object_handles)
        assert end >= start, f"Start index ({start}) is lower than end index ({end})."
        for obj_ix in range(start, end):
            obj_h = object_handles[obj_ix]
            print("+++++++++++++++++++++++++")
            print("+++++++++++++++++++++++++")
            print(f"Optimizing '{obj_h}' : {obj_ix} of {len(object_handles)}")
            print("+++++++++++++++++++++++++")
            try:
                results.append(
                    cpo.optimize_object_col_shape(
                        obj_h,
                        method="coacd",
                        param_range_override=param_range_overrides,
                    )
                )
                print(
                    f"Completed optimization of '{obj_h}' : {obj_ix} of {len(object_handles)}"
                )
            except Exception as err:
                failures.append((obj_ix, obj_h, err))
        # display results
        print("Object Optimization Results:")
        for obj_h, obj_result in zip(object_handles, results):
            print(f"    {obj_h}: {obj_result}")
        print("Failures:")
        for f in failures:
            print(f" {f}")
        write_failure_ids(failures)
    # ----------------------------------------------------

    # ----------------------------------------------------
    # run the pipeline for a set of object parsed from a scene
    if args.scenes:
        scene_object_handles: Dict[str, List[str]] = {}

        # deduplicate the list
        unique_scenes = list(dict.fromkeys(args.scenes))

        # first validate the scene names have a unique match
        scene_handles = cpo.mm.get_scene_handles()
        for scene_name in unique_scenes:
            matching_scenes = [h for h in scene_handles if scene_name in h]
            assert (
                len(matching_scenes) > 0
            ), f"No scenes found matching provided scene name '{scene_name}'."
            assert (
                len(matching_scenes) == 1
            ), f"More than one scenes found matching provided scene name '{scene_name}': {matching_scenes}."

        # collect all the objects for all the scenes in advance
        for scene_name in unique_scenes:
            objects_in_scene = get_objects_in_scene(
                dataset_path=args.dataset, scene_handle=scene_name, mm=cpo.mm
            )
            assert (
                len(objects_in_scene) > 0
            ), f"No objects found in scene '{scene_name}'. Are you sure this is a valid scene?"

            # skip excluded objects
            included_objects = []
            for obj_h in objects_in_scene:
                exclude_object = False
                for ex_obj in excluded_object_strings:
                    if ex_obj in obj_h:
                        exclude_object = True
                        print(f"Excluding object {obj_h}.")
                        break
                if not exclude_object:
                    included_objects.append(obj_h)
            scene_object_handles[scene_name] = included_objects

        if args.export_fp_model_ids:
            # intercept optimization to instead export a txt file with model ids for import into the model categorizer tool
            with open(args.export_fp_model_ids, "w") as f:
                aggregated_object_ids = []
                for scene_objects in scene_object_handles.values():
                    rec_obj_in_scene = [
                        scene_objects[i]
                        for i in range(len(scene_objects))
                        if object_has_receptacles(scene_objects[i], otm)
                    ]
                    aggregated_object_ids.extend(rec_obj_in_scene)
                aggregated_object_ids = list(dict.fromkeys(aggregated_object_ids))
                for obj_h in aggregated_object_ids:
                    obj_name = obj_h.split(".object_config.json")[0].split("/")[-1]
                    # TODO: this will change once the Model Categorizer supports these
                    if "_part_" not in obj_name:
                        f.write("fpModel." + obj_name + "\n")
            print(f"Export fpModel ids to {args.export_fp_model_ids}")
            exit()

        # optimize each scene
        all_scene_results: Dict[
            str, Dict[str, List[Tuple[str, float, float, Any]]]
        ] = {}
        for scene, objects_in_scene in scene_object_handles.items():
            # clear and re-initialize the caches between scenes to prevent memory overflow on large batches.
            cpo.init_caches()

            # ----------------------------------------------------
            # get a subset of objects with receptacles defined
            rec_obj_in_scene = [
                objects_in_scene[i]
                for i in range(len(objects_in_scene))
                if object_has_receptacles(objects_in_scene[i], otm)
            ]
            print(
                f"Number of objects in scene '{scene}' with receptacles = {len(rec_obj_in_scene)}"
            )
            # ----------------------------------------------------

            # ----------------------------------------------------
            # load object orientation metadata
            # BUG: Receptacles are not re-oriented by internal re-orientation transforms. Need to fix this...
            # reorient_objects = False
            # if reorient_objects:
            #     fp_models_metadata_file = (
            #         "/home/alexclegg/Documents/dev/fphab/fpModels_metadata.csv"
            #     )
            #     obj_orientations = parse_object_orientations_from_metadata_csv(
            #         fp_models_metadata_file
            #     )
            #     correct_object_orientations(all_handles, obj_orientations, cpo.mm)
            # ----------------------------------------------------

            # run shape opt for all objects in the scene
            scene_results: Dict[str, List[Tuple[str, float, float, Any]]] = {}
            for obj_h in rec_obj_in_scene:
                scene_results[obj_h] = cpo.optimize_object_col_shape(
                    obj_h, method="coacd", param_range_override=param_range_overrides
                )

            all_scene_results[scene] = scene_results

            print("------------------------------------")
            print(f"Finished optimization of scene '{scene}': \n    {scene_results}")
            print("------------------------------------")

        print("==========================================")
        print(f"Finished optimization of all scenes: \n {all_scene_results}")
        print("==========================================")


if __name__ == "__main__":
    main()
