# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import argparse
import random
import time
from typing import Any, Dict, List, Tuple

import magnum as mn
import numpy as np

import habitat_sim
from habitat_sim.utils.settings import default_sim_settings, make_cfg

# object samples:
# chair - good approximation: 0a5e809804911e71de6a4ef89f2c8fef5b9291b3.glb
# shelves - bad approximation: d1d1e0cdaba797ee70882e63f66055675c3f1e7f.glb


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


def sample_points_from_sphere(
    center: mn.Vector3, radius: float, num_points: int = 100
) -> List[List[mn.Vector3]]:
    """
    Sample num_points from a sphere defined by center and radius.
    Return all points in two identical lists to indicate pairwise raycasting.
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


def run_pairwise_raycasts(
    points: List[List[mn.Vector3]], sim: habitat_sim.Simulator, min_dist: float = 0.1
) -> List[habitat_sim.physics.RaycastResults]:
    """
    Raycast between each pair of points from different surfaces.
    :param min_dist: The minimum ray distance to allow. Cull all candidate pairs closer than this distance.
    """
    all_raycast_results: List[habitat_sim.physics.RaycastResults] = []
    for fix0 in range(len(points)):
        for fix1 in range(len(points)):
            if fix0 != fix1:  # no pairs on the same face
                for p0 in points[fix0]:
                    for p1 in points[fix1]:
                        if (p0 - p1).length() > min_dist:
                            # this is a valid pair of points
                            ray = habitat_sim.geo.Ray(p0, p1 - p0)  # origin, direction
                            # raycast
                            all_raycast_results.append(sim.cast_ray(ray=ray))
                            # reverse direction as separate entry (because exiting a convex does not generate a hit record)
                            ray2 = habitat_sim.geo.Ray(p1, p0 - p1)  # origin, direction
                            # raycast
                            all_raycast_results.append(sim.cast_ray(ray=ray2))

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

            if gt_results.has_hits():
                # draw first and last hits for gt and proxy
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


def evaluate_collision_shape(
    object_handle: str,
    sim_settings: Dict[str, Any],
    sample_shape="sphere",
    mm=None,
    num_point_samples=100,
) -> None:
    """
    Runs in-engine evaluation of collision shape accuracy for a single object.
    Uses a raycast from a bounding shape to approximate surface error between a proxy shape and ground truth (the render shape).
    Returns:
      ground_truth and proxy raw raycast results,
      the object's template handle,
      all test points used for the raycast,
      scalar error metrics
    1. initializes a simulator with the object render shape as a stage collision mesh.
    2. uses the scene bouding box to sample points on a configured bounding shape (e.g. inflated AABB or sphere).
    3. raycasts between sampled point pairs on both ground truth and collision proxy shapes to heuristicall measure error.
    4. computes scalar error metrics

    :param object_handle: The object to evaluate.
    :param sim_settings: Any simulator settings for initialization (should be physics enabled and point to correct dataset).
    :param sample_shape: The desired bounding shape for raycast: "sphere" or "aabb".
    :param mm: A pre-configured MetadataMediator may be provided to reduce initialization time. Should have the correct dataset already configured.
    """
    profile_metrics = {}
    start_time = time.time()
    check_time = time.time()
    cfg = make_cfg(sim_settings)
    if mm is not None:
        cfg.metadata_mediator = mm
    with habitat_sim.Simulator(cfg) as sim:
        profile_metrics["init0"] = time.time() - check_time
        check_time = time.time()
        # evaluate the collision shape defined in an object's template
        # 1. get the template from MM
        matching_templates = sim.get_object_template_manager().get_template_handles(
            object_handle
        )
        assert (
            len(matching_templates) == 1
        ), f"Multiple or no template matches for handle '{object_handle}': ({matching_templates})"
        obj_template_handle = matching_templates[0]
        obj_template = sim.get_object_template_manager().get_template_by_handle(
            obj_template_handle
        )
        obj_template.compute_COM_from_shape = False
        obj_template.com = mn.Vector3(0)
        # obj_template.bounding_box_collisions = True
        # obj_template.is_collidable = False
        sim.get_object_template_manager().register_template(obj_template)
        # 2. Setup a stage from the object's render mesh
        stm = sim.get_stage_template_manager()
        stage_template_name = "obj_as_stage_template"
        new_stage_template = stm.create_new_template(handle=stage_template_name)
        new_stage_template.render_asset_handle = obj_template.render_asset_handle
        stm.register_template(
            template=new_stage_template, specified_handle=stage_template_name
        )
        # 3. Initialize the simulator for the stage
        new_settings = sim_settings.copy()
        new_settings["scene"] = stage_template_name
        new_config = make_cfg(new_settings)
        sim.reconfigure(new_config)
        profile_metrics["init_stage"] = time.time() - check_time
        check_time = time.time()

        # 4. compute initial metric baselines
        scene_bb = sim.get_active_scene_graph().get_root_node().cumulative_bb
        inflated_scene_bb = scene_bb.scaled(mn.Vector3(1.25))
        inflated_scene_bb = mn.Range3D.from_center(
            scene_bb.center(), inflated_scene_bb.size() / 2.0
        )
        test_points = None
        if sample_shape == "aabb":
            # bounding box sample
            test_points = sample_points_from_range3d(
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
        profile_metrics["sample_points"] = time.time() - check_time
        check_time = time.time()

        gt_raycast_results = run_pairwise_raycasts(test_points, sim)
        profile_metrics["raycast_stage"] = time.time() - check_time
        check_time = time.time()

        # 5. load the object with proxy (in NONE stage)
        new_settings = sim_settings.copy()
        new_config = make_cfg(new_settings)
        sim.reconfigure(new_config)
        obj = sim.get_rigid_object_manager().add_object_by_template_handle(
            obj_template_handle
        )
        obj.translation = obj.com
        profile_metrics["init_object"] = time.time() - check_time
        check_time = time.time()

        # 6. compute the metric for proxy object
        pr_raycast_results = run_pairwise_raycasts(test_points, sim)
        profile_metrics["raycast_object"] = time.time() - check_time
        check_time = time.time()

        # 7. compare metrics
        normalized_error = get_raycast_results_cumulative_error_metric(
            gt_raycast_results, pr_raycast_results
        )
        profile_metrics["compute_metrics"] = time.time() - check_time
        check_time = time.time()
        profile_metrics["total"] = time.time() - start_time

        return (
            gt_raycast_results,
            pr_raycast_results,
            obj_template_handle,
            test_points,
            normalized_error,
            profile_metrics,
        )


def main():
    parser = argparse.ArgumentParser(
        description="Automate collision shape creation and validation."
    )
    parser.add_argument("--dataset", type=str, help="path to SceneDataset.")
    parser.add_argument(
        "--object-handle", type=str, help="handle identifying the object to evaluate."
    )
    args = parser.parse_args()

    sim_settings = default_sim_settings.copy()
    sim_settings["scene_dataset_config_file"] = args.dataset

    evaluate_collision_shape(args.object_handle, sim_settings)


if __name__ == "__main__":
    main()
