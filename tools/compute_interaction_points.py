import math
import os
from typing import Any, Dict, List, Tuple, Union

# NOTE: (requires habitat-lab) get metadata for semantics
import habitat.sims.habitat_simulator.sim_utilities as sutils
import magnum as mn

import habitat_sim
from habitat_sim import Simulator, physics
from habitat_sim.metadata import MetadataMediator
from habitat_sim.utils.settings import default_sim_settings, make_cfg


def find_interaction_surface_points(
    sim: habitat_sim.Simulator,
    obj: Union[physics.ManagedRigidObject, physics.ManagedArticulatedObject],
    num_vertical_slices: int = 10,
    num_radial_slices: int = 10,
    cull_points=True,
    max_point_set_size: int = 20,
) -> Tuple[List[mn.Vector3], List[mn.Vector3]]:
    """
    Use raycasting to find a set of points on the lateral surfaces of an object.

    :return: the list of interaction points and the list of cast rays
    """

    assert num_vertical_slices >= 1, "Must at least slice in half."
    assert num_radial_slices >= 4, "Must at least form a 2D simplex."
    assert max_point_set_size >= 3, "Must at least form a 2D simplex."

    surface_points: List[mn.Vector3] = []

    # compute the ray set:
    aabb = obj.aabb
    ray_set: List[habitat_sim.geo.Ray] = []

    # compute the circle size to contain the aabb: xz diagonal length+10%.
    # Used for max ray distance and cylinder sampling.
    size_x = aabb.size_x()
    size_y = aabb.size_y()
    size_z = aabb.size_z()
    circle_rad = math.sqrt((size_x / 2.0) ** 2 + (size_z / 2.0) ** 2) * 1.1
    if False:
        # cylinder (furniture):
        # for each vertical slice, select rays in a circle
        for i in range(1, num_vertical_slices + 1):
            y_val = aabb.bottom + (i / num_vertical_slices) * size_y
            center_point = mn.Vector3(0, y_val, 0)
            for r in range(num_radial_slices):
                cx = circle_rad * math.cos(2 * math.pi * r / num_radial_slices)
                cz = circle_rad * math.sin(2 * math.pi * r / num_radial_slices)
                origin_point = mn.Vector3(cx, y_val, cz)
                ray_set.append(
                    habitat_sim.geo.Ray(origin_point, center_point - origin_point)
                )

    if True:
        # cast from box edges along box axes
        # NOTE: using num_radial_slices per face
        # for each vertical slice, select rays from each xz edge
        front_to_back = mn.Vector3(aabb.right - aabb.left, 0, 0) * 1.1
        back_to_front = mn.Vector3(aabb.left - aabb.right, 0, 0) * 1.1
        left_to_right = mn.Vector3(0, 0, aabb.back - aabb.front) * 1.1
        right_to_left = mn.Vector3(0, 0, aabb.front - aabb.back) * 1.1
        for i in range(num_vertical_slices):
            y_val = aabb.bottom + (i / (num_vertical_slices - 1)) * size_y
            # front to back
            for s in range(num_radial_slices):
                z_val = aabb.back + size_z * (s / (num_radial_slices - 1))
                origin_point1 = mn.Vector3(aabb.left * 1.1, y_val, z_val)
                origin_point2 = mn.Vector3(aabb.right * 1.1, y_val, z_val)
                ray_set.append(habitat_sim.geo.Ray(origin_point1, front_to_back))
                ray_set.append(habitat_sim.geo.Ray(origin_point2, back_to_front))
            # left to right
            for s in range(num_radial_slices):
                x_val = aabb.left + size_x * (s / (num_radial_slices - 1))
                origin_point1 = mn.Vector3(x_val, y_val, aabb.front * 1.1)
                origin_point2 = mn.Vector3(x_val, y_val, aabb.back * 1.1)
                ray_set.append(habitat_sim.geo.Ray(origin_point1, left_to_right))
                ray_set.append(habitat_sim.geo.Ray(origin_point2, right_to_left))

    # sphere (objects):
    # TODO: compute the sphere rad size to contain the aabb: max(size)/2
    # TODO: jittered spherical sample or icosphere verts
    # TODO: aim at the sphere center

    # move object to a safe (far away) location and re-orient to identity
    cached_transform = obj.transformation
    cached_mt = obj.motion_type
    obj.motion_type = physics.MotionType.KINEMATIC
    obj.translation += mn.Vector3(9000, 9000, 9000)
    obj.rotation = mn.Quaternion()  # identity

    # raycast:
    for ray in ray_set:
        # move the local ray to global space
        ray.origin += obj.translation
        # cast ray and get fist contact point
        ray_results = sim.cast_ray(ray, max_distance=circle_rad * 2)
        if ray_results.has_hits():
            surface_points.append(ray_results.hits[0].point)
        ray.origin -= obj.translation

    # culling:
    if cull_points:
        # first compute pairwise distance
        distances: List[Tuple[float, int, int]] = []
        for pix in range(len(surface_points)):
            for pix2 in range(pix + 1, len(surface_points)):
                # tuple (dist, index1, index2)
                distances.append(
                    ((surface_points[pix] - surface_points[pix2]).length(), pix, pix2)
                )
        remove_ixs = []
        # pairwise nearest point removal
        while len(surface_points) - len(remove_ixs) > max_point_set_size:
            # sort smallest distance to the top
            distances.sort(key=lambda x: x[0])
            # determine which of the pair to remove by identifying the one with next closest neighbor
            candidates = [distances[0][1], distances[0][2]]
            remove_ix = candidates[0]
            for _dist, ix1, ix2 in distances[1:]:
                if ix1 in candidates:
                    remove_ix = ix1
                    break
                if ix2 in candidates:
                    remove_ix = ix2
                    break
            remove_ixs.append(remove_ix)
            # remove the index from distances
            distances = [tpl for tpl in distances if remove_ix not in tpl]
        surface_points = [
            surface_points[ix]
            for ix in range(len(surface_points))
            if ix not in remove_ixs
        ]

    surface_points = obj.transform_world_pts_to_local(surface_points, link_id=-1)

    # return the object to initial state
    obj.transformation = cached_transform
    obj.motion_type = cached_mt

    return surface_points, ray_set

    # follow-ups:
    # TODO: links

    # tests:
    # TODO: scaled object
    # TODO: re-oriented object
    # TODO: thin structures
    # TODO: L shaped couch


def save_interaction_points_to_markerset(
    obj: Union[physics.ManagedRigidObject, physics.ManagedArticulatedObject],
    interaction_points: List[mn.Vector3],
) -> None:
    """
    Save the set of interaction points into the object's user_defined metadata as a MarkerSet.
    """

    obj.marker_sets.set_task_link_markerset_points(
        "interaction_surface_points", "body", "primary", interaction_points
    )


def save_markerset_attributes(
    sim: habitat_sim.Simulator,
    obj: Union[physics.ManagedRigidObject, physics.ManagedArticulatedObject],
) -> None:
    """
    Modify the attributes for the passed object to include the
    currently edited markersets and save those attributes to disk
    """
    # get the name of the attrs used to initialize the object
    obj_init_attr_handle = obj.creation_attributes.handle

    if obj.is_articulated:
        # save AO config
        attrMgr = sim.metadata_mediator.ao_template_manager
    else:
        # save obj config
        attrMgr = sim.metadata_mediator.object_template_manager
    # get copy of initialization attributes as they were in manager,
    # unmodified by scene instance values such as scale
    init_attrs = attrMgr.get_template_by_handle(obj_init_attr_handle)
    # TEMP TODO Remove this when fixed in Simulator
    # Clean up sub-dirs being added to asset handles.
    if obj.is_articulated:
        init_attrs.urdf_filepath = init_attrs.urdf_filepath.split(os.sep)[-1]
        init_attrs.render_asset_handle = init_attrs.render_asset_handle.split(os.sep)[
            -1
        ]
    else:
        init_attrs.render_asset_handle = init_attrs.render_asset_handle.split(os.sep)[
            -1
        ]
        init_attrs.collision_asset_handle = init_attrs.collision_asset_handle.split(
            os.sep
        )[-1]
    # put edited subconfig into initial attributes' markersets
    markersets = init_attrs.get_marker_sets()
    for subconfig_key in obj.marker_sets.get_subconfig_keys():
        markersets.save_subconfig(
            subconfig_key, obj.marker_sets.get_subconfig(subconfig_key)
        )

    # reregister template
    attrMgr.register_template(init_attrs, init_attrs.handle, True)
    # save to original location - uses saved location in attributes
    attrMgr.save_template_by_handle(init_attrs.handle, True)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--dataset",
        default="default",
        type=str,
        metavar="DATASET",
        help='dataset configuration file to use (default: "default")',
    )

    parser.add_argument(
        "--scenes",
        nargs="+",
        type=str,
        help="A subset of scene names to process. Limits the iteration to less than the full set of scenes.",
        default=None,
    )

    args = parser.parse_args()

    # create an initial simulator config
    sim_settings: Dict[str, Any] = default_sim_settings
    sim_settings["scene_dataset_config_file"] = args.dataset
    cfg = make_cfg(sim_settings)

    # pre-initialize a MetadataMediator to iterate over scenes
    mm = MetadataMediator()
    mm.active_dataset = args.dataset

    target_scenes = mm.get_scene_handles()
    if args.scenes is not None:
        target_scenes = args.scenes
    num_scenes = len(target_scenes)

    for s_ix, scene_handle in enumerate(target_scenes):
        print("=================================================================")
        print(
            f"Setting up scene for {scene_handle} ({s_ix}|{num_scenes} = {s_ix/float(num_scenes)*100}%)"
        )

        cfg.sim_cfg.scene_id = scene_handle
        print(" - init")
        with Simulator(cfg) as sim:
            objects = sutils.get_all_objects(sim)
            print(f" - processing {len(objects)} objects:")
            for oix, obj in enumerate(objects):
                print(f"   - obj ({oix}/{len(objects)}) {obj.handle}")
                # if not obj.marker_sets.has_taskset("interaction_surface_points"):
                print("      - computing interaction points")
                surface_points, debug_rays = find_interaction_surface_points(
                    sim,
                    obj,
                    num_radial_slices=10,
                    num_vertical_slices=10,
                    cull_points=True,
                    max_point_set_size=20,
                )
                save_interaction_points_to_markerset(obj, surface_points)
                save_markerset_attributes(sim, obj)
