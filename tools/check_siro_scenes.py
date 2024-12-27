import json
import os
from collections import defaultdict
from typing import Any, Dict, List, Optional, Tuple

import habitat.datasets.rearrange.samplers.receptacle as hab_receptacle
import habitat.sims.habitat_simulator.sim_utilities as sutils
import magnum as mn
import numpy as np

# NOTE: (requires habitat-llm) get metadata for semantics
from dataset_generation.benchmark_generation.generate_episodes import (
    MetadataInterface,
    default_metadata_dict,
)
from habitat.datasets.rearrange.navmesh_utils import (
    get_largest_island_index,
    unoccluded_navmesh_snap,
)
from habitat.datasets.rearrange.samplers.object_sampler import ObjectSampler
from habitat.sims.habitat_simulator.debug_visualizer import DebugVisualizer

# NOTE: (requires habitat-lab) get metadata for semantics
import habitat_sim
from habitat_sim import NavMeshSettings, Simulator
from habitat_sim.metadata import MetadataMediator
from habitat_sim.physics import ManagedArticulatedObject
from habitat_sim.utils.settings import default_sim_settings, make_cfg

rand_colors = [mn.Color4(mn.Vector3(np.random.random(3))) for _ in range(100)]


def to_str_csv(data: Any) -> str:
    """
    Format some data element as a string for csv such that it fits nicely into a cell.
    """

    if isinstance(data, str):
        return data
    if isinstance(data, (int, float)):
        return str(data)
    if isinstance(data, list):
        list_str = ""
        for elem in data:
            list_str += f"{elem};"
        return list_str

    raise NotImplementedError(f"Data type {type(data)} is not supported in csv string.")


def get_labels_from_dict(results_dict: Dict[str, Dict[str, Any]]) -> List[str]:
    """
    Get a list of column labels for the csv by scraping dict keys from the inner dict layers.
    """
    labels = []
    for scene_dict in results_dict.values():
        for dict_key in scene_dict:
            if dict_key not in labels:
                labels.append(dict_key)
    return labels


def export_results_csv(filepath: str, results_dict: Dict[str, Dict[str, Any]]) -> None:
    assert filepath.endswith(".csv")

    col_labels = get_labels_from_dict(results_dict)

    with open(filepath, "w") as f:
        # first write the column labels
        f.write("scene,")
        for c_label in col_labels:
            f.write(f"{c_label},")
        f.write("\n")

        # now a row for each scene
        for scene_handle, scene_dict in results_dict.items():
            # write the scene column
            f.write(f"{scene_handle},")
            for label in col_labels:
                if label in scene_dict:
                    f.write(f"{to_str_csv(scene_dict[label])},")
                else:
                    f.write(",")
            f.write("\n")
    print(f"Wrote results csv to {filepath}")


def check_joint_popping(
    sim: Simulator, out_dir: str = None, dbv: DebugVisualizer = None
) -> List[str]:
    """
    Get a list of ao handles for objects which are not stable during simulation.
    Checks the initial joint state, then simulates 1 second, then check the joint state again. Changes indicate popping, collisions, loose hinges, or other instability.

    :param out_dir: If provided, save debug images to the output directory prefixed "joint_pop__<handle>__".
    """

    if out_dir is not None and dbv is None:
        dbv = DebugVisualizer(sim)

    # record the ao handles
    unstable_aos = []
    # record the sum of errors across all joints
    cumulative_errors = []

    ao_initial_joint_states = {}

    for ao_handle, ao in (
        sim.get_articulated_object_manager().get_objects_by_handle_substring().items()
    ):
        ao_initial_joint_states[ao_handle] = ao.joint_positions

    sim.step_physics(2.0)

    # cumulative error must be above this threshold to count as "unstable"
    eps = 1e-3

    for ao_handle, ao in (
        sim.get_articulated_object_manager().get_objects_by_handle_substring().items()
    ):
        jp = ao.joint_positions
        if ao_initial_joint_states[ao_handle] != jp:
            cumulative_error = sum(
                [
                    abs(ao_initial_joint_states[ao_handle][i] - jp[i])
                    for i in range(len(jp))
                ]
            )
            if cumulative_error > eps:
                cumulative_errors.append(cumulative_error)
                unstable_aos.append(ao_handle)
                if out_dir is not None:
                    dbv.peek(ao_handle, peek_all_axis=True).save(
                        output_path=out_dir, prefix=f"joint_pop__{ao_handle}__"
                    )

    return unstable_aos, cumulative_errors


def draw_region_debug(sim: Simulator, region_ix: int) -> None:
    """
    Draw a wireframe for the semantic region at index region_ix.
    """
    region = sim.semantic_scene.regions[region_ix]
    color = rand_colors[region_ix]
    for edge in region.volume_edges:
        sim.get_debug_line_render().draw_transformed_line(
            edge[0],
            edge[1],
            color,
        )


def draw_all_regions_debug(sim: Simulator) -> None:
    for reg_ix in range(len(sim.semantic_scene.regions)):
        draw_region_debug(sim, reg_ix)


def save_region_visualizations(
    sim: Simulator, out_dir: str, dbv: DebugVisualizer
) -> None:
    """
    Save top-down images focused on each region with debug lines.
    """

    os.makedirs(out_dir, exist_ok=True)

    draw_all_regions_debug(sim)
    dbv.peek("stage").save(output_path=os.path.join(out_dir), prefix="all_regions_")

    for rix, region in enumerate(sim.semantic_scene.regions):
        normalized_region_id = region.id.replace("/", "|").replace(" ", "_")
        draw_region_debug(sim, rix)
        aabb = mn.Range3D.from_center(region.aabb.center, region.aabb.sizes / 2.0)
        reg_obs = dbv._peek_bb(aabb, cam_local_pos=mn.Vector3(0, 1, 0))
        reg_obs.save(
            output_path=os.path.join(out_dir), prefix=f"{normalized_region_id}_"
        )


def get_region_counts(sim: Simulator) -> Dict[str, int]:
    """
    Count all the region categories in the active scene.
    """

    region_counts = defaultdict(lambda: 0)
    for region in sim.semantic_scene.regions:
        region_counts[region.category.name()] += 1
    return region_counts


def save_region_counts_csv(region_counts: Dict[str, int], filepath: str) -> None:
    """
    Save the region counts to a csv file.
    """

    assert filepath.endswith(".csv")

    with open(filepath, "w") as f:
        f.write("region_name, count\n")
        for region_name, count in region_counts.items():
            f.write(f"{region_name}, {count}, \n")

    print(f"Wrote region counts csv to {filepath}")


def check_rec_accessibility(
    sim,
    rec: hab_receptacle.Receptacle,
    clutter_object_handles: List[str],
    max_height: float = 1.2,
    clean_up=True,
    island_index: int = -1,
) -> Tuple[bool, str]:
    """
    Use unoccluded navmesh snap to check whether a Receptacle is accessible.
    """

    assert len(clutter_object_handles) > 0

    print(f"Checking Receptacle accessibility for {rec.unique_name}")

    # first check if the receptacle is close enough to the navmesh
    rec_global_keypoints = sutils.get_global_keypoints_from_bb(
        rec.bounds, rec.get_global_transform(sim)
    )
    floor_point = None
    for keypoint in rec_global_keypoints:
        floor_point = sim.pathfinder.snap_point(keypoint, island_index=island_index)
        if not np.isnan(floor_point[0]):
            break
    if np.isnan(floor_point[0]):
        print(" - Receptacle too far from active navmesh boundary.")
        return False, "access_filtered"

    # then check that the height is acceptable
    rec_min = min(rec_global_keypoints, key=lambda x: x[1])
    if rec_min[1] - floor_point[1] > max_height:
        print(
            f" - Receptacle exceeds maximum height {rec_min[1]-floor_point[1]} vs {max_height}."
        )
        return False, "height_filtered"

    # try to sample 10 objects on the receptacle
    target_number = 10
    obj_samp = ObjectSampler(
        clutter_object_handles,
        ["rec set"],
        orientation_sample="up",
        num_objects=(1, target_number),
    )
    obj_samp.max_sample_attempts = len(clutter_object_handles)
    obj_samp.max_placement_attempts = 10
    obj_samp.target_objects_number = target_number
    rec_set_unique_names = [rec.unique_name]
    rec_set_obj = hab_receptacle.ReceptacleSet(
        "rec set", [""], [], rec_set_unique_names, []
    )
    recep_tracker = hab_receptacle.ReceptacleTracker(
        {},
        {"rec set": rec_set_obj},
    )

    new_objs = []
    try:
        new_objs = obj_samp.sample(sim, recep_tracker, [], snap_down=True)
    except Exception as e:
        print(f" - generation failed with internal exception {repr(e)}")

    # if we can't sample objects, this receptacle is out
    if len(new_objs) == 0:
        print(" - failed to sample any objects.")
        return False, "access_filtered"
    print(f" - sampled {len(new_objs)} / {target_number} objects.")

    # now try unoccluded navmesh snapping to the objects to test accessibility
    obj_positions = [obj.translation for obj, _ in new_objs]
    for obj, _ in new_objs:
        obj.translation += mn.Vector3(100, 0, 0)
    failure_count = 0

    for o_ix, (obj, _) in enumerate(new_objs):
        obj.translation = obj_positions[o_ix]
        snap_point = unoccluded_navmesh_snap(
            obj.translation, 1.3, sim.pathfinder, sim, obj.object_id, island_index
        )
        # self.dbv.look_at(look_at=obj.translation, look_from=snap_point)
        # self.dbv.get_observation().show()
        if snap_point is None:
            failure_count += 1
        obj.translation += mn.Vector3(100, 0, 0)
    for o_ix, (obj, _) in enumerate(new_objs):
        obj.translation = obj_positions[o_ix]
    failure_rate = (float(failure_count) / len(new_objs)) * 100
    print(f" - failure_rate = {failure_rate}")
    print(
        f" - accessibility rate = {len(new_objs)-failure_count}|{len(new_objs)} ({100-failure_rate}%)"
    )

    accessible = failure_rate < 20  # 80% accessibility required

    if clean_up:
        # removing all clutter objects currently
        rom = sim.get_rigid_object_manager()
        for obj, _ in new_objs:
            rom.remove_object_by_handle(obj.handle)

    if not accessible:
        return False, "access_filtered"

    return True, "active"


def init_rec_filter_data_dict() -> Dict[str, Any]:
    """
    Get an empty rec_filter_data dictionary.
    """
    return {
        "active": [],
        "manually_filtered": [],
        "access_filtered": [],
        "access_threshold": -1,  # set in filter procedure
        "stability_filtered": [],
        "stability threshold": -1,  # set in filter procedure
        "height_filtered": [],
        "max_height": 1.2,
        "min_height": 0,
    }


def write_rec_filter_json(filepath: str, json_dict: Dict[str, Any]) -> None:
    """
    Write the receptacle filter json dict.
    """

    assert filepath.endswith(".json")
    os.makedirs(os.path.dirname(filepath), exist_ok=True)
    with open(filepath, "w") as f:
        f.write(json.dumps(json_dict, indent=2))


def set_filter_status_for_rec(
    rec: hab_receptacle.Receptacle,
    filter_status: str,
    rec_filter_data: Dict[str, Any],
    ignore_existing_status: Optional[List[str]] = None,
) -> None:
    """
    Set the filter status of a Receptacle in the filter dictionary.

    :param rec: The Receptacle instance.
    :param filter_status: The status to assign.
    :param rec_filter_data: The current filter dictionary to modify.
    :param ignore_existing_status: An optional list of filter types to lock, preventing re-assignment.
    """

    if ignore_existing_status is None:
        ignore_existing_status = []
    filter_types = [
        "access_filtered",
        "stability_filtered",
        "height_filtered",
        "manually_filtered",
        "active",
    ]
    assert filter_status in filter_types
    filtered_rec_name = rec.unique_name
    for filter_type in filter_types:
        if filtered_rec_name in rec_filter_data[filter_type]:
            if filter_type in ignore_existing_status:
                print(
                    f"Trying to assign filter status {filter_status} but existing status {filter_type} in ignore list. Aborting assignment."
                )
                return
            else:
                rec_filter_data[filter_type].remove(filtered_rec_name)
    rec_filter_data[filter_status].append(filtered_rec_name)


def navmesh_config_and_recompute(sim) -> None:
    """
    Re-compute the navmesh with specific settings.
    """

    navmesh_settings = NavMeshSettings()
    navmesh_settings.set_defaults()
    navmesh_settings.agent_height = 1.3  # spot
    navmesh_settings.agent_radius = 0.3  # human || spot
    navmesh_settings.include_static_objects = True
    sim.recompute_navmesh(
        sim.pathfinder,
        navmesh_settings,
    )


def initialize_clutter_object_set(sim) -> None:
    """
    Get the template handles for configured clutter objects.
    """
    clutter_object_set = [
        "002_master_chef_can",
        "003_cracker_box",
        "004_sugar_box",
        "005_tomato_soup_can",
        "007_tuna_fish_can",
        "008_pudding_box",
        "009_gelatin_box",
        "010_potted_meat_can",
        "024_bowl",
    ]
    clutter_object_handles = []
    for obj_name in clutter_object_set:
        matching_handles = (
            sim.metadata_mediator.object_template_manager.get_template_handles(obj_name)
        )
        assert (
            len(matching_handles) > 0
        ), f"No matching template for '{obj_name}' in the dataset."
        clutter_object_handles.append(matching_handles[0])
    return clutter_object_handles


def run_rec_filter_analysis(
    sim, out_dir: str, open_default_links: bool = True, keep_manual_filters: bool = True
) -> None:
    """
    Collect all receptacles for the scene and run an accessibility check, saving the resulting filter file.

    :param out_dir: Where to write the filter files.
    :param open_default_links: Whether or not to open default links when considering final accessible Receptacles set.
    :param keep_manual_filters: Whether to keep or override existing manual filter definitions.
    """

    rec_filter_dict = init_rec_filter_data_dict()

    # load the clutter objects
    sim.metadata_mediator.object_template_manager.load_configs(
        "data/objects/ycb/configs/"
    )
    clutter_object_handles = initialize_clutter_object_set(sim)

    # recompute the navmesh with expect parameters
    navmesh_config_and_recompute(sim)

    # get the largest indoor island
    largest_island = get_largest_island_index(sim.pathfinder, sim, allow_outdoor=False)

    # keep manually filtered receptacles
    ignore_existing_status = []
    if keep_manual_filters:
        existing_scene_filter_file = hab_receptacle.get_scene_rec_filter_filepath(
            sim.metadata_mediator, sim.curr_scene_name
        )
        if existing_scene_filter_file is not None:
            filter_strings = hab_receptacle.get_excluded_recs_from_filter_file(
                existing_scene_filter_file, filter_types=["manually_filtered"]
            )
            rec_filter_dict["manually_filtered"] = filter_strings
        ignore_existing_status.append("manually_filtered")

    recs = hab_receptacle.find_receptacles(
        sim, exclude_filter_strings=rec_filter_dict["manually_filtered"]
    )

    # compute a map from parent object to Receptacles
    parent_handle_to_rec: Dict[str, List[hab_receptacle.Receptacle]] = defaultdict(
        lambda: []
    )
    for rec in recs:
        parent_handle_to_rec[rec.parent_object_handle].append(rec)

    # compute the default accessibility with all closed links
    default_active_set: List[hab_receptacle.Receptacle] = []
    for rix, rec in enumerate(recs):
        rec_accessible, filter_type = check_rec_accessibility(
            sim, rec, clutter_object_handles, island_index=largest_island
        )
        if rec_accessible:
            default_active_set.append(rec)
        set_filter_status_for_rec(
            rec,
            filter_type,
            rec_filter_dict,
            ignore_existing_status=ignore_existing_status,
        )
        print(f"-- progress = {rix}/{len(recs)} --")

    # open default links and re-compute accessibility for each AO
    # the difference between default state accessibility and open state accessibility defines the "within_set"
    within_set: List[hab_receptacle.Receptacle] = []
    if open_default_links:
        all_objects = sutils.get_all_objects(sim)
        aos = [obj for obj in all_objects if isinstance(obj, ManagedArticulatedObject)]
        for aoix, ao in enumerate(aos):
            default_link = sutils.get_ao_default_link(ao, True)
            if default_link is not None:
                sutils.open_link(ao, default_link)
                # recompute accessibility
                for child_rec in parent_handle_to_rec[ao.handle]:
                    rec_accessible, filter_type = check_rec_accessibility(
                        sim,
                        child_rec,
                        clutter_object_handles,
                        island_index=largest_island,
                    )
                    if rec_accessible and child_rec not in default_active_set:
                        # found a Receptacle which is only accessible when the default_link is open
                        within_set.append(child_rec)
                        set_filter_status_for_rec(
                            child_rec,
                            filter_type,
                            rec_filter_dict,
                            ignore_existing_status=ignore_existing_status,
                        )
                sutils.close_link(ao, default_link)
            print(f"-- progress = {aoix}/{len(aos)} --")

    # write the within set to the filter file
    rec_filter_dict["within_set"] = [
        within_rec.unique_name for within_rec in within_set
    ]

    # write the filter file to JSON
    filter_filepath = os.path.join(
        out_dir, f"scene_filter_files/{sim.curr_scene_name}.rec_filter.json"
    )
    write_rec_filter_json(filter_filepath, rec_filter_dict)


def get_global_faucet_points(sim: habitat_sim.Simulator) -> Dict[str, List[mn.Vector3]]:
    """
    Gets a global set of points identifying faucets for each object in the scene.
    Returns a dict mapping object handles to global points.
    """
    objs = sutils.get_all_objects(sim)
    obj_markersets = {}
    for obj in objs:
        all_obj_marker_sets = obj.marker_sets
        if all_obj_marker_sets.has_taskset("faucets"):
            # this object has faucet annotations
            obj_markersets[obj.handle] = []
            faucet_marker_sets = all_obj_marker_sets.get_taskset_points("faucets")
            for link_name, link_faucet_markers in faucet_marker_sets.items():
                link_id = -1
                if link_name != "root":
                    link_id = obj.get_link_id_from_name(link_name)
                for _marker_subset_name, points in link_faucet_markers.items():
                    global_points = obj.transform_local_pts_to_world(points, link_id)
                    obj_markersets[obj.handle].extend(global_points)
    return obj_markersets


def draw_global_point_set(global_points: List[mn.Vector3], debug_line_render):
    for point in global_points:
        debug_line_render.draw_circle(
            translation=point,
            radius=0.02,
            normal=mn.Vector3(0, 1, 0),
            color=mn.Color4.red(),
            num_segments=12,
        )


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
        "--out-dir",
        default="siro_test_results/",
        type=str,
        help="directory in which to cache images and results csv.",
    )
    parser.add_argument(
        "--save-images",
        default=False,
        action="store_true",
        help="save images during tests into the output directory.",
    )
    parser.add_argument(
        "--actions",
        nargs="+",
        type=str,
        help="A set of strings indicating check actions to be performed on the dataset.",
        default=None,
    )
    args = parser.parse_args()

    available_check_actions = [
        "faucet_points",
        "rec_unique_names",
        "rec_filters",
        "region_counts",
        "joint_popping",
        "visualize_regions",
        "analyze_semantics",
    ]

    target_check_actions = []

    assert args.actions is not None, "Must select and action."

    for target_action in args.actions:
        assert (
            target_action in available_check_actions
        ), f"provided action {target_action} is not in the valid set: {available_check_actions}"

    target_check_actions = args.actions

    os.makedirs(args.out_dir, exist_ok=True)

    # create an initial simulator config
    sim_settings: Dict[str, Any] = default_sim_settings
    sim_settings["scene_dataset_config_file"] = args.dataset
    cfg = make_cfg(sim_settings)

    # pre-initialize a MetadataMediator to iterate over scenes
    mm = MetadataMediator()
    mm.active_dataset = args.dataset
    cfg.metadata_mediator = mm

    mi = MetadataInterface(default_metadata_dict)

    # keyed by scene handle
    scene_test_results: Dict[str, Dict[str, Any]] = {}

    # count all region category names in all scenes
    region_counts: Dict[str, int] = defaultdict(lambda: 0)

    num_scenes = len(mm.get_scene_handles())

    # for each scene, initialize a fresh simulator and run tests
    for s_ix, scene_handle in enumerate(mm.get_scene_handles()):
        print("=================================================================")
        print(
            f"Setting up scene for {scene_handle} ({s_ix}|{num_scenes} = {s_ix/float(num_scenes)*100}%)"
        )
        cfg.sim_cfg.scene_id = scene_handle
        print(" - init")
        with Simulator(cfg) as sim:
            dbv = DebugVisualizer(sim)

            mi.refresh_scene_caches(sim)

            scene_test_results[sim.curr_scene_name] = {}
            scene_test_results[sim.curr_scene_name][
                "ros"
            ] = sim.get_rigid_object_manager().get_num_objects()
            scene_test_results[sim.curr_scene_name][
                "aos"
            ] = sim.get_articulated_object_manager().get_num_objects()

            scene_out_dir = os.path.join(args.out_dir, f"{sim.curr_scene_name}/")

            ##########################################
            # get images of all the global faucet points in the scene
            if "faucet_points" in target_check_actions:
                scene_obj_global_faucet_points = get_global_faucet_points(sim)
                for obj_handle, global_points in scene_obj_global_faucet_points.items():
                    circles = [
                        (point, 0.1, mn.Vector3(0, 1, 0), mn.Color4.red())
                        for point in global_points
                    ]
                    dbv.peek(
                        obj_handle, peek_all_axis=True, debug_circles=circles
                    ).save(scene_out_dir, f"faucets_{obj_handle}_")

            ##########################################
            # gather all Receptacle.unique_name in the scene
            if "rec_unique_names" in target_check_actions:
                all_recs = hab_receptacle.find_receptacles(sim)
                unique_names = [rec.unique_name for rec in all_recs]
                scene_test_results[sim.curr_scene_name][
                    "rec_unique_names"
                ] = unique_names

            ##########################################
            # receptacle filter computation
            if "rec_filters" in target_check_actions:
                run_rec_filter_analysis(
                    sim, args.out_dir, open_default_links=True, keep_manual_filters=True
                )

            ##########################################
            # Check region counts
            if "region_counts" in target_check_actions:
                print(" - region counts")
                scene_region_counts = get_region_counts(sim)
                for region_name, count in scene_region_counts.items():
                    region_counts[region_name] += count

            ##########################################
            # Check for joint popping
            if "joint_popping" in target_check_actions:
                print(" - check joint popping")
                unstable_aos, joint_errors = check_joint_popping(
                    sim, out_dir=scene_out_dir if args.save_images else None, dbv=dbv
                )
                if len(unstable_aos) > 0:
                    scene_test_results[sim.curr_scene_name]["unstable_aos"] = ""
                    for ix, ao_handle in enumerate(unstable_aos):
                        scene_test_results[sim.curr_scene_name][
                            "unstable_aos"
                        ] += f"{ao_handle}({joint_errors[ix]}) | "

            ############################################
            # analyze and visualize regions
            if "visualize_regions" in target_check_actions:
                print(" - check and visualize regions")
                if args.save_images:
                    save_region_visualizations(
                        sim, os.path.join(scene_out_dir, "regions/"), dbv
                    )
                expected_regions = ["kitchen", "living room", "bedroom"]
                all_region_cats = [
                    region.category.name() for region in sim.semantic_scene.regions
                ]
                missing_expected_regions = [
                    expected_region
                    for expected_region in expected_regions
                    if expected_region not in all_region_cats
                ]
                if len(missing_expected_regions) > 0:
                    scene_test_results[sim.curr_scene_name][
                        "missing_expected_regions"
                    ] = ""
                    for expected_region in missing_expected_regions:
                        scene_test_results[sim.curr_scene_name][
                            "missing_expected_regions"
                        ] += f"{expected_region} | "

            ##############################################
            # analyze semantics
            if "analyze_semantics" in target_check_actions:
                print(" - check and visualize semantics")
                scene_test_results[sim.curr_scene_name][
                    "objects_missing_semantic_class"
                ] = []
                missing_semantics_output = os.path.join(
                    scene_out_dir, "missing_semantics/"
                )
                for obj in sutils.get_all_objects(sim):
                    if mi.get_object_instance_category(obj) is None:
                        scene_test_results[sim.curr_scene_name][
                            "objects_missing_semantic_class"
                        ].append(obj.handle)
                        if args.save_images:
                            os.makedirs(missing_semantics_output, exist_ok=True)
                            dbv.peek(obj, peek_all_axis=True).save(
                                missing_semantics_output, f"{obj.handle}__"
                            )

    csv_filepath = os.path.join(args.out_dir, "siro_scene_test_results.csv")
    export_results_csv(csv_filepath, scene_test_results)
    if "region_counts" in target_check_actions:
        region_count_csv_filepath = os.path.join(args.out_dir, "region_counts.csv")
        save_region_counts_csv(region_counts, region_count_csv_filepath)
