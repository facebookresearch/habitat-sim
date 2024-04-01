import os
from typing import Any, Dict, List

# NOTE: (requires habitat-lab) get metadata for semantics
import habitat.sims.habitat_simulator.sim_utilities as sutils
import magnum as mn
import numpy as np

# NOTE: (requires habitat-llm) get metadata for semantics
from dataset_generation.benchmark_generation.generate_episodes import (
    MetadataInterface,
    default_metadata_dict,
)
from habitat.sims.habitat_simulator.debug_visualizer import DebugVisualizer

from habitat_sim import Simulator
from habitat_sim.metadata import MetadataMediator
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
            list_str += f"{elem} |"
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
    args = parser.parse_args()

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

    # for each scene, initialize a fresh simulator and run tests
    for scene_handle in mm.get_scene_handles():
        print(f"Setting up scene for {scene_handle}")
        cfg.sim_cfg.scene_id = scene_handle
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
            # Check for joint popping
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
            print(" - check and visualize regions")
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
                scene_test_results[sim.curr_scene_name]["missing_expected_regions"] = ""
                for expected_region in missing_expected_regions:
                    scene_test_results[sim.curr_scene_name][
                        "missing_expected_regions"
                    ] += f"{expected_region} | "

            ##############################################
            # analyze semantics
            print(" - check and visualize semantics")
            scene_test_results[sim.curr_scene_name][
                "objects_missing_semantic_class"
            ] = []
            missing_semantics_output = os.path.join(scene_out_dir, "missing_semantics/")
            for obj in sutils.get_all_objects(sim):
                if mi.get_object_instance_category(obj) is None:
                    scene_test_results[sim.curr_scene_name][
                        "objects_missing_semantic_class"
                    ].append(obj.handle)
                    os.makedirs(missing_semantics_output, exist_ok=True)
                    dbv.peek(obj, peek_all_axis=True).save(
                        missing_semantics_output, f"{obj.handle}__"
                    )

    csv_filepath = os.path.join(args.out_dir, "siro_scene_test_results.csv")
    export_results_csv(csv_filepath, scene_test_results)
