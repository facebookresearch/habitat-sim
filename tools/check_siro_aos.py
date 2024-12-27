import os
from typing import Any, Dict, List

# NOTE: (requires habitat-lab) get metadata for semantics
import magnum as mn
import numpy as np

# NOTE: (requires habitat-llm) get metadata for semantics
from dataset_generation.benchmark_generation.generate_episodes import (
    MetadataInterface,
    default_metadata_dict,
    object_hash_from_handle,
)
from habitat.datasets.rearrange.samplers.receptacle import find_receptacles
from habitat.sims.habitat_simulator.debug_visualizer import DebugVisualizer

from habitat_sim import Simulator
from habitat_sim.metadata import MetadataMediator
from habitat_sim.physics import ManagedArticulatedObject
from habitat_sim.utils.settings import default_sim_settings, make_cfg

rand_colors = [mn.Color4(mn.Vector3(np.random.random(3))) for _ in range(100)]


def to_str_csv(data: Any) -> str:
    """
    Format some data element as a string for csv such that it fits nicely into a cell.
    """
    if data is None:
        return "None"
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
    for ao_dict in results_dict.values():
        for dict_key in ao_dict:
            if dict_key not in labels:
                labels.append(dict_key)
    return labels


def export_results_csv(filepath: str, results_dict: Dict[str, Dict[str, Any]]) -> None:
    assert filepath.endswith(".csv")

    col_labels = get_labels_from_dict(results_dict)

    with open(filepath, "w") as f:
        # first write the column labels
        f.write("ao,")
        for c_label in col_labels:
            f.write(f"{c_label},")
        f.write("\n")

        # now a row for each scene
        for ao_handle, ao_dict in results_dict.items():
            # write the ao handle column
            f.write(f"{ao_handle},")
            for label in col_labels:
                if label in ao_dict:
                    f.write(f"{to_str_csv(ao_dict[label])},")
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


def recompute_ao_bbs(ao: ManagedArticulatedObject) -> None:
    """
    Recomputes the link SceneNode bounding boxes for all ao links.
    NOTE: Gets around an observed loading bug. Call before trying to peek an AO.
    """
    for link_ix in range(-1, ao.num_links):
        link_node = ao.get_link_scene_node(link_ix)
        link_node.compute_cumulative_bb()


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

    # keyed by ao handle
    ao_test_results: Dict[str, Dict[str, Any]] = {}

    ao_ix = 0
    # split up the load per-simulator reconfigure to balance memory overhead with init time
    iters_per_sim = 50
    ao_handles = mm.ao_template_manager.get_template_handles()
    while ao_ix < len(ao_handles):
        with Simulator(cfg) as sim:
            dbv = DebugVisualizer(sim)
            aom = sim.get_articulated_object_manager()

            for _i in range(iters_per_sim):
                if ao_ix >= len(ao_handles):
                    # early escape if done
                    break

                ao_handle = ao_handles[ao_ix]
                ao_short_handle = ao_handle.split("/")[-1].split(".")[0]
                ao_ix += 1
                ao_test_results[ao_short_handle] = {}
                asset_failure_message = None
                ao = None

                # first try to load the asset
                try:
                    ao = aom.add_articulated_object_by_template_handle(ao_handle)
                except Exception as e:
                    print(f"Failed to load asset {ao_handle}. '{repr(e)}'")
                    asset_failure_message = repr(e)

                if ao is None:
                    # load failed, record the message and continue
                    ao_test_results[ao_short_handle]["failure_log"] = to_str_csv(
                        asset_failure_message
                    )
                    continue

                # check joint popping
                unstable_aos, joint_errors = check_joint_popping(
                    sim, out_dir=args.out_dir if args.save_images else None, dbv=dbv
                )

                if len(unstable_aos) > 0:
                    ao_test_results[ao_short_handle][
                        "joint_popping_error"
                    ] = joint_errors[0]

                ###########################################
                # produce a gif of actuation
                # TODO:

                ###########################################
                # load the receptacles
                try:
                    recs = find_receptacles(sim)
                except Exception as e:
                    print(f"Failed to load receptacles for {ao_handle}. '{repr(e)}'")
                    asset_failure_message = repr(e)
                    ao_test_results[ao_short_handle]["failure_log"] = to_str_csv(
                        asset_failure_message
                    )

                ###########################################
                # snap an image and sort into category subfolder
                recompute_ao_bbs(ao)
                hash_name = object_hash_from_handle(ao_handle)
                cat = mi.get_object_category(hash_name)
                if cat is None:
                    cat = "None"

                ao_peek = dbv.peek(ao.handle, peek_all_axis=True)
                cat_dir = os.path.join(args.out_dir, f"ao_categories/{cat}/")
                os.makedirs(cat_dir, exist_ok=True)
                ao_peek.save(cat_dir, prefix=hash_name + "__")

                #############################################
                # DONE: clear the scene for next iteration
                aom.remove_all_objects()

        # check if done with last ao
        if ao_ix >= len(ao_handles):
            break

    csv_filepath = os.path.join(args.out_dir, "siro_ao_test_results.csv")
    export_results_csv(csv_filepath, ao_test_results)
