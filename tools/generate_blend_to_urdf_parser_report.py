# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import argparse
import csv
import os
from typing import Any, Callable, Dict, List


def file_endswith(filepath: str, end_str: str) -> bool:
    """
    Return whether or not the file ends with a string.
    """
    return filepath.endswith(end_str)


def find_files(
    root_dir: str, discriminator: Callable[[str, str], bool], disc_str: str
) -> List[str]:
    """
    Recursively find all filepaths under a root directory satisfying a particular constraint as defined by a discriminator function.

    :param root_dir: The roor directory for the recursive search.
    :param discriminator: The discriminator function which takes a filepath and discriminator string and returns a bool.

    :return: The list of all absolute filepaths found satisfying the discriminator.
    """
    filepaths: List[str] = []

    if not os.path.exists(root_dir):
        print(" Directory does not exist: " + str(root_dir))
        return filepaths

    for entry in os.listdir(root_dir):
        entry_path = os.path.join(root_dir, entry)
        if os.path.isdir(entry_path):
            sub_dir_filepaths = find_files(entry_path, discriminator, disc_str)
            filepaths.extend(sub_dir_filepaths)
        # apply a user-provided discriminator function to cull filepaths
        elif discriminator(entry_path, disc_str):
            filepaths.append(entry_path)
    return filepaths


def find_subdirectory_names(root_dir: str) -> List[str]:
    """
    Lists all immediate child directories for a provided root directory.
    """
    assert os.path.exists(root_dir)

    dirpaths = []

    for entry in os.listdir(root_dir):
        entry_path = os.path.join(root_dir, entry)
        if os.path.isdir(entry_path):
            dirpaths.append(entry)

    return dirpaths


def load_model_list_from_csv(
    filepath: str, header_label: str = "Model ID"
) -> List[str]:
    """
    Scrape a csv file for a list of model ids under a header label.
    """
    assert filepath.endswith(".csv"), "This isn't a .csv file."
    assert os.path.exists(filepath)
    ids = []

    with open(filepath, newline="") as f:
        reader = csv.reader(f)
        labels = []
        id_column = None
        for rix, row in enumerate(reader):
            if rix == 0:
                labels = row
                id_column = labels.index(header_label)
            else:
                # allow empty cells to keep consistency with row ordering in the sheet (for copy/paste)
                ids.append(row[id_column])

    return ids


# Print iterations progress
# NOTE: copied from https://stackoverflow.com/questions/3173320/text-progress-bar-in-terminal-with-block-characters
def printProgressBar(
    iteration,
    total,
    prefix="",
    suffix="",
    decimals=1,
    length=100,
    fill="█",
    printEnd="\r",
):
    """
    Call in a loop to create terminal progress bar
    @params:
        iteration   - Required  : current iteration (Int)
        total       - Required  : total iterations (Int)
        prefix      - Optional  : prefix string (Str)
        suffix      - Optional  : suffix string (Str)
        decimals    - Optional  : positive number of decimals in percent complete (Int)
        length      - Optional  : character length of bar (Int)
        fill        - Optional  : bar fill character (Str)
        printEnd    - Optional  : end character (e.g. "\r", "\r\n") (Str)
    """
    percent = ("{0:." + str(decimals) + "f}").format(100 * (iteration / float(total)))
    filledLength = int(length * iteration // total)
    bar = fill * filledLength + "-" * (length - filledLength)
    print(f"\r{prefix} |{bar}| {percent}% {suffix}", end=printEnd)
    # Print New Line on Complete
    if iteration == total:
        print()


# -----------------------------------------
# Generates a report checking the success of blend to urdf parsing batches.
# e.g. python tools/generate_blend_to_urdf_parser_report.py --root-dir <urdf_outputs_root_directory> --report-model-list <path-to>/all_scenes_artic_models-M1.csv
# e.g. add " --report-filepath <your_filepath" to designate an alternative csv output.
# -----------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="Generate a parser report csv checking for expeted output from a batched conversion process."
    )
    parser.add_argument(
        "--root-dir",
        type=str,
        help="Path to a root directory containing sub-directories with exported urdf, config, and asset files from the conversion.",
    )
    parser.add_argument(
        "--report-filepath",
        type=str,
        default=None,
        help="Path to export the report csv. If not provided, default to <root-dir>/parser_report.csv",
    )
    parser.add_argument(
        "--report-model-list",
        type=str,
        default=None,
        help="Path to a csv file with a single column of model hashes. If provided, the generated report will follow the same model sequence. E.g. for export into a spreadsheet. By default, expects each sub-directory from root-dir to correspond to a converted model.",
    )
    parser.add_argument(
        "--do-simulator-load-test",
        action="store_true",
        help="If set, try to load the models in a Simulator to validate them. This is significantly slower than checking asset metadata.",
    )
    parser.add_argument(
        "--reconfigure-period",
        type=int,
        default=10,
        help="Only used with 'do-simulator-load-test'. Sets the number of AOs to load in each batch before reconfiguring the simulator to release asset memory.",
    )

    args = parser.parse_args()
    root_dir = args.root_dir
    assert os.path.isdir(root_dir), "directory must exist."

    # maps models to booleans for each category
    parse_results_report: Dict[str, List[bool]] = {}
    cat_columns = [
        "model_id",
        "folder",
        "urdf",
        "config",
        "receptacles",
        "render_meshes",
        "load_test",
    ]
    # collect global sums of each error type
    global_count: Dict[str, int] = {}
    for check in cat_columns[1:]:
        global_count[check] = 0

    report_filepath = os.path.join(args.root_dir + "parser_report.csv")
    if args.report_filepath is not None:
        assert args.report_filepath.endswith(".csv"), "Must be a csv file."
        assert len(args.report_filepath) > 4, "Must provided more than the filetype."
        report_filepath = args.report_filepath

    # scrape all existing subdirectories
    exported_folder_names = find_subdirectory_names(root_dir=root_dir)
    exported_folder_claimed = [False for exported_folder_name in exported_folder_names]

    # get model ids list
    model_ids = exported_folder_names
    if args.report_model_list is not None:
        model_ids = load_model_list_from_csv(filepath=args.report_model_list)

    sim = None
    aom = None
    aoam = None
    sim_cfg = None
    if args.do_simulator_load_test:
        # create a Simulator
        from habitat_sim import Simulator
        from habitat_sim.utils.settings import default_sim_settings, make_cfg

        sim_settings: Dict[str, Any] = default_sim_settings
        sim_cfg = make_cfg(sim_settings)
        sim = Simulator(sim_cfg)
        aom = sim.get_articulated_object_manager()
        aoam = sim.metadata_mediator.ao_template_manager
        assert sim

    # start the progress bar in terminal
    num_models = len(model_ids)
    printProgressBar(0, num_models, prefix="Progress:", suffix="Complete", length=50)

    # for each model ids, check for existance of each expected output
    for mix, model_id in enumerate(model_ids):
        if mix % args.reconfigure_period == 0 and sim is not None:
            # reconfigure the simulator at a set frequency to release asset memory
            sim.close()
            sim = Simulator(sim_cfg)
            aom = sim.get_articulated_object_manager()
            aoam = sim.metadata_mediator.ao_template_manager

        printProgressBar(
            mix, num_models, prefix="Progress:", suffix="Complete", length=50
        )
        folder_path = os.path.join(root_dir, model_id)
        folder_exists = False
        if model_id in exported_folder_names:
            folder_exists = True
        # NOTE: silly override to
        elif model_id + ".glb" in exported_folder_names:
            folder_path = folder_path + ".glb"
            folder_exists = True

        if folder_exists:
            exported_folder_claimed[
                exported_folder_names.index(folder_path.split("/")[-1])
            ] = True

            urdf_matches = find_files(folder_path, file_endswith, ".urdf")
            urdf_exists = len(urdf_matches) > 0

            config_matches = find_files(folder_path, file_endswith, ".ao_config.json")
            config_exists = len(config_matches) > 0

            # NOTE: there could be missing assets here, but without parsing the blend file again, we wouldn't know. Heuristic is to expect at least one.
            num_rec_meshes = len(
                find_files(folder_path, file_endswith, "_receptacle_mesh.glb")
            )
            one_receptacle_exists = num_rec_meshes > 0

            one_render_mesh_exists = (
                len(find_files(folder_path, file_endswith, ".glb")) - num_rec_meshes
            ) > 0

            load_test = False
            if sim is not None and urdf_exists and one_render_mesh_exists:
                # minimum requirements to try loading an asset
                if config_exists:
                    # load from config
                    try:
                        # try to load the config
                        new_template_ids = aoam.load_configs(config_matches[0])
                        # try to load the asset from config
                        ao = aom.add_articulated_object_by_template_id(
                            new_template_ids[0]
                        )
                        load_test = True
                        aom.remove_object_by_id(ao.object_id)
                    except Exception as e:
                        print(
                            f"Failed to load asset from config {config_matches[0]}. '{repr(e)}'"
                        )
                        repr(e)
                else:
                    # load from URDF
                    try:
                        ao = aom.add_articulated_object_from_urdf(urdf_matches[0])
                        load_test = True
                        aom.remove_object_by_id(ao.object_id)
                    except Exception as e:
                        print(
                            f"Failed to load asset from urdf {urdf_matches[0]}. '{repr(e)}'"
                        )
                        repr(e)

            parse_results_report[model_id] = [
                model_id,
                folder_exists,
                urdf_exists,
                config_exists,
                one_receptacle_exists,
                one_render_mesh_exists,
                load_test,
            ]
            global_count["folder"] += int(not folder_exists)
            global_count["config"] += int(not config_exists)
            global_count["receptacles"] += int(not one_receptacle_exists)
            global_count["urdf"] += int(not urdf_exists)
            global_count["render_meshes"] += int(not one_render_mesh_exists)
            global_count["load_test"] += int(not load_test)

        else:
            parse_results_report[model_id] = [False for i in range(len(cat_columns))]
            parse_results_report[model_id][0] = model_id
            for key in global_count:
                global_count[key] += 1

    # export results to a file
    os.makedirs(os.path.dirname(report_filepath), exist_ok=True)
    with open(report_filepath, "w", newline="") as f:
        writer = csv.writer(f, delimiter=",", quotechar="|", quoting=csv.QUOTE_MINIMAL)
        # write the header labels
        writer.writerow(cat_columns)
        # write the contents
        for model_id in model_ids:
            writer.writerow(parse_results_report[model_id])

    print("-----------------------------------------------")
    print(f"Wrote report to {report_filepath}.\n")

    unclaimed_folders = [
        exported_folder_names[folder_index]
        for folder_index, claimed in enumerate(exported_folder_claimed)
        if not claimed
    ]
    if len(unclaimed_folders) > 0:
        print(
            f"The following folders were unclaimed. Likely the root node is misnamed: {unclaimed_folders}"
        )
    print("-----------------------------------------------")

    print(f"global_counts = {global_count}")


if __name__ == "__main__":
    main()
