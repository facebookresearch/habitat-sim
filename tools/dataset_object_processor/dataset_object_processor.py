import argparse
import csv
import datetime
import json
import os
from enum import Enum
from typing import Any, Dict, List, Optional

import git
import magnum as mn
import psutil
from colorama import Fore, init
from magnum import trade
from processor_settings import default_sim_settings, make_cfg

import habitat_sim
from habitat_sim import attributes, attributes_managers, physics

repo = git.Repo(".", search_parent_directories=True)
HABITAT_SIM_PATH = repo.working_tree_dir

# A line of dashes to divide sections of terminal output
divider: str = "\n" + "-" * 72

# TODO: possibly move to separate utils file
class CSVWriter:
    """
    Generalized utility to write csv files
    """

    file_path = None

    def create_unique_filename(
        csv_dir_path: str = None, filename_prefix: str = None
    ) -> str:
        """
        Create unique file name / file path for the next csv we write
        based off of the current date and time. Also create directory
        in which we save csv files if one doesn't already exist
        """
        # Current date and time so we can make unique file names for each csv
        date_and_time = datetime.datetime.now()

        # year-month-day
        date = date_and_time.strftime("%Y-%m-%d")

        # hour:min:sec - capital H is military time, %I is standard time
        # (am/pm time format)
        time = date_and_time.strftime("%H:%M:%S")

        # make directory to store csvs if it doesn't exist
        if csv_dir_path is None:
            raise RuntimeError(
                "CSVWriter.create_unique_filename: must provide a directory to save CSV file."
            )
        if not os.path.exists(csv_dir_path):
            os.makedirs(csv_dir_path)

        # create csv file name (TODO: make more descriptive file name)
        if filename_prefix is None or filename_prefix == "":
            filename_prefix = ""
        else:
            filename_prefix = f"{filename_prefix}__"
        CSVWriter.file_path = (
            f"{csv_dir_path}{filename_prefix}date_{date}__time_{time}.csv"
        )
        return CSVWriter.file_path

    def write_file(
        headers: List[str],
        csv_rows: List[List[str]],
        file_path: str = None,
    ) -> None:
        """
        Write column titles and csv data into csv file with the provided
        file path. Use default file path if none is provided
        """
        if file_path is None:
            if CSVWriter.file_path is None:
                raise RuntimeError(
                    "CSVWriter.write_file: must provide a file path to save CSV file."
                )
            else:
                file_path = CSVWriter.file_path

        if not len(csv_rows[0]) == len(headers):
            raise RuntimeError(
                "Number of headers does not equal number of columns in CSVWriter.write_file()."
            )

        with open(file_path, "w") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(headers)
            writer.writerows(csv_rows)


# TODO: possibly move to separate utils file
class MemoryUnitConverter:
    """
    class to convert computer memory value units, i.e.
    1,024 bytes to 1 kilobyte, or (1 << 10) bytes
    1,048,576 bytes to 1 megabyte, or (1 << 20) bytes
    1,073,741,824 bytes to 1 gigabyte, or (1 << 30) bytes
    """

    BYTES = 0
    KILOBYTES = 1
    MEGABYTES = 2
    GIGABYTES = 3

    UNIT_STRS = ["bytes", "KB", "MB", "GB"]
    UNIT_CONVERSIONS = [1, 1 << 10, 1 << 20, 1 << 30]


# TODO: possibly move to separate utils file
class ANSICodes(Enum):
    """
    Terminal printing ANSI color and format codes
    """

    HEADER = "\033[95m"
    BROWN = "\033[38;5;130m"
    ORANGE = "\033[38;5;202m"
    YELLOW = "\033[38;5;220m"
    PURPLE = "\033[38;5;177m"
    BRIGHT_RED = "\033[38;5;196m"
    BRIGHT_BLUE = "\033[38;5;27m"
    BRIGHT_MAGENTA = "\033[38;5;201m"
    BRIGHT_CYAN = "\033[38;5;14m"
    WARNING = "\033[93m"
    FAIL = "\033[91m"
    BOLD = "\033[1m"
    ITALIC = "\033[3m"
    UNDERLINE = "\033[4m"


def write_to_console(message: str = "") -> None:
    global silent
    if not silent:
        print(message)


def convert_memory_units(
    size: float,
    unit_type: int = MemoryUnitConverter.KILOBYTES,
    decimal_num_round: int = 4,
) -> float:
    """
    Convert units of a float, bytes, to desired unit, then round the result
    """
    new_size = size / MemoryUnitConverter.UNIT_CONVERSIONS[unit_type]
    return round(new_size, decimal_num_round)


def get_mem_size_str(
    size: float,
    unit_type: int = MemoryUnitConverter.KILOBYTES,
) -> str:
    """
    Convert bytes to desired memory unit size, then create string
    that will be written into csv file rows
    """
    new_size: float = convert_memory_units(size, unit_type)
    unit_str: str = MemoryUnitConverter.UNIT_STRS[unit_type]
    return f"{new_size} {unit_str}"


def process_asset_memory(
    importer: trade.AbstractImporter,
    template: attributes.ObjectAttributes,
) -> List[str]:
    """
    Use the trade.AbstractImporter class to query data size of mesh and image
    of asset
    """
    # construct absolute file paths for render and collision assets
    asset_paths: List[str] = [
        os.path.join(HABITAT_SIM_PATH, template.render_asset_handle),
        os.path.join(HABITAT_SIM_PATH, template.collision_asset_handle),
    ]

    # Log render and collision asset handles
    text_format = Fore.GREEN
    write_to_console(
        text_format
        + f"-render: {template.render_asset_handle}\n"
        + f"-collision: {template.collision_asset_handle}\n"
    )

    # Create variables to store mesh data
    mesh_count = 0
    index_data_size = 0  # bytes
    vertex_data_size = 0  # bytes

    # Create variables to store image data
    image_count = 0
    image_data_size = 0  # bytes

    # Calculate mesh data for both render and collision assets
    for asset_path in asset_paths:
        importer.open_file(asset_path)

        # Get mesh data
        mesh_count += importer.mesh_count
        for i in range(importer.mesh_count):
            mesh: trade.MeshData = importer.mesh(i)
            index_data_size += len(mesh.index_data)
            vertex_data_size += len(mesh.vertex_data)

        # Get image data
        for i in range(importer.image2d_count):
            image_count += importer.image2d_level_count(i)
            for j in range(importer.image2d_level_count(i)):
                image: trade.ImageData2D = importer.image2d(i, j)
                image_data_size += len(image.data)

    total_mesh_data_size = index_data_size + vertex_data_size

    # construct mesh data related strings formatted for csv file
    mesh_count_unit = "mesh" if mesh_count == 1 else "meshes"
    mesh_count_str = f"{mesh_count} {mesh_count_unit}"
    index_data_str = get_mem_size_str(index_data_size)
    vertex_data_str = get_mem_size_str(vertex_data_size)
    mesh_data_str = get_mem_size_str(total_mesh_data_size)

    # construct image data related strings formatted for csv file
    image_count_units = "image" if image_count == 1 else "images"
    image_count_str = f"{image_count} {image_count_units}"
    image_data_str = get_mem_size_str(image_data_size)

    # return results as a list of strings formatted for csv rows
    return [
        mesh_count_str,
        index_data_str,
        vertex_data_str,
        mesh_data_str,
        image_count_str,
        image_data_str,
        "CPU Mem ?",
    ]


def process_asset_physics(
    importer: trade.AbstractImporter,
    rigid_obj: physics.ManagedBulletRigidObject,
    template: attributes.ObjectAttributes,
    sim_settings: Dict[str, Any] = None,
) -> List[str]:
    """
    Run series of tests on asset to see how it responds in physics simulations
    """
    default_transfoms = sim_settings["default_transforms"]
    rigid_obj.translation = default_transfoms.get("default_obj_pos")

    default_rotation = sim_settings["default_transforms"].get("default_obj_rot")
    angle_degrees = mn.Deg(default_rotation.get("angle"))
    axis = mn.Vector3(default_rotation.get("axis"))
    rigid_obj.rotation = mn.Quaternion.rotation(mn.Rad(angle_degrees), axis)

    # Log position and rotation of rigid object
    text_format = ANSICodes.BRIGHT_MAGENTA.value
    write_to_console(
        text_format
        + "Transforms"
        + divider
        + "\n"
        + f"pos: {rigid_obj.translation}\n"
        + f"rot: angle - {rigid_obj.rotation.scalar} rad, axis - {rigid_obj.rotation.vector}\n"
    )

    physics_data: List[str] = [
        "Idle ?",
        "Stable ?",
        "Transl ?",
        "Rot ?",
    ]

    return physics_data


def process_imported_asset(
    importer: trade.AbstractImporter,
    handle: str,
    rigid_obj_mgr: physics.RigidObjectManager,
    obj_template_mgr: attributes_managers.ObjectAttributesManager,
    sim_settings: Dict[str, Any] = None,
) -> List[str]:
    """
    Use the trade.AbstractImporter class to query data size of mesh and image
    of asset and how the asset behaves during physics simulations
    """
    # Calculate memory state before loading object into memory
    start_mem = psutil.virtual_memory()._asdict()

    # # TODO testing
    # start_total_memory, start_used_memory, start_free_memory = map(
    #     int, os.popen('free -t -m').readlines()[-1].split()[1:]
    # )

    # Load object using RigidObjectManager
    rigid_obj = rigid_obj_mgr.add_object_by_template_handle(handle)

    # Calculate memory state after loading object into memory
    end_mem = psutil.virtual_memory()._asdict()

    # get average deltas of each RAM memory metric specified in the config file
    metrics: Dict[str, int] = sim_settings["mem_metrics_to_use"]
    avg_ram_delta = 0  # bytes
    for metric, order in metrics.items():
        # "order" is either -1 or 1. 1 means the delta is calculated
        # as (end - start), whereas -1 means (start - end).
        # e.g. data used should be higher after loading, so order == 1
        # but data free should be higher before loading, so order == -1
        avg_ram_delta += (end_mem.get(metric) - start_mem.get(metric)) * order
    avg_ram_delta /= len(metrics.keys())
    avg_ram_str = get_mem_size_str(avg_ram_delta)

    # TODO testing
    # end_total_memory, end_used_memory, end_free_memory = map(
    #     int, os.popen('free -t -m').readlines()[-1].split()[1:]
    # )

    # TODO: remove this when I figure out why ram usage is sometimes negative
    global negative_ram_count

    if avg_ram_delta < 0:
        negative_ram_count += 1
        avg_ram_str = "-----" + avg_ram_str + "-----"

    text_format = ANSICodes.BRIGHT_RED.value
    write_to_console(text_format + "\nstart" + divider)
    for key, value in start_mem.items():
        value_str = value
        if key != "percent":
            value_str = get_mem_size_str(value)
        write_to_console(text_format + f"{key} : {value_str}")

    write_to_console(text_format + "\nend" + divider)
    for key, value in end_mem.items():
        value_str = value
        if key != "percent":
            value_str = get_mem_size_str(value)
        write_to_console(text_format + f"{key} : {value_str}")

    write_to_console(text_format + "\nend - start" + divider)
    for (key_s, value_s), (key_e, value_e) in zip(start_mem.items(), end_mem.items()):
        value_str = value_e - value_s
        if key_s != "percent" and key_e != "percent":
            value_str = get_mem_size_str(value_e - value_s)
        write_to_console(text_format + f"{key_s} : {value_str}")

    write_to_console(text_format + "\naverage RAM used" + divider + f"\n{avg_ram_str}")

    #
    template = obj_template_mgr.get_template_by_handle(handle)
    text_format = ANSICodes.YELLOW.value
    write_to_console(text_format + "\nHandles" + divider + f"\ntemplate: {handle}")

    # run object through tests defined in dataset_processor_config.json file
    memory_data: List[str] = [avg_ram_str]
    sim_time_data: List[str] = []
    render_time_data: List[str] = []
    physics_data: List[str] = []
    data_to_collect = sim_settings["data_to_collect"]
    if data_to_collect.get("memory_data"):
        memory_data += process_asset_memory(importer, template)
    if data_to_collect.get("sim_time_ratio"):
        sim_time_data.append("...")
    if data_to_collect.get("render_time_ratio"):
        render_time_data.append("...")
    if data_to_collect.get("physics_data"):
        physics_data = process_asset_physics(
            importer, rigid_obj, template, sim_settings
        )

    # return results as a list of strings formatted for csv rows
    return [handle] + memory_data + sim_time_data + render_time_data + physics_data


def parse_dataset(
    sim: habitat_sim.Simulator = None, sim_settings: Dict[str, Any] = None
) -> List[List[str]]:
    """
    Load and process dataset objects using template handles
    """
    if sim is None:
        raise RuntimeError(
            ANSICodes.FAIL.value + "No simulator provided to parse_dataset(...)."
        )

    dataset_path = sim_settings["scene_dataset_config_file"]
    metadata_mediator = sim.metadata_mediator
    if (
        metadata_mediator is None
        or metadata_mediator.dataset_exists(dataset_path) is False
    ):
        raise RuntimeError(
            ANSICodes.FAIL.value
            + "No meta data mediator or dataset exists in parse_dataset(...)."
        )

    # get dataset that is currently being used by the simulator
    active_dataset: str = metadata_mediator.active_dataset
    text_format = ANSICodes.BRIGHT_BLUE.value + ANSICodes.BOLD.value
    write_to_console(text_format + "\nActive Dataset" + divider)
    text_format = ANSICodes.BRIGHT_BLUE.value
    write_to_console(text_format + f"{active_dataset}\n")

    # # get exhaustive str of information about the dataset
    # dataset_report: str = metadata_mediator.dataset_report(dataset_path)
    # text_format = ANSICodes.BRIGHT_CYAN.value + ANSICodes.BOLD.value
    # write_to_console(text_format + "Dataset Report" + divider)
    # text_format = ANSICodes.BRIGHT_CYAN.value
    # write_to_console(text_format + f"{dataset_report}\n")

    # get handles of every scene from the simulator
    scene_handles: List[str] = metadata_mediator.get_scene_handles()
    text_format = ANSICodes.BRIGHT_MAGENTA.value + ANSICodes.BOLD.value
    write_to_console(text_format + "Scene Handles" + divider)
    text_format = ANSICodes.BRIGHT_MAGENTA.value
    for handle in scene_handles:
        write_to_console(text_format + f"{handle}\n")

    # get List of Unified Robotics Description Format files
    urdf_paths = metadata_mediator.urdf_paths
    urdf_paths_list = list(urdf_paths.keys())
    text_format = ANSICodes.BRIGHT_RED.value + ANSICodes.BOLD.value
    write_to_console(text_format + f"num urdf paths: {len(urdf_paths_list)}" + divider)
    if len(urdf_paths_list) == 0:
        urdf_paths["Paths"] = "None"
    text_format = ANSICodes.BRIGHT_RED.value
    for key, val in urdf_paths.items():
        write_to_console(text_format + f"{key} : {val}\n")

    # get AssetAttributesManager and get template handles of each primitive asset
    asset_template_manager = metadata_mediator.asset_template_manager
    text_format = ANSICodes.ORANGE.value + ANSICodes.BOLD.value
    write_to_console(
        text_format
        + f"number of primitive asset templates: {asset_template_manager.get_num_templates()}"
        + divider
    )
    template_handles = asset_template_manager.get_template_handles()
    templates_info = asset_template_manager.get_templates_info()
    for (handle, info) in zip(template_handles, templates_info):
        write_to_console(Fore.GREEN + f"{handle}")
        write_to_console(ANSICodes.ORANGE.value + ANSICodes.ITALIC.value + f"{info}\n")

    # Get ObjectAttributesManager for objects from dataset, load the dataset,
    # and store all the object template handles in a List
    obj_template_mgr = sim.get_object_template_manager()
    obj_template_mgr.load_configs(dataset_path)
    assert obj_template_mgr.get_num_templates() > 0
    object_template_handles = obj_template_mgr.get_file_template_handles("")
    text_format = ANSICodes.BRIGHT_MAGENTA.value + ANSICodes.BOLD.value
    write_to_console(
        text_format
        + f"number of objects in dataset: {len(object_template_handles)}"
        + divider
        + "\n"
    )

    # Get RigidObjectManager and add templates from ObjectAttributesManager
    # process each asset with trade.AbstractImporter and construct csv data
    csv_rows: List[List[str]] = []
    manager = trade.ImporterManager()
    importer = manager.load_and_instantiate("AnySceneImporter")
    rigid_obj_mgr = sim.get_rigid_object_manager()
    for handle in object_template_handles:
        row = process_imported_asset(
            importer, handle, rigid_obj_mgr, obj_template_mgr, sim_settings
        )
        csv_rows.append(row)

    # clean up
    importer.close()

    # return list of rows to be written to csv file
    return csv_rows


def get_headers_for_csv(sim_settings) -> List[str]:
    """
    Collect the column titles we'll need given which tests we ran
    """
    headers: List[str] = []
    data_to_collect = sim_settings["data_to_collect"]
    if data_to_collect.get("memory_data"):
        headers += sim_settings["memory_data_headers"]
    if data_to_collect.get("sim_time_ratio"):
        headers += sim_settings["sim_time_headers"]
    if data_to_collect.get("render_time_ratio"):
        headers += sim_settings["render_time_headers"]
    if data_to_collect.get("physics_data"):
        headers += sim_settings["physics_data_headers"]
    return headers


def create_csv_file(
    headers: List[str],
    csv_rows: List[List[str]],
    csv_dir_path: str = None,
    csv_file_prefix: str = None,
) -> None:
    """
    Set directory where our csv's will be saved, create the csv file name,
    create the column names of our csv data, then open and write the csv
    file
    """
    file_path = CSVWriter.create_unique_filename(csv_dir_path, csv_file_prefix)

    text_format = ANSICodes.PURPLE.value + ANSICodes.BOLD.value
    write_to_console(text_format + "Writing csv results to:" + divider)
    text_format = ANSICodes.PURPLE.value
    write_to_console(text_format + f"{file_path}\n")

    CSVWriter.write_file(headers, csv_rows, file_path)

    text_format = ANSICodes.PURPLE.value
    write_to_console(text_format + "CSV writing done\n")

    # TODO: remove this when I figure out why ram usage is sometimes negative
    text_format = ANSICodes.BRIGHT_RED.value
    global negative_ram_count
    write_to_console(
        text_format + f"Negative RAM usage count: {negative_ram_count}" + divider
    )


def build_parser(
    parser: Optional[argparse.ArgumentParser] = None,
) -> argparse.ArgumentParser:
    """
    Parse arguments or set defaults when running script for scene,
    dataset, and set if we are processing physics data of dataset
    objects
    """
    if parser is None:
        parser = argparse.ArgumentParser(
            description="Tool to evaluate all objects in a dataset. Assesses CPU, GPU, mesh size, "
            " and other characteristics to determine if an object will be problematic when using it"
            " in a simulation",
            formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        )
    parser = argparse.ArgumentParser()

    # optional arguments
    parser.add_argument(
        "--config_file_path",
        default="tools/dataset_object_processor/configs/default.dataset_processor_config.json",
        type=str,
        help="config file to load"
        ' (default: "tools/dataset_object_processor/configs/default.dataset_processor_config.json")',
    )
    parser.add_argument(
        "--csv_file_prefix",
        default=None,
        type=str,
        metavar="PREFIX",
        help="Prefix string of file name for CSV and/or video recording file to create, e.g."
        " ycb,"
        " replica_CAD,"
        " (default: None)",
    )
    return parser


def main() -> None:
    """
    Create Simulator, parse dataset, write csv file
    """
    # TODO: remove this when I figure out why ram usage is sometimes negative
    global negative_ram_count
    negative_ram_count = 0

    # setup colorama terminal color printing so that format and color reset
    # after each write_to_console statement
    global silent
    init(autoreset=True)

    # parse arguments from command line: scene, dataset, if we process physics
    args = build_parser().parse_args()

    # Populate sim_settings with info from dataset_processor_config.json file
    sim_settings: Dict[str, Any] = default_sim_settings
    with open(os.path.join(HABITAT_SIM_PATH, args.config_file_path)) as config_json:
        sim_settings.update(json.load(config_json))
    silent = sim_settings["silent"]

    # Configure and make simulator
    cfg = make_cfg(sim_settings)
    sim = habitat_sim.Simulator(cfg)

    # TODO
    text_format = ANSICodes.PURPLE.value
    for key, value in sim_settings.items():
        write_to_console(text_format + f"{key} : {value}\n")

    # Parse dataset and write CSV. "headers" stores the titles of each column
    headers = get_headers_for_csv(sim_settings)
    csv_rows: List[List[str]] = parse_dataset(sim, sim_settings)
    csv_dir_path = os.path.join(
        HABITAT_SIM_PATH, sim_settings["output_paths"].get("csv")
    )
    csv_file_prefix = sim_settings["output_paths"].get("output_file_prefix")
    create_csv_file(headers, csv_rows, csv_dir_path, csv_file_prefix)


if __name__ == "__main__":
    main()
