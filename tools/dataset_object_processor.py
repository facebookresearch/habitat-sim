import argparse
import csv
import datetime
import os
from enum import Enum
from typing import Any, Dict, List, Optional

import git
from colorama import Fore, init
from magnum import trade

import habitat_sim
from habitat_sim.utils.settings import default_sim_settings

repo = git.Repo(".", search_parent_directories=True)
HABITAT_SIM_PATH = repo.working_tree_dir

# Hard-coded paths for common datasets this script will process
YCB_PATH = "./data/objects/ycb/ycb.scene_dataset_config.json"

REPLICA_CAD_PATH = "./data/replica_cad/replicaCAD.scene_dataset_config.json"

# TODO, which dataset is this? Fetch Robot?
ROBOT_PATH = ""

# dictionary to refer to datasets by simple names rather than full paths
dataset_name_to_path_dict: Dict[str, str] = {}


class CSVWriter:
    """
    Generalized utility to write csv files
    """

    file_path = None

    def create_unique_filename(csv_dir_path: str, filename_prefix: str = None) -> str:
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
        if not os.path.exists(csv_dir_path):
            os.makedirs(csv_dir_path)

        # create csv file name (TODO: make more descriptive file name)
        if filename_prefix is None:
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
                file_path = CSVWriter.create_unique_filename()
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


def print_help_text() -> None:
    help_text = """
=========================================================

To save effort, you can refer to pre-coded dataset names
instead of full paths for the "--dataset_name" parameter.
If you enter both a dataset name and a dataset path,
it will use dataset name instead.

The ones registered in the script currently are:
ycb
replica_CAD
fetch_robot

=========================================================
"""
    print(ANSICodes.BRIGHT_MAGENTA.value + f"{help_text}")


def convert_units(
    size: float, unit_type: int = MemoryUnitConverter.KILOBYTES, decimals: int = 4
) -> float:
    """
    Convert units of a float, bytes to desired unit, then round the result
    """
    new_size = size / MemoryUnitConverter.UNIT_CONVERSIONS[unit_type]
    return round(new_size, decimals)


def get_mem_size_str(
    size: float,
    unit_type: int = MemoryUnitConverter.KILOBYTES,
) -> str:
    """
    Convert bytes to desired memory unit size, then create string
    that will be written into csv file rows
    """
    new_size: float = convert_units(size, unit_type)
    unit_str: str = MemoryUnitConverter.UNIT_STRS[unit_type]
    return f"{new_size} {unit_str}"


def process_imported_asset(
    importer: trade.AbstractImporter,
    render_asset_handle: str = "",
) -> List[str]:
    """
    Use the trade.AbstractImporter class to query data size of mesh and image
    of asset
    """
    # get absolute asset file path
    asset_path = os.path.join(HABITAT_SIM_PATH, render_asset_handle)

    # Open file with AbstractImporter
    importer.open_file(asset_path)

    # Get mesh data
    index_data_size = 0  # bytes
    vertex_data_size = 0  # bytes
    mesh_data_size = 0  # bytes
    for i in range(importer.mesh_count):
        mesh: trade.MeshData = importer.mesh(i)
        index_data_size += len(mesh.index_data)
        vertex_data_size += len(mesh.vertex_data)
        mesh_data_size += index_data_size + vertex_data_size
    index_data_str = get_mem_size_str(index_data_size)
    vertex_data_str = get_mem_size_str(vertex_data_size)
    mesh_data_str = get_mem_size_str(mesh_data_size)

    # Get image data
    image_data_size = 0  # bytes
    for i in range(importer.image2d_count):
        for j in range(importer.image2d_level_count(i)):
            image: trade.ImageData2D = importer.image2d(i, j)
            image_data_size += len(image.data)
    image_data_str = get_mem_size_str(image_data_size)

    # return results as a tuple formatted for csv rows
    return [
        render_asset_handle,
        index_data_str,
        vertex_data_str,
        mesh_data_str,
        image_data_str,
    ]


def parse_dataset(
    sim: habitat_sim.Simulator = None, dataset_path: str = None
) -> List[List[str]]:
    """
    Load and process dataset objects using template handles
    """
    if sim is None:
        raise RuntimeError(
            ANSICodes.FAIL.value + "No simulator provided to parse_dataset(...)."
        )

    metadata_mediator = sim.metadata_mediator
    if (
        metadata_mediator is None
        or metadata_mediator.dataset_exists(dataset_path) is False
    ):
        raise RuntimeError(
            ANSICodes.FAIL.value
            + "No meta data mediator or dataset exists in parse_dataset(...)."
        )

    # print detailed information about how to use utilize certain parameters
    # to run this script
    print_help_text()

    # get dataset that is currently being used by the simulator
    active_dataset: str = metadata_mediator.active_dataset
    text_format = ANSICodes.BRIGHT_BLUE.value + ANSICodes.BOLD.value
    print(text_format + "\nActive Dataset")
    print(text_format + "-" * 72)
    text_format = ANSICodes.BRIGHT_BLUE.value
    print(text_format + f"{active_dataset}\n")
    print("")

    # get exhaustive List of information about the dataset
    dataset_report: str = metadata_mediator.dataset_report(dataset_path)
    text_format = ANSICodes.BRIGHT_CYAN.value + ANSICodes.BOLD.value
    print(text_format + "Dataset Report")
    print(text_format + "-" * 72)
    text_format = ANSICodes.BRIGHT_CYAN.value
    print(text_format + f"{dataset_report}")
    print("")

    # get handles of every scene from the simulator
    scene_handles: List[str] = metadata_mediator.get_scene_handles()
    text_format = ANSICodes.BRIGHT_MAGENTA.value + ANSICodes.BOLD.value
    print(text_format + "Scene Handles")
    print(text_format + "-" * 72)
    text_format = ANSICodes.BRIGHT_MAGENTA.value
    for handle in scene_handles:
        print(text_format + f"{handle}\n")
    print("")

    # get List of Unified Robotics Description Format files
    urdf_paths = metadata_mediator.urdf_paths
    urdf_paths_list = list(urdf_paths.keys())
    text_format = ANSICodes.BRIGHT_RED.value + ANSICodes.BOLD.value
    print(text_format + f"num urdf paths: {len(urdf_paths_list)}")
    print(text_format + "-" * 72)
    if len(urdf_paths_list) == 0:
        urdf_paths["Paths"] = "None"
    text_format = ANSICodes.BRIGHT_RED.value
    for key, val in urdf_paths.items():
        print(text_format + f"{key} : {val}")
    print("")

    # get asset template manager and get template handles of each primitive asset
    asset_template_manager = metadata_mediator.asset_template_manager
    text_format = ANSICodes.ORANGE.value + ANSICodes.BOLD.value
    print(
        text_format
        + f"\nnumber of primitive asset templates: {asset_template_manager.get_num_templates()}"
    )
    print(text_format + "-" * 72)
    template_handles = asset_template_manager.get_template_handles()
    templates_info = asset_template_manager.get_templates_info()
    for (handle, info) in zip(template_handles, templates_info):
        print(Fore.GREEN + f"{handle}")
        print(ANSICodes.ORANGE.value + ANSICodes.ITALIC.value + f"{info}\n")
    print("")

    # Get rigid object manager
    rigid_object_manager = sim.get_rigid_object_manager()
    text_format = ANSICodes.YELLOW.value + ANSICodes.BOLD.value
    print(text_format + "Rigid object manager objects")
    print(text_format + "-" * 72)
    text_format = ANSICodes.YELLOW.value
    print(text_format + rigid_object_manager.get_objects_CSV_info())
    print("")

    # Get object attribute manager for objects from dataset, load the dataset,
    # store all the object template handles in a List, then process each asset
    object_attributes_manager = sim.get_object_template_manager()
    object_attributes_manager.load_configs(dataset_path)
    object_template_handles = object_attributes_manager.get_file_template_handles("")
    text_format = ANSICodes.BRIGHT_MAGENTA.value + ANSICodes.BOLD.value
    print(
        text_format + f"\nnumber of objects in dataset: {len(object_template_handles)}"
    )
    print(text_format + "-" * 72)
    print("")

    # process each asset with trade.AbstractImporter and construct csv data
    csv_rows: List[List[str]] = []
    manager = trade.ImporterManager()
    importer = manager.load_and_instantiate("AnySceneImporter")
    for handle in object_template_handles:
        template = object_attributes_manager.get_template_by_handle(handle)
        csv_rows.append(process_imported_asset(importer, template.render_asset_handle))

    # clean up
    importer.close()

    # return list of rows to be written to csv file
    return csv_rows


def create_csv_file(csv_rows: List[str], csv_file_prefix: str = "") -> None:
    """
    Set directory where our csv's will be saved, create the csv file name,
    create the column names of our csv data, then open and write the csv
    file
    """
    csv_dir_path = f"{HABITAT_SIM_PATH}/data/dataset_csvs/"
    file_path = CSVWriter.create_unique_filename(csv_dir_path, csv_file_prefix)
    text_format = ANSICodes.PURPLE.value + ANSICodes.BOLD.value
    print(text_format + f"Writing csv results to: \n{file_path}")
    print(text_format + "-" * 72)
    headers = [
        "mesh name",
        "mesh index data size",
        "mesh vertex data size",
        "total mesh data size",
        "image data size",
    ]
    CSVWriter.write_file(headers, csv_rows)
    text_format = ANSICodes.PURPLE.value
    print(text_format + "CSV writing done\n")


def make_configuration(sim_settings):
    """
    Create config of Simulator that will process the dataset
    """
    # simulator configuration
    sim_cfg = habitat_sim.SimulatorConfiguration()
    if "scene_dataset_config_file" in sim_settings:
        sim_cfg.scene_dataset_config_file = sim_settings["scene_dataset_config_file"]
    sim_cfg.frustum_culling = sim_settings.get("frustum_culling", False)
    if not hasattr(sim_cfg, "scene_id"):
        raise RuntimeError(
            "Error: Please upgrade habitat-sim. SimulatorConfig API version mismatch"
        )
    sim_cfg.scene_id = sim_settings["scene"]
    assert os.path.exists(sim_cfg.scene_id)

    # camera will likely not be used
    camera_sensor_spec = habitat_sim.CameraSensorSpec()
    camera_sensor_spec.sensor_type = habitat_sim.SensorType.COLOR
    camera_sensor_spec.sensor_subtype = habitat_sim.SensorSubType.PINHOLE
    camera_sensor_spec.resolution = [sim_settings["height"], sim_settings["width"]]
    camera_sensor_spec.position = [0, sim_settings["sensor_height"], 0]

    agent_cfg = habitat_sim.agent.AgentConfiguration()
    agent_cfg.sensor_specifications = [camera_sensor_spec]

    return habitat_sim.Configuration(sim_cfg, [agent_cfg])


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
        "--scene",
        default="./data/test_assets/scenes/simple_room.glb",
        type=str,
        help="""
scene/stage file to load (default: "./data/test_assets/scenes/simple_room.glb")
        """,
    )
    parser.add_argument(
        "--dataset_name",
        default=None,
        type=str,
        metavar="DATASET_PATH",
        help="""
for convenience, simple name of dataset configuration file to use,
use None if you want to enter the file path instead (default: None)
        """,
    )
    parser.add_argument(
        "--dataset_path",
        default="./data/objects/ycb/ycb.scene_dataset_config.json",
        type=str,
        metavar="DATASET_PATH",
        help="""
relative path of dataset configuration file to use
(default: "./data/objects/ycb/ycb.scene_dataset_config.json")
        """,
    )
    parser.add_argument(
        "--disable_physics",
        default=False,
        action="store_true",
        help="""
disable physics simulation (default: False)
        """,
    )
    parser.add_argument(
        "--csv_file_prefix",
        default=None,
        type=str,
        metavar="PREFIX",
        help="""
Prefix string of file name for CSV file to create, e.g.
ycb,
replica_CAD,
fetch_robot,
(default: None)
        """,
    )
    return parser


def main() -> None:
    """
    Create Simulator, parse dataset, write csv file
    """

    # setup colorama terminal color printing so that format and color reset
    # after each print statement
    init(autoreset=True)

    # parse arguments from command line: scene, dataset, if we process physics
    args = build_parser().parse_args()

    # Setting up sim_settings
    sim_settings: Dict[str, Any] = default_sim_settings
    sim_settings["scene"] = args.scene
    sim_settings["simple_dataset_name"] = args.dataset_name
    sim_settings["scene_dataset_config_file"] = args.dataset_path
    sim_settings["enable_physics"] = not args.disable_physics
    sim_settings["csv_file_prefix"] = args.csv_file_prefix

    # determine if using dataset_name or dataset_path to reference dataset config file
    # dictionary to refer to datasets by simple names rather than full paths
    dataset_name_to_path_dict["ycb"] = YCB_PATH
    dataset_name_to_path_dict["replica_CAD"] = REPLICA_CAD_PATH
    dataset_name_to_path_dict["fetch_robot"] = ROBOT_PATH
    if (
        args.dataset_name is not None
        and dataset_name_to_path_dict[args.dataset_name] is not None
    ):
        dataset_path = dataset_name_to_path_dict[args.dataset_name]
        sim_settings["scene_dataset_config_file"] = dataset_path
        print(
            ANSICodes.BRIGHT_CYAN.value
            + f"\ndataset\n {args.dataset_name} : {dataset_name_to_path_dict[args.dataset_name]}\n"
        )
    else:
        dataset_path = args.dataset_path
        print(ANSICodes.BRIGHT_CYAN.value + f"\ndataset path:\n {dataset_path}\n")

    # create simulator for this scene and dataset
    cfg = make_configuration(sim_settings)
    sim = habitat_sim.Simulator(cfg)

    # parse dataset and write csv file
    csv_rows: List[List[str]] = parse_dataset(sim, dataset_path)
    create_csv_file(csv_rows, args.csv_file_prefix)


if __name__ == "__main__":
    main()
