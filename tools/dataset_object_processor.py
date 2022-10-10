import argparse
import csv
import datetime
import os
from typing import Any, Dict, List, Optional, Tuple

import git
from magnum import trade

import habitat_sim
from habitat_sim.utils.settings import default_sim_settings

repo = git.Repo(".", search_parent_directories=True)
HABITAT_SIM_PATH = repo.working_tree_dir
DATA_PATH = os.path.join(HABITAT_SIM_PATH, "data")
YCB_PATH = os.path.join(
    DATA_PATH, "versioned_data/ycb_1.2/ycb.scene_dataset_config.json"
)
REPLICA_CAD_PATH = os.path.join(
    DATA_PATH,
    "versioned_data/replica_cad_dataset_1.5/reHABITAT_SIM_PATHplicaCAD.scene_dataset_config.json",
)
ROBOT_PATH = ""  # TODO, which dataset is this? Robot fetch?


class CSVWriter:
    """
    Generalized utility to write csv files
    """

    csv_dir_path = None
    file_path = None

    def set_csv_dir_path(csv_dir_path: str = None) -> None:
        """"""
        if csv_dir_path is None:
            CSVWriter.csv_dir_path = f"{DATA_PATH}/dataset_csvs"
        else:
            CSVWriter.csv_dir_path = csv_dir_path

    def create_unique_filename() -> str:
        """ """
        # Current date and time so we can make unique file names for each csv
        date_and_time = datetime.datetime.now()

        # year-month-day
        date = date_and_time.strftime("%Y-%m-%d")

        # hour:min:sec - capital H is military time, %I is standard time
        # (am/pm time format)
        time = date_and_time.strftime("%H:%M:%S")

        # make directory to store csvs if it doesn't exist
        if CSVWriter.csv_dir_path is None:
            CSVWriter.set_csv_dir_path()
        dir_exists = os.path.exists(CSVWriter.csv_dir_path)
        if not dir_exists:
            os.makedirs(CSVWriter.csv_dir_path)

        # create csv file name (TODO: make more descriptive file name)
        CSVWriter.file_path = f"{CSVWriter.csv_dir_path}/date_{date}__time_{time}.csv"
        return CSVWriter.file_path

    def write_file(
        headers: List[str] = None,
        csv_rows: List[str] = None,
        file_path: str = None,
    ) -> None:
        """"""
        if headers is None:
            raise RuntimeError("No headers provided to CSVWriter.write_file().")
        if csv_rows is None:
            raise RuntimeError("No CSV file data provided to CSVWriter.write_file().")
        if file_path is None:
            if CSVWriter.file_path is None:
                file_path = CSVWriter.create_unique_filename()
            else:
                file_path = CSVWriter.file_path

        with open(file_path, "w") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(headers)
            writer.writerows(csv_rows)


class MemoryUnitConverter:
    """
    class to convert computer memory value units, i.e.
    1,024 bytes to 1 kilobyte
    1,048,576 bytes to 1 megabyte
    etc.
    """

    BYTES = 0
    KILOBYTES = 1
    MEGABYTES = 2
    GIGABYTES = 3

    UNIT_STRS = ["bytes", "KB", "MB", "GB"]
    UNIT_CONVERSIONS = [1, 1 << 10, 1 << 20, 1 << 30]


class PrintColors:
    """
    Console printing ANSI color codes
    """

    HEADER = "\033[95m"
    WHITE = "\u001b[37m"
    RED = "\033[1;31m"
    GREEN = "\033[92m"
    BLUE = "\033[94m"
    CYAN = "\033[96m"
    MAGENTA = "\u001b[35m"
    BROWN = "\033[0;33m"
    LIGHT_RED = "\033[1;31m"
    PURPLE = "\033[1;35m"
    LIGHT_CYAN = "\033[1;36m"
    TEST = "\u001a[35m"
    WARNING = "\033[93m"
    FAIL = "\033[91m"
    ENDC = "\033[0m"
    BOLD = "\033[1m"
    UNDERLINE = "\033[4m"


def print_in_color(print_string="", color=PrintColors.WHITE) -> None:
    """
    Allows us to print to console in different colors
    """
    print(color + print_string + PrintColors.ENDC)


def convert_units(
    size: float, unit_type: int = MemoryUnitConverter.KILOBYTES, decimals: int = 4
) -> float:
    """
    Convert units of a float, then round the result
    """
    new_size = size / MemoryUnitConverter.UNIT_CONVERSIONS[unit_type]
    return round(new_size, decimals)


def get_mem_size_str(
    size: float,
    unit_type: int = MemoryUnitConverter.KILOBYTES,
) -> str:
    new_size: float = convert_units(size, unit_type)
    unit_str: str = MemoryUnitConverter.UNIT_STRS[unit_type]
    return f"{new_size} {unit_str}"


def process_imported_asset(
    importer: trade.AbstractImporter,
    render_asset_handle: str = "",
) -> Tuple:
    """
    Use the trade.AbstractImporter class to query data size of mesh and image
    of asset
    """
    # get asset path
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

    return [
        render_asset_handle,
        index_data_str,
        vertex_data_str,
        mesh_data_str,
        image_data_str,
    ]


def parse_dataset(
    sim: habitat_sim.Simulator = None, dataset_path: str = None
) -> List[str]:
    """ """
    if sim is None:
        return []

    metadata_mediator = sim.metadata_mediator
    if (
        metadata_mediator is None
        or metadata_mediator.dataset_exists(dataset_path) is False
    ):
        return []

    # get dataset that is currently being used by the simulator
    active_dataset: str = metadata_mediator.active_dataset
    print_in_color("* " * 39, PrintColors.BLUE)
    print_in_color(f"{active_dataset}\n", PrintColors.BLUE)

    # get exhaustive List of information about the dataset
    dataset_report: str = metadata_mediator.dataset_report(dataset_path)
    print_in_color("* " * 39, PrintColors.CYAN)
    print_in_color(f"{dataset_report}\n", PrintColors.CYAN)

    # get handles of every scene from the simulator
    scene_handles: List[str] = metadata_mediator.get_scene_handles()
    print_in_color("* " * 39, PrintColors.PURPLE)
    for handle in scene_handles:
        print_in_color(f"{handle}\n", PrintColors.PURPLE)

    # get List of Unified Robotics Description Format files
    urdf_paths = metadata_mediator.urdf_paths
    urdf_paths_list = list(urdf_paths.keys())
    print_in_color("-" * 72, PrintColors.MAGENTA)
    print_in_color(f"num urdf paths: {len(urdf_paths_list)}\n", PrintColors.MAGENTA)
    for name in urdf_paths_list:
        print_in_color("-" * 72, PrintColors.MAGENTA)
        print_in_color(f"{name}\n", PrintColors.MAGENTA)

    # get asset template manager and get template handles of each primitive asset
    asset_template_manager = metadata_mediator.asset_template_manager
    print_in_color(
        f"\nnumber of primitive asset templates: {asset_template_manager.get_num_templates()}",
        PrintColors.BROWN,
    )
    print_in_color("-" * 72, PrintColors.BROWN)
    template_handles = asset_template_manager.get_template_handles()
    templates_info = asset_template_manager.get_templates_info()
    for (handle, info) in zip(template_handles, templates_info):
        print_in_color(f"{handle}", PrintColors.GREEN)
        print_in_color(f"{info}\n", PrintColors.BROWN + PrintColors.UNDERLINE)

    # Get rigid object manager
    rigid_object_manager = sim.get_rigid_object_manager()
    print_in_color(rigid_object_manager.get_objects_CSV_info(), PrintColors.CYAN)

    # Get object attribute manager for objects from dataset, load the dataset,
    # store all the object template handles in a List, then process each asset
    object_attributes_manager = sim.get_object_template_manager()
    object_attributes_manager.load_configs(dataset_path)
    object_template_handles = object_attributes_manager.get_file_template_handles("")
    print_in_color(
        f"\nnumber of objects in dataset: {len(object_template_handles)}",
        PrintColors.PURPLE,
    )
    print_in_color("-" * 72, PrintColors.PURPLE)
    print("")

    # process each asset with trade.AbstractImporter and construct csv data
    csv_rows: List[str] = []
    manager = trade.ImporterManager()
    importer = manager.load_and_instantiate("AnySceneImporter")
    for handle in object_template_handles:
        template = object_attributes_manager.get_template_by_handle(handle)
        csv_rows.append(process_imported_asset(importer, template.render_asset_handle))

    # clean up
    importer.close()

    return csv_rows


def write_csv(csv_rows: List[str] = None) -> None:
    """ """
    CSVWriter.set_csv_dir_path()
    file_path = CSVWriter.create_unique_filename()
    print_in_color(f"Writing csv results to {file_path}", PrintColors.CYAN)
    headers = [
        "mesh name",
        "mesh index data size",
        "mesh vertex data size",
        "total mesh data size",
        "image data size",
    ]
    CSVWriter.write_file(headers, csv_rows)


def make_configuration(sim_settings):
    """ """
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
    """ """
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
        help='scene/stage file to load (default: "./data/test_assets/scenes/simple_room.glb")',
    )
    parser.add_argument(
        "--dataset",
        default="./data/objects/ycb/ycb.scene_dataset_config.json",
        type=str,
        metavar="DATASET",
        help='dataset configuration file to use (default: "./data/objects/ycb/ycb.scene_dataset_config.json")',
    )
    parser.add_argument(
        "--disable_physics",
        default=False,
        action="store_true",
        help="disable physics simulation (default: False)",
    )
    return parser


def main() -> None:
    """ """
    args = build_parser().parse_args()

    # Setting up sim_settings
    sim_settings: Dict[str, Any] = default_sim_settings
    sim_settings["scene"] = args.scene
    sim_settings["scene_dataset_config_file"] = args.dataset
    sim_settings["enable_physics"] = not args.disable_physics

    cfg = make_configuration(sim_settings)
    sim = habitat_sim.Simulator(cfg)
    csv_rows: List[str] = parse_dataset(sim, args.dataset)
    write_csv(csv_rows)


if __name__ == "__main__":
    main()
