import argparse
import math
import os
from typing import Any, Dict, List, Optional

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
    "versioned_data/replica_cad_dataset_1.5/replicaCAD.scene_dataset_config.json",
)
ROBOT_PATH = ""  # TODO, which dataset is this? Robot fetch?


class MemoryUnitConverter:
    """
    class to convert computer memory value units, e.g.
    1,000,000 bytes to 1 megabyte
    """

    BYTES = 0
    KILOBYTES = 1
    MEGABYTES = 2
    GIGABYTES = 3

    UNIT_STRS = ["bytes", "KB", "MB", "GB"]
    EXPONENT = [1, -3, -6, -9]


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
    size: float,
    conversion: float = 1.0,
    decimals: int = 4,
) -> float:
    """
    Convert units of a float, then round the result
    """
    return round(size * conversion, decimals)


def process_imported_asset(
    importer: trade.AbstractImporter,
    asset_path: str = "",
    unit_type: int = MemoryUnitConverter.KILOBYTES,
) -> None:
    """
    Use the trade.AbstractImporter class to query data size of mesh and image
    of asset
    """
    conversion: int = math.pow(10.0, MemoryUnitConverter.EXPONENT[unit_type])
    unit_str: str = MemoryUnitConverter.UNIT_STRS[unit_type]

    # Open file with AbstractImporter
    importer.open_file(asset_path)

    # Get mesh data
    print_in_color("\nMesh Data", PrintColors.GREEN)
    print_in_color("-" * 72, PrintColors.GREEN)
    mesh_data_size = 0  # bytes
    for i in range(importer.mesh_count):
        mesh: trade.MeshData = importer.mesh(i)
        index_data_size = len(mesh.index_data)
        vertex_data_size = len(mesh.vertex_data)
        mesh_data_size = index_data_size + vertex_data_size
        print_in_color(f"mesh name: {importer.mesh_name(i)}", PrintColors.GREEN)
        print_in_color(f"mesh index: {i}", PrintColors.GREEN)
        print_in_color(
            f"mesh level count: {importer.mesh_level_count(i)}",
            PrintColors.GREEN,
        )
        print_in_color(
            f"index data size: {convert_units(index_data_size, conversion)} {unit_str}",
            PrintColors.GREEN,
        )
        print_in_color(
            f"vertex data size: {convert_units(vertex_data_size, conversion)} {unit_str}",
            PrintColors.GREEN,
        )
        print_in_color(
            f"total mesh size: {convert_units(mesh_data_size, conversion)} {unit_str}\n",
            PrintColors.GREEN,
        )

    # Get image data
    print_in_color("Image Data", PrintColors.RED)
    print_in_color("-" * 72, PrintColors.RED)
    print_in_color(f"num 2D images: {importer.image2d_count}", PrintColors.RED)
    image_data_size = 0  # bytes
    for i in range(importer.image2d_count):
        print_in_color(f"image index: {i}", PrintColors.RED)
        for j in range(importer.image2d_level_count(i)):
            image: trade.ImageData2D = importer.image2d(i, j)
            image_data_size += len(image.data)
            converted_size = convert_units(len(image.data), conversion)
            print_in_color(
                f"- image mip map level - {j}, size - ({image.size.x}, {image.size.y}), mem - {converted_size} {unit_str}",
                PrintColors.RED,
            )
        print_in_color(
            f"image index {i} total data size: {convert_units(image_data_size, conversion)} {unit_str}",
            PrintColors.RED,
        )

    print("\n\n")


def parse_dataset(sim: habitat_sim.Simulator = None, dataset_path: str = None) -> None:
    """ """
    if sim is None:
        return

    metadata_mediator = sim.metadata_mediator
    if (
        metadata_mediator is None
        or metadata_mediator.dataset_exists(dataset_path) is False
    ):
        return

    # get dataset that is currently being used by the simulator
    active_dataset: str = metadata_mediator.active_dataset
    print_in_color("* " * 39, PrintColors.BLUE)
    print_in_color(f"{active_dataset}\n", PrintColors.BLUE)

    # get exhaustive list of information about the dataset
    dataset_report: str = metadata_mediator.dataset_report(dataset_path)
    print_in_color("* " * 39, PrintColors.CYAN)
    print_in_color(f"{dataset_report}\n", PrintColors.CYAN)

    # get handles of every scene from the simulator
    scene_handles: List[str] = metadata_mediator.get_scene_handles()
    print_in_color("* " * 39, PrintColors.PURPLE)
    for handle in scene_handles:
        print_in_color(f"{handle}\n", PrintColors.PURPLE)

    # get list of Unified Robotics Description Format files
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
    # store all the object template handles in a list, then process each asset
    object_attributes_manager = sim.get_object_template_manager()
    object_attributes_manager.load_configs(dataset_path)
    object_template_handles = object_attributes_manager.get_file_template_handles("")
    print_in_color(
        f"\nnumber of ojects in dataset: {len(object_template_handles)}",
        PrintColors.PURPLE,
    )
    print_in_color("-" * 72, PrintColors.PURPLE)
    print("")
    manager = trade.ImporterManager()
    importer = manager.load_and_instantiate("AnySceneImporter")
    for handle in object_template_handles:
        template = object_attributes_manager.get_template_by_handle(handle)
        asset_path = os.path.join(HABITAT_SIM_PATH, template.render_asset_handle)
        print_in_color(asset_path, PrintColors.CYAN)
        print_in_color("-" * 72, PrintColors.CYAN)
        process_imported_asset(importer, asset_path)

    # clean up
    importer.close()


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
    parse_dataset(sim, args.dataset)


if __name__ == "__main__":
    main()
