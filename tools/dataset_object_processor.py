import argparse
import os
from typing import Any, Dict, Optional

import git

import habitat_sim
from habitat_sim.utils.settings import default_sim_settings

repo = git.Repo(".", search_parent_directories=True)
dir_path = repo.working_tree_dir
data_path = os.path.join(dir_path, "data")
YCB_PATH = os.path.join(
    data_path, "versioned_data/ycb_1.2/ycb.scene_dataset_config.json"
)
REPLICA_CAD_PATH = os.path.join(
    data_path,
    "versioned_data/replica_cad_dataset_1.5/replicaCAD.scene_dataset_config.json",
)
ROBOT_PATH = ""  # TODO, which dataset is this? Robot fetch?


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
    YELLOW = "\u001b[33m"
    BROWN = "\033[0;33m"
    LIGHT_RED = "\033[1;31m"
    LIGHT_GREEN = "\033[1;32m"
    LIGHT_BLUE = "\033[1;34m"
    LIGHT_PURPLE = "\033[1;35m"
    LIGHT_CYAN = "\033[1;36m"
    LIGHT_WHITE = "\033[1;37m"
    LIGHT_GRAY = "\033[0;37m"
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


def build_parser(
    parser: Optional[argparse.ArgumentParser] = None,
) -> argparse.ArgumentParser:
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


def parse_dataset(sim: habitat_sim.Simulator = None, dataset_path: str = None) -> None:
    if sim is None:
        return

    metadata_mediator = sim.metadata_mediator
    if (
        metadata_mediator is None
        or metadata_mediator.dataset_exists(dataset_path) is False
    ):
        return
    active_dataset: str = metadata_mediator.active_dataset
    print_in_color("* " * 39, PrintColors.CYAN)
    print_in_color(f"{active_dataset}\n", PrintColors.CYAN)
    dataset_report: str = metadata_mediator.dataset_report(dataset_path)
    print_in_color("* " * 39, PrintColors.BLUE)
    print_in_color(f"{dataset_report}\n", PrintColors.BLUE)


def make_configuration(sim_settings):
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

    camera_sensor_spec = habitat_sim.CameraSensorSpec()
    camera_sensor_spec.sensor_type = habitat_sim.SensorType.COLOR
    camera_sensor_spec.sensor_subtype = habitat_sim.SensorSubType.PINHOLE
    camera_sensor_spec.resolution = [sim_settings["height"], sim_settings["width"]]
    camera_sensor_spec.position = [0, sim_settings["sensor_height"], 0]

    agent_cfg = habitat_sim.agent.AgentConfiguration()
    agent_cfg.sensor_specifications = [camera_sensor_spec]

    return habitat_sim.Configuration(sim_cfg, [agent_cfg])


def main() -> None:
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
