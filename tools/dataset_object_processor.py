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
    print(active_dataset)
    dataset_report: str = metadata_mediator.dataset_report(dataset_path)
    print(dataset_report)


def make_configuration(sim_settings):

    # simulator configuration
    sim_cfg = habitat_sim.SimulatorConfiguration()
    sim_cfg.scene_id = sim_settings["scene_dataset_config_file"]
    assert os.path.exists(sim_cfg.scene_id)

    sensor_cfg = habitat_sim.CameraSensorSpec()
    sensor_cfg.resolution = [544, 720]
    agent_cfg = habitat_sim.agent.AgentConfiguration()
    agent_cfg.sensor_specifications = [sensor_cfg]

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
