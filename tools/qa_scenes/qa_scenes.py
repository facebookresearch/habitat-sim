import json
import math
import os
import time
from typing import Any, Dict, Optional

import git
import numpy as np
from colorama import init
from matplotlib import pyplot as plt
from PIL import Image
from qa_scene_utils import ANSICodes, print_if_logging, section_divider_str

import habitat_sim
from habitat_sim.utils import common as utils
from habitat_sim.utils import viz_utils as vut
from habitat_sim.utils.common import d3_40_colors_rgb
from habitat_sim.utils.settings import default_sim_settings, make_cfg

# get the output directory and data path
repo = git.Repo(".", search_parent_directories=True)
dir_path = repo.working_tree_dir
data_path = os.path.join(dir_path, "data")
output_directory = "tools/qa_scenes/qa_scenes_output/"  # @param {type:"string"}
output_path = os.path.join(dir_path, output_directory)
if not os.path.exists(output_path):
    os.mkdir(output_path)

# NOTE: change this as you will
default_config_file = "tools/qa_scenes/configs/default.qa_scene_config.json"

# Change to do something like this maybe: https://stackoverflow.com/a/41432704
def display_sample(
    rgb_obs,
    semantic_obs: Optional[np.array] = None,
    depth_obs: Optional[np.array] = None,
    output_file=None,
):

    print(f"output file = {output_file}")
    rgb_img = Image.fromarray(rgb_obs, mode="RGBA")

    arr = [rgb_img]
    titles = ["rgb"]
    if semantic_obs is not None:
        semantic_img = Image.new("P", (semantic_obs.shape[1], semantic_obs.shape[0]))
        semantic_img.putpalette(d3_40_colors_rgb.flatten())
        semantic_img.putdata((semantic_obs.flatten() % 40).astype(np.uint8))
        semantic_img = semantic_img.convert("RGBA")
        arr.append(semantic_img)
        titles.append("semantic")

    if depth_obs is not None:
        depth_img = Image.fromarray((depth_obs / 10 * 255).astype(np.uint8), mode="L")
        arr.append(depth_img)
        titles.append("depth")

    plt.figure(figsize=(12, 8))
    for i, data in enumerate(arr):
        ax = plt.subplot(1, 3, i + 1)
        ax.axis("off")
        ax.set_title(titles[i])
        plt.imshow(data)

    if output_file is not None:
        print("saving")
        plt.savefig(fname=output_file)
    # else:
    #    plt.show(block=False)
    ...


def pil_save_obs(output_file, rgb=None, semantic=None, depth=None):
    images = []
    if rgb is not None:
        images.append(vut.observation_to_image(rgb, "color"))
    if semantic is not None:
        images.append(vut.observation_to_image(semantic, "semantic"))
    if depth is not None:
        images.append(vut.observation_to_image(depth, "depth"))

    if len(images) == 0:
        print("No images, aborting.")
        return

    concat_image_width = 0
    image_starts = []
    image_height = 0
    for image in images:
        image_starts.append(concat_image_width)
        concat_image_width += image.width
        image_height = max(image_height, image.height)

    contact_image = Image.new("RGB", (concat_image_width, image_height))

    for im_ix, image in enumerate(images):
        contact_image.paste(image, (image_starts[im_ix], 0))

    contact_image.save(output_file)


def make_cfg_mm(settings):
    """
    Create a Configuration with an attached MetadataMediator for shared dataset access and re-use
    """
    config = make_cfg(settings)

    # create and attach a MetadataMediator
    mm = habitat_sim.metadata.MetadataMediator(config.sim_cfg)

    return habitat_sim.Configuration(config.sim_cfg, config.agents, mm)


######################################################
# navmesh metrics code
######################################################


def collect_navmesh_metrics(sim):
    nav_metrics = {}
    if sim.pathfinder.is_loaded:
        nav_metrics["num_islands"] = sim.pathfinder.num_islands
        nav_metrics["island_areas"] = []
        largest_island_area = 0
        second_largest_island_area = 0
        for island_id in range(sim.pathfinder.num_islands):
            nav_metrics["island_areas"].append(sim.pathfinder.island_area(island_id))
            if nav_metrics["island_areas"][island_id] > largest_island_area:
                second_largest_island_area = largest_island_area
                largest_island_area = nav_metrics["island_areas"][island_id]
            elif nav_metrics["island_areas"][island_id] > second_largest_island_area:
                second_largest_island_area = nav_metrics["island_areas"][island_id]
        nav_metrics["island_ratio"] = (
            largest_island_area - second_largest_island_area
        ) / largest_island_area
        nav_metrics["total_area"] = sim.pathfinder.island_area(-1)
        nav_metrics["largest_island_coverage"] = (
            largest_island_area / nav_metrics["total_area"]
        )
    return nav_metrics


def aggregate_navmesh_metrics(all_scenes_navmesh_metrics, filename):
    import csv

    # save a csv of navmesh metrics
    with open(filename, "w") as f:
        writer = csv.writer(f, quoting=csv.QUOTE_ALL)
        writer.writerow(["Scene", "num_islands", "area", "ratio", "coverage", "areas"])
        for scene_hash, metrics in all_scenes_navmesh_metrics.items():
            row_data = [
                scene_hash,
                metrics["num_islands"],
                metrics["total_area"],
                metrics["island_ratio"],
                metrics["largest_island_coverage"],
            ]
            row_data.extend(metrics["island_areas"])
            writer.writerow(row_data)


def save_navmesh_data(sim):
    os.makedirs("navmeshes/", exist_ok=True)
    if sim.pathfinder.is_loaded:
        for island in range(sim.pathfinder.num_islands):
            vert_data = sim.pathfinder.build_navmesh_vertices(island)
            index_data = sim.pathfinder.build_navmesh_vertex_indices(island)
            export_navmesh_data_to_obj(
                filename=f"navmeshes/{island}.obj",
                vertex_data=vert_data,
                index_data=index_data,
            )
    else:
        print("Cannot save navmesh data, no pathfinder loaded")


def export_navmesh_data_to_obj(filename, vertex_data, index_data):
    with open(filename, "w") as f:
        file_data = ""
        for vert in vertex_data:
            file_data += (
                "v " + str(vert[0]) + " " + str(vert[1]) + " " + str(vert[2]) + "\n"
            )
        assert len(index_data) % 3 == 0, "must be triangles"
        for ix in range(int(len(index_data) / 3)):
            # NOTE: obj starts indexing at 1
            file_data += (
                "f "
                + str(index_data[ix * 3] + 1)
                + " "
                + str(index_data[ix * 3 + 1] + 1)
                + " "
                + str(index_data[ix * 3 + 2] + 1)
                + "\n"
            )
        f.write(file_data)


######################################################
# end navmesh metrics code
######################################################


def profile_scene(cfg):
    """
    Profile a scene's performance for rendering, collisions, physics, etc...
    """

    profile_info = {
        "avg_render_time": 0,  # averge time taken to render a frame
        "physics_realtime_ratio": 0,  # simulation : realtime ratio
    }

    with habitat_sim.Simulator(cfg) as sim:
        # profile rendering
        agent = sim.initialize_agent(sim_settings["default_agent"])
        agent_state = habitat_sim.AgentState()
        render_time_total = 0
        orientation_samples = 4
        for _orientation in range(orientation_samples):
            agent_state.rotation = utils.quat_from_angle_axis(
                theta=_orientation * (math.pi / 2.0), axis=np.array([0, 1, 0])
            )
            agent.set_state(agent_state)
            start_time = time.time()
            sim.get_sensor_observations()
            render_time_total += time.time() - start_time
        profile_info["avg_render_time"] = render_time_total / orientation_samples

        # TODO: profile physics
        # sim_horizon = 10  # seconds of simulation to sample over

        # TODO: run collision grid test
        ...


def collision_grid_test(sim):
    # TODO: refactor this
    # run a collision detection grid
    cell_size = 0.5

    obj_templates_mgr = sim.get_object_template_manager()
    rigid_obj_mgr = sim.get_rigid_object_manager()

    scene_bb = sim.get_active_scene_graph().get_root_node().cumulative_bb

    cube_handle = obj_templates_mgr.get_template_handles("cubeSolid")[0]
    cube_template_cpy = obj_templates_mgr.get_template_by_handle(cube_handle)
    cube_template_cpy.scale = np.ones(3) * cell_size
    obj_templates_mgr.register_template(cube_template_cpy, "my_scaled_cube")
    obj = rigid_obj_mgr.add_object_by_template_handle("my_scaled_cube")

    current_cell = scene_bb.min
    while current_cell.x < scene_bb.max.x:
        current_cell.y = scene_bb.min.y
        while current_cell.y < scene_bb.max.y:
            current_cell.z = scene_bb.min.z
            while current_cell.z < scene_bb.max.z:
                # TODO: the check
                obj.translation = current_cell
                print(obj.translation)
                sim.perform_discrete_collision_detection()
                current_cell.z += cell_size
            current_cell.y += cell_size
        current_cell.x += cell_size


def iteratively_test_all_scenes(cfg: habitat_sim.Configuration, generate_navmesh=False):

    mm = cfg.metadata_mediator

    # now iterate over all scenes in the dataset via the mm

    # print dataset report
    text_format = ANSICodes.BRIGHT_CYAN.value
    print_if_logging(None, text_format + "\nDATASET REPORT:\n")
    print_if_logging(None, text_format + section_divider_str)
    print_if_logging(None, text_format + mm.dataset_report())

    # print active dataset
    text_format = ANSICodes.ORANGE.value
    print(text_format + f"Active dataset: {mm.active_dataset}\n")
    print(text_format + section_divider_str)

    # print list of scenes
    text_format = ANSICodes.BRIGHT_MAGENTA.value
    print_if_logging(None, text_format + "SCENES:\n")
    print(text_format + section_divider_str)
    for scene_handle in mm.get_scene_handles():
        print_if_logging(None, text_format + "- " + scene_handle + "\n")

    # get and print list of stage handles
    text_format = ANSICodes.BROWN.value
    print(text_format + "STAGES:\n")
    stage_handles = mm.stage_template_manager.get_templates_by_handle_substring()
    # optionally manually cull some problem scenes
    # stage_handles = [x for x in stage_handles if "106366104_174226320" not in x]
    # stage_handles = [x for x in stage_handles if x.split("/")[-1].split(".")[0] in modified_stage_handles]
    failure_log = []
    all_scenes_navmesh_metrics = {}

    # process each stage
    for stage_handle in stage_handles:
        text_format = ANSICodes.BROWN.value
        print(text_format + section_divider_str)
        print(text_format + f"- {stage_handle}\n")
        cfg.sim_cfg.scene_id = stage_handle

        if stage_handle == "NONE":
            continue

        # attempt to construct simulator
        try:
            with habitat_sim.Simulator(cfg) as sim:
                stage_filename = stage_handle.split("/")[-1]
                text_format = ANSICodes.PURPLE.value
                print_if_logging(
                    sim, text_format + f"\n  -stage filename: {stage_filename} \n"
                )

                # generate and save navmesh
                if generate_navmesh:
                    stage_directory = stage_handle[: -len(stage_filename)]
                    print_if_logging(
                        sim, text_format + f"  -stage directory: {stage_directory} \n"
                    )
                    # scene_dataset_directory = mm.active_dataset[:-len(mm.active_dataset.split("/")[-1])]
                    # stage_directory = os.path.join(scene_dataset_directory, stage_directory)

                    navmesh_filename = (
                        stage_filename[: -len(stage_filename.split(".")[-1])]
                        + "navmesh"
                    )
                    print_if_logging(
                        sim, text_format + f"  -navmesh filename: {navmesh_filename} \n"
                    )
                    navmesh_settings = habitat_sim.NavMeshSettings()
                    navmesh_settings.set_defaults()
                    sim.recompute_navmesh(sim.pathfinder, navmesh_settings)

                    if os.path.exists(stage_directory):
                        sim.pathfinder.save_nav_mesh(stage_directory + navmesh_filename)
                    else:
                        failure_log.append(
                            (
                                stage_handle,
                                f"No target directory for navmesh: {stage_directory}",
                            )
                        )

                # init agent and get its sensor observations from 4 different poses
                agent = sim.initialize_agent(sim_settings["default_agent"])
                agent_state = habitat_sim.AgentState()
                for _orientation in range(4):
                    agent_state.rotation = utils.quat_from_angle_axis(
                        theta=_orientation * (math.pi / 2.0), axis=np.array([0, 1, 0])
                    )
                    agent.set_state(agent_state)
                    sim.get_sensor_observations()

                    # # to extract the images:
                    # observations = sim.get_sensor_observations()
                    # rgb = observations["color_sensor"]
                    # semantic = observations["semantic_sensor"]
                    # depth = observations["depth_sensor"]
                    # pil_save_obs(output_file=output_path+stage_handle.split("/")[-1]+str(_orientation)+".png", rgb=rgb, semantic=semantic, depth=depth)

                # process navmesh
                all_scenes_navmesh_metrics[stage_filename] = collect_navmesh_metrics(
                    sim
                )
                save_navmesh_data(sim)

        except Exception as e:
            failure_log.append((stage_handle, e))

    # print failure log
    text_format = ANSICodes.GREEN.value
    print(text_format + f"\nFailure log = {failure_log}\n")

    # print number of stages we attempted to process
    print(
        text_format + f"Tried {len(stage_handles)-1} stages.\n"
    )  # manually decrement the "NONE" scene
    print(text_format + section_divider_str)

    # create cvs detailing navmesh metrics
    aggregate_navmesh_metrics(
        all_scenes_navmesh_metrics, filename=output_path + "navmesh_metrics.csv"
    )

    text_format = ANSICodes.BRIGHT_RED.value
    print(text_format + "PROCESSING COMPLETE\n")
    print(text_format + section_divider_str)


def process_config_json_file(
    sim_settings: Dict[str, Any], config_json
) -> Dict[str, Any]:
    """
    Update nested sim_settings dictionary. Modifies sim_settings in place.
    """
    for key, value in config_json.items():
        if isinstance(value, Dict) and value:
            nested_settings = process_config_json_file(sim_settings.get(key, {}), value)
            sim_settings[key] = nested_settings
        else:
            sim_settings[key] = config_json[key]

    return sim_settings


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config_file_path",
        dest="config_file_path",
        type=str,
        help="config file to load" f' (default: "{default_config_file}")',
    )
    parser.set_defaults(config_file_path=os.path.join(dir_path, default_config_file))
    args, _ = parser.parse_known_args()

    # Populate sim_settings with info from dataset_processor_config.json file
    sim_settings: Dict[str, Any] = default_sim_settings.copy()

    with open(os.path.join(dir_path, args.config_file_path)) as config_json:
        process_config_json_file(sim_settings, json.load(config_json))

    sim_settings["scene_dataset_config_file"] = os.path.join(
        data_path, sim_settings["scene_dataset_config_file"]
    )

    # setup colored console print statement logic
    init(autoreset=True)

    # create sim config and process scenes/stages
    text_format = ANSICodes.BRIGHT_RED.value
    print(text_format + "\nBEGIN PROCESSING:\n")
    print(text_format + section_divider_str)

    cfg = make_cfg_mm(sim_settings)
    iteratively_test_all_scenes(cfg, generate_navmesh=True)
