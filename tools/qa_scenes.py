import math
import os
import time
from typing import Optional

import git
import numpy as np
from matplotlib import pyplot as plt
from PIL import Image

import habitat_sim
from habitat_sim.utils import common as utils
from habitat_sim.utils import viz_utils as vut
from habitat_sim.utils.common import d3_40_colors_rgb
from habitat_sim.utils.settings import default_sim_settings, make_cfg

# get the output directory and data path
repo = git.Repo(".", search_parent_directories=True)
dir_path = repo.working_tree_dir
data_path = os.path.join(dir_path, "data")
output_directory = "qa_scenes_output/"  # @param {type:"string"}
output_path = os.path.join(dir_path, output_directory)
if not os.path.exists(output_path):
    os.mkdir(output_path)

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


# setup the scene
# NOTE: change this as you will
scene_dataset = os.path.join(
    data_path, "scene_datasets/mp3d_example/mp3d.scene_dataset_config.json"
)

sim_settings = default_sim_settings.copy()
sim_settings["enable_physics"] = False  # kinematics only
sim_settings["scene_dataset_config_file"] = scene_dataset
# square image
sim_settings["width"]: 256
sim_settings["height"]: 256


def make_cfg_mm(settings):
    """
    Create a Configuration with an attached MetadataMediator for shared dataset access and re-use
    """
    config = make_cfg(settings)

    # create an attach a MetadataMediator
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
        "physics_realtime_ratio": 0,  # simulation : realtime ration
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


def iteratively_test_all_scenes(config_with_mm, generate_navmesh=False):
    # now iterate over all scenes in the dataset via the mm
    print("-------------------------------")
    print(config_with_mm.metadata_mediator.dataset_report())
    print("SCENES")
    for scene_handle in config_with_mm.metadata_mediator.get_scene_handles():
        print(scene_handle)

    stage_handles = (
        config_with_mm.metadata_mediator.stage_template_manager.get_templates_by_handle_substring()
    )

    # optionally manually cull some problem scenes
    # stage_handles = [x for x in stage_handles if "106366104_174226320" not in x]
    # stage_handles = [x for x in stage_handles if x.split("/")[-1].split(".")[0] in modified_stage_handles]

    print("STAGES")
    failure_log = []
    all_scenes_navmesh_metrics = {}
    for stage_handle in stage_handles:
        print("=================================================")
        print(f"    {stage_handle}")
        config_with_mm.sim_cfg.scene_id = stage_handle
        if stage_handle == "NONE":
            continue
        try:
            with habitat_sim.Simulator(config_with_mm) as sim:
                stage_filename = stage_handle.split("/")[-1]
                if generate_navmesh:
                    stage_directory = stage_handle[: -len(stage_filename)]
                    # scene_dataset_directory = sim.metadata_mediator.active_dataset[:-len(sim.metadata_mediator.active_dataset.split("/")[-1])]
                    # stage_directory = os.path.join(scene_dataset_directory, stage_directory)
                    navmesh_filename = (
                        stage_filename[: -len(stage_filename.split(".")[-1])]
                        + "navmesh"
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
                agent = sim.initialize_agent(sim_settings["default_agent"])
                agent_state = habitat_sim.AgentState()
                for _orientation in range(4):
                    agent_state.rotation = utils.quat_from_angle_axis(
                        theta=_orientation * (math.pi / 2.0), axis=np.array([0, 1, 0])
                    )
                    agent.set_state(agent_state)
                    sim.get_sensor_observations()
                    # to extract the images:
                    # observations = sim.get_sensor_observations()
                    # rgb = observations["color_sensor"]
                    # semantic = observations["semantic_sensor"]
                    # depth = observations["depth_sensor"]
                    # pil_save_obs(output_file=output_path+stage_handle.split("/")[-1]+str(_orientation)+".png", rgb=rgb, semantic=semantic, depth=depth)
                all_scenes_navmesh_metrics[stage_filename] = collect_navmesh_metrics(
                    sim
                )
                save_navmesh_data(sim)

        except Exception as e:
            failure_log.append((stage_handle, e))
        print("=================================================")
    print(f"Failure log = {failure_log}")
    print(
        f"Tried {len(stage_handles)-1} stages."
    )  # manually decrement the "NONE" scene
    print("-------------------------------")
    aggregate_navmesh_metrics(
        all_scenes_navmesh_metrics, filename=output_path + "navmesh_metrics.csv"
    )


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--no-display", dest="display", action="store_false")
    parser.add_argument("--no-make-video", dest="make_video", action="store_false")
    parser.add_argument("--dataset", dest="dataset", type=str)
    parser.set_defaults(show_video=True, make_video=True)
    args, _ = parser.parse_known_args()
    show_video = args.display
    display = args.display
    do_make_video = args.make_video

    sim_settings["scene_dataset_config_file"] = args.dataset

    iteratively_test_all_scenes(make_cfg_mm(sim_settings), generate_navmesh=True)
