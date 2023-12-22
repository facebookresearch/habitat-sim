# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import ctypes
import json
import math
import os
import sys
import time
from typing import Any, Dict, List, Optional, Set, Tuple, Union

FLAGS = sys.getdlopenflags()
sys.setdlopenflags(FLAGS | ctypes.RTLD_GLOBAL)

import git
import magnum as mn
import numpy as np
from colorama import init
from PIL import Image
from qa_scene_settings import default_sim_settings
from qa_scene_utils import (  # print_dataset_info,
    ANSICodes,
    create_unique_filename,
    print_if_logging,
    section_divider_str,
)

import habitat_sim
from habitat_sim.agent import Agent, AgentState

# from habitat_sim.physics import MotionType
# from habitat_sim.simulator import ObservationDict
from habitat_sim.utils import common as utils
from habitat_sim.utils import viz_utils as vut
from habitat_sim.utils.settings import make_cfg
from habitat_sim.utils.sim_utils import get_floor_navigable_extents

# clean up types with TypeVars
NavmeshMetrics = Dict[str, Union[int, float]]

# get the data path
REPO = git.Repo(".", search_parent_directories=True)
DIR_PATH = REPO.working_tree_dir
DATA_PATH = os.path.join(DIR_PATH, "data")

# get the output directory
OUTPUT_DIRECTORY = "./tools/qa_scenes/qa_scenes_output/"  # @param {type:"string"}
OUTPUT_PATH_BASE = os.path.join(DIR_PATH, OUTPUT_DIRECTORY)
os.makedirs(OUTPUT_PATH_BASE, exist_ok=True)
# build subpaths for stage and scene output

OUTPUT_PATH_DICT = {}
OUTPUT_PATH_DICT["base"] = OUTPUT_PATH_BASE
for subdir in ["stage", "scene"]:
    tmp_path = os.path.join(OUTPUT_PATH_BASE, subdir + "/")
    OUTPUT_PATH_DICT[subdir] = tmp_path
    os.makedirs(tmp_path, exist_ok=True)

# if there are no "scene_instance.json" files in this dataset, create default scenes
# in this folder using the stages in the dataset instead.
DEFAULT_SCENE_DIR = os.path.join(DATA_PATH, "default_qa_scenes")
os.makedirs(DEFAULT_SCENE_DIR, exist_ok=True)

MAX_TEST_TIME = sys.float_info.max


def pil_save_obs(
    output_file,
    rgb: np.ndarray,
    semantic: Optional[np.array] = None,
    depth: Optional[np.array] = None,
) -> None:
    images = []
    if rgb is not None:
        images.append(vut.observation_to_image(rgb, "color"))
    if semantic is not None:
        images.append(vut.observation_to_image(semantic, "semantic"))
    if depth is not None:
        images.append(vut.observation_to_image(depth, "depth"))

    if len(images) == 0:
        print_if_logging(silent, "No images, aborting.")
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


######################################################
# navmesh metrics code
######################################################


def collect_navmesh_metrics(sim: habitat_sim.Simulator) -> NavmeshMetrics:
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


def aggregate_navmesh_metrics(
    all_scenes_navmesh_metrics: Dict[str, NavmeshMetrics], filename
) -> None:
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


def save_navmesh_data(sim: habitat_sim.Simulator):
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
        print_if_logging(silent, "Cannot save navmesh data, no pathfinder loaded")


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


######################################################
# scene processing timed test code
######################################################


def place_scene_topdown_camera(sim: habitat_sim.Simulator, agent: Agent) -> None:
    """
    Place the camera in the scene center looking down.
    """
    scene_bb = sim.get_active_scene_graph().get_root_node().cumulative_bb
    max_dim = max(scene_bb.size_x(), scene_bb.size_z())
    cam_pos = scene_bb.center()
    cam_pos[1] += 0.52 * max_dim + scene_bb.size_y() / 2.0
    look_down = mn.Quaternion.from_matrix(
        mn.Matrix4.look_at(
            eye=cam_pos, target=scene_bb.center(), up=mn.Vector3.x_axis()
        ).rotation()
    )
    agent.scene_node.translation = cam_pos
    agent.scene_node.rotation = look_down


def render_sensor_observations(
    sim: habitat_sim.Simulator,
    scene_filename: str,
    output_filepath: str,
    min_max_avg_render_times: Dict[str, List[float]],
    detailed_info_json_dict: Dict[str, Any],
    failure_log: List[Tuple[str, Any]],
) -> None:
    """ """
    text_format = ANSICodes.PURPLE.value
    print_if_logging(
        silent, text_format + f"\nRendering sensor obs for scene: {scene_filename}"
    )

    # init agent
    agent = sim.initialize_agent(sim_settings["default_agent"])
    agent_state = AgentState()

    # need sensor observation for each cardinal direction and from above
    cardinal_directions: int = 4
    num_poses: int = cardinal_directions + 1
    rgb: List[np.ndarray] = [None] * num_poses
    semantic: List[np.ndarray] = [None] * num_poses
    depth: List[np.ndarray] = [None] * num_poses

    # take sensor observation for each cardinal direction
    render_time_total: float = 0.0
    min_render_time: float = MAX_TEST_TIME
    max_render_time: float = 0.0
    render_times_for_pose: List[float] = [0.0] * num_poses

    try:
        for pose_num in range(num_poses):
            image_filename_suffix: str = ""
            if pose_num == num_poses - 1:
                # take sensor observation from above with all objects visible
                place_scene_topdown_camera(sim, agent)
                image_filename_suffix = "__from_above"
            else:
                # take sensor observation from horizontal direction
                agent_state.rotation = utils.quat_from_angle_axis(
                    theta=pose_num * (math.pi / 2.0), axis=np.array([0, 1, 0])
                )
                agent.set_state(agent_state)
                angle_deg: int = pose_num * 90
                image_filename_suffix = f"__{angle_deg}_deg"

            start_time: float = time.time()
            observations = sim.get_sensor_observations()
            render_times_for_pose[pose_num]: float = time.time() - start_time

            if render_times_for_pose[pose_num] < min_render_time:
                min_render_time = render_times_for_pose[pose_num]
            if render_times_for_pose[pose_num] > max_render_time:
                max_render_time = render_times_for_pose[pose_num]

            render_time_total += render_times_for_pose[pose_num]

            if "color_sensor" in observations:
                rgb[pose_num] = observations["color_sensor"]
            if "semantic_sensor" in observations:
                semantic[pose_num] = observations["semantic_sensor"]
            if "depth_sensor" in observations:
                depth[pose_num] = observations["depth_sensor"]

            pil_save_obs(
                output_file=output_filepath
                + scene_filename.split(".")[0]
                + image_filename_suffix
                + ".png",
                rgb=rgb[pose_num],
                semantic=semantic[pose_num],
                depth=depth[pose_num],
            )

        avg_render_time: float = render_time_total / num_poses

        # save min, max, and avg render times for this scene
        min_max_avg_render_times[scene_filename] = [
            min_render_time,
            max_render_time,
            avg_render_time,
        ]

    except Exception as e:
        msg = f"error in render_sensor_observations(...) for scene: {scene_filename}"
        failure_log.append((msg, e))

        # save min, max, and avg render times for this scene
        min_max_avg_render_times[scene_filename] = [
            MAX_TEST_TIME,
            MAX_TEST_TIME,
            MAX_TEST_TIME,
        ]

    # construct json entry for exhaustive render test info and append it to list
    # variables to store exhaustive collision test info for json
    render_test_json_info: List[Dict[str, Any]] = []
    render_test_constants: Dict[str, Any] = {
        "render_test_max_time": sim_settings["render_test_max_time"],
    }
    render_test_json_info.append(render_test_constants)

    json_entry: Dict[str, Any] = sim_settings["render_test_json_entry"].copy()
    json_entry["cardinal_dir_render_times"] = [
        render_times_for_pose[0],
        render_times_for_pose[1],
        render_times_for_pose[2],
        render_times_for_pose[3],
    ]
    json_entry["overhead_render_time"] = render_times_for_pose[4]

    json_entry["min_render_time"] = min_max_avg_render_times[scene_filename][0]
    json_entry["max_render_time"] = min_max_avg_render_times[scene_filename][1]
    json_entry["avg_render_time"] = min_max_avg_render_times[scene_filename][2]

    json_entry["exceeds_time_threshold"] = bool(
        min_max_avg_render_times[scene_filename][1]
        > sim_settings["render_test_max_time"]
    )
    render_test_json_info.append(json_entry)

    # add this render test info to our comprehensive json dict
    detailed_info_json_dict[scene_filename].update(
        {"render_test": render_test_json_info}
    )


def collision_grid_test(
    sim: habitat_sim.Simulator,
    scene_filename: str,
    min_max_avg_collision_times: Dict[str, List[float]],
    detailed_info_json_dict: Dict[str, Any],
    failure_log: List[Tuple[str, Any]],
):
    """ """
    text_format = ANSICodes.PURPLE.value
    print_if_logging(
        silent, text_format + f"\nRunning collision test for scene: {scene_filename}"
    )

    # create scaled cube to iterate through scene and test its collisions
    cell_size = sim_settings["collision_test_cell_size"]

    # scales from center of cube, so we scale halfway in both positive and
    # negative directions for each dimension
    scale_factor = 0.5 * cell_size

    obj_templates_mgr = sim.get_object_template_manager()
    rigid_obj_mgr = sim.get_rigid_object_manager()

    scene_bb = sim.get_active_scene_graph().get_root_node().cumulative_bb

    cube_handle = obj_templates_mgr.get_template_handles("cubeSolid")[0]
    cube_template_cpy = obj_templates_mgr.get_template_by_handle(cube_handle)
    cube_template_cpy.scale = np.ones(3) * scale_factor

    obj_handle = "my_scaled_cube"
    obj_templates_mgr.register_template(cube_template_cpy, obj_handle)
    scaled_cube = rigid_obj_mgr.add_object_by_template_handle(obj_handle)

    grid_pos = scene_bb.min

    # variables to store exhaustive collision test info for json
    collision_test_json_info: List[Dict[str, Any]] = []
    collision_test_overall_data: Dict[str, Any] = {
        "--": "Overall Collision Test Info",
        "collision_test_cell_size": sim_settings["collision_test_cell_size"],
        "max_time_threshold": sim_settings["collision_test_max_time_threshold"],
    }
    collision_test_json_info.append(collision_test_overall_data)
    test_num: int = 1
    ijk_indices = [0, 0, 0]

    # iterate cube through a 3D grid of positions, checking its collisions at
    # each one.
    min_collision_time: float = MAX_TEST_TIME
    max_collision_time: float = 0.0
    total_collision_time: float = 0.0
    avg_collision_time: float = MAX_TEST_TIME

    running_collision_test = True
    while running_collision_test:
        scaled_cube.translation = grid_pos
        try:
            start_time = time.time()
            sim.perform_discrete_collision_detection()
            collision_test_time = time.time() - start_time
            total_collision_time += collision_test_time

            # determine if this is the min or max collision time so far
            if collision_test_time < min_collision_time:
                min_collision_time = collision_test_time
            if collision_test_time > max_collision_time:
                max_collision_time = collision_test_time

            # construct json entry for exhaustive collision test info and append it to list
            entry: Dict[str, Any] = sim_settings["collision_test_json_entry"].copy()
            entry["collision_test_num"] = test_num
            entry["ijk_indices"] = [ijk_indices[0], ijk_indices[1], ijk_indices[2]]
            entry["pos"] = [grid_pos.x, grid_pos.y, grid_pos.z]
            entry["test_time"] = collision_test_time
            entry["exceeds_time_threshold"] = bool(
                collision_test_time > sim_settings["collision_test_max_time_threshold"]
            )
            collision_test_json_info.append(entry)

        except Exception as e:
            # store any exceptions raised when constructing simulator
            msg = f"failure in collision_grid_test(...) for scene: {scene_filename}\nat cube position: {grid_pos}"
            failure_log.append((msg, e))

        # TODO: the following code would probably be clearer as a nested for-loop.
        # That way we don't have to manually track the i,j, and k indices

        # update z coordinate
        grid_pos.z += cell_size
        ijk_indices[2] += 1

        # if z coordinate exceeds max
        if grid_pos.z >= scene_bb.max.z:
            # reset z coordinate and update y coordinate
            grid_pos.z = scene_bb.min.z
            ijk_indices[2] = 0

            grid_pos.y += cell_size
            ijk_indices[1] += 1

            # if y coordinate exceeds max
            if grid_pos.y >= scene_bb.max.y:
                # reset y coordinate and update x coordinate
                grid_pos.y = scene_bb.min.y
                ijk_indices[1] = 0

                grid_pos.x += cell_size
                ijk_indices[0] += 1

                # if x coordinate exceeds max
                if grid_pos.x >= scene_bb.max.x:
                    # we are done running collision test
                    running_collision_test = False

        test_num += 1

    # save collision time summary to save in csv later
    avg_collision_time = total_collision_time / test_num
    min_max_avg_collision_times[scene_filename] = [
        min_collision_time,
        max_collision_time,
        avg_collision_time,
    ]

    # add this collision test info to our comprehensive json dict
    collision_test_overall_data["min_collision_time"] = min_collision_time
    collision_test_overall_data["max_collision_time"] = max_collision_time
    collision_test_overall_data["avg_collision_time"] = avg_collision_time
    if sim_settings["collision_test_save_detail_json"]:
        detailed_info_json_dict[scene_filename].update(
            {"collision_test": collision_test_json_info}
        )

    rigid_obj_mgr.remove_object_by_handle(scaled_cube.handle)


def asset_sleep_test(
    sim: habitat_sim.Simulator,
    scene_filename: str,
    min_max_avg_asleep_times: Dict[str, List[float]],
    detailed_info_json_dict: Dict[str, Any],
    failure_log: List[Tuple[str, Any]],
) -> None:
    """
    step world for how many seconds specified in the config file and record
    how long it takes for each rigid object to fall asleep, if at all.
    """
    # get object attributes manager, rigid object manager, and get asset filenames
    obj_templates_mgr = sim.get_object_template_manager()
    obj_templates_mgr.load_configs(sim_settings["scene_dataset_config_file"])
    rigid_obj_mgr = sim.get_rigid_object_manager()

    if rigid_obj_mgr.get_num_objects() == 0:
        failure_log.append(
            (
                "No dataset objects in scene to perform asset sleep test on.",
                scene_filename,
            )
        )
        min_max_avg_asleep_times[scene_filename] = [
            0.0,
            0.0,
            0.0,
        ]
        return

    rigid_obj_handles: List[str] = rigid_obj_mgr.get_object_handles("")

    # store times it takes for each object to fall asleep, as well as the min,
    # max, and avg times
    all_asleep_times: Dict[str, float] = {}
    min_asleep_time: float = MAX_TEST_TIME
    max_asleep_time: float = 0.0
    avg_asleep_time: float = MAX_TEST_TIME

    # variables to store exhaustive sleep test info for json
    asleep_test_json_info: List[Dict[str, Any]] = []
    asleep_test_overall_data: Dict[str, Any] = {
        "--": "Overall Sleep Test Summary",
        "asset_sleep_test_duration_seconds": sim_settings[
            "asset_sleep_test_duration_seconds"
        ],
    }
    asleep_test_json_info.append(asleep_test_overall_data)

    # Step physics at given fps for given number of seconds and test if the
    # dataset objects are asleep
    physics_step_dur: float = 1.0 / sim_settings["sleep_test_steps_per_sec"]
    curr_sim_time: float = 0.0
    test_duration: float = sim_settings["asset_sleep_test_duration_seconds"]

    total_obj_asleep: int = 0
    while curr_sim_time <= test_duration:
        for handle in rigid_obj_handles:
            rigid_obj = rigid_obj_mgr.get_object_by_handle(handle)

            # object is asleep, record the current time
            if not rigid_obj.awake and all_asleep_times.get(handle) is None:
                all_asleep_times[handle] = curr_sim_time
                if curr_sim_time < min_asleep_time:
                    min_asleep_time = curr_sim_time
                if curr_sim_time > max_asleep_time:
                    max_asleep_time = curr_sim_time

                total_obj_asleep += 1
                if total_obj_asleep == len(rigid_obj_handles):
                    break

        sim.step_world(physics_step_dur)
        curr_sim_time += physics_step_dur

    # after stepping physics for the given number of seconds, check if any
    # objects are still awake and store the results in a comprehensive json
    any_obj_awake: bool = False
    asleep_time_total: float = 0.0
    for handle in rigid_obj_handles:
        rigid_obj = rigid_obj_mgr.get_object_by_handle(handle)
        if rigid_obj.awake:
            any_obj_awake = True
            max_asleep_time = MAX_TEST_TIME
            all_asleep_times[handle] = MAX_TEST_TIME
        else:
            asleep_time_total += all_asleep_times[handle]

        # construct json entry for exhaustive collision test info and append it to list
        entry: Dict[str, Any] = sim_settings["asleep_test_json_entry"].copy()
        entry["asset_handle"] = handle
        entry["test_time"] = all_asleep_times[handle]
        entry["successfully_sleeps"] = not rigid_obj.awake
        asleep_test_json_info.append(entry)

    if not any_obj_awake:
        avg_asleep_time = asleep_time_total / len(rigid_obj_handles)

    min_max_avg_asleep_times[scene_filename] = [
        min_asleep_time,
        max_asleep_time,
        avg_asleep_time,
    ]

    # update asleep test info summary (min, max, and avg sleep times) and add this
    # sleep test info to our comprehensive json dict
    asleep_test_overall_data["min_asleep_time"] = min_asleep_time
    asleep_test_overall_data["max_asleep_time"] = max_asleep_time
    asleep_test_overall_data["avg_asleep_time"] = avg_asleep_time
    detailed_info_json_dict[scene_filename].update(
        {"asleep_test": asleep_test_json_info}
    )


def save_test_times_csv(
    scene_handles: List[str],
    start_index: int,
    end_index: int,
    min_max_avg_render_times: Dict[str, List[float]],
    min_max_avg_collision_times: Dict[str, List[float]],
    min_max_avg_asleep_times: Dict[str, List[float]],
    filename: str,
) -> None:
    import csv

    # save a csv of navmesh metrics
    with open(filename, "w") as f:
        writer = csv.writer(f, quoting=csv.QUOTE_ALL)
        writer.writerow(
            [
                "Scene",
                "Render Times ->",
                "min",
                "max",
                "avg",
                "Collision Times ->",
                "min",
                "max",
                "avg",
                "Sleep Times ->",
                "min",
                "max",
                "avg",
            ]
        )
        for i in range(start_index, end_index):
            if scene_handles[i] == "NONE":
                continue

            scene_filename = scene_handles[i].split("/")[-1]
            row_data = [scene_filename, ""]
            if (
                sim_settings["render_sensor_obs"]
                and scene_filename in min_max_avg_render_times
            ):
                row_data.append(min_max_avg_render_times[scene_filename][0])
                row_data.append(min_max_avg_render_times[scene_filename][1])
                row_data.append(min_max_avg_render_times[scene_filename][2])
            else:
                row_data.append("")
                row_data.append("")
                row_data.append("")
            row_data.append("")

            if (
                sim_settings["run_collision_test"]
                and scene_filename in min_max_avg_collision_times
            ):
                row_data.append(min_max_avg_collision_times[scene_filename][0])
                row_data.append(min_max_avg_collision_times[scene_filename][1])
                row_data.append(min_max_avg_collision_times[scene_filename][2])
            else:
                row_data.append("")
                row_data.append("")
                row_data.append("")
            row_data.append("")

            if (
                sim_settings["run_asset_sleep_test"]
                and scene_filename in min_max_avg_asleep_times
            ):
                row_data.append(min_max_avg_asleep_times[scene_filename][0])
                row_data.append(min_max_avg_asleep_times[scene_filename][1])
                row_data.append(min_max_avg_asleep_times[scene_filename][2])
            else:
                row_data.append("")
                row_data.append("")
                row_data.append("")

            writer.writerow(row_data)


######################################################
# end scene processing timed test code
######################################################


def process_scene_or_stage(
    cfg: habitat_sim.Configuration,
    scene_handle: str,
    all_scenes_navmesh_metrics: Dict[str, NavmeshMetrics],
    min_max_avg_render_times: Dict[str, List[float]],
    min_max_avg_collision_times: Dict[str, List[float]],
    min_max_avg_asleep_times: Dict[str, List[float]],
    detailed_info_json_dict: Dict[str, Any],
    failure_log: List[Tuple[str, Any]],
):
    cfg.sim_cfg.scene_id = scene_handle

    # print scene handle
    text_format = ANSICodes.BRIGHT_MAGENTA.value
    print_if_logging(silent, text_format + section_divider_str)
    print_if_logging(silent, text_format + f"-{scene_handle}\n")

    try:
        # track which stage we're on for more accurate failure logging
        process_scene_or_stage_phase = "initialization"
        with habitat_sim.Simulator(cfg) as sim:
            text_format = ANSICodes.BRIGHT_RED.value
            # TODO Needs to be split based on os path sep.
            # use os.path.basename(scene_handle)
            scene_filename = scene_handle.split("/")[-1]
            # Determine whether stage or scene
            is_stage = scene_filename.find("stage_config") != -1
            file_type = "stage" if is_stage else "scene"
            output_filepath = OUTPUT_PATH_DICT[file_type]
            print_if_logging(
                silent,
                text_format + f"\n\t---processing {file_type}: {scene_filename}\n",
            )
            detailed_info_json_dict[scene_filename] = {}

            if sim_settings["generate_navmesh"]:
                process_scene_or_stage_phase = "generating navmesh"
                # create navmesh settings and re-compute navmesh
                navmesh_settings = habitat_sim.NavMeshSettings()
                navmesh_settings.set_defaults()
                sim.recompute_navmesh(
                    sim.pathfinder,
                    navmesh_settings,
                    include_static_objects=sim_settings["navmesh_include_static"],
                )

            process_scene_or_stage_phase = "navmesh metrics"
            # compute the navmesh metrics and cache the results
            all_scenes_navmesh_metrics[scene_filename] = collect_navmesh_metrics(sim)

            process_scene_or_stage_phase = "island topdown map"

            # save a topdown island maps of all floors in the scene
            if sim_settings["save_navmesh_island_maps"]:
                floor_extents = get_floor_navigable_extents(sim)
                for f_ix, floor in enumerate(floor_extents):
                    floor_height = floor["mean"]
                    island_top_down_map = sim.pathfinder.get_topdown_island_view(
                        0.1, floor_height
                    )
                    island_colored_map_image = vut.get_island_colored_map(
                        island_top_down_map
                    )
                    island_colored_map_image.save(
                        output_filepath
                        + scene_filename.split(".")[0]
                        + f"_floor_{f_ix}_navmesh_islands.png"
                    )

            # Get sensor observations from 5 different poses.
            # Record all rendering times, as well as min, max, and avg rendering time.
            # Save the observations in an image and the times in a json/csv
            if sim_settings["render_sensor_obs"]:
                process_scene_or_stage_phase = "render_sensor_observations"
                render_sensor_observations(
                    sim,
                    scene_filename,
                    output_filepath,
                    min_max_avg_render_times,
                    detailed_info_json_dict,
                    failure_log,
                )

            # run collision grid test, save all test times, as well as min, max, and
            # avgerage times in a json/csv
            if sim_settings["run_collision_test"]:
                process_scene_or_stage_phase = "collision_grid_test"
                collision_grid_test(
                    sim,
                    scene_filename,
                    min_max_avg_collision_times,
                    detailed_info_json_dict,
                    failure_log,
                )

            # run physics for a set number of seconds (default to 10 seconds) to see
            # how long the assets take to go to sleep
            if sim_settings["run_asset_sleep_test"]:
                process_scene_or_stage_phase = "asset_sleep_test"
                asset_sleep_test(
                    sim,
                    scene_filename,
                    min_max_avg_asleep_times,
                    detailed_info_json_dict,
                    failure_log,
                )

    except Exception as e:
        # store any exceptions raised when processing scene
        msg = f"error with scene {scene_handle} in process_scene(...) function phase '{process_scene_or_stage_phase}'"
        failure_log.append((msg, e))


def strip_path(handle: str) -> str:
    handle_cpy = handle
    handle_sans_path = handle_cpy.split("/")[-1]
    return handle_sans_path


def process_scenes_and_stages(
    cfg_with_mm: habitat_sim.Configuration, sim_settings: Dict[str, Any]
) -> None:
    mm = cfg_with_mm.metadata_mediator

    # get scene handles
    scene_handles: List[str] = []

    # get all pre-defined scene instances from MetadataMediator
    if sim_settings["process_scene_instances"]:
        scene_handles.extend(mm.get_scene_handles())

    # treat stages as simple scenes with just the stage and no other assets
    if sim_settings["process_stages"]:
        scene_handles.extend(
            mm.stage_template_manager.get_templates_by_handle_substring()
        )

    # determine indices of scenes to process
    start_index = 0
    end_index = len(scene_handles)

    # determine if start index provided in config is valid
    if (
        isinstance(sim_settings["start_scene_index"], int)
        and sim_settings["start_scene_index"] >= 0
        and sim_settings["start_scene_index"] < len(scene_handles)
    ):
        start_index = sim_settings["start_scene_index"]

    # determine if end index provided in config is valid
    if (
        isinstance(sim_settings["end_scene_index"], int)
        and sim_settings["end_scene_index"] >= sim_settings["start_scene_index"]
        and sim_settings["end_scene_index"] < len(scene_handles)
    ):
        # end index is exclusive
        end_index = sim_settings["end_scene_index"] + 1

    text_format = ANSICodes.BRIGHT_MAGENTA.value
    print_if_logging(silent, text_format + "HANDLES TO PROCESS")

    # this set keeps track of whether or not a stage or scene with the same name
    # has already been processed. Sometimes stages and scenes have the same name
    # but different folders and extensions, so we don't want to do duplicate work
    scenes_processed: Set[str] = set()

    # data to write to csvs and json, as well as a failure log
    all_scenes_navmesh_metrics: Dict[str, NavmeshMetrics] = {}
    min_max_avg_render_times: Dict[str, List[float]] = {}
    min_max_avg_collision_times: Dict[str, List[float]] = {}
    min_max_avg_asleep_times: Dict[str, List[float]] = {}
    detailed_info_json_dict: Dict[str, Any] = {}
    failure_log: List[Tuple[str, Any]] = []

    # process specified scenes and stages
    print(f"Start loop for {start_index} to {end_index}")
    for i in range(start_index, end_index):
        if scene_handles[i] == "NONE":
            continue
        handle_sans_path = strip_path(scene_handles[i])
        if handle_sans_path in scenes_processed:
            continue
        process_scene_or_stage(
            cfg,
            scene_handles[i],
            all_scenes_navmesh_metrics,
            min_max_avg_render_times,
            min_max_avg_collision_times,
            min_max_avg_asleep_times,
            detailed_info_json_dict,
            failure_log,
        )
        scenes_processed.add(handle_sans_path)

    # create cvs detailing scene navmesh metrics
    navmesh_cvs_filename = create_unique_filename(
        dir_path=OUTPUT_PATH_DICT["base"],
        filename_prefix=sim_settings["output_file_prefix"],
        filename_suffix="scene_navmesh_metrics",
        extension=".csv",
    )
    aggregate_navmesh_metrics(all_scenes_navmesh_metrics, filename=navmesh_cvs_filename)

    # create cvs detailing rendering, collision, and asset sleep test metric
    # summaries. More comprehensive metrics will be stored in a json
    test_times_cvs_filename = create_unique_filename(
        dir_path=OUTPUT_PATH_DICT["base"],
        filename_prefix=sim_settings["output_file_prefix"],
        filename_suffix="test_time_metrics",
        extension=".csv",
    )
    save_test_times_csv(
        scene_handles,
        start_index,
        end_index,
        min_max_avg_render_times,
        min_max_avg_collision_times,
        min_max_avg_asleep_times,
        test_times_cvs_filename,
    )

    # save json file with comprehensive test data for each scene
    json_filename = create_unique_filename(
        dir_path=OUTPUT_PATH_DICT["base"],
        filename_prefix=sim_settings["output_file_prefix"],
        filename_suffix="detailed_info",
        extension=".json",
    )
    with open(json_filename, "w") as outfile:
        json.dump(detailed_info_json_dict, fp=outfile, indent=2)

    # print failure log
    text_format = ANSICodes.GREEN.value
    print_if_logging(silent, text_format + "\nOverall failure log:")
    for error in failure_log:
        print_if_logging(silent, text_format + f"{error}")
    print_if_logging(silent, text_format + section_divider_str)

    # print number of scenes and stages we attempted to process
    print_if_logging(
        silent, text_format + f"\nTried {len(scenes_processed)} scenes and stages."
    )
    print_if_logging(silent, text_format + section_divider_str)


def construct_default_scenes(stage_handles: List[str]) -> List[str]:
    """ """
    text_format = ANSICodes.BRIGHT_RED.value
    print_if_logging(
        silent,
        text_format
        + "No scenes available; constructing defaults using available stages.\n",
    )

    non_null_stage_handles: List[str] = []
    for stage_handle in stage_handles:
        if stage_handle != "NONE":
            non_null_stage_handles.append(stage_handle)

    if len(non_null_stage_handles) == 0:
        print_if_logging(silent, text_format + "No stages available either.\n")

    return []


def parse_config_json_file(
    sim_settings: Dict[str, Any],
    config_json,
) -> Dict[str, Any]:
    """
    Update possibly nested sim_settings dictionary from config file.
    Modifies sim_settings in place.
    """
    for key, value in config_json.items():
        if isinstance(value, Dict) and value:
            nested_settings = parse_config_json_file(sim_settings.get(key, {}), value)
            sim_settings[key] = nested_settings
        else:
            sim_settings[key] = config_json[key]

    return sim_settings


def make_cfg_mm(sim_settings: Dict[str, Any]) -> habitat_sim.Configuration:
    """
    Create a Configuration with an attached MetadataMediator for shared dataset access and re-use
    """
    config = make_cfg(sim_settings)

    # create and attach a MetadataMediator
    mm = habitat_sim.metadata.MetadataMediator(config.sim_cfg)

    return habitat_sim.Configuration(config.sim_cfg, config.agents, mm)


if __name__ == "__main__":
    import argparse

    # parse command line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config_file_path",
        dest="config_file_path",
        type=str,
        help="config file to load (default replicaCAD)",
    )
    args, _ = parser.parse_known_args()

    # Populate sim_settings with data from qa_scene_config.json file
    sim_settings: Dict[str, Any] = default_sim_settings.copy()
    if args.config_file_path:
        with open(os.path.join(DIR_PATH, args.config_file_path)) as config_json:
            parse_config_json_file(sim_settings, json.load(config_json))

    # dataset paths in config are relative to data folder
    sim_settings["scene_dataset_config_file"] = os.path.join(
        DATA_PATH, sim_settings["scene_dataset_config_file"]
    )

    # setup colored console print statement logic
    init(autoreset=True)

    # whether or not we are making print statements
    global silent
    silent = sim_settings["silent"]

    # begin processing
    text_format = ANSICodes.BRIGHT_RED.value
    print_if_logging(silent, text_format + "\nBEGIN PROCESSING")
    print_if_logging(silent, text_format + section_divider_str)

    if sim_settings["run_viewer"]:
        # create viewer app
        # QASceneProcessingViewer(sim_settings).exec()
        raise NotImplementedError
        # TODO: add a viewer module extension
    else:
        # make simulator configuration and process all scenes without viewing
        # them in app
        cfg = make_cfg_mm(sim_settings)
        process_scenes_and_stages(cfg, sim_settings)

    # done processing
    text_format = ANSICodes.BRIGHT_RED.value
    print_if_logging(silent, text_format + "\nPROCESSING COMPLETE")
    print_if_logging(silent, text_format + section_divider_str)
