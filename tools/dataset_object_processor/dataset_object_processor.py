import argparse
import gc
import json
import os
import time
from ntpath import basename
from typing import Any, Dict, List, Optional, Tuple

import git
import magnum as mn
import psutil
from colorama import Fore, init
from magnum import trade
from processor_settings import default_sim_settings
from processor_utils import (
    ANSICodes,
    DatasetProcessorSim,
    RotationAxis,
    configure_sim,
    create_csv_file,
    create_video,
    get_csv_headers,
    get_mem_size_str,
    get_multi_sample_ram_use,
    print_debug,
    print_if_logging,
    print_mem_usage_info,
    section_divider,
    snap_down_object,
    update_sim_settings,
)

from habitat_sim import attributes_managers, physics
from habitat_sim.logging import logger

repo = git.Repo(".", search_parent_directories=True)
HABITAT_SIM_PATH = repo.working_tree_dir


def record_revolving_obj(
    sim: DatasetProcessorSim,
    revolution_angle: float,
    angle_delta: float,
) -> Dict[int, Any]:
    """
    record object rotating about its x-axis, then its y-axis

    :param sim: DatasetProcessorSim(hsim.Simulator) instance
    :param angle_delta: how much the object should rotate every frame
    """
    last_rotation = sim.curr_obj.rotation
    if (
        sim.draw_task == "draw_collision_mesh"
        or sim.draw_task == "draw_collision_mesh_wireframe"
    ):
        obj_attributes_mgr = sim.get_object_template_manager()
        obj_attributes = sim.curr_obj.creation_attributes
        if sim.draw_task == "draw_collision_mesh":
            # replace render asset with collision asset if drawing collision asset
            obj_attributes.render_asset_handle = sim.collision_asset_handle
            obj_attributes.force_flat_shading = True
        else:
            # make asset invisible if just drawing collision asset *mesh*
            obj_attributes.is_visibile = False
        obj_attributes_mgr.register_template(obj_attributes, obj_attributes.handle)
        rigid_obj_mgr = sim.get_rigid_object_manager()
        rigid_obj_mgr.remove_all_objects()
        sim.curr_obj = rigid_obj_mgr.add_object_by_template_handle(
            obj_attributes.handle
        )

    # init object transforms
    sim.curr_obj.translation = sim.default_obj_pos
    sim.curr_obj.rotation = last_rotation

    # record object rotating about its y-axis, then its x-axis
    observations = []
    for axis in list(RotationAxis):
        curr_angle = 0.0
        while curr_angle < revolution_angle:
            # determine if this iteration will push the object past
            # "revolution_angle" degrees (which is specified in the
            # config), meaning we are done. If so, cut off the rotation
            # so that it doesn't exceed said angle
            if curr_angle + angle_delta >= revolution_angle:
                angle_delta = revolution_angle - curr_angle

            # rotate about object's local x or y axis
            rot_rad = mn.Rad(mn.Deg(angle_delta))
            sim.curr_obj.rotate_local(rot_rad, mn.Vector3(axis))

            # save this observation in a buffer of frames
            obs = sim.get_sensor_observations()
            observations.append(obs)

            # update current rotation angle
            curr_angle += angle_delta

    last_rotation = sim.curr_obj.rotation

    # restore render asset and visibility if done drawing collision asset/asset mesh
    if (
        sim.draw_task == "draw_collision_mesh"
        or sim.draw_task == "draw_collision_mesh_wireframe"
    ):
        obj_attributes_mgr = sim.get_object_template_manager()
        obj_attributes = sim.curr_obj.creation_attributes
        obj_attributes.render_asset_handle = sim.render_asset_handle
        obj_attributes.is_visibile = True
        obj_attributes.force_flat_shading = False
        obj_attributes_mgr.register_template(obj_attributes, obj_attributes.handle)
        rigid_obj_mgr = sim.get_rigid_object_manager()
        rigid_obj_mgr.remove_all_objects()
        sim.curr_obj = rigid_obj_mgr.add_object_by_template_handle(
            obj_attributes.handle
        )
        sim.curr_obj.rotation = last_rotation

    return observations


def record_asset_display_video(sim: DatasetProcessorSim) -> None:
    """
    Loop over each recording task specified in the config file, i.e. show bounding box,
    show collision asset, show collision mesh, and/or show collision mesh wireframe, then,
    using the transforms of the agent and sensor, record the observations of the revolving
    ManagedBulletRigidObject, which is in Kinematic mode and just being displayed, not
    simulated. The object currently revolves "revolution_angle" degrees once around its x-axis,
    then once around its y-axis for each task. Then save the cumulation of observations as
    a video file.

    :param sim: The Simulator instance.
    """
    # init video and rotation parameters
    video_vars: Dict[str, Any] = sim.sim_settings["video_vars"]
    tasks: Dict[str, bool] = video_vars.get("tasks")
    revolution_angle = video_vars.get("revolution_angle")
    deg_per_sec = revolution_angle / sim.sim_settings["video_vars"].get(
        "revolution_dur"
    )
    dt = 1.0 / sim.sim_settings["physics_vars"].get("fps")
    angle_delta = deg_per_sec * dt

    # init agent transform and object movement type
    sim.default_agent.body.object.translation = sim.default_agent_pos
    sim.default_agent.body.object.rotation = sim.default_agent_rot
    sim.curr_obj.motion_type = physics.MotionType.KINEMATIC

    # Loop over each recording task specified in the config file
    for task, required in tasks.items():
        if not required or task == "record_physics":
            continue
        # save which video recording task we are performing
        print_if_logging(sim, ANSICodes.YELLOW.value + task + "\n")
        sim.draw_task = task
        # record object rotating about its x-axis, then its y-axis
        sim.observations += record_revolving_obj(sim, revolution_angle, angle_delta)


def run_physics_simulations(
    sim: DatasetProcessorSim,
    max_wait_time: float,
    recording: bool,
) -> Tuple[List, List, List, List]:
    """
    Drop current stationary rigid object from six different orthogonal initial rotations
    and wait for it to fall asleep (become idle), or wait for it to time out. If
    recording is set to True, also save the frames to be stored in a video file.
    :param sim: The Simulator instance.
    :param max_wait_time: the max time in seconds we will wait for object to fall asleep
    :param recording: whether or not we will save these simulation frames to a video file
    """
    # we must test object in 6 different orientations, each corresponding to a face
    # of an imaginary cube bounding the object. Each rotation in rotations is of the form:
    # (angle, (axis.x, axis.y, axis.z)) where angle is in degrees
    rotations: List[Tuple] = [
        (mn.Rad(mn.Deg(rotation[0])), mn.Vector3(rotation[1]))
        for rotation in sim.sim_settings["sim_test_rotations"]
    ]
    wait_times: List[float] = [0.0] * len(rotations)
    translation_deltas: List[float] = [0.0] * len(rotations)
    rotation_deltas: List[mn.Quaternion] = [mn.Quaternion.identity_init()] * len(
        rotations
    )
    sim_time_ratios: List[float] = [0.0] * len(rotations)

    # Loop over the 6 rotations and simulate snapping the object down and waiting for
    # it to become idle (or until "max_wait_time" from the config file expires)
    dt = 1.0 / sim.sim_settings["physics_vars"].get("fps")  # seconds
    for i in range(len(rotations)):

        # Reset object state with new rotation using angle-axis to construct
        # a quaternion. If axis is (0, 0, 0), that means there is no rotation
        sim.curr_obj.motion_type = physics.MotionType.DYNAMIC
        sim.curr_obj.awake = True
        sim.curr_obj.translation = sim.default_obj_pos
        angle = rotations[i][0]
        axis = rotations[i][1]
        if angle == 0.0 or axis.is_zero():
            sim.curr_obj.rotation = mn.Quaternion.identity_init()
        else:
            sim.curr_obj.rotation = mn.Quaternion.rotation(angle, axis)

        # Record a few frames of the object stationary before snapping down
        if recording:
            t = 0
            pre_drop_record_duration = 0.5
            while t < pre_drop_record_duration:
                sim.observations.append(sim.get_sensor_observations())
                t += dt

        # snap rigid object to surface below
        success = snap_down_object(sim, sim.curr_obj)
        if not success:
            logger.warning(
                ANSICodes.BRIGHT_RED.value
                + "dataset_object_processor.process_asset_physics(...) - snapping failed"
            )
        snap_pos: mn.Vector3 = sim.curr_obj.translation
        snap_rot: mn.Quaternion = sim.curr_obj.rotation

        # simulate until rigid object becomes idle or time limit from config file runs out
        sim_steps: int = 0
        while sim.curr_obj.awake and wait_times[i] < max_wait_time:
            sim_steps += 1
            start_sim_time = time.time()
            sim.step_physics(dt)
            end_sim_time = time.time()
            sim_time_ratios[i] += (end_sim_time - start_sim_time) / dt
            wait_times[i] += dt
            # Save observations of object as it falls asleep or times out
            if recording:
                obs = sim.get_sensor_observations()
                sim.observations.append(obs)

        # store final information once object becomes idle or it times out
        decimal_place = 3
        wait_times[i] = round(wait_times[i], decimal_place)
        translation_deltas[i] = (sim.curr_obj.translation - snap_pos).length()
        rotation_deltas[i] = sim.curr_obj.rotation * snap_rot.conjugated()
        sim_time_ratios[i] /= sim_steps

    return (wait_times, translation_deltas, rotation_deltas, sim_time_ratios)


def process_asset_physics(sim: DatasetProcessorSim) -> List[str]:
    """
    Run series of tests on asset to see how it responds in physics simulations.
    We snap an object down onto the surface below, then see how long it takes
    for the object to stabilize and become idle.
    :param sim: The Simulator instance.
    """
    # Print object that we will be imminently simulating
    text_format = ANSICodes.GREEN.value
    print_if_logging(
        sim,
        text_format
        + f"\nstart simulating {basename(sim.curr_obj.creation_attributes.handle)}"
        + section_divider,
    )

    # determine if we are recording a video, recording physics, and if so, set up
    # agent recording transforms
    record_video = sim.sim_settings["outputs"].get("video")
    record_physics = sim.sim_settings["video_vars"].get("tasks").get("record_physics")
    recording = record_video and record_physics
    if recording:
        sim.draw_task = "record_physics"
        print_if_logging(sim, ANSICodes.YELLOW.value + sim.draw_task + "\n")
        recording_pos = sim.sim_settings["video_vars"].get("physics_recording_pos")
        sim.default_agent.body.object.translation = mn.Vector3(recording_pos)
        recording_rot = sim.sim_settings["video_vars"].get("physics_recording_rot")
        angle = recording_rot.get("angle")
        axis = mn.Vector3(recording_rot.get("axis"))
        sim.default_agent.body.object.rotation = mn.Quaternion.rotation(
            mn.Rad(mn.Deg(angle)), axis
        )

    # Test object in 6 different orientations, each corresponding to a face
    # of an imaginary cube bounding the object.
    max_wait_time = sim.sim_settings["physics_vars"].get("max_wait_time")  # seconds
    (
        wait_times,
        translation_deltas,
        rotation_deltas,
        sim_time_ratios,
    ) = run_physics_simulations(sim, max_wait_time, recording)

    # convert each result to lists of strings for csv file
    time_units = sim.sim_settings["physics_vars"].get("time_units")
    times_as_strs = [
        "****timed out****" if t >= max_wait_time else f"{t} {time_units}"
        for t in wait_times
    ]
    decimal_place = 3
    distance_units = sim.sim_settings["physics_vars"].get("distance_units")
    translation_deltas_strs = [
        f"{round(t, decimal_place)} {distance_units}" for t in translation_deltas
    ]
    decimal_place = 1
    angle_units = sim.sim_settings["physics_vars"].get("angle_units")
    rotation_deltas_strs = [
        f"angle:  {round(float(mn.Deg(r.angle())), decimal_place)} {angle_units}\n"
        + f"axis:  ({round(r.axis().x, decimal_place)}, {round(r.axis().y, decimal_place)}, {round(r.axis().z, decimal_place)})"
        for r in rotation_deltas
    ]
    decimal_place = 7
    sim_time_strs = [f"{round(ratio, decimal_place)}" for ratio in sim_time_ratios]

    # return concatenated list of physics results formatted for csv file row
    return (
        [f"({round(max_wait_time, 3)} {time_units} max)"]
        + times_as_strs
        + [""]
        + translation_deltas_strs
        + [""]
        + rotation_deltas_strs
        + [""]
        + sim_time_strs
    )


def process_asset_render_time() -> List[str]:
    return ["..."]


def process_asset_mem_usage(
    sim: DatasetProcessorSim,
    importer: trade.AbstractImporter,
) -> List[str]:
    """
    Use the trade.AbstractImporter class to query data size of mesh and image
    of asset
    :param importer: AbstractImporter to open files and get mem size info
    :param template: ObjectAttributes to get render asset and collision asset
    """
    # construct absolute file paths for render and collision assets
    asset_paths: List[str] = [
        os.path.join(HABITAT_SIM_PATH, sim.render_asset_handle),
        os.path.join(HABITAT_SIM_PATH, sim.collision_asset_handle),
    ]

    # get the render and collision asset file names for CSV
    render_asset_filename = basename(sim.render_asset_handle)
    collision_asset_filename = basename(sim.collision_asset_handle)

    # Log render asset handle and collision asset handle
    text_format = Fore.GREEN
    print_if_logging(
        sim,
        text_format
        + f"-render: {sim.render_asset_handle}\n"
        + f"-collision: {sim.collision_asset_handle}\n",
    )

    # Create variables to store mesh data
    mesh_count = 0
    index_data_size = 0  # bytes
    vertex_data_size = 0  # bytes
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

    # return results as a list of strings formatted for csv rows
    mesh_count_units = "meshes" if mesh_count != 1 else "mesh"
    image_count_units = "images" if image_count != 1 else "image"
    return [
        render_asset_filename,
        collision_asset_filename,
        f"{mesh_count} {mesh_count_units}",
        get_mem_size_str(index_data_size),
        get_mem_size_str(vertex_data_size),
        get_mem_size_str(total_mesh_data_size),
        f"{image_count} {image_count_units}",
        get_mem_size_str(image_data_size),
        get_mem_size_str(total_mesh_data_size + image_data_size),
    ]


def process_asset(
    sim: DatasetProcessorSim,
    importer: trade.AbstractImporter,
    handle: str,
    obj_template_mgr: attributes_managers.ObjectAttributesManager,
    ram_multi_sample: bool,
    ram_use_multi_sample: float,
) -> List[str]:
    """
    Depending on what the user specifies in the dataset_processor_config.json file,
    use the trade.AbstractImporter class to query data size of mesh and image
    of asset and how the asset behaves during physics simulations. Likewise,
    make a recording of the object in Kinematic mode displaying any/all of its
    bounding box, collision asset, and bullet collision mesh.
    :param sim: Simulator instance
    :param importer: AbstractImporter, used to process memory stats of the mesh
    :param handle: template handle of the asset to test from ObjectAttributes
    :param obj_template_mgr: the ObjectAttributes of the asset to test
    """
    # Get memory state before and after loading object with RigidObjectManager,
    # then use psutil to get a sense of how much RAM was used during the
    # loading process
    rigid_obj_mgr = sim.get_rigid_object_manager()
    start_mem = psutil.virtual_memory()._asdict()
    sim.curr_obj = rigid_obj_mgr.add_object_by_template_handle(handle)
    end_mem = psutil.virtual_memory()._asdict()

    # Get average deltas of each memory metric specified in the config file.
    # The variable "subtract_order" below is either -1 or 1. 1 means the delta is
    # calculated as (end - start), whereas -1 means (start - end). e.g. Data "used"
    # should be higher after loading, so subtract_order == 1, but data "free"
    # should be higher before loading, so subtract_order == -1
    metrics: List[str] = sim.sim_settings["ram_metrics_to_use"]
    avg_ram_delta = 0  # bytes
    for metric in metrics:
        subtract_order = sim.sim_settings["mem_delta_order"].get(metric)
        # "percent" isn't calculated in bytes, so skip it
        if metric != "percent":
            avg_ram_delta += (
                end_mem.get(metric) - start_mem.get(metric)
            ) * subtract_order

    avg_ram_delta /= len(metrics)

    # ------------------------------------------------------------------------
    # TODO: remove this when I figure out why ram usage is sometimes negative
    global negative_ram_count
    ram_usage = ram_use_multi_sample if ram_multi_sample else avg_ram_delta
    if ram_usage < 0:
        negative_ram_count += 1
    # ------------------------------------------------------------------------

    # Print memory usage info before and after loading object (single sample)
    ram_use_single_sample = get_mem_size_str(avg_ram_delta)
    print_mem_usage_info(sim, start_mem, end_mem, ram_use_single_sample)

    # Log object template handle if "silent" is false in the config file
    text_format = ANSICodes.GREEN.value
    print_if_logging(
        sim,
        text_format + "Handles" + section_divider + "\n" + f"-template: {handle}\n",
    )

    # Get object attributes by template handle and associated render and collision
    # mesh assets
    template = obj_template_mgr.get_template_by_handle(handle)
    sim.render_asset_handle = template.render_asset_handle
    sim.collision_asset_handle = template.collision_asset_handle

    # record video of object's bounding box, its collision asset, its collision
    # asset wireframe, or none of these if denoted in the dataset_processor_config.json
    # file
    sim.observations = []
    if sim.record_video:
        record_asset_display_video(sim)

    # collect the data we need to write a csv (excluding physics data, which we
    # need to collect separately in case we are recording it too)
    csv_row: List[str] = []
    if sim.write_csv:
        csv_row += [basename(handle)]
        if sim.csv_data_to_collect.get("memory_data"):
            ram_str = (
                get_mem_size_str(ram_use_multi_sample)
                if ram_multi_sample
                else ram_use_single_sample
            )
            data = ["", ram_str] + process_asset_mem_usage(sim, importer)
            csv_row += data
        if sim.csv_data_to_collect.get("render_time_ratio"):
            data = [""] + process_asset_render_time()
            csv_row += data

    # If we are collecting physics data to write to a csv or to record in a video
    # file, we will call "process_asset_physics()"
    if sim.collect_physics_data or sim.record_physics:
        data = [""] + process_asset_physics(sim)
        if sim.write_csv and sim.collect_physics_data:
            csv_row += data

    if sim.record_video:
        video_file_dir = os.path.join(
            HABITAT_SIM_PATH, sim.sim_settings["output_paths"].get("video")
        )
        create_video(sim, video_file_dir)

    # remove this object so the next one can be added without interacting
    # with the previous one
    rigid_obj_mgr.remove_all_objects()

    # TODO: script crashes when making many high res videos. Seeing if RAM is the issue
    gc.collect()

    # Print that we are done processing this asset
    text_format = ANSICodes.BRIGHT_CYAN.value
    print_if_logging(
        sim,
        text_format + f"\ndone processing asset: {basename(handle)}" + section_divider,
    )

    # return results as a list of strings formatted for csv rows
    return csv_row


def process_dataset(
    sim: DatasetProcessorSim,
) -> None:
    """
    Load and process dataset objects using template handles
    :param sim: Simulator instance
    """
    dataset_path = sim.sim_settings["scene_dataset_config_file"]
    metadata_mediator = sim.metadata_mediator
    if (
        metadata_mediator is None
        or metadata_mediator.dataset_exists(dataset_path) is False
    ):
        raise RuntimeError(
            "process_dataset(...): No meta data mediator or dataset exists."
        )

    # get dataset that is currently being used by the simulator
    active_dataset: str = metadata_mediator.active_dataset

    text_format = ANSICodes.BRIGHT_BLUE.value + ANSICodes.BOLD.value
    print_if_logging(sim, text_format + "\nActive Dataset" + section_divider)
    text_format = ANSICodes.BRIGHT_BLUE.value
    print_if_logging(sim, text_format + f"{active_dataset}\n")

    # Get ObjectAttributesManager for objects from dataset, load the dataset,
    # and store all the object template handles in a List
    obj_template_mgr = sim.get_object_template_manager()
    obj_template_mgr.load_configs(dataset_path)
    sim.obj_template_handles = obj_template_mgr.get_file_template_handles("")

    # loop over the asset indices specified in the config file, starting with index
    # "start_obj_index" and proceeding for "num_objects" iterations of the loop.
    # If "start_obj_index" and/or "num_objects" is not specified in the config file,
    # or is not an int, process every asset in the dataset. If the specified range
    # exceeds the end of the dataset, make "end_index" the index of the last asset.
    start_index = sim.sim_settings["start_obj_index"]
    num_objs = sim.sim_settings["num_objects"]
    if not isinstance(start_index, int) or not isinstance(num_objs, int):
        start_index = 0
        num_objs = len(sim.obj_template_handles)
    end_index = start_index + num_objs
    if end_index > len(sim.obj_template_handles):
        end_index = len(sim.obj_template_handles)

    # create variables to store if we are writing to a csv and which data we are
    # collecting, and/or if we are recording a video and which tasks we are recording
    sim.write_csv = sim.sim_settings["outputs"].get("csv")
    sim.record_video = sim.sim_settings["outputs"].get("video")
    sim.record_physics = (
        sim.sim_settings["video_vars"].get("tasks").get("record_physics")
    )
    sim.csv_data_to_collect = sim.sim_settings["csv_data_to_collect"]
    sim.collect_physics_data = sim.csv_data_to_collect.get("physics_data")

    # calculate average RAM used over "num_samples" if config requests it
    num_samples: int = 1
    ram_usages: List[float] = [0.0] * (end_index - start_index)
    ram_multiple_samples: bool = sim.sim_settings["csv_data_to_collect"].get(
        "ram_calc_multiple_samples"
    )
    if sim.write_csv and ram_multiple_samples:
        num_samples = sim.sim_settings["memory_vars"].get("ram_calc_num_samples")
        (sim, ram_usages) = get_multi_sample_ram_use(
            sim, start_index, end_index, num_samples
        )

    # Init vars to store the list of rows formatted for a csv file, as well as a
    # trade.AbstractImporter to assess the memory usage of the assets
    csv_rows: List[List[str]] = []
    importer = trade.ImporterManager().load_and_instantiate("AnySceneImporter")

    # TODO: maybe remove, not sure if threading is needed
    sim.video_thread = None
    if sim.record_video:
        # TODO: maybe remove, as this is done conditionally in viz_utils.py
        os.environ["IMAGEIO_FFMPEG_EXE"] = "/usr/bin/ffmpeg"

    # loop over each asset we are considering in the dataset and process it,
    # i.e. study its memory usage and run it through some physics simulations,
    # if the config file requests them.
    for i in range(start_index, end_index):
        handle = sim.obj_template_handles[i]
        row = process_asset(
            sim,
            importer,
            handle,
            obj_template_mgr,
            ram_multiple_samples,
            ram_usages[i - start_index],
        )
        csv_rows.append(row)

    # Write csv if specified in the config file. "headers" stores the titles of
    # each csv column.
    if sim.write_csv:
        headers = get_csv_headers(sim)
        if sim.sim_settings["csv_data_to_collect"].get("memory_data"):
            headers[2] = f"RAM usage\n{num_samples} sample(s)"
        csv_dir_path = os.path.join(
            HABITAT_SIM_PATH, sim.sim_settings["output_paths"].get("csv")
        )
        csv_file_prefix = sim.sim_settings["output_paths"].get("output_file_prefix")
        create_csv_file(sim, headers, csv_rows, csv_dir_path, csv_file_prefix)

    # clean up
    importer.close()

    # return list of rows to be written to csv file
    return csv_rows


def build_parser(
    parser: Optional[argparse.ArgumentParser] = None,
) -> argparse.ArgumentParser:
    """
    Parse config file argument or set default when running script for scene and
    dataset
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
        default="tools/dataset_object_processor/configs/default_1080p.dataset_processor_config.json",
        type=str,
        help="config file to load"
        ' (default: "tools/dataset_object_processor/configs/default_1080p.dataset_processor_config.json")',
    )
    return parser


def main() -> None:
    """
    Create Simulator, parse dataset (which also records a video if requested),
    then writes a csv file if requested
    """
    # parse arguments from command line: scene, dataset, if we process physics
    args = build_parser().parse_args()

    # setup colorama terminal color printing so that format and color reset
    # after each print_if_logging() statement
    init(autoreset=True)

    # Populate sim_settings with info from dataset_processor_config.json file
    sim_settings: Dict[str, Any] = default_sim_settings
    with open(os.path.join(HABITAT_SIM_PATH, args.config_file_path)) as config_json:
        update_sim_settings(sim_settings, json.load(config_json))

    # Configure and make simulator
    sim = configure_sim(sim_settings)

    # Print sim settings
    text_format = ANSICodes.PURPLE.value
    for key, value in sim_settings.items():
        print_if_logging(sim, text_format + f"{key} : {value}\n")

    # TODO: remove this when I figure out why ram usage is sometimes negative
    global negative_ram_count
    negative_ram_count = 0

    # Iterate through dataset objects and write csv and/or make video recording
    # if specified in the config file.
    process_dataset(sim)

    # ------------------------------------------------------------------------
    # TODO: remove this when I figure out why ram usage is sometimes negative
    text_format = ANSICodes.BRIGHT_MAGENTA.value
    print_debug(
        sim,
        text_format
        + f"negative RAM usage count: {negative_ram_count}"
        + section_divider,
    )
    # ------------------------------------------------------------------------

    # close simulator object
    sim.close(destroy=True)


if __name__ == "__main__":
    main()
