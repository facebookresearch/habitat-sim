import argparse
import json
import os
import time
from ntpath import basename
from typing import Any, Dict, List, Optional, Tuple

import git
import magnum as mn
import processor_utils as pcsu
import psutil
from colorama import Fore, init
from magnum import trade
from processor_settings import default_sim_settings, make_cfg
from processor_utils import ANSICodes, CSVWriter, MemoryUnitConverter, RotationAxis

import habitat_sim as hsim
from habitat_sim import attributes, attributes_managers, physics
from habitat_sim.logging import logger
from habitat_sim.utils import viz_utils as vut

repo = git.Repo(".", search_parent_directories=True)
HABITAT_SIM_PATH = repo.working_tree_dir


def print_mem_usage_info(
    start_mem: Dict[str, int],
    end_mem: Dict[str, int],
    avg_ram_used_str: str,
) -> None:
    """"""
    # Print memory usage info before loading object
    text_format = ANSICodes.BRIGHT_RED.value
    pcsu.print_if_logging(text_format + "\nstart mem state" + pcsu.section_divider)
    for key, value in start_mem.items():
        value_str = value
        if key != "percent":
            value_str = get_mem_size_str(value)
        pcsu.print_if_logging(text_format + f"{key} : {value_str}")

    # Print memory usage info after loading object
    pcsu.print_if_logging(text_format + "\nend mem state" + pcsu.section_divider)
    for key, value in end_mem.items():
        value_str = value
        if key != "percent":
            value_str = get_mem_size_str(value)
        pcsu.print_if_logging(text_format + f"{key} : {value_str}")

    # Print difference in memory usage before and after loading object
    pcsu.print_if_logging(text_format + "\nchange in mem states" + pcsu.section_divider)
    for (key_s, value_s), (key_e, value_e) in zip(start_mem.items(), end_mem.items()):
        value_str = value_e - value_s
        if key_s != "percent" and key_e != "percent":
            value_str = get_mem_size_str(value_e - value_s)
        pcsu.print_if_logging(text_format + f"{key_s} : {value_str}")

    # Print rough estimate of RAM used when loading object
    pcsu.print_if_logging(
        text_format
        + "\naverage RAM used"
        + pcsu.section_divider
        + f"\n{avg_ram_used_str}"
    )


def convert_memory_units(
    size: float,
    unit_type: int = MemoryUnitConverter.KILOBYTES,
    decimal_num_round: int = 2,
) -> float:
    """
    Convert units from bytes to desired unit, then round the result
    """
    new_size = size / MemoryUnitConverter.UNIT_CONVERSIONS[unit_type]
    return round(new_size, decimal_num_round)


def get_mem_size_str(
    size: float,
    unit_type: int = MemoryUnitConverter.KILOBYTES,
) -> str:
    """
    Convert bytes to desired memory unit size, then create string
    that will be written into csv file rows. Add commas to numbers
    """
    new_size: float = convert_memory_units(size, unit_type)
    new_size_str: str = "{:,}".format(new_size)
    unit_str: str = MemoryUnitConverter.UNIT_STRS[unit_type]
    return f"{new_size_str} {unit_str}"


def get_bounding_box_corners(
    obj: physics.ManagedBulletRigidObject,
) -> List[mn.Vector3]:
    """
    Return a list of object bounding box corners in object local space.
    """
    bounding_box = obj.root_scene_node.cumulative_bb
    return [
        bounding_box.back_bottom_left,
        bounding_box.back_bottom_right,
        bounding_box.back_top_right,
        bounding_box.back_top_left,
        bounding_box.front_top_left,
        bounding_box.front_top_right,
        bounding_box.front_bottom_right,
        bounding_box.front_bottom_left,
    ]


def bounding_box_ray_prescreen(
    sim: hsim.simulator,
    obj: physics.ManagedBulletRigidObject,
    support_obj_ids: Optional[List[int]] = None,
    check_all_corners: bool = False,
) -> Dict[str, Any]:
    """
    Pre-screen a potential placement by casting rays in the gravity direction from the object center of mass (and optionally each corner of its bounding box) checking for interferring objects below.
    :param sim: The Simulator instance.
    :param obj: The RigidObject instance.
    :param support_obj_ids: A list of object ids designated as valid support surfaces for object placement. Contact with other objects is a criteria for placement rejection.
    :param check_all_corners: Optionally cast rays from all bounding box corners instead of only casting a ray from the center of mass.
    """
    if support_obj_ids is None:
        # set default support surface to stage/ground mesh
        support_obj_ids = [-1]
    lowest_key_point: mn.Vector3 = None
    lowest_key_point_height = None
    highest_support_impact: mn.Vector3 = None
    highest_support_impact_height = None
    highest_support_impact_with_stage = False
    raycast_results = []
    gravity_dir = sim.get_gravity().normalized()
    object_local_to_global = obj.transformation
    bounding_box_corners = get_bounding_box_corners(obj)
    key_points = [mn.Vector3(0)] + bounding_box_corners  # [COM, c0, c1 ...]
    support_impacts: Dict[int, mn.Vector3] = {}  # indexed by keypoints
    for ix, key_point in enumerate(key_points):
        world_point = object_local_to_global.transform_point(key_point)
        # NOTE: instead of explicit Y coordinate, we project onto any gravity vector
        world_point_height = world_point.projected_onto_normalized(
            -gravity_dir
        ).length()
        if lowest_key_point is None or lowest_key_point_height > world_point_height:
            lowest_key_point = world_point
            lowest_key_point_height = world_point_height
        # cast a ray in gravity direction
        if ix == 0 or check_all_corners:
            ray = hsim.geo.Ray(world_point, gravity_dir)
            raycast_results.append(sim.cast_ray(ray))
            # classify any obstructions before hitting the support surface
            for hit in raycast_results[-1].hits:
                if hit.object_id == obj.object_id:
                    continue
                elif hit.object_id in support_obj_ids:
                    hit_point = ray.origin + ray.direction * hit.ray_distance
                    support_impacts[ix] = hit_point
                    support_impact_height = mn.math.dot(hit_point, -gravity_dir)

                    if (
                        highest_support_impact is None
                        or highest_support_impact_height < support_impact_height
                    ):
                        highest_support_impact = hit_point
                        highest_support_impact_height = support_impact_height
                        highest_support_impact_with_stage = hit.object_id == -1

                # terminates at the first non-self ray hit
                break
    # compute the relative base height of the object from its lowest bounding_box corner and COM
    base_rel_height = (
        lowest_key_point_height
        - obj.translation.projected_onto_normalized(-gravity_dir).length()
    )

    # account for the affects of stage mesh margin
    margin_offset = (
        0
        if not highest_support_impact_with_stage
        else sim.get_stage_initialization_template().margin
    )

    surface_snap_point = (
        None
        if 0 not in support_impacts
        else support_impacts[0] + gravity_dir * (base_rel_height - margin_offset)
    )
    # return list of obstructed and grounded rays, relative base height,
    # distance to first surface impact, and ray results details
    return {
        "base_rel_height": base_rel_height,
        "surface_snap_point": surface_snap_point,
        "raycast_results": raycast_results,
    }


def snap_down_object(
    sim: hsim.simulator,
    obj: physics.ManagedBulletRigidObject,
    support_obj_ids: Optional[List[int]] = None,
) -> bool:
    """
    Attempt to project an object in the gravity direction onto the surface below it.
    :param sim: The Simulator instance.
    :param obj: The RigidObject instance.
    :param support_obj_ids: A list of object ids designated as valid support surfaces for object placement. Contact with other objects is a criteria for placement rejection. If none provided, default support surface is the stage/ground mesh (-1).
    :param vdb: Optionally provide a DebugVisualizer (vdb) to render debug images of each object's computed snap position before collision culling.
    Reject invalid placements by checking for penetration with other existing objects.
    Returns boolean success.
    If placement is successful, the object state is updated to the snapped location.
    If placement is rejected, object position is not modified and False is returned.
    To use this utility, generate an initial placement for any object above any of the designated support surfaces and call this function to attempt to snap it onto the nearest surface in the gravity direction.
    """
    cached_position = obj.translation

    if support_obj_ids is None:
        # set default support surface to stage/ground mesh
        support_obj_ids = [-1]

    bounding_box_ray_prescreen_results = bounding_box_ray_prescreen(
        sim, obj, support_obj_ids
    )

    if bounding_box_ray_prescreen_results["surface_snap_point"] is None:
        # no support under this object, return failure
        return False

    # finish up
    if bounding_box_ray_prescreen_results["surface_snap_point"] is not None:
        # accept the final location if a valid location exists
        obj.translation = bounding_box_ray_prescreen_results["surface_snap_point"]
        sim.perform_discrete_collision_detection()
        cps = sim.get_physics_contact_points()
        for cp in cps:
            if (
                cp.object_id_a == obj.object_id or cp.object_id_b == obj.object_id
            ) and (
                (cp.contact_distance < -0.01)
                or not (
                    cp.object_id_a in support_obj_ids
                    or cp.object_id_b in support_obj_ids
                )
            ):
                obj.translation = cached_position
                return False
        return True
    else:
        # no valid position found, reset and return failure
        obj.translation = cached_position
        return False


def record_revolving_obj(
    sim: hsim.simulator,
    rigid_obj: physics.ManagedBulletRigidObject,
    angle_delta: float,
    axis,
) -> Dict[int, Any]:
    """ """
    curr_angle = 0.0
    finished = False
    observations = []
    while curr_angle < 360.0:
        # rotate_recorded_obj(rigid_obj, curr_angle, angle_delta, axis)
        if curr_angle + angle_delta >= 360.0:
            finished = True
            angle_delta = 360.0 - curr_angle
            curr_angle = 360.0

        # rotate about object's local y axis (up vector)
        if axis == RotationAxis.Y:
            y_rot_rad = mn.Rad(mn.Deg(angle_delta))
            rigid_obj.rotate_y_local(y_rot_rad)

        # rotate about object's local x axis (horizontal vector)
        else:
            x_rot_rad = mn.Rad(mn.Deg(angle_delta))
            rigid_obj.rotate_x_local(x_rot_rad)

        # save this observation in a buffer of frames
        observations.append(sim.get_sensor_observations())

        # update current rotation angle
        curr_angle += angle_delta

        # reset current rotation angle if it passed 360 degrees
        if finished:
            curr_angle = 360.0

    return observations


def record_video(
    sim: hsim.simulator,
    rigid_obj: physics.ManagedBulletRigidObject,
    sim_settings: Dict[str, Any],
) -> None:
    """
    Loop over each recording task specified in the config file, i.e. show bounding box,
    show collision asset, and show bullet collision mesh, then using the transforms
    of the agent and sensor, record the observations of the revolving ManagedBulletRigidObject,
    which is in Kinematic mode and just being displayed, not simulated. The object
    currently revolves 360 degrees once around its x-axis, then once around its y-axis for
    each task. Then save the cumulation of observations as a video file.
    :param sim: The Simulator instance.
    :param rigid_obj: The ManagedBulletRigidObject to record
    :param sim_settings: simulator settings, defines tasks, rotation speed, and time step
    """
    # init object
    default_transforms = sim_settings["default_transforms"]
    obj_rot = default_transforms.get("default_obj_rot")
    rigid_obj.translation = default_transforms.get("default_obj_pos")
    angle = mn.Rad(mn.Deg(obj_rot.get("angle")))
    axis = mn.Vector3(obj_rot.get("axis"))
    rigid_obj.rotation = mn.Quaternion.rotation(angle, axis)
    rigid_obj.motion_type = physics.MotionType.KINEMATIC

    # init video and rotation parameters
    video_vars: Dict[str, Any] = sim_settings["video_vars"]
    tasks: Dict[str, bool] = video_vars.get("tasks")
    deg_per_sec = 360.0 / sim_settings["video_vars"].get("revolution_dur")
    dt = 1.0 / sim_settings["physics_vars"].get("fps")
    angle_delta = deg_per_sec * dt

    # Loop over each recording task specified in the config file
    observations = []
    for task, required in tasks.items():
        if required == False:
            continue
        if task == "show_bbox":
            pcsu.print_if_logging("draw bbox")
            # rgb = sim_settings["bbox_rgb"]
            # line_color = mn.Color4.from_xyz(rgb)
            # bb_corners: List[mn.Vector3] = get_bounding_box_corners(rigid_obj)
            # num_corners = len(bb_corners)
            # sim.get_debug_line_render().set_line_width(0.01)
            # obj_transform = rigid_obj.transformation
        elif task == "show_collision_asset":
            pcsu.print_if_logging("draw collision asset")
        elif task == "show_bullet_collision_mesh":
            pcsu.print_if_logging("draw bullet collision mesh")

        # record object rotating about its x-axis, then its y-axis
        for axis in list(RotationAxis):
            observations += record_revolving_obj(sim, rigid_obj, angle_delta, axis)

    # construct file path and write "observations" to video file
    video_file_dir = os.path.join(
        HABITAT_SIM_PATH, sim_settings["output_paths"].get("video")
    )
    obj_handle = rigid_obj.handle.replace("_:0000", "")
    video_file_prefix = sim_settings["output_paths"].get("output_file_prefix")
    video_file_prefix += f"_{obj_handle}"
    file_path = pcsu.create_unique_filename(video_file_dir, ".mp4", video_file_prefix)
    vut.make_video(
        observations,
        "color_sensor",
        "color",
        file_path,
        open_vid=False,
    )


def process_mem_usage(
    importer: trade.AbstractImporter,
    template: attributes.ObjectAttributes,
) -> List[str]:
    """
    Use the trade.AbstractImporter class to query data size of mesh and image
    of asset
    :param importer: AbstractImporter to open files and get mem size info
    :param template: ObjectAttributes to get render asset and collision asset
    """
    # construct absolute file paths for render and collision assets
    render_asset_handle = template.render_asset_handle
    collision_asset_handle = template.collision_asset_handle
    asset_paths: List[str] = [
        os.path.join(HABITAT_SIM_PATH, render_asset_handle),
        os.path.join(HABITAT_SIM_PATH, collision_asset_handle),
    ]

    # get the render and collision asset file names for CSV
    render_asset_filename = basename(render_asset_handle)
    collision_asset_filename = basename(collision_asset_handle)

    # Log render asset handle and collision asset handle
    text_format = Fore.GREEN
    pcsu.print_if_logging(
        text_format
        + f"-render: {render_asset_handle}\n"
        + f"-collision: {collision_asset_handle}\n"
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
    mesh_count_units = "mesh" if mesh_count == 1 else "meshes"
    image_count_units = "image" if image_count == 1 else "images"
    return [
        render_asset_filename,
        collision_asset_filename,
        f"{mesh_count} {mesh_count_units}",
        get_mem_size_str(index_data_size),
        get_mem_size_str(vertex_data_size),
        get_mem_size_str(total_mesh_data_size),
        f"{image_count} {image_count_units}",
        get_mem_size_str(image_data_size),
    ]


def process_render_time() -> List[str]:
    return ["..."]


def process_physics(
    sim: hsim.simulator,
    rigid_obj: physics.ManagedBulletRigidObject,
    sim_settings: Dict[str, Any],
) -> List[str]:
    """
    Run series of tests on asset to see how it responds in physics simulations.
    We snap an object down onto the surface below, then see how long it takes
    for the object to stabilize and become idle.
    :param sim: The Simulator instance.
    :param rigid_obj: The ManagedBulletRigidObject to test
    :param sim_settings: simulator settings, defines initial orientations to test
    object from, object initial position, fps, max amount of time we wait until
    object becomes idle, and our units for time and angle
    """
    # we must test object in 6 different orientations, each corresponding to a face
    # of an imaginary cube bounding the object. Each rotation in rotations is of the form:
    # (angle, (axis.x, axis.y, axis.z)) where angle is in degrees
    rotations: List[Tuple] = [
        (mn.Rad(mn.Deg(rotation[0])), mn.Vector3(rotation[1]))
        for rotation in sim_settings["sim_test_rotations"]
    ]
    wait_times: List[float] = [0.0] * len(rotations)
    translation_deltas: List[float] = [0.0] * len(rotations)
    rotation_deltas: List[mn.Quaternion] = [mn.Quaternion.identity_init()] * len(
        rotations
    )
    sim_time_ratios: List[float] = [0.0] * len(rotations)

    # TODO: debugging, remove
    obj_init_attributes = rigid_obj.creation_attributes
    text_format = ANSICodes.BRIGHT_RED.value
    pcsu.print_debug(
        text_format
        + f"\nstart simulating {basename(obj_init_attributes.handle)}"
        + pcsu.section_divider
    )

    # Loop over the 6 rotations and simulate snapping the object down and waiting for
    # it to become idle (at least until "max_wait_time" from the config file expires)
    default_pos = sim_settings["default_transforms"].get("default_obj_pos")
    dt = 1.0 / sim_settings["physics_vars"].get("fps")  # seconds
    max_wait_time = sim_settings["physics_vars"].get("max_wait_time")  # seconds
    for i in range(len(rotations)):

        # Reset object state with new rotation
        rigid_obj.motion_type = physics.MotionType.DYNAMIC
        rigid_obj.awake = True
        rigid_obj.translation = default_pos
        angle = mn.Deg(rotations[i][0])
        axis = rotations[i][1]
        rigid_obj.rotation = mn.Quaternion.rotation(angle, axis)

        # TODO: debugging, remove
        pcsu.print_debug(f"init as quaternion - {rigid_obj.rotation}")
        pcsu.print_quaternion_debug(
            "init - ", rigid_obj.rotation, ANSICodes.YELLOW.value
        )

        # snap rigid object to surface below
        success = snap_down_object(sim, rigid_obj)
        if not success:
            logger.warning(
                ANSICodes.BRIGHT_RED.value
                + "dataset_object_processor.process_physics(...) - snapping failed"
            )
        snap_pos: mn.Vector3 = rigid_obj.translation
        snap_rot: mn.Quaternion = rigid_obj.rotation

        # TODO: debugging, remove
        pcsu.print_quaternion_debug("snap - ", snap_rot, ANSICodes.BRIGHT_CYAN.value)

        # simulate until rigid object becomes idle or time limit from config file runs out
        sim_steps: int = 0
        while rigid_obj.awake and wait_times[i] < max_wait_time:
            sim_steps += 1
            start_sim_time = time.time()
            sim.step_physics(dt)
            end_sim_time = time.time()
            sim_time_ratios[i] += (end_sim_time - start_sim_time) / dt
            wait_times[i] += dt

        # TODO: debugging, remove
        sim_time_ms = round((sim_time_ratios[i] / sim_steps) * 1000, 5)
        pcsu.print_debug(ANSICodes.BOLD.value + f"sim time: {sim_time_ms} ms")
        pcsu.print_debug(ANSICodes.BOLD.value + f"sim steps: {sim_steps}")

        # TODO: debugging, remove
        pcsu.print_quaternion_debug(
            "idle - ", rigid_obj.rotation, ANSICodes.BRIGHT_MAGENTA.value
        )

        # store final information once object becomes idle or it times out
        wait_times[i] = round(wait_times[i], 3)
        translation_deltas[i] = (rigid_obj.translation - snap_pos).length()
        rotation_deltas[i] = rigid_obj.rotation * snap_rot.conjugated()
        sim_time_ratios[i] /= sim_steps

        # TODO: debugging, remove
        pcsu.print_quaternion_debug(
            "delta r - ", rotation_deltas[i], ANSICodes.BRIGHT_BLUE.value
        )
        pcsu.print_debug("")

    # convert results to lists of strings for csv file
    time_units = sim_settings["physics_vars"].get("time_units")
    times_as_strs = [
        "*** timed out ***" if t >= max_wait_time else f"{t} {time_units}"
        for t in wait_times
    ]
    decimal = 3
    translation_deltas_strs = [f"{round(t, decimal)} units" for t in translation_deltas]
    decimal = 1
    angle_units = sim_settings["physics_vars"].get("angle_units")
    rotation_deltas_strs = [
        f"angle:  {round(float(mn.Deg(r.angle())), decimal)} {angle_units}\n"
        + f"axis:  ({round(r.axis().x, decimal)}, {round(r.axis().y, decimal)}, {round(r.axis().z, decimal)})"
        for r in rotation_deltas
    ]
    decimal = 7
    sim_time_strs = [f"{round(ratio, decimal)}" for ratio in sim_time_ratios]

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


def process_asset(
    sim: hsim.simulator,
    importer: trade.AbstractImporter,
    handle: str,
    obj_template_mgr: attributes_managers.ObjectAttributesManager,
    sim_settings: Dict[str, Any],
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
    :param sim_settings: Simulator settings, defines which data to collect, if
    we are making a csv and/or a video recording, and also passed to the function
    to test the physics of the ManagedBulletRigidObject
    """
    # Get memory state before and after loading object with RigidObjectManager,
    # then use psutil to get a sense of how much RAM was used during the
    # loading process
    rigid_obj_mgr = sim.get_rigid_object_manager()
    rigid_obj_mgr.remove_all_objects()
    start_mem = psutil.virtual_memory()._asdict()
    rigid_obj = rigid_obj_mgr.add_object_by_template_handle(handle)
    end_mem = psutil.virtual_memory()._asdict()

    # Get average deltas of each memory metric specified in the config file.
    # The variable "order" below is either -1 or 1. 1 means the delta is
    # calculated as (end - start), whereas -1 means (start - end).
    # e.g. Data "used" should be higher after loading, so order == 1,
    # but data "free" should be higher before loading, so order == -1
    metrics: List[str] = sim_settings["mem_metrics_to_use"]
    avg_ram_delta = 0  # bytes
    for metric in metrics:
        order = sim_settings["mem_delta_order"].get(metric)
        # "percent" isn't calculated in bytes
        if metric != "percent":
            avg_ram_delta += (end_mem.get(metric) - start_mem.get(metric)) * order
    avg_ram_delta /= len(metrics)
    avg_ram_used_str = get_mem_size_str(avg_ram_delta)

    # TODO: remove this when I figure out why ram usage is sometimes negative
    global negative_ram_count
    if avg_ram_delta < 0:
        negative_ram_count += 1
        avg_ram_used_str = "***  " + avg_ram_used_str + "  ***"

    # Print memory usage info before and after loading object
    print_mem_usage_info(start_mem, end_mem, avg_ram_used_str)

    # Get object attributes by template handle and associated mesh assets
    template = obj_template_mgr.get_template_by_handle(handle)

    # Log object template handle
    text_format = Fore.GREEN
    pcsu.print_if_logging(
        text_format
        + "\n"
        + "Handles"
        + pcsu.section_divider
        + "\n"
        + f"-template: {handle}\n"
    )

    # run object through tests defined in dataset_processor_config.json file
    csv_row: List[str] = []
    if sim_settings["outputs"].get("csv"):
        data_to_collect = sim_settings["data_to_collect"]
        csv_row += [basename(handle)]
        if data_to_collect.get("memory_data"):
            data = ["", avg_ram_used_str] + process_mem_usage(importer, template)
            csv_row += data
        if data_to_collect.get("render_time_ratio"):
            data = [""] + process_render_time()
            csv_row += data
        if data_to_collect.get("physics_data"):
            data = [""] + process_physics(sim, rigid_obj, sim_settings)
            csv_row += data

    if sim_settings["outputs"].get("video"):
        record_video(sim, rigid_obj, sim_settings)

    # return results as a list of strings formatted for csv rows
    return csv_row


def parse_dataset(
    sim: hsim.simulator,
    sim_settings: Dict[str, Any],
) -> List[List[str]]:
    """
    Load and process dataset objects using template handles
    :param sim: Simulator instance
    :param sim_settings: Simulator settings, defines dataset to use,
    and is also passed to function to test each dataset object
    """
    dataset_path = sim_settings["scene_dataset_config_file"]
    metadata_mediator = sim.metadata_mediator
    if (
        metadata_mediator is None
        or metadata_mediator.dataset_exists(dataset_path) is False
    ):
        raise RuntimeError(
            "parse_dataset(...): No meta data mediator or dataset exists."
        )

    # get dataset that is currently being used by the simulator
    active_dataset: str = metadata_mediator.active_dataset
    text_format = ANSICodes.BRIGHT_BLUE.value + ANSICodes.BOLD.value
    pcsu.print_if_logging(text_format + "\nActive Dataset" + pcsu.section_divider)
    text_format = ANSICodes.BRIGHT_BLUE.value
    pcsu.print_if_logging(text_format + f"{active_dataset}\n")

    # Get ObjectAttributesManager for objects from dataset, load the dataset,
    # and store all the object template handles in a List
    obj_template_mgr = sim.get_object_template_manager()
    obj_template_mgr.load_configs(dataset_path)
    object_template_handles = obj_template_mgr.get_file_template_handles("")

    # Process each asset with trade.AbstractImporter and construct csv data
    csv_rows: List[List[str]] = []
    manager = trade.ImporterManager()
    importer = manager.load_and_instantiate("AnySceneImporter")

    for handle in object_template_handles:
        row = process_asset(sim, importer, handle, obj_template_mgr, sim_settings)
        csv_rows.append(row)

    # # TODO: debugging, remove
    # for i in range(3):
    #     handle = object_template_handles[i]
    #     row = process_asset(sim, importer, handle, obj_template_mgr, sim_settings)
    #     csv_rows.append(row)

    # clean up
    importer.close()

    # return list of rows to be written to csv file
    return csv_rows


def get_csv_headers(sim_settings) -> List[str]:
    """
    Collect the csv column titles we'll need given which tests we ran
    :param sim_settings: Simulator settings, defines which metrics
    we are collecting for each object
    """
    headers: List[str] = sim_settings["object_name"]
    data_to_collect = sim_settings["data_to_collect"]
    if data_to_collect.get("memory_data"):
        headers += sim_settings["memory_data_headers"]
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
    :param headers: column titles of csv file
    :param csv_rows: List of Lists of strings defining asset processing results
    for each dataset object
    :param csv_dir_path: absolute path to directory where csv file will be saved
    :param csv_file_prefix: prefix we will add to beginning of the csv filename
    to specify which dataset this csv is describing
    """
    file_path = pcsu.create_unique_filename(csv_dir_path, ".csv", csv_file_prefix)

    text_format = ANSICodes.PURPLE.value + ANSICodes.BOLD.value
    pcsu.print_if_logging(
        text_format + "Writing csv results to:" + pcsu.section_divider
    )
    text_format = ANSICodes.PURPLE.value
    pcsu.print_if_logging(text_format + f"{file_path}\n")

    CSVWriter.write_file(headers, csv_rows, file_path)

    text_format = ANSICodes.PURPLE.value
    pcsu.print_if_logging(text_format + "CSV writing done\n")

    # TODO: remove this when I figure out why ram usage is sometimes negative
    text_format = ANSICodes.BRIGHT_MAGENTA.value
    global negative_ram_count
    pcsu.print_debug(
        text_format
        + f"negative RAM usage count: {negative_ram_count}"
        + pcsu.section_divider
    )


def configure_sim(sim_settings: Dict[str, Any]):
    """
    Configure simulator while adding post configuration for the transform of
    the agent
    """
    cfg = make_cfg(sim_settings)
    sim = hsim.Simulator(cfg)
    default_transforms = sim_settings["default_transforms"]

    # init agent
    agent_state = hsim.AgentState()
    agent = sim.initialize_agent(sim_settings["default_agent"], agent_state)
    agent.body.object.translation = default_transforms.get("default_agent_pos")
    agent_rot = default_transforms.get("default_agent_rot")
    agent.body.object.rotation = mn.Quaternion.rotation(
        mn.Rad(mn.Deg(agent_rot.get("angle"))), mn.Vector3(agent_rot.get("axis"))
    )

    return sim


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
        default="tools/dataset_object_processor/configs/default.dataset_processor_config.json",
        type=str,
        help="config file to load"
        ' (default: "tools/dataset_object_processor/configs/default.dataset_processor_config.json")',
    )
    return parser


def main() -> None:
    """
    Create Simulator, parse dataset (which also records a video if requested),
    then writes a csv file if requested
    """
    # TODO: remove this when I figure out why ram usage is sometimes negative
    global negative_ram_count
    negative_ram_count = 0

    # parse arguments from command line: scene, dataset, if we process physics
    args = build_parser().parse_args()

    # Populate sim_settings with info from dataset_processor_config.json file
    sim_settings: Dict[str, Any] = default_sim_settings
    with open(os.path.join(HABITAT_SIM_PATH, args.config_file_path)) as config_json:
        sim_settings.update(json.load(config_json))

    # setup colorama terminal color printing so that format and color reset
    # after each pcsu.print_if_logging() statement
    init(autoreset=True)
    pcsu.silent = sim_settings["silent"]
    pcsu.debug_print = sim_settings["debug_print"]

    # Configure and make simulator
    sim = configure_sim(sim_settings)

    # Print sim settings
    text_format = ANSICodes.PURPLE.value
    for key, value in sim_settings.items():
        pcsu.print_if_logging(text_format + f"{key} : {value}\n")

    # Parse dataset and write CSV if specified in the config file.
    # "headers" stores the titles of each column
    csv_rows: List[List[str]] = parse_dataset(sim, sim_settings)
    if sim_settings["outputs"].get("csv"):
        headers = get_csv_headers(sim_settings)
        csv_dir_path = os.path.join(
            HABITAT_SIM_PATH, sim_settings["output_paths"].get("csv")
        )
        csv_file_prefix = sim_settings["output_paths"].get("output_file_prefix")
        create_csv_file(headers, csv_rows, csv_dir_path, csv_file_prefix)


if __name__ == "__main__":
    main()
