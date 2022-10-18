import argparse
import csv
import datetime
import json
import os
from enum import Enum
from ntpath import basename
from typing import Any, Dict, List, Optional, Tuple

import git
import magnum as mn
import psutil
from colorama import Fore, init
from magnum import trade
from processor_settings import default_sim_settings, make_cfg

import habitat_sim
from habitat_sim import attributes, attributes_managers, physics
from habitat_sim.logging import logger

repo = git.Repo(".", search_parent_directories=True)
HABITAT_SIM_PATH = repo.working_tree_dir

# A line of dashes to divide sections of terminal output
section_divider: str = "\n" + "-" * 72

# TODO: possibly move to separate utils file
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


# TODO: possibly move to separate utils file
class CSVWriter:
    """
    Generalized utility to write csv files
    """

    file_path = None

    def create_unique_filename(
        csv_dir_path: str = None, filename_prefix: str = None
    ) -> str:
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
        if csv_dir_path is None:
            raise RuntimeError(
                "CSVWriter.create_unique_filename: must provide a directory to save CSV file."
            )
        if not os.path.exists(csv_dir_path):
            os.makedirs(csv_dir_path)

        # create csv file name (TODO: make more descriptive file name)
        if filename_prefix is None or filename_prefix == "":
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
                raise RuntimeError(
                    "CSVWriter.write_file: must provide a file path to save CSV file."
                )
            else:
                file_path = CSVWriter.file_path

        # TODO: debugging, remove
        print_if_logging(ANSICodes.BRIGHT_CYAN.value + "HEADERS")
        for header in headers:
            print_if_logging(ANSICodes.BRIGHT_CYAN.value + header + "\n")
        print_if_logging(ANSICodes.BRIGHT_RED.value + "VALUES")
        for value in csv_rows[0]:
            print_if_logging(ANSICodes.BRIGHT_RED.value + value + "\n")
        print_if_logging(
            f" ===== num columns: {len(csv_rows[0])}, num headers: {len(headers)} ============="
        )

        # make sure the number of columns line up
        if not len(csv_rows[0]) == len(headers):
            raise RuntimeError(
                "Number of headers does not equal number of columns in CSVWriter.write_file()."
            )

        with open(file_path, "w") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(headers)
            writer.writerows(csv_rows)


# TODO: possibly move to separate utils file
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


def print_if_logging(message: str = "") -> None:
    global silent
    if not silent:
        print(message)


def print_debug(message: str = "") -> None:
    global debug_print
    if debug_print:
        print(message)


def print_quaternion_debug(name: str, q: mn.Quaternion, color) -> None:
    global debug_print
    if debug_print:
        decimal = 1
        angle = round(float(mn.Deg(q.angle())), decimal)
        x = round(q.axis().x, decimal)
        y = round(q.axis().y, decimal)
        z = round(q.axis().z, decimal)
        print(color + name + f"{angle} ({x}, {y}, {z})")


def print_mem_usage_info(
    start_mem: Dict[str, int],
    end_mem: Dict[str, int],
    avg_ram_used_str: str,
) -> None:
    """"""
    # Print memory usage info before loading object
    text_format = ANSICodes.BRIGHT_RED.value
    print_if_logging(text_format + "\nstart mem state" + section_divider)
    for key, value in start_mem.items():
        value_str = value
        if key != "percent":
            value_str = get_mem_size_str(value)
        print_if_logging(text_format + f"{key} : {value_str}")

    # Print memory usage info after loading object
    print_if_logging(text_format + "\nend mem state" + section_divider)
    for key, value in end_mem.items():
        value_str = value
        if key != "percent":
            value_str = get_mem_size_str(value)
        print_if_logging(text_format + f"{key} : {value_str}")

    # Print difference in memory usage before and after loading object
    print_if_logging(text_format + "\nchange in mem states" + section_divider)
    for (key_s, value_s), (key_e, value_e) in zip(start_mem.items(), end_mem.items()):
        value_str = value_e - value_s
        if key_s != "percent" and key_e != "percent":
            value_str = get_mem_size_str(value_e - value_s)
        print_if_logging(text_format + f"{key_s} : {value_str}")

    # Print rough estimate of RAM used when loading object
    print_if_logging(
        text_format + "\naverage RAM used" + section_divider + f"\n{avg_ram_used_str}"
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
    obj: habitat_sim.physics.ManagedBulletRigidObject,
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
    sim: habitat_sim.simulator,
    obj: habitat_sim.physics.ManagedBulletRigidObject,
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
            ray = habitat_sim.geo.Ray(world_point, gravity_dir)
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
    sim: habitat_sim.simulator,
    obj: habitat_sim.physics.ManagedBulletRigidObject,
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


def process_mem_usage(
    importer: trade.AbstractImporter,
    template: attributes.ObjectAttributes,
) -> List[str]:
    """
    Use the trade.AbstractImporter class to query data size of mesh and image
    of asset
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
    print_if_logging(
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
    ...


def process_physics(
    sim: habitat_sim.simulator,
    rigid_obj: physics.ManagedBulletRigidObject,
    sim_settings: Dict[str, Any],
) -> List[str]:
    """
    Run series of tests on asset to see how it responds in physics simulations
    """
    # we must test object in 6 different orientations, each corresponding to a face
    # of an imaginary cube bounding the object. Each rotation in rotations is of the form:
    # (angle, (axis.x, axis.y, axis.z)) where angle is in degrees
    rotations: List[Tuple] = [
        (mn.Rad(mn.Deg(rotation[0])), mn.Vector3(rotation[1]))
        for rotation in sim_settings["sim_test_rotations"]
    ]
    times: List[float] = [0.0] * len(rotations)
    translation_deltas: List[float] = [0.0] * len(rotations)
    rotation_deltas: List[mn.Quaternion] = [mn.Quaternion.identity_init()] * len(
        rotations
    )

    # TODO: debugging, remove
    obj_init_attributes = rigid_obj.creation_attributes
    text_format = ANSICodes.BRIGHT_RED.value
    print_debug(
        text_format
        + f"\nstart simulating {basename(obj_init_attributes.handle)}"
        + section_divider
    )

    # Loop over the 6 rotations and simulate snapping the object down and waiting for
    # it to become idle (at least until "max_wait_time" from the config file expires)
    default_position = sim_settings["default_transforms"].get("default_obj_pos")
    dt = 1.0 / sim_settings["physics_vars"].get("fps")  # seconds
    max_wait_time = sim_settings["physics_vars"].get("max_wait_time")  # seconds
    for i in range(len(rotations)):

        # Reset object state with new rotation
        rigid_obj.motion_type = physics.MotionType.DYNAMIC
        rigid_obj.awake = True
        rigid_obj.translation = default_position
        angle = mn.Deg(rotations[i][0])
        axis = rotations[i][1]
        rigid_obj.rotation = mn.Quaternion.rotation(angle, axis)

        # TODO: debugging, remove
        print_debug(f"init as quaternion - {rigid_obj.rotation}")
        print_quaternion_debug("init - ", rigid_obj.rotation, ANSICodes.YELLOW.value)

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
        print_quaternion_debug("snap - ", snap_rot, ANSICodes.BRIGHT_CYAN.value)

        # simulate until rigid object becomes idle or time limit from config file runs out
        while rigid_obj.awake and times[i] < max_wait_time:
            sim.step_physics(dt)
            times[i] += dt

        # TODO: debugging, remove
        print_quaternion_debug(
            "idle - ", rigid_obj.rotation, ANSICodes.BRIGHT_MAGENTA.value
        )

        # store final information once object becomes idle or it times out
        times[i] = round(times[i], 3)
        translation_deltas[i] = (rigid_obj.translation - snap_pos).length()
        rotation_deltas[i] = rigid_obj.rotation * snap_rot.conjugated()

        # TODO: debugging, remove
        print_quaternion_debug(
            "delta r - ", rotation_deltas[i], ANSICodes.BRIGHT_BLUE.value
        )
        print_debug("")

    # convert results to lists of strings
    time_units = sim_settings["physics_vars"].get("time_units")
    times_as_strs = [
        "timed out" if time >= max_wait_time else f"{time} {time_units}"
        for time in times
    ]
    translation_deltas_strs = [
        f"{round(translation, 3)} units" for translation in translation_deltas
    ]
    decimal = 1
    rotation_deltas_strs = [
        f"angle: {round(float(mn.Deg(r.angle())), decimal)}\n"
        + f"axis: ({round(r.axis().x, decimal)}, {round(r.axis().y, decimal)}, {round(r.axis().z, decimal)})"
        for r in rotation_deltas
    ]

    # return concatenated list of physics results formatted for CSV file row
    return (
        ["..."]
        + [f"--- max {round(max_wait_time, 3)} {time_units} ---"]
        + times_as_strs
        + ["..."]
        + [""]
        + translation_deltas_strs
        + [""]
        + rotation_deltas_strs
    )


def process_asset(
    sim: habitat_sim.simulator,
    importer: trade.AbstractImporter,
    handle: str,
    obj_template_mgr: attributes_managers.ObjectAttributesManager,
    sim_settings: Dict[str, Any],
) -> List[str]:
    """
    Use the trade.AbstractImporter class to query data size of mesh and image
    of asset and how the asset behaves during physics simulations
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
    print_if_logging(
        text_format
        + "\n"
        + "Handles"
        + section_divider
        + "\n"
        + f"-template: {handle}\n"
    )

    # run object through tests defined in dataset_processor_config.json file
    always_displayed = [basename(handle), avg_ram_used_str]
    memory_data: List[str] = []
    render_time_data: List[str] = []
    physics_data: List[str] = []
    data_to_collect = sim_settings["data_to_collect"]
    if data_to_collect.get("memory_data"):
        memory_data += process_mem_usage(importer, template)
    if data_to_collect.get("render_time_ratio"):
        render_time_data.append("...")
    if data_to_collect.get("physics_data"):
        physics_data = process_physics(sim, rigid_obj, sim_settings)

    # return results as a list of strings formatted for csv rows
    return always_displayed + memory_data + render_time_data + physics_data


def parse_dataset(
    sim: habitat_sim.simulator,
    sim_settings: Dict[str, Any],
) -> List[List[str]]:
    """
    Load and process dataset objects using template handles
    """
    dataset_path = sim_settings["scene_dataset_config_file"]
    metadata_mediator = sim.metadata_mediator
    if (
        metadata_mediator is None
        or metadata_mediator.dataset_exists(dataset_path) is False
    ):
        raise RuntimeError(
            ANSICodes.FAIL.value
            + "No meta data mediator or dataset exists in parse_dataset(...)."
        )

    # get dataset that is currently being used by the simulator
    active_dataset: str = metadata_mediator.active_dataset
    text_format = ANSICodes.BRIGHT_BLUE.value + ANSICodes.BOLD.value
    print_if_logging(text_format + "\nActive Dataset" + section_divider)
    text_format = ANSICodes.BRIGHT_BLUE.value
    print_if_logging(text_format + f"{active_dataset}\n")

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

    # clean up
    importer.close()

    # return list of rows to be written to csv file
    return csv_rows


def get_csv_headers(sim_settings) -> List[str]:
    """
    Collect the column titles we'll need given which tests we ran
    """
    headers: List[str] = sim_settings["always_displayed"]
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
    """
    file_path = CSVWriter.create_unique_filename(csv_dir_path, csv_file_prefix)

    text_format = ANSICodes.PURPLE.value + ANSICodes.BOLD.value
    print_if_logging(text_format + "Writing csv results to:" + section_divider)
    text_format = ANSICodes.PURPLE.value
    print_if_logging(text_format + f"{file_path}\n")

    CSVWriter.write_file(headers, csv_rows, file_path)

    text_format = ANSICodes.PURPLE.value
    print_if_logging(text_format + "CSV writing done\n")

    # TODO: remove this when I figure out why ram usage is sometimes negative
    text_format = ANSICodes.BRIGHT_RED.value
    global negative_ram_count
    print_if_logging(
        text_format
        + f"Negative RAM usage count: {negative_ram_count}"
        + section_divider
    )


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
        "--config_file_path",
        default="tools/dataset_object_processor/configs/default.dataset_processor_config.json",
        type=str,
        help="config file to load"
        ' (default: "tools/dataset_object_processor/configs/default.dataset_processor_config.json")',
    )
    parser.add_argument(
        "--csv_file_prefix",
        default=None,
        type=str,
        metavar="PREFIX",
        help="Prefix string of file name for CSV and/or video recording file to create, e.g."
        " ycb,"
        " replica_CAD,"
        " (default: None)",
    )
    return parser


def main() -> None:
    """
    Create Simulator, parse dataset, write csv file
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
    # after each print_if_logging statement
    global silent
    global debug_print
    init(autoreset=True)
    silent = sim_settings["silent"]
    debug_print = sim_settings["debug_print"]

    # Configure and make simulator
    cfg = make_cfg(sim_settings)
    sim = habitat_sim.Simulator(cfg)

    # Print sim settings
    text_format = ANSICodes.PURPLE.value
    for key, value in sim_settings.items():
        print_if_logging(text_format + f"{key} : {value}\n")

    # Parse dataset and write CSV. "headers" stores the titles of each column
    headers = get_csv_headers(sim_settings)
    csv_rows: List[List[str]] = parse_dataset(sim, sim_settings)
    csv_dir_path = os.path.join(
        HABITAT_SIM_PATH, sim_settings["output_paths"].get("csv")
    )
    csv_file_prefix = sim_settings["output_paths"].get("output_file_prefix")
    create_csv_file(headers, csv_rows, csv_dir_path, csv_file_prefix)


if __name__ == "__main__":
    main()
