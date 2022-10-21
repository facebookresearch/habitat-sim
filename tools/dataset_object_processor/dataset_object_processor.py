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
from processor_utils import ANSICodes, CSVWriter, RotationAxis

import habitat_sim as hsim
from habitat_sim import attributes, attributes_managers, physics
from habitat_sim.logging import logger
from habitat_sim.utils import viz_utils as vut

repo = git.Repo(".", search_parent_directories=True)
HABITAT_SIM_PATH = repo.working_tree_dir


class DatasetProcessorSim(hsim.Simulator):
    """ """

    draw_task: str = None
    sim_settings: Dict[str, Any] = None
    curr_obj: physics.ManagedBulletRigidObject = None

    def debug_draw(self, sensor_uuid: Optional[str] = None) -> None:
        r"""Override this method in derived Simulator class to add optional,
        application specific debug line drawing commands to the sensor output.
        See Simulator.get_debug_line_render().
        :param sensor_uuid: The uuid of the sensor being rendered to optionally
        limit debug drawing to specific visualizations (e.g. a third person eval camera)
        """
        # determine which task we are drawing and call the associated function
        if self.draw_task == "draw_bbox":
            self.debug_draw_bbox()
        elif self.draw_task == "draw_collision_asset":
            self.debug_draw_collision_asset()
        elif self.draw_task == "draw_bullet_collision_mesh":
            self.debug_draw_bullet_collision_mesh()

    def debug_draw_bbox(self) -> None:
        """ """
        rgb = self.sim_settings["bbox_rgb"]
        line_color = mn.Color4.from_xyz(rgb)
        bb_corners: List[mn.Vector3] = pcsu.get_bounding_box_corners(self.curr_obj)
        num_corners = len(bb_corners)
        self.get_debug_line_render().set_line_width(0.01)
        obj_transform = self.curr_obj.transformation

        # only need to iterate over first 4 corners to draw whole thing
        for i in range(int(num_corners / 2)):
            # back of box
            back_corner_local_pos = bb_corners[i]
            back_corner_world_pos = obj_transform.transform_point(back_corner_local_pos)
            next_back_index = (i + 1) % 4
            next_back_corner_local_pos = bb_corners[next_back_index]
            next_back_corner_world_pos = obj_transform.transform_point(
                next_back_corner_local_pos
            )
            self.get_debug_line_render().draw_transformed_line(
                back_corner_world_pos,
                next_back_corner_world_pos,
                line_color,
            )
            # side edge that this corner is a part of
            front_counterpart_index = num_corners - i - 1
            front_counterpart_local_pos = bb_corners[front_counterpart_index]
            front_counterpart_world_pos = obj_transform.transform_point(
                front_counterpart_local_pos
            )
            self.get_debug_line_render().draw_transformed_line(
                back_corner_world_pos,
                front_counterpart_world_pos,
                line_color,
            )
            # front of box
            next_front_index = (front_counterpart_index - 4 - 1) % 4 + 4
            next_front_corner_local_pos = bb_corners[next_front_index]
            next_front_corner_world_pos = obj_transform.transform_point(
                next_front_corner_local_pos
            )
            self.get_debug_line_render().draw_transformed_line(
                front_counterpart_world_pos,
                next_front_corner_world_pos,
                line_color,
            )

    def debug_draw_collision_asset(self) -> None:
        ...

    def debug_draw_bullet_collision_mesh(self) -> None:
        ...


def bounding_box_ray_prescreen(
    sim: DatasetProcessorSim,
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
    bounding_box_corners = pcsu.get_bounding_box_corners(obj)
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
    sim: DatasetProcessorSim,
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
    sim: DatasetProcessorSim,
    angle_delta: float,
) -> Dict[int, Any]:
    """ """
    observations = []
    for axis in list(RotationAxis):
        curr_angle = 0.0
        finished = False
        while curr_angle < 360.0:
            # determine if this iteration will push the object past
            # 360.0 degrees, meaning we are done
            if curr_angle + angle_delta >= 360.0:
                finished = True
                angle_delta = 360.0 - curr_angle
                curr_angle = 360.0

            # rotate about object's local x or y axis
            rot_rad = mn.Rad(mn.Deg(angle_delta))
            sim.curr_obj.rotate_local(rot_rad, mn.Vector3(axis))

            # save this observation in a buffer of frames
            observations.append(sim.get_sensor_observations())

            # update current rotation angle
            curr_angle += angle_delta

            # reset current rotation angle if it passed 360 degrees
            if finished:
                curr_angle = 360.0

    return observations


def record_asset_video(sim: DatasetProcessorSim) -> None:
    """
    Loop over each recording task specified in the config file, i.e. show bounding box,
    show collision asset, and show bullet collision mesh, then using the transforms
    of the agent and sensor, record the observations of the revolving ManagedBulletRigidObject,
    which is in Kinematic mode and just being displayed, not simulated. The object
    currently revolves 360 degrees once around its x-axis, then once around its y-axis for
    each task. Then save the cumulation of observations as a video file.
    :param sim: The Simulator instance.
    """
    # init video and rotation parameters
    video_vars: Dict[str, Any] = sim.sim_settings["video_vars"]
    tasks: Dict[str, bool] = video_vars.get("tasks")
    deg_per_sec = 360.0 / sim.sim_settings["video_vars"].get("revolution_dur")
    dt = 1.0 / sim.sim_settings["physics_vars"].get("fps")
    angle_delta = deg_per_sec * dt

    # init object transforms
    default_transforms = sim.sim_settings["default_transforms"]
    obj_rot = default_transforms.get("default_obj_rot")
    sim.curr_obj.translation = default_transforms.get("default_obj_pos")
    angle = mn.Rad(mn.Deg(obj_rot.get("angle")))
    axis = mn.Vector3(obj_rot.get("axis"))
    sim.curr_obj.rotation = mn.Quaternion.rotation(angle, axis)
    sim.curr_obj.motion_type = physics.MotionType.KINEMATIC

    # Loop over each recording task specified in the config file
    observations = []
    for task, required in tasks.items():
        if required == False:
            continue
        # record object rotating about its x-axis, then its y-axis
        pcsu.print_if_logging(ANSICodes.YELLOW.value + "draw " + task)
        sim.draw_task = task
        observations += record_revolving_obj(sim, angle_delta)

    # construct file path and write "observations" to video file
    video_file_dir = os.path.join(
        HABITAT_SIM_PATH, sim.sim_settings["output_paths"].get("video")
    )
    obj_handle = sim.curr_obj.handle.replace("_:0000", "")
    video_file_prefix = sim.sim_settings["output_paths"].get("output_file_prefix")
    video_file_prefix += f"_{obj_handle}"
    file_path = pcsu.create_unique_filename(video_file_dir, ".mp4", video_file_prefix)
    vut.make_video(
        observations,
        "color_sensor",
        "color",
        file_path,
        open_vid=False,
    )


def process_asset_mem_usage(
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
        pcsu.get_mem_size_str(index_data_size),
        pcsu.get_mem_size_str(vertex_data_size),
        pcsu.get_mem_size_str(total_mesh_data_size),
        f"{image_count} {image_count_units}",
        pcsu.get_mem_size_str(image_data_size),
    ]


def process_asset_render_time() -> List[str]:
    return ["..."]


def process_asset_physics(sim: DatasetProcessorSim) -> List[str]:
    """
    Run series of tests on asset to see how it responds in physics simulations.
    We snap an object down onto the surface below, then see how long it takes
    for the object to stabilize and become idle.
    :param sim: The Simulator instance.
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

    # Print object that we will be imminently simulating
    text_format = ANSICodes.BRIGHT_RED.value
    pcsu.print_if_logging(
        text_format
        + f"\nstart simulating {basename(sim.curr_obj.creation_attributes.handle)}"
        + pcsu.section_divider
    )

    # Loop over the 6 rotations and simulate snapping the object down and waiting for
    # it to become idle (at least until "max_wait_time" from the config file expires)
    default_pos = sim.sim_settings["default_transforms"].get("default_obj_pos")
    dt = 1.0 / sim.sim_settings["physics_vars"].get("fps")  # seconds
    max_wait_time = sim.sim_settings["physics_vars"].get("max_wait_time")  # seconds
    for i in range(len(rotations)):

        # Reset object state with new rotation using angle-axis to construct
        # a quaternion. If axis is (0, 0, 0), that means there is no rotation
        sim.curr_obj.motion_type = physics.MotionType.DYNAMIC
        sim.curr_obj.awake = True
        sim.curr_obj.translation = default_pos
        angle = rotations[i][0]
        axis = rotations[i][1]
        if axis.is_zero():
            sim.curr_obj.rotation = mn.Quaternion.identity_init()
        else:
            sim.curr_obj.rotation = mn.Quaternion.rotation(angle, axis)

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

        # store final information once object becomes idle or it times out
        wait_times[i] = round(wait_times[i], 3)
        translation_deltas[i] = (sim.curr_obj.translation - snap_pos).length()
        rotation_deltas[i] = sim.curr_obj.rotation * snap_rot.conjugated()
        sim_time_ratios[i] /= sim_steps

    # convert results to lists of strings for csv file
    time_units = sim.sim_settings["physics_vars"].get("time_units")
    times_as_strs = [
        "****timed out****" if t >= max_wait_time else f"{t} {time_units}"
        for t in wait_times
    ]
    decimal = 3
    translation_deltas_strs = [f"{round(t, decimal)} units" for t in translation_deltas]
    decimal = 1
    angle_units = sim.sim_settings["physics_vars"].get("angle_units")
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
    sim: DatasetProcessorSim,
    importer: trade.AbstractImporter,
    handle: str,
    obj_template_mgr: attributes_managers.ObjectAttributesManager,
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
    metrics: List[str] = sim.sim_settings["mem_metrics_to_use"]
    avg_ram_delta = 0  # bytes
    for metric in metrics:
        subtract_order = sim.sim_settings["mem_delta_order"].get(metric)
        # "percent" isn't calculated in bytes, so skip it
        if metric != "percent":
            avg_ram_delta += (
                end_mem.get(metric) - start_mem.get(metric)
            ) * subtract_order

    avg_ram_delta /= len(metrics)
    avg_ram_used_str = pcsu.get_mem_size_str(avg_ram_delta)

    # TODO: remove this when I figure out why ram usage is sometimes negative
    global negative_ram_count
    if avg_ram_delta < 0:
        negative_ram_count += 1
        avg_ram_used_str = "****" + avg_ram_used_str + "****"

    # Print memory usage info before and after loading object
    pcsu.print_mem_usage_info(start_mem, end_mem, avg_ram_used_str)

    # Log object template handle if "silent" is false in the config file
    text_format = Fore.GREEN
    pcsu.print_if_logging(
        text_format
        + "\n"
        + "Handles"
        + pcsu.section_divider
        + "\n"
        + f"-template: {handle}\n"
    )

    # Get object attributes by template handle and associated mesh assets
    template = obj_template_mgr.get_template_by_handle(handle)

    # run object through tests defined in dataset_processor_config.json file
    csv_row: List[str] = []
    if sim.sim_settings["outputs"].get("csv"):
        data_to_collect = sim.sim_settings["data_to_collect"]
        csv_row += [basename(handle)]
        if data_to_collect.get("memory_data"):
            data = ["", avg_ram_used_str] + process_asset_mem_usage(importer, template)
            csv_row += data
        if data_to_collect.get("render_time_ratio"):
            data = [""] + process_asset_render_time()
            csv_row += data
        if data_to_collect.get("physics_data"):
            data = [""] + process_asset_physics(sim)
            csv_row += data

    # record video of object if denoted in dataset_processor_config.json file
    if sim.sim_settings["outputs"].get("video"):
        record_asset_video(sim)

    # remove this object so the next one can be added without interacting
    # with the previous one
    rigid_obj_mgr.remove_all_objects()

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
    pcsu.print_if_logging(text_format + "\nActive Dataset" + pcsu.section_divider)
    text_format = ANSICodes.BRIGHT_BLUE.value
    pcsu.print_if_logging(text_format + f"{active_dataset}\n")

    # Get ObjectAttributesManager for objects from dataset, load the dataset,
    # and store all the object template handles in a List
    obj_template_mgr = sim.get_object_template_manager()
    obj_template_mgr.load_configs(dataset_path)
    object_template_handles = obj_template_mgr.get_file_template_handles("")

    # Init vars to store the list of rows formatted for a csv file, as well as a
    # trade.AbstractImporter to assess the memory usage of the assets
    csv_rows: List[List[str]] = []
    manager = trade.ImporterManager()
    importer = manager.load_and_instantiate("AnySceneImporter")

    # loop over the assets specified in the config file, starting with index
    # "start_obj_index" and proceeding for "num_objects" iterations of the loop.
    # If "start_obj_index" and/or "num_objects" is not specified in the config file,
    # or is not an int, process every asset in the dataset
    start_index = sim.sim_settings["start_obj_index"]
    num_objs = sim.sim_settings["num_objects"]
    if not isinstance(start_index, int) or not isinstance(num_objs, int):
        start_index = 0
        num_objs = len(object_template_handles)

    for i in range(num_objs):
        if i >= len(object_template_handles):
            break
        handle = object_template_handles[start_index + i]
        row = process_asset(sim, importer, handle, obj_template_mgr)
        csv_rows.append(row)

    # Write csv if specified in the config file. "headers" stores the titles of
    # each csv column.
    if sim.sim_settings["outputs"].get("csv"):
        headers = get_csv_headers(sim)
        csv_dir_path = os.path.join(
            HABITAT_SIM_PATH, sim.sim_settings["output_paths"].get("csv")
        )
        csv_file_prefix = sim.sim_settings["output_paths"].get("output_file_prefix")
        create_csv_file(headers, csv_rows, csv_dir_path, csv_file_prefix)

    # clean up
    importer.close()

    # return list of rows to be written to csv file
    return csv_rows


def get_csv_headers(sim: DatasetProcessorSim) -> List[str]:
    """
    Collect the csv column titles we'll need given which tests we ran
    """
    headers: List[str] = sim.sim_settings["object_name"]
    data_to_collect = sim.sim_settings["data_to_collect"]
    if data_to_collect.get("memory_data"):
        headers += sim.sim_settings["memory_data_headers"]
    if data_to_collect.get("render_time_ratio"):
        headers += sim.sim_settings["render_time_headers"]
    if data_to_collect.get("physics_data"):
        headers += sim.sim_settings["physics_data_headers"]

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
        text_format + "\nWriting csv results to:" + pcsu.section_divider
    )
    text_format = ANSICodes.PURPLE.value
    pcsu.print_if_logging(text_format + f"{file_path}\n")

    CSVWriter.write_file(headers, csv_rows, file_path)

    text_format = ANSICodes.PURPLE.value
    pcsu.print_if_logging(text_format + "CSV writing done\n")


def update_sim_settings(
    sim_settings: Dict[str, Any], config_settings
) -> Dict[str, Any]:
    """
    Update nested sim_settings dictionary. Modifies sim_settings in place.
    """
    for key, value in config_settings.items():
        if isinstance(value, Dict) and value:
            returned = update_sim_settings(sim_settings.get(key, {}), value)
            sim_settings[key] = returned
        else:
            sim_settings[key] = config_settings[key]

    return sim_settings


def configure_sim(sim_settings: Dict[str, Any]):
    """
    Configure simulator while adding post configuration for the transform of
    the agent
    """
    cfg = make_cfg(sim_settings)
    sim = DatasetProcessorSim(cfg)
    default_transforms = sim_settings["default_transforms"]

    # init agent
    agent_state = hsim.AgentState()
    agent = sim.initialize_agent(sim_settings["default_agent"], agent_state)
    agent.body.object.translation = mn.Vector3(
        default_transforms.get("default_agent_pos")
    )
    agent_rot = default_transforms.get("default_agent_rot")
    angle = agent_rot.get("angle")
    axis = mn.Vector3(agent_rot.get("axis"))

    # construct rotation as quaternion, and if axis is (0, 0, 0), that means there
    # is no rotation
    if axis.is_zero():
        agent.body.object.rotation = mn.Quaternion.identity_init()
    else:
        agent.body.object.rotation = mn.Quaternion.rotation(mn.Rad(mn.Deg(angle)), axis)

    sim.sim_settings = sim_settings
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
    # parse arguments from command line: scene, dataset, if we process physics
    args = build_parser().parse_args()

    # Populate sim_settings with info from dataset_processor_config.json file
    sim_settings: Dict[str, Any] = default_sim_settings
    with open(os.path.join(HABITAT_SIM_PATH, args.config_file_path)) as config_json:
        update_sim_settings(sim_settings, json.load(config_json))

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

    # TODO: remove this when I figure out why ram usage is sometimes negative
    global negative_ram_count
    negative_ram_count = 0

    # Iterate through dataset objects and write csv and/or make video recording
    # if specified in the config file.
    process_dataset(sim)

    # TODO: remove this when I figure out why ram usage is sometimes negative
    text_format = ANSICodes.BRIGHT_MAGENTA.value
    pcsu.print_debug(
        text_format
        + f"negative RAM usage count: {negative_ram_count}"
        + pcsu.section_divider
    )


if __name__ == "__main__":
    main()
