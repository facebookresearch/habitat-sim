import argparse
import json
import glob
import os.path
from os import makedirs
import csv
import gc
import cv2
import magnum as mn
import numpy as np
from habitat.sims.habitat_simulator.debug_visualizer import (
    DebugObservation,
    DebugVisualizer,
)
from PIL import Image, ImageChops, ImageDraw, ImageFont
from PIL.Image import Image as ImageClass

from habitat_sim import Simulator
from habitat_sim.metadata import MetadataMediator
from habitat_sim.physics import ManagedRigidObject

from habitat_sim.utils.settings import default_sim_settings, make_cfg


def register_glb(leaf: str) -> tuple[str, str]:
    """
    Registers a GLB file to a template in the library so it can be added to the scene.
    The registered GLB will have a library handle matching its GLB filename without extension
    """
    asset_path = (
        f"data/fp-models/objects/decomposed/{object_uuid}/{object_uuid}{leaf}.glb"
    )
    library_handle = f"{object_uuid}{leaf}"
    glb_template = template_manager.create_new_template(handle=asset_path)
    assert glb_template.has_value("render_asset")
    template_id = template_manager.register_template(
        template=glb_template, specified_handle=library_handle
    )
    return library_handle, template_id


def register_meshes(object_uuid: str) -> list:
    """
    Given a UUID, loads the part models into the library.
    Returns a list of 2-element lists [object handle,object semantic label].
    """
    print(f"{object_uuid} :")
    return_data = []
    parts_metadata_path = f"data/fp-models/objects/decomposed/{object_uuid}/{object_uuid}.parts.metadata.json"
    with open(parts_metadata_path, "r") as file:
        json_data = json.load(file)

    for obj_data in json_data.get("parts", {}):
        label = ""
        if obj_data.get("isRef") is not False:  # only register unique GLBs
            part_Id = obj_data.get("partId", "")
            label = obj_data.get("label", "")
            handle, template_id = register_glb(f"_part_{part_Id}")
            print(f"Part {part_Id} - {label} - ID {template_id}")
            return_data.append([handle, label])
    return return_data


def observe_mesh(
    object_handle: str,
) -> tuple[DebugObservation, float]:
    """
    Loads and takes a peek image of the named object, and also returns its bounding box volume
    """
    # dbug
    dbug = False
    subject = rom.add_object_by_template_handle(object_lib_handle=f"{object_handle}")
    observation = dbv.peek(subject, peek_all_axis=True)
    bb = subject.aabb
    bb_size = bb.size()
    bb_volume = bb_size.x * bb_size.y * bb_size.z
    if not dbug:
        rom.remove_object_by_handle(handle=subject.handle)
    return observation, bb_volume


def observe_part(object_handle: str, label: str) -> None:
    print(f"Observing {object_handle} - {label}")
    part_obs, part_volume = observe_mesh(object_handle)
    result_image = make_image(part_obs, object_handle, label, part_volume)

    if part_volume < VOLUME_THRESHOLD:  
        pickable_status = "Yes"
    else:
        pickable_status = "No"

    # log to logfile
    with open(log_path, "a", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                object_handle,
                "No",
                pickable_status,
                "No",
                "",
                label,
                "TODO",
                "TODO",
                str(part_volume),
            ]
        )

    # log to console
    if not args.batch:
        result_image.show()

    try:
        out_path = f"{args.folder}/parts_census/{label}"
        makedirs(out_path, exist_ok=True)
        destination = f"{out_path}/{object_handle}.png"
        result_image.save(destination)
    except IOError:
        print(f"{COLOR_WARNING}Can't save image for{COLOR_RESET} {object_handle}!")


def make_image(
    part_obs: DebugObservation, handle: str, label: str, part_volume: float
) -> ImageClass:
    """Returns the debug peek with superimposed text of the UUID, volume, and label."""

    parts_img_PIL = part_obs.get_image().convert("RGB")

    # write the UUID, semantic category and volume on the image
    draw_context = ImageDraw.Draw(parts_img_PIL)
    draw_font = ImageFont.truetype("data/fonts/ProggyClean.ttf", 64)
    text_color = (255, 255, 0)
    if part_volume < VOLUME_THRESHOLD:
        text_color = (0,255,0)
    draw_context.text(
        (64, 64),
        f"{handle}\n{label}\nVolume: {part_volume}",
        font=draw_font,
        fill= text_color,
    )

    return parts_img_PIL


# Declarations
VOLUME_THRESHOLD = 0.07 # ~ 3 cubic feet
MAX_ITEMS = 500
CHUNK_SIZE = 25
COLOR_WARNING = "\033[93m"
COLOR_RESET = "\033[0m"

parts_list = []
image_size = 1024

if __name__ == "__main__":
    argparser = argparse.ArgumentParser()
    argparser.add_argument(
        "-o",
        "--object",
        type=str,
        default="00a2b0f3886ccb5ffddac704f8eeec324a5e14c6",  # bookshelf
        # default="a62fd2ce7682a979db3b7b0d4cd541ae15ea6db7", #table with 5 rotated chairs
        help="the UUID of the decomposed object to load, default 00a2b0f3886ccb5ffddac704f8eeec324a5e14c6",
        action="store",
    )
    argparser.add_argument(
        "-d",
        "--dataset",
        type=str,
        default="fp-models",
        help="the dataset to use, default fp-models",
        action="store",
    )
    argparser.add_argument(
        "-b",
        "--batch",
        default=False,
        action="store_true",
        help="Run as a batch over multiple models",
    )
    argparser.add_argument(
        "-f",
        "--folder",
        type=str,
        default="/home/jseagull/dev/fp-models/diagnostics",
        help="output folder for images and CSV",
        action="store",
    )
    args = argparser.parse_args()

    # start logging
    assert os.path.exists(args.folder)
    log_data = []
    log_path = f"{args.folder}/parts_census/parts_census.csv"
    with open(log_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "uuid",
                "batch",
                "articulated_object",
                "pickable",
                "needs_decomp",
                "ctgry",
                "category",
                "collision_proxy",
                "receptacle",
            ]
        )

    if not args.batch:
        object_uuid_list = [args.object]
        chunked_part_list = [object_uuid_list]
    else:
        full_uuid_list = [
            os.path.basename(dir)
            for dir in glob.glob("/home/jseagull/dev/fp-models/objects/decomposed/*")
        ]

    sim_settings = default_sim_settings.copy()
    sim_settings["width"] = sim_settings["height"] = image_size
    sim_settings["enable_hbao"] = True  # Ambient Occlusion
    sim_settings["default_agent_navmesh"] = False
    sim_settings["scene_dataset_config_file"] = (
        args.dataset
    )  # sim and mm both need to know about the dataset
    # sim_settings["scene"] = scene

    cfg = make_cfg(sim_settings)
    mm = MetadataMediator()
    mm.active_dataset = args.dataset
    cfg.metadata_mediator = mm
    template_manager = mm.object_template_manager

    print("\n######## Initializing Assets... ########################\n")
    print(f"active dataset: {mm.active_dataset}")
    print(f"Dataset exists? {mm.dataset_exists(args.dataset)}\n")
    print("Populating templates...")
    for object_uuid in full_uuid_list:
        # register the parts in the library, and add the object's JSON data to our local dictionary
        parts_list += register_meshes(object_uuid)
        print(parts_list)
    print("\n######## Starting Simulator... #########################\n")

    # chunk the parts list
    chunked_part_list = []
    for i in range(0, len(parts_list), CHUNK_SIZE):
        chunked_part_list.append(parts_list[i : i + CHUNK_SIZE])

    for sublist in chunked_part_list:
        with Simulator(cfg) as sim:
            print("")
            dbv = DebugVisualizer(sim, resolution=(1024, 1024))
            rom = sim.get_rigid_object_manager()
            for part in sublist:
                observe_part(part[0], part[1])
            sim.close()
            gc.collect()

    # write log
