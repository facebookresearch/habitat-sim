import argparse
import json
import glob
import os.path
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


def register_glb(leaf: str) -> None:
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
    # print(f"Registered {leaf} as {glb_template.handle} with library ID {template_id}")


def register_meshes(object_uuid: str) -> dict:
    """
    Given a UUID, loads the reference model and part models into the library.
    Returns a dict with the object's JSON data.
    """
    print(f"{object_uuid} ", end="")
    object_leaf_list = ["_dedup"]
    parts_metadata_path = f"data/fp-models/objects/decomposed/{object_uuid}/{object_uuid}.parts.metadata.json"
    with open(parts_metadata_path, "r") as file:
        json_data = json.load(file)

    for obj_data in json_data.get("parts", {}):
        if obj_data.get("isRef") is not False:  # only register unique GLBs
            part_Id = obj_data.get("partId", "")
            object_leaf_list.append(f"_part_{part_Id}")
    print(f"({len(object_leaf_list)} meshes)... ", end="")
    for obj_data in object_leaf_list:
        register_glb(obj_data)
    print("OK")
    return json_data


def observe_dedup_object(
    object_uuid,
) -> tuple[DebugObservation, mn.Range3D, mn.Matrix4]:
    """
    Loads and takes a peek image of the reference object, and also returns its transform
    and AABB so we can use that to frame the assembled decomp objects
    """
    # dbug
    dbug = False
    poi = rom.add_object_by_template_handle(object_lib_handle=f"{object_uuid}_dedup")
    observation = dbv.peek(poi, peek_all_axis=True)
    bb = poi.aabb
    transform = poi.transformation
    if not dbug:
        rom.remove_object_by_handle(handle=poi.handle)
    return observation, bb, transform


def place_object(scene_obj: ManagedRigidObject, obj_data: dict) -> None:
    """loads an object from the library and positions it according to transform data from the JSON"""
    # snippets
    # obj_xform = np.array(obj_data.get("transform", {}).get("matrix", None))
    # obj_xform = np.reshape(obj_xform, (1, 4, 4))
    # obj_xform = np.swapaxes(obj_xform, 1, 2)
    # obj_xform = mn.Matrix4(obj_xform[0])
    # obj_rot = obj_data.get("transform",{}).get("quaternion",None)
    # obj_scale = obj_data.get("transform",{}).get("scale",None)

    # object imports with COM at origin, but we have to rotate it around its pivot
    scene_obj.translate(scene_obj.com_correction)
    obj_rot = obj_data.get("transform", {}).get("quaternion", None)
    # mn.Quaternion constructor expects nested tuple ((x,y,z), w)
    scene_obj.rotation = mn.Quaternion(
        ((obj_rot[0], obj_rot[1], obj_rot[2]), obj_rot[3])
    )
    scene_obj.translation = (
        mn.Vector3(obj_data.get("transform", {}).get("position", None))
        - scene_obj.com_correction
    )


def observe_composed_object(poi_bb, poi_transform, json_data) -> DebugObservation:
    """
    Loads and places all components of the decomposed object, takes a peek, and unloads them
    """
    dbug = False
    for obj_data in json_data.get("parts", {}):
        # dbug
        part_Id = obj_data.get("partId", "")
        if dbug and part_Id != "19":
            continue
        render_mesh_Id = part_Id
        if obj_data.get("isRef") is False:  # Is a child instance
            render_mesh_Id = obj_data.get("refPartId", "")

        scene_obj = rom.add_object_by_template_handle(
            object_lib_handle=f"{object_uuid}_part_{render_mesh_Id}"
        )
        # print(f"Added part {part_Id} (referencing mesh {render_mesh_Id})")

        place_object(scene_obj, obj_data)

    observation = dbv.peek(poi_bb, peek_all_axis=True, subject_transform=poi_transform)
    rom.remove_all_objects()
    return observation


def compare_decomposed(object_uuid) -> None:
    this_object = object_data.get(object_uuid, {})
    print(f"{object_uuid}...", end="")
    dedup_obs, poi_bb, poi_transform = observe_dedup_object(object_uuid)
    parts_obs = observe_composed_object(poi_bb, poi_transform, this_object)
    results = analyze_observations(dedup_obs, parts_obs, "")
    # log to logfile
    with open(log_path, "a", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow([object_uuid, results[1]])
    # log to console
    if results[1] < 0.25:
        print(f"OK")
        destination_subfolder = "pass"
    else:
        print(f"{COLOR_WARNING}CHECK ({results[1]}% variance){COLOR_RESET}")
        destination_subfolder = "fail"

    if not args.batch:
        results[0].show()

    try:
        destination = f"{args.folder}/{destination_subfolder}/{object_uuid}.png"
        results[0].save(destination)
    except IOError:
        print(f"{COLOR_WARNING}Can't save image for{COLOR_RESET} {object_uuid}!")


def analyze_observations(
    dedup_obs: DebugObservation, parts_obs: DebugObservation, destination_folder: str
) -> tuple[ImageClass, float]:
    """Returns a composite image of the reference mesh image, the composed parts image, and an image showing the difference between the two. Also returns a score of how different they are."""

    def PIL_to_cv2(input_img: ImageClass) -> np.array:
        """Converts an 8 bpc PIL RGB image to a BGR numpy array for processing with OpenCV"""
        np_arr = np.array(input_img).astype(np.uint8)
        output_arr = np.zeros(np.shape(np_arr), dtype=np.uint8)
        output_arr = cv2.cvtColor(np_arr, cv2.COLOR_RGB2BGR)
        return output_arr

    def cv2_to_PIL(input_arr: np.array) -> ImageClass:
        """Converts a BGR numpy array to an 8bpc RGB PIL Image"""
        input_arr = cv2.cvtColor(input_arr, cv2.COLOR_BGR2RGB)
        return Image.fromarray(input_arr)

    # prepare PIL and OpenCV versions of the observations for analysis
    dedup_img_PIL = dedup_obs.get_image().convert("RGB")
    dedup_img_cv2 = PIL_to_cv2(dedup_img_PIL)
    parts_img_PIL = parts_obs.get_image().convert("RGB")
    parts_img_cv2 = PIL_to_cv2(parts_img_PIL)

    ### Image analysis ###

    # Absolute difference score
    res = cv2.absdiff(dedup_img_cv2, parts_img_cv2)
    res = res.astype(np.uint8)
    res_nz = np.count_nonzero(res)
    nz_pct = round((res_nz * 100) / res.size, 2)

    # Make difference image
    diff_img_PIL = ImageChops.invert(
        ImageChops.difference(dedup_img_PIL, parts_img_PIL).convert("RGB")
    )

    # Build diagnostic image array
    img_list = [dedup_img_PIL, parts_img_PIL, diff_img_PIL]
    diag_w = parts_img_PIL.size[0]
    diag_h = parts_img_PIL.size[1]
    diag_img = Image.new("RGB", (diag_w, diag_h * 3))
    paste_row = 0
    for i in range(3):
        diag_img.paste(img_list[i], (0, paste_row))
        paste_row += diag_h

    # write the UUID and variance on the image
    draw_context = ImageDraw.Draw(diag_img)
    draw_font = ImageFont.truetype("data/fonts/ProggyClean.ttf", 128)
    draw_context.text(
        (64, (2 * diag_h + 64)),
        f"{object_uuid} - {nz_pct}%",
        font=draw_font,
        fill=(0, 0, 0),
    )

    return (diag_img, nz_pct)


# Declarations

START_ITEM = 0
MAX_ITEMS = 500
CHUNK_SIZE = 25
COLOR_WARNING = "\033[93m"
COLOR_RESET = "\033[0m"

object_data = {}
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
    log_path = f"{args.folder}/decomp_check.csv"
    with open(log_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["uuid", "variance_pct"])

    if not args.batch:
        object_uuid_list = [args.object]
        chunked_uuid_list = [object_uuid_list]
    else:
        full_uuid_list = [
            os.path.basename(dir)
            for dir in glob.glob("/home/jseagull/dev/fp-models/objects/decomposed/*")
        ]
        object_uuid_list = full_uuid_list[0:MAX_ITEMS]
        # test values
        # object_uuid_list = ["00a2b0f3886ccb5ffddac704f8eeec324a5e14c6","a62fd2ce7682a979db3b7b0d4cd541ae15ea6db7"]
        chunked_uuid_list = []
        for i in range(START_ITEM, len(object_uuid_list), CHUNK_SIZE):
            chunked_uuid_list.append(object_uuid_list[i : i + CHUNK_SIZE])

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
    for object_uuid in object_uuid_list:
        # register the _dedup and sub-objects in the library, and add the object's JSON data to our local dictionary
        object_data[object_uuid] = register_meshes(object_uuid)
    print("\n######## Starting Simulator... #########################\n")

    for uuid_sublist in chunked_uuid_list:
        with Simulator(cfg) as sim:
            print("")
            dbv = DebugVisualizer(sim, resolution=(1024, 1024))
            rom = sim.get_rigid_object_manager()
            for object_uuid in uuid_sublist:
                compare_decomposed(object_uuid)
            sim.close()
            gc.collect()

    # write log
