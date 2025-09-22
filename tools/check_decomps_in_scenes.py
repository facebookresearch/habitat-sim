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

from habitat_sim import Simulator, SimulatorConfiguration
from habitat_sim.metadata import MetadataMediator
from habitat_sim.physics import ManagedRigidObject, ManagedBulletRigidObject

from habitat_sim.utils.settings import default_sim_settings, make_cfg

# Declarations

START_ITEM = 0
MAX_ITEMS = 500
CHUNK_SIZE = 25
IMAGE_SIZE = 256
COLOR_WARNING = "\033[93m"
COLOR_RESET = "\033[0m"
DECOMP_DATA_PATH = "/home/jseagull/dev/fphab/decomp_replacements.csv"
SCENE_PATH_ROOT = "/home/jseagull/dev/fphab/scenes"
IMAGE_OUTPUT_PATH_ROOT = "/home/jseagull/dev/fphab/diag/"
cfg = SimulatorConfiguration()

def initialize_habitat() -> tuple[MetadataMediator, SimulatorConfiguration, list]:
    """Register all GLBs as templates, configure simulator, and init the metadata mediator"""

    def register_glb(asset_path: str) -> str:
        """
        Registers a GLB file to a template in the library so it can be added to the scene.
        The registered GLB will have a library handle matching its GLB filename without extension
        """
        library_handle = os.path.splitext(os.path.basename(asset_path))[0]
        glb_template = template_manager.create_new_template(handle=asset_path)
        assert glb_template.has_value("render_asset")
        template_id = template_manager.register_template(
            template=glb_template, specified_handle=library_handle
        )
        return library_handle

    # configure simulator

    sim_settings["width"] = sim_settings["height"] = IMAGE_SIZE
    sim_settings["enable_hbao"] = True  # Ambient Occlusion
    sim_settings["default_agent_navmesh"] = False
    sim_settings["scene_dataset_config_file"] = dataset_old
    cfg = make_cfg(sim_settings)

    # init metadataMediator
    mm = MetadataMediator()
    mm.active_dataset = dataset_old
    cfg.metadata_mediator = mm
    template_manager = mm.object_template_manager

    print("\n######## Initializing Assets... ########################\n")
    print(f"active dataset: {mm.active_dataset}")
    print(f"Dataset exists? {mm.dataset_exists(args.dataset)}")

    # populate Metadata Mediator with object templates
    full_glb_iter = glob.iglob(
        "/home/jseagull/dev/fphab/objects/**/*.glb", recursive=True
    )
    exclude_patterns = [".collider.glb", ".filteredSupportSurface.glb", "openings"]
    full_glb_list = [
        f
        for f in full_glb_iter
        if not any(pattern in f for pattern in exclude_patterns)
    ]
    print(f"Populating {len(full_glb_list)} templates...")
    handle_list = []
    for glb in full_glb_list:
        # register the objects in the library, and add the handle to our list
        handle_list.append(register_glb(glb))

    return mm, cfg, handle_list


def load_decomp_data(csv_filepath: str) -> set:
    """Pull the set of replaceable UUIDs from a CSV"""
    decomp_uuids = set()
    with open(csv_filepath, mode="r", newline="", encoding="utf-8") as csvfile:
        csv_reader = csv.DictReader(csvfile)
        for row in csv_reader:
            if "reference" in row:
                decomp_uuids.add(row["reference"])
    return decomp_uuids


def scan_scene_for_objects(scene_id: str) -> list:
    """
    Given a scene instance name, look at the objects and check against the set of replaceable UUIDs
    Returns a list of replaceable object UUIDs found in the scene
    """
    scene_path = os.path.join(SCENE_PATH_ROOT, (scene_id + ".scene_instance.json"))
    objects_to_replace = set()
    num_replacements = 0
    with open(scene_path, "r") as file:
        scene_instance = json.load(file)
    for obj in scene_instance["object_instances"]:
        if obj["template_name"] in decomp_uuids:
            # print(f"{obj['template_name']} needs replaced...")
            num_replacements += 1
            objects_to_replace.add(obj["template_name"])
    if num_replacements > 0:
        print(
            f"{scene_id} - Found {num_replacements} instances of {len(objects_to_replace)} decomps out of {len(scene_instance['object_instances'])}:"
        )
        print(objects_to_replace)
    return list(objects_to_replace)

def switch_datasets(target:str) -> bool:
    if target is "new":
        target_dataset = dataset_new
    elif target is "old":
        target_dataset = dataset_old
    else:
        return False
    sim_settings["scene_dataset_config_file"] = target_dataset
    mm.active_dataset = target_dataset
    cfg = make_cfg(sim_settings)
    return cfg

def observe_reference_object(
    obj: ManagedBulletRigidObject,
) -> tuple[DebugObservation, mn.Range3D, mn.Matrix4]:
    """
    Loads and takes a peek image of the reference object, and also returns its transform
    and AABB so we can use that to frame the assembled decomp objects
    """
    observation = dbv.peek(obj, peek_all_axis=True)
    obj_bb = obj.aabb
    obj_transform = obj.transformation
    return observation, obj_bb, obj_transform


def observe_new_object(poi_bb, poi_transform) -> DebugObservation:
    """
    Takes a peek at a particular point in a scene, as logged from observe_reference_object()
    """
    observation = dbv.peek(poi_bb, peek_all_axis=True, subject_transform=poi_transform)
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


object_data = {}

if __name__ == "__main__":
    argparser = argparse.ArgumentParser()
    argparser.add_argument(
        "-s",
        "--scene",
        type=str,
        default="102343992",
        help="the scene instance ID to shoot",
        action="store",
    )
    argparser.add_argument(
        "-d",
        "--dataset",
        type=str,
        default="data/fphab/hssd-hab-articulated.scene_dataset_config.json",
        help="the 'before' dataset to use, default hssd-hab-articulated in fphab",
        action="store",
    )
    argparser.add_argument(
        "-b",
        "--batch",
        default=False,
        action="store_true",
        help="Run as a batch over multiple scenes",
    )
    argparser.add_argument(
        "-o",
        "--output_path",
        type=str,
        default="/home/jseagull/dev/fp-models/diagnostics",
        help="output folder for images and CSV",
        action="store",
    )
    args = argparser.parse_args()
    dataset_old = args.dataset
    dataset_new = dataset_old.split(".")[0] + "_v2.scene_dataset_config.json"

    # start logging
    os.environ["MAGNUM_LOG"] = "quiet"
    # os.environ['HABITAT_SIM_LOG']='quiet'

    assert os.path.exists(args.output_path)
    log_data = []
    log_path = f"{args.output_path}/decomp_scene_check.csv"
    with open(log_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["scene", "base_uuid", "variance_pct"])

    if not args.batch:
        scene_list = [args.scene]
        chunked_scene_list = [scene_list]
    else:
        full_scene_list = [
            os.path.splitext(os.path.basename(scene))[0]
            for scene in glob.glob(
                "/home/jseagull/dev/fphab/scenes/*.scene_instance.json"
            )
        ]

    sim_settings = default_sim_settings.copy()
    mm, cfg, object_handle_list = initialize_habitat()
    decomp_uuids = load_decomp_data(DECOMP_DATA_PATH)

    print("\n######## Starting Simulator... #########################\n")

    for scene_id in scene_list:
        sim_settings["scene"] = scene_id
        replaced_uuids = scan_scene_for_objects(scene_id)
        if len(replaced_uuids) == 0:
            print(f"no replaced objects in {scene_id} - skipping")
            continue
        cfg = switch_datasets("old")
        decomp_poi_list = []
        # snap before images
        with Simulator(cfg) as sim:
            print("")
            dbv = DebugVisualizer(sim, resolution=(IMAGE_SIZE, IMAGE_SIZE))
            rom = sim.get_rigid_object_manager()
            handles = []
            for object_uuid in replaced_uuids:
                handles.extend(rom.get_object_handles(search_str=object_uuid))
            for handle in handles:
                ref_object = rom.get_object_by_handle(handle)
                if ref_object is not None:
                    ref_img, poi_bb, poi_transform = observe_reference_object(
                        ref_object
                    )
                    decomp_poi_list.append((poi_bb,poi_transform))
                    before_file = scene_id + "-" + handle + "_ref.png"
                    out_path = os.path.join(IMAGE_OUTPUT_PATH_ROOT, before_file)
                    ref_img.save(out_path)
                    # dbug
                    ref_img.show()
                else:
                    print(f"ERROR - couldn't get object for handle {handle}")
            sim.close()
            gc.collect()

        cfg = switch_datasets("new")
        with Simulator(cfg) as sim:
            print("")
            dbv = DebugVisualizer(sim, resolution=(IMAGE_SIZE, IMAGE_SIZE))
            rom = sim.get_rigid_object_manager()
            for decomp in decomp_poi_list:
                decomp_img = dbv.peek(decomp[0], peek_all_axis=True, subject_transform=decomp[1])
                after_file = scene_id + "-" + handle + "_new.png"
                out_path = os.path.join(IMAGE_OUTPUT_PATH_ROOT, after_file)
                ref_img.save(out_path)
                # dbug
                ref_img.show()
            sim.close()
            gc.collect()

    # write log
"""
Init metadata mediator
set up sim
Get scene list
    for each scene:
    Get list of replaceable objects in original scene
    Load old scene # heres the miracle
    peek at each object in list w RGB/depth and log location
    Load new scene
    peek each logged location w RGB/Depth
    Do image processing:
        subtract old depth from new depth image
        count nonblack pixels
        composite before/after/depth to output image
        write text on image
        save image
    log results
"""
