import argparse
import json
import glob
import os.path
import csv
import gc
import magnum as mn
import numpy as np
from habitat.sims.habitat_simulator.debug_visualizer import (
    DebugObservation,
    DebugVisualizer,
)
from PIL import Image, ImageChops, ImageDraw, ImageFont
from PIL.Image import Image as ImageClass
import cv2

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
FAIL_THRESHOLD = 1
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
    sim_settings = default_sim_settings.copy()
    sim_settings["width"] = sim_settings["height"] = IMAGE_SIZE
    #debugObservation only does RGB, skipping for now
    #sim_settings["depth_sensor"] = True
    #sim_settings["color_sensor"] = False
    #sim_settings["requires_textures"] = True
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


def switch_datasets(target: str) -> bool:
    if target == "new":
        target_dataset = dataset_new
    elif target == "old":
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


def observe_new_object(
    poi_bb: mn.Range3D, poi_transform: mn.Matrix4
) -> DebugObservation:
    """
    Takes a peek at a particular point in a scene, as logged from observe_reference_object()
    """
    observation = dbv.peek(poi_bb, peek_all_axis=True, subject_transform=poi_transform)
    return observation

def make_friendly_image_name(observation:dict) -> str:
    '''Make a human-friendly output basename retaining meaningful info:
    [scene id]_[first 6 of UUID]_[2-digit scene instance]
    '''
    short_uuid = observation["handle"].split(":")[0][0:6]
    instance = observation["handle"].split(":")[1][-2:]
    name_str = f"{observation['scene_id']}_{short_uuid}_{instance}"
    return name_str

def analyze_observations(observation:dict
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
    ref_img = observation["ref_img"]
    ref_img_cv2 = PIL_to_cv2(ref_img)
    decomp_img = observation["decomp_img"]
    decomp_img_cv2 = PIL_to_cv2(decomp_img)

    ### Image analysis ###

    # Absolute difference score
    res = cv2.absdiff(ref_img_cv2, decomp_img_cv2)
    res = res.astype(np.uint8)
    res_nz = np.count_nonzero(res)
    nz_pct = round((res_nz * 100) / res.size, 2)

    # Make difference image
    diff_img_PIL = ImageChops.invert(
        ImageChops.difference(ref_img, decomp_img).convert("RGB")
    )

    # Build diagnostic image array
    img_list = [ref_img, decomp_img, diff_img_PIL]
    diag_w = decomp_img.size[0]
    diag_h = decomp_img.size[1]
    diag_img = Image.new("RGB", (diag_w, diag_h * 3))
    paste_row = 0
    for i in range(3):
        diag_img.paste(img_list[i], (0, paste_row))
        paste_row += diag_h

    # write the UUID and variance on the image
    caption = f"{make_friendly_image_name(observation)}\n{nz_pct}%"
    if nz_pct < FAIL_THRESHOLD:
        fill_color = (0,0,0)
    else:
        fill_color = (128,0,0)
    draw_context = ImageDraw.Draw(diag_img)
    draw_font = ImageFont.truetype("data/fonts/ProggyClean.ttf", 36)
    draw_context.text(
        (64, (2 * diag_h + 64)),
        caption,
        font=draw_font,
        fill=fill_color,
    )

    return (diag_img, nz_pct)



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
        default="/home/jseagull/dev/fphab/diag",
        help="output folder for images and CSV",
        action="store",
    )
    args = argparser.parse_args()
    dataset_old = args.dataset
    dataset_new = dataset_old.split(".")[0] + "_v2.scene_dataset_config.json"

    # start logging
    os.environ["MAGNUM_LOG"] = "quiet"
    os.environ['HABITAT_SIM_LOG']='quiet'

    assert os.path.exists(args.output_path)
    log_data = []
    log_path = f"{args.output_path}/decomp_scene_check.csv"
    with open(log_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["scene", "base_uuid", "instance","variance"])

    if not args.batch:
        scene_list = [args.scene]
    else:
        scene_list = [
            (os.path.basename(scene)).split(".")[0]
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
        # snap 'before' images
        cfg = switch_datasets("old")
        observation_list = []
        with Simulator(cfg) as sim:
            dbv = DebugVisualizer(sim, resolution=(IMAGE_SIZE, IMAGE_SIZE))
            rom = sim.get_rigid_object_manager()
            handles = []
            for object_uuid in replaced_uuids:
                handles.extend(rom.get_object_handles(search_str=object_uuid))
            for handle in handles:
                ref_object = rom.get_object_by_handle(handle)
                if ref_object is not None:
                    ref_obs, poi_bb, poi_transform = observe_reference_object(
                        ref_object
                    )
                    ref_img = ref_obs.get_image().convert("RGB")
                    observation_list.append(
                        {
                            "scene_id" : scene_id,
                            "poi_bb": poi_bb,
                            "poi_transform": poi_transform,
                            "handle": handle,
                            "ref_img": ref_img,
                        }
                    )
                    # out_path = make_friendly_image_name(scene_id,handle,"old")
                    # ref_img.save(out_path)
                    # dbug
                    # print(f"output path: {out_path}")
                    # ref_img.show()
                else:
                    print(f"ERROR - couldn't get object for handle {handle}")
            sim.close()
            gc.collect()
        #now snap the updated scene
        cfg = switch_datasets("new")
        with Simulator(cfg) as sim:
            dbv = DebugVisualizer(sim, resolution=(IMAGE_SIZE, IMAGE_SIZE))
            rom = sim.get_rigid_object_manager()
            for decomp in observation_list:
                decomp_obs = dbv.peek(
                    decomp["poi_bb"], peek_all_axis=True, subject_transform=decomp["poi_transform"]
                )
                decomp_img = decomp_obs.get_image().convert("RGB")
                decomp["decomp_img"] = decomp_img
                #out_path = make_friendly_image_name(scene_id,decomp["handle"],"")
                #decomp_img.save(out_path)
                # dbug
                #ref_img.show()
            sim.close()
            gc.collect()
        #now we do image processing on the list of dicts and log results to CSV
        for observation in observation_list:
            composite_image, diff_pct = analyze_observations(observation)
            with open(log_path, "a", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                log_handle = observation["handle"].split("_")[0]
                log_instance = observation["handle"].split(":")[1]
                writer.writerow([observation["scene_id"], log_handle, log_instance, diff_pct])
            if diff_pct < FAIL_THRESHOLD:
                print(f"OK")
                destination_subfolder = "pass"
            else:
                print(f"{COLOR_WARNING}CHECK ({diff_pct}% variance){COLOR_RESET}")
                destination_subfolder = "fail"
            try:
                destination_file = make_friendly_image_name(observation)
                destination_path = os.path.join(IMAGE_OUTPUT_PATH_ROOT, destination_subfolder,(destination_file + ".png"))
                composite_image.save(destination_path)
            except IOError:
                print(f"{COLOR_WARNING}Can't save image for{COLOR_RESET} {object_uuid}!")
