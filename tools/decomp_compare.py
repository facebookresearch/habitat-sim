from habitat_sim import Simulator
from habitat_sim.physics import ManagedRigidObject
# from habitat_sim.utils import viz_utils as vut
from habitat_sim.utils.settings import default_sim_settings, make_cfg
from habitat.sims.habitat_simulator.debug_visualizer import (
    DebugVisualizer,
    DebugObservation,
)
from habitat_sim.metadata import MetadataMediator

import magnum as mn
import argparse

import cv2
from PIL import Image
import numpy as np

import json


def PIL_to_cv2(input_img: Image) -> np.array:
    """Converts an 8 bpc PIL RGB image to a BGR numpy array for processing with OpenCV"""
    np_arr = np.array(input_img).astype(np.uint8)
    output_arr = np.zeros(np.shape(np_arr), dtype=np.uint8)
    output_arr = cv2.cvtColor(np_arr, cv2.COLOR_RGB2BGR)
    return output_arr


def cv2_to_PIL(input_arr: np.array) -> Image:
    """Converts a BGR numpy array to an 8bpc RGB PIL Image"""
    input_arr = cv2.cvtColor(input_arr, cv2.COLOR_BGR2RGB)
    return Image.fromarray(input_arr)


def register_glb(leaf: str) -> None:
    """
    Registers a GLB file to a template in the library so it can be added to the scene.
    The registered GLB will have a library handle matching its GLB filename without extension
    """
    asset_path = (
        f"data/fp-models/objects/decomposed/{object_uuid}/{object_uuid}{leaf}.glb"
    )
    library_handle = f"{object_uuid}{leaf}"
    glb_template = mm_oam.create_new_template(handle=asset_path)
    assert glb_template.has_value("render_asset")
    # print(f"Render template created for {glb_template.file_directory}")
    template_id = mm_oam.register_template(
        template=glb_template, specified_handle=library_handle
    )
    print(f"Registered {leaf} as {glb_template.handle} with library ID {template_id}")


def observe_dedup_object() -> tuple[DebugObservation, mn.Range3D, mn.Matrix4]:
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
    # snippets
    # obj_xform = np.array(obj_data.get("transform", {}).get("matrix", None))
    # obj_xform = np.reshape(obj_xform, (1, 4, 4))
    # obj_xform = np.swapaxes(obj_xform, 1, 2)
    # obj_xform = mn.Matrix4(obj_xform[0])
    # obj_rot = obj_data.get("transform",{}).get("quaternion",None)
    # obj_scale = obj_data.get("transform",{}).get("scale",None)
    # scene_obj.translate(scene_obj.com_correction)
    obj_pos = obj_data.get("transform", {}).get("position", None)
    scene_obj.translate(mn.Vector3(obj_pos))
    #print(f"final location of pivot: {scene_obj.translation}")


def observe_composed_object(poi_bb, poi_transform) -> DebugObservation:
    """
    Loads and places all components of the decomposed object, takes a peek, and unloads them
    """
    dbug = False
    for obj_data in data.get("parts", {}):
        # dbug
        part_Id = obj_data.get("partId", "")
        if dbug:
            if part_Id != "19":
                continue
        render_mesh_Id = part_Id
        if obj_data.get("isRef") is False:  # Is a child instance
            render_mesh_Id = obj_data.get("refPartId", "")

        scene_obj = rom.add_object_by_template_handle(
            object_lib_handle=f"{object_uuid}_part_{render_mesh_Id}"
        )
        print(f"Added part {part_Id} (referencing mesh {render_mesh_Id})")

        place_object(scene_obj, obj_data)

    observation = dbv.peek(poi_bb, peek_all_axis=True, subject_transform=poi_transform)
    rom.remove_all_objects()
    return observation


dataset_paths = [
    "/home/jseagull/dev/habitat-sim/data/fp-models/fp-models.scene_dataset_config.json",  # 0 full path
    "data/fp-models/fp-models.scene_dataset_config.json",  # 1 relative path
    "../fp-models/fp-models.scene_dataset_config.json",  # 2 relative path without symlink
    "fp-models/fp-models.scene_dataset_config.json",  # 3 path within /data
    "./data/fp-models/fp-models.scene_dataset_config.json",  # 4 absolute path from current directory
    "data/fp-models/",  # 5 location without leaf
    "./data/fp-models/",  # 6 location without leaf, absolute path
    "data/fp-models/fp-models",  # 7 leaf without extension ### THIS ENABLES OBJECT TO LOAD, but active dataset is default and stage cannot load
    "fp-models",  # 8 leaf without extension, relative path ### Dataset and model load but scene doesnt
    "data/fp-models",  # 9 no trailing slash
]

# scene = "grid" # from-scratch scene I made with scale marks
image_size = 1024

argparser = argparse.ArgumentParser()
argparser.add_argument(
    "-o",
    "--object",
    type=str,
    default="00a2b0f3886ccb5ffddac704f8eeec324a5e14c6", # bookshelf
    #default="a62fd2ce7682a979db3b7b0d4cd541ae15ea6db7", #table with 5 rotated chairs
    help="the UUID of the decomposed object to load, default 00a2b0f3886ccb5ffddac704f8eeec324a5e14c6",
    action="store",
)
argparser.add_argument(
    "-d",
    "--dataset",
    type=str,
    default=dataset_paths[8],
    help="the dataset to use, default fp-models",
    action="store",
)
args = argparser.parse_args()

object_uuid = args.object
dedup_glb_path = (
    f"data/fp-models/objects/decomposed/{object_uuid}/{object_uuid}_dedup.glb"
)
parts_metadata_path = (
    f"data/fp-models/objects/decomposed/{object_uuid}/{object_uuid}.parts.metadata.json"
)

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
mm_oam = mm.object_template_manager

print("\n######## Initializing Assets... ########################\n")
print(f"active dataset: {mm.active_dataset}")
print(f"Dataset exists? {mm.dataset_exists(args.dataset)}\n")

# register object templates
object_leaf_list = ["_dedup"]

with open(parts_metadata_path, "r") as file:
    data = json.load(file)

for obj_data in data.get("parts", {}):
    if obj_data.get("isRef") is not False:  # only register unique GLBs
        part_Id = obj_data.get("partId", "")
        object_leaf_list.append(f"_part_{part_Id}")

for obj_data in object_leaf_list:
    register_glb(obj_data)

print("\n######## Starting Simulator... #########################\n")


with Simulator(cfg) as sim:
    dbv = DebugVisualizer(sim, resolution=(1024, 1024))
    rom = sim.get_rigid_object_manager()

    # load and observe the reference object
    dedup_obs, poi_bb, poi_transform = observe_dedup_object()

    # load decomposed parts and take a picture
    parts_obs = observe_composed_object(poi_bb, poi_transform)

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
    nz_pct = round((res_nz * 100) / res.size, 3)
    print(f"{nz_pct}% image variance ({res_nz} pixels)")

    # Make difference image
    # diff_img_PIL = ImageChops.difference(dedup_img_PIL, parts_img_PIL).convert('RGB')
    # diff_img_cv2 = cv2.subtract(dedup_img_cv2,parts_img_cv2)
    diff_img_cv2 = cv2.threshold(
        cv2.cvtColor(cv2.subtract(dedup_img_cv2, parts_img_cv2), cv2.COLOR_BGR2GRAY),
        0,
        255,
        cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU,
    )[1]
    diff_img_PIL = cv2_to_PIL(diff_img_cv2)

    # Build diagnostic image array
    img_list = [dedup_img_PIL, parts_img_PIL, diff_img_PIL]
    diag_w = parts_img_PIL.size[0]
    diag_h = parts_img_PIL.size[1]
    diag_img = Image.new("RGB", (diag_w, diag_h * 3))
    paste_row = 0
    for i in range(3):
        diag_img.paste(img_list[i], (0, paste_row))
        paste_row += diag_h
    # dbug
    # dedup_obs.show()
    # parts_obs.show()
    # diff_img_PIL.show()
    diag_img.show()
