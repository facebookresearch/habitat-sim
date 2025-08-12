from habitat_sim import Simulator
from habitat_sim.utils import viz_utils as vut
from habitat_sim.utils.settings import default_sim_settings, make_cfg
from habitat.sims.habitat_simulator.debug_visualizer import DebugVisualizer
from habitat_sim.metadata import MetadataMediator

import magnum as mn # need for aabb datatype
import argparse

import cv2
from PIL import Image
import numpy as np

import json

def PIL_to_cv2(input_img: Image) -> np.array:
    '''Converts an 8 bpc PIL RGB image to a BGR numpy array for processing with OpenCV'''
    np_arr = np.array(input_img).astype(np.uint8)
    output_arr = np.zeros(np.shape(np_arr), dtype=np.uint8)
    output_arr = cv2.cvtColor(np_arr, cv2.COLOR_RGB2BGR)
    return output_arr

def cv2_to_PIL(input_arr: np.array) -> Image:
    '''Converts a BGR numpy array to an 8bpc RGB PIL Image'''
    input_arr = cv2.cvtColor(input_arr, cv2.COLOR_BGR2RGB)
    return Image.fromarray(input_arr)

dataset_paths = ["/home/jseagull/dev/habitat-sim/data/fp-models/fp-models.scene_dataset_config.json", #0 full path
                "data/fp-models/fp-models.scene_dataset_config.json", #1 relative path
                "../fp-models/fp-models.scene_dataset_config.json", #2 relative path without symlink
                "fp-models/fp-models.scene_dataset_config.json", #3 path within /data
                "./data/fp-models/fp-models.scene_dataset_config.json", #4 absolute path from current directory
                "data/fp-models/", #5 location without leaf
                "./data/fp-models/", #6 location without leaf, absolute path
                "data/fp-models/fp-models", #7 leaf without extension ### THIS ENABLES OBJECT TO LOAD, but active dataset is default and stage cannot load
                "fp-models", #8 leaf without extension, relative path ### Dataset and model load but scene doesnt
                "data/fp-models" # 9 no trailing slash
                ]

scene = "grid" # from-scratch scene I made with scale marks
image_size = 1024

argparser = argparse.ArgumentParser()
argparser.add_argument(
    "-o", "--object", type=str, default="00a2b0f3886ccb5ffddac704f8eeec324a5e14c6",
    help="the UUID of the decomposed object to load, default 00a2b0f3886ccb5ffddac704f8eeec324a5e14c6",
    action="store")
argparser.add_argument(
    "-d", "--dataset", type=str, default=dataset_paths[8],
    help="the dataset to use, default fp-models",
    action="store")
args = argparser.parse_args()

object_uuid = args.object
decomp_glb_path = f"data/fp-models/objects/decomposed/{object_uuid}/{object_uuid}_dedup.glb"
parts_metadata_path = f"data/fp-models/objects/decomposed/{object_uuid}/{object_uuid}.parts_metadata.json"

sim_settings = default_sim_settings.copy()
sim_settings["width"] = sim_settings["height"] = image_size
sim_settings["enable_hbao"] = True #Ambient Occlusion
sim_settings["default_agent_navmesh"] = False
sim_settings["scene_dataset_config_file"] = args.dataset #sim and mm both need to know about the dataset
# sim_settings["scene"] = scene

cfg = make_cfg(sim_settings)
mm = MetadataMediator()
mm.active_dataset = args.dataset
cfg.metadata_mediator = mm

mm_oam = mm.object_template_manager
mm_sam = mm.stage_template_manager

print("\n########################################################\n")
print(f"active dataset: {mm.active_dataset}")
print(f"Dataset exists? {mm.dataset_exists(args.dataset)}\n")
# print(f"Dataset Report: {mm.dataset_report(dataset)}")
#print(f"Scene handles: {mm.get_scene_handles()}")

my_template = mm_oam.create_new_template(handle=decomp_glb_path)
my_template_id = mm_oam.register_template(template=my_template, specified_handle="decomp_object")
if (my_template.has_value('render_asset')):
    print(f"Render asset created for {my_template.file_directory}")
else:
    print("ERROR - Couldn't create render asset")

#print(f"Keys in my_template: {my_template.get_keys_and_types()}")
#print(f"Render Asset full path location: {my_template.find_value_location('render_asset_fullpath')}")
#print(f"Render Asset path: {my_template.render_asset_fullpath}")
#my_template.render_asset = object_leaf

print("\n######## Starting Simulator... #########################\n")    

with Simulator(cfg) as sim:
    dbv = DebugVisualizer(sim,resolution=(1024,1024))
    scene_rom = sim.get_rigid_object_manager()

    # load decomp object
    poi = scene_rom.add_object_by_template_handle(object_lib_handle="decomp_object")
    poi_bb = poi.aabb
    poi_transform = poi.transformation

    #make a 3x2 contact sheet of decomp object
    decomp_obs = dbv.peek(poi, peek_all_axis=True)
    decomp_img_PIL = decomp_obs.get_image().convert('RGB')
    decomp_img_cv2 = PIL_to_cv2(decomp_img_PIL)

    #unload decomp object
    scene_rom.remove_object_by_handle(handle = poi.handle)

    #load parts from JSON

    #make a 3x2 of assembled parts
    parts_obs = dbv.peek(poi_bb, peek_all_axis=True, subject_transform = poi_transform)
    parts_img_PIL = parts_obs.get_image().convert('RGB')
    parts_img_cv2 = PIL_to_cv2(parts_img_PIL)
    
    #Image analysis
    #diff image
    #diff_img_PIL = ImageChops.difference(decomp_img_PIL, parts_img_PIL).convert('RGB')
    #diff_img_cv2 = cv2.subtract(decomp_img_cv2,parts_img_cv2)
    diff_img_cv2 = cv2.threshold(
        cv2.cvtColor(
            cv2.subtract(decomp_img_cv2,parts_img_cv2), 
            cv2.COLOR_BGR2GRAY), 
            0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)[1]
    diff_img_PIL = cv2_to_PIL(diff_img_cv2)

    #make diag image array
    img_list = [decomp_img_PIL, parts_img_PIL, diff_img_PIL]
    diag_w = parts_img_PIL.size[0]
    diag_h = parts_img_PIL.size[1]
    diag_img = Image.new('RGB',(diag_w, diag_h*3))
    paste_row = 0
    for i in range(3):
        diag_img.paste(img_list[i],(0,paste_row))
        paste_row += diag_h


    #dbug
    # decomp_obs.show()
    # parts_obs.show()
    # diff_img_PIL.show()
    diag_img.show()

    #Absolute difference
    res = cv2.absdiff(decomp_img_cv2, parts_img_cv2)
    res = res.astype(np.uint8)
    res_nz = np.count_nonzero(res)
    nz_pct = round((res_nz*100)/res.size ,3)
    print(f"{nz_pct}% image variance ({res_nz} pixels)")