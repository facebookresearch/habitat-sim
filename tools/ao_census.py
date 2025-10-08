"""
AO census
scan /articulated scenes for articulated object uuids and check scale 
"""

import os
import json
import csv
from typing import Optional

# Change to /scenes-articulated_v2 and re-run
SCENE_DIR = "/home/jseagull/dev/fphab/scenes-articulated"
AO_PATH = "/home/jseagull/dev/fphab/urdf"
MAX_FILES_TO_PROCESS = 9999

def collect_scenes(dir: str) -> list:
    file_list = []
    for entry in os.listdir(dir):
        full_path = os.path.join(dir, entry)
        if os.path.isfile(full_path) and entry.endswith(".scene_instance.json"):
            file_list.append(full_path)
    return file_list[:MAX_FILES_TO_PROCESS]

def eval_object(obj:dict, obj_type:str, ao_replacements:int) -> Optional[list]:
    if obj["template_name"] in ao_uuids:
        scale_ok = True
        scale = obj.get("non_uniform_scale")
        if scale is not None:
            if any(.99 <= term <= 1.01 for term in scale):
                ao_replacements += 1
                scale_ok = False
            scale_x = scale[0]
            scale_y = scale[1]
            scale_z = scale[2]
        else:
            scale_x = scale_y = scale_z = 1.0
        return [scene,obj["template_name"],obj_type,scale_x,scale_y,scale_z, scale_ok]
    else:
        return None

def scan_scene_for_objects(scene: str) -> list:
    """
    Given a scene instance name, look at the objects and check against the set of AO UUIDs
    Returns:[scene_id, uuid, scale_x,scale_y,scale_z]
    """
    global total_objects
    global total_replacements
    ao_objects = []
    replaceable_rigid_objects = 0
    ao_replacements = 0
    with open(scene, "r") as file:
        scene_instance = json.load(file)

    for obj in scene_instance["object_instances"]:
        result = eval_object(obj,"rigid",ao_replacements)
        if result is not None:
            replaceable_rigid_objects += 1
            ao_objects.append(result)

    for obj in scene_instance["articulated_object_instances"]:
        result = eval_object(obj,"artic",ao_replacements)
        if result is not None:
            ao_objects.append(result)

    if replaceable_rigid_objects>0:
        print(
            f"{scene} - {replaceable_rigid_objects} rigid to replace, bad scale in {ao_replacements} existing AOs."
        )
    total_objects += replaceable_rigid_objects
    total_replacements += ao_replacements
    return ao_objects


# main
total_objects = 0  # these are declared global in scan_scene_for_objects
total_replacements = 0

scene_list = collect_scenes(SCENE_DIR)
ao_uuids = os.listdir(AO_PATH)
all_ao = []
for scene in scene_list:
    scene_ao = scan_scene_for_objects(scene)
    all_ao.extend(scene_ao)
#write log
output_path = "/home/jseagull/dev/fphab/ao_scale_census.csv"
header_row = ["scene","uuid", "scale_x", "scale_y", "scale_z","scale_ok"]
with open(output_path, "w", newline="", encoding="utf-8") as f:
    writer = csv.writer(f)
    writer.writerow(header_row)
    writer.writerows(all_ao)
