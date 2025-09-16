import json
import os
import re

'''Recursively searches a folder for GLB files with the string '_part_#' in the filename, 
and creates an identity .object_config.json file for it so Habitat can load the object.
'''
def build_object_config(model:str)-> dict:
    object_config_dict = {
        "render_asset" : model,
        "up" : [0,1,0],
        "front" : [0,0,-1],
    } 
    return object_config_dict

def process_directory(dir:str)-> None:
    config_ext = ".object_config.json"
    for file in os.listdir(dir):
        if re.search(r"_part_\d*\.glb$",file):
            print(file)
            object_config_file = os.path.splitext(file)[0] + config_ext
            object_config_path = os.path.join(dir,object_config_file)
            print(f"config file: {object_config_path}")
            if os.path.exists(object_config_path):
                print(f"Config exists, skipping {file}")
            else:
                config_data = build_object_config(file)
                print(json.dumps(config_data, indent=4))
                print(f"saving to {object_config_path}...")
                with open(object_config_path, 'w', encoding='utf-8') as f:
                    json.dump(config_data, f, indent=4)
        else:
            print(f"regex miss, skipping {file}...")

#test data
#process_directory("/home/jseagull/dev/hssd-hab/objects/decomposed/4ee147972ecbe17397b25f900518b00646a9de98/") # no configs present
#process_directory("/home/jseagull/dev/hssd-hab/objects/decomposed/5af7796800bd2a6c6033146badb377fb217d8873/") # all configs present

subfolders = [f.path for f in os.scandir('/home/jseagull/dev/fphab/objects/decomposed') if f.is_dir()]
for f in subfolders:
    process_directory(f)