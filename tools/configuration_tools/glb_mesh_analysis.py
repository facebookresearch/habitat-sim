#!/usr/bin/env python

# glb_mesh_analysis : This program provides tools for analyzing
# glb-based meshes for constituent object stats, using the trimesh library

from collections import defaultdict
from glob import glob

import glb_mesh_tools as gut
import numpy as np

# new scene instance glbs dir, where files to analyze reside
STAT_SCENE_GLBS_DIR = "../../data/datasets/replicaCAD_extra_scenes/"

# This function will take a .glb file and find the objects contained inside
def get_new_scene_info():
    # build list of all new scenes
    new_scene_glbs = glob(STAT_SCENE_GLBS_DIR + "*.glb")
    each_scene_obj_dict = {}
    for new_scene_filename in new_scene_glbs:
        # returns a default dict with key == object name and value is a list of transformations
        each_scene_obj_dict[new_scene_filename] = gut.process_scene_for_objects(
            new_scene_filename
        )

    all_scene_translations_dict = defaultdict(list)
    # per object scene population
    obj_per_scene_dict = defaultdict(dict)
    # per object max counts
    obj_max_per_scene_counts = defaultdict(int)
    for scene_name, obj_in_scene_dict in each_scene_obj_dict.items():
        # obj_in_scene_dict is a scene's default dict
        # print("\n{}\nObjects and counts : ".format(scene_name))
        for obj, transforms in obj_in_scene_dict.items():
            translations = gut.get_trans_list_from_transforms(transforms)
            # print("{} | {}".format(obj, translations))
            all_scene_translations_dict[obj].extend(translations)
            obj_per_scene_dict[obj][scene_name] = translations
            if obj_max_per_scene_counts[obj] < len(translations):
                obj_max_per_scene_counts[obj] = len(translations)
    print("\nTotals:\n")
    for obj, translations in all_scene_translations_dict.items():
        print("{} | {}  ".format(obj, len(translations)))
    print("\n")

    all_obj_location_stats = {}
    for obj, scene_dict in obj_per_scene_dict.items():
        if "door" in obj or "frl_apartment_lower_shelf_01_part_01" in obj:
            continue

        if obj_max_per_scene_counts[obj] == 1:
            location = []
            # easy stats, just average location of all scenes present in
            for _, translations in scene_dict.items():
                location.append(translations[0])
            obj_location_stats = gut.calc_stats(location)
            all_obj_location_stats[obj] = {
                "number_of_scenes": len(scene_dict),
                "max_per_scene": obj_max_per_scene_counts[obj],
                "object_stats": [obj_location_stats],
            }
            print(
                "Object : {} : # of scenes present : {} max per scene : {} mean location : {} location std : {}".format(
                    obj,
                    len(scene_dict),
                    obj_max_per_scene_counts[obj],
                    obj_location_stats["mean"],
                    obj_location_stats["std"],
                )
            )
        else:
            # harder stats, need to do clustering
            # take every scene individually, check each object copy to see where that copy belongs in particular clustering.
            # every entry has at most 2 representations per scene
            # find a scene with 2 objects
            saved_scene = ""
            for scene, translations in scene_dict.items():
                if len(translations) == 2:
                    saved_scene = scene
                    break
            # use the two entries in saved_scene as exemplars
            base_obj_0_loc = scene_dict[saved_scene][0]
            obj_0_location = []
            obj_0_location.append(base_obj_0_loc)
            base_obj_1_loc = scene_dict[saved_scene][1]
            obj_1_location = []
            obj_1_location.append(base_obj_1_loc)

            for scene, translations in scene_dict.items():
                if scene == saved_scene:
                    continue
                if len(translations) == 1:
                    # find closest of 2 entries in saved_scene
                    obj_loc = translations[0]
                    dist_0 = np.linalg.norm(obj_loc - base_obj_0_loc)
                    dist_1 = np.linalg.norm(obj_loc - base_obj_1_loc)
                    if dist_0 > dist_1:
                        obj_1_location.append(obj_loc)
                    else:
                        obj_0_location.append(obj_loc)
                elif len(translations) == 2:
                    # find best mapping between these 2 objects and the root 2 objects
                    obj_A_loc = translations[0]
                    obj_B_loc = translations[1]
                    dist_0A = np.linalg.norm(obj_A_loc - base_obj_0_loc)
                    dist_1A = np.linalg.norm(obj_A_loc - base_obj_1_loc)
                    dist_0B = np.linalg.norm(obj_B_loc - base_obj_0_loc)
                    dist_1B = np.linalg.norm(obj_B_loc - base_obj_1_loc)
                    if (dist_0A + dist_1B) > (dist_1A + dist_0B):
                        # A->1 and B-> 0 is closest mapping
                        obj_0_location.append(obj_B_loc)
                        obj_1_location.append(obj_A_loc)
                    else:
                        # A->0 and B->1 is closest mapping
                        obj_0_location.append(obj_A_loc)
                        obj_1_location.append(obj_B_loc)
                else:
                    # doesn't happen
                    print(
                        "Object {} in scene {} Weird result {} ".format(
                            obj, scene, len(translations)
                        )
                    )
                    continue
            obj_0_stats = gut.calc_stats(obj_0_location)
            obj_1_stats = gut.calc_stats(obj_1_location)
            all_obj_location_stats[obj] = {
                "number_of_scenes": len(scene_dict),
                "max_per_scene": obj_max_per_scene_counts[obj],
                "object_stats": [obj_0_stats, obj_1_stats],
            }

            print(
                "\nObject : {} : # of scenes present : {}  | max per scene : {} | Counts : {} | mean locations : {} location stds : {}\n".format(
                    obj,
                    len(scene_dict),
                    obj_max_per_scene_counts[obj],
                    [obj_0_stats["num_entries"], obj_1_stats["num_entries"]],
                    [obj_0_stats["mean"], obj_1_stats["mean"]],
                    [obj_0_stats["std"], obj_1_stats["std"]],
                )
            )

    return all_obj_location_stats


def write_results_csv(filename, all_object_stats):
    import csv

    header_string = "object name, number of scenes, max number per scene, obj 0 count, obj 0 mean location, obj 0 location std, obj 1 count, obj 1 mean location, obj 1 location std".split(
        ","
    )
    with open(filename, "w", newline="") as csvfile:
        writer = csv.writer(csvfile, delimiter=",")
        writer.writerow(header_string)
        for obj, entry in all_object_stats.items():
            write_list = [obj, entry["number_of_scenes"], entry["max_per_scene"]]
            for objstats in entry["object_stats"]:
                write_list.append(objstats["num_entries"])
                write_list.append(objstats["mean"])
                write_list.append(objstats["std"])
            writer.writerow(write_list)


def main():
    # find all stats for object locations in each scene
    all_object_stats = get_new_scene_info()
    write_results_csv("temp_res.csv", all_object_stats)
    # print(all_object_stats, sep='\n')


if __name__ == "__main__":
    main()
