#!/usr/bin/env python

# glb_mesh_analysis : This program provides tools for analyzing
# glb-based meshes for constituent object stats

from glob import glob

import glb_mesh_tools as gut
import numpy as np

# Scene glbs dir, where scenes to analyze reside
STAT_SCENE_GLBS_DIR = "../../data/datasets/replicaCAD_extra_scenes/"

# Ignore objects within queried scenes that have these substrings in their names
IGNORE_OBJECT_NAMES = ["door", "frl_apartment_lower_shelf_01_part_01"]

# Build a dictionary containing all scene object statatistics
def calc_object_stats(obj_per_scene_transforms, obj_max_per_scene_counts):
    all_obj_location_stats = {}

    for obj, scene_dict in obj_per_scene_transforms.items():
        # This really ought to be solved using a Bipartite perfect matching
        if obj_max_per_scene_counts[obj] == 1:
            location = []
            # easy stats, just average location of all scenes present in
            for transforms in scene_dict.values():
                # will always be a length 1 list
                translations = gut.get_trans_list_from_transforms(transforms)
                location.append(translations[0])
            obj_location_stats = gut.calc_stats(location)
            all_obj_location_stats[obj] = {
                "number_of_scenes": len(scene_dict),
                "max_per_scene": obj_max_per_scene_counts[obj],
                "object_stats": [obj_location_stats],
            }
            print(
                f"Object : {obj} : # of scenes present : {len(scene_dict)} max per scene : {obj_max_per_scene_counts[obj]} "
                f"mean location : { obj_location_stats['mean']} location std : {obj_location_stats['std']}"
            )
        else:
            # harder stats, need to do clustering to build correct
            # correspondence between instances across scenes
            # First, take every scene individually, check each object copy to
            # see where that copy belongs in particular clustering.
            # every entry has at most 2 representations per scene
            # find a scene with 2 objects
            saved_scene = ""
            for scene, transforms in scene_dict.items():
                if len(transforms) == 2:
                    saved_scene = scene
                    break
            # use the two entries in saved_scene as exemplars
            base_translations = gut.get_trans_list_from_transforms(
                scene_dict[saved_scene]
            )
            base_obj_0_loc = base_translations[0]
            obj_0_location = []
            obj_0_location.append(base_obj_0_loc)
            base_obj_1_loc = base_translations[1]
            obj_1_location = []
            obj_1_location.append(base_obj_1_loc)

            for scene, transforms in scene_dict.items():
                if scene == saved_scene:
                    continue
                translations = gut.get_trans_list_from_transforms(transforms)
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
                        f"Object {obj} in scene {scene} Weird result {len(translations)} "
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
                f"\nObject : {obj} : # of scenes present : {len(scene_dict)}  | max per scene : {obj_max_per_scene_counts[obj]} | "
                f"Counts : {[obj_0_stats['num_entries'], obj_1_stats['num_entries']]} | "
                f"mean locations : {[obj_0_stats['mean'], obj_1_stats['mean']]} location stds : { [obj_0_stats['std'], obj_1_stats['std']]}\n"
            )

    return all_obj_location_stats


def write_results_csv(filename, all_object_stats):
    import csv

    header_list = [
        "object name",
        "number of scenes",
        "max number per scene",
        "obj 0 count",
        "obj 0 mean location",
        "obj 0 location std",
        "obj 1 count",
        "obj 1 mean location",
        "obj 1 location std",
    ]
    with open(filename, "w", newline="") as csvfile:
        writer = csv.writer(csvfile, delimiter=",")
        writer.writerow(header_list)
        for obj, entry in all_object_stats.items():
            write_list = [obj, entry["number_of_scenes"], entry["max_per_scene"]]
            for objstats in entry["object_stats"]:
                write_list.append(objstats["num_entries"])
                write_list.append(objstats["mean"])
                write_list.append(objstats["std"])
            writer.writerow(write_list)


def main():
    # build list of all new scenes
    new_scene_glbs = glob(STAT_SCENE_GLBS_DIR + "*.glb")
    # find all stats for object locations in each scene
    obj_per_scene_transforms, obj_max_per_scene_counts = gut.get_objects_in_scenes(
        new_scene_glbs, IGNORE_OBJECT_NAMES
    )
    # calculate per-object stats
    all_object_stats = calc_object_stats(
        obj_per_scene_transforms, obj_max_per_scene_counts
    )
    # save resuts
    write_results_csv("temp_res.csv", all_object_stats)
    # print(all_object_stats, sep='\n')


if __name__ == "__main__":
    main()
