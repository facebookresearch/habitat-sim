#!/usr/bin/env python3

# This script will copy Appen annotated files into an existing HM3D dataset,
# along with making the necessary modifications to the semantic txt file to
# be compatible with Habitat sim.

import re
import shutil
from collections import defaultdict
from os.path import exists as os_exists
from os.path import isfile as os_isfile
from os.path import join as os_join
from os.path import sep as os_sep
from os.path import split as os_split
from typing import Dict, List

import config_utils as ut

#
# Destination where HM3D datasets reside.  Under this directory should be the
# various partition directories, and under each of these should be the
# per-scene directories for the scenes belonging to that partition,
# having the format
#   <XXXXX>-<YYYYYYYYYYY>, where
#     <XXXXX> represents a numeric value from 00000 -> 00999
#     <YYYYYYYYYYY> represents an alpha-numeric hash representing the scene

HM3D_DEST_DIR = "/home/john/Facebook/habitat-sim/data/scene_datasets/HM3D"

#
# Source directory for Appen annotations.  Under this directory should reside individual
# sub directories for each annotated scene, where each sub directory has the format
#   <XXXXX>-<YYYYYYYYYYY>.semantic/Output/
# and the Output sub directory contains at least 2 files that vary only by extension :
#   <YYYYYYYYYYY>.semantic.<glb|txt>, where
#     <XXXXX> represents a numeric value from 00000 -> 00999. This should be the numeric
#             tag for the particular scene, but it does not have to be.  5 0's will suffice
#             if the tag is unknown.
#     <YYYYYYYYYYY> represents an alpha-numeric hash representing the scene.
# The expected files and what they represent are
#   .txt is the Semantic Scene descriptor file, which holds the information mapping vertex
#        colors to labels and regions
#   .glb is the mesh file for the scene holding both vertex-color-based and texture-based
#        semantic annotations.

HM3D_ANNOTATION_SRC_DIR = "/home/john/Datasets In Progress/HM3D_Semantic/Appen_Scenes"
# HM3D_ANNOTATION_SRC_DIR = "/home/john/Datasets In Progress/HM3D_Semantic/Appen_29_scenes_redo_feb_14"
#
# Appen annotation source scene directory regex.
# This regex describes the format of the per-scene directories in the Appen work,
# and may change depending on whether the format we receive from Appen changes.
HM3D_ANNOTATION_SUBDIR_RE = r"(?i)[0-9]{5}-[a-z0-9]{11}\.semantic$"

#
# Whether or not to build annotation scene dataset configs.
BUILD_SD_CONFIGS = True

#
# Whether or not to save source file names in annotation file lists for individual partitions
SAVE_FILELISTS_WITH_SOURCE = True

#
# Perform color count on semantic colors and print bad results instead of modifying and moving assets.
COUNT_SEMANTIC_COLORS_PER_SCENE = False

#
# Prefix for config - leave empty string for none. Use this to build configs on a
# subset of scenes for testing without having to view all scenes
CONFIG_PREFIX = ""

##############################################################################
## You should not need to modify anything below here


# numeric spans of each data partition.  The directoriues
# holding the scene data are prefixed by numbers in these ranges
HM3D_DATA_PARTITIONS = {
    "minival": (800, 809),
    "test": (900, 999),
    "train": (0, 799),
    "val": (800, 899),
}

# sentinel string for semantic scene descriptor text files in HM3D
# DO NOT CHANGE.  This is looked for in Habitat-sim
HM3D_SSD_STRING = "HM3D Semantic Annotations"


# Validate source semantic txt files' format
# Each line should have the following format :
# 1,4035BC,"ceiling corridor",2
# positive int, hex color, string, positive int
def validate_src_SSD(line):
    lineItems = line.split(",")
    # each line should be 4 elements, but element 2 might contain one or more commas
    # so check if there are 4 items, or there are more
    if (len(lineItems) < 4) or (
        len(lineItems) > 4
        and lineItems[2].count('"') != 1
        and lineItems[-2].count('"') != 1
    ):
        return f"Incorrect # of items : should be 4, is {len(lineItems)}"
    if not lineItems[0].strip().isdigit():
        return f"First entry (unique ID) is not positive integer : {lineItems[0]}"
    if not re.compile("^[A-F0-9]{6}$").match(lineItems[1].strip()):
        return f"Second entry (color) is not 6 digit hex value : {lineItems[1]}"
    if len(lineItems[2].strip()) == 0:
        return "Third entry (category) cannot be an empty string"
    if not lineItems[-1].strip().isdigit():
        return f"Last entry (region) is not non-negative integer : {lineItems[-1]}"
    return "fine"


# Modify the given semantic text file to include the necessary sentinel string at the
# beginning that Habitat-Sim expects, and then save to appropriate dest directory
def modify_and_copy_SSD(src_filename: str, dest_filename: str):
    with open(dest_filename, "w") as dest, open(src_filename, "r") as src:
        dest.write(f"{HM3D_SSD_STRING}\n")
        i = 1
        printFileOffset = True
        fileIsOffset = False
        for line in src:
            valid_res = validate_src_SSD(line)
            if valid_res != "fine":
                print(
                    f"!!!! Error in source SSD `{src_filename}` on line {i} : {line} has format error : `{valid_res}`"
                )
            line_parts = line.split(",", 1)
            if int(line_parts[0].strip()) != i:
                fileIsOffset = True
                line = f"{i},{line_parts[-1]}"

            if fileIsOffset and printFileOffset:
                print(f"erroneous line corrected {i} : {line}", end="")
                fileIsOffset = False
                printFileOffset = False
            dest.write(f"{line}")
            i += 1


def buildFileListing():
    # go through annotation directory, find paths to all files of
    # interest (annotated glbs and semantic lexicon/text files)
    # and match to desired destination in HM3D install directory

    # directory name pattern to find annotation files
    annotation_dir_pattern = re.compile(HM3D_ANNOTATION_SUBDIR_RE)
    # directory listing
    dir_listing = ut.get_directories_matching_regex(
        HM3D_ANNOTATION_SRC_DIR, annotation_dir_pattern
    )
    # destination directory will be based on numeric field, if available
    file_names_and_paths = {}
    for src_dir, dirname_full in dir_listing:
        src_dirname_full = os_join(src_dir, dirname_full)
        file_names_and_paths[src_dirname_full] = []
        dirname = dirname_full.split(".semantic")[0]
        scene_name = dirname.split("-")[-1]
        # get directory and file names for both semantic src files
        src_file_list = ut.get_files_matching_regex(
            src_dirname_full, re.compile(scene_name + ".semantic.")
        )
        if len(src_file_list) != 2:
            print(
                f"Problem with source dir files {dirname_full} : Unable to find 2 source files "
                f"({len(src_file_list)} files instead) so skipping this source dir."
            )
            continue
        # find appropriate destination directory for given source scene
        dest_dir_list = ut.get_directories_matching_regex(
            HM3D_DEST_DIR, re.compile(".*" + scene_name + "$")
        )
        # Must find at least 1 dest. Might have 2 if val and minival
        if len(dest_dir_list) > 0:
            for dest_dir in dest_dir_list:
                scene_dest_dir = os_join(dest_dir[0], dest_dir[1])
                partition_subdir = dest_dir[0].split(os_sep)[-1]
                partition_tag = partition_subdir.split("-")[1].strip()
                scene_dest_subdir = os_join(partition_subdir, dest_dir[1])
                src_files = {}
                for src_full_dir, _, src_filename in src_file_list:
                    if ".txt" in src_filename:
                        key = "ssdfile"
                    else:
                        key = "glbfile"

                    # fully qualified source path and filename
                    src_files["src_path_" + key] = os_join(src_full_dir, src_filename)
                    # TODO perhaps we wish to rename the file in the destination? If so, do so here, instead of using src_filename
                    dest_filename = src_filename
                    # subdir and filename
                    src_files["dest_subdir_" + key] = os_join(
                        scene_dest_subdir, dest_filename
                    )
                    # fully qualified destination path/filename
                    src_files["dest_path_" + key] = os_join(
                        scene_dest_dir, dest_filename
                    )

                tmp_dict = {"dest_dir": scene_dest_dir}
                tmp_dict["dest_subdir"] = scene_dest_subdir
                # partition tag for destination file subdirectory
                tmp_dict["dest_part_tag"] = partition_tag
                for k, v in src_files.items():
                    tmp_dict[k] = v
                file_names_and_paths[src_dirname_full].append(tmp_dict)

        else:
            print(
                f"Problem with source dir {dirname_full} : Unable to find destination dir due to {len(dest_dir_list)} "
                f"matching destinations in current HM3D dataset so skipping this source dir.",
                end="",
            )
            for bad_dest in dest_dir_list:
                print(f"\t{bad_dest}", end="")

            continue

    return file_names_and_paths


# This will verify that the passed file exists.
def verify_file(filename: str, src_dir: str, type_key: str, failures: Dict):
    success = os_exists(filename) and os_isfile(filename)
    proc = "has"
    if not success:
        proc = "HAS NOT"
        if src_dir not in failures:
            failures[src_dir] = {}
        failures[src_dir][type_key] = filename
    print(
        f"\t\t{type_key} File {proc} been successfully modified and copied to {filename}"
    )
    return success


def build_annotation_configs(part_file_list_dict: Dict, output_files: List):
    print("build_annotation_configs:")

    def modify_paths_tag(
        paths: Dict, file_ext_key: str, path_glob_file: str, rel_file_dirs: List
    ):
        # replace existing list of paths in paths dict with rel_file_names list
        tmp_paths = []
        for file_path in rel_file_dirs:
            tmp_paths.append(os_join(file_path, path_glob_file))
        paths[file_ext_key] = tmp_paths

    # get the filenames of the 5 existing configs
    src_json_configs = ut.get_files_matching_regex(
        HM3D_DEST_DIR, re.compile("hm3d_.+\\.scene_dataset_config\\.json")
    )
    file_dirs_for_config = {}
    tmp_glbl_filedirs = set()
    # only want single representation for each scene
    for key in HM3D_DATA_PARTITIONS:
        tmp_file_dirs = set()
        for filename in part_file_list_dict[key]:
            path_no_filename = os_split(filename)[0] + os_sep
            tmp_file_dirs.add(path_no_filename.split("hm3d-*-habitat/")[-1])
            tmp_glbl_filedirs.add(path_no_filename)
        file_dirs_for_config[key] = sorted(tmp_file_dirs)

    file_dirs_for_config["base"] = sorted(tmp_glbl_filedirs)

    config_filenames = {}
    new_config_filenames = {}
    new_filepaths_for_config = {}
    # build new filenames for annotation configs
    if len(CONFIG_PREFIX) == 0:
        new_config_file_prefix = "hm3d_annotated_"
    else:
        new_config_file_prefix = f"hm3d_annotated_{CONFIG_PREFIX}"
    for config in src_json_configs:
        filename = os_join(config[0], config[-1])
        # want to overwrite old versions
        if "hm3d_annotated_" in filename:
            continue
        # success = os_exists(filename) and os_isfile(filename)
        json_filename = config[-1]
        config_filenames[json_filename] = filename
        new_config_filenames[json_filename] = filename.replace(
            "hm3d_", new_config_file_prefix
        )

        # set default path list
        new_filepaths_for_config[json_filename] = file_dirs_for_config["base"]
        for part, scene_path_list in file_dirs_for_config.items():
            if part in json_filename:
                new_filepaths_for_config[json_filename] = scene_path_list
                break

    for config_key, src_config_filename in config_filenames.items():
        dest_config_filename = new_config_filenames[config_key]
        scene_path_list = new_filepaths_for_config[config_key]
        if len(scene_path_list) == 0:
            continue
        print(
            f"{config_key} # files : {len(scene_path_list)} : \n\t{src_config_filename}\n\t{dest_config_filename}"
        )
        # print("\n\tfiles:")
        # for pathname in scene_path_list:
        #     print(f"\t\t{pathname}", end="")

        # load each existing json config, appropriately modify it, and then save as new configs
        if BUILD_SD_CONFIGS:
            src_json_config = ut.load_json_into_dict(src_config_filename)
            for key, json_obj in src_json_config.items():
                # Modify only those configs that hold paths
                if "paths" in json_obj:
                    # Modify both stages and scene instance configs
                    # this is the dictionary of lists of places to look for specified config type files
                    paths_dict = src_json_config[key]["paths"]
                    for path_type_key, lu_path_list in paths_dict.items():
                        # assume the list has at least one element
                        lu_glob_file = lu_path_list[0].split(os_sep)[-1]
                        modify_paths_tag(
                            paths_dict, path_type_key, lu_glob_file, scene_path_list
                        )
                # add tag/value in stages dflt attributes denoting the
                # annotation dataset supports texture semantics
                if "stages" in key:
                    dflt_attribs = src_json_config[key]["default_attributes"]
                    dflt_attribs["has_semantic_textures"] = True

            ut.save_json_to_file(src_json_config, dest_config_filename)

        # add subdirectory-qualified file paths to new configs to output_files list so that they will
        # be included in zip file
        rel_config_filename = dest_config_filename.split(HM3D_DEST_DIR)[-1].split(
            os_sep, 1
        )[-1]
        print(
            f"Adding rel_config_filename : {rel_config_filename} to output_files.",
            end="",
        )
        output_files.append(rel_config_filename)


def save_annotated_file_lists(output_files: List):
    print("save_annotated_file_lists:")
    # write text files that hold listings of appropriate relative filepaths for
    # annotated files as well as for each partition's configs for all scenes that
    # have annotations

    # single file listing holding all annotated file filenames (annotated glb and txt only)
    # and annotated config filenames
    if SAVE_FILELISTS_WITH_SOURCE:
        use_src_prfx = "_and_src"
    else:
        use_src_prfx = ""
    if len(CONFIG_PREFIX) == 0:
        new_file_list_prefix = f"HM3D_annotation{use_src_prfx}"
    else:
        new_file_list_prefix = f"HM3D_annotation{use_src_prfx}_{CONFIG_PREFIX}"

    with open(os_join(HM3D_DEST_DIR, f"{new_file_list_prefix}_files.txt"), "w") as dest:
        dest.write("\n".join(output_files))

    # save listing of all files (semantic, render, navmesh) for each partition:
    # each entry is a tuple with idx 0 == dest filename; idx 1 == list of paths
    partition_lists = {}
    for part in HM3D_DATA_PARTITIONS:
        partition_lists[part] = (
            os_join(HM3D_DEST_DIR, f"{new_file_list_prefix}_{part}_dataset.txt"),
            [],
        )
    # for each entry in output_files list, check which partition it belongs to
    # all partition files should have hm3d-<partition>-habitat as the lowest
    # relative directory
    for filename in output_files:
        fileparts = filename.split(os_sep)
        if len(fileparts) < 2:
            # filename is root level and not part of a partition
            continue
        file_part_key = fileparts[0].split("-")[1].strip()
        partition_lists[file_part_key][1].append(filename)
        # add source file filenames to filelisting if desired
        if SAVE_FILELISTS_WITH_SOURCE and ".semantic.glb" in filename:
            filename_base = filename.split(".semantic.glb")[0]
            partition_lists[file_part_key][1].append(f"{filename_base}.basis.glb")
            partition_lists[file_part_key][1].append(f"{filename_base}.basis.navmesh")
    # write each per-partition file listing to build per-partition annotated-only dataset archives
    # (holding semantic, render and navmesh assets)
    for _, v in partition_lists.items():
        if len(v[1]) == 0:
            continue
        with open(v[0], "w") as dest:
            dest.write("\n".join(v[1]))


def count_SSD_colors(ssd_filename):
    counts_dict = defaultdict(lambda: {"count": 0, "names": []})
    # count the # of occurrences of colors in each scene. SHOULD ONLY BE 1 PER COLOR!
    with open(ssd_filename, "r") as src:
        for line in src:
            items = line.split(",")
            color = items[1].strip()
            tag = line.split('"')[1].strip()
            name = f"{tag}_{items[0]}"
            counts_dict[color]["names"].append(name)
            if not re.compile("^[A-F0-9]{6}$").match(color):
                counts_dict[color]["count"] = -9999
            else:
                counts_dict[color]["count"] += 1

    return counts_dict


def main():
    # build dictionary of src and dest file names and paths
    file_names_and_paths = buildFileListing()
    # dictionary keyed by partition valued by list of partition subdir and filename of written file
    part_file_list_dict = {}
    for key in HM3D_DATA_PARTITIONS:
        part_file_list_dict[key] = []

    if COUNT_SEMANTIC_COLORS_PER_SCENE:
        # go through every semantic annotation file and perform a count of each color present.
        per_scene_counts = {}
        for _, data_dict_list in file_names_and_paths.items():
            for data_dict in data_dict_list:
                ssd_filename = data_dict["src_path_ssdfile"]
                print(f"{ssd_filename}", end="")
                tmp_dict = count_SSD_colors(ssd_filename)
                dict_key = ssd_filename.split(HM3D_ANNOTATION_SRC_DIR)[-1].split(
                    ".semantic/Output"
                )[0]
                per_scene_counts[dict_key] = tmp_dict
        print(f"\n\n{len(per_scene_counts)} items\n\n")
        # display results
        total_items = 0
        for scenename, count_dict in per_scene_counts.items():
            for color, count_and_names in count_dict.items():
                count = count_and_names["count"]
                total_items += count
                names = count_and_names["names"]
                if count > 1:
                    print(
                        f"!!! In scene {scenename} : Bad Color Count : '{color}' is present {count} times : {names}",
                        end="",
                    )
        print(f"Total Items :{total_items}", end="")
    else:
        # Failures here will be files that did not get copied (or modified if appropriate)
        failures = {}
        # fully qualified paths to all output files
        output_files = []
        # move semantic glbs and scene descriptor text files
        for src_dir, data_dict_list in file_names_and_paths.items():
            # print(f"Src : {src_dir} : # of data_dicts : {len(data_dict_list)} ", end="")
            for data_dict in data_dict_list:
                partition_tag = data_dict["dest_part_tag"]
                # print(
                #     f"\tDest subdir under HM3D directory : {data_dict['dest_subdir']} | partition tag : {partition_tag}", end=""
                # )
                # modify src SSD and save to dest
                dest_ssd_filename = data_dict["dest_path_ssdfile"]
                modify_and_copy_SSD(data_dict["src_path_ssdfile"], dest_ssd_filename)
                # verify success
                ssd_success = verify_file(dest_ssd_filename, src_dir, "SSD", failures)
                if ssd_success:
                    output_files.append(data_dict["dest_subdir_ssdfile"])
                    part_file_list_dict[partition_tag].append(
                        data_dict["dest_subdir_ssdfile"].replace(partition_tag, "*", 1)
                    )
                # Copy glb file to appropriate location
                dest_glb_filename = data_dict["dest_path_glbfile"]
                shutil.copy(data_dict["src_path_glbfile"], dest_glb_filename)
                # verify success
                glb_success = verify_file(dest_glb_filename, src_dir, "GLB", failures)
                if glb_success:
                    output_files.append(data_dict["dest_subdir_glbfile"])
                    part_file_list_dict[partition_tag].append(
                        data_dict["dest_subdir_glbfile"].replace(partition_tag, "*", 1)
                    )

        print(
            f"# of src files processed : {len(file_names_and_paths)} | # of dest files written : {len(output_files)} | # of failures : {len(failures)} ",
            end="",
        )
        # for part, files in part_file_list_dict.items():
        #     print(f"Partition : {part} | # files {len(files)}", end="")
        #     for filename in files:
        #         print(f"\t{filename}", end="")

        # Get relative paths to all 5 annotation configs, as well as build scene dataset configs,if requested
        build_annotation_configs(part_file_list_dict, output_files)

        # save filenames of the annotation files that have been written, to facilitate archiving
        save_annotated_file_lists(output_files)

        # display failures if they have occurred
        if len(failures) > 0:
            print("\n!!!!! The following files failed to be written : ", end="")
            for src_dir, fail_dict in failures.items():
                for file_type, file_name in fail_dict.items():
                    print(
                        f"Src : {src_dir} :\n\tType : {file_type} | Filename : {file_name} ",
                        end="",
                    )


if __name__ == "__main__":
    main()
