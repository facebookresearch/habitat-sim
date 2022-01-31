#!/usr/bin/env python3

# This script will copy Appen annotated files into an existing HM3D dataset,
# along with making the necessary modifications to the semantic txt file to
# be compatible with Habitat sim.

import re
import shutil
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

#
# Appen annotation source scene directory regex.
# This regex describes the format of the per-scene directories in the Appen work,
# and may change depending on whether the format we receive from Appen changes.

HM3D_ANNOTATION_SUBDIR_RE = r"(?i)[0-9]{5}-[a-z0-9]{11}\.semantic$"

#
# Whether or not to build annotation scene dataset configs.
BUILD_SD_CONFIGS = True


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
    if len(lineItems) != 4:
        return "Incorrect # of items : should be 4, is {}".format(len(lineItems))
    if not lineItems[0].strip().isdigit():
        return "First entry (unique ID) is not positive integer : {}".format(
            lineItems[0]
        )
    if not re.compile("^[A-F0-9]{6}$").match(lineItems[1].strip()):
        return "Second entry (color) is not 6 digit hex value : {}".format(lineItems[1])
    if len(lineItems[2].strip()) == 0:
        return "Third entry (category) cannot be an empty string"
    if not lineItems[3].strip().isdigit():
        return "Forth entry (region) is not non-negative integer : {}".format(
            lineItems[3]
        )
    if not lineItems[0].strip().isdigit():
        return "First entry is not positive integer; is : {}".format(lineItems[0])
    return "fine"


# Modify the given semantic text file to include the necessary sentinel string at the
# beginning that Habitat-Sim expects, and then save to appropriate dest directory
def modify_and_copy_SSD(src_filename: str, dest_filename: str):
    with open(dest_filename, "w") as dest, open(src_filename, "r") as src:
        dest.write("{}\n".format(HM3D_SSD_STRING))
        i = 0
        for line in src:
            if i > 0:
                valid_res = validate_src_SSD(line)
                if valid_res != "fine":
                    print(
                        "!!!! Error in source SSD `{}` on line {} : {} has format error : `{}`".format(
                            src_filename, i, line, valid_res
                        )
                    )
            dest.write("{}".format(line))
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
                "Problem with source dir files {} : Unable to find 2 source files ({} instead) so skipping this source dir.".format(
                    dirname_full, len(src_file_list)
                )
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
                "Problem with source dir {} : Unable to find destination dir due to {} matching destinations in current HM3D dataset so skipping this source dir.".format(
                    dirname_full, len(dest_dir_list)
                )
            )
            for bad_dest in dest_dir_list:
                print("\t{}".format(bad_dest))

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
        "\t\t{} File {} been successfully modified and copied to {}".format(
            type_key, proc, filename
        )
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
    for config in src_json_configs:
        filename = os_join(config[0], config[-1])
        # want to overwrite old versions
        if "hm3d_annotated_" in filename:
            continue
        # success = os_exists(filename) and os_isfile(filename)
        json_filename = config[-1]
        config_filenames[json_filename] = filename
        new_config_filenames[json_filename] = filename.replace(
            "hm3d_", "hm3d_annotated_"
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
        print(
            "{} # files : {} : \n\t{}\n\t{}".format(
                config_key,
                len(scene_path_list),
                src_config_filename,
                dest_config_filename,
            )
        )
        # print("\n\tfiles:")
        # for pathname in scene_path_list:
        #     print("\t\t{}".format(pathname))

        # load each existing json config, appropriately modify it, and then save as new configs
        if BUILD_SD_CONFIGS:
            src_json_config = ut.load_json_into_dict(src_config_filename)
            for key, json_obj in src_json_config.items():
                if "paths" in json_obj:
                    # Modify both stages and
                    # this is the dictionary of lists of places to look for specified config type files
                    paths_dict = src_json_config[key]["paths"]
                    for path_type_key, lu_path_list in paths_dict.items():
                        # assume the list has at least one element
                        lu_glob_file = lu_path_list[0].split(os_sep)[-1]
                        modify_paths_tag(
                            paths_dict, path_type_key, lu_glob_file, scene_path_list
                        )
            ut.save_json_to_file(src_json_config, dest_config_filename)

        # add subdirectory-qualified file paths to new configs to output_files list so that they will
        # be included in zip file
        rel_config_filename = dest_config_filename.split(HM3D_DEST_DIR)[-1].split(
            os_sep, 1
        )[-1]
        print(
            "Adding rel_config_filename : {} to output_files.".format(
                rel_config_filename
            )
        )
        output_files.append(rel_config_filename)


def save_annotated_file_lists(output_files: List):
    print("save_annotated_file_lists:")
    # write text files that hold listings of appropriate relative filepaths for
    # annotated files as well as for each partition for all scenes that have annotations

    # all annotated files (annotated glb and txt only)
    with open(os_join(HM3D_DEST_DIR, "HM3D_annotation_files.txt"), "w") as dest:
        dest.write("\n".join(output_files))

    # save listing for each partition
    # each entry is a tuple with idx 0 == dest filename; idx 1 == list of paths
    partition_lists = {}
    for part in HM3D_DATA_PARTITIONS:
        partition_lists[part] = (
            os_join(HM3D_DEST_DIR, "HM3D_annotation_{}_dataset.txt".format(part)),
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
        # add base files
        if ".semantic.glb" in filename:
            filename_base = filename.split(".semantic.glb")[0]
            partition_lists[file_part_key][1].append(
                "{}.basis.glb".format(filename_base)
            )
            partition_lists[file_part_key][1].append(
                "{}.basis.navmesh".format(filename_base)
            )
    # write each per-partition file listing to build per-partition annotated-only datasets
    for _, v in partition_lists.items():
        with open(v[0], "w") as dest:
            dest.write("\n".join(v[1]))


def main():
    # build dictionary of src and dest file names and paths
    file_names_and_paths = buildFileListing()
    # Failures here will be files that did not get copied (or modified if appropriate)
    failures = {}
    # dictionary keyed by partition valued by list of partition subdir and filename of written file
    part_file_list_dict = {}
    for key in HM3D_DATA_PARTITIONS:
        part_file_list_dict[key] = []
    # fully qualified paths to all output files
    output_files = []
    # move semantic glbs and scene descriptor text files
    for src_dir, data_dict_list in file_names_and_paths.items():
        # print("Src : {} : # of data_dicts : {} ".format(src_dir, len(data_dict_list)))
        for data_dict in data_dict_list:
            partition_tag = data_dict["dest_part_tag"]
            # print(
            #     "\tDest subdir under HM3D directory : {} | partition tag : {}".format(
            #         data_dict["dest_subdir"], partition_tag
            #     )
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
        "# of src files processed : {} | # of dest files written : {} | # of failures : {} ".format(
            len(file_names_and_paths), len(output_files), len(failures)
        )
    )
    # for part, files in part_file_list_dict.items():
    #     print("Partition : {} | # files {}".format(part, len(files)))
    #     for filename in files:
    #         print("\t{}".format(filename))

    # If requesting to build scene dataset configs, build them, and also get relative paths to all 5 annotation configs

    build_annotation_configs(part_file_list_dict, output_files)

    # save filenames of the annotation files that have been written, to facilitate archiving
    save_annotated_file_lists(output_files)

    # display failures if they have occurred
    if len(failures) > 0:
        print("!!!!! The following files failed to be written : ")
        for src_dir, fail_dict in failures.items():
            for file_type, file_name in fail_dict.items():
                print(
                    "Src : {} :\n\tType : {} | Filename : {} ".format(
                        src_dir, file_type, file_name
                    )
                )


if __name__ == "__main__":
    main()
