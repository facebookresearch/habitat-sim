#!/usr/bin/env python3

# This script will copy Appen annotated files into an existing HM3D dataset,
# along with making the necessary modifications to the semantic txt file to
# be compatible with Habitat sim.

import re
import shutil
from os.path import exists, isfile, join
from typing import Dict

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

HM3D_ANNOTATION_SRC_DIR = "/home/john/Datasets In Progress/HM3D_Semantic/Appen_scenes"

#
# Appen annotation source scene directory regex.
# This regex describes the format of the per-scene directories in the Appen work,
# and may change depending on whether the format we receive from Appen changes.

HM3D_ANNOTATION_SUBDIR_RE = r"(?i)[0-9]{5}-[a-z0-9]{11}\.semantic$"


##############################################################################
## You should not need to modify anything below here

# sentinel string for semantic scene descriptor text files in HM3D
# DO NOT CHANGE.  This is looked for in Habitat-sim
HM3D_SSD_STRING = "HM3D Semantic Annotations"


# Modify the given semantic text file to include the necessary sentinel string at the
# beginning that Habitat-Sim expects, and then save to appropriate dest directory
def modify_and_copy_SSD(src_filename: str, dest_filename: str):
    with open(dest_filename, "w") as dest, open(src_filename, "r") as src:
        dest.write("{}\n".format(HM3D_SSD_STRING))
        for line in src:
            dest.write("{}".format(line))


def buildFileListing():
    # go through annotation directory, find paths to all files of
    # interest (annotated glbs and semantic lexicon/text files)

    # directory name pattern to find annotation files
    annotation_dir_pattern = re.compile(HM3D_ANNOTATION_SUBDIR_RE)
    # directory listing
    dir_listing = ut.get_directories_matching_regex(
        HM3D_ANNOTATION_SRC_DIR, annotation_dir_pattern
    )
    # destination directory will be based on numeric field, if available
    file_names_and_paths = {}
    for src_dir, dirname_full in dir_listing:
        src_dirname_full = join(src_dir, dirname_full)
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
                scene_dest_dir = join(dest_dir[0], dest_dir[1])
                src_files = {}
                for src_full_dir, _, src_filename in src_file_list:
                    if ".txt" in src_filename:
                        key = "ssdfile"
                    else:
                        key = "glbfile"

                    src_files["dest_" + key + "_path"] = join(
                        scene_dest_dir, src_filename
                    )
                    # TODO perhaps we wish to rename the file in the destination? If so, do so here, instead of using src_filename
                    src_files["src_" + key + "_path"] = join(src_full_dir, src_filename)

                tmp_dict = {"dest_dir": scene_dest_dir}
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
    for k, v in file_names_and_paths.items():
        print("Src : {} : # components : {}".format(k, len(v)))

    return file_names_and_paths


def verify_files(filename: str, file_key: str, type_key: str, failures: Dict):

    success = exists(filename) and isfile(filename)
    proc = "has"
    if not success:
        proc = "HAS NOT"
        if file_key not in failures:
            failures[file_key] = {}
        failures[file_key][type_key] = filename
    print(
        "\t{} File {} been successfully modified and copied to {}".format(
            type_key, proc, filename
        )
    )


def main():
    # build dictionary of src and dest file names and paths
    file_names_and_paths = buildFileListing()
    # move semantic glbs and scene descriptor text files
    failures = {}
    for src_dir, data_dict_list in file_names_and_paths.items():
        print("Src : {} : size of data_dict : {} ".format(src_dir, len(data_dict_list)))
        for data_dict in data_dict_list:

            # modify src SSD and save to dest
            dest_ssd_filename = data_dict["dest_ssdfile_path"]
            modify_and_copy_SSD(data_dict["src_ssdfile_path"], dest_ssd_filename)
            # verify success
            verify_files(dest_ssd_filename, src_dir, "SSD", failures)

            # Copy glb file to appropriate location
            dest_glb_filename = data_dict["dest_glbfile_path"]
            shutil.copy(data_dict["src_glbfile_path"], dest_glb_filename)
            # verify success
            verify_files(dest_glb_filename, src_dir, "GLB", failures)

    print(
        "# of files processed : {} | # of failures : {} ".format(
            len(file_names_and_paths), len(failures)
        )
    )
    if len(failures) > 0:
        print("The following files failed to be written : ")
        for src_dir, fail_dict in failures.items():
            for file_type, file_name in fail_dict.items():
                print(
                    "Src : {} :\n\tType : {} | Filename : {} ".format(
                        src_dir, file_type, file_name
                    )
                )


if __name__ == "__main__":
    main()
