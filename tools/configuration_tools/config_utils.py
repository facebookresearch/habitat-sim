#!/usr/bin/env python

import json
import os
import re

# sub-extensions used with json config files to denote the type
# of configuration.  Append ".json" to make full file name
CONFIG_EXTENSIONS = {
    "lighting": ".lighting_config",
    "object": ".object_config",
    "physics": ".physics_config",
    "dataset": ".scene_dataset_config",
    "scene": ".scene_instance",
    "stage": ".stage_config",
}

# transform passed src_file_path to be relative to passed target_path, verify,
# and return result.  If fails, returns None
def transform_path_relative(src_file_path, target_path, checkExists=False):
    abs_src_file_path = os.path.normpath(src_file_path)
    # verify this is an existing file if requested
    if not (checkExists) or (os.path.isfile(abs_src_file_path)):
        return os.path.relpath(abs_src_file_path, start=target_path)
    else:
        print(
            "transform_path_relative : Path {} does not point to valid file. Aborting."
        )
        return abs_src_file_path


# Takes a loaded json config and for all the path data
# values, will modify them to be relative to destination dir
# instead of src_dir
def mod_config_paths_rel_dest(file_tuple, json_data, debug=False):
    src_file = os.path.abspath(file_tuple[0])
    dest_dir = os.path.dirname(os.path.abspath(file_tuple[1]))
    # all possible tags that might have file paths
    # only modifies if tag is present in file
    filename_tag_list = [
        "render_asset",
        "collision_asset",
        "semantic_asset",
        "nav_asst",
        "house_filename",
    ]
    # find src_file directory, which is base directory of asset's relative path
    asset_base_dir = os.path.split(src_file)[0]

    for tag in filename_tag_list:
        # check if tag exists already in file
        if tag in json_data:
            old_file_name = json_data[tag]
            abs_old_file = os.path.normpath(os.path.join(asset_base_dir, old_file_name))
            base_string = (
                "Attempting to modify {} config to have asset relative"
                ' filepath "{}" changed to absolute "{}" (with config moved to Dest dir {}) and this'
                " filepath ".format(src_file, old_file_name, abs_old_file, dest_dir)
            )
            if os.path.isfile(abs_old_file):
                new_file_name = os.path.relpath(abs_old_file, start=dest_dir)
                json_data[tag] = new_file_name
                if debug:
                    print(
                        "{} found! Relative path used {}".format(
                            base_string, new_file_name
                        )
                    )
            else:
                print("{} not found, so no changes were made.".format(base_string))


# this will load an object's config file, add/modify the specified field
# using the given tag, and save the file to the specified location/file name.
# If dry_run is specified, nothing will be written but rather the expected output
# will be displaed
def mod_json_val_and_save(file_tuple, indent=2, dry_run=False):
    # file_tuple[0] is config json src file loc - if empty, create dest_file
    # file_tuple[1] is dest file loc.
    # file_tuple[2] is dict : key is json tag for data, value is value to set in json
    # file_tuple[3] is boolean : whether or not to make config file name values absolute paths.
    # indent is indention to use for json file writing
    src_file = file_tuple[0]
    dest_file = file_tuple[1]
    json_mod_vals = file_tuple[2]

    if src_file == "":
        # if no src_file then creating a new json config
        json_data = json_mod_vals
    else:
        # whether paths in resultant configs should stay relative to old
        # config location or should be changed to be relative to new destination
        set_paths_rel_dest = file_tuple[3]
        with open(src_file, "r") as src:
            json_data = json.load(src)
            if set_paths_rel_dest:
                mod_config_paths_rel_dest(file_tuple, json_data)
    for tag, value in json_mod_vals.items():
        json_data[tag] = value
    if not dry_run:
        with open(dest_file, "w") as dest:
            json.dump(json_data, dest, indent=indent)
    else:
        print(
            "mod_json_val_and_save (dry run) : JSON data to be written :"
            "\n{}\n to file named : {}".format(json_data, dest_file)
        )


# takes a source file directory, returns a list of tuples of paths, dirnames, filenames
# if no regex_str is specified, returns all files
def get_files_matching_regex(src_dir, regex_str="", debug=False):
    res_list = []
    found_count = 0
    if regex_str == "":
        for path, dirnames, files in os.walk(src_dir):
            for fname in files:
                found_count += 1
                res_list.append((path, dirnames, fname))
    else:
        config_file_pattern = re.compile(regex_str)
        # find all files matching pattern
        for path, dirnames, files in os.walk(src_dir):
            for fname in files:
                found_count += 1
                if config_file_pattern.match(fname):
                    res_list.append((path, dirnames, fname))
                elif debug:
                    print(
                        "get_files_matching_regex : {} not match regex {}".format(
                            fname, regex_str
                        )
                    )
    print(
        "get_files_matching_regex : Found and matched {} of {} files in {}. If inaccurate, set debug=True".format(
            len(res_list), found_count, src_dir
        )
    )
    return res_list


# takes a source directory, returns a list of tuples of paths amd dirnames
# list of filenames if no regex_str is specified, returns all directories
def get_directories_matching_regex(src_dir, regex_str="", debug=False):
    res_list = []
    found_count = 0
    if regex_str == "":
        for path, dirnames, _ in os.walk(src_dir):
            for dirname in dirnames:
                found_count += 1
                res_list.append((path, dirname))
    else:
        config_file_pattern = re.compile(regex_str)
        # find all subdirectories matching pattern
        for path, dirnames, _ in os.walk(src_dir):
            for dirname in dirnames:
                found_count += 1
                if config_file_pattern.match(dirname):
                    res_list.append((path, dirname))
                elif debug:
                    print(
                        "get_directories_matching_regex : {} not match regex {}".format(
                            dirname, regex_str
                        )
                    )
    print(
        "get_directories_matching_regex : Found and matched {} of {} directories in {}. If inaccurate, set debug=True".format(
            len(res_list), found_count, src_dir
        )
    )
    return res_list
