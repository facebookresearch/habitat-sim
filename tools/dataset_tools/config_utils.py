#!/usr/bin/env python

import json
import os
import re
from glob import glob
from os.path import basename, join
from typing import Any, Dict, Optional, Tuple

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


def transform_path_relative(
    src_file_path: str, target_path: str, check_exists: Optional[bool] = False
) -> str:
    """transform passed src_file_path to be relative to passed target_path, verify,
    and return result.  If fails, returns None
    :param src_file_path: Path and file name of source file
    :param target_path: Path to use to find relative path
    :param check_exists: Whether to verify that the specified source file exists
    before the relative path is constructed
    :return: The file name of src_file_path including path relative to target, or the
    absolute file path if check_exists is specified and the source file does not exist.
    """
    abs_src_file_path = os.path.normpath(src_file_path)
    # verify this is an existing file if requested
    if not (check_exists) or (os.path.isfile(abs_src_file_path)):
        return os.path.relpath(abs_src_file_path, start=target_path)
    else:
        print(
            "transform_path_relative : Path {} does not point to valid file. Aborting."
        )
        return abs_src_file_path


def build_config_src_dest_dict(
    config_src_dir: str,
    config_dest_dir: str,
    predicate: Optional[Any] = lambda x: True,
    debug: Optional[bool] = False,
) -> Dict[str, Tuple[str, str]]:
    """Build a dictionary keyed by config file name where value is a tuple holding the fully
    qualified source path to the file and the fully qualified dest path for the file.
    :param config_src_dir: Source directory where the configs can be found
    :param config_dest_dir: Destination directory where the configs should be written to
    :param predicate: lambda function to use to filter results based on basename values
    :param debug: Whether to display the results of the mapping.
    :return: Dictionary holding the requested mappings for all files found in config_src_dir
    """
    # key is file name, value is tuple of (srcfile, destfile)
    config_json_dict = {
        basename(x): (x, join(config_dest_dir, basename(x)))
        for x in glob(join(config_src_dir, "*.json"))
        if predicate(x)
    }
    if (len(config_json_dict)) == 0:
        print(
            "build_config_src_dest_dict( config_src_dir : {}, config_dest_dir : {}) : No files found/mapped!".format(
                config_src_dir, config_dest_dir
            )
        )
    if debug:
        for k, v in config_json_dict.items():
            print("Basename : {} : value : {}".format(k, v))
    return config_json_dict


def mod_config_paths_rel_dest(
    file_tuple: Tuple[str, str],
    json_data: Dict[str, Any],
    debug: Optional[bool] = False,
):
    """Takes the passed json_data configuration and re-maps all the file paths
    it references to be relative to a new directory, specified in file_tuple.
    :param file_tuple: A tuple containing the source file name of the given json
    config, and the destination file directory, which is most likely where the JSON file
    will be written.
    :param json_data: Dictionary holding the JSON configuration file.
    :param debug: Display when remapped relative paths are found. Not found paths
    are always displayed.
    """

    src_file = os.path.abspath(file_tuple[0])
    dest_dir = os.path.dirname(os.path.abspath(file_tuple[1]))
    # all possible object or stage config tags that might have file paths
    # only modifies if tag is present in json_data
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


def update_dict_recurse(dest_dict, update_dict):
    from collections.abc import Mapping

    for k, v in update_dict.items():
        if isinstance(v, Mapping):
            dest_dict[k] = update_dict_recurse(dest_dict.get(k, {}), v)
        else:
            dest_dict[k] = v
    return dest_dict


def load_json_into_dict(filename: str) -> Dict[str, Any]:
    """This function loads a JSON file after verifying that the passed file name
    references a file in the system. If multiple matches are found, the first is used.
    :param filename: name of JSON file we wish to load.  If not fully path qualified,
    will search for file and use first matching path found.
    :return: Dictionary containing JSON values read from file
    """
    from glob import glob

    file_list = glob(os.path.join("**", filename), recursive=True)
    if len(file_list) == 0:
        print(
            "JSON File {} not found, so no configuration data loaded. Aborting. ".format(
                filename
            )
        )
        return {}
    src_file = file_list[0]

    with open(src_file, "r") as src:
        json_data = json.load(src)
    return json_data


def mod_json_val_and_save(
    file_tuple: Tuple[str, str, Dict[str, Any], bool],
    indent: Optional[int] = 2,
    dry_run: Optional[bool] = False,
):
    """This function will either load an existing config file, add/modify the specified field
    using the given tag, and save the file to the specified location/file name, or,
    if the given src_name is empty, it will save the passed dictionary as json using the provided
    dest_file name.
    If dry_run is specified, nothing will be written but rather the expected output will be displayed.

    :param file_tuple: A 3-4 element tuple that contains the following information
        file_tuple[0] is config json src file loc - if empty, create dest_file
        file_tuple[1] is dest file loc.
        file_tuple[2] is dict : key is json tag for data, value is value to set in json
        file_tuple[3] is boolean : whether or not to make config file paths in existing config files
            relative to new dest_file location. Only relevant if modifying an existing json
    :param indent: JSON indention to use when writing file
    :param dry_run: If True, display results of modifications but do not write output.
    """

    src_file = file_tuple[0]
    dest_file = file_tuple[1]
    json_mod_vals = file_tuple[2]
    # whether paths in resultant configs should be changed to be relative to new destination
    set_paths_rel_dest = False
    if len(file_tuple) > 3:
        set_paths_rel_dest = file_tuple[3]

    if src_file == "":
        # if no src_file then creating a new json config
        json_data = json_mod_vals
    else:
        json_data = load_json_into_dict(src_file)
        # Update loaded json with passed modified values
        json_data = update_dict_recurse(json_data, json_mod_vals)

    # if specified, make all embedded paths in json_data
    if set_paths_rel_dest:
        mod_config_paths_rel_dest(file_tuple, json_data)

    if not dry_run:
        with open(dest_file, "w") as dest:
            json.dump(json_data, dest, indent=indent)
    else:
        print(
            "mod_json_val_and_save (dry run) : JSON data to be written :"
            "\n{}\n to file named : {}".format(json_data, dest_file)
        )


def get_files_matching_regex(
    src_dir: str, regex_str: Optional[str] = "", debug: Optional[bool] = False
):
    """This function walks a given source directory for all files matching
    passed regex string.  If no regex is specified, it returns all files found.
    :param src_dir: Directory to walk
    :param regex_str: String describing regex to match when building file list.
    If unspecified or empty, return all files, optional.
    :param debug: Whether to display files not matching given regex in src_dir.
    :return: List of tuples containing path, dirname and file name for each result.
    """
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


def build_disp_string_elem(k: str, v: Any, tab: str, new_tab: str):
    """This function is a utility to build a reasonably formatted
    display string for dictionaries or lists
    """
    res = ""
    k_disp_str = ""
    if len(k) > 0:
        k_disp_str = "{} : ".format(k)
    if isinstance(v, dict):
        res += tab + "{}({}\n{}{}),\n".format(
            k_disp_str, tab, dict_to_disp_string(v, new_tab), tab
        )
    elif isinstance(v, list):
        res += tab + "{}[\n{}{}],\n".format(
            k_disp_str, list_to_disp_string(v, new_tab), tab
        )
    else:
        res += tab + "{}{},\n".format(k_disp_str, v)
    return res


def list_to_disp_string(ara: list, tab):
    """This function is a utility to build a reasonably formatted
    display string for a list variable's contents
    """
    res = ""
    new_tab = tab + "\t"
    for elem in ara:
        res += build_disp_string_elem("", elem, tab, new_tab)
    return res


def dict_to_disp_string(dictionary, tab):
    """This function is a utility to build a reasonably formatted
    display string for a dictionary variable's contents
    """
    res = ""
    new_tab = tab + "\t"
    for k, v in dictionary.items():
        res += build_disp_string_elem(k, v, tab, new_tab)
    return res


# takes a source directory, returns a list of tuples of paths amd dirnames
# list of filenames if no regex_str is specified, returns all directories
def get_directories_matching_regex(
    src_dir: str, regex_str: Optional[str] = "", debug: Optional[bool] = False
):
    """This function walks a given source directory for all subdirectories matching
    passed regex string.  If no regex is specified, it returns all subdirectories found.
    :param src_dir: Directory to walk
    :param regex_str: String describing regex to match when building subdirectories list.
    If unspecified or empty, return all subdirectories, optional.
    :param debug: Whether to display subdirectories not matching given regex in src_dir.
    :return: List of tuples containing path and subdirectory name each result.
    """
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
