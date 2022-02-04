#!/usr/bin/env python

# modify_configs.py : This program will modify all json
# configuration files whose name match a specific pattern

import os
from os.path import join

import config_utils as ut

# specify directory where configs can be found.  Will crawl
# all directories under specified directory looking for json
# configs whose names match given regex search parameters.
CONFIG_BASE_DIR = "/Users/jmturner/Documents/ObjectRearrange/p-viz-plan/data/objects"

# desired config filename regex, not including '.json'
CONFIG_FILE_RE = "[0-9]{3}(-[a-z0-9A-Z])?_[_a-z0-9A-Z]+[.]{1}object_config"

# output directory, where modified configs should be placed. If
# empty, will place them in directory where they were found. If
# specified, will reconstruct src file directory structure
CONFIG_OUT_DIR = "/Users/jmturner/Documents/ObjectRearrange/ycb_mod_cfgs"

# whether to modify config names. If so, will add specified
# modification to name. WARNING depending on modification, this
# can cause the file name to be huge.
MODIFY_NAME = True

# whether or not to update paths to assets in config to be absolute so that saving
# this file in an alternate location does not break the config load
ABSOLUTE_ASSET_PATHS = True

# json values to modify.  Add key-value pairs of values to add to all json files whose
# names match specified regex
JSON_MOD_VALS = {"scale": [5.0, 5.0, 5.0]}


def main():

    # get all files in specified directory whose names match passed regex string
    # returns list of tuples of path,dirnames, file names
    file_list = ut.get_files_matching_regex(CONFIG_BASE_DIR, f"{CONFIG_FILE_RE}.json")
    # build dictionary of 4-element tuples to modify json values
    res_dict = {}
    for path, _, filename in file_list:
        src_file = join(path, filename)
        # use same path as default
        dest_path = path
        # add initial entries upon dict creation for this file's json entries
        new_json_vals = {}
        for k, v in JSON_MOD_VALS.items():
            new_json_vals[k] = v

        if len(CONFIG_OUT_DIR) > 0:
            dest_path = path.replace(CONFIG_BASE_DIR, CONFIG_OUT_DIR)
            os.makedirs(dest_path, exist_ok=True)
        dest_name = filename
        if MODIFY_NAME:
            # modify name to contain mod json modification, so files don't conflict
            mod_val_strs = []
            file_name_parts = filename.split(".")
            file_name = file_name_parts[0]
            config_type = file_name_parts[1]
            for json_tag, json_val in new_json_vals.items():
                if type(json_val) is list:
                    mod_val_strs.append(
                        f'{json_tag}_[{(",").join(str(x).replace(".", "-") for x in json_val)}]'
                    )
                else:
                    mod_val_strs.append(f"{json_tag}_{json_val}")

            dest_name = f'{file_name}_{("_").join(mod_val_strs)}.{config_type}.json'
        dest_file = join(dest_path, dest_name)
        res_dict[filename] = (src_file, dest_file, new_json_vals, ABSOLUTE_ASSET_PATHS)

    sorted_key_list = sorted(res_dict.keys())

    for k in sorted_key_list:
        v = res_dict[k]
        # print(f"file:{k} | src loc:{v[0]} | dest file:{v[1]} | json tag & val :{v[2]} ")
        ut.mod_json_val_and_save(v)


if __name__ == "__main__":
    main()
