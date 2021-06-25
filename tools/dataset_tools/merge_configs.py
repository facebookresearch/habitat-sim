#!/usr/bin/env python

# merge_configs - tool for taking new configs and merging in data from existing configs.
# Assets referenced in configs can be specified to be repathed as they are writtend, so that
# they will always be relative to the destination location for the merged configs.
import os
from os.path import join

import config_utils as ut

# whether or not to update paths to assets in config to be absolute so that saving
# this file in an alternate location does not break the config load
ABSOLUTE_ASSET_PATHS = True

# Where scene dataset data is located, relative to this file
DATA_SET_DIR = "../../data/scene_datasets/replicaCAD/"

# where base json configs are located - these will be written over with CONFIG_MOD_DIR jsons
CONFIG_SRC_DIR = join(DATA_SET_DIR, "old_configs/objects")
# where modifying json configs are located - these will be written onto the CONFIG_SRC_DIR jsons
CONFIG_MOD_DIR = join(DATA_SET_DIR, "merge_configs/objects")
# where modified json configs should be written
CONFIG_DEST_DIR = join(DATA_SET_DIR, "new_configs/objects")
os.makedirs(CONFIG_DEST_DIR, exist_ok=True)


def main():

    # Build a dictionary keyed by config file name where value is a tuple holding the fully
    # qualified source path to the file and the fully qualified dest path for the file.
    config_json_dict = ut.build_config_src_dest_dict(
        CONFIG_SRC_DIR, CONFIG_DEST_DIR, debug=True
    )

    # Load configs to use to write over CONFIG_SRC_DIR configs.
    config_json_merge_dict = ut.build_config_src_dest_dict(
        CONFIG_MOD_DIR, CONFIG_MOD_DIR, debug=True
    )

    # save results to file
    for k, v in config_json_dict.items():
        mod_json_dict = {}
        # get modifying JSON if exists
        if k in config_json_merge_dict:
            # if a modifying JSON exists, load it into a dictionary
            mod_json_dict = ut.load_json_into_dict(config_json_merge_dict[k][0])

        # save modified JSON value to new location
        ut.mod_json_val_and_save((v[0], v[1], mod_json_dict, ABSOLUTE_ASSET_PATHS))


if __name__ == "__main__":
    main()
