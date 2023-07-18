#!/usr/bin/env python

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

# npz2scn - tool for extracting semantic scene information from 3dscenegraph
#           dataset (https://3dscenegraph.stanford.edu/)

import argparse
import json
from typing import Any, Dict

import numpy as np


# Convert ndarrays to python lists so that we can serialize.
def listify(entry: Dict[str, Any]) -> None:
    for key in entry:
        if type(entry[key]) is np.ndarray:
            entry[key] = entry[key].tolist()


def main():
    parser = argparse.ArgumentParser(
        description="Extracts object IDs from npz. Used for loading 3dscenegraph.stanford.edu semantic data."
    )
    parser.add_argument("npz_path", metavar="foo.npz", help="path to .npz file to load")
    parser.add_argument(
        "scn_path", metavar="foo.scn", help="path to output file to write"
    )
    args = parser.parse_args()
    data = np.load(args.npz_path, allow_pickle=True)["output"].item()
    objs = data["object"]
    for obj in objs.values():
        listify(obj)
    rooms = data["room"]
    for room in rooms.values():
        listify(room)
    output = {"objects": list(objs.values()), "rooms": list(rooms.values())}
    with open(args.scn_path, "w") as f:
        x = f.write(json.dumps(output, indent=2))
        print("wrote %d bytes" % x)


if __name__ == "__main__":
    main()
