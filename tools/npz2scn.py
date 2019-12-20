#!/usr/bin/env python

# npz2scn - tool for extracting semantic scene information from 3dscenegraph
#           dataset (https://3dscenegraph.stanford.edu/)

import argparse
import json
import sys
from typing import Any, Dict

import numpy as np


# Convert ndarrays to python lists so that we can serialize.
# Transform coordinates by rotating Y-axis to Z-axis
def fix_coords(entry: Dict[str, Any]) -> None:
    size = entry["size"].tolist()
    size[1], size[2] = size[2], size[1]
    entry["size"] = size
    loc = entry["location"].tolist()
    loc[1], loc[2] = -loc[2], loc[1]
    entry["location"] = loc


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
        fix_coords(obj)
    rooms = data["room"]
    for room in rooms.values():
        fix_coords(room)
    output = dict()
    output["objects"] = list(objs.values())
    output["rooms"] = list(rooms.values())
    with open(args.scn_path, "w") as f:
        x = f.write(json.dumps(output, indent=2))
        print("wrote %d bytes" % x)


if __name__ == "__main__":
    main()
