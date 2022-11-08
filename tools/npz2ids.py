#!/usr/bin/env python

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

# npz2ids - tool for extracting object_ids from 3dscenegraph dataset
#           (https://3dscenegraph.stanford.edu/)

import argparse

import numpy as np


def main():
    parser = argparse.ArgumentParser(description="Extract object IDs from npz.")
    parser.add_argument("npz_path", metavar="foo.npz", help="path to .npz file to load")
    parser.add_argument(
        "ids_path", metavar="foo.ids", help="path to output file to write"
    )
    args = parser.parse_args()
    data = np.load(args.npz_path, allow_pickle=True)["output"].item()
    with open(args.ids_path, "wb") as f:
        object_ids = data["building"]["object_inst_segmentation"]
        x = f.write(object_ids.astype(np.int16).tobytes())
        print("wrote %d bytes" % x)


if __name__ == "__main__":
    main()
