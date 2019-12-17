#!/usr/bin/env python

# npz2ids - tool for extracting object_ids from 3dscenegraph dataset
#           (https://3dscenegraph.stanford.edu/)

import sys

import numpy as np


def main():
    npz_path = sys.argv[1]
    ids_path = sys.argv[2]
    data = np.load(npz_path, allow_pickle=True)["output"].item()
    f = open(ids_path, "wb")
    object_ids = data["building"]["object_inst_segmentation"]
    x = f.write(object_ids.astype(np.int16).tobytes())
    print("wrote %d bytes" % x)
    f.close()


if __name__ == "__main__":
    main()
