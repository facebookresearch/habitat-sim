# check for rotations in parts metadata

import glob
import json

import numpy as np


def round_to_identity(q, threshold=1e-6) -> bool:
    """ """
    identity_scale = np.array([1.0, 1.0, 1.0])
    if np.allclose(q, identity_scale, atol=threshold):
        return True
    return False


dirty_files = 0

pattern = "/home/jseagull/dev/fp-models/objects/decomposed/**/*.parts.metadata.json"
all_objects = glob.glob(pattern, recursive=True)

# each file
for decomp in all_objects:
    file_has_scaling = False
    with open(decomp, "r") as file:
        group_data = json.load(file)
    uuid = group_data.get("id", "")
    # each object
    for obj_data in group_data.get("parts", {}):
        obj_scale = obj_data.get("transform", {}).get("scale", None)
        obj_scale = np.array(obj_scale)
        if not round_to_identity(obj_scale, 0.01):
            if not file_has_scaling:
                print(f"\n{uuid}:")
                file_has_scaling = True
            part_Id = obj_data.get("partId", "")
            pretty_scale = np.around(obj_scale, 3)
            print(f"Object {part_Id} : {pretty_scale}")
    if file_has_scaling:
        dirty_files += 1
    # else:
    # print(f"{uuid} OK")
print(f"{dirty_files} out of {len(all_objects)} have scaled parts.")
