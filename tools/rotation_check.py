# check for rotations in parts metadata

import glob
import json
import numpy as np
import magnum as mn

def round_to_identity(q, threshold=1e-6) -> bool:
    """
    Rounds a quaternion to the identity quaternion if its components
    are within a given threshold of the identity (1, 0, 0, 0).

    Args:
        q (np.array): A 4-element numpy array representing the quaternion (w, x, y, z).
        threshold (float): The maximum allowed difference for each component from the identity.

    Returns:
        np.array: The rounded quaternion.
    """
    identity_quat = np.array([1.0, 0.0, 0.0, 0.0])
    if np.allclose(q, identity_quat, atol=threshold):
        return True
    elif np.allclose(
        q, -identity_quat, atol=threshold
    ):  # Account for -q representing the same rotation
        return True
    else:
        return False

dirty_files = 0

pattern = "/home/jseagull/dev/fp-models/objects/decomposed/**/*.parts.metadata.json"
all_objects = glob.glob (pattern,recursive=True)

#each file
for decomp in all_objects:
    file_has_rotations = False
    with open(decomp, "r") as file:
        group_data = json.load(file)
    uuid = group_data.get("id","")    
    #each object
    for obj_data in group_data.get("parts", {}):
        obj_quat = obj_data.get("transform",{}).get("quaternion",None)
        obj_quat = np.array([obj_quat[3],obj_quat[0],obj_quat[1],obj_quat[2]])
        if not round_to_identity(obj_quat, .1):
            if not file_has_rotations:
                print(f"\n{uuid}:")
                file_has_rotations = True  
            part_Id = obj_data.get("partId", "")
            pretty_quat = np.around(obj_quat,3)
            print(f"Object {part_Id} : {pretty_quat}")
    if file_has_rotations:
        dirty_files += 1
    #else:
        #print(f"{uuid} OK")
print(f"{dirty_files} out of {len(all_objects)} have rotated parts.")