# fix scale in JSON

import argparse
import magnum as mn
import json
import numpy as np
from math import modf
from typing import Union


def get_object_data() -> dict:
    for obj_data in json_data.get("parts", {}):
        partId = obj_data.get("partId", "")
        if partId != part:
            continue
        return obj_data


if __name__ == "__main__":
    argparser = argparse.ArgumentParser()
    argparser.add_argument(
        "-u",
        "--uuid",
        type=str,
        default="077b4aad95f9bcb3954d0ce719605f6a08cd253f",  # bookshelf
        help="the UUID of the decomposed object to load, default 077b4aad95f9bcb3954d0ce719605f6a08cd253f",
        action="store",
    )

    argparser.add_argument(
        "-o",
        "--object",
        type=str,
        default="1",
        help="the object number needing correction",
        action="store",
    )

    argparser.add_argument(
        "-q",
        "--quat",
        type=float,
        nargs=4,
        required=True,
        help="new quat rotation value as BLENDER WXYZ",
        action="store",
    )

    args = argparser.parse_args()
    uuid = args.uuid
    part = args.object
    # Blender [WXYZ] becomes Magnum ((XZY)W)
    quat_new = [args.quat[1], args.quat[3], args.quat[2], args.quat[0]]

    parts_metadata_path = (
        f"data/fp-models/objects/decomposed/{uuid}/{uuid}.parts.metadata.json"
    )
    with open(parts_metadata_path, "r") as file:
        json_data = json.load(file)
    object_data = get_object_data()
    part_transform = object_data.get("transform", {})

    print("Starting Transform...")
    # print(json.dumps(part_transform))

    # Get/Convert Scale values
    scale_str = part_transform["scale"]
    scale_old_mn = mn.Vector3(scale_str[0], scale_str[1], scale_str[2])
    scale_old_mn = mn.Matrix3x3.from_diagonal(scale_old_mn)

    # Get/Convert Rotation values
    quat_old = part_transform["quaternion"]
    quat_old_mn = mn.Quaternion(((quat_old[0], quat_old[1], quat_old[2]), quat_old[3]))
    quat_old_matrix = quat_old_mn.to_matrix()
    quat_new_mn = mn.Quaternion(
        (quat_new[0], quat_new[1], quat_new[2]), quat_new[3]
    ).normalized()
    quat_new_matrix = quat_new_mn.to_matrix()
    # print(f"Old quat matrix:\n{quat_old_matrix}")
    # print(f"New quat matrix:\n{quat_new_matrix}")

    # Get/Convert Position values
    position_str = part_transform["position"]
    position_mn = mn.Vector3(position_str[0], position_str[1], position_str[2])

    # Build transform matrices
    old_matrix = part_transform["matrix"]
    #  3x3 rotation/scale section
    rs_matrix_old = quat_old_matrix @ scale_old_mn
    rs_matrix_new = quat_new_matrix  # new scale is ident
    #  4x4 full matrix
    prs_matrix_old = mn.Matrix4.from_(rs_matrix_old, position_mn)
    prs_matrix_new = mn.Matrix4.from_(rs_matrix_new, position_mn)

    # Set JSON values
    part_transform["quaternion"] = quat_new
    part_transform["scale"] = [1, 1, 1]
    json_matrix = json.dumps((np.array(prs_matrix_new).T.flatten()).tolist())
    part_transform["matrix"] = json_matrix

    # sanity check
    # print("------------------------------------\nOriginal JSON Matrix:")
    # print(json.dumps(old_matrix))
    print(f"Original matrix determinant: {prs_matrix_old.determinant()}")
    # print("------------------------------------\nOriginal Matrix from JSON PRS:")
    # old_matrix_rowmajor = np.array(prs_matrix_old)
    # old_matrix_colmajor = (old_matrix_rowmajor.T).flatten()
    # print(json.dumps(old_matrix_colmajor.tolist()))
    print(f"New matrix determinant: {prs_matrix_new.determinant()}")
    # print('------------------------------------\nNew Matrix:')
    # print(json_matrix)
    # print(f"Full transform Dict:\n{part_transform}")
    
    # write new cvalues to JSON

    # for idx, obj_data in enumerate(json_data["parts"]):
    #     print(f"{idx}: checking {obj_data['partId']} against {part}")
    #     if obj_data["partId"] == part:
    #         print(
    #             f"Updating part {part}...\nOld data:\n{obj_data['transform']}\nNew data:"
    #         )
    #         obj_data["transform"] = part_transform
    #         json_data["parts"][idx] = obj_data
    #         print(f"{json_data['parts'][idx]['transform']}")
    #         break
    #     else:
    #         print("not found")

    with open(parts_metadata_path, "w") as file:
        json.dump(json_data, file)
    print(f"Updated part {part} in {uuid}.")