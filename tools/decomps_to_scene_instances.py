"""Decomps to scene instances
uses a CSV of decomp associations to replace monolithic objects
in HSSD scenes with the decomposed parts
"""

import os
import json
import csv
from typing import Tuple
from copy import deepcopy
import magnum as mn

DECOMP_DATA_PATH = "/home/jseagull/dev/fphab/objects/decomposed/decomp_replacements.csv"
COM_CORRECTION_PATH = "/home/jseagull/dev/fphab/objects/com_correction.csv"
DECOMP_METADATA_PATH = "/home/jseagull/dev/fp-models/objects/decomposed/"
MAX_FILES_TO_PROCESS = 9999
SCENE_DIR = "/home/jseagull/dev/fphab/scenes-uncluttered"


def collect_scenes(dir: str) -> list:
    file_list = []
    for entry in os.listdir(dir):
        full_path = os.path.join(dir, entry)
        if os.path.isfile(full_path) and entry.endswith(".scene_instance.json"):
            file_list.append(full_path)
    return file_list[:MAX_FILES_TO_PROCESS]


def load_decomp_data(csv_filepath: str) -> Tuple[list, set]:
    data = []
    replaceable_uuids = set()
    with open(csv_filepath, mode="r", newline="", encoding="utf-8") as csvfile:
        csv_reader = csv.DictReader(csvfile)
        for row in csv_reader:
            if "reference" in row:
                replaceable_uuids.add(row["reference"])
            data.append(row)
    for row in data:
        ...
    return data, replaceable_uuids


def load_com_correction_data() -> list:
    data = []
    with open(COM_CORRECTION_PATH, mode="r", encoding="utf-8") as file:
        reader = csv.DictReader(file)
        for row in reader:
            data.append(row)
    return data


def scan_scene_for_objects(scene: str) -> Tuple[list, dict]:
    """
    Given a scene instance name, look at the objects and check against the set of replaceable UUIDs
    Returns:
    - a list of object dicts (inc transforms) to replace with decomps
    - a dict of the old scene with replaceable objects stripped out
    """
    global total_objects
    global total_replacements
    objects_to_replace = []
    with open(scene, "r") as file:
        scene_instance = json.load(file)
    # make a stub of the updated scene with all non-object data intact
    stripped_scene = deepcopy(scene_instance)
    stripped_scene["object_instances"] = []
    num_objects = len(scene_instance["object_instances"])
    for obj in scene_instance["object_instances"]:
        if obj["template_name"] in replaceable_uuids:
            # print(f"{obj['template_name']} needs replaced...")
            objects_to_replace.append(obj)
        else:
            stripped_scene["object_instances"].append(obj)
    num_replacements = len(objects_to_replace)
    print(
        f"{scene} - Found {num_replacements} replaceable objects out of {num_objects}."
    )
    total_objects += num_objects
    total_replacements += num_replacements
    # print(objects_to_replace)
    return objects_to_replace, stripped_scene


def replace_scene_object(original_template: dict, scene: dict) -> dict:
    """
    Append decomposed parts to the scene instance, matched to the corrected transform matrix of the original
    """
    global DECOMP_METADATA_PATH

    def get_metadata() -> dict:
        decomp_metadata_file = os.path.join(
            DECOMP_METADATA_PATH, base_uuid, (base_uuid + ".parts.metadata.json")
        )
        # print(f"loading {decomp_metadata_file}")
        with open(decomp_metadata_file, "r") as file:
            metadata = json.load(file)
        return metadata

    def get_object_tm(template: dict) -> mn.Matrix4:
        # print(template)
        # not all object instance in a scene instance have all transforms
        try:
            pos = template["translation"]
            pos_vec = mn.Vector3(pos[0], pos[1], pos[2])
            pos_matrix = mn.Matrix4.translation(pos_vec)
        except KeyError:
            pos_matrix = mn.Matrix4.identity_init()

        try:
            rot = template["rotation"]
            rot_quat = mn.Quaternion(
                (float(rot[0]), float(rot[1]), float(rot[2])), float(rot[3])
            )
            rot_matrix = mn.Matrix4.from_(
                (rot_quat.to_matrix()), mn.Vector3(0.0, 0.0, 0.0)
            )
        except KeyError:
            rot_matrix = mn.Matrix4.identity_init()

        try:
            nus = template["non_uniform_scale"]
            nus_vec = mn.Vector3(nus[0], nus[1], nus[2])
            nus_matrix = mn.Matrix4.scaling(nus_vec)
        except KeyError:
            nus_matrix = mn.Matrix4.identity_init()

        object_tm = pos_matrix @ rot_matrix @ nus_matrix
        # print(f"created object TM for {template['template_name']}:\ntranslation: {pos_matrix}\nrotation:{rot_matrix}\nscale:{nus_matrix}\ncombined:{object_tm}")
        return object_tm

    def get_prs_from_tm(tm: mn.Matrix4) -> dict:
        new_template = {}
        # translation
        translation_vector = tm.translation
        # rotation
        # need to handle negative scale values
        # and 0 scale
        rot_scale_matrix = tm.rotation_scaling()
        rotation_matrix_normalized = mn.Matrix3x3()
        scaling = mn.Vector3()

        determinant = rot_scale_matrix.determinant()

        columns = [rot_scale_matrix[0], rot_scale_matrix[1], rot_scale_matrix[2]]
        for i, col in enumerate(columns):
            scale_factor = col.length()
            # Handle the case where the scale factor is zero
            if scale_factor > 1e-6:
                scaling[i] = scale_factor
                rotation_matrix_normalized[i] = col / scale_factor
            else:
                scaling[i] = 0.0
                # If scale is zero, use a default identity basis vector
                if i == 0:
                    rotation_matrix_normalized[i] = mn.Vector3.x_axis()
                elif i == 1:
                    rotation_matrix_normalized[i] = mn.Vector3.y_axis()
                else:
                    rotation_matrix_normalized[i] = mn.Vector3.z_axis()

        if determinant < 0:
            # A negative determinant means an odd number of axes were negatively scaled.
            # We can enforce the negativity on one axis (e.g., X), and adjust the rotation
            # to compensate for the change in handedness.
            scaling[0] = -scaling[0]
            rotation_matrix_normalized[0] = -rotation_matrix_normalized[0]

        rotation = mn.Quaternion.from_matrix(rotation_matrix_normalized)
        rotation_vector4 = rotation.wxyz  # Quat to vector4

        # make dict entries
        new_template["translation"] = [
            translation_vector[0],
            translation_vector[1],
            translation_vector[2],
        ]
        new_template["rotation"] = [
            rotation_vector4[0],
            rotation_vector4[1],
            rotation_vector4[2],
            rotation_vector4[3],
        ]
        if abs((scaling.length()) - 1.0) < 0.001:
            new_template["non_uniform_scale"] = [1.0, 1.0, 1.0]
        else:
            new_template["non_uniform_scale"] = [scaling[0], scaling[1], scaling[2]]
        return new_template

    def get_com_correction(uuid: str) -> mn.Matrix4:
        """Get the COM correction to use for this object."""
        com_dict = [entry for entry in com_correction_data if entry["uuid"] == uuid][0]
        assert com_dict is not None
        com_vector = mn.Vector3(
            float(com_dict["x"]), float(com_dict["y"]), float(com_dict["z"])
        )
        return mn.Matrix4.translation(com_vector)

    def make_intermediate_object(
        part_metadata: dict, is_instance: bool = False
    ) -> dict:
        """translates the transform data and GLB reference from the part metadata into
        the same nomenclature used in the scene instance files"""
        new_obj = {}
        new_obj["translation"] = part_metadata["transform"]["position"]
        new_obj["rotation"] = part_metadata["transform"]["quaternion"]
        new_obj["non_uniform_scale"] = part_metadata["transform"]["scale"]
        new_obj["motion_type"] = "static"
        template_handle = base_uuid + "_part_"
        old_part_id = part_metadata["partId"]
        log_str = f"Replacing part {old_part_id} "

        if is_instance:
            ref_part_id = part_metadata["refPartId"]
            template_handle += ref_part_id
            log_str += f"(instance of {ref_part_id})..."
        else:
            template_handle += old_part_id
            log_str += "original..."

        new_obj["template_name"] = template_handle
        # print(log_str)
        return new_obj

    def filter_parts() -> list:
        parts_list = []
        for part in decomp_metadata["parts"]:
            part_uuid = base_uuid + "_part_" + str(part["partId"])
            print(f"checking part {part_uuid}... ", end="")

            if (
                next((d for d in decomp_list if d.get("uuid") == part_uuid), None)
                is not None
            ):
                # object is a base object found in the non-pickable decomp list
                print("FOUND original")
                parts_list.append(make_intermediate_object(part))

            elif part.get("isRef", "") is False:
                # object is an instance
                ref_uuid = base_uuid + "_part_" + part.get("refPartId", "")
                if (
                    next((d for d in decomp_list if d.get("uuid") == ref_uuid), None)
                    is not None
                ):
                    # instanced object's base object is in the non-pickable decomp list
                    print("FOUND instance")
                    parts_list.append(make_intermediate_object(part, is_instance=True))
                else:
                    print("skipping pickable (instance)")
                    ...
            else:
                print("skipping pickable (original)")
                ...
        # print(f"parts list: {parts_list}")
        return parts_list

    base_uuid = original_template["template_name"]
    decomp_metadata = get_metadata()
    scene_instance_transform = get_object_tm(original_template)

    keys = ["translation", "rotation", "non_uniform_scale"]
    printable_source_dict = {}
    for key, value in original_template.items():
        if key in keys:
            print(f"found {key}!")
            printable_source_dict[key] = value

    objects_to_add = filter_parts()

    for new_part in objects_to_add:
        print(f"Adding {new_part['template_name']}...")
        part_transform = get_object_tm(new_part)
        part_com_correction = get_com_correction(new_part["template_name"])
        # print(f"original part transform: {part_transform}")
        final_decomp_transform = (
            part_com_correction.inverted()
            @ scene_instance_transform
            @ part_transform
            @ part_com_correction
        )
        # print(f"Adjusted part transform: {part_transform}")
        part_transform_dict = get_prs_from_tm(final_decomp_transform)
        print(f"Original Transform: {printable_source_dict}")
        print(f"Part Transform: {part_transform_dict}")

        for key, value in part_transform_dict.items():
            # print(f"key: {key}\nvalue: {value}")
            new_part[key] = value

        scene["object_instances"].append(new_part)

    return scene


# main
total_objects = 0  # these are declared global in scan_scene_for_objects
total_replacements = 0

scene_list = collect_scenes(SCENE_DIR)

# load CSV data and enrich with parts metadata instances/transforms
decomp_list, replaceable_uuids = load_decomp_data(DECOMP_DATA_PATH)
com_correction_data = load_com_correction_data()
for scene in scene_list:
    objects_to_replace, new_scene = scan_scene_for_objects(scene)
    for obj in objects_to_replace:
        print(f"replacing {obj['template_name']}...")
        new_scene = replace_scene_object(obj, new_scene)
    # write new scene instance to target dir
    out_path = os.path.join((SCENE_DIR + "_v2"),os.path.basename(scene))
    with open(out_path, "w") as json_file:
        json.dump(new_scene, json_file, indent=4)

print(
    f"\nReplaced {total_replacements} out of {total_objects} objects across {len(scene_list)} scenes."
)
