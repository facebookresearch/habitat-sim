import argparse
import json
import os
from typing import Any, Dict, List, Optional

import magnum as mn


def build_parser(
    parser: Optional[argparse.ArgumentParser] = None,
) -> argparse.ArgumentParser:
    if parser is None:
        parser = argparse.ArgumentParser(
            description="Tool for converting and processing json files.",
            formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        )

    parser.add_argument("--input", required=True, type=str, help="Input JSON file.")

    parser.add_argument(
        "--output",
        type=str,
        default="data/json_processor_output/",
        help="Output JSON directory.",
    )

    # Options for operation:
    # - "make_pretty" - parse and re-export the JSON to a nice, human readable format
    # - "hw_to_hab" - convert HW json into a hab scene dataset
    # - "hw_object_configs" - generate object configs for HW assets (note, some post-processing must be done manually)
    parser.add_argument(
        "--operation",
        default="make_pretty",
        type=str,
        help="The operation to apply to the JSON.",
    )

    return parser


def pretty_convert_json(input_path, output_path):
    assert output_path.endswith(".json"), "Must provide a json file output path."
    with open(input_path, "r") as handle:
        parsed = json.load(handle)
        with open(output_path, "w") as outfile:
            print(f"Writing pretty json to {output_path}")
            json.dump(parsed, outfile, indent=4, sort_keys=True)


def create_hw_prim_config(asset_filepath):
    """
    Generate a config file to accompany a HW asset.
    """
    asset_filename = asset_filepath.split("/")[-1]
    output_path = asset_filepath.split(".")[0] + ".object_config.json"
    object_config = {
        "render_asset": asset_filename,
        "up": [0.0, 1.0, 0.0],
        "front": [0.0, 0.0, -1.0],
        "COM": [0, 0, 0],
    }
    # NOTE: primitive_ramp.object_config.json should include:
    # "up": [-1.0, 0.0, 0.0],
    # "front": [0.0, 0.0, -1.0],
    with open(output_path, "w") as outfile:
        print(f"writing object config to '{output_path}'")
        json.dump(object_config, outfile, indent=4, sort_keys=False)


def generate_object_configs(input_directory, file_type=".glb"):
    """
    Generate object configs to accompany all assets in a directory with the designated type.
    """
    for item in os.listdir(input_directory):
        item_path = os.path.join(input_directory, item)
        if item_path.endswith(file_type):
            create_hw_prim_config(item_path)


def create_base_hw_scenedataset_config(output):
    """
    Create an empty SceneDatasetConfig for HW worlds to dump.
    """
    scene_dataset_config = {
        "scene_instances": {"paths": {".json": ["configs/scenes"]}},
        # NOTE: this is a back reference to a standard set of objects.
        "objects": {"paths": {".json": ["../../hw_prims"]}},
    }

    with open(output, "w") as outfile:
        print(f"writing HW SceneDataset to '{output}'")
        json.dump(scene_dataset_config, outfile, indent=4, sort_keys=True)


def create_hw_scenedataset_directories(output_dir):
    """
    Create the correct directory structure in a target directory if it does not already exist.
    """
    print(f"Creating a SceneDataset for HW in '{output_dir}'")
    os.makedirs(output_dir, exist_ok=True)
    # scene configs
    os.makedirs(os.path.join(output_dir, "configs/scenes"), exist_ok=True)


def hw_type_to_obj_handle(hw_type):
    """
    Construct the asset name from the hw asset path.
    """
    obj_handle = hw_type.split("\\")[-1]
    obj_handle = "hw_prims/" + obj_handle
    return obj_handle


def get_hw_ground_plane_instance():
    """
    Construct the default HW ground plane metadata.
    """
    object_instance = {
        "template_name": "hw_prims/primitive_cube",
        "translation": [0, -0.1, 0],
        "non_uniform_scale": [100.0, 0.1, 100.0],
        "motion_type": "STATIC",
    }
    return object_instance


def convert_hw_to_hab(input_path, output):
    """
    Convert a Horizon Worlds json to a Habitat SceneDataset config directory with same name in output directory.

    Organization of HW JSON:
     - Top level object containing "entities" list and version info
     - "entities" list contains Dict objects with "type" key:
      - "SpawnPoint" entity: contains info about player (agent?) spawning.
      - "StaticLocalMesh" entity contains info about a mesh for the stage.
    """

    # need to transform from Horizon to Habitat coordinate space
    hw_to_hab = mn.Quaternion.rotation(mn.Rad(mn.math.pi), mn.Vector3(0, 0, 1))

    scene_id = input_path.split("/")[-1].split(".")[0]

    # first perform initial setup
    create_hw_scenedataset_directories(os.path.join(output, scene_id))
    create_base_hw_scenedataset_config(
        os.path.join(output, scene_id + "/" + scene_id + ".scene_dataset_config.json")
    )

    # now create a scene instance
    object_instances = [get_hw_ground_plane_instance()]

    with open(input_path, "r") as handle:
        hw_json = json.load(handle)
        entities = hw_json["entities"]
        print("Entities: ")
        for entity_info in entities:
            print(f"    - {entity_info['type']}")
            if entity_info["type"] == "StaticLocalMesh":
                # this is a static object instance, process it into the SceneInstance

                # TODO: cache or communicate the object name?
                # object_name = entity_info["id"]

                object_type = None
                object_transform = {}
                # objects have a list of component dicts including meshes and materials
                components: List[Dict[str:Any]] = entity_info["components"]
                for component in components:
                    if component["type"] == "Mesh":
                        #'Mesh' contains the primitive type string
                        object_type = component["props"][0]["value"]
                    elif component["type"] == "Transform":
                        #'Transform' contains the position, rotation, scale
                        # NOTE: converting to Habitat scene_instance config format
                        props: List[Dict[str:Any]] = component["props"]
                        for property_info in props:
                            if property_info["name"] == "position":
                                position = [
                                    float(x) for x in property_info["value"].split("\n")
                                ]
                                position = mn.Vector3(position)
                                # flip the camera forward axis (horizon is +Z forward)
                                position[2] *= -1
                                object_transform["translation"] = list(position)
                            elif property_info["name"] == "rotation":
                                # NOTE: rotation convention in HW is xyzw, hab expects wxyz
                                naive_rotation = [
                                    float(x) for x in property_info["value"].split("\n")
                                ]
                                hab_rotation = mn.Quaternion(
                                    mn.Vector3(naive_rotation[:-1]), naive_rotation[-1]
                                )
                                # simulate a reflection
                                hab_rotation = hw_to_hab * hab_rotation * hw_to_hab
                                object_transform["rotation"] = [hab_rotation.scalar]
                                object_transform["rotation"].extend(
                                    list(hab_rotation.vector)
                                )
                            elif property_info["name"] == "scale":
                                scale = [
                                    float(x) for x in property_info["value"].split("\n")
                                ]
                                object_transform["non_uniform_scale"] = scale
                    # elif component["type"] == "MeshColor":
                    #    pass
                    # TODO: need material editing to support this
                    # TODO: 'materialColor' property
                    # elif component["type"] == "MeshTexture":
                    #    pass
                    # TODO: need material/texture editing to support this
                    # elif component["type"] == "Visibility":
                    #    pass
                    # TODO: 'hidden' property
                    # elif component["type"] == "CollisionEnabled":
                    #    pass
                    # TODO: "collisionDisabled" property
                    # TODO: "collisionLayer" property
                obj_handle = hw_type_to_obj_handle(object_type)
                if obj_handle:
                    # construct the instance
                    object_instance = {
                        "template_name": obj_handle,
                        "translation": object_transform["translation"],
                        "rotation": object_transform["rotation"],
                        "non_uniform_scale": object_transform["non_uniform_scale"],
                        # TODO: get dynamics from somewhere?
                        "motion_type": "STATIC",
                    }
                    object_instances.append(object_instance)
                else:
                    print(
                        f"Can't process object type {object_type}, not implemented. Skipping..."
                    )

    # after looping all the objects, construct the scene instance
    scene_instance = {
        "stage_instance": {"template_name": "NONE"},
        "object_instances": object_instances,
    }
    scene_instance_filepath = os.path.join(
        output, scene_id + "/configs/scenes/" + scene_id + ".scene_instance.json"
    )
    with open(scene_instance_filepath, "w") as outfile:
        print(f"writing HW SceneInstance to '{output}'")
        json.dump(scene_instance, outfile, indent=4, sort_keys=True)


def main():
    args = build_parser().parse_args()

    if not os.path.exists(args.output):
        print(f"Output directory {args.output} does not exist, creating.")
        os.makedirs(args.output)

    if args.operation == "make_pretty":
        assert args.input.endswith(".json"), "Must provide a json file."
        assert not os.path.isfile(args.output), "Output should be a directory."
        output_file = os.path.join(args.output, args.input.split("/")[-1])
        pretty_convert_json(args.input, output_file)
    elif args.operation == "hw_to_hab":
        assert args.input.endswith(".json"), "Must provide a json file."
        assert not os.path.isfile(args.output), "Output should be a directory."
        convert_hw_to_hab(args.input, args.output)
    elif args.operation == "hw_object_configs":
        assert not os.path.isfile(args.input), "Input should be a directory."
        generate_object_configs(args.input)


if __name__ == "__main__":
    main()
