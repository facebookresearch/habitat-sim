#!/usr/bin/env python3

# Copyright (c) Meta Platforms, Inc. and affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import csv
import json
from typing import Callable, Dict, List, Union

from habitat_sim.attributes import ArticulatedObjectAttributes, ObjectAttributes
from habitat_sim.metadata import MetadataMediator

# Object Semantics Writer Script
# This program reads a metadata .csv file containing a mapping of object handles (i.e., hashes or names)
# to semantic classes (e.g. "lamp" or "bed"), enumerates the classes and then writes the class index for
# each object in its respective object config file to support semantic rendering.

# this is the common index of un-identified objects
UNKNOWN_INDEX = 0

HSSD_DATASET_DIR = "data/scene_datasets/floorplanner_ss/"


def load_object_metadata(obj_metadata_csv: str) -> Dict[str, List[str]]:
    """
    Reads a csv file mapping object handles (i.e. hashes) to semantic classes.
    Returns a Dict mapping all semantic classes to their complete set of member object handles.
    """
    # load object affordances metadata
    obj_classes_to_handles: Dict[str, List[str]] = {}
    with open(obj_metadata_csv, "r", newline="") as csvfile:
        csvreader = csv.reader(csvfile, delimiter=",")
        is_first = True
        for row in csvreader:
            if is_first:
                is_first = False
                continue
            obj_handle = row[0]
            obj_class = row[3].replace("/", "_").replace("#", "")  # the condensed class
            if obj_class not in obj_classes_to_handles:
                obj_classes_to_handles[obj_class] = []
            obj_classes_to_handles[obj_class].append(obj_handle)
    return obj_classes_to_handles


def assign_indices_to_classes(
    obj_classes_to_handles: Dict[str, List[str]]
) -> Dict[str, int]:
    """
    Processes the provided object classes and assigns indices to each.
    NOTE: 'unknown' is a special case. All classes with 'unknown' in the string are condensed to index 0.
    NOTE: 'N_A' is condensed to 'unknown' at index 0.
    """
    next_index = UNKNOWN_INDEX + 1
    classes_to_indices = {}
    classes_sorted = sorted(obj_classes_to_handles.keys())
    for obj_class in classes_sorted:
        if "unknown" in obj_class or obj_class == "N_A":
            classes_to_indices[obj_class] = UNKNOWN_INDEX
        else:
            classes_to_indices[obj_class] = next_index
            next_index += 1
    return classes_to_indices


def write_lexicon(
    semantic_classes_to_indices: Dict[str, int],
    out_path: str = f"{HSSD_DATASET_DIR}semantics/hssd-hab_semantic_lexicon.json",
) -> None:
    """
    Writes a semantic lexicon JSON file compatible with Habitat SceneDataset format.
    This file maps semantics classes to indices which are then embedded in object configs.

    Format:
    {
    "classes": [
      {
        "name": "alarm_clock",
        "id": 1
      },
      ...
      ]}
    """

    lexicon = {"classes": [{"name": "unknown", "id": UNKNOWN_INDEX}]}

    for sem_class, index in semantic_classes_to_indices.items():
        if index != UNKNOWN_INDEX:
            lexicon["classes"].append({"name": sem_class, "id": index})

    # write the lexicon to a file
    with open(out_path, "w") as f:
        f.write(json.dumps(lexicon, sort_keys=True, indent=4))


def set_attr_semantic_ids(
    attributes: List[Union[ObjectAttributes, ArticulatedObjectAttributes]],
    semantic_classes_to_indices: Dict[str, int],
    semantic_classes_to_handle: Dict[str, List[str]],
    register_callback: Callable = None,
) -> None:
    """
    Set the semantic_id property within the Attributes object.
    """
    handles_to_indices = {
        handle: semantic_classes_to_indices[sem_class]
        for sem_class, handles in semantic_classes_to_handle.items()
        for handle in handles
    }
    print("Setting semantics: ")
    for attr in attributes:
        handle = attr.handle.split("/")[-1].split(".")[0]
        if handle in handles_to_indices:
            attr.semantic_id = handles_to_indices[handle]
            print(f"     {handle} : {attr.semantic_id}")
            if register_callback is not None:
                register_callback(attr)
        else:
            print(
                f"    - Attributes '{handle}' not in the semantic registry, skipping. Full name {attr.handle}"
            )


def main(dataset: str, obj_metadata_csv: str):
    # aggregate semantics from csv file
    obj_semantics = load_object_metadata(obj_metadata_csv)
    class_indices = assign_indices_to_classes(obj_semantics)
    # write the lexicon file
    write_lexicon(class_indices)

    mm = MetadataMediator()
    mm.active_dataset = dataset
    otm = mm.object_template_manager
    aotm = mm.ao_template_manager
    print(f" rigid object templates: {len(otm.get_file_template_handles())}")
    print(f" articulated object templates: {len(aotm.get_template_handles())}")
    print(f" urdf paths: {len(mm.urdf_paths)}")

    # Handle the rigids
    set_attr_semantic_ids(
        otm.get_templates_by_handle_substring().values(),
        class_indices,
        obj_semantics,
        register_callback=otm.register_template,
    )
    # resave the templates back to file
    for handle in otm.get_file_template_handles():
        otm.save_template_by_handle(handle, True)

    # Handle the AOs
    set_attr_semantic_ids(
        aotm.get_templates_by_handle_substring().values(),
        class_indices,
        obj_semantics,
        register_callback=aotm.register_template,
    )
    # resave the templates back to file
    for handle in aotm.get_template_handles():
        aotm.save_template_by_handle(handle, True)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()

    # optional arguments
    parser.add_argument(
        "--dataset",
        default=f"{HSSD_DATASET_DIR}hssd-hab-siro-wip.scene_dataset_config.json",
        type=str,
        help=f'scene dataset file to load (default: "{HSSD_DATASET_DIR}hssd-hab-siro-wip.scene_dataset_config.json"',
    )

    # required arguments
    parser.add_argument(
        "--semantic-csv",
        default=f"{HSSD_DATASET_DIR}hssd_obj_semantics_condensed.csv",
        type=str,
        help=f'csv file containing the mapping from object handles to semantic classes (default : "{HSSD_DATASET_DIR}hssd_obj_semantics_condensed.csv")',
    )

    args = parser.parse_args()

    # run the program
    main(dataset=args.dataset, obj_metadata_csv=args.semantic_csv)
