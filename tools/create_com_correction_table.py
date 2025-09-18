import argparse
import glob
import os.path
import csv
import gc
import magnum as mn

from habitat_sim import Simulator
from habitat_sim.metadata import MetadataMediator
from habitat_sim.physics import ManagedRigidObject

from habitat_sim.utils.settings import default_sim_settings, make_cfg


def register_glb(asset_path: str) -> None:
    """
    Registers a GLB file to a template in the library so it can be added to the scene.
    The registered GLB will have a library handle matching its GLB filename without extension
    """
    library_handle = os.path.splitext(os.path.basename(asset_path))[0]
    glb_template = template_manager.create_new_template(handle=asset_path)
    assert glb_template.has_value("render_asset")
    template_id = template_manager.register_template(
        template=glb_template, specified_handle=library_handle
    )
    return library_handle
    # print(f"Registered {leaf} as {glb_template.handle} with library ID {template_id}")


def get_com_correction(obj_handle:str)-> mn.Vector3:
    """
    Loads object and returns its com correction
    """
    obj = rom.add_object_by_template_handle(object_lib_handle=obj_handle)
    cc = obj.com_correction
    rom.remove_object_by_handle(handle=obj.handle)
    return cc

# Declarations

START_ITEM = 0
CHUNK_SIZE = 100

object_data = {}

if __name__ == "__main__":
    argparser = argparse.ArgumentParser()
    argparser.add_argument(
        "-d",
        "--dataset",
        type=str,
        default="hssd-hab",
        help="dataset to look for objects",
        action="store",
    )
    argparser.add_argument(
        "-o",
        "--output_folder",
        type=str,
        default="/home/jseagull/dev/fphab/objects",
        help="destination folder for output CSV",
        action="store",
    )
    argparser.add_argument(
        "-m",
        "--max_items",
        type=int,
        default=25000,
        help="number of models to record (for testing)",
        action="store",
    )
    args = argparser.parse_args()

    # start logging
    assert os.path.exists(args.output_folder)
    output_path = f"{args.output_folder}/com_correction.csv"
    header_row = ["uuid", "x", "y", "z"]
    with open(output_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(header_row)

    # gather files
    full_glb_iter = glob.iglob(
        "/home/jseagull/dev/fphab/objects/**/*.glb", recursive=True
    )
    exclude_patterns = [".collider.glb", ".filteredSupportSurface.glb", "openings"]
    # truncate glb list at max_items
    full_glb_list = [
        f
        for f in full_glb_iter
        if not any(pattern in f for pattern in exclude_patterns)
    ][0 : args.max_items]

    # init simulator
    sim_settings = default_sim_settings.copy()
    sim_settings["width"] = sim_settings["height"] = 256
    sim_settings["enable_hbao"] = False  # Ambient Occlusion
    sim_settings["default_agent_navmesh"] = False
    sim_settings["scene_dataset_config_file"] = (
        args.dataset
    )  # sim and mm both need to know about the dataset

    cfg = make_cfg(sim_settings)
    mm = MetadataMediator()
    mm.active_dataset = args.dataset
    cfg.metadata_mediator = mm
    template_manager = mm.object_template_manager

    print("\n######## Initializing Assets... ########################\n")
    print(f"active dataset: {mm.active_dataset}")
    print(f"Dataset exists? {mm.dataset_exists(args.dataset)}")
    print(f"Populating {len(full_glb_list)} templates...")
    handle_list = []
    for glb in full_glb_list:
        # register the objects in the library, and add the handle to our list
        handle_list.append(register_glb(glb))
    # chunk the handle list for memory management
    chunked_handle_list = []
    for i in range(START_ITEM, len(handle_list), CHUNK_SIZE):
        chunked_handle_list.append(handle_list[i : i + CHUNK_SIZE])

    print("\n######## Starting Simulator... #########################\n")
    icount = 0
    for uuid_sublist in chunked_handle_list:
        with Simulator(cfg) as sim:
            print("")
            rom = sim.get_rigid_object_manager()
            chunk_rows = []
            for obj in uuid_sublist:
                obj_cc = get_com_correction(obj)
                row = [obj, obj_cc.x, obj_cc.y, obj_cc.z]
                chunk_rows.append(row)
            with open(output_path, "a", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerows(chunk_rows)
            sim.close()
            icount += 1
            print(f"wrote chunk {icount} of {len(chunked_handle_list)}")
            gc.collect()