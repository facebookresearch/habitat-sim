try:
    import bpy
except ImportError:
    raise ImportError(
        "Failed to import Blender modules. This script can't run "
        "standalone. Run `blender --python path/to/blender_load_replay.py ...`. Watch the terminal for "
        "debug/error output."
    )

import argparse
import json
import os
from dataclasses import dataclass


@dataclass
class ImportItem:
    filepath: str
    do_join_all: bool = True
    force_color: list = None


def import_scene_helper(raw_filepath):
    # these fixups will be tried one at a time, in order, until a matching, existing file is found
    model_filepath_fixups = [
        # use uncompressed versions when available
        ("/stages/", "/stages_uncompressed/"),
        ("/urdf/", "/urdf_uncompressed/"),
        # ("./data/robots/hab_fetch/robots/../meshes/", "/home/eundersander/projects/blender/fetch_meshes_modified/")
        # for Fetch, avoid blender .dae import because it seems to be broken; fall back to collision STL
        (".dae", "_collision.STL"),
        # sometimes the collision STL doesn't have _collision in the name
        (".dae", ".STL"),
        # the null fixup (use model filepath as-is)
        None,
    ]

    filepath = None
    for fixup in model_filepath_fixups:
        if fixup is None:
            filepath = raw_filepath
        else:
            if raw_filepath.find(fixup[0]) != -1:
                filepath = raw_filepath.replace(fixup[0], fixup[1])
            else:
                continue
        if os.path.exists(filepath):
            break
    else:
        raise RuntimeError("can't find file " + raw_filepath)

    ext = os.path.splitext(filepath)[1].lower()
    if ext == ".glb" or ext == ".gltf":
        filename = os.path.basename(filepath)
        bpy.ops.import_scene.gltf(
            filepath=filepath, files=[{"name": filename, "name": filename}], loglevel=50
        )
    elif ext == ".obj":
        bpy.ops.import_scene.obj(filepath=filepath)
    elif ext == ".dae":
        bpy.ops.wm.collada_import(filepath=filepath)
    elif ext == ".stl":  # noqa: SIM106
        bpy.ops.import_mesh.stl(filepath=filepath)
    else:
        raise RuntimeError("no importer found for " + filepath)

    return filepath


def import_item(item):

    adjusted_filepath = import_scene_helper(item.filepath)

    bpy.ops.object.select_all(action="SELECT")
    bpy.context.view_layer.objects.active = bpy.context.selected_objects[0]

    childless_empties = [
        e
        for e in bpy.context.selected_objects
        if e.type.startswith("EMPTY") and not e.children
    ]
    if len(childless_empties):
        print("removing {} childless EMPTY nodes".format(len(childless_empties)))
        while childless_empties:
            bpy.data.objects.remove(childless_empties.pop())
        bpy.ops.object.select_all(action="SELECT")
        bpy.context.view_layer.objects.active = bpy.context.selected_objects[0]

    if item.do_join_all:
        if len(bpy.context.selected_objects) > 1:
            bpy.ops.object.join()
            bpy.ops.object.select_all(action="SELECT")
        o = bpy.context.selected_objects[0]
        bpy.context.view_layer.objects.active = bpy.context.selected_objects[0]
        bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
        filename = os.path.basename(item.filepath)
        joined_name = os.path.splitext(filename)[0]
        o.name = joined_name
        o.data.name = joined_name

        # fixup for STL importer
        ext = os.path.splitext(adjusted_filepath)[1].lower()
        if ext == ".stl":
            o.rotation_mode = "XYZ"
            o.rotation_euler[0] += 1.5708
            bpy.ops.object.transform_apply(location=False, rotation=True, scale=False)

    # currently unused for here for reference, in case we add color override to gfx-replay
    if item.force_color:
        o = bpy.context.selected_objects[0]
        mtrl = o.data.materials[0]
        mtrl.node_tree.nodes["Principled BSDF"].inputs[0].default_value = (
            item.force_color[0],
            item.force_color[1],
            item.force_color[2],
            1,
        )

    retval = bpy.context.selected_objects[0]

    for o in bpy.context.selected_objects:
        o.hide_set(True)

    return retval


def import_gfx_replay(replay_filepath):

    with open(replay_filepath, "r") as f:
        json_root = json.load(f)
        assert "keyframes" in json_root
        keyframes = json_root["keyframes"]
        assert len(keyframes) > 0

    render_asset_map = {}
    asset_info_by_filepath = {}
    asset_info_by_key = {}

    do_add_anim_keyframes = len(keyframes) > 1
    for keyframe_index, keyframe in enumerate(keyframes):

        if "loads" in keyframe:
            for asset_info in keyframe["loads"]:
                filepath = asset_info["filepath"]
                asset_info_by_filepath[filepath] = asset_info

        if "creations" in keyframe:
            for creation_dict in keyframe["creations"]:
                filepath = creation_dict["creation"]["filepath"]
                obj = import_item(ImportItem(filepath))
                if "scale" in creation_dict["creation"]:
                    obj.scale = creation_dict["creation"]["scale"]
                instance_key = creation_dict["instanceKey"]
                render_asset_map[instance_key] = obj
                asset_info_by_key[instance_key] = asset_info_by_filepath[filepath]

        if "stateUpdates" in keyframe:
            for update_dict in keyframe["stateUpdates"]:
                instance_key = update_dict["instanceKey"]
                translation = update_dict["state"]["absTransform"]["translation"]
                rotation = update_dict["state"]["absTransform"]["rotation"]
                obj = render_asset_map[instance_key]

                obj.rotation_mode = "QUATERNION"

                asset_info = asset_info_by_key[instance_key]

                # note coordinate convention change for Blender
                obj.location = (translation[0], -translation[2], translation[1])
                obj.rotation_quaternion = (
                    rotation[0],
                    rotation[1],
                    -rotation[3],
                    rotation[2],
                )

                frame = asset_info["frame"]
                if frame["up"] == [0.0, 1.0, 0.0]:
                    pass
                elif frame["up"] == [0.0, 0.0, 1.0]:
                    obj.rotation_mode = "XYZ"
                    obj.rotation_euler[0] -= 1.5708
                else:
                    raise NotImplementedError("unexpected coordinate frame " + frame)

        if do_add_anim_keyframes:
            for instance_key in render_asset_map:
                obj = render_asset_map[instance_key]
                obj.keyframe_insert(data_path="location", frame=keyframe_index)
                obj.keyframe_insert(
                    data_path="rotation_quaternion", frame=keyframe_index
                )

    for o in bpy.context.scene.objects:
        o.hide_set(False)

    print("")
    if len(keyframes) > 1:
        print(
            "Success! Imported {} with {} render instances and {} animation keyframes.".format(
                replay_filepath, len(render_asset_map), len(keyframes)
            )
        )
    else:
        print(
            "Success! Imported {} with {} render instances (no animation found)".format(
                replay_filepath, len(render_asset_map)
            )
        )
    print("")
    print(
        "Explore the Blender GUI window to visualize your replay, then close it when done."
    )


def main(replay_filepath, root_dir):
    os.chdir(root_dir)  # todo: get working directory from the replay, itself
    import_gfx_replay(replay_filepath)


if __name__ == "__main__":

    import sys

    argv = sys.argv
    argv = argv[argv.index("--") + 1 :]  # get all args after "--"

    parser = argparse.ArgumentParser()
    parser.add_argument("--replay", type=str, required=True)
    parser.add_argument("--root-dir", type=str, required=True)
    args = parser.parse_args(argv)

    main(args.replay, args.root_dir)
