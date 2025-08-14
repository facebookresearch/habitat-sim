import bpy
import json
from typing import Any, Dict, List
from pathlib import Path

def purge_unused_data():
    """Remove all mesh, image, and collection data that are not used in the scene."""
    for collection in bpy.data.collections:
        if not collection.objects and not collection.children:
            bpy.data.collections.remove(collection)
    for mesh in bpy.data.meshes:
        if not mesh.users:
            bpy.data.meshes.remove(mesh)
    for image in bpy.data.images:
        if not image.users:
            bpy.data.images.remove(image)

def open_json(json_path: Path) -> Dict[str, Any]:
    if not json_path.exists():
        raise FileNotFoundError(f"JSON file not found: {json_path}")
    with open(json_path, 'r') as file:
        data = json.load(file)
    return data

def swap_axes(data: Dict[str, Any]) -> Dict[str, Any]:
    transform = {"position": [0, 0, 0],
                 "quaternion": [1, 0, 0, 0],
                 "scale": [1, 1, 1]}
    if "position" in data:
        position = data["position"]
        transform["position"] = [position[0], -position[2], position[1]]
    if "quaternion" in data:
        quaternion = data["quaternion"]
        # blender wants WXYZ in Z-up, Habitat JSON is Y-up as XYZW, so we swap a bunch of axes here  
        transform["quaternion"] = [quaternion[3], quaternion[0], quaternion[2], quaternion[1]] 
    if "scale" in data:
        scale = data["scale"]
        transform["scale"] = [scale[0], scale[2], scale[1]]
    else:
        print("No transform found in data, using default.")
    return transform

def place_object(obj: bpy.types.Object, transform: Dict[str, Any]) -> None:
    """Place the object in the scene based on the provided transform."""
    obj.location = transform.get("position", (0, 0, 0))
    obj.rotation_quaternion = transform.get("quaternion", (1, 0, 0, 0))
    obj.scale = transform.get("scale", (1, 1, 1))

def make_collection(name: str, color: str, parent: bpy.types.Collection) -> bpy.types.Collection:
    if name in bpy.data.collections:
        return bpy.data.collections[name]
    new_collection = bpy.data.collections.new(name)
    new_collection.color_tag = color #this is an enum from bpy.types import ColorTag
    bpy.context.scene.collection.children.link(new_collection) # Link to the scene collection
    # If a parent is specified, link to it as well
    bpy.context.scene.collection.children.unlink(new_collection)
    parent.children.link(new_collection)
    return new_collection

def relink_object_to_collection(
    obj: List[bpy.types.Object], 
    collection: bpy.types.Collection, 
    remove_from_root: bool = True
    ) -> None:
    """Relink an object to a specified collection."""
    for o in obj:
        if remove_from_root:
            bpy.context.scene.collection.objects.unlink(o)
        collection.objects.link(o)  # Link to the new collection

def make_decomp_from_json(uuid: str) -> None:
    """Create a Blender scene from a decomposed JSON file."""
    json_path = Path(f"/home/jseagull/dev/fp-models/objects/decomposed/{uuid}/{uuid}.parts.metadata.json")
    data = open_json(json_path)
    part_root = json_path.parents[0] 
    file_prefix = json_path.name.split('.')[0]
    part_prefix = f"{file_prefix}_part_"
    prefix = data.get("id", "")

    parent_collection = make_collection(file_prefix, "NONE", bpy.context.scene.collection)
    dedup_collection = make_collection(file_prefix + "_dedup", "COLOR_05", bpy.context.scene.collection)

    # Import dedup file for comparison
    existing_objs = set(bpy.context.scene.objects)
    dedup_file = part_root / f"{file_prefix}_dedup.glb"
    if not dedup_file.exists():
        raise FileNotFoundError(f"Dedup file not found: {dedup_file}")
    bpy.ops.import_scene.gltf(filepath=str(dedup_file))
    new_objs = set(bpy.context.scene.objects) - existing_objs
    relink_object_to_collection(new_objs, dedup_collection)

    # Import and place unique objects
    for obj in data.get("parts", {}):
        if obj.get("isRef") is not False:  # True or None - actual mesh
            part_Id = obj.get("partId", "")
            transform_dict = swap_axes(obj.get("transform", {}))
            existing_objs = set(bpy.context.scene.objects)
            part_file = part_root / f"{part_prefix}{part_Id}.glb"

            bpy.ops.import_scene.gltf(filepath=str(part_file))
            new_objs = set(bpy.context.scene.objects) - existing_objs
            part_collection = make_collection(f"part_{part_Id}", "COLOR_04", parent_collection)
            relink_object_to_collection(new_objs, part_collection)
            for new_obj in new_objs:
                if transform_dict:
                    place_object(new_obj, transform_dict)

    # Clone and place instances now that all meshes exist
    for obj in data.get("parts", {}):
        if obj.get("isRef") is False:  # These are instances
            part_Id = obj.get("partId", "")
            ref_Id = obj.get("refPartId", "")
            transform_dict = swap_axes(obj.get("transform", {}))
            ref_collection = bpy.data.collections.get(f"part_{ref_Id}")
            if ref_collection:
                part_collection = make_collection(f"part_{part_Id}", "COLOR_03", parent_collection)
                for obj in ref_collection.objects:
                    new_obj = obj.copy()
                    relink_object_to_collection([new_obj], part_collection, False)
                    if transform_dict:
                        place_object(new_obj, transform_dict)
            else:
                raise ValueError(f"Reference collection for part {ref_Id} not found.")
    print(f"Decomposed scene created from {json_path.name} with prefix {prefix}.")
    
if __name__ == "__main__":
    purge_unused_data() #otherwise names get appended when testing
    uuid = "077b4aad95f9bcb3954d0ce719605f6a08cd253f"
    make_decomp_from_json(uuid)