import json
import math

import bpy
import mathutils

# define the output directory for the metadata dump
output_directory = "/home/alexclegg/Documents/VR_mocap_resources/"
current_filepath = bpy.data.filepath
assert current_filepath != "", "Cannot be default file, need to have a blend filename."
output_filepath = (
    output_directory + current_filepath.split("/")[-1].split(".")[0] + ".json"
)
print(f"Saving metadata to {output_filepath}")

# construct the metadata dict
metadata = {"agent_spawns": [], "target_volumes": []}

# define the tags
agent_spawn_tag = "agent_spawn_volume_"
target_spawn_tag = "spawn_volume_"

for obj in bpy.context.scene.objects:
    if target_spawn_tag in obj.name:
        # NOTE: rotate the scene from Blender to Habitat coordinate system
        to_hab = mathutils.Quaternion((1.0, 0.0, 0.0), math.radians(-90.0))
        object_rotation = obj.rotation_quaternion.copy()
        object_rotation.rotate(to_hab)
        object_translation = obj.location.copy()
        object_translation.rotate(to_hab)

        volume_instance_data = {
            "translation": object_translation[:],
            # NOTE: quaternion is saved in [w,x,y,z] format
            "rotation": object_rotation[:],
            "size": obj.scale[:],
        }
        if agent_spawn_tag in obj.name:
            volume_instance_data["name"] = obj.name.split(agent_spawn_tag)[-1]
            metadata["agent_spawns"].append(volume_instance_data)
        else:
            volume_instance_data["name"] = obj.name.split(target_spawn_tag)[-1]
            metadata["target_volumes"].append(volume_instance_data)

# write metadata to json file
with open(output_filepath, "w") as outfile:
    json.dump(metadata, outfile, indent=4)
