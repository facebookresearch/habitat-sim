import json

import bpy
import mathutils

output_filename = (
    "/run/media/alexclegg/Extreme_SSD/datasets/fremont/urdf/receptacle_output.json"
)


def clean_vector_string(vector):
    return str(list(vector)).replace("[", "").replace("]", "").replace(",", "")


user_defined = {}
for obj in bpy.context.scene.objects:
    if "receptacle_" in obj.name:
        receptacle_info = {}
        receptacle_info["name"] = obj.name

        # get top level parent
        # top_parent = obj.parent
        # while top_parent.parent is not None:
        #    top_parent = top_parent.parent
        # NOTE: hardcoded for now
        receptacle_info["parent_object"] = "kitchen_island"

        receptacle_info["parent_link"] = obj.parent.name.split("link_")[-1]
        receptacle_info["position"] = list(obj.location)

        # NOTE: need half-extents for the final size
        receptacle_info["scale"] = list(obj.scale * 0.5)

        # NOTE: default hardcoded value for now
        receptacle_info["up"] = [0, 1, 0]

        user_defined[obj.name] = receptacle_info

with open(output_filename, "w") as f:
    json.dump(user_defined, f, indent=4)
