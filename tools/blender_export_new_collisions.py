import xml.etree.ElementTree as ET
from xml.dom import minidom

import bpy
import mathutils

output_filename = (
    "/run/media/alexclegg/Extreme_SSD/datasets/fremont/urdf/collision_output.xml"
)


def clean_vector_string(vector):
    return str(list(vector)).replace("[", "").replace("]", "").replace(",", "")


root = ET.Element("root")
for obj in bpy.context.scene.objects:
    if "collision_box_shape_new" in obj.name:
        collision = ET.SubElement(root, "collision")
        origin = ET.SubElement(collision, "origin")
        origin.set("rpy", clean_vector_string(obj.rotation_euler))
        origin.set("xyz", clean_vector_string(obj.location))
        geometry = ET.SubElement(collision, "geometry")
        # TODO: add more than boxes...
        box = ET.SubElement(geometry, "box")
        box.set("size", clean_vector_string(obj.scale))


xmlstr = minidom.parseString(ET.tostring(root)).toprettyxml(indent="   ")
with open(output_filename, "w") as f:
    f.write(xmlstr)
