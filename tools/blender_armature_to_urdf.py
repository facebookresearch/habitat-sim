# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import os
import xml.dom.minidom as minidom
import xml.etree.ElementTree as ET
from collections import defaultdict
from typing import Any, Dict, List, Tuple

import bpy
from mathutils import Quaternion, Vector

# Colors from https://colorbrewer2.org/
colors = [
    (166, 206, 227),
    (31, 120, 180),
    (178, 223, 138),
    (51, 160, 44),
    (251, 154, 153),
    (227, 26, 28),
    (253, 191, 111),
    (255, 127, 0),
    (202, 178, 214),
    (106, 61, 154),
    (255, 255, 153),
    (177, 89, 40),
]
colors = [(c[0] / 255, c[1] / 255, c[2] / 255) for c in colors]
next_color = 0

# state vars
armature = None
counter = 0
links = []
joints = []
bones_to_meshes: Dict[Any, List[Any]] = defaultdict(
    list
)  # maps bones to lists of mesh objects

# constants
LINK_NAME_FORMAT = "{bone_name}"
JOINT_NAME_FORMAT = "{bone_name}"
ORIGIN_NODE_FLOAT_PRECISION = 6
ORIGIN_NODE_FORMAT = "{{:.{0}f}} {{:.{0}f}} {{:.{0}f}}".format(
    ORIGIN_NODE_FLOAT_PRECISION
)
ZERO_ORIGIN_NODE = lambda: ET.fromstring('<origin xyz="0 0 0" rpy="0 0 0"/>')
INERTIA_NODE_FMT = '<inertia ixx="{}" ixy="{}" ixz="{}" iyy="{}" iyz="{}" izz="{}" />'
AXIS_NODE_FORMAT = lambda: ET.fromstring('<axis xyz="0 0 0" />')
BASE_LIMIT_NODE_STR = None


def set_base_limit_str(effort, velocity):
    """
    Default effort and velocity limits for Joints.
    """
    global BASE_LIMIT_NODE_STR
    BASE_LIMIT_NODE_STR = '<limit effort="{:.4f}" lower="-1.57079632679" upper="1.57079632679" velocity="{:.4f}"/>'.format(
        effort, velocity
    )


def deselect_all() -> None:
    """
    Deselect all objects.
    """
    for obj in bpy.context.selected_objects:
        obj.select_set(False)


def is_mesh(obj) -> bool:
    """
    Is the object a MESH?
    """
    return obj.type == "MESH"


def is_collision_mesh(mesh_obj) -> bool:
    """
    Is the object a collision mesh.
    """
    return "collision" in mesh_obj.name


def get_mesh_heirarchy(
    mesh_obj,
    select_set: bool = True,
    include_non_collision: bool = True,
    include_collison: bool = False,
) -> List[Any]:
    """
    Select all MESH objects in the heirarchy specifically targeting or omitting meshes with "collision" in the name.

    :param mesh_obj: The Blender mesh object.
    :param select_set: Whether or not to select the objects as well as recording them.
    :param include_non_collision: Include objects without 'collision' in the name.
    :param include_collison: Include objects with 'collision' in the name.

    :return: The list of Blender mesh objects.
    """
    selected_objects = []
    is_col_mesh = is_collision_mesh(mesh_obj)
    if is_mesh(mesh_obj) and (
        (is_col_mesh and include_collison)
        or (include_non_collision and not is_col_mesh)
    ):
        selected_objects.append(mesh_obj)
        if select_set:
            mesh_obj.select_set(True)
    for child in mesh_obj.children:
        if child.type != "ARMATURE":
            selected_objects.extend(
                get_mesh_heirarchy(
                    child, select_set, include_non_collision, include_collison
                )
            )
    return selected_objects


def walk_armature(this_bone, handler):
    """
    Recursively apply a handler function to bone children to traverse the armature.
    """
    handler(this_bone)
    for child in this_bone.children:
        walk_armature(child, handler)


def bone_info(bone):
    """
    Print relevant bone info to console.
    """
    print(" bone info")
    print(f"    name: {bone.name}")
    print("    children")
    for child in bone.children:
        print(f"       - {child.name}")


def node_info(node):
    """
    Print relevant info about an object node.
    """
    print(" node info")
    print(f"    name: {node.name}")
    print("    children")
    for child in node.children:
        print(f"       - {child.name}")


def get_origin_from_matrix(M):
    """
    Construct a URDF 'origin' element from a Matrix by separating translation from rotation and converting to Euler.
    """
    translation = M.to_translation()
    euler = M.to_euler()
    origin_xml_node = ET.Element("origin")
    origin_xml_node.set("rpy", ORIGIN_NODE_FORMAT.format(euler.x, euler.y, euler.z))
    origin_xml_node.set(
        "xyz", ORIGIN_NODE_FORMAT.format(translation.x, translation.y, translation.z)
    )

    return origin_xml_node


def get_next_color() -> Tuple[int, int, int]:
    """
    Global function to get the next color in the colors list.
    """
    global next_color
    this_color = colors[next_color % len(colors)]
    next_color += 1
    return this_color


def add_color_material_to_visual(color, xml_visual) -> None:
    """
    Add a color material to a visual node.
    """
    this_xml_material = ET.Element("material")
    this_xml_material.set("name", "mat_col_{}".format(color))
    this_xml_color = ET.Element("color")
    this_xml_color.set("rgba", "{:.2f} {:.2f} {:.2f} 1.0".format(*color))
    this_xml_material.append(this_xml_color)
    xml_visual.append(this_xml_material)


def bone_to_urdf(this_bone, collision_visuals=False, joint_visual=False):
    """This function extracts the basic properties of the bone and populates
    links and joint lists with the corresponding urdf nodes"""

    global counter

    # Create the joint xml node
    if this_bone.parent:
        this_xml_link = create_bone_link(this_bone)
    else:
        this_xml_link = create_root_bone_link(this_bone)

    # NOTE: default intertia (assume overriden automatically in-engine)
    this_xml_link.append(ET.fromstring(INERTIA_NODE_FMT.format(1.0, 0, 0, 1.0, 0, 1.0)))

    # NOTE: default unit mass TODO: estimate somehow?
    this_xml_link.append(ET.fromstring('<mass value="{:.6f}"/>'.format(1.0)))

    # TODO: scrape the list of mesh filenames which would be generated by an export.
    collision_objects = []
    for mesh_obj in bones_to_meshes[this_bone.name]:
        collision_objects.extend(
            get_mesh_heirarchy(
                mesh_obj,
                select_set=False,
                include_collison=True,
                include_non_collision=False,
            )
        )
        if mesh_obj.parent is not None and mesh_obj.parent.type == "ARMATURE":
            # this object is the mesh name for export, so use it
            # Create the visual node
            this_xml_visual = ET.Element("visual")
            this_xml_mesh_geom = ET.Element("geometry")
            this_xml_mesh = ET.Element("mesh")
            this_xml_mesh.set("filename", f"{mesh_obj.name}.glb")
            this_xml_mesh.set("scale", "1.0 1.0 1.0")
            this_xml_mesh_geom.append(this_xml_mesh)
            # NOTE: we can use zero because we reset the origin for the meshes before exporting them to glb
            this_xml_visual.append(ZERO_ORIGIN_NODE())
            this_xml_visual.append(this_xml_mesh_geom)
            if not collision_visuals:
                this_xml_link.append(this_xml_visual)

    # NOTE: visual debugging tool to add a box at the joint pivot locations
    if joint_visual:
        this_xml_visual = ET.Element("visual")
        this_xml_test_geom = ET.Element("geometry")
        this_xml_box = ET.Element("box")
        this_xml_box.set("size", f"{0.1} {0.1} {0.1}")
        this_xml_test_geom.append(this_xml_box)
        this_xml_visual.append(ZERO_ORIGIN_NODE())
        this_xml_visual.append(this_xml_test_geom)
        this_xml_link.append(this_xml_visual)

    # NOTE: color each link's collision shapes for debugging
    this_color = get_next_color()

    for col in collision_objects:
        assert (
            "collision_box" in col.name or "collision_cylinder" in col.name
        ), "Only supporting collision boxes and cylinders currently."

        set_obj_origin_to_center(col)
        clear_obj_transform(
            col, apply=True, include_scale_apply=False, include_rot_apply=False
        )
        set_obj_origin_to_xyz(col, col.parent.matrix_world.translation)
        clear_obj_transform(col)
        set_obj_origin_to_center(col)

        # Create the collision node
        this_xml_collision = ET.Element("collision")
        if collision_visuals:
            this_xml_collision = ET.Element("visual")
            add_color_material_to_visual(this_color, this_xml_collision)
        this_xml_col_geom = ET.Element("geometry")
        xml_shape = None
        if "collision_box" in col.name:
            this_xml_box = ET.Element("box")
            box_size = col.scale
            this_xml_box.set("size", f"{box_size.x} {box_size.y} {box_size.z}")
            xml_shape = this_xml_box
        if "collision_cylinder" in col.name:
            this_xml_cyl = ET.Element("cylinder")
            scale = col.scale
            # radius XY axis scale must match
            assert scale.x == scale.y, "XY dimensions must match. Used as radius."
            this_xml_cyl.set("radius", f"{scale.x/2.0}")
            # NOTE: assume Z axis is length of the cylinder
            this_xml_cyl.set("length", f"{scale.z}")
            xml_shape = this_xml_cyl
        this_xml_col_geom.append(xml_shape)
        # first get the rotation
        xml_origin = get_origin_from_matrix(col.matrix_local)
        # then get local translation
        col_link_position = col.location
        xml_origin.set(
            "xyz",
            ORIGIN_NODE_FORMAT.format(
                col_link_position.x, col_link_position.y, col_link_position.z
            ),
        )
        this_xml_collision.append(xml_origin)
        this_xml_collision.append(this_xml_col_geom)
        this_xml_link.append(this_xml_collision)

    if not this_bone.children:
        pass
        # We reached the end of the chain.

    counter += 1


def create_root_bone_link(this_bone):
    """
    Construct the root link element from a bone.
    Called for bones with no parent (i.e. the root node)
    """
    xml_link = ET.Element("link")
    xml_link_name = this_bone.name
    xml_link.set("name", xml_link_name)
    links.append(xml_link)

    this_bone.name = xml_link_name
    return xml_link


def get_origin_from_bone(bone):
    """
    Construct an origin element for a joint from a bone.
    """
    translation = (
        bone.matrix_local.to_translation() - bone.parent.matrix_local.to_translation()
    )

    origin_xml_node = ET.Element("origin")
    origin_xml_node.set("rpy", "0 0 0")
    origin_xml_node.set(
        "xyz", ORIGIN_NODE_FORMAT.format(translation.x, translation.y, translation.z)
    )

    return origin_xml_node


def create_bone_link(this_bone):
    """
    Construct Link and Joint elements from a bone.
    """
    global counter

    # construct limits and joint type from animation frames
    bone_limits = get_anim_limits_info(this_bone)

    # Get bone properties
    parent_bone = this_bone.parent
    base_joint_name = JOINT_NAME_FORMAT.format(
        counter=counter, bone_name=this_bone.name
    )

    # ------------- Create joint--------------

    joint = ET.Element("joint")
    joint.set("name", base_joint_name)

    # create origin node
    origin_xml_node = get_origin_from_bone(this_bone)

    # create parent node
    parent_xml_node = ET.Element("parent")
    parent_xml_node.set("link", parent_bone.name)

    xml_link = ET.Element("link")
    xml_link_name = this_bone.name
    xml_link.set("name", xml_link_name)
    links.append(xml_link)

    # create child node
    child_xml_node = ET.Element("child")
    child_xml_node.set("link", xml_link_name)

    joint.append(parent_xml_node)
    joint.append(child_xml_node)

    # create limits node
    limit_node = ET.fromstring(BASE_LIMIT_NODE_STR)

    axis = Vector()

    # Revolute
    if len(bone_limits["lower_limit"]) == 4:
        joint.set("type", "revolute")
        begin = Quaternion(bone_limits["lower_limit"])
        end = Quaternion(bone_limits["upper_limit"])
        rest = Quaternion()
        diff = begin.rotation_difference(end)
        axis, angle = diff.to_axis_angle()
        # NOTE: rest pose could be applied to the bone resulting in an additional rotation stored in the matrix property
        rest_correction = this_bone.matrix
        # NOTE: Blender bones and armature are always Y-up, so we need to rotate the axis into URDF coordinate space (Z-up)
        bone_axis = this_bone.vector
        to_z_up = bone_axis.rotation_difference(Vector([0, 0, 1]))
        # apply all rotations to arrive at the URDF Joint axis
        axis = rest_correction @ (to_z_up @ axis)
        rest_diff = begin.rotation_difference(rest)
        rest_axis, rest_angle = rest_diff.to_axis_angle()
        limit_node.set("lower", f"{-rest_angle}")
        limit_node.set("upper", f"{angle-rest_angle}")

    # Prismatic
    if len(bone_limits["lower_limit"]) == 3:
        joint.set("type", "prismatic")
        upper_vec = Vector(bone_limits["upper_limit"])
        lower_vec = Vector(bone_limits["lower_limit"])
        displacement = upper_vec - lower_vec
        axis = displacement.normalized()
        limit_node.set("lower", f"{-lower_vec.length}")
        limit_node.set("upper", f"{upper_vec.length}")

    xml_axis = AXIS_NODE_FORMAT()
    xml_axis.set("xyz", ORIGIN_NODE_FORMAT.format(axis.x, axis.y, axis.z))

    joint.append(xml_axis)
    joint.append(limit_node)
    joint.append(origin_xml_node)
    joints.append(joint)
    ret_link = xml_link

    return ret_link


# ==========================================


def set_obj_origin_to_center(obj) -> None:
    """
    Set object origin to it's own center.
    """
    deselect_all()
    obj.select_set(True)
    bpy.ops.object.origin_set(type="ORIGIN_GEOMETRY", center="MEDIAN")


def set_obj_origin_to_xyz(obj, xyz) -> None:
    """
    Set object origin to a global xyz location.
    """
    deselect_all()
    bpy.context.scene.cursor.location = xyz
    obj.select_set(True)
    bpy.ops.object.origin_set(type="ORIGIN_CURSOR", center="MEDIAN")


def set_obj_origin_to_bone(obj, bone):
    """
    Set the object origin to the bone transformation.
    """
    set_obj_origin_to_xyz(obj, bone.matrix_local.translation)


def clear_obj_transform(
    arm, apply=False, include_scale_apply=True, include_rot_apply=True
):
    """
    Clear the armature transform to align it with the origin.
    """
    deselect_all()
    arm.select_set(True)
    if apply:
        bpy.ops.object.transform_apply(
            location=True, rotation=include_rot_apply, scale=include_scale_apply
        )
    else:
        bpy.ops.object.location_clear(clear_delta=False)


def get_anim_limits_info(bone):
    """
    Get limits info from animation action tracks.
    """
    bone_limits = {"rest_pose": [], "lower_limit": [], "upper_limit": []}
    if "root" in bone.name:
        # no joint data defined for the root
        return bone_limits
    is_prismatic = False
    is_revolute = False

    for ac in bpy.data.actions:
        if bone.name in ac.name:
            key_match = [key for key in bone_limits if key in ac.name][0]
            limit_list = []
            for _fkey, fcurve in ac.fcurves.items():
                assert (
                    len(fcurve.keyframe_points) == 1
                ), "Expecting one keyframe per track."
                index = fcurve.array_index
                value = fcurve.keyframe_points[0].co[1]
                if "quaternion" in fcurve.data_path:
                    if len(limit_list) == 0:
                        limit_list = [0, 0, 0, 0]
                    is_revolute = True
                if "location" in fcurve.data_path:
                    if len(limit_list) == 0:
                        limit_list = [0, 0, 0]
                    is_prismatic = True
                limit_list[index] = value
            bone_limits[key_match] = limit_list
    assert (
        is_prismatic or is_revolute
    ), f"Bone {bone.name} does not have animation data."
    assert not (
        is_prismatic and is_revolute
    ), f"Bone {bone.name} has both rotation and translation defined."
    return bone_limits


def get_parent_bone(obj):
    """
    Climb the node tree looking for the parent bone of an object.
    Return the parent bone or None if a parent bone does not exist.
    """
    if obj.parent_bone != "":
        return armature.data.bones[obj.parent_bone]
    if obj.parent is None:
        return None
    return get_parent_bone(obj.parent)


def get_root_bone():
    """
    Find the root bone.
    """
    root_bone = None
    for b in armature.data.bones:
        if not b.parent:
            assert root_bone is None, "More than one root bone found."
            root_bone = b
    return root_bone


def get_armature():
    """
    Search the objects for an armature object.
    """
    for obj in bpy.data.objects:
        if obj.type == "ARMATURE":
            return obj
    return None


def export(dirpath, settings, export_meshes: bool = True):
    """
    Run the Armature to URDF converter and export the .urdf file.
    Recursively travserses the armature bone tree and constructs Links and Joints.
    Note: This process is destructive and requires undo or revert in the editor after use.
    """

    global LINK_NAME_FORMAT, JOINT_NAME_FORMAT, armature, root_bone, links, joints, counter
    counter = 0
    links = []
    joints = []

    # get the armature
    armature = settings.get("armature")
    if armature is None:
        armature = bpy.data.objects["Armature"]

    # find the root bone
    root_bone = get_root_bone()

    if "link_name_format" in settings:
        LINK_NAME_FORMAT = settings["link_name_format"]

    if "joint_name_format" in settings:
        JOINT_NAME_FORMAT = settings["joint_name_format"]

    # set the defaults to 100 T units and 3 units/sec (meters or radians)
    effort, velocity = (100, 3)
    if "def_limit_effort" in settings:
        effort = settings["def_limit_effort"]
    if "def_limit_vel" in settings:
        velocity = settings["def_limit_vel"]
    set_base_limit_str(effort, velocity)

    # clear the armature transform to remove unwanted transformations for later
    clear_obj_transform(armature, apply=True)

    # print all mesh object parents, reset origins for mesh export and transformation registery, collect bone to mesh map
    root_node = None
    for obj in bpy.data.objects:
        if obj.type == "MESH":
            parent_bone = get_parent_bone(obj)
            set_obj_origin_to_bone(obj, parent_bone)
            print(f"MESH: {obj.name}")
            if obj.parent is not None:
                print(f" -p> {obj.parent.name}")
            if obj.parent_bone != "":
                bones_to_meshes[obj.parent_bone].append(obj)
                print(f" -pb> {obj.parent_bone}")
        elif obj.type == "EMPTY":
            print(f"EMPTY: {obj.name}")
            if obj.parent is None and len(obj.children) > 0:
                print(" --IS ROOT")
                root_node = obj

    # make export directory for the object
    assert root_node is not None, "No root node, aborting."
    final_out_path = os.path.join(dirpath, f"{root_node.name}")
    os.makedirs(final_out_path, exist_ok=True)

    # export mesh components
    if export_meshes:
        for mesh_list in bones_to_meshes.values():
            for mesh_obj in mesh_list:
                if mesh_obj.parent is not None and mesh_obj.parent.type == "ARMATURE":
                    clear_obj_transform(mesh_obj)
                    deselect_all()
                    get_mesh_heirarchy(mesh_obj)
                    bpy.ops.export_scene.gltf(
                        filepath=os.path.join(final_out_path, mesh_obj.name),
                        use_selection=True,
                        export_yup=False,
                    )
        return None

    # print("------------------------")
    # print("Bone info recursion:")
    # walk_armature(root_bone, bone_info)
    # print("------------------------")
    # print("Node info recursion:")
    # walk_armature(root_node, node_info)
    # print("------------------------")

    # Recursively generate the xml elements
    walk_armature(root_bone, bone_to_urdf)

    # add all the joints and links to the root
    root_xml = ET.Element("robot")  # create <robot name="test_robot">
    root_xml.set("name", armature.name)

    root_xml.append(ET.Comment("LINKS"))
    for l in links:
        root_xml.append(l)

    root_xml.append(ET.Comment("JOINTS"))
    for j in joints:
        root_xml.append(j)

    # dump the xml string
    ET_raw_string = ET.tostring(root_xml, encoding="unicode")
    dom = minidom.parseString(ET_raw_string)
    ET_pretty_string = dom.toprettyxml()

    with open(os.path.join(final_out_path, "armature.urdf"), "w") as f:
        f.write(ET_pretty_string)

    return ET_pretty_string


if __name__ == "__main__":
    # NOTE: this must be run from within Blender and by defaults saves viles in "blender_armatures/" relative to the directory containing the script
    export_path = os.path.join(
        os.path.dirname(bpy.context.space_data.text.filepath), "blender_armatures"
    )
    # Optionally override the save directory with an absolute path of your choice
    # export_path = "/home/my_path_choice/"

    # To use this script:
    # 1. run with export meshes True
    # 2. undo changes in the editor
    # 3. run with export meshes False
    # 4. undo changes in the editor
    export_meshes = False

    urdf_str = export(
        export_path,
        {
            "armature": get_armature(),
            #'def_limit_effort': 100, #custom default effort limit for joints
            #'def_limit_vel': 3, #custom default vel limit for joints
        },
        export_meshes=export_meshes,
    )
    print(f"\n ======== Output saved to {export_path} ========\n")
